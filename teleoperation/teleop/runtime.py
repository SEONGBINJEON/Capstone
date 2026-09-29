import json
import math
import socket
import time
from dataclasses import asdict, replace
from pathlib import Path

from .config import SafetyError, TICKS_PER_DEG, joints, require_calibration, follower_bounds
from .control import Controller, health
from .hardware import profile_values


class Events:
    """Output-only fanout. No incoming network command can drive a physical motor."""
    def __init__(self, path=None, udp_port=None):
        self.file = None
        self.sock = None
        self.destination = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.file = open(path, 'a', buffering=1)
        if udp_port:
            if not 1024 <= udp_port <= 65535:
                raise SafetyError("UDP 포트는 1024~65535입니다.")
            self.destination = ('127.0.0.1', udp_port)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)

    def emit(self, payload):
        text = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        if self.file:
            self.file.write(text+'\n')
        if self.sock:
            try:
                self.sock.sendto(text.encode(), self.destination)
            except (BlockingIOError, ConnectionRefusedError):
                pass  # A stalled optional simulator must not block the control loop.

    def close(self):
        if self.file:
            self.file.close()
        if self.sock:
            self.sock.close()


def preflight(rig, config, pair, state, startup_mode='matched'):
    require_calibration(config, pair)
    health(config, state, rig.metadata)
    for _, j in joints(config, pair):
        c = j['calibration']
        for role in ('leader', 'follower'):
            i = j[role]['id']
            if c['signature'].get(role) != rig.signature(i):
                raise SafetyError(f"ID {i}: 보정 후 모터 모드/방향/원점 설정이 달라졌습니다. 재보정 필요")
            if state[i].torque != 0:
                raise SafetyError(f"ID {i}: 시작 전 토크가 꺼져 있어야 합니다. 팔을 받친 상태에서 확인하세요.")
        fid = j['follower']['id']; meta = rig.metadata[fid]
        if meta['operating_mode'] not in (3,4):
            raise SafetyError(f"ID {fid}: 지원되지 않는 위치 제어 모드입니다.")
        if meta['drive_mode'] & 0x08:
            raise SafetyError(f"ID {fid}: 목표 갱신 시 자동 토크 켜짐 설정을 지원하지 않습니다.")
        if state[fid].watchdog:
            raise SafetyError(f"ID {fid}: 기존 watchdog 설정을 확인하세요. 자동 초기화하지 않습니다.")
        lo, hi = follower_bounds(j)
        lower,upper=rig.position_limits(fid)
        if not lower <= lo < hi <= upper:
            raise SafetyError(f"ID {fid}: 보정 범위가 모터에 설정된 위치 제한 밖입니다.")
    return Controller(config, pair, state, startup_mode=startup_mode)


def arm(rig, config, pair, state, armed):
    # Stage current-position goals for every follower while torque remains off.
    entries = joints(config, pair)
    for _, j in entries:
        fid = j['follower']['id']
        acceleration, velocity = profile_values(rig.metadata[fid]['drive_mode'], state[fid].position,
                                                 state[fid].position, config['safety']['max_speed_deg_s'],
                                                 1/config['safety']['rate_hz'])
        rig.write(fid, 108, acceleration, 4)
        rig.write(fid, 112, velocity, 4)
        rig.write(fid, 116, state[fid].position, 4)
        rig.write(fid, 98, 15, 1)  # Configure watchdog before any torque enable
    # Keep the initial hold goals fresh: re-read after staging, before enabling.
    refreshed = rig.sample()
    health(config, refreshed, rig.metadata)
    for _, j in entries:
        fid = j['follower']['id']
        if abs(refreshed[fid].position-state[fid].position) > 8:
            raise SafetyError(f"ID {fid}: 준비 중 위치가 움직였습니다. 다시 시작하세요.")
    for _, j in entries:
        fid = j['follower']['id']
        armed.add(fid)  # Include uncertain enable writes in cleanup.
        rig.write(fid, 64, 1, 1)


def hold(rig, armed, *, preserve_stationary=False):
    """Best effort current-position hold; never drop a gravity-loaded arm by torque-off."""
    if not armed:
        return []
    errors = []
    try:
        states = rig.sample()
    except Exception as exc:
        return [f"현재 위치를 읽지 못해 hold 전송 불가: {exc}. 통신 watchdog에 의존합니다."]
    # Do not clear a watchdog fault: that would silently re-enable goal acceptance.
    for i in sorted(armed):
        try:
            if states[i].watchdog == 255 or states[i].hardware_error:
                raise SafetyError("오류가 남아 있어 목표값을 다시 보내지 않습니다.")
            if not 0 <= states[i].position <= 4095:
                raise SafetyError("현재 위치가 단회전 범위 밖입니다.")
            state = states[i]
            # A loaded servo can rest slightly away from its goal. Recapturing
            # that position on every normal hold exit moves the reference and
            # allows another increment of sag. Preserve a completed, stationary
            # hold; faults and unfinished moves still request a current-pose stop.
            if (preserve_stationary and state.torque == 1 and state.present_velocity == 0
                    and state.goal_position is not None and state.position_trajectory is not None
                    and abs(state.goal_position-state.position) <= 5*TICKS_PER_DEG):
                lo, hi = rig.position_limits(i)
                if lo <= state.goal_position <= hi and state.position_trajectory == rig.raw_goal(i, state.goal_position):
                    continue
            lo, hi = rig.position_limits(i)
            if not lo-2*TICKS_PER_DEG <= state.position <= hi+2*TICKS_PER_DEG:
                raise SafetyError('현재 위치가 유지 가능한 모터 범위를 벗어났습니다.')
            rig.write(i, 116, max(lo,min(hi,state.position)), 4)
        except Exception as exc:
            errors.append(f"ID {i}: {exc}")
    return errors


def hold_preflight(rig, config, pair, states, resume=False, recover_watchdog=False):
    """Holding the present pose needs no leader/follower geometric calibration."""
    if recover_watchdog and not resume:
        raise SafetyError('통신 감시 오류 복구는 기존 자세 유지 재개에서만 허용합니다.')
    followers = {j['follower']['id'] for _,j in joints(config,pair)}
    checked = {i:replace(s,watchdog=0) if resume and recover_watchdog and i in followers and s.watchdog==255
               else s for i,s in states.items()}
    health(config, checked, rig.metadata)
    for _,j in joints(config,pair):
        i = j['follower']['id']; meta = rig.metadata[i]; state = states[i]
        if meta['operating_mode'] not in (3,4) or meta['drive_mode'] & 8:
            raise SafetyError(f'ID {i}: 현재 자세 유지에 지원되지 않는 모터 모드')
        if resume:
            if state.torque != 1 or state.watchdog not in (0,15,255):
                raise SafetyError(f'ID {i}: 기존 토크 ON 유지 상태가 아닙니다.')
            goal = state.goal_position
            lower,upper=rig.position_limits(i)
            if goal is None or not lower <= goal <= upper or abs(goal-state.position)>8:
                raise SafetyError(f'ID {i}: 기존 목표와 현재 위치가 달라 유지 재개를 거부합니다.')
        elif state.torque or state.watchdog:
            raise SafetyError(f'ID {i}: 토크 OFF / watchdog 0 상태에서 유지 모드를 시작하세요.')
        lower,upper=rig.position_limits(i)
        if not lower <= state.position <= upper:
            raise SafetyError(f'ID {i}: 현재 위치가 저장된 위치 제한 밖이라 토크를 켤 수 없습니다.')


def calibration_jog(rig, config, pair, states, joint_index, degrees):
    """One observed direction check, at most 3 degrees; never authorizes teleoperation."""
    if pair not in ('A','B') or joint_index not in (1,2,3,4) or not math.isfinite(degrees) or not 0 < abs(degrees) <= 3:
        raise SafetyError('방향 확인은 A/B의 팔 관절 하나를 0° 초과 3° 이하로만 움직일 수 있습니다.')
    joint = config['pairs'][pair][joint_index-1]
    fid = joint['follower']['id']
    start = states[fid].position
    target = start + round(degrees*TICKS_PER_DEG)
    meta = rig.metadata[fid]
    lo,hi = rig.position_limits(fid)
    if joint['calibration'].get('reviewed'):
        lower,upper = follower_bounds(joint)
        lo,hi = max(lo,lower),min(hi,upper)
    if target == start or not lo <= min(start,target) <= max(start,target) <= hi:
        raise SafetyError(f'ID {fid}: 방향 확인 경로가 저장된 이동 제한 밖입니다.')
    return fid,start,target


def hold_session(rig, config, pair, duration, events, resume=False, recover_watchdog=False,
                 jog_joint=None, jog_degrees=None, home_joint=None, home_offset=None,
                 supported=False, config_path=None, backup_dir='work/backups',extended_base=False):
    if (jog_joint is None) != (jog_degrees is None) or (jog_joint is not None and not resume):
        raise SafetyError('방향 확인은 --resume-hold와 --jog-joint/--jog-degrees를 함께 지정해야 합니다.')
    if (home_joint is None) != (home_offset is None):
        raise SafetyError('원점 변경 관절과 값을 함께 지정하세요.')
    if home_joint is not None and (not resume or not supported or not config_path or jog_joint is not None):
        raise SafetyError('원점 변경은 팔을 받치고 --resume-hold/--supported를 지정해야 하며 방향 확인과 동시에 할 수 없습니다.')
    if extended_base and (not resume or not supported or not config_path or home_joint is not None or jog_joint is not None or pair!='A'):
        raise SafetyError('11번 확장 위치 모드 전환은 --resume-hold/--supported와 함께 단독으로만 실행할 수 있습니다.')
    first = rig.sample()
    hold_preflight(rig,config,pair,first,resume,recover_watchdog)
    jog = calibration_jog(rig,config,pair,first,jog_joint,jog_degrees) if jog_joint is not None else None
    if home_joint is not None:
        from .maintenance import homing_shift_plan, shift_home_at_rest
        homing_shift_plan(rig,config,pair,home_joint,home_offset,first)
    armed = set(); count = 0; reason = 'duration_complete'
    jog_done = not bool(jog)
    period = 1/config['safety']['rate_hz']
    targets = {j['follower']['id']:(first[j['follower']['id']].goal_position if resume else first[j['follower']['id']].position)
               for _,j in joints(config,pair)}
    try:
        if resume:
            refreshed = rig.sample()
            hold_preflight(rig,config,pair,refreshed,resume,recover_watchdog)
            for i in targets:
                if abs(refreshed[i].position-first[i].position)>8 or refreshed[i].goal_position != targets[i]:
                    raise SafetyError(f'ID {i}: 유지 재개 준비 중 위치 또는 목표가 바뀌었습니다.')
            # Keep the existing hold goals and torque; restart only the communication timer.
            for i in targets:
                armed.add(i)
                if refreshed[i].watchdog == 255:
                    rig.write(i,98,0,1)
                rig.write(i,98,15,1)
        else:
            arm(rig,config,pair,first,armed)
        if home_joint is not None:
            i,goal=shift_home_at_rest(rig,config,pair,home_joint,home_offset,config_path,backup_dir,events)
            targets[i]=goal
        if extended_base:
            from .maintenance import enable_extended_base_at_rest
            i,goal=enable_extended_base_at_rest(rig,config,config_path,backup_dir,events)
            targets[i]=goal
        if jog:
            current = rig.sample()
            health(config,current,rig.metadata)
            for i in targets:
                if not current[i].torque or abs(current[i].position-first[i].position)>8:
                    raise SafetyError(f'ID {i}: 방향 확인 준비 중 유지 상태가 달라졌습니다.')
            jog = calibration_jog(rig,config,pair,current,jog_joint,jog_degrees)
            fid,origin,target = jog
            rig.goals({fid:target},current_positions={i:s.position for i,s in current.items()},
                      speed_deg_s=2,period_s=period)
            targets[fid] = target
        start = time.monotonic(); previous = start
        jog_done = not bool(jog); settled = 0
        while duration is None or time.monotonic()-start < duration:
            state = rig.sample(); now = time.monotonic()
            if now-previous > config['safety']['max_cycle_s']:
                raise SafetyError('자세 유지 중 통신 시간 초과')
            health(config,state,rig.metadata)
            for i,target in targets.items():
                if not state[i].torque:
                    raise SafetyError(f'ID {i}: 자세 유지 중 토크가 꺼졌습니다.')
                tolerance = 8 if count == 0 else config['safety']['max_tracking_error_deg']*TICKS_PER_DEG
                if jog:
                    tolerance = 8
                    if i == jog[0] and not jog_done:
                        origin,target = jog[1:]
                        if not min(origin,target)-8 <= state[i].position <= max(origin,target)+8:
                            raise SafetyError(f'ID {i}: 방향 확인 경로를 벗어났습니다.')
                        tolerance = abs(target-origin)+8
                if abs(state[i].position-target) > tolerance:
                    raise SafetyError(f'ID {i}: 자세 유지 오차 초과')
            if jog and not jog_done:
                # Match the established hold tolerance; the physical servo can settle
                # 5–7 ticks from a tiny goal while reporting its in-position bit.
                settled = settled + 1 if abs(state[jog[0]].position-jog[2]) <= 8 else 0
                jog_done = settled >= 3
                if not jog_done and now-start > 5:
                    raise SafetyError('방향 확인 이동 시간 초과')
            events.emit({'schema':'teleop.holding.v1','pair':pair,'sequence':count,
                         'phase':'jogging' if not jog_done else 'holding',
                         'unix_time':time.time(),'targets':targets,
                         'positions':{i:state[i].position for i in targets},
                         'motors':{i:asdict(s) for i,s in state.items()}})
            previous = now; count += 1
            time.sleep(max(0, period-(time.monotonic()-now)))
    except BaseException as exc:
        reason = type(exc).__name__
        raise
    finally:
        errors = hold(rig,armed, preserve_stationary=jog_done and reason in ('duration_complete','KeyboardInterrupt'))
        events.emit({'schema':'teleop.stopped.v1','reason':reason,'frames':count,
                     'hold_errors':errors,'torque_left_on':bool(armed)})
    return {'mode':'hold_only','pair':pair,'frames':count,'torque_left_on':bool(armed)}


def run_session(rig, config, pair, duration, enable_motion, events, startup_mode='matched'):
    period = 1/config['safety']['rate_hz']
    first = rig.sample()
    controller = preflight(rig, config, pair, first, startup_mode=startup_mode)
    armed = set(); sequence = 0; started = time.monotonic()
    reason = 'duration_complete'
    try:
        if enable_motion:
            arm(rig, config, pair, first, armed)
            first = rig.sample()
            health(config, first, rig.metadata)
            for _, j in joints(config, pair):
                fid = j['follower']['id']
                if abs(first[fid].position-controller.commanded[fid]) > 8:
                    raise SafetyError(f"ID {fid}: 토크 활성화 후 위치 기준이 바뀌었습니다.")
        previous = time.monotonic()
        started = previous
        while duration is None or time.monotonic()-started < duration:
            begin = time.monotonic()
            state = rig.sample()
            now = time.monotonic()
            if now-begin > config['safety']['max_cycle_s']:
                raise SafetyError("모터 상태 읽기 시간 초과")
            if any(now-s.sampled_at > config['safety']['max_cycle_s'] for s in state.values()):
                raise SafetyError("오래된 모터 상태 수신")
            health(config, state, rig.metadata)
            for _, j in joints(config, pair):
                if state[j['leader']['id']].torque:
                    raise SafetyError("리더 토크가 켜졌습니다. 추종 중지")
                if enable_motion and not state[j['follower']['id']].torque:
                    raise SafetyError("팔로워 토크가 꺼졌습니다. 추종 중지")
            dt = now-previous
            phase = controller.phase
            speed = controller.speed_deg_s
            goals, details = controller.step(state, dt, check_tracking=enable_motion)
            # One common target vector drives hardware and the simulation output.
            if enable_motion:
                rig.goals(goals, current_positions={i:s.position for i,s in state.items()},
                          speed_deg_s=speed, period_s=period)
            events.emit({'schema':'teleop.targets.v1','sequence':sequence,'unix_time':time.time(),
                         'mode':'hardware' if enable_motion else 'dry_run',
                         'hardware_command_sent':enable_motion,'phase':phase,'joints':details})
            sequence += 1; previous = now
            delay = period-(time.monotonic()-begin)
            if delay > 0:
                time.sleep(delay)
    except BaseException as exc:
        reason = type(exc).__name__
        raise
    finally:
        errors = hold(rig, armed)
        events.emit({'schema':'teleop.stopped.v1','reason':reason,'frames':sequence,
                     'hold_errors':errors,'torque_left_on':bool(armed)})
    return {'frames':sequence,'seconds':time.monotonic()-started,'motion_enabled':enable_motion}
