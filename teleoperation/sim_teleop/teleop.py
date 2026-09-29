"""Leader 26~30 (OpenRB-150) -> Follower 11~15 (OpenCR) teleoperation.

Publishes joint state over UDP for isaac_view.py.
Ctrl+C: stop following, followers hold their pose with torque on.
    python3 sim_teleop/teleop.py              # align slowly, then follow
    python3 sim_teleop/teleop.py --status     # read-only snapshot
    python3 sim_teleop/teleop.py --release    # torque off (support the arm!)
"""
import argparse
import json
import math
import socket
import sys
import time

from common import (SafetyError, TICKS_PER_DEG, delta_ticks, follower_ids, follower_to_sim,
                    health, joint_target, leader_ids, load_config, open_rig)


def fmt_states(cfg, rig, s):
    lines = []
    for j in cfg['joints']:
        lid, fid = j['leader']['id'], j['follower']['id']
        L, F = s[lid], s[fid]
        lines.append(f"{j['name']:8s} L{lid}={L.position:5d}  F{fid}={F.position:5d} torque={F.torque} "
                     f"goal={F.goal_position} T={F.temperature}°C V={F.voltage}V hw={F.hardware_error}")
    return '\n'.join(lines)


class Publisher:
    def __init__(self, cfg):
        self.dest = (cfg['udp']['host'], cfg['udp']['port'])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, payload):
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.dest)
        except OSError:
            pass


def sim_payload(cfg, s, goals, phase, source):
    joints = {}
    for j in cfg['joints']:
        fid, lid = j['follower']['id'], j['leader']['id']
        pos = s[fid].position
        if source == 'target' and fid in goals:
            pos = goals[fid]
        rad, frac = follower_to_sim(j, pos)
        joints[j['sim']['joint']] = {'rad': rad, 'deg': math.degrees(rad),
                                     'follower_ticks': s[fid].position, 'leader_ticks': s[lid].position,
                                     'gripper_fraction': frac}
    return {'t': time.time(), 'phase': phase, 'joints': joints}


def release(cfg):
    with open_rig(cfg, allow_motion=True) as rig:
        s = rig.sample()
        print(fmt_states(cfg, rig, s))
        on = [i for i in follower_ids(cfg) if s[i].torque]
        if not on:
            print('팔로워 토크가 이미 꺼져 있습니다.'); return
        print('5초 후 팔로워 토크를 끕니다. 팔이 떨어지지 않게 받쳐 주세요. 중단: Ctrl+C')
        for k in range(5, 0, -1):
            print(k, flush=True); time.sleep(1)
        for i in on:
            rig.write(i, 64, 0, 1)
        time.sleep(.1)
        s = rig.sample()
        print('토크 상태:', {i: s[i].torque for i in follower_ids(cfg)})


def run(cfg, args):
    period = cfg['period_s']
    saf = cfg['safety']
    pub = Publisher(cfg)
    with open_rig(cfg, allow_motion=True) as rig:
        # ---- preflight ------------------------------------------------
        samples = []
        for _ in range(4):
            samples.append(rig.sample()); time.sleep(.05)
        s = samples[-1]
        health(cfg, s, rig.metadata)
        print(fmt_states(cfg, rig, s))
        for lid in leader_ids(cfg):
            if s[lid].torque:
                raise SafetyError(f'리더 ID {lid} 토크가 켜져 있습니다. 리더는 토크 OFF여야 합니다.')
        fids = follower_ids(cfg)
        if args.match:
            # Treat the current leader pose as the current follower pose (in-memory only).
            # The simulation keeps its own reference (sim.zero_ticks = upright follower zero).
            for j in cfg['joints']:
                if j.get('kind') == 'gripper':
                    continue
                j['sim'].setdefault('zero_ticks', j['follower_zero'])
                j['leader_zero'] = s[j['leader']['id']].position
                j['follower_zero'] = s[j['follower']['id']].position
            print('--match: 현재 리더 자세 = 현재 팔로워 자세로 간주 (정렬 이동 없음)')
        for fid in fids:
            if max(x[fid].position for x in samples) - min(x[fid].position for x in samples) > 4:
                raise SafetyError(f'ID {fid}: 시작 준비 중 팔로워가 움직였습니다.')
            if rig.metadata[fid]['operating_mode'] not in (3, 4):
                raise SafetyError(f'ID {fid}: 위치 제어 모드가 아닙니다.')
        bounds = {}
        for j in cfg['joints']:
            fid = j['follower']['id']
            lo, hi = rig.position_limits(fid)
            if j.get('kind') == 'gripper':
                glo, ghi = sorted((j['follower_closed'], j['follower_open']))
                lo, hi = max(lo, glo), min(hi, ghi)
            bounds[fid] = (lo, hi)
            if not lo - 2 * TICKS_PER_DEG <= s[fid].position <= hi + 2 * TICKS_PER_DEG:
                raise SafetyError(f"ID {fid} ({j['name']}): 현재 위치 {s[fid].position}가 모터 허용 범위 {lo}~{hi} 밖입니다. "
                                  f"토크 OFF 상태에서 팔로워 관절을 손으로 {(lo-s[fid].position)/TICKS_PER_DEG if s[fid].position<lo else (hi-s[fid].position)/TICKS_PER_DEG:+.0f}° 이상 돌려 범위 안에 두고 다시 시작하세요.")
            tgt, _ = joint_target(j, s[j['leader']['id']].position, bounds[fid])
            gap = abs(tgt - s[fid].position) / TICKS_PER_DEG
            print(f"  {j['name']:8s} 시작 정렬 필요 이동량 {gap:6.1f}°  (목표 {tgt:.0f}, 현재 {s[fid].position})")
            if gap > saf['max_initial_align_deg']:
                raise SafetyError(f"{j['name']}: 시작 정렬 이동량 {gap:.0f}°가 너무 큽니다. 리더를 팔로워와 비슷한 자세로 두고 다시 시작하세요.")
        if args.dry_run:
            print('dry-run: 모터에 쓰지 않고 종료합니다.'); return

        # ---- arm followers (hold current pose) -------------------------
        armed = set()
        for fid in fids:
            if s[fid].watchdog == 255:
                rig.write(fid, 98, 0, 1)
            if not s[fid].torque:
                rig.write(fid, 108, 20, 4)
                rig.write(fid, 112, 30, 4)
                rig.write(fid, 116, max(bounds[fid][0], min(bounds[fid][1], s[fid].position)), 4)
            rig.write(fid, 98, 15, 1)   # 15*20ms = 300ms comms watchdog
        s2 = rig.sample()
        for fid in fids:
            if abs(s2[fid].position - s[fid].position) > 8:
                raise SafetyError(f'ID {fid}: 준비 중 위치가 움직였습니다.')
        for fid in fids:
            armed.add(fid)
            if not s2[fid].torque:
                rig.write(fid, 64, 1, 1)
        s = rig.sample()
        if any(not s[fid].torque for fid in fids):
            raise SafetyError('팔로워 토크 ON 실패')
        print('팔로워 토크 ON. 저속 정렬 시작 (Ctrl+C: 자세 유지 후 종료)')

        commanded = {fid: float(s[fid].goal_position if s[fid].goal_position is not None else s[fid].position) for fid in fids}
        for fid in fids:
            commanded[fid] = max(bounds[fid][0], min(bounds[fid][1], commanded[fid]))
        last_offsets = {j['leader']['id']: s[j['leader']['id']].position for j in cfg['joints']}
        phase = 'aligning'; settled = 0; blocked_reason = None; blocked_at = 0.0
        stall = {fid: [] for fid in fids}; stall_info = None   # (fid, held_position, push_sign)
        last_goals = {}; last_speed = None; previous = time.monotonic(); count = 0
        try:
            while True:
                begin = time.monotonic()
                s = rig.sample(); now = time.monotonic(); dt = now - previous; previous = now
                if dt > saf['max_cycle_s']:
                    raise SafetyError(f'제어 주기 초과 ({dt*1000:.0f} ms)')
                health(cfg, s, rig.metadata)
                if any(not s[fid].torque for fid in fids):
                    raise SafetyError('추종 중 팔로워 토크가 꺼졌습니다.')
                if any(s[fid].watchdog != 15 for fid in fids):
                    raise SafetyError('팔로워 통신 감시 상태가 바뀌었습니다.')
                for lid in leader_ids(cfg):
                    if abs(delta_ticks(s[lid].position, last_offsets[lid])) > saf['max_leader_step_deg'] * TICKS_PER_DEG:
                        raise SafetyError(f'리더 ID {lid}: 위치 급변 (한 주기에 {saf["max_leader_step_deg"]}° 초과)')
                    last_offsets[lid] = s[lid].position

                targets = {}; limited = []
                for j in cfg['joints']:
                    fid = j['follower']['id']
                    targets[fid], lim = joint_target(j, s[j['leader']['id']].position, bounds[fid])
                    if lim: limited.append(fid)

                if phase == 'blocked':
                    gap = max(abs(targets[fid] - s[fid].position) for fid in fids) / TICKS_PER_DEG
                    if stall_info:
                        # Stalled against something: resume only when the leader's target for that joint
                        # comes back to (or past) the held position, or the whole pose matches again.
                        bfid, held, sign = stall_info
                        retreated = (targets[bfid] - held) * sign <= 2 * TICKS_PER_DEG
                        ready = gap <= 3.0 or retreated
                        if not ready and count % 20 == 0:
                            print(f'걸림 대기: {blocked_reason} / 리더의 해당 관절을 반대 방향으로 되돌리세요 (목표까지 {abs(targets[bfid]-s[bfid].position)/TICKS_PER_DEG:.1f}°)', flush=True)
                    else:
                        ready = now - blocked_at >= saf.get('blocked_hold_s', 1.0) or gap <= 3.0
                    if ready:
                        # Rejoin at normal speed, anchored at the current pose (no slow re-alignment).
                        phase = 'following'; settled = 0; stall_info = None
                        stall = {fid: [] for fid in fids}
                        commanded = {fid: float(s[fid].position) for fid in fids}
                        print(f'추종 재개 (리더 목표까지 {gap:.1f}°)', flush=True)
                    else:
                        pub.send(sim_payload(cfg, s, targets, phase, cfg.get('sim_source', 'follower')))
                        count += 1; time.sleep(max(0, period - (time.monotonic() - begin))); continue

                speed = cfg['align_speed_deg_s'] if phase == 'aligning' else cfg['follow_speed_deg_s']
                goals = {}; all_reached = True
                for fid in fids:
                    p = s[fid]
                    err = (p.raw_position - p.position_trajectory) if (p.raw_position is not None and p.position_trajectory is not None) else (p.position - commanded[fid])
                    if abs(err) > saf['max_tracking_error_deg'] * TICKS_PER_DEG:
                        blocked_reason = f'ID {fid} 추종 오차 {abs(err)/TICKS_PER_DEG:.0f}° 초과'; stall_info = None
                        break
                    # Stall guard: sustained error in one direction with no motion -> hold before the motor overloads.
                    err_cmd = targets[fid] - p.position
                    hist = stall[fid]
                    if abs(err_cmd) <= saf['startup_tolerance_deg'] * TICKS_PER_DEG or (hist and hist[-1][2] * err_cmd <= 0):
                        hist = []
                    hist = [v for v in hist if now - v[0] <= 1.6]
                    if abs(err_cmd) > saf['startup_tolerance_deg'] * TICKS_PER_DEG:
                        hist.append((now, p.position, err_cmd))
                    stall[fid] = hist
                    if hist and hist[-1][0] - hist[0][0] >= saf.get('stall_s', 1.4) and max(v[1] for v in hist) - min(v[1] for v in hist) < .5 * TICKS_PER_DEG:
                        blocked_reason = f'ID {fid} 걸림: 목표와 {abs(err_cmd)/TICKS_PER_DEG:.0f}° 차이인데 {saf.get("stall_s", 1.4)}초 이상 움직이지 않음'
                        stall_info = (fid, p.position, 1 if err_cmd > 0 else -1)
                        break
                    lo, hi = bounds[fid]
                    cap = speed * TICKS_PER_DEG * dt
                    anchor = commanded[fid]
                    if (targets[fid] - p.position) * (anchor - p.position) < 0:
                        anchor = float(p.position)
                    g = anchor + max(-cap, min(cap, targets[fid] - anchor))
                    lead = min(saf.get('max_lead_deg', 10.0), max(8.0, speed * .12)) * TICKS_PER_DEG
                    g = max(lo, min(hi, max(p.position - lead, min(p.position + lead, g))))
                    goals[fid] = g
                    all_reached &= abs(g - targets[fid]) < .5 and abs(p.position - targets[fid]) <= saf['startup_tolerance_deg'] * TICKS_PER_DEG
                else:
                    blocked_reason = None
                if blocked_reason:
                    # Hold: re-send current positions once, then wait.
                    for fid in fids:
                        lo, hi = bounds[fid]
                        rig.write(fid, 116, round(max(lo, min(hi, s[fid].position))), 4)
                    phase = 'blocked'; last_goals = {}; blocked_at = now
                    print('자세 유지 (잠시 후 저속 재개): ' + blocked_reason, flush=True)
                    count += 1; continue

                commanded = goals
                rounded = {fid: round(v) for fid, v in goals.items()}
                changed = {fid: g for fid, g in rounded.items() if last_goals.get(fid) != g or last_speed != speed}
                if changed:
                    rig.stream_goals(changed, s, profile_speed_deg_s=speed, period_s=period)
                last_goals = rounded; last_speed = speed
                if phase == 'aligning':
                    settled = settled + 1 if all_reached else 0
                    if settled >= 3:
                        phase = 'following'; print('실시간 추종 중', flush=True)
                pub.send(sim_payload(cfg, s, rounded, phase, cfg.get('sim_source', 'follower')))
                if count % 20 == 0:
                    gaps = ' '.join(f"{j['name']}:{abs(targets[j['follower']['id']]-s[j['follower']['id']].position)/TICKS_PER_DEG:4.1f}°" for j in cfg['joints'])
                    print(f'[{phase}] 목표차 {gaps}' + (f'  범위끝:{limited}' if limited else ''), flush=True)
                count += 1
                time.sleep(max(0, period - (time.monotonic() - begin)))
        except KeyboardInterrupt:
            print('\n중지 요청: 현재 자세 유지 (토크 ON 유지)')
        finally:
            try:
                s = rig.sample()
                for fid in armed:
                    lo, hi = bounds[fid]
                    if s[fid].hardware_error or s[fid].watchdog == 255:
                        continue
                    rig.write(fid, 116, round(max(lo, min(hi, s[fid].position))), 4)
                # Disable the watchdog so the hold survives after this process exits.
                for fid in armed:
                    rig.write(fid, 98, 0, 1)
                print('자세 유지 목표 전송 완료. 토크를 끄려면: python3 sim_teleop/teleop.py --release')
            except Exception as exc:  # noqa: BLE001
                print('유지 목표 전송 실패:', exc)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--status', action='store_true', help='읽기만 하고 종료')
    ap.add_argument('--release', action='store_true', help='팔로워 토크 OFF (팔을 받친 상태에서)')
    ap.add_argument('--dry-run', action='store_true', help='시작 정렬량만 계산하고 모터에 쓰지 않음')
    ap.add_argument('--match', action='store_true', help='시작 시 현재 리더 자세를 현재 팔로워 자세로 간주 (두 팔을 같은 자세로 놓고 실행)')
    args = ap.parse_args()
    cfg = load_config()
    try:
        if args.status:
            with open_rig(cfg, allow_motion=False) as rig:
                s = rig.sample(); print(fmt_states(cfg, rig, s))
                for j in cfg['joints']:
                    fid = j['follower']['id']
                    lo, hi = rig.position_limits(fid)
                    tgt, lim = joint_target(j, s[j['leader']['id']].position, (lo, hi))
                    print(f"  {j['name']:8s} 리더 기준 목표 {tgt:7.0f}  현재 {s[fid].position:5d}  차이 {(tgt-s[fid].position)/TICKS_PER_DEG:6.1f}°  범위 {lo}~{hi}{' (범위끝)' if lim else ''}")
        elif args.release:
            release(cfg)
        else:
            run(cfg, args)
    except SafetyError as exc:
        print('안전 정지:', exc, file=sys.stderr); sys.exit(2)


if __name__ == '__main__':
    main()
