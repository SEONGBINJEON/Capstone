"""Supervised A shoulder tilt check, scoped to one measured pose and motor 12."""
import time
from dataclasses import asdict, replace

from .config import SafetyError, TICKS_PER_DEG
from .control import delta_ticks, health
from .runtime import hold


def check_shoulder_tilt(rig, config, candidate, events, *, hands_clear=False, bases_level=False, tune_position=False, relative_test=False, restore_output=False):
    if tune_position and restore_output:
        raise SafetyError('게인 실험과 출력 복원을 동시에 하지 않습니다.')
    output_ceiling=250
    if not hands_clear or not bases_level:
        raise SafetyError('수평 베이스와 손·측정기 제거 확인이 필요합니다.')
    j = config['pairs']['A'][1]
    c = candidate['pairs']['A'][1]['calibration']
    if j['leader']['id'] != 17 or j['follower']['id'] != 12 or c['direction'] != 1 or c['scale'] != 1:
        raise SafetyError('이번 확인은 A 17→12 동일 비율만 지원합니다.')
    if rig.signature(12) != dict(model=1030, drive_mode=0, operating_mode=3, homing_offset=0):
        raise SafetyError('12번 모델 또는 제어 설정이 바뀌었습니다.')
    if any(j['calibration']['signature'][role] != rig.signature(j[role]['id']) for role in ('leader','follower')):
        raise SafetyError('어깨 보정 당시 설정과 다릅니다.')
    measurement = c['inclination_measurement']
    frames = []
    for _ in range(10):
        s = rig.sample()
        health(config, {i:replace(v, watchdog=0) if i in range(11,16) and v.watchdog==255 else v
                        for i,v in s.items()}, rig.metadata)
        for i,v in s.items():
            if v.torque != int(i in range(11,16)):
                raise SafetyError('리더 OFF / 팔로워 ON 상태가 아닙니다.')
        for i in range(11,16):
            if s[i].present_velocity != 0 or s[i].temperature >= 55:
                raise SafetyError('팔이 정지하지 않았거나 온도 확인이 필요합니다.')
        if s[12].watchdog != 255 or s[12].goal_position is None:
            raise SafetyError('예상한 12번 통신 감시 정지 상태가 아닙니다.')
        if s[12].present_pwm is None or abs(s[12].present_pwm)>250:
            raise SafetyError('어깨 출력이 정지 복구 범위를 벗어났습니다.')
        frames.append(s)
        time.sleep(.05)
    first = frames[-1]
    if restore_output and (first[12].position_p_gain,first[12].position_i_gain,first[12].position_d_gain,first[12].goal_pwm)!=(1200,0,0,250):
        raise SafetyError('출력 복원 전 어깨 설정이 기록과 다릅니다.')
    if tune_position and (first[12].position_p_gain,first[12].position_i_gain,first[12].position_d_gain,first[12].goal_pwm)!=(800,0,0,885):
        raise SafetyError('실험 전 어깨 게인/출력 설정이 예상과 다릅니다.')
    if any(max(f[i].position for f in frames)-min(f[i].position for f in frames)>3 for i in range(11,16)):
        raise SafetyError('정지 확인 중 위치가 바뀌었습니다.')
    origin = first[12].position
    old_goal = first[12].goal_position
    target = round(c['follower_zero'] + delta_ticks(first[17].position, c['leader_zero']))
    candidate_target = target
    if relative_test:
        if not (tune_position or restore_output):
            raise SafetyError('상대 이동 확인은 제한 출력의 어깨 실험에서만 허용합니다.')
        # A stopped servo's reference can drift at communication handovers.
        # This trial is limited to 9 degrees from its NEW observed position;
        # reaching this intermediate point does not validate the calibration.
        target=max(target, origin-round(9*TICKS_PER_DEG))
    lo, hi = rig.position_limits(12)
    # This is an explicitly bounded movement recovery, not a generic hold resume.
    # Both the existing goal and new goal must lift along the inspected path.
    if not lo <= target <= old_goal <= origin <= hi or origin-target > 10*TICKS_PER_DEG:
        raise SafetyError('12번의 확인된 상승 경로 또는 최대10도 이동을 벗어납니다.')
    if origin-old_goal > 2*TICKS_PER_DEG or first[12].position_trajectory != old_goal:
        raise SafetyError('기존 목표가 가까운 정지 궤적이 아니므로 복구하지 않습니다.')
    if abs(first[17].position-measurement['leader_position'])>4 or (not relative_test and abs(origin-measurement['follower_position'])>4):
        raise SafetyError('기울기 측정 후 어깨 위치가 바뀌었습니다.')
    if any(first[i].watchdog not in (0,15,255) for i in range(11,16)):
        raise SafetyError('예상하지 못한 통신 감시 설정')
    events.emit({'schema':'teleop.shoulder_check_plan.v1', 'origin':origin, 'old_goal':old_goal,
                 'target':target, 'max_profile_speed_deg_s':1.374,
                 'recovery_max_move_deg':(origin-old_goal)/TICKS_PER_DEG,
                 'calibration_applied':False, 'relative_test':relative_test,
                 'candidate_target':candidate_target, 'motors':{i:asdict(v) for i,v in first.items()}})
    moving = False

    def sample_checked():
        begin = time.monotonic()
        s = rig.sample()
        now = time.monotonic()
        if now-begin>config['safety']['max_cycle_s'] or any(now-v.sampled_at>config['safety']['max_cycle_s'] for v in s.values()):
            raise SafetyError('어깨 확인 중 통신 지연')
        for i,v in s.items():
            if v.torque != first[i].torque:
                raise SafetyError('토크 상태 변화')
            if i != 12 and (v.watchdog != first[i].watchdog or v.goal_position != first[i].goal_position):
                # Torque-off leaders update their goal registers with hand motion.
                if i in range(11,16):
                    raise SafetyError('다른 팔로워의 유지 상태 변화')
            if i in (11,13,14,15) and abs(v.position-first[i].position)>8:
                raise SafetyError('다른 관절이 유지 범위를 벗어났습니다.')
            if i in (16,17,18,19,20) and abs(delta_ticks(v.position,first[i].position))>2*TICKS_PER_DEG:
                raise SafetyError('확인 중 리더가 움직였습니다.')
        if s[12].watchdog != 15:
            raise SafetyError('12번 통신 감시 복구 상태 변화')
        health(config, {i:replace(v,watchdog=0) if i in (11,13,14,15) and v.watchdog==255 else v
                        for i,v in s.items()}, rig.metadata)
        p = s[12]
        if not target-8 <= p.position <= origin+8:
            raise SafetyError('어깨가 계획한 경로를 벗어났습니다.')
        if p.position_trajectory is None or p.raw_position is None or abs(p.raw_position-p.position_trajectory)>3*TICKS_PER_DEG:
            raise SafetyError('어깨 내부 궤적 추종 오차 초과')
        if p.present_pwm is None or abs(p.present_pwm)>output_ceiling or p.temperature>=55:
            raise SafetyError('어깨 출력 또는 온도 확인 필요')
        return s

    try:
        # The latched servo rejects profile writes too. Only recover when its
        # existing, readable profile is already the reviewed low-speed setting.
        if rig.read_register(12,108,4)!=5 or rig.read_register(12,112,4)!=1:
            raise SafetyError('기존 저속 프로파일이 다르므로 정지 복구를 거부합니다.')
        moving = True
        rig.write(12,98,0,1)
        rig.write(12,98,15,1)
        state = sample_checked()
        if state[12].goal_position != old_goal or state[12].profile_acceleration != 5 or state[12].profile_velocity != 1:
            raise SafetyError('복구 중 기존 목표 또는 저속 프로파일이 바뀌었습니다.')
        events.emit({'schema':'teleop.shoulder_watchdog_recovered.v1','old_goal':old_goal,
                     'position':state[12].position})
        if tune_position:
            rig.tune_a_shoulder(100,250)
            rig.tune_a_shoulder(84,1200)
            events.emit({'schema':'teleop.shoulder_tuning.v1','p_before':800,'p_after':1200,
                         'goal_pwm_before':885,'goal_pwm_after':250,'i':0,'d':0})
        if restore_output:
            rig.restore_a_shoulder_output(885)
            output_ceiling=885
            events.emit({'schema':'teleop.shoulder_output_restored.v1','goal_pwm_before':250,'goal_pwm_after':885,
                         'p_unchanged':1200,'position_speed_guards_unchanged':True})
        rig.write(12,116,target,4)
        start = previous = time.monotonic()
        settled = 0; saturated_since = None
        while time.monotonic()-start < 20:
            state = sample_checked()
            now = time.monotonic()
            if now-previous>config['safety']['max_cycle_s']:
                raise SafetyError('어깨 확인 제어 주기 초과')
            p = state[12]
            if restore_output and (p.position_p_gain!=1200 or p.goal_pwm!=885):
                raise SafetyError('복원한 어깨 출력 또는 게인 설정 변화')
            if tune_position and (p.position_p_gain!=1200 or p.goal_pwm!=250):
                raise SafetyError('어깨 출력 제한 또는 게인 상태 변화')
            if tune_position and abs(p.present_pwm)>=248:
                saturated_since=now if saturated_since is None else saturated_since
                if now-saturated_since>.5:
                    raise SafetyError('제한 출력에 계속 도달해 어깨 이동 중지')
            else:
                saturated_since=None
            if p.goal_position != target:
                raise SafetyError('어깨 목표 읽기 불일치')
            events.emit({'schema':'teleop.shoulder_tilt_check.v1','unix_time':time.time(),
                         'target':target,'motors':{i:asdict(v) for i,v in state.items()}})
            arrived = p.position_trajectory == target and p.present_velocity == 0 and abs(p.position-target)<=2*TICKS_PER_DEG
            settled = settled+1 if arrived else 0
            if settled >= 5:
                result={'schema':'teleop.shoulder_tilt_reached.v1','target':target,'actual':p.position,
                        'candidate_target':candidate_target,'relative_test':relative_test,
                        'actual_move_deg':(p.position-origin)/TICKS_PER_DEG,
                        'remaining_servo_error_deg':(p.position-target)/TICKS_PER_DEG,
                        'requires_physical_angle_verification':True}
                events.emit(result)
                return result
            previous = now
            time.sleep(.04)
        raise SafetyError('20초 이내 어깨 도착 확인 실패')
    except BaseException:
        if moving:
            events.emit({'schema':'teleop.shoulder_check_stopped.v1','hold_errors':hold(rig,{12})})
        raise
