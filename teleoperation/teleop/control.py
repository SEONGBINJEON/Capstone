import math
from .config import SafetyError, TICKS_PER_DEG, joints, require_calibration, follower_bounds


def delta_ticks(position, zero):
    """Continuous around encoder wrap; calibrated leader travel must stay within ±180°."""
    return (position - zero + 2048) % 4096 - 2048


def health(config, states, metadata):
    for _,j in joints(config):
        fid=j['follower']['id']
        if fid in states and metadata[fid]['operating_mode']==4:
            w=j['follower'].get('extended_position')
            if not w or not w['min']-2*TICKS_PER_DEG <= states[fid].position <= w['max']+2*TICKS_PER_DEG:
                raise SafetyError(f'ID {fid}: 확장 위치 제어의 소프트웨어 범위를 벗어났습니다.')
    for i, s in states.items():
        if s.hardware_error:
            raise SafetyError(f"ID {i}: 하드웨어 오류 0x{s.hardware_error:02x}")
        if s.temperature >= config["safety"]["max_temperature_c"]:
            raise SafetyError(f"ID {i}: 온도 제한 초과 ({s.temperature}°C)")
        if s.watchdog == 255:
            raise SafetyError(f"ID {i}: 통신 감시 타이머 오류. 상태 확인 후 수동 복구가 필요합니다.")
    for i, s in states.items():
        lo, hi = metadata[i]['voltage_min'], metadata[i]['voltage_max']
        if metadata[i]['model'] == 1200:
            lo, hi = max(lo, 3.7), min(hi, 6.0)
        if not lo <= s.voltage <= hi:
            raise SafetyError(f"ID {i}: 설정된 전압 범위 초과 ({s.voltage}V, {lo}~{hi}V)")



class Controller:
    def __init__(self, config, pair, initial, startup_mode='matched'):
        require_calibration(config, pair)
        if startup_mode not in ('matched', 'align'):
            raise SafetyError('지원되지 않는 시작 방식입니다.')
        self.config = config
        self.entries = joints(config, pair)
        self.commanded = {j['follower']['id']: float(initial[j['follower']['id']].position) for _, j in self.entries}
        self.last_leader = {j['leader']['id']: initial[j['leader']['id']].position for _, j in self.entries}
        self.start_leader = self.last_leader.copy()
        self.phase = 'aligning' if startup_mode == 'align' else 'following'
        self.alignment_targets = {}
        self.alignment_elapsed = 0.
        self.settled_frames = 0
        for _, j in self.entries:
            c = j['calibration']; f = j['follower']['id']
            lo, hi = self.bounds(j)
            if not lo <= initial[f].position <= hi:
                raise SafetyError(f"ID {f}: 시작 위치가 보정된 이동 범위 밖입니다.")
            target, saturated = self.target(j, initial[j['leader']['id']].position)
            if startup_mode == 'align' and saturated:
                raise SafetyError(f'ID {f}: 리더의 시작 자세에 대응하는 목표가 팔로워 이동 범위 밖입니다.')
            self.alignment_targets[f] = target
            if startup_mode == 'matched' and abs(target-initial[f].position)/TICKS_PER_DEG > config['safety']['startup_tolerance_deg']:
                raise SafetyError(f"ID {f}: 시작 자세 차이가 큽니다. 리더와 팔로워를 기준 자세에 맞추세요.")

    @property
    def speed_deg_s(self):
        limit = self.config['safety']['max_speed_deg_s']
        return min(5., limit) if self.phase == 'aligning' else limit

    def bounds(self, joint):
        return follower_bounds(joint)

    def target(self, joint, leader_position):
        c = joint['calibration']
        if joint.get('kind') == 'gripper':
            travel = delta_ticks(c['leader_open'], c['leader_closed'])
            fraction = delta_ticks(leader_position, c['leader_closed']) / travel
            clipped = max(0., min(1., fraction))
            return c['follower_closed'] + clipped*(c['follower_open']-c['follower_closed']), fraction != clipped
        leader_angle = delta_ticks(leader_position, c['leader_zero']) / TICKS_PER_DEG
        if not c['leader_min_deg'] <= leader_angle <= c['leader_max_deg']:
            raise SafetyError(f"ID {joint['leader']['id']}: 보정된 리더 이동 범위 초과")
        raw = c['follower_zero'] + leader_angle * TICKS_PER_DEG * c['direction'] * c['scale']
        lo, hi = self.bounds(joint)
        limited = max(lo, min(hi, raw))
        return limited, limited != raw

    def step(self, states, dt, check_tracking=True):
        if not math.isfinite(dt) or not 0 < dt <= self.config['safety']['max_cycle_s']:
            raise SafetyError("제어 루프 시간 초과: 오래된 위치로 추종하지 않습니다.")
        aligning = self.phase == 'aligning'
        if aligning:
            if self.alignment_elapsed + dt > 60:
                raise SafetyError('시작 정렬 시간 초과: 현재 자세에서 중지합니다.')
            for lid, start in self.start_leader.items():
                if abs(delta_ticks(states[lid].position, start)) > 2*TICKS_PER_DEG:
                    raise SafetyError(f'ID {lid}: 시작 정렬 중 리더가 움직였습니다. 리더를 안정적으로 둔 뒤 다시 시작하세요.')
        result = {}; next_command = {}; next_leader = {}; details = []
        for pair, j in self.entries:
            lid, fid = j['leader']['id'], j['follower']['id']
            if abs(delta_ticks(states[lid].position, self.last_leader[lid]))/TICKS_PER_DEG > self.config['safety']['max_leader_step_deg']:
                raise SafetyError(f"ID {lid}: 리더 위치가 갑자기 변했습니다.")
            if check_tracking and abs(states[fid].position-self.commanded[fid])/TICKS_PER_DEG > self.config['safety']['max_tracking_error_deg']:
                raise SafetyError(f"ID {fid}: 팔로워 추종 오차가 큽니다.")
            target, saturated = self.target(j, states[lid].position)
            if aligning:
                target, saturated = self.alignment_targets[fid], False
            cap = self.speed_deg_s*TICKS_PER_DEG*dt
            command = self.commanded[fid]+max(-cap,min(cap,target-self.commanded[fid]))
            next_command[fid] = command; next_leader[lid] = states[lid].position
            result[fid] = round(command)
            c = j['calibration']
            reference = c['follower_closed'] if j.get('kind') == 'gripper' else c['follower_zero']
            details.append({'pair':pair,'joint':j['name'],'kind':j.get('kind','joint'),'leader_id':lid,'follower_id':fid,
                            'target_ticks':result[fid], 'target_motor_offset_rad':(result[fid]-reference)*2*math.pi/4096,
                            'actual_ticks':states[fid].position,'saturated':saturated})
            if j.get('kind') == 'gripper':
                details[-1]['gripper_open_fraction'] = (result[fid]-c['follower_closed'])/(c['follower_open']-c['follower_closed'])
        self.commanded.update(next_command); self.last_leader.update(next_leader)
        if aligning:
            self.alignment_elapsed += dt
            reached = all(abs(self.commanded[f]-target) < 1e-6 and
                          (not check_tracking or abs(states[f].position-target) <= 2*TICKS_PER_DEG)
                          for f,target in self.alignment_targets.items())
            self.settled_frames = self.settled_frames + 1 if reached else 0
            if self.settled_frames >= 3:
                self.phase = 'following'
        return result, details
