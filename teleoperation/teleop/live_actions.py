"""Bounded commissioning actions used by the persistent hardware connection."""
import json
import time
from dataclasses import asdict, replace
from pathlib import Path

from .config import SafetyError, TICKS_PER_DEG
from .control import delta_ticks, health
from .runtime import hold


class ShoulderTrial:
    def __init__(self, config, state):
        self.c = config['pairs']['A'][1]['calibration']
        if not self.c.get('pose_alignment_verified') or self.c['direction']!=1 or self.c['scale']!=1:
            raise SafetyError('확인된 A 어깨 보정이 필요합니다.')
        self.leader_start = self.last_leader = state[17].position
        self.goal = float(state[12].goal_position)
        self.origin = state[12].position
        self.center = self.target(self.leader_start)
        if abs(self.center-self.origin)>5*TICKS_PER_DEG:
            raise SafetyError('어깨 시작 자세 차이가5도를 넘습니다.')
        self.phase = 'aligning'
        self.settled = 0

    def target(self, position):
        return self.c['follower_zero']+delta_ticks(position,self.c['leader_zero'])

    def step(self, state, dt):
        if not 0<dt<=.15:
            raise SafetyError('추종 제어 시간 초과')
        l,p=state[17],state[12]
        if abs(delta_ticks(l.position,self.leader_start))>5*TICKS_PER_DEG:
            raise SafetyError('첫 어깨 추종의 시작점 ±5도 범위를 벗어났습니다.')
        if abs(delta_ticks(l.position,self.last_leader))>5*TICKS_PER_DEG:
            raise SafetyError('리더 위치 점프')
        if self.phase=='aligning' and abs(delta_ticks(l.position,self.leader_start))>2*TICKS_PER_DEG:
            raise SafetyError('시작 정렬 중 리더가 움직였습니다.')
        if p.position_trajectory is None or p.raw_position is None or abs(p.raw_position-p.position_trajectory)>3*TICKS_PER_DEG:
            raise SafetyError('어깨 내부 궤적 추종 오차 초과')
        if not self.center-5*TICKS_PER_DEG-8 <= p.position <= max(self.origin,self.center+5*TICKS_PER_DEG)+8:
            raise SafetyError('어깨 시험 위치 범위 초과')
        target=self.center if self.phase=='aligning' else self.target(l.position)
        cap=2*TICKS_PER_DEG*dt
        self.goal+=max(-cap,min(cap,target-self.goal))
        self.last_leader=l.position
        if self.phase=='aligning':
            reached=abs(self.goal-self.center)<.5 and abs(p.position-self.center)<=2*TICKS_PER_DEG and abs(p.position_trajectory-round(self.center))<=1
            self.settled=self.settled+1 if reached else 0
            if self.settled>=3:self.phase='following'
        return round(self.goal)


def publish_service(control_path):
    import os
    Path('work/teleop_service.json').write_text(json.dumps({'pid':os.getpid(),'control_path':str(control_path)}))


def execute_action(rig, config, events, command, control_path):
    publish_service(control_path)
    if command.get('action')=='torque_off':
        from .direct import write_status
        ids={j['follower']['id'] for entries in config['pairs'].values()
             for j in entries if j.get('enabled',True)}
        rig.write_ids=ids
        errors=[]
        for i in sorted(ids):
            try:rig.write(i,64,0,1)
            except Exception as exc:errors.append(f'ID {i} 토크 OFF: {exc}')
        for i in sorted(ids):
            try:rig.write(i,98,0,1)
            except Exception as exc:errors.append(f'ID {i} 통신 감시 해제: {exc}')
        state=rig.sample()
        remaining=[i for i in sorted(ids) if state[i].torque]
        if remaining:errors.append('토크가 남은 모터: '+str(remaining))
        payload={'schema':'teleop.torque_off.v1','unix_time':time.time(),'pair':'both',
                 'phase':'fault' if errors else 'torque_off','reason':'; '.join(errors) or '사용자 요청 토크 OFF',
                 'released_ids':sorted(ids),'motors':{i:asdict(v) for i,v in state.items()}}
        events.emit(payload);write_status(payload)
        if errors:raise SafetyError('; '.join(errors))
        print('팔로워 토크 OFF 확인: '+str(sorted(ids)),flush=True)
        return
    if command.get('action')=='prepare_B_base':
        from .config import save
        import importlib
        from types import MethodType
        from . import hardware
        importlib.reload(hardware)
        rig.sample=MethodType(hardware.Rig.sample,rig)
        s=rig.sample();j=config['pairs']['B'][0];cal=j['calibration']
        if (rig.signature(21)!=cal['signature']['follower'] or s[21].torque or s[21].watchdog
                or rig.metadata[21]['operating_mode']!=3 or cal.get('basis')!='user_confirmed_B_upright_2026_09_13'):
            raise SafetyError('B 베이스 기준점/토크 OFF 상태 확인 필요')
        Path('work/backups/B_base_before_extended.json').write_text(json.dumps({'config':config,'state':asdict(s[21]),'metadata':rig.metadata[21]},indent=2))
        window={'reference':2048,'raw_reference':int(cal['follower_zero']),'min':1,'max':4095}
        rig.specs[21]['extended_position']=window
        rig.write_ids={21}
        rig.set_extended_mode(21)
        revised=rig.sample()
        if revised[21].torque or abs(delta_ticks(revised[21].raw_position,s[21].raw_position))>16:
            raise SafetyError('B 베이스 모드 변경 중 위치 변화')
        j['follower']['extended_position']=window;cal['follower_zero']=2048
        cal['signature']['follower']=rig.signature(21)
        save('config/robot.json',config);rig.config=config
        print('B 베이스 양방향 좌표 설정 완료. 토크 OFF 유지.',flush=True)
        return
    if command.get('action')=='inspect_B':
        import importlib
        from . import hardware
        importlib.reload(hardware)
        s=hardware.extend_rig(rig,config,'B')
        Path('work/current_both_inspection.json').write_text(json.dumps(hardware.report(rig,s),indent=2))
        events.emit({'schema':'teleop.both_inspected.v1','unix_time':time.time()})
        print('A/B 20개 모터 읽기 연결 완료. B 쓰기 없음.',flush=True)
        return
    if command.get('action')=='hold_A':
        from .runtime import hold_preflight,arm
        frames=[rig.sample() for _ in range(5)]
        first=frames[-1]
        hold_preflight(rig,config,'A',first)
        if any(first[i].torque for i in range(16,21)):
            raise SafetyError('리더 토크가 켜져 있습니다.')
        if any(max(s[i].position for s in frames)-min(s[i].position for s in frames)>4 for i in range(11,16)):
            raise SafetyError('팔로워를 안정적으로 받쳐 주세요.')
        rig.write_ids=set(range(11,16));armed=set()
        try:
            arm(rig,config,'A',first,armed)
            # Reapply the same previously validated shoulder RAM tuning after
            # the board reset, while the user supports the stationary arm.
            rig.tune_a_shoulder(100,250)
            rig.tune_a_shoulder(84,1200)
            rig.restore_a_shoulder_output(885)
        except BaseException:
            hold(rig,armed)
            raise
        events.emit({'schema':'teleop.A_hold_ready.v1','unix_time':time.time(),
                     'targets':{i:first[i].position for i in range(11,16)}})
        print('A 현재 자세 유지 ON. 리더 추종은 손을 놓은 뒤 시작합니다.',flush=True)
        return
    if command.get('action') in ('follow_A','follow_B','follow_both'):
        import importlib
        from types import MethodType
        from . import config as configuration, control, hardware, runtime, direct
        # Fixed local modules only; keep existing serial ownership and torque.
        importlib.reload(configuration)
        importlib.reload(control)
        importlib.reload(hardware)
        importlib.reload(runtime)
        importlib.reload(direct)
        disabled={j[r]['id'] for entries in config['pairs'].values() for j in entries
                  if not j.get('enabled',True) for r in ('leader','follower')}
        for i in disabled:
            spec=rig.specs.pop(i,None)
            if spec:
                rig.groups[spec['bus']].removeParam(i)
                rig.metadata.pop(i,None);rig.follower_ids.discard(i);rig.write_ids.discard(i)
                rig.extended_turn_offsets.pop(i,None)
        rig.config=config
        rig.stream_goals=MethodType(hardware.Rig.stream_goals,rig)
        rig.sample=MethodType(hardware.Rig.sample,rig)
        rig.restore_a_shoulder_gain=MethodType(hardware.Rig.restore_a_shoulder_gain,rig)
        for i in sorted(rig.follower_ids):
            rig.metadata[i]['velocity_limit']=rig.read_register(i,44,4)
        rig.sample()
        pair={'follow_A':'A','follow_B':'B','follow_both':'both'}[command['action']]
        try:
            direct.write_status({'unix_time':time.time(),'pair':pair,'phase':'preparing'})
            return direct.run_direct(rig,config,events,pair=pair,motor_limits=command.get('motor_limits') is True,command=command,control_path=control_path)
        except BaseException as exc:
            import traceback
            Path('work/teleop_last_error.txt').write_text(traceback.format_exc())
            direct.write_status({'unix_time':time.time(),'pair':pair,'phase':'fault','reason':str(exc)})
            raise
    if command.get('action')!='follow_shoulder':
        raise SafetyError('지원하지 않는 실물 동작 요청')
    frames=[]
    for _ in range(10):
        s=rig.sample()
        health(config,{i:replace(v,watchdog=0) if i in range(11,16) and v.watchdog==255 else v for i,v in s.items()},rig.metadata)
        if any(s[i].torque!=int(i in range(11,16)) for i in s):raise SafetyError('토크 상태 변경')
        frames.append(s);time.sleep(.05)
    first=frames[-1]
    for i in range(11,16):
        if max(f[i].position for f in frames)-min(f[i].position for f in frames)>4 or first[i].present_velocity!=0:
            raise SafetyError('시작 자세가 정지 상태가 아닙니다.')
    j=config['pairs']['A'][1]
    for role in ('leader','follower'):
        if rig.signature(j[role]['id'])!=j['calibration']['signature'][role]:raise SafetyError('어깨 설정 변경')
    p=first[12]
    if (p.position_p_gain,p.position_i_gain,p.position_d_gain,p.goal_pwm)!=(1200,0,0,885):raise SafetyError('어깨 제어 설정이 확인 당시와 다릅니다.')
    if p.watchdog not in (15,255) or p.goal_position is None or abs(p.goal_position-p.position)>2*TICKS_PER_DEG or p.position_trajectory!=p.goal_position:
        raise SafetyError('어깨의 정지 목표/통신 감시 상태 확인 필요')
    if p.profile_acceleration!=5 or p.profile_velocity not in (1,2):raise SafetyError('기존 저속 설정 확인 필요')
    trial=ShoulderTrial(config,first)
    lo,hi=rig.position_limits(12)
    if not lo<=trial.center-5*TICKS_PER_DEG<=trial.center+5*TICKS_PER_DEG<=hi:raise SafetyError('시험 목표가 모터 제한을 벗어납니다.')
    rig.write_ids={12}
    try:
        if p.watchdog==255:rig.write(12,98,0,1)
        rig.write(12,98,15,1)
        rig.write(12,112,2,4)  # 2.748deg/s hardware limit; commands slew at2deg/s.
        start=previous=time.monotonic();motion_started=None;last_goal=p.goal_position
        announced=False
        while True:
            begin=time.monotonic();s=rig.sample();now=time.monotonic()
            if any(now-v.sampled_at>.15 for v in s.values()):raise SafetyError('오래된 위치 응답')
            control=json.loads(Path(control_path).read_text())
            if control.get('nonce')!=command.get('nonce'):return
            for i,v in s.items():
                if v.torque!=first[i].torque:raise SafetyError('추종 중 토크 상태 변화')
                if i in (11,13,14,15) and (v.goal_position!=first[i].goal_position or v.watchdog!=first[i].watchdog or abs(v.position-first[i].position)>8):
                    raise SafetyError('다른 팔로워의 유지 상태 변화')
            if s[12].watchdog!=15:raise SafetyError('어깨 통신 감시 오류')
            health(config,{i:replace(v,watchdog=0) if i in (11,13,14,15) and v.watchdog==255 else v for i,v in s.items()},rig.metadata)
            if s[12].temperature>=55:raise SafetyError('어깨 온도 확인 필요')
            goal=trial.step(s,now-previous)
            if goal!=last_goal:rig.write(12,116,goal,4);last_goal=goal
            if trial.phase=='following' and not announced:
                print('리더17→팔로워12 추종 준비 완료. 시작 자세에서 ±5도, 최대2도/초.',flush=True);announced=True
            if trial.phase=='following' and abs(delta_ticks(s[17].position,trial.leader_start))>4 and motion_started is None:motion_started=now
            events.emit({'schema':'teleop.shoulder_follow.v1','unix_time':time.time(),'phase':trial.phase,'target':goal,
                         'leader_start':trial.leader_start,'motors':{i:asdict(v) for i,v in s.items()}})
            if (motion_started and now-motion_started>60) or now-start>180:return
            if trial.phase=='aligning' and now-start>20:raise SafetyError('시작 정렬 시간 초과')
            previous=now;time.sleep(max(0,.05-(time.monotonic()-begin)))
    finally:
        events.emit({'schema':'teleop.shoulder_follow_stopped.v1','hold_errors':hold(rig,{12},preserve_stationary=True)})
