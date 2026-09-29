"""A-pair direct angle teleoperation with explicitly selected motor-limit policy.

Motor limits do not certify collision-free linkage geometry. This mode preserves
that distinction instead of marking unknown mechanical ranges as reviewed.
"""
import json
import time
from dataclasses import asdict,replace
from pathlib import Path

from .config import SafetyError,TICKS_PER_DEG,finite,require_gripper
from .control import delta_ticks,health
from .runtime import arm,hold


class MotionBlocked(SafetyError):
    """Recoverable tracking failure; communication and other arms stay active."""
    def __init__(self,motor_id,message):
        self.motor_id=motor_id
        super().__init__(f'ID {motor_id}: {message}')


class PairPause:
    def __init__(self,controller,state,reason,blocked_at=None):
        self.reason=str(reason);self.near_time=0.;self.moved=False
        self.blocked_at=blocked_at or {};self.retreat_time=0.;self.resume_method=None
        self.goals={j['follower']['id']:state[j['follower']['id']].goal_position for j in controller.joints}
        self.leaders={j['leader']['id']:state[j['leader']['id']].position for j in controller.joints}
        self.gap_deg=float('inf')

    def observe(self,controller,state,dt):
        gaps=[]
        for j in controller.joints:
            lid,fid=j['leader']['id'],j['follower']['id']
            controller.desired[fid]=controller.target(j,state[lid].position)
            gaps.append(abs(controller.desired[fid]-state[fid].position)/TICKS_PER_DEG)
            self.moved |= abs(delta_ticks(state[lid].position,self.leaders[lid]))>2*TICKS_PER_DEG
            if state[fid].goal_position!=self.goals[fid]:
                raise SafetyError(f'ID {fid}: 대기 중 유지 목표가 외부에서 바뀌었습니다.')
        self.gap_deg=max(gaps,default=0.)
        near=self.moved and self.gap_deg<=3.
        self.near_time=self.near_time+dt if near else 0.
        # A reversal that still targets the obstructed side must not retry.
        # Require every joint that was pushing to cross its held angle by 2°.
        retreat=self.moved and bool(self.blocked_at) and all(
            (controller.desired[fid]-v['position'])*v['direction']<=-2*TICKS_PER_DEG
            for fid,v in self.blocked_at.items())
        self.retreat_time=self.retreat_time+dt if retreat else 0.
        self.resume_method=('leader_retreated' if self.retreat_time>=.2 else
                            'leader_pose_matched' if self.near_time>=.5 else None)
        return self.resume_method is not None


def blocked_directions(controller,state,motor_id):
    """Capture the attempted motion before hold cancels the servo trajectory."""
    result={}
    for j in controller.joints:
        fid=j['follower']['id'];v=state[fid]
        error=(v.position_trajectory-v.raw_position
               if v.position_trajectory is not None and v.raw_position is not None
               else controller.commanded[fid]-v.position)
        if fid==motor_id and abs(error)<1:
            error=controller.commanded[fid]-v.position
        if abs(error)>controller.config['safety']['startup_tolerance_deg']*TICKS_PER_DEG or (fid==motor_id and abs(error)>=1):
            result[fid]={'position':v.position,'direction':1 if error>0 else -1}
    # Without evidence of the triggering joint's push direction, use pose rejoin.
    return result if motor_id in result else {}


class DirectA:
    def __init__(self,rig,config,state,pair="A"):
        self.config=config;self.pair=pair;self.joints=[j for j in config['pairs'][pair] if j.get('enabled',True)];self.bounds={}
        tuning=config.get('direct_'+pair,config.get('direct_A',{}))
        self.follow_speed=tuning.get('follow_speed_deg_s',60.)
        self.align_speed=tuning.get('align_speed_deg_s',15.)
        if not finite(self.follow_speed) or not 0<self.follow_speed<=180 or not finite(self.align_speed) or not 0<self.align_speed<=30:
            raise SafetyError('추종 속도는 최대 180°/s, 정렬은 최대 30°/s입니다.')
        self.range_tolerance=2*TICKS_PER_DEG
        self.limited={};self.desired={}
        self.commanded={};self.initial_leaders={};self.last_offsets={}
        self.phase='aligning';self.settled=0;self.elapsed=0.
        self.frozen={}
        for j in self.joints:
            lid,fid=j['leader']['id'],j['follower']['id'];c=j['calibration']
            for role in ('leader','follower'):
                if (c.get('signature') or {}).get(role)!=rig.signature(j[role]['id']):
                    raise SafetyError(f'ID {j[role]["id"]}: 보정 당시 모터 설정과 다릅니다.')
            if rig.metadata[fid]['operating_mode'] not in (3,4) or rig.metadata[fid]['drive_mode']&8:
                raise SafetyError(f'ID {fid}: 지원되지 않는 위치 제어 모드')
            self.bounds[fid]=rig.position_limits(fid)
            if j.get('kind')=='gripper':
                require_gripper(c,pair+'/gripper')
                lo,hi=sorted((c['follower_closed'],c['follower_open']))
                self.bounds[fid]=(max(lo,self.bounds[fid][0]),min(hi,self.bounds[fid][1]))
            elif not all(finite(c.get(k)) for k in ('leader_zero','follower_zero','direction','scale')) or c['direction'] not in (-1,1) or not 0<c['scale']<=10:
                raise SafetyError(f'ID {fid}: 관절 기준점/방향/비율이 필요합니다.')
            lo,hi=self.bounds[fid]
            if not lo-self.range_tolerance<=state[fid].position<=hi+self.range_tolerance:raise SafetyError(f'ID {fid}: 현재 위치가 모터 범위 밖입니다.')
            self.initial_leaders[lid]=state[lid].position
            self.last_offsets[lid]=self.offset(j,state[lid].position)
            self.frozen[fid]=self.target(j,state[lid].position)
            if abs(self.frozen[fid]-state[fid].position)>150*TICKS_PER_DEG:
                raise SafetyError(f'ID {fid}: 시작 정렬 이동량이150도를 넘습니다.')
            self.commanded[fid]=max(lo,min(hi,float(state[fid].goal_position if state[fid].torque else state[fid].position)))
        self.stall={fid:[] for fid in self.commanded}

    def offset(self,j,p):
        c=j['calibration']
        return delta_ticks(p,c['leader_closed'] if j.get('kind')=='gripper' else c['leader_zero'])

    def target(self,j,p):
        c=j['calibration'];fid=j['follower']['id']
        if j.get('kind')=='gripper':
            fraction=self.offset(j,p)/delta_ticks(c['leader_open'],c['leader_closed'])
            fraction=max(0.,min(1.,fraction))
            goal=c['follower_closed']+fraction*(c['follower_open']-c['follower_closed'])
        else:
            goal=c['follower_zero']+self.offset(j,p)*c['direction']*c['scale']
        lo,hi=self.bounds[fid]
        self.limited[fid]=not lo<=goal<=hi
        return max(lo,min(hi,goal))

    def step(self,state,dt):
        if not 0<dt<=self.config['safety']['max_cycle_s']:raise SafetyError('추종 제어 주기 초과')
        next_elapsed=self.elapsed+dt
        if self.phase=='aligning' and next_elapsed>180:raise SafetyError('시작 정렬 시간 초과')
        speed=self.align_speed if self.phase=='aligning' else self.follow_speed
        next_goals={};offsets={};stalls={};all_reached=True
        for j in self.joints:
            lid,fid=j['leader']['id'],j['follower']['id'];p=state[fid];offset=self.offset(j,state[lid].position)
            if abs(offset-self.last_offsets[lid])>self.config['safety']['max_leader_step_deg']*TICKS_PER_DEG:
                raise SafetyError(f'ID {lid}: 리더 위치 급변 또는180도 경계 통과')
            lo,hi=self.bounds[fid]
            if not lo-self.range_tolerance<=p.position<=hi+self.range_tolerance:raise SafetyError(f'ID {fid}: 팔로워 모터 범위 초과')
            tracking_error=(p.raw_position-p.position_trajectory if p.raw_position is not None and p.position_trajectory is not None else p.position-self.commanded[fid])
            if abs(tracking_error)>self.config['safety']['max_tracking_error_deg']*TICKS_PER_DEG:
                raise MotionBlocked(fid,'추종 오차 초과')
            # Always replace intent with the newest leader sample, including
            # startup. Never finish an obsolete pose before accepting a return.
            target=self.target(j,state[lid].position)
            self.desired[fid]=target
            cap=speed*TICKS_PER_DEG*dt
            anchor=self.commanded[fid]
            if (target-p.position)*(anchor-p.position)<0:
                anchor=float(p.position)
            goal=anchor+max(-cap,min(cap,target-anchor))
            # Bound command lead so large hand motions cannot accumulate a
            # long unfinished servo trajectory. This is not a command queue.
            lead=max(8.,speed*.12)*TICKS_PER_DEG
            goal=max(lo,min(hi,max(p.position-lead,min(p.position+lead,goal))))
            next_goals[fid]=goal;offsets[lid]=offset
            error=self.commanded[fid]-p.position
            history=self.stall[fid]
            # Idle time is not failed motion. Require sustained outstanding
            # error in the same direction before evaluating lack of progress.
            if abs(error)<=self.config['safety']['startup_tolerance_deg']*TICKS_PER_DEG or (history and history[-1][2]*error<=0):
                history=[]
            else:
                history=[v for v in history if next_elapsed-v[0]<=1.5]
            if abs(error)>self.config['safety']['startup_tolerance_deg']*TICKS_PER_DEG:
                history.append((next_elapsed,p.position,error))
            stalls[fid]=history
            if history and history[-1][0]-history[0][0]>=1.4 and max(v[1] for v in history)-min(v[1] for v in history)<.25*TICKS_PER_DEG:
                raise MotionBlocked(fid,'목표와 차이가 있지만 모터가 움직이지 않습니다.')
            all_reached &= abs(goal-target)<.5 and abs(p.position-target)<=self.config['safety']['startup_tolerance_deg']*TICKS_PER_DEG
        self.commanded=next_goals;self.last_offsets=offsets;self.stall=stalls;self.elapsed=next_elapsed
        if self.phase=='aligning':
            self.settled=self.settled+1 if all_reached else 0
            if self.settled>=3:self.phase='following'
        return {i:round(v) for i,v in next_goals.items()}


def run_direct(rig,config,events,*,pair="A",motor_limits=False,command=None,control_path=None):
    if not motor_limits:raise SafetyError('모터 제한을 사용하는 A 직접 추종 모드를 명시해야 합니다.')
    pairs=['A','B'] if pair=='both' else [pair]
    if any(p not in ('A','B') for p in pairs):raise SafetyError('지원하지 않는 팔 선택')
    followers={j['follower']['id'] for p in pairs for j in config['pairs'][p] if j.get('enabled',True)}
    leaders={j['leader']['id'] for p in pairs for j in config['pairs'][p] if j.get('enabled',True)}
    states=[]
    for _ in range(5):
        s=rig.sample()
        health(config,{i:replace(v,watchdog=0) if i in followers and v.watchdog==255 else v for i,v in s.items()},rig.metadata)
        states.append(s);time.sleep(.05)
    first=states[-1];armed=set()
    expected_torque={i:v.torque for i,v in first.items()}
    expected_torque.update({i:1 for i in followers})
    if any(first[i].torque for i in leaders):raise SafetyError('리더 토크가 켜져 있습니다.')
    for p in pairs:
        if len({first[j['follower']['id']].torque for j in config['pairs'][p] if j.get('enabled',True)})!=1:
            raise SafetyError(p+' 팔로워 토크 상태가 섞여 있습니다.')
    for i in followers:
        if max(s[i].position for s in states)-min(s[i].position for s in states)>4:raise SafetyError('시작 준비 중 팔로워가 움직였습니다.')
        if first[i].torque:
            lo,hi=rig.position_limits(i)
            if first[i].watchdog not in (0,15,255) or first[i].goal_position is None or not lo<=first[i].goal_position<=hi or abs(first[i].goal_position-first[i].position)>config['safety']['startup_tolerance_deg']*TICKS_PER_DEG:
                raise SafetyError(f'ID {i}: 기존 유지 목표에 충분히 가깝지 않습니다.')
            if first[i].position_trajectory!=rig.raw_goal(i,first[i].goal_position) or first[i].present_velocity!=0:
                raise SafetyError(f'ID {i}: 기존 이동이 아직 끝나지 않았습니다.')
        elif first[i].watchdog not in (0,15):raise SafetyError('토크 OFF 상태에 남은 통신 감시 오류 확인 필요')
        elif first[i].watchdog==15 and (first[i].goal_position is None or abs(first[i].goal_position-first[i].position)>8):
            raise SafetyError(f'ID {i}: 중단된 토크 준비 목표 확인 필요')
    controllers={p:DirectA(rig,config,first,p) for p in pairs}
    paused={}
    initial_alignment=True
    start_paused=set((command or {}).get('start_paused_pairs',[]))
    if not start_paused<=set(pairs):raise SafetyError('대기할 팔 선택이 잘못됐습니다.')
    resume_token=(command or {}).get('resume_token')
    rig.write_ids=followers
    reason='stopped';phase=None
    try:
        for p in pairs:
            ids={j['follower']['id'] for j in config['pairs'][p] if j.get('enabled',True)}
            if first[next(iter(ids))].torque:
                for i in sorted(ids):
                    armed.add(i)
                    if first[i].watchdog==255:rig.write(i,98,0,1)
                    rig.write(i,98,15,1)
            else:
                for i in ids:
                    if first[i].watchdog==15:rig.write(i,98,0,1)
                if p=='A':rig.restore_a_shoulder_gain()
                arm(rig,config,p,first,armed)
        for p in start_paused:
            paused[p]=PairPause(controllers[p],first,'장애물 시험 후 대기')
        if paused:initial_alignment=False
        previous=time.monotonic();count=0;last_goals={};last_speeds={}
        while True:
            begin=time.monotonic();s=rig.sample();now=time.monotonic()
            if any(now-v.sampled_at>config['safety']['max_cycle_s'] for v in s.values()):raise SafetyError('오래된 모터 응답')
            request=json.loads(Path(control_path).read_text()) if control_path else {}
            if control_path and request.get('nonce')!=command.get('nonce'):return
            manual_resume=set()
            if request.get('resume_token')!=resume_token:
                resume_token=request.get('resume_token')
                manual_resume=set(request.get('resume_pairs',[])) & set(paused)
            health(config,s,rig.metadata)
            if any(s[i].torque!=expected_torque[i] for i in s):raise SafetyError('추종 중 토크 상태 변화')
            if any(s[i].watchdog!=15 for i in followers):raise SafetyError('팔로워 통신 감시 상태 변화')
            goals={};desired={};limited=[];pair_speeds={};motion_packets={}
            for p,controller in list(controllers.items()):
                if p in paused:
                    ready=paused[p].observe(controller,s,now-previous)
                    if ready or p in manual_resume:
                        resume_method='explicit' if p in manual_resume else paused[p].resume_method
                        try:
                            controller=DirectA(rig,config,s,p)
                        except SafetyError as exc:
                            paused[p].reason='재개 대기: '+str(exc)
                        else:
                            # Automatic rejoin is an ongoing operator motion,
                            # not a fresh startup. Anchor at the held goal and
                            # use normal slew/profile limits immediately.
                            if resume_method in ('leader_retreated','leader_pose_matched'):
                                controller.phase='following'
                            controllers[p]=controller;del paused[p]
                            initial_alignment=False
                            events.emit({'schema':'teleop.pair_resumed.v1','unix_time':time.time(),'pair':p,
                                         'method':resume_method})
                            print(p+(' 실시간 추종 재개' if controller.phase=='following' else ' 저속 정렬 후 추종 재개'),flush=True)
                    if p in paused:
                        goals.update(paused[p].goals);desired.update(controller.desired)
                        limited.extend(i for i,v in controller.limited.items() if v)
                        pair_speeds[p]=0.
                        continue
                if initial_alignment:controller.phase='aligning'
                starting=controller.phase=='aligning'
                try:
                    pair_goals=controller.step(s,now-previous)
                except MotionBlocked as exc:
                    ids={j['follower']['id'] for j in controller.joints}
                    blocked_at=blocked_directions(controller,s,exc.motor_id)
                    # Remove the unreachable goals once, retaining stationary
                    # load-bearing goals on the unaffected joints of this arm.
                    errors=hold(rig,ids,preserve_stationary=True)
                    if errors:raise SafetyError('막힌 팔 유지 실패: '+'; '.join(errors))
                    s=rig.sample();health(config,s,rig.metadata)
                    # Hold may finish a little later than the triggering sample.
                    for fid,v in blocked_at.items():v['position']=s[fid].position
                    paused[p]=PairPause(controller,s,exc,blocked_at);initial_alignment=False
                    paused[p].observe(controller,s,0.)
                    goals.update(paused[p].goals);desired.update(controller.desired)
                    pair_speeds[p]=0.
                    events.emit({'schema':'teleop.pair_paused.v1','unix_time':time.time(),'pair':p,
                                 'motor_id':exc.motor_id,'reason':str(exc),'hold_goals':paused[p].goals,
                                 'blocked_at':blocked_at})
                    print(p+' 자세 유지 대기: '+str(exc)+' / 다른 팔과 리더 읽기는 계속',flush=True)
                    continue
                goals.update(pair_goals);desired.update(controller.desired)
                limited.extend(i for i,v in controller.limited.items() if v)
                profile_speed=min(60,2*controller.align_speed) if starting else controller.follow_speed
                pair_speeds[p]=controller.align_speed if starting else controller.follow_speed
                changed={i:g for i,g in pair_goals.items() if last_goals.get(i)!=g or last_speeds.get(p)!=profile_speed}
                if changed:motion_packets.setdefault(profile_speed,{}).update(changed)
                last_speeds[p]=profile_speed
            if initial_alignment and all(c.phase=='following' for c in controllers.values()):initial_alignment=False
            for speed,changed in motion_packets.items():
                rig.stream_goals(changed,s,profile_speed_deg_s=speed,period_s=.05)
            last_goals=goals
            global_phase=('paused' if len(paused)==len(controllers) else 'partial' if paused else
                          'aligning' if any(c.phase=='aligning' for c in controllers.values()) else 'following')
            if phase!=global_phase:
                phase=global_phase
                label={'aligning':'시작 자세 정렬 중','following':'실시간 추종 중','partial':'일부 팔 대기 / 나머지 추종','paused':'자세 유지 대기 / 연결 유지'}[phase]
                print(pair+' '+label,flush=True)
            if count%4==0:
                payload={'schema':'teleop.direct_A.v1' if pair=='A' else 'teleop.direct.v1',
                         'unix_time':time.time(),'pair':pair,'phase':phase,'targets':goals,
                         'desired':desired,'limited_ids':limited,'pair_speeds_deg_s':pair_speeds,
                         'pair_phases':{p:'paused' if p in paused else c.phase for p,c in controllers.items()},
                         'paused_pairs':{p:{'reason':v.reason,'rejoin_gap_deg':v.gap_deg,
                                           'blocked_at':v.blocked_at} for p,v in paused.items()},
                         'command_speed_deg_s':max(pair_speeds.values(),default=0),
                         'range_policy':'motor_limits_not_collision_model','motors':{i:asdict(v) for i,v in s.items()}}
                events.emit(payload)
                write_status(payload)
            previous=now;count+=1;time.sleep(max(0,.05-(time.monotonic()-begin)))
    except BaseException as exc:
        reason=type(exc).__name__;raise
    finally:
        errors=hold(rig,armed,preserve_stationary=reason in ('stopped','KeyboardInterrupt'))
        events.emit({'schema':'teleop.direct_A_stopped.v1','unix_time':time.time(),'pair':pair,'reason':reason,'hold_errors':errors})
        write_status({'unix_time':time.time(),'pair':pair,'phase':'hold','reason':reason,'hold_errors':errors})


def write_status(payload):
    import os
    payload=dict(payload,pid=os.getpid())
    path=Path('work/teleop_status.json');path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(payload,ensure_ascii=False));tmp.replace(path)


def run_direct_A(rig,config,events,**kwargs):
    return run_direct(rig,config,events,pair='A',**kwargs)
