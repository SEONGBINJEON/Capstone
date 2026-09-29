"""Slow alignment to one operator-confirmed pose. This does not certify joint ranges."""
import math
import time
from dataclasses import asdict

from .config import SafetyError, TICKS_PER_DEG, finite, joints, require_gripper, follower_bounds
from .control import delta_ticks, health
from .runtime import hold


def confirmed_pose_targets(rig,config,pair,state):
    targets={}
    for _,j in joints(config,pair):
        lid,fid=j['leader']['id'],j['follower']['id'];c=j['calibration']
        for role in ('leader','follower'):
            if (c.get('signature') or {}).get(role)!=rig.signature(j[role]['id']):
                raise SafetyError(f'ID {j[role]["id"]}: 보정 이후 모터 설정이 바뀌었습니다.')
        if state[lid].torque or not state[fid].torque:
            raise SafetyError('리더 토크 OFF, 팔로워 자세 유지 상태에서만 정렬합니다.')
        if rig.metadata[fid]['operating_mode'] not in (3,4) or rig.metadata[fid]['drive_mode'] & 8:
            raise SafetyError(f'ID {fid}: 정렬에 지원되지 않는 모터 모드입니다.')
        if state[fid].goal_position is None or abs(state[fid].position-state[fid].goal_position)>8:
            raise SafetyError(f'ID {fid}: 현재 유지 목표에 도착하지 않았습니다.')
        if j.get('kind')=='gripper':
            require_gripper(c,f'{pair}/gripper')
            fraction=delta_ticks(state[lid].position,c['leader_closed'])/delta_ticks(c['leader_open'],c['leader_closed'])
            if not 0<=fraction<=1:raise SafetyError('리더 그리퍼가 기록된 개폐 범위 밖입니다.')
            target=round(c['follower_closed']+fraction*(c['follower_open']-c['follower_closed']))
        else:
            if not all(finite(c.get(k)) for k in ('leader_zero','follower_zero','direction','scale')) or c['direction'] not in (-1,1) or not 0<c['scale']<=10:
                raise SafetyError('기준점·방향·비율이 확인되지 않은 관절입니다.')
            target=round(c['follower_zero']+delta_ticks(state[lid].position,c['leader_zero'])*c['direction']*c['scale'])
        lo,hi=rig.position_limits(fid)
        if j.get('kind')=='gripper' or c.get('reviewed'):
            lower,upper=follower_bounds(j);lo,hi=max(lo,lower),min(hi,upper)
        if not lo<=min(target,state[fid].position)<=max(target,state[fid].position)<=hi:
            raise SafetyError(f'ID {fid}: 정렬 경로가 허용된 위치 범위 밖입니다.')
        if abs(target-state[fid].position)>150*TICKS_PER_DEG:
            raise SafetyError(f'ID {fid}: 이번 정렬의 최대 이동량150도를 넘습니다.')
        targets[fid]=target
    return targets


def align_confirmed_pose(rig,config,pair,events,*,pose_confirmed=False):
    if pair!='A' or not pose_confirmed:
        raise SafetyError('사용자가 확인한 A 시작 자세만 정렬할 수 있습니다.')
    first=rig.sample();health(config,first,rig.metadata)
    targets=confirmed_pose_targets(rig,config,pair,first)
    origins={i:first[i].position for i in targets}
    leaders={j['leader']['id']:first[j['leader']['id']].position for _,j in joints(config,pair)}
    distance=max(abs(targets[i]-origins[i]) for i in targets)
    # Wait for each bounded waypoint profile to finish before starting the next.
    duration=distance/TICKS_PER_DEG
    steps=max(1,math.ceil(distance/(3*TICKS_PER_DEG)))
    period=.05;started=previous=time.monotonic();settled=0;count=0
    commanded=origins.copy();waypoint=0;sent_at=None
    events.emit({'schema':'teleop.confirmed_pose_plan.v1','targets':targets,'origins':origins,
                 'estimated_seconds':duration,'max_waypoint_delta_deg':3,'max_profile_speed_deg_s':2,
                 'range_calibration_changed':False})
    try:
        while time.monotonic()-started<3*duration+30:
            begin=time.monotonic();state=rig.sample();now=time.monotonic();dt=now-previous
            if not 0<dt<=config['safety']['max_cycle_s'] or any(now-s.sampled_at>config['safety']['max_cycle_s'] for s in state.values()):
                raise SafetyError('정렬 중 통신 지연 또는 오래된 위치 수신')
            health(config,state,rig.metadata)
            for lid,initial in leaders.items():
                if state[lid].torque or abs(delta_ticks(state[lid].position,initial))>2*TICKS_PER_DEG:
                    raise SafetyError(f'ID {lid}: 정렬 중 리더가 움직였습니다. 현재 위치에서 중지합니다.')
            for i in targets:
                if not state[i].torque:raise SafetyError(f'ID {i}: 정렬 중 토크가 꺼졌습니다.')
                if not min(origins[i],targets[i])-TICKS_PER_DEG<=state[i].position<=max(origins[i],targets[i])+TICKS_PER_DEG:
                    raise SafetyError(f'ID {i}: 확인한 정렬 경로를 벗어났습니다.')
                lo,hi=rig.position_limits(i)
                if not lo<=state[i].position<=hi:raise SafetyError(f'ID {i}: 모터 이동 범위 초과')
                if state[i].position_trajectory is None or state[i].raw_position is None:
                    raise SafetyError(f'ID {i}: 모터 내부 이동 경로 응답 누락')
                if abs(state[i].raw_position-state[i].position_trajectory)>3*TICKS_PER_DEG:
                    raise SafetyError(f'ID {i}: 모터 내부 이동 경로에 대한 오차가3도를 넘었습니다.')
            if sent_at is None:
                waypoint+=1;fraction=waypoint/steps
                commanded={i:round(origins[i]+fraction*(targets[i]-origins[i])) for i in targets}
                rig.goals(commanded,current_positions={i:s.position for i,s in state.items()},
                          speed_deg_s=2,period_s=period)
                sent_at=now;settled=0
            fraction=waypoint/steps
            reached=all(abs(state[i].position-commanded[i])<=2*TICKS_PER_DEG and
                        abs(state[i].position_trajectory-rig.raw_goal(i,commanded[i]))<=1 for i in targets)
            settled=settled+1 if reached else 0
            events.emit({'schema':'teleop.confirmed_pose_alignment.v1','unix_time':time.time(),'sequence':count,
                         'progress':fraction,'waypoint':waypoint,'waypoints':steps,'targets':targets,'commands':commanded,
                         'motors':{i:asdict(s) for i,s in state.items()}})
            if settled>=3:
                if waypoint==steps:
                    result={'schema':'teleop.confirmed_pose_aligned.v1','targets':targets,
                            'positions':{i:state[i].position for i in targets},'seconds':now-started}
                    events.emit(result)
                    return result
                sent_at=None
            elif now-sent_at>15:
                raise SafetyError('정렬 중간 목표의 도착 확인 시간 초과')
            previous=now;count+=1
            time.sleep(max(0,period-(time.monotonic()-begin)))
        raise SafetyError('정렬 도착 확인 시간 초과')
    finally:
        errors=hold(rig,set(targets))
        if errors:
            events.emit({'schema':'teleop.alignment_hold_errors.v1','errors':errors})
