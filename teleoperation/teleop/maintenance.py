"""Explicit supported-pose maintenance; no homing movement or automatic mode change."""
import copy
import json
import time
from dataclasses import asdict
from pathlib import Path

from .config import SafetyError, save, follower_bounds
from .control import health, delta_ticks
from .hardware import profile_values


def homing_shift_plan(rig, config, pair, joint_index, offset, states):
    if pair not in ('A','B') or joint_index not in (1,2,3,4):
        raise SafetyError('원점 변경은 선택한 A/B의 팔 관절 하나만 지원합니다.')
    if not isinstance(offset,int) or isinstance(offset,bool) or not -1024 <= offset <= 1024:
        raise SafetyError('위치 모드 원점은 -1024~1024 범위의 정수여야 합니다.')
    j = config['pairs'][pair][joint_index-1]; fid = j['follower']['id']
    meta = rig.metadata[fid]; c = j['calibration']; s = states[fid]
    if c.get('signature',{}).get('follower') != rig.signature(fid) or meta['operating_mode'] != 3:
        raise SafetyError('저장된 보정과 실제 팔로워 설정이 다릅니다.')
    old = meta['homing_offset']; shift = offset-old
    if not shift:
        raise SafetyError('이미 같은 원점 설정입니다.')
    if not s.torque or s.goal_position is None or abs(s.position-s.goal_position)>8:
        raise SafetyError('기존 목표에 도착한 토크 ON 상태에서 원점 변경을 준비하세요.')
    zero = c.get('follower_zero')
    if not isinstance(zero,(int,float)) or not 0 <= zero+shift <= 4095:
        raise SafetyError('변경 후 보정 기준점이 단회전 범위를 벗어납니다.')
    lo,hi = max(0,meta['position_min']),min(4095,meta['position_max'])
    if not lo <= s.position+shift <= hi:
        raise SafetyError('변경 후 현재 자세의 목표가 저장된 위치 제한 밖입니다.')
    if c.get('reviewed'):
        lower,upper=follower_bounds(j)
        if not lo <= lower+shift < upper+shift <= hi:
            raise SafetyError('변경 후 기존 보정 범위를 보존할 수 없습니다.')
    return dict(id=fid,old_offset=old,new_offset=offset,shift=shift,
                old_zero=zero,new_zero=zero+shift,expected_position=s.position+shift)


def shift_home_at_rest(rig, config, pair, joint_index, offset, config_path, backup_dir, events):
    """Caller keeps the arm supported. Any failure after disabling leaves that joint OFF."""
    first = rig.sample(); health(config,first,rig.metadata)
    plan = homing_shift_plan(rig,config,pair,joint_index,offset,first)
    fid = plan['id']
    followers = {j['follower']['id'] for j in config['pairs'][pair]}
    if any(not first[i].torque or first[i].goal_position is None or abs(first[i].position-first[i].goal_position)>8 for i in followers):
        raise SafetyError('팔로워 전체가 현재 목표를 유지하고 있어야 합니다.')
    if json.loads(Path(config_path).read_text()) != config:
        raise SafetyError('설정 파일이 실행 중 변경되었습니다. 원점 변경을 중지합니다.')
    directory = Path(backup_dir); directory.mkdir(parents=True,exist_ok=True)
    backup = directory/f'home_shift_{fid}_{time.time_ns()}.json'
    backup.write_text(json.dumps(dict(plan=plan,config=config,
                                    motors={i:dict(state=asdict(s),metadata=rig.metadata[i]) for i,s in first.items()}),indent=2)+'\n')
    events.emit({'schema':'teleop.maintenance.v1','phase':'backed_up','plan':plan,'backup':str(backup)})
    # Ensure every sampled pose is still where it was before releasing just one joint.
    fresh = rig.sample(); health(config,fresh,rig.metadata)
    if any(not fresh[i].torque or abs(fresh[i].position-first[i].position)>8 for i in followers):
        raise SafetyError('원점 변경 준비 중 자세가 움직였습니다.')
    try:
        rig.write(fid,98,0,1)
        rig.write(fid,64,0,1)
        released = rig.sample()
        if released[fid].torque or abs(delta_ticks(released[fid].position,first[fid].position))>8:
            raise SafetyError('토크 해제 중 베이스가 움직였습니다.')
        rig.set_homing_offset(fid,offset)
        updated = rig.sample(); health(config,updated,rig.metadata)
        if updated[fid].torque or abs(updated[fid].position-plan['expected_position'])>8:
            raise SafetyError('원점 변경 후 위치 좌표가 예상과 다릅니다.')
        if any(not updated[i].torque or abs(updated[i].position-first[i].position)>8 for i in followers-{fid}):
            raise SafetyError('다른 관절의 자세 유지 상태가 달라졌습니다.')
        goal = updated[fid].position
        meta = rig.metadata[fid]
        if not max(0,meta['position_min']) <= goal <= min(4095,meta['position_max']):
            raise SafetyError('변경 후 목표가 단회전 제한 밖입니다.')
        revised = copy.deepcopy(config)
        c = revised['pairs'][pair][joint_index-1]['calibration']
        c.update(follower_zero=plan['new_zero'],reviewed=False)
        c['signature']['follower'] = rig.signature(fid)
        c['homing_shift'] = dict(old_offset=plan['old_offset'],new_offset=offset,backup=str(backup))
        save(config_path,revised)
        config.clear();config.update(revised)
        acceleration,velocity=profile_values(meta['drive_mode'],goal,goal,2,1/config['safety']['rate_hz'])
        rig.write(fid,108,acceleration,4);rig.write(fid,112,velocity,4)
        rig.write(fid,116,goal,4);rig.write(fid,98,15,1)
        staged=rig.sample();health(config,staged,rig.metadata)
        if abs(staged[fid].position-goal)>8:
            raise SafetyError('토크 복귀 준비 중 베이스가 움직였습니다.')
        rig.write(fid,64,1,1)
        final=rig.sample();health(config,final,rig.metadata)
        if not final[fid].torque or abs(final[fid].position-goal)>8:
            raise SafetyError('원점 변경 후 토크 복귀 검증 실패')
        events.emit({'schema':'teleop.maintenance.v1','phase':'complete','plan':plan,'position':final[fid].position})
        return fid,goal
    except BaseException:
        # Never energize against an uncertain coordinate system, including after an uncertain enable write.
        try:
            rig.write(fid,64,0,1)
        except Exception:
            pass
        events.emit({'schema':'teleop.maintenance.v1','phase':'failed','id':fid,
                     'support_required':True,'message':'변경 대상 토크 OFF를 시도했습니다. 팔을 계속 받치고 설정 확인이 필요합니다.','backup':str(backup)})
        raise


def enable_extended_base_at_rest(rig,config,config_path,backup_dir,events):
    """Only A/11, with a fixed ±45° operating window and unchanged physical pose."""
    j=config['pairs']['A'][0];fid=j['follower']['id'];c=j['calibration']
    if fid!=11 or rig.metadata[fid]['model']!=1060 or rig.metadata[fid]['operating_mode']!=3:
        raise SafetyError('이 전환은 위치 모드인 A 베이스 XL430/11번만 지원합니다.')
    if c['signature']['follower']!=rig.signature(fid):
        raise SafetyError('베이스 보정과 실제 모터 설정이 다릅니다.')
    ref=c['follower_zero']
    if not isinstance(ref,int) or not 512 <= ref <= 3583:
        raise SafetyError('좌우45도 범위를 만들 수 없는 기준점입니다.')
    window=dict(reference=ref,min=ref-512,max=ref+512)
    first=rig.sample();health(config,first,rig.metadata)
    followers={entry['follower']['id']for entry in config['pairs']['A']}
    if any(not first[i].torque or first[i].goal_position is None or abs(first[i].position-first[i].goal_position)>8 for i in followers):
        raise SafetyError('모든 팔로워가 기존 목표를 유지하는 상태여야 합니다.')
    if not window['min'] <= first[fid].position <= window['max']:
        raise SafetyError('현재 베이스 위치가 좌우45도 범위 밖입니다.')
    if json.loads(Path(config_path).read_text())!=config:
        raise SafetyError('설정 파일이 실행 중 변경되었습니다.')
    directory=Path(backup_dir);directory.mkdir(parents=True,exist_ok=True)
    backup=directory/f'extended_base_{time.time_ns()}.json'
    backup.write_text(json.dumps(dict(config=config,window=window,
                                    motors={i:dict(state=asdict(s),metadata=rig.metadata[i])for i,s in first.items()}),indent=2)+'\n')
    events.emit({'schema':'teleop.maintenance.v1','phase':'extended_backed_up','backup':str(backup),'window':window})
    try:
        fresh=rig.sample();health(config,fresh,rig.metadata)
        if any(not fresh[i].torque or abs(fresh[i].position-first[i].position)>8 for i in followers):
            raise SafetyError('베이스 모드 변경 준비 중 자세가 움직였습니다.')
        rig.write(fid,98,0,1);rig.write(fid,64,0,1)
        off=rig.sample()
        if off[fid].torque or abs(delta_ticks(off[fid].position,first[fid].position))>8:
            raise SafetyError('베이스 토크 해제 중 위치가 움직였습니다.')
        rig.specs[fid]['extended_position']=window
        rig.set_extended_mode(fid)
        updated=rig.sample();health(config,updated,rig.metadata)
        if updated[fid].torque or abs(updated[fid].position-first[fid].position)>8:
            raise SafetyError('모드 변경 후 베이스 위치 기준이 달라졌습니다.')
        if any(not updated[i].torque or abs(updated[i].position-first[i].position)>8 for i in followers-{fid}):
            raise SafetyError('다른 팔로워의 유지 상태가 달라졌습니다.')
        revised=copy.deepcopy(config);entry=revised['pairs']['A'][0]
        entry['follower']['extended_position']=window
        entry['calibration']['signature']['follower']=rig.signature(fid)
        entry['calibration'].update(reviewed=False,extended_mode_backup=str(backup))
        save(config_path,revised);config.clear();config.update(revised)
        goal=updated[fid].position
        acceleration,velocity=profile_values(rig.metadata[fid]['drive_mode'],goal,goal,2,1/config['safety']['rate_hz'])
        rig.write(fid,108,acceleration,4);rig.write(fid,112,velocity,4)
        rig.write(fid,116,goal,4);rig.write(fid,98,15,1)
        staged=rig.sample();health(config,staged,rig.metadata)
        if abs(staged[fid].position-goal)>8:raise SafetyError('토크 복귀 준비 중 베이스가 움직였습니다.')
        rig.write(fid,64,1,1)
        final=rig.sample();health(config,final,rig.metadata)
        if not final[fid].torque or abs(final[fid].position-goal)>8:
            raise SafetyError('확장 위치 모드의 토크 복귀 검증 실패')
        events.emit({'schema':'teleop.maintenance.v1','phase':'extended_complete','id':fid,
                     'window':window,'position':final[fid].position,'raw_position':final[fid].raw_position})
        return fid,goal
    except BaseException:
        try:rig.write(fid,64,0,1)
        except Exception:pass
        events.emit({'schema':'teleop.maintenance.v1','phase':'extended_failed','id':fid,
                     'backup':str(backup),'support_required':True})
        raise
