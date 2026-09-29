import copy

import pytest

from conftest import FakeRig,MemoryEvents
from teleop.alignment import confirmed_pose_targets,align_confirmed_pose
from teleop.config import SafetyError,TICKS_PER_DEG


def held(configured,states):
    for i in range(11,16):
        states[i].torque=1;states[i].goal_position=states[i].position
    return FakeRig(configured,states)


def test_one_pose_does_not_certify_unknown_ranges(configured,states):
    rig=held(configured,states)
    for j in configured['pairs']['A'][:4]:
        j['calibration'].update(reviewed=False,leader_min_deg=None,leader_max_deg=None,
                                follower_min_deg=None,follower_max_deg=None)
    rig.states[17].position+=100
    before=copy.deepcopy(configured)
    target=confirmed_pose_targets(rig,configured,'A',rig.sample())
    assert target[12]==2148 and configured==before and not rig.writes


@pytest.mark.parametrize('fault',['signature','torque','range','mapping','gripper'])
def test_plan_rejects_invalid_pose_before_writes(configured,states,fault):
    rig=held(configured,states)
    if fault=='signature':rig.metadata[12]['homing_offset']=100
    if fault=='torque':rig.states[17].torque=1
    if fault=='range':rig.metadata[12]['position_max']=2050;rig.states[17].position+=100
    if fault=='mapping':configured['pairs']['A'][1]['calibration']['direction']=None
    if fault=='gripper':configured['pairs']['A'][4]['calibration']['leader_open']=None
    with pytest.raises(SafetyError):confirmed_pose_targets(rig,configured,'A',rig.sample())
    assert not rig.writes


def simulation(configured,states,monkeypatch,leader_moves=False):
    rig=held(configured,states);rig.states[17].position+=50;rig.states[18].position-=30
    clock=[0.]
    def monotonic():clock[0]+=.001;return clock[0]
    def sleep(t):clock[0]+=max(0,t)
    monkeypatch.setattr('time.monotonic',monotonic);monkeypatch.setattr('time.sleep',sleep)
    original_sample=rig.sample;original_goals=rig.goals;commands=[]
    def sample():
        result=original_sample()
        for i,s in result.items():
            s.raw_position=s.position;s.position_trajectory=s.goal_position
        return result
    def goals(targets,**kwargs):
        assert set(targets)==set(range(11,16)) and kwargs['speed_deg_s']==2
        commands.append((clock[0],targets.copy()))
        original_goals(targets,**kwargs)
        for i,g in targets.items():rig.states[i].goal_position=g
        if leader_moves:rig.states[17].position+=30
    rig.sample=sample;rig.goals=goals;rig.raw_goal=lambda i,target:target
    return rig,commands


def test_slow_coordinated_alignment_reaches_frozen_target(configured,states,monkeypatch):
    rig,commands=simulation(configured,states,monkeypatch);events=MemoryEvents()
    result=align_confirmed_pose(rig,configured,'A',events,pose_confirmed=True)
    assert result['targets'][12]==2098 and result['targets'][13]==2018
    assert result['positions']==result['targets']
    for (t0,a),(t1,b) in zip(commands,commands[1:]):
        assert all(abs(b[i]-a[i])<=3*TICKS_PER_DEG+1 for i in b)
    assert len(commands)==2
    assert not any(w[0]!='goals' and w[1]==64 for w in rig.writes)
    assert result['schema']=='teleop.confirmed_pose_aligned.v1'


def test_leader_movement_stops_new_commands_and_holds(configured,states,monkeypatch):
    rig,commands=simulation(configured,states,monkeypatch,leader_moves=True)
    with pytest.raises(SafetyError,match='리더가 움직'):
        align_confirmed_pose(rig,configured,'A',MemoryEvents(),pose_confirmed=True)
    assert len(commands)==1
    assert all(any(w[0]==i and w[1]==116 for w in rig.writes if w[0]!='goals') for i in range(11,16))


def test_explicit_pose_confirmation_required(configured,states):
    rig=held(configured,states)
    with pytest.raises(SafetyError):align_confirmed_pose(rig,configured,'A',MemoryEvents())
    assert not rig.writes
