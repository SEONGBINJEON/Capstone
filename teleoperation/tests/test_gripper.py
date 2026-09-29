import pytest
from teleop.config import SafetyError, require_calibration
from teleop.control import Controller
from teleop.hardware import position_mode_coordinate, profile_values, Rig
from teleop.cli import build_parser
from conftest import FakeRig,MemoryEvents
from teleop.runtime import run_session


def test_gripper_uses_endpoints_and_opposite_direction(configured,states):
    j=configured['pairs']['A'][4]
    j['calibration'].update(leader_closed=1000,leader_open=2000,follower_closed=2500,follower_open=1500)
    states[20].position=1500;states[15].position=2000
    c=Controller(configured,'A',states)
    assert c.target(j,1000)==(2500,False)
    assert c.target(j,2000)==(1500,False)
    assert c.target(j,1500)==(2000,False)
    assert c.target(j,2100)==(1500,True)
    assert c.target(j,900)==(2500,True)


def test_gripper_encoder_wrap(configured,states):
    j=configured['pairs']['A'][4]
    j['calibration'].update(leader_closed=4000,leader_open=200,follower_closed=1800,follower_open=2296)
    states[20].position=4000;states[15].position=1800
    c=Controller(configured,'A',states)
    assert c.target(j,200)==(2296,False)


def test_gripper_not_calibrated(configured):
    configured['pairs']['A'][4]['calibration']['leader_open']=None
    with pytest.raises(SafetyError):require_calibration(configured,'A')


def test_normalized_position_respects_homing_offset():
    meta={'operating_mode':3,'homing_offset':0}
    assert position_mode_coordinate(4132,meta)==36
    assert position_mode_coordinate(-16,meta)==4080
    meta['homing_offset']=100
    assert position_mode_coordinate(4196,meta)==100
    meta['homing_offset']=2048
    with pytest.raises(SafetyError):position_mode_coordinate(4096,meta)


def test_time_profile_uses_milliseconds_and_limits_speed():
    acceleration,duration=profile_values(4,2200,2000,15,.05)
    assert duration>=100 and acceleration<=duration/2
    distance_deg=200*360/4096
    peak_speed=distance_deg/((duration-acceleration)/1000)
    assert peak_speed<=15
    assert profile_values(0,2200,2000,15,.05)==(5,10)
    with pytest.raises(SafetyError):profile_values(8,2200,2000,15,.05)


def test_time_profile_mode_passes_preflight(configured,states):
    j=configured['pairs']['A'][4]
    j['calibration']['signature']['follower']['drive_mode']=4
    rig=FakeRig(configured,states)
    assert run_session(rig,configured,'A',.01,False,MemoryEvents())['frames']>0


def test_active_pair_is_only_motion_target(configured):
    configured['active_pair']='A'
    r=Rig(configured,allow_motion=True)
    assert set(r.specs)==set(range(11,21))
    assert r.write_ids==set(range(11,16))
    assert 21 not in r.write_ids
    assert build_parser().parse_args(['monitor']).pair is None


def test_hold_mode_only_enables_follower_at_existing_pose(configured,states):
    from teleop.runtime import hold_session
    rig=FakeRig(configured,states);events=MemoryEvents()
    result=hold_session(rig,configured,'A',.01,events)
    assert result['mode']=='hold_only'
    assert not [w for w in rig.writes if w[0]=='goals']
    assert {w[0] for w in rig.writes} <= set(range(11,16))
    assert all(w[2]==2048 for w in rig.writes if w[1]==116)
    assert all(rig.states[i].torque==0 for i in range(16,21))


def test_hold_refuses_outside_stored_limits(configured,states):
    from teleop.runtime import hold_session
    rig=FakeRig(configured,states);rig.metadata[13]['position_min']=2500
    with pytest.raises(SafetyError):hold_session(rig,configured,'A',.01,MemoryEvents())
    assert rig.writes==[]


def test_gripper_cannot_start_with_missing_endpoint(configured,states):
    configured['pairs']['A'][4]['calibration']['follower_open']=None
    rig=FakeRig(configured,states)
    with pytest.raises(SafetyError):run_session(rig,configured,'A',.01,True,MemoryEvents())
    assert not rig.writes
