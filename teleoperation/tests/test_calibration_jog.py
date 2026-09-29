import pytest

from conftest import MemoryEvents
from test_resume_hold import held_rig
from teleop.config import SafetyError
from teleop.runtime import hold_session


def test_single_jog_preserves_other_goals_and_torque(configured,states):
    rig=held_rig(configured,states); events=MemoryEvents()
    profiles=[]; original=rig.goals
    def goals(targets,**kwargs):
        profiles.append(kwargs)
        original(targets,**kwargs)
    rig.goals=goals
    hold_session(rig,configured,'A',.15,events,resume=True,recover_watchdog=True,jog_joint=1,jog_degrees=3)
    assert [w[1] for w in rig.writes if w[0]=='goals']==[{11:2082}]
    assert profiles[0]['speed_deg_s']==2
    assert not any(w[0]!='goals' and w[1]==64 for w in rig.writes)
    assert all(rig.states[i].position==2048 for i in range(12,16))
    assert events.items[-2]['phase']=='holding'


@pytest.mark.parametrize('joint,degrees',[(1,4),(1,-4),(1,0),(1,float('nan')),(5,1),(None,1),(1,None)])
def test_bad_jog_cannot_write(configured,states,joint,degrees):
    rig=held_rig(configured,states)
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,jog_joint=joint,jog_degrees=degrees)
    assert not rig.writes


def test_jog_refuses_encoder_boundary_without_wrapping(configured,states):
    rig=held_rig(configured,states)
    configured['pairs']['A'][0]['calibration']['reviewed']=False
    rig.states[11].position=1;rig.states[11].goal_position=1
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,jog_joint=1,jog_degrees=-3)
    assert not rig.writes


def test_jog_refuses_hardware_limit_even_without_geometric_calibration(configured,states):
    rig=held_rig(configured,states)
    configured['pairs']['A'][2]['calibration']['reviewed']=False
    rig.metadata[13]['position_max']=2060
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,jog_joint=3,jog_degrees=3)
    assert not rig.writes


def test_jog_detects_uncommanded_joint_movement(configured,states):
    rig=held_rig(configured,states); original=rig.goals
    def goals(targets,**kwargs):
        original(targets,**kwargs)
        rig.states[12].position+=12
    rig.goals=goals
    with pytest.raises(SafetyError,match='ID 12'):
        hold_session(rig,configured,'A',.1,MemoryEvents(),resume=True,recover_watchdog=True,jog_joint=1,jog_degrees=3)


def test_jog_arrival_matches_existing_hold_tolerance(configured,states):
    rig=held_rig(configured,states);original=rig.goals;events=MemoryEvents()
    def goals(targets,**kwargs):
        original(targets,**kwargs)
        rig.states[11].position-=6
    rig.goals=goals
    hold_session(rig,configured,'A',.15,events,resume=True,recover_watchdog=True,jog_joint=1,jog_degrees=3)
    assert events.items[-2]['phase']=='holding'
