import pytest

from conftest import FakeRig, MemoryEvents
from teleop.config import SafetyError
from teleop.runtime import hold_session


def held_rig(configured, states):
    rig = FakeRig(configured, states)
    for i in range(11,16):
        rig.states[i].torque = 1
        rig.states[i].watchdog = 255
        rig.states[i].goal_position = rig.states[i].position
    return rig


def test_resume_keeps_torque_and_existing_goals(configured, states):
    rig = held_rig(configured,states)
    events = MemoryEvents()
    hold_session(rig,configured,'A',.01,events,resume=True,recover_watchdog=True)
    assert not [w for w in rig.writes if w[0]=='goals' or w[1]==64]
    assert all(w[0] in range(11,16) for w in rig.writes)
    assert all(rig.states[i].watchdog==15 for i in range(11,16))
    assert all(w[2]==2048 for w in rig.writes if w[1]==116)  # Cleanup hold only.
    assert set(events.items[0]['motors']) == set(states)


@pytest.mark.parametrize('field,value', [('goal_position',2200),('goal_position',None),
                                         ('torque',0),('hardware_error',4),('temperature',80),('voltage',20)])
def test_resume_invalid_state_never_writes(configured, states, field, value):
    rig = held_rig(configured,states)
    setattr(rig.states[13],field,value)
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True)
    assert not rig.writes


def test_watchdog_recovery_is_explicit_and_scoped(configured, states):
    rig = held_rig(configured,states)
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True)
    assert not rig.writes
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),recover_watchdog=True)
    assert not rig.writes
    rig.states[16].watchdog=255
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True)
    assert not rig.writes
