import pytest

from conftest import FakeRig, MemoryEvents
from teleop.runtime import hold, hold_session


def loaded_rig(configured, states):
    rig = FakeRig(configured, states)
    rig.raw_goal = lambda i, goal: goal
    for i in range(11,16):
        s = rig.states[i]
        s.torque = 1
        s.watchdog = 15
        s.goal_position = 2048
        s.position = 2055
        s.raw_position = 2055
        s.position_trajectory = 2048
        s.present_velocity = 0
    original_write = rig.write
    def write(i, address, value, size):
        original_write(i, address, value, size)
        if address == 116:
            # Static load adds an offset each time the reference is recaptured.
            s = rig.states[i]
            s.goal_position = s.position_trajectory = value
            s.position = s.raw_position = value+7
    rig.write = write
    return rig


def test_repeated_normal_hold_exit_does_not_accumulate_sag(configured, states):
    rig = loaded_rig(configured, states)
    for _ in range(3):
        hold_session(rig, configured, 'A', .001, MemoryEvents(), resume=True)
    assert not any(w[1] == 116 for w in rig.writes)
    assert all(rig.states[i].goal_position == 2048 and rig.states[i].position == 2055 for i in range(11,16))


@pytest.mark.parametrize('field,value', [('present_velocity',1), ('position_trajectory',2047),
                                        ('position_trajectory',None), ('position',2110)])
def test_unfinished_or_unverified_hold_still_stops_at_current_position(configured, states, field, value):
    rig = loaded_rig(configured, states)
    setattr(rig.states[12], field, value)
    position = rig.states[12].position
    assert hold(rig, {12}, preserve_stationary=True) == []
    assert rig.writes == [(12,116,position)]


def test_fault_cleanup_keeps_existing_current_pose_stop_behavior(configured, states):
    rig = loaded_rig(configured, states)
    hold(rig, {12})
    assert rig.writes == [(12,116,2055)]
    rig.writes.clear()
    rig.states[12].watchdog = 255
    assert hold(rig, {12}, preserve_stationary=True)
    assert rig.writes == []


def test_stationary_extended_goal_compares_raw_trajectory(configured, states):
    rig = loaded_rig(configured, states)
    rig.raw_goal = lambda i, goal: goal+4096
    rig.states[11].position_trajectory = 2048+4096
    assert hold(rig, {11}, preserve_stationary=True) == []
    assert rig.writes == []


def test_accepted_static_load_offset_does_not_recapture_and_sag(configured,states):
    rig=loaded_rig(configured,states);rig.states[12].position=2093
    assert hold(rig,{12},preserve_stationary=True)==[]
    assert not rig.writes
