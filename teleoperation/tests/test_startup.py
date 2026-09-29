import copy

import pytest

from teleop.config import SafetyError, TICKS_PER_DEG
from teleop.control import Controller
from teleop.runtime import run_session
from conftest import FakeRig, MemoryEvents


def test_align_starts_at_follower_and_converges_to_absolute_pose(configured, states):
    original = copy.deepcopy(configured)
    states[16].position += 200
    states[20].position = 1800  # Also align an initially open gripper slowly.
    states[15].position = 2296
    c = Controller(configured, 'A', states, startup_mode='align')
    assert c.commanded[11] == 2048
    assert c.commanded[15] == 2296
    for _ in range(200):
        previous = c.commanded.copy()
        goals, _ = c.step(states, .1)
        assert all(abs(c.commanded[f]-previous[f]) <= 5*TICKS_PER_DEG*.1 + 1e-8 for f in goals)
        for f, goal in goals.items():
            states[f].position = goal
        if c.phase == 'following':
            break
    assert c.phase == 'following'
    assert goals[11] == 2248 and goals[15] == 1800
    assert configured == original  # Never replace the upright calibration with a resting pose.
    states[16].position += 10
    assert c.step(states, .1)[0][11] == 2258


def test_align_requires_measured_settling(configured, states):
    states[16].position += 100
    c = Controller(configured, 'A', states, startup_mode='align')
    for _ in range(30):
        c.step(states, .1)
    assert c.commanded[11] == 2148
    assert c.phase == 'aligning'  # Command completion alone is not measured arrival.
    states[11].position = 2148
    for _ in range(3):
        c.step(states, .1)
    assert c.phase == 'following'


def test_moving_leader_during_align_stops_without_updating_goals(configured, states):
    c = Controller(configured, 'A', states, startup_mode='align')
    before = c.commanded.copy()
    states[16].position += 24
    with pytest.raises(SafetyError, match='정렬 중 리더'):
        c.step(states, .1)
    assert c.commanded == before


def test_alignment_wrap_uses_short_path_for_leader(configured, states):
    configured['pairs']['A'][0]['calibration']['leader_zero'] = 4090
    states[16].position = 4090
    c = Controller(configured, 'A', states, startup_mode='align')
    states[16].position = 2
    c.step(states, .1)
    assert c.phase == 'aligning'


def test_unreachable_initial_pose_is_rejected_not_clamped(configured, states):
    states[16].position += round(80*TICKS_PER_DEG)
    with pytest.raises(SafetyError, match='목표가 팔로워 이동 범위 밖'):
        Controller(configured, 'A', states, startup_mode='align')


def test_alignment_timeout_and_input_range_remain_enforced(configured, states):
    c = Controller(configured, 'A', states, startup_mode='align')
    c.alignment_elapsed = 60
    with pytest.raises(SafetyError, match='시간 초과'):
        c.step(states, .1)
    states[16].position += round(100*TICKS_PER_DEG)
    with pytest.raises(SafetyError, match='리더 이동 범위'):
        Controller(configured, 'A', states, startup_mode='align')


def test_alignment_dry_run_has_no_writes(configured, states):
    states[16].position += 200
    rig = FakeRig(configured, states)
    events = MemoryEvents()
    run_session(rig, configured, 'A', .02, False, events, startup_mode='align')
    assert not rig.writes
    assert events.items[0]['phase'] == 'aligning'


def test_alignment_hardware_profiles_are_slow_and_only_write_a(configured, states):
    states[16].position += 200
    rig = FakeRig(configured, states)
    original = rig.goals
    speeds = []
    def goals(targets, **kwargs):
        speeds.append(kwargs['speed_deg_s'])
        original(targets, **kwargs)
    rig.goals = goals
    run_session(rig, configured, 'A', .02, True, MemoryEvents(), startup_mode='align')
    assert speeds and all(s == 5 for s in speeds)
    assert all(w[0] in range(11, 16) for w in rig.writes if w[0] != 'goals')


def test_alignment_does_not_bypass_calibration_or_existing_hold(configured, states):
    rig = FakeRig(configured, states)
    configured['pairs']['A'][0]['calibration']['reviewed'] = False
    with pytest.raises(SafetyError):
        run_session(rig, configured, 'A', .02, True, MemoryEvents(), startup_mode='align')
    assert not rig.writes
    configured['pairs']['A'][0]['calibration']['reviewed'] = True
    rig.states[11].torque = 1
    with pytest.raises(SafetyError, match='토크가 꺼져'):
        run_session(rig, configured, 'A', .02, True, MemoryEvents(), startup_mode='align')
    assert not rig.writes
