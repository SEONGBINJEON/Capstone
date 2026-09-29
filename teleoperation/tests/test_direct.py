import copy
import json
import pytest
from conftest import FakeRig,MemoryEvents
from teleop.config import SafetyError,TICKS_PER_DEG
from teleop.direct import DirectA,run_direct_A


def setup(configured,states):
    for i in range(11,16):
        s=states[i];s.torque=1;s.watchdog=255;s.goal_position=s.position
        s.raw_position=s.position_trajectory=s.position;s.present_velocity=0
    rig=FakeRig(configured,states);rig.raw_goal=lambda i,g:g
    return rig,DirectA(rig,configured,states)


def test_full_mapping_preserves_unreviewed_mechanical_ranges(configured,states):
    for j in configured['pairs']['A'][:4]:
        j['calibration'].update(reviewed=False,leader_min_deg=None,leader_max_deg=None,follower_min_deg=None,follower_max_deg=None)
    before=copy.deepcopy(configured)
    rig,c=setup(configured,states)
    for _ in range(3):c.step(states,.05)
    assert c.phase=='following'
    for i in range(16,20):states[i].position+=20
    states[20].position=2296
    goals=c.step(states,.05)
    assert set(goals)==set(range(11,16)) and all(2048<g<=2151 for g in goals.values())
    assert configured==before and not rig.writes


def test_joint_limit_clamps_and_returns_without_stopping_others(configured,states):
    rig,c=setup(configured,states)
    c.phase='following';c.bounds[13]=(2000,2050)
    states[18].position+=50;states[17].position+=20
    goals=c.step(states,.05)
    assert goals[13]==2050 and goals[12]>2048 and c.limited[13]
    states[13].position=2050;states[18].position=2020
    goals=c.step(states,.05)
    assert goals[13]<2050 and not c.limited[13]


def test_reversal_replaces_unfinished_forward_goal(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    c.commanded[12]=2120
    states[12].position=2080;states[17].position=2048
    goals=c.step(states,.05)
    assert goals[12]<2080  # Reverse immediately, not after reaching 2120.


def test_boundary_feedback_tolerance_and_hold(configured,states):
    from teleop.runtime import hold
    rig,c=setup(configured,states)
    rig.metadata[13]['position_min']=1024
    states[13].position=1022;states[13].goal_position=1032
    states[18].position=900
    c=DirectA(rig,configured,states)
    assert c.step(states,.05)[13]>=1024
    rig.states[13]=copy.deepcopy(states[13]);rig.states[13].watchdog=15
    assert hold(rig,{13})==[]
    assert rig.writes[-1]==(13,116,1024)
    states[13].position=990
    with pytest.raises(SafetyError,match='모터 범위'):c.step(states,.05)


def test_startup_tracks_latest_leader_instead_of_frozen_pose(configured,states):
    rig,c=setup(configured,states)
    states[17].position+=100
    first=c.step(states,.05)
    states[12].position=first[12]
    states[17].position=2030
    assert c.step(states,.05)[12]<first[12]


def test_follow_speed_has_no_hidden_eight_degree_cap(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    states[17].position+=200
    assert c.step(states,.05)[12]-2048==round(configured['direct_A']['follow_speed_deg_s']*TICKS_PER_DEG*.05)


def test_startup_can_begin_from_rest_without_rezero(configured,states):
    rig,c=setup(configured,states)
    states[17].position+=200
    c=DirectA(rig,configured,states)
    target=c.frozen[12]
    for _ in range(160):
        goals=c.step(states,.05)
        for i,g in goals.items():states[i].position=g
        if c.phase=='following':break
    assert c.phase=='following' and c.commanded[12]==target


def test_stalled_motor_stops_without_relaxing_guard(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    states[17].position+=100
    with pytest.raises(SafetyError,match='움직이지'):
        for _ in range(100):c.step(states,.05)


def test_movement_after_long_idle_does_not_inherit_stall_time(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    for _ in range(200):c.step(states,.05)
    states[18].position+=200
    # A servo needs time to begin moving after a fresh command. The preceding
    # ten seconds at rest must never count as time spent failing that command.
    for _ in range(6):goals=c.step(states,.05)
    for _ in range(60):
        for i,g in goals.items():states[i].position=g
        goals=c.step(states,.05)
    assert goals[13]==2248 and c.phase=='following'


def test_sustained_stall_after_idle_still_stops(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    for _ in range(200):c.step(states,.05)
    states[18].position+=200
    began=c.elapsed
    with pytest.raises(SafetyError,match='움직이지'):
        for _ in range(100):c.step(states,.05)
    assert c.elapsed-began>=1.4


def test_all_a_session_writes_only_followers_and_pauses(configured,states,monkeypatch,tmp_path):
    setup(configured,states)
    clock=[0.]
    def now():clock[0]+=.001;return clock[0]
    monkeypatch.setattr('time.monotonic',now)
    monkeypatch.setattr('time.sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    path=tmp_path/'control.json';command={'nonce':1};path.write_text(json.dumps(command))
    class Rig(FakeRig):
        count=0
        def raw_goal(self,i,g):return g
        def stream_goals(self,goals,s,**kwargs):
            assert set(goals)<=set(range(11,16))
            self.writes.append(('stream',goals.copy()))
            for i,g in goals.items():
                v=self.states[i];v.goal_position=v.position=v.raw_position=v.position_trajectory=g
        def sample(self):
            self.count+=1
            if self.count==12:
                for i in range(16,21):self.states[i].position+=30
            if self.count==30:path.write_text('{"nonce":2}')
            return super().sample()
    rig=Rig(configured,states);events=MemoryEvents()
    run_direct_A(rig,configured,events,motor_limits=True,command=command,control_path=path)
    assert any(w[0]=='stream' and all(g>2048 for g in w[1].values()) for w in rig.writes)
    assert all(w[0]=='stream' or w[0] in range(11,16) for w in rig.writes)
    assert not any(w[0]!='stream' and w[1]==64 for w in rig.writes)


def test_stream_profile_uses_trajectory_not_load_deflected_position(configured,states,monkeypatch):
    from teleop.hardware import Rig
    from teleop.hardware import COMM_SUCCESS
    captures=[]
    class Writer:
        def __init__(self,*_):pass
        def addParam(self,i,data):captures.append((i,data));return True
        def txPacket(self):return COMM_SUCCESS
    monkeypatch.setattr('teleop.hardware.GroupSyncWrite',Writer)
    rig=Rig(configured,'A',allow_motion=True)
    rig.metadata[13]=dict(model=1060,drive_mode=4,operating_mode=3,homing_offset=0,position_min=0,position_max=4095)
    rig.ports['followers']=object()
    states[13].position=2000;states[13].position_trajectory=2099
    rig.stream_goals({13:2100},states,profile_speed_deg_s=15,period_s=.05)
    data=bytes(captures[0][1])
    assert int.from_bytes(data[4:8],'little')==100
    assert int.from_bytes(data[8:12],'little')==2100


def test_dual_pair_mixed_initial_torque_and_independent_mapping(configured,states,monkeypatch,tmp_path):
    from teleop.direct import run_direct
    setup(configured,states)
    for i in range(11,16):states[i].watchdog=15
    for i in range(21,26):
        states[i].goal_position=states[i].position_trajectory=states[i].raw_position=states[i].position
        states[i].present_velocity=0
    states[27].position+=200
    clock=[0.]
    def now():clock[0]+=.001;return clock[0]
    monkeypatch.setattr('time.monotonic',now)
    monkeypatch.setattr('time.sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    monkeypatch.setattr('teleop.direct.write_status',lambda payload:None)
    path=tmp_path/'control.json';command={'nonce':1};path.write_text(json.dumps(command))
    class Rig(FakeRig):
        count=0
        def raw_goal(self,i,g):return g
        def stream_goals(self,goals,s,**kwargs):
            assert set(goals)<=set(range(11,16))|set(range(21,26))
            self.writes.append(('stream',goals.copy()))
            for i,g in goals.items():
                v=self.states[i];v.goal_position=v.position=v.raw_position=v.position_trajectory=g
        def sample(self):
            self.count+=1
            if self.count==55:self.states[16].position+=100;self.states[26].position-=100
            if self.count==100:path.write_text('{"nonce":2}')
            return super().sample()
    rig=Rig(configured,states);events=MemoryEvents()
    run_direct(rig,configured,events,pair='both',motor_limits=True,command=command,control_path=path)
    assert any(w[0]=='stream' and w[1].get(11,2048)>2048 and w[1].get(21,2048)<2048 for w in rig.writes)
    assert {(w[0],w[2]) for w in rig.writes if w[0]!='stream' and w[1]==64}=={(i,1) for i in range(21,26)}
    phases=[r for r in events.items if r.get('phase')=='following']
    assert phases and all(abs(r['targets'][22]-2248)<1 for r in phases)


def test_arrival_tolerance_is_not_also_a_stall(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    states[17].position+=45
    for _ in range(200):c.step(states,.05)
    assert c.commanded[12]==2093 and c.phase=='following'


def test_disabled_b_gripper_not_required_or_commanded(configured,states):
    configured['pairs']['B'][4]['enabled']=False
    configured['pairs']['B'][4]['calibration']={}
    states.pop(25);states.pop(30)
    rig=FakeRig(configured,states)
    c=DirectA(rig,configured,states,'B')
    assert set(c.step(states,.05))=={21,22,23,24}


def test_future_goal_gap_is_not_internal_servo_tracking_error(configured,states):
    rig,c=setup(configured,states);c.phase='following'
    c.commanded[12]=2250
    states[12].raw_position=states[12].position_trajectory=2048
    states[17].position=2300
    c.step(states,.05)
    states[12].position_trajectory=2300
    with pytest.raises(SafetyError,match='추종 오차'):c.step(states,.05)


@pytest.mark.parametrize('drive,expected_accel,expected_velocity',[(0,50,131),(4,12,50)])
def test_fast_profile_removes_slow_acceleration_and_double_frame_delay(configured,states,monkeypatch,drive,expected_accel,expected_velocity):
    from teleop.hardware import Rig
    captures=[]
    class Writer:
        def __init__(self,*_):pass
        def addParam(self,i,data):captures.append(bytes(data));return True
        def txPacket(self):return 0
    monkeypatch.setattr('teleop.hardware.GroupSyncWrite',Writer)
    rig=Rig(configured,'A',allow_motion=True)
    rig.metadata[13]=dict(model=1060,drive_mode=drive,operating_mode=3,homing_offset=0,position_min=0,position_max=4095,velocity_limit=265)
    rig.ports['followers']=object();states[13].position_trajectory=2099
    rig.stream_goals({13:2100},states,profile_speed_deg_s=180,period_s=.05)
    assert int.from_bytes(captures[0][:4],'little')==expected_accel
    assert int.from_bytes(captures[0][4:8],'little')==expected_velocity
