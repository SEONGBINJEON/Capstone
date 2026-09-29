import copy
import pytest
from conftest import FakeRig, MemoryEvents
from teleop.config import SafetyError
from teleop.shoulder_check import check_shoulder_tilt


def setup_check(configured, states, monkeypatch):
    clock=[100.]
    monkeypatch.setattr('time.monotonic',lambda:clock[0])
    monkeypatch.setattr('time.sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    configured['pairs']['A'][1]['calibration']['signature']['follower'].update(model=1030,drive_mode=0,operating_mode=3,homing_offset=0)
    candidate=copy.deepcopy(configured)
    candidate['pairs']['A'][1]['calibration'].update(leader_zero=2028,follower_zero=2335,
        inclination_measurement={'leader_position':3063,'follower_position':3472})
    class ShoulderRig(FakeRig):
        def restore_a_shoulder_output(self,value):
            assert value==885 and self.states[12].goal_pwm==250
            self.writes.append((12,100,value));self.states[12].goal_pwm=value
        def tune_a_shoulder(self,address,value):
            assert (address,value) in ((100,250),(84,1200))
            if address==84:assert self.states[12].goal_pwm==250
            self.writes.append((12,address,value))
            setattr(self.states[12],{100:'goal_pwm',84:'position_p_gain'}[address],value)
        def raw_goal(self,i,g):return g
        def write(self,i,address,value,size):
            assert i==12 and address in (98,108,112,116)
            super().write(i,address,value,size)
            if address==116:self.states[i].goal_position=value
            if address==108:self.states[i].profile_acceleration=value
            if address==112:self.states[i].profile_velocity=value
        def read_register(self,i,address,size):
            return getattr(self.states[i],{108:'profile_acceleration',112:'profile_velocity'}[address])
        def sample(self):
            s=self.states[12]
            if s.watchdog==15:
                s.position_trajectory=max(s.goal_position,s.position_trajectory-2)
                s.position=s.raw_position=s.position_trajectory+13
            return super().sample()
    rig=ShoulderRig(configured,states)
    rig.states[17].position=3063
    for i in range(11,16):
        s=rig.states[i]
        s.torque=1;s.watchdog=255;s.goal_position=s.position
        s.raw_position=s.position;s.position_trajectory=s.position
        s.present_velocity=0;s.present_pwm=0
    s=rig.states[12]
    s.position=s.raw_position=3472
    s.goal_position=s.position_trajectory=3459
    s.present_pwm=-82
    s.profile_acceleration=5;s.profile_velocity=1
    s.position_p_gain=800;s.position_i_gain=0;s.position_d_gain=0;s.goal_pwm=885
    return rig,candidate


def test_bounded_recovery_and_shoulder_lift_only_write12(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    events=MemoryEvents()
    result=check_shoulder_tilt(rig,configured,candidate,events,hands_clear=True,bases_level=True)
    assert result['target']==3370 and result['actual']==3383
    assert rig.writes==[(12,98,0),(12,98,15),(12,116,3370)]
    assert all(rig.states[i].watchdog==255 for i in (11,13,14,15))
    assert result['requires_physical_angle_verification']


def test_tuning_caps_hardware_output_before_increasing_p(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    result=check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True,tune_position=True)
    assert result['target']==3370
    assert rig.writes==[(12,98,0),(12,98,15),(12,100,250),(12,84,1200),(12,116,3370)]


def test_restore_original_cap_keeps_low_speed_and_position_guards(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    rig.states[12].position_p_gain=1200;rig.states[12].goal_pwm=250
    result=check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True,restore_output=True,relative_test=True)
    assert result['target']==3370
    assert rig.writes==[(12,98,0),(12,98,15),(12,100,885),(12,116,3370)]
    assert rig.states[12].profile_velocity==1


def test_relative_trial_caps_total_travel_without_claiming_final_alignment(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    s=rig.states[12];s.position=s.raw_position=3486;s.goal_position=s.position_trajectory=3473
    result=check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True,tune_position=True,relative_test=True)
    assert result['candidate_target']==3370 and result['target']==3384
    assert result['relative_test'] and result['requires_physical_angle_verification']


def test_unexpected_gain_blocks_tuning_before_any_write(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    rig.states[12].position_i_gain=10
    with pytest.raises(SafetyError,match='게인'):
        check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True,tune_position=True)
    assert not rig.writes


def test_sustained_output_saturation_stops_without_raising_limit(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    sample=rig.sample
    def saturated():
        if rig.states[12].goal_position==3370:rig.states[12].present_pwm=-250
        return sample()
    rig.sample=saturated
    with pytest.raises(SafetyError,match='제한 출력'):
        check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True,tune_position=True)
    assert [(w[1],w[2]) for w in rig.writes if w[1]==100]==[(100,250)]
    assert rig.writes[-1][1]==116 and rig.writes[-1][2]>3370


def test_hardware_tuning_requires_verified_cap_and_scope(configured):
    from teleop.hardware import Rig
    rig=Rig(configured,'A',allow_motion=True)
    rig.metadata[12]=dict(model=1030,drive_mode=0,operating_mode=3,homing_offset=0)
    rig.read_register=lambda i,a,n:{64:1,98:15,100:885}[a]
    with pytest.raises(SafetyError,match='출력 제한'):
        rig.tune_a_shoulder(84,1200)
    with pytest.raises(SafetyError):rig.tune_a_shoulder(84,1600)
    with pytest.raises(SafetyError):rig.tune_a_shoulder(100,885)
    rig.write_ids=set()
    with pytest.raises(SafetyError):rig.tune_a_shoulder(100,250)


@pytest.mark.parametrize('fault',['hands','tilt','far_goal','moving','hardware','measurement','reverse_path','output','profile'])
def test_invalid_check_never_writes(configured,states,monkeypatch,fault):
    rig,candidate=setup_check(configured,states,monkeypatch)
    hands=True;level=True
    if fault=='hands':hands=False
    if fault=='tilt':level=False
    if fault=='far_goal':rig.states[12].goal_position=3400
    if fault=='moving':rig.states[12].present_velocity=1
    if fault=='hardware':rig.states[13].hardware_error=4
    if fault=='measurement':candidate['pairs']['A'][1]['calibration']['inclination_measurement']['leader_position']+=20
    if fault=='reverse_path':candidate['pairs']['A'][1]['calibration']['follower_zero']+=200
    if fault=='output':rig.states[12].present_pwm=-251
    if fault=='profile':rig.states[12].profile_velocity=0
    with pytest.raises(SafetyError):
        check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=hands,bases_level=level)
    assert not rig.writes


def test_other_joint_motion_stops_shoulder_without_unlocking_others(configured,states,monkeypatch):
    rig,candidate=setup_check(configured,states,monkeypatch)
    sample=rig.sample
    def disturbed():
        if any(w[1]==116 for w in rig.writes):rig.states[13].position+=9
        return sample()
    rig.sample=disturbed
    with pytest.raises(SafetyError,match='다른 관절'):
        check_shoulder_tilt(rig,configured,candidate,MemoryEvents(),hands_clear=True,bases_level=True)
    assert all(w[0]==12 for w in rig.writes)
    assert rig.writes[-1][1]==116 and rig.writes[-1][2]>3370


def test_saved_gain_restores_before_torque_and_refuses_unexpected_state(configured):
    from teleop.hardware import Rig
    rig=Rig(configured,'A',allow_motion=True)
    rig.metadata[12]=dict(model=1030,drive_mode=0,operating_mode=3,homing_offset=0)
    registers={64:0,98:0,84:800,82:0,80:0,100:885};writes=[]
    rig.read_register=lambda i,a,n:registers[a]
    class Packet:
        def writeTxRx(self,port,i,a,n,data):
            assert registers[64]==0
            registers[a]=int.from_bytes(bytes(data),'little');writes.append((i,a,registers[a]));return 0,0
    rig.packet=Packet();rig.ports['followers']=object()
    rig.restore_a_shoulder_gain()
    assert writes==[(12,84,1200)]
    rig.restore_a_shoulder_gain();assert len(writes)==1
    registers.update({84:800,64:1})
    with pytest.raises(SafetyError):rig.restore_a_shoulder_gain()
    assert len(writes)==1
