import pytest
import json
from conftest import FakeRig,MemoryEvents
from teleop.config import SafetyError, TICKS_PER_DEG
from teleop.live_actions import ShoulderTrial,execute_action


@pytest.mark.parametrize('failed_id',[None,21])
def test_explicit_torque_off_releases_enabled_followers_and_reports_failure(configured,states,failed_id):
    configured['pairs']['B'][-1]['enabled']=False
    ids=set(range(11,16))|set(range(21,25))
    for i in ids|{25}:states[i].torque=1
    class ReleaseRig(FakeRig):
        def write(self,i,a,v,n):
            assert i in ids and a in (64,98) and v==0
            if i==failed_id and a==64:raise SafetyError('injected write failure')
            super().write(i,a,v,n)
    rig=ReleaseRig(configured,states);events=MemoryEvents()
    if failed_id is None:
        execute_action(rig,configured,events,{'action':'torque_off'},'unused')
        assert events.items[-1]['phase']=='torque_off'
    else:
        with pytest.raises(SafetyError,match='injected write failure'):
            execute_action(rig,configured,events,{'action':'torque_off'},'unused')
        assert events.items[-1]['phase']=='fault'
    assert all(rig.states[i].torque==int(i==failed_id) for i in ids)
    assert rig.states[25].torque==1
    assert all(rig.states[i].watchdog==0 for i in ids)


def trial(configured,states):
    states[12].goal_position=2048
    states[12].raw_position=states[12].position_trajectory=2048
    configured['pairs']['A'][1]['calibration']['pose_alignment_verified']=True
    t=ShoulderTrial(configured,states)
    for _ in range(3):t.step(states,.05)
    assert t.phase=='following'
    return t


def test_uses_fixed_calibration_and_slew_rate(configured,states):
    t=trial(configured,states)
    states[17].position+=40
    g=t.step(states,.05)
    assert 2048<g<=2050 and t.target(states[17].position)==2088


@pytest.mark.parametrize('fault',['range','trajectory','delay'])
def test_trial_stops_on_bounded_range_tracking_or_delay(configured,states,fault):
    t=trial(configured,states)
    if fault=='range':states[17].position+=int(5*TICKS_PER_DEG)+1
    if fault=='trajectory':states[12].position_trajectory+=int(3*TICKS_PER_DEG)+1
    with pytest.raises(SafetyError):t.step(states,.3 if fault=='delay' else .05)


def test_live_session_only_commands_shoulder_and_can_pause(configured,states,monkeypatch,tmp_path):
    configured['pairs']['A'][1]['calibration']['pose_alignment_verified']=True
    for i in range(11,16):
        s=states[i];s.torque=1;s.watchdog=255;s.goal_position=s.position
        s.raw_position=s.position_trajectory=s.position;s.present_velocity=0
    s=states[12];s.position_p_gain=1200;s.position_i_gain=s.position_d_gain=0
    s.goal_pwm=885;s.profile_acceleration=5;s.profile_velocity=1
    clock=[0.]
    def now():clock[0]+=.001;return clock[0]
    monkeypatch.setattr('time.monotonic',now)
    monkeypatch.setattr('time.sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    command={'nonce':1,'action':'follow_shoulder'}
    path=tmp_path/'control.json';path.write_text(json.dumps(command))
    class TrialRig(FakeRig):
        count=0
        def raw_goal(self,i,g):return g
        def write(self,i,a,v,n):
            super().write(i,a,v,n)
            if a==116:
                s=self.states[i];s.goal_position=s.position_trajectory=s.raw_position=s.position=v
        def sample(self):
            self.count+=1
            if self.count==18:self.states[17].position+=30
            if self.count==30:path.write_text(json.dumps({'nonce':2,'action':'hold'}))
            return super().sample()
    rig=TrialRig(configured,states);events=MemoryEvents()
    execute_action(rig,configured,events,command,path)
    assert all(w[0]==12 and w[1] in (98,112,116) for w in rig.writes)
    assert any(w[1]==116 and w[2]>2048 for w in rig.writes)
    assert events.items[-1]['schema']=='teleop.shoulder_follow_stopped.v1'
