import json
import pytest
from teleop.config import SafetyError
from teleop.hardware import Rig, signed32
from teleop.runtime import Events, preflight, run_session
from conftest import FakeRig, MemoryEvents


def test_dry_run_has_zero_writes(configured,states):
    rig=FakeRig(configured,states);events=MemoryEvents()
    result=run_session(rig,configured,'both',.06,False,events)
    assert result['frames']>=1 and rig.writes==[]
    frames=[e for e in events.items if e['schema']=='teleop.targets.v1']
    assert all(not e['hardware_command_sent'] and len(e['joints'])==10 for e in frames)


def test_motion_only_writes_followers_and_holds(configured,states):
    rig=FakeRig(configured,states);events=MemoryEvents()
    run_session(rig,configured,'both',.06,True,events)
    followers={11,12,13,14,15,21,22,23,24,25}
    assert all(w[0] in followers for w in rig.writes if w[0]!='goals')
    frames=[e for e in events.items if e['schema']=='teleop.targets.v1']
    sent=[w[1] for w in rig.writes if w[0]=='goals']
    assert sent==[{j['follower_id']:j['target_ticks'] for j in e['joints']} for e in frames]
    assert not any(w[1:]==(64,0) for w in rig.writes)
    assert all(rig.states[i].watchdog==15 for i in followers)


def test_unreviewed_no_motor_write(configured,states):
    rig=FakeRig(configured,states);configured['pairs']['A'][0]['calibration']['reviewed']=False
    with pytest.raises(SafetyError):run_session(rig,configured,'both',.01,True,MemoryEvents())
    assert rig.writes==[]

@pytest.mark.parametrize('field,value',[('operating_mode',4),('drive_mode',8),('homing_offset',100)])
def test_changed_config_no_write(configured,states,field,value):
    rig=FakeRig(configured,states);rig.metadata[11][field]=value
    with pytest.raises(SafetyError):preflight(rig,configured,'both',rig.sample())
    assert rig.writes==[]


def test_encoder_outside_single_turn_rejected(configured,states):
    states[11].position=4110;rig=FakeRig(configured,states)
    with pytest.raises(SafetyError):run_session(rig,configured,'both',.01,True,MemoryEvents())
    assert rig.writes==[]


def test_transport_blocks_writes_in_readonly_mode(configured):
    rig=Rig(configured,allow_motion=False)
    with pytest.raises(SafetyError):rig.write(11,64,1,1)
    with pytest.raises(SafetyError):rig.goals({11:2048}, current_positions={11:2048}, speed_deg_s=15, period_s=.05)
    rig=Rig(configured,allow_motion=True)
    with pytest.raises(SafetyError):rig.write(16,64,1,1)
    with pytest.raises(SafetyError):rig.write(11,7,50,1)


def test_missing_sample_prevents_new_goals(configured,states):
    rig=FakeRig(configured,states);original=rig.sample;counter=0
    def sample():
        nonlocal counter
        counter+=1
        if counter>=4:raise SafetyError('lost bus')
        return original()
    rig.sample=sample
    with pytest.raises(SafetyError):run_session(rig,configured,'both',1,True,MemoryEvents())
    assert not [w for w in rig.writes if w[0]=='goals']


def test_jsonl_roundtrip(tmp_path):
    path=tmp_path/'frames.jsonl';events=Events(path)
    events.emit({'schema':'teleop.targets.v1','sequence':1,'joints':[]});events.close()
    assert json.loads(path.read_text())['sequence']==1


def test_signed_position():
    assert signed32(0xfffffff0)==-16


def test_torque_enable_failure_has_watchdog_and_cleanup(configured,states):
    rig=FakeRig(configured,states);original=rig.write;events=MemoryEvents()
    def write(i,address,value,size):
        if i==12 and address==64:raise SafetyError('enable failed')
        return original(i,address,value,size)
    rig.write=write
    with pytest.raises(SafetyError):run_session(rig,configured,'both',1,True,events)
    assert rig.states[11].watchdog==15
    assert rig.states[11].torque==1
    assert not [w for w in rig.writes if w[0]=='goals']
    assert events.items[-1]['schema']=='teleop.stopped.v1'


def test_watchdog_fault_is_not_cleared_during_hold(configured,states):
    from teleop.runtime import hold
    rig=FakeRig(configured,states);rig.states[11].watchdog=255
    errors=hold(rig,{11})
    assert errors and not rig.writes
