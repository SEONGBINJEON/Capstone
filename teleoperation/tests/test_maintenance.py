import json

import pytest

from conftest import MemoryEvents
from test_resume_hold import held_rig
from teleop.config import SafetyError, save
from teleop.hardware import Rig
from teleop.maintenance import homing_shift_plan
from teleop.runtime import hold_session


def setup_shift(configured, states, tmp_path):
    c=configured['pairs']['A'][0]['calibration']
    c.update(follower_zero=36,reviewed=False,follower_min_deg=None,follower_max_deg=None)
    rig=held_rig(configured,states)
    rig.states[11].position=27;rig.states[11].goal_position=27
    original_write=rig.write
    def write(i,address,value,size):
        original_write(i,address,value,size)
        if address==116:rig.states[i].goal_position=value
    rig.write=write
    def set_offset(i,value):
        assert rig.states[i].torque==0
        rig.writes.append((i,20,value))
        rig.states[i].position += value-rig.metadata[i]['homing_offset']
        rig.metadata[i]['homing_offset']=value
    rig.set_homing_offset=set_offset
    path=tmp_path/'robot.json';save(path,configured)
    return rig,path


def run_shift(rig,config,path,**kwargs):
    return hold_session(rig,config,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,
                        home_joint=1,home_offset=1024,supported=True,config_path=path,
                        backup_dir=path.parent/'backups',**kwargs)


def test_shift_preserves_physical_pose_and_other_torque(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path)
    run_shift(rig,configured,path)
    assert rig.states[11].position-rig.metadata[11]['homing_offset']==27
    assert rig.states[11].position==1051 and rig.states[11].torque==1
    assert [(w[0],w[2])for w in rig.writes if w[1]==64]==[(11,0),(11,1)]
    assert [w for w in rig.writes if w[1]==20]==[(11,20,1024)]
    assert not any(w[0]=='goals' for w in rig.writes)
    assert all(rig.states[i].torque==1 and rig.states[i].position==2048 for i in range(12,16))
    cal=json.loads(path.read_text())['pairs']['A'][0]['calibration']
    assert cal['follower_zero']==1060 and not cal['reviewed']
    assert cal['signature']['follower']['homing_offset']==1024
    backup=json.loads(next((tmp_path/'backups').glob('*.json')).read_text())
    assert backup['config']['pairs']['A'][0]['calibration']['follower_zero']==36


def test_shift_requires_physical_support_before_any_write(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path)
    with pytest.raises(SafetyError,match='받치고'):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,
                     home_joint=1,home_offset=1024,config_path=path)
    assert not rig.writes


@pytest.mark.parametrize('offset',[2048,-2048,0,True])
def test_invalid_or_noop_shift_plan(configured,states,tmp_path,offset):
    rig,path=setup_shift(configured,states,tmp_path)
    with pytest.raises(SafetyError):homing_shift_plan(rig,configured,'A',1,offset,rig.sample())
    assert not rig.writes


def test_pose_moves_when_released_does_not_reenable(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path)
    original=rig.write
    def write(i,address,value,size):
        original(i,address,value,size)
        if address==64 and value==0:rig.states[i].position+=20
    rig.write=write
    with pytest.raises(SafetyError):run_shift(rig,configured,path)
    assert rig.states[11].torque==0
    assert not [w for w in rig.writes if w[1]==20 or w[1:]==(64,1)]
    assert all(rig.states[i].torque==1 for i in range(12,16))


def test_uncertain_eeprom_write_leaves_target_off(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path)
    def fail(i,value):raise SafetyError('no response')
    rig.set_homing_offset=fail
    with pytest.raises(SafetyError):run_shift(rig,configured,path)
    assert rig.states[11].torque==0
    assert (11,64,1)not in rig.writes


def test_save_failure_after_offset_never_energizes(configured,states,tmp_path,monkeypatch):
    rig,path=setup_shift(configured,states,tmp_path)
    def fail(*args):raise OSError('disk full')
    monkeypatch.setattr('teleop.maintenance.save',fail)
    with pytest.raises(OSError):run_shift(rig,configured,path)
    assert rig.metadata[11]['homing_offset']==1024
    assert rig.states[11].torque==0 and (11,64,1)not in rig.writes


def test_coordinate_mismatch_after_torque_enable_disables_target(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path);original=rig.write
    def write(i,address,value,size):
        original(i,address,value,size)
        if i==11 and address==64 and value==1:rig.states[i].position+=4096
    rig.write=write
    with pytest.raises(SafetyError):run_shift(rig,configured,path)
    assert rig.states[11].torque==0


def test_normal_transport_still_blocks_eeprom_and_leader(configured):
    rig=Rig(configured,allow_motion=True)
    with pytest.raises(SafetyError):rig.write(11,20,1024,4)
    with pytest.raises(SafetyError):rig.set_homing_offset(16,1024)
    with pytest.raises(SafetyError):Rig(configured).set_homing_offset(11,1024)
