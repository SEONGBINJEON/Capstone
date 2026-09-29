import json

import pytest

from conftest import FakeRig,MemoryEvents
from test_maintenance import setup_shift
from teleop.config import SafetyError,save,motors
from teleop.hardware import Rig
from teleop.runtime import hold_session


def extended_rig(configured,states,raw):
    configured['pairs']['A'][0]['follower']['extended_position']={'reference':1060,'min':548,'max':1572}
    rig=Rig(configured,'A',allow_motion=True)
    rig.metadata=FakeRig(configured,states).metadata
    rig.metadata[11]['operating_mode']=4
    rig.metadata[11]['homing_offset']=1024
    class Group:
        data_dict={i:None for i in rig.specs}
        def txRxPacket(self):return 0
        def isAvailable(self,*args):return True
        def getData(self,i,address,size):
            value={64:1 if i<16 else 0,98:15 if i<16 else 0,116:raw if i==11 else 2048,
                   132:raw if i==11 else 2048,144:120,146:30}.get(address,0)
            return value & ((1<<(8*size))-1)
    group=Group();rig.groups={'test':group}
    return rig,group


@pytest.mark.parametrize('raw',[912,5008,-3184])
def test_restart_turn_coordinate_round_trip(configured,states,raw):
    rig,_=extended_rig(configured,states,raw)
    s=rig.sample()[11]
    assert s.position==912 and s.goal_position==912 and s.raw_position==raw
    assert rig.raw_goal(11,911)==raw-1
    assert rig.position_limits(11)==(548,1572)
    with pytest.raises(SafetyError):rig.raw_goal(11,547)
    with pytest.raises(SafetyError):rig.raw_goal(11,1573)


def test_unexpected_turn_jump_is_not_renormalized(configured,states):
    rig,group=extended_rig(configured,states,5008);rig.sample()
    original=group.getData
    def get(i,address,size):
        value=original(i,address,size)
        return value-4096 if i==11 and address==132 else value
    group.getData=get
    assert rig.sample()[11].position==-3184
    assert rig.extended_turn_offsets[11]==4096


def test_start_outside_window_and_missing_window_are_rejected(configured,states):
    rig,_=extended_rig(configured,states,2500)
    with pytest.raises(SafetyError):rig.sample()
    del rig.specs[11]['extended_position']
    with pytest.raises(SafetyError):rig.sample()


def test_signed_goal_packets_and_write_bounds(configured,states):
    rig,_=extended_rig(configured,states,-3184);rig.sample()
    writes=[]
    class Packet:
        def writeTxRx(self,*args):writes.append(args);return 0,0
    rig.packet=Packet();rig.ports={'followers':object()}
    rig.write(11,116,911,4)
    assert int.from_bytes(bytes(writes[0][-1]),'little',signed=True)==-3185
    with pytest.raises(SafetyError):rig.write(11,116,1600,4)
    assert len(writes)==1


def prepare_transition(configured,states,tmp_path):
    rig,path=setup_shift(configured,states,tmp_path)
    c=configured['pairs']['A'][0]['calibration'];c['follower_zero']=1060
    c['signature']['follower']['homing_offset']=1024
    rig.metadata[11]['homing_offset']=1024
    rig.states[11].position=1026;rig.states[11].goal_position=1026
    rig.specs={m['id']:m for m in motors(configured)}
    def set_mode(i):
        assert rig.states[i].torque==0
        rig.writes.append((i,11,4));rig.metadata[i]['operating_mode']=4
    rig.set_extended_mode=set_mode
    old_limits=rig.position_limits
    def limits(i):
        if rig.metadata[i]['operating_mode']==4:
            w=rig.specs[i]['extended_position'];return w['min'],w['max']
        return old_limits(i)
    rig.position_limits=limits
    save(path,configured)
    return rig,path


def test_transition_keeps_pose_only_toggles_base_and_sets_window(configured,states,tmp_path):
    rig,path=prepare_transition(configured,states,tmp_path)
    hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,
                 extended_base=True,supported=True,config_path=path,backup_dir=tmp_path/'backup')
    assert rig.states[11].position==1026 and rig.states[11].torque==1
    assert [w for w in rig.writes if w[1]==64]==[(11,64,0),(11,64,1)]
    j=json.loads(path.read_text())['pairs']['A'][0]
    assert j['follower']['extended_position']=={'reference':1060,'min':548,'max':1572}
    assert j['calibration']['signature']['follower']['operating_mode']==4
    assert not j['calibration']['reviewed']


def test_transition_requires_support(configured,states,tmp_path):
    rig,path=prepare_transition(configured,states,tmp_path)
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,
                     extended_base=True,config_path=path)
    assert not rig.writes


def test_failed_mode_write_keeps_base_off(configured,states,tmp_path):
    rig,path=prepare_transition(configured,states,tmp_path)
    def fail(i):raise SafetyError('write uncertain')
    rig.set_extended_mode=fail
    with pytest.raises(SafetyError):
        hold_session(rig,configured,'A',.01,MemoryEvents(),resume=True,recover_watchdog=True,
                     extended_base=True,supported=True,config_path=path,backup_dir=tmp_path/'backup')
    assert rig.states[11].torque==0
    assert (11,64,1)not in rig.writes
    assert all(rig.states[i].torque==1 for i in range(12,16))


@pytest.mark.parametrize('raw',[4080,-16,8176])
def test_calibrated_raw_reference_centers_base_without_homing_write(configured,states,raw):
    rig,_=extended_rig(configured,states,raw)
    rig.specs[11]['extended_position']={'reference':2048,'raw_reference':4080,'min':1,'max':4095}
    s=rig.sample()[11]
    assert s.position==2048
    assert rig.raw_goal(11,2058)==raw+10
    assert rig.raw_goal(11,2038)==raw-10
