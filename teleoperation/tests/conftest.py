import copy
import time
import pytest
from teleop.config import load, joints
from teleop.hardware import State


@pytest.fixture(autouse=True)
def isolate_live_status(monkeypatch):
    monkeypatch.setattr('teleop.direct.write_status',lambda payload:None)
    monkeypatch.setattr('teleop.live_actions.publish_service',lambda path:None)

@pytest.fixture
def configured():
    c = load('config/robot.json')
    for entries in c['pairs'].values():
        for j in entries:j['enabled']=True
    for _, j in joints(c):
        j['calibration'].update(reviewed=True,leader_zero=2048,follower_zero=2048,
                                direction=1,scale=1,leader_min_deg=-90,leader_max_deg=90,
                                follower_min_deg=-60,follower_max_deg=60,
                                signature={'leader':{'model':j['leader']['model'],'drive_mode':0,'operating_mode':3,'homing_offset':0},
                                           'follower':{'model':j['follower']['model'],'drive_mode':0,'operating_mode':3,'homing_offset':0}})
    for _, j in joints(c):
        if j.get("kind") == "gripper":
            j["calibration"].update(leader_closed=1800,leader_open=2296,
                                     follower_closed=1800,follower_open=2296)
    return c

@pytest.fixture
def states(configured):
    result={}
    for _, j in joints(configured):
        for role in ('leader','follower'):
            m=j[role]
            result[m['id']]=State(m['id'],2048,4.3 if m['model']==1200 else 12.,25,0,0,0,time.monotonic())
    return result

class FakeRig:
    def restore_a_shoulder_gain(self):
        self.writes.append((12,84,1200))
    def __init__(self, config, states):
        self.states=copy.deepcopy(states);self.writes=[];self.metadata={}
        for _, j in joints(config):
            for role in ('leader','follower'):
                m=j[role];self.metadata[m['id']]=dict(j['calibration']['signature'][role],position_min=0,position_max=4095,voltage_min=3,voltage_max=16)
    def sample(self):
        for s in self.states.values():s.sampled_at=time.monotonic()
        return copy.deepcopy(self.states)
    def signature(self,i):
        return {k:self.metadata[i][k] for k in ('model','drive_mode','operating_mode','homing_offset')}
    def position_limits(self,i):
        return self.metadata[i]['position_min'],self.metadata[i]['position_max']
    def write(self,i,address,value,size):
        self.writes.append((i,address,value))
        if address==64:self.states[i].torque=value
        if address==98:self.states[i].watchdog=value
    def goals(self,goals,**kwargs):
        self.writes.append(('goals',goals))
        for i,value in goals.items():self.states[i].position=value

class MemoryEvents:
    def __init__(self):self.items=[]
    def emit(self,item):self.items.append(item)
