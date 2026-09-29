import runpy

import pytest

from conftest import FakeRig, MemoryEvents


@pytest.mark.parametrize('uncommanded_joint_motion',[False,True])
def test_bounded_base_alignment_only_commands_base(configured,states,monkeypatch,uncommanded_joint_motion):
    j=configured['pairs']['A'][0]
    j['calibration'].update(follower_zero=1060,leader_zero=4093)
    j['calibration']['signature']['follower']['operating_mode']=4
    j['follower']['extended_position']={'reference':1060,'min':548,'max':1572}
    states[11].position=972;states[16].position=3944
    for i in range(11,16):
        states[i].torque=1;states[i].goal_position=states[i].position
    class BaseRig(FakeRig):
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def raw_goal(self,i,target):return target
        def position_limits(self,i):return (548,1572) if i==11 else (0,4095)
        def goals(self,targets,**kwargs):
            assert kwargs['speed_deg_s']==2 and set(targets)=={11}
            super().goals(targets,**kwargs)
            self.states[11].goal_position=targets[11]
            if uncommanded_joint_motion:self.states[12].position+=20
        def sample(self):
            result=super().sample()
            result[11].raw_position=result[11].position
            result[11].position_trajectory=result[11].goal_position
            return result
    rig=BaseRig(configured,states)
    class EventLog(MemoryEvents):
        def close(self):pass
    events=EventLog();holds=[]
    def hold_session(*args,**kwargs):holds.append(args[3])
    monkeypatch.setattr('teleop.config.load',lambda path:configured)
    monkeypatch.setattr('teleop.hardware.Rig',lambda *args,**kwargs:rig)
    monkeypatch.setattr('teleop.runtime.Events',lambda *args:events)
    monkeypatch.setattr('teleop.runtime.hold_session',hold_session)
    monkeypatch.setattr('signal.signal',lambda *args:None)
    runpy.run_path('work/align_A_base.py')
    assert [w[1]for w in rig.writes if w[0]=='goals']==[{11:911}]
    expected='teleop.base_alignment_failed.v1' if uncommanded_joint_motion else 'teleop.base_aligned.v1'
    assert any(e['schema']==expected for e in events.items)
    assert holds==[.2,None]
    assert not any(w[0]!='goals' and w[1]==64 for w in rig.writes)
