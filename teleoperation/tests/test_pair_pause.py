import json
import pytest
from conftest import FakeRig,MemoryEvents
from teleop.direct import DirectA,PairPause,blocked_directions,run_direct
from teleop.config import SafetyError


def held(states):
    for i,s in states.items():
        s.torque=int(i in set(range(11,16))|set(range(21,26)))
        s.watchdog=15 if s.torque else 0
        s.goal_position=s.raw_position=s.position_trajectory=s.position
        s.present_velocity=0


def test_paused_arm_does_not_auto_retry_when_hold_error_clears(configured,states):
    held(states);rig=FakeRig(configured,states);c=DirectA(rig,configured,states,'B')
    states[26].position+=300
    pause=PairPause(c,states,'blocked')
    for _ in range(200):assert not pause.observe(c,states,.05)
    assert pause.goals[21]==2048
    states[26].position=2048
    for _ in range(9):assert not pause.observe(c,states,.05)
    assert pause.observe(c,states,.1)


def test_retreat_requires_crossing_hold_angle_and_all_blocked_joints(configured,states):
    held(states);rig=FakeRig(configured,states);c=DirectA(rig,configured,states,'B')
    states[26].position+=300
    states[27].position-=300
    pause=PairPause(c,states,'blocked',{
        21:{'position':2048,'direction':1},22:{'position':2048,'direction':-1}})
    # Returning toward the obstacle contact, but still on its blocked side,
    # cannot re-enable the old push even after a long wait.
    states[26].position=2100
    for _ in range(200):assert not pause.observe(c,states,.05)
    states[26].position=2000
    for _ in range(20):assert not pause.observe(c,states,.05)
    # Both blocked joints retreat. An unrelated joint need not match exactly.
    states[27].position=2100;states[28].position=2400
    for _ in range(3):assert not pause.observe(c,states,.05)
    assert pause.observe(c,states,.05)
    assert pause.resume_method=='leader_retreated' and pause.gap_deg>20


def test_blocked_direction_uses_trajectory_before_hold_in_raw_coordinates(configured,states):
    held(states);rig=FakeRig(configured,states);c=DirectA(rig,configured,states,'B')
    states[21].raw_position=4080;states[21].position_trajectory=3880
    states[22].position_trajectory=2148
    states[23].position_trajectory=2050
    directions=blocked_directions(c,states,21)
    assert directions=={21:{'position':2048,'direction':-1},22:{'position':2048,'direction':1}}


def simulation(configured,states,monkeypatch,tmp_path,*,fatal=False,manual=False,retreat=False):
    held(states)
    clock=[0.]
    def now():clock[0]+=.001;return clock[0]
    monkeypatch.setattr('time.monotonic',now)
    monkeypatch.setattr('time.sleep',lambda dt:clock.__setitem__(0,clock[0]+dt))
    path=tmp_path/'control.json';command={'nonce':1};path.write_text(json.dumps(command))
    class Rig(FakeRig):
        count=0;blocked=True
        def __init__(self,*args):super().__init__(*args);self.packets=[]
        def raw_goal(self,i,g):return g
        def stream_goals(self,goals,s,**kwargs):
            self.packets.append((self.count,goals.copy(),kwargs['profile_speed_deg_s']))
            for i,g in goals.items():
                v=self.states[i];v.goal_position=v.position_trajectory=g
                if i!=21 or not self.blocked:v.position=v.raw_position=g
        def write(self,i,a,v,n):
            super().write(i,a,v,n)
            if a==116:self.states[i].goal_position=self.states[i].position_trajectory=v
        def sample(self):
            self.count+=1
            if self.count==15:self.states[26].position+=300
            if 20<=self.count<=40:self.states[16].position+=2
            if self.count==50:
                if fatal:self.states[22].hardware_error=32
                elif manual:
                    self.blocked=False
                    path.write_text(json.dumps(dict(command,resume_token=1,resume_pairs=['B'])))
                elif retreat:
                    self.blocked=False
                    self.states[26].position=2000
                    self.states[27].position=2300
                else:self.states[26].position=2048
            if self.count==85:self.blocked=False;self.states[26].position=2100
            if self.count==120:path.write_text('{"nonce":2}')
            return super().sample()
    rig=Rig(configured,states);events=MemoryEvents()
    if fatal:
        with pytest.raises(SafetyError,match='하드웨어 오류'):
            run_direct(rig,configured,events,pair='both',motor_limits=True,command=command,control_path=path)
    else:run_direct(rig,configured,events,pair='both',motor_limits=True,command=command,control_path=path)
    return rig,events


def test_obstacle_pauses_only_b_keeps_a_and_resumes_without_old_goal(configured,states,monkeypatch,tmp_path):
    rig,events=simulation(configured,states,monkeypatch,tmp_path)
    pauses=[r for r in events.items if r['schema']=='teleop.pair_paused.v1']
    resumes=[r for r in events.items if r['schema']=='teleop.pair_resumed.v1']
    assert len(pauses)==1 and pauses[0]['pair']=='B' and pauses[0]['hold_goals'][21]==2048
    assert len(resumes)==1 and resumes[0]['method']=='leader_pose_matched'
    resumed_index=events.items.index(resumes[0])
    after=next(r for r in events.items[resumed_index+1:] if 'pair_phases' in r)
    assert after['pair_phases']['B']=='following' and after['pair_speeds_deg_s']['B']==180
    assert any(25<=count<=40 and 11 in goals for count,goals,_ in rig.packets)
    assert not any(25<=count<50 and 21 in goals for count,goals,_ in rig.packets)
    assert any(count>=85 and goals.get(21,2048)>2048 for count,goals,_ in rig.packets)
    assert not any(w[1]==64 and w[2]==0 for w in rig.writes)


def test_manual_resume_is_slow_for_b_without_slowing_a(configured,states,monkeypatch,tmp_path):
    rig,events=simulation(configured,states,monkeypatch,tmp_path,manual=True)
    resumes=[r for r in events.items if r['schema']=='teleop.pair_resumed.v1']
    assert resumes and resumes[0]['method']=='explicit'
    statuses=[r for r in events.items if 'pair_phases' in r]
    assert any(r['pair_phases']=={'A':'following','B':'aligning'} and r['pair_speeds_deg_s']['A']==180 and r['pair_speeds_deg_s']['B']==15 for r in statuses)


def test_hardware_fault_is_not_downgraded_to_obstacle_pause(configured,states,monkeypatch,tmp_path):
    _,events=simulation(configured,states,monkeypatch,tmp_path,fatal=True)
    assert any(r['schema']=='teleop.pair_paused.v1' for r in events.items)
    assert events.items[-1]['schema']=='teleop.direct_A_stopped.v1'


def test_reverse_resumes_without_command_or_matching_unblocked_joints(configured,states,monkeypatch,tmp_path):
    rig,events=simulation(configured,states,monkeypatch,tmp_path,retreat=True)
    resumes=[r for r in events.items if r['schema']=='teleop.pair_resumed.v1']
    assert resumes and resumes[0]['method']=='leader_retreated'
    assert not any(25<=count<50 and 21 in goals for count,goals,_ in rig.packets)
    assert any(50<=count<85 and goals.get(21,2048)<2048 for count,goals,_ in rig.packets)
    statuses=[r for r in events.items if 'pair_phases' in r]
    resumed_index=events.items.index(resumes[0])
    after=next(r for r in events.items[resumed_index+1:] if 'pair_phases' in r)
    assert after['pair_phases']=={'A':'following','B':'following'}
    assert after['pair_speeds_deg_s']=={'A':180,'B':180}
    first_resume=next((g,speed) for count,g,speed in rig.packets if count>=50 and 21 in g)
    goals,speed=first_resume
    assert speed==180
    assert 2048<goals[22]<2300  # slew from held pose; never jump to far leader goal
    assert abs(goals[22]-2048)<=12*4096/360
    assert rig.packets[0][2]==30  # first startup still uses slow alignment profile
