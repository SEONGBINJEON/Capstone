import copy
import pytest
from teleop.config import SafetyError, TICKS_PER_DEG, require_calibration, validate
from teleop.control import Controller, delta_ticks, health
from conftest import FakeRig


def test_pair_mapping(configured,states):
    control=Controller(configured,'both',states)
    states[16].position+=20;states[26].position-=20
    goals,detail=control.step(states,.1,False)
    assert goals[11]>2048 and goals[21]<2048
    assert goals[12]==2048 and goals[22]==2048
    assert len(detail)==10


def test_wrap_near_neutral(configured,states):
    j=configured['pairs']['A'][0];j['calibration']['leader_zero']=4090
    states[16].position=4090
    c=Controller(configured,'A',states)
    target,_=c.target(j,6)
    assert target==2060
    assert delta_ticks(6,4090)==12


def test_reverse_and_scale(configured,states):
    j=configured['pairs']['A'][0];j['calibration'].update(direction=-1,scale=2)
    c=Controller(configured,'A',states)
    assert c.target(j,2058)[0]==2028


def test_speed_and_saturation(configured,states):
    c=Controller(configured,'A',states);states[16].position+=100
    goals,_=c.step(states,.05,False)
    assert 0<goals[11]-2048<=round(15*TICKS_PER_DEG*.05)
    target,saturated=c.target(configured['pairs']['A'][0],2048+round(80*TICKS_PER_DEG))
    lo,hi=c.bounds(configured['pairs']['A'][0])
    assert target==hi and saturated


def test_no_partial_controller_update_on_invalid_joint(configured,states):
    c=Controller(configured,'both',states);before=c.commanded.copy()
    states[16].position+=10;states[30].position+=1000
    with pytest.raises(SafetyError):c.step(states,.05,False)
    assert c.commanded==before

@pytest.mark.parametrize('dt',[0,-1,.3,float('nan')])
def test_reject_bad_time(configured,states,dt):
    c=Controller(configured,'both',states)
    with pytest.raises(SafetyError):c.step(states,dt)


def test_startup_mismatch(configured,states):
    states[11].position+=100
    with pytest.raises(SafetyError,match='시작 자세'):Controller(configured,'A',states)


def test_tracking_failure(configured,states):
    c=Controller(configured,'A',states);states[11].position+=200
    with pytest.raises(SafetyError,match='추종 오차'):c.step(states,.05)

@pytest.mark.parametrize('field,value',[('reviewed',False),('direction',0),('scale',float('nan')),('leader_max_deg',180),('follower_min_deg',-300),('signature',None)])
def test_invalid_calibration(configured,field,value):
    configured['pairs']['A'][0]['calibration'][field]=value
    with pytest.raises(SafetyError):require_calibration(configured,'A')


def test_voltage_hardware_and_temperature(configured,states):
    r=FakeRig(configured,states)
    states[16].voltage=12
    with pytest.raises(SafetyError):health(configured,states,r.metadata)
    states[16].voltage=4.3;states[11].hardware_error=4
    with pytest.raises(SafetyError):health(configured,states,r.metadata)
    states[11].hardware_error=0;states[11].temperature=80
    with pytest.raises(SafetyError):health(configured,states,r.metadata)


def test_reject_duplicate_ids(configured):
    configured['pairs']['B'][0]['leader']['id']=16
    with pytest.raises(SafetyError):validate(configured)
