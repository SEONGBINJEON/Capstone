import time
from run_teleop import describe


def test_status_distinguishes_command_error_from_large_leader_lag():
    s={'phase':'following','unix_time':time.time(),'motors':{'22':{'position':1500,'goal_position':1580}},'desired':{'22':3000}}
    text=describe(s)
    assert '7.0°' in text and '131.8°' in text


def test_stale_telemetry_does_not_claim_following():
    assert '끊겼습니다' in describe({'phase':'following','unix_time':time.time()-10})


def test_fault_reason_visible():
    assert 'USB 오류' in describe({'phase':'fault','reason':'USB 오류'})
