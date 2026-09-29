"""Shared helpers: config loading, Rig construction, angle conversions."""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'vendor'))
sys.path.insert(0, ROOT)

from teleop.config import SafetyError, TICKS_PER_DEG  # noqa: E402
from teleop.hardware import Rig  # noqa: E402

CONFIG_PATH = os.path.join(HERE, 'config.json')


def load_config(path=CONFIG_PATH):
    with open(path) as f:
        cfg = json.load(f)
    if len(cfg['joints']) != 5:
        raise SafetyError('joints는 5개(관절 4 + 그리퍼 1)여야 합니다.')
    return cfg


def save_config(cfg, path=CONFIG_PATH):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
        f.write('\n')
    os.replace(tmp, path)


def rig_config(cfg):
    """Translate sim_teleop/config.json into the shape teleop.hardware.Rig expects."""
    pairs = []
    for j in cfg['joints']:
        leader = dict(j['leader'], bus='leaders')
        follower = dict(j['follower'], bus='followers')
        pairs.append({'name': j['name'], 'kind': j.get('kind', 'joint'), 'enabled': True,
                      'leader': leader, 'follower': follower, 'calibration': {}})
    return {'version': 1, 'baud': cfg['baud'], 'buses': dict(cfg['buses']),
            'pairs': {'A': pairs}, 'safety': dict(cfg['safety'], rate_hz=20, max_speed_deg_s=15)}


class TolerantRig(Rig):
    """Rig that tolerates the status-packet ALERT bit (0x80) with no error code.

    A leader XC430 on the OpenRB 4.5V rail permanently flags 'input voltage'
    in Hardware Error Status; every reply then carries the alert bit. The
    actual error byte is still checked per motor in health()."""
    def check(self, result, error, operation):
        if error == 0x80:
            return super().check(result, 0, operation)
        return super().check(result, error, operation)


def open_rig(cfg, allow_motion):
    return TolerantRig(rig_config(cfg), pair='A', allow_motion=allow_motion)


def delta_ticks(position, zero):
    """Shortest signed distance on the 4096-tick circle."""
    return (position - zero + 2048) % 4096 - 2048


def leader_ids(cfg):
    return [j['leader']['id'] for j in cfg['joints']]


def follower_ids(cfg):
    return [j['follower']['id'] for j in cfg['joints']]


def joint_target(j, leader_pos, bounds):
    """Follower goal (ticks, logical coordinate) for a leader reading."""
    lo, hi = bounds
    if j.get('kind') == 'gripper':
        travel = delta_ticks(j['leader_open'], j['leader_closed'])
        frac = delta_ticks(leader_pos, j['leader_closed']) / travel if travel else 0.0
        frac = max(0.0, min(1.0, frac))
        goal = j['follower_closed'] + frac * (j['follower_open'] - j['follower_closed'])
    else:
        goal = j['follower_zero'] + delta_ticks(leader_pos, j['leader_zero']) * j['direction'] * j['scale']
    limited = not lo <= goal <= hi
    return max(lo, min(hi, goal)), limited


def follower_to_sim(j, follower_pos):
    """Simulation joint value (radians) for a follower position (ticks)."""
    sim = j['sim']
    if j.get('kind') == 'gripper':
        span = j['follower_open'] - j['follower_closed']
        frac = (follower_pos - j['follower_closed']) / span if span else 0.0
        frac = max(0.0, min(1.0, frac))
        deg = sim['closed_deg'] + frac * (sim['open_deg'] - sim['closed_deg'])
        return math.radians(deg), frac
    zero = sim.get('zero_ticks', j['follower_zero'])
    deg = (follower_pos - zero) / TICKS_PER_DEG * sim.get('sign', 1) + sim.get('offset_deg', 0)
    return math.radians(deg), None


def health(cfg, states, metadata):
    """Temperature / hardware / watchdog / voltage checks. Leaders in the
    ignore list may report an input-voltage flag (OpenRB 4.5V rail on XC430)."""
    ignore = set(cfg.get('leader_ignore_hardware_error_ids', []))
    leaders = set(leader_ids(cfg))
    for i, s in states.items():
        if s.hardware_error and not (i in leaders and i in ignore and s.hardware_error == 1):
            raise SafetyError(f'ID {i}: 하드웨어 오류 0x{s.hardware_error:02x}')
        if s.temperature >= cfg['safety']['max_temperature_c']:
            raise SafetyError(f'ID {i}: 온도 제한 초과 ({s.temperature}°C)')
        if s.watchdog == 255 and i not in leaders:
            raise SafetyError(f'ID {i}: 통신 감시 타이머 오류. 상태 확인 후 수동 복구가 필요합니다.')
        if i in leaders and i in ignore:
            continue
        lo, hi = metadata[i]['voltage_min'], metadata[i]['voltage_max']
        if metadata[i]['model'] == 1200:
            lo, hi = max(lo, 3.7), min(hi, 6.0)
        if not lo <= s.voltage <= hi:
            raise SafetyError(f'ID {i}: 전압 범위 초과 ({s.voltage}V, {lo}~{hi}V)')
