import json
import math
from pathlib import Path

MODELS = {1030: "XM430-W210", 1060: "XL430-W250", 1070: "XC430-W150", 1200: "XL330-M288"}
TICKS_PER_DEG = 4096 / 360

class SafetyError(RuntimeError):
    pass

def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

def load(path):
    config = json.loads(Path(path).read_text())
    validate(config)
    return config

def validate(config):
    if config.get("version") != 1 or config.get("baud") != 1000000:
        raise SafetyError("설정 형식 또는 통신 속도가 잘못됐습니다. version=1, baud=1000000 필요")
    if config.get("active_pair", "A") not in ("A", "B", "both"):
        raise SafetyError("active_pair는 A, B, both 중 하나여야 합니다.")
    if config.get('startup_mode', 'matched') not in ('matched', 'align'):
        raise SafetyError('startup_mode는 matched 또는 align이어야 합니다.')
    buses = config["buses"]
    if not buses or len(set(buses.values())) != len(buses):
        raise SafetyError("보드 포트가 비어 있거나 중복됩니다.")
    seen = set()
    for pair, joints in config["pairs"].items():
        if pair not in ("A", "B") or len(joints) != 5:
            raise SafetyError("A/B 각 쌍에는 5개 관절이 필요합니다.")
        for j in joints:
            if not isinstance(j.get('enabled',True),bool):
                raise SafetyError('enabled는 true 또는 false여야 합니다.')
            if j.get("kind", "joint") not in ("joint", "gripper"):
                raise SafetyError("관절 종류는 joint 또는 gripper여야 합니다.")
            for role in ("leader", "follower"):
                m = j[role]
                if 'extended_position' in m:
                    w=m['extended_position']
                    if role!='follower' or not isinstance(w,dict) or not all(isinstance(w.get(k),int) and not isinstance(w[k],bool) for k in ('min','max','reference')):
                        raise SafetyError('확장 위치 제어 범위 설정이 유효하지 않습니다.')
                    if not 0 <= w['min'] < w['reference'] < w['max'] <= 4095 or max(w['reference']-w['min'],w['max']-w['reference'])>=2048:
                        raise SafetyError('확장 위치 제어 범위는 기준점 양쪽 180도 미만이어야 합니다.')
                    if 'raw_reference' in w and (not isinstance(w['raw_reference'],int) or isinstance(w['raw_reference'],bool)):
                        raise SafetyError('확장 위치 원시 기준값은 정수여야 합니다.')
                if m["bus"] not in buses or m["model"] not in MODELS or not isinstance(m["id"], int) or not 0 <= m["id"] <= 252:
                    raise SafetyError("모터 설정이 잘못됐습니다.")
                if m["id"] in seen:
                    raise SafetyError("모터 ID가 중복됩니다.")
                seen.add(m["id"])
    safety = config["safety"]
    for key in ("rate_hz", "max_cycle_s", "max_speed_deg_s", "startup_tolerance_deg", "max_tracking_error_deg", "max_leader_step_deg", "max_temperature_c"):
        if not finite(safety[key]) or safety[key] <= 0:
            raise SafetyError(f"유효하지 않은 안전 설정: {key}")
    if not 1 <= safety["rate_hz"] <= 50 or not 1/safety["rate_hz"] < safety["max_cycle_s"] <= .2:
        raise SafetyError("루프는 1~50 Hz, 시간 초과는 루프 주기보다 크고 0.2초 이하여야 합니다.")
    if not 0 < safety["max_speed_deg_s"] <= 30 or not 0 < safety["startup_tolerance_deg"] <= 5:
        raise SafetyError("초기 버전은 최대 30°/s, 시작 오차 5°까지 허용합니다.")
    if safety["max_leader_step_deg"] >= 180:
        raise SafetyError("리더 위치 점프 한계는 180° 미만이어야 합니다.")

def joints(config, pair="both"):
    return [(name, j) for name, js in config["pairs"].items() if pair in ("both", name) for j in js if j.get("enabled",True)]

def motors(config, pair="both"):
    return [j[role] for _, j in joints(config, pair) for role in ("leader", "follower")]

def require_calibration(config, pair="both"):
    for name, j in joints(config, pair):
        c = j["calibration"]
        prefix = f"{name}/{j['name']}"
        if j.get("kind") == "gripper":
            require_gripper(c, prefix)
            continue
        if c.get("reviewed") is not True:
            raise SafetyError(f"{prefix}: 기준 위치·회전 방향·이동 범위를 보정하고 reviewed=true로 설정해야 합니다.")
        keys = ("leader_zero", "follower_zero", "direction", "scale", "leader_min_deg", "leader_max_deg", "follower_min_deg", "follower_max_deg")
        if not all(finite(c.get(k)) for k in keys):
            raise SafetyError(f"{prefix}: 보정 값이 없거나 유효하지 않습니다.")
        if c["direction"] not in (-1, 1) or not 0 < c["scale"] <= 10:
            raise SafetyError(f"{prefix}: direction은 ±1, scale은 0 초과 10 이하입니다.")
        if not -180 < c["leader_min_deg"] <= 0 <= c["leader_max_deg"] < 180 or c["leader_min_deg"] == c["leader_max_deg"]:
            raise SafetyError(f"{prefix}: 리더 범위는 기준점 기준 ±180° 안쪽이어야 합니다.")
        if not c["follower_min_deg"] <= 0 <= c["follower_max_deg"] or c["follower_min_deg"] == c["follower_max_deg"]:
            raise SafetyError(f"{prefix}: 팔로워 이동 범위에 기준점이 포함되어야 합니다.")
        if not 0 <= c["follower_zero"] <= 4095:
            raise SafetyError(f"{prefix}: 팔로워 기준 위치는 단회전 범위여야 합니다.")
        lo = c["follower_zero"] + c["follower_min_deg"] * TICKS_PER_DEG
        hi = c["follower_zero"] + c["follower_max_deg"] * TICKS_PER_DEG
        if not 0 <= lo < hi <= 4095 or math.ceil(lo) >= math.floor(hi):
            raise SafetyError(f"{prefix}: 보정 범위가 단회전 위치 제어 범위를 벗어납니다.")
        if not isinstance(c.get("signature"), dict):
            raise SafetyError(f"{prefix}: capture-neutral로 기준점을 저장해야 합니다.")

def require_gripper(c, prefix):
    keys = ("leader_closed", "leader_open", "follower_closed", "follower_open")
    if c.get("reviewed") is not True or not all(finite(c.get(k)) for k in keys):
        raise SafetyError(f"{prefix}: 양쪽 그리퍼의 열린/닫힌 위치를 기록하고 확인해야 합니다.")
    travel = (c["leader_open"] - c["leader_closed"] + 2048) % 4096 - 2048
    if not 8 <= abs(travel) < 2048:
        raise SafetyError(f"{prefix}: 리더 개폐 범위는 8 tick 이상, 180° 미만이어야 합니다.")
    lo, hi = sorted((c["follower_closed"], c["follower_open"]))
    if not 0 <= lo < hi <= 4095 or hi-lo < 8:
        raise SafetyError(f"{prefix}: 팔로워 개폐 범위가 잘못됐습니다.")
    if not isinstance(c.get("signature"), dict):
        raise SafetyError(f"{prefix}: 그리퍼 보정 당시 모터 설정 기록이 필요합니다.")

def follower_bounds(j):
    c = j["calibration"]
    if j.get("kind") == "gripper":
        lo, hi = sorted((c["follower_closed"], c["follower_open"]))
    else:
        lo = c["follower_zero"] + c["follower_min_deg"]*TICKS_PER_DEG
        hi = c["follower_zero"] + c["follower_max_deg"]*TICKS_PER_DEG
    return math.ceil(lo), math.floor(hi)

def save(path, config):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)
