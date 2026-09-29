"""Run teleop.run() against a simulated rig for a few seconds. No hardware."""
import sys, os, time, math, _thread
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import importlib.util
import common
spec = importlib.util.spec_from_file_location('sim_follow', os.path.join(HERE, 'teleop.py'))
teleop = importlib.util.module_from_spec(spec); spec.loader.exec_module(teleop)
from teleop.hardware import State  # real dataclass (package, ROOT on sys.path via common)

cfg = common.load_config()
LEAD = {26: 2000, 27: 2036, 28: 2047, 29: 2051, 30: 700}   # leader wanders around zero
FOLL = {11: 1060, 12: 2334, 13: 1995, 14: 2064, 15: 1400}
META = {i: dict(model=1200, drive_mode=0, operating_mode=0, homing_offset=0, voltage_min=3.7, voltage_max=6, position_min=0, position_max=4095) for i in LEAD}
META[30] = dict(META[26], model=1070, operating_mode=3, voltage_min=6.5, voltage_max=12)
for i in FOLL: META[i] = dict(model=1060, drive_mode=4, operating_mode=3, homing_offset=0, voltage_min=6.5, voltage_max=14.8, position_min=0, position_max=4095)
META[11]['operating_mode'] = 4; META[12]['drive_mode'] = 0; META[15]['drive_mode'] = 0

class FakeRig:
    def __init__(self):
        self.metadata = META; self.t0 = time.monotonic(); self.torque = {i: 0 for i in FOLL}; self.wd = {i: 0 for i in FOLL}
        self.goal = dict(FOLL); self.pos = {i: float(v) for i, v in FOLL.items()}; self.writes = []
    def __enter__(self): return self
    def __exit__(self, *a): pass
    def position_limits(self, i): return (548, 1572) if i == 11 else (0, 4095)
    def sample(self):
        t = time.monotonic() - self.t0
        if t > 1.5:  # after 1.5 s the leader starts moving
            LEAD[27] = 2036 + int(300 * math.sin((t - 1.5) * 1.5)); LEAD[30] = 700 + int(400 * math.sin((t - 1.5) * 1.0))
        st = {}
        for i, p in LEAD.items():
            st[i] = State(i, p, 4.5 if i != 30 else 4.4, 25, 0, 1 if i == 30 else 0, 0, time.monotonic(), p, p, p, 0)
        for i in FOLL:
            if self.torque[i]:  # first-order approach to goal
                self.pos[i] += (self.goal[i] - self.pos[i]) * 0.5
            p = round(self.pos[i])
            st[i] = State(i, p, 12.0, 35, self.torque[i], 0, self.wd[i], time.monotonic(), p, self.goal[i], self.goal[i], 0)
        return st
    def write(self, i, addr, val, size):
        self.writes.append((i, addr, val))
        if addr == 64: self.torque[i] = val
        if addr == 98: self.wd[i] = val
        if addr == 116: self.goal[i] = val
    def stream_goals(self, targets, states, *, profile_speed_deg_s, period_s):
        assert 0 < profile_speed_deg_s <= 180
        for i, g in targets.items():
            lo, hi = self.position_limits(i); assert lo <= g <= hi, (i, g); self.goal[i] = g

rig = FakeRig()
common.open_rig = lambda cfg, allow_motion: rig
teleop.open_rig = lambda cfg, allow_motion: rig
class A: dry_run = False; match = True
_thread.start_new_thread(lambda: (time.sleep(6), _thread.interrupt_main()), ())
teleop.run(cfg, A)
print('WRITES(first 12):', rig.writes[:12])
print('final goals:', rig.goal, 'torque:', rig.torque, 'wd:', rig.wd)
