"""Isaac Sim viewer: mirrors the follower arm from UDP joint packets.

Run with Isaac Sim's python:
    ~/isaacsim-6.0.1/python.sh sim_teleop/isaac_view.py                 # kitchen scene (default)
    ~/isaacsim-6.0.1/python.sh sim_teleop/isaac_view.py --scene plain   # plain grid floor
Screenshots while running:  touch work/capture_request   -> work/shots/shot_<time>.png
Options: --headless, --usd PATH, --port 5005, --capture PNG (headless one-shot), --root-rotate-x 90
"""
import argparse
import json
import math
import os
import socket
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
import sys  # noqa: E402
sys.path.insert(0, HERE)

def default_robot_usd():
    """Repo layout first (isaac_sim/robot_model/...), then $ROBOT_USD, then the original dev path."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.environ.get('ROBOT_USD'),
                  os.path.join(here, '..', '..', 'isaac_sim', 'robot_model', 'usd_isaac', 'robot_isaac', 'robot_isaac.usda'),
                  os.path.expanduser('~/Desktop/urdf_v3/my_manipulator/usd_isaac/robot_isaac/robot_isaac.usda')]
    for c in candidates:
        if c and os.path.exists(c):
            return os.path.abspath(c)
    return candidates[1]


ap = argparse.ArgumentParser()
ap.add_argument('--usd', default=default_robot_usd())
ap.add_argument('--port', type=int, default=5005)
ap.add_argument('--headless', action='store_true')
ap.add_argument('--scene', choices=['plain', 'kitchen'], default='kitchen')
ap.add_argument('--root-rotate-x', type=float, default=90.0, help='URDF zero pose lies flat; rotate root about X (deg) so the base stands up')
ap.add_argument('--capture', default=None, help='PNG path: render a few frames, save a screenshot, exit (for checks)')
ap.add_argument('--props', action='store_true', help='also load Isaac library props over the network (experimental: can render black)')
ap.add_argument('--settle-frames', type=int, default=90, help='frames to render before a --capture')
ap.add_argument('--pose', default=None, help='JSON {"joint_11":deg,...} to pose the robot when no UDP packets (for captures)')
ap.add_argument('--config', default=os.path.join(HERE, 'config.json'))
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({'headless': args.headless, 'width': 1920, 'height': 1080, 'window_width': 1920, 'window_height': 1080})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux, UsdShade, UsdPhysics, Sdf, Gf  # noqa: E402

SHOT_DIR = os.path.join(ROOT, 'work', 'shots')
CAPTURE_REQUEST = os.path.join(ROOT, 'work', 'capture_request')
os.makedirs(SHOT_DIR, exist_ok=True)


class Receiver(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.5)
        self.latest = None; self.stamp = 0.0; self.count = 0

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                self.latest = json.loads(data.decode()); self.stamp = time.time(); self.count += 1
            except ValueError:
                pass


from scene_lib import make_material, bind, box, cylinder, mini_frying_pan, kitchen_materials, build_kitchen  # noqa: E402


# ----------------------------------------------------------------------------- load robot
with open(args.config) as f:
    cfg = json.load(f)
sim_joint_names = [j['sim']['joint'] for j in cfg['joints']]
view_sign = {j['sim']['joint']: j['sim'].get('view_sign', 1) for j in cfg['joints']}

ctx = omni.usd.get_context()
ctx.open_stage(args.usd)
for _ in range(10):
    app.update()
stage = ctx.get_stage()
robot_prim = stage.GetDefaultPrim()
robot_path = str(robot_prim.GetPath())

if args.root_rotate_x:
    root_x = UsdGeom.Xformable(robot_prim)
    root_x.ClearXformOpOrder()
    root_x.AddRotateXOp().Set(float(args.root_rotate_x))
    for _ in range(3):
        app.update()

# Fin-Ray fingers (follower_07, follower_07_2) in blue.
finray = make_material(stage, '/World/Looks/FinRayBlue', (0.08, 0.32, 0.9), roughness=0.45)
for p in Usd.PrimRange(robot_prim):
    if 'follower_07' in p.GetName() and p.IsA(UsdGeom.Xformable):
        bind(p, finray)

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
bb = cache.ComputeWorldBound(robot_prim).ComputeAlignedRange()
bmin, bmax = Gf.Vec3d(bb.GetMin()), Gf.Vec3d(bb.GetMax())
center = (bmin + bmax) / 2; size = max((bmax - bmin).GetLength(), 0.3)
print(f'ROBOT BBOX min={tuple(round(v,3) for v in bmin)} max={tuple(round(v,3) for v in bmax)}')

world = World(stage_units_in_meters=1.0)
if args.scene == 'plain':
    world.scene.add_default_ground_plane(z_position=float(bmin[2]) - 0.002)
else:
    build_kitchen(stage, bmin, bmax, with_props=args.props)

dome = UsdLux.DomeLight.Define(stage, Sdf.Path('/World/DomeLight')); dome.CreateIntensityAttr(700 if args.scene == 'kitchen' else 1000)
key = UsdLux.DistantLight.Define(stage, Sdf.Path('/World/KeyLight')); key.CreateIntensityAttr(2500)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))
if args.scene == 'kitchen':
    fill = UsdLux.SphereLight.Define(stage, Sdf.Path('/World/FillLight'))
    fill.CreateIntensityAttr(20000); fill.CreateRadiusAttr(0.15)
    UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(center + Gf.Vec3d(-0.8, -1.2, 0.9))
world.reset()

art = Articulation(robot_path)
art.initialize()
names = list(art.dof_names)
print('DOF:', names)
idx = [names.index(n) for n in sim_joint_names]
n = len(names)
art.set_gains(kps=np.full((1, n), 1e6, dtype=np.float32), kds=np.full((1, n), 1e4, dtype=np.float32))
art.switch_control_mode('position')

base = Gf.Vec3d(center[0], center[1], float(bmin[2]))
if args.scene == 'kitchen':
    eye = base + Gf.Vec3d(-0.75, -0.95, 0.55); target = base + Gf.Vec3d(0.0, 0.0, 0.22)
else:
    eye = base + Gf.Vec3d(-0.55, -0.7, 0.42); target = base + Gf.Vec3d(0.0, 0.0, 0.2)
set_camera_view(eye=[float(v) for v in eye], target=[float(v) for v in target])

rx = Receiver(args.port); rx.start()
print(f'UDP {args.port} 수신 대기. 스크린샷: touch work/capture_request')

current = np.array(art.get_joint_positions()[0], dtype=np.float32)
if args.pose:
    pose = json.loads(args.pose)
    for k, name in enumerate(sim_joint_names):
        if name in pose:
            current[idx[k]] = math.radians(pose[name])


def apply_pose():
    art.set_joint_position_targets(current)
    art.set_joint_positions(current)
    art.set_joint_velocities(np.zeros_like(current))


def capture(path):
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    capture_viewport_to_file(get_active_viewport(), path)
    for _ in range(20):
        app.update()
    print('CAPTURED', path, flush=True)


if args.capture:
    for _ in range(args.settle_frames):
        apply_pose(); world.step(render=True)
    capture(args.capture); app.close(); raise SystemExit

last_report = 0.0
while app.is_running():
    pkt = rx.latest
    if pkt is not None:
        target = current.copy()
        for k, name in enumerate(sim_joint_names):
            j = pkt['joints'].get(name)
            if j is not None:
                target[idx[k]] = float(j['rad']) * view_sign.get(name, 1)
        current = target
    apply_pose()
    world.step(render=True)
    if os.path.exists(CAPTURE_REQUEST):
        try:
            os.remove(CAPTURE_REQUEST)
        except OSError:
            pass
        capture(os.path.join(SHOT_DIR, time.strftime('shot_%Y%m%d_%H%M%S.png')))
    if time.time() - last_report > 2.0:
        last_report = time.time()
        if pkt is None:
            print('패킷 없음 (teleop.py 실행 대기)', flush=True)
        else:
            degs = ' '.join(f"{nm}:{math.degrees(current[idx[k]]):6.1f}°" for k, nm in enumerate(sim_joint_names))
            print(f"[{pkt.get('phase')}] {degs}  (age {time.time()-rx.stamp:.2f}s, n={rx.count})", flush=True)
app.close()
