"""Render presentation stills in Isaac Sim (staged visuals, not actual training).

    ~/isaacsim-6.0.1/python.sh sim_teleop/render_gallery.py [--only name,name] [--out DIR]

Scenes:
  parallel_arms      100 follower arms in a 10x10 grid, random poses (RL-farm look)
  parallel_kitchens  3x3 kitchen environments, each with a robot in a different pose
  hero               dramatic 3/4 view of the arm on the counter
  gripper_closeup    Fin-Ray gripper over the mini frying pan
  grasp_sequence     three poses of a pan grasp (frames 01..03)
  top_view           workspace from above
"""
import argparse
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
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
ap.add_argument('--out', default=os.path.join(ROOT, 'work', 'shots', 'gallery'))
ap.add_argument('--only', default=None)
ap.add_argument('--frames', type=int, default=120, help='render frames before each capture')
ap.add_argument('--seed', type=int, default=7)
args, _ = ap.parse_known_args()
os.makedirs(args.out, exist_ok=True)
random.seed(args.seed)

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({'headless': True, 'width': 1920, 'height': 1080})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf  # noqa: E402
from scene_lib import make_material, bind, box, cylinder, mini_frying_pan, kitchen_materials, build_kitchen, xform  # noqa: E402

DOF = ['joint_11', 'joint_22', 'joint_33', 'joint_44', 'joint_55', 'joint_66', 'joint_77']
FINRAY_BLUE = (0.08, 0.32, 0.9)
# reference poses (deg): current demo pose and a few plausible working poses
POSES = {
    'demo': dict(joint_11=0, joint_22=23, joint_33=-117, joint_44=-85, joint_55=0),
    'reach': dict(joint_11=-20, joint_22=55, joint_33=-70, joint_44=-60, joint_55=25),
    'lift': dict(joint_11=15, joint_22=10, joint_33=-60, joint_44=-40, joint_55=0),
    'hover': dict(joint_11=5, joint_22=40, joint_33=-100, joint_44=-45, joint_55=35),
    'grasp': dict(joint_11=5, joint_22=48, joint_33=-108, joint_44=-45, joint_55=5),
    'carry': dict(joint_11=-25, joint_22=25, joint_33=-85, joint_44=-45, joint_55=5),
}


def random_pose():
    pj = os.path.join(args.out, 'poses.json')
    if os.path.exists(pj):
        POSES.update(json.load(open(pj)))
    keys = [k for k in ('hover', 'grasp', 'carry', 'demo') if k in POSES]
    b = dict(POSES[random.choice(keys)])
    return dict(joint_11=random.uniform(-40, 40), joint_22=b['joint_22'] + random.uniform(-12, 12),
                joint_33=b['joint_33'] + random.uniform(-12, 12), joint_44=b['joint_44'] + random.uniform(-15, 15),
                joint_55=random.uniform(0, 40))


def pose_vec(p):
    v = np.zeros(len(DOF), dtype=np.float32)
    for k, name in enumerate(DOF):
        if name in p:
            v[k] = math.radians(p[name])
    return v


def new_stage():
    ctx = omni.usd.get_context(); ctx.new_stage()
    for _ in range(3):
        app.update()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def add_robot(stage, path, pos, yaw=0.0):
    prim = add_reference_to_stage(usd_path=args.usd, prim_path=path)
    x = UsdGeom.Xformable(prim); x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddRotateZOp().Set(float(yaw))
    x.AddRotateXOp().Set(90.0)
    finray = make_material(stage, path + '_FinRay', FINRAY_BLUE, roughness=0.45)
    for p in Usd.PrimRange(prim):
        if 'follower_07' in p.GetName() and p.IsA(UsdGeom.Xformable):
            bind(p, finray)
    return prim


def robot_bbox(stage, prim):
    for _ in range(2):
        app.update()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    bb = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return Gf.Vec3d(bb.GetMin()), Gf.Vec3d(bb.GetMax())


def lights(stage, dome=800, key=2500, key_rot=(-50, 25, 0), fill_pos=None, fill=15000):
    d = UsdLux.DomeLight.Define(stage, Sdf.Path('/World/DomeLight')); d.CreateIntensityAttr(dome)
    k = UsdLux.DistantLight.Define(stage, Sdf.Path('/World/KeyLight')); k.CreateIntensityAttr(key)
    UsdGeom.Xformable(k.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(*key_rot))
    if fill_pos is not None:
        f = UsdLux.SphereLight.Define(stage, Sdf.Path('/World/FillLight')); f.CreateIntensityAttr(fill); f.CreateRadiusAttr(0.2)
        UsdGeom.Xformable(f.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*fill_pos))


def render(world, arts_and_poses, eye, target, out, frames=None):
    set_camera_view(eye=[float(v) for v in eye], target=[float(v) for v in target])
    for _ in range(frames or args.frames):
        for art, q in arts_and_poses:
            art.set_joint_positions(q); art.set_joint_velocities(np.zeros_like(q)); art.set_joint_position_targets(q)
        world.step(render=True)
    capture_viewport_to_file(get_active_viewport(), out)
    for _ in range(25):
        app.update()
    print('CAPTURED', out, flush=True)


def finish_world(stage):
    world = World(stage_units_in_meters=1.0)
    world.reset()
    return world


def articulate(path_regex, n_expected=None):
    art = Articulation(path_regex); art.initialize()
    return art


def find_link(stage, robot_path, name):
    for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path)):
        if p.GetName() == name and p.IsA(UsdGeom.Xform):
            return str(p.GetPath())
    raise RuntimeError(name + ' not found')


def solve_poses(world, art, stage, robot_path, targets):
    """Grid-sample joint angles in the physics sim, read the gripper link pose, and keep
    the closest match per target. targets: {name: (xyz, min_finger_drop)}. Returns {name: pose dict}."""
    from isaacsim.core.prims import RigidPrim
    grip = RigidPrim(find_link(stage, robot_path, 'follower_06'))
    grip.initialize()

    def quat_rot(q, v):  # q = (w, x, y, z)
        w, x, y, z = [float(c) for c in q]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R @ np.asarray(v, dtype=float)

    # In the zero pose the gripper points straight up (+z). Find that axis in the link frame.
    q0 = pose_vec(dict(joint_55=10))
    art.set_joint_positions(q0[None]); art.set_joint_velocities(np.zeros((1, len(DOF)), dtype=np.float32))
    world.step(render=False)
    _, o0 = grip.get_world_poses(); o0 = np.array(o0[0])
    w, x, y, z = o0
    Rt = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
                   [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
                   [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]])
    local_up = Rt @ np.array([0.0, 0.0, 1.0])
    best = {k: (1e9, None) for k in targets}
    grid = [(a, b, c, d) for a in (-12, -6, 0, 6, 12) for b in range(-30, 91, 8) for c in range(-30, 131, 8) for d in range(-60, 121, 10)]
    for a, b, c, d in grid:
        q = pose_vec(dict(joint_11=a, joint_22=b, joint_33=c, joint_44=d, joint_55=10))
        art.set_joint_positions(q[None]); art.set_joint_velocities(np.zeros((1, len(DOF)), dtype=np.float32))
        world.step(render=False)
        pos, orn = grip.get_world_poses()
        g = np.array(pos[0]); down = -float(quat_rot(orn[0], local_up)[2])   # 1 = gripper pointing straight down
        for k, (xyz, min_down) in targets.items():
            err = float(np.linalg.norm(g - np.array(xyz))) + max(0.0, min_down - down) * 0.3
            if err < best[k][0]:
                best[k] = (err, dict(joint_11=a, joint_22=b, joint_33=c, joint_44=d, joint_55=10))
    for k, (e, pdict) in best.items():
        print(f'POSE {k}: err={e:.3f} {pdict}', flush=True)
    return {k: v[1] for k, v in best.items()}


# ----------------------------------------------------------------------------- scenes
def scene_parallel_arms():
    stage = new_stage()
    n, spacing = 10, 0.55
    floor = make_material(stage, '/World/Looks/DarkFloor', (0.08, 0.09, 0.11), roughness=0.35, metallic=0.2)
    line = make_material(stage, '/World/Looks/GridLine', (0.15, 0.45, 0.85), roughness=0.5)
    pad = make_material(stage, '/World/Looks/Pad', (0.16, 0.17, 0.2), roughness=0.5)
    prims = []
    for i in range(n):
        for j in range(n):
            x, y = (i - (n - 1) / 2) * spacing, (j - (n - 1) / 2) * spacing
            prims.append(add_robot(stage, f'/World/Arms/Arm_{i}_{j}', (x, y, 0.0)))
    bmin, bmax = robot_bbox(stage, prims[0])
    z0 = float(bmin[2]) - 0.001
    box(stage, '/World/Floor', (0, 0, z0 - 0.02), (n * spacing + 6, n * spacing + 6, 0.04), floor)
    # pads + glowing grid lines
    for i in range(n):
        for j in range(n):
            x, y = (i - (n - 1) / 2) * spacing, (j - (n - 1) / 2) * spacing
            box(stage, f'/World/Pads/Pad_{i}_{j}', (x + (bmin[0] + bmax[0]) / 2 - prims[0].GetAttribute('xformOp:translate').Get()[0],
                                                     y + (bmin[1] + bmax[1]) / 2 - prims[0].GetAttribute('xformOp:translate').Get()[1], z0 - 0.0005),
                (spacing - 0.06, spacing - 0.06, 0.001), pad, collide=False)
    for k in range(n + 1):
        c = (k - n / 2) * spacing
        box(stage, f'/World/Grid/X_{k}', (c, 0, z0 + 0.0002), (0.006, n * spacing + 0.4, 0.0004), line, collide=False)
        box(stage, f'/World/Grid/Y_{k}', (0, c, z0 + 0.0002), (n * spacing + 0.4, 0.006, 0.0004), line, collide=False)
    lights(stage, dome=350, key=1800, key_rot=(-55, 35, 0), fill_pos=(-2.5, -3.5, 2.5), fill=60000)
    world = finish_world(stage)
    art = articulate('/World/Arms/Arm_.*')
    q = np.stack([pose_vec(random_pose()) for _ in range(n * n)])
    ext = n * spacing
    render(world, [(art, q)], eye=(-ext * 0.62, -ext * 0.95, ext * 0.55), target=(0.0, 0.15, 0.15),
           out=os.path.join(args.out, 'parallel_arms_100.png'))
    render(world, [(art, q)], eye=(-ext * 0.25, -ext * 0.42, ext * 0.16), target=(0.0, 0.2, 0.1),
           out=os.path.join(args.out, 'parallel_arms_100_low.png'))


def scene_parallel_kitchens():
    stage = new_stage()
    n, spacing = 3, 2.6
    mats = kitchen_materials(stage)
    pj = os.path.join(args.out, 'poses.json')
    if os.path.exists(pj):
        POSES.update(json.load(open(pj)))
    prims = []
    for i in range(n):
        for j in range(n):
            x, y = (i - 1) * spacing, (j - 1) * spacing
            p = add_robot(stage, f'/World/Env_{i}_{j}/Robot', (x, y, 0.0))
            bmin, bmax = robot_bbox(stage, p)
            build_kitchen(stage, bmin, bmax, mats=mats, root=f'/World/Env_{i}_{j}/Kitchen', compact=True, pan_rigid=False)
            prims.append(p)
    # dark studio floor beneath all environments
    floor = make_material(stage, '/World/Looks/StudioFloor', (0.1, 0.11, 0.13), roughness=0.4)
    z0 = float(bmin[2]) - 0.861 - 0.03
    box(stage, '/World/StudioFloor', (0, 0, z0), (n * spacing + 8, n * spacing + 8, 0.02), floor, collide=False)
    lights(stage, dome=600, key=2200, key_rot=(-50, 30, 0))
    world = finish_world(stage)
    art = articulate('/World/Env_.*/Robot')
    names = [k for k in ('hover', 'grasp', 'carry', 'demo') if k in POSES]
    q = np.stack([pose_vec(POSES[names[k % len(names)]]) for k in range(n * n)])
    ext = n * spacing
    render(world, [(art, q)], eye=(-ext * 0.35, -ext * 0.9, ext * 0.62), target=(0.0, 0.3, -0.2),
           out=os.path.join(args.out, 'parallel_kitchens_3x3.png'))
    render(world, [(art, q)], eye=(-ext * 0.12, -ext * 0.75, ext * 0.3), target=(0.0, 0.2, -0.15),
           out=os.path.join(args.out, 'parallel_kitchens_3x3_low.png'))


def scene_single_kitchen():
    """hero, gripper close-up, grasp sequence, top view: one kitchen, several cameras/poses."""
    stage = new_stage()
    p = add_robot(stage, '/World/Robot', (0, 0, 0))
    bmin, bmax = robot_bbox(stage, p)
    mats = build_kitchen(stage, bmin, bmax, pan_rigid=False)  # static: the pose search sweeps the arm through it
    lights(stage, dome=650, key=2500, key_rot=(-50, 25, 0),
           fill_pos=((bmin[0] + bmax[0]) / 2 - 0.8, (bmin[1] + bmax[1]) / 2 - 1.2, bmin[2] + 0.9), fill=20000)
    world = finish_world(stage)
    art = articulate('/World/Robot')
    base = Gf.Vec3d((bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, float(bmin[2]))
    pan = np.array([base[0] + 0.02, base[1] - 0.18, base[2]])
    # pan handle centre: 8 cm from the pan centre along the handle (yaw -20 deg)
    handle = pan + np.array([0.08 * math.cos(math.radians(-20)), 0.08 * math.sin(math.radians(-20)), 0.0])
    solved = solve_poses(world, art, stage, '/World/Robot', {
        'hover': (handle + np.array([0, 0.0, 0.16]), 0.8),
        'grasp': (handle + np.array([0, 0.0, 0.075]), 0.8),
        'carry': (pan + np.array([-0.04, 0.08, 0.24]), 0.5),
    })
    POSES.update(solved)
    json.dump(POSES, open(os.path.join(args.out, 'poses.json'), 'w'), indent=1)
    hover_q = pose_vec(POSES['hover'])
    # hero
    render(world, [(art, pose_vec(POSES['demo'])[None])], eye=base + Gf.Vec3d(-0.62, -0.78, 0.38), target=base + Gf.Vec3d(0.02, -0.05, 0.2),
           out=os.path.join(args.out, 'hero_demo_pose.png'))
    render(world, [(art, hover_q[None])], eye=base + Gf.Vec3d(0.7, -0.75, 0.42), target=base + Gf.Vec3d(0.0, -0.08, 0.18),
           out=os.path.join(args.out, 'hero_hover_pose.png'))
    # gripper close-up over the pan
    panv = Gf.Vec3d(*pan)
    hv = Gf.Vec3d(*handle)
    render(world, [(art, pose_vec(POSES['grasp'])[None])], eye=hv + Gf.Vec3d(-0.26, -0.22, 0.14), target=hv + Gf.Vec3d(0.0, 0.0, 0.04),
           out=os.path.join(args.out, 'gripper_closeup_pan.png'))
    # grasp sequence; in the carry frame the pan hangs from the gripper
    from isaacsim.core.prims import RigidPrim
    grip = RigidPrim(find_link(stage, '/World/Robot', 'follower_06')); grip.initialize()
    pan_prim = stage.GetPrimAtPath('/World/Kitchen/MiniPan')
    pan_home = pan_prim.GetAttribute('xformOp:translate').Get()
    for k, name in enumerate(('hover', 'grasp', 'carry')):
        q = pose_vec(POSES[name])
        if name == 'carry':
            art.set_joint_positions(q[None]); world.step(render=False)
            g = np.array(grip.get_world_poses()[0][0])
            pan_prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(float(g[0]) - 0.075, float(g[1]), float(g[2]) - 0.07))
            pan_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3f(0, 0, 0))
        render(world, [(art, q[None])], eye=base + Gf.Vec3d(-0.55, -0.7, 0.35), target=base + Gf.Vec3d(0.02, -0.1, 0.16),
               out=os.path.join(args.out, f'grasp_sequence_{k+1:02d}_{name}.png'), frames=60)
    render(world, [(art, pose_vec(POSES['carry'])[None])], eye=base + Gf.Vec3d(0.55, -0.75, 0.4), target=base + Gf.Vec3d(0.0, -0.1, 0.18),
           out=os.path.join(args.out, 'hero_carry_pan.png'))
    pan_prim.GetAttribute('xformOp:translate').Set(pan_home); pan_prim.GetAttribute('xformOp:rotateXYZ').Set(Gf.Vec3f(0, 0, -20))
    # top view
    render(world, [(art, pose_vec(POSES['demo'])[None])], eye=base + Gf.Vec3d(0.0, -0.05, 1.6), target=base + Gf.Vec3d(0.0, -0.049, 0.0),
           out=os.path.join(args.out, 'top_view_workspace.png'))


SCENES = {'parallel_arms': scene_parallel_arms, 'parallel_kitchens': scene_parallel_kitchens,
          'hero': scene_single_kitchen}
selected = [s.strip() for s in args.only.split(',')] if args.only else list(SCENES)
for name in selected:
    t0 = time.time()
    try:
        SCENES[name]()
    except Exception as exc:  # noqa: BLE001
        import traceback; traceback.print_exc()
        print('SCENE FAILED', name, exc, flush=True)
    print(f'SCENE {name} done in {time.time()-t0:.0f}s', flush=True)
app.close()
