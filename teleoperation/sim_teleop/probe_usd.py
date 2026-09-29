"""Headless probe: load USD, print DOFs, world bbox, and capture a PNG of the viewport."""
import sys
from isaacsim import SimulationApp
usd = sys.argv[1] if len(sys.argv) > 1 else '/home/jeonsoengbin/Desktop/urdf_v3/my_manipulator/usd_isaac/robot_isaac/robot_isaac.usda'
out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/claude-1000/probe.png'
app = SimulationApp({"headless": True, "width": 1280, "height": 720})
import omni.usd, omni.kit.app
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
ctx = omni.usd.get_context(); ctx.open_stage(usd)
for _ in range(20): app.update()
stage = ctx.get_stage(); root = stage.GetDefaultPrim()
print("DEFAULT PRIM:", root.GetPath())
meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
print("MESH COUNT:", len(meshes))
for p in meshes[:12]: print("  MESH", p.GetPath(), "instance" if p.IsInstanceProxy() else "")
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
bb = cache.ComputeWorldBound(root).ComputeAlignedRange()
print("WORLD BBOX min", bb.GetMin(), "max", bb.GetMax())
world = World(stage_units_in_meters=1.0); world.reset()
for _ in range(5): app.update()
c = (Gf.Vec3d(bb.GetMin()) + Gf.Vec3d(bb.GetMax())) / 2; size = (Gf.Vec3d(bb.GetMax()) - Gf.Vec3d(bb.GetMin())).GetLength()
eye = c + Gf.Vec3d(1, 1, 0.6) * max(size, 0.3) * 1.2
set_camera_view(eye=list(eye), target=list(c))
for _ in range(30): app.update()
vp = get_active_viewport(); capture_viewport_to_file(vp, out)
for _ in range(30): app.update()
print("CAPTURED", out)
app.close()
