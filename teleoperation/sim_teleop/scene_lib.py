"""Scene building helpers for the Isaac Sim viewer and the gallery renderer (import after SimulationApp)."""
import math

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf


# ----------------------------------------------------------------------------- helpers
def make_material(stage, path, color, roughness=0.5, metallic=0.0, opacity=1.0):
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + '/Shader'))
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants)


def xform(prim, pos, rot_xyz=(0, 0, 0), scale=(1, 1, 1)):
    x = UsdGeom.Xformable(prim); x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_xyz))
    x.AddScaleOp().Set(Gf.Vec3f(*scale))


def box(stage, path, pos, size, mat, rot=(0, 0, 0), collide=True, rigid=False, mass=None):
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path)); cube.CreateSizeAttr(1.0)
    xform(cube.GetPrim(), pos, rot, size)
    bind(cube.GetPrim(), mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        if mass:
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(mass)
    return cube.GetPrim()


def cylinder(stage, path, pos, radius, height, mat, rot=(0, 0, 0), collide=True):
    cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(path))
    cyl.CreateRadiusAttr(radius); cyl.CreateHeightAttr(height); cyl.CreateAxisAttr('Z')
    xform(cyl.GetPrim(), pos, rot)
    bind(cyl.GetPrim(), mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()


def mini_frying_pan(stage, path, pos, yaw_deg, mats, scale=1.0, rigid=True):
    """Miniature pan sized for the Fin-Ray gripper: 90 mm pan, 85 mm handle."""
    root = UsdGeom.Xform.Define(stage, Sdf.Path(path)); prim = root.GetPrim()
    xform(prim, pos, (0, 0, yaw_deg), (scale, scale, scale))
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.06)
    r, wall, depth, base_t = 0.045, 0.003, 0.016, 0.003
    cylinder(stage, path + '/base', (0, 0, base_t / 2), r, base_t, mats['pan_dark'])
    n = 28
    for i in range(n):
        a = 2 * math.pi * i / n
        seg = 2 * math.pi * r / n * 1.08
        box(stage, f'{path}/wall_{i}', (math.cos(a) * (r - wall / 2), math.sin(a) * (r - wall / 2), base_t + depth / 2),
            (wall, seg, depth), mats['pan_dark'], rot=(0, 0, math.degrees(a)))
    cylinder(stage, path + '/surface', (0, 0, base_t + 0.0005), r - wall, 0.001, mats['pan_light'], collide=False)
    hl, hw, ht = 0.085, 0.014, 0.008
    box(stage, path + '/handle', (r + hl / 2 - 0.004, 0, base_t + depth * 0.75), (hl, hw, ht), mats['handle'])
    box(stage, path + '/handle_neck', (r + 0.004, 0, base_t + depth * 0.75), (0.012, 0.02, ht), mats['pan_dark'])
    return prim


def kitchen_materials(stage, looks='/World/Looks'):
    mats = {
        'counter': make_material(stage, looks + '/Counter', (0.86, 0.85, 0.82), roughness=0.25),
        'edge': make_material(stage, looks + '/CounterEdge', (0.55, 0.53, 0.5), roughness=0.4),
        'cabinet': make_material(stage, looks + '/Cabinet', (0.22, 0.24, 0.27), roughness=0.6),
        'handle_metal': make_material(stage, looks + '/HandleMetal', (0.75, 0.76, 0.78), roughness=0.3, metallic=0.9),
        'tile': make_material(stage, looks + '/Tile', (0.93, 0.94, 0.95), roughness=0.2),
        'grout': make_material(stage, looks + '/Grout', (0.78, 0.79, 0.8), roughness=0.8),
        'floor': make_material(stage, looks + '/Floor', (0.42, 0.33, 0.25), roughness=0.55),
        'board': make_material(stage, looks + '/Board', (0.72, 0.55, 0.36), roughness=0.6),
        'pan_dark': make_material(stage, looks + '/PanDark', (0.12, 0.12, 0.13), roughness=0.35, metallic=0.6),
        'pan_light': make_material(stage, looks + '/PanLight', (0.25, 0.25, 0.27), roughness=0.3, metallic=0.7),
        'handle': make_material(stage, looks + '/PanHandle', (0.05, 0.05, 0.06), roughness=0.7),
        'steel': make_material(stage, looks + '/Steel', (0.7, 0.71, 0.73), roughness=0.25, metallic=1.0),
        'towel': make_material(stage, looks + '/Towel', (0.2, 0.45, 0.75), roughness=0.9),
    }
    mats['ceramic'] = make_material(stage, looks + '/Ceramic', (0.95, 0.95, 0.93), roughness=0.2)
    mats['ceramic_blue'] = make_material(stage, looks + '/CeramicBlue', (0.25, 0.42, 0.7), roughness=0.25)
    mats['glass'] = make_material(stage, looks + '/Glass', (0.85, 0.9, 0.95), roughness=0.05, opacity=0.35)
    mats['spice_red'] = make_material(stage, looks + '/SpiceRed', (0.7, 0.15, 0.1), roughness=0.6)
    mats['spice_yellow'] = make_material(stage, looks + '/SpiceYellow', (0.85, 0.65, 0.15), roughness=0.6)
    mats['spice_green'] = make_material(stage, looks + '/SpiceGreen', (0.3, 0.5, 0.2), roughness=0.6)
    mats['cap'] = make_material(stage, looks + '/Cap', (0.1, 0.1, 0.1), roughness=0.5)
    mats['wood_dark'] = make_material(stage, looks + '/WoodDark', (0.35, 0.22, 0.12), roughness=0.6)
    return mats


def build_kitchen(stage, bmin, bmax, mats=None, root='/World/Kitchen', with_props=False, props_root=None, compact=False, pan_rigid=True):
    """Countertop under the robot (bbox bmin/bmax), cabinet, backsplash, floor, and props."""
    if mats is None:
        mats = kitchen_materials(stage)
    cx, cy = (bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2
    top_z = float(bmin[2]) - 0.001            # counter surface just under the robot base
    W, D, T = 1.6, 0.8, 0.04                  # counter width (x), depth (y), thickness
    ccy = cy + 0.12                            # robot sits toward the front edge
    box(stage, root + '/CounterTop', (cx, ccy, top_z - T / 2), (W, D, T), mats['counter'])
    box(stage, root + '/CounterEdge', (cx, ccy - D / 2 + 0.01, top_z - T / 2), (W, 0.02, T + 0.004), mats['edge'])
    floor_z = top_z - 0.86
    box(stage, root + '/Cabinet', (cx, ccy + 0.02, (top_z - T + floor_z) / 2), (W - 0.02, D - 0.06, top_z - T - floor_z), mats['cabinet'])
    for i, dx in enumerate((-0.4, 0.0, 0.4)):
        box(stage, root + f'/Drawer_{i}', (cx + dx, ccy - D / 2 + 0.022, top_z - 0.16), (0.36, 0.01, 0.16), mats['cabinet'], collide=False)
        box(stage, root + f'/DrawerHandle_{i}', (cx + dx, ccy - D / 2 + 0.012, top_z - 0.12), (0.14, 0.012, 0.012), mats['handle_metal'], collide=False)
        box(stage, root + f'/Door_{i}', (cx + dx, ccy - D / 2 + 0.022, (top_z - 0.26 + floor_z) / 2), (0.36, 0.01, top_z - 0.26 - floor_z - 0.02), mats['cabinet'], collide=False)
    fw = 2.4 if compact else 6.0
    box(stage, root + '/Floor', (cx, ccy, floor_z - 0.01), (fw, fw, 0.02), mats['floor'])
    wy = ccy + D / 2 + 0.02
    ww, wh = (2.0, 1.25) if compact else (4.0, 2.6)
    box(stage, root + '/Wall', (cx, wy + 0.05, top_z - 0.86 + wh / 2 if compact else top_z + 0.6), (ww, 0.1, wh + 0.86 if compact else wh), mats['grout'], collide=False)
    tile, gap = 0.15, 0.004
    tw = ww - 0.4
    nx, nz = int(tw / tile), int((1.0 if compact else 1.2) / tile)
    for ix in range(nx):
        for iz in range(nz):
            box(stage, root + f'/Tile_{ix}_{iz}',
                (cx - tw / 2 + tile / 2 + ix * tile, wy, top_z + 0.02 + tile / 2 + iz * tile),
                (tile - gap, 0.004, tile - gap), mats['tile'], collide=False)
    box(stage, root + '/Shelf', (cx, wy - 0.12, top_z + 0.62), (1.4, 0.24, 0.025), mats['board'], collide=False)
    box(stage, root + '/Rail', (cx, wy - 0.02, top_z + 0.42), (1.2, 0.012, 0.012), mats['steel'], collide=False)
    box(stage, root + '/CuttingBoard', (cx - 0.5, ccy + 0.05, top_z + 0.008), (0.34, 0.24, 0.016), mats['board'])
    box(stage, root + '/Towel', (cx + 0.55, ccy - 0.05, top_z + 0.004), (0.26, 0.18, 0.008), mats['towel'], collide=False)
    box(stage, root + '/Hob', (cx + 0.32, ccy + 0.12, top_z + 0.003), (0.32, 0.28, 0.006), mats['pan_dark'], collide=False)
    cylinder(stage, root + '/HobRing', (cx + 0.32, ccy + 0.12, top_z + 0.0065), 0.07, 0.001, mats['edge'], collide=False)
    # mini frying pan within reach in front of the robot (rigid body, can be pushed/grasped)
    mini_frying_pan(stage, root + '/MiniPan', (cx + 0.02, ccy - 0.3, top_z + 0.0005), -20, mats, rigid=pan_rigid)
    # a second pan resting on the hob (static)
    mini_frying_pan(stage, root + '/MiniPanOnHob', (cx + 0.32, ccy + 0.12, top_z + 0.0065), 35, mats, rigid=False)
    # procedural props: mugs, spice bottles, plate, wooden spoon, knife block

    def mug(path, pos, mat, yaw=0):
        cylinder(stage, path + '/body', (pos[0], pos[1], pos[2] + 0.045), 0.04, 0.09, mat)
        cylinder(stage, path + '/inner', (pos[0], pos[1], pos[2] + 0.05), 0.034, 0.085, mats['cabinet'], collide=False)
        a = math.radians(yaw)
        box(stage, path + '/handle', (pos[0] + math.cos(a) * 0.05, pos[1] + math.sin(a) * 0.05, pos[2] + 0.05), (0.012, 0.035, 0.05), mat, rot=(0, 0, yaw), collide=False)

    def bottle(path, pos, mat, r=0.018, h=0.1):
        cylinder(stage, path + '/body', (pos[0], pos[1], pos[2] + h / 2), r, h, mats['glass'], collide=False)
        cylinder(stage, path + '/content', (pos[0], pos[1], pos[2] + h * 0.35), r * 0.9, h * 0.7, mat, collide=False)
        cylinder(stage, path + '/cap', (pos[0], pos[1], pos[2] + h + 0.008), r * 0.8, 0.016, mats['cap'], collide=False)

    mug(root + '/MugA', (cx - 0.62, ccy + 0.2, top_z), mats['ceramic'], yaw=40)
    mug(root + '/MugB', (cx - 0.72, ccy + 0.28, top_z), mats['ceramic_blue'], yaw=-20)
    for i, (m, dx) in enumerate(((mats['spice_red'], 0.0), (mats['spice_yellow'], 0.05), (mats['spice_green'], 0.1))):
        bottle(root + f'/Spice_{i}', (cx + 0.55 + dx, ccy + 0.3, top_z), m)
    cylinder(stage, root + '/Plate', (cx - 0.25, ccy + 0.26, top_z + 0.006), 0.11, 0.012, mats['ceramic'], collide=False)
    cylinder(stage, root + '/PlateRim', (cx - 0.25, ccy + 0.26, top_z + 0.013), 0.095, 0.003, mats['edge'], collide=False)
    box(stage, root + '/Spoon', (cx - 0.42, ccy + 0.02, top_z + 0.02), (0.02, 0.2, 0.008), mats['wood_dark'], rot=(0, 0, 15), collide=False)
    box(stage, root + '/KnifeBlock', (cx + 0.68, ccy + 0.3, top_z + 0.09), (0.09, 0.14, 0.18), mats['wood_dark'], rot=(0, 0, -15), collide=False)
    for i in range(3):
        box(stage, root + f'/Knife_{i}', (cx + 0.66 + i * 0.02, ccy + 0.27 + i * 0.005, top_z + 0.2), (0.003, 0.02, 0.04), mats['steel'], rot=(0, 0, -15), collide=False)
    # shelf items
    mug(root + '/ShelfMug', (cx - 0.45, wy - 0.12, top_z + 0.635), mats['ceramic_blue'], yaw=10)
    for i in range(4):
        cylinder(stage, root + f'/Jar_{i}', (cx + 0.1 + i * 0.12, wy - 0.12, top_z + 0.635 + 0.06), 0.04, 0.12, mats['glass'], collide=False)
        cylinder(stage, root + f'/JarLid_{i}', (cx + 0.1 + i * 0.12, wy - 0.12, top_z + 0.635 + 0.125), 0.041, 0.01, mats['cap'], collide=False)
    aroot = None
    if with_props:
        try:
            aroot = props_root or get_assets_root_path()
        except Exception:  # noqa: BLE001
            aroot = None
    if aroot:
        props = [
            (root + '/Mug', aroot + '/Isaac/Props/Mugs/SM_Mug_A2.usd', (cx - 0.62, ccy + 0.22, top_z), 40, 1.0),
            (root + '/Bowl', aroot + '/Isaac/Props/YCB/Axis_Aligned/024_bowl.usd', (cx - 0.3, ccy + 0.25, top_z), 0, 1.0),
            (root + '/Mustard', aroot + '/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd', (cx + 0.6, ccy + 0.25, top_z), 15, 1.0),
            (root + '/Pitcher', aroot + '/Isaac/Props/YCB/Axis_Aligned/019_pitcher_base.usd', (cx + 0.68, ccy + 0.12, top_z), -30, 1.0),
            (root + '/Banana', aroot + '/Isaac/Props/YCB/Axis_Aligned/011_banana.usd', (cx - 0.5, ccy + 0.06, top_z + 0.016), 25, 1.0),
            (root + '/Teapot', aroot + '/Isaac/Props/Teapot/utah_teapot.usdc', (cx - 0.45, wy - 0.12, top_z + 0.635), 0, 1.0),
        ]
        for path, url, pos, yaw, sc in props:
            try:
                prim = add_reference_to_stage(usd_path=url, prim_path=path)
                xform(prim, pos, (0, 0, yaw), (sc, sc, sc))
                print('PROP', path)
            except Exception as exc:  # noqa: BLE001
                print('PROP FAILED', path, exc)
    return mats


