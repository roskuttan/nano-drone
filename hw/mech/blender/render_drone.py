"""render_drone.py - product renders of the nano drone in Blender Cycles.

Run headless (Blender 4.x/5.x):
    blender -b --factory-startup -P hw/mech/blender/render_drone.py -- [--samples 256] [--views hero,rear,top,under]
              [--res 2400x1600] [--cpu]

Input:  hw/output/mech/blender/nanodrone_scene.obj  (one object per body, mm, Z up; written from the Fusion model)
Output: hw/output/mech/blender/render_<view>.png and nanodrone_studio.blend (the full scene, open it in Blender)

Object names are "<component>__<body>"; materials are picked from them (enclosure, PCB, parts, motor bells and
bases, props, screws, battery). The Lite top is hidden; the Pro enclosure is shown.
"""
import math
import os
import sys

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(HW, "output", "mech", "blender")
OBJ = os.path.join(OUT, "nanodrone_scene.obj")
MM = 0.001
FLOOR_Z = -14.8 * MM          # lowest point of the bottom ducts

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def arg(name, default):
    return args[args.index(name) + 1] if name in args else default


SAMPLES = int(arg("--samples", "256"))
VIEWS = arg("--views", "hero,rear,top,under").split(",")
RES = [int(v) for v in arg("--res", "2400x1600").split("x")]
USE_CPU = "--cpu" in args


# ------------------------------------------------------------------ materials
def principled(name, color, rough=0.5, metal=0.0, coat=0.0, trans=0.0, ior=1.45, alpha=1.0, grain=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    p = nt.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*color, 1.0)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    p.inputs["IOR"].default_value = ior
    for key in ("Coat Weight", "Clearcoat"):
        if key in p.inputs:
            p.inputs[key].default_value = coat
            break
    for key in ("Transmission Weight", "Transmission"):
        if key in p.inputs:
            p.inputs[key].default_value = trans
            break
    p.inputs["Alpha"].default_value = alpha
    if grain > 0:                               # MJF powder texture: fine noise into bump and roughness
        tex = nt.nodes.new("ShaderNodeTexNoise")
        tex.inputs["Scale"].default_value = 9000.0
        tex.inputs["Detail"].default_value = 2.0
        bump = nt.nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = grain
        bump.inputs["Distance"].default_value = 0.00002
        nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
        nt.links.new(bump.outputs["Normal"], p.inputs["Normal"])
    return m


def materials():
    return {
        "shell": principled("MJF PA12 black", (0.018, 0.018, 0.02), rough=0.62, grain=0.35),
        "pcb": principled("Soldermask green", (0.02, 0.16, 0.06), rough=0.25, coat=0.3),
        "ic": principled("IC epoxy", (0.012, 0.012, 0.014), rough=0.42),
        "cap": principled("MLCC ceramic", (0.42, 0.33, 0.22), rough=0.45),
        "res": principled("Resistor", (0.02, 0.02, 0.022), rough=0.35),
        "pad": principled("ENIG gold", (0.95, 0.72, 0.36), rough=0.22, metal=1.0),
        "led": principled("LED", (0.9, 0.9, 0.85), rough=0.1, trans=0.6),
        "bell": principled("Anodised red", (0.55, 0.03, 0.03), rough=0.32, metal=1.0),
        "base": principled("Anodised dark", (0.06, 0.06, 0.065), rough=0.35, metal=1.0),
        "blades": principled("Prop smoke PC", (0.18, 0.2, 0.22), rough=0.12, trans=0.85, ior=1.58),
        "screw": principled("Steel", (0.62, 0.62, 0.64), rough=0.25, metal=1.0),
        "pack": principled("LiPo wrap", (0.006, 0.02, 0.09), rough=0.4, coat=0.3),
        "lead": principled("Silicone wire", (0.6, 0.03, 0.02), rough=0.55),
        "plug": principled("BT2.0 nylon", (0.8, 0.35, 0.01), rough=0.45),
        "floor": principled("Studio sweep", (0.72, 0.73, 0.75), rough=0.6),
        "gloss_floor": principled("Black gloss floor", (0.09, 0.09, 0.10), rough=0.22, coat=0.6),
    }


def pick(name, mats):
    comp, _, body = name.partition("__")
    if comp.startswith("Enclosure"):
        return mats["shell"]
    if body == "PCB":
        return mats["pcb"]
    if body.startswith("part_"):
        ref = body[5:]
        if ref.startswith(("U", "Y")):
            return mats["ic"]
        if ref.startswith(("C", "L", "FB")):
            return mats["cap"]
        if ref.startswith("R"):
            return mats["res"]
        if ref.startswith("J"):
            return mats["pad"]
        if ref.startswith("D"):
            return mats["led"]
        return mats["ic"]
    key = body.split("_")[0]
    return mats.get(key, mats["shell"])


# ------------------------------------------------------------------ scene
def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_drone(mats):
    before = set(bpy.data.objects)
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=OBJ, forward_axis="Y", up_axis="Z", global_scale=MM)
    else:
        bpy.ops.import_scene.obj(filepath=OBJ, axis_forward="Y", axis_up="Z", global_scale=MM)
    objs = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    for o in objs:
        o.data.materials.clear()
        o.data.materials.append(pick(o.name, mats))
        for poly in o.data.polygons:
            poly.use_smooth = True
        if o.name.startswith("Enclosure_Top_Lite"):
            o.hide_render = True
            o.hide_viewport = True
    return objs


def sweep(mat):
    """Seamless studio backdrop: floor curving up into a wall behind the drone."""
    import bmesh
    me = bpy.data.meshes.new("sweep")
    bm = bmesh.new()
    width, depth, height, r = 1.2, 0.6, 0.6, 0.18
    prof = [(-depth, 0.0)]
    for i in range(17):
        a = -math.pi / 2 + (math.pi / 2) * i / 16
        prof.append((r * math.cos(a), r + r * math.sin(a)))
    prof.append((r, height))
    rows = []
    for x in (-width / 2, width / 2):
        rows.append([bm.verts.new((x, -y, z)) for y, z in prof])
    for i in range(len(prof) - 1):
        bm.faces.new((rows[0][i], rows[1][i], rows[1][i + 1], rows[0][i + 1]))
    bm.to_mesh(me)
    ob = bpy.data.objects.new("Backdrop", me)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = (0.0, 0.0, FLOOR_Z)
    me.materials.append(mat)
    for p in me.polygons:
        p.use_smooth = True
    return ob


def area_light(name, loc, target, size, power, color=(1.0, 1.0, 1.0)):
    ld = bpy.data.lights.new(name, "AREA")
    ld.shape = "RECTANGLE"
    ld.size, ld.size_y = size, size * 0.6
    ld.energy = power
    ld.color = color
    ob = bpy.data.objects.new(name, ld)
    bpy.context.scene.collection.objects.link(ob)
    ob.location = loc
    aim(ob, target)
    return ob


def aim(ob, target):
    d = Vector(target) - ob.location
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def camera():
    cd = bpy.data.cameras.new("Cam")
    cd.lens = 100.0
    cd.sensor_width = 36.0
    cd.dof.use_dof = True
    cd.dof.aperture_fstop = 64.0
    ob = bpy.data.objects.new("Cam", cd)
    bpy.context.scene.collection.objects.link(ob)
    bpy.context.scene.camera = ob
    return ob


VIEW_DIRS = {                 # camera direction from the drone centre (mm frame), distance in metres
    "hero": ((150.0, 95.0, 42.0), 0.29),
    "rear": ((-130.0, -140.0, 95.0), 0.30),
    "top": ((10.0, -40.0, 220.0), 0.31),
    "under": ((110.0, 140.0, -100.0), 0.30),
}


def setup_render():
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.render.resolution_x, sc.render.resolution_y = RES
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False
    sc.cycles.samples = SAMPLES
    sc.view_settings.exposure = -0.9
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 12
    sc.cycles.transmission_bounces = 12
    try:
        sc.view_settings.view_transform = "AgX"
        sc.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        sc.view_settings.view_transform = "Filmic"
    device = "CPU"
    if not USE_CPU:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for kind in ("OPTIX", "CUDA"):
            try:
                prefs.compute_device_type = kind
                prefs.get_devices()
                gpus = [d for d in prefs.devices if d.type == kind]
                if gpus:
                    for d in prefs.devices:
                        d.use = d.type == kind
                    device = kind
                    break
            except TypeError:
                continue
    sc.cycles.device = "GPU" if device != "CPU" else "CPU"
    if device == "OPTIX":                     # the default OIDN-on-CPU denoiser costs ~20 s per 1080p frame here
        sc.cycles.denoiser = "OPTIX"
    world = bpy.data.worlds.new("Studio")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.55, 0.57, 0.6, 1.0)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.12
    sc.world = world
    return device


def main():
    clear()
    mats = materials()
    objs = import_drone(mats)
    centre = Vector((0.0, 0.0, 0.0))
    back = sweep(mats["floor"])
    area_light("Key", (0.16, 0.10, 0.22), centre, 0.35, 4.0, (1.0, 0.97, 0.93))
    area_light("Fill", (-0.22, 0.12, 0.08), centre, 0.45, 1.2, (0.92, 0.96, 1.0))
    area_light("Rim", (-0.06, -0.24, 0.16), centre, 0.25, 3.0)
    area_light("Top", (0.0, 0.0, 0.32), centre, 0.5, 1.5)
    cam = camera()
    device = setup_render()
    os.makedirs(OUT, exist_ok=True)
    print("render: %d objects, device %s, %d samples, %dx%d" % (len(objs), device, SAMPLES, RES[0], RES[1]))
    for v in VIEWS:
        d, dist = VIEW_DIRS[v]
        dv = Vector(d).normalized()
        cam.location = centre + dv * dist
        aim(cam, centre + Vector((0.0, 0.0, -1.0 * MM)))
        cam.data.dof.focus_distance = dist
        back.hide_render = v == "under"            # floating underside shot against the world colour
        # turn the sweep so its floor runs toward the camera and its wall stands behind the drone
        az = math.atan2(dv.y, dv.x)
        back.rotation_euler = (0.0, 0.0, az - math.pi / 2)
        back.location = (-math.cos(az) * 0.12, -math.sin(az) * 0.12, FLOOR_Z)
        bpy.context.scene.render.filepath = os.path.join(OUT, "render_%s.png" % v)
        bpy.ops.render.render(write_still=True)
        print("wrote", bpy.context.scene.render.filepath)
    back.hide_render = False
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "nanodrone_studio.blend"))
    print("saved", os.path.join(OUT, "nanodrone_studio.blend"))


if __name__ == "__main__":
    main()
