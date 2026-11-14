"""product_video.py - the 3D shots of the nano drone product video (Blender Cycles, 1080p24).

    blender -b --factory-startup -P hw/mech/blender/product_video.py -- [--start N] [--end N] [--only N,M]
            [--samples 32] [--res 1920x1080] [--look dark|light] [--aspect landscape|portrait]

One timeline, frames 1-744, cut into six shots (the title cards and the outro are added by edit_product_video.py):

  | Shot | Frames  | Content |
  |---|---|---|
  | 1 Reveal   | 1-120   | low, slow push-in; props spin up |
  | 2 Hero     | 121-264 | turntable with a crane move |
  | 3 Top      | 265-384 | high, slow turn: the four ducts |
  | 4 Explode  | 385-504 | top shell and battery lift off the board, camera pushes in on the electronics |
  | 5 Macro    | 505-600 | close-up of a bottom duct with the prop running |
  | 6 Take-off | 601-744 | the drone lifts, leans forward and the camera follows |

Frames go to hw/output/mech/blender/product_dark/ (or product/ for --look light; "_portrait" appended for the
9:16 Instagram Story version, 1080x1920, same horizontal framing with the cameras a little closer) as frame_####.png (existing frames are skipped, so the render can be
resumed). Materials, lights, backdrop and the OptiX setup come from render_drone.py; keyframe helpers from
animate_drone.py.
"""
import math
import os
import sys
import time

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_drone as rd  # noqa: E402
from animate_drone import MOTOR, SPIN_DIR, key  # noqa: E402

LOOK = rd.arg("--look", "dark")         # "dark": black void, glossy floor, rim lights; "light": grey studio sweep
PORTRAIT = rd.arg("--aspect", "landscape") == "portrait"
OUT = os.path.join(rd.OUT, ("product" if LOOK == "light" else "product_dark") + ("_portrait" if PORTRAIT else ""))
FPS, END = 24, 744
MM = rd.MM
SHOTS = {"reveal": (1, 120), "hero": (121, 264), "top": (265, 384), "explode": (385, 504),
         "macro": (505, 600), "takeoff": (601, 744)}
REV_PER_FRAME = 0.36
SAMPLES = int(rd.arg("--samples", "32"))
RES = [int(v) for v in rd.arg("--res", "1080x1920" if PORTRAIT else "1920x1080").split("x")]
# portrait keeps the horizontal field of view (sensor_fit HORIZONTAL) and pulls the camera in a little where
# the drone does not already fill the width
ZOOM = {"reveal": 0.88, "hero": 0.9, "top": 0.9, "explode": 0.95, "macro": 1.0, "takeoff": 0.9}
START, STOP = int(rd.arg("--start", "1")), int(rd.arg("--end", str(END)))
ONLY = [int(v) for v in rd.arg("--only", "").split(",") if v]
AZ = math.atan2(95.0, 150.0)          # hero azimuth, the backdrop faces it


def cam_pos(elev_deg, dist, az=AZ, centre=(0.0, 0.0, 0.0)):
    e = math.radians(elev_deg)
    return (centre[0] + math.cos(az) * math.cos(e) * dist, centre[1] + math.sin(az) * math.cos(e) * dist,
            centre[2] + math.sin(e) * dist)


def hold(ob, path, frame, value):
    """Key that holds its value until the next key (a cut): CONSTANT interpolation."""
    key(ob, path, frame, value, "CONSTANT")


def dark_sweep(mat):
    """Big seamless sweep for the dark set: 2 m of floor toward the camera, a 0.35 m curve, a 1.5 m wall."""
    import bmesh
    me = bpy.data.meshes.new("dark_sweep")
    bm = bmesh.new()
    width, depth, height, r = 5.0, 2.0, 1.5, 0.35
    prof = [(-depth, 0.0)]
    for i in range(25):
        a = -math.pi / 2 + (math.pi / 2) * i / 24
        prof.append((r * math.cos(a), r + r * math.sin(a)))
    prof.append((r, height))
    rows = [[bm.verts.new((x, -y, z)) for y, z in prof] for x in (-width / 2, width / 2)]
    for i in range(len(prof) - 1):
        bm.faces.new((rows[0][i], rows[1][i], rows[1][i + 1], rows[0][i + 1]))
    bm.to_mesh(me)
    me.materials.append(mat)
    for p in me.polygons:
        p.use_smooth = True
    ob = bpy.data.objects.new("Sweep", me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def dark_studio(mats, objs, centre):
    """Black void, a glossy dark floor that mirrors the drone, a pool of light under it, two long cool rim
    strips behind, and a warm key on a rig that drifts inside every shot so the highlights travel.
    Light linking keeps the panels off the floor: it is lit by the Pool spot only, so it shows a soft
    gradient and the drone's reflection instead of big white rectangles."""
    wall_dir = Vector((-math.cos(AZ), -math.sin(AZ), 0.0))
    floor = dark_sweep(mats["gloss_floor"])
    floor.rotation_euler = (0.0, 0.0, AZ - math.pi / 2)          # floor runs toward the camera, wall behind
    floor.location = wall_dir * 0.45 + Vector((0.0, 0.0, rd.FLOOR_Z - 0.00005))
    set_coll = bpy.data.collections.new("SetReceivers")
    bpy.context.scene.collection.children.link(set_coll)
    set_coll.objects.link(floor)
    receivers = bpy.data.collections.new("DroneReceivers")
    bpy.context.scene.collection.children.link(receivers)
    for o in objs:
        receivers.objects.link(o)
    shell = mats["shell"].node_tree
    shell.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.38      # satin: crisp rim arcs
    for n in shell.nodes:
        if n.type == "BUMP":
            n.inputs["Strength"].default_value = 0.18
    # a soft halo on the sweep's wall behind the drone: separates the black ducts from the void and puts a
    # faint glow into the floor reflection; the sweep is lit by this and the Pool only
    halo = bpy.data.lights.new("Halo", "SPOT")
    halo.energy, halo.spot_size, halo.spot_blend = 14.0, math.radians(32.0), 1.0
    halo.color = (0.85, 0.9, 1.0)
    halo_ob = bpy.data.objects.new("Halo", halo)
    bpy.context.scene.collection.objects.link(halo_ob)
    halo_ob.location = (0.0, 0.0, 0.30)
    rd.aim(halo_ob, wall_dir * 0.75 + Vector((0.0, 0.0, 0.10)))
    halo_ob.light_linking.receiver_collection = set_coll
    pack = mats["pack"].node_tree.nodes["Principled BSDF"]
    pack.inputs["Base Color"].default_value = (0.012, 0.014, 0.02, 1.0)
    pack.inputs["Roughness"].default_value = 0.5                  # satin black wrap, not a mirror
    for k in ("Coat Weight", "Clearcoat"):
        if k in pack.inputs:
            pack.inputs[k].default_value = 0.0
    mats["plug"].node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.35, 0.12, 0.01, 1.0)
    rig = bpy.data.objects.new("LightRig", None)
    bpy.context.scene.collection.objects.link(rig)
    rig.location = centre
    # strip lights, not panels: a glossy black product shows its lights as reflections, so long thin strips
    # draw thin bright arcs along the duct lips and the battery edges instead of filling every face with grey
    def strip(name, loc, length, power, color):
        l = rd.area_light(name, loc, centre, length, power, color)
        l.data.size_y = 0.08
        return l
    panels = [strip("Key", (0.22, 0.04, 0.24), 0.6, 5.0, (1.0, 0.94, 0.86)),
              strip("RimL", (-0.22, -0.14, 0.18), 0.6, 14.0, (0.80, 0.88, 1.0)),
              strip("RimR", (-0.04, -0.27, 0.19), 0.6, 14.0, (0.80, 0.88, 1.0)),
              rd.area_light("Fill", (-0.24, 0.16, 0.06), centre, 0.5, 0.3, (0.9, 0.94, 1.0))]
    top = strip("Top", (0.02, 0.0, 0.30), 0.7, 1.5, (1.0, 1.0, 1.0))
    for l in panels:
        l.parent = rig
    for l in panels + [top]:
        l.light_linking.receiver_collection = receivers
    pool = bpy.data.lights.new("Pool", "SPOT")
    pool.energy, pool.spot_size, pool.spot_blend = 10.0, math.radians(55.0), 1.0
    pool.color = (1.0, 0.97, 0.92)
    pool.shadow_soft_size = 0.08
    pool_ob = bpy.data.objects.new("Pool", pool)
    bpy.context.scene.collection.objects.link(pool_ob)
    pool_ob.location = (0.0, 0.0, 0.42)
    pool_ob.visible_glossy = False                 # no hotspot of the spot in the floor
    pool_ob.light_linking.receiver_collection = set_coll
    rd.aim(pool_ob, (0.0, 0.0, rd.FLOOR_Z))
    for s0, s1 in SHOTS.values():                  # the rig swings 28 deg inside every shot
        key(rig, "rotation_euler", s0, (0.0, 0.0, math.radians(-14.0)), "LINEAR")
        key(rig, "rotation_euler", s1, (0.0, 0.0, math.radians(14.0)), "LINEAR")


def build():
    rd.clear()
    mats = rd.materials()
    objs = rd.import_drone(mats)
    sc = bpy.context.scene
    sc.render.fps = FPS
    sc.frame_start, sc.frame_end = 1, END

    drone = bpy.data.objects.new("Drone", None)
    sc.collection.objects.link(drone)
    top_group = bpy.data.objects.new("TopGroup", None)      # the parts that lift off in the explode shot
    sc.collection.objects.link(top_group)
    top_group.parent = drone
    focus = bpy.data.objects.new("Focus", None)
    sc.collection.objects.link(focus)
    focus.parent = drone

    for o in objs:
        o.parent = drone
        if o.name.startswith("Enclosure_Top__") or o.name.split("__")[-1].split("_")[0] in ("pack", "lead", "plug"):
            o.parent = top_group
        if o.name.startswith("Visual__blades_"):
            m = o.name.split("_")[-1]
            c = Vector((MOTOR[m][0] * MM, MOTOR[m][1] * MM, 0.0))
            o.data.transform(Matrix.Translation(-c / o.scale.x))
            o.location = c
            s = SPIN_DIR[m] * 2 * math.pi
            key(o, "rotation_euler", 1, (0.0, 0.0, 0.0), "LINEAR")
            key(o, "rotation_euler", 72, (0.0, 0.0, s * 1.5), "LINEAR")              # spin-up over 3 s
            key(o, "rotation_euler", END, (0.0, 0.0, s * (1.5 + REV_PER_FRAME * (END - 72))), "LINEAR")

    rev = 2 * math.pi
    # ---- drone turntable / flight (one continuous rotation, held still in the explode and macro shots)
    key(drone, "rotation_euler", 1, (0.0, 0.0, 0.0))
    key(drone, "rotation_euler", 120, (0.0, 0.0, rev * 0.12))
    key(drone, "rotation_euler", 264, (0.0, 0.0, rev * 0.92))
    key(drone, "rotation_euler", 384, (0.0, 0.0, rev * 1.25))
    hold(drone, "rotation_euler", 600, (0.0, 0.0, rev * 1.25))
    key(drone, "rotation_euler", 601, (0.0, 0.0, rev * 1.25))
    key(drone, "rotation_euler", END, (math.radians(-6.0), 0.0, rev * 1.55))
    key(drone, "location", 640, (0.0, 0.0, 0.0))
    key(drone, "location", END, (0.0, 0.0, 48.0 * MM))
    # ---- explode: shell + battery rise 30 mm, hold, snap back at the cut
    key(top_group, "location", 400, (0.0, 0.0, 0.0))
    key(top_group, "location", 444, (0.0, 0.0, 30.0 * MM))
    hold(top_group, "location", 504, (0.0, 0.0, 30.0 * MM))
    key(top_group, "location", 505, (0.0, 0.0, 0.0))

    centre = Vector((0.0, 0.0, 0.0))
    if LOOK == "light":
        back = rd.sweep(mats["floor"])
        rd.area_light("Key", (0.16, 0.10, 0.22), centre, 0.35, 4.0, (1.0, 0.97, 0.93))
        rd.area_light("Fill", (-0.22, 0.12, 0.08), centre, 0.45, 1.2, (0.92, 0.96, 1.0))
        rd.area_light("Rim", (-0.06, -0.24, 0.16), centre, 0.25, 3.0)
        rd.area_light("Top", (0.0, 0.0, 0.32), centre, 0.5, 1.5)
        back.rotation_euler = (0.0, 0.0, AZ - math.pi / 2)
        back.location = (-math.cos(AZ) * 0.12, -math.sin(AZ) * 0.12, rd.FLOOR_Z)
    else:
        dark_studio(mats, objs, centre)

    cam = rd.camera()
    cam.data.sensor_fit = "HORIZONTAL"        # same horizontal framing in 16:9 and 9:16
    cam.data.dof.aperture_fstop = 40.0
    tr = cam.constraints.new("TRACK_TO")
    tr.target, tr.track_axis, tr.up_axis = focus, "TRACK_NEGATIVE_Z", "UP_Y"

    def shot(keys, focus_at=(0.0, 0.0, -1.0), fstop=40.0):
        """keys: (frame, elev, dist[, az]) with the last one held to the cut; focus point in drone mm."""
        zoom = ZOOM[[n for n, (a, b) in SHOTS.items() if a == keys[0][0]][0]] if PORTRAIT else 1.0
        for i, k in enumerate(keys):
            f, elev, dist = k[:3]
            dist *= zoom
            az = k[3] if len(k) > 3 else AZ
            (hold if i == len(keys) - 1 else key)(cam, "location", f, cam_pos(elev, dist, az))
            cam.data.dof.focus_distance = dist
            cam.data.dof.keyframe_insert(data_path="focus_distance", frame=f)
        cam.data.dof.aperture_fstop = fstop
        cam.data.dof.keyframe_insert(data_path="aperture_fstop", frame=keys[0][0])
        hold(focus, "location", keys[0][0], tuple(v * MM for v in focus_at))

    shot([(1, 5.0, 0.40), (120, 7.0, 0.31)])                                   # reveal: push in
    shot([(121, 12.0, 0.30), (200, 34.0, 0.27), (264, 22.0, 0.28)])            # hero: crane
    shot([(265, 62.0, 0.31), (384, 58.0, 0.29)])                               # top
    shot([(385, 32.0, 0.27), (504, 36.0, 0.21)], focus_at=(0.0, 0.0, 8.0))     # explode: push in on the board
    # macro: drone held at 1.25 rev (90 deg), so M2's duct (13, 13) sits at (-13, 13); camera low on its outside
    # camera on the hero side of it so the backdrop stays behind, low enough to show the landing bead
    duct = (-13.0, 13.0, -12.0)
    maz = AZ + 0.55
    shot([(505, 14.0, 0.165, maz), (600, 10.0, 0.14, maz - 0.22)], focus_at=duct, fstop=22.0)
    shot([(601, 8.0, 0.33), (744, 14.0, 0.30)])                               # take-off: camera follows Focus

    device = rd.setup_render()
    if LOOK != "light":
        w = sc.world.node_tree.nodes["Background"]
        w.inputs["Color"].default_value = (0.3, 0.36, 0.45, 1.0)
        w.inputs["Strength"].default_value = 0.015
        sc.view_settings.exposure = -0.7
        sc.cycles.glossy_bounces = 6
    sc.render.resolution_x, sc.render.resolution_y = RES
    sc.cycles.samples = SAMPLES
    sc.render.use_motion_blur = True
    sc.render.motion_blur_shutter = 0.5
    sc.render.use_persistent_data = True
    sc.render.image_settings.file_format = "PNG"
    sc.render.image_settings.color_mode = "RGB"
    return device


def main():
    device = build()
    os.makedirs(OUT, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(rd.OUT, "nanodrone_product_shots%s%s.blend"
                                                      % ("" if LOOK == "light" else "_dark",
                                                         "_portrait" if PORTRAIT else "")))
    sc = bpy.context.scene
    sc.render.use_overwrite = False
    sc.render.use_file_extension = True
    print("product shots: device %s, %d samples, %dx%d" % (device, SAMPLES, RES[0], RES[1]), flush=True)
    t = time.time()
    if ONLY:
        for f in ONLY:
            sc.frame_set(f)
            sc.render.filepath = os.path.join(OUT, "frame_%04d.png" % f)
            bpy.ops.render.render(write_still=True)
            print("frame %d done" % f, flush=True)
    else:
        sc.render.filepath = os.path.join(OUT, "frame_")
        sc.frame_start, sc.frame_end = START, STOP
        bpy.ops.render.render(animation=True)
    print("rendered in %.0f s" % (time.time() - t), flush=True)


if __name__ == "__main__":
    main()
