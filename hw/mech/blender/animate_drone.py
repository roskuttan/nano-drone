"""animate_drone.py - product video of the nano drone in Blender Cycles.

    blender -b --factory-startup -P hw/mech/blender/animate_drone.py -- [--start N] [--end N] [--samples 48]
            [--res 1920x1080] [--only N,M]

Shot (14 s at 24 fps, 336 frames): the drone sits on a studio sweep and turns a full revolution on a turntable
while the camera cranes from a low hero angle up and back down. The props spin with motion blur. In the last
four seconds the drone lifts off and leans forward, the camera following it up.

Frames go to hw/output/mech/blender/anim/frame_####.png (frames that already exist are skipped, so the render can
be resumed or split). encode_video.py then assembles the MP4. Materials, lights and backdrop come from
render_drone.py.
"""
import math
import os
import sys
import time

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_drone as rd  # noqa: E402

OUT = os.path.join(rd.OUT, "anim")
FPS, END = 24, 336
MM = rd.MM
REV_PER_FRAME = 0.36            # prop turn per frame: with a 0.5 shutter each blade blurs through 65 degrees
MOTOR = {"M1": (13.0, -13.0), "M2": (13.0, 13.0), "M3": (-13.0, -13.0), "M4": (-13.0, 13.0)}
SPIN_DIR = {"M1": 1, "M2": -1, "M3": -1, "M4": 1}

args = rd.args
SAMPLES = int(rd.arg("--samples", "32"))
RES = [int(v) for v in rd.arg("--res", "1920x1080").split("x")]
START, STOP = int(rd.arg("--start", "1")), int(rd.arg("--end", str(END)))
ONLY = [int(v) for v in rd.arg("--only", "").split(",") if v]


def key(ob, path, frame, value, interp="BEZIER"):
    setattr(ob, path, value)
    ob.keyframe_insert(data_path=path, frame=frame)
    act = ob.animation_data.action
    if hasattr(act, "fcurves"):                      # Blender 4.x
        fcurves = act.fcurves
    else:                                            # Blender 5 layered actions
        fcurves = [fc for layer in act.layers for strip in layer.strips
                   for cb in strip.channelbags for fc in cb.fcurves]
    for fc in fcurves:
        for kp in fc.keyframe_points:
            if abs(kp.co.x - frame) < 0.5:
                kp.interpolation = interp


def build():
    rd.clear()
    mats = rd.materials()
    objs = rd.import_drone(mats)
    sc = bpy.context.scene
    sc.render.fps = FPS
    sc.frame_start, sc.frame_end = 1, END

    drone = bpy.data.objects.new("Drone", None)
    sc.collection.objects.link(drone)
    focus = bpy.data.objects.new("Focus", None)
    sc.collection.objects.link(focus)
    focus.parent = drone
    focus.location = (0.0, 0.0, -1.0 * MM)

    for o in objs:
        o.parent = drone
        if o.name.startswith("Visual__blades_"):
            m = o.name.split("_")[-1]
            c = Vector((MOTOR[m][0] * MM, MOTOR[m][1] * MM, 0.0))
            # the importer may keep the mm->m scale on the object, so shift the mesh in its own units
            o.data.transform(__import__("mathutils").Matrix.Translation(-c / o.scale.x))
            o.location = c
            key(o, "rotation_euler", 1, (0.0, 0.0, 0.0), "LINEAR")
            key(o, "rotation_euler", END, (0.0, 0.0, SPIN_DIR[m] * 2 * math.pi * REV_PER_FRAME * (END - 1)), "LINEAR")

    # turntable: one revolution, easing in and out; lift-off with a forward lean over the last 4 s
    key(drone, "rotation_euler", 1, (0.0, 0.0, 0.0))
    key(drone, "rotation_euler", 240, (0.0, 0.0, 2 * math.pi * 0.78))
    key(drone, "rotation_euler", END, (math.radians(-5.0), 0.0, 2 * math.pi))
    key(drone, "location", 236, (0.0, 0.0, 0.0))
    key(drone, "location", END, (0.0, 0.0, 38.0 * MM))

    back = rd.sweep(mats["floor"])
    centre = Vector((0.0, 0.0, 0.0))
    rd.area_light("Key", (0.16, 0.10, 0.22), centre, 0.35, 4.0, (1.0, 0.97, 0.93))
    rd.area_light("Fill", (-0.22, 0.12, 0.08), centre, 0.45, 1.2, (0.92, 0.96, 1.0))
    rd.area_light("Rim", (-0.06, -0.24, 0.16), centre, 0.25, 3.0)
    rd.area_light("Top", (0.0, 0.0, 0.32), centre, 0.5, 1.5)

    cam = rd.camera()
    cam.data.dof.aperture_fstop = 40.0
    tr = cam.constraints.new("TRACK_TO")
    tr.target, tr.track_axis, tr.up_axis = focus, "TRACK_NEGATIVE_Z", "UP_Y"
    az = math.atan2(95.0, 150.0)
    back.rotation_euler = (0.0, 0.0, az - math.pi / 2)
    back.location = (-math.cos(az) * 0.12, -math.sin(az) * 0.12, rd.FLOOR_Z)

    def cam_pos(elev_deg, dist):
        e = math.radians(elev_deg)
        return (math.cos(az) * math.cos(e) * dist, math.sin(az) * math.cos(e) * dist, math.sin(e) * dist)

    for f, elev, dist in ((1, 10.0, 0.31), (120, 34.0, 0.27), (236, 18.0, 0.25), (END, 8.0, 0.30)):
        key(cam, "location", f, cam_pos(elev, dist))
        cam.data.dof.focus_distance = dist
        cam.data.dof.keyframe_insert(data_path="focus_distance", frame=f)

    device = rd.setup_render()
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
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(rd.OUT, "nanodrone_video.blend"))
    sc = bpy.context.scene
    sc.render.filepath = os.path.join(OUT, "frame_")
    sc.render.use_overwrite = False          # existing frames are skipped, so a render can be resumed
    sc.render.use_file_extension = True
    print("animation: device %s, %d samples, %dx%d" % (device, SAMPLES, RES[0], RES[1]), flush=True)
    t = time.time()
    if ONLY:                                  # a few single frames (look checks): one render call each
        for f in ONLY:
            sc.frame_set(f)
            sc.render.filepath = os.path.join(OUT, "frame_%04d.png" % f)
            bpy.ops.render.render(write_still=True)
            print("frame %d done" % f, flush=True)
    else:                                     # one render session for the whole range (no per-frame restart)
        sc.frame_start, sc.frame_end = START, STOP
        bpy.ops.render.render(animation=True)
    print("rendered in %.0f s" % (time.time() - t), flush=True)


if __name__ == "__main__":
    main()
