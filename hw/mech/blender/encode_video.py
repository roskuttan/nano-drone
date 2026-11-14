"""encode_video.py - assemble the animation frames into an H.264 MP4 with a fade in and out.

    blender -b --factory-startup -P hw/mech/blender/encode_video.py

Reads hw/output/mech/blender/anim/frame_####.png, writes hw/output/mech/blender/nanodrone_product_video.mp4.
"""
import os
import re

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(HW, "output", "mech", "blender")
ANIM = os.path.join(OUT, "anim")
FPS, FADE = 24, 18


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    frames = sorted(f for f in os.listdir(ANIM) if re.match(r"frame_\d{4}\.png$", f))
    n = len(frames)
    sc = bpy.context.scene
    sc.render.fps = FPS
    sc.frame_start, sc.frame_end = 1, n
    sc.render.resolution_x, sc.render.resolution_y = 1920, 1080
    sc.render.resolution_percentage = 100
    sc.view_settings.view_transform = "Standard"   # the frames are already tone-mapped; do not apply AgX again
    sc.view_settings.look = "None"
    sc.sequence_editor_create()
    se = sc.sequence_editor
    strips = se.strips if hasattr(se, "strips") else se.sequences
    black = strips.new_effect(name="black", type="COLOR", channel=1, frame_start=1, length=n)
    black.color = (0.0, 0.0, 0.0)
    img = strips.new_image(name="frames", filepath=os.path.join(ANIM, frames[0]), channel=2, frame_start=1)
    for f in frames[1:]:
        img.elements.append(f)
    img.blend_type = "ALPHA_OVER"
    for frame, alpha in ((1, 0.0), (FADE, 1.0), (n - FADE, 1.0), (n, 0.0)):
        img.blend_alpha = alpha
        img.keyframe_insert(data_path="blend_alpha", frame=frame)
    r = sc.render
    if hasattr(r.image_settings, "media_type"):   # Blender 5: video output is its own media type
        r.image_settings.media_type = "VIDEO"
    r.image_settings.file_format = "FFMPEG"
    r.ffmpeg.format = "MPEG4"
    r.ffmpeg.codec = "H264"
    r.ffmpeg.constant_rate_factor = "HIGH"
    r.ffmpeg.ffmpeg_preset = "GOOD"
    r.ffmpeg.gopsize = 12
    r.filepath = os.path.join(OUT, "nanodrone_product_video.mp4")
    bpy.ops.render.render(animation=True)
    print("wrote", r.filepath, "frames", n)


main()
