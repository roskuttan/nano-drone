"""edit_product_video.py - cut the product video: shots, titles, lower-thirds, spec card, outro, H.264 MP4.

    blender -b --factory-startup -P hw/mech/blender/edit_product_video.py -- [--look dark|light]
            [--aspect landscape|portrait] [--stills N,M]

Reads the frames from product_video.py (product_dark/ for the dark look, product/ for the light one) and writes
hw/output/mech/blender/nanodrone_product_video.mp4 (dark) or nanodrone_product_video_light.mp4, 1080p24, with
the soundtrack from make_audio.py (AAC). The dark look gets white captions and dips to black at the cuts, the
light one ink captions and dips to white. --aspect portrait cuts the 1080x1920 Instagram Story version
(nanodrone_product_video_story.mp4) from the "_portrait" frames: same timeline and sound, captions laid out inside the
Story safe area (the top and bottom 13 % are under Instagram's own UI). Edit NAME / TAGLINE / shot text below
to change the copy.
"""
import os
import re
import sys

import bpy

_args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
STILLS = _args[_args.index("--stills") + 1] if "--stills" in _args else ""     # e.g. --stills 60,200,780
LOOK = _args[_args.index("--look") + 1] if "--look" in _args else "dark"
DARK = LOOK != "light"
PORTRAIT = (_args[_args.index("--aspect") + 1] if "--aspect" in _args else "landscape") == "portrait"

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(HW, "output", "mech", "blender")
FRAMES = os.path.join(OUT, ("product_dark" if DARK else "product") + ("_portrait" if PORTRAIT else ""))
SUFFIX = ("" if DARK else "_light") + ("_story" if PORTRAIT else "")
AUDIO = os.path.join(OUT, "nanodrone_product_audio.wav")     # from make_audio.py
FPS = 24
W, H = (1080, 1920) if PORTRAIT else (1920, 1080)
NAME = "NANO"
TAGLINE = "The world's smallest brushless drone"

FONT_REG = r"C:\Windows\Fonts\bahnschrift.ttf"
FONT_LIGHT = r"C:\Windows\Fonts\calibril.ttf"
FONT_BOLD = r"C:\Windows\Fonts\arialbd.ttf"
INK = (0.08, 0.08, 0.09, 1.0)          # text on the light studio shots
PAPER = (0.96, 0.96, 0.97, 1.0)        # text on the dark cards and the dark shots
CAPTION = PAPER if DARK else INK
CAPTION_SUB = (0.80, 0.80, 0.82, 1.0) if DARK else INK
DIP = (0.0, 0.0, 0.0) if DARK else (1.0, 1.0, 1.0)
ACCENT = (0.85, 0.12, 0.10, 1.0)       # the motors' red
BLACK = (0.02, 0.02, 0.025, 1.0)

# shots: (name, first frame, last frame) in the rendered sequence; the cards are appended after them
SHOTS = [("reveal", 1, 120), ("hero", 121, 264), ("top", 265, 384), ("explode", 385, 504),
         ("macro", 505, 600), ("takeoff", 601, 744)]
CARD_INTRO, CARD_SPECS, CARD_OUTRO = 54, 96, 72   # frames: title card, spec card, outro
FADE = 10

# lower-thirds: (shot, in-offset, out-offset within the shot, headline, sub-line)
CALLOUTS = [
    ("hero", 12, 132, "19 g ready to fly", "36 x 36 mm board, 61 mm across the ducts"),
    ("top", 10, 110, "Four 31 mm props", "0702 23 000 KV motors in rolled-lip ducts, 1 mm tip gap"),
    ("explode", 40, 118, "The board is the frame", "6-layer 0.8 mm PCB, one STM32 + DRV8311 ESC per motor"),
    ("macro", 8, 88, "Ducts double as landing gear", "MJF nylon, 3.4 g for the whole enclosure"),
    ("takeoff", 24, 132, "1S 300 mAh, BT2.0", "STM32G431 flight controller, 2.4 GHz SX1280 link"),
]
SPECS = [("Weight", "19 g ready to fly, 16 g without the enclosure"),
         ("Size", "36 x 36 mm board, 61 x 61 x 30 mm with ducts"),
         ("Propulsion", "4 x 0702 23 000 KV, 31 mm 3-blade props, stacked"),
         ("Electronics", "STM32G431 FC, 4 x STM32G071 + DRV8311 ESCs, 96 kHz PWM"),
         ("Sensors, radio", "LSM6DSV16X IMU, SX1280 2.4 GHz"),
         ("Battery", "1S 300 mAh LiHV, BT2.0, 6-7 min hover"),
         ("Enclosure", "MJF PA12, four ducts, landing gear built in")]

_chan = [4]


def chan():
    _chan[0] += 1
    return _chan[0]


def font(path):
    return bpy.data.fonts.load(path) if os.path.exists(path) else None


def fade_in_out(strip, start, end, slide=0):
    """Alpha fade at both ends plus a small upward slide on the way in."""
    for f, a in ((start, 0.0), (start + FADE, 1.0), (end - FADE, 1.0), (end, 0.0)):
        strip.blend_alpha = a
        strip.keyframe_insert(data_path="blend_alpha", frame=f)
    if slide:
        base = strip.transform.offset_y
        for f, dy in ((start, -slide), (start + FADE + 4, 0)):
            strip.transform.offset_y = base + dy
            strip.transform.keyframe_insert(data_path="offset_y", frame=f)


def text(strips, body, start, end, x, y, size, color, fnt=None, anchor=("LEFT", "BOTTOM"), slide=18, wrap=0.0):
    s = strips.new_effect(name=body[:24], type="TEXT", channel=chan(), frame_start=start, length=end - start)
    s.text = body
    s.font_size = size
    s.color = color
    s.location = (x, y)
    if fnt:
        s.font = fnt
    for attr, val in (("anchor_x", anchor[0]), ("anchor_y", anchor[1]), ("align_x", anchor[0]),
                      ("align_y", anchor[1]), ("alignment_x", anchor[0])):
        if hasattr(s, attr):
            try:
                setattr(s, attr, val)
            except TypeError:
                pass
    s.use_shadow = False
    if wrap:
        s.wrap_width = wrap
    s.blend_type = "ALPHA_OVER"
    fade_in_out(s, start, end, slide)
    return s


def bar(strips, start, end, x_px, y_px, w_px, h_px, color):
    """Accent bar: a colour strip scaled down to a rectangle (offsets are from the frame centre)."""
    s = strips.new_effect(name="bar", type="COLOR", channel=chan(), frame_start=start, length=end - start)
    s.color = color[:3]
    s.transform.scale_x, s.transform.scale_y = w_px / W, h_px / H
    s.transform.offset_x, s.transform.offset_y = x_px - W / 2 + w_px / 2, y_px - H / 2 + h_px / 2
    s.blend_type = "ALPHA_OVER"
    fade_in_out(s, start, end, 18)
    return s


def card(strips, start, end):
    s = strips.new_effect(name="card", type="COLOR", channel=chan(), frame_start=start, length=end - start)
    s.color = BLACK[:3]
    s.blend_type = "ALPHA_OVER"
    fade_in_out(s, start, end)
    return s


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    frames = sorted(f for f in os.listdir(FRAMES) if re.match(r"frame_\d{4}\.png$", f))
    n3d = len(frames)
    off = CARD_INTRO                        # the shots start after the title card
    total = off + n3d + CARD_SPECS + CARD_OUTRO
    sc = bpy.context.scene
    sc.render.fps = FPS
    sc.frame_start, sc.frame_end = 1, total
    sc.render.resolution_x, sc.render.resolution_y = W, H
    sc.render.resolution_percentage = 100
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"
    sc.sequence_editor_create()
    se = sc.sequence_editor
    strips = se.strips if hasattr(se, "strips") else se.sequences
    reg, light, bold = font(FONT_REG), font(FONT_LIGHT), font(FONT_BOLD)

    black = strips.new_effect(name="black", type="COLOR", channel=1, frame_start=1, length=total)
    black.color = BLACK[:3]
    img = strips.new_image(name="shots", filepath=os.path.join(FRAMES, frames[0]), channel=2, frame_start=off + 1)
    for f in frames[1:]:
        img.elements.append(f)
    img.blend_type = "ALPHA_OVER"
    for f, a in ((off + 1, 0.0), (off + FADE + 6, 1.0), (off + n3d - FADE, 1.0), (off + n3d, 0.0)):
        img.blend_alpha = a
        img.keyframe_insert(data_path="blend_alpha", frame=f)
    # dips at the cuts: to black on the dark look, to white on the light one
    for _, s0, _ in SHOTS[1:]:
        s0 += off
        w = strips.new_effect(name="dip", type="COLOR", channel=3, frame_start=s0 - 5, length=10)
        w.color = DIP
        w.blend_type = "ALPHA_OVER"
        for f, a in ((s0 - 5, 0.0), (s0, 0.85), (s0 + 4, 0.0)):
            w.blend_alpha = a
            w.keyframe_insert(data_path="blend_alpha", frame=f)

    # ---- title card, then the reveal shot runs clean
    rv = dict((s[0], (s[1] + off, s[2] + off)) for s in SHOTS)
    if PORTRAIT:
        text(strips, NAME, 6, off + 12, 0.5, 0.545, 210, PAPER, bold, ("CENTER", "CENTER"), slide=30)
        text(strips, TAGLINE, 14, off + 12, 0.5, 0.465, 40, (0.78, 0.78, 0.8, 1.0), light, ("CENTER", "CENTER"),
             slide=24, wrap=0.86)
    else:
        text(strips, NAME, 6, off + 12, 0.5, 0.57, 230, PAPER, bold, ("CENTER", "CENTER"), slide=30)
        text(strips, TAGLINE, 14, off + 12, 0.5, 0.44, 46, (0.78, 0.78, 0.8, 1.0), light, ("CENTER", "CENTER"), slide=24)

    # ---- lower-thirds
    for shot, i0, i1, head, sub in CALLOUTS:
        s0, _ = rv[shot]
        a, b = s0 + i0, s0 + i1
        if PORTRAIT:                                   # above the Story reply bar, below the drone
            bar(strips, a, b, 70, int(0.175 * H), 6, 112, ACCENT)
            text(strips, head, a + 2, b, 0.092, 0.218, 54, CAPTION, reg, ("LEFT", "BOTTOM"))
            text(strips, sub, a + 6, b, 0.092, 0.178, 28, CAPTION_SUB, light, ("LEFT", "BOTTOM"), wrap=0.86)
        else:
            bar(strips, a, b, 118, 118, 6, 118, ACCENT)
            text(strips, head, a + 2, b, 0.075, 0.165, 64, CAPTION, reg, ("LEFT", "BOTTOM"))
            text(strips, sub, a + 6, b, 0.075, 0.112, 32, CAPTION_SUB, light, ("LEFT", "BOTTOM"))

    # ---- spec card
    c0 = off + n3d + 1
    c1 = c0 + CARD_SPECS
    card(strips, c0, c1)
    if PORTRAIT:                                       # label over value, stacked down the safe area
        text(strips, "Specifications", c0 + 4, c1, 0.5, 0.835, 52, PAPER, reg, ("CENTER", "CENTER"))
        bar(strips, c0 + 6, c1, W / 2 - 36, int(0.805 * H), 72, 4, ACCENT)
        for i, (k, v) in enumerate(SPECS):
            y = 0.755 - i * 0.082
            text(strips, k, c0 + 8 + i * 3, c1, 0.5, y, 26, (0.72, 0.72, 0.74, 1.0), light, ("CENTER", "CENTER"))
            text(strips, v, c0 + 8 + i * 3, c1, 0.5, y - 0.03, 30, PAPER, reg, ("CENTER", "CENTER"), wrap=0.88)
    else:
        text(strips, "Specifications", c0 + 4, c1, 0.5, 0.86, 60, PAPER, reg, ("CENTER", "CENTER"))
        bar(strips, c0 + 6, c1, W / 2 - 40, int(0.81 * H), 80, 4, ACCENT)
        for i, (k, v) in enumerate(SPECS):
            y = 0.72 - i * 0.082
            text(strips, k, c0 + 8 + i * 3, c1, 0.42, y, 34, (0.72, 0.72, 0.74, 1.0), light, ("RIGHT", "CENTER"))
            text(strips, v, c0 + 8 + i * 3, c1, 0.45, y, 34, PAPER, reg, ("LEFT", "CENTER"))

    # ---- outro
    o0, o1 = c1 + 1, total
    card(strips, o0, o1)
    text(strips, NAME, o0 + 4, o1, 0.5, 0.545 if PORTRAIT else 0.56, 190, PAPER, bold, ("CENTER", "CENTER"),
         slide=24)
    text(strips, "Hardware, enclosure and firmware spec generated from code", o0 + 14, o1, 0.5,
         0.47 if PORTRAIT else 0.42, 30 if PORTRAIT else 34, (0.72, 0.72, 0.74, 1.0), light, ("CENTER", "CENTER"),
         wrap=0.84 if PORTRAIT else 0.0)

    if os.path.exists(AUDIO):
        snd = strips.new_sound(name="soundtrack", filepath=AUDIO, channel=chan(), frame_start=1)
        snd.volume = 1.0
    r = sc.render
    if hasattr(r.image_settings, "media_type"):
        r.image_settings.media_type = "VIDEO"
    r.image_settings.file_format = "FFMPEG"
    r.ffmpeg.format = "MPEG4"
    r.ffmpeg.codec = "H264"
    r.ffmpeg.constant_rate_factor = "HIGH"
    r.ffmpeg.ffmpeg_preset = "GOOD"
    r.ffmpeg.gopsize = 12
    r.ffmpeg.audio_codec = "AAC"
    r.ffmpeg.audio_bitrate = 192
    r.ffmpeg.audio_mixrate = 48000
    r.filepath = os.path.join(OUT, "nanodrone_product_video%s.mp4" % SUFFIX)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "nanodrone_product_edit%s.blend" % SUFFIX))
    stills = [int(v) for v in STILLS.split(",") if v]
    if stills:                                    # typography check: PNGs of a few frames instead of the MP4
        r.image_settings.media_type = "IMAGE" if hasattr(r.image_settings, "media_type") else None
        r.image_settings.file_format = "PNG"
        for f in stills:
            sc.frame_set(f)
            r.filepath = os.path.join(OUT, "edit_preview%s_%04d.png" % (SUFFIX, f))
            bpy.ops.render.render(write_still=True)
            print("preview", r.filepath)
        return
    bpy.ops.render.render(animation=True)
    print("wrote", r.filepath, "frames", total, "(%.1f s)" % (total / FPS))


main()
