"""kifp - generate the nano drone's custom KiCad 10 footprints (.kicad_mod)."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kisch import Q, q, dump, fmt  # noqa: E402

VERSION = "20260206"


def _txt(kind, text, x, y, layer, size=0.5, hide=False):
    eff = ["effects", ["font", ["size", fmt(size), fmt(size)], ["thickness", fmt(size * 0.15)]]]
    if hide:
        eff.append(["hide", "yes"])
    return ["property", q(kind), q(text), ["at", fmt(x), fmt(y), "0"], ["layer", q(layer)], eff]


def _stroke(w):
    return ["stroke", ["width", fmt(w)], ["type", "solid"]]


def circle(cx, cy, r, layer, w=0.1, fill=False):
    return ["fp_circle", ["center", fmt(cx), fmt(cy)], ["end", fmt(cx + r), fmt(cy)], _stroke(w),
            ["fill", "yes" if fill else "no"], ["layer", q(layer)]]


def line(x0, y0, x1, y1, layer, w=0.1):
    return ["fp_line", ["start", fmt(x0), fmt(y0)], ["end", fmt(x1), fmt(y1)], _stroke(w), ["layer", q(layer)]]


def text(t, x, y, layer, size=0.5):
    eff = ["effects", ["font", ["size", fmt(size), fmt(size)], ["thickness", fmt(size * 0.15)]]]
    if layer.startswith("B."):                       # back-side text reads correctly from below
        eff.append(["justify", "mirror"])
    return ["fp_text", "user", q(t), ["at", fmt(x), fmt(y), "0"], ["layer", q(layer)], eff]


def pth(num, x, y, pad, drill, shape="circle"):
    return ["pad", q(num), "thru_hole", shape, ["at", fmt(x), fmt(y)], ["size", fmt(pad), fmt(pad)],
            ["drill", fmt(drill)], ["layers", q("*.Cu"), q("*.Mask")], ["remove_unused_layers", "no"]]


def npth(x, y, d):
    return ["pad", q(""), "np_thru_hole", "circle", ["at", fmt(x), fmt(y)], ["size", fmt(d), fmt(d)],
            ["drill", fmt(d)], ["layers", q("*.Cu"), q("*.Mask")]]


def footprint(name, descr, items, attr="through_hole", ref_y=-2.0, val_y=2.0):
    return ["footprint", q(name), ["version", VERSION], ["generator", q("kifp")],
            ["generator_version", q("10.0")], ["layer", q("F.Cu")], ["descr", q(descr)],
            ["tags", q("nanodrone")],
            _txt("Reference", "REF**", 0, ref_y, "F.SilkS"),
            _txt("Value", name, 0, val_y, "F.Fab"),
            ["attr", attr]] + items + [["embedded_fonts", "no"]]


def write(libdir, fp):
    os.makedirs(libdir, exist_ok=True)
    with open(os.path.join(libdir, str(fp[1]) + ".kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(fp) + "\n")


# ---------------------------------------------------------------------------
def motor_pads(pitch=1.0, pad=0.75, drill=0.4):
    """3 plated holes for 30 AWG motor leads (BetaFPV 0702SE II, 0.26 mm bare copper), A-B-C left to right.
    1.0 mm pitch lets the DRV8311's 0.4 mm-pitch OUT pins fan out with one 45 deg jog each."""
    items = []
    for i in range(3):
        x = (i - 1) * pitch
        items.append(pth(str(i + 1), x, 0, pad, drill, "rect" if i == 0 else "circle"))
        items.append(text("ABC"[i], x, -0.85, "F.SilkS", 0.45))
    w = pitch + pad / 2 + 0.25
    items += [line(-w, -0.75, w, -0.75, "F.CrtYd", 0.05), line(w, -0.75, w, 0.75, "F.CrtYd", 0.05),
              line(w, 0.75, -w, 0.75, "F.CrtYd", 0.05), line(-w, 0.75, -w, -0.75, "F.CrtYd", 0.05)]
    return footprint("MotorPads_3x_PTH", "Three plated holes for 30AWG motor phase wires, 1.0 mm pitch",
                     items, ref_y=-1.9, val_y=1.4)


def motor_mount(pcd=6.6, hole=1.5, centre=2.6, base_d=9.0):
    """BetaFPV 0702 II: '3*M1.4-EQS O6.6mm' -> 3 holes at 120 deg on a 6.6 mm bolt circle.
    Hole 1 points toward +x (rotate the footprint so it faces the frame centre or away)."""
    items = []
    r = pcd / 2
    for k in range(3):
        a = math.radians(120 * k)
        items.append(npth(r * math.cos(a), r * math.sin(a), hole))
    items.append(npth(0, 0, centre))                      # shaft circlip clearance
    for side in ("F", "B"):
        items.append(circle(0, 0, base_d / 2, side + ".SilkS", 0.12))
        items.append(circle(0, 0, base_d / 2 + 0.25, side + ".CrtYd", 0.05))
    items.append(circle(0, 0, base_d / 2, "F.Fab", 0.1))
    return footprint("Motor_0702_Mount", "BetaFPV 0702/0702SE II motor: 3x M1.4 EQS on 6.6 mm PCD + "
                     "2.6 mm shaft clearance; silk = 9 mm motor base", items, attr="exclude_from_pos_files",
                     ref_y=-5.2, val_y=5.2)


def battery_pads(pitch=2.5, pad=1.8, drill=0.9):
    items = [pth("1", -pitch / 2, 0, pad, drill, "rect"), pth("2", pitch / 2, 0, pad, drill),
             text("+", -pitch / 2, -1.5, "F.SilkS", 0.8), text("-", pitch / 2, -1.5, "F.SilkS", 0.8),
             text("+", -pitch / 2, -1.5, "B.SilkS", 0.8), text("-", pitch / 2, -1.5, "B.SilkS", 0.8)]
    w, h = pitch / 2 + pad / 2 + 0.25, pad / 2 + 0.25
    items += [line(-w, -h, w, -h, "F.CrtYd", 0.05), line(w, -h, w, h, "F.CrtYd", 0.05),
              line(w, h, -w, h, "F.CrtYd", 0.05), line(-w, h, -w, -h, "F.CrtYd", 0.05)]
    return footprint("BatteryPads_PTH", "1S battery lead pads (BT2.0 pigtail, 22-26 AWG), pad 1 = +",
                     items, ref_y=-2.4, val_y=2.0)


def antenna_pads(pitch=1.6):
    items = [pth("1", 0, 0, 0.9, 0.45, "circle"), pth("2", pitch, 0, 1.2, 0.6, "rect"),
             text("ANT", 0, -1.0, "F.SilkS", 0.5)]
    items += [line(-0.7, -0.85, pitch + 0.85, -0.85, "F.CrtYd", 0.05),
              line(pitch + 0.85, -0.85, pitch + 0.85, 0.85, "F.CrtYd", 0.05),
              line(pitch + 0.85, 0.85, -0.7, 0.85, "F.CrtYd", 0.05),
              line(-0.7, 0.85, -0.7, -0.85, "F.CrtYd", 0.05)]
    return footprint("AntennaPads_PTH", "2.4 GHz antenna feed: pad 1 = 50R feed (coax core / wire monopole), "
                     "pad 2 = ground (coax shield / dipole arm)", items, ref_y=-1.8, val_y=1.5)


def smd_round(num, x, y, d, paste=False, mask_margin=None):
    layers = [q("F.Cu"), q("F.Mask")] + ([q("F.Paste")] if paste else [])
    pad = ["pad", q(num), "smd", "circle", ["at", fmt(x), fmt(y)], ["size", fmt(d), fmt(d)], ["layers"] + layers]
    if mask_margin is not None:
        pad.append(["solder_mask_margin", fmt(mask_margin)])
    return pad


def antenna_pads_smd(pitch=1.3):
    """SMD antenna feed: pad 1 = 50R feed (0.6 x 1.0), pad 2 = GND (0.8 x 1.0); wire/coax soldered + glued."""
    items = [["pad", q("1"), "smd", "roundrect", ["at", "0", "0"], ["size", "0.6", "1.0"],
              ["layers", q("F.Cu"), q("F.Mask"), q("F.Paste")], ["roundrect_rratio", "0.2"]],
             ["pad", q("2"), "smd", "roundrect", ["at", fmt(pitch), "0"], ["size", "0.8", "1.0"],
              ["layers", q("F.Cu"), q("F.Mask"), q("F.Paste")], ["roundrect_rratio", "0.2"]],
             text("ANT", 0, -0.9, "F.Fab", 0.4),
             line(-0.3, -0.5, pitch + 0.4, -0.5, "F.Fab", 0.05), line(pitch + 0.4, -0.5, pitch + 0.4, 0.5, "F.Fab", 0.05),
             line(pitch + 0.4, 0.5, -0.3, 0.5, "F.Fab", 0.05), line(-0.3, 0.5, -0.3, -0.5, "F.Fab", 0.05)]
    return footprint("AntennaPads_SMD", "2.4 GHz antenna feed, SMD: pad 1 = 50R feed, pad 2 = ground", items,
                     attr="smd", ref_y=-1.4, val_y=1.2)


def swd_pads(n=5, pitch=1.27, d=0.8, names=("3V0", "DIO", "CLK", "RST", "GND"),
             name="SWD_Pads_1x05_P1.27mm", descr="SWD pogo pads 0.8 mm: 3V0 SWDIO SWCLK NRST GND"):
    """Pogo-pin SWD strip, pads numbered left to right (no paste: flat pads)."""
    items = []
    x0 = -(n - 1) * pitch / 2
    for i in range(n):
        items.append(smd_round(str(i + 1), x0 + i * pitch, 0, d))
        items.append(["fp_text", "user", q(names[i]), ["at", fmt(x0 + i * pitch), "-0.75", "0"], ["layer", q("F.Fab")],
                      ["effects", ["font", ["size", "0.3", "0.3"], ["thickness", "0.05"]]]])
    w = (n - 1) * pitch / 2 + d / 2
    items += [line(-w, -d / 2, w, -d / 2, "F.Fab", 0.05), line(w, -d / 2, w, d / 2, "F.Fab", 0.05),
              line(w, d / 2, -w, d / 2, "F.Fab", 0.05), line(-w, d / 2, -w, -d / 2, "F.Fab", 0.05)]
    return footprint(name, descr, items, attr="smd", ref_y=-1.3, val_y=1.0)


def esc_swd_pads():
    """Per-ESC programming pads (layout rev B): 1=SWDIO 2=SWCLK 3=NRST, 0.7 mm at 1.0 mm pitch. GND and 3V0 for
    the probe come from the nearest GND/3V0 copper; the ESC is flashed once here, later over the DShot wire."""
    return swd_pads(3, 1.0, 0.7, ("DIO", "CLK", "RST"), "SWD_Pads_1x03_P1.0mm",
                    "ESC SWD pogo pads 0.7 mm: SWDIO SWCLK NRST")


def fiducial(d=0.6, mask=1.2):
    items = [smd_round("", 0, 0, d, mask_margin=(mask - d) / 2),
             circle(0, 0, mask / 2, "F.Fab", 0.05)]
    return footprint("Fiducial_0.6mm_Mask1.2mm", "Fiducial 0.6 mm copper, 1.2 mm mask opening", items,
                     attr="smd", ref_y=-1.0, val_y=1.0)


if __name__ == "__main__":
    lib = sys.argv[1]
    for fp in (motor_pads(), motor_mount(), battery_pads(), antenna_pads(), antenna_pads_smd(), swd_pads(),
               esc_swd_pads(), fiducial()):
        write(lib, fp)
        print("wrote", fp[1])
