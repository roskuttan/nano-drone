"""Shared helpers for the post-layout studies (hw/sim_post): board geometry from the routed .kicad_pcb,
stackup, materials, coarse rasters, plotting style (borrowed from hw/sim/simlib.py) and result files.

Coordinates: design mm, origin at the board centre, +x right, +y down (KiCad), same as the layout scripts.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
SIM = os.path.join(HW, "sim")
for p in (os.path.join(SIM, ".pylibs"), SIM, os.path.join(HW, "tools"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import pcbnew  # noqa: E402
import simlib as S  # noqa: E402  (plot style, palette, motor/driver models)
import extract as X  # noqa: E402  (fine rasters)

plt = S.plt
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)
PCB = os.environ.get("NANODRONE_PCB", os.path.join(HW, "kicad", "nanodrone.kicad_pcb"))
OX, OY = X.OX, X.OY

# ---- stackup (JLC 6-layer 0.8 mm, as written into the board by nanodrone_route.py) -------------------------
CU_LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "B.Cu"]
CU_IDS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu, pcbnew.B_Cu]
T_CU = np.array([0.035, 0.0152, 0.0152, 0.0152, 0.0152, 0.035]) * 1e-3          # m
T_DIEL = np.array([0.0994, 0.16, 0.13, 0.16, 0.0994]) * 1e-3                    # m, between Cu i and i+1
ER_DIEL = np.array([4.45, 4.6, 4.45, 4.6, 4.45])
T_MASK = 0.010e-3
ER_MASK = 3.8
RHO_CU = 1.72e-8          # ohm m at 20 C
ALPHA_CU = 0.00393        # 1/K
K_CU = 385.0              # W/mK
K_FR4_XY = 0.8            # W/mK (glass weave, in-plane)
K_FR4_Z = 0.30            # W/mK
RHOC_FR4 = 1.85e3 * 1100  # J/m3K
RHOC_CU = 8.96e3 * 385
T_PLATE = 0.018e-3        # via barrel plating (JLC min 18 um) [A]


def load(path=None):
    b = pcbnew.LoadBoard(path or PCB)
    return b


def xy(v):
    return v.x / 1e6 - OX, v.y / 1e6 - OY


def footprints(b):
    return {f.GetReference(): f for f in b.GetFootprints()}


def fp_side(f):
    return 0 if f.GetLayer() == pcbnew.F_Cu else 5


def motor_centres(b):
    """Motor axes from the Motor_0702_Mount footprints -> {name: (x, y, side_idx)}; side 0 = top, 5 = bottom."""
    out = {}
    for f in b.GetFootprints():
        if "Motor_0702_Mount" in f.GetFPIDAsString():
            out[f.GetReference()] = (*xy(f.GetPosition()), fp_side(f))
    return out


def fine_rasters(b, nets=("GND", "VBAT")):
    """Fine (0.05 mm) rasters: outline, all copper per layer, per-net copper per layer."""
    arr = {"outline": X.board_outline(b)}
    for name, lid in zip(CU_LAYERS, CU_IDS):
        arr["cu", name] = X.all_copper(b, lid)
        for n in nets:
            arr[n, name] = X.net_copper(b, n, lid)
    return arr


def block_mean(a, k):
    n = a.shape[0] // k
    return a[:n * k, :n * k].reshape(n, k, n, k).mean(axis=(1, 3))


def cell_of(x, y, res, half=X.HALF):
    return int((y + half) / res), int((x + half) / res)


def ring_fraction(raster, x, y, r, res=X.RES, half=X.HALF, n=24):
    """Fraction of n points on a circle of radius r (mm) that land on True pixels of a fine raster."""
    hit = 0
    for k in range(n):
        a = 2 * math.pi * k / n
        px, py = x + r * math.cos(a), y + r * math.sin(a)
        i, j = int((py + half) / res), int((px + half) / res)
        if 0 <= i < raster.shape[0] and 0 <= j < raster.shape[1] and raster[i, j]:
            hit += 1
    return hit / n


def power_vias(b):
    """GND/VBAT vias and plated footprint holes: [(x, y, net, drill_mm, pad_mm)]."""
    out = []
    for t in b.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() in ("GND", "VBAT"):
            out.append((*xy(t.GetPosition()), t.GetNetname(), t.GetDrillValue() / 1e6, t.GetWidth(pcbnew.F_Cu) / 1e6))
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and p.GetDrillSize().x > 0 and \
                    p.GetNetname() in ("GND", "VBAT"):
                out.append((*xy(p.GetPosition()), p.GetNetname(), p.GetDrillSize().x / 1e6, p.GetSize().x / 1e6))
    return out


def tracks(b, layer_ids=None):
    """[(net, layer_idx, x0, y0, x1, y1, w_mm)] for every track segment (arcs as chords)."""
    lid = {l: i for i, l in enumerate(CU_IDS)}
    out = []
    for t in b.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            continue
        L = t.GetLayer()
        if L not in lid or (layer_ids and L not in layer_ids):
            continue
        (x0, y0), (x1, y1) = xy(t.GetStart()), xy(t.GetEnd())
        out.append((t.GetNetname(), lid[L], x0, y0, x1, y1, t.GetWidth() / 1e6))
    return out


def write_results(name, data):
    def conv(o):
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, (np.bool_,)):
            return bool(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, tuple):
            return list(o)
        raise TypeError(type(o))
    path = os.path.join(OUT, name + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, default=conv)
    print("  wrote", os.path.relpath(path, HW))


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", os.path.relpath(path, HW))
    return path


def board_extent_axes(ax, half=X.HALF):
    ax.set_xlim(-half, half)
    ax.set_ylim(half, -half)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
