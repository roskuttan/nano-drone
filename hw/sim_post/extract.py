"""Post-layout extraction: copper rasters per layer + via list + net geometry from the routed board.

Rasters: each copper layer is plotted by kicad-cli (board-area SVG), rendered by PyMuPDF at RES mm/px
and thresholded -> boolean numpy array. Per-net rasters for the power nets are made by plotting with
only that net visible is not supported by kicad-cli, so net separation is done geometrically from
pcbnew (zones' filled polygons + tracks + pads) for the nets that matter (GND, VBAT).
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HW, "tools"))
PYLIBS = [os.path.join(HW, "sim", ".pylibs"),
          r"C:\Users\angel\AppData\Local\Temp\claude\e9d9e4576166a4b9\8404409f-3ecc-4ad5-ba45-8b90dd39ffba\scratchpad\pylibs"]
for p in PYLIBS:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.append(p)

import pcbnew  # noqa: E402

CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
PCB = os.path.join(HW, "kicad", "nanodrone.kicad_pcb")
OUT = os.path.join(HERE, "out")
OX, OY = 100.0, 100.0
RES = 0.05                      # mm per pixel
HALF = 18.5                     # raster covers [-HALF, HALF] mm in x and y
N = int(round(2 * HALF / RES))
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "B.Cu"]
LAYER_IDS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu, pcbnew.B_Cu]


def mm_to_px(x, y):
    return (np.asarray(x) + HALF) / RES, (np.asarray(y) + HALF) / RES


def poly_raster(polys):
    """Rasterise a list of polygons (list of (x,y) mm arrays, with holes as separate entries flagged)
    using an even-odd scanline fill. polys: list of (pts, is_hole)."""
    img = np.zeros((N, N), dtype=np.uint8)
    ys = (np.arange(N) + 0.5) * RES - HALF
    for pts, is_hole in polys:
        pts = np.asarray(pts, dtype=float)
        x0, y0 = pts[:, 0], pts[:, 1]
        x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
        rows = np.where((ys >= min(y0.min(), y1.min())) & (ys <= max(y0.max(), y1.max())))[0]
        for r in rows:
            y = ys[r]
            m = (y0 <= y) != (y1 <= y)
            if not m.any():
                continue
            xs = np.sort(x0[m] + (y - y0[m]) * (x1[m] - x0[m]) / (y1[m] - y0[m]))
            for a, b in zip(xs[0::2], xs[1::2]):
                c0 = int(max(0, math.ceil((a + HALF) / RES - 0.5)))
                c1 = int(min(N - 1, math.floor((b + HALF) / RES - 0.5)))
                if c1 >= c0:
                    if is_hole:
                        img[r, c0:c1 + 1] ^= 1
                    else:
                        img[r, c0:c1 + 1] ^= 1
    return img.astype(bool)


def shape_polys(sps):
    """SHAPE_POLY_SET -> [(pts, is_hole)] in design mm."""
    out = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        out.append(([((o.CPoint(k).x / 1e6) - OX, (o.CPoint(k).y / 1e6) - OY) for k in range(o.PointCount())], False))
        for h in range(sps.HoleCount(i)):
            hh = sps.Hole(i, h)
            out.append(([((hh.CPoint(k).x / 1e6) - OX, (hh.CPoint(k).y / 1e6) - OY) for k in range(hh.PointCount())], True))
    return out


def net_copper(board, net, layer):
    """Raster of one net's copper on one layer (zones + tracks + pads)."""
    sps = pcbnew.SHAPE_POLY_SET()
    for z in board.Zones():
        if z.GetNetname() == net and z.IsOnLayer(layer) and z.HasFilledPolysForLayer(layer):
            sps.Append(z.GetFilledPolysList(layer))
    for t in board.GetTracks():
        if t.GetNetname() == net and t.IsOnLayer(layer):
            t.TransformShapeToPolygon(sps, layer, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    for f in board.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net and p.IsOnLayer(layer):
                p.TransformShapeToPolygon(sps, layer, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    sps.Simplify()
    return poly_raster(shape_polys(sps))


def board_outline(board):
    for d in board.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
            return poly_raster(shape_polys(d.GetPolyShape()))
    raise RuntimeError("no outline")


def all_copper(board, layer):
    sps = pcbnew.SHAPE_POLY_SET()
    for z in board.Zones():
        if z.IsOnLayer(layer) and z.HasFilledPolysForLayer(layer):
            sps.Append(z.GetFilledPolysList(layer))
    for t in board.GetTracks():
        if t.IsOnLayer(layer):
            t.TransformShapeToPolygon(sps, layer, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    for f in board.GetFootprints():
        for p in f.Pads():
            if p.IsOnLayer(layer):
                p.TransformShapeToPolygon(sps, layer, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    sps.Simplify()
    return poly_raster(shape_polys(sps))


def vias(board):
    out = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            x, y = t.GetPosition().x / 1e6 - OX, t.GetPosition().y / 1e6 - OY
            out.append(dict(x=x, y=y, net=t.GetNetname(), drill=t.GetDrillValue() / 1e6,
                            d=t.GetWidth(pcbnew.F_Cu) / 1e6))
    for f in board.GetFootprints():          # plated pads with drills (thermal vias in footprints, PTH)
        for p in f.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and p.GetDrillSize().x > 0:
                x, y = p.GetPosition().x / 1e6 - OX, p.GetPosition().y / 1e6 - OY
                out.append(dict(x=x, y=y, net=p.GetNetname(), drill=p.GetDrillSize().x / 1e6,
                                d=p.GetSize().x / 1e6, ref=f.GetReference()))
    return out


def extract(path=PCB, nets=("GND", "VBAT")):
    os.makedirs(OUT, exist_ok=True)
    b = pcbnew.LoadBoard(path)
    res = {"res_mm": RES, "half_mm": HALF, "layers": LAYERS}
    arr = {"outline": board_outline(b)}
    for name, lid in zip(LAYERS, LAYER_IDS):
        arr["cu_" + name] = all_copper(b, lid)
        for n in nets:
            arr["%s_%s" % (n, name)] = net_copper(b, n, lid)
    np.savez_compressed(os.path.join(OUT, "copper.npz"), **arr)
    vs = vias(b)
    json.dump(dict(res, vias=vs), open(os.path.join(OUT, "vias.json"), "w"))
    cov = {k: float(v.sum() / max(arr["outline"].sum(), 1)) for k, v in arr.items() if k != "outline"}
    print("board area %.1f mm2, vias %d" % (arr["outline"].sum() * RES * RES, len(vs)))
    for k in sorted(cov):
        print("  %-14s coverage %5.1f %%" % (k, 100 * cov[k]))
    return arr, vs


if __name__ == "__main__":
    extract(sys.argv[1] if len(sys.argv) > 1 else PCB)
