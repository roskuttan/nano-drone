"""board_ref.py [board.kicad_pcb] - mechanical reference of the flight board for the enclosure generator.

Writes output/mech/board_ref.json in the Fusion frame: board centre at the origin (KiCad (100, 100) mm), board top
surface at z = 0, X = KiCad x, Y = -KiCad y (front = +Y), millimetres. Contents:
  outline  Edge.Cuts polygon
  holes    every drill >= 0.3 mm (ref, pos, d, plated); the motor mounts are H1-H4 (centre 2.6 mm + 3 x 1.5 mm)
  parts    footprint bounding boxes per side with a body height for the clearance checks (package heuristics)
  motors   M1-M4 positions (from H1-H4) and the side each motor sits on
The enclosure generator (mech/fusion/nanodrone_enclosure) reads this file. Re-run it after any placement change.
"""
import json
import os
import sys

import pcbnew

HW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HW, "output", "mech", "board_ref.json")
MOTOR_SIDE = {"M1": "top", "M2": "bottom", "M3": "bottom", "M4": "top"}   # motors on top: M1/M4 (see ARCHITECTURE)


def F(v):
    return (round(v.x / 1e6 - 100, 4), round(-(v.y / 1e6 - 100), 4))


def height(f):
    """Body height (mm) by package; pads-only footprints get 0.05."""
    fp, r = f.GetFPID().GetUniStringLibId(), f.GetReference()
    if "0603" in fp:
        return 0.9
    if "0402" in fp:
        return 0.55
    if "0201" in fp:
        return 0.33
    if "QFN-48" in fp or "QFN-28" in fp:
        return 0.6
    if "WQFN" in fp:
        return 0.8
    if r == "U4":
        return 0.9
    if r == "U3":
        return 0.86
    if "SON" in fp:
        return 0.45
    if r.startswith("Y"):
        return 0.6
    if r.startswith("D"):
        return 0.45
    if r.startswith(("J", "H", "FID")):
        return 0.05
    return 0.6


def main():
    pcb = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HW, "kicad", "nanodrone.kicad_pcb")
    b = pcbnew.LoadBoard(pcb)
    out = {"units": "mm", "frame": "board centre origin, board top z=0, front=+Y", "board_t": 0.8}
    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
            o = d.GetPolyShape().Outline(0)
            out["outline"] = [F(o.CPoint(i)) for i in range(o.PointCount())]
    holes, motors = [], {}
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetDrillSize().x >= 0.3e6:
                holes.append({"ref": f.GetReference(), "pos": F(p.GetPosition()), "d": round(p.GetDrillSize().x / 1e6, 3),
                              "plated": p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH})
                if f.GetReference() in ("H1", "H2", "H3", "H4") and p.GetDrillSize().x > 2.0e6:
                    m = "M" + f.GetReference()[1]
                    motors[m] = {"pos": list(F(p.GetPosition())), "side": MOTOR_SIDE[m]}
    out["holes"] = holes
    parts = []
    for f in b.GetFootprints():
        r = f.GetReference()
        if r.startswith(("H", "FID")):
            continue
        bb = f.GetBoundingBox(False, False)
        x0, y0 = bb.GetLeft() / 1e6 - 100, bb.GetTop() / 1e6 - 100
        x1, y1 = bb.GetRight() / 1e6 - 100, bb.GetBottom() / 1e6 - 100
        parts.append({"ref": r, "side": "top" if f.GetLayer() == pcbnew.F_Cu else "bottom", "h": height(f),
                      "x0": round(x0, 3), "x1": round(x1, 3), "y0": round(-y1, 3), "y1": round(-y0, 3),
                      "value": f.GetValue()})
    out["parts"] = parts
    out["motors"] = dict(sorted(motors.items()))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(out, fh, indent=0)
    print("%s: outline %d pts, holes %d, parts %d, motors %s" % (OUT, len(out.get("outline", [])), len(holes),
                                                              len(parts), sorted(motors)))


if __name__ == "__main__":
    main()
