"""Nano drone PCB - stage 2: rules, stackup, EMC-critical routing, autorouting, planes, DRC.

Run after nanodrone_pcb.py (placement). Steps:
  1. JLC 6-layer rules + netclasses into the .kicad_pro, stackup + via fill/cap defaults into the board
  2. power fan-out: filled+capped via-in-pad on GND/VBAT pads (decoupling loops close through planes)
  3. hand-routed critical nets: motor phases (short, wide), RF 50R line + GND via fence, FC EP vias
  4. Freerouting (headless) for the remaining nets on L1/L3/L6 only (L2, L4, L5 are planes)
  5. planes / pours + stitching, zone fill, DRC report
Layer plan: L1 F.Cu sig+GND | L2 In1 GND plane | L3 In2 sig+GND | L4 In3 VBAT plane | L5 In4 GND | L6 B.Cu sig+GND
"""
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HW, "tools"))
import pcbnew  # noqa: E402
import kipcb as P  # noqa: E402

PRJ = os.path.join(HW, "kicad")
PCB_PATH = os.environ.get("NANODRONE_PCB", os.path.join(PRJ, "nanodrone.kicad_pcb"))
PRO_PATH = os.path.join(PRJ, "nanodrone.kicad_pro")
DRU_PATH = os.path.join(PRJ, "nanodrone.kicad_dru")
DSN_PATH = os.path.join(PRJ, "nanodrone.dsn")
SES_PATH = os.path.join(PRJ, "nanodrone.ses")
KICAD_CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
JAVA = r"C:\Program Files\Eclipse Adoptium\jre-25.0.3.9-hotspot\bin\java.exe"
FREEROUTING = os.path.join(HW, "tools", "freerouting-2.5.0.jar")

RF_W = 0.12          # 50R grounded CPWG on L6 over L5 under solder mask (sim study 7: 0.15/0.15 bare = 49.8R,
RF_GAP = 0.15        # mask lowers it to 45.5R -> 0.12 mm under mask)
RF_ISO = 0.20        # RF copper to any other non-GND net (DRU rule "rf_isolation", also the DSN RF class clearance)
PHASE_W = 0.50
F, B = pcbnew.F_Cu, pcbnew.B_Cu
L2, L3, L4, L5 = pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu

# ---------------------------------------------------------------------------
# 1. rules, netclasses, stackup
# ---------------------------------------------------------------------------
# phase-to-everything clearance (netclass) and phase-to-signal track rule; the phase-to-analog spacing is PA_SPACING
NONET_KEEP = 0.45      # stitching vias keep this far from no-net pads (fiducial 0.3 mm mask + 0.15)
PHASE_CL = float(os.environ.get("NANODRONE_PHASE_CL", "0.15"))
NETCLASSES = [  # name, track, clearance, via_d, via_drill, patterns  (router widths; widened after routing)
    ("Default", 0.09, 0.09, 0.30, 0.15, []),                 # JLC multilayer min 3.5/3.5 mil (digital only)
    ("POWER", 0.15, 0.10, 0.30, 0.15, ["VBAT", "GND"]),
    ("RAIL", 0.15, 0.10, 0.30, 0.15, ["+3V0", "+3V0A", "+3V0_RF", "+3V0_IMU", "RF_VREG", "RF_VRPA",
                                       "M?_AVDD", "M?_CP"]),
    ("PHASE", 0.19, PHASE_CL, 0.30, 0.15, ["M?_OUT?"]),      # PA_SPACING to ANALOG enforced by .kicad_dru
    ("ANALOG", 0.10, 0.12, 0.30, 0.15, ["M?_BEMF?", "M?_ISENS?", "VBAT_SENSE"]),
    ("RF", RF_W, RF_GAP, 0.30, 0.15, ["RF_IO", "RF_N?", "ANT"]),
    ("XTAL", 0.12, 0.12, 0.30, 0.15, ["RF_XTA", "RF_XTB"]),
    ("DSHOT", 0.09, 0.09, 0.30, 0.15, ["M?_DSHOT"]),
]

STACKUP = """\t\t(stackup
\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))
\t\t\t(layer "F.Paste" (type "Top Solder Paste"))
\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1" (type "prepreg") (thickness 0.0994) (material "2116 RC58%") (epsilon_r 4.45) (loss_tangent 0.02))
\t\t\t(layer "In1.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 2" (type "core") (thickness 0.16) (material "FR4") (epsilon_r 4.6) (loss_tangent 0.02))
\t\t\t(layer "In2.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 3" (type "prepreg") (thickness 0.13) (material "2116 RC58%") (epsilon_r 4.45) (loss_tangent 0.02))
\t\t\t(layer "In3.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 4" (type "core") (thickness 0.16) (material "FR4") (epsilon_r 4.6) (loss_tangent 0.02))
\t\t\t(layer "In4.Cu" (type "copper") (thickness 0.0152))
\t\t\t(layer "dielectric 5" (type "prepreg") (thickness 0.0994) (material "2116 RC58%") (epsilon_r 4.45) (loss_tangent 0.02))
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))
\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen"))
\t\t\t(copper_finish "ENIG")
\t\t\t(dielectric_constraints no)
\t\t)
"""

# Phase-to-analog track spacing (rev B: 0.30 -> 0.20 mm). Field-solver tables of sim_post P3: a 35 V/us phase edge
# couples ~4.8 mV into a BEMF line along 3 mm at 0.20 mm (2.2 mV at 0.30 mm). That glitch settles through the
# 5 kOhm divider in tens of ns, and the ESC samples BEMF >= 2.8 us after every edge (FIRMWARE_REQUIREMENTS E7), so
# the sample window sees none of it. ISENS lines are driven from ~50 Ohm and are immune either way. Magnetic
# coupling from the phase ripple is ~0.1 mV.
PA_SPACING = float(os.environ.get("NANODRONE_PA_SPACING", "0.20"))
# a phase net and its own BEMF divider node (Mx_OUTy / Mx_BEMFy) are exempt from PA_SPACING (see grid_route)
PA_SAME_PHASE = os.environ.get("NANODRONE_PA_SAME_PHASE", "1") == "1"
_PAIRS = ["M%d_%s%s" % (m, k, ph) for m in (1, 2, 3, 4) for ph in "ABC" for k in ("OUT",)]
PA_EXEMPT = (" && !(" + " || ".join("(A.NetName == '%s' && B.NetName == '%s')" % (o, o.replace("OUT", "BEMF"))
                                     for o in _PAIRS) + ")") if PA_SAME_PHASE else ""
DRU = """(version 1)
# JLCPCB 6-layer standard process (via-in-pad POFV), EMC separation rules.
# When several custom rules match, KiCad applies the one that comes last in the file, so the specific
# phase-to-analog rule must follow the general phase-to-signal rule.
(rule "phase_to_signal_tracks"
  (constraint clearance (min %.2fmm))
  (condition "A.NetClass == 'PHASE' && A.Type == 'Track' && B.Type == 'Track' && B.NetClass != 'PHASE' && B.NetClass != 'POWER'"))
(rule "rf_isolation"
  (constraint clearance (min 0.20mm))
  (condition "A.NetClass == 'RF' && B.NetName != 'GND' && B.NetClass != 'RF'"))
(rule "via_in_pad_small"
  (constraint hole_size (min 0.15mm))
  (constraint via_diameter (min 0.25mm))
  (condition "A.Type == 'Via'"))
(rule "phase_to_analog_tracks"
  (constraint clearance (min %.2fmm))
  (condition "A.NetClass == 'PHASE' && B.NetClass == 'ANALOG' && A.Type == 'Track' && B.Type == 'Track'%s"))
""" % (PHASE_CL, PA_SPACING, PA_EXEMPT)


def patch_project(pro_path=None, dru_path=None):
    """Rules into the project: the canonical files, or a variant board's copies when paths are given."""
    pro_path, dru_path = pro_path or PRO_PATH, dru_path or DRU_PATH
    pro = json.load(open(pro_path))
    ds = pro.setdefault("board", {}).setdefault("design_settings", {})
    rules = ds.setdefault("rules", {})
    rules.update({"min_clearance": 0.09, "min_track_width": 0.09, "min_via_diameter": 0.25,
                  "min_through_hole_diameter": 0.15, "min_via_annular_width": 0.05, "min_hole_to_hole": 0.2,
                  "min_hole_clearance": 0.15, "min_copper_edge_clearance": 0.25, "min_microvia_diameter": 0.2,
                  "min_microvia_drill": 0.1, "min_text_height": 0.6, "min_text_thickness": 0.1,
                  "solder_mask_to_copper_clearance": 0.0, "min_silk_clearance": 0.0})
    ds["track_widths"] = [0.0, 0.1, 0.12, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.8]
    ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.25, "drill": 0.15},
                            {"diameter": 0.3, "drill": 0.15}, {"diameter": 0.35, "drill": 0.2},
                            {"diameter": 0.45, "drill": 0.25}]
    sev = ds.setdefault("rule_severities", {})
    sev.update({"silk_overlap": "ignore", "silk_over_copper": "ignore", "silk_edge_clearance": "ignore",
                "lib_footprint_issues": "ignore", "lib_footprint_mismatch": "ignore",
                "text_height": "ignore", "text_thickness": "ignore"})
    ns = pro.setdefault("net_settings", {})
    base = dict(ns.get("classes", [{}])[0]) if ns.get("classes") else {}
    classes, patterns = [], []
    for i, (name, tw, cl, vd, vdr, pats) in enumerate(NETCLASSES):
        c = dict(base)
        c.update({"name": name, "track_width": tw, "clearance": cl, "via_diameter": vd, "via_drill": vdr,
                  "microvia_diameter": 0.25, "microvia_drill": 0.1,
                  "priority": 2147483647 if name == "Default" else i})
        classes.append(c)
        patterns += [{"netclass": name, "pattern": p} for p in pats]
    ns["classes"] = classes
    ns["netclass_patterns"] = patterns
    json.dump(pro, open(pro_path, "w"), indent=2)
    open(dru_path, "w").write(DRU)


def patch_board_text():
    t = open(PCB_PATH, encoding="utf-8").read()
    t = re.sub(r"\t\t\(stackup.*?\n\t\t\)\n", "", t, flags=re.S)
    t = t.replace("(pad_to_mask_clearance 0)", "(pad_to_mask_clearance 0)\n" + STACKUP.rstrip("\n"), 1)
    t = re.sub(r"\(capping no\)", "(capping yes)", t)
    t = re.sub(r"\(filling no\)", "(filling yes)", t)
    t = re.sub(r"\(thickness [0-9.]+\)\n(\t*)\(legacy_teardrops", r"(thickness 0.8)\n\1(legacy_teardrops", t)
    open(PCB_PATH, "w", encoding="utf-8").write(t)


# ---------------------------------------------------------------------------
class Board:
    """Thin editing layer over a loaded pcbnew BOARD (design coords = centre-relative mm)."""

    def __init__(self, path):
        self.b = pcbnew.LoadBoard(path)
        self.fp = {f.GetReference(): f for f in self.b.GetFootprints()}
        self.nets = self.b.GetNetsByName()
        self.added = 0

    def net(self, name):
        return self.nets[name] if hasattr(self.nets, "__getitem__") else self.b.FindNet(name)

    def pad(self, ref, num):
        for p in self.fp[ref].Pads():
            if p.GetNumber() == num:
                return p
        raise KeyError((ref, num))

    # ---- drill spacing: every hole (vias, plated and non-plated pads) keeps MIN_WEB edge to edge ----------
    MIN_WEB = 0.2

    def _holes(self):
        if getattr(self, "_hl", None) is None:
            self._hl = []
            for t in self.b.GetTracks():
                if isinstance(t, pcbnew.PCB_VIA):
                    x, y = P.XY(t.GetPosition())
                    self._hl.append((x, y, P.tomm(t.GetDrillValue())))
            for f in self.b.GetFootprints():
                for p in f.Pads():
                    if p.GetDrillSize().x > 0:
                        x, y = P.XY(p.GetPosition())
                        self._hl.append((x, y, P.tomm(max(p.GetDrillSize().x, p.GetDrillSize().y))))
        return self._hl

    def edge_ok(self, x, y, r, clearance=0.27):
        """copper of radius r at (x, y) keeps `clearance` to the board edge"""
        key = round(r + clearance, 3)
        cache = self.__dict__.setdefault("_edge", {})
        if key not in cache:
            poly = None
            for d in self.b.GetDrawings():
                if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
                    poly = pcbnew.SHAPE_POLY_SET(d.GetPolyShape())
            poly.Deflate(P.mm(key), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, P.mm(0.005))
            cache[key] = poly
        return cache[key].Contains(P.V(x, y))

    def hole_ok(self, x, y, drill):
        return all(math.hypot(hx - x, hy - y) >= (hd + drill) / 2 + self.MIN_WEB for hx, hy, hd in self._holes())

    def via(self, x, y, net, d=0.3, drill=0.15, in_pad=True, locked=True):
        self._holes().append((x, y, drill))
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(P.V(x, y))
        v.SetWidth(P.mm(d))
        v.SetDrill(P.mm(drill))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetNet(self.b.FindNet(net))
        if in_pad:
            v.SetFillingMode(pcbnew.FILLING_MODE_FILLED)
            v.SetCappingMode(pcbnew.CAPPING_MODE_CAPPED)
        v.SetLocked(locked)
        self.b.Add(v)
        self.added += 1
        self._index_add(("via", net, x, y, d / 2), x - d / 2, y - d / 2, x + d / 2, y + d / 2, None)
        return v

    def track(self, pts, w, layer, net, locked=True):
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            t = pcbnew.PCB_TRACK(self.b)
            t.SetStart(P.V(x0, y0))
            t.SetEnd(P.V(x1, y1))
            t.SetWidth(P.mm(w))
            t.SetLayer(layer)
            t.SetNet(self.b.FindNet(net))
            t.SetLocked(locked)
            self.b.Add(t)
            self._index_add(("trk", net, x0, y0, x1, y1, w / 2), min(x0, x1) - w / 2, min(y0, y1) - w / 2,
                            max(x0, x1) + w / 2, max(y0, y1) + w / 2, layer)

    # ---- spatial index (1 mm buckets): pads, tracks, vias -------------------------------------------
    BK = 1.0

    def _index(self):
        if getattr(self, "_idx", None) is not None:
            return self._idx
        self._idx = {}
        for f in self.b.GetFootprints():
            for p in f.Pads():
                if not p.IsOnCopperLayer() and p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                    continue
                bb = p.GetBoundingBox()
                x0, y0 = P.tomm(bb.GetLeft()) - P.OX, P.tomm(bb.GetTop()) - P.OY
                x1, y1 = P.tomm(bb.GetRight()) - P.OX, P.tomm(bb.GetBottom()) - P.OY
                thru = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
                layer = None if thru else (F if p.IsOnLayer(F) else B)
                if f.GetReference().startswith("FID"):      # 1.2 mm mask opening: keep copper/vias out
                    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                    x0, y0, x1, y1 = cx - 0.65, cy - 0.65, cx + 0.65, cy + 0.65
                    layer = None
                self._index_add(("pad", p.GetNetname(), x0, y0, x1, y1), x0, y0, x1, y1, layer)
        for t in self.b.GetTracks():
            if isinstance(t, pcbnew.PCB_VIA):
                x, y = P.XY(t.GetPosition())
                r = P.tomm(t.GetWidth(F)) / 2
                self._index_add(("via", t.GetNetname(), x, y, r), x - r, y - r, x + r, y + r, None)
            else:
                ax, ay = P.XY(t.GetStart())
                bx, by = P.XY(t.GetEnd())
                hw = P.tomm(t.GetWidth()) / 2
                self._index_add(("trk", t.GetNetname(), ax, ay, bx, by, hw), min(ax, bx) - hw, min(ay, by) - hw,
                                max(ax, bx) + hw, max(ay, by) + hw, t.GetLayer())
        return self._idx

    def _index_add(self, item, x0, y0, x1, y1, layer):
        if getattr(self, "_idx", None) is None:
            return
        for gx in range(int(math.floor(x0 / self.BK)), int(math.floor(x1 / self.BK)) + 1):
            for gy in range(int(math.floor(y0 / self.BK)), int(math.floor(y1 / self.BK)) + 1):
                self._idx.setdefault((gx, gy), []).append((layer, item))

    # obstacle test against copper of other nets (pads, vias, tracks) on a layer set
    def clear_of(self, x, y, r, net, layers=None, margin=0.1, cls=False):
        """True when a disc of radius r at (x, y) keeps `margin` from every other net's pad, via and track. With
        cls=True the margin per item is the larger of `margin` and that net's class clearance, and no-net items
        (fiducials) keep NONET_KEEP so nothing lands in their solder-mask opening."""
        idx = self._index()
        reach = r + (max(margin, NONET_KEEP, max(c[2] for c in NETCLASSES)) if cls else margin)
        seen = set()
        for gx in range(int(math.floor((x - reach) / self.BK)), int(math.floor((x + reach) / self.BK)) + 1):
            for gy in range(int(math.floor((y - reach) / self.BK)), int(math.floor((y + reach) / self.BK)) + 1):
                for layer, it in idx.get((gx, gy), ()):
                    if id(it) in seen:
                        continue
                    seen.add(id(it))
                    if it[1] == net and it[1] != "":
                        continue
                    if layers is not None and layer is not None and layer not in layers:
                        continue
                    kind = it[0]
                    rch = reach
                    if cls:
                        rch = r + (NONET_KEEP if not it[1] else max(margin, net_rule(it[1])[1]))
                    if kind == "pad":
                        _, _, bx0, by0, bx1, by1 = it
                        nx, ny = min(max(x, bx0), bx1), min(max(y, by0), by1)
                        if math.hypot(x - nx, y - ny) < rch:
                            return False
                    elif kind == "via":
                        _, _, vx, vy, vr = it
                        if math.hypot(x - vx, y - vy) < rch + vr:
                            return False
                    else:
                        _, _, ax, ay, bx, by, hw = it
                        if seg_dist(x, y, ax, ay, bx, by) < rch + hw:
                            return False
        return True

    def save(self):
        pcbnew.SaveBoard(PCB_PATH, self.b)


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2_ = dx * dx + dy * dy
    t = 0 if L2_ == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2_))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def pad_geom(p):
    x, y = P.XY(p.GetPosition())
    bb = p.GetBoundingBox()
    w, h = P.tomm(bb.GetWidth()), P.tomm(bb.GetHeight())
    return x, y, w, h


# ---------------------------------------------------------------------------
# 2. power fan-out
# ---------------------------------------------------------------------------
def power_fanout(bd, handled=()):
    """GND/VBAT pads -> planes. Try: via-in-pad (filled+capped), then a ring of candidate via spots
    around the pad joined by a short stub on the pad's layer. Narrow QFN pins skip via-in-pad."""
    made, skipped = 0, []
    handled = set(handled)
    for ref, f in bd.fp.items():
        if ref[0] in "JH" or ref.startswith("FID"):
            continue
        fx, fy = P.XY(f.GetPosition())
        for p in f.Pads():
            net = p.GetNetname()
            if net not in ("GND", "VBAT") or not p.IsOnCopperLayer():
                continue
            if p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or (ref, p.GetNumber()) in handled:
                continue
            x, y, w, h = pad_geom(p)
            if w > 1.5 and h > 1.5:           # exposed pads: vias already in footprint or added later
                continue
            layer = F if p.IsOnLayer(F) else B
            if min(w, h) >= 0.28 and bd.clear_of(x, y, 0.125, net, margin=0.16) and bd.hole_ok(x, y, 0.15)                     and bd.edge_ok(x, y, 0.125):
                bd.via(x, y, net, d=0.25, drill=0.15)
                made += 1
                continue
            # candidate spots: outward first, then a ring
            ox, oy = x - fx, y - fy
            n0 = math.hypot(ox, oy) or 1.0
            base = math.atan2(oy / n0, ox / n0)
            r0 = max(w, h) / 2 + 0.22
            done = False
            for r in (r0, r0 + 0.12, r0 + 0.25, r0 + 0.4, r0 + 0.6):
                for k in range(24):
                    a = base + (k // 2 + 1) * (math.pi / 12) * (1 if k % 2 else -1) if k else base
                    vx, vy = x + r * math.cos(a), y + r * math.sin(a)
                    if not bd.clear_of(vx, vy, 0.125, net, margin=0.16) or not bd.hole_ok(vx, vy, 0.15)                             or not bd.edge_ok(vx, vy, 0.125):
                        continue
                    sw = min(min(w, h), 0.2)
                    if not all(bd.clear_of(x + (vx - x) * t, y + (vy - y) * t, sw / 2, net, layers=[layer],
                                           margin=0.16) and bd.edge_ok(x + (vx - x) * t, y + (vy - y) * t, sw / 2)
                               for t in (0.5, 0.75, 1.0)):
                        continue
                    bd.track([(x, y), (vx, vy)], sw, layer, net)
                    bd.via(vx, vy, net, d=0.25, drill=0.15, in_pad=False)
                    made += 1
                    done = True
                    break
                if done:
                    break
            if not done:
                # fallback: tie the pad to the nearest existing same-net via with a straight track
                cand = sorted((math.hypot(vx - x, vy - y), vx, vy) for (vx, vy, vn) in
                              [(P.XY(v.GetPosition()) + (v.GetNetname(),)) for v in bd.b.GetTracks()
                               if isinstance(v, pcbnew.PCB_VIA)] if vn == net)
                for dist, vx, vy in cand[:8]:
                    if dist > 2.5:
                        break
                    sw = min(min(w, h), 0.2)
                    n_ = max(2, int(dist / 0.05))
                    if all(bd.clear_of(x + (vx - x) * k / n_, y + (vy - y) * k / n_, sw / 2, net, layers=[layer],
                                       margin=0.16) for k in range(1, n_)):
                        bd.track([(x, y), (vx, vy)], sw, layer, net)
                        made += 1
                        done = True
                        break
            if not done:
                skipped.append("%s.%s" % (ref, p.GetNumber()))
    print("power fan-out: %d vias, skipped %d %s" % (made, len(skipped), skipped[:20]))


def drv_power(bd, refs=("U11", "U12", "U13", "U14")):
    """DRV8311 power pins, done by hand before the generic fan-out:
    - every GND pin is tied straight into the exposed pad (which carries the thermal vias),
    - the two VM pins at the package corner are joined around the corner, with up to 3 VBAT vias there,
    - 4 extra filled+capped vias go into the exposed pad (9 total) for heat.
    Returns the set of (ref, pad number) handled here."""
    done = set()
    nv = 0
    for ref in refs:
        f = bd.fp[ref]
        layer = F if f.GetLayer() == F else B
        cx, cy = P.XY(f.GetPosition())
        ep = [p for p in f.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and P.tomm(p.GetSize().x) > 1.5][0]
        ex, ey = P.XY(ep.GetPosition())
        eh = P.tomm(ep.GetSize().x) / 2
        vm = []
        for p in f.Pads():
            if p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or P.tomm(p.GetSize().x) > 1.5:
                continue
            net = p.GetNetname()
            x, y, w, h = pad_geom(p)
            if w >= h:
                nx_, ny_ = (1.0 if x > cx else -1.0), 0.0
            else:
                nx_, ny_ = 0.0, (1.0 if y > cy else -1.0)
            L = max(w, h)
            if net == "GND":                       # pin -> exposed pad (inward)
                d_in = abs((x - ex) * nx_ + (y - ey) * ny_) - eh + 0.15
                bd.track([(x, y), (x - nx_ * d_in, y - ny_ * d_in)], 0.2, layer, "GND")
                done.add((ref, p.GetNumber()))
            elif net == "VBAT":
                vm.append((p, x, y, nx_, ny_, L))
                done.add((ref, p.GetNumber()))
        if len(vm) == 2:
            (pa, xa, ya, nxa, nya, La), (pb, xb, yb, nxb, nyb, Lb) = vm
            ea = (xa + nxa * La / 2, ya + nya * La / 2)
            eb = (xb + nxb * Lb / 2, yb + nyb * Lb / 2)
            # corner point beyond both pins: each coordinate comes from the pin whose outward normal lies on that
            # axis (pin on the x-facing edge gives x, pin on the y-facing edge gives y)
            q = (ea[0] if nxa != 0 else eb[0], ea[1] if nya != 0 else eb[1])
            q = (q[0] + (nxa + nxb) * 0.05, q[1] + (nya + nyb) * 0.05)
            bd.track([(xa, ya), ea, q, eb, (xb, yb)], 0.25, layer, "VBAT")
            cands = [(q[0] + (nxa + nxb) * 0.32, q[1] + (nya + nyb) * 0.32),
                     (q[0] + nxa * 0.45 - nxb * 0.15, q[1] + nya * 0.45 - nyb * 0.15),
                     (q[0] + nxb * 0.45 - nxa * 0.15, q[1] + nyb * 0.45 - nya * 0.15),
                     (q[0] + nxa * 0.75, q[1] + nya * 0.75), (q[0] + nxb * 0.75, q[1] + nyb * 0.75),
                     (q[0] + (nxa + nxb) * 0.62, q[1] + (nya + nyb) * 0.62),
                     (q[0] + nxa * 0.75 + nxb * 0.35, q[1] + nya * 0.75 + nyb * 0.35),
                     (q[0] + nxb * 0.75 + nxa * 0.35, q[1] + nyb * 0.75 + nya * 0.35)]
            pads_xy = [P.XY(pp.GetPosition()) for pp in f.Pads()]
            cands = [c for c in cands if min(math.hypot(c[0] - a, c[1] - b_) for a, b_ in pads_xy) > 0.45]
            k = 0
            for (vx, vy) in cands:
                if k >= 3:
                    break
                if bd.clear_of(vx, vy, 0.125, "VBAT", margin=0.16) and bd.hole_ok(vx, vy, 0.15) and                         bd.edge_ok(vx, vy, 0.125):
                    bd.track([q, (vx, vy)], 0.25, layer, "VBAT")
                    bd.via(vx, vy, "VBAT", d=0.25, drill=0.15, in_pad=False)
                    k += 1
                    nv += 1
            if k == 0:
                done -= {(ref, pa.GetNumber()), (ref, pb.GetNumber())}
        for dx, dy in ((0.45, 0.45), (-0.45, 0.45), (0.45, -0.45), (-0.45, -0.45)):
            if bd.hole_ok(ex + dx, ey + dy, 0.15):
                bd.via(ex + dx, ey + dy, "GND", d=0.25, drill=0.15, in_pad=True)
    print("DRV power hookup: GND pins into the exposed pads, %d VM vias, EP via fields topped up" % nv)
    return done


def escape_stubs(bd, refs=("U11", "U12", "U13", "U14")):
    """Short straight escapes from every signal pin of the 0.4 mm-pitch DRV8311s (router starts outside)."""
    n = 0
    for ref in refs:
        f = bd.fp[ref]
        fx, fy = P.XY(f.GetPosition())
        layer = F if f.GetLayer() == F else B
        for p in f.Pads():
            net = p.GetNetname()
            x, y, w, h = pad_geom(p)
            if not net or net in ("GND", "VBAT") or (w > 1.5 and h > 1.5):
                continue
            if w >= h:
                ux, uy = (1 if x > fx else -1), 0
            else:
                ux, uy = 0, (1 if y > fy else -1)
            L = (w if ux else h) / 2 + 0.35
            ex, ey = x + ux * L, y + uy * L
            if bd.clear_of(ex, ey, 0.1, net, layers=[layer], margin=0.1):
                bd.track([(x, y), (ex, ey)], 0.19 if net.startswith("M") and "_OUT" in net else 0.15, layer, net)
                n += 1
    print("DRV escape stubs:", n)


def ep_vias(bd, ref, pitch=1.0, n=3, d=0.3):
    for p in bd.fp[ref].Pads():
        x, y, w, h = pad_geom(p)
        if w > 1.5 and h > 1.5 and p.GetNetname():
            k = 0
            for i in range(n):
                for j in range(n):
                    vx, vy = x + (i - (n - 1) / 2) * pitch, y + (j - (n - 1) / 2) * pitch
                    if not bd.hole_ok(vx, vy, 0.15):
                        continue
                    bd.via(vx, vy, p.GetNetname(), d=d, drill=0.15)
                    k += 1
            print("%s exposed pad: %d GND vias" % (ref, k))


# ---------------------------------------------------------------------------
# 3. critical routes
# ---------------------------------------------------------------------------
def route_phases(bd):
    done, left = 0, []
    for m in (1, 2, 3, 4):
        u, j = "U%d" % (10 + m), "J%d" % (10 + m)
        layer = F if bd.fp[u].GetLayer() == F else B
        ux_, uy_ = P.XY(bd.fp[u].GetPosition())
        for pin, pad in (("10", "1"), ("11", "2"), ("12", "3")):
            sp = bd.pad(u, pin)
            net = sp.GetNetname()
            x, y, w, h = pad_geom(sp)
            # exit perpendicular to the package edge
            ox, oy = (1 if x > ux_ else -1, 0) if w > h else (0, 1 if y > uy_ else -1)
            e = (w if ox else h) / 2 + 0.25
            x1, y1 = x + ox * e, y + oy * e
            tx, ty = P.XY(bd.pad(j, pad).GetPosition())
            ok = all(bd.clear_of(x1 + (tx - x1) * k / 10, y1 + (ty - y1) * k / 10, PHASE_W / 2, net,
                                 layers=[layer], margin=0.2) for k in range(1, 10))
            if ok:
                bd.track([(x, y), (x1, y1)], 0.2, layer, net)
                bd.track([(x1, y1), (tx, ty)], PHASE_W, layer, net)
                done += 1
            else:
                left.append(net)
    print("phase traces: %d hand-routed, left to router: %s" % (done, left))


def phase_power_legs(bd, w=0.19):
    """Motor-current leg of every phase as fixed copper. It is a straight track on the driver's own outer layer,
    from the end of the OUT pin's escape stub to the motor pad (0.85-1.04 mm). The 4 A peak phase current then never
    depends on the router's layer choice. The router only adds the BEMF sense branch to the divider (microamps),
    and the finish stage widens the legs toward 0.5 mm."""
    done, left = 0, []
    for m in (1, 2, 3, 4):
        u, j = "U%d" % (10 + m), "J%d" % (10 + m)
        layer = F if bd.fp[u].GetLayer() == F else B
        for pin, pad in (("10", "1"), ("11", "2"), ("12", "3")):
            sp = bd.pad(u, pin)
            net = sp.GetNetname()
            px, py = P.XY(sp.GetPosition())
            ends = [P.XY(e) for t in bd.b.GetTracks() if t.GetNetname() == net and t.IsLocked()
                    and not isinstance(t, pcbnew.PCB_VIA) for e in (t.GetStart(), t.GetEnd())]
            sx, sy = max(ends, key=lambda e: (e[0] - px) ** 2 + (e[1] - py) ** 2) if ends else (px, py)
            tx, ty = P.XY(bd.pad(j, pad).GetPosition())
            n = 20
            if all(bd.clear_of(sx + (tx - sx) * k / n, sy + (ty - sy) * k / n, w / 2, net, layers=[layer],
                               margin=0.15) for k in range(1, n)):
                bd.track([(sx, sy), (tx, ty)], w, layer, net)        # locked prep copper
                done += 1
            else:
                left.append(net)
    print("phase power legs: %d of 12 laid as fixed copper%s" % (
        done, (", left to the router: %s" % left) if left else ""))
    return left


def route_rf(bd):
    """Straight 50R line along the placed chain + GND via fence (both sides)."""
    nodes = ["RF_IO", "RF_N1", "RF_N2", "RF_N3", "ANT"]
    rfl = F if bd.fp["U4"].GetLayer() == F else B             # RF chain on the radio's side
    import nanodrone_pcb as NP
    nx4, ny4, px4, py4 = NP.rfio_normal(bd.b)
    rx, ry = px4 + nx4 * NP.RF_STUB, py4 + ny4 * NP.RF_STUB    # line start, RF_STUB out of the RFIO pad
    ax, ay = P.XY(bd.pad("J2", "1").GetPosition())
    L = math.hypot(ax - rx, ay - ry)
    ux, uy = (ax - rx) / L, (ay - ry) / L
    vx, vy = -uy, ux
    # every pad on RF nets, projected on the line
    pads = [(0.0, 0.0, rx, ry, "RF_IO")]               # virtual on-line point: end of the RFIO stub
    bd.track([(px4, py4), (rx, ry)], RF_W, rfl, "RF_IO")  # perpendicular stub out of the pin row
    for f in bd.b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() in nodes and not (f.GetReference() == "U4" and p.GetNumber() == "22"):
                x, y = P.XY(p.GetPosition())
                s = (x - rx) * ux + (y - ry) * uy
                d = (x - rx) * vx + (y - ry) * vy
                pads.append((s, d, x, y, p.GetNetname()))
    # per net: connect its pads; on-line pads joined along the line, off-line pads (shunt C, C4) with a stub
    # perpendicular to the line from the pad's projection (which lies inside the adjacent series pad)
    drawn = set()

    def seg(net, pts_):
        for (xa, ya), (xb, yb) in zip(pts_[:-1], pts_[1:]):
            if math.hypot(xb - xa, yb - ya) < 1e-3:
                continue
            key = (net, round(min(xa, xb), 3), round(min(ya, yb), 3), round(max(xa, xb), 3), round(max(ya, yb), 3))
            if key in drawn:
                continue
            drawn.add(key)
            bd.track([(xa, ya), (xb, yb)], RF_W, rfl, net)
    for net in nodes:
        pts = sorted([pp for pp in pads if pp[4] == net])
        online = [pp for pp in pts if abs(pp[1]) < 0.15]
        for (s0, d0, x0, y0, _), (s1, d1, x1, y1, _) in zip(online[:-1], online[1:]):
            seg(net, [(x0, y0), (x1, y1)])
        for (s1, d1, x1, y1, _) in [pp for pp in pts if abs(pp[1]) >= 0.15]:
            mx, my = rx + ux * s1, ry + uy * s1
            seg(net, [(mx, my), (x1, y1)])
            if online and not any(abs(o[0] - s1) < 0.4 for o in online):
                # projection not inside an on-line pad: join it to the nearest on-line pad along the line
                o = min(online, key=lambda q: abs(q[0] - s1))
                seg(net, [(o[2], o[3]), (mx, my)])
    # shunt-cap ground pads: via in pad
    for f in bd.b.GetFootprints():
        ps = list(f.Pads())
        if len(ps) == 2 and {ps[0].GetNetname(), ps[1].GetNetname()} & set(nodes) and \
                "GND" in {ps[0].GetNetname(), ps[1].GetNetname()}:
            g = ps[0] if ps[0].GetNetname() == "GND" else ps[1]
            gx, gy = P.XY(g.GetPosition())
            if bd.clear_of(gx, gy, 0.15, "GND", margin=0.16) and bd.hole_ok(gx, gy, 0.15) and bd.edge_ok(gx, gy, 0.15):
                bd.via(gx, gy, "GND", d=0.3, drill=0.15)
    # via fence, 1.0 mm pitch along both sides, as close as the parts allow. 1.0 mm is lambda/70 at 2.45 GHz and
    # lambda/23 at the 3rd harmonic. 0.5 mm built an all-layer wall that cut the M4 ESC MCU off from its driver:
    # 0.25 mm between via pads cannot pass a 0.1 mm track with 0.1 mm clearance.
    n = 0
    s = 0.5
    FENCE_PITCH = 1.0
    while s < L - 0.3:
        for side in (+1, -1):
            for extra in (0.0, 0.25, 0.5):
                off = side * (RF_W / 2 + RF_GAP + 0.25 + extra)
                x, y = rx + ux * s + vx * off, ry + uy * s + vy * off
                if bd.clear_of(x, y, 0.125, "GND", margin=0.17) and bd.hole_ok(x, y, 0.15) and bd.edge_ok(x, y, 0.125):
                    bd.via(x, y, "GND", d=0.25, drill=0.15, in_pad=False)
                    n += 1
                    break
        s += FENCE_PITCH
    print("RF line %.2f mm routed at %.2f mm, fence vias %d" % (L, RF_W, n))


# ---------------------------------------------------------------------------
# 4. autorouting
# ---------------------------------------------------------------------------
CLASS_OF = {}
for _name, _tw, _cl, _vd, _vdr, _pats in NETCLASSES:
    if _name == "RAIL" and os.environ.get("NANODRONE_RAIL_W"):
        # router width for the low-current rails (all <= 0.15 A); +3V0 is widened again by the finish stage
        _tw = float(os.environ["NANODRONE_RAIL_W"])
    if _name == "PHASE" and os.environ.get("NANODRONE_PHASE_W"):
        # with the motor-current legs pre-routed by phase_power_legs(), the router only lays the BEMF sense
        # branch (OUT -> divider top resistor, microamps), which can be narrow; the finish stage widens anyway
        _tw = float(os.environ["NANODRONE_PHASE_W"])
    for _p in _pats:
        CLASS_OF[_p] = (_tw, _cl)


def net_rule(net):
    for pat, rule in CLASS_OF.items():
        if re.fullmatch(pat.replace("?", ".").replace("+", r"\+"), net):
            return rule
    return NETCLASSES[0][1], NETCLASSES[0][2]      # Default class


def _pad_layers(p):
    if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH,):
        return {F, B, L3}
    return {F} if p.IsOnLayer(F) else {B}


def direct_route(bd, skip=("GND", "VBAT")):
    """Route the easy connections geometrically: per net, join connected clusters (union-find over pads,
    tracks, vias) along the shortest pad pair, trying a straight segment, then two-segment doglegs
    (axis-aligned and 45 deg), on a layer both pads share. Every candidate is checked against all
    other-net copper with the netclass clearance."""
    b = bd.b
    pads_by_net = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            n = p.GetNetname()
            if n and n not in skip and p.IsOnCopperLayer():
                pads_by_net.setdefault(n, []).append(p)
    routed, failed = 0, []
    for net, pads in sorted(pads_by_net.items()):
        if len(pads) < 2:
            continue
        w, cl = net_rule(net)
        # union-find of this net's pads through existing copper
        items = list(pads) + [t for t in b.GetTracks() if t.GetNetname() == net]
        parent = list(range(len(items)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def touch(a, c):
            sa = a.GetEffectiveShape(F) if not isinstance(a, pcbnew.PAD) else a.GetEffectiveShape(next(iter(_pad_layers(a))))
            for lay in (F, B, L3, L2, L4, L5):
                if a.IsOnLayer(lay) and c.IsOnLayer(lay):
                    if a.GetEffectiveShape(lay).Collide(c.GetEffectiveShape(lay), 0):
                        return True
            return False

        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if find(i) != find(j) and touch(items[i], items[j]):
                    parent[find(i)] = find(j)
        clusters = {}
        for i, it in enumerate(items[:len(pads)]):
            clusters.setdefault(find(i), []).append(it)
        groups = list(clusters.values())
        while len(groups) > 1:
            best = None
            for gi in range(len(groups)):
                for gj in range(gi + 1, len(groups)):
                    for pa in groups[gi]:
                        for pb in groups[gj]:
                            common = _pad_layers(pa) & _pad_layers(pb)
                            if not common:
                                continue
                            ax, ay = P.XY(pa.GetPosition())
                            bx, by = P.XY(pb.GetPosition())
                            dd = math.hypot(ax - bx, ay - by)
                            if best is None or dd < best[0]:
                                best = (dd, gi, gj, pa, pb, common)
            if best is None:
                break
            dd, gi, gj, pa, pb, common = best
            ax, ay = P.XY(pa.GetPosition())
            bx, by = P.XY(pb.GetPosition())
            ok = False
            for layer in sorted(common, key=lambda l: (l != F and l != B, l)):
                paths = [[(ax, ay), (bx, by)]]
                dx, dy = bx - ax, by - ay
                paths += [[(ax, ay), (bx, ay), (bx, by)], [(ax, ay), (ax, by), (bx, by)]]
                m = min(abs(dx), abs(dy))
                sx, sy = math.copysign(1, dx), math.copysign(1, dy)
                paths += [[(ax, ay), (ax + sx * m, ay + sy * m), (bx, by)],
                          [(ax, ay), (bx - sx * m, by - sy * m), (bx, by)]]
                for path in paths:
                    if seg_path_clear(bd, path, w, cl, net, layer, (pa, pb)):
                        bd.track(path, w, layer, net, locked=False)
                        ok = True
                        break
                if ok:
                    break
            if ok:
                routed += 1
                groups[gi] += groups[gj]
                del groups[gj]
            else:
                failed.append(net)
                # give up on this pair; drop the farther group so the loop ends
                del groups[gj]
    print("direct router: %d connections routed, %d left to Freerouting" % (routed, len(failed)))
    return failed


def net_clusters(b, net, pads):
    """Union-find of a net's pads, tracks and vias by copper contact -> list of clusters (lists of items)."""
    items = list(pads) + [t for t in b.GetTracks() if t.GetNetname() == net]
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    lays = (F, B, L3, L2, L4, L5)
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if find(i) == find(j):
                continue
            a, c = items[i], items[j]
            for lay in lays:
                if a.IsOnLayer(lay) and c.IsOnLayer(lay) and a.GetEffectiveShape(lay).Collide(
                        c.GetEffectiveShape(lay), 0):
                    parent[find(i)] = find(j)
                    break
    cl = {}
    for i, it in enumerate(items):
        cl.setdefault(find(i), []).append(it)
    return list(cl.values())


ROUTE_LAYERS = [F, L3, B]       # grid routing layers; NANODRONE_L4=1 adds L4 (restricted, see grid_route)
L4_CLASSES = tuple(os.environ.get("NANODRONE_L4_CLASSES", "Default,DSHOT,RAIL").split(","))  # never phase, RF, crystal
L4_RF_KEEPOUT = 0.5             # mm around the L5 relief cut-outs under the RF pads (there L4 is the reference;
                                # > 3x the L4-L5-L6 dielectric stack, so no L4 track sees the pad's field)


def item_points(it):
    """Routing access points of an item: [(layer, x, y)] on the grid routing layers."""
    x, y = P.XY(it.GetPosition()) if not isinstance(it, pcbnew.PCB_TRACK) or isinstance(it, pcbnew.PCB_VIA) \
        else P.XY(it.GetStart())
    if isinstance(it, pcbnew.PCB_VIA):
        return [(l, x, y) for l in ROUTE_LAYERS]
    if isinstance(it, pcbnew.PAD):
        if it.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            return [(l, x, y) for l in ROUTE_LAYERS]
        return [(F if it.IsOnLayer(F) else B, x, y)]
    if it.GetLayer() in ROUTE_LAYERS:
        ex, ey = P.XY(it.GetEnd())
        return [(it.GetLayer(), x, y), (it.GetLayer(), ex, ey)]
    return []


def grid_route(bd, skip=("GND", "VBAT"), max_iter=400, max_rips=10):
    """Grid maze routing of every non-plane net, with rip-up and reroute.

    1. Nets are routed one by one, shortest MST first. Inside a net, clusters join in Prim order and each new
       route becomes a start for the next connection.
    2. A net that fails is retried after its blockers are found: the failed connection is re-routed in soft mode
       (routed copper of other nets passable at a penalty, fixed copper still hard). The nets whose routes the
       soft path crosses are ripped up, the failed net is routed first, then the ripped nets are re-routed. Cells
       where the soft path crossed get a growing history cost (PathFinder-style), which breaks cycles.
    3. Only the final set of routes is written to the board."""
    import gridroute as GR
    b = bd.b
    # Hybrid/repair mode (NANODRONE_SEED=1): the board already carries autorouted, unlocked copper (Freerouting).
    # That copper is lifted off the board and loaded as ripable routes. Only nets that are incomplete or named in
    # NANODRONE_RIP (DRC offenders) are re-routed; the rip-up loop may also rip and re-route any seeded net.
    seed = os.environ.get("NANODRONE_SEED") == "1"
    rip_first = {n for n in os.environ.get("NANODRONE_RIP", "").split(",") if n}
    seeded, incomplete0 = {}, set()
    if seed:
        pads_net = {}
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() and p.GetNetname() not in skip and p.IsOnCopperLayer():
                    pads_net.setdefault(p.GetNetname(), []).append(p)
        for net, pads in pads_net.items():
            if len(pads) > 1 and len(net_clusters(b, net, pads)) > 1:
                incomplete0.add(net)
        # NANODRONE_KEEP: these nets keep their copper as fixed board copper (not lifted), so an incomplete net
        # only gets its missing links routed and its partial tree stays as it is
        keep = {n for n in os.environ.get("NANODRONE_KEEP", "").split(",") if n}
        for t in list(b.GetTracks()):
            n = t.GetNetname()
            if t.IsLocked() or not n or n in skip or n in keep:
                continue
            if isinstance(t, pcbnew.PCB_VIA):
                x, y = P.XY(t.GetPosition())
                seeded.setdefault(n, ([], []))[1].append((x, y, t.GetWidth(F) / 2e6))
            elif t.GetLayer() in (F, L3, B, L4):
                (x0, y0), (x1, y1) = P.XY(t.GetStart()), P.XY(t.GetEnd())
                seeded.setdefault(n, ([], []))[0].append((t.GetLayer(), x0, y0, x1, y1, t.GetWidth() / 2e6))
            b.Delete(t)
        print("seed mode: %d nets with autorouted copper lifted, %d incomplete %s, %d to rip %s" % (
            len(seeded), len(incomplete0), sorted(incomplete0), len(rip_first), sorted(rip_first)), flush=True)
    global ROUTE_LAYERS
    use_l4 = os.environ.get("NANODRONE_L4") == "1"
    ROUTE_LAYERS = [F, L3, L4, B] if use_l4 else [F, L3, B]
    grid = GR.Grid(b, ROUTE_LAYERS, res=float(os.environ.get("NANODRONE_GRID_RES", "0.05")))
    if use_l4:
        # L4 is the VBAT plane (hub) / GND fill: a restricted, expensive escape layer. The RF feed on L6 is
        # shielded from L4 by the L5 ground everywhere except the L5 relief cut-outs under the RF pads, where
        # L4 becomes the reference: those (plus a margin) are L4 keep-outs.
        n_ko = 0
        ref_layer = L5 if bd.fp["U4"].GetLayer() == B else L2      # the radio's reference plane layer
        for z in b.Zones():
            if not (z.GetIsRuleArea() and z.IsOnLayer(ref_layer)):
                continue
            o = z.Outline().Outline(0)
            pts = [P.XY(o.CPoint(k)) for k in range(o.PointCount())]
            grid._fill_poly(grid.owner[L4], pts, -1)
            for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
                grid.keepout(L4, x0, y0, x1, y1, L4_RF_KEEPOUT, vias=False)   # L4 tracks only: a via is too small to matter
            n_ko += 1
        grid.layer_cost = {L4: float(os.environ.get("NANODRONE_L4_COST", "3.0"))}
        print("grid router: L4 enabled for %s (cost x%.1f), %d RF keep-outs" % (
            "/".join(L4_CLASSES), grid.layer_cost[L4], n_ko), flush=True)
    grid.init_rrr()

    # NANODRONE_NOLAYER="NET:B;NET2:F,L3": layers a net must not use (e.g. keep a track off B so the B pour can
    # flow through where it would otherwise wall off a pour island)
    lname = {"F": F, "L3": L3, "L4": L4, "B": B}
    nolayer = {}
    for item_ in os.environ.get("NANODRONE_NOLAYER", "").split(";"):
        if ":" in item_:
            n_, ls_ = item_.split(":", 1)
            nolayer[n_] = {lname[x_] for x_ in ls_.split(",") if x_ in lname}

    def allowed(net):
        lays = None
        if use_l4 and net_class(net) not in L4_CLASSES:
            lays = [F, L3, B]
        if net in nolayer:
            lays = [l_ for l_ in (lays or ROUTE_LAYERS) if l_ not in nolayer[net]]
        return lays
    print("grid router: rasters built (%d x %d cells, 3 layers)" % (grid.n, grid.n), flush=True)
    code = {}
    pads_by_net = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            n = p.GetNetname()
            if n and n not in skip and p.IsOnCopperLayer():
                pads_by_net.setdefault(n, []).append(p)
                code[n] = p.GetNetCode()
    phase = {code[n] for n in code if re.fullmatch(r"M\d_OUT[ABC]", n)}
    analog = {code[n] for n in code if re.fullmatch(r"(M\d_(BEMF|ISENS)[ABC]|VBAT_SENSE)", n)}
    rf = {code[n] for n in code if n in ("RF_IO", "RF_N1", "RF_N2", "RF_N3", "ANT")}
    clusters = {}
    for net, pads in pads_by_net.items():
        if len(pads) < 2:
            continue
        groups = net_clusters(b, net, pads)
        if len(groups) < 2:
            continue
        pts = [[p for it in g for p in item_points(it)] for g in groups]
        pts = [p if p else [(F, *P.XY(g[0].GetPosition()))] for p, g in zip(pts, groups)]
        clusters[net] = pts

    def prim_len(pts):
        inside, tot = {0}, 0.0
        while len(inside) < len(pts):
            best = min(((math.hypot(xa - xb, ya - yb), j) for i in inside for j in range(len(pts)) if j not in inside
                        for (_la, xa, ya) in pts[i] for (_lb, xb, yb) in pts[j]), key=lambda v: v[0])
            tot += best[0]
            inside.add(best[1])
        return tot
    order = sorted(clusters, key=lambda n: prim_len(clusters[n]),
                   reverse=os.environ.get("NANODRONE_ORDER", "short") == "long")
    if os.environ.get("NANODRONE_PRIO") == "1":
        # critical nets first, so they get the short, direct paths: phases (EMC loop area, 0.30 mm rule),
        # then the analog sense lines that must keep clear of them, then the rails; the rest shortest first
        rank = {"PHASE": 0, "ANALOG": 1, "RAIL": 2, "XTAL": 2}
        first = [n for n in os.environ.get("NANODRONE_FIRST", "").split(",") if n]   # e.g. the +3V0 tree
        order = sorted(order, key=lambda n: (-1 if n in first else rank.get(net_class(n), 3)))
    # NANODRONE_ORDER: these nets go first, in exactly this order (local finishing: try permutations of a few nets)
    exact = [n for n in os.environ.get("NANODRONE_ORDER", "").split(",") if n]
    if exact:
        order = [n for n in exact if n in order] + [n for n in order if n not in exact]
    via_cost = float(os.environ.get("NANODRONE_VIA_COST", "12"))
    full_order = list(order)    # every routable net in priority order (seed mode narrows "order")
    nconn = sum(len(v) - 1 for v in clusters.values())
    print("grid router: %d nets, %d connections" % (len(order), nconn), flush=True)

    # KiCad's clearance between two nets is the larger of their netclass clearances: every class with a larger
    # clearance than the routed net's own becomes an extra rule against that class's copper (pads included)
    cls_codes, cls_cl = {}, {}
    for n_, c_ in code.items():
        k_ = net_class(n_)
        cls_codes.setdefault(k_, set()).add(c_)
        cls_cl[k_] = net_rule(n_)[1]
    net_of_code = {c_: n_ for n_, c_ in code.items()}
    # Same-phase exception (PA_SAME_PHASE): Mx_OUTy and its own divided node Mx_BEMFy meet at the divider resistor.
    # The coupling between them only adds a fast edge to that phase's own divided voltage, which is sampled
    # >= 2.8 us after every edge, so the pair keeps the class clearance and not the phase-to-analog spacing.
    # The .kicad_dru rule excludes the same pairs.
    pa_partner = {}
    if PA_SAME_PHASE:
        for n_, c_ in code.items():
            m_ = re.fullmatch(r"(M\d)_OUT([ABC])", n_)
            if m_ and "%s_BEMF%s" % m_.groups() in code:
                b_ = code["%s_BEMF%s" % m_.groups()]
                pa_partner[c_], pa_partner[b_] = {b_}, {c_}

    def specials(nc):
        sp_ = []
        if nc in analog:
            sp_.append((phase - pa_partner.get(nc, set()), PA_SPACING, True))  # track-to-track only, not pads
        if nc in phase:
            sp_.append((analog - pa_partner.get(nc, set()), PA_SPACING, True))
        if nc not in rf:
            sp_.append((rf, 0.20, False))
        own_cl = net_rule(net_of_code[nc])[1] if nc in net_of_code else 0.10
        for k_, cl_ in cls_cl.items():
            if cl_ > own_cl + 1e-9 and k_ != "RF" and nc not in cls_codes[k_]:
                sp_.append((cls_codes[k_], cl_, False))
        return sp_

    def route_net(net, soft_pen=None):
        """Route all connections of a net. Returns (ok, failed_connection or None). With soft_pen the routes may
        overlap other nets' (ripable) routes at that penalty per cell - the negotiated-congestion mode."""
        w, cl = net_rule(net)
        nc = code[net]
        grid.special = specials(nc)
        pts = clusters[net]
        # a cluster that cannot be reached yet is deferred and retried once the tree has grown (the bigger tree
        # offers other start points, e.g. a bus pin reached from the far side). If nothing at all connects to the
        # seed cluster (a boxed-in pin), the tree is started again from the next cluster, so one blocked pin costs
        # only its own leg and not the whole net
        first_fail = None
        for seed in range(len(pts)):
            inside = {seed}
            reach = list(pts[seed])
            deferred = set()
            while len(inside) < len(pts):
                best = None
                for j in range(len(pts)):
                    if j in inside or j in deferred:
                        continue
                    for (la, xa, ya) in reach:
                        for (lb, xb, yb) in pts[j]:
                            d = math.hypot(xa - xb, ya - yb)
                            if best is None or d < best[0]:
                                best = (d, j)
                if best is None:
                    break
                j = best[1]
                path = grid.route(nc, reach, pts[j], w / 2, cl, via_r=0.125, via_cost=via_cost,
                                  allowed_layers=allowed(net), soft=soft_pen is not None, soft_pen=soft_pen or 25.0)
                if not path:
                    deferred.add(j)
                    first_fail = first_fail or (reach, pts[j])
                    continue
                deferred.clear()
                polylines, vias = GR.simplify(path)
                segs = [(layer, x0, y0, x1, y1, w / 2) for layer, poly in polylines
                        for (x0, y0), (x1, y1) in zip(poly[:-1], poly[1:])]
                if not segs and not vias:
                    # start and goal in the same grid cell: bridge the exact nearest points with one short track so
                    # the copper really touches (the cell-centre path alone carries no copper)
                    l0, x0_, y0_ = path[0]
                    near = lambda p: (p[0] != l0, math.hypot(p[1] - x0_, p[2] - y0_))
                    a_, b_ = min(reach, key=near), min(pts[j], key=near)
                    segs = [(l0, a_[1], a_[2], b_[1], b_[2], w / 2)]
                grid.add_route(nc, segs, [(vx, vy, 0.125) for vx, vy in vias], name=net, w=w, path=path)
                inside.add(j)
                reach = reach + list(pts[j]) + [(l, x, y) for l, x, y in path[::3]]
            if len(inside) == len(pts):
                return True, None
            if len(inside) > 1:
                return False, first_fail              # a partial tree stands; the rest is left to the repair
        return False, first_fail

    def conflicts_of(rid):
        """Other-net routes inside this route's clearance envelope (incl. the special rules), and the hot cells."""
        R = grid.routes[rid]
        if "path" not in R:
            return set(), []
        w, cl = net_rule(R["name"])
        spc = specials(R["net"])                      # (codes, clearance, tracks_only), as A* applies them
        blk, hot = grid.blockers(R["path"], R["net"], w / 2, cl, 0.125, spc, self_rid=rid)
        return {k for k in blk if k in grid.routes and grid.routes[k]["net"] != R["net"]}, hot

    def negotiate(failed):
        """PathFinder-style negotiated congestion: nets in conflict are re-routed with overlaps allowed at a
        present-congestion penalty that grows every iteration, on top of a history cost that accumulates where
        overlaps keep happening. Ends when no route overlaps another; nets still in conflict are ripped and
        returned as failed (for the rip-up loop)."""
        pres = float(os.environ.get("NANODRONE_PRES0", "3.0"))
        grow = float(os.environ.get("NANODRONE_PRES_GROW", "1.4"))
        hinc = float(os.environ.get("NANODRONE_HIST_INC", "0.8"))
        one_side = os.environ.get("NANODRONE_NEG_ONESIDE") == "1"
        patience = int(os.environ.get("NANODRONE_NEG_PATIENCE", "0"))     # 0 = run all iterations
        todo, dead = list(failed), set()
        bad = set(todo)
        best_n, stale = None, 0
        import collections as _co
        for itn in range(1, int(os.environ.get("NANODRONE_NEG_ITERS", "40")) + 1):
            t0 = time.time()
            for net in todo:
                grid.rip_net(code[net])
                ok, _fc = route_net(net, soft_pen=pres)
                if not ok:
                    dead.add(net)                     # blocked by fixed copper even with overlaps allowed
                    grid.rip_net(code[net])
                else:
                    dead.discard(net)
            bad, n_conf, pairs = set(), 0, set()
            for rid in list(grid.routes):
                cs, hot = conflicts_of(rid)
                if not cs:
                    continue
                n_conf += 1
                a = grid.routes[rid]["name"]
                bad.add(a)
                for k in cs:
                    bn = grid.routes[k]["name"]
                    bad.add(bn)
                    pairs.add(tuple(sorted((a, bn))))
                for (ll, r, c) in hot:
                    grid.hist[ll][max(r - 2, 0):r + 3, max(c - 2, 0):c + 3] += hinc
            freq = _co.Counter(n for p in pairs for n in p)
            print("  negotiate %d: pres %.1f, %d routes in conflict, %d nets to re-route, %d blocked %s (%.0f s) "
                  "top %s" % (itn, pres, n_conf, len(bad), len(dead), sorted(dead), time.time() - t0,
                              freq.most_common(8)), flush=True)
            if not bad and not dead:
                return []
            if best_n is None or n_conf < best_n:
                best_n, stale = n_conf, 0
            else:
                stale += 1
            if os.path.exists(PCB_PATH + ".stop"):
                print("  negotiate: stop file found, handing over", flush=True)
                break
            if patience and stale >= patience:
                print("  negotiate: no improvement for %d rounds, handing over to the rip-up loop" % stale, flush=True)
                break
            if one_side and itn % 3 != 0:
                # only the lower-priority net of each conflicting pair moves (every third round both do)
                order_i = {n: i for i, n in enumerate(full_order)}
                movers = {max(p, key=lambda n: order_i.get(n, 0)) for p in pairs}
                todo = sorted(movers | dead, key=full_order.index)
            else:
                todo = sorted(bad | dead, key=full_order.index)
            pres *= grow
        # not converged: keep the higher-priority net of each conflict, rip the rest and hand them back
        rest = []
        for net in sorted(bad | dead, key=full_order.index, reverse=True):
            clash = any(conflicts_of(k)[0] for k in [k for k, R in grid.routes.items() if R["name"] == net])
            if clash or net in dead:
                grid.rip_net(code[net])
                rest.append(net)
        return rest

    if seed:
        for n, (segs, vias) in seeded.items():
            if n in clusters and (segs or vias):      # stray copper of nets complete without it is dropped
                grid.add_route(code[n], segs, vias, name=n, w=net_rule(n)[0])
        order = [n for n in order if n in incomplete0 or n in rip_first]
        for n in order:
            grid.rip_net(code[n])
        print("seed mode: %d seeded routes, re-routing %d nets" % (len(grid.routes), len(order)), flush=True)
    failed, fcs = [], {}
    for net in order:
        ok, fc = route_net(net)
        if not ok:
            failed.append(net)
            fcs[net] = fc
    print("grid router pass 1: %d of %d nets complete, %d failed %s" % (len(order) - len(failed), len(order),
                                                                       len(failed), sorted(failed)), flush=True)
    if os.environ.get("NANODRONE_DIAG") == "1":
        # self-check of the conflict detector: hard-routed routes must not conflict with each other
        import collections
        n_c, kinds = 0, collections.Counter()
        for rid in list(grid.routes):
            cs, _hot = conflicts_of(rid)
            if cs:
                n_c += 1
                for k in cs:
                    kinds[(net_class(grid.routes[rid]["name"]), net_class(grid.routes[k]["name"]))] += 1
        print("DIAG: %d of %d hard routes report conflicts; class pairs %s" % (
            n_c, len(grid.routes), dict(kinds.most_common(12))), flush=True)
        sys.exit(0)
    if os.environ.get("NANODRONE_DIAG") == "2":
        # where and by whom are the failed connections blocked: soft-route each one through routed copper
        import collections
        who = collections.Counter()
        LN = {F: "F", L3: "L3", L4: "L4", B: "B"}
        for net in failed:
            reach, goal = fcs[net]
            nc = code[net]
            w, cl = net_rule(net)
            grid.special = specials(nc)
            path = grid.route(nc, reach, goal, w / 2, cl, via_r=0.125, via_cost=via_cost, soft=True, soft_pen=25.0,
                              allowed_layers=allowed(net))
            g0 = goal[0]
            if not path:
                print("DIAG2 %-11s goal %s (%.2f,%.2f): blocked by FIXED copper" % (net, LN.get(g0[0]), g0[1], g0[2]))
                continue
            blk, hot = grid.blockers(path, nc, w / 2, cl, 0.125, list(grid.special), tol=0)
            names = sorted({grid.routes[k]["name"] for k in blk if k in grid.routes})
            for n_ in names:
                who[n_] += 1
            xs = [h[2] for h in hot] or [0]
            ys = [h[1] for h in hot] or [0]
            cx, cy = (grid.to_xy(sum(ys) // len(ys), sum(xs) // len(xs)) if hot else (0, 0))
            print("DIAG2 %-11s goal %s (%.2f,%.2f) crosses %d routes near (%.1f,%.1f): %s" % (
                net, LN.get(g0[0]), g0[1], g0[2], len(names), cx, cy, names[:10]))
        print("DIAG2 most-crossed nets: %s" % who.most_common(15))
        sys.exit(0)
    if os.environ.get("NANODRONE_RRR") == "negotiate" and failed:
        failed = negotiate(failed)
        fcs = {}
        print("negotiated congestion: %d nets left for the rip-up loop %s" % (len(failed), failed), flush=True)
        os.environ.setdefault("NANODRONE_MAX_ITER", "100")
    if os.environ.get("NANODRONE_RRR", "rip") == "passes":
        # negotiated multi-pass: history cost where failed nets were blocked, then reroute everything from scratch
        # with the failed nets first; keep the best pass
        max_passes = int(os.environ.get("NANODRONE_PASSES", "25"))
        best = (len(failed), [dict(R) for R in grid.routes.values()], list(failed))
        for pn in range(2, max_passes + 1):
            for net in failed:
                if net not in fcs:
                    continue
                nc = code[net]
                w, cl = net_rule(net)
                grid.special = specials(nc)
                reach, goal = fcs[net]
                path = grid.route(nc, reach, goal, w / 2, cl, via_r=0.125, via_cost=via_cost, soft=True, soft_pen=25.0,
                              allowed_layers=allowed(net))
                if path:
                    spc = list(grid.special)
                    _blk, hot = grid.blockers(path, nc, w / 2, cl, 0.125, spc, tol=0)
                    for (ll, r, c) in hot:
                        grid.hist[ll][max(r - 3, 0):r + 4, max(c - 3, 0):c + 4] += 1.0
            for l in grid.layers:                     # rip everything: back to the fixed copper
                grid.owner[l][:] = grid.base_owner[l]
                grid.trk[l][:] = grid.base_trk[l]
                grid.rid[l][:] = 0
            grid.hole[:] = grid.base_hole
            grid.routes = {}
            order = failed + [n for n in order if n not in failed]
            failed, fcs = [], {}
            for net in order:
                ok, fc = route_net(net)
                if not ok:
                    failed.append(net)
                    fcs[net] = fc
            print("grid router pass %d: %d failed %s" % (pn, len(failed), sorted(failed)), flush=True)
            if len(failed) < best[0]:
                best = (len(failed), [dict(R) for R in grid.routes.values()], list(failed))
            if not failed:
                break
        if best[0] < len(failed):                     # restore the best pass
            for l in grid.layers:
                grid.owner[l][:] = grid.base_owner[l]
                grid.trk[l][:] = grid.base_trk[l]
                grid.rid[l][:] = 0
            grid.hole[:] = grid.base_hole
            grid.routes = {}
            for R in best[1]:
                grid.add_route(R["net"], R["segs"], R["vias"], name=R["name"], w=R["w"])
            failed = best[2]
        os.environ["NANODRONE_MAX_ITER"] = os.environ.get("NANODRONE_MAX_ITER", "60")
    rips = {}
    queue = list(failed)
    it = 0
    dead = set()
    max_iter = int(os.environ.get("NANODRONE_MAX_ITER", str(max_iter)))
    while queue and it < max_iter and not os.path.exists(PCB_PATH + ".stop"):    # <board>.stop: end + write
        it += 1
        net = queue.pop(0)
        nc = code[net]
        grid.rip_net(nc)
        ok, fc = route_net(net)
        if ok:
            continue
        # soft route of the failing connection to find who is in the way
        grid.rip_net(nc)
        w, cl = net_rule(net)
        grid.special = specials(nc)
        reach, goal = fc
        path = grid.route(nc, reach, goal, w / 2, cl, via_r=0.125, via_cost=via_cost, soft=True, soft_pen=25.0,
                              allowed_layers=allowed(net))
        if not path:
            dead.add(net)
            print("  [%d] %s: no path even through routed copper (fixed copper blocks it)" % (it, net), flush=True)
            continue
        spc = list(grid.special)
        blk, hot = grid.blockers(path, nc, w / 2, cl, 0.125, spc, tol=0)
        for (ll, r, c) in hot:
            grid.hist[ll][max(r - 3, 0):r + 4, max(c - 3, 0):c + 4] += 0.6
        bnets = sorted({grid.routes[k]["name"] for k in blk if k in grid.routes})
        for bn in bnets:
            rips[bn] = rips.get(bn, 0) + 1
            grid.rip_net(code[bn])
        ok, fc = route_net(net)
        if not ok:
            rips[net] = rips.get(net, 0) + 1
            if rips[net] <= max_rips:
                queue.append(net)
            else:
                dead.add(net)
        for bn in bnets:
            okb, _ = route_net(bn)
            if not okb and bn not in queue:
                if rips.get(bn, 0) <= max_rips:
                    queue.append(bn)
                else:
                    dead.add(bn)
        if it % 10 == 0 or not queue:
            print("  [%d] queue %d, last %s (ripped %s)" % (it, len(queue), net, bnets), flush=True)
    # nets still queued or given up are incomplete; the DRC unconnected-items list is the final check
    incomplete = sorted(set(queue) | dead)
    nseg = 0
    for k, R in grid.routes.items():
        for (layer, x0, y0, x1, y1, hw) in R["segs"]:
            bd.track([(x0, y0), (x1, y1)], round(2 * hw, 4), layer, R["name"], locked=False)
            nseg += 1
        for (vx, vy, r) in R["vias"]:                # grid vias 0.25 mm, seeded (Freerouting) vias 0.30 mm
            bd.via(vx, vy, R["name"], d=round(2 * r, 4), drill=0.15, in_pad=False, locked=False)
    print("grid router: %d iterations, %d routes / %d segments written, incomplete nets: %d %s" % (
        it, len(grid.routes), nseg, len(incomplete), incomplete), flush=True)
    return incomplete


def seg_path_clear(bd, path, w, cl, net, layer, endpoints):
    """All segments of path clear of other-net copper on layer (pads incl. PTH, tracks, vias)."""
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(L / 0.05) + 1)
        for k in range(n + 1):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            if not bd.clear_of(x, y, w / 2, net, layers=[layer], margin=cl):
                return False
    return True


def _sexpr_blocks(t, head):
    """Yield (start, end) spans of every balanced '(head ...)' block in text t."""
    i = 0
    while True:
        i = t.find("(" + head, i)
        if i < 0:
            return
        j, depth = i, 0
        while True:
            c = t[j]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        yield i, j + 1
        i = j + 1


def _cut(t, spans):
    for s, e in sorted(spans, reverse=True):
        t = t[:s] + t[e:]
    return t


def prepare_dsn(path, plane_layers=("In1.Cu", "In3.Cu", "In4.Cu")):
    """Make the DSN routable without touching the reference planes.

    * L2 / L4 / L5 are declared power layers and KiCad's plane areas on them (GND, VBAT, GND fill) are kept.
      Freerouting 2.5 routes traces on any layer without a plane, power or not ("treated as a signal layer"),
      so the planes are what keeps L2/L5 solid GND references and L4 the VBAT plane. 2.5 does not treat planes
      as obstacles for other nets' vias (plane_as_obstacle = false).
    * The GND/VBAT nets stay in the network: the prep fan-out vias already tie their pins to the planes, and
      Freerouting fans out any pin that is left.
    * rf_isolation: the RF class clearance becomes 0.20 mm, so no autorouted copper comes closer to the RF feed.
    (An autoroute_settings block with layer_rule/active off empties the board in 2.5's parser - not used.)"""
    t = open(path, encoding="utf-8").read()
    for lay in plane_layers:
        t = re.sub(r"\(layer %s\s*\(type signal\)" % re.escape(lay), "(layer %s (type power)" % lay, t)
    t = re.sub(r"(\(class RF [^()]*\(circuit[^)]*\)\s*\)\s*\(rule\s*\(width \d+\)\s*\(clearance )\d+",
               lambda m: m.group(1) + "%d" % round(RF_ISO * 1000), t)
    open(path, "w", encoding="utf-8").write(t)
    print("DSN prepared: %s power layers with their planes, RF clearance %.2f mm" % (", ".join(plane_layers), RF_ISO))


def autoroute(bd, passes=40):
    bd.save()
    b = pcbnew.LoadBoard(PCB_PATH)
    if not pcbnew.ExportSpecctraDSN(b, DSN_PATH):
        raise RuntimeError("DSN export failed")
    prepare_dsn(DSN_PATH)
    if os.path.exists(SES_PATH):
        os.remove(SES_PATH)
    cmd = [JAVA, "-jar", FREEROUTING, "--gui.enabled=false", "--profile.allow_telemetry=false",
           "--usage_and_diagnostic_data.disable_analytics=true", "--router.automatic_neckdown=false",
           *os.environ.get("FR_EXTRA", "").split(),     # e.g. "--router.fanout.enabled=false" (finisher runs)
           "-de", DSN_PATH, "-do", SES_PATH, "-mp", str(passes)]
    print("freerouting:", " ".join(cmd[3:]))
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=int(os.environ.get("FR_TIMEOUT", "3600")))
    tail = (r.stdout + r.stderr).strip().splitlines()[-12:]
    print("\n".join(tail))
    if not os.path.exists(SES_PATH):
        raise RuntimeError("no SES produced")
    b = pcbnew.LoadBoard(PCB_PATH)
    if not pcbnew.ImportSpecctraSES(b, SES_PATH):
        raise RuntimeError("SES import failed")
    pcbnew.SaveBoard(PCB_PATH, b)


# ---------------------------------------------------------------------------
# 5. planes / pours
# ---------------------------------------------------------------------------
def _bd(b):
    bd = Board.__new__(Board)
    bd.b, bd.fp, bd.nets, bd.added = b, {f.GetReference(): f for f in b.GetFootprints()}, b.GetNetsByName(), 0
    return bd


def trim_stubs(path, drv=("U11", "U12", "U13", "U14")):
    """Remove prep escape stubs that hang free. The prep stage gives every driver signal pin a short locked
    stub, and some of them end up unused. One reason is that the pin is unconnected by design (MODE floats for
    3x PWM, SOC is not used). The other is that the router joined the pad directly. A stub with one end on its
    driver pad and the other end touching nothing of its net is deleted; repeat until stable."""
    b = pcbnew.LoadBoard(path)
    pads = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            pads.setdefault(p.GetNetname(), []).append((f.GetReference(), p))
    removed = 0
    for _ in range(5):
        items = {}
        for t in b.GetTracks():
            items.setdefault(t.GetNetname(), []).append(t)
        dead = []
        # iterate the same proxy objects that `items` holds: SWIG hands out a new wrapper per GetTracks() call, so
        # an identity test against a second listing would never exclude the stub itself
        for t in [t for its in items.values() for t in its]:
            if isinstance(t, pcbnew.PCB_VIA) or not t.IsLocked() or t.GetNetname() in ("GND", "VBAT"):
                continue
            net, lay = t.GetNetname(), t.GetLayer()
            on_drv = [e for e in (t.GetStart(), t.GetEnd())
                      if any(r in drv and p.IsOnLayer(lay) and p.HitTest(e) for r, p in pads.get(net, ()))]
            if len(on_drv) != 1:
                continue
            other = t.GetEnd() if on_drv[0] == t.GetStart() else t.GetStart()
            touched = any(o is not t and o.IsOnLayer(lay) and o.HitTest(other, 0) for o in items.get(net, ())) or                 any(p.IsOnLayer(lay) and p.HitTest(other) for r, p in pads.get(net, ()) if r not in drv)
            if not touched:
                dead.append(t)
        if not dead:
            break
        for t in dead:
            b.Delete(t)
        removed += len(dead)
    pcbnew.SaveBoard(path, b)
    print("unused driver escape stubs removed: %d" % removed, flush=True)


def widen(path, nets_re=r"^(M\d_OUT[ABC]|VBAT|GND|\+3V0)$", target=0.5, step=0.05, margin=0.0):
    """Grow tracks of power/phase nets toward `target` width where clearance allows. Phase tracks keep 0.31 mm
    to everything (the 0.30 mm phase-to-analog DRU rule), power tracks 0.16 mm (covers the PHASE class 0.15)."""
    b = pcbnew.LoadBoard(path)
    bd = _bd(b)
    rx = re.compile(nets_re)
    grown = 0
    # a phase net whose motor-current leg is fixed prep copper (phase_power_legs) carries only the BEMF sense branch
    # in its router copper (microamps): only the locked leg and stubs grow. A wide sense branch would just add
    # coupling area and, on L4, cut the VBAT plane further
    legs = {t.GetNetname() for t in b.GetTracks()
            if t.IsLocked() and not isinstance(t, pcbnew.PCB_VIA) and re.fullmatch(r"M\d_OUT[ABC]", t.GetNetname())
            and any(p.GetNetname() == t.GetNetname() and (p.HitTest(t.GetStart()) or p.HitTest(t.GetEnd()))
                    for f in b.GetFootprints() if f.GetReference().startswith("J1") for p in f.Pads())}
    tracks = [t for t in b.GetTracks() if not isinstance(t, pcbnew.PCB_VIA) and rx.match(t.GetNetname())
              and not (t.GetNetname() in legs and not t.IsLocked())]
    for t in tracks:
        net, layer = t.GetNetname(), t.GetLayer()
        ax, ay = P.XY(t.GetStart())
        bx, by = P.XY(t.GetEnd())
        cl = 0.31 if net.startswith("M") else 0.16
        w = P.tomm(t.GetWidth())
        best = w
        cand = w + step
        while cand <= target + 1e-6:
            n = max(2, int(math.hypot(bx - ax, by - ay) / 0.1) + 1)
            t.SetWidth(P.mm(0.01))
            ok = all(bd.clear_of(ax + (bx - ax) * k / n, ay + (by - ay) * k / n, cand / 2, net,
                                 layers=[layer], margin=cl + margin) and
                     bd.edge_ok(ax + (bx - ax) * k / n, ay + (by - ay) * k / n, cand / 2) for k in range(n + 1))
            t.SetWidth(P.mm(best))
            if not ok:
                break
            best = cand
            cand += step
        if best > w + 1e-6:
            t.SetWidth(P.mm(best))
            grown += 1
            hw = best / 2                     # re-index at the new width so neighbours see the grown copper
            bd._index_add(("trk", net, ax, ay, bx, by, hw), min(ax, bx) - hw, min(ay, by) - hw,
                          max(ax, bx) + hw, max(ay, by) + hw, layer)
    pcbnew.SaveBoard(path, b)
    print("widened %d of %d power/phase segments (target %.2f mm)" % (grown, len(tracks), target))


VBAT_HALF = 10.2           # L4 VBAT plane: hub square (cavity modes > 3 GHz, sim_post P5) + narrow lobes to outlying VBAT vias


def planes(path, inner_only=False):
    """L2/L5 GND planes, L4 = VBAT plane on the hub (rounded square, VBAT_HALF) with GND fill around it, and
    (unless inner_only) GND pours on L1/L3/L6. Zone clearance 0.15 mm = the RF CPWG gap on L6."""
    b = pcbnew.LoadBoard(path)
    corners = None
    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
            sps = d.GetPolyShape()
            contour = sps.Outline(0)
            corners = [pcbnew.VECTOR2I(int(contour.CPoint(i).x), int(contour.CPoint(i).y))
                       for i in range(contour.PointCount())]
    # read the outline before touching zones; Delete (not Remove) so no orphaned SWIG objects remain
    for z in list(b.Zones()):
        b.Delete(z)
    h, r = VBAT_HALF, 1.5
    hub = []
    for cx, cy, a0 in ((h - r, -h + r, -90), (h - r, h - r, 0), (-h + r, h - r, 90), (-h + r, -h + r, 180)):
        for k in range(9):
            a = math.radians(a0 + 90 * k / 8)
            hub.append(P.V(cx + r * math.cos(a), cy + r * math.sin(a)))

    def zone(net, layer, prio, clearance=0.15, full=True, name="", pts=None):
        z = pcbnew.ZONE(b)
        z.SetLayer(layer)
        z.SetNet(b.FindNet(net))
        for c in (pts or corners):
            z.AppendCorner(pcbnew.VECTOR2I(c.x, c.y), -1)
        z.SetAssignedPriority(prio)
        z.SetLocalClearance(P.mm(clearance))
        z.SetMinThickness(P.mm(0.12))
        z.SetThermalReliefGap(P.mm(0.15))
        z.SetThermalReliefSpokeWidth(P.mm(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if full else pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetZoneName(name)
        b.Add(z)

    # RF pad relief: no L5 GND under the signal pads of the matching parts and the antenna feed pad. Over the
    # 0.0994 mm L6-L5 prepreg each 0402 pad would be a 0.18 pF shunt (sim_post P4); with L5 cut it references
    # the L4 VBAT plane 0.26 mm down (an RF ground through the 200 pF plane pair), about 0.07 pF.
    rf_nets = {"RF_IO", "RF_N1", "RF_N2", "ANT"}
    n_relief = 0
    u4 = [f for f in b.GetFootprints() if f.GetReference() == "U4"][0]
    rf_top = u4.GetLayer() == F
    relief_layer = L2 if rf_top else L5           # the GND plane right under the RF side
    rf_box = None
    for f in b.GetFootprints():
        nets = {p.GetNetname() for p in f.Pads()}
        if not nets & rf_nets or f.GetReference() == "U4":
            continue
        for p in f.Pads():
            if p.GetNetname() not in rf_nets:
                continue
            bb = p.GetBoundingBox()
            m = P.mm(0.05)
            ra = pcbnew.ZONE(b)
            ra.SetIsRuleArea(True)
            ra.SetLayer(relief_layer)
            for (x, y) in ((bb.GetLeft() - m, bb.GetTop() - m), (bb.GetRight() + m, bb.GetTop() - m),
                           (bb.GetRight() + m, bb.GetBottom() + m), (bb.GetLeft() - m, bb.GetBottom() + m)):
                ra.AppendCorner(pcbnew.VECTOR2I(int(x), int(y)), -1)
            x0b, y0b, x1b, y1b = bb.GetLeft() - m, bb.GetTop() - m, bb.GetRight() + m, bb.GetBottom() + m
            rf_box = (x0b, y0b, x1b, y1b) if rf_box is None else (min(rf_box[0], x0b), min(rf_box[1], y0b),
                                                                 max(rf_box[2], x1b), max(rf_box[3], y1b))
            ra.SetDoNotAllowZoneFills(True)
            ra.SetDoNotAllowTracks(False)
            ra.SetDoNotAllowVias(False)
            ra.SetDoNotAllowPads(False)
            ra.SetDoNotAllowFootprints(False)
            ra.SetZoneName("RF_pad_relief")
            b.Add(ra)
            n_relief += 1
    if rf_top and rf_box:
        # with the relief on L2 the pads look down to L3: keep L3 signal tracks out from under the RF chain (the L3
        # GND pour still fills there, so it becomes the pads' reference)
        g = P.mm(0.4)
        ka = pcbnew.ZONE(b)
        ka.SetIsRuleArea(True)
        ka.SetLayer(L3)
        for (x, y) in ((rf_box[0] - g, rf_box[1] - g), (rf_box[2] + g, rf_box[1] - g), (rf_box[2] + g, rf_box[3] + g),
                       (rf_box[0] - g, rf_box[3] + g)):
            ka.AppendCorner(pcbnew.VECTOR2I(int(x), int(y)), -1)
        ka.SetDoNotAllowTracks(True)
        ka.SetDoNotAllowZoneFills(False)
        ka.SetDoNotAllowVias(False)
        ka.SetDoNotAllowPads(False)
        ka.SetDoNotAllowFootprints(False)
        ka.SetZoneName("RF_L3_no_tracks")
        b.Add(ka)
    # back-side silk text must be mirrored (footprints placed before the kifp fix)
    for f in b.GetFootprints():
        for it in f.GraphicalItems():
            if isinstance(it, pcbnew.PCB_TEXT) and it.GetLayer() == pcbnew.B_SilkS and not it.IsMirrored():
                it.SetMirrored(True)
        # part-number fields come in visible on the silkscreen from the netlist: BOM data, never printed
        for fld in f.GetFields():
            if fld.GetName() in ("MPN", "LCSC") and fld.IsVisible():
                fld.SetVisible(False)
    # fiducials: keep the pours out of the 1.2 mm mask opening (r 0.6) + 0.15 mm
    for f in b.GetFootprints():
        if not f.GetReference().startswith("FID"):
            continue
        fx, fy = P.XY(f.GetPosition())
        lay = F if f.GetLayer() == F else B
        ra = pcbnew.ZONE(b)
        ra.SetIsRuleArea(True)
        ra.SetLayer(lay)
        for k in range(16):
            a = 2 * math.pi * k / 16
            ra.AppendCorner(P.V(fx + 0.78 * math.cos(a), fy + 0.78 * math.sin(a)), -1)
        ra.SetDoNotAllowZoneFills(True)
        ra.SetDoNotAllowTracks(False)
        ra.SetDoNotAllowVias(False)
        ra.SetDoNotAllowPads(False)
        ra.SetDoNotAllowFootprints(False)
        ra.SetZoneName("fiducial_clear")
        b.Add(ra)
    # lobes: VBAT vias/holes outside the hub square get a 1.4 mm wide bridge back to the square
    lobes = []
    holes = [P.XY(v.GetPosition()) for v in b.GetTracks() if isinstance(v, pcbnew.PCB_VIA) and v.GetNetname() == "VBAT"]
    holes += [P.XY(p_.GetPosition()) for f_ in b.GetFootprints() for p_ in f_.Pads()
              if p_.GetNetname() == "VBAT" and p_.GetDrillSize().x > 0]
    for (vx, vy) in holes:
        if max(abs(vx), abs(vy)) <= h - 0.45:
            continue
        hw_ = 0.7
        if abs(vx) >= abs(vy):
            sx_ = math.copysign(1, vx)
            xa_, xb_ = sx_ * (h - 1.0), vx + sx_ * 0.6
            pts_ = [(xa_, vy - hw_), (xb_, vy - hw_), (xb_, vy + hw_), (xa_, vy + hw_)]
        else:
            sy_ = math.copysign(1, vy)
            ya_, yb_ = sy_ * (h - 1.0), vy + sy_ * 0.6
            pts_ = [(vx - hw_, ya_), (vx + hw_, ya_), (vx + hw_, yb_), (vx - hw_, yb_)]
        lobes.append([P.V(x_, y_) for x_, y_ in pts_])
    # one VBAT polygon: hub square united with the lobes (overlapping same-net zones are a DRC error)
    vb = pcbnew.SHAPE_POLY_SET()
    vb.NewOutline()
    for c in hub:
        vb.Append(c)
    for lb in lobes:
        lp = pcbnew.SHAPE_POLY_SET()
        lp.NewOutline()
        for c in lb:
            lp.Append(c)
        vb.BooleanAdd(lp)
    vo = vb.Outline(0)
    vbat_pts = [pcbnew.VECTOR2I(int(vo.CPoint(i).x), int(vo.CPoint(i).y)) for i in range(vo.PointCount())]
    zone("GND", L2, 1, name="L2_GND_plane")
    zone("VBAT", L4, 2, name="L4_VBAT_plane_hub", pts=vbat_pts)
    zone("GND", L4, 1, clearance=0.2, name="L4_GND_fill")
    zone("GND", L5, 1, name="L5_GND_plane")
    if not inner_only:
        zone("GND", F, 0, clearance=0.15, name="L1_GND_pour")
        zone("GND", L3, 0, clearance=0.15, name="L3_GND_pour")
        zone("GND", B, 0, clearance=0.15, name="L6_GND_pour")
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(path, b)
    print("planes: L2/L5 GND, L4 VBAT hub plane + GND fill%s, %d RF pad reliefs on L5" % (
        "" if inner_only else ", L1/L3/L6 GND pours", n_relief))
    if lobes:
        print("planes: %d VBAT lobes to outlying VBAT vias" % len(lobes))
    return b


def plane_change(b, v, _cache={}):
    """True when signal via v joins copper on both sides of the core: L1/L3 (return plane L2) and L4/L6 (return plane
    L5), so its return current must cross between the GND planes through a nearby GND via. Unknown counts as True."""
    key = id(b)
    if key not in _cache:                               # tracks and pads by net, built once per board
        tr, pd = {}, {}
        for t in b.GetTracks():
            if not isinstance(t, pcbnew.PCB_VIA):
                tr.setdefault(t.GetNetname(), []).append((t.GetLayer(), t.GetStart(), t.GetEnd()))
        for f in b.GetFootprints():
            for p in f.Pads():
                pd.setdefault(p.GetNetname(), []).append(p)
        _cache.clear()
        _cache[key] = (tr, pd)
    tr, pd = _cache[key]
    net, pos, rv = v.GetNetname(), v.GetPosition(), v.GetWidth(F) // 2 + 1000
    lays = {l for l, a, e in tr.get(net, ()) if (a - pos).EuclideanNorm() <= rv or (e - pos).EuclideanNorm() <= rv}
    for p in pd.get(net, ()):
        if p.HitTest(pos):
            lays |= {l for l in (F, B) if p.IsOnLayer(l)}
    return len({"L2" if l in (F, L3) else "L5" for l in lays}) != 1


def _gnd_via(bd, x, y, margin=0.15, d=0.3, cls=False):
    if bd.clear_of(x, y, d / 2, "GND", margin=margin, cls=cls) and bd.hole_ok(x, y, 0.15) and bd.edge_ok(x, y, d / 2):
        bd.via(x, y, "GND", d=d, drill=0.15, in_pad=False)
        return True
    return False


def stitch(path, edge_pitch=0.9):
    """GND stitching after routing:
    1. board-edge fence (0.9 mm pitch, 0.55 mm in from the edge): closes the plane cavities at the edge,
    2. return-path vias: every signal via gets a GND via within 1.0 mm (reference changes L1 <-> L6),
    3. thermal vias around each DRV8311 (ring 1.9-3.2 mm from its centre, 0.55 mm grid, max 10),
    4. RF fence top-up along the antenna feed (0.5 mm steps, both sides)."""
    b = pcbnew.LoadBoard(path)
    bd = _bd(b)
    outline = None
    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
            outline = pcbnew.SHAPE_POLY_SET(d.GetPolyShape())
    inner = pcbnew.SHAPE_POLY_SET(outline)
    inner.Deflate(P.mm(0.55), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, P.mm(0.01))
    ring = inner.Outline(0)
    pts = [P.XY(ring.CPoint(i)) for i in range(ring.PointCount())]
    n_edge = 0
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        seg = math.hypot(x1 - x0, y1 - y0)
        while acc <= seg:
            x, y = x0 + (x1 - x0) * acc / max(seg, 1e-9), y0 + (y1 - y0) * acc / max(seg, 1e-9)
            n_edge += _gnd_via(bd, x, y)
            acc += edge_pitch
        acc -= seg
    # 2. return-path vias
    gv = [P.XY(v.GetPosition()) for v in b.GetTracks() if isinstance(v, pcbnew.PCB_VIA) and v.GetNetname() == "GND"]
    gv += [P.XY(p.GetPosition()) for f in b.GetFootprints() for p in f.Pads()
           if p.GetNetname() == "GND" and p.GetDrillSize().x > 0]
    # a signal via only needs a GND via beside it when it changes the return plane: L1/L3 tracks return on L2,
    # L4/L6 tracks on L5. Plane-changing vias go first, so they get the free spots.
    sig = sorted(((P.XY(v.GetPosition()), plane_change(b, v)) for v in b.GetTracks()
                  if isinstance(v, pcbnew.PCB_VIA) and v.GetNetname() not in ("GND", "VBAT", "")),
                 key=lambda e: not e[1])
    n_ret, n_far, n_far_pc, n_pc = 0, 0, 0, sum(1 for _s, pc in sig if pc)
    for (sx, sy), pc in sig:
        if any(math.hypot(gx - sx, gy - sy) <= 1.0 for gx, gy in gv):
            continue
        ok = False
        # nearest spot first (0.30 .. 1.00 mm, 24 directions); each spot tries a 0.30 then a 0.25 mm via, keeping
        # each neighbour's own class clearance (>= 0.10 mm) instead of a flat 0.15 mm
        for r in [0.30 + 0.05 * i for i in range(15)]:
            for k in range(24):
                a = 2 * math.pi * k / 24
                x, y = sx + r * math.cos(a), sy + r * math.sin(a)
                if _gnd_via(bd, x, y, margin=0.10, cls=True) or _gnd_via(bd, x, y, margin=0.10, d=0.25, cls=True):
                    gv.append((x, y))
                    ok = True
                    break
            if ok:
                break
        n_ret += ok
        n_far += not ok
        n_far_pc += (not ok) and pc
    # 3. DRV thermal vias
    n_th = 0
    for ref in ("U11", "U12", "U13", "U14"):
        cx, cy = P.XY(bd.fp[ref].GetPosition())
        k = 0
        cands = []
        for i in range(-6, 7):
            for j in range(-6, 7):
                x, y = cx + i * 0.55, cy + j * 0.55
                dch = max(abs(x - cx), abs(y - cy))
                if 1.9 <= dch <= 3.2:
                    cands.append((dch, x, y))
        for _d, x, y in sorted(cands):
            if k >= 10:
                break
            if _gnd_via(bd, x, y):
                k += 1
        n_th += k
    # 4. RF fence top-up
    import nanodrone_pcb as NP
    nx4, ny4, px4, py4 = NP.rfio_normal(b)
    rx, ry = px4 + nx4 * NP.RF_STUB, py4 + ny4 * NP.RF_STUB
    ax, ay = P.XY(bd.pad("J2", "1").GetPosition())
    L = math.hypot(ax - rx, ay - ry)
    ux, uy = (ax - rx) / L, (ay - ry) / L
    vx, vy = -uy, ux
    n_rf = 0
    s = 0.25
    while s < L:
        for side in (1, -1):
            for off in (0.45, 0.6, 0.8):
                x, y = rx + ux * s + vx * side * off, ry + uy * s + vy * side * off
                if _gnd_via(bd, x, y, margin=0.17):
                    n_rf += 1
                    break
        s += 0.5
    print("stitching vias: edge %d, return-path %d (%d signal vias still > 1 mm from GND, %d of them among the %d "
          "that change return plane), DRV thermal %d, RF fence %d" % (n_edge, n_ret, n_far, n_far_pc, n_pc, n_th, n_rf))
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(path, b)


def stitch_islands(path, net="GND", rf_pads=(("J2", "2"),)):
    """After the pours: a pour island of `net` that holds no via (and no plated hole) of the net only touches pads,
    so it floats. One example is the antenna ground J2.2 with the divider GND pads under U24 once their fan-out vias
    moved. Each such island gets a GND via at the free spot nearest to its pads. An island holding an RF ground pad
    gets up to three, so the antenna return stays short."""
    b = pcbnew.LoadBoard(path)
    bd = _bd(b)
    vias = [v.GetPosition() for v in b.GetTracks() if isinstance(v, pcbnew.PCB_VIA) and v.GetNetname() == net]
    pth = [p.GetPosition() for f in b.GetFootprints() for p in f.Pads()
           if p.GetNetname() == net and p.GetDrillSize().x > 0]
    gpads = [(f.GetReference(), p.GetNumber(), p) for f in b.GetFootprints() for p in f.Pads() if p.GetNetname() == net]
    added, floating = 0, []
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != net:
            continue
        for layer in (F, L3, B):
            if not z.IsOnLayer(layer):
                continue
            polys = z.GetFilledPolysList(layer)
            for i in range(polys.OutlineCount()):
                sps = pcbnew.SHAPE_POLY_SET()
                sps.AddOutline(polys.Outline(i))
                if any(sps.Contains(q) for q in vias + pth):
                    continue
                inside = [(r, n, p) for r, n, p in gpads if p.IsOnLayer(layer) and sps.Contains(p.GetPosition())]
                want = 3 if any((r, n) in rf_pads for r, n, _p in inside) else 1
                anchors = [P.XY(p.GetPosition()) for r, n, p in sorted(inside, key=lambda e: (e[0], e[1]) not in rf_pads)]
                bbx = polys.Outline(i).BBox()
                x0, y0 = P.XY(pcbnew.VECTOR2I(bbx.GetLeft(), bbx.GetTop()))
                x1, y1 = P.XY(pcbnew.VECTOR2I(bbx.GetRight(), bbx.GetBottom()))
                cands = []
                for ix in range(int((x1 - x0) / 0.05) + 1):
                    for iy in range(int((y1 - y0) / 0.05) + 1):
                        x, y = x0 + ix * 0.05, y0 + iy * 0.05
                        if sps.Contains(P.V(x, y)):
                            dmin = min([math.hypot(x - ax, y - ay) for ax, ay in anchors] or [0.0])
                            cands.append((dmin, x, y))
                cands.sort()
                got = 0
                for _d, x, y in cands:
                    if got >= want:
                        break
                    if _gnd_via(bd, x, y, margin=0.10, cls=True) or _gnd_via(bd, x, y, margin=0.10, d=0.25, cls=True):
                        got += 1
                if not got:
                    # no via fits inside: a straight track on the island's layer from one of its pads (the RF ground
                    # pad first) to the nearest free via site up to 2 mm away
                    for r_, n_, p_ in sorted(inside, key=lambda e: (e[0], e[1]) not in rf_pads):
                        px_, py_ = P.XY(p_.GetPosition())
                        sites = []
                        for ix in range(-40, 41):
                            for iy in range(-40, 41):
                                x, y = px_ + ix * 0.05, py_ + iy * 0.05
                                d_ = math.hypot(x - px_, y - py_)
                                if 0.2 <= d_ <= 2.0:
                                    sites.append((d_, x, y))
                        sites.sort()
                        for d_, x, y in sites:
                            if not (bd.clear_of(x, y, 0.125, net, margin=0.10, cls=True) and bd.hole_ok(x, y, 0.15)
                                    and bd.edge_ok(x, y, 0.125)):
                                continue
                            k_ = max(2, int(d_ / 0.025))
                            if all(bd.clear_of(px_ + (x - px_) * k / k_, py_ + (y - py_) * k / k_, 0.1, net,
                                               layers=[layer], margin=0.10, cls=True) for k in range(1, k_)):
                                bd.track([(px_, py_), (x, y)], 0.2, layer, net)
                                bd.via(x, y, net, d=0.25, drill=0.15, in_pad=False)
                                got = 1
                                break
                        if got:
                            break
                added += got
                if not got:
                    floating.append(("%s island at (%.2f,%.2f) with pads %s" % (
                        b.GetLayerName(layer), (x0 + x1) / 2, (y0 + y1) / 2, [r for r, _n, _p in inside])))
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(path, b)
    print("pour islands stitched: %d GND vias added%s" % (added, (", still floating: %s" % floating) if floating else ""),
          flush=True)


def fix_power_islands(path):
    """After the pours: GND/VBAT pads that DRC still reports unconnected get a straight track to the nearest
    same-net via, pad or track end within 2.5 mm (clearance-checked on the pad's layer)."""
    out = os.path.join(PRJ, "drc_islands.json")
    proj = os.path.splitext(path)[0] + ".kicad_pro"
    if not os.path.exists(proj):
        import shutil
        shutil.copy(PRO_PATH, proj)
        shutil.copy(DRU_PATH, os.path.splitext(path)[0] + ".kicad_dru")
    subprocess.run([KICAD_CLI, "pcb", "drc", "--all-track-errors", "--format", "json", "-o", out, path], capture_output=True, text=True)
    r = json.load(open(out))
    b = pcbnew.LoadBoard(path)
    bd = _bd(b)
    fixed = 0
    todo = []
    for u in r.get("unconnected_items", []):
        for it in u["items"]:
            m = re.search(r"\[(GND|VBAT)\]", it["description"])
            if m and it["description"].startswith("Pad"):
                todo.append((m.group(1), it["pos"]["x"] - P.OX, it["pos"]["y"] - P.OY))
    for net, x, y in todo:
        pad = None
        for f in b.GetFootprints():
            for p in f.Pads():
                px, py = P.XY(p.GetPosition())
                if p.GetNetname() == net and abs(px - x) < 0.02 and abs(py - y) < 0.02:
                    pad = p
        if pad is None:
            continue
        layer = F if pad.IsOnLayer(F) else B
        targets = []
        for v in b.GetTracks():
            if v.GetNetname() != net:
                continue
            if isinstance(v, pcbnew.PCB_VIA):
                targets.append(P.XY(v.GetPosition()))
            elif v.GetLayer() == layer:
                targets += [P.XY(v.GetStart()), P.XY(v.GetEnd())]
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() == net and p.IsOnLayer(layer) and p is not pad:
                    targets.append(P.XY(p.GetPosition()))
        done = False
        for d, tx, ty in sorted((math.hypot(tx - x, ty - y), tx, ty) for tx, ty in targets):
            if d > 2.5 or d < 0.05:
                continue
            n_ = max(2, int(d / 0.05))
            if all(bd.clear_of(x + (tx - x) * k / n_, y + (ty - y) * k / n_, 0.1, net, layers=[layer], margin=0.16)
                   for k in range(1, n_)):
                bd.track([(x, y), (tx, ty)], 0.2, layer, net)
                fixed += 1
                done = True
                break
        if not done:
            print("  power island not fixed: %s pad at (%.2f, %.2f)" % (net, x, y))
    if fixed:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(path, b)
    print("power islands fixed: %d of %d" % (fixed, len(todo)))


def drc(path):
    out = os.path.join(PRJ, "drc.json")
    subprocess.run([KICAD_CLI, "pcb", "drc", "--all-track-errors", "--format", "json", "--severity-all", "--schematic-parity",
                    "-o", out, path], capture_output=True, text=True)
    r = json.load(open(out))
    import collections
    c = collections.Counter()
    for v in r.get("violations", []):
        c[(v["severity"], v["type"])] += 1
    unc = r.get("unconnected_items", [])
    par = r.get("schematic_parity", [])
    print("DRC:", dict(c), "| unconnected:", len(unc), "| parity:", len(par))
    return r


def net_class(net):
    for name, _tw, _cl, _vd, _vdr, pats in NETCLASSES:
        if any(re.fullmatch(p.replace("?", ".").replace("+", r"\+"), net) for p in pats):
            return name
    return "Default"


# which net keeps its copper when two autorouted nets collide (higher stays): analog yields to phase, signals yield
# to rails, the RF and plane nets are never ripped (they are prep-routed and locked anyway)
NET_PRIO = {"RF": 7, "POWER": 6, "PHASE": 5, "RAIL": 4, "XTAL": 4, "DSHOT": 2, "Default": 1, "ANALOG": 0}
DRC_IGNORE = ("track_dangling", "via_dangling", "nonmirrored_text_on_back_layer", "silk_overlap", "silk_over_copper",
              "silk_edge_clearance", "text_height", "text_thickness", "lib_footprint_issues",
              "lib_footprint_mismatch", "courtyards_overlap", "isolated_copper", "starved_thermal")


def _drc_json(path, out):
    import shutil
    base = os.path.splitext(path)[0]
    if not os.path.exists(base + ".kicad_pro"):
        shutil.copy(PRO_PATH, base + ".kicad_pro")
    if not os.path.exists(base + ".kicad_dru") or open(base + ".kicad_dru").read() != DRU:
        open(base + ".kicad_dru", "w").write(DRU)
    subprocess.run([KICAD_CLI, "pcb", "drc", "--all-track-errors", "--format", "json", "--severity-all", "-o", out, path],
                   capture_output=True, text=True)
    return json.load(open(out, encoding="utf-8"))


def drc_rip_list(path, skip=("GND", "VBAT")):
    """Refill the pours, run DRC and return the nets whose autorouted (unlocked) copper takes part in a violation:
    the lower-priority net of each offending pair (both on a tie)."""
    import collections
    b = pcbnew.LoadBoard(path)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(path, b)
    r = _drc_json(path, os.path.join(PRJ, "drc_repair.json"))
    b = pcbnew.LoadBoard(path)
    by_uuid = {t.m_Uuid.AsString(): t for t in b.GetTracks()}
    rip, why = set(), collections.Counter()
    drop = {}                                         # autorouted plane-net copper in a violation: deleted
    for v in r.get("violations", []):
        if v["type"] in DRC_IGNORE:
            continue
        nets, others = [], []
        for it in v["items"]:
            t = by_uuid.get(it.get("uuid"))
            n = t.GetNetname() if t is not None else ""
            if t is not None and not t.IsLocked() and n in skip:
                drop[it.get("uuid")] = t              # GND/VBAT reach the planes through the prep vias anyway
                continue
            if t is not None and not t.IsLocked() and n and n not in skip:
                nets.append(n)
            else:
                m = re.search(r"\[([^\]]+)\]", it["description"])
                others.append(m.group(1) if m else "")
        if not nets:
            continue
        if others or len(set(nets)) == 1:                 # vs fixed copper / pad / edge: the autorouted net goes
            pick = set(nets)
        else:
            pr = {n: NET_PRIO.get(net_class(n), 1) for n in nets}
            lo = min(pr.values())
            pick = {n for n in nets if pr[n] == lo}
        rip |= pick
        why[v["type"]] += 1
    if drop:
        for t in drop.values():
            b.Delete(t)
        pcbnew.SaveBoard(path, b)
    print("DRC repair list: %d nets from %s; %d autorouted GND/VBAT items deleted" % (
        len(rip), dict(why), len(drop)), flush=True)
    return rip


def prune_dangling(path, skip=("GND", "VBAT")):
    """Delete unlocked track segments with a free end and unlocked vias that connect on fewer than two layers;
    repeat until stable (autorouter leftovers after rip-up)."""
    b = pcbnew.LoadBoard(path)
    removed = 0
    for _ in range(50):
        items = {}
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname():
                    items.setdefault(p.GetNetname(), []).append(p)
        for t in b.GetTracks():
            if t.GetNetname():
                items.setdefault(t.GetNetname(), []).append(t)
        dead = []
        for net, its in items.items():
            if net in skip:
                continue
            for t in its:
                if not isinstance(t, pcbnew.PCB_TRACK) or t.IsLocked():
                    continue
                if isinstance(t, pcbnew.PCB_VIA):
                    pos, rv = t.GetPosition(), t.GetWidth(F) // 2
                    hit = {l for l in (F, L2, L3, L4, L5, B) for o in its
                           if o is not t and o.IsOnLayer(l) and o.HitTest(pos, rv)}
                    if len(hit) < 2:
                        dead.append(t)
                    continue
                lay = t.GetLayer()
                for end in (t.GetStart(), t.GetEnd()):
                    if not any(o is not t and o.IsOnLayer(lay) and o.HitTest(end, 0) for o in its):
                        dead.append(t)
                        break
        if not dead:
            break
        for t in dead:
            b.Delete(t)
        removed += len(dead)
    if removed:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(path, b)
    print("dangling copper pruned: %d items" % removed, flush=True)


def drop_nfault_pullups(path):
    """Rev C change (NANODRONE_NFAULT_PU=0 in the schematic): the four 10k nFAULT pull-ups go and the ESC MCU's PB3
    internal pull-up (25-55k) replaces them. Each one sat 3-5 mm from its pins, in the hub, and needed its own +3V0
    branch. This removes those footprints from a routed board. Locked prep copper within 1 mm of their pads is
    unlocked, so the prune can take whatever only served them."""
    b = pcbnew.LoadBoard(path)
    spots, gone = [], []
    for f in list(b.GetFootprints()):
        nets = {p.GetNetname() for p in f.Pads()} - {""}   # 0201 footprint carries two unnamed aperture pads
        if f.GetReference().startswith("R") and "+3V0" in nets and len(nets) == 2 and \
                any(re.fullmatch(r"M\d_nFAULT", n) for n in nets):
            spots += [(p.GetNetname(), p.GetPosition()) for p in f.Pads()]
            gone.append(f.GetReference())
            b.Remove(f)
    near = lambda n, q: any(n == sn and (q - sp).EuclideanNorm() < pcbnew.FromMM(1.0) for sn, sp in spots)
    unlocked = 0
    for t in b.GetTracks():
        ends = [t.GetPosition()] if isinstance(t, pcbnew.PCB_VIA) else [t.GetStart(), t.GetEnd()]
        if t.IsLocked() and any(near(t.GetNetname(), q) for q in ends):
            t.SetLocked(False)
            unlocked += 1
    pcbnew.SaveBoard(path, b)
    print("nFAULT pull-ups removed: %s, %d locked items near them unlocked" % (sorted(gone), unlocked), flush=True)
    prune_dangling(path)


def eco_m4_column(path, dx=0.2):
    """Rev C change: motor 4's BEMF divider column (the six resistors between M4_OUTx, M4_BEMFx and GND) moves dx mm
    east, as in nanodrone_pcb.py (M4_COL_X). At rev B the bottom resistors sat right under ESC4's SWD pins (U24
    pins 20/21), so no via could stand there and the SWD pair never left the corner. Copper ending on the old pad
    positions is deleted. That is the GND fan-out stubs (the pours reconnect them) and the ends of the sense and
    BEMF tracks (the repair run re-routes those nets)."""
    b = pcbnew.LoadBoard(path)
    swap = os.environ.get("NANODRONE_M4COL_SWAP") == "1"   # phase resistor to the west column, GND one east
    moved, spots = [], []
    for f in b.GetFootprints():
        nets = {p.GetNetname() for p in f.Pads()} - {""}
        if f.GetReference().startswith("R") and any(n.startswith("M4_BEMF") for n in nets) and \
                all(re.fullmatch(r"M4_(OUT|BEMF)[ABC]|GND", n) for n in nets):
            spots += [(p.GetNetname(), p.GetPosition()) for p in f.Pads() if p.GetNetname()]
            d = dx
            if swap:          # the pair's columns are 0.65 mm apart: phase R (east) goes west, GND R (west) east
                d += -0.65 if any(n.startswith("M4_OUT") for n in nets) else 0.65
            f.Move(pcbnew.VECTOR2I(pcbnew.FromMM(d), 0))
            moved.append(f.GetReference())
    near = lambda n, q: any(n == sn and (q - sp).EuclideanNorm() < pcbnew.FromMM(0.3) for sn, sp in spots)
    dead = [t for t in b.GetTracks()
            if any(near(t.GetNetname(), q) for q in
                   ([t.GetPosition()] if isinstance(t, pcbnew.PCB_VIA) else [t.GetStart(), t.GetEnd()]))]
    for t in dead:
        b.Delete(t)
    pcbnew.SaveBoard(path, b)
    print("M4 divider column moved %.2f mm east: %s, %d copper items on the old pads deleted" % (
        dx, sorted(moved), len(dead)), flush=True)
    prune_dangling(path)


if __name__ == "__main__":
    stage = sys.argv[1] if len(sys.argv) > 1 else "all"
    if stage == "eco_nfault":                        # rev C: no external nFAULT pull-ups (see drop_nfault_pullups)
        drop_nfault_pullups(PCB_PATH)
    if stage == "eco_m4col":                         # rev C: motor 4 divider column 0.2 mm east (see eco_m4_column)
        eco_m4_column(PCB_PATH, float(os.environ.get("NANODRONE_M4COL_DX", "0.2")))
    if stage in ("all", "prep"):
        patch_project()
        patch_board_text()
        bd = Board(PCB_PATH)
        ep_vias(bd, "U2", pitch=1.3, n=3)            # FC exposed pad -> GND planes (LDO pad: fan-out below)
        route_rf(bd)
        handled = drv_power(bd)
        escape_stubs(bd)
        power_fanout(bd, handled)
        bd.save()
        planes(PCB_PATH, inner_only=True)            # planes exist before export -> router drops vias into them
    if stage in ("all", "grid"):
        bd = Board(PCB_PATH)
        grid_route(bd)
        bd.save()
    if stage == "route":                             # optional Freerouting pass (not part of "all")
        bd = Board(PCB_PATH)
        autoroute(bd, passes=int(os.environ.get("FR_PASSES", "40")))
    if stage == "legs":                              # motor-current legs as fixed copper (see phase_power_legs)
        bd = Board(PCB_PATH)
        phase_power_legs(bd)
        bd.save()
    if stage == "repair":                            # after "route": re-route DRC offenders with the grid router
        for rnd in range(int(os.environ.get("NANODRONE_REPAIR_ROUNDS", "3"))):
            rip = drc_rip_list(PCB_PATH)
            if rnd == 0:                             # nets to re-route regardless of DRC (e.g. the phase nets)
                rip |= {n for n in os.environ.get("NANODRONE_RIP_EXTRA", "").split(",") if n}
            b0 = pcbnew.LoadBoard(PCB_PATH)
            todo = any(len(net_clusters(b0, n, [p for f in b0.GetFootprints() for p in f.Pads()
                                                if p.GetNetname() == n])) > 1
                       for n in {p.GetNetname() for f in b0.GetFootprints() for p in f.Pads()}
                       if n and n not in ("GND", "VBAT"))
            if not rip and not todo:
                break
            print("repair round %d" % (rnd + 1), flush=True)
            os.environ["NANODRONE_SEED"] = "1"
            os.environ["NANODRONE_RIP"] = ",".join(sorted(rip))
            bd = Board(PCB_PATH)
            grid_route(bd)
            bd.save()
            prune_dangling(PCB_PATH)
    if stage in ("all", "finish"):
        trim_stubs(PCB_PATH)
        widen(PCB_PATH)
        planes(PCB_PATH)
        stitch(PCB_PATH)
        planes(PCB_PATH)                             # refill with the stitching vias in place
        stitch_islands(PCB_PATH)                     # pour islands that only touch pads get their own GND via
        fix_power_islands(PCB_PATH)
    if stage in ("all", "finish", "drc"):
        drc(PCB_PATH)
