"""Nano drone enclosure generator for Autodesk Fusion.

Builds a 3D-printed enclosure around the flight board. The top shell has a canopy, motor spacer bosses, a battery bay
and two prop ducts around the top motors M1/M4. The bottom shell has a belly, motor shims, two prop ducts around the
bottom motors M2/M3, which double as the landing gear, and an antenna exit.

Input:  hw/output/mech/board_ref.json (board outline, holes, part boxes and motor positions, Fusion frame), written
        by hw/tools/board_ref.py from the KiCad board.
Output: components Enclosure_Top, Enclosure_Top_Lite (hidden, no top ducts), Enclosure_Bottom and Visual
        (stand-in props, motors, screws, battery) in the active design, plus fit checks against every reference body.
        export=True writes STL/STEP files to hw/output/mech/enclosure/. images=True adds viewport images, and
        renders=True queues raytraced local renders (see render()). Design notes: hw/docs/ENCLOSURE.md.

Frame: board centre at the origin, board top surface at z = 0, front = +Y, Z up. This file works in millimetres;
the Fusion API takes centimetres (P() converts).

The motor screws hold both shells, so there are no extra fasteners:
  M1/M4 (motors on top):     head under the bottom-shell tab | PCB 0.8 | top-shell boss 3.0 | motor    -> M1.4 x 6
  M2/M3 (motors underneath): head on the top-shell tab       | PCB 0.8 | bottom-shell shim 1.0 | motor -> M1.4 x 4
Both leave about 1.5 mm of thread in the motor base. A longer screw can reach the stator windings.

Run it from Fusion (Utilities > Add-Ins > Scripts and Add-Ins > + > this folder) or through the Fusion MCP runner,
which imports this module and calls main().
"""
import json
import math
import os

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
BOARD_JSON = os.path.join(HW, "output", "mech", "board_ref.json")
OUT_DIR = os.path.join(HW, "output", "mech", "enclosure")
CM = 0.1
S2 = math.sqrt(2.0)

# ------------------------------------------------------------------ stack (mm)
BOARD_T = 0.8
TOP_SP = 3.0                 # spacer boss under the top motors = their base height
BOT_SHIM = 1.0               # shim under the bottom motors
TAB_T = 0.7                  # clamp tab under the screw heads
SKIN = 0.6                   # canopy / belly plate
CLR = 1.4                    # plate gap to the board surface (tallest part 0.9 mm)
BODY = 11.85                 # body half-size: walls stand 0.25 mm outside the 22 x 22 mm board hub
BODY_RC = 3.0                # body corner radius (the corners merge into the motor bosses / tabs)
BODY_EDGE_R = 0.5            # rounded outer edge on the plate side of each body half
SKIRT_T = 0.6                # body side wall; the top and bottom walls stop at the board faces, so the
                             # board edge shows as a thin seam between them
MOTOR_R, MOTOR_H = 4.5, 8.5  # 0702: bell diameter 9 mm, base to prop seat 8.5 mm
PROP_R, PROP_HUB = 15.5, 3.4  # Gemfan 1219S-3, 31 mm
SCREW_R, CENTRE_R = 0.85, 1.5  # M1.4 clearance (1.7 mm: MJF holes are +/-0.3 mm), shaft/circlip clearance
BOSS_RI = 2.2
TAB_R = 5.2
SHIM_R = 6.0
PART_CLR = 0.4               # shell to component clearance (sideways and above), > MJF tolerance 0.3 mm
PAD_KEEP_H = {"J1": 1.2, "J2": 1.0, "J11": 1.0, "J12": 1.0, "J13": 1.0, "J14": 1.0}  # solder joint + wire

# ------------------------------------------------------------------ ducts
R_IN = PROP_R + 1.0          # 1 mm tip gap
WALL = 0.7
LIP_R = 0.55                 # rolled inlet lip, 1.1 mm thick
BEAD_R = 0.55                # landing bead under the bottom ducts
TOP_PROP_Z = (TOP_SP + MOTOR_H, TOP_SP + MOTOR_H + PROP_HUB)            # 11.5 .. 14.9
BOT_BASE = -BOARD_T - BOT_SHIM                                         # -1.8
BOT_PROP_Z = (BOT_BASE - MOTOR_H - PROP_HUB, BOT_BASE - MOTOR_H)       # -13.7 .. -10.3
TOP_DUCT_Z = (TOP_PROP_Z[0] - 0.9, TOP_PROP_Z[1] + 0.6)                # 10.6 .. 15.5
BOT_DUCT_Z = (BOT_PROP_Z[0] - 1.1, BOT_PROP_Z[1] + 0.9)                # -14.8 .. -9.4
STRUT_W, STRUT_T = 0.9, 1.5  # fin struts, rounded top and bottom
STRUT_CLR = 0.5              # strut to prop disk, vertical
STRUT_SPREAD = 110.0         # struts at the outward direction and +/- this angle
BRIDGE_W = 2.4

# ------------------------------------------------------------------ battery bay (1S 300 mAh, 62 x 11.8 x 7 mm)
BATT_W = 11.8
RAIL_T, RAIL_H, RAIL_GAP = 0.8, 3.5, 0.4    # 12.6 mm bay for the 11.8 mm pack (both +/-0.3 mm)
RAIL_OFF = BATT_W / 2 + RAIL_GAP + RAIL_T / 2
STRAP_W, STRAP_H = 4.6, 1.0  # slot for a 4 mm hook-and-loop strap

ANT_TUBE_R, ANT_BORE_R = 1.25, 0.8   # antenna guide: bore fits 1.13 mm coax or a 1 mm whip

DENSITY = (("PA12 MJF", 1.01), ("SLA resin", 1.15), ("PETG FDM", 1.27))
TOP_MOTORS, BOT_MOTORS = ("M1", "M4"), ("M2", "M3")
SHELL_COMPS = ("Enclosure_Top", "Enclosure_Top_Lite", "Enclosure_Bottom")
OLD_COMPS = SHELL_COMPS + ("Top_Shell", "_enclosure_scratch")

# Mass table for the CG estimate (g), sim/SIMULATION_REPORT.md: AUW 16.0 g = dry 7.7 g + 300 mAh pack 8.3 g.
# The board line is what is left after the motors and props.
M_BATT, M_MOTOR, M_PROP = 8.3, 1.45, 0.25
M_BOARD = 16.0 - M_BATT - 4 * M_MOTOR - 4 * M_PROP

_TB = None


def TB():
    global _TB
    if _TB is None:
        _TB = adsk.fusion.TemporaryBRepManager.get()
    return _TB


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * CM, y * CM, z * CM)


def V(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


# ------------------------------------------------------------------ solid helpers (temporary B-rep, mm)
def _op(a, b, kind):
    TB().booleanOperation(a, b, kind)
    return a


def union(a, *bs):
    for b in bs:
        _op(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return a


def cut(a, *bs):
    for b in bs:
        _op(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return a


def inter(a, b):
    return _op(a, b, adsk.fusion.BooleanTypes.IntersectionBooleanType)


def cyl(x, y, z0, z1, r):
    return TB().createCylinderOrCone(P(x, y, z0), r * CM, P(x, y, z1), r * CM)


def rod(a, b, r):
    return TB().createCylinderOrCone(P(*a), r * CM, P(*b), r * CM)


def obox(c, ld, wd, l, w, h):
    return TB().createBox(adsk.core.OrientedBoundingBox3D.create(P(*c), V(*ld), V(*wd), l * CM, w * CM, h * CM))


def box(cx, cy, cz, lx, ly, lz, ang=0.0):
    ca, sa = math.cos(ang), math.sin(ang)
    return obox((cx, cy, cz), (ca, sa, 0.0), (-sa, ca, 0.0), lx, ly, lz)


def box2(x0, x1, y0, y1, z0, z1):
    return box((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, x1 - x0, y1 - y0, z1 - z0)


def ring(x, y, z0, z1, ri, ro):
    return cut(cyl(x, y, z0, z1, ro), cyl(x, y, z0 - 1, z1 + 1, ri))


def stadium(p0, p1, r, z0, z1):
    s = union(cyl(p0[0], p0[1], z0, z1, r), cyl(p1[0], p1[1], z0, z1, r))
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if L > 1e-6:
        union(s, box((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (z0 + z1) / 2, L, 2 * r, z1 - z0,
                     math.atan2(p1[1] - p0[1], p1[0] - p0[0])))
    return s


def slot(cx, cy, length, width, ang, z0, z1):
    """Vertical slot with round ends, `length` overall, long axis at `ang` (rad)."""
    h = (length - width) / 2
    ca, sa = math.cos(ang), math.sin(ang)
    return stadium((cx - h * ca, cy - h * sa), (cx + h * ca, cy + h * sa), width / 2, z0, z1)


def rrect(hx, hy, rc, z0, z1):
    zc, h = (z0 + z1) / 2, z1 - z0
    b = union(box(0, 0, zc, 2 * hx - 2 * rc, 2 * hy, h), box(0, 0, zc, 2 * hx, 2 * hy - 2 * rc, h))
    for sx in (-1, 1):
        for sy in (-1, 1):
            union(b, cyl(sx * (hx - rc), sy * (hy - rc), z0, z1, rc))
    return b


def rounded_slab(hx, hy, rc, z0, z1, r, top=True):
    """Rounded-rectangle prism z0..z1 whose top (or bottom) perimeter edge is rounded with radius r < rc:
    core slabs plus cylinders along the straight edges and tori at the corners (exact, no fillet feature)."""
    ze = z1 - r if top else z0 + r
    a = rrect(hx, hy, rc, z0, z1 - r) if top else rrect(hx, hy, rc, z0 + r, z1)
    union(a, rrect(hx - r, hy - r, rc - r, z0, z1))
    for s in (-1, 1):
        union(a, rod((-(hx - rc), s * (hy - r), ze), (hx - rc, s * (hy - r), ze), r),
              rod((s * (hx - r), -(hy - rc), ze), (s * (hx - r), hy - rc, ze), r))
        for t in (-1, 1):
            union(a, torus_at(s * (hx - rc), t * (hy - rc), ze, rc - r, r))
    return a


def torus_at(x, y, z, R, r):
    """Torus about a vertical axis. TemporaryBRepManager.createTorus ignores its centre point in this Fusion
    build (it always lands on the origin), so build it there and move it."""
    return moved(TB().createTorus(P(0, 0, 0), V(0, 0, 1), R * CM, r * CM), x, y, z)


def capsule(a, b, w, t):
    """Bar from a to b: width w horizontal, height t across the bar in the vertical plane, round top and bottom."""
    d = [b[i] - a[i] for i in range(3)]
    L = math.sqrt(sum(x * x for x in d))
    u = [x / L for x in d]
    h = math.hypot(d[0], d[1])
    wv = (-d[1] / h, d[0] / h, 0.0)
    n = (u[1] * wv[2] - u[2] * wv[1], u[2] * wv[0] - u[0] * wv[2], u[0] * wv[1] - u[1] * wv[0])
    c = [(a[i] + b[i]) / 2 for i in range(3)]
    body = obox(c, u, wv, L, w, max(t - w, 0.01))
    off = (t - w) / 2
    for s in (1, -1):
        o = [n[i] * off * s for i in range(3)]
        union(body, rod([a[i] + o[i] for i in range(3)], [b[i] + o[i] for i in range(3)], w / 2))
    return body


def polar(m, r, ang_deg, z=0.0):
    a = math.radians(ang_deg)
    return (m[0] + r * math.cos(a), m[1] + r * math.sin(a), z)


def deg(x, y):
    return math.degrees(math.atan2(y, x))


def diag(c, off):
    """Point at c along the battery axis (y = x) and off across it (+ is towards -x/+y)."""
    return ((c - off) / S2, (c + off) / S2)


def moved(body, dx, dy, dz=0.0):
    b = TB().copy(body)
    m = adsk.core.Matrix3D.create()
    m.translation = V(dx * CM, dy * CM, dz * CM)
    TB().transform(b, m)
    return b


# ------------------------------------------------------------------ board data
def load_board():
    with open(BOARD_JSON) as f:
        d = json.load(f)
    mot = {m: tuple(v["pos"]) for m, v in d["motors"].items()}
    groups = {}
    for h in d["holes"]:
        if not h["plated"] and h["d"] >= 1.0:
            groups.setdefault(h["ref"], []).append(h)
    screws = {}
    for hl in groups.values():
        cen = [h for h in hl if h["d"] > 2.0][0]["pos"]
        m = min(mot, key=lambda k: math.hypot(mot[k][0] - cen[0], mot[k][1] - cen[1]))
        screws[m] = [tuple(h["pos"]) for h in hl if h["d"] < 2.0]
    parts = {p["ref"]: p for p in d["parts"]}
    return mot, screws, parts


def centre(p):
    return ((p["x0"] + p["x1"]) / 2, (p["y0"] + p["y1"]) / 2)


def part_box(p, mx, my, z0, z1, edge=False):
    """Part bounding box grown by mx/my; with edge=True it runs out past the nearest plate edge (open notch)."""
    x0, x1, y0, y1 = p["x0"] - mx, p["x1"] + mx, p["y0"] - my, p["y1"] + my
    if edge:
        cx, cy = centre(p)
        if abs(cx) >= abs(cy):
            x0, x1 = (x0, BODY + 1) if cx > 0 else (-BODY - 1, x1)
        else:
            y0, y1 = (y0, BODY + 1) if cy > 0 else (-BODY - 1, y1)
    return box2(x0, x1, y0, y1, z0, z1)


def part_keepouts(parts, side):
    """Every component on `side` grown by PART_CLR: the shell may not enter these."""
    out = None
    for ref, p in parts.items():
        if p["side"] != side or ref.startswith(("H", "FID")):
            continue
        h = PAD_KEEP_H.get(ref, p["h"]) + PART_CLR
        if side == "top":
            b = part_box(p, PART_CLR, PART_CLR, -0.01, h)
        else:
            b = part_box(p, PART_CLR, PART_CLR, -BOARD_T - h, -BOARD_T + 0.01)
        out = b if out is None else union(out, b)
    return out


# ------------------------------------------------------------------ ducts (revolved section, real arcs)
def duct_section(zb, zt, lip_r, bead_r):
    """Closed (r, z) outline: straight inner wall at R_IN, rolled inlet lip on top, taper to the wall,
    bottom edge rounded (bead_r = WALL/2) or a landing bead (bead_r > WALL/2)."""
    ri, ro = R_IN, R_IN + WALL
    lc = zt - lip_r
    A0, Am, A1 = (ri, lc), (ri + lip_r, zt), (ri + 2 * lip_r, lc)
    T1 = (ro, lc - (2 * lip_r - WALL) - 0.6)
    bc = zb + bead_r
    B0, Bm, B1 = (ri + 2 * bead_r, bc), (ri + bead_r, zb), (ri, bc)
    segs = [("L", A0), ("A", Am, A1), ("L", T1)]
    if 2 * bead_r > WALL + 1e-6:
        segs.append(("L", (ro, bc + (2 * bead_r - WALL) + 0.6)))
    segs += [("L", B0), ("A", Bm, B1)]
    return B1, segs


def revolve_temp(root, start, segs):
    """Revolve a closed (r, z) outline about Z in a scratch component; return a temporary copy of the solid."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "_enclosure_scratch"
    sk = comp.sketches.add(comp.xZConstructionPlane)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs

    def sp(p):
        return sk.modelToSketchSpace(adsk.core.Point3D.create(p[0] * CM, 0, p[1] * CM))

    def at(skp, p):
        g = skp.worldGeometry
        return abs(g.x - p[0] * CM) < 1e-6 and abs(g.z - p[1] * CM) < 1e-6

    cur, first = sp(start), None
    for i, s in enumerate(segs):
        end = first if i == len(segs) - 1 else sp(s[-1])
        e = lines.addByTwoPoints(cur, end) if s[0] == "L" else arcs.addByThreePoints(cur, sp(s[1]), end)
        a, b = e.startSketchPoint, e.endSketchPoint
        if first is None:
            first = a if at(a, start) else b
        cur = b if at(b, s[-1]) else a
    if sk.profiles.count != 1:
        raise RuntimeError("duct section: %d profiles" % sk.profiles.count)
    revs = comp.features.revolveFeatures
    ri = revs.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                          adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    body = TB().copy(revs.add(ri).bodies.item(0))
    occ.deleteMe()
    return body


def socket(m, ang, z0, z1, r=0.75):
    """Round post fusing a strut tip into the duct wall: a vertical pin with domed ends, flush with the inner wall
    and standing proud of the outer wall by as much as the lip."""
    x, y, _ = polar(m, R_IN + WALL / 2, ang)
    p = cyl(x, y, z0 + r, z1 - r, r)
    union(p, TB().createSphere(P(x, y, z0 + r), r * CM), TB().createSphere(P(x, y, z1 - r), r * CM))
    return cut(p, cyl(m[0], m[1], z0 - 1, z1 + 1, R_IN))


def _strut_line(root_z_fn, limit_fn):
    """Iterate slope s and root height for a fin strut whose vertical half-height depends on s."""
    s = 0.7
    for _ in range(10):
        hv = STRUT_T / 2 * math.sqrt(1 + s * s)
        za = root_z_fn(hv)
        s = limit_fn(za, hv)
    return s, hv, za


def top_strut(m, ang):
    """Fin strut from the M1/M4 boss up to the duct bottom, kept STRUT_CLR under the prop disk."""
    r_a, r_c = 3.8, PROP_R + 0.5
    z_lim = TOP_PROP_Z[0] - STRUT_CLR
    s, hv, za = _strut_line(lambda hv: 0.15 + hv, lambda za, hv: (z_lim - hv - za) / (r_c - r_a))
    r_b = R_IN + WALL + 0.6
    st = capsule(polar(m, r_a, ang, za), polar(m, r_b, ang, za + s * (r_b - r_a)), STRUT_W, STRUT_T)
    inter(st, cyl(m[0], m[1], -30, 30, R_IN + WALL))
    rs = R_IN + WALL / 2
    return union(st, socket(m, ang, za + s * (rs - r_a) - hv, TOP_DUCT_Z[0] + 1.2))


def bot_strut(m, ang):
    """Fin strut from the M2/M3 shim down to the duct lip, kept STRUT_CLR above the prop disk."""
    r_a, r_c = 5.9, PROP_R + 0.5
    z_lim = BOT_PROP_Z[1] + STRUT_CLR
    s, hv, za = _strut_line(lambda hv: -BOARD_T - 0.05 - hv, lambda za, hv: (za - hv - z_lim) / (r_c - r_a))
    r_b = R_IN + WALL + 0.6
    st = capsule(polar(m, r_a, ang, za), polar(m, r_b, ang, za - s * (r_b - r_a)), STRUT_W, STRUT_T)
    inter(st, cyl(m[0], m[1], -30, 30, R_IN + WALL))
    rs = R_IN + WALL / 2
    union(st, socket(m, ang, BOT_DUCT_Z[1] - 1.2, za - s * (rs - r_a) + hv))
    x, y, _ = polar(m, 5.5, ang)
    return union(st, box(x, y, -BOARD_T - BOT_SHIM / 2, 2.2, STRUT_W + 0.6, BOT_SHIM, math.radians(ang)))


def duct_assembly(duct, m, top):
    d = moved(duct, m[0], m[1])
    out = deg(*m)
    for a in (out, out + STRUT_SPREAD, out - STRUT_SPREAD):
        union(d, top_strut(m, a) if top else bot_strut(m, a))
    return d


def bridge(ma, mb, zr):
    """Round-edged bar joining two neighbouring duct rings at the centre, trimmed to their inner walls."""
    L = math.hypot(mb[0] - ma[0], mb[1] - ma[1])
    u = ((mb[0] - ma[0]) / L, (mb[1] - ma[1]) / L)
    t0, t1 = R_IN - 0.2, L - (R_IN - 0.2)
    zc, h = (zr[0] + zr[1]) / 2, zr[1] - zr[0] - 0.5
    br = capsule((ma[0] + u[0] * t0, ma[1] + u[1] * t0, zc), (ma[0] + u[0] * t1, ma[1] + u[1] * t1, zc), BRIDGE_W, h)
    for m in (ma, mb):
        cut(br, cyl(m[0], m[1], zr[0] - 1, zr[1] + 1, R_IN))
    return br


# ------------------------------------------------------------------ shells
def rail_span(parts, off):
    """Longest run of the battery rail (along the battery axis) that stays on the canopy and clear of J1/J3."""
    keep = []
    for ref in ("J1", "J3"):
        p = parts[ref]
        keep.append((p["x0"] - 0.9, p["x1"] + 0.9, p["y0"] - 0.9, p["y1"] + 0.9))
    ok = []
    for i in range(-200, 201):
        c = i * 0.1
        good = True
        for dc in (-0.4, 0.4):
            for do in (-RAIL_T / 2, RAIL_T / 2):
                x, y = diag(c + dc, off + do)
                if abs(x) > BODY - 0.9 or abs(y) > BODY - 0.9:
                    good = False
                if any(k[0] < x < k[1] and k[2] < y < k[3] for k in keep):
                    good = False
        ok.append((c, good))
    best, run = (0, 0), []
    for c, g in ok + [(None, False)]:
        if g:
            run.append(c)
        else:
            if run and run[-1] - run[0] > best[1] - best[0]:
                best = (run[0], run[-1])
            run = []
    return best


def body_slab(z0, z1, top):
    """Body outline prism with the outer edge on the plate side rounded (top edge for the top half)."""
    return rounded_slab(BODY, BODY, BODY_RC, z0, z1, BODY_EDGE_R, top)


def tray(z_plate0, z_plate1, z_wall_end):
    """Body half: plate z_plate0..z_plate1 plus a perimeter wall running from the plate to z_wall_end (a board face)."""
    lo, hi = min(z_plate0, z_wall_end), max(z_plate1, z_wall_end)
    shell = body_slab(lo, hi, top=z_wall_end < z_plate0)
    if z_wall_end < z_plate0:                       # top half: pocket under the plate
        pocket = rrect(BODY - SKIRT_T, BODY - SKIRT_T, BODY_RC - SKIRT_T, z_wall_end - 1.0, z_plate0)
    else:                                           # bottom half: pocket above the plate
        pocket = rrect(BODY - SKIRT_T, BODY - SKIRT_T, BODY_RC - SKIRT_T, z_plate1, z_wall_end + 1.0)
    return cut(shell, pocket)


def build_top(mot, screws, parts, ducts, lite):
    z0, z1 = CLR, CLR + SKIN
    sh = tray(z0, z1, 0.0)
    # driver vents: three radial slots over each top-side DRV8311 (U11 near M1, U14 near M4)
    for u in ("U11", "U14"):
        cx, cy = centre(parts[u])
        a = math.atan2(cy, cx)
        for o in (-1.6, 0.0, 1.6):
            cut(sh, slot(cx - math.sin(a) * o, cy + math.cos(a) * o, 4.4, 0.9, a, z0 - 1, z1 + 1))
    # windows: battery pigtail J1, SWD J3, motor pads J11/J14 (open to the edge), status LED D1
    cut(sh, part_box(parts["J1"], 0.4, 0.4, -0.5, z1 + 1, edge=True))
    cut(sh, part_box(parts["J3"], 0.5, 0.4, z0 - 1, z1 + 1))
    for j in ("J11", "J14"):
        cut(sh, part_box(parts[j], 0.4, 0.4, -0.5, z1 + 1, edge=True))
    lx, ly = centre(parts["D1"])
    cut(sh, cyl(lx, ly, z0 - 1, z1 + 1, 0.75))
    # spacer bosses under the top motors, with a notch where the motor wires leave the base
    for m in TOP_MOTORS:
        mx, my = mot[m]
        boss = ring(mx, my, 0.0, TOP_SP, BOSS_RI, MOTOR_R)
        px, py = centre(parts["J1" + m[1]])
        wa = math.atan2(py - my, px - mx)
        cut(boss, box(mx + 3.6 * math.cos(wa), my + 3.6 * math.sin(wa), TOP_SP - 0.3, 3.2, 1.8, 0.8, wa))
        union(sh, boss)
    # clamp tabs under the M2/M3 screw heads, joined to the canopy by a corner post
    foot = body_slab(0.0, z1, True)
    for m in BOT_MOTORS:
        mx, my = mot[m]
        union(sh, cyl(mx, my, 0.0, TAB_T, TAB_R), inter(cyl(mx, my, 0.0, z1, TAB_R), TB().copy(foot)))
    # battery bay: two rails along y = x, a strap slot through both at the base
    spans = {}
    for off in (RAIL_OFF, -RAIL_OFF):
        c0, c1 = rail_span(parts, off)
        spans[off] = (c0, c1)
        union(sh, stadium(diag(c0, off), diag(c1, off), RAIL_T / 2, z1 - 0.01, z1 + RAIL_H))
    cs = (max(spans[RAIL_OFF][0], spans[-RAIL_OFF][0]) + min(spans[RAIL_OFF][1], spans[-RAIL_OFF][1])) / 2
    for off in (RAIL_OFF, -RAIL_OFF):
        x, y = diag(cs, off)
        cut(sh, box(x, y, z1 + STRAP_H / 2, STRAP_W, RAIL_T + 1.0, STRAP_H, math.radians(45)))
    # ducts around the top props, joined by a bridge over the battery
    if not lite:
        for m in TOP_MOTORS:
            union(sh, duct_assembly(ducts["top"], mot[m], True))
        union(sh, bridge(mot["M1"], mot["M4"], TOP_DUCT_Z))
    # clearances and holes last
    ko = part_keepouts(parts, "top")
    if ko is not None:
        cut(sh, ko)
    for m, (mx, my) in mot.items():
        cut(sh, cyl(mx, my, -1.0, TOP_SP + 1.0, CENTRE_R))
        for sx, sy in screws[m]:
            cut(sh, cyl(sx, sy, -1.0, TOP_SP + 1.0, SCREW_R))
    return sh, cs


def build_bottom(mot, screws, parts, ducts):
    z1, z0 = -BOARD_T - CLR, -BOARD_T - CLR - SKIN
    sh = tray(z0, z1, -BOARD_T)
    for m in BOT_MOTORS:                        # clear the bottom motors' bells
        mx, my = mot[m]
        cut(sh, cyl(mx, my, z0 - 1, z1 + 1, MOTOR_R + 0.7))
    # driver vents over U12 (near M2) and U13 (near M3)
    for u in ("U12", "U13"):
        cx, cy = centre(parts[u])
        a = math.atan2(cy, cx)
        for o in (-1.6, 0.0, 1.6):
            cut(sh, slot(cx - math.sin(a) * o, cy + math.cos(a) * o, 4.4, 0.9, a, z0 - 1, z1 + 1))
    for j in ("J12", "J13"):                    # motor pads: soldering access, open to the edge
        cut(sh, part_box(parts[j], 0.4, 0.4, z0 - 1, -0.5, edge=True))
    # antenna exit: chin under J2 with a solder window, guide tube pointing forward
    p = parts["J2"]
    jx = (p["x0"] + p["x1"]) / 2
    union(sh, box2(jx - 2.6, jx + 2.6, BODY - 1.0, BODY + 1.0, z0, z1))
    cut(sh, box2(p["x0"] - 0.4, p["x1"] + 0.4, p["y0"] - 0.4, BODY + 0.1, z0 - 1, -0.5))
    tz = z0 + 1.25
    tube = rod((jx, BODY - 0.2, tz), (jx, BODY + 3.0, tz), ANT_TUBE_R)
    cut(tube, rod((jx, BODY - 1.0, tz), (jx, BODY + 3.5, tz), ANT_BORE_R))
    union(sh, tube)
    # clamp tabs under the M1/M4 screw heads, joined to the belly by a corner post
    foot = body_slab(z0, -BOARD_T, False)
    for m in TOP_MOTORS:
        mx, my = mot[m]
        union(sh, cyl(mx, my, -BOARD_T - TAB_T, -BOARD_T, TAB_R),
              inter(cyl(mx, my, z0, -BOARD_T, TAB_R), TB().copy(foot)))
    # shims under the bottom motors, a short wall down to the belly, ducts with struts
    for m in BOT_MOTORS:
        mx, my = mot[m]
        cup = inter(ring(mx, my, z0, -BOARD_T, MOTOR_R + 0.7, SHIM_R), TB().copy(foot))
        union(sh, cyl(mx, my, BOT_BASE, -BOARD_T, SHIM_R), cup)
        union(sh, duct_assembly(ducts["bot"], mot[m], False))
        px, py = centre(parts["J1" + m[1]])
        wa = math.atan2(py - my, px - mx)        # motor wires leave the base towards their pads
        cut(sh, box(mx + 5.6 * math.cos(wa), my + 5.6 * math.sin(wa), (z0 + BOT_BASE) / 2 - 0.05,
                    2.6, 2.4, BOT_BASE - z0 + 0.1, wa))
    union(sh, bridge(mot["M2"], mot["M3"], BOT_DUCT_Z))
    ko = part_keepouts(parts, "bottom")
    if ko is not None:
        cut(sh, ko)
    for m, (mx, my) in mot.items():
        cut(sh, cyl(mx, my, -BOARD_T - 4.0, 0.0, CENTRE_R))
        for sx, sy in screws[m]:
            cut(sh, cyl(sx, sy, -BOARD_T - 4.0, 0.0, SCREW_R))
    return sh


# ------------------------------------------------------------------ design plumbing
def find_comp(root, name):
    for o in root.occurrences:
        if o.component.name == name:
            return o
    return None


def add_component(root, name, body, body_name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(body, bf)
    bf.finishEdit()
    b = comp.bRepBodies.item(0)
    b.name = body_name
    return occ, b


def sync_reference(design, ref):
    """Keep the reference motors/props on the stack heights used here (the reference was built for 3.5 mm)."""
    moved_ = []
    for m in TOP_MOTORS:
        for name, z_target in (("motor_" + m, TOP_SP), ("prop_disk_" + m, TOP_SP + MOTOR_H)):
            b = ref.bRepBodies.itemByName(name)
            dz = z_target - b.boundingBox.minPoint.z / CM
            if abs(dz) > 1e-3:
                coll = adsk.core.ObjectCollection.create()
                coll.add(b)
                mt = adsk.core.Matrix3D.create()
                mt.translation = V(0, 0, dz * CM)
                mi = ref.features.moveFeatures.createInput2(coll)
                mi.defineAsFreeMove(mt)
                ref.features.moveFeatures.add(mi)
                moved_.append("%s %+.2f" % (name, dz))
    vals = {"top_spacer": TOP_SP, "bot_shim": BOT_SHIM, "skin_t": SKIN, "comp_clear": CLR, "duct_wall": WALL,
            "duct_h": TOP_DUCT_Z[1] - TOP_DUCT_Z[0], "prop_gap": R_IN - PROP_R}
    for k, v in vals.items():
        up = design.userParameters.itemByName(k)
        if up:
            up.expression = "%g mm" % v
            up.comment = "set by nanodrone_enclosure.py (informational)"
    return moved_


def appearance(app, design, name):
    a = design.appearances.itemByName(name)
    if a:
        return a
    lib = app.materialLibraries.itemByName("Fusion Appearance Library")
    return design.appearances.addByCopy(lib.appearances.itemByName(name), name)


SHELL_LOOK = "Plastic - Matte (Black)"          # MJF PA12, dyed black
LOOKS = {"PCB": "Paint - Enamel Glossy (Green)", "battery": "Plastic - Matte (Blue)",
         "motor": "Aluminum - Anodized Rough (Red)", "prop": "Plastic - Translucent Matte (Gray)",
         "part": "Plastic - Matte (Black)", "bell": "Aluminum - Anodized Rough (Red)",
         "base": "Aluminum - Anodized Rough (Grey)", "blades": "Plastic - Translucent Glossy (Gray)",
         "screw": "Stainless Steel - Satin", "pack": "Plastic - Matte (Blue)", "lead": "Rubber - Soft",
         "plug": "Plastic - Matte (Yellow)"}


def style(app, design, ref, shells, visual=None):
    shell_a = appearance(app, design, SHELL_LOOK)
    for b in shells:
        b.appearance = shell_a
    show_ref = visual is None
    for b in ref.bRepBodies:
        key = "PCB" if b.name == "PCB" else b.name.split("_")[0]
        if key in LOOKS:
            b.appearance = appearance(app, design, LOOKS[key])
        if key == "prop":
            b.opacity = 0.18
        if key in ("motor", "prop", "battery"):      # the visual models stand in for these
            b.isLightBulbOn = show_ref
    if visual is not None:
        for b in visual.bRepBodies:
            b.appearance = appearance(app, design, LOOKS[b.name.split("_")[0]])


# ------------------------------------------------------------------ visual stand-ins (not used by the checks)
def prop_model(mx, my, z0, z1, hand):
    """3-blade 31 mm prop: hub plus blades cut from a pitched slab by a tapered cone (chord 4.4 -> 1.8 mm)."""
    zc = (z0 + z1) / 2
    p = cyl(mx, my, z0, z1, 1.75)
    for k in range(3):
        a = math.radians(k * 120.0 + 15.0)
        rd, tg = (math.cos(a), math.sin(a), 0.0), (-math.sin(a), math.cos(a), 0.0)
        cone = TB().createCylinderOrCone(P(mx + 1.2 * rd[0], my + 1.2 * rd[1], zc), 2.2 * CM,
                                         P(mx + 16.5 * rd[0], my + 16.5 * rd[1], zc), 0.9 * CM)
        pt = math.radians(18.0) * hand
        wd = (tg[0] * math.cos(pt), tg[1] * math.cos(pt), math.sin(pt))
        blade = obox((mx + 8.5 * rd[0], my + 8.5 * rd[1], zc), rd, wd, 17.0, 6.0, 0.45)
        union(p, inter(blade, cone))
    return inter(p, cyl(mx, my, z0 - 1, z1 + 1, PROP_R))


def screw_head(x, y, z, up):
    """M1.4 pan head (2.6 x 0.9 mm) seated at height z, crown towards +Z if up."""
    s = 1 if up else -1
    h = cyl(x, y, z, z + s * 0.5, 1.3)
    return union(h, TB().createCylinderOrCone(P(x, y, z + s * 0.5), 1.3 * CM, P(x, y, z + s * 0.9), 0.9 * CM))


def battery_model():
    """1S 300 mAh pack on the canopy along y = x, rounded long edges, lead and BT2.0 plug at the M2 end."""
    L, W, H, r = 62.0, BATT_W, 7.0, 1.4
    z0 = CLR + SKIN
    ang = math.radians(45.0)
    u = (math.cos(ang), math.sin(ang), 0.0)
    v = (-math.sin(ang), math.cos(ang), 0.0)
    pack = union(obox((0, 0, z0 + H / 2), u, v, L, W - 2 * r, H), obox((0, 0, z0 + H / 2), u, v, L, W, H - 2 * r))
    for sv in (-1, 1):
        for sz in (-1, 1):
            o = (v[0] * sv * (W / 2 - r), v[1] * sv * (W / 2 - r), z0 + H / 2 + sz * (H / 2 - r))
            union(pack, rod((o[0] - u[0] * L / 2, o[1] - u[1] * L / 2, o[2]), (o[0] + u[0] * L / 2, o[1] + u[1] * L / 2, o[2]), r))
    end = L / 2
    lead = None
    for sv in (-1.0, 1.0):
        a = (u[0] * end + v[0] * sv, u[1] * end + v[1] * sv, z0 + H / 2)
        b = (u[0] * (end + 5.0) + v[0] * sv, u[1] * (end + 5.0) + v[1] * sv, z0 + H / 2)
        w = rod(a, b, 0.45)
        lead = w if lead is None else union(lead, w)
    plug = obox((u[0] * (end + 7.0), u[1] * (end + 7.0), z0 + H / 2), u, v, 4.2, 5.6, 3.2)
    return pack, lead, plug


def build_visuals(root, mot, screws):
    """Props, two-tone motors, screw heads and a battery for pictures; the checks use the plain reference bodies."""
    items = []
    for m, (mx, my) in sorted(mot.items()):
        if m in TOP_MOTORS:
            base, bell, hub, hand = (TOP_SP, TOP_SP + 1.1), (TOP_SP + 1.25, TOP_SP + MOTOR_H), TOP_PROP_Z, 1
        else:
            base, bell, hub, hand = (BOT_BASE - 1.1, BOT_BASE), (BOT_BASE - MOTOR_H, BOT_BASE - 1.25), BOT_PROP_Z, -1
        items.append(("base_" + m, cyl(mx, my, base[0], base[1], 4.25)))
        cap = 1 if m in TOP_MOTORS else -1
        bz_out = bell[1] if cap > 0 else bell[0]
        bz_in = bell[0] if cap > 0 else bell[1]
        b = cyl(mx, my, bz_in, bz_out - cap * 0.6, MOTOR_R)
        union(b, TB().createCylinderOrCone(P(mx, my, bz_out - cap * 0.6), MOTOR_R * CM, P(mx, my, bz_out), 3.7 * CM))
        items.append(("bell_" + m, b))
        items.append(("blades_" + m, prop_model(mx, my, hub[0], hub[1], hand)))
        top_heads = m in BOT_MOTORS                  # M2/M3 screws go in from the top
        z = TAB_T if top_heads else -BOARD_T - TAB_T
        heads = None
        for sx, sy in screws[m]:
            h = screw_head(sx, sy, z, top_heads)
            heads = h if heads is None else union(heads, h)
        items.append(("screw_" + m, heads))
    pack, lead, plug = battery_model()
    items += [("pack_1S300", pack), ("lead_1S300", lead), ("plug_BT20", plug)]
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Visual"
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    for _, body in items:
        comp.bRepBodies.add(body, bf)
    bf.finishEdit()
    for i, (name, _) in enumerate(items):
        comp.bRepBodies.item(i).name = name
    return occ


def shell_bodies(root):
    out = {}
    for name in SHELL_COMPS:
        o = find_comp(root, name)
        if o and o.bRepBodies.count:
            out[name] = o.bRepBodies.item(0)          # occurrence proxy: measure and export need assembly context
    return out


def overlap_mm3(a, b):
    ta, tb_ = TB().copy(a), TB().copy(b)
    TB().booleanOperation(ta, tb_, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    return ta.volume * 1000.0 if ta.lumps.count else 0.0


def report_mass(shells):
    lines = []
    for name, b in shells.items():
        v = b.volume * 1000.0
        lines.append("%-20s %7.0f mm3  " % (name, v) + "  ".join("%s %.2f g" % (k, v * d / 1000.0) for k, d in DENSITY))
    return lines


def checks(app, ref, shells, visual=None):
    """Clearances (min distance) and interference (overlap volume) against the board reference and, when present,
    the visual stand-ins (screw heads, prop blades, motor bells, battery pack and plug)."""
    mm = app.measureManager
    refs = {b.name: b for b in ref.bRepBodies}
    extra = [(b.name, b) for b in visual.bRepBodies] if visual is not None else []
    out = []
    for name, sb in shells.items():
        if name == "Enclosure_Top_Lite":
            continue
        rows = []
        tops = name != "Enclosure_Bottom"
        for key in ("PCB", "battery_1S300", "motor_M1", "motor_M2", "motor_M3", "motor_M4",
                    "prop_disk_M1", "prop_disk_M2", "prop_disk_M3", "prop_disk_M4"):
            if key == "battery_1S300" and not tops:
                continue
            d = mm.measureMinimumDistance(sb, refs[key]).value / CM
            rows.append((key, d, overlap_mm3(sb, refs[key])))
        for key, b in extra:
            d = mm.measureMinimumDistance(sb, b).value / CM
            if d < 3.0:
                rows.append(("visual " + key, d, overlap_mm3(sb, b)))
        dmin, worst, ov = 99.0, "", 0.0
        for k, b in refs.items():
            if not k.startswith("part_"):
                continue
            d = mm.measureMinimumDistance(sb, b).value / CM
            if d < dmin:
                dmin, worst = d, k
            if d < 1e-3:
                ov += overlap_mm3(sb, b)
        rows.append(("parts (closest %s)" % worst[5:], dmin, ov))
        out.append(name)
        for k, d, o in rows:
            out.append("   %-24s clearance %6.2f mm   overlap %.3f mm3" % (k, d, o))
    return out


def cg(ref, shells, top_name):
    refs = {b.name: b for b in ref.bRepBodies}

    def c(b):
        bb = b.boundingBox
        return [(bb.minPoint.x + bb.maxPoint.x) / 2 / CM, (bb.minPoint.y + bb.maxPoint.y) / 2 / CM,
                (bb.minPoint.z + bb.maxPoint.z) / 2 / CM]

    items = [(M_BATT, c(refs["battery_1S300"])), (M_BOARD, c(refs["PCB"]))]
    for m in ("M1", "M2", "M3", "M4"):
        items += [(M_MOTOR, c(refs["motor_" + m])), (M_PROP, c(refs["prop_disk_" + m]))]
    for name in (top_name, "Enclosure_Bottom"):
        b = shells[name]
        p = b.physicalProperties.centerOfMass
        items.append((b.volume * 1.01, [p.x / CM, p.y / CM, p.z / CM]))
    tm = sum(w for w, _ in items)
    g = [sum(w * p[i] for w, p in items) / tm for i in range(3)]
    return tm, g


def export_files(design, root, shells):
    os.makedirs(OUT_DIR, exist_ok=True)
    em = design.exportManager
    files = []
    for name, b in shells.items():
        path = os.path.join(OUT_DIR, "nanodrone_%s.stl" % name.lower())
        opt = em.createSTLExportOptions(b, path)
        opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        opt.isBinaryFormat = True
        em.execute(opt)
        files.append(path)
        path = os.path.join(OUT_DIR, "nanodrone_%s.step" % name.lower())
        occ = find_comp(root, name)
        shown = occ.isLightBulbOn
        occ.isLightBulbOn = True                  # STEP export skips hidden components
        em.execute(em.createSTEPExportOptions(path, b.parentComponent))
        occ.isLightBulbOn = shown
        files.append(path)
    path = os.path.join(OUT_DIR, "nanodrone_assembly.step")
    em.execute(em.createSTEPExportOptions(path, root))
    files.append(path)
    # Full Fusion archive of the design (all components, Lite top included). A new document cannot be saved to the
    # cloud from an MCP script (the upload never completes), so this local .f3d is the dependable copy; open or
    # upload it from Fusion.
    path = os.path.join(OUT_DIR, "nanodrone_enclosure.f3d")
    em.execute(em.createFusionArchiveExportOptions(path, root))
    files.append(path)
    return files


VIEWS = {
    "hero": ((150.0, 95.0, 42.0), (0, 0, 1)),
    "iso_front": ((120.0, 150.0, 95.0), (0, 0, 1)),
    "iso_rear": ((-130.0, -140.0, 90.0), (0, 0, 1)),
    "underside": ((110.0, 140.0, -100.0), (0, 0, 1)),
    "side": ((0.0, 200.0, 8.0), (0, 0, 1)),
    "top": ((0.0, 0.1, 220.0), (0, 1, 0)),
}


def set_view(vp, name, margin=1.0):
    eye, up = VIEWS[name]
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.PerspectiveCameraType
    cam.isSmoothTransition = False
    cam.target = P(0, 0, 0)
    cam.eye = P(*eye)
    cam.upVector = V(*up)
    vp.camera = cam
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.isFitView = True
    vp.camera = cam
    if margin != 1.0:                             # back off from the fit for some air around the model
        cam = vp.camera
        cam.isSmoothTransition = False
        t, e = cam.target, cam.eye
        cam.eye = adsk.core.Point3D.create(t.x + (e.x - t.x) * margin, t.y + (e.y - t.y) * margin,
                                           t.z + (e.z - t.z) * margin)
        vp.camera = cam
    vp.refresh()
    return vp.camera


def snapshots(app, names=None, w=1600, h=1100, tag="enclosure"):
    os.makedirs(os.path.join(OUT_DIR, "renders"), exist_ok=True)
    vp = app.activeViewport
    files = []
    for name in names or VIEWS:
        set_view(vp, name)
        path = os.path.join(OUT_DIR, "renders", "%s_%s.png" % (tag, name))
        vp.saveAsImageFile(path, w, h)
        files.append(path)
    return files


def render(app, design, names=("hero", "iso_rear"), quality=75, w=2100, h=1400, ground=True):
    """Queue raytraced local renders (a background process; each file appears when its job finishes).
    Size and aspect only apply in the Render workspace with a custom aspect ratio. Do not edit the design until
    the jobs are done: a change kills the running job. Returns the RenderFutures (poll renderState)."""
    os.makedirs(os.path.join(OUT_DIR, "renders"), exist_ok=True)
    rm = design.renderManager
    if not rm.isRenderWorkspaceActive:
        rm.activateRenderWorkspace()
    ss = rm.sceneSettings
    ss.backgroundSolidColor = adsk.core.Color.create(244, 245, 247, 255)
    ss.isGroundDisplayed = ground
    ss.isGroundReflections = ground
    if ground:                                    # roughness can only be set while reflections are on
        ss.groundRoughness = 0.3
    ss.isDepthOfFieldEnabled = False
    ss.aspectRatio = adsk.fusion.RenderAspectRatios.CustomRenderAspectRatio
    ss.aspectRatioWidth = w
    ss.aspectRatioHeight = h
    r = rm.rendering
    r.aspectRatio = adsk.fusion.RenderAspectRatios.CustomRenderAspectRatio
    r.resolution = adsk.fusion.RenderResolutions.CustomRenderResolution
    r.resolutionWidth = w
    r.resolutionHeight = h
    r.renderQuality = quality
    r.isBackgroundTransparent = False
    vp = app.activeViewport
    futures = []
    for name in names:
        cam = set_view(vp, name, margin=1.05)
        path = os.path.join(OUT_DIR, "renders", "render_%s.png" % name)
        if os.path.exists(path):
            os.remove(path)
        futures.append(r.startLocalRender(path, cam))
    return futures


def main(build=True, check=True, export=False, images=False, renders=False):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    mot, screws, parts = load_board()
    ref_occ = find_comp(root, "Reference")
    ref = ref_occ.component
    log = []
    if build:
        log += ["reference moved: " + ", ".join(sync_reference(design, ref) or ["none"])]
        for occ in [o for o in root.occurrences if o.component.name in OLD_COMPS + ("Visual",)]:
            occ.deleteMe()
        ducts = {"top": revolve_temp(root, *duct_section(TOP_DUCT_Z[0], TOP_DUCT_Z[1], LIP_R, WALL / 2)),
                 "bot": revolve_temp(root, *duct_section(BOT_DUCT_Z[0], BOT_DUCT_Z[1], LIP_R, BEAD_R))}
        top, cs = build_top(mot, screws, parts, ducts, lite=False)
        lite, _ = build_top(mot, screws, parts, ducts, lite=True)
        bot = build_bottom(mot, screws, parts, ducts)
        bodies = []
        for name, body, bname in (("Enclosure_Top", top, "TopShell"), ("Enclosure_Top_Lite", lite, "TopShell_Lite"),
                                  ("Enclosure_Bottom", bot, "BottomShell")):
            occ, b = add_component(root, name, body, bname)
            bodies.append(b)
            if name.endswith("Lite"):
                occ.isLightBulbOn = False
        visual = build_visuals(root, mot, screws)
        style(app, design, ref, bodies, visual.component)
        log.append("strap slot at %.1f mm along the battery axis" % cs)
    shells = shell_bodies(root)
    log += report_mass(shells)
    for top_name in ("Enclosure_Top", "Enclosure_Top_Lite"):
        tm, g = cg(ref_occ, shells, top_name)
        log.append("AUW with %-18s %.2f g, CG (%.2f, %.2f, %.2f) mm" % (top_name, tm, g[0], g[1], g[2]))
    for name, b in shells.items():
        log.append("%s: %d lump(s), %d face(s)" % (name, b.lumps.count, b.faces.count))
    if check:
        log += checks(app, ref_occ, shells, find_comp(root, "Visual"))
    if export:
        log += ["wrote " + f for f in export_files(design, root, shells)]
    if images:
        log += ["image " + f for f in snapshots(app)]
    if renders:
        log += ["render queued " + f.filename for f in render(app, design)]
    print("\n".join(log))
    return log


def run(_context):
    """Entry point when run from Fusion's Scripts and Add-Ins dialog: build, check, export, show the summary."""
    log = main(build=True, check=True, export=True, images=True)
    keep = [l for l in log if not l.startswith(("wrote ", "image "))]
    adsk.core.Application.get().userInterface.messageBox("\n".join(keep[:40]), "Nano drone enclosure")
