"""Nano drone PCB - layout as code (stage 1: outline, stackup rules, placement, zones).

Frame: compact stacked quad, 0702 motors, 31 mm props, adjacent motor spacing 26 mm
(diagonal 36.77 mm). Motors (Betaflight order): M1 rear-right TOP, M2 front-right BOTTOM,
M3 rear-left BOTTOM, M4 front-left TOP. Coordinates in mm, board centre = (0, 0), +y = rear.
Each motor driver sits on the same side as its motor.
"""
import json
import math
M4_COL_X = -2.20            # motor 4 BEMF divider column, top resistors (rev C; rev B had -2.40)
import re
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HW, "tools"))
import pcbnew  # noqa: E402
import kipcb as P  # noqa: E402

KICAD_BIN = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin"
PRJ = os.path.join(HW, "kicad")
PCB_PATH = os.environ.get("NANODRONE_PCB", os.path.join(PRJ, "nanodrone.kicad_pcb"))
NET_PATH = os.path.join(PRJ, "nanodrone.net")

# ---------------------------------------------------------------------------
# Geometry parameters
# ---------------------------------------------------------------------------
SPACING = 26.0                     # adjacent motor spacing
MC = SPACING / 2                   # motor centre offset (13, 13)
HUB = 11.0                         # hub half-size (22 x 22 mm; corners mostly inside the motor-mount discs)
HUB_R = 2.0                        # hub corner radius
DISC_R = 5.0                       # motor mount disc radius
ARM_W = 6.5                        # arm width
FILLET = 1.2                       # outline fillet radius
MOTORS = {1: (MC, MC, "F"), 2: (MC, -MC, "B"), 3: (-MC, MC, "B"), 4: (-MC, -MC, "F")}

POWER_NETS = {"GND", "VBAT", "+3V0"}

SHEET_ICS = {"/Power/": ["U1"], "/Flight controller STM32G431/": ["U2"], "/IMU LSM6DSV16X/": ["U3"],
             "/Radio SX1280/": ["U4"],
             "/Motor 1 ESC/": ["U21", "U11"], "/Motor 2 ESC/": ["U22", "U12"],
             "/Motor 3 ESC/": ["U23", "U13"], "/Motor 4 ESC/": ["U24", "U14"]}
SHEET_IC = {k: v[0] for k, v in SHEET_ICS.items()}


RF_STUB = 0.6                      # RFIO pad centre -> start of the RF line, along the pin's outward normal
# radio side: B (F was tried: the radio then overlaps the FC and the RF chain runs into M4's top-side ESC)
RADIO_SIDE = os.environ.get("NANODRONE_RADIO_SIDE", "B")
RADIO_OPP = "B" if RADIO_SIDE == "F" else "F"


def rfio_normal(pcb_or_board):
    """(nx, ny, px, py): outward unit normal of the SX1280 RFIO pin (U4 pad 22) and the pad centre.
    Works with a kipcb.PCB or a raw pcbnew BOARD (also used by nanodrone_route.route_rf)."""
    if hasattr(pcb_or_board, "fps"):
        fpu = pcb_or_board.fps["U4"]
    else:
        fpu = {f.GetReference(): f for f in pcb_or_board.GetFootprints()}["U4"]
    pad = [p for p in fpu.Pads() if p.GetNumber() == "22"][0]
    px, py = P.XY(pad.GetPosition())
    cx, cy = P.XY(fpu.GetPosition())
    dx, dy = px - cx, py - cy
    if abs(dx) > abs(dy):
        return math.copysign(1.0, dx), 0.0, px, py
    return 0.0, math.copysign(1.0, dy), px, py


def export_netlist():
    subprocess.run([os.path.join(KICAD_BIN, "kicad-cli.exe"), "sch", "export", "netlist", "--format",
                    "kicadsexpr", "-o", NET_PATH, os.path.join(PRJ, "nanodrone.kicad_sch")], check=True)


def rounded_rect(h, r, n=12):
    pts = []
    for cx, cy, a0 in ((h - r, -h + r, -90), (h - r, h - r, 0), (-h + r, h - r, 90), (-h + r, -h + r, 180)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def build():
    export_netlist()
    nl = P.Netlist(NET_PATH)
    if os.path.exists(PCB_PATH):
        os.remove(PCB_PATH)
    pcb = P.PCB(PCB_PATH, nl, {"nanodrone": os.path.join(PRJ, "lib", "nanodrone.pretty")})
    pcb.load_all()
    comp = {c["ref"]: c for c in nl.comps}
    pcb._unplaced = set(pcb.fps)

    # ---------------- outline ----------------
    shapes = [rounded_rect(HUB, HUB_R)]
    for m, (x, y, _) in MOTORS.items():
        shapes.append(P.PCB.circle_pts(x, y, DISC_R))
        shapes.append(P.PCB.bar_pts(0, 0, x, y, ARM_W))
    pcb.set_outline(shapes, round_r=FILLET)

    def fixed(ref, x, y, rot=0, side="F"):
        pcb.place(ref, x, y, rot, side)
        pcb._unplaced.discard(ref)

    # ---------------- motors: mounts + phase-wire pads ----------------
    for m, (x, y, side) in MOTORS.items():
        inward = math.degrees(math.atan2(-y, -x))
        h = "H%d" % m
        best = None
        for rot in range(0, 360, 5):                 # keep the inward direction between two holes
            pcb.place(h, x, y, rot, "F")
            angs = []
            for p in pcb.fps[h].Pads():
                qx, qy = P.XY(p.GetPosition())
                px, py = qx - x, qy - y
                if math.hypot(px, py) > 1.0:
                    angs.append(math.degrees(math.atan2(py, px)))
            dmin = min(abs((a - inward + 180) % 360 - 180) for a in angs)
            if best is None or dmin > best[0]:
                best = (dmin, rot)
        fixed(h, x, y, best[1], "F")
        # wire pads: a vertical column beside the driver's output edge, at the hub's left/right edge, level with
        # the OUT pins (the old spot at the driver corner was 0.09 mm from the package and 0.16 mm from VM).
        # Gives 0.4 mm to the DRV pins, about 0.6 mm to the motor base, and sub-mm phase traces.
        sx, sy = math.copysign(1, x), math.copysign(1, y)
        fixed("J%d" % (10 + m), sx * (HUB - 0.7), sy * (HUB - 3.6 - 0.53), 90, side)

    # ---------------- connectors ----------------
    fixed("J1", HUB - 1.4, 0.0, 90, "F")             # battery pads, right edge
    fixed("J2", -3.4, -(HUB - 1.0), 0, RADIO_SIDE)   # antenna pads (SMD, radio side), front edge, wire runs forward

    # ---------------- ICs ----------------
    fixed("U2", 0.0, 0.0, 0, "F")                    # flight controller, centre top
    D = HUB - 3.6                                    # DRV8311 diagonal offset (OUT pins toward the motor pads)
    E = HUB - 2.9                                    # ESC MCU distance from centre (outer pad row 0.5 mm off the edge)
    EX = 1.4                                         # ESC MCU x offset: leaves a ~2.5 mm routing channel to its DRV
    ic_pos = {"U11": (D, D, "F"), "U21": (EX, E, "F"),           # M1 rear-right (top motor)
              "U14": (-D, -D, "F"), "U24": (-EX, -E, "F"),       # M4 front-left (top motor)
              "U12": (D, -D, "B"), "U22": (EX, -E, "B"),         # M2 front-right (bottom motor)
              "U13": (-D, D, "B"), "U23": (-EX, E, "B"),         # M3 rear-left (bottom motor)
              "U3": (0.0, 3.6, "B"),                  # IMU, bottom, just behind centre
              "U4": (-(HUB - 4.8), -1.8, RADIO_SIDE),  # radio, front-left
              "U1": (HUB - 3.9, 3.1, "F")}            # LDO next to the battery pads
    for ref, (x, y, side) in ic_pos.items():
        fixed(ref, x, y, 0, side)
    XTAL = (-(HUB - 1.7), 1.3)                       # 52 MHz crystal beside the radio (final spot found below)

    # keep-outs: other side of through-via fields
    def ko(side, cx, cy, hx, hy):
        pcb.keepouts[side].append(P.Box(cx - hx, cy - hy, cx + hx, cy + hy))

    ko("B", 0, 0, 1.45, 1.45)                        # FC exposed-pad GND vias
    for ref, (x, y, side) in ic_pos.items():
        if ref in ("U11", "U12", "U13", "U14"):
            ko("B" if side == "F" else "F", x, y, 1.2, 1.2)   # DRV thermal vias
    ko(RADIO_OPP, ic_pos["U4"][0], ic_pos["U4"][1], 1.6, 1.6)  # radio EP vias

    # auto-rotate ICs to minimise ratsnest length to everything already placed
    def ratsnest_cost(ref):
        cost = 0.0
        for p in pcb.fps[ref].Pads():
            net = p.GetNetname()
            if not net or net in POWER_NETS:
                continue
            px, py = P.XY(p.GetPosition())
            others = [(r, n) for r, n in nl.nets[net] if r != ref and r not in pcb._unplaced]
            for r, n in others:
                try:
                    ox, oy = pcb.pad_xy(r, n)
                except KeyError:
                    continue
                cost += math.hypot(px - ox, py - oy)
        return cost

    for _ in range(3):
        for ref in ("U11", "U12", "U13", "U14", "U21", "U22", "U23", "U24", "U3", "U4", "U1"):
            x, y, side = ic_pos[ref]
            best = None
            for rot in (0, 90, 180, 270):
                pcb.place(ref, x, y, rot, side)
                c = ratsnest_cost(ref)
                if best is None or c < best[0]:
                    best = (c, rot)
            pcb.place(ref, x, y, best[1], side)
    # radio: RFIO (pin 22) edge must face the antenna, so the RF line leaves perpendicular to the pin row instead of
    # running along it over the neighbouring GND pins (ratsnest choice overridden)
    jx0, jy0 = P.XY(pcb.fps["J2"].GetPosition())
    x4, y4, s4 = ic_pos["U4"]
    best = None
    for rot in (0, 90, 180, 270):
        pcb.place("U4", x4, y4, rot, s4)
        nx_, ny_, _px, _py = rfio_normal(pcb)
        dx_, dy_ = jx0 - _px, jy0 - _py
        dot = (nx_ * dx_ + ny_ * dy_) / math.hypot(dx_, dy_)
        if best is None or dot > best[0]:
            best = (dot, rot)
    pcb.place("U4", x4, y4, best[1], s4)
    print("U4 rotation %d: RFIO normal . antenna direction = %.2f" % (best[1], best[0]))
    # crystal: right outside the radio's XTA/XTB pin row (whichever way the radio now faces), shortest
    # collision-free XTA/XTB ratsnest; the nominal spot is the fallback search centre
    occ_b = pcb.occupied(RADIO_SIDE, exclude=("Y1",))
    xp = [P.XY(p.GetPosition()) for p in pcb.fps["U4"].Pads() if p.GetNetname() in ("RF_XTA", "RF_XTB")]
    mx_, my_ = sum(x for x, _ in xp) / len(xp), sum(y for _, y in xp) / len(xp)
    cx4, cy4 = P.XY(pcb.fps["U4"].GetPosition())
    ddx, ddy = mx_ - cx4, my_ - cy4
    nxx, nyy = (math.copysign(1.0, ddx), 0.0) if abs(ddx) > abs(ddy) else (0.0, math.copysign(1.0, ddy))
    centres = [(mx_ + nxx * r_ + nyy * t_, my_ + nyy * r_ + nxx * t_) for r_ in (1.6, 1.9, 2.2, 2.6)
               for t_ in (0.0, -0.4, 0.4, -0.8, 0.8)]
    centres += [(XTAL[0] + dx, XTAL[1] + dy) for dx in (0.0, -0.2, 0.2, -0.4) for dy in (0.0, 0.3, 0.6, 0.9, 1.2)]
    best = None
    for (qx, qy) in centres:
        for rot in (0, 90, 180, 270):
            pcb.place("Y1", qx, qy, rot, RADIO_SIDE)
            bx = pcb.courtyard("Y1")
            if any(pcb.collide(bx, o, 0.1) for o in occ_b) or not pcb.inside_outline(bx, 0.3):
                continue
            c = ratsnest_cost("Y1")
            if best is None or c < best[0]:
                best = (c, qx, qy, rot)
    if best is None:
        raise RuntimeError("no room for Y1")
    fixed("Y1", best[1], best[2], best[3], RADIO_SIDE)
    print("Y1 at (%.2f, %.2f) rot %d" % (best[1], best[2], best[3]))

    # ---------------- routing channels: keep passives ~0.6 mm off every fine-pitch IC's pin rows -------------
    CHANNEL = 0.6
    for ref in ("U2", "U11", "U12", "U13", "U14", "U21", "U22", "U23", "U24", "U4", "U3"):
        bx = pcb.courtyard(ref, margin=0.0)
        pcb.keepouts[pcb.side[ref]].append(bx.grow(CHANNEL + (0.3 if ref[:2] in ("U1", "U2") and len(ref) == 3 else 0.0)))

    # ---------------- motor wire pads: orientation with non-crossing phase traces ----------------
    for m, (x, y, side) in MOTORS.items():
        j, u = "J%d" % (10 + m), "U%d" % (10 + m)
        jx_, jy_ = P.XY(pcb.fps[j].GetPosition())
        rot0 = pcb.fps[j].GetOrientationDegrees()
        best = None
        for rot in (rot0, rot0 + 180):
            pcb.place(j, jx_, jy_, rot, pcb.side[j])
            cost = 0.0
            for pin, pad in (("10", "1"), ("11", "2"), ("12", "3")):      # OUTA/B/C -> pads 1/2/3
                ux_, uy_ = pcb.pad_xy(u, pin)
                px_, py_ = pcb.pad_xy(j, pad)
                cost += math.hypot(ux_ - px_, uy_ - py_)
            if best is None or cost < best[0]:
                best = (cost, rot)
        pcb.place(j, jx_, jy_, best[1], pcb.side[j])

    # ---------------- RF chain: straight line RFIO -> antenna, shunts alternating sides ----------------
    def by_nets(a, b, kind):
        for c in nl.comps:
            r = c["ref"]
            if r[0] != kind or r not in pcb._unplaced:
                continue
            ns = {p.GetNetname() for p in pcb.fps[r].Pads() if p.GetNetname()}
            if ns == {a, b}:
                return r
        return None

    nx4, ny4, px4, py4 = rfio_normal(pcb)
    rx, ry = px4 + nx4 * RF_STUB, py4 + ny4 * RF_STUB   # RF line starts RF_STUB out from the RFIO pad
    ax, ay = pcb.pad_xy("J2", "1")                  # antenna feed
    L_ = math.hypot(ax - rx, ay - ry)
    ux, uy = (ax - rx) / L_, (ay - ry) / L_
    vx, vy = -uy, ux
    ang = -math.degrees(math.atan2(uy, ux))         # KiCad rotation that aligns local x with u
    series = [("L", "RF_IO", "RF_N1"), ("L", "RF_N1", "RF_N2"), ("C", "RF_N2", "ANT")]
    shunts = [("RF_IO", -1), ("RF_N1", -1), ("RF_N2", -1)]
    # Compact matching network right at RFIO, then plain 50-ohm CPWG to the antenna pad. Series parts are packed
    # along the line with GAP between same-node pads. Each shunt C sits beside the series pad of its node, RF pad
    # facing the line (0.05 mm off the series pad), GND pad away from the line (via-in-pad later), so there is no
    # stub and no RF stub crosses a GND pad. C4 (|| L2) runs alongside L2 on the opposite side.
    GAP, SIDE_GAP = 0.20, 0.05

    def sd(x, y):
        return (x - rx) * ux + (y - ry) * uy, (x - rx) * vx + (y - ry) * vy

    def put(r, s, d, rot):
        pcb.place(r, rx + ux * s + vx * d, ry + uy * s + vy * d, rot, RADIO_SIDE)
        pcb._unplaced.discard(r)

    def pad_ext(r):
        """pads of r in line coordinates: [(net, s, d, half_s, half_d)] (parts aligned with or across the line)"""
        out = []
        for p in pcb.fps[r].Pads():
            x, y = P.XY(p.GetPosition())
            s, d = sd(x, y)
            # exact half extents of the rotated pad rectangle along u and v
            w, h = P.tomm(p.GetSize().x), P.tomm(p.GetSize().y)
            th = -math.radians(p.GetOrientation().AsDegrees())        # KiCad angles are clockwise (y down)
            ex, ey = math.cos(th), math.sin(th)                       # pad local x axis in board coords
            fx, fy = -ey, ex                                          # pad local y axis
            hs = abs(ex * ux + ey * uy) * w / 2 + abs(fx * ux + fy * uy) * h / 2
            hd = abs(ex * vx + ey * vy) * w / 2 + abs(fx * vx + fy * vy) * h / 2
            out.append((p.GetNetname(), s, d, hs, hd))
        return out
    u4_end = 0.0                                    # the RFIO stub ends at s = 0
    spos, sref = {}, {}
    cur = u4_end

    def first_net(pe):
        return sorted(pe, key=lambda q: q[1])[0][0]
    for kind, a, b in series:
        r = by_nets(a, b, kind)
        rot_r = ang
        put(r, 0.0, 0.0, rot_r)
        pe = pad_ext(r)
        if first_net(pe) != a:                      # pad of net a toward the radio
            rot_r = ang + 180
            put(r, 0.0, 0.0, rot_r)
            pe = pad_ext(r)
        lo = min(s - hs for (n, s, d, hs, hd) in pe)
        hi = max(s + hs for (n, s, d, hs, hd) in pe)
        sc = cur + GAP - lo
        put(r, sc, 0.0, rot_r)
        spos[(a, b)] = sc
        sref[(a, b)] = r
        cur = sc + hi
    chain_end = cur
    # C4 alongside L2 (+v side), its RF_N1 pad next to L2's RF_N1 pad
    c4 = by_nets("RF_N1", "RF_N2", "C")
    sL2 = spos[("RF_N1", "RF_N2")]
    rot_c4 = ang
    put(c4, sL2, 0.0, rot_c4)
    if first_net(pad_ext(c4)) != "RF_N1":
        rot_c4 = ang + 180
        put(c4, sL2, 0.0, rot_c4)
    hd_l2 = max(hd for (n, s, d, hs, hd) in pad_ext(sref[("RF_N1", "RF_N2")]))
    hd_c4 = max(hd for (n, s, d, hs, hd) in pad_ext(c4))
    put(c4, sL2, hd_l2 + SIDE_GAP + hd_c4, rot_c4)
    # shunt caps on the -v side, beside their node's series pad; first-occurring node pad is the anchor
    node_anchor = {"RF_IO": ("L", "RF_IO", "RF_N1", "RF_IO"), "RF_N1": ("L", "RF_IO", "RF_N1", "RF_N1"),
                   "RF_N2": ("L", "RF_N1", "RF_N2", "RF_N2")}
    for net, _sgn in shunts:
        r = by_nets(net, "GND", "C")
        if r is None:
            continue
        kind, a, b, n_ = node_anchor[net]
        sr = sref[(a, b)]
        s_a, hd_a = [(s, hd) for (n, s, d, hs, hd) in pad_ext(sr) if n == n_][0]
        best = None
        for rot in (ang + 90, ang - 90):
            put(r, s_a, -1.0, rot)
            pe = pad_ext(r)
            d_rf = [d for (n, s, d, hs, hd) in pe if n == net][0]
            d_gnd = [d for (n, s, d, hs, hd) in pe if n == "GND"][0]
            if d_rf > d_gnd and (best is None or d_rf > best[1]):
                best = (rot, d_rf, [hd for (n, s, d, hs, hd) in pe if n == net][0])
        rot, d_rf, hd_rf = best
        # move so the RF pad's inner edge sits SIDE_GAP outside the series pad
        target = -(hd_a + SIDE_GAP + hd_rf)
        put(r, s_a, -1.0 + (target - d_rf), rot)
    rf_ok = chain_end < L_ - 0.3
    print("RF line length %.2f mm, matching network packed into %.2f mm, explicit=%s" % (L_, chain_end, rf_ok))

    # ---------------- passives ----------------
    taken = {}

    def ic_center(ref):
        return P.XY(pcb.fps[ref].GetPosition())

    RF_CHAIN = ["RF_IO", "RF_N1", "RF_N2", "RF_N3", "ANT"]

    def part_nets(ref):
        return [n for n in dict.fromkeys(p.GetNetname() for p in pcb.fps[ref].Pads()) if n]

    def anchor_for(ref):
        """(anchor_ref, pad, net) - 1) a sheet IC pin on the most specific non-GND net (ICs in
        priority order, untaken pads first), 2) an already placed passive on such a net,
        3) any IC on such a net."""
        ics = SHEET_ICS.get(comp[ref]["sheet"], [])
        nets = sorted((n for n in part_nets(ref) if n != "GND"), key=lambda n: len(nl.nets.get(n, [])))
        fallback = None
        for n in nets:
            for ic in ics:
                pads = [num for r, num in nl.nets[n] if r == ic]
                free = [num for num in pads if (ic, num) not in taken]
                if free:
                    return ic, free[0], n
                if pads and fallback is None:
                    fallback = (ic, pads[0], n)
        if fallback:
            return fallback
        for n in nets:
            for r, num in nl.nets[n]:
                if r != ref and r not in pcb._unplaced and not r.startswith("U"):
                    return r, num, n
        for n in nets:
            for r, num in nl.nets[n]:
                if r.startswith("U") and r != ref:
                    return r, num, n
        return None

    # BEMF divider top resistors (OUTx -> BEMFx): on the side opposite the driver, in the open band between the
    # driver and its ESC MCU (x = DRV - 2.9 mm). The phase sense line reaches them from the motor-pad PTH on L3
    # (shielded between the L2 GND and L4 VBAT planes, free under the driver), and the divided BEMF node is a short
    # hop to the MCU. At the motor-pad column itself the BEMF pads were boxed in by the driver's back-side exposed
    # pad and could not take a via under the driver's OUT pins (sim_post / router subset test).
    for m in (1, 2, 3, 4):
        drv = "U%d" % (10 + m)
        opp = "B" if pcb.side[drv] == "F" else "F"
        dx_, dy_ = P.XY(pcb.fps[drv].GetPosition())
        sx = math.copysign(1.0, dx_)
        for k, ph in enumerate("ABC"):
            out, bemf = "M%d_OUT%s" % (m, ph), "M%d_BEMF%s" % (m, ph)
            rr = [c["ref"] for c in nl.comps if c["ref"].startswith("R") and set(part_nets(c["ref"])) == {out, bemf}]
            if not rr:
                continue
            anchor = (dx_ - sx * 2.9, dy_ + (k - 1) * 0.9)
            if opp == pcb.side["U4"] and m == 4:
                # motor 4's band is where the radio's RF chain runs on that side: explicit pairs in the open patch
                # east of the RF line (x -3.9..-1.9, below the radio). Top R west (phase pad facing west, where
                # the sense line arrives from under the RF line on L3), bottom R east (GND pad outward).
                rb = [c["ref"] for c in nl.comps if c["ref"].startswith("R") and
                      set(part_nets(c["ref"])) == {bemf, "GND"}]
                # rev C: the column sits 0.2 mm further east (top R at x = -2.20) so ESC4's SWD pins (U24 pins 20/21,
                # directly above the bottom resistors) get via sites; at -2.40 the SWD pair could not leave the corner
                yk = -5.15 - k * 1.7

                def put_vertical(ref_, x_, want_net, want_north):
                    for rot_ in (90, 270):
                        pcb.place(ref_, x_, yk, rot_, opp)
                        ys_ = {p_.GetNetname(): P.XY(p_.GetPosition())[1] for p_ in pcb.fps[ref_].Pads()}
                        if (ys_[want_net] < yk) == want_north:
                            break
                    pcb._unplaced.discard(ref_)
                # vertical pair, BEMF pads side by side at the south end, phase pad and GND pad at the north end
                put_vertical(rr[0], M4_COL_X, out, True)    # phase pad on the open east side
                if rb:
                    put_vertical(rb[0], M4_COL_X - 0.65, "GND", True)  # GND pad: the L6 pour reaches it
                continue
            try:
                pcb.autoplace(rr[0], anchor, (dx_, dy_), opp, max_r=3.0)
            except RuntimeError as e:
                print("  !", e)
                continue
            # bottom resistor (BEMFx -> GND) right beside it: the divider is one compact unit and only the
            # divided node runs to the ADC pin
            rb = [c["ref"] for c in nl.comps if c["ref"].startswith("R") and set(part_nets(c["ref"])) == {bemf, "GND"}]
            if rb:
                bx_, by_ = [P.XY(p.GetPosition()) for p in pcb.fps[rr[0]].Pads() if p.GetNetname() == bemf][0]
                try:
                    pcb.autoplace(rb[0], (bx_, by_), (dx_, dy_), opp, max_r=2.0)
                except RuntimeError as e:
                    print("  !", e)
    # 100 nF VM decoupler of each driver: opposite side, right under the VM corner (the corner vias land on it),
    # so the switching loop VM pin -> cap -> GND closes through ~1 mm of vias instead of a remote cap
    for m in (1, 2, 3, 4):
        drv = "U%d" % (10 + m)
        opp = "B" if pcb.side[drv] == "F" else "F"
        dx_, dy_ = P.XY(pcb.fps[drv].GetPosition())
        sheet = comp[drv]["sheet"]
        caps = [c["ref"] for c in nl.comps if c["ref"].startswith("C") and c["ref"] in pcb._unplaced and
                comp[c["ref"]]["sheet"] == sheet and set(part_nets(c["ref"])) == {"VBAT", "GND"}]
        small = [r for r in caps if "100n" in comp[r].get("value", "")] or caps[:1]
        for r in small[:1]:
            try:
                pcb.autoplace(r, (dx_ + math.copysign(1.9, dx_), dy_ + math.copysign(1.9, dy_)), (dx_, dy_), opp,
                              max_r=2.0)
            except RuntimeError as e:
                print("  !", e)
    side_of_sheet = {"/Motor 1 ESC/": "F", "/Motor 4 ESC/": "F", "/Motor 2 ESC/": "B", "/Motor 3 ESC/": "B",
                     "/Flight controller STM32G431/": "F", "/IMU LSM6DSV16X/": "B", "/Radio SX1280/": RADIO_SIDE,
                     "/Power/": "F"}
    jx, jy = pcb.pad_xy("J2", "1")
    pending = sorted((r for r in pcb._unplaced if (r[0] in "CRLD" or r.startswith("FB"))
                      and not r.startswith("FID")),
                     key=lambda r: (comp[r]["sheet"], r))
    failed = []
    progress = True
    while pending and progress:
        progress = False
        for ref in list(pending):
            a = anchor_for(ref)
            if a is None:
                continue
            ar, anum, net = a
            # defer chain parts whose upstream neighbour is not placed yet
            if ar.startswith("U") and ar not in SHEET_ICS.get(comp[ref]["sheet"], []):
                upstream = [r for n in part_nets(ref) if n != "GND" for r, _ in nl.nets[n]
                            if r != ref and r[0] in "CRL" and r in pcb._unplaced]
                if upstream and any(r in pending for r in upstream):
                    continue
            ax, ay = pcb.pad_xy(ar, anum)
            if pcb.is_tht(ar) or not ar.startswith("U"):
                side = pcb.side[ar] if not pcb.is_tht(ar) else side_of_sheet.get(comp[ref]["sheet"], "F")
            else:
                side = pcb.side[ar]
            if any(n in RF_CHAIN for n in part_nets(ref)):
                cx, cy = 2 * ax - jx, 2 * ay - jy          # prefer heading toward the antenna pads
            elif ar.startswith("U"):
                cx, cy = ic_center(ar)
            else:
                cx, cy = 0.0, 0.0
            try:
                pcb.autoplace(ref, (ax, ay), (cx, cy), side, max_r=9.0)
                taken[(ar, anum)] = ref
            except RuntimeError as e:
                failed.append(ref)
                print("  !", e)
            pending.remove(ref)
            progress = True
    failed += pending
    # SWD pogo strip (top, rear-left corner, under the battery) + fiducials
    try:
        pcb.autoplace("J3", (-(HUB - 3.8), 0.0), (0, 0), "F", max_r=6.0)
    except RuntimeError as e:
        failed.append("J3")
        print("  !", e)
    # rev B option (NANODRONE_ESC_TP=1): per-ESC SWD pogo pads next to each ESC MCU's SWD pins
    for m in (1, 2, 3, 4):
        ref = "J%d" % (20 + m)
        if ref not in pcb._unplaced:
            continue
        u = "U%d" % (20 + m)
        ax, ay = pcb.pad_xy(u, "20")
        for s in (pcb.side[u], "B" if pcb.side[u] == "F" else "F"):
            try:
                pcb.autoplace(ref, (ax, ay), ic_center(u), s, max_r=6.0)
                break
            except RuntimeError as e:
                print("  !", e)
        else:
            failed.append(ref)
    for ref, (x, y, s) in zip(sorted(r for r in pcb._unplaced if r.startswith("FID")),
                              [(-(HUB - 1.6), 3.0, "F"), (HUB - 1.6, -3.5, "F"), (HUB - 1.6, 3.0, "B"),
                               (-(HUB - 1.4), -4.5, "B")]):
        try:
            pcb.autoplace(ref, (x, y), (0, 0), s, max_r=6.0)
        except RuntimeError as e:
            failed.append(ref)
            print("  !", e)
    print("placed; unplaced/failed:", sorted(failed), "still unplaced:", sorted(pcb._unplaced))
    rf_parts = {r for r, f in pcb.fps.items() if {p.GetNetname() for p in f.Pads()} & {"RF_IO", "RF_N1", "RF_N2", "ANT"} and r not in ("U4", "J2")}
    bemf_top = {r for r, f in pcb.fps.items() if r.startswith("R") and
                any(re.fullmatch(r"M\d_OUT[ABC]", p.GetNetname()) for p in f.Pads())}
    pcb.rewrite_courtyards(tight=rf_parts | bemf_top, tight_margin=0.02)
    pcb.save()
    return pcb


if __name__ == "__main__":
    build()
