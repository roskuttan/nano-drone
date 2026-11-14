"""Nano drone PCB - layout as code (stage 1: outline, stackup rules, placement, zones).

Frame: compact stacked quad, 0702 motors, 31 mm props, adjacent motor spacing 26 mm
(diagonal 36.77 mm). Motors (Betaflight order): M1 rear-right TOP, M2 front-right BOTTOM,
M3 rear-left BOTTOM, M4 front-left TOP. Coordinates in mm, board centre = (0, 0), +y = rear.
Each motor driver sits on the same side as its motor.
"""
import json
import math
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
PCB_PATH = os.path.join(PRJ, "nanodrone.kicad_pcb")
NET_PATH = os.path.join(PRJ, "nanodrone.net")

# ---------------------------------------------------------------------------
# Geometry parameters
# ---------------------------------------------------------------------------
SPACING = 26.0                     # adjacent motor spacing
MC = SPACING / 2                   # motor centre offset (13, 13)
HUB = 9.0                          # hub half-size (18 x 18 mm)
HUB_R = 2.0                        # hub corner radius
DISC_R = 5.0                       # motor mount disc radius
ARM_W = 6.5                        # arm width
FILLET = 1.2                       # outline fillet radius
MOTORS = {1: (MC, MC, "F"), 2: (MC, -MC, "B"), 3: (-MC, MC, "B"), 4: (-MC, -MC, "F")}

POWER_NETS = {"GND", "VBAT", "+3V0"}

SHEET_IC = {"/Power/": "U1", "/MCU STM32G474/": "U2", "/IMU LSM6DSV16X/": "U3", "/Radio SX1280/": "U4",
            "/Motor 1 ESC/": "U11", "/Motor 2 ESC/": "U12", "/Motor 3 ESC/": "U13", "/Motor 4 ESC/": "U14"}


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
        # wire pads on the arm, row perpendicular to the arm, 12.4 mm from centre
        d = math.hypot(x, y)
        ux, uy = x / d, y / d
        r_pad = d - DISC_R - 0.2
        jx, jy = ux * r_pad, uy * r_pad
        rot = -math.degrees(math.atan2(uy, ux)) + 90
        fixed("J%d" % (10 + m), jx, jy, rot, side)

    # ---------------- connectors ----------------
    fixed("J1", HUB - 1.6, 0.0, 90, "F")             # battery pads, right edge
    fixed("J2", -1.8, -(HUB - 1.1), 0, "B")          # antenna pads, front edge (wire runs forward in-plane)

    # ---------------- ICs ----------------
    fixed("U2", 0.0, 0.0, 180, "F")                  # MCU (rotation from pinopt)
    ic_pos = {"U11": (5.7, 5.7, "F"), "U14": (-5.7, -5.7, "F"),
              "U12": (5.7, -5.7, "B"), "U13": (-5.7, 5.7, "B"),
              "U3": (0.0, 4.9, "B"),                  # IMU, rear-centre bottom
              "U4": (-4.7, -1.9, "B"),                # radio, front-left bottom
              "U1": (5.2, 2.5, "F")}                  # LDO next to the battery pads
    for ref, (x, y, side) in ic_pos.items():
        fixed(ref, x, y, 0, side)
    XTAL = (-7.4, 0.9)
    fixed("Y1", XTAL[0], XTAL[1], 0, "B")            # 52 MHz crystal beside the radio

    # keep-outs: other side of through-via fields
    def ko(side, cx, cy, hx, hy):
        pcb.keepouts[side].append(P.Box(cx - hx, cy - hy, cx + hx, cy + hy))

    ko("B", 0, 0, 2.45, 2.6)                         # MCU via-in-pad field
    for ref, (x, y, side) in ic_pos.items():
        if ref in ("U11", "U12", "U13", "U14"):
            ko("B" if side == "F" else "F", x, y, 1.2, 1.2)   # DRV thermal vias
    ko("F", ic_pos["U4"][0], ic_pos["U4"][1], 1.6, 1.6)      # radio EP vias

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

    for _ in range(2):
        for ref in ("U11", "U12", "U13", "U14", "U3", "U4", "U1"):
            x, y, side = ic_pos[ref]
            best = None
            for rot in (0, 90, 180, 270):
                pcb.place(ref, x, y, rot, side)
                c = ratsnest_cost(ref)
                if best is None or c < best[0]:
                    best = (c, rot)
            pcb.place(ref, x, y, best[1], side)
    for ref in ("Y1",):
        best = None
        for rot in (0, 90, 180, 270):
            pcb.place(ref, XTAL[0], XTAL[1], rot, "B")
            c = ratsnest_cost(ref)
            if best is None or c < best[0]:
                best = (c, rot)
        pcb.place(ref, XTAL[0], XTAL[1], best[1], "B")

    # ---------------- passives ----------------
    taken = {}

    def ic_center(ref):
        return P.XY(pcb.fps[ref].GetPosition())

    RF_CHAIN = ["RF_IO", "RF_N1", "RF_N2", "RF_N3", "ANT"]

    def part_nets(ref):
        return [n for n in dict.fromkeys(p.GetNetname() for p in pcb.fps[ref].Pads()) if n]

    def anchor_for(ref):
        """(anchor_ref, pad, net) - 1) sheet IC pin on the most specific non-GND net,
        2) an already placed passive on such a net, 3) any IC on such a net."""
        ic = SHEET_IC.get(comp[ref]["sheet"])
        nets = sorted((n for n in part_nets(ref) if n != "GND"), key=lambda n: len(nl.nets.get(n, [])))
        for n in nets:
            pads = [num for r, num in nl.nets[n] if r == ic]
            if pads:
                free = [num for num in pads if (ic, num) not in taken] or pads
                return ic, free[0], n
        for n in nets:
            for r, num in nl.nets[n]:
                if r != ref and r not in pcb._unplaced and not r.startswith("U"):
                    return r, num, n
        for n in nets:
            for r, num in nl.nets[n]:
                if r.startswith("U") and r != ref:
                    return r, num, n
        return None

    side_of_sheet = {"/Motor 1 ESC/": "F", "/Motor 4 ESC/": "F", "/Motor 2 ESC/": "B", "/Motor 3 ESC/": "B",
                     "/MCU STM32G474/": "F", "/IMU LSM6DSV16X/": "B", "/Radio SX1280/": "B", "/Power/": "F"}
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
            if ar.startswith("U") and ar != SHEET_IC.get(comp[ref]["sheet"]):
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
        pcb.autoplace("J3", (-4.0, 5.0), (0, 0), "F", max_r=6.0, prefer_rot=45)
    except RuntimeError as e:
        failed.append("J3")
        print("  !", e)
    for ref, (x, y, s) in zip(sorted(r for r in pcb._unplaced if r.startswith("FID")),
                              [(-7.6, 2.5, "F"), (7.6, -2.5, "F"), (7.6, 2.5, "B"), (-7.6, -4.5, "B")]):
        try:
            pcb.autoplace(ref, (x, y), (0, 0), s, max_r=6.0)
        except RuntimeError as e:
            failed.append(ref)
            print("  !", e)
    print("placed; unplaced/failed:", sorted(failed), "still unplaced:", sorted(pcb._unplaced))
    pcb.rewrite_courtyards()
    pcb.save()
    return pcb


if __name__ == "__main__":
    build()
