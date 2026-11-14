"""pinopt - min-cost MCU pin assignment for the nano drone (STM32G474MEYx, WLCSP81).

Hard constraints (from ST open pin data XML):
  * motor k uses one timer for INH A/B/C on three *distinct* channels (CH1..CH4)
  * motor k uses one ADC instance for its 3 BEMF + 2 current-sense inputs, so the
    four motors sample in parallel, each triggered by its own timer
  * IMU and radio each get a full SPI instance; SWD/BOOT0/NRST pins are reserved
  * backup-domain pins (PC13..PC15) only take input signals
Soft objective: each signal group lands on balls facing the part it connects to
(given the MCU rotation), so the fan-out is short and does not cross.

Search: per (rotation), rank timer->motor and ADC->motor maps with small Hungarian
problems, then solve the full slot->ball assignment (Hungarian) for the best few
combinations and keep the cheapest.
"""
import itertools
import json
import math
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

NS = {"m": "http://dummy.com"}
BIG = 1e6

RESERVED = {"PA13", "PA14", "PB8", "PG10"}           # SWDIO, SWCLK, BOOT0, NRST
INPUT_ONLY = {"PC13", "PC14", "PC15"}
TIMERS = ["TIM1", "TIM8", "TIM20", "TIM2", "TIM3", "TIM4", "TIM5"]
ADCS = ["ADC1", "ADC2", "ADC3", "ADC4", "ADC5"]

# board directions (KiCad coords: +x right, +y down = rear). Betaflight quad-X numbering.
MOTOR_DIR = {1: (1, 1), 2: (1, -1), 3: (-1, 1), 4: (-1, -1)}
GROUP_DIR = {"RF": (0, 1), "VBAT": (0.3, 1), "LED": (0, -1), "DRV": None, "IMU": None}


# ----------------------------------------------------------------------------
def hungarian(cost):
    """Min-cost assignment for an n x m matrix (n <= m). Returns col index per row."""
    n, m = cost.shape
    INF = float("inf")
    u = np.zeros(n + 1)
    v = np.zeros(m + 1)
    p = np.zeros(m + 1, dtype=int)      # p[j] = row matched to column j (1-based), 0 = free
    way = np.zeros(m + 1, dtype=int)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(m + 1, INF)
        used = np.zeros(m + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            cur = cost[i0 - 1] - u[i0] - v[1:]
            free = ~used[1:]
            upd = free & (cur < minv[1:])
            minv[1:][upd] = cur[upd]
            way[1:][upd] = j0
            cand = np.where(free, minv[1:], INF)
            j1 = int(np.argmin(cand)) + 1
            delta = cand[j1 - 1]
            u[p[used]] += delta
            v[used] -= delta
            minv[1:][free] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    res = np.full(n, -1)
    for j in range(1, m + 1):
        if p[j]:
            res[p[j] - 1] = j - 1
    return res


# ----------------------------------------------------------------------------
def load_pins(xml_path):
    root = ET.parse(xml_path).getroot()
    pins = {}
    for p in root.findall("m:Pin", NS):
        if p.get("Type") != "I/O":
            continue
        name = re.sub(r"[-_ (].*$", "", p.get("Name"))
        pins[name] = dict(ball=p.get("Position"), full=p.get("Name"),
                          sigs=[s.get("Name") for s in p.findall("m:Signal", NS)])
    return root.get("RefName"), pins


def ball_xy(footprint_dir, footprint):
    import pcbnew
    fp = pcbnew.FootprintLoad(footprint_dir, footprint)
    return {p.GetNumber(): (p.GetPosition().x / 1e6, p.GetPosition().y / 1e6) for p in fp.Pads()}


def rot(xy, r):
    x, y = xy
    for _ in range(r % 4):          # 90 deg steps, KiCad CCW visual with y down
        x, y = y, -x
    return x, y


class Problem:
    def __init__(self, xml_path, balls):
        self.ref, self.pins = load_pins(xml_path)
        self.balls = balls
        self.avail = sorted(p for p in self.pins if p not in RESERVED)
        self.idx = {p: i for i, p in enumerate(self.avail)}
        s = lambda p: self.pins[p]["sigs"]
        self.tim = {}   # tim -> ch -> [pins]
        self.adc = {}   # adc -> [pins]
        self.spi = {}   # spi -> fn -> [pins]
        for p in self.avail:
            for sig in s(p):
                m = re.fullmatch(r"(TIM\d+)_CH([1-4])", sig)
                if m and p not in INPUT_ONLY:
                    self.tim.setdefault(m.group(1), {}).setdefault(int(m.group(2)), []).append(p)
                m = re.fullmatch(r"(ADC\d)_IN\d+", sig)
                if m and p not in INPUT_ONLY:
                    self.adc.setdefault(m.group(1), set()).add(p)
                m = re.fullmatch(r"(SPI\d)_(SCK|MISO|MOSI)", sig)
                if m and p not in INPUT_ONLY:
                    self.spi.setdefault(m.group(1), {}).setdefault(m.group(2), []).append(p)
        self.spis = [k for k, v in self.spi.items() if len(v) == 3]

    def pos(self, pin, r):
        return rot(self.balls[self.pins[pin]["ball"]], r)

    RING_COST = {0: 0.0, 1: 2.0, 2: 6.0, 3: 12.0, 4: BIG}   # HDI escape depth (L1 / L2 / L3 / L6 / none)
    ANALOG_RING_EXTRA = {0: 0.0, 1: 3.0, 2: 9.0, 3: 18.0, 4: BIG}

    def ring(self, pin):
        ball = self.pins[pin]["ball"]
        r, c = "ABCDEFGHJ".index(ball[0]), int(ball[1:]) - 1
        return min(r, 8 - r, c, 8 - c)

    def dcost(self, pin, direction, r, analog=False):
        x, y = self.pos(pin, r)
        k = self.ring(pin)
        pen = self.RING_COST[k] + (self.ANALOG_RING_EXTRA[k] if analog else 0.0)
        if direction is None:
            return pen
        dx, dy = direction
        n = math.hypot(dx, dy)
        tx, ty = 12 * dx / n, 12 * dy / n          # a point well outside the package
        return math.hypot(x - tx, y - ty) + pen


def build_slots(P, tmap, chsel, amap, imu_spi, rf_spi, n_isense):
    """Return list of (name, direction, candidate pin list)."""
    slots = []
    gpio = [p for p in P.avail if p not in INPUT_ONLY]
    anyin = list(P.avail)
    for m in (1, 2, 3, 4):
        t, chs, a = tmap[m], chsel[m], amap[m]
        d = MOTOR_DIR[m]
        for ph, ch in zip("ABC", chs):
            slots.append(("M%d_INH%s" % (m, ph), d, P.tim[t][ch], "%s_CH%d" % (t, ch)))
        for ph in "ABC":
            slots.append(("M%d_INL%s" % (m, ph), d, gpio, "GPIO"))
            slots.append(("M%d_BEMF%s" % (m, ph), d, sorted(P.adc[a]), a))
        for ph in "AB"[:n_isense]:
            slots.append(("M%d_ISENS%s" % (m, ph), d, sorted(P.adc[a]), a))
    allc = sorted(set().union(*P.adc.values()))
    slots.append(("VBAT_SENSE", GROUP_DIR["VBAT"], allc, "ADC"))
    for fn in ("SCK", "MISO", "MOSI"):
        slots.append(("IMU_%s" % fn, None, P.spi[imu_spi][fn], imu_spi))
        slots.append(("RF_%s" % fn, GROUP_DIR["RF"], P.spi[rf_spi][fn], rf_spi))
    slots += [("IMU_CS", None, gpio, "GPIO"), ("IMU_INT1", None, anyin, "GPIO in"),
              ("IMU_INT2", None, anyin, "GPIO in")]
    slots += [("RF_NSS", GROUP_DIR["RF"], gpio, "GPIO"), ("RF_NRESET", GROUP_DIR["RF"], gpio, "GPIO"),
              ("RF_BUSY", GROUP_DIR["RF"], anyin, "GPIO in"), ("RF_DIO1", GROUP_DIR["RF"], anyin, "GPIO in")]
    slots += [("MOT_nSLEEP", None, gpio, "GPIO"), ("MOT_nFAULT", None, anyin, "GPIO in"),
              ("LED", GROUP_DIR["LED"], gpio, "GPIO")]
    return slots


ANALOG_SLOTS = ("BEMF", "ISENS", "VBAT_SENSE")


def solve_full(P, slots, r):
    n, m = len(slots), len(P.avail)
    if n > m:
        return None
    C = np.full((n, m), BIG)
    for i, (name, d, cands, _) in enumerate(slots):
        analog = any(k in name for k in ANALOG_SLOTS)
        for p in cands:
            C[i, P.idx[p]] = P.dcost(p, d, r, analog)
    a = hungarian(C)
    total = float(sum(C[i, a[i]] for i in range(n)))
    if total >= BIG:
        return None
    return total, {slots[i][0]: (P.avail[a[i]], slots[i][3]) for i in range(n)}


def top_maps(P, r, kind, n_isense, k=6):
    """Rank timer->motor or ADC->motor maps by their group-only cost."""
    res = []
    if kind == "tim":
        cands = [t for t in TIMERS if t in P.tim and len(P.tim[t]) >= 3]
        for combo in itertools.permutations(cands, 4):
            cost, sel = 0.0, {}
            for m, t in zip((1, 2, 3, 4), combo):
                best = None
                for chs in itertools.combinations(sorted(P.tim[t]), 3):
                    c = sum(min(P.dcost(p, MOTOR_DIR[m], r) for p in P.tim[t][ch]) for ch in chs)
                    if best is None or c < best[0]:
                        best = (c, chs)
                cost += best[0]
                sel[m] = best[1]
            res.append((cost, dict(zip((1, 2, 3, 4), combo)), sel))
    else:
        for combo in itertools.permutations([a for a in ADCS if a in P.adc], 4):
            cost = 0.0
            for m, a in zip((1, 2, 3, 4), combo):
                cs = sorted(P.dcost(p, MOTOR_DIR[m], r) for p in P.adc[a])
                if len(cs) < 3 + n_isense:
                    cost = BIG
                    break
                cost += sum(cs[:3 + n_isense])
            res.append((cost, dict(zip((1, 2, 3, 4), combo)), None))
    res.sort(key=lambda x: x[0])
    return res[:k]


def optimise(xml_path, balls, n_isense=2):
    P = Problem(xml_path, balls)
    print("%s: %d usable I/O; timers %s; ADC pins %s; SPI %s" % (
        P.ref, len(P.avail), {t: sorted(c) for t, c in P.tim.items()},
        {a: len(v) for a, v in sorted(P.adc.items())}, P.spis))
    best = None
    for r in range(4):
        tms = top_maps(P, r, "tim", n_isense)
        ams = top_maps(P, r, "adc", n_isense)
        for (tc, tmap, chsel), (ac, amap, _) in itertools.product(tms, ams):
            for imu_spi, rf_spi in itertools.permutations(P.spis, 2):
                slots = build_slots(P, tmap, chsel, amap, imu_spi, rf_spi, n_isense)
                sol = solve_full(P, slots, r)
                if sol and (best is None or sol[0] < best[0]):
                    best = (sol[0], r, tmap, amap, imu_spi, rf_spi, sol[1])
        print("  rotation %d done, best so far %s" % (r, None if not best else round(best[0], 1)))
    return P, best


if __name__ == "__main__":
    xml, fpdir, fpname, out = sys.argv[1:5]
    P, best = optimise(xml, ball_xy(fpdir, fpname))
    if not best:
        print("NO FEASIBLE ASSIGNMENT")
        sys.exit(1)
    cost, r, tmap, amap, imu_spi, rf_spi, asg = best
    print("BEST cost %.1f rotation %d deg; timers %s; ADCs %s; IMU %s; RF %s"
          % (cost, 90 * r, tmap, amap, imu_spi, rf_spi))
    used = {p for p, _ in asg.values()}
    spare = sorted(set(P.avail) - used)
    print("spare:", spare, "rings:", [(p, P.ring(p)) for p in spare])
    rows = []
    ring_hist = {}
    for k in sorted(asg):
        p, fn = asg[k]
        ring_hist.setdefault(P.ring(p), []).append(k)
        rows.append(dict(net=k, pin=p, ball=P.pins[p]["ball"], function=fn, ring=P.ring(p)))
        print("  %-12s %-5s %-3s ring%d %s" % (k, p, P.pins[p]["ball"], P.ring(p), fn))
    for k in sorted(ring_hist):
        print("ring %d: %d signals %s" % (k, len(ring_hist[k]), sorted(ring_hist[k]) if k >= 2 else ""))
    json.dump(dict(mcu=P.ref, rotation_deg=90 * r, timers=tmap, adcs=amap, imu_spi=imu_spi,
                   rf_spi=rf_spi, spare=spare, pins=rows), open(out, "w"), indent=1)
