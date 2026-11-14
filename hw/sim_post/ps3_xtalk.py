"""Post-layout study P3: motor-phase crosstalk into every other net, and signal return-path vias.

Method
------
1. 2-D field solves (fd2d) of the three routing-layer cross-sections (L1 and L6 microstrip under 10 um LPI mask,
   L3 stripline between the L2 GND and L4 VBAT planes). They give the mutual capacitance c_m(s) and mutual inductance
   l_m(s) per metre between a phase track (0.1 / 0.2 / 0.5 mm) and a 0.1 mm victim at edge spacing s.
   The L2 and L5 GND planes screen every adjacent layer pair except L3/L4: L4 carries escape routing, and where it
   does there is no VBAT plane between a phase track on L3 and a victim on L4. A fourth solve covers that case
   (L3 over L4, between the L2 and L5 planes, as a function of the centre offset in plan view). It leaves out the L4
   copper around the victim, which is conservative because that copper would screen part of the field.
2. Every victim track is walked in 0.05 mm steps. At each step the nearest phase track on the same layer is found. If
   it is within 1 mm edge-to-edge and roughly parallel (|cos| > 0.5), c_m(s) and l_m(s) accumulate over that step,
   weighted by |cos|. A victim on L4 is also checked against the nearest phase track on L3 (centre offset ≤ 0.8 mm).
3. Aggressor: SR 55 V/us (max at SLEW=AGND [DS]) and di/dt 1.5 A/us (VM/L_phase). Victim: the Thevenin impedance of
   the node class (below). Peak glitch = I*R*(1-exp(-t_edge/RC)), plus the inductive part l_m*l*di/dt.
4. Return path: every signal via that changes reference plane is checked for a GND via within 1.0 mm.

Outputs: out/p3_xtalk.png, out/p3_xtalk.json
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postlib as PL  # noqa: E402
import fd2d  # noqa: E402
import numpy as np  # noqa: E402

S = PL.S
plt = PL.plt
SR_MAX = 55e6
DIDT = 1.5e6
VM = 4.2
PHASE_RE = re.compile(r"^M\d_OUT[ABC]$")
SKIP = {"GND", "VBAT"}

# victim node classes: regex -> (label, R_thevenin ohm, C_node F, threshold V, why)
CLASSES = [
    (r"^M\d_BEMF[ABC]$", "BEMF divider", 5e3, 5e-12, 0.0073, "10 LSB at 12 bit, during the sample window"),
    (r"^M\d_ISENS[ABC]$", "CSA output", 50.0, 5e-12, 0.0073, "10 LSB"),
    (r"^VBAT_SENSE$", "VBAT divider + 100 nF", 5e3, 100e-9, 0.0073, "10 LSB"),
    (r"^M\d_nFAULT$", "open-drain + 10k", 10e3, 7e-12, 0.9, "V_IL margin 0.3*VDD"),
    (r"(NRST|NRESET)$", "reset (40k pull-up)", 40e3, 7e-12, 0.9, "spurious reset"),
    (r"(SWDIO|SWCLK|BOOT0)$", "debug/boot (40k)", 40e3, 7e-12, 0.9, "V_IL margin"),
    (r"^RF_XT[AB]$", "52 MHz crystal", 1e3, 5e-12, 0.010, "spur/phase-noise budget"),
    (r".*", "push-pull logic", 40.0, 5e-12, 0.9, "V_IL/V_IH margin"),
]


def classify(net):
    for rx, lab, R, C, thr, why in CLASSES:
        if re.search(rx, net):
            return lab, R, C, thr, why
    raise ValueError(net)


def section(layer, wa, wv, s):
    """Aggressor (index 0) and victim (index 1) side by side, edge spacing s (all mm)."""
    mm = 1e-3
    tm = 10e-6
    erm = 3.8
    if layer in (0, 5):
        h, er, t = PL.T_DIEL[0], PL.ER_DIEL[0], PL.T_CU[0]
        a = [(-wa * mm, 0.0, h, h + t)]
        v = [(s * mm, (s + wv) * mm, h, h + t)]
        gnd = [(-3e-3, 3e-3, -0.2e-3, 0.0)]
        slabs = [(0.0, h, er)]
        extra = [(-3e-3, 3e-3, h, h + t + tm, erm)]
        box = (-3e-3, 3e-3, -0.2e-3, 2.5e-3)
    elif layer == 3:              # L4 escape routing: L5 GND below (core), L2 GND above (prepreg + core; the L3 pour
        t = PL.T_CU[3]            # in between is left out, which is conservative)
        y0 = PL.T_DIEL[3]
        ytop = y0 + PL.T_DIEL[2] + PL.T_DIEL[1]
        a = [(-wa * mm, 0.0, y0, y0 + t)]
        v = [(s * mm, (s + wv) * mm, y0, y0 + t)]
        gnd = [(-3e-3, 3e-3, -0.05e-3, 0.0), (-3e-3, 3e-3, ytop, ytop + 0.05e-3)]
        slabs = [(0.0, y0, PL.ER_DIEL[3]), (y0, y0 + PL.T_DIEL[2], PL.ER_DIEL[2]), (y0 + PL.T_DIEL[2], ytop, PL.ER_DIEL[1])]
        extra = []
        box = (-3e-3, 3e-3, -0.05e-3, ytop + 0.05e-3)
    else:                         # L3: core (L2 side) above, prepreg (L4 side) below with the trace embedded
        t = PL.T_CU[2]
        hb = PL.T_DIEL[2] - t     # trace bottom to L4
        ht = PL.T_DIEL[1]         # trace top to L2
        y0 = hb
        a = [(-wa * mm, 0.0, y0, y0 + t)]
        v = [(s * mm, (s + wv) * mm, y0, y0 + t)]
        gnd = [(-3e-3, 3e-3, -0.05e-3, 0.0), (-3e-3, 3e-3, y0 + t + ht, y0 + t + ht + 0.05e-3)]
        slabs = [(0.0, y0 + t, PL.ER_DIEL[2]), (y0 + t, y0 + t + ht, PL.ER_DIEL[1])]
        extra = []
        box = (-3e-3, 3e-3, -0.05e-3, y0 + t + ht + 0.05e-3)
    sec = fd2d.Section([a, v], gnd, slabs, extra, box=box, dmin=3e-6, dmax=60e-6)
    L, C, C0 = sec.lc()
    return -C[0, 1], L[0, 1], C[1, 1], L[1, 1]


WAS = (0.1, 0.2, 0.5)                                  # phase track widths solved (sense branch, signal, power)
BS_DX = np.array([0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.65, 0.8])


def broadside(wa, wv, dx, agg="L3"):
    """Phase track (width wa, mm) on L3 over a victim on L4 (width wv), or with agg="L4" on L4 under a victim on L3,
    centre offset dx in plan view, between the L2 and L5 GND planes; no VBAT plane between them (L4 escape routing).
    Returns c_m, l_m per metre."""
    mm = 1e-3
    t3, t4 = PL.T_CU[2], PL.T_CU[3]
    y4 = PL.T_DIEL[3]                              # L4 copper base = top face of the L4-L5 core
    y3 = y4 + PL.T_DIEL[2] - t3                    # L3 copper on the bottom face of the L2-L3 core
    ytop = y4 + PL.T_DIEL[2] + PL.T_DIEL[1]        # L2 plane
    ya, ta, yv, tv = (y3, t3, y4, t4) if agg == "L3" else (y4, t4, y3, t3)
    a = [((dx - wa / 2) * mm, (dx + wa / 2) * mm, ya, ya + ta)]
    v = [(-wv / 2 * mm, wv / 2 * mm, yv, yv + tv)]
    gnd = [(-3e-3, 3e-3, -0.05e-3, 0.0), (-3e-3, 3e-3, ytop, ytop + 0.05e-3)]
    slabs = [(0.0, y4, PL.ER_DIEL[3]), (y4, y4 + PL.T_DIEL[2], PL.ER_DIEL[2]), (y4 + PL.T_DIEL[2], ytop, PL.ER_DIEL[1])]
    sec = fd2d.Section([a, v], gnd, slabs, box=(-3e-3, 3e-3, -0.05e-3, ytop + 0.05e-3), dmin=3e-6, dmax=60e-6)
    L, C, C0 = sec.lc()
    return -C[0, 1], L[0, 1]


def bs_tables(agg="L3"):
    out = {}
    for wa in WAS:
        r = np.array([broadside(wa, 0.1, dx, agg) for dx in BS_DX])
        out[wa] = (r[:, 0], r[:, 1])
        print("  broadside, %s phase %.1f mm, victim on %s: c_m(0)=%.1f pF/m c_m(0.2)=%.1f pF/m l_m(0)=%.0f nH/m"
              % (agg, wa, "L4" if agg == "L3" else "L3", 1e12 * r[0, 0], 1e12 * np.interp(0.2, BS_DX, r[:, 0]),
                 1e9 * r[0, 1]), flush=True)
    return out


def bs_coupling(bst, wa, dx):
    wa = min(max(wa, WAS[0]), WAS[-1])
    cms = [np.interp(dx, BS_DX, bst[w][0]) for w in WAS]
    lms = [np.interp(dx, BS_DX, bst[w][1]) for w in WAS]
    return float(np.interp(wa, WAS, cms)), float(np.interp(wa, WAS, lms))


def tables():
    seps = np.array([0.08, 0.1, 0.13, 0.16, 0.2, 0.25, 0.3, 0.4, 0.5, 0.65, 0.8, 1.0, 1.2])
    out = {}
    for layer in (0, 2, 3):
        for wa in WAS:
            cm, lm, cv, lv = [], [], [], []
            for s in seps:
                a, b, c, d = section(layer, wa, 0.1, s)
                cm.append(a)
                lm.append(b)
                cv.append(c)
                lv.append(d)
            out[layer, wa] = (np.array(cm), np.array(lm), np.array(cv), np.array(lv))
            print("  layer %s, phase %.1f mm: c_m(0.1)=%.1f pF/m c_m(0.3)=%.1f pF/m l_m(0.1)=%.0f nH/m, victim C %.0f pF/m" % (
                {0: "L1/L6", 2: "L3", 3: "L4"}[layer], wa, 1e12 * cm[1], 1e12 * cm[6], 1e9 * lm[1], 1e12 * cv[1]), flush=True)
    for wa in WAS:
        out[5, wa] = out[0, wa]
    return seps, out


def coupling(seps, tab, layer, wa, s):
    wa = min(max(wa, WAS[0]), WAS[-1])
    cms = [np.interp(s, seps, tab[layer, w][0]) for w in WAS]
    lms = [np.interp(s, seps, tab[layer, w][1]) for w in WAS]
    return float(np.interp(wa, WAS, cms)), float(np.interp(wa, WAS, lms))


def seg_point_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / np.where(L2 == 0, 1, L2), 0, 1)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def main():
    S.style()
    print("field-solver tables:", flush=True)
    seps, tab = tables()
    bst = {2: bs_tables("L3"), 3: bs_tables("L4")}       # by aggressor layer index
    b = PL.load()
    # nFAULT victims: 10k external pull-up if the board has one, else the ESC MCU's internal pull-up (55k worst case)
    pu = any(f.GetReference().startswith("R") and {p.GetNetname() for p in f.Pads()} - {""} >= {"+3V0"} and
             any(re.fullmatch(r"M\d_nFAULT", p.GetNetname()) for p in f.Pads()) for f in b.GetFootprints())
    if not pu:
        CLASSES[3] = (r"^M\d_nFAULT$", "open-drain + internal pull-up", 55e3, 7e-12, 0.9, "V_IL margin 0.3*VDD")
    print("nFAULT pull-up: %s" % ("10k external" if pu else "MCU internal, 55k worst case"), flush=True)
    tr = PL.tracks(b)
    agg = [t for t in tr if PHASE_RE.match(t[0])]
    vict = [t for t in tr if not PHASE_RE.match(t[0]) and t[0] not in SKIP and t[0]]
    print("tracks: %d phase segments, %d victim segments (%d on L4)" % (
        len(agg), len(vict), sum(1 for t in vict if t[1] == 3)), flush=True)
    A = {l: np.array([(x0, y0, x1, y1, w) for (n, ll, x0, y0, x1, y1, w) in agg if ll == l]).reshape(-1, 5)
         for l in range(6)}
    An = {l: [n for (n, ll, *_r) in agg if ll == l] for l in range(6)}
    BS = {3: 2, 2: 3}      # victim layer -> aggressor layer with no plane between them (L3 and L4 where L4 is routed)
    per = {}

    def nearest(aa, px, py, ux, uy, edge_w):
        """index, distance (edge-to-edge if edge_w is given, else centre line) and |cos| of the nearest track."""
        dist = seg_point_dist(px, py, aa[:, 0], aa[:, 1], aa[:, 2], aa[:, 3])
        if edge_w is not None:
            dist = dist - aa[:, 4] / 2 - edge_w / 2
        j = int(np.argmin(dist))
        al = math.hypot(aa[j, 2] - aa[j, 0], aa[j, 3] - aa[j, 1])
        cosang = abs(ux * (aa[j, 2] - aa[j, 0]) / al + uy * (aa[j, 3] - aa[j, 1]) / al) if al > 1e-6 else 0.0
        return j, float(dist[j]), cosang

    for (net, l, x0, y0, x1, y1, w) in vict:
        same = l in (0, 2, 3, 5) and len(A[l]) > 0
        bl = BS.get(l)
        brd = bl is not None and len(A[bl]) > 0
        L = math.hypot(x1 - x0, y1 - y0)
        if not (same or brd) or L < 1e-6:
            continue
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        ns = max(1, int(L / 0.05))
        d = per.setdefault(net, dict(len=0.0, cm=0.0, lm=0.0, coupled=0.0, s_min=9.9, bs_mm=0.0, aggressors={}))
        d["len"] += L
        for k in range(ns):
            px = x0 + (x1 - x0) * (k + 0.5) / ns
            py = y0 + (y1 - y0) * (k + 0.5) / ns
            hits = []
            if same:
                j, s, cosang = nearest(A[l], px, py, ux, uy, w)
                if s <= 1.0 and cosang >= 0.5:
                    cm, lm = coupling(seps, tab, l, A[l][j, 4], max(s, 0.08))
                    hits.append((cm, lm, cosang, s, An[l][j], False))
            if brd:
                j, dx, cosang = nearest(A[bl], px, py, ux, uy, None)
                if dx <= BS_DX[-1] and cosang >= 0.5:
                    cm, lm = bs_coupling(bst[bl], A[bl][j, 4], dx)
                    hits.append((cm, lm, cosang, max(dx - A[bl][j, 4] / 2 - w / 2, 0.0), An[bl][j], True))
            for cm, lm, cosang, s, an, is_bs in hits:
                dl = (L / ns) * 1e-3 * cosang
                d["cm"] += cm * dl
                d["lm"] += lm * dl
                d["coupled"] += dl * 1e3
                if is_bs:
                    d["bs_mm"] += dl * 1e3
                d["s_min"] = min(d["s_min"], s)
                d["aggressors"][an] = d["aggressors"].get(an, 0.0) + cm * dl
    rows = []
    for net, d in per.items():
        lab, R, C, thr, why = classify(net)
        t_edge = VM / SR_MAX
        I = d["cm"] * SR_MAX
        tau = R * (C + d["cm"])
        v_c = I * R * (1 - math.exp(-t_edge / tau)) if tau > 0 else I * R
        v_l = d["lm"] * DIDT
        v = v_c + v_l
        rows.append(dict(net=net, cls=lab, len_mm=d["len"], coupled_mm=d["coupled"], s_min=d["s_min"],
                         bs_mm=d["bs_mm"],
                         cm_fF=d["cm"] * 1e15, lm_pH=d["lm"] * 1e12, v_cap_mV=v_c * 1e3, v_ind_mV=v_l * 1e3,
                         v_mV=v * 1e3, thr_mV=thr * 1e3, margin_db=20 * math.log10(thr / v) if v > 0 else 99,
                         settle_ns=5 * tau * 1e9, why=why,
                         worst_aggressor=max(d["aggressors"], key=d["aggressors"].get) if d["aggressors"] else ""))
    rows.sort(key=lambda r: -r["v_mV"] / r["thr_mV"])
    print("%-14s %-22s %7s %7s %6s %8s %8s %9s %8s" % ("net", "class", "len", "coupled", "s_min", "C_m fF",
                                                        "V mV", "limit mV", "margin"))
    for r in rows[:30]:
        print("%-14s %-22s %7.2f %7.2f %6.2f %8.2f %8.3f %9.2f %7.1f dB" % (
            r["net"], r["cls"], r["len_mm"], r["coupled_mm"], r["s_min"], r["cm_fF"], r["v_mV"], r["thr_mV"],
            r["margin_db"]))
    bs_rows = [r for r in rows if r["bs_mm"] > 0]
    bs_sum = dict(nets=len(bs_rows), coupled_mm=sum(r["bs_mm"] for r in bs_rows),
                  worst=min(bs_rows, key=lambda r: r["margin_db"])["net"] if bs_rows else "",
                  worst_margin_db=min([r["margin_db"] for r in bs_rows] or [99]),
                  victims=[dict(net=r["net"], bs_mm=r["bs_mm"], margin_db=r["margin_db"]) for r in bs_rows])
    print("broadside (L3/L4 phase over a victim on the other layer): %d nets, %.2f mm coupled, worst %s at %.1f dB margin" % (
        bs_sum["nets"], bs_sum["coupled_mm"], bs_sum["worst"] or "-", bs_sum["worst_margin_db"]), flush=True)
    # ---- return-path vias ------------------------------------------------------------------------------------------
    gv = np.array([(x, y) for (x, y, n, d_, p) in PL.power_vias(b) if n == "GND"])
    pcb = PL.pcbnew
    trk_by_net = {}
    for t in b.GetTracks():
        if not isinstance(t, pcb.PCB_VIA):
            trk_by_net.setdefault(t.GetNetname(), []).append(t)
    pads_by_net = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            pads_by_net.setdefault(p.GetNetname(), []).append(p)

    def plane_change(v):
        # L1/L3 tracks return on L2, L4/L6 tracks on L5: a via joining both sides moves its return current
        # between the GND planes and needs a GND via close by
        pos, rv = v.GetPosition(), v.GetWidth(pcb.F_Cu) // 2 + 1000
        lays = {t.GetLayer() for t in trk_by_net.get(v.GetNetname(), ())
                if (t.GetStart() - pos).EuclideanNorm() <= rv or (t.GetEnd() - pos).EuclideanNorm() <= rv}
        lays |= {l for p in pads_by_net.get(v.GetNetname(), ()) if p.HitTest(pos)
                 for l in (pcb.F_Cu, pcb.B_Cu) if p.IsOnLayer(l)}
        return len({"L2" if l in (pcb.F_Cu, pcb.In2_Cu) else "L5" for l in lays}) != 1

    ret = []
    for t in b.GetTracks():
        if isinstance(t, pcb.PCB_VIA) and t.GetNetname() not in SKIP and t.GetNetname():
            x, y = PL.xy(t.GetPosition())
            dmin = float(np.min(np.hypot(gv[:, 0] - x, gv[:, 1] - y))) if len(gv) else 99
            ret.append(dict(net=t.GetNetname(), x=x, y=y, d_gnd_via=dmin, plane_change=plane_change(t)))
    far = [r for r in ret if r["d_gnd_via"] > 1.0]
    pcv = [r for r in ret if r["plane_change"]]
    far_pc = [r for r in pcv if r["d_gnd_via"] > 1.0]
    print("signal vias: %d, with a GND via within 1.0 mm: %d, worst %.2f mm (%s); plane-changing vias: %d, "
          "with a GND via within 1.0 mm: %d, worst %.2f mm" % (
              len(ret), len(ret) - len(far), max([r["d_gnd_via"] for r in ret] or [0]),
              max(ret, key=lambda r: r["d_gnd_via"])["net"] if ret else "-", len(pcv), len(pcv) - len(far_pc),
              max([r["d_gnd_via"] for r in pcv] or [0])))
    # ---- figure ----------------------------------------------------------------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(15, 5.5), gridspec_kw={"width_ratios": [1.6, 1]})
    top = rows[:22]
    ys = np.arange(len(top))
    ax[0].barh(ys, [r["margin_db"] for r in top], color=[S.PAL[0] if r["margin_db"] > 6 else S.CRIT for r in top])
    ax[0].set_yticks(ys)
    ax[0].set_yticklabels(["%s (%s)" % (r["net"], r["cls"]) for r in top], fontsize=7.5)
    ax[0].invert_yaxis()
    ax[0].axvline(6, color=S.CRIT, ls="--", lw=1)
    ax[0].text(6.3, len(top) - 0.5, "6 dB margin", fontsize=7.5, color=S.INK2)
    ax[0].set_xlabel("margin to the class limit (dB), worst first")
    ax[0].set_title("Phase-edge crosstalk (55 V/us, 1.5 A/us) into each net, from the routed copper")
    for layer, wa, col in ((0, 0.5, S.PAL[1]), (2, 0.5, S.PAL[2]), (0, 0.2, S.PAL[3])):
        ax[1].semilogy(seps, 1e12 * tab[layer, wa][0], "o-", color=col,
                       label="c_m %s, phase %.1f mm" % ("L1/L6" if layer == 0 else "L3", wa))
    ax[1].semilogy(BS_DX, 1e12 * bst[2][0.1][0], "s--", color=S.PAL[4 % len(S.PAL)],
                   label="c_m L3 phase 0.1 mm over L4 (x = centre offset)")
    ax[1].set(xlabel="edge spacing (mm)", ylabel="mutual capacitance (pF/m)",
              title="Field-solver coupling vs spacing (victim 0.1 mm)")
    ax[1].legend(fontsize=7.5)
    PL.save(fig, "p3_xtalk.png")
    PL.write_results("p3_xtalk", dict(rows=rows, return_vias=dict(total=len(ret), far=far, plane_changing=len(pcv),
                                                                far_plane_changing=far_pc), broadside=bs_sum,
                                      bs_table=dict(dx=BS_DX, cm={"%s_%s" % (al, k): v[0] for al, t_ in bst.items() for k, v in t_.items()},
                                                    lm={"%s_%s" % (al, k): v[1] for al, t_ in bst.items() for k, v in t_.items()}),
                                      tables={("%s_%.1f" % (k[0], k[1])): dict(seps=seps, cm=v[0], lm=v[1])
                                              for k, v in tab.items()}))


if __name__ == "__main__":
    main()
