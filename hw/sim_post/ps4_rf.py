"""Post-layout study P4: the 2.4 GHz feed exactly as routed: line impedance, GND gap, reference plane, via fence,
and the matching/harmonic network cascaded with the real line lengths and pad parasitics.

Method
------
* RF tracks (RF_IO, RF_N1..3, ANT) are read from the board. Along the line, every 0.05 mm, the edge-to-edge gap to the
  L6 GND pour is measured on a 0.05 mm distance transform, and the L5 reference plane under the line is checked.
  Z0 per segment comes from fd2d: CPWG on L6 over L5 under 10 um LPI mask (gap filled), with gap = measured gap
  (microstrip when the gap exceeds 0.6 mm).
* GND vias within 0.8 mm of the line edge form the fence. The largest along-line spacing per side is compared with
  lambda/20 at 2.45 GHz and lambda/10 at the 3rd harmonic.
* Network: pads are projected onto the line axis and ordered. Each net is a transmission line between its pads.
  Series parts bridge the nets; shunt parts hang off their pad. Every 0402 pad adds its plate + fringe capacitance
  to L5. Component models (Q, SRF, ESL, ESR) are those of hw/sim study 7, so 'ideal-layout' reproduces study 7.
  Ports are 50 ohm, as in study 7.

Outputs: out/p4_rf.png, out/p4_rf.json
"""
import cmath
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postlib as PL  # noqa: E402
import fd2d  # noqa: E402
import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402

S = PL.S
plt = PL.plt
RF_NETS = ["RF_IO", "RF_N1", "RF_N2", "RF_N3", "ANT"]
F0 = 2.45e9
TAN_D = 0.02
H5 = PL.T_DIEL[4]


def z0_cpwg(W, g, tm=10e-6, erm=3.8, h=None, er=None):
    h = H5 if h is None else h
    er = PL.ER_DIEL[4] if er is None else er
    t = PL.T_CU[5]
    box = 1.5e-3
    g = min(g, 1.0e-3)
    cond = [[(-W / 2, W / 2, h, h + t)]]
    gnd = [(-box, box, -box, 0.0), (W / 2 + g, box, h, h + t), (-box, -W / 2 - g, h, h + t)]
    sec = fd2d.Section(cond, gnd, [(0.0, h, er)], [(-box, box, h, h + t + tm, erm)], box=(-box, box, -box, box))
    return fd2d.z0_single(sec)


def comp_models():
    import importlib.util
    spec = importlib.util.spec_from_file_location("sim07", os.path.join(PL.SIM, "sim_07_rf.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    S.style()
    m07 = comp_models()
    b = PL.load()
    fine = PL.fine_rasters(b)
    fp = PL.footprints(b)
    # line axis: end of the RFIO stub (RF_STUB out of the pin row) -> antenna pad, as in nanodrone_route.route_rf
    sys.path.insert(0, os.path.join(PL.HW, "design"))
    import nanodrone_pcb as NP
    nx4, ny4, px4, py4 = NP.rfio_normal(b)
    x0, y0 = px4 + nx4 * NP.RF_STUB, py4 + ny4 * NP.RF_STUB
    antp = [p for p in fp["J2"].Pads() if p.GetNetname() == "ANT"][0]
    x1, y1 = PL.xy(antp.GetPosition())
    Lax = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / Lax, (y1 - y0) / Lax
    vx, vy = -uy, ux

    def proj(x, y):
        return (x - x0) * ux + (y - y0) * uy, (x - x0) * vx + (y - y0) * vy
    # ---- tracks, gap, reference plane --------------------------------------------------------------------------
    gnd6 = fine["GND", "B.Cu"]
    gnd5 = fine["GND", "In4.Cu"]
    dist = ndimage.distance_transform_edt(~gnd6) * PL.X.RES        # mm to nearest L6 GND copper
    segs = []
    for (net, l, ax, ay, bx, by, w) in PL.tracks(b):
        if net not in RF_NETS:
            continue
        L = math.hypot(bx - ax, by - ay)
        n = max(2, int(L / 0.05))
        gaps, refok = [], []
        for k in range(n):
            px, py = ax + (bx - ax) * (k + 0.5) / n, ay + (by - ay) * (k + 0.5) / n
            i, j = PL.cell_of(px, py, PL.X.RES)
            gaps.append(dist[i, j] - w / 2)
            # reference plane: L5 GND solid in a +/- 3W band under the trace
            rr = int(math.ceil(3 * w / PL.X.RES))
            refok.append(gnd5[i - rr:i + rr + 1, j - rr:j + rr + 1].mean())
        segs.append(dict(net=net, layer=PL.CU_LAYERS[l], len=L, w=w, gap_min=float(min(gaps)),
                         gap_med=float(np.median(gaps)), ref_fill_min=float(min(refok))))
    total = sum(s["len"] for s in segs)
    print("RF tracks: %d segments, %.2f mm, layers %s" % (len(segs), total, sorted({s["layer"] for s in segs})))
    # inside the matching network the pad reliefs merge: there the line references L4 (0.26 mm down)
    h_rel = H5 + PL.T_DIEL[3]
    er_rel = (PL.ER_DIEL[4] * H5 + PL.ER_DIEL[3] * PL.T_DIEL[3]) / h_rel
    zcache = {}
    for s in segs:
        rel = s["ref_fill_min"] < 0.5
        key = (round(s["w"], 3), round(min(s["gap_med"], 0.6), 3), rel)
        if key not in zcache:
            g_ = (key[1] if key[1] < 0.6 else 1.0) * 1e-3
            zcache[key] = z0_cpwg(key[0] * 1e-3, g_, h=h_rel, er=er_rel) if rel else z0_cpwg(key[0] * 1e-3, g_)
        s["z0"], s["eeff"] = zcache[key]
        s["ref"] = "L4 (relief)" if rel else "L5"
        print("  %-6s %s L %.2f mm  W %.3f  gap min/med %.3f/%.3f mm  ref %-11s Z0 %.1f ohm" % (
            s["net"], s["layer"], s["len"], s["w"], s["gap_min"], s["gap_med"], s["ref"], s["z0"]))
    zl5 = [s for s in segs if s["ref"] == "L5"] or segs
    zline = float(np.average([s["z0"] for s in zl5], weights=[s["len"] for s in zl5]))
    w_med = float(np.median([s["w"] for s in segs])) * 1e-3
    g_med = float(np.median([s["gap_med"] for s in segs])) * 1e-3
    z_relief, eeff_relief = z0_cpwg(w_med, max(g_med, 0.15e-3), h=h_rel, er=er_rel)
    print("line Z0: %.1f ohm over L5, %.1f ohm over the pad reliefs (L4 reference, gap %.2f mm)" % (
        zline, z_relief, g_med * 1e3))
    # stackup tolerance: JLC lists 2116 prepreg at er 4.16-4.25 (we use 4.45) and 0.8 mm boards at +/-0.1 mm
    wq = float(np.median([s["w"] for s in segs])) * 1e-3
    sens = {}
    for lab_, hh, ee_ in (("nominal (h 0.0994, er 4.45)", H5, 4.45), ("er 4.16", H5, 4.16), ("er 4.25", H5, 4.25),
                          ("h -10 %", 0.9 * H5, 4.45), ("h +10 %", 1.1 * H5, 4.45), ("h +10 %, er 4.16", 1.1 * H5, 4.16),
                          ("mask 20 um", H5, 4.45)):
        sens[lab_] = z0_cpwg(wq, 0.15e-3, h=hh, er=ee_, tm=20e-6 if "mask" in lab_ else 10e-6)[0]
    print("CPWG Z0 tolerance (W %.3f / g 0.150 mm): %s" % (wq * 1e3, ", ".join("%s %.1f" % kv for kv in sens.items())))
    eeff = float(np.average([s["eeff"] for s in segs], weights=[s["len"] for s in segs]))
    # ---- via fence ------------------------------------------------------------------------------------------------
    gv = [(x, y) for (x, y, n, d, p) in PL.power_vias(b) if n == "GND"]
    sides = {+1: [], -1: []}
    for (x, y) in gv:
        s, d = proj(x, y)
        if -0.3 < s < Lax + 0.3 and abs(d) < 0.8 + 0.06:
            sides[1 if d > 0 else -1].append(s)
    lam = 1e3 * fd2d.C_LIGHT / F0 / math.sqrt(eeff)
    fence = {}
    for sd, lst in sides.items():
        pts = sorted([0.0] + lst + [Lax])
        gapmax = max(np.diff(pts)) if len(pts) > 1 else Lax
        fence[sd] = dict(n=len(lst), max_spacing=float(gapmax))
    print("fence: %d / %d vias per side, max spacing %.2f / %.2f mm (lambda/20 @2.45G = %.2f, lambda/10 @7.35G = %.2f)"
          % (fence[1]["n"], fence[-1]["n"], fence[1]["max_spacing"], fence[-1]["max_spacing"], lam / 20, lam / 30))
    # ---- topology along the line ----------------------------------------------------------------------------------
    pads = []
    for ref, f in fp.items():
        ps = list(f.Pads())
        nets = [p.GetNetname() for p in ps]
        if ref in ("U4", "J2") or not (set(nets) & set(RF_NETS)):
            continue
        val = f.GetValue()
        for p in ps:
            if p.GetNetname() in RF_NETS:
                s, d = proj(*PL.xy(p.GetPosition()))
                # reference under the pad: L5 if GND copper is there, else the L4 plane (pad relief cut-out)
                px_, py_ = PL.xy(p.GetPosition())
                i_, j_ = PL.cell_of(px_, py_, PL.X.RES)
                l5 = fine["GND", "In4.Cu"][i_ - 3:i_ + 4, j_ - 3:j_ + 4].mean()
                h_ref = H5 if l5 > 0.5 else H5 + PL.T_DIEL[3]
                pads.append(dict(ref=ref, net=p.GetNetname(), s=s, d=d, val=val, nets=tuple(nets),
                                 area=(p.GetSize().x / 1e6) * (p.GetSize().y / 1e6), h_ref=h_ref,
                                 dnp=bool(f.IsDNP()) or "DNP" in val))
    pads.sort(key=lambda q: q["s"])
    # element list along the line: ('line', z0, eeff, len_mm) / ('shunt', kind, value) / ('series', ...)
    eps0 = fd2d.EPS0
    er5 = PL.ER_DIEL[4]
    print("pads on the line:")
    for q in pads:
        print("  s %6.2f d %+5.2f  %-4s %-4s %s%s  ref %s" % (q["s"], q["d"], q["ref"], q["net"], q["val"],
                                                             " (DNP)" if q["dnp"] else "",
                                                             "L5" if q["h_ref"] == H5 else "L4 (relief)"))
    res_pad_relief = sum(1 for q in pads if q["h_ref"] != H5)

    def over_l5(s):
        x_, y_ = x0 + ux * s, y0 + uy * s
        i_, j_ = PL.cell_of(x_, y_, PL.X.RES)
        return gnd5[i_ - 2:i_ + 3, j_ - 2:j_ + 3].mean() > 0.5

    def line(s_a, s_b):
        L_ = max(s_b - s_a, 0.0)
        if over_l5((s_a + s_b) / 2):
            return ("line", zline, eeff, L_)
        return ("line", z_relief, eeff_relief, L_)

    def parse_val(v):
        v = v.split()[0].replace("pF", "e-12").replace("nH", "e-9").replace("R", "")
        return float(v) if v else 0.0

    # ---- topology by node: RF_IO -> ... -> ANT, series groups between nodes, shunts on nodes ----------------
    live = [q for q in pads if not q["dnp"]]
    series_refs = sorted({q["ref"] for q in live if "GND" not in q["nets"] and len(set(q["nets"]) & set(RF_NETS)) == 2})
    shunt_refs = sorted({q["ref"] for q in live if "GND" in q["nets"]})
    chain = ["RF_IO"]
    groups = []                                                     # [(from_node, to_node, [refs])]
    while True:
        nxt = [r for r in series_refs if chain[-1] in [q["net"] for q in live if q["ref"] == r]
               and not any(r in g[2] for g in groups)]
        if not nxt:
            break
        to = [q["net"] for q in live if q["ref"] == nxt[0] and q["net"] != chain[-1]][0]
        grp = [r for r in nxt if {q["net"] for q in live if q["ref"] == r} == {chain[-1], to}]
        groups.append((chain[-1], to, grp))
        chain.append(to)
    val = {r: parse_val([q["val"] for q in live if q["ref"] == r][0]) for r in series_refs + shunt_refs}
    print("chain:", " -> ".join(chain), "| series:", [(a, b, g) for a, b, g in groups], "| shunts:", shunt_refs)

    def pad_s(ref, net):
        return [q["s"] for q in live if q["ref"] == ref and q["net"] == net][0]

    def build(values, layout=True):
        """element list for given component values (dict ref -> value)"""
        elems = [("line", zline, eeff, NP.RF_STUB)] if layout else []
        for k, node in enumerate(chain):
            s_in = 0.0 if k == 0 else max(pad_s(r, node) for r in groups[k - 1][2])
            s_out = Lax if k == len(chain) - 1 else min(pad_s(r, node) for r in groups[k][2])
            ev = []
            for q in live:
                if q["net"] != node:
                    continue
                if layout:
                    ev.append((q["s"], ("Csh_pad", 1.3 * eps0 * er5 * q["area"] * 1e-6 / q["h_ref"])))
                if q["ref"] in shunt_refs:
                    ev.append((q["s"], ("Csh", values[q["ref"]])))
            ev.sort(key=lambda e: e[0])
            cur = s_in
            for s, el in ev:
                if layout:
                    elems.append(line(cur, min(s, s_out)))
                    cur = max(cur, min(s, s_out))
                elems.append(el)
            if layout:
                elems.append(line(cur, s_out))
            if k < len(groups):
                a, b, g = groups[k]
                Ls = [r for r in g if r.startswith("L")]
                Cs = [r for r in g if r.startswith("C")]
                if Ls and Cs:
                    elems.append(("LCpar", values[Ls[0]], values[Cs[0]]))
                elif Ls:
                    elems.append(("Lse", values[Ls[0]]))
                elif Cs:
                    elems.append(("Cse", values[Cs[0]]))
                else:
                    elems.append(("Rse0", 0.0))
        return elems

    def cascade(f, elems):
        w = 2 * np.pi * f
        pr = dict(q_l=30.0, srf_l=8e9, esr_c=0.2, esl_c=0.35e-9)
        A = np.ones_like(w, dtype=complex)
        B = np.zeros_like(w, dtype=complex)
        C = np.zeros_like(w, dtype=complex)
        D = np.ones_like(w, dtype=complex)

        def mul(a2, b2, c2, d2):
            nonlocal A, B, C, D
            A, B, C, D = A * a2 + B * c2, A * b2 + B * d2, C * a2 + D * c2, C * b2 + D * d2
        one, zero = np.ones_like(w, dtype=complex), np.zeros_like(w, dtype=complex)
        for e in elems:
            if e[0] == "line":
                z0, ee, L = e[1], e[2], e[3] * 1e-3
                if L <= 0:
                    continue
                beta = w * math.sqrt(ee) / fd2d.C_LIGHT
                alpha = beta * TAN_D / 2 * 0.8 + 0.02 * np.sqrt(f / 1e9)
                g = alpha + 1j * beta
                mul(np.cosh(g * L), z0 * np.sinh(g * L), np.sinh(g * L) / z0, np.cosh(g * L))
            elif e[0] in ("Csh", "Csh_pad"):
                esl = pr["esl_c"] if e[0] == "Csh" else 0.0
                esr = pr["esr_c"] if e[0] == "Csh" else 0.0
                mul(one, zero, 1 / m07.z_cap(e[1], w, esr, esl) if e[1] > 0 else zero, one)
            elif e[0] == "Lse":
                mul(one, m07.z_ind(e[1], w, pr["q_l"], pr["srf_l"]), zero, one)
            elif e[0] == "LCpar":
                zl = m07.z_ind(e[1], w, pr["q_l"], pr["srf_l"])
                zc = m07.z_cap(e[2], w, pr["esr_c"], pr["esl_c"])
                mul(one, zl * zc / (zl + zc), zero, one)
            elif e[0] == "Cse":
                mul(one, m07.z_cap(e[1], w, 0.1, 0.4e-9), zero, one)
            elif e[0] == "Rse0":
                mul(one, 0.02 + 1j * w * 0.4e-9, zero, one)
        z0 = 50.0
        den = A + B / z0 + C * z0 + D
        return 2 / den, (A + B / z0 - C * z0 - D) / den

    def metrics(s21, s11, f):
        band = (f >= 2.40e9) & (f <= 2.4835e9)
        return dict(il_max=float((-20 * np.log10(np.abs(s21[band]))).max()),
                    rl_min=float((-20 * np.log10(np.abs(s11[band]))).min()),
                    att_2f0=float(-20 * np.log10(np.abs(s21[np.argmin(abs(f - 4.9e9))]))),
                    att_3f0=float(-20 * np.log10(np.abs(s21[np.argmin(abs(f - 7.35e9))]))))

    f = np.linspace(0.5e9, 8e9, 1501)
    res = {"segments": segs, "z_line": zline, "z_relief": z_relief, "eeff": eeff, "fence": fence,
           "lambda_mm": lam, "line_len": Lax,
           "z0_tolerance": sens, "chain": chain, "values_bom": {r: val[r] for r in val},
           "pads_with_L5_relief": res_pad_relief, "rf_pads": len(pads)}
    out = {}
    ref21, ref11 = cascade(f, build(val, layout=False))
    for lab, layout in (("ideal layout (study 7)", False), ("as routed", True)):
        s21, s11 = cascade(f, build(val, layout))
        out[lab] = dict(metrics(s21, s11, f), s21=20 * np.log10(np.abs(s21)), s11=20 * np.log10(np.abs(s11)))
        print("%-24s IL %.2f dB max, RL %.1f dB min (2.400-2.4835 GHz), 2f0 %.1f dB, 3f0 %.1f dB" % (
            lab, out[lab]["il_max"], out[lab]["rl_min"], out[lab]["att_2f0"], out[lab]["att_3f0"]))
    # ---- post-layout re-tune: values that make the as-routed network reproduce the reference response -----------
    from scipy.optimize import minimize
    tune = [r for r in series_refs + shunt_refs if not (r.startswith("C") and val[r] > 20e-12)]   # keep DC block
    # broadband target (1.6-3.4 GHz, in-band weighted x4): the as-routed 2-port should reproduce the reference 2-port
    fb = np.linspace(1.6e9, 3.4e9, 73)
    wb = np.where((fb >= 2.35e9) & (fb <= 2.53e9), 4.0, 1.0)
    r21, r11 = cascade(fb, build(val, layout=False))
    f2 = np.array([4.8e9, 4.9e9, 5.0e9])
    a2_ref = -20 * np.log10(np.abs(cascade(f2, build(val, layout=False))[0])).min()
    # start: subtract each node's pad capacitance from its shunt capacitor
    padc = {}
    for q in live:
        padc[q["net"]] = padc.get(q["net"], 0.0) + 1.3 * eps0 * er5 * q["area"] * 1e-6 / q["h_ref"]
    start = {}
    for r in tune:
        if r in shunt_refs:
            node = [q["net"] for q in live if q["ref"] == r and q["net"] != "GND"][0]
            start[r] = max(val[r] - padc[node], 0.15e-12)
        else:
            start[r] = val[r]

    def vals(x):
        v = dict(val)
        for r, lx in zip(tune, x):
            v[r] = start[r] * math.exp(lx)
        return v

    def cost(x):
        v = vals(x)
        s21, s11 = cascade(fb, build(v, True))
        c = float(np.sum(wb * (np.abs(s21 - r21) ** 2 + np.abs(s11 - r11) ** 2)) / np.sum(wb))
        a2 = -20 * np.log10(np.abs(cascade(f2, build(v, True))[0])).min()
        return c + 0.002 * max(0.0, a2_ref - 3 - a2) ** 2
    best = None
    for xs0 in (np.zeros(len(tune)), np.full(len(tune), -0.1), np.full(len(tune), 0.1)):
        o = minimize(cost, xs0, method="L-BFGS-B", bounds=[(-1.0, 1.0)] * len(tune))
        if best is None or o.fun < best.fun:
            best = o
    opt = best
    cont = {r: vals(opt.x)[r] for r in tune}
    c_start = cost(np.zeros(len(tune)))
    c_bom = float(np.sum(wb * (np.abs(cascade(fb, build(val, True))[0] - r21) ** 2 +
                               np.abs(cascade(fb, build(val, True))[1] - r11) ** 2)) / np.sum(wb))
    print("re-tune fit (weighted mean |dS|^2 to the reference): BOM values %.4f, pad-compensated start %.4f, "
          "optimum %.4f" % (c_bom, c_start, opt.fun))
    C_STEPS = np.round(np.arange(0.1, 3.01, 0.1), 2) * 1e-12                       # GJM15 C0G, 0.1 pF steps
    L_STEPS = np.array([1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1,
                        5.6, 6.2, 6.8]) * 1e-9                                     # LQW15AN series values
    snap = {}
    for r, v in cont.items():
        steps = L_STEPS if r.startswith("L") else C_STEPS
        snap[r] = float(steps[np.argmin(np.abs(np.log(steps / v)))])

    def dcost(sn):
        v = dict(val)
        v.update(sn)
        s21, s11 = cascade(fb, build(v, True))
        c = float(np.sum(wb * (np.abs(s21 - r21) ** 2 + np.abs(s11 - r11) ** 2)) / np.sum(wb))
        a2 = -20 * np.log10(np.abs(cascade(f2, build(v, True))[0])).min()
        return c + 0.002 * max(0.0, a2_ref - 3 - a2) ** 2
    # discrete coordinate descent over neighbouring standard values
    cur = dcost(snap)
    improved = True
    while improved:
        improved = False
        for r in tune:
            steps = L_STEPS if r.startswith("L") else C_STEPS
            i0 = int(np.argmin(np.abs(steps - snap[r])))
            for di in (-2, -1, 1, 2):
                if 0 <= i0 + di < len(steps):
                    trial = dict(snap)
                    trial[r] = float(steps[i0 + di])
                    c = dcost(trial)
                    if c < cur - 1e-9:
                        snap, cur, improved = trial, c, True
    print("discrete re-tune cost %.4f" % cur)
    vs = dict(val)
    vs.update(snap)
    s21, s11 = cascade(f, build(vs, True))
    out["as routed, re-tuned values"] = dict(metrics(s21, s11, f), s21=20 * np.log10(np.abs(s21)),
                                             s11=20 * np.log10(np.abs(s11)))
    dev = float(np.sqrt(np.mean(np.abs(cascade(fb, build(vs, True))[1] - r11) ** 2)))
    res["retune"] = {r: dict(bom=val[r], continuous=cont[r], fitted=snap[r]) for r in tune}
    res["retune_rms_s11_error"] = dev
    print("post-layout re-tune (BOM -> fitted standard value): " + ", ".join(
        "%s %s -> %s" % (r, ("%.1f nH" % (val[r] * 1e9)) if r.startswith("L") else ("%.1f pF" % (val[r] * 1e12)),
                         ("%.1f nH" % (snap[r] * 1e9)) if r.startswith("L") else ("%.1f pF" % (snap[r] * 1e12)))
        for r in tune))
    print("%-24s IL %.2f dB max, RL %.1f dB min, 2f0 %.1f dB, 3f0 %.1f dB; rms |dS11| to reference %.3f" % (
        "as routed, re-tuned", out["as routed, re-tuned values"]["il_max"], out["as routed, re-tuned values"]["rl_min"],
        out["as routed, re-tuned values"]["att_2f0"], out["as routed, re-tuned values"]["att_3f0"], dev))
    res["networks"] = {k: {kk: vv for kk, vv in v.items() if kk not in ("s21", "s11")} for k, v in out.items()}
    # ---- Monte Carlo: part tolerances (GJM15 +/-0.05 pF, LQW15AN +/-0.1 nH), pad C +/-20 %, line Z0 +/-5 % ----------
    rng = np.random.default_rng(7)
    fm = np.linspace(2.40e9, 2.4835e9, 9)
    mc_il, mc_rl = [], []
    for _ in range(400):
        v = dict(val)
        for r in tune:
            v[r] = max(val[r] + rng.uniform(-1, 1) * (0.1e-9 if r.startswith("L") else 0.05e-12), 0.05e-12)
        kpad = 1 + rng.uniform(-0.2, 0.2)
        kz = 1 + rng.uniform(-0.05, 0.05)
        el = [(e[0], e[1] * kz, e[2], e[3]) if e[0] == "line" else ((e[0], e[1] * kpad) if e[0] == "Csh_pad" else e)
              for e in build(v, True)]
        s21, s11 = cascade(fm, el)
        mc_il.append(float((-20 * np.log10(np.abs(s21))).max()))
        mc_rl.append(float((-20 * np.log10(np.abs(s11))).min()))
    q = lambda a, p_: float(np.percentile(a, p_))
    res["monte_carlo"] = dict(n=400, il_p5=q(mc_il, 5), il_p50=q(mc_il, 50), il_p95=q(mc_il, 95),
                              rl_p5=q(mc_rl, 5), rl_p50=q(mc_rl, 50), rl_p95=q(mc_rl, 95))
    print("Monte Carlo (400 boards, BOM values): IL p50 %.2f / p95 %.2f dB, RL p50 %.1f / p5 %.1f dB" % (
        q(mc_il, 50), q(mc_il, 95), q(mc_rl, 50), q(mc_rl, 5)))
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.8))
    for k, (lab, v) in enumerate(out.items()):
        ax[0].plot(f / 1e9, v["s21"], color=S.PAL[k], label="S21 " + lab)
        ax[1].plot(f / 1e9, v["s11"], color=S.PAL[k], label="S11 " + lab)
    for a in ax:
        a.axvspan(2.4, 2.4835, color=S.PAL[2], alpha=0.12)
        a.set_xlabel("GHz")
        a.legend(fontsize=8)
    ax[0].set(ylabel="dB", ylim=(-60, 1), title="Matching / harmonic network, 50-ohm ports")
    ax[1].set(ylabel="dB", ylim=(-40, 0), title="Input match (line Z0 %.1f ohm, %.1f mm, eeff %.2f)" % (zline, Lax, eeff))
    PL.save(fig, "p4_rf.png")
    PL.write_results("p4_rf", res)


if __name__ == "__main__":
    main()
