"""Post-layout study P6: net inventory, timing and the routed 3V0 rail.

* Per-net routed length per layer, vias, propagation delay (eeff from the fd2d cross-sections), and the ratio of the
  delay to the edge's critical length (t_r / 2 = the point where a line stops being lumped). STM32 GPIO edges at
  'very high' speed: 2 ns [A].
* Crystal nets RF_XTA/RF_XTB: stray capacitance to GND (fd2d per-length C x length + pad plate C) and the resulting
  crystal pulling (C1 3 fF, C0 1.5 pF, CL 10 pF [A]: S = C1 / (2 (C0+CL)^2)).
* 3V0 distribution: a DC resistor network built from the routed +3V0 tracks and vias (pads joined by copper contact).
  The LDO output feeds the measured loads, and the drop to every 3V0 pin is reported.

Outputs: out/p6_nets.json, out/p6_nets.png
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postlib as PL  # noqa: E402
import fd2d  # noqa: E402
import numpy as np  # noqa: E402
import scipy.sparse as sp  # noqa: E402
import scipy.sparse.linalg as spla  # noqa: E402

S = PL.S
plt = PL.plt
T_RISE = 2e-9
LOAD_3V0 = {"U2": 0.035, "U21": 0.007, "U22": 0.007, "U23": 0.007, "U24": 0.007, "U11": 0.002, "U12": 0.002,
            "U13": 0.002, "U14": 0.002}            # A drawn from +3V0 pins [A] (radio/IMU sit behind filters)


def eeff_layers():
    """eeff and C per metre of a 0.1 mm line on L1/L6 (masked microstrip) and L3 (stripline)."""
    h, t = PL.T_DIEL[0], PL.T_CU[0]
    w = 0.1e-3
    box = (-2e-3, 2e-3, -0.1e-3, 2e-3)
    ms = fd2d.Section([[(-w / 2, w / 2, h, h + t)]], [(-2e-3, 2e-3, -0.1e-3, 0.0)], [(0.0, h, PL.ER_DIEL[0])],
                      [(-2e-3, 2e-3, h, h + t + 10e-6, 3.8)], box=box)
    L, C, C0 = ms.lc()
    out = {0: (C[0, 0] / C0[0, 0], C[0, 0]), 5: (C[0, 0] / C0[0, 0], C[0, 0])}
    t3 = PL.T_CU[2]
    hb = PL.T_DIEL[2] - t3
    ht = PL.T_DIEL[1]
    st = fd2d.Section([[(-w / 2, w / 2, hb, hb + t3)]], [(-2e-3, 2e-3, -0.05e-3, 0.0),
                                                         (-2e-3, 2e-3, hb + t3 + ht, hb + t3 + ht + 0.05e-3)],
                      [(0.0, hb + t3, PL.ER_DIEL[2]), (hb + t3, hb + t3 + ht, PL.ER_DIEL[1])],
                      box=(-2e-3, 2e-3, -0.05e-3, hb + t3 + ht + 0.05e-3))
    L, C, C0 = st.lc()
    out[2] = (C[0, 0] / C0[0, 0], C[0, 0])
    return out


def rail_drop(b, net="+3V0"):
    """DC solve of the routed rail: nodes = track ends + pads + vias, joined where copper touches."""
    pts, edges = [], []

    def nid(x, y, l):
        for i, (px, py, pl) in enumerate(pts):
            if pl == l and abs(px - x) < 0.02 and abs(py - y) < 0.02:
                return i
        pts.append((x, y, l))
        return len(pts) - 1
    rho = PL.RHO_CU * (1 + PL.ALPHA_CU * 35)
    segs = [t for t in PL.tracks(b) if t[0] == net]
    for (_n, l, x0, y0, x1, y1, w) in segs:
        a, c = nid(x0, y0, l), nid(x1, y1, l)
        L = math.hypot(x1 - x0, y1 - y0)
        edges.append((a, c, rho * L * 1e-3 / (w * 1e-3 * PL.T_CU[l]) + 1e-6))
    vias = [t for t in b.GetTracks() if isinstance(t, PL.pcbnew.PCB_VIA) and t.GetNetname() == net]
    for v in vias:
        x, y = PL.xy(v.GetPosition())
        r = v.GetDrillValue() / 2e9
        rv = rho * 0.78e-3 / (math.pi * ((r + PL.T_PLATE) ** 2 - r ** 2))
        # every routing layer, L4 included (it carries escape routing since rev B); barrel split by layer spacing
        ids = [nid(x, y, l) for l in (0, 2, 3, 5)]
        edges.append((ids[0], ids[1], rv * 0.33))
        edges.append((ids[1], ids[2], rv * 0.17))
        edges.append((ids[2], ids[3], rv * 0.50))
    # pads: a pad node per pad, tied to every track end / via that lies inside it on its layer
    padnode = {}
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() != net:
                continue
            l = 0 if p.IsOnLayer(PL.pcbnew.F_Cu) else 5
            x, y = PL.xy(p.GetPosition())
            bb = p.GetBoundingBox()
            hw, hh = bb.GetWidth() / 2e6 + 0.01, bb.GetHeight() / 2e6 + 0.01
            k = len(pts)
            pts.append((x, y, -1))
            padnode[(f.GetReference(), p.GetNumber())] = k
            for i, (px, py, pl) in enumerate(pts[:-1]):
                if pl == l and abs(px - x) <= hw and abs(py - y) <= hh:
                    edges.append((k, i, 1e-5))
    # track ends touching the middle of other segments (T-junctions)
    for i, (px, py, pl) in enumerate(pts):
        if pl < 0:
            continue
        for (a, c, R) in list(edges):
            if a == i or c == i or pts[a][2] != pl or pts[c][2] != pl:
                continue
            ax, ay, _ = pts[a]
            cx, cy, _ = pts[c]
            L = math.hypot(cx - ax, cy - ay)
            if L < 1e-6:
                continue
            t = ((px - ax) * (cx - ax) + (py - ay) * (cy - ay)) / L ** 2
            if 0.02 < t < 0.98 and math.hypot(px - (ax + t * (cx - ax)), py - (ay + t * (cy - ay))) < 0.02:
                edges.append((i, a, R * t + 1e-6))
                edges.append((i, c, R * (1 - t) + 1e-6))
    N = len(pts)
    r = np.array([e[0] for e in edges])
    c = np.array([e[1] for e in edges])
    g = 1.0 / np.array([e[2] for e in edges])
    G = sp.coo_matrix((np.concatenate([g, g, -g, -g]), (np.concatenate([r, c, r, c]), np.concatenate([r, c, c, r]))),
                      shape=(N, N)).tolil()
    src = [k for (ref, num), k in padnode.items() if ref == "U1"]
    q = np.zeros(N)
    load_nodes = {}
    for ref, amps in LOAD_3V0.items():
        ks = [k for (rf, num), k in padnode.items() if rf == ref]
        if not ks:
            continue
        for k in ks:
            q[k] -= amps / len(ks)
        load_nodes[ref] = ks
    from scipy.sparse.csgraph import connected_components
    _n, lab = connected_components(G.tocsr(), directed=False)
    src_comp = {lab[k] for k in src}
    island = np.array([lab[i] not in src_comp for i in range(N)])
    for k in src:
        G[k, k] += 1e6
    G = G.tocsc() + sp.diags(np.where(island, 1.0, 0.0))     # pin unconnected islands to 0 V
    q[island] = 0.0
    V = spla.spsolve(G, q)
    drops = {}
    for ref, ks in load_nodes.items():
        if all(island[k] for k in ks):
            drops[ref] = None                                   # this load's 3V0 pin is not tied to the LDO
        else:
            drops[ref] = float(-np.mean([V[k] for k in ks if not island[k]]))
    return drops, len(segs), len(vias)


def main():
    S.style()
    b = PL.load()
    ee = eeff_layers()
    print("eeff: L1/L6 %.2f (%.2f ps/mm, %.0f pF/m), L3 %.2f (%.2f ps/mm)" % (
        ee[0][0], 1e12 * math.sqrt(ee[0][0]) / fd2d.C_LIGHT * 1e-3, ee[0][1] * 1e12, ee[2][0],
        1e12 * math.sqrt(ee[2][0]) / fd2d.C_LIGHT * 1e-3))
    nets = {}
    for (net, l, x0, y0, x1, y1, w) in PL.tracks(b):
        d = nets.setdefault(net, dict(len={}, delay_ps=0.0, vias=0, c_pF=0.0))
        L = math.hypot(x1 - x0, y1 - y0)
        d["len"][PL.CU_LAYERS[l]] = d["len"].get(PL.CU_LAYERS[l], 0) + L
        lk = l if l in ee else 0
        d["delay_ps"] += L * 1e-3 * math.sqrt(ee[lk][0]) / fd2d.C_LIGHT * 1e12
        d["c_pF"] += ee[lk][1] * L * 1e-3 * 1e12
    for t in b.GetTracks():
        if isinstance(t, PL.pcbnew.PCB_VIA) and t.GetNetname() in nets:
            nets[t.GetNetname()]["vias"] += 1
    crit = T_RISE / 2 * 1e12
    rows = []
    for net, d in nets.items():
        tot = sum(d["len"].values())
        rows.append(dict(net=net, length=tot, layers=d["len"], vias=d["vias"], delay_ps=d["delay_ps"],
                         lumped_ratio=d["delay_ps"] / crit, c_pF=d["c_pF"]))
    rows.sort(key=lambda r: -r["length"])
    sig = [r for r in rows if not re.match(r"^(GND|VBAT)$", r["net"])]
    print("longest signal nets:")
    for r in sig[:12]:
        print("  %-14s %6.2f mm  vias %d  delay %5.0f ps  (%.2f of t_r/2)" % (r["net"], r["length"], r["vias"],
                                                                             r["delay_ps"], r["lumped_ratio"]))
    worst = max(sig, key=lambda r: r["lumped_ratio"])
    # crystal
    xt = {}
    for n in ("RF_XTA", "RF_XTB"):
        cpad = 0.0
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetNetname() == n:
                    a = (p.GetSize().x / 1e6) * (p.GetSize().y / 1e6) * 1e-6
                    cpad += 1.3 * fd2d.EPS0 * PL.ER_DIEL[0] * a / PL.T_DIEL[0] * 1e12
        ctr = nets.get(n, {}).get("c_pF", 0.0)
        xt[n] = dict(trace_pF=ctr, pads_pF=cpad, total_pF=ctr + cpad,
                     length=sum(nets.get(n, {}).get("len", {}).values()))
    c_series = 1 / (1 / xt["RF_XTA"]["total_pF"] + 1 / xt["RF_XTB"]["total_pF"]) if all(
        v["total_pF"] > 0 for v in xt.values()) else 0.0
    pull = 3e-15 / (2 * (1.5e-12 + 10e-12) ** 2) * 1e-12 * 1e6        # ppm per pF
    print("crystal: XTA %.2f pF, XTB %.2f pF stray -> +%.2f pF on CL -> %.1f ppm pull (S = %.1f ppm/pF)" % (
        xt["RF_XTA"]["total_pF"], xt["RF_XTB"]["total_pF"], c_series, c_series * pull, pull))
    drops, nseg, nvia = rail_drop(b)
    print("3V0 rail: %d segments, %d vias; DC drop LDO -> load: %s" % (
        nseg, nvia, " ".join("%s %s" % (k, "%.1f mV" % (1e3 * v) if v is not None else "NOT CONNECTED")
                             for k, v in sorted(drops.items()))))
    fig, ax = plt.subplots(figsize=(10, 4.6))
    top = sig[:20]
    ax.barh(np.arange(len(top)), [r["length"] for r in top], color=S.PAL[0])
    ax.set_yticks(np.arange(len(top)))
    ax.set_yticklabels([r["net"] for r in top], fontsize=7.5)
    ax.invert_yaxis()
    ax.set(xlabel="routed length (mm)", title="Longest signal nets (all are lumped: worst delay %.0f ps = %.2f x t_r/2)"
           % (worst["delay_ps"], worst["lumped_ratio"]))
    PL.save(fig, "p6_nets.png")
    PL.write_results("p6_nets", dict(nets=rows, eeff={str(k): v for k, v in ee.items()}, crystal=xt,
                                     crystal_cl_add_pF=c_series, crystal_pull_ppm=c_series * pull,
                                     rail_3v0_drop=drops, worst_lumped=worst))


if __name__ == "__main__":
    main()
