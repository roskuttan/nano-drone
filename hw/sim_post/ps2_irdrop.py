"""Post-layout study P2: DC IR drop, current density and via currents in the VBAT and GND networks.

Model
-----
* For each net, every copper layer is a resistive sheet of 0.1 mm cells. Sheet conductance = fill * t_Cu / rho
  (rho at 60 C) from the real artwork; neighbour links use the harmonic mean.
* Every via / plated hole of the net is a barrel resistor chain (18 um plating [A]) with one node per layer. It ties to
  a layer only where the net's copper surrounds the hole.
* The battery terminal pad of the net is the 0 V reference (Dirichlet). DRV8311 supply/ground pads sink/source each
  driver's DC bus current; the LDO pads take the 3V0 load.
* Operating points come from the hw/sim switching model (cached in out/p1_thermal.json when present):
  hover, full throttle, and a 4 A-per-driver instantaneous peak (punch-out, study 2).

Outputs: out/p2_irdrop_maps.png, out/p2_irdrop.json
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postlib as PL  # noqa: E402
import numpy as np  # noqa: E402
import scipy.sparse as sp  # noqa: E402
import scipy.sparse.linalg as spla  # noqa: E402

S = PL.S
plt = PL.plt
RES = 0.1
K = int(round(RES / PL.X.RES))
RHO = PL.RHO_CU * (1 + PL.ALPHA_CU * 40)     # copper at 60 C
DRIVERS = ["U11", "U12", "U13", "U14"]


class Network:
    def __init__(self, b, fine, net):
        t0 = time.time()
        self.net = net
        self.b = b
        self.fp = PL.footprints(b)
        inside = PL.block_mean(fine["outline"].astype(float), K) > 0.5
        self.fill = [PL.block_mean(fine[net, L].astype(float), K) * inside for L in PL.CU_LAYERS]
        n = self.fill[0].shape[0]
        self.n = n
        self.lin = []
        cnt = 0
        for l in range(6):
            m = self.fill[l] > 0.05
            lin = -np.ones((n, n), dtype=np.int64)
            lin[m] = np.arange(cnt, cnt + int(m.sum()))
            cnt += int(m.sum())
            self.lin.append(lin)
        rows, cols, gs = [], [], []
        for l in range(6):
            sig = np.where(self.fill[l] > 0.05, self.fill[l], 0) * PL.T_CU[l] / RHO      # S per square
            lin = self.lin[l]
            for di, dj in ((0, 1), (1, 0)):
                a, c = lin[:n - di, :n - dj], lin[di:, dj:]
                m = (a >= 0) & (c >= 0)
                sa, sc = sig[:n - di, :n - dj][m], sig[di:, dj:][m]
                rows.append(a[m])
                cols.append(c[m])
                gs.append(2.0 / (1.0 / sa + 1.0 / sc))
        nn = cnt
        self.vias = []
        holes = [(x, y, d, pad) for (x, y, nt, d, pad) in PL.power_vias(b) if nt == net]
        for (x, y, drill, pad) in holes:
            i, j = PL.cell_of(x, y, RES)
            r = drill / 2 * 1e-3
            ab = math.pi * ((r + PL.T_PLATE) ** 2 - r ** 2)
            vid = nn
            nn += 6
            for l in range(5):
                rows.append(np.array([vid + l]))
                cols.append(np.array([vid + l + 1]))
                gs.append(np.array([ab / (RHO * PL.T_DIEL[l])]))
            conn = []
            for l, L in enumerate(PL.CU_LAYERS):
                if PL.ring_fraction(fine[net, L], x, y, pad / 2 + 0.06, n=64) > 0.0 and self.lin[l][i, j] >= 0:
                    rows.append(np.array([vid + l]))
                    cols.append(np.array([self.lin[l][i, j]]))
                    gs.append(np.array([1e3]))
                    conn.append(l)
            self.vias.append(dict(x=x, y=y, drill=drill, vid=vid, layers=conn))
        self.nn = nn
        r, c, g = np.concatenate(rows), np.concatenate(cols), np.concatenate(gs)
        G = sp.coo_matrix((np.concatenate([g, g, -g, -g]),
                           (np.concatenate([r, c, r, c]), np.concatenate([r, c, c, r]))), shape=(nn, nn)).tocsc()
        # reference: battery terminal of this net (plated pad -> all its tied layers)
        jpad = [p for p in self.fp["J1"].Pads() if p.GetNetname() == net][0]
        bx, by = PL.xy(jpad.GetPosition())
        ref = []
        for v in self.vias:
            if abs(v["x"] - bx) < 1e-3 and abs(v["y"] - by) < 1e-3:
                ref = [v["vid"] + l for l in v["layers"]] or [v["vid"]]
        if not ref:
            raise RuntimeError("battery pad not found in %s network" % net)
        # copper islands not galvanically tied to the battery terminal carry no current: pin them to 0 V
        from scipy.sparse.csgraph import connected_components
        ncomp, lab = connected_components(G, directed=False)
        self.island = lab != lab[ref[0]]
        gref = np.zeros(nn)
        gref[ref] = 1e6
        gref[self.island] = 1.0
        self.n_island_nodes = int(self.island.sum())
        self.G = (G + sp.diags(gref)).tocsc()
        self.lu = spla.splu(self.G, permc_spec="MMD_AT_PLUS_A")
        self.links = (r, c, g)
        print("%s network: %d nodes (%d in islands), %d holes, ref %s, %.1f s" % (net, nn, self.n_island_nodes, len(self.vias), ref[:3],
                                                                time.time() - t0), flush=True)

    def pad_nodes(self, ref, pins=None):
        out = []
        for p in self.fp[ref].Pads():
            if p.GetNetname() != self.net or (pins and p.GetNumber() not in pins):
                continue
            if p.GetAttribute() != PL.pcbnew.PAD_ATTRIB_SMD:
                continue
            l = PL.fp_side(self.fp[ref])
            x, y = PL.xy(p.GetPosition())
            w, h = p.GetBoundingBox().GetWidth() / 1e6, p.GetBoundingBox().GetHeight() / 1e6
            for yy in np.arange(y - h / 2 + RES / 2, y + h / 2, RES):
                for xx in np.arange(x - w / 2 + RES / 2, x + w / 2, RES):
                    i, j = PL.cell_of(xx, yy, RES)
                    if self.lin[l][i, j] >= 0:
                        out.append(self.lin[l][i, j])
        return sorted(set(out))

    def solve(self, loads):
        """loads: {ref: amps drawn from this net at that part's pads}. Returns node potentials (V) relative to the
        battery terminal; for VBAT negative = drop, for GND the sign is flipped so positive = ground bounce."""
        q = np.zeros(self.nn)
        self.load_nodes = {}
        for ref, amps in loads.items():
            nodes = self.pad_nodes(ref)
            if not nodes:
                raise RuntimeError("no %s copper under %s pads" % (self.net, ref))
            q[nodes] -= amps / len(nodes)
            self.load_nodes[ref] = nodes
        if self.net == "GND":
            q = -q
        return self.lu.solve(q)

    def branch_currents(self, V):
        r, c, g = self.links
        return r, c, g * (V[r] - V[c])

    def layer_img(self, V, l):
        img = np.full((self.n, self.n), np.nan)
        m = self.lin[l] >= 0
        img[m] = V[self.lin[l][m]]
        return img

    def via_currents(self, V):
        out = []
        for v in self.vias:
            vid = v["vid"]
            # largest barrel-segment current
            ab = math.pi * ((v["drill"] / 2e3 + PL.T_PLATE) ** 2 - (v["drill"] / 2e3) ** 2)
            seg = [abs((V[vid + l] - V[vid + l + 1]) * ab / (RHO * PL.T_DIEL[l])) for l in range(5)]
            out.append(dict(x=v["x"], y=v["y"], drill=v["drill"], i_max=max(seg)))
        return out

    def dissipation(self, V):
        r, c, g = self.links
        return float(np.sum(g * (V[r] - V[c]) ** 2))

    def sheet_current_density(self, V, l):
        """|J_s| in A/mm (current per mm of width) per cell on layer l."""
        lin = self.lin[l]
        n = self.n
        sig = np.where(self.fill[l] > 0.05, self.fill[l], 0) * PL.T_CU[l] / RHO
        img = self.layer_img(V, l)
        gx = np.zeros((n, n))
        gy = np.zeros((n, n))
        gx[:, 1:-1] = (img[:, 2:] - img[:, :-2]) / (2 * RES * 1e-3)
        gy[1:-1, :] = (img[2:, :] - img[:-2, :]) / (2 * RES * 1e-3)
        J = sig * np.hypot(np.nan_to_num(gx), np.nan_to_num(gy)) * 1e-3     # A per mm width
        J[lin < 0] = np.nan
        return J


def op_points():
    path = os.path.join(PL.OUT, "p1_thermal.json")
    if os.path.exists(path):
        L = json.load(open(path))["losses"]
    else:
        import ps1_thermal as P1
        L = P1.driver_losses()
    i3 = S.I_3V0_TYP
    return {"hover": {"drv": L["hover"]["i_bus"], "ldo": i3},
            "full throttle": {"drv": L["full"]["i_bus"], "ldo": i3},
            "peak 4 A/driver": {"drv": 4.0, "ldo": 0.163}}


def main():
    S.style()
    b = PL.load()
    fine = PL.fine_rasters(b)
    nets = {n: Network(b, fine, n) for n in ("VBAT", "GND")}
    ops = op_points()
    res = {"ops": ops, "cases": {}}
    keep = None
    for lab, op in ops.items():
        loads = {r: op["drv"] for r in DRIVERS}
        loads["U1"] = op["ldo"]
        out = {}
        Vs = {}
        for name, nw in nets.items():
            V = nw.solve(loads)
            Vs[name] = V
            per = {}
            for r in DRIVERS + ["U1"]:
                per[r] = float(np.mean(np.abs(V[nw.load_nodes[r]])))
            vc = nw.via_currents(V)
            out[name] = dict(drop=per, p_diss=nw.dissipation(V),
                             via_i_max=max(v["i_max"] for v in vc),
                             vias_over_1A=sum(1 for v in vc if v["i_max"] > 1.0),
                             worst_vias=sorted(vc, key=lambda v: -v["i_max"])[:6])
        loop = {r: out["VBAT"]["drop"][r] + out["GND"]["drop"][r] for r in DRIVERS}
        out["loop_drop"] = loop
        out["total_current"] = 4 * op["drv"] + op["ldo"]
        res["cases"][lab] = out
        print("%-16s I_tot %5.2f A | VBAT drop %s | GND rise %s | loop max %.1f mV | P_cu %.1f mW | via I_max "
              "%.2f A (VBAT) %.2f A (GND)" % (
                  lab, out["total_current"],
                  " ".join("%s %.1f" % (r, 1e3 * out["VBAT"]["drop"][r]) for r in DRIVERS),
                  " ".join("%s %.1f" % (r, 1e3 * out["GND"]["drop"][r]) for r in DRIVERS),
                  1e3 * max(loop.values()), 1e3 * (out["VBAT"]["p_diss"] + out["GND"]["p_diss"]),
                  out["VBAT"]["via_i_max"], out["GND"]["via_i_max"]), flush=True)
        if lab == "full throttle":
            keep = Vs
    # ---- maps (full throttle) ------------------------------------------------------------------------------------
    fig, ax = plt.subplots(2, 3, figsize=(16, 10))
    plots = [("VBAT", 3, "L4 VBAT plane: drop (mV)"), ("VBAT", 0, "L1 VBAT copper: drop (mV)"),
             ("VBAT", 5, "L6 VBAT copper: drop (mV)"), ("GND", 1, "L2 GND plane: rise (mV)"),
             ("GND", 4, "L5 GND plane: rise (mV)"), ("VBAT", 3, "L4 VBAT plane: sheet current (A/mm)")]
    for k, (net, l, title) in enumerate(plots):
        a = ax.flat[k]
        nw = nets[net]
        if "sheet current" in title:
            img = nw.sheet_current_density(keep[net], l)
            im = a.imshow(img, extent=(-PL.X.HALF, PL.X.HALF, PL.X.HALF, -PL.X.HALF), cmap="viridis",
                          vmax=np.nanpercentile(img, 99.5))
            title += ", max %.2f" % np.nanmax(img)
        else:
            img = 1e3 * np.abs(nw.layer_img(keep[net], l))
            im = a.imshow(img, extent=(-PL.X.HALF, PL.X.HALF, PL.X.HALF, -PL.X.HALF), cmap="magma")
        for r in DRIVERS + ["U1", "J1"]:
            x, y = PL.xy(nw.fp[r].GetPosition())
            a.text(x, y, r, ha="center", va="center", fontsize=7, color="#f2f2f2")
        PL.board_extent_axes(a)
        a.set_title(title + " (full throttle)", fontsize=9)
        a.grid(False)
        fig.colorbar(im, ax=a, shrink=0.75)
    PL.save(fig, "p2_irdrop_maps.png")
    PL.write_results("p2_irdrop", res)


if __name__ == "__main__":
    main()
