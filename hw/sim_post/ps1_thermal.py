"""Post-layout study P1: thermal model of the routed board (6 copper sheets + dielectrics + real vias).

Model
-----
* Each copper layer is a 2-D sheet of 0.2 mm cells. In-plane conductance per cell = k_Cu*t_Cu*fill + k_FR4,xy*t_assoc,
  where fill is the real copper fraction from the routed artwork (tracks + pads + filled zones, 0.05 mm raster) and
  t_assoc is half of each adjacent dielectric. Neighbour links use the harmonic mean.
* Sheets are coupled through the dielectrics (k_FR4,z) and through every GND/VBAT via and plated footprint hole:
  a copper barrel (18 um plating [A]) with one node per layer. It ties to a sheet only where that layer's copper of
  the same net surrounds the hole (ring test on the fine raster); plane antipads stay open.
* Heat sources:
  - DRV8311 loss from the hw/sim switching model at the chosen operating point, split into a conduction part that
    scales with RDS(on)(Tj) and a fixed part (dead-time diode + switching + standby). It is injected at a junction
    node joined to the exposed-pad cells through RthJC(bot) 4.8 C/W [DS] + 0.3 C/W solder [A].
  - Logic: G431 0.12 W, G071 0.025 W each, SX1280 (LDO mode) 0.06 W, IMU 2 mW, LDO (VBAT-3.0)*I3V0.
  - Phase-trace I^2R from the real track geometry at the simulated phase RMS current.
  - 15 % [A] of each bottom-mounted motor's loss conducts into its mounting disc. Top motors sit on 3 mm spacers.
* Convection on both faces, plus linearised radiation 7 W/m2K: h_wash under the prop discs (0.2R..R),
  h_free = 12 W/m2K elsewhere, and none under a motor base on its mounting face. h_wash scales with
  (thrust/hover)^0.25, from v_induced ~ sqrt(T) and laminar h ~ sqrt(v).
* Steady state uses a fixed-point loop on Tj. Transients use implicit Euler at 50 ms with FR4 + Cu + package heat
  capacity.

Outputs: out/p1_thermal_maps.png, out/p1_tj_scenarios.png, out/p1_burst.png, out/p1_thermal.json
"""
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
RES = 0.2                        # mm per thermal cell
K = int(round(RES / PL.X.RES))   # fine pixels per cell
T_AMB = 25.0
H_FREE = 12.0
H_RAD = 7.0
R_PROP = 15.5
JC = 4.8 + 0.3                   # C/W junction -> exposed pad (RthJC(bot) [DS] + solder [A])
MOTOR_FRAC = 0.15
F_PWM = 96e3                     # chosen (study 9)
OCV = 4.05                       # as study 3 (about 1.5 min into a flight)

LOGIC = {"U2": 0.12, "U21": 0.025, "U22": 0.025, "U23": 0.025, "U24": 0.025, "U4": 0.06, "U3": 0.002}
PKG_C = {"U11": (3, 3, 0.75), "U12": (3, 3, 0.75), "U13": (3, 3, 0.75), "U14": (3, 3, 0.75),
         "U21": (4, 4, 0.55), "U22": (4, 4, 0.55), "U23": (4, 4, 0.55), "U24": (4, 4, 0.55),
         "U2": (7, 7, 0.55), "U4": (4, 4, 0.9), "U3": (3, 2.5, 0.83), "U1": (1, 1, 0.4)}
RHOC_PKG = 1.8e6                 # J/m3K, mould compound + lead frame [A]
DRIVERS = ["U11", "U12", "U13", "U14"]


# ------------------------------------------------------------------------------------------------------------
def driver_losses():
    """Hover / full-throttle per-driver losses at Tj 100 C, split (conduction, fixed), from the hw/sim model."""
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    th = auw / (4 * S.OVERLAP)
    rds100 = S.rds_of_T(100.0, "typ")
    out = {}
    cases = [("hover", "typ", S.L_LL, F_PWM), ("hover_48k", "typ", S.L_LL, 48e3),
             ("hover_worst", "max", 3e-6, F_PWM), ("full", "typ", S.L_LL, F_PWM)]
    for lab, which, L, f in cases:
        rds = S.rds_of_T(100.0, which)
        if lab.startswith("full"):
            avg, r = S.sim_full_throttle(soc_ocv=OCV, r_ds=rds, l_ll=L, f_pwm=f)
        else:
            avg, r = S.sim_at_thrust(th, soc_ocv=OCV, r_ds=rds, l_ll=L, f_pwm=f, record=False)
        stby = r["vm"] * 0.011
        p_motor = r["p_cu_motor"] + S.KT * S.I0 * r["w_m"]
        out[lab] = dict(which=which, l_ll=L, f_pwm=f, rds100=rds, p_cond=r["p_cond"],
                        p_fixed=r["p_diode"] + r["p_sw"] + stby, i_ph_rms=r["i_ph_rms"], i_bus=r["i_bus_avg"],
                        vm=r["vm"], p_motor=p_motor, thrust_n=(S.KT_P * r["w_m"] ** 2), duty=r["duty"])
        print("  %-12s P_cond %.3f W  P_fixed %.3f W  I_ph,rms %.2f A  motor %.2f W  VM %.2f V" % (
            lab, r["p_cond"], out[lab]["p_fixed"], r["i_ph_rms"], p_motor, r["vm"]), flush=True)
    out["hover_thrust_n"] = th
    out["rds100_typ"] = rds100
    return out


def p_driver(case, tj):
    return case["p_cond"] * S.rds_of_T(tj, case["which"]) / case["rds100"] + case["p_fixed"]


# ------------------------------------------------------------------------------------------------------------
class Model:
    def __init__(self, b):
        t0 = time.time()
        self.b = b
        self.fp = PL.footprints(b)
        fine = PL.fine_rasters(b)
        self.fine = fine
        out_c = PL.block_mean(fine["outline"].astype(float), K)
        self.n = out_c.shape[0]
        self.inside = out_c > 0.5
        self.fill = [PL.block_mean(fine["cu", L].astype(float), K) * self.inside for L in PL.CU_LAYERS]
        n = self.n
        ncell = int(self.inside.sum())
        self.ncell = ncell
        lin = -np.ones((n, n), dtype=np.int64)
        lin[self.inside] = np.arange(ncell)
        self.lin = lin
        A = (RES * 1e-3) ** 2
        self.A = A
        t_assoc = np.zeros(6)
        for l in range(6):
            if l > 0:
                t_assoc[l] += PL.T_DIEL[l - 1] / 2
            if l < 5:
                t_assoc[l] += PL.T_DIEL[l] / 2
        self.t_assoc = t_assoc
        rows, cols, gs = [], [], []

        def link(a, c, g):
            rows.append(a)
            cols.append(c)
            gs.append(g)
        # in-plane
        for l in range(6):
            kt = PL.K_CU * PL.T_CU[l] * self.fill[l] + PL.K_FR4_XY * t_assoc[l]
            base = l * ncell
            for di, dj in ((0, 1), (1, 0)):
                a = lin[:n - di, :n - dj]
                c = lin[di:, dj:]
                m = (a >= 0) & (c >= 0)
                ka = kt[:n - di, :n - dj][m]
                kc = kt[di:, dj:][m]
                link(base + a[m], base + c[m], 2.0 / (1.0 / ka + 1.0 / kc))
        # through dielectrics
        cell_ids = np.arange(ncell)
        for l in range(5):
            g = PL.K_FR4_Z * A / PL.T_DIEL[l]
            link(l * ncell + cell_ids, (l + 1) * ncell + cell_ids, np.full(ncell, g))
        nn = 6 * ncell
        # vias (power vias + plated holes): one node per layer
        vias = PL.power_vias(b)
        for f in b.GetFootprints():          # plated holes of other nets (motor/battery pads): outer layers only
            for p in f.Pads():
                if p.GetAttribute() == PL.pcbnew.PAD_ATTRIB_PTH and p.GetDrillSize().x > 0 and \
                        p.GetNetname() not in ("GND", "VBAT"):
                    vias.append((*PL.xy(p.GetPosition()), p.GetNetname(), p.GetDrillSize().x / 1e6,
                                 p.GetSize().x / 1e6))
        nv_conn = 0
        self.via_list = []
        for (x, y, net, drill, pad) in vias:
            i, j = PL.cell_of(x, y, RES)
            if not (0 <= i < n and 0 <= j < n) or lin[i, j] < 0:
                continue
            r = drill / 2 * 1e-3
            ab = math.pi * ((r + PL.T_PLATE) ** 2 - r ** 2)
            vid = nn
            nn += 6
            for l in range(5):
                link(vid + l, vid + l + 1, PL.K_CU * ab / PL.T_DIEL[l])
            conn = []
            for l, L in enumerate(PL.CU_LAYERS):
                if net in ("GND", "VBAT"):
                    ok = PL.ring_fraction(fine[net, L], x, y, pad / 2 + 0.06, n=64) > 0.0
                else:
                    ok = l in (0, 5)
                if ok:
                    link(vid + l, l * ncell + lin[i, j], 1.0)
                    conn.append(l)
                    nv_conn += 1
            self.via_list.append((x, y, net, drill, conn))
        # junction nodes of the drivers
        self.jnode = {}
        for ref in DRIVERS:
            f = self.fp[ref]
            ep = [p for p in f.Pads() if p.GetAttribute() == PL.pcbnew.PAD_ATTRIB_SMD and p.GetSize().x > 1.5e6]
            x, y = PL.xy(ep[0].GetPosition())
            w = ep[0].GetSize().x / 1e6
            layer = PL.fp_side(f)
            cells = []
            for yy in np.arange(y - w / 2 + RES / 2, y + w / 2, RES):
                for xx in np.arange(x - w / 2 + RES / 2, x + w / 2, RES):
                    ci, cj = PL.cell_of(xx, yy, RES)
                    if lin[ci, cj] >= 0:
                        cells.append(layer * ncell + lin[ci, cj])
            jn = nn
            nn += 1
            for c in cells:
                link(jn, c, 1.0 / JC / len(cells))
            self.jnode[ref] = (jn, layer, x, y)
        self.nn = nn
        self.links = (np.concatenate([np.atleast_1d(np.asarray(r)) for r in rows]),
                      np.concatenate([np.atleast_1d(np.asarray(c)) for c in cols]),
                      np.concatenate([np.atleast_1d(np.asarray(g, dtype=float)) for g in gs]))
        self.vias_used = len(self.via_list)
        self.via_layer_links = nv_conn
        # heat capacity
        C = np.zeros(nn)
        for l in range(6):
            cl = A * (PL.RHOC_CU * PL.T_CU[l] * self.fill[l] + PL.RHOC_FR4 * t_assoc[l])
            C[l * ncell:(l + 1) * ncell] = cl[self.inside]
        for vid in range(6 * ncell, nn):
            C[vid] = 1e-6
        for ref, (w, h, z) in PKG_C.items():
            f = self.fp[ref]
            x, y = PL.xy(f.GetPosition())
            layer = PL.fp_side(f)
            cells = self.cells_in_box(x, y, w, h)
            ctot = w * h * z * 1e-9 * RHOC_PKG
            for c in cells:
                C[layer * ncell + c] += ctot / len(cells)
        for ref, (jn, *_r) in self.jnode.items():
            C[jn] = 1.5e-3
        self.C = C
        self.motors = PL.motor_centres(b)
        print("thermal model: %d x %d cells @ %.2f mm, %d cells/layer, %d nodes, %d vias (%d via-layer ties), "
              "C_total %.2f J/K, %.1f s" % (n, n, RES, ncell, nn, self.vias_used, nv_conn, C.sum(),
                                            time.time() - t0), flush=True)

    def cells_in_box(self, x, y, w, h):
        out = []
        for yy in np.arange(y - h / 2 + RES / 2, y + h / 2, RES):
            for xx in np.arange(x - w / 2 + RES / 2, x + w / 2, RES):
                ci, cj = PL.cell_of(xx, yy, RES)
                if 0 <= ci < self.n and 0 <= cj < self.n and self.lin[ci, cj] >= 0:
                    out.append(self.lin[ci, cj])
        return out

    def cell_xy(self):
        ii, jj = np.nonzero(self.inside)
        return (jj + 0.5) * RES - PL.X.HALF, (ii + 0.5) * RES - PL.X.HALF

    def h_maps(self, h_wash):
        """Convective + radiative h per cell for the top (L1) and bottom (L6) faces."""
        x, y = self.cell_xy()
        top = np.full(self.ncell, H_FREE)
        bot = np.full(self.ncell, H_FREE)
        for name, (mx, my, side) in self.motors.items():
            r = np.hypot(x - mx, y - my)
            wash = (r > 0.2 * R_PROP) & (r < R_PROP)
            top[wash] = np.maximum(top[wash], h_wash)
            bot[wash] = np.maximum(bot[wash], h_wash)
            base = r < 4.5
            if side == 0:
                top[base] = 0.0 if False else top[base]      # top motors on spacers: air gap stays ventilated
            else:
                bot[base] = 0.0
        return top + H_RAD, bot + H_RAD

    def system(self, h_wash):
        r, c, g = self.links
        nn = self.nn
        top, bot = self.h_maps(h_wash)
        gamb = np.zeros(nn)
        gamb[:self.ncell] = top * self.A
        gamb[5 * self.ncell:6 * self.ncell] = bot * self.A
        G = sp.coo_matrix((np.concatenate([g, g, -g, -g]),
                           (np.concatenate([r, c, r, c]), np.concatenate([r, c, c, r]))), shape=(nn, nn)).tocsc()
        G = G + sp.diags(gamb)
        return G.tocsc(), gamb

    def static_q(self, case, i_ph_rms):
        """Heat sources other than the driver junctions (W per node)."""
        q = np.zeros(self.nn)
        nc = self.ncell
        for ref, p in LOGIC.items():
            f = self.fp[ref]
            x, y = PL.xy(f.GetPosition())
            w, h = PKG_C.get(ref, (2, 2, 0))[:2]
            cells = self.cells_in_box(x, y, w * 0.8, h * 0.8)
            for cc in cells:
                q[PL.fp_side(f) * nc + cc] += p / len(cells)
        # LDO
        i3v0 = S.I_3V0_TYP
        p_ldo = max(case["vm"] - 3.0, 0) * i3v0
        f = self.fp["U1"]
        x, y = PL.xy(f.GetPosition())
        for cc in self.cells_in_box(x, y, 0.8, 0.8):
            q[PL.fp_side(f) * nc + cc] += p_ldo / len(self.cells_in_box(x, y, 0.8, 0.8))
        # phase-trace I^2R (hot copper resistivity at 60 C)
        rho = PL.RHO_CU * (1 + PL.ALPHA_CU * 40)
        p_tr = 0.0
        for net, l, x0, y0, x1, y1, w in PL.tracks(self.b):
            if not (net.startswith("M") and "_OUT" in net):
                continue
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 1e-6:
                continue
            R = rho * L * 1e-3 / (w * 1e-3 * PL.T_CU[l])
            p = i_ph_rms ** 2 * R
            p_tr += p
            ns = max(1, int(L / RES))
            for k in range(ns):
                xx = x0 + (x1 - x0) * (k + 0.5) / ns
                yy = y0 + (y1 - y0) * (k + 0.5) / ns
                ci, cj = PL.cell_of(xx, yy, RES)
                if self.lin[ci, cj] >= 0:
                    q[l * nc + self.lin[ci, cj]] += p / ns
        # bottom-motor heat through the mounting disc
        x, y = self.cell_xy()
        p_mot = 0.0
        for name, (mx, my, side) in self.motors.items():
            if side != 5:
                continue
            m = (np.hypot(x - mx, y - my) < 4.5) & (np.hypot(x - mx, y - my) > 2.0)
            q[5 * nc:6 * nc][m] += MOTOR_FRAC * case["p_motor"] / m.sum()
            p_mot += MOTOR_FRAC * case["p_motor"]
        return q, dict(p_ldo=p_ldo, p_phase_traces=p_tr, p_motor_in=p_mot, p_logic=sum(LOGIC.values()))

    def steady(self, case, h_wash, tj0=80.0):
        G, gamb = self.system(h_wash)
        lu = spla.splu(G, permc_spec="MMD_AT_PLUS_A")
        q0, meta = self.static_q(case, case["i_ph_rms"])
        tj = {ref: tj0 for ref in DRIVERS}
        T = None
        for it in range(60):
            q = q0.copy()
            for ref, (jn, *_r) in self.jnode.items():
                q[jn] += p_driver(case, min(tj[ref], 400.0))
            T = lu.solve(q + gamb * T_AMB)
            new = {ref: T[self.jnode[ref][0]] for ref in DRIVERS}
            d = max(abs(new[r] - tj[r]) for r in DRIVERS)
            tj = new
            if d < 0.01 or max(tj.values()) > 400:
                break
        runaway = max(tj.values()) > 300
        return T, tj, meta, runaway

    def transient(self, schedule, T0, dt=0.05):
        """schedule: list of (duration_s, case, h_wash). Returns t, Tj per driver, T_max on board."""
        ts, tjs = [0.0], [[T0[self.jnode[r][0]] for r in DRIVERS]]
        T = T0.copy()
        t = 0.0
        cache = {}
        for dur, case, hw in schedule:
            key = (id(case), hw)
            if key not in cache:
                G, gamb = self.system(hw)
                M = (sp.diags(self.C / dt) + G).tocsc()
                cache[key] = (spla.splu(M, permc_spec="MMD_AT_PLUS_A"), gamb, self.static_q(case, case["i_ph_rms"])[0])
            lu, gamb, q0 = cache[key]
            for _ in range(int(round(dur / dt))):
                q = q0.copy()
                for ref, (jn, *_r) in self.jnode.items():
                    q[jn] += p_driver(case, min(T[jn], 400.0))
                T = lu.solve(self.C / dt * T + q + gamb * T_AMB)
                t += dt
                ts.append(t)
                tjs.append([T[self.jnode[r][0]] for r in DRIVERS])
        return np.array(ts), np.array(tjs), T

    def layer_map(self, T, l):
        img = np.full((self.n, self.n), np.nan)
        img[self.inside] = T[l * self.ncell:(l + 1) * self.ncell]
        return img

    def probe(self, T, ref):
        f = self.fp[ref]
        x, y = PL.xy(f.GetPosition())
        ci, cj = PL.cell_of(x, y, RES)
        return float(T[PL.fp_side(f) * self.ncell + self.lin[ci, cj]])


# ------------------------------------------------------------------------------------------------------------
def main():
    S.style()
    print("driver losses (hw/sim switching model, Tj 100 C):", flush=True)
    L = driver_losses()
    b = PL.load()
    m = Model(b)
    hover, full = L["hover"], L["full"]
    thr_ratio = full["thrust_n"] / L["hover_thrust_n"]
    h_scale = thr_ratio ** 0.25
    scen = [("hover, h_wash 60 (nominal)", hover, 60.0), ("hover, h_wash 30", hover, 30.0),
            ("hover, h_wash 100", hover, 100.0), ("hover worst (3 uH, RDS max), h_wash 60", L["hover_worst"], 60.0),
            ("hover at 48 kHz, h_wash 60", L["hover_48k"], 60.0), ("hover, still air (h 12)", hover, 0.0),
            ("full throttle steady, h_wash 60x%.2f" % h_scale, full, 60.0 * h_scale)]
    res = {"losses": L, "scenarios": {}, "model": dict(cells=m.ncell, res_mm=RES, nodes=m.nn, vias=m.vias_used,
                                                        via_layer_ties=m.via_layer_links, C_total=float(m.C.sum()),
                                                        h_free=H_FREE, h_rad=H_RAD, motor_frac=MOTOR_FRAC)}
    T_nom = None
    for lab, case, hw in scen:
        T, tj, meta, run = m.steady(case, hw)
        probes = {r: m.probe(T, r) for r in ("U2", "U3", "U4", "U1", "J1")}
        res["scenarios"][lab] = dict(tj=tj, runaway=run, p_driver={r: p_driver(case, min(tj[r], 400)) for r in DRIVERS},
                                     probes=probes, meta=meta, h_wash=hw,
                                     board_max=float(np.nanmax(T[:6 * m.ncell])),
                                     board_mean=float(T[:m.ncell].mean()))
        print("  %-44s Tj %s %s | IMU %.1f FC %.1f radio %.1f LDO %.1f C" % (
            lab, " ".join("%s %.1f" % (r, tj[r]) for r in DRIVERS), "RUNAWAY" if run else "",
            probes["U3"], probes["U2"], probes["U4"], probes["U1"]), flush=True)
        if T_nom is None:
            T_nom = T
            res["meta_nominal"] = meta
    # ---- transients: hover steady -> full-throttle bursts -> hover ---------------------------------------------
    bursts = {}
    for dur in (3.0, 5.0, 10.0):
        t, tj, _ = m.transient([(dur, full, 60.0 * h_scale), (10.0, hover, 60.0)], T_nom)
        pk = tj.max(axis=1)
        cross = [float(t[k]) for k in range(len(t)) if pk[k] >= 125.0]
        bursts["%.0f s" % dur] = dict(t=t[::4], tj_max=pk[::4], peak=float(pk.max()),
                                      t_125=cross[0] if cross else None)
        print("  burst %4.0f s: peak Tj %.1f C%s" % (dur, pk.max(), ", 125 C at %.1f s" % cross[0] if cross else ""))
    t, tj, _ = m.transient([(30.0, full, 60.0 * h_scale)], T_nom)
    pk = tj.max(axis=1)
    cross = [float(t[k]) for k in range(len(t)) if pk[k] >= 125.0]
    bursts["sustained"] = dict(t=t[::4], tj_max=pk[::4], peak=float(pk.max()), t_125=cross[0] if cross else None)
    print("  sustained full throttle: Tj %.1f C after 30 s%s" % (pk[-1], ", 125 C at %.1f s" % cross[0] if cross else ""))
    res["bursts"] = {k: dict(peak=v["peak"], t_125=v["t_125"]) for k, v in bursts.items()}
    res["h_scale_full"] = h_scale
    # ---- burst budget: longest full-throttle burst from hover that keeps Tj below 125 / 150 C ------------------------
    t, tj, _ = m.transient([(20.0, full, 60.0 * h_scale)], T_nom, dt=0.02)
    pk = tj.max(axis=1)
    budget = {}
    for lim in (125.0, 150.0, 175.0):
        k = np.nonzero(pk >= lim)[0]
        budget["%.0f C" % lim] = float(t[k[0]]) if len(k) else None
    res["burst_budget_s"] = budget
    print("  full-throttle budget from hover: %s" % ", ".join("%s after %s s" % (k, "%.2f" % v if v else ">20")
                                                            for k, v in budget.items()))
    # ---- compact model for the firmware: Foster fit of Zth(t) for a 1 W step on all four junctions ----------------
    from scipy.optimize import curve_fit
    G, gamb = m.system(60.0)
    ts_all, z_all = [], []
    T = np.full(m.nn, T_AMB)
    t_acc = 0.0
    for dt, dur in ((0.001, 0.05), (0.005, 0.45), (0.05, 9.5), (0.5, 110.0)):
        M = (sp.diags(m.C / dt) + G).tocsc()
        lu = spla.splu(M, permc_spec="MMD_AT_PLUS_A")
        q = np.zeros(m.nn)
        for ref, (jn, *_r) in m.jnode.items():
            q[jn] = 1.0
        for _ in range(int(round(dur / dt))):
            T = lu.solve(m.C / dt * T + q + gamb * T_AMB)
            t_acc += dt
            ts_all.append(t_acc)
            z_all.append(max(T[m.jnode[r][0]] for r in DRIVERS) - T_AMB)
    ts_all, z_all = np.array(ts_all), np.array(z_all)

    def foster(tt, r1, t1, r2, t2, r3, t3):
        return r1 * (1 - np.exp(-tt / t1)) + r2 * (1 - np.exp(-tt / t2)) + r3 * (1 - np.exp(-tt / t3))
    p0 = (6.0, 0.01, 15.0, 1.0, max(z_all[-1] - 21, 5.0), 20.0)
    popt, _ = curve_fit(foster, ts_all, z_all, p0=p0, bounds=(0, np.inf), maxfev=20000)
    fit_err = float(np.max(np.abs(foster(ts_all, *popt) - z_all)))
    pairs = sorted([(popt[i], popt[i + 1]) for i in (0, 2, 4)], key=lambda p: p[1])
    res["foster_all4_1W"] = dict(R_C_per_W=[p[0] for p in pairs], tau_s=[p[1] for p in pairs],
                                 max_fit_error_C=fit_err, zth_inf=float(z_all[-1]),
                                 note="Tj = T_amb + sum_i R_i * filtered(P), each pole a 1st-order lag tau_i; "
                                      "P = the same per-driver loss on all four drivers; h_wash 60")
    print("  firmware Foster model (1 W on each driver): %s, max fit error %.2f C" % (
        ", ".join("R %.2f C/W tau %.3f s" % p for p in pairs), fit_err))
    # ---- copper-weight sensitivity: 1 oz inner layers ---------------------------------------------------------------
    t_cu_save = PL.T_CU.copy()
    PL.T_CU[1:5] = 0.0305e-3
    m1 = Model(b)
    T1, tj1, _m, _r = m1.steady(hover, 60.0)
    t, tjb, _ = m1.transient([(3.0, full, 60.0 * h_scale)], T1)
    PL.T_CU[:] = t_cu_save
    res["inner_1oz"] = dict(tj_hover=tj1, burst3_peak=float(tjb.max()))
    print("  1 oz inner layers: hover Tj %s, 3 s burst peak %.1f C" % (
        " ".join("%.1f" % v for v in tj1.values()), tjb.max()))

    # ---- figures -------------------------------------------------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.2))
    vmax = np.nanmax([np.nanmax(m.layer_map(T_nom, l)) for l in (0, 5)])
    for k, (l, title) in enumerate(((0, "L1 (top)"), (5, "L6 (bottom)"), (1, "L2 GND plane"))):
        img = m.layer_map(T_nom, l)
        im = ax[k].imshow(img, extent=(-PL.X.HALF, PL.X.HALF, PL.X.HALF, -PL.X.HALF), cmap="inferno",
                          vmin=T_AMB, vmax=vmax)
        for ref in ("U11", "U12", "U13", "U14", "U2", "U3", "U4", "U1"):
            f = m.fp[ref]
            x, y = PL.xy(f.GetPosition())
            on = PL.fp_side(f) == l
            ax[k].text(x, y, ref, ha="center", va="center", fontsize=7,
                       color="white" if on else "#9a9a9a", alpha=1.0 if on else 0.7)
        PL.board_extent_axes(ax[k])
        ax[k].set_title("%s, hover, h_wash 60: max %.1f C" % (title, np.nanmax(img)))
        ax[k].grid(False)
    fig.colorbar(im, ax=ax, shrink=0.8, label="T (C)")
    PL.save(fig, "p1_thermal_maps.png")

    fig, ax = plt.subplots(figsize=(11, 4.8))
    labs = list(res["scenarios"])
    xs = np.arange(len(labs))
    for k, ref in enumerate(DRIVERS):
        vals = [min(res["scenarios"][s]["tj"][ref], 260) for s in labs]
        ax.bar(xs + (k - 1.5) * 0.2, vals, 0.2, color=S.PAL[k], label=ref)
    S.hline_limit(ax, 125, "125 C design limit")
    S.hline_limit(ax, 180, "180 C TSD (min)")
    ax.set_xticks(xs)
    ax.set_xticklabels([s.replace(", ", ",\n", 1) for s in labs], fontsize=7.5)
    ax.set_ylabel("Tj (C), clipped at 260")
    ax.set_title("DRV8311 junction temperature on the routed board, ambient %.0f C, PWM 96 kHz unless noted" % T_AMB)
    ax.legend(fontsize=8)
    PL.save(fig, "p1_tj_scenarios.png")

    fig, ax = plt.subplots(figsize=(9, 4.6))
    for k, (lab, v) in enumerate(bursts.items()):
        ax.plot(v["t"], v["tj_max"], color=S.PAL[k], label="full throttle " + lab)
    S.hline_limit(ax, 125, "125 C")
    S.hline_limit(ax, 180, "TSD 180 C min")
    ax.set(xlabel="time from hover steady state (s)", ylabel="hottest Tj (C)",
           title="Full-throttle bursts from hover (h_wash 60 at hover, x%.2f at full)" % h_scale)
    ax.legend(fontsize=8)
    PL.save(fig, "p1_burst.png")
    PL.write_results("p1_thermal", res)


if __name__ == "__main__":
    main()
