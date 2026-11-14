"""
Study 3 - Thermal: DRV8311 losses and junction temperature on the 20x20 mm 6-layer hub PCB.

1) Driver losses per DRV8311 from the switching-level simulation (simlib.sim_6step: conduction
   incl. ripple, body-diode (dead time + demagnetisation), switching I*V*t_rf*f) at RDS(on) of
   25 C and 150 C (typ or max) -> linear in RDS(Tj); cross-checked with TI Table 12-1 formulas.
2) 2.5-D finite-difference model: 6 copper layers (40x40 cells of 0.5 mm each), in-plane copper
   (fill factors) + FR4, vertical FR4 + via barrels (background stitching vias + 5 thermal vias in
   each exposed pad), convection + radiation from top and bottom faces.  Implicit Euler in time.
   Tj = T(L1 under exposed pad) + P * (RthJC(bot) 4.8 C/W + 1.0 C/W solder/pad).
Pass criterion: Tj < 125 C.

Run:  python sim_03_thermal.py
"""
import math
import simlib as S
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from simlib import plt, PAL, CRIT, INK2

# ---------------- board / material assumptions ----------------
BW = 20e-3
NX = 40
DX = BW / NX
T_CU = [35e-6, 15.2e-6, 15.2e-6, 15.2e-6, 15.2e-6, 35e-6]
T_DIEL = [0.0994e-3, 0.16e-3, 0.13e-3, 0.16e-3, 0.0994e-3]
FILL = S.asm("cu_fill", [0.45, 0.90, 0.35, 0.75, 0.90, 0.45],
             "copper fill L1..L6 (L1/L6 parts+pours, L2/L5 GND planes, L3 signals, L4 VBAT/3V0 islands)")
K_CU, K_XY, K_Z = 390.0, 0.8, 0.30
RC_CU, RC_FR4 = 3.45e6, 2.0e6
S.asm("fr4", {"k_xy": K_XY, "k_z": K_Z, "rho_c": RC_FR4}, "FR4 properties (generic)")
H_RAD = S.asm("h_rad", 8.0, "linearised radiation, emissivity 0.9 solder mask, ~60-90 C surface")
H_STILL = S.asm("h_still", 12.0, "natural convection, small horizontal plate", (8, 15))
H_WASH = S.asm("h_wash", (30.0, 80.0), "forced convection in prop downwash (user range; flat-plate "
                                      "laminar estimate 60-85 W/m2K at 5-9 m/s induced velocity)")
AREA_F = S.asm("area_factor", 1.2, "component/edge surface-area enhancement")
VIA_A = math.pi * ((0.1e-3) ** 2 - (0.08e-3) ** 2)          # 0.2 mm drill, 20 um plating
VIA_DENS = S.asm("via_density", 0.3e6, "background through-via density 0.3 /mm^2 (GND stitching + signals)")
N_VIA_PAD = S.asm("n_via_pad", 5, "thermal vias (0.2 mm) in each DRV8311 exposed pad (TI land pattern)")
C_EXTRA = S.asm("c_components", 0.25, "lumped heat capacity of components (J/K)")
R_JPCB = S.ds("drv_theta_jc_bot") + S.asm("r_pad_solder", 1.0, "pad solder + local spreading C/W")
T_AMB = S.asm("t_amb", 25.0, "ambient C (add 10 K for 35 C summer)")
DRV_POS = [(5, 5), (15, 5), (5, 15), (15, 15)]
OCV_TH = S.asm("ocv_thermal", 4.05, "battery OCV used for thermal cases (~1.5 min into flight)")
I_DRV_STBY = 0.011


def idx(l, ix, iy):
    return l * NX * NX + iy * NX + ix


def cells_of(xc, yc, size):
    half = size / 2
    out = []
    for iy in range(NX):
        for ix in range(NX):
            x, y = (ix + 0.5) * DX * 1e3, (iy + 0.5) * DX * 1e3
            if abs(x - xc) < half and abs(y - yc) < half:
                out.append((ix, iy))
    return out


PAD_CELLS = [cells_of(x, y, 2.0) for (x, y) in DRV_POS]


def build(h_conv):
    n = 6 * NX * NX
    rows, cols, vals = [], [], []
    cap = np.zeros(n)
    gamb = np.zeros(n)

    def add(i, j, g):
        rows.extend([i, j, i, j])
        cols.extend([i, j, j, i])
        vals.extend([g, g, -g, -g])

    a = DX * DX
    for l in range(6):
        t_share = (T_DIEL[l - 1] / 2 if l > 0 else 0) + (T_DIEL[l] / 2 if l < 5 else 0)
        k_sheet = K_CU * T_CU[l] * FILL[l] + K_XY * t_share
        c_cell = (RC_CU * T_CU[l] * FILL[l] + RC_FR4 * (t_share + T_CU[l] * (1 - FILL[l]))) * a
        for iy in range(NX):
            for ix in range(NX):
                i = idx(l, ix, iy)
                cap[i] += c_cell
                if ix + 1 < NX:
                    add(i, idx(l, ix + 1, iy), k_sheet)
                if iy + 1 < NX:
                    add(i, idx(l, ix, iy + 1), k_sheet)
                if l < 5:
                    nv = VIA_DENS * a
                    g = K_Z * a / T_DIEL[l] + nv * K_CU * VIA_A / T_DIEL[l]
                    add(i, idx(l + 1, ix, iy), g)
    # thermal vias in pads (through all layers)
    for cells in PAD_CELLS:
        nv = N_VIA_PAD / len(cells)
        for (ix, iy) in cells:
            for l in range(5):
                add(idx(l, ix, iy), idx(l + 1, ix, iy), nv * K_CU * VIA_A / T_DIEL[l])
            cap[idx(0, ix, iy)] += 0.0144 / len(cells)   # package heat capacity (3x3x0.8 mm)
    # surfaces
    for iy in range(NX):
        for ix in range(NX):
            for l in (0, 5):
                gamb[idx(l, ix, iy)] = (h_conv + H_RAD) * a * AREA_F
                cap[idx(l, ix, iy)] += C_EXTRA / (2 * NX * NX)
    G = sp.csr_matrix((vals, (rows, cols)), shape=(n, n))
    G = G + sp.diags(gamb)
    return G.tocsc(), cap, gamb


def static_sources(vm, i3v0, i_bat, i_ph_rms):
    """non-driver heat (W) as a node vector."""
    n = 6 * NX * NX
    q = np.zeros(n)

    def put(l, xc, yc, size, p):
        cs = cells_of(xc, yc, size)
        for (ix, iy) in cs:
            q[idx(l, ix, iy)] += p / len(cs)
    put(0, 10, 10, 3.0, 3.0 * 0.035)                    # STM32G431 FC
    put(0, 10, 2, 1.0, max(vm - 3.0, 0) * i3v0)         # TPS7A2030
    for (x, y) in DRV_POS:
        put(5, x, y, 2.0, 3.0 * 0.008)                  # STM32G071 under each driver (bottom)
        put(0, x + (2.5 if x < 10 else -2.5), y, 1.0, 3 * i_ph_rms ** 2 * 0.005)   # phase traces
    put(5, 10, 16, 3.0, 3.0 * 0.012)                    # SX1280 (avg)
    # VBAT/GND distribution I^2R (3 mOhm total, assumption) spread over L2/L4/L5
    for l in (1, 3, 4):
        q[l * NX * NX:(l + 1) * NX * NX] += 0.003 * i_bat ** 2 / 3 / (NX * NX)
    return q


def driver_loss_model(case_sims):
    """returns function P(Tj) per driver from two sims at RDS1, RDS2 (+standby)."""
    (r1, p1), (r2, p2), vm, which = case_sims

    def f(tj):
        r = S.rds_of_T(tj, which)
        return p1 + (p2 - p1) * (r - r1) / (r2 - r1) + vm * I_DRV_STBY
    return f


def run_sims(L, which, f_pwm=S.F_PWM):
    """hover and full-throttle driver loss at RDS(25C) and RDS(150C)."""
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    t_h = auw / (4 * S.OVERLAP)
    out = {}
    for lab in ("hover", "full"):
        pts = []
        meta = None
        for tj in (25.0, 150.0):
            rds = S.rds_of_T(tj, which)
            if lab == "hover":
                avg, r = S.sim_at_thrust(t_h, soc_ocv=OCV_TH, r_ds=rds, l_ll=L, f_pwm=f_pwm, record=False)
            else:
                avg, r = S.sim_full_throttle(soc_ocv=OCV_TH, r_ds=rds, l_ll=L, f_pwm=f_pwm)
            pts.append((rds, r["p_drv_total"]))
            if tj == 25.0:
                meta = r
        out[lab] = {"model": driver_loss_model((pts[0], pts[1], meta["vm"], which)), "sim": meta,
                    "pts": pts}
    return out


def ti_table_12_1(i_pk, vm, rds_total, f_pwm=S.F_PWM, duty=0.5, t_dead=S.T_DEAD, vf=S.VF_BODY):
    """DRV8311 Table 12-1 (trapezoidal) with RDS,ON per FET = total/2."""
    p_con = 2 * i_pk ** 2 * rds_total / 2
    t_rf = vm / S.SR
    p_sw = 0.0 if duty >= 1 else i_pk * vm * t_rf * f_pwm
    p_d = 0.0 if duty >= 1 else 2 * i_pk * vf * t_dead * f_pwm
    return {"P_con": p_con, "P_sw": p_sw, "P_diode": p_d, "P_standby": vm * I_DRV_STBY,
            "total": p_con + p_sw + p_d + vm * I_DRV_STBY}


def solve_steady(G, gamb, q_static, ploss, tj0=60.0):
    n = G.shape[0]
    tj = np.full(4, tj0)
    lu = spla.splu(G)
    for _ in range(30):
        q = q_static.copy()
        p = np.array([ploss(t) for t in tj])
        for k, cells in enumerate(PAD_CELLS):
            for (ix, iy) in cells:
                q[idx(0, ix, iy)] += p[k] / len(cells)
        T = lu.solve(q + gamb * T_AMB)
        tj_new = np.array([np.mean([T[idx(0, ix, iy)] for (ix, iy) in cells]) for cells in PAD_CELLS]) + p * R_JPCB
        if np.max(np.abs(tj_new - tj)) < 0.01:
            tj = tj_new
            break
        tj = 0.5 * tj + 0.5 * tj_new
    return T, tj, p


def transient(G, cap, gamb, schedule, T0, dt=0.05):
    """schedule: list of (duration, q_static, ploss_fn)."""
    A = (sp.diags(cap / dt) + G).tocsc()
    lu = spla.splu(A)
    T = T0.copy()
    ts, tjs, ps = [], [], []
    t = 0.0
    for (dur, q_static, ploss) in schedule:
        nst = int(round(dur / dt))
        for _ in range(nst):
            tj = np.array([np.mean([T[idx(0, ix, iy)] for (ix, iy) in cells]) for cells in PAD_CELLS])
            p_now = np.array([ploss(tj_k + 0.0) for tj_k in tj])
            tjj = tj + p_now * R_JPCB
            p_now = np.array([ploss(x) for x in tjj])
            q = q_static.copy()
            for k, cells in enumerate(PAD_CELLS):
                for (ix, iy) in cells:
                    q[idx(0, ix, iy)] += p_now[k] / len(cells)
            T = lu.solve(cap / dt * T + q + gamb * T_AMB)
            t += dt
            tj = np.array([np.mean([T[idx(0, ix, iy)] for (ix, iy) in cells]) for cells in PAD_CELLS]) + p_now * R_JPCB
            ts.append(t)
            tjs.append(tj.max())
            ps.append(p_now.mean())
    return np.array(ts), np.array(tjs), np.array(ps), T


def main():
    print("Study 3: thermal")
    res = {"assumptions": {k: S.ASM[k] for k in ("cu_fill", "h_rad", "h_still", "h_wash", "area_factor",
                                                  "via_density", "n_via_pad", "c_components", "t_amb",
                                                  "ocv_thermal")}}
    loss_sets = {"nominal (L 6 uH, RDS typ)": run_sims(6e-6, "typ"),
                 "worst (L 3 uH, RDS max)": run_sims(3e-6, "max"),
                 "mitigated (L 6 uH, RDS typ, 96 kHz)": run_sims(6e-6, "typ", 96e3)}
    # loss table at Tj = 100 C
    tab = {}
    for name, ls in loss_sets.items():
        for lab in ("hover", "full"):
            sim = ls[lab]["sim"]
            p100 = ls[lab]["model"](100.0)
            tab[f"{name} | {lab}"] = {
                "P_total_at_Tj100_W": p100, "P_cond_25C_W": sim["p_cond"], "P_diode_W": sim["p_diode"],
                "P_sw_W": sim["p_sw"], "P_standby_W": sim["vm"] * I_DRV_STBY, "I_bus_A": sim["i_bus_avg"],
                "I_phase_rms_A": sim["i_ph_rms"], "I_torque_equiv_A": sim["torque"] / S.KT, "VM": sim["vm"],
                "duty": sim["duty"]}
            print(f"  {name:28s} {lab:5s}: P(Tj=100C) = {p100:.3f} W/driver  (I_bus {sim['i_bus_avg']:.2f} A, "
                  f"Iph_rms {sim['i_ph_rms']:.2f} A, D {sim['duty']:.2f})")
    res["driver_losses"] = tab
    # TI Table 12-1 cross-check (ideal flat-top current = torque-equivalent DC current)
    ti = {}
    for lab in ("hover", "full"):
        sim = loss_sets["nominal (L 6 uH, RDS typ)"][lab]["sim"]
        ti[lab] = ti_table_12_1(sim["torque"] / S.KT, sim["vm"], S.rds_of_T(100.0), duty=sim["duty"])
        print(f"  TI Table 12-1 ({lab}, flat-top I={sim['torque']/S.KT:.2f} A, Tj 100 C): {ti[lab]['total']:.3f} W")
    res["ti_table_12_1_check_Tj100"] = ti

    # board summary
    G, cap, gamb = build(30.0)
    res["board_heat_capacity_J_per_K"] = float(cap.sum())
    print(f"  board heat capacity {cap.sum():.2f} J/K")

    # static sources
    def qstat(lab, ls):
        sim = ls[lab]["sim"]
        i_bat = 4 * sim["i_bus_avg"] + S.I_HK
        return static_sources(sim["vm"], S.I_3V0_TYP, i_bat, sim["i_ph_rms"])

    ss = {}
    maps = {}
    for name, ls in loss_sets.items():
        for h in (H_STILL, H_WASH[0], H_WASH[1]):  # H_STILL = no airflow (canopy/bench)
            G, cap, gamb = build(h)
            for lab in ("hover", "full"):
                if lab == "full" and h == H_STILL:
                    continue
                T, tj, p = solve_steady(G, gamb, qstat(lab, ls), ls[lab]["model"])
                key = f"{name} | {lab} | h={h:.0f}"
                ss[key] = {"Tj_max_C": float(tj.max()), "T_board_mean_C": float(T.mean()), "exceeds_TSD_180C_runaway": bool(tj.max() > 180.0),
                           "P_driver_W": float(p.mean())}
                if lab == "hover" and h == H_WASH[0] and name.startswith("nominal"):
                    maps["hover"] = T
                print(f"  steady {key:52s} Tj = {tj.max():6.1f} C  board mean {T.mean():6.1f} C  P/drv {p.mean():.2f} W")
    res["steady_state"] = ss

    # bursts: hover steady state -> 5 s full throttle -> 10 s hover
    bursts = {}
    curves = {}
    for name, ls in loss_sets.items():
        for h in H_WASH:
            G, cap, gamb = build(h)
            T0, tj0, _ = solve_steady(G, gamb, qstat("hover", ls), ls["hover"]["model"])
            sched = [(5.0, qstat("full", ls), ls["full"]["model"]), (10.0, qstat("hover", ls), ls["hover"]["model"])]
            ts, tjs, ps, Tend = transient(G, cap, gamb, sched, T0)
            ts = np.concatenate([[0.0], ts])
            tjs = np.concatenate([[tj0.max()], tjs])
            i3 = np.argmin(np.abs(ts - 3.0))
            i5 = np.argmin(np.abs(ts - 5.0))
            over = ts[tjs >= 125.0]
            key = f"{name} | h={h:.0f}"
            bursts[key] = {"Tj_start_C": float(tjs[0]), "Tj_after_3s_C": float(tjs[i3]),
                           "Tj_after_5s_C": float(tjs[i5]), "Tj_peak_C": float(tjs.max()),
                           "t_to_125C_s": float(over[0]) if over.size else None}
            curves[key] = (ts, tjs)
            if name.startswith("nominal") and h == H_WASH[0]:
                # map at end of the 5 s burst
                ts2, tjs2, ps2, Tb = transient(G, cap, gamb, sched[:1], T0)
                maps["burst"] = Tb
            print(f"  burst {key:40s} Tj0 {tjs[0]:6.1f}  3 s {tjs[i3]:6.1f}  5 s {tjs[i5]:6.1f}  "
                  f"t(125C) {bursts[key]['t_to_125C_s']}")
    res["bursts"] = bursts
    # time constant estimate
    G, cap, gamb = build(H_WASH[0])
    res["global_time_constant_s_h30"] = float(cap.sum() / gamb.sum())

    # ================= plots =================
    fig, ax = plt.subplots(figsize=(8.5, 4.0))
    labels = []
    comps = {"conduction": [], "diode": [], "switching": [], "standby": []}
    for name, ls in loss_sets.items():
        for lab in ("hover", "full"):
            sim = ls[lab]["sim"]
            scale = ls[lab]["model"](100.0) - sim["vm"] * I_DRV_STBY
            base = sim["p_cond"] + sim["p_diode"] + sim["p_sw"]
            k = scale / base if base > 0 else 1.0
            comps["conduction"].append(sim["p_cond"] * k)
            comps["diode"].append(sim["p_diode"] * k)
            comps["switching"].append(sim["p_sw"] * k)
            comps["standby"].append(sim["vm"] * I_DRV_STBY)
            labels.append(f"{lab}\n{name.split(' (')[0]}")
    x = np.arange(len(labels))
    bottom = np.zeros(len(labels))
    for (cname, vals), col in zip(comps.items(), PAL):
        ax.bar(x, vals, 0.6, bottom=bottom, color=col, edgecolor=S.SURF, linewidth=2, label=cname)
        bottom += np.array(vals)
    for xi, b in zip(x, bottom):
        ax.text(xi, b + 0.03, f"{b:.2f} W", ha="center", va="bottom", fontsize=8, color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("loss per DRV8311 at Tj = 100 C (W)")
    ax.set_title(f"DRV8311 loss breakdown (OCV {OCV_TH} V; nominal = L 6 uH/RDS typ, worst = L 3 uH/RDS max)")
    ax.legend(loc="upper left")
    S.save(fig, "s3_driver_losses.png")

    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.3))
    for ax, key, title in ((axs[0], "hover", "L1 temperature, hover steady state (h = 30 W/m2K)"),
                           (axs[1], "burst", "L1 temperature after 5 s full throttle (h = 30)")):
        T = maps[key][:NX * NX].reshape(NX, NX)
        im = ax.imshow(T, origin="lower", extent=(0, 20, 0, 20), cmap="Oranges")
        cb = fig.colorbar(im, ax=ax, shrink=0.85)
        cb.set_label("C")
        for (xx, yy) in DRV_POS:
            ax.add_patch(plt.Rectangle((xx - 1.5, yy - 1.5), 3, 3, fill=False, ec=INK2, lw=0.8))
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title(title, fontsize=9.5)
        ax.grid(False)
    fig.tight_layout()
    S.save(fig, "s3_temperature_maps.png")

    fig, ax = plt.subplots(figsize=(8.5, 4.4))
    for (key, (ts, tjs)), col in zip(curves.items(), PAL):
        ax.plot(ts, tjs, color=col, label=key)
    ax.axvspan(0, 5, color="#eeeeea", zorder=0)
    ax.text(2.5, 22, "full throttle", ha="center", color=INK2, fontsize=8)
    S.hline_limit(ax, 125, "criterion Tj < 125 C")
    S.hline_limit(ax, 150, "DRV8311 TJ max operating 150 C", ls=":")
    S.hline_limit(ax, 180, "thermal shutdown min 180 C", ls="-.")
    ax.set_xlabel("time from start of full-throttle burst (s)")
    ax.set_ylabel("hottest DRV8311 junction (C)")
    ax.set_title(f"5 s full-throttle burst from hover steady state (ambient {T_AMB:.0f} C)")
    ax.set_ylim(20, 350)
    ax.legend(loc="upper right", fontsize=7.5)
    S.save(fig, "s3_burst_tj.png")

    S.write_results("study3_thermal", res)


if __name__ == "__main__":
    main()
