"""
Study 1 - Battery hot-plug transient on VBAT.

Model (lumped, nonlinear in the MLCC capacitance):
  Vbatt step -> R_s (cell+connector R_batt + pigtail R_pig) -> L_s (pigtail+tab loop)
  -> node P  : 2x22uF 0603 X5R (DC-bias C(V), ESR)           [battery pads]
  -> L_pl,R_pl (VBAT plane/L4 pads->drivers)
  -> node D  : 4x22uF + 4x100nF                               [driver VM pins, lumped]
  -> L_tr,R_tr (trace to LDO)
  -> node LDO: 1uF 0402 at TPS7A20 VIN                         [assumption]
No load current (worst case: no extra damping).  Fixed-step trapezoidal integration (5 ns).

Limits: TPS7A20 VIN abs max 6.5 V (ROC max 6.0 V); DRV8311 VM abs max 24 V.
Run:  python sim_01_hotplug.py
"""
import math
import simlib as S  # must be first: adds hw/sim/.pylibs to sys.path
import numpy as np

from simlib import plt, PAL, CRIT, INK2

# ---------------- assumptions specific to this study ----------------
C22 = 22e-6
ESR22 = S.ASM["c22_esr"]["value"]
L_PL = S.asm("l_plane_vbat", 0.8e-9, "VBAT plane+vias, battery pads -> driver caps", (0.4e-9, 1.5e-9))
R_PL = S.asm("r_plane_vbat", 0.002, "VBAT plane+vias resistance", (0.001, 0.004))
L_TR = S.asm("l_ldo_feed", 3e-9, "VBAT trace to TPS7A20 VIN", (1e-9, 5e-9))
R_TR = S.asm("r_ldo_feed", 0.02, "VBAT trace resistance to LDO")
C_LDO_IN = S.asm("c_ldo_in", 1e-6, "TPS7A20 input cap 1 uF 0402 X5R (TI recommends >=0.47 uF eff.)")
R_LDO_IN_ESR = 0.015
RET_1U_4V = 0.55  # 1 uF 0402 10V X5R retained at ~4 V (assumption)


def c_bank(v, n22, tol, v0, extra=0.0):
    return n22 * C22 * tol * S.c22_retention(v, v0) + extra


def c_ldo_in(v):
    return C_LDO_IN * (1 - (1 - RET_1U_4V) * min(max(v, 0.0) / 4.0, 1.2))


def simulate(v_bat=S.V_FULL, r_batt=S.R_BATT, l_s=S.L_PIG, tol=1.0, v0=4.0, t_end=120e-6,
             extra_damp=None, r_pig=S.R_PIG, dense=True, h=5e-9):
    """Fixed-step trapezoidal (A-stable) integration of the linear-in-state network; the MLCC
    capacitances are re-evaluated at the step midpoint (one corrector pass).
    extra_damp: None or (C, ESR) of an additional damping capacitor at node P (e.g. polymer)."""
    r_s = r_batt + r_pig
    esr_p = ESR22 / 2
    esr_d = ESR22 / 4
    esr_l = R_LDO_IN_ESR
    cd_extra = 4 * 100e-9 * 0.9
    gp = 1.0 / esr_p
    if extra_damp is None:
        cx, gx = 1.0, 0.0
    else:
        cx, gx = extra_damp[0], 1.0 / extra_damp[1]
    gs = gp + gx
    # vp = p . x  (states: i_s, vcp, i_pl, vcd, i_ldo, vcl, vcx)
    p = np.array([1 / gs, gp / gs, -1 / gs, 0, 0, 0, gx / gs])
    e = np.eye(7)
    vd_row = e[3] + esr_d * (e[2] - e[4])
    vl_row = e[5] + esr_l * e[4]
    icp_row = gp * (p - e[1])
    icx_row = gx * (p - e[6])
    base_rows = [
        (-r_s * e[0] - p) / l_s,                  # di_s/dt
        icp_row,                                  # dvcp/dt * C_P
        (p - R_PL * e[2] - vd_row) / L_PL,        # di_pl/dt
        e[2] - e[4],                              # dvcd/dt * C_D
        (vd_row - R_TR * e[4] - vl_row) / L_TR,   # di_ldo/dt
        e[4],                                     # dvcl/dt * C_L
        icx_row,                                  # dvcx/dt * C_x
    ]
    A0 = np.array(base_rows)
    b0 = np.zeros(7)
    b0[0] = v_bat / l_s

    def amat(x):
        cp = c_bank(x[1], 2, tol, v0)
        cd = c_bank(x[3], 4, tol, v0, cd_extra)
        cl = c_ldo_in(x[5])
        sc = np.array([1, 1 / cp, 1, 1 / cd, 1, 1 / cl, 1 / cx])
        return A0 * sc[:, None]

    n = int(round(t_end / h))
    x = np.zeros(7)
    I7 = np.eye(7)
    keep = max(1, n // 3000) if dense else 1
    ts, xs = [0.0], [x.copy()]
    pk_vl = pk_vd = pk_i = 0.0
    for k in range(n):
        A = amat(x)
        x_pred = np.linalg.solve(I7 - 0.5 * h * A, (I7 + 0.5 * h * A) @ x + h * b0)
        A = amat(0.5 * (x + x_pred))
        x = np.linalg.solve(I7 - 0.5 * h * A, (I7 + 0.5 * h * A) @ x + h * b0)
        vl = vl_row @ x
        vd = vd_row @ x
        pk_vl = max(pk_vl, vl)
        pk_vd = max(pk_vd, vd)
        pk_i = max(pk_i, x[0])
        if dense and (k + 1) % keep == 0:
            ts.append((k + 1) * h)
            xs.append(x.copy())
    X = np.array(xs).T
    return {"t": np.array(ts), "i_s": X[0], "vp": p @ X, "vd": vd_row @ X, "vl": vl_row @ X,
            "pk_vl": pk_vl, "pk_vd": pk_vd, "pk_i": pk_i}


def main():
    print("Study 1: battery hot-plug transient")
    res = {}
    # --- nominal and worst case within the stated assumption ranges ---
    cases = {
        "nominal": dict(r_batt=S.R_BATT, l_s=S.L_PIG, tol=1.0, v0=4.0),
        "worst_in_range": dict(r_batt=0.025, l_s=50e-9, tol=0.8, v0=3.2),
        "extended_low_R(10mOhm cell+conn)": dict(r_batt=0.010, l_s=50e-9, tol=0.8, v0=3.2),
    }
    waves = {}
    for name, kw in cases.items():
        r = simulate(v_bat=S.V_FULL, **kw)
        waves[name] = r
        res[name] = {"params": kw, "peak_V_LDO_in": r["pk_vl"], "peak_V_driver": r["pk_vd"],
                     "peak_inrush_A": r["pk_i"]}
        print(f"  {name:36s} Vpk(LDO)={r['pk_vl']:.3f} V  Vpk(DRV)={r['pk_vd']:.3f} V  Ipk={r['pk_i']:.1f} A")

    # --- sweep R_batt for several L at worst-case C ---
    r_list = np.array([0.0, 0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.040, 0.050, 0.060])
    l_list = [25e-9, 40e-9, 50e-9, 60e-9]
    sweep = {}
    for l_s in l_list:
        pk = []
        for rb in r_list:
            r = simulate(v_bat=S.V_FULL, r_batt=rb, l_s=l_s, tol=0.8, v0=3.2, dense=False, t_end=80e-6)
            pk.append(r["pk_vl"])
        sweep[l_s] = np.array(pk)
        print(f"  L={l_s*1e9:.0f} nH: " + " ".join(f"{p:.2f}" for p in pk))
    res["sweep_worstC"] = {"r_batt_ohm": r_list, "peak_V_LDO_in": {f"{k*1e9:.0f}nH": v for k, v in sweep.items()}}

    # sensitivity to charge voltage (4.45 V over-charged LiHV) at worst in-range
    r445 = simulate(v_bat=4.45, r_batt=0.025, l_s=50e-9, tol=0.8, v0=3.2)
    res["worst_in_range_4V45"] = {"peak_V_LDO_in": r445["pk_vl"]}
    print(f"  worst in range @4.45 V: {r445['pk_vl']:.3f} V")

    # optional damping network demonstration at extended low R
    rdamp = simulate(v_bat=S.V_FULL, r_batt=0.010, l_s=50e-9, tol=0.8, v0=3.2, extra_damp=(47e-6, 0.06))
    res["extended_low_R_with_47uF_60mOhm"] = {"peak_V_LDO_in": rdamp["pk_vl"]}
    print(f"  extended low R + 47uF/60mOhm polymer: {rdamp['pk_vl']:.3f} V")

    # analytic check: linear RLC with average large-signal C
    v = S.V_FULL
    vv = np.linspace(0, v, 400)
    c_avg_nom = np.trapezoid(6 * C22 * S.c22_retention(vv, 4.0), vv) / v
    z0 = math.sqrt(S.L_PIG / c_avg_nom)
    zeta = (S.R_BATT + S.R_PIG) / (2 * z0)
    ovs = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2)) if zeta < 1 else 0.0
    f0 = 1 / (2 * math.pi * math.sqrt(S.L_PIG * c_avg_nom))
    res["analytic_nominal"] = {"C_avg_large_signal_uF": c_avg_nom * 1e6, "Z0_mOhm": z0 * 1e3, "zeta": zeta,
                               "overshoot_pct": ovs * 100, "f0_kHz": f0 / 1e3}
    print(f"  analytic nominal: Cavg={c_avg_nom*1e6:.1f} uF Z0={z0*1e3:.1f} mOhm zeta={zeta:.2f} "
          f"overshoot={ovs*100:.1f}% f0={f0/1e3:.0f} kHz")
    # minimum total series R that keeps the peak below the ROC max (6.0 V) for 60 nH, worst C
    pk60 = sweep[60e-9]
    rr = r_list + S.R_PIG
    below = rr[pk60 < 6.0]
    res["min_total_R_for_6V0_at_60nH_mOhm"] = float(below.min() * 1e3) if below.size else None

    # regen pump-up energy estimate (battery disconnect while braking) - risk note
    w_ft = S.op_point(duty=1.0, soc_ocv=4.2)["w"]
    e_rotor = 0.5 * S.J_ROT * w_ft ** 2
    c_tot = 6 * C22 * 0.5
    e_10pct = 0.5 * S.J_ROT * (w_ft ** 2 - (0.9 * w_ft) ** 2)
    v_pump = math.sqrt(4.2 ** 2 + 2 * e_10pct / c_tot)
    res["regen_note"] = {"rotor_KE_J_fullthrottle": e_rotor, "E_10pct_slowdown_one_rotor_J": e_10pct,
                         "V_if_dumped_into_bulk_V": v_pump}
    print(f"  regen note: rotor KE {e_rotor*1e3:.0f} mJ; 10% slow-down of ONE rotor into ~66 uF -> {v_pump:.0f} V")

    # ---------------- plots ----------------
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0))
    ax = axs[0]
    for (name, r), col in zip(waves.items(), PAL):
        lab = {"nominal": "nominal (Rbatt 40 mOhm, 40 nH, C nom)",
               "worst_in_range": "worst in range (25 mOhm, 50 nH, C -20%/40%)",
               "extended_low_R(10mOhm cell+conn)": "beyond range (10 mOhm, 50 nH, C min)"}[name]
        ax.plot(r["t"] * 1e6, r["vl"], color=col, label=lab)
    S.hline_limit(ax, 6.5, "TPS7A20 VIN abs max 6.5 V")
    S.hline_limit(ax, 6.0, "TPS7A20 VIN ROC max 6.0 V", ls=":")
    ax.set_xlabel("time after contact (us)")
    ax.set_ylabel("V at TPS7A20 VIN (V)")
    ax.set_title("Hot-plug of a 4.35 V LiHV pack")
    ax.set_ylim(0, 7.2)
    ax.set_xlim(0, 60)
    ax.legend(loc="lower right")
    ax = axs[1]
    for (l_s, pk), col in zip(sweep.items(), PAL):
        ax.plot((r_list + S.R_PIG) * 1e3, pk, marker="o", ms=4, color=col, label=f"L = {l_s*1e9:.0f} nH")
    ax.axvspan((0.025 + S.R_PIG) * 1e3, (0.060 + S.R_PIG) * 1e3, color="#eeeeea", zorder=0)
    ax.text((0.042 + S.R_PIG) * 1e3, 8.45, "stated R range", color=INK2, fontsize=8, ha="center")
    S.hline_limit(ax, 6.5, "abs max 6.5 V")
    S.hline_limit(ax, 6.0, "ROC max 6.0 V", ls=":")
    ax.set_xlabel("total series R: cell+connector+pigtail (mOhm)")
    ax.set_ylabel("peak V at TPS7A20 VIN (V)")
    ax.set_title("Peak vs damping R (worst-case MLCC C: -20%, 40% @4 V)")
    ax.set_ylim(4.2, 8.8)
    ax.legend(loc="upper right")
    S.save(fig, "s1_hotplug.png")

    fig, ax = plt.subplots(figsize=(6, 3.4))
    r = waves["worst_in_range"]
    ax.plot(r["t"] * 1e6, r["i_s"], color=PAL[0])
    ax.set_xlabel("time after contact (us)")
    ax.set_ylabel("inrush current (A)")
    ax.set_title("Inrush through BT2.0 (worst in range)")
    S.save(fig, "s1_inrush.png")

    S.write_results("study1_hotplug", res)


if __name__ == "__main__":
    main()
