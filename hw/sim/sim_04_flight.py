"""
Study 4 - Flight performance: hover throttle/current, max thrust, thrust-to-weight, flight time
(260 / 300 mAh LiHV with sag model), motor mechanical time constant and thrust step response.

* Hover and full-throttle operating points come from the switching-level model (includes PWM
  ripple losses) at a table of battery OCVs; the discharge is integrated on that table.
* Battery: OCV(SoC) LiHV curve + R_batt(SoC) (+50% near empty) + pigtail, all assumptions in simlib.
* Step response: averaged electrical model (tau_e ~ 10 us << tau_m) + rotor inertia ODE,
  4 motors stepping together (battery sag included), optional firmware current limit.

Run:  python sim_04_flight.py
"""
import math
import simlib as S
import numpy as np
from scipy.integrate import solve_ivp
from simlib import plt, PAL, CRIT, INK2

RDS_HOVER = S.rds_of_T(80.0)      # warm board in hover (study 3, h~80)
RDS_FULL = S.rds_of_T(100.0)
OCVS = np.array([4.30, 4.10, 3.90, 3.75, 3.60, 3.45, 3.30])
MIX = S.asm("mixed_thrust_factor", 1.3, "mean thrust / hover thrust for a 'mixed' (sporty) flight profile")


def hover_table(auw_g, factor=1.0):
    t_m = auw_g * 1e-3 * S.G / (4 * S.OVERLAP) * factor
    rows = []
    for ocv in OCVS:
        avg, r = S.sim_at_thrust(t_m, soc_ocv=ocv, r_ds=RDS_HOVER, record=False)
        if r is None:
            rows.append((ocv, float("nan"), float("nan"), float("nan")))
            continue
        rows.append((ocv, r["duty"], r["i_bus_avg"], r["vm"]))
    return np.array(rows)


def full_table():
    rows = []
    for ocv in OCVS:
        avg, r = S.sim_full_throttle(soc_ocv=ocv, r_ds=RDS_FULL)
        rows.append((ocv, S.KT_P * r["w_m"] ** 2, r["i_bus_avg"], r["vm"], r["rpm"]))
    return np.array(rows)


def discharge(cap_mah, tab, dt=1.0):
    """integrate SoC with the per-motor bus-current table (vs OCV). returns t, soc, v_board."""
    soc = 1.0
    t = 0.0
    ts, socs, vb, ib = [], [], [], []
    ocv_t, i_t = tab[::-1, 0], tab[::-1, 2]
    while t < 3600:
        o = float(S.ocv(soc))
        i_mot = float(np.interp(o, ocv_t, i_t))
        i_tot = 4 * i_mot + S.I_HK
        v = o - (float(S.r_batt_soc(soc)) + S.R_PIG) * i_tot
        ts.append(t); socs.append(soc); vb.append(v); ib.append(i_tot)
        if v < S.V_LAND or soc <= 0.0 or not np.isfinite(i_mot):
            break
        soc -= i_tot * dt / 3600.0 / (cap_mah / 1000.0)
        t += dt
    return np.array(ts), np.array(socs), np.array(vb), np.array(ib)


def step_sim(d_from, d_to, ocv=3.95, j=S.J_ROT, i_lim=None, t_end=0.25, r_ds=RDS_HOVER):
    r_m = S.R_LL + S.R_EXT + r_ds
    r_path = S.R_BATT + S.R_PIG
    w0 = S.op_point(duty=d_from, soc_ocv=ocv, r_ds=r_ds)["w"]

    def elec(w, d):
        e = S.KE * w
        vm = (ocv - r_path * S.I_HK + 4 * r_path * d * e / r_m) / (1 + 4 * r_path * d * d / r_m)
        i = (d * vm - e) / r_m
        if i_lim is not None and i > i_lim:
            # firmware reduces duty so that I = i_lim:  D = (E + I R)/VM,
            # VM = a - 4 r_path D I  ->  VM^2 - a VM + 4 r_path I (E + I R) = 0
            i = i_lim
            a = ocv - r_path * S.I_HK
            vm = 0.5 * (a + math.sqrt(max(a * a - 16 * r_path * i * (e + i * r_m), 0.0)))
        return i, vm

    def f(t, y):
        w = y[0]
        i, _ = elec(w, d_to)
        return [(S.KT * i - S.KQ_P * w ** 2 - S.KT * S.I0 * np.sign(w)) / j]
    sol = solve_ivp(f, (0, t_end), [w0], max_step=2e-4, rtol=1e-8, atol=1e-6)
    w = sol.y[0]
    i = np.array([elec(x, d_to)[0] for x in w])
    vm = np.array([elec(x, d_to)[1] for x in w])
    return sol.t, w, S.KT_P * w ** 2, i, vm


def rise_metrics(t, y):
    y0, y1 = y[0], y[-1]
    def cross(frac):
        lvl = y0 + frac * (y1 - y0)
        k = np.flatnonzero((y - lvl) * np.sign(y1 - y0) >= 0)
        return t[k[0]] if k.size else float("nan")
    return {"t10": cross(0.1), "t63": cross(0.632), "t90": cross(0.9), "t10_90": cross(0.9) - cross(0.1)}


def main():
    print("Study 4: flight performance")
    res = {"rds_hover": RDS_HOVER, "rds_full": RDS_FULL}
    packs = {"260": S.M_DRY + 7.6, "300": S.M_DRY + 8.3}
    hov = {k: hover_table(m) for k, m in packs.items()}
    mixed = {k: hover_table(m, MIX) for k, m in packs.items()}
    full = full_table()
    res["full_throttle_vs_ocv"] = [{"ocv": r[0], "thrust_per_motor_g": r[1] / S.G * 1e3, "I_bus_A": r[2],
                                    "VM": r[3], "rpm": r[4]} for r in full]
    for k, auw in packs.items():
        tab = hov[k]
        tmax_vehicle = 4 * full[:, 1] * S.OVERLAP / S.G * 1e3
        twr = tmax_vehicle / auw
        t, soc, vb, ib = discharge(S.CAP_MAH[k], tab)
        tm, socm, vbm, ibm = discharge(S.CAP_MAH[k], mixed[k])
        res[f"pack_{k}"] = {
            "AUW_g": auw, "hover_thrust_per_motor_isolated_g": auw / (4 * S.OVERLAP),
            "hover_vs_ocv": [{"ocv": r[0], "duty": r[1], "I_bus_motor_A": r[2], "VM": r[3],
                              "I_batt_total_A": 4 * r[2] + S.I_HK} for r in tab],
            "TWR_vs_ocv": {f"{o:.2f}": v for o, v in zip(OCVS, twr)},
            "flight_time_hover_min": t[-1] / 60, "soc_at_land_hover": soc[-1],
            "flight_time_mixed_min": tm[-1] / 60, "soc_at_land_mixed": socm[-1],
            "mean_batt_current_hover_A": float(ib.mean()), "mean_batt_current_mixed_A": float(ibm.mean()),
        }
        print(f"  {k} mAh: AUW {auw:.1f} g, hover D {tab[2,1]:.2f}@3.90V, I_motor {tab[2,2]:.2f} A, "
              f"TWR {twr[0]:.2f}(4.30V) .. {twr[-2]:.2f}(3.45V), flight hover {t[-1]/60:.1f} min, "
              f"mixed {tm[-1]/60:.1f} min")
        res[f"pack_{k}"]["_curves"] = {"t": t, "v": vb, "tm": tm, "vm": vbm}
    # sensitivity of hover current/time to inductance (at OCV 3.90 V, 260 pack)
    sens = {}
    t_m = packs["260"] * 1e-3 * S.G / (4 * S.OVERLAP)
    for l_ll in (3e-6, 6e-6, 15e-6):
        for fp in (48e3, 96e3):
            avg, r = S.sim_at_thrust(t_m, soc_ocv=3.90, r_ds=RDS_HOVER, l_ll=l_ll, f_pwm=fp, record=False)
            sens[f"L{l_ll*1e6:.0f}uH_{fp/1e3:.0f}kHz"] = {"I_bus_motor_A": r["i_bus_avg"],
                                                         "hover_power_W": r["vm"] * (4 * r["i_bus_avg"] + S.I_HK)}
    res["hover_sensitivity_OCV3.90_260"] = sens
    print("  hover I_bus sensitivity:", {k: round(v["I_bus_motor_A"], 3) for k, v in sens.items()})

    # ---- motor time constant and step response ----
    d_h = S.op_point(thrust=t_m, soc_ocv=3.95, r_ds=RDS_HOVER)["duty"]
    op_h = S.op_point(duty=d_h, soc_ocv=3.95, r_ds=RDS_HOVER)
    w0 = op_h["w"]
    r_m = S.R_LL + S.R_EXT + RDS_HOVER
    tau_lin = S.J_ROT / (2 * S.KQ_P * w0 + S.KT * S.KE / r_m)
    tau_rng = [j / (2 * S.KQ_P * w0 + S.KT * S.KE / r_m) for j in S.ASM["j_rot"]["range"]]
    d_12 = S.op_point(thrust=1.2 * t_m, soc_ocv=3.95, r_ds=RDS_HOVER)["duty"]
    rds_cold = S.rds_of_T(25.0)
    d_h435 = S.op_point(thrust=t_m, soc_ocv=4.35, r_ds=rds_cold)["duty"]
    steps = {
        "hover->1.2x thrust": step_sim(d_h, d_12),
        "hover->full (D=1)": step_sim(d_h, 1.0),
        "hover->full, 4.35 V, cold RDS": step_sim(d_h435, 1.0, ocv=4.35, r_ds=rds_cold),
        "same + 3.5 A firmware limit": step_sim(d_h435, 1.0, ocv=4.35, r_ds=rds_cold, i_lim=3.5),
        "full->hover duty (decel)": step_sim(1.0, d_h),
    }
    st = {}
    for name, (t, w, thr, i, vm) in steps.items():
        m = rise_metrics(t, thr)
        st[name] = {**m, "I_peak_A": float(i.max()), "I_min_A": float(i.min()), "VM_min_V": float(vm.min()),
                    "VM_max_V": float(vm.max()), "thrust_start_g": thr[0] / S.G * 1e3,
                    "thrust_end_g": thr[-1] / S.G * 1e3}
        print(f"  step {name:26s} t10-90 {m['t10_90']*1e3:6.1f} ms  t63 {m['t63']*1e3:6.1f} ms  "
              f"Ipk {i.max():.2f} A  Imin {i.min():.2f} A  VMmin {vm.min():.2f} V")
    res["motor_time_constant"] = {"tau_linear_hover_ms": tau_lin * 1e3,
                                  "tau_range_ms_for_J_range": [x * 1e3 for x in tau_rng],
                                  "J_kgm2": S.J_ROT, "w0_rad_s": w0}
    res["steps"] = st
    print(f"  linearised motor/prop time constant at hover: {tau_lin*1e3:.1f} ms "
          f"(J range -> {tau_rng[0]*1e3:.1f}..{tau_rng[1]*1e3:.1f} ms)")

    # ================= plots =================
    fig, axs = plt.subplots(2, 2, figsize=(11, 7.4))
    ax = axs[0, 0]
    for (k, tab), col in zip(hov.items(), PAL):
        ax.plot(tab[:, 0], tab[:, 1], marker="o", ms=4, color=col, label=f"{k} mAh (AUW {packs[k]:.1f} g)")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("hover duty")
    ax.set_title("Hover throttle (duty) vs battery state")
    ax.invert_xaxis()
    ax.legend()
    ax = axs[0, 1]
    for (k, tab), col in zip(hov.items(), PAL):
        ax.plot(tab[:, 0], 4 * tab[:, 2] + S.I_HK, marker="o", ms=4, color=col, label=f"{k} mAh")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("hover battery current (A)")
    ax.set_title("Hover battery current (4 ESC + electronics)")
    ax.invert_xaxis()
    ax.legend()
    ax = axs[1, 0]
    ax.plot(full[:, 0], 4 * full[:, 1] * S.OVERLAP / S.G * 1e3, marker="o", ms=4, color=PAL[2],
            label="DRV8311 (RDS @100 C), overlap -10%")
    ax.axhline(4 * 32.0 * S.OVERLAP, color=INK2, ls=":", lw=1)
    ax.text(4.28, 4 * 32.0 * S.OVERLAP, "vendor 4x32 g x0.9 (low-R ESC, ~4.2 V)", fontsize=7.5, color=INK2,
            va="bottom")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("max vehicle thrust (g)")
    ax.set_title("Maximum thrust (full throttle, with sag)")
    ax.invert_xaxis()
    ax.set_ylim(0, 125)
    ax.legend(loc="lower left")
    ax = axs[1, 1]
    for (k, auw), col in zip(packs.items(), PAL):
        ax.plot(full[:, 0], 4 * full[:, 1] * S.OVERLAP / S.G * 1e3 / auw, marker="o", ms=4, color=col,
                label=f"{k} mAh")
    S.hline_limit(ax, 2.0, "TWR 2 (minimum for sporty flight)", ls=":")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("thrust-to-weight")
    ax.set_title("Thrust-to-weight vs battery state")
    ax.invert_xaxis()
    ax.legend()
    fig.tight_layout()
    S.save(fig, "s4_hover_thrust_twr.png")

    fig, ax = plt.subplots(figsize=(8.5, 4.2))
    ci = 0
    for k in packs:
        c = res[f"pack_{k}"]["_curves"]
        ax.plot(c["t"] / 60, c["v"], color=PAL[ci], label=f"{k} mAh hover ({c['t'][-1]/60:.1f} min)")
        ax.plot(c["tm"] / 60, c["vm"], color=PAL[ci], ls="--", label=f"{k} mAh mixed x{MIX} ({c['tm'][-1]/60:.1f} min)")
        ci += 1
    S.hline_limit(ax, S.V_LAND, f"landing threshold {S.V_LAND} V (loaded)")
    ax.set_xlabel("flight time (min)")
    ax.set_ylabel("loaded battery voltage at board (V)")
    ax.set_title("Discharge at hover and mixed flying")
    ax.legend(loc="upper right")
    S.save(fig, "s4_flight_time.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0))
    for (name, (t, w, thr, i, vm)), col in zip(steps.items(), PAL):
        axs[0].plot(t * 1e3, thr / S.G * 1e3, color=col, label=name)
        axs[1].plot(t * 1e3, i, color=col, label=name)
    axs[0].set_xlabel("time (ms)")
    axs[0].set_ylabel("thrust per motor, isolated (g)")
    axs[0].set_title(f"Thrust step response (J = {S.J_ROT:.1e} kg m2, OCV 3.95 V)")
    axs[0].legend(loc="center right", fontsize=7.5)
    S.hline_limit(axs[1], 5.0, "IOUT peak 5 A")
    S.hline_limit(axs[1], 5.8, "OCP min 5.8 A", ls=":")
    axs[1].set_xlabel("time (ms)")
    axs[1].set_ylabel("motor current, DC-equivalent (A)")
    axs[1].set_title("Motor current during the step")
    axs[1].set_ylim(-3, 6.5)
    fig.tight_layout()
    S.save(fig, "s4_step_response.png")

    for k in packs:
        res[f"pack_{k}"].pop("_curves")
    S.write_results("study4_flight", res)


if __name__ == "__main__":
    main()
