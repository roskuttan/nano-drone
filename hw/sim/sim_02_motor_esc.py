"""
Study 2 - Motor + ESC electrical model (BetaFPV 0702SE II 23000KV on DRV8311H, 1S).

Switching-level simulation (simlib.sim_6step): star motor with sinusoidal BEMF, 6-step
(120 deg) commutation with timing advance, DRV8311 3x-PWM mode (complementary PWM with handshake
dead time on the PWM phase, LS ON on the low phase, Hi-Z + body-diode clamping on the floating
phase), 48 kHz edge-aligned PWM, constant rotor speed per operating point (inertia >> tau_e).
Duty is solved so that mean EM torque = prop torque kQ*w^2 + friction Kt*I0 (prop fitted in
simlib.calibrate_prop).  VM includes battery sag (4 identical motors).

Run:  python sim_02_motor_esc.py
"""
import math
import simlib as S  # first: sets sys.path for .pylibs
import numpy as np
from simlib import plt, PAL, CRIT, INK2

OCV_NOM = 3.85          # mid-SoC open-circuit voltage
RDS = S.ds("drv_rds_25_typ")
ADC_LSB = 3.0 / 4096    # VDDA = VREF+ = 3.0 V (G071 UFQFPN28: VREF+ bonded to VDDA - assumption)
DIV = 0.5               # 10k/10k BEMF divider


def ripple_pp(w):
    """mean pk-pk of phase-A current over PWM periods in the middle of the windows where A is the
    continuously-ON low-side phase (sectors 3,4)."""
    t, ia, sect = w["t"], w["ia"], w["sect"]
    mask = (sect == 3) | (sect == 4)
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return float("nan")
    # split contiguous runs
    breaks = np.flatnonzero(np.diff(idx) > 1)
    runs = np.split(idx, breaks + 1)
    pps = []
    tp = 1.0 / S.F_PWM
    for r in runs:
        if r.size < 10:
            continue
        a, b = r[0], r[-1]
        span = t[b] - t[a]
        sel = r[(t[r] > t[a] + 0.2 * span) & (t[r] < t[b] - 0.2 * span)]
        if sel.size < 5:
            continue
        k = ((t[sel] - t[sel[0]]) // tp).astype(int)
        for kk in np.unique(k):
            s = sel[k == kk]
            if s.size > 5:
                pps.append(ia[s].max() - ia[s].min())
    return float(np.median(pps)) if pps else float("nan")


def locked_rotor_ipk(vm, duty, l_ll, r_tot, f_pwm=S.F_PWM):
    """steady-state peak current at standstill (E=0), synchronous rectification."""
    tau = l_ll / r_tot
    T = 1 / f_pwm
    return vm / r_tot * (1 - math.exp(-duty * T / tau)) / (1 - math.exp(-T / tau))


def summarize(name, avg, r, t_thrust):
    fe = r["f_e"]
    f_comm = r["f_comm"]
    step_t = 1.0 / f_comm
    t_zc = (30.0 + S.ADVANCE) / 60.0 * step_t      # ZC position after commutation
    epk = r["epk"]
    swing_adc = 0.75 * epk * DIV                      # +/- around VM/4 (floating phase, PWM-on)
    slope_adc = 0.75 * epk * (2 * math.pi * fe) * DIV  # V/s at ZC
    d = {
        "case": name,
        "thrust_g_isolated": t_thrust / S.G * 1e3,
        "duty": r["duty"], "VM_V": r["vm"], "rpm": r["rpm"], "f_e_Hz": fe, "f_comm_Hz": f_comm,
        "pwm_periods_per_step": r["f_pwm"] / f_comm,
        "I_torque_equiv_A": r["torque"] / S.KT,
        "I_bus_avg_A": r["i_bus_avg"], "I_bus_rms_A": r["i_bus_rms"],
        "I_phase_rms_A": r["i_ph_rms"], "I_phase_pk_A": r["i_ph_pk"],
        "ripple_pp_A": ripple_pp(r["wave"]) if "wave" in r else None,
        "E_phase_pk_V": epk, "bemf_swing_at_adc_V(+/-)": swing_adc,
        "bemf_swing_at_adc_LSB(+/-)": swing_adc / ADC_LSB,
        "zc_slope_at_adc_V_per_us": slope_adc * 1e-6,
        "zc_slope_LSB_per_pwm_period": slope_adc / r["f_pwm"] / ADC_LSB,
        "demag_max_us": r["demag_max"] * 1e6, "zc_after_commutation_us": t_zc * 1e6,
        "P_cond_W": r["p_cond"], "P_diode_W": r["p_diode"], "P_sw_W": r["p_sw"],
        "OCP_margin_vs_5.8A_min_A": S.ds("drv_ocp")[0] - r["i_ph_pk"],
        "margin_vs_5A_abs_max_A": S.ds("drv_iout_peak") - r["i_ph_pk"],
        "L_LL_uH": r["l_ll"] * 1e6, "f_pwm_kHz": r["f_pwm"] / 1e3,
    }
    return d


def main():
    print("Study 2: motor + ESC electrical model")
    print(f"  prop fit GF1219S-3: kT={S.KT_P:.3e} N s^2  kQ={S.KQ_P:.3e} N m s^2  "
          f"(I@32g on ref ESC {S.I_32G_REF:.2f} A, {S.RPM_32G_REF:.0f} rpm, FM {S.FM_P:.2f})")
    res = {"prop_fit": {"kT": S.KT_P, "kQ": S.KQ_P, "I_at_32g_refESC_A": S.I_32G_REF,
                        "rpm_at_32g_refESC": S.RPM_32G_REF, "figure_of_merit": S.FM_P,
                        "QT_2blade_m": S.QT_2B, "s_3blade": S.S_3BLADE},
           "points": []}
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    t_hover = auw / (4 * S.OVERLAP)
    t_mid = 10e-3 * S.G
    waves = {}

    # ---- hover & mid-throttle points, L sweep at 48 kHz and 96 kHz ----
    l_list = [3e-6, 6e-6, 10e-6, 15e-6]
    sens = {}
    for (lab, thr) in (("hover", t_hover), ("10g", t_mid)):
        for fp in (48e3, 96e3):
            for l_ll in l_list:
                avg, r = S.sim_at_thrust(thr, soc_ocv=OCV_NOM, r_ds=RDS, l_ll=l_ll, f_pwm=fp, record=True)
                d = summarize(f"{lab} L={l_ll*1e6:.0f}uH {fp/1e3:.0f}kHz", avg, r, thr)
                res["points"].append(d)
                sens[(lab, fp, l_ll)] = d
                if fp == 48e3 and l_ll in (3e-6, 6e-6, 15e-6):
                    waves[(lab, l_ll)] = r
                print(f"  {d['case']:24s} D={d['duty']:.3f} rpm={d['rpm']:.0f} Ipk={d['I_phase_pk_A']:.2f} "
                      f"Irms={d['I_phase_rms_A']:.2f} rip={d['ripple_pp_A']:.2f} Ibus={d['I_bus_avg_A']:.2f} "
                      f"Pdrv={r['p_drv_total']:.3f}")
    # ---- full throttle ----
    full = {}
    for ocv in (OCV_NOM, S.V_FULL):
        for l_ll in l_list:
            avg, r = S.sim_full_throttle(soc_ocv=ocv, r_ds=RDS, l_ll=l_ll)
            thr = S.KT_P * r["w_m"] ** 2
            d = summarize(f"full OCV={ocv:.2f} L={l_ll*1e6:.0f}uH", avg, r, thr)
            res["points"].append(d)
            full[(ocv, l_ll)] = d
            if ocv == OCV_NOM and l_ll == 6e-6:
                waves[("full", l_ll)] = r
            print(f"  {d['case']:24s} rpm={d['rpm']:.0f} T={d['thrust_g_isolated']:.1f} g Ipk={d['I_phase_pk_A']:.2f} "
                  f"Irms={d['I_phase_rms_A']:.2f} Ibus={d['I_bus_avg_A']:.2f} demag={d['demag_max_us']:.1f}us "
                  f"zc@{d['zc_after_commutation_us']:.1f}us")

    # ---- locked rotor / start-up peak current vs duty ----
    r_tot = S.R_LL + S.R_EXT + RDS
    duties = np.linspace(0.02, 1.0, 99)
    lr = {}
    for l_ll in l_list:
        lr[l_ll] = np.array([locked_rotor_ipk(S.V_FULL, dd, l_ll, r_tot) for dd in duties])
    dmax = {f"{l*1e6:.0f}uH": float(duties[np.flatnonzero(lr[l] < 5.0)[-1]]) for l in l_list}
    res["locked_rotor"] = {"VM": S.V_FULL, "R_tot": r_tot, "I_stall_100pct_A": S.V_FULL / r_tot,
                           "max_duty_for_Ipk_below_5A": dmax}
    print(f"  locked-rotor: I(D=1) = {S.V_FULL / r_tot:.1f} A; max start duty for Ipk<5 A: {dmax}")

    # ---- punch-out (hover -> D=1 instantaneous) peak current, avg model ----
    hov = S.op_point(thrust=t_hover, soc_ocv=S.V_FULL, r_ds=RDS)
    e_h = S.KE * hov["w"]
    r_path = S.R_BATT + S.R_PIG
    # VM sags with 4 motors: VM = OCV - r_path*(4 I + I_hk); I = (VM - E)/r_tot
    i_punch = (S.V_FULL - r_path * S.I_HK - e_h) / (r_tot + 4 * r_path)
    res["punch_out_initial_I_A"] = {"OCV": S.V_FULL, "I_dc": i_punch, "note": "plus PWM-free; ripple 0 at D=1"}
    print(f"  punch-out from hover at 4.35 V: initial I = {i_punch:.2f} A per motor")

    # ---- BEMF / sampling vs speed (analytic, from model constants) ----
    rpm = np.linspace(1000, 60000, 200)
    wm = rpm * 2 * math.pi / 60
    epk = S.BEMF_K * S.KE * wm
    fe = S.POLE_PAIRS * wm / (2 * math.pi)
    swing_lsb = 0.75 * epk * DIV / ADC_LSB
    per_step = {fp: fp / (6 * fe) for fp in (48e3, 96e3)}
    off_err_deg = np.degrees(np.arcsin(np.clip(0.010 / (0.75 * epk * DIV), 0, 1)))  # 10 mV offset
    res["bemf_vs_speed"] = {"rpm_for_100LSB_swing": float(rpm[np.argmax(swing_lsb > 100)]),
                            "E_pk_per_krpm_V": float(S.BEMF_K * S.KE * 1000 * 2 * math.pi / 60)}

    # ================= plots =================
    # 1) hover waveforms
    r = waves[("hover", 6e-6)]
    w = r["wave"]
    te = 1 / r["f_e"]
    t0 = w["t"][-1] - 1.0 * te   # last (settled) electrical cycle
    m = w["t"] >= t0
    fig, axs = plt.subplots(3, 1, figsize=(10, 9.2))
    ax = axs[0]
    for key, col, lab in (("ia", PAL[0], "phase A"), ("ib", PAL[1], "phase B"), ("ic", PAL[2], "phase C")):
        ax.plot((w["t"][m] - t0) * 1e6, w[key][m], color=col, lw=0.8, label=lab)
    ax.set_ylabel("phase current (A)")
    ax.set_title(f"Hover, L_LL = 6 uH, 48 kHz, VM = {r['vm']:.2f} V, D = {r['duty']:.2f}, "
                 f"{r['rpm']:.0f} rpm (f_e = {r['f_e']:.0f} Hz)")
    ax.legend(loc="upper right", ncol=3)
    ax.set_xlabel("time (us)")
    ax = axs[1]
    for (l_ll, col) in ((3e-6, PAL[0]), (6e-6, PAL[1]), (15e-6, PAL[2])):
        ww = waves[("hover", l_ll)]["wave"]
        sect = ww["sect"]
        idx = np.flatnonzero(sect == 3)
        if idx.size == 0:
            continue
        a = idx[len(idx) // 2]
        tz = ww["t"][a]
        mm = (ww["t"] >= tz - 1.5 / 48e3) & (ww["t"] <= tz + 2.5 / 48e3)
        ax.plot((ww["t"][mm] - tz) * 1e6, -ww["ia"][mm], color=col, lw=1.2,
                label=f"L_LL = {l_ll*1e6:.0f} uH")
    ax.set_xlabel("time (us), zoom on 4 PWM periods (low-side phase A, sign inverted)")
    ax.set_ylabel("|i_A| (A)")
    ax.set_title("PWM current ripple vs motor inductance (hover)")
    ax.legend(loc="upper right", ncol=3)
    ax = axs[2]
    va_adc = w["va"][m] * DIV
    ax.plot((w["t"][m] - t0) * 1e6, va_adc, color=PAL[0], lw=0.6, label="phase A at ADC (10k/10k)")
    ea_adc = (w["ea"][m] * 1.5 + 0.0) * DIV
    # mid-on-time samples during floating windows of A (sectors 2 and 5)
    tp = 1 / r["f_pwm"]
    tt = w["t"][m] - t0
    k = np.floor((w["t"][m]) / tp)
    tsamp = (k + 0.5 * r["duty"]) * tp
    flo = (w["sect"][m] == 2) | (w["sect"][m] == 5)
    near = np.abs(w["t"][m] - tsamp) < 0.5 * 25e-9 + 1e-12
    sel = flo & near
    ax.plot(tt[sel] * 1e6, va_adc[sel], "o", ms=5, color=PAL[1], label="PWM-on mid-point samples (floating)")
    ax.axhline(r["vm"] / 2 * DIV, color=INK2, lw=0.8, ls=":")
    ax.text(0.995, r["vm"] / 2 * DIV, "VM/2 x 0.5 (ZC threshold) ", transform=ax.get_yaxis_transform(),
            ha="right", va="bottom", fontsize=7.5, color=INK2)
    ax.set_xlabel("time (us)")
    ax.set_ylabel("ADC pin voltage (V)")
    ax.set_title("BEMF sensing: phase A terminal voltage after 10k/10k divider")
    ax.legend(loc="upper right")
    fig.tight_layout()
    S.save(fig, "s2_hover_waveforms.png")

    # 2) full-throttle waveforms
    r = waves[("full", 6e-6)]
    w = r["wave"]
    te = 1 / r["f_e"]
    t0 = w["t"][-1] - 1.0 * te   # last (settled) electrical cycle
    m = w["t"] >= t0
    fig, axs = plt.subplots(2, 1, figsize=(10, 6.4))
    ax = axs[0]
    for key, col, lab in (("ia", PAL[0], "phase A"), ("ib", PAL[1], "phase B"), ("ic", PAL[2], "phase C")):
        ax.plot((w["t"][m] - t0) * 1e6, w[key][m], color=col, lw=1.0, label=lab)
    S.hline_limit(ax, 5.0, "DRV8311 peak IOUT 5 A (abs max)")
    S.hline_limit(ax, -5.0, "", va="top")
    ax.set_ylim(-6, 6)
    ax.set_ylabel("phase current (A)")
    ax.set_xlabel("time (us)")
    ax.set_title(f"Full throttle (D = 1), L_LL = 6 uH, OCV {OCV_NOM} V -> VM = {r['vm']:.2f} V, "
                 f"{r['rpm']:.0f} rpm, f_e = {r['f_e']:.0f} Hz")
    ax.legend(loc="lower right", ncol=3)
    ax = axs[1]
    ax.plot((w["t"][m] - t0) * 1e6, w["va"][m] * DIV, color=PAL[0], lw=0.9, label="phase A at ADC")
    ax.axhline(r["vm"] / 2 * DIV, color=INK2, lw=0.8, ls=":")
    ax.set_xlabel("time (us)")
    ax.set_ylabel("ADC pin voltage (V)")
    ax.set_title(f"Floating-phase BEMF at full throttle: demag. clamp {r['demag_max']*1e6:.1f} us after "
                 f"commutation, ZC expected {(30+S.ADVANCE)/60/r['f_comm']*1e6:.1f} us after")
    ax.legend(loc="upper right")
    fig.tight_layout()
    S.save(fig, "s2_full_waveforms.png")

    # 3) sensitivity to inductance
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    ax = axs[0]
    li = np.array(l_list) * 1e6
    ci = 0
    for lab in ("hover", "10g"):
        for fp, ls in ((48e3, "-"), (96e3, "--")):
            ax.plot(li, [sens[(lab, fp, l)]["I_phase_pk_A"] for l in l_list], ls, marker="o", ms=4,
                    color=PAL[0 if lab == "hover" else 1],
                    label=f"{'hover' if lab == 'hover' else '10 g/motor'} @ {fp/1e3:.0f} kHz")
    ax.plot(li, [full[(S.V_FULL, l)]["I_phase_pk_A"] for l in l_list], marker="s", ms=4, color=PAL[2],
            label="full throttle @ 4.35 V OCV")
    S.hline_limit(ax, 5.0, "IOUT peak 5 A (abs max / ROC)")
    S.hline_limit(ax, 5.8, "OCP min 5.8 A", ls=":")
    ax.set_ylim(0, 6.5)
    ax.set_xlabel("motor inductance L_LL (uH)")
    ax.set_ylabel("peak phase current (A)")
    ax.set_title("Peak phase current vs inductance")
    ax.legend(loc="lower left", fontsize=7.5)
    ax = axs[1]
    for lab in ("hover", "10g"):
        for fp, ls in ((48e3, "-"), (96e3, "--")):
            ax.plot(li, [sens[(lab, fp, l)]["P_cond_W"] + sens[(lab, fp, l)]["P_diode_W"] +
                         sens[(lab, fp, l)]["P_sw_W"] for l in l_list], ls, marker="o", ms=4,
                    color=PAL[0 if lab == "hover" else 1],
                    label=f"{'hover' if lab == 'hover' else '10 g/motor'} @ {fp/1e3:.0f} kHz")
    ax.set_xlabel("motor inductance L_LL (uH)")
    ax.set_ylabel("DRV8311 loss per driver (W), RDS 25 C typ")
    ax.set_title("Driver loss (cond. + diode + switching) vs inductance")
    ax.legend(loc="upper right", fontsize=7.5)
    fig.tight_layout()
    S.save(fig, "s2_inductance_sensitivity.png")

    # 4) BEMF visibility vs speed + locked rotor current
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0))
    ax = axs[0]
    ax.plot(rpm / 1e3, swing_lsb, color=PAL[0], label="floating-phase BEMF swing at ADC (+/- LSB)")
    ax.set_yscale("log")
    ax.set_xlabel("motor speed (krpm)")
    ax.set_ylabel("ADC counts (12-bit, 3.0 V ref)")
    ax.set_title("BEMF visibility at the ADC (10k/10k divider)")
    ax2y = [per_step[48e3], per_step[96e3]]
    ax.plot(rpm / 1e3, off_err_deg, color=PAL[1], label="ZC angle error from 10 mV offset (deg el.)")
    ax.plot(rpm / 1e3, per_step[48e3], color=PAL[2], label="PWM periods per 60-deg step @48 kHz")
    ax.plot(rpm / 1e3, per_step[96e3], color=PAL[3], ls="--", label="PWM periods per step @96 kHz")
    ax.axvline(waves[("hover", 6e-6)]["rpm"] / 1e3, color=INK2, lw=0.8, ls=":")
    ax.text(waves[("hover", 6e-6)]["rpm"] / 1e3, 0.15, " hover", color=INK2, fontsize=8)
    ax.axvline(full[(OCV_NOM, 6e-6)]["rpm"] / 1e3, color=INK2, lw=0.8, ls=":")
    ax.text(full[(OCV_NOM, 6e-6)]["rpm"] / 1e3, 0.15, " full", color=INK2, fontsize=8)
    ax.set_ylim(0.1, 3000)
    ax.legend(loc="upper center", fontsize=7.2)
    ax = axs[1]
    for l_ll, col in zip(l_list, PAL):
        ax.plot(duties, lr[l_ll], color=col, label=f"L_LL = {l_ll*1e6:.0f} uH")
    S.hline_limit(ax, 5.0, "IOUT peak 5 A")
    S.hline_limit(ax, 5.8, "OCP min 5.8 A", ls=":")
    ax.set_xlabel("duty at standstill / start-up")
    ax.set_ylabel("peak phase current (A)")
    ax.set_title(f"Locked-rotor current at VM = {S.V_FULL} V (R = {r_tot:.2f} ohm)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    S.save(fig, "s2_bemf_and_startup.png")

    S.write_results("study2_motor_esc", res)


if __name__ == "__main__":
    main()
