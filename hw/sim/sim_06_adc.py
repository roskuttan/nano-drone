"""
Study 6 - ADC front ends on the STM32G071 ESC MCUs (VDDA = VREF+ = 3V0).

(a) BEMF divider 10k/10k (5 kOhm Thevenin) -> ADC: required sampling time for 12/10-bit from
    DS12232 Table 57 (max RAIN vs ts) and from a charge-sharing + RC-recovery model; effect of an
    external pin capacitor; settling after PWM edges and minimum on-time for on-time sampling.
(b) DRV8311 CSA output (SOx) -> ADC: direct vs series-R + C reservoir; resolution, offset, range.
(c) VBAT divider 10k/10k + 100 nF: kick-back, sampling-rate bias, filtering of the 48 kHz ripple,
    response time, accuracy budget.

Run:  python sim_06_adc.py
"""
import math
import simlib as S
import numpy as np
from simlib import plt, PAL, CRIT, INK2

VREF = 3.0
F_ADC = S.ds("g071_fadc")
C_ADC = S.ds("g071_cadc")
C_PIN = S.ds("g071_cio")
C_PCB = S.asm("c_pcb_adc", 2e-12, "PCB trace/pad capacitance at an ADC pin")
LSB12 = VREF / 4096
TS_OPTS = [1.5, 3.5, 7.5, 12.5, 19.5, 39.5, 79.5, 160.5]
CSA_TAU = S.asm("csa_tau", 0.22e-6, "DRV8311 CSA small-signal time constant from tSET 1 us to 1% (=4.6 tau)")


def table57(bits):
    rows = S.ds("g071_rain_max")[bits]
    return np.array([r[1] for r in rows]), np.array([r[2] for r in rows]), np.array([r[0] for r in rows])


def min_ts_from_table(r_src, bits):
    ts, rmax, cyc = table57(bits)
    ok = np.flatnonzero(rmax >= r_src)
    return (cyc[ok[0]], ts[ok[0]]) if ok.size else (None, None)


def residual_lsb(ts, r_src, c_ext, dv=VREF, bits=12, extra_tau=0.0):
    """charge sharing CADC <-> node cap, then exponential recovery through r_src."""
    c_node = C_PIN + C_PCB + c_ext
    v0 = dv * C_ADC / (C_ADC + c_node)
    tau = r_src * (C_ADC + c_node) + extra_tau
    lsb = VREF / 2 ** bits
    return v0 * np.exp(-ts / tau) / lsb


def main():
    print("Study 6: ADC front ends (STM32G071)")
    res = {}
    # ---------------- (a) BEMF divider ----------------
    r_src = 10e3 * 10e3 / 20e3
    a = {}
    for bits in (12, 10):
        cyc, ts = min_ts_from_table(r_src, bits)
        a[f"{bits}bit_table57_min_ts"] = {"cycles": cyc, "ts_ns": ts * 1e9,
                                          "t_conv_us": (cyc + bits + 0.5) / F_ADC * 1e6}
        print(f"  BEMF 5k source, {bits}-bit: Table 57 -> ts >= {cyc} cycles ({ts*1e9:.0f} ns), "
              f"conversion {(cyc + bits + 0.5)/F_ADC*1e6:.2f} us")
    tsv = np.linspace(20e-9, 20e-6, 4000)
    model = {}
    for c_ext in (0.0, 100e-12, 1e-9):
        r = residual_lsb(tsv, r_src, c_ext)
        k = np.flatnonzero(r < 0.5)
        model[c_ext] = (r, tsv[k[0]] if k.size else None)
        a[f"model_ts_for_0.5LSB_Cext_{c_ext*1e12:.0f}pF_ns"] = (tsv[k[0]] * 1e9) if k.size else None
        print(f"  model C_ext={c_ext*1e12:6.0f} pF: 12-bit 0.5 LSB after "
              f"{(tsv[k[0]]*1e9 if k.size else float('nan')):.0f} ns (full-scale channel-to-channel step)")
    # settling of the divider node after a PWM edge (no external cap)
    tau_node = r_src * (C_PIN + C_PCB)
    t_settle = tau_node * math.log(VREF / 2 / (0.5 * LSB12))
    tpd = S.ds("drv_tpd_gnd")[1]
    t_slew = 4.35 / S.ds("drv_sr_gnd")[0]
    t_ring = S.asm("t_ring", 0.2e-6, "post-edge ringing allowance")
    cyc12, ts12 = min_ts_from_table(r_src, 12)
    t_first = tpd + t_slew + t_ring + t_settle
    t_need = t_first + ts12
    a["divider_node_tau_ns"] = tau_node * 1e9
    a["min_on_time_for_on_sampling_us"] = t_need * 1e6
    a["min_duty_48kHz"] = t_need * 48e3
    a["min_duty_96kHz"] = t_need * 96e3
    print(f"  earliest valid BEMF sample {t_first*1e6:.2f} us after PWM input edge "
          f"(tPD max {tpd*1e6:.1f} + slew {t_slew*1e6:.2f} + ring {t_ring*1e6:.1f} + settle {t_settle*1e6:.2f});"
          f" with ts: {t_need*1e6:.2f} us -> min duty {t_need*48e3*100:.0f}% @48 kHz, {t_need*96e3*100:.0f}% @96 kHz")
    vmax_pin = (S.V_FULL + S.VF_BODY) / 2
    vmin_pin = -S.VF_BODY / 2
    a["pin_range_V"] = [vmin_pin, vmax_pin]
    a["pin_max_at_hotplug_peak_V"] = (5.03 + S.VF_BODY) / 2
    a["neg_injection_uA_est"] = max(0.0, (S.VF_BODY / 2 - 0.3)) / r_src * 1e6
    # alternative divider 4.7k/4.7k
    cyc_alt, ts_alt = min_ts_from_table(2350, 12)
    a["alt_4k7_4k7_12bit_min_ts"] = {"cycles": cyc_alt, "ts_ns": ts_alt * 1e9}
    res["a_bemf"] = a
    # ---------------- (b) CSA ----------------
    g = S.ds("drv_csa_gain")
    b = {"gain_V_per_A": g, "mid_V": 1.5, "range_A": [-(1.5 - 0.25) / g, (2.75 - 1.5) / g],
         "lsb_mA": LSB12 / g * 1e3, "offset_LSB": S.ds("drv_csa_offset") * g / LSB12}
    tsv2 = np.linspace(20e-9, 5e-6, 2000)
    # direct: kick across CADC + pin; recovery by the amplifier (tau CSA)
    v0_direct = 1.5 * C_ADC / (C_ADC + C_PIN + C_PCB)
    r_direct = v0_direct * np.exp(-tsv2 / CSA_TAU) / LSB12
    rc = {}
    for (rs, cx) in ((100.0, 1e-9), (220.0, 2.2e-9), (100.0, 2.2e-9)):
        v0 = 1.5 * C_ADC / (C_ADC + C_PIN + C_PCB + cx)
        tau = rs * (cx + C_ADC + C_PIN + C_PCB) + 0.0
        rc[(rs, cx)] = v0 * np.exp(-tsv2 / tau) / LSB12
    def first_below(r):
        k = np.flatnonzero(r < 0.5)
        return tsv2[k[0]] if k.size else None
    b["direct_ts_for_0.5LSB_ns"] = first_below(r_direct) * 1e9
    for (rs, cx), r in rc.items():
        b[f"RC_{rs:.0f}ohm_{cx*1e9:.1f}nF_ts_ns"] = first_below(r) * 1e9
        b[f"RC_{rs:.0f}ohm_{cx*1e9:.1f}nF_fc_MHz"] = 1 / (2 * math.pi * rs * cx) / 1e6
    print(f"  CSA: {b['lsb_mA']:.2f} mA/LSB, offset +/-{b['offset_LSB']:.0f} LSB, range {b['range_A'][0]:.1f}..{b['range_A'][1]:.1f} A;"
          f" direct ts {b['direct_ts_for_0.5LSB_ns']:.0f} ns, 100R/1nF ts {b['RC_100ohm_1.0nF_ts_ns']:.0f} ns")
    # ripple-induced sampling error: from study-2 style ripple at hover
    b["note_ripple"] = "sample the continuously-ON low-side phase at the middle of the PWM on (or off) interval"
    res["b_csa"] = b
    # ---------------- (c) VBAT divider ----------------
    r_th, c_f = 5e3, 100e-9
    tau = r_th * c_f
    c = {"tau_ms": tau * 1e3, "fc_Hz": 1 / (2 * math.pi * tau),
         "kickback_LSB_fullscale": VREF * C_ADC / (C_ADC + c_f) / LSB12,
         "ripple_atten_48kHz_dB": 20 * math.log10(1 / math.sqrt(1 + (2 * math.pi * 48e3 * tau) ** 2)),
         "lsb_at_battery_mV": 2 * LSB12 * 1e3, "range_V": 2 * VREF}
    for fs in (1e3, 10e3):
        c[f"sampling_bias_mV_fs{fs/1e3:.0f}k"] = r_th * C_ADC * 1.5 * fs * 1e3
    vpp_sync = 0.443  # study 5 worst synchronous VBAT ripple (V pk-pk)
    c["ripple_at_adc_mVpp_from_443mVpp"] = vpp_sync * 0.5 * 10 ** (c["ripple_atten_48kHz_dB"] / 20) * 1e3
    c["t_99pct_ms"] = tau * math.log(100) * 1e3
    c["accuracy_budget_mV_at_4V2"] = {"divider_1pct_ratio": 4.2 * 0.01 * 1e3 / 2,
                                      "VREF_LDO_1p5pct": 4.2 * 0.015 * 1e3,
                                      "with_VREFINT_cal_(G071 VREFINT)": "~0.5-1% -> 20-40 mV"}
    c["quiescent_drain_mA"] = S.V_FULL / 20e3 * 1e3
    c["days_to_drain_260mAh"] = 0.26 / (S.V_FULL / 20e3) / 24
    res["c_vbat"] = c
    print(f"  VBAT: tau {tau*1e3:.2f} ms, 48 kHz ripple {c['ripple_atten_48kHz_dB']:.0f} dB -> "
          f"{c['ripple_at_adc_mVpp_from_443mVpp']:.2f} mVpp at ADC, kick {c['kickback_LSB_fullscale']:.2f} LSB")

    # ================= plots =================
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    ax = axs[0]
    for (c_ext, (r, tsel)), col in zip(model.items(), PAL):
        ax.semilogy(tsv * 1e6, r, color=col, label=f"5 kOhm divider, C_ext = {c_ext*1e12:.0f} pF")
    for cyc in TS_OPTS[2:7]:
        ax.axvline(cyc / F_ADC * 1e6, color="#d9d8d2", lw=0.8)
        ax.text(cyc / F_ADC * 1e6, 3e3, f"{cyc}", fontsize=7, color=INK2, rotation=90, va="top")
    S.hline_limit(ax, 0.5, "0.5 LSB (12-bit)")
    ax.set_xscale("log")
    ax.set_xlim(0.05, 20)
    ax.set_ylim(1e-2, 5e3)
    ax.set_xlabel("sampling time ts (us)  [labels: ADC cycles @35 MHz]")
    ax.set_ylabel("residual error after ts (LSB, full-scale step)")
    ax.set_title("BEMF divider -> G071 ADC settling (charge-sharing model)")
    ax.legend(loc="lower left", fontsize=7.5)
    ax = axs[1]
    for bits, col in ((12, PAL[0]), (10, PAL[1])):
        ts, rmax, cyc = table57(bits)
        ax.loglog(ts * 1e9, rmax, marker="o", ms=4, color=col, label=f"Table 57 max RAIN, {bits}-bit")
    ax.axhline(r_src, color=INK2, ls="--", lw=1)
    ax.text(50, r_src * 1.1, "BEMF divider 5 kOhm", fontsize=7.5, color=INK2)
    ax.axhline(2350, color=INK2, ls=":", lw=1)
    ax.text(50, 2350 * 1.1, "4.7k/4.7k option 2.35 kOhm", fontsize=7.5, color=INK2)
    ax.set_xlabel("sampling time ts (ns) @35 MHz")
    ax.set_ylabel("max source resistance RAIN (ohm)")
    ax.set_title("STM32G071 DS12232 Table 57")
    ax.legend(loc="lower right")
    fig.tight_layout()
    S.save(fig, "s6_bemf_adc.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0))
    ax = axs[0]
    ax.semilogy(tsv2 * 1e6, r_direct, color=PAL[0], label="SOx direct to pin (CSA recovers kick)")
    for ((rs, cx), r), col in zip(rc.items(), PAL[1:]):
        ax.semilogy(tsv2 * 1e6, r, color=col, label=f"{rs:.0f} ohm + {cx*1e9:.1f} nF reservoir")
    S.hline_limit(ax, 0.5, "0.5 LSB")
    ax.set_xlabel("sampling time ts (us)")
    ax.set_ylabel("residual error (LSB, 1.5 V step)")
    ax.set_ylim(1e-2, 2e3)
    ax.set_title("CSA output -> ADC sampling")
    ax.legend(loc="upper right", fontsize=7.5)
    ax = axs[1]
    t = np.linspace(0, 4e-3, 4000)
    v_bat = 4.0 - 0.5 * (t > 0.5e-3)  # 0.5 V sag step
    v_adc = 0.5 * (4.0 - 0.5 * (t > 0.5e-3) * (1 - np.exp(-(t - 0.5e-3).clip(0) / tau)))
    ax.plot(t * 1e3, 0.5 * v_bat, color=INK2, lw=1, ls=":", label="VBAT/2 (0.5 V sag step at 0.5 ms)")
    ax.plot(t * 1e3, v_adc, color=PAL[0], label="ADC pin (10k/10k + 100 nF)")
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("V")
    ax.set_title(f"VBAT divider: tau = {tau*1e3:.1f} ms, 48 kHz ripple {c['ripple_atten_48kHz_dB']:.0f} dB")
    ax.legend(loc="upper right")
    fig.tight_layout()
    S.save(fig, "s6_csa_vbat.png")

    S.write_results("study6_adc", res)


if __name__ == "__main__":
    main()
