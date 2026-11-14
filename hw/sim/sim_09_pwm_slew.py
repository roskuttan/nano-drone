"""Study 9: PWM frequency x DRV8311H SLEW-pin setting -> driver loss, timing margins, EMI proxy.

Follow-up to studies 2/3 (hover loss 0.78 W @ 48 kHz, 0.57 W @ 96 kHz, SLEW=AGND). The HW variant fixes the
handshake dead time and propagation delay by the SLEW pin, so frequency and slew are swept together.

[DS] SLVSFN2B s7.5 p.10-11, typ values at VM 12 V (rise/fall SR, tDEAD handshake, tPD):
    SLEW = AGND        35 / 35 V/us   500 ns   1000 ns
    SLEW = 47k to AGND 75 / 75 V/us   450 ns    650 ns
    SLEW = Hi-Z       180 / 180 V/us  425 ns    550 ns
    SLEW = AVDD       230 / 270 V/us  425 ns    500 ns
[A] SR and tDEAD at VM 3.6-4.2 V taken equal to the 12 V figures (the gate drive is VM-independent through
    the DRV8311 charge pump/LDO; no 1S data is given).

Outputs: out/s9_pwm_slew.png, out/study9_pwm_slew.json
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simlib as S  # noqa: E402
import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

SLEW = [  # label, SR rise (V/s), tDEAD (s), tPD (s)
    ("AGND (35 V/us)", 35e6, 500e-9, 1000e-9),
    ("47k (75 V/us)", 75e6, 450e-9, 650e-9),
    ("Hi-Z (180 V/us)", 180e6, 425e-9, 550e-9),
    ("AVDD (230 V/us)", 230e6, 425e-9, 500e-9),
]
FREQS = [48e3, 72e3, 96e3, 128e3, 160e3, 192e3]
TJ = 100.0
OCV = 3.85


def hover_thrust():
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    return auw / (4 * S.OVERLAP)


def emi_envelope_dbuv(vm, f_pwm, duty, t_r, f):
    """Upper bound of the trapezoidal-pulse harmonic envelope (dBuV) at frequency f."""
    T = 1.0 / f_pwm
    a = 2 * vm * duty                                      # 2A*tau/T
    env = a
    f1 = 1 / (math.pi * duty * T)
    f2 = 1 / (math.pi * t_r)
    if f > f1:
        env *= f1 / f
    if f > f2:
        env *= f2 / f
    return 20 * math.log10(env / 1e-6)


def main():
    S.style()
    th = hover_thrust()
    rds = S.rds_of_T(TJ, "typ")
    rows = []
    for lab, sr, td, tpd in SLEW:
        for f in FREQS:
            avg, r = S.sim_at_thrust(th, soc_ocv=OCV, r_ds=rds, f_pwm=f, record=False, sr=sr, t_dead=td)
            p_stby = r["vm"] * 0.011
            row = dict(slew=lab, sr=sr, t_dead=td, t_pd=tpd, f_pwm=f, duty=r["duty"], vm=r["vm"],
                       p_cond=r["p_cond"], p_diode=r["p_diode"], p_sw=r["p_sw"],
                       p_total=r["p_drv_total"] + p_stby, i_bus=r["i_bus_avg"], i_ph_pk=r["i_ph_pk"],
                       i_ph_rms=r["i_ph_rms"], p_cu_motor=r["p_cu_motor"],
                       tpd_frac=tpd * f, dead_frac=2 * td * f,
                       min_on_bemf=(2.8e-6 + 0.557e-6) * f,
                       emi_10M=emi_envelope_dbuv(r["vm"], f, r["duty"], r["vm"] / sr, 10e6),
                       emi_30M=emi_envelope_dbuv(r["vm"], f, r["duty"], r["vm"] / sr, 30e6),
                       emi_100M=emi_envelope_dbuv(r["vm"], f, r["duty"], r["vm"] / sr, 100e6))
            rows.append(row)
            print("%-17s %5.0f kHz  D %.3f  cond %.3f  diode %.3f  sw %.3f  total %.3f W  Ipk %.2f A  "
                  "Ibat/mot %.2f A  EMI30M %.1f dBuV" % (lab, f / 1e3, row["duty"], row["p_cond"],
                                                         row["p_diode"], row["p_sw"], row["p_total"],
                                                         row["i_ph_pk"], row["i_bus"], row["emi_30M"]), flush=True)
    best = min(rows, key=lambda r: r["p_total"])
    base = [r for r in rows if r["slew"].startswith("AGND") and r["f_pwm"] == 48e3][0]
    print("best: %s @ %.0f kHz -> %.3f W (baseline AGND 48 kHz %.3f W)" % (best["slew"], best["f_pwm"] / 1e3,
                                                                          best["p_total"], base["p_total"]))

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
    for k, (lab, sr, td, tpd) in enumerate(SLEW):
        rr = [r for r in rows if r["slew"] == lab]
        fx = [r["f_pwm"] / 1e3 for r in rr]
        ax[0].plot(fx, [r["p_total"] for r in rr], "o-", color=S.PAL[k], label=lab)
        ax[1].plot(fx, [r["p_cond"] for r in rr], "o-", color=S.PAL[k], label=lab + " cond")
        ax[1].plot(fx, [r["p_diode"] + r["p_sw"] for r in rr], "s--", color=S.PAL[k], label=lab + " diode+sw")
        ax[2].plot(fx, [r["emi_30M"] for r in rr], "o-", color=S.PAL[k], label=lab)
    ax[0].set(xlabel="PWM frequency (kHz)", ylabel="DRV8311 loss per driver at hover (W)",
              title="Hover driver loss, Tj %.0f C, OCV %.2f V, L %.0f uH" % (TJ, OCV, S.L_LL * 1e6))
    ax[1].set(xlabel="PWM frequency (kHz)", ylabel="W", title="Loss split: conduction vs dead-time diode + switching")
    ax[2].set(xlabel="PWM frequency (kHz)", ylabel="dBuV (trapezoid envelope)",
              title="Phase-node harmonic envelope at 30 MHz")
    for a in ax:
        a.legend(fontsize=7)
    S.save(fig, "s9_pwm_slew.png")
    S.write_results("study9_pwm_slew", {"rows": rows, "best": best, "baseline": base,
                                        "assumptions": "SR/tDEAD at 1S = 12 V datasheet values; Tj 100 C; "
                                                       "OCV %.2f V; L %.1f uH" % (OCV, S.L_LL * 1e6)})


if __name__ == "__main__":
    main()
