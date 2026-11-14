"""
Study 8 - TPS7A2030 dropout and brown-out margins during battery sag.

VBAT at the board = OCV(SoC) - (R_batt + R_pigtail) * I_total for: hover, steady full throttle
(averaged model, D = 1) and the punch-out current peak (hover -> D = 1 step, first instant).
TPS7A20 dropout modelled as R_DO = VDO_max(300 mA)/300 mA = 0.467 ohm (CMOS pass device; datasheet
s7.3.3 'dropout scales with output current').  3V0 = min(3.0, VIN - I_3V0 * R_DO).

Thresholds: DRV8311 VM ROC min 3.0 V, UVLO falling 2.5/2.6/2.7 V (deglitch 5-13 us);
SX1280 VBAT min 1.8 V; LSM6DSV16X 1.71 V; STM32 PDR/BOR0 1.60-1.69 V (BOR4 2.70-2.90 V if enabled).

Run:  python sim_08_ldo_brownout.py
"""
import math
import simlib as S
import numpy as np
from simlib import plt, PAL, CRIT, INK2

R_DO = S.ds("ldo_vdo_300mA") / 0.300
BUDGET = {  # (typ, worst) mA, source
    "STM32G431 FC @170 MHz": (35.0, 56.0, "G474 DS12288 Table 21 p.90 proxy: 29.5 typ (+periph.), 56 max @125C"),
    "4x STM32G071 @64 MHz": (4 * 7.8, 4 * 10.2, "DS12232 Table 25 p.59: 6.3 typ / 7.7 max + ADC/COMP/TIM ~1.5-2.5 mA (assumption)"),
    "SX1280": (12.0, 48.0, "Table 3-5/3-8 x2 LDO-mode (s14.7.1): RX ~6 mA x2 typ, TX +12.5 dBm 24 mA x2 worst"),
    "LSM6DSV16X": (0.65, 0.65, "DS13510 Table 4 p.14"),
    "4x DRV8311 CSAREF": (4 * 1.7, 4 * 3.0, "DRV8311 s7.5 p.12 ICSAREF 1.7 typ / 3.0 max (ROC lists up to 7.5)"),
    "misc (LED, pull-ups, VDDA)": (3.0, 6.0, "assumption"),
}


def main():
    print("Study 8: LDO dropout / brown-out")
    i_typ = sum(v[0] for v in BUDGET.values()) / 1e3
    i_wst = sum(v[1] for v in BUDGET.values()) / 1e3
    vin_min_typ = 3.0 + i_typ * R_DO
    vin_min_wst = 3.0 + i_wst * R_DO
    res = {"budget_mA": {k: {"typ": v[0], "worst": v[1], "src": v[2]} for k, v in BUDGET.items()},
           "I_3V0_typ_A": i_typ, "I_3V0_worst_A": i_wst, "R_DO_ohm": R_DO,
           "VIN_min_regulation_typ_V": vin_min_typ, "VIN_min_regulation_worst_V": vin_min_wst,
           "ldo_dissipation_W_at_4V35_worst": (4.35 - 3.0) * i_wst,
           "ldo_Tj_rise_C_JEDEC": (4.35 - 3.0) * i_wst * S.ds("ldo_theta_ja")}
    print(f"  3V0 load: typ {i_typ*1e3:.0f} mA, worst {i_wst*1e3:.0f} mA; regulation needs VIN >= "
          f"{vin_min_typ:.3f} V (typ) / {vin_min_wst:.3f} V (worst)")
    rds_cold = S.rds_of_T(25.0)
    r_m = S.R_LL + S.R_EXT + rds_cold
    ocvs = np.linspace(3.30, 4.35, 43)
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    t_h = auw / (4 * S.OVERLAP)
    i_hk_w = i_wst + 4 * 0.012
    curves = {}
    for rb in (0.025, 0.040, 0.060):
        r_path = rb + S.R_PIG
        vh, vf, vp, v3p = [], [], [], []
        for o in ocvs:
            hov = S.op_point(thrust=t_h, soc_ocv=o, r_batt=rb, r_ds=rds_cold, i_hk=i_hk_w)
            # switching-model correction for ripple losses at hover (study 4: ~+25% bus current at L 6 uH)
            i_h = 4 * hov["duty"] * hov["i_dc"] * 1.25 + i_hk_w
            vh.append(o - r_path * i_h)
            full = S.op_point(duty=1.0, soc_ocv=o, r_batt=rb, r_ds=rds_cold, i_hk=i_hk_w)
            vf.append(full["vm"])
            e_h = S.KE * hov["w"]
            i_pk = (o - r_path * i_hk_w - e_h) / (r_m + 4 * r_path)
            v_p = o - r_path * (4 * i_pk + i_hk_w)
            vp.append(v_p)
            v3p.append(min(3.0, v_p - i_wst * R_DO))
        curves[rb] = {k: np.array(v) for k, v in (("hover", vh), ("full", vf), ("punch", vp), ("v3_punch", v3p))}

    def first_ocv_above(arr, thr):
        k = np.flatnonzero(arr >= thr)
        return round(float(ocvs[k[0]]), 3) if k.size else None
    summ = {}
    for rb, c in curves.items():
        key = f"Rbatt_{rb*1e3:.0f}mOhm"
        summ[key] = {
            "min_OCV_full_throttle_VM_ge_3V0": first_ocv_above(c["full"], 3.0),
            "min_OCV_punch_VM_ge_3V0": first_ocv_above(c["punch"], 3.0),
            "min_OCV_punch_VM_ge_2V7(UVLO max)": first_ocv_above(c["punch"], 2.7),
            "min_OCV_LDO_regulates_during_punch": first_ocv_above(c["punch"], vin_min_wst),
            "VM_punch_at_OCV3V6": float(np.interp(3.6, ocvs, c["punch"])),
            "VM_full_at_OCV3V6": float(np.interp(3.6, ocvs, c["full"])),
            "VM_hover_at_OCV3V45": float(np.interp(3.45, ocvs, c["hover"])),
            "V3V0_min_punch_at_OCV3V45": float(np.interp(3.45, ocvs, c["v3_punch"])),
            "V3V0_min_overall": float(c["v3_punch"].min()),
        }
        print(f"  R_batt {rb*1e3:.0f} mOhm: full-throttle VM>=3.0 V needs OCV >= {summ[key]['min_OCV_full_throttle_VM_ge_3V0']}, "
              f"punch VM>=2.7 V needs OCV >= {summ[key]['min_OCV_punch_VM_ge_2V7(UVLO max)']}, "
              f"LDO regulates in punch for OCV >= {summ[key]['min_OCV_LDO_regulates_during_punch']}; "
              f"3V0 min {summ[key]['V3V0_min_overall']:.2f} V")
    res["sag"] = summ
    res["thresholds"] = {"DRV8311_VM_ROC_min": 3.0, "DRV8311_UVLO_fall": S.ds("drv_uvlo_fall"),
                         "SX1280_min": 1.8, "IMU_min": 1.71, "STM32_PDR_BOR0_fall_max": 1.69,
                         "STM32G071_BOR4_fall": S.ds("g071_bor")["bor4_fall"]}

    # ================= plots =================
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.4))
    ax = axs[0]
    for (scen, col) in (("hover", PAL[0]), ("full", PAL[1]), ("punch", PAL[2])):
        lo = curves[0.060][scen]
        hi = curves[0.025][scen]
        ax.fill_between(ocvs, lo, hi, color=col, alpha=0.18, lw=0)
        lab = {"hover": "hover", "full": "steady full throttle", "punch": "punch-out peak (hover -> D=1)"}[scen]
        ax.plot(ocvs, curves[0.040][scen], color=col, label=f"{lab} (R_batt 40, band 25-60 mOhm)")
    S.hline_limit(ax, vin_min_wst, f"TPS7A2030 dropout edge {vin_min_wst:.2f} V (worst load)", ls="-.")
    S.hline_limit(ax, 3.0, "DRV8311 VM ROC min 3.0 V", ls=":", va="top")
    S.hline_limit(ax, 2.7, "DRV8311 UVLO falling max 2.7 V")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("VBAT at board / VM (V)")
    ax.set_title("VBAT sag vs battery state (4 motors, DRV8311 RDS @25 C)")
    ax.set_ylim(2.0, 4.4)
    ax.legend(loc="lower right", fontsize=7.3)
    ax = axs[1]
    for rb, col in zip((0.025, 0.040, 0.060), PAL):
        ax.plot(ocvs, curves[rb]["v3_punch"], color=col, label=f"3V0 during punch-out, R_batt {rb*1e3:.0f} mOhm")
    S.hline_limit(ax, 2.90, "STM32 BOR4 falling max 2.90 V (only if enabled)", ls=":")
    S.hline_limit(ax, 1.8, "SX1280 VBAT min 1.8 V")
    S.hline_limit(ax, 1.69, "STM32 PDR/BOR0 max 1.69 V", ls=":", va="top")
    ax.set_xlabel("battery OCV (V)")
    ax.set_ylabel("3V0 rail (V)")
    ax.set_title(f"TPS7A2030 output (worst load {i_wst*1e3:.0f} mA, R_DO {R_DO:.3f} ohm)")
    ax.set_ylim(1.5, 3.2)
    ax.legend(loc="lower right", fontsize=7.5)
    fig.tight_layout()
    S.save(fig, "s8_brownout.png")
    S.write_results("study8_ldo_brownout", res)


if __name__ == "__main__":
    main()
