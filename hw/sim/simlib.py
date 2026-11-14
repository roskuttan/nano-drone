"""
simlib.py - shared parameters, datasheet citations, assumptions and models for the
Nano-drone hub-board simulation studies (hw/sim).

Every study script imports this module so that ALL studies use one consistent set of
numbers.  Each script is still runnable on its own:  python sim_xxx.py

Conventions
-----------
* DS[...]   : values taken from a datasheet; each entry carries a citation string.
* ASM[...]  : engineering assumptions (NOT from a datasheet); each carries a range/note.
* SI units everywhere inside the code (V, A, ohm, H, F, s, m, kg, W, K).
"""
import os
import sys
import json
import math

HERE = os.path.dirname(os.path.abspath(__file__))
_PYLIBS = os.path.join(HERE, ".pylibs")
if _PYLIBS not in sys.path:
    sys.path.insert(0, _PYLIBS)

import numpy as np  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

# --------------------------------------------------------------------------------------
# Plot style (categorical palette in fixed order; limit lines use the reserved
# 'critical' status colour and are always labelled)
# --------------------------------------------------------------------------------------
PAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
CRIT = "#d03b3b"     # limit / fail lines (always with a text label)
GOOD = "#0ca30c"
INK = "#0b0b0b"
INK2 = "#52514e"
GRIDC = "#e4e3df"
SURF = "#fcfcfb"


def style():
    plt.rcParams.update({
        "figure.facecolor": SURF,
        "axes.facecolor": SURF,
        "savefig.facecolor": SURF,
        "axes.edgecolor": INK2,
        "axes.labelcolor": INK,
        "axes.titlecolor": INK,
        "axes.titlesize": 10.5,
        "axes.labelsize": 9.5,
        "axes.grid": True,
        "grid.color": GRIDC,
        "grid.linewidth": 0.7,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": INK2,
        "ytick.color": INK2,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "lines.linewidth": 1.6,
        "font.size": 9.5,
    })


style()


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", os.path.relpath(path, HERE))
    return path


def hline_limit(ax, y, text, color=CRIT, ls="--", xpos=0.01, va="bottom"):
    ax.axhline(y, color=color, ls=ls, lw=1.1)
    ax.text(xpos, y, " " + text, transform=ax.get_yaxis_transform(), color=INK2,
            fontsize=7.5, va=va, ha="left")


def write_results(name, data):
    def conv(o):
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))
    path = os.path.join(OUT, name + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, default=conv)
    print("  wrote", os.path.relpath(path, HERE))


# --------------------------------------------------------------------------------------
# DATASHEET VALUES (with citations)
# --------------------------------------------------------------------------------------
DS = {
    # ---- TI DRV8311 (SLVSFN2B, Feb 2022) ----
    "drv_vm_absmax": (24.0, "DRV8311 SLVSFN2B s7.1 Abs Max p.7: VM -0.3..24 V"),
    "drv_vm_roc": ((3.0, 20.0), "DRV8311 s7.3 ROC p.7: VVM 3..20 V"),
    "drv_iout_peak": (5.0, "DRV8311 s7.1 p.7 Peak output current 5 A; s7.3 p.7 IOUT peak 5 A"),
    "drv_rds_25_typ": (0.300, "DRV8311 s7.5 p.10: RDS(on) HS+LS, 3V<=VM<=6V, IOUT=1A, TJ=25C: 300 typ / 350 max mOhm"),
    "drv_rds_25_max": (0.350, "DRV8311 s7.5 p.10"),
    "drv_rds_150_typ": (0.450, "DRV8311 s7.5 p.10: TJ=150C: 450 typ / 500 max mOhm"),
    "drv_rds_150_max": (0.500, "DRV8311 s7.5 p.10"),
    "drv_sr_gnd": ((18e6, 35e6, 55e6), "DRV8311 s7.5 p.10: slew rate SLEW=AGND 18/35/55 V/us (rise)"),
    "drv_tdead_gnd": ((500e-9, 1200e-9), "DRV8311 s7.5 p.11: tDEAD SLEW=AGND handshake 500 typ / 1200 max ns"),
    "drv_tpd_gnd": ((1000e-9, 1500e-9), "DRV8311 s7.5 p.11: tPD SLEW=AGND 1000 typ / 1500 max ns"),
    "drv_csa_gain": (0.25, "DRV8311 s7.5 p.11 GCSA=0.25 V/A (GAIN=AGND); Table 8-5 p.29"),
    "drv_csa_tset": (1e-6, "DRV8311 s7.5 p.11: CSA settling to 1% = 1 us with 30 pF on SOx"),
    "drv_csa_lin": ((0.25, "VCSAREF-0.25"), "DRV8311 s7.5 p.12: SOx linear range 0.25..VCSAREF-0.25 V"),
    "drv_csa_offset": (0.070, "DRV8311 s7.5 p.12: IOFFSET +/-70 mA (0.25 V/A, full temp)"),
    "drv_icsaref": ((1.7e-3, 3.0e-3), "DRV8311 s7.5 p.12: ICSAREF 1.7 typ / 3.0 max mA @ CSAREF=3.0 V (ROC s7.3 p.7 lists up to 7.5 mA)"),
    "drv_uvlo_fall": ((2.5, 2.6, 2.7), "DRV8311 s7.5 p.12: VUVLO VM falling 2.5/2.6/2.7 V"),
    "drv_uvlo_rise": ((2.6, 2.7, 2.8), "DRV8311 s7.5 p.12: VUVLO VM rising 2.6/2.7/2.8 V"),
    "drv_uvlo_deg": ((5e-6, 7.5e-6, 13e-6), "DRV8311 s7.5 p.12: tUVLO deglitch 5/7.5/13 us"),
    "drv_ocp": ((5.8, 9.0, 11.5), "DRV8311 s7.5 p.12: IOCP (MODE Hi-Z/AGND) 5.8/9/11.5 A"),
    "drv_ocp_blank": (0.2e-6, "DRV8311 s7.5 p.13: tBLANK (HW variant) 0.2 us; tOCP_DEG 1 us"),
    "drv_ivm": ((10e-3, 15e-3), "DRV8311 s7.5 p.8: IVM operating 10-12 typ / 15 max mA"),
    "drv_tsd": ((180, 190, 200), "DRV8311 s7.5 p.13: TTSD 180/190/200 C"),
    "drv_tj_max": (150, "DRV8311 s7.3 p.8: operating TJ max 150 C"),
    "drv_theta_ja": (42.6, "DRV8311 s7.4 p.8: RthJA 42.6 C/W (JEDEC)"),
    "drv_theta_jb": (15.7, "DRV8311 s7.4 p.8: RthJB 15.7 C/W"),
    "drv_theta_jc_bot": (4.8, "DRV8311 s7.4 p.8: RthJC(bot) 4.8 C/W"),
    "drv_theta_jc_top": (37.9, "DRV8311 s7.4 p.8: RthJC(top) 37.9 C/W"),
    "drv_pkg": ("WQFN-24 3.0x3.0 mm, exposed pad ~1.9x1.9 mm", "DRV8311 p.1 Device Information; RRW0024A drawing"),
    "drv_loss_formulas": ("PCON=2*IPK^2*RDS,ON ; PSW=IPK*VPK*trf*fPWM ; Pdiode=2*IPK*VF*tDEAD*fPWM",
                          "DRV8311 s12.3.1 Table 12-1 p.85 (trapezoidal)"),
    # ---- TI TPS7A20 (ZHCSKY8G, May 2022, Chinese edition of SBVS338) ----
    "ldo_vin_absmax": (6.5, "TPS7A20 ZHCSKY8G s6.1 Abs Max p.5: VIN -0.3..6.5 V"),
    "ldo_vin_roc": ((1.6, 6.0), "TPS7A20 s6.3 ROC p.5: VIN 1.6..6.0 V"),
    "ldo_vdo_300mA": (0.140, "TPS7A20 s6.5 p.6: VDO max 140 mV @300 mA, 2.5V<=VOUT<5.5V (DQN/X2SON)"),
    "ldo_load_reg": (0.013, "TPS7A20 s6.5 p.6: load regulation 13 mV, 1..300 mA (DQN)"),
    "ldo_cout": ((1e-6, 200e-6), "TPS7A20 s6.3 p.5: COUT 1..200 uF; eff. >=0.47 uF; ESR<=100 mOhm"),
    "ldo_psrr": ({"20mA": {"10k": 75, "100k": 75, "1M": 45}, "300mA": {"10k": 75, "100k": 60, "1M": 40}},
                 "TPS7A20 s6.5 p.7: PSRR (VIN=VOUT+1 V)"),
    "ldo_icl": ((0.360, 0.520, 0.730), "TPS7A20 s6.5 p.7: ICL DQN 360/520/730 mA"),
    "ldo_theta_ja": (166.1, "TPS7A20 s6.4 p.6: RthJA DQN (X2SON) 166.1 C/W"),
    "ldo_uvlo_fall": ((1.11, 1.30, 1.55), "TPS7A20 s6.5 p.7: UVLO VIN falling (DQN)"),
    # ---- Semtech SX1280 (DS.SX1280-1.W.APP rev 3.2, Mar 2020) ----
    "sx_vbat_op": ((1.8, 3.7), "SX1280 Table 3-2 p.19: VBAT/VBAT_IO operating 1.8..3.7 V"),
    "sx_vbat_absmax": (3.9, "SX1280 Table 3-1 p.19: VBAT max 3.9 V"),
    "sx_itx_12p5": (24e-3, "SX1280 Table 3-8 p.26: IDD_T13 24 mA @ +12.5 dBm (DC-DC on)"),
    "sx_irx": ((5.5e-3, 8.2e-3), "SX1280 Table 3-5 p.22: LoRa RX 5.5..8.2 mA (DC-DC on)"),
    "sx_ldo_penalty": (2.0, "SX1280 s14.7.1 SetRegulatorMode: 'penalty for using the LDO is a doubling of current consumption'"),
    "sx_match": ("C0 0.8pF, L1 3.0nH (LQW15AN3N0B80), C1 1.2pF, L2 2.5nH (LQW15AN2N5C00)||C4 0.5pF, C2 1.2pF, C3 100pF",
                 "SX1280 s15.1 Fig.15-1 p.145 and Table 15-1 BOM p.146"),
    # ---- ST STM32G071 (DS12232 rev 3) ----
    "g071_cadc": (5e-12, "STM32G071 DS12232 r3 Table 56 p.85: CADC 5 pF typ"),
    "g071_rain_max": ({12: [(1.5, 43e-9, 50), (3.5, 100e-9, 680), (7.5, 214e-9, 2200), (12.5, 357e-9, 4700),
                           (19.5, 557e-9, 8200), (39.5, 1129e-9, 15000), (79.5, 2271e-9, 33000), (160.5, 4586e-9, 50000)],
                       10: [(1.5, 43e-9, 68), (3.5, 100e-9, 820), (7.5, 214e-9, 3300), (12.5, 357e-9, 5600),
                            (19.5, 557e-9, 10000), (39.5, 1129e-9, 22000), (79.5, 2271e-9, 39000), (160.5, 4586e-9, 50000)]},
                      "STM32G071 DS12232 r3 Table 57 p.86-87: max RAIN vs sampling time @ fADC=35 MHz"),
    "g071_fadc": (35e6, "STM32G071 Table 56 p.85: fADC max 35 MHz (Range 1); 2.5 MSps @12 bit; tCONV = ts + 12.5 cycles"),
    "g071_cio": (5e-12, "STM32G071 Table 51 p.79: CIO I/O pin capacitance 5 pF"),
    "g071_irun64": ((6.3e-3, 7.7e-3), "STM32G071 Table 25 p.59: IDD Run 64 MHz flash 6.3 typ / 7.7 max(125C) mA"),
    "g071_bor": ({"pdr": (1.60, 1.64, 1.69), "bor4_fall": (2.70, 2.81, 2.90)}, "STM32G071 Table 23 p.56: VPDR, VBOR4 falling"),
    "g071_iinj": (-5e-3, "STM32G071 Table 19 p.54: IINJ(PIN) -5 mA (analog performance disturbed by injection)"),
    # ---- ST STM32G474 (DS12288 rev 5) used as proxy for STM32G431 ----
    "g4_irun170": ((29.5e-3, 31e-3, 56e-3), "STM32G474 DS12288 r5 Table 21 p.90: IDD Run 170 MHz 29.5 typ, 31 max(25C), 56 max(125C) mA (all periph. disabled); used as G431 proxy"),
    "g4_bor": ({"bor0_fall": (1.60, 1.64, 1.69)}, "STM32G474 DS12288 r5 Table 19 p.86: VBOR0 falling 1.60/1.64/1.69 V"),
    # ---- ST LSM6DSV16X (DS13510 rev 4) ----
    "imu_idd": (0.65e-3, "LSM6DSV16X DS13510 r4 s4.2 Table 4 p.14: IddHP 0.65 mA"),
    "imu_vdd": ((1.71, 3.6), "LSM6DSV16X Table 4 p.14: Vdd 1.71..3.6 V"),
}


def ds(key):
    return DS[key][0]


# --------------------------------------------------------------------------------------
# ASSUMPTIONS (engineering estimates - NOT datasheet values)
# --------------------------------------------------------------------------------------
ASM = {}


def asm(key, value, note, rng=None):
    ASM[key] = {"value": value, "range": rng, "note": note}
    return value


# ---- battery & power path ----
V_FULL = asm("v_full", 4.35, "LiHV full charge (OCV)", (4.20, 4.45))
R_BATT = asm("r_batt", 0.040, "battery DC internal resistance incl. BT2.0 connector (user range)", (0.025, 0.060))
R_PIG = asm("r_pig", 0.010, "BT2.0 pigtail 30-40 mm (user value)", (0.008, 0.012))
L_PIG = asm("l_pig", 40e-9, "pigtail + cell-tab loop inductance (user 25-50 nH)", (25e-9, 50e-9))
CAP_MAH = asm("cap_mah", {"260": 260, "300": 300}, "rated capacity; model assumes 100% of rated available at <=10C")
M_BATT = asm("m_batt", {"260": 7.6, "300": 8.3}, "battery mass g (user)")
M_DRY = asm("m_dry", 7.7, "AUW without battery, g (gives 15.3 g / 16.0 g AUW)", (7.2, 8.2))
OVERLAP = asm("overlap_eff", 0.90, "stacked/overlapping-prop thrust efficiency (10% penalty)", (0.85, 0.95))
V_LAND = asm("v_land", 3.30, "loaded pack voltage at which pilot lands (hover)", (3.2, 3.4))

# LiHV open-circuit voltage curve (generic, assumption)
_SOC = np.array([0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
_OCV = np.array([3.30, 3.55, 3.65, 3.72, 3.77, 3.81, 3.86, 3.93, 4.01, 4.10, 4.21, 4.35])
asm("ocv_curve", {"soc": _SOC.tolist(), "ocv": _OCV.tolist()}, "generic LiHV OCV(SoC) curve")


def ocv(soc):
    return np.interp(np.clip(soc, 0.0, 1.0), _SOC, _OCV)


def r_batt_soc(soc, r0=R_BATT):
    """IR rises ~50% toward empty (assumption)."""
    soc = np.clip(soc, 0.0, 1.0)
    return r0 * (1.0 + 0.5 * np.clip((0.2 - soc) / 0.2, 0.0, 1.0))


# ---- capacitors ----
asm("c22_dcbias", "C/C0 = 1/(1+(V/V0)^1.6), V0=4.0 V nominal (50% at 4 V), V0=3.2..5.2 V gives 40..60% at 4 V",
    "22 uF 0603 10 V X5R DC-bias curve fit (user: 40-60% retained at 4 V)")
asm("c22_tol", 0.20, "22 uF initial tolerance +/-20% (M)")
asm("c22_esr", 0.004, "22 uF 0603 ESR near resonance", (0.003, 0.006))
asm("esl_0603", 0.55e-9, "0603 mounted ESL incl. vias to L2/L5", (0.4e-9, 0.8e-9))
asm("esl_0402", 0.40e-9, "0402 mounted ESL incl. vias", (0.3e-9, 0.6e-9))


def c22_retention(v, v0=4.0, n=1.6):
    return 1.0 / (1.0 + (np.maximum(v, 0.0) / v0) ** n)


# ---- motor ----
KV = 23000.0
KE = 60.0 / (2.0 * math.pi * KV)          # V*s/rad (6-step DC-equivalent); = Kt in N*m/A
KT = KE
POLE_PAIRS = 6                              # 9N12P (user fact)
R_LL = asm("r_ll", 0.23, "motor line-to-line (lead-to-lead) resistance; retailer 0.17-0.29 ohm interpreted as lead-to-lead", (0.17, 0.29))
R_EXT = asm("r_ext", 0.010, "PCB traces + solder joints, line-to-line", (0.005, 0.02))
I0 = asm("i0", 0.30, "no-load (friction+iron) current, treated as constant torque Kt*I0", (0.2, 0.45))
L_LL = asm("l_ll", 6e-6, "motor line-to-line inductance (unknown; user range 3-15 uH)", (3e-6, 15e-6))
J_ROT = asm("j_rot", 2.2e-8, "rotor+prop inertia kg m^2 (bell ~0.5 g @ r=4.2 mm: 0.9e-8; 1219S-3 prop ~0.2 g blades: 1.3e-8)",
            (1.5e-8, 3.0e-8))
VF_BODY = asm("vf_body", 0.8, "DRV8311 FET body-diode forward voltage (not in datasheet)", (0.7, 1.0))
BEMF_SHAPE = asm("bemf_shape", "sinusoidal", "9N12P concentrated-winding motors have near-sinusoidal BEMF")
ADVANCE = asm("advance_deg", 15.0, "commutation timing advance (typical AM32/BLHeli setting)", (0.0, 22.5))
F_PWM = 48e3
T_DEAD = ds("drv_tdead_gnd")[0]
SR = ds("drv_sr_gnd")[1]

# ---- prop calibration ----
G = 9.80665
RHO = 1.20
V_TEST = asm("v_test", 4.20, "ESC input voltage in the published thrust tests (not stated by vendor)", (3.9, 4.35))
R_ESC_REF = asm("r_esc_ref", 0.030, "HS+LS resistance of the vendor's reference ESC used for thrust tests", (0.015, 0.05))
S_3BLADE = asm("s_3blade", 1.20, "torque-per-thrust ratio GF1219S-3 / Gemfan 31mm 2-blade (higher pitch & solidity)", (1.0, 1.4))

# published points (user facts)
HM_T, HM_I = 21.7e-3 * G, 2.86          # Happymodel SE0702 23k, Gemfan 31 mm 2-blade
HM40_T, HM40_I = 27.1e-3 * G, 4.0       # 40 mm 2-blade
BF_TMAX = 32.0e-3 * G                    # BetaFPV 0702SE II 23k, GF1219S-3 max thrust

QT_2B = KT * (HM_I - I0) / HM_T          # torque/thrust ratio [m] of the 31 mm 2-blade (from the HM point)


def calibrate_prop(s=S_3BLADE, i0=I0, r_ll=R_LL, v_test=V_TEST, r_esc=R_ESC_REF):
    """Return (kT, kQ, I_at_32g, rpm_at_32g, FM) for GF1219S-3 so that T=32 g at full throttle
    on the vendor's reference ESC at v_test, with torque/thrust = s * (2-blade value)."""
    qt2 = KT * (HM_I - i0) / HM_T
    qt3 = s * qt2
    i32 = i0 + qt3 * BF_TMAX / KT
    w32 = (v_test - i32 * (r_ll + r_esc)) / KE
    kT = BF_TMAX / w32 ** 2
    kQ = qt3 * kT
    p_shaft = kQ * w32 ** 3
    a_disk = math.pi * (0.031 / 2) ** 2
    p_ideal = BF_TMAX ** 1.5 / math.sqrt(2 * RHO * a_disk)
    return kT, kQ, i32, w32 * 60 / (2 * math.pi), p_ideal / p_shaft


KT_P, KQ_P, I_32G_REF, RPM_32G_REF, FM_P = calibrate_prop()
PROP_DIAM = 0.031


def rds_of_T(tj, which="typ"):
    """DRV8311 HS+LS RDS(on) at VM 3-6 V, linear between 25 C and 150 C datasheet points."""
    if which == "max":
        r25, r150 = ds("drv_rds_25_max"), ds("drv_rds_150_max")
    else:
        r25, r150 = ds("drv_rds_25_typ"), ds("drv_rds_150_typ")
    return r25 + (r150 - r25) * (tj - 25.0) / 125.0


# housekeeping current drawn from VBAT: LDO input (~= 3V0 load) + 4x DRV8311 IVM
I_3V0_TYP = asm("i_3v0_typ", 0.095, "3V0 load typical (see study 8 budget)")
I_HK = asm("i_hk", I_3V0_TYP + 4 * 0.011, "VBAT housekeeping: 3V0 load + 4x DRV8311 IVM (~11 mA each at 48 kHz)")


# --------------------------------------------------------------------------------------
# Averaged (DC-equivalent) motor + driver + battery operating point
# --------------------------------------------------------------------------------------
def motor_current(w, kQ=KQ_P, i0=I0):
    return i0 + kQ * w ** 2 / KT


def op_point(thrust=None, duty=None, soc_ocv=3.85, r_batt=R_BATT, r_ds=0.30, kT=KT_P, kQ=KQ_P,
             r_ll=R_LL, i0=I0, n_mot=4, f_pwm=F_PWM, t_dead=T_DEAD, i_hk=I_HK, r_pig=R_PIG):
    """Steady-state averaged operating point.  Give either per-motor (isolated) thrust [N] or duty.
    soc_ocv: battery open-circuit voltage [V]."""
    r_m = r_ll + R_EXT + r_ds
    r_path = r_batt + r_pig
    if thrust is not None:
        w = math.sqrt(thrust / kT)
        i = motor_current(w, kQ, i0)
        d = 0.5
        for _ in range(60):
            vm = soc_ocv - r_path * (n_mot * d * i + i_hk)
            v_need = KE * w + i * r_m
            d_new = v_need / vm + t_dead * f_pwm * (vm + 2 * VF_BODY) / vm
            d = 0.5 * d + 0.5 * min(d_new, 1.5)
        sat = d > 1.0
        if sat:
            return op_point(duty=1.0, soc_ocv=soc_ocv, r_batt=r_batt, r_ds=r_ds, kT=kT, kQ=kQ, r_ll=r_ll,
                            i0=i0, n_mot=n_mot, f_pwm=f_pwm, t_dead=t_dead, i_hk=i_hk, r_pig=r_pig) | {"saturated": True}
    else:
        d = duty
        dt_loss = 0.0 if d >= 1.0 else t_dead * f_pwm

        def resid(w):
            i = motor_current(w, kQ, i0)
            vm = soc_ocv - r_path * (n_mot * d * i + i_hk)
            return (d - dt_loss) * vm - 2 * VF_BODY * dt_loss - KE * w - i * r_m
        lo, hi = 0.0, soc_ocv / KE
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            if resid(mid) > 0:
                lo = mid
            else:
                hi = mid
        w = 0.5 * (lo + hi)
        i = motor_current(w, kQ, i0)
        vm = soc_ocv - r_path * (n_mot * d * i + i_hk)
    i_bat = n_mot * d * i + i_hk
    return {"w": w, "rpm": w * 60 / (2 * math.pi), "i_dc": i, "duty": d, "vm": vm, "i_bat": i_bat,
            "thrust": kT * w ** 2, "thrust_g": kT * w ** 2 / G * 1e3, "torque": kQ * w ** 2,
            "f_e": POLE_PAIRS * w / (2 * math.pi), "saturated": False,
            "p_in_motor": vm * d * i}


# --------------------------------------------------------------------------------------
# Switching-level 3-phase 6-step simulation (DRV8311 3x PWM mode)
# --------------------------------------------------------------------------------------
_SECT = [(0, 1, 2), (0, 2, 1), (1, 2, 0), (1, 0, 2), (2, 0, 1), (2, 1, 0)]   # (PWM-high, low, float)
BEMF_K = 1.0 / (3.0 / math.pi * math.sqrt(3.0))    # phase-peak BEMF = BEMF_K * KE * w_m  (=0.6046)


def sim_6step(vm, w_m, duty, l_ll=L_LL, r_ll=R_LL, r_ds=0.30, f_pwm=F_PWM, t_dead=T_DEAD, vf=VF_BODY,
              adv_deg=ADVANCE, n_cycles=2, settle_cycles=1.5, dt=25e-9, record=True, theta0=0.0,
              pwm_phase=0.0, sr=SR, rec_decim=1):
    """Fixed-step simulation of a star-connected 3-phase motor with sinusoidal BEMF driven by a
    DRV8311 in 3x PWM mode (complementary PWM with handshake dead time on the 'high' phase,
    low-side ON on the 'low' phase, Hi-Z on the floating phase, body-diode clamping).
    Speed is held constant (rotor inertia >> electrical time constants).

    Returns dict with averaged results over n_cycles electrical cycles (after settle_cycles)
    and (optionally) recorded waveforms.
    """
    L = l_ll / 2.0
    rph = (r_ll + R_EXT) / 2.0
    rf = r_ds / 2.0
    we = POLE_PAIRS * w_m
    epk = BEMF_K * KE * w_m
    tpwm = 1.0 / f_pwm
    te = 2 * math.pi / we
    t_settle = settle_cycles * te
    t_end = t_settle + n_cycles * te
    n = int(t_end / dt)
    adv = math.radians(adv_deg)
    twopi3 = 2 * math.pi / 3
    pi3 = math.pi / 3
    pi6 = math.pi / 6
    d_on = duty * tpwm
    full_on = duty >= 1.0
    t_tr = vm / sr
    sin = math.sin

    ia = ib = ic = 0.0
    nrec = n // rec_decim + 1
    if record:
        R = {k: np.empty(nrec) for k in ("t", "ia", "ib", "ic", "va", "vb", "vc", "ea", "ibus", "vn", "sect")}
    # accumulators (measurement window only)
    acc_T = acc_pcond = acc_pdiode = acc_psw = acc_ibus = acc_ibus2 = acc_pcu = 0.0
    acc_ia2 = 0.0
    acc_n = 0
    ipk = 0.0
    prev_on = None
    prev_sect = None
    demag_t = []
    demag_start = None
    irec = 0
    i_ph = [0.0, 0.0, 0.0]
    for k in range(n):
        t = k * dt
        th = we * t + theta0
        ea = epk * sin(th)
        eb = epk * sin(th - twopi3)
        ec = epk * sin(th + twopi3)
        e = (ea, eb, ec)
        sect = int(((th + adv - pi6) % (2 * math.pi)) / pi3)
        hi, lo, fl = _SECT[sect]
        # PWM state of the high phase
        if full_on:
            st_hi = 1  # HS on
            on = True
        else:
            tp = (t + pwm_phase) % tpwm
            on = tp < d_on
            if tp < t_dead or (d_on <= tp < d_on + t_dead):
                st_hi = 0  # dead time -> diodes
            else:
                st_hi = 1 if on else -1  # 1: HS on, -1: LS on
        i_ph[0], i_ph[1], i_ph[2] = ia, ib, ic
        # per-phase source voltage and resistance; mode: 1 HS, -1 LS, 2 diode, 0 floating
        vs = [0.0, 0.0, 0.0]
        rs = [0.0, 0.0, 0.0]
        md = [0, 0, 0]
        for x in range(3):
            if x == hi:
                s = st_hi
            elif x == lo:
                s = -1
            else:
                s = 0
            if s == 1:
                vs[x] = vm; rs[x] = rf + rph; md[x] = 1
            elif s == -1:
                vs[x] = 0.0; rs[x] = rf + rph; md[x] = -1
            else:
                ix = i_ph[x]
                if ix > 0.0:
                    vs[x] = -vf; rs[x] = rph; md[x] = 2
                elif ix < 0.0:
                    vs[x] = vm + vf; rs[x] = rph; md[x] = 2
                else:
                    md[x] = 0
        # neutral voltage from conducting phases
        for _pass in range(2):
            num = 0.0
            nc = 0
            for x in range(3):
                if md[x] != 0:
                    num += vs[x] - rs[x] * i_ph[x] - e[x]
                    nc += 1
            vn = num / nc if nc else 0.0
            changed = False
            for x in range(3):
                if md[x] == 0:
                    vx = vn + e[x]
                    if vx > vm + vf:
                        vs[x] = vm + vf; rs[x] = rph; md[x] = 2; changed = True
                    elif vx < -vf:
                        vs[x] = -vf; rs[x] = rph; md[x] = 2; changed = True
            if not changed:
                break
        if nc < 2:
            # nothing can conduct
            new = [0.0, 0.0, 0.0]
        else:
            new = [0.0, 0.0, 0.0]
            for x in range(3):
                if md[x] != 0:
                    new[x] = i_ph[x] + dt * (vs[x] - rs[x] * i_ph[x] - e[x] - vn) / L
            # diode current zero crossing -> clamp to zero and rebalance
            for x in range(3):
                if md[x] == 2 and (new[x] * i_ph[x] < 0.0 or (i_ph[x] == 0.0 and
                                                             ((vs[x] > vm and new[x] > 0) or (vs[x] < 0 and new[x] < 0)))):
                    new[x] = 0.0
            ssum = new[0] + new[1] + new[2]
            if ssum != 0.0:
                cond = [x for x in range(3) if md[x] in (1, -1) or (md[x] == 2 and new[x] != 0.0)]
                if cond:
                    corr = ssum / len(cond)
                    for x in cond:
                        new[x] -= corr
        # terminal voltages for recording
        if t >= t_settle:
            # losses & torque (use values at start of step)
            tq = (ea * i_ph[0] + eb * i_ph[1] + ec * i_ph[2]) / w_m
            pc = 0.0
            pd = 0.0
            ib_ = 0.0
            for x in range(3):
                ix = i_ph[x]
                if md[x] == 1 or md[x] == -1:
                    pc += ix * ix * rf
                    if md[x] == 1:
                        ib_ += ix
                elif md[x] == 2:
                    pd += abs(ix) * vf
                    if vs[x] > vm:
                        ib_ += ix
            acc_T += tq
            acc_pcond += pc
            acc_pdiode += pd
            acc_ibus += ib_
            acc_ibus2 += ib_ * ib_
            acc_pcu += (ia * ia + ib * ib + ic * ic) * rph
            acc_ia2 += ia * ia
            acc_n += 1
            a = max(abs(ia), abs(ib), abs(ic))
            if a > ipk:
                ipk = a
            if not full_on and prev_on is not None and on != prev_on:
                acc_psw += 0.5 * vm * abs(i_ph[hi]) * t_tr / dt  # energy -> power*dt later
            if prev_sect is not None and sect != prev_sect:
                demag_start = t
            if demag_start is not None and md[fl] == 0:
                demag_t.append(t - demag_start)
                demag_start = None
        prev_on = on
        prev_sect = sect
        if record and (k % rec_decim == 0):
            vt = [0.0, 0.0, 0.0]
            for x in range(3):
                if md[x] == 1:
                    vt[x] = vm - rf * i_ph[x]
                elif md[x] == -1:
                    vt[x] = -rf * i_ph[x]
                elif md[x] == 2:
                    vt[x] = vs[x]
                else:
                    vt[x] = vn + e[x]
            ibr = 0.0
            for x in range(3):
                if md[x] == 1 or (md[x] == 2 and vs[x] > vm):
                    ibr += i_ph[x]
            R["t"][irec] = t; R["ia"][irec] = ia; R["ib"][irec] = ib; R["ic"][irec] = ic
            R["va"][irec] = vt[0]; R["vb"][irec] = vt[1]; R["vc"][irec] = vt[2]; R["ea"][irec] = ea
            R["ibus"][irec] = ibr; R["vn"][irec] = vn; R["sect"][irec] = sect
            irec += 1
        ia, ib, ic = new
    res = {
        "vm": vm, "w_m": w_m, "rpm": w_m * 60 / (2 * math.pi), "duty": duty, "l_ll": l_ll, "r_ds": r_ds,
        "f_pwm": f_pwm, "f_e": we / (2 * math.pi), "f_comm": 6 * we / (2 * math.pi),
        "torque": acc_T / acc_n, "p_cond": acc_pcond / acc_n, "p_diode": acc_pdiode / acc_n,
        "p_sw": acc_psw * dt / (acc_n * dt), "i_bus_avg": acc_ibus / acc_n,
        "i_bus_rms": math.sqrt(acc_ibus2 / acc_n), "p_cu_motor": acc_pcu / acc_n,
        "i_ph_rms": math.sqrt(acc_ia2 / acc_n), "i_ph_pk": ipk, "epk": epk,
        "demag_max": max(demag_t) if demag_t else 0.0,
        "demag_mean": float(np.mean(demag_t)) if demag_t else 0.0,
    }
    res["p_drv_total"] = res["p_cond"] + res["p_diode"] + res["p_sw"]
    if record:
        for kk in R:
            R[kk] = R[kk][:irec]
        res["wave"] = R
    return res


def sim_find_duty(vm, w_m, t_load, d0=0.5, **kw):
    """Secant search for the duty that gives mean electromagnetic torque == t_load at speed w_m."""
    kw = dict(kw)
    kw["record"] = False
    d_a = min(max(d0, 0.05), 0.98)
    r_a = sim_6step(vm, w_m, d_a, **kw)
    f_a = r_a["torque"] - t_load
    d_b = min(d_a + 0.05, 0.99) if f_a < 0 else max(d_a - 0.05, 0.02)
    r_b = sim_6step(vm, w_m, d_b, **kw)
    f_b = r_b["torque"] - t_load
    for _ in range(8):
        if abs(f_b) < 0.003 * t_load:
            break
        if f_b == f_a:
            break
        d_c = d_b - f_b * (d_b - d_a) / (f_b - f_a)
        d_c = min(max(d_c, 0.02), 1.0)
        d_a, f_a = d_b, f_b
        r_b = sim_6step(vm, w_m, d_c, **kw)
        d_b, f_b = d_c, r_b["torque"] - t_load
        if d_c >= 1.0 and f_b < 0:
            break
    return d_b, r_b


def sim_full_throttle(soc_ocv=3.85, r_batt=R_BATT, r_ds=0.30, l_ll=L_LL, f_pwm=F_PWM, n_mot=4, **kw):
    """D = 1: find the speed at which mean EM torque equals prop + friction torque, with VM from
    the battery equation (4 motors identical).  Returns (op_point_avg, sim_result)."""
    avg = op_point(duty=1.0, soc_ocv=soc_ocv, r_batt=r_batt, r_ds=r_ds, f_pwm=f_pwm, n_mot=n_mot)
    w = avg["w"]
    r_path = r_batt + R_PIG

    def mismatch(w_try):
        i_guess = motor_current(w_try)
        vm = soc_ocv - r_path * (n_mot * i_guess + I_HK)
        for _ in range(2):
            r = sim_6step(vm, w_try, 1.0, l_ll=l_ll, r_ds=r_ds, f_pwm=f_pwm, record=False, **kw)
            vm = soc_ocv - r_path * (n_mot * r["i_bus_avg"] + I_HK)
        r = sim_6step(vm, w_try, 1.0, l_ll=l_ll, r_ds=r_ds, f_pwm=f_pwm, record=False, **kw)
        return r["torque"] - (KQ_P * w_try ** 2 + KT * I0), r, vm
    wa = w
    fa, ra, vma = mismatch(wa)
    wb = w * (0.97 if fa < 0 else 1.03)
    fb, rb, vmb = mismatch(wb)
    for _ in range(6):
        if abs(fb) < 0.003 * (KQ_P * wb ** 2) or fb == fa:
            break
        wc = wb - fb * (wb - wa) / (fb - fa)
        wa, fa = wb, fb
        wb = wc
        fb, rb, vmb = mismatch(wb)
    rec = sim_6step(vmb, wb, 1.0, l_ll=l_ll, r_ds=r_ds, f_pwm=f_pwm, record=True, **kw)
    return avg, rec


def sim_at_thrust(thrust, soc_ocv=3.85, r_batt=R_BATT, r_ds=0.30, l_ll=L_LL, f_pwm=F_PWM, n_mot=4,
                  record=True, **kw):
    """Constant speed defined by the required (isolated) thrust; duty found by secant so that
    EM torque = prop + friction torque.  VM from the battery equation (iterated once)."""
    w = math.sqrt(thrust / KT_P)
    t_load = KQ_P * w ** 2 + KT * I0
    avg = op_point(thrust=thrust, soc_ocv=soc_ocv, r_batt=r_batt, r_ds=r_ds, f_pwm=f_pwm, n_mot=n_mot)
    if avg.get("saturated"):
        return avg, None
    vm = avg["vm"]
    d = avg["duty"]
    r_path = r_batt + R_PIG
    for _ in range(2):
        d, r = sim_find_duty(vm, w, t_load, d0=d, l_ll=l_ll, r_ds=r_ds, f_pwm=f_pwm, **kw)
        vm_new = soc_ocv - r_path * (n_mot * r["i_bus_avg"] + I_HK)
        if abs(vm_new - vm) < 2e-3:
            vm = vm_new
            break
        vm = vm_new
    if record:
        r = sim_6step(vm, w, d, l_ll=l_ll, r_ds=r_ds, f_pwm=f_pwm, record=True, **kw)
    return avg, r


def fmt(x, n=3):
    return float(f"{x:.{n}g}")


if __name__ == "__main__":
    print("KE = %.4e V s/rad" % KE)
    print("Prop GF1219S-3 calibration: kT=%.3e N s^2, kQ=%.3e N m s^2, I@32g(ref ESC)=%.2f A, rpm@32g=%.0f, FM=%.2f"
          % (KT_P, KQ_P, I_32G_REF, RPM_32G_REF, FM_P))
    print("QT_2B = %.3e m" % QT_2B)
    for name, auw in (("260", M_DRY + 7.6), ("300", M_DRY + 8.3)):
        t_m = auw * 1e-3 * G / (4 * OVERLAP)
        op = op_point(thrust=t_m)
        print(name, "AUW %.1f g, hover per-motor isolated thrust %.2f g" % (auw, t_m / G * 1e3),
              {k: round(v, 3) if isinstance(v, float) else v for k, v in op.items()})
    op = op_point(duty=1.0)
    print("full throttle:", {k: round(v, 3) if isinstance(v, float) else v for k, v in op.items()})
