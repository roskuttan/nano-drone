"""
Study 5 - Power distribution network.

(a) 3V0 rail impedance vs frequency at every load point + time-domain load-step response.
    Behavioural TPS7A20 output impedance  Zldo = R_ol || (R_a + jw L_a)  (R_a from load regulation,
    unity-gain frequency assumed), star distribution on the L4 3V0 island, MLCCs as series R-L-C with
    DC-bias derating, ferrite beads as R_dc + (L || R_p || C_p).
    Decoupling (design update): ESC MCU 100 nF + 1 uF (0201) each (+100 nF on DRV8311 CSAREF);
    FC 3x100 nF + 4.7 uF, FC VDDA 1 uF + 100 nF behind a ferrite; radio 2x100 nF + 4.7 uF behind a
    ferrite; IMU 2x100 nF; LDO output 2.2 uF.
(b) VBAT ripple from 4 DRV8311 bridges switching at 48 kHz (bus current per driver from the
    switching-level model, folded to one PWM period) - synchronous, ideally interleaved and random
    PWM phase; harmonic (phasor) solution of the battery + bulk-MLCC network.

Run:  python sim_05_pdn.py
"""
import math
import simlib as S
import numpy as np
from simlib import plt, PAL, CRIT, INK2

V3 = 3.0
# ------------------------------------------------------------------ capacitor library (assumptions)
CAPS = S.asm("pdn_caps", {
    "100n_0402": dict(c=100e-9, ret=0.90, esr=0.030, esl=0.40e-9),
    "100n_0201": dict(c=100e-9, ret=0.80, esr=0.040, esl=0.30e-9),
    "1u_0201": dict(c=1e-6, ret=0.45, esr=0.020, esl=0.30e-9),
    "1u_0402": dict(c=1e-6, ret=0.65, esr=0.015, esl=0.40e-9),
    "2u2_0402": dict(c=2.2e-6, ret=0.55, esr=0.010, esl=0.40e-9),
    "4u7_0402": dict(c=4.7e-6, ret=0.45, esr=0.008, esl=0.40e-9),
}, "MLCC models at 3.0 V DC bias: retained C fraction, ESR, mounted ESL (generic X5R/X7R values)")
FB = S.asm("ferrite", dict(rdc=0.10, l=0.30e-6, rp=150.0, cp=1e-12),
           "generic 0402 ferrite bead ~120 ohm @100 MHz (part not specified): Rdc + (L || Rp || Cp)")
LDO = S.asm("ldo_zout_model", dict(r_a=S.ds("ldo_load_reg") / 0.3, r_ol=2.0, f_u=1.0e6),
            "TPS7A20 Zout behavioural: R_a = load-reg/300 mA (datasheet), R_ol = 2 ohm and UGF 1 MHz assumed")
BR = S.asm("pdn_branches", {"FC": (0.6e-9, 0.002), "RADIO": (1.0e-9, 0.003), "ESC": (1.2e-9, 0.004),
                            "IMU": (0.8e-9, 0.003)},
           "3V0 island (L4 over L5 GND, 0.16 mm) + via inductance / resistance from LDO to each load")
LOADS = S.asm("i_3v0_loads", {"FC": 0.035, "VDDA": 0.001, "RADIO": 0.012, "ESC": 0.008 + 0.0017, "IMU": 0.00065},
              "3V0 static loads (typ): G431 170 MHz ~35 mA (G474 Table 21 proxy 29.5 mA + periph.), "
              "radio RX LDO-mode avg 12 mA, G071 8 mA + CSAREF 1.7 mA, IMU 0.65 mA")
STEP = S.asm("i_3v0_step", {"RADIO": 0.048, "FC": 0.020},
             "load step: SX1280 TX +12.5 dBm in LDO mode (24 mA x2 per SX1280 s14.7.1) + 20 mA FC wake")
Z_T1 = 0.01 * V3 / (STEP["RADIO"] + STEP["FC"])
Z_T3 = 0.03 * V3 / (STEP["RADIO"] + STEP["FC"])


def zcap(kind, w):
    p = CAPS[kind]
    return p["esr"] + 1j * w * p["esl"] + 1.0 / (1j * w * p["c"] * p["ret"])


def zpar(*zs):
    return 1.0 / sum(1.0 / z for z in zs)


def zfb(w):
    zl = 1j * w * FB["l"]
    zc = 1.0 / (1j * w * FB["cp"])
    return FB["rdc"] + zpar(zl, FB["rp"] + 0j, zc)


def zldo(w):
    r_a, r_ol, f_u = LDO["r_a"], LDO["r_ol"], LDO["f_u"]
    t0 = r_ol / r_a
    l_a = r_a / (2 * math.pi * f_u / t0)
    return zpar(r_ol + 0j, r_a + 1j * w * l_a)


NODES = ["LDO", "FC", "VDDA", "RADIO", "ESC1", "ESC2", "ESC3", "ESC4", "IMU"]
NI = {n: i for i, n in enumerate(NODES)}


def ymatrix(w):
    """w: array of angular frequencies -> Y[nf, 9, 9]."""
    nf = w.size
    Y = np.zeros((nf, len(NODES), len(NODES)), dtype=complex)

    def shunt(n, z):
        i = NI[n]
        Y[:, i, i] += 1.0 / z

    def branch(a, b, z):
        i, j = NI[a], NI[b]
        y = 1.0 / z
        Y[:, i, i] += y
        Y[:, j, j] += y
        Y[:, i, j] -= y
        Y[:, j, i] -= y
    shunt("LDO", zldo(w))
    shunt("LDO", zcap("2u2_0402", w))
    branch("LDO", "FC", BR["FC"][1] + 1j * w * BR["FC"][0])
    for _ in range(3):
        shunt("FC", zcap("100n_0402", w))
    shunt("FC", zcap("4u7_0402", w))
    branch("FC", "VDDA", zfb(w))
    shunt("VDDA", zcap("1u_0402", w))
    shunt("VDDA", zcap("100n_0402", w))
    branch("LDO", "RADIO", BR["RADIO"][1] + 1j * w * BR["RADIO"][0] + zfb(w))
    for _ in range(2):
        shunt("RADIO", zcap("100n_0402", w))
    shunt("RADIO", zcap("4u7_0402", w))
    for k in range(1, 5):
        n = f"ESC{k}"
        branch("LDO", n, BR["ESC"][1] + 1j * w * BR["ESC"][0])
        shunt(n, zcap("100n_0201", w))
        shunt(n, zcap("1u_0201", w))
        shunt(n, zcap("100n_0402", w))          # DRV8311 CSAREF 0.1 uF (datasheet recommendation)
    branch("FC", "IMU", BR["IMU"][1] + 1j * w * BR["IMU"][0])
    for _ in range(2):
        shunt("IMU", zcap("100n_0402", w))
    return Y


def part_a(res):
    f = np.logspace(3, 8.5, 1200)
    w = 2 * np.pi * f
    Z = np.linalg.inv(ymatrix(w))
    zself = {n: np.abs(Z[:, NI[n], NI[n]]) for n in NODES}
    band = f <= 20e6
    out = {}
    for n in NODES:
        zz = zself[n]
        k = np.argmax(zz * band)
        out[n] = {"Zmax_below_20MHz_ohm": float(zz[band].max()), "f_at_Zmax_Hz": float(f[k]),
                  "pass_3pct_target": bool(zz[band].max() < Z_T3), "pass_1pct_target": bool(zz[band].max() < Z_T1)}
        print(f"  Z({n:5s}) max {zz[band].max():.3f} ohm @ {f[k]/1e3:.0f} kHz  "
              f"(targets 1%: {Z_T1:.2f}, 3%: {Z_T3:.2f})")
    res["targets_ohm"] = {"1pct": Z_T1, "3pct": Z_T3, "step_A": STEP}
    res["impedance"] = out

    # ---- time-domain load step via FFT of the periodic stimulus ----
    dt = 5e-9
    N = 2 ** 15
    t = np.arange(N) * dt
    def ramp(t0, tr, amp, t1):
        x = np.clip((t - t0) / tr, 0, 1) - np.clip((t - t1) / tr, 0, 1)
        return amp * x
    i_src = np.zeros((len(NODES), N))
    i_src[NI["RADIO"]] = ramp(10e-6, 2e-6, STEP["RADIO"], 90e-6)
    i_src[NI["FC"]] = ramp(10e-6, 0.1e-6, STEP["FC"], 90e-6)
    fk = np.fft.rfftfreq(N, dt)
    wk = 2 * np.pi * fk
    wk[0] = 2 * np.pi * 1e-3
    Zk = np.linalg.inv(ymatrix(wk))
    Ik = np.fft.rfft(i_src, axis=1)
    Vk = np.einsum("fij,jf->if", Zk, Ik)
    v = -np.fft.irfft(Vk, n=N, axis=1)       # voltage deviation (load current pulls node down)
    # static loads DC drop
    i_dc = np.zeros(len(NODES))
    for n, ii in LOADS.items():
        if n == "ESC":
            for k in range(1, 5):
                i_dc[NI[f"ESC{k}"]] = ii
        else:
            i_dc[NI[n]] = ii
    z0 = np.linalg.inv(ymatrix(np.array([2 * np.pi * 1e-3])))[0].real
    v_dc = -(z0 @ i_dc)
    step_res = {}
    for n in NODES:
        dv = v[NI[n]]
        step_res[n] = {"dV_min_mV": float(dv.min() * 1e3), "dV_max_mV": float(dv.max() * 1e3),
                       "static_drop_mV": float(v_dc[NI[n]] * 1e3)}
    res["load_step"] = step_res
    for n in ("RADIO", "FC", "VDDA", "ESC1"):
        print(f"  step: {n:5s} dV min {step_res[n]['dV_min_mV']:.1f} mV  max {step_res[n]['dV_max_mV']:.1f} mV  "
              f"static {step_res[n]['static_drop_mV']:.1f} mV")
    # noise transfer FC node -> VDDA through the ferrite (peaking of the bead + 1 uF filter)
    h_vdda = np.abs(Z[:, NI["VDDA"], NI["FC"]] / Z[:, NI["FC"], NI["FC"]])
    k = np.argmax(h_vdda)
    res["fc_to_vdda_transfer"] = {"peak_gain": float(h_vdda[k]), "f_peak_kHz": float(f[k] / 1e3),
                                  "atten_at_10MHz_dB": float(20 * np.log10(np.interp(10e6, f, h_vdda)))}
    print(f"  FC->VDDA transfer: peak x{h_vdda[k]:.2f} at {f[k]/1e3:.0f} kHz, "
          f"{20*np.log10(np.interp(10e6, f, h_vdda)):.1f} dB at 10 MHz")
    res["radio_ferrite_LC_resonance_kHz"] = 1 / (2 * math.pi * math.sqrt(FB["l"] * 4.7e-6 * 0.45)) / 1e3

    # plots
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for n, col in zip(["LDO", "FC", "VDDA", "RADIO", "ESC1", "IMU"], PAL):
        ax.loglog(f, zself[n], color=col, label=n)
    S.hline_limit(ax, Z_T3, f"target 3% (90 mV / {1e3*(STEP['RADIO']+STEP['FC']):.0f} mA) = {Z_T3:.2f} ohm")
    S.hline_limit(ax, Z_T1, f"target 1% (ADC ref) = {Z_T1:.2f} ohm", ls=":")
    ax.axvline(20e6, color=INK2, lw=0.8, ls=":")
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("|Z| seen by the load (ohm)")
    ax.set_title("3V0 rail self-impedance at each load (TPS7A2030 + MLCCs + ferrites)")
    ax.set_ylim(5e-3, 20)
    ax.legend(loc="lower left", ncol=3)
    S.save(fig, "s5a_3v0_impedance.png")

    fig, ax = plt.subplots(figsize=(9, 4.0))
    m = (t > 5e-6) & (t < 120e-6)
    for n, col in zip(["LDO", "FC", "VDDA", "RADIO", "ESC1"], PAL):
        ax.plot(t[m] * 1e6, v[NI[n]][m] * 1e3, color=col, label=n)
    ax.set_xlabel("time (us)  [radio TX +48 mA (2 us ramp) and FC +20 mA at 10 us, released at 90 us]")
    ax.set_ylabel("3V0 deviation (mV)")
    ax.set_title("3V0 load-step response at each load")
    ax.legend(loc="lower right", ncol=5)
    S.save(fig, "s5a_3v0_step.png")


# ======================================================================= part (b)
L_PL = 0.8e-9
R_PL = 0.002


def bus_current_period(thrust, f_pwm=S.F_PWM, l_ll=S.L_LL):
    """PWM-period-folded average bus current of one DRV8311 from the switching model."""
    avg, r = S.sim_at_thrust(thrust, soc_ocv=3.90, r_ds=S.rds_of_T(80.0), l_ll=l_ll, f_pwm=f_pwm, record=True,
                             n_cycles=2)
    wv = r["wave"]
    keep = wv["t"] >= 1.5 / r["f_e"]            # drop the start-up (settling) part
    tt, ib = wv["t"][keep], wv["ibus"][keep]
    tp = 1 / f_pwm
    dt = wv["t"][1] - wv["t"][0]
    nper = int(round(tp / dt))
    ph = np.floor((tt % tp) / dt).astype(int)
    ph = np.clip(ph, 0, nper - 1)
    acc = np.bincount(ph, weights=ib, minlength=nper)
    cnt = np.bincount(ph, minlength=nper)
    return acc / np.maximum(cnt, 1), dt, r


def vbat_network(fk, phases, i_k, r_path, extra=None):
    """phasor solve: nodes [P, D1..D4]; returns V_P(f), V_D(f) [4], I_cap22 at D1 and P.
    extra: optional (C, ESR, ESL) bulk capacitor added at the battery pads."""
    w = 2 * np.pi * fk
    w = np.where(w == 0, 2 * np.pi * 1e-3, w)
    z22 = S.ASM["c22_esr"]["value"] + 1j * w * S.ASM["esl_0603"]["value"] + \
        1 / (1j * w * 22e-6 * float(S.c22_retention(3.9)))
    z100 = 0.03 + 1j * w * 0.4e-9 + 1 / (1j * w * 90e-9)
    z1u = 0.015 + 1j * w * 0.4e-9 + 1 / (1j * w * 0.55e-6)     # TPS7A20 input cap
    zb = r_path + 1j * w * S.L_PIG
    zpl = R_PL + 1j * w * L_PL
    nf = fk.size
    Y = np.zeros((nf, 5, 5), dtype=complex)
    Y[:, 0, 0] += 1 / zb + 2 / z22 + 1 / z1u
    if extra is not None:
        Y[:, 0, 0] += 1 / (extra[1] + 1j * w * extra[2] + 1 / (1j * w * extra[0]))
    for k in range(1, 5):
        y = 1 / zpl
        Y[:, 0, 0] += y
        Y[:, k, k] += y + 1 / z22 + 1 / z100
        Y[:, 0, k] -= y
        Y[:, k, 0] -= y
    I = np.zeros((nf, 5), dtype=complex)
    for k in range(4):
        I[:, k + 1] = -i_k * np.exp(-2j * np.pi * fk / fk[1] * phases[k]) if nf > 1 else -i_k
    V = np.linalg.solve(Y, I[..., None])[..., 0]
    icap_d1 = V[:, 1] / z22
    icap_p = V[:, 0] / z22
    return V, icap_d1, icap_p


def ripple_case(ip, dt, phases, r_path=S.R_BATT + S.R_PIG, extra=None):
    n = ip.size
    Ik = np.fft.rfft(ip) / n                    # complex amplitudes (one-sided, DC at 0)
    fk = np.fft.rfftfreq(n, dt)
    V, icd, icp = vbat_network(fk, phases, Ik, r_path, extra)
    # back to time: V_k = Z(f_k) c_k on the one-sided grid; irfft(n * V) restores the real waveform
    vt = np.array([np.fft.irfft(V[:, j] * n, n=n) for j in range(5)])
    ic_d1 = np.fft.irfft(icd * n, n=n)
    ic_p = np.fft.irfft(icp * n, n=n)
    ac = vt - vt.mean(axis=1, keepdims=True)
    return {"vpp_D1": float(np.ptp(ac[1])), "vpp_P": float(np.ptp(ac[0])), "v_D1": ac[1], "v_P": ac[0],
            "icap22_D1_rms": float(np.sqrt(np.mean((ic_d1 - ic_d1.mean()) ** 2))),
            "icap22_P_rms": float(np.sqrt(np.mean((ic_p - ic_p.mean()) ** 2)))}


def part_b(res):
    auw = (S.M_DRY + 7.6) * 1e-3 * S.G
    pts = {"hover": auw / (4 * S.OVERLAP), "7 g/motor": 7e-3 * S.G, "10 g/motor": 10e-3 * S.G}
    rng = np.random.default_rng(1)
    out = {}
    waves = {}
    for name, thr in pts.items():
        ip, dt, r = bus_current_period(thr)
        sync = ripple_case(ip, dt, [0, 0, 0, 0])
        inter = ripple_case(ip, dt, [0, 0.25, 0.5, 0.75])
        rnd = [ripple_case(ip, dt, list(rng.random(4)))["vpp_D1"] for _ in range(200)]
        bulk = ripple_case(ip, dt, [0, 0, 0, 0], extra=(100e-6, 0.025, 1.0e-9))
        out[name] = {"duty": r["duty"], "I_bus_avg_A": float(ip.mean()), "I_bus_ac_rms_A": float(ip.std()),
                     "vpp_D1_sync_plus_100uF_polymer_mV": bulk["vpp_D1"] * 1e3,
                     "vpp_D1_sync_mV": sync["vpp_D1"] * 1e3, "vpp_P_sync_mV": sync["vpp_P"] * 1e3,
                     "vpp_D1_interleaved_mV": inter["vpp_D1"] * 1e3,
                     "vpp_D1_random_median_mV": float(np.median(rnd)) * 1e3,
                     "vpp_D1_random_p95_mV": float(np.percentile(rnd, 95)) * 1e3,
                     "icap22_driver_rms_sync_A": sync["icap22_D1_rms"],
                     "icap22_pads_rms_sync_A": sync["icap22_P_rms"],
                     "ldo_3v0_ripple_uV_at_60dB_PSRR": sync["vpp_P"] * 1e6 * 10 ** (-60 / 20)}
        waves[name] = (dt, sync, inter, ip)
        print(f"  VBAT ripple {name:10s} D={r['duty']:.2f} Ibus {ip.mean():.2f} A (ac {ip.std():.2f} A rms): "
              f"sync {sync['vpp_D1']*1e3:.0f} mVpp (pads {sync['vpp_P']*1e3:.0f}), interleaved "
              f"{inter['vpp_D1']*1e3:.0f}, random p95 {out[name]['vpp_D1_random_p95_mV']:.0f} mVpp; "
              f"I_rms 22uF@drv {sync['icap22_D1_rms']:.2f} A")
    res["vbat_ripple"] = out

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.0))
    dt, sync, inter, ip = waves["10 g/motor"]
    tt = np.arange(ip.size) * dt * 1e6
    ax = axs[0]
    ax.plot(np.concatenate([tt, tt + tt[-1] + dt * 1e6]), np.tile(sync["v_D1"] * 1e3, 2), color=PAL[0],
            label="synchronous PWM (worst)")
    ax.plot(np.concatenate([tt, tt + tt[-1] + dt * 1e6]), np.tile(inter["v_D1"] * 1e3, 2), color=PAL[1],
            label="90-deg interleaved")
    ax.set_xlabel("time (us), two 48 kHz periods")
    ax.set_ylabel("VBAT ripple at a driver VM pin (mV)")
    ax.set_title("VBAT ripple, 10 g/motor (D ~ 0.75), 4 drivers")
    ax.legend(loc="lower right")
    ax = axs[1]
    names = list(out.keys())
    x = np.arange(len(names))
    wdt = 0.25
    for j, (key, lab) in enumerate((("vpp_D1_sync_mV", "synchronous"), ("vpp_D1_random_p95_mV", "random phase, 95th pct"),
                                    ("vpp_D1_interleaved_mV", "ideal interleave"))):
        ax.bar(x + (j - 1) * wdt, [out[n][key] for n in names], wdt, color=PAL[j], edgecolor=S.SURF, lw=2, label=lab)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("ripple pk-pk at driver VM (mV)")
    ax.set_title("VBAT ripple vs operating point")
    ax.legend(loc="upper left")
    fig.tight_layout()
    S.save(fig, "s5b_vbat_ripple.png")


def main():
    print("Study 5: PDN")
    res = {"a_3v0": {}, "b_vbat": {}}
    part_a(res["a_3v0"])
    part_b(res["b_vbat"])
    S.write_results("study5_pdn", res)


if __name__ == "__main__":
    main()
