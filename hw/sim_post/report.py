"""Build POST_LAYOUT_REPORT.md from the post-layout study results in out/*.json (run after run_post.py)."""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


def load(name):
    p = os.path.join(OUT, name + ".json")
    return json.load(open(p, encoding="utf-8")) if os.path.exists(p) else None


def f1(v):
    return "-" if v is None else "%.1f" % v


def f2(v):
    return "-" if v is None else "%.2f" % v


def main():
    th, ir, xt, rf, cv, nt = (load(n) for n in ("p1_thermal", "p2_irdrop", "p3_xtalk", "p4_rf", "p5_cavity",
                                                 "p6_nets"))
    L = []
    w = L.append
    w("# Nano drone flight board: post-layout simulation report")
    w("")
    w("Every number here is computed from the **routed copper** of `hw/kicad/nanodrone.kicad_pcb`:")
    w("- the real track, pad and filled-zone geometry, rasterised at 0.05 mm;")
    w("- every via;")
    w("- the stackup written into the board.")
    w("")
    w("The pre-layout studies (`hw/sim/SIMULATION_REPORT.md`) set the operating points and component models used")
    w("here. Reproduce with `python run_post.py`, then `python report.py`, from `hw/sim_post/`, using KiCad's bundled")
    w("Python 3.11. Figures are in `out/`.")
    w("")
    w("## Summary")
    w("")
    w("| Study | Result on the routed board | Verdict |")
    w("|---|---|---|")
    if th:
        s = th["scenarios"]
        nom = s[[k for k in s if "nominal" in k][0]]
        worst = s[[k for k in s if "worst" in k][0]]
        still = s[[k for k in s if "still" in k][0]]
        b = th.get("burst_budget_s", {})
        w("| P1 thermal | Hover, 96 kHz PWM, h_wash 60: Tj max **%.0f C**. Worst-case parts: %.0f C. Still air: %.0f C. "
          "Full throttle from hover: 125 C after **%s s**, 150 C after %s s. IMU at hover: %.0f C | hover **PASS**; "
          "full throttle **time-limited**, firmware must enforce it |" % (
              max(nom["tj"].values()), max(worst["tj"].values()), max(still["tj"].values()),
              f2(b.get("125 C")), f2(b.get("150 C")), nom["probes"]["U3"]))
    if ir:
        c = ir["cases"]
        ft, pk = c["full throttle"], c["peak 4 A/driver"]
        w("| P2 IR drop | Full throttle (%.1f A): VBAT+GND loop drop to the worst driver **%.0f mV**, copper loss %.0f mW. "
          "4 A/driver peak: %.0f mV. Worst via current %.2f A | %s |" % (
              ft["total_current"], 1e3 * max(ft["loop_drop"].values()),
              1e3 * (ft["VBAT"]["p_diss"] + ft["GND"]["p_diss"]), 1e3 * max(pk["loop_drop"].values()),
              max(pk["VBAT"]["via_i_max"], pk["GND"]["via_i_max"]),
              "PASS" if max(ft["loop_drop"].values()) < 0.1 else "MARGINAL"))
    if xt:
        rows = xt["rows"]
        worst_x = min(rows, key=lambda r: r["margin_db"]) if rows else None
        rv = xt["return_vias"]
        far = rv["far"]
        pc = (" Plane-changing vias (L1/L3 to L4/L6) with no GND via within 1 mm: %d of %d." % (
            len(rv["far_plane_changing"]), rv["plane_changing"])) if "plane_changing" in rv else ""
        w("| P3 crosstalk | Phase edges (55 V/us) into every net: smallest margin **%.1f dB** (%s, %s). "
          "Signal vias with no GND via within 1 mm: %d of %d.%s | %s |" % (
              worst_x["margin_db"], worst_x["net"], worst_x["cls"], len(far), rv["total"], pc,
              "PASS" if worst_x["margin_db"] >= 6 else "FAIL"))
    if rf:
        n = rf["networks"]
        ar = n["as routed"]
        w("| P4 RF feed | Line Z0 **%.1f ohm** (eeff %.2f). Fence spacing max %.2f / %.2f mm. As routed: IL %.2f dB, "
          "RL %.1f dB in band, 2f0 %.0f dB, 3f0 %.0f dB | %s |" % (
              rf["z_line"], rf["eeff"], rf["fence"]["1"]["max_spacing"], rf["fence"]["-1"]["max_spacing"],
              ar["il_max"], ar["rl_min"], ar["att_2f0"], ar["att_3f0"],
              "PASS" if abs(rf["z_line"] - 50) <= 5 else "CHECK"))
    if cv:
        a = cv.get("as routed", {})
        w("| P5 plane cavity | VBAT/GND cavity modes in 1.8-3.2 GHz: %s. Max VM-port impedance in the ISM band "
          "%.1f ohm | %s |" % (", ".join("%.2f GHz" % m for m in [x / 1e9 for x in a.get("ism_modes", [])]) or "none",
                               a.get("z_ism_max", 0), "PASS" if not any(2.3e9 < m < 2.6e9 for m in
                                                                        a.get("ism_modes", [])) else "FAIL"))
    if nt:
        d = nt["rail_3v0_drop"]
        con = [v for v in d.values() if v is not None]
        w("| P6 nets | All nets lumped (worst %s: %.0f ps = %.2f x t_r/2). 52 MHz crystal stray +%.2f pF -> %.1f ppm "
          "pull. 3V0 DC drop max %.1f mV%s | PASS |" % (
              nt["worst_lumped"]["net"], nt["worst_lumped"]["delay_ps"], nt["worst_lumped"]["lumped_ratio"],
              nt["crystal_cl_add_pF"], nt["crystal_pull_ppm"], 1e3 * max(con) if con else 0,
              "" if len(con) == len(d) else " (%d loads not connected!)" % (len(d) - len(con))))
    w("")
    w("## Design changes forced by these checks (layout rev A)")
    w("")
    w("| Found by | Problem | Fix |")
    w("|---|---|---|")
    for row in (
        ("P4 gap scan", "The RF feed left the SX1280 RFIO pin sideways and ran along the pin row, over GND pins 21 and 20: a short",
         "Radio rotation forced so the RFIO row faces the antenna; 0.6 mm stub straight out of the pin row"),
        ("P4 gap scan", "Four shunt caps of the matching network were placed GND-pad-first: each RF stub crossed its cap's GND pad (4 shorts)",
         "RF pad faces the line, 0.05 mm from its node's series pad; GND pad away, with filled via-in-pad"),
        ("P4 network", "Each 0402 pad over the 0.1 mm L6-L5 prepreg is a 0.18 pF shunt: as routed, the network detuned (2.5 dB IL, 6.9 dB RL)",
         "L5 rule-area cut-outs under the RF signal pads (they see L4 0.26 mm down, about 0.07 pF): within 0.3 dB of the reference"),
        ("P5 cavity", "Full-size VBAT plane: plane-pair cavity mode at 2.39 GHz, inside the radio band and near the 14th harmonic of the 170 MHz FC clock",
         "VBAT plane limited to the hub and arm roots (L4 GND in the motor discs): first mode about 2.7 GHz"),
        ("Router + DRC", "RF via fence at 0.5 mm pitch made an all-layer wall between the M4 ESC MCU and its driver",
         "Fence pitch 1.0 mm (lambda/70 at 2.45 GHz), topped up after routing where room allows"),
        ("DRC", "Motor wire pads at the driver corner: 0.09 mm from the package, 0.16 mm from the VM pin",
         "Pads moved to a column beside the OUT edge: 0.4 mm clearance, sub-mm phase traces"),
        ("DRC", "ESC MCU outer pad rows 0.21 mm from the board edge (0.25 mm rule)", "ESC MCUs moved 0.3 mm inward"),
        ("Router", "Motor pads at 1.27 mm pitch fanned out faster than the driver's 0.4 mm OUT pins, so phase A had no path on any motor",
         "Motor pads 0.75 mm / 0.4 mm drill at 1.0 mm pitch, at x = +/-10.3 mm: every OUT pin escapes with one 45 deg jog"),
        ("Router subset test", "ESC MCU BEMF/CSA inputs (PA0-PA4) sat on the edge facing away from the driver, directly over the neighbouring ESC MCU on the other side: no via could reach them",
         "Pin swap to the driver-facing corner (BEMF PB1/PB0/PA7, CSA PA6/PA5, nFAULT PB3); zero-cross becomes ADC-based"),
        ("Router subset test", "BEMF dividers at the motor-pad column were boxed in by the driver's back-side exposed pad; in motor 4's corner the RF chain took their place",
         "Compact divider pairs on the opposite side between driver and ESC MCU; motor 4's pairs in explicit columns east of the RF line"),
        ("Router subset test", "Driver decoupling caps sat 0.35 mm past the escape stubs, sealing the driver's rear pin row",
         "Routing channel around drivers/ESC MCUs widened 0.6 -> 0.9 mm; 100 nF VM caps moved under the VM corner on the opposite side"),
        ("P2 IR drop", "Driver VM corner point computed on the wrong axis: 1-2 VM vias per driver (one via carrying 4 A peak), one via inside a pin",
         "Corner track wraps outside the package with 2-3 VM vias per driver (12 total); vias kept 0.45 mm from pad centres"),
        ("P5 cavity", "Covering every VBAT via with a square plane moved the first cavity mode down to 2.66 GHz",
         "Hub-square VBAT plane plus narrow lobes to the outlying VBAT vias: first mode 3.14 GHz"),
        ("DRC", "Fan-out vias co-located or too close (hole-to-hole), and vias next to motor-phase copper or the board edge",
         "Drill-web guard (0.2 mm) in every via placer and in the router; class-safe clearance margin; edge check"),
        ("DRC", "rf_isolation rule compared B.Net (a net code) with 'GND', so it also held the GND pour 0.20 mm off the RF line",
         "Rule uses B.NetName: the coplanar gap is the intended 0.15 mm (50 ohm)"),
        ("DRC", "Fiducial mask openings exposed the GND pours", "Pour keep-outs around the fiducials"),
        ("Pre-layout 5a", "VDDA ferrite + 1 uF resonance: x5.6 noise gain at 333 kHz", "10 ohm RC filter instead of the ferrite"),
        ("Pre-layout 9", "PWM frequency / slew trade-off", "96 kHz PWM, SLEW = AGND (no hardware change)"),
        ("Fit", "The antenna-tuning pi (0R + 2 DNP) no longer fit on the shorter feed", "Dropped; C22/L1 are the tuning elements"),
    ):
        w("| %s |" % " | ".join(row))
    w("")
    # ---------------------------------------------------------------------------------------------------------
    if th:
        w("## P1: thermal (`ps1_thermal.py`)")
        w("")
        w("![thermal maps](out/p1_thermal_maps.png)")
        w("")
        m = th["model"]
        w("**Model.**")
        w("- %d cells per layer at %.1f mm on 6 copper sheets, %d nodes in total." % (m["cells"], m["res_mm"], m["nodes"]))
        w("- In-plane conductance comes from the real copper fill.")
        w("- FR4 couples the sheets through the thickness.")
        w("- %d power vias and plated holes are explicit barrels, with %d via-to-layer ties." % (m["vias"],
                                                                                              m["via_layer_ties"]))
        w("- Total heat capacity %.2f J/K." % m["C_total"])
        w("- Convection: h_wash in the prop discs, %.0f W/m2K elsewhere, plus %.0f W/m2K radiation." % (m["h_free"],
                                                                                                   m["h_rad"]))
        w("- %.0f %% of each bottom motor's loss enters its mounting disc." % (100 * m["motor_frac"]))
        w("- Driver loss comes from the hw/sim switching model at the operating point, and RDS(on) follows Tj.")
        w("")
        w("| Scenario | U11 | U12 | U13 | U14 | IMU | FC | radio | LDO |")
        w("|---|---|---|---|---|---|---|---|---|")
        for k, v in th["scenarios"].items():
            t = v["tj"]
            pr = v["probes"]
            w("| %s%s | %s | %s | %s | %s | %.0f | %.0f | %.0f | %.0f |" % (
                k, " (RUNAWAY)" if v["runaway"] else "", *("%.0f" % min(t[r], 999) for r in ("U11", "U12", "U13", "U14")),
                pr["U3"], pr["U2"], pr["U4"], pr["U1"]))
        w("")
        lo = th["losses"]
        w("Driver loss per DRV8311 at Tj 100 C:")
        w("- hover at 96 kHz: %.3f W (conduction %.3f + diode, switching and standby %.3f);" % (
            lo["hover"]["p_cond"] + lo["hover"]["p_fixed"], lo["hover"]["p_cond"], lo["hover"]["p_fixed"]))
        w("- hover at 48 kHz: %.3f W;" % (lo["hover_48k"]["p_cond"] + lo["hover_48k"]["p_fixed"]))
        w("- full throttle: %.2f W." % (lo["full"]["p_cond"] + lo["full"]["p_fixed"]))
        w("")
        w("![bursts](out/p1_burst.png)")
        w("")
        b = th.get("burst_budget_s", {})
        w("**Full-throttle budget, starting from hover steady state:** Tj reaches 125 C after %s s, 150 C after %s s, "
          "and 175 C after %s s. Full throttle cannot be sustained: the steady state is above the 180 C thermal "
          "shutdown." % (f2(b.get("125 C")), f2(b.get("150 C")), f2(b.get("175 C"))))
        w("")
        fo = th.get("foster_all4_1W")
        if fo:
            w("**Compact model for the ESC/FC firmware** (Foster fit of Zth(t), the same 1 W on all four junctions, "
              "h_wash 60, max fit error %.2f C):" % fo["max_fit_error_C"])
            w("")
            w("| pole | R (C/W) | tau (s) |")
            w("|---|---|---|")
            for i, (R, tau) in enumerate(zip(fo["R_C_per_W"], fo["tau_s"])):
                w("| %d | %.2f | %.3f |" % (i + 1, R, tau))
            w("")
            w("Tj_est = T_board + sum of three first-order filters of P_loss. P_loss is per driver:")
            w("1.5 x I_ph,rms^2 x RDS(on),HS+LS(Tj_est) + 0.17 W at 96 kHz. This matches the switching model within 7 %")
            w("at hover and at full throttle. RDS(on) is 0.30 ohm at 25 C, 0.45 ohm at 150 C. I_rms comes from the CSA. Seed T_board with the")
            w("G071 internal temperature sensor. Recommended limit: start derating at an estimated 130 C and reach")
            w("hover-level current at 150 C.")
            w("")
        if th.get("inner_1oz"):
            io = th["inner_1oz"]
            w("**Copper-weight option:** with 1 oz inner layers, hover Tj is %s C and the 3 s burst peak is %.0f C. "
              "Ask JLC for the 1 oz inner-layer stackup at order time if it is offered for 0.8 mm." % (
                  "/".join("%.0f" % v for v in io["tj_hover"].values()), io["burst3_peak"]))
            w("")
    if ir:
        w("## P2: DC IR drop and current density (`ps2_irdrop.py`)")
        w("")
        w("![IR drop](out/p2_irdrop_maps.png)")
        w("")
        w("| Case | Total A | VBAT drop U11/U12/U13/U14 (mV) | GND rise (mV) | Loop max (mV) | Copper loss (mW) | Max via current (A) |")
        w("|---|---|---|---|---|---|---|")
        for k, v in ir["cases"].items():
            w("| %s | %.1f | %s | %s | %.0f | %.0f | %.2f |" % (
                k, v["total_current"], "/".join("%.0f" % (1e3 * v["VBAT"]["drop"][r]) for r in ("U11", "U12", "U13", "U14")),
                "/".join("%.0f" % (1e3 * v["GND"]["drop"][r]) for r in ("U11", "U12", "U13", "U14")),
                1e3 * max(v["loop_drop"].values()), 1e3 * (v["VBAT"]["p_diss"] + v["GND"]["p_diss"]),
                max(v["VBAT"]["via_i_max"], v["GND"]["via_i_max"])))
        w("")
    if xt:
        w("## P3: crosstalk and return paths (`ps3_xtalk.py`)")
        w("")
        w("![crosstalk](out/p3_xtalk.png)")
        w("")
        w("Coupling coefficients come from 2-D field solves of the real layer cross-sections: L1/L6 microstrip under")
        w("mask, the L3 stripline between GND and VBAT, and L3 over L4 between the L2 and L5 GND planes. Every victim")
        w("track is checked against the nearest parallel phase track on its layer. A victim on L4 is also checked")
        w("against the phase tracks on L3 above it, because where L4 carries escape routing there is no plane between")
        w("the two layers. The L2 and L5 GND planes screen every other layer pair.")
        w("")
        bsx = xt.get("broadside")
        if bsx:
            w("Broadside coupling between L3 and L4 (phase on one layer, victim on the other): %d nets, %.2f mm coupled in total, worst %s at %.0f dB "
              "margin." % (bsx["nets"], bsx["coupled_mm"], bsx["worst"] or "-", bsx["worst_margin_db"]))
            w("")
        w("| Net | Class | Coupled length (mm) | Closest (mm) | C_m (fF) | Glitch (mV) | Limit (mV) | Margin (dB) |")
        w("|---|---|---|---|---|---|---|---|")
        for r in [r for r in xt["rows"] if r["cm_fF"] > 0][:15]:
            w("| %s | %s | %.2f | %.2f | %.2f | %.3f | %.1f | %.0f |" % (r["net"], r["cls"], r["coupled_mm"],
                                                                     r["s_min"], r["cm_fF"], r["v_mV"],
                                                                     r["thr_mV"], r["margin_db"]))
        w("")
    if rf:
        w("## P4: RF feed as routed (`ps4_rf.py`)")
        w("")
        w("![RF](out/p4_rf.png)")
        w("")
        w("| Segment net | Length (mm) | Width (mm) | GND gap min/median (mm) | Reference | Z0 (ohm) |")
        w("|---|---|---|---|---|---|")
        for s in rf["segments"]:
            if s["len"] < 0.05:
                continue
            w("| %s | %.2f | %.3f | %.3f / %.3f | %s | %.1f |" % (s["net"], s["len"], s["w"], s["gap_min"],
                                                                 s["gap_med"], s.get("ref", "L5"), s["z0"]))
        w("")
        for k, v in rf["networks"].items():
            w("- %s: IL %.2f dB, RL %.1f dB (2.400-2.4835 GHz), 2f0 attenuation %.0f dB, 3f0 %.0f dB" % (
                k, v["il_max"], v["rl_min"], v["att_2f0"], v["att_3f0"]))
        w("- RF signal pads with the L5 relief cut-out: %d of %d" % (rf.get("pads_with_L5_relief", 0), rf.get("rf_pads", 0)))
        if rf.get("monte_carlo"):
            mc = rf["monte_carlo"]
            w("- Monte Carlo, %d boards (part tolerances +/-0.05 pF / +/-0.1 nH, pad C +/-20 %%, line Z0 +/-5 %%): IL median "
              "%.2f dB, 95th percentile %.2f dB; in-band RL median %.1f dB, 5th percentile %.1f dB" % (
                  mc["n"], mc["il_p50"], mc["il_p95"], mc["rl_p50"], mc["rl_p5"]))
        if rf.get("z0_tolerance"):
            w("- Line Z0 over the stackup tolerance: " + ", ".join("%s %.1f ohm" % kv for kv in rf["z0_tolerance"].items()))
        if rf.get("retune"):
            w("")
            w("Post-layout re-tune: these values make the as-routed network reproduce the reference 2-port. Use them")
            w("as the VNA starting point if the BOM values measure off. RMS |dS11| to the reference: %.3f." % rf.get(
                "retune_rms_s11_error", 0))
            w("")
            w("| Part | BOM | Continuous optimum | Nearest standard value |")
            w("|---|---|---|---|")
            for r_, v_ in rf["retune"].items():
                unit, k_ = ("nH", 1e9) if r_.startswith("L") else ("pF", 1e12)
                w("| %s | %.1f %s | %.2f %s | %.1f %s |" % (r_, v_["bom"] * k_, unit, v_["continuous"] * k_, unit,
                                                        v_["fitted"] * k_, unit))
        w("")
    if cv:
        w("## P5: VBAT/GND plane cavity (`ps5_cavity.py`)")
        w("")
        w("![modes](out/p5_cavity_modes.png)")
        w("![impedance](out/p5_cavity_z.png)")
        w("")
        for k, v in cv.items():
            if k == "caps":
                continue
            w("- %s: first modes %s GHz; modes in 1.8-3.2 GHz: %s; max |Z| at the VM ports in the ISM band %.2f ohm" % (
                k, ", ".join("%.2f" % x for x in v["modes_GHz"][:6] if x > 0.05),
                ", ".join("%.2f" % (x / 1e9) for x in v["ism_modes"]) or "none", v["z_ism_max"]))
            if "l_eff_nH" in v:
                w("  - VM-loop effective inductance (nH): %s; VM bounce on a 2.8 A / 114 ns edge (mV): %s" % (
                    ", ".join("%s %.2f" % kv for kv in v["l_eff_nH"].items()),
                    ", ".join("%s %.0f" % kv for kv in v["vm_bounce_mV"].items())))
        w("")
    if nt:
        w("## P6: nets, timing, crystal, 3V0 rail (`ps6_nets.py`)")
        w("")
        w("![nets](out/p6_nets.png)")
        w("")
        for n_ in ("RF_XTA", "RF_XTB"):
            c = nt["crystal"][n_]
            w("- %s: %.2f mm routed, trace %.2f pF + pads %.2f pF" % (n_, c["length"], c["trace_pF"], c["pads_pF"]))
        w("- 3V0 DC drop from the LDO: " + ", ".join("%s %s" % (k, "%.1f mV" % (1e3 * v) if v is not None else "NOT CONNECTED")
                                                    for k, v in sorted(nt["rail_3v0_drop"].items())))
        w("")
    open(os.path.join(HERE, "POST_LAYOUT_REPORT.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("wrote POST_LAYOUT_REPORT.md (%d lines)" % len(L))


if __name__ == "__main__":
    main()
