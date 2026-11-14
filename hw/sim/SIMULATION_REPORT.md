# Nano-drone hub board: engineering simulation report

Scope: 20x20 mm, 6-layer 0.8 mm hub PCB with 4x DRV8311H, 4x STM32G071 ESC MCUs, STM32G431 FC,
SX1280 radio, LSM6DSV16X IMU, TPS7A2030 3V0 LDO, 1S LiHV, BetaFPV 0702SE II 23000KV with GF1219S-3 props.
All scripts, plots and raw results live in `hw/sim/`. Nothing outside `hw/sim/` was touched.

How to reproduce: from `hw/sim/` run `python sim_0X_*.py` (each study regenerates its own PNG/JSON in `out/`),
or `python run_all.py` (about 5 min). Python is KiCad's bundled 3.11. Extra packages (scipy, matplotlib)
are in `hw/sim/.pylibs`. All shared numbers (datasheet values with citations, and every assumption with
its range) are in `simlib.py`, so all eight studies use the same inputs.

Labels used below. **[DS]** is a datasheet value with document, section and page. **[A]** is an
engineering assumption, not a datasheet value, with the range used for sensitivity.

---

## 0. Summary

| # | Study | Key result | Limit | Verdict |
|---|-------|-----------|-------|---------|
| 1 | Battery hot-plug | Peak VBAT at TPS7A20 VIN: **4.35 V** nominal (overdamped, zeta 1.2). **5.03 V** at the worst case inside the stated ranges (25 mOhm cell, 50 nH, MLCC -20% and 40% retained). Goes above 6.5 V only if total series R is below about 21-24 mOhm (at 50-60 nH). Inrush 70-80 A peak. | TPS7A20 VIN 6.5 V abs max (6.0 V ROC); DRV8311 VM 24 V | **PASS** (1.5 V margin) |
| 2 | Motor + ESC electrical | Hover: 24.7 krpm, D 0.42, f_e 2.47 kHz, 14.8 k commutations/s (3.2 PWM periods per step). Phase I_pk is **2.8 A** at L 6 uH and **4.0 A** at 3 uH, because the ripple is large (3-5 A pk-pk). Full throttle: 45.5-49.6 krpm, I_pk 2.5-3.1 A. Punch-out from hover: **4.4 A**. Locked rotor at D=1: 8.1 A. BEMF at the ADC is Â±0.24 V at hover (332 LSB). | IOUT 5 A peak (abs max/ROC); OCP 5.8 A min | **PASS in flight, conditional**: needs a start-up duty limit of 0.24-0.39 and a punch-out current limit |
| 3 | Thermal (DRV8311 Tj) | Loss per driver at Tj 100 C: hover **0.78 W** (0.57 W at 96 kHz, 1.54 W worst case), full throttle **2.3-2.5 W**. Hover steady-state Tj: **77 C** at h=80 but **145 C** at h=30 W/m2K. A 5 s full-throttle burst from hover reaches **130 C** at h=80 (crosses 125 C at 4.1 s); with 96 kHz PWM it reaches 122 C. Still air: runaway. | Tj < 125 C (criterion); TSD 180 C min | **FAIL** at h=30 and in still air. **Marginal** at h=80: bursts must be 4 s or less, or 5 s with 96 kHz PWM |
| 4 | Flight performance | Hover 2.1-2.5 A battery current (D 0.39-0.51). Max thrust 15.7 g/motor at 4.30 V OCV (vendor: 32 g). TWR **3.7 down to 2.5** (260 mAh). Flight time at hover **6.7 min** (260 mAh) / **7.4 min** (300 mAh); mixed flying 5.2 / 5.7 min. Motor-prop time constant **46 ms** (31-63 ms). | TWR above 2 (soft) | **PASS**, but max thrust is about half the vendor figure because of DRV8311 RDS(on) and sag |
| 5a | 3V0 PDN | Z_max **0.47 Ohm @ 89 kHz** at the MCU nodes and 0.84 Ohm at the radio (ferrite + 4.7 uF). A 68 mA load step gives -21 mV at the radio and -13 mV at the FC. The VDDA ferrite + 1 uF peaks with **x5.6 noise gain at 333 kHz**. | 1.32 Ohm (3%); 0.44 Ohm (1%) | **PASS** vs 3% target. 1% target slightly exceeded. **VDDA filter needs a fix** |
| 5b | VBAT ripple | At the driver VM pins: **385-443 mV pk-pk** with synchronous PWM, 275-354 mV (95th percentile) with random phase, 48-51 mV ideally interleaved. 22 uF ripple current 0.5-0.6 A rms. Ripple on 3V0 via PSRR is under 0.5 mV. | No hard limit violated | **PASS**. Interleaving recommended |
| 6 | ADC front ends | BEMF 5 kOhm source: ts **>= 19.5 cycles** (557 ns) for 12-bit, 12.5 cycles for 10-bit. Do not add a pin capacitor. On-time sampling needs D >= 13% at 48 kHz (27% at 96 kHz). CSA: add **100 Ohm + 1 nF** (ts 12.5 cycles), otherwise ts is about 1.6 us. VBAT divider is fine (kick-back 0.2 LSB, 48 kHz ripple -44 dB). | DS12232 Table 57 | **PASS** with these settings |
| 7 | RF CPWG + match | 50 Ohm CPWG on L6/L5: **W 0.150 / g 0.150 mm** gives 49.8 Ohm on bare copper (45.5 Ohm under 15 um mask; for a masked line use W 0.12 / g 0.15). Reference network between 50-Ohm ports: IL 0.35-0.46 dB ideal, **0.9-1.4 dB** with 0402 parasitics; 2f0 attenuation **27 dB** (ideal) / 34 dB; 3f0 30 dB (ideal) / over 70 dB (fragile notch). | 0.09 mm min feature; 50 Ohm | **PASS** (manufacturable). The match must be tuned on a VNA |
| 8 | LDO / brown-out | 3V0 load is 89 mA typical / 163 mA worst, so regulation needs VIN >= 3.04 / 3.08 V. During punch-outs the LDO drops out when OCV is below 3.85 V (40 mOhm pack) or 4.15 V (60 mOhm pack). 3V0 minimum is **2.45-2.76 V**. DRV8311 VM falls below the 2.7 V UVLO maximum on punch-outs when OCV is below **3.33 V** (40 mOhm) or **3.58 V** (60 mOhm). | Radio 1.8 V; BOR0 1.69 V; DRV8311 UVLO 2.5-2.7 V; VM ROC 3.0 V | **Logic: PASS. Motor drivers: CONDITIONAL**: needs a VBAT-based throttle/current limit or a pack of 40 mOhm or less |

### Design changes, highest priority first

1. **Power-stage losses and thermal (studies 2, 3, 4).** At 1S the DRV8311 is specified at 300 mOhm (HS+LS) typical and
   350 mOhm max at 25 C, rising to 450 / 500 mOhm at 150 C ([DS] SLVSFN2B s7.5 p.10). That is more than the motor's own
   0.23 Ohm. It halves the achievable thrust and dissipates 0.6-0.8 W per driver at hover and 2.3-2.5 W at full throttle.
   On a 20x20 mm board this only works with strong downwash cooling. Recommended:
   a. **Run the PWM at 96 kHz** (the DRV8311 supports 200 kHz). Hover driver loss drops 27% (0.78 to 0.57 W), current
      ripple halves, and flight time increases by about 17%. The cost is a higher minimum duty for on-time BEMF
      sampling (27%, see item 6).
   b. **Measure the motor inductance** with an LCR meter. Between 3 and 15 uH, hover driver loss changes by 3x and the
      hover-current part of flight time changes by about 30%.
   c. Put thermal copper on every layer under the drivers. Use the maximum number of 0.2 mm vias in each exposed pad
      plus a via ring around it. Ask JLC for 1 oz inner layers if available. Place the drivers in the direct prop wash.
   d. Firmware thermal derating: the G071 internal temperature sensor sits next to each driver. Also limit sustained
      full throttle to about 3-4 s at h of about 80 W/m2K.
   e. If bench tests show the board above about 100 C at hover, change the power stage to one with much lower
      RDS(on), for example discrete N-FET half bridges with a gate driver, as typical whoop AIOs use.
2. **Current protection (study 2/4).** The DRV8311 peak IOUT is **5 A**, but OCP only trips at **5.8 A min / 9 A typ**
   ([DS] s7.1 p.7, s7.5 p.12), so OCP does not protect the 5 A rating. Required in firmware:
   - start-up and locked-rotor duty 0.24 or less (if L = 3 uH), or 0.39 or less (L = 6 uH);
   - a cycle-by-cycle current limit from the CSA at about 3.5-4 A;
   - a throttle ramp/current limit on punch-outs. The initial current is 4.4 A at 4.35 V with a cold board.
3. **Low-battery sag (study 8).** Add a VBAT-dependent throttle/current limit so VM stays at or above 3.0 V (DRV8311 ROC
   minimum) and well above the 2.7 V UVLO maximum. Specify packs with an internal resistance of 40 mOhm or less.
   Optionally change the LDO to **TPS7A2028 (2.8 V)**: it then stays in regulation down to VIN of about 2.88 V, so
   VDDA/VREF stays stable during punch-outs. All 3V0 loads are rated for 2.8 V. Never enable STM32 BOR level 4
   (2.70-2.90 V falling, [DS] DS12232 Table 23 p.56) on a 3.0 V rail.
4. **FC VDDA filter (study 5a).** The ferrite + 1 uF + 100 nF resonates at about 333 kHz and amplifies FC-node noise
   **x5.6**. Replace the bead with **10 Ohm** (an RC filter at 16 kHz with no peaking; VDDA draws about 1 mA, so the
   drop is about 10 mV), or use a lossy bead with DCR of at least 1 Ohm. The radio ferrite + 4.7 uF (resonance about
   200 kHz) is acceptable (worst step -21 mV), but a lossier bead would damp it.
5. **VBAT ripple (study 5b).** Synchronise the four ESC PWM timers with 90 degree offsets, for example by feeding an FC
   timer output to each G071 timer trigger input. This cuts ripple from about 0.4 V to about 50 mV pk-pk. Adding a
   100 uF polymer only reduces it to about 0.24 V. Take BEMF zero-crossing thresholds from VM measured at the same
   instant, not from a fixed or filtered value.
6. **ADC configuration (study 6).**
   - BEMF channels: ts = 19.5 cycles (12-bit) or 12.5 cycles (10-bit).
   - No capacitor on the BEMF divider pins. Alternatively use 4.7k/4.7k to allow 12-bit at 12.5 cycles.
   - Sample at least 2.8 us after the PWM input edge.
   - Use the G071 comparators for zero-crossing at high speed. Full throttle has only 1.6 PWM periods per commutation
     step at 48 kHz.
   - CSA outputs: 100 Ohm + 1 nF C0G at the ADC pin. Sample the continuously-on low-side phase in the middle of a
     PWM interval, because the ripple is 3-5 A pk-pk.
   - VBAT: calibrate against VREFINT. The 3V0 tolerance of Â±1.5% otherwise gives Â±63 mV of error.
7. **RF (study 7).** Use W = 0.150 mm and g = 0.150 mm with a solder-mask opening over the line, or W = 0.12 mm and
   g = 0.15 mm under mask. Add a GND via fence at about 1 mm pitch. Keep the wire-wound LQW15AN inductors from the
   reference BOM: Q = 60 instead of 30 saves about 0.25 dB. Tune C0/L1 on a VNA against the real RFIO impedance.
8. **Hot-plug and regeneration (study 1).** No hot-plug change is needed. Optionally add a 47 uF polymer (it takes the
   below-range case from 6.70 V to 4.98 V) if the measured pack + connector ESR near 80 kHz is under 20 mOhm.
   Regeneration is a separate risk: if the battery disconnects (crash, bouncing connector) while the ESCs brake with
   complementary PWM, the rotor energy (0.3 J per rotor at full speed) can pump VBAT far above 6.5 V. An energy bound
   gives 42 V for a 10% slowdown of one rotor into about 66 uF. ESC firmware should stop synchronous braking when
   VBAT rises above about 4.6 V.

---

## 1. Common inputs

### 1.1 Datasheet values used [DS]

| Part / value | Number | Source |
|---|---|---|
| DRV8311 VM abs max / ROC | 24 V / 3-20 V | SLVSFN2B s7.1 p.7, s7.3 p.7 |
| DRV8311 peak output current | 5 A (abs max and ROC) | s7.1 p.7, s7.3 p.7 |
| DRV8311 RDS(on) HS+LS, 3 V <= VM <= 6 V | 300 typ / 350 max mOhm @25 C; 450 / 500 @150 C | s7.5 p.10 |
| DRV8311 slew SLEW=AGND | 18 / 35 / 55 V/us | s7.5 p.10 |
| DRV8311 dead time (handshake, SLEW=AGND) | 500 typ / 1200 max ns | s7.5 p.11 |
| DRV8311 tPD (SLEW=AGND) | 1000 typ / 1500 max ns | s7.5 p.11 |
| DRV8311 CSA gain (GAIN=AGND), settling, linear range, offset | 0.25 V/A; 1 us to 1% (30 pF); 0.25..VCSAREF-0.25 V; Â±70 mA | s7.5 p.11-12; Table 8-5 p.29 |
| DRV8311 CSA output | SOx = VREF/2 Â± GÂ·I (low-side sense FET) | s8.3.10 eq.2 p.38 |
| DRV8311 ICSAREF | 1.7 typ / 3.0 max mA @3.0 V (ROC lists up to 7.5 mA) | s7.5 p.12; s7.3 p.7 |
| DRV8311 UVLO VM falling / deglitch | 2.5 / 2.6 / 2.7 V; 5-13 us | s7.5 p.12 |
| DRV8311 OCP (MODE Hi-Z) | 5.8 / 9 / 11.5 A | s7.5 p.12; Table 8-5 p.29 |
| DRV8311 IVM operating | 10-12 typ / 15 max mA | s7.5 p.8 |
| DRV8311 thermal | RthJA 42.6, RthJB 15.7, RthJC(bot) 4.8 C/W; TSD 180/190/200 C; TJ op. max 150 C | s7.4 p.8; s7.5 p.13 |
| DRV8311 loss formulas (trapezoidal) | PCON = 2Â·IPKÂ²Â·RDS,ON; PSW = IPKÂ·VPKÂ·trfÂ·fPWM; Pdiode = 2Â·IPKÂ·VFÂ·tDEADÂ·fPWM | s12.3.1 Table 12-1 p.85 |
| DRV8311 package | WQFN-24 3x3 mm, exposed pad about 1.9x1.9 mm | p.1; RRW0024A drawing |
| TPS7A20 VIN abs max / ROC | 6.5 V / 1.6-6.0 V | ZHCSKY8G (Chinese ed. of SBVS338) s6.1 p.5, s6.3 p.5 |
| TPS7A20 dropout | 140 mV max @300 mA, 2.5 V <= VOUT < 5.5 V, DQN (X2SON) | s6.5 p.6 |
| TPS7A20 load regulation, COUT range, ESR | 13 mV (1-300 mA); 1-200 uF; ESR <= 100 mOhm | s6.5 p.6; s6.3 p.5 |
| TPS7A20 PSRR (VIN = VOUT+1 V) | 75 dB @10/100 kHz (20 mA); 60 dB @100 kHz (300 mA) | s6.5 p.7 |
| TPS7A20 RthJA (DQN) | 166.1 C/W | s6.4 p.6 |
| SX1280 VBAT operating / max | 1.8-3.7 V / 3.9 V | DS rev 3.2, Table 3-2 / 3-1 p.19 |
| SX1280 TX / RX current (DC-DC on) | 24 mA @+12.5 dBm; RX 5.5-8.2 mA | Table 3-8 p.26; Table 3-5 p.22 |
| SX1280 LDO-mode penalty | "doubling of current consumption" | s14.7.1 p.143 |
| SX1280 reference match | C0 0.8 pF, L1 3.0 nH LQW15AN3N0B80, C1 1.2 pF, L2 2.5 nH LQW15AN2N5C00 // C4 0.5 pF, C2 1.2 pF, C3 100 pF | s15.1 Fig. 15-1 p.145; Table 15-1 p.146 |
| STM32G071 ADC | CADC 5 pF; fADC 35 MHz; 2.5 MSps; tCONV = ts + 12.5 cyc | DS12232 r3 Table 56 p.85 |
| STM32G071 max RAIN vs ts | 12-bit: 12.5 cyc -> 4.7 kOhm, 19.5 cyc -> 8.2 kOhm; 10-bit: 12.5 cyc -> 5.6 kOhm | Table 57 p.86-87 |
| STM32G071 CIO, IINJ, IDD 64 MHz, BOR | 5 pF; -5 mA; 6.3 typ / 7.7 max mA; PDR 1.60-1.69 V, BOR4 falling 2.70-2.90 V | Table 51 p.79; Table 19 p.54; Table 25 p.59; Table 23 p.56 |
| STM32G431 (proxy: STM32G474 DS12288 r5) | IDD 170 MHz 29.5 typ / 31 max (25 C) / 56 max (125 C) mA; BOR0 1.60-1.69 V | Table 21 p.90; Table 19 p.86 |
| LSM6DSV16X | IddHP 0.65 mA; Vdd 1.71-3.6 V | DS13510 r4 s4.2 Table 4 p.14 |

The STM32G431 datasheet was not available locally. The STM32G474 (same G4 core and process) is used as a proxy and
labelled as such.

### 1.2 Global assumptions [A] (value, range used)

| Item | Value | Range / note |
|---|---|---|
| LiHV full charge | 4.35 V | 4.20-4.45 V |
| Battery DC IR incl. BT2.0 connector | 40 mOhm | 25-60 mOhm (user). Rises 50% toward empty |
| Pigtail R / L | 10 mOhm / 40 nH | 25-50 nH (swept to 60 nH) |
| OCV(SoC) | generic LiHV table (3.30 V @0% ... 4.35 V @100%) | see `simlib.py` |
| AUW | 15.3 g (260 mAh, 7.6 g) / 16.0 g (300 mAh, 8.3 g) | dry mass 7.7 g |
| Overlapping/stacked-prop thrust efficiency | 0.90 | 0.85-0.95; hover thrust per motor = W/(4Â·0.9) = 4.25 g isolated |
| Landing voltage (loaded) | 3.30 V | 3.2-3.4 V |
| 22 uF 0603 10 V X5R DC bias | C/C0 = 1/(1+(V/V0)^1.6), V0 = 4.0 V (50% at 4 V) | V0 3.2-5.2 V (40-60% at 4 V); Â±20% tolerance; ESR 4 mOhm; ESL 0.55 nH |
| Motor KV, pole pairs | 23000 rpm/V gives Ke = Kt = 4.152e-4 VÂ·s/rad (6-step DC equivalent); 6 pole pairs (9N12P) | KV from vendor |
| Motor R (lead-to-lead) | 0.23 Ohm | 0.17-0.29 Ohm (retailer data, read as lead-to-lead) |
| Motor L (line-to-line) | 6 uH | **3-15 uH, unknown; swept** |
| No-load current I0 | 0.30 A | 0.2-0.45 A |
| BEMF shape, timing advance | sinusoidal; 15 degrees | 0-22.5 degrees |
| FET body diode VF | 0.8 V | 0.7-1.0 V (not in datasheet) |
| Rotor + prop inertia J | 2.2e-8 kgÂ·mÂ² | 1.5-3.0e-8 |
| Housekeeping VBAT current | 3V0 load 95 mA + 4x11 mA IVM | |

**Motor/prop calibration.** For a fixed-pitch prop, thrust T = kTÂ·wÂ² and torque Q = kQÂ·wÂ². The torque-per-thrust ratio
of the Gemfan 31 mm 2-blade comes from the Happymodel point (21.7 g @ 2.86 A): Q/T = KtÂ·(I-I0)/T = 4.99e-3 m.
- **[A]** The GF1219S-3 (higher pitch, 3 blades) has s = 1.2x that ratio (range 1.0-1.4).
- **[A]** The BetaFPV 32 g maximum thrust was measured at 4.2 V on a low-loss reference ESC (30 mOhm).
- Result: kT = 6.24e-9 NÂ·sÂ² and kQ = 3.74e-11 NÂ·mÂ·sÂ², which implies 4.83 A at 32 g, 67.7 krpm and a figure of
  merit of 0.31 (plausible for 31 mm props).
- Consequence: hover current per motor is I0 + (Q/T)Â·T/Kt, about 0.90 A. This depends only weakly on the
  motor-voltage assumptions. Speed-dependent quantities (max thrust, BEMF) depend more on them.

---

## 2. Study 1: battery hot-plug (`sim_01_hotplug.py`, `out/s1_hotplug.png`, `out/s1_inrush.png`)

**Method.** Nonlinear lumped network, integrated with a fixed-step trapezoidal solver (5 ns):
- 4.35 V step through R_s (cell + connector + pigtail) and L_s;
- battery-pad node with 2x22 uF, using the DC-bias C(V) curve and ESR;
- VBAT plane (0.8 nH, 2 mOhm [A]) to the driver node with 4x22 uF + 4x100 nF;
- 3 nH / 20 mOhm trace [A] to the TPS7A20 input with 1 uF [A].

No load current is assumed, which is the worst case. Sweeps cover R_batt 0-60 mOhm, L 25-60 nH, MLCC tolerance Â±20%
and retention 40-60% at 4 V.

**Results.**
- The large bank (about 97 uF large-signal average) and the low-inductance pigtail give a characteristic impedance
  of only Z0 of about 20 mOhm. The battery path resistance (35-70 mOhm) therefore damps the circuit critically or
  better: zeta = 1.23 nominally, f0 = 81 kHz.
- Peak VIN at the TPS7A20:
  - nominal: 4.35 V (no overshoot);
  - worst case inside the stated ranges (25 mOhm cell, 50 nH, C -20% and 40% retained): **5.03 V**;
  - same case with a 4.45 V (overcharged) pack: 5.17 V.
- Total series R needed to stay under 6.0 V at 60 nH: at least 30 mOhm. Under 6.5 V: about 21-24 mOhm (at 50-60 nH). This is below
  the stated range; for example, 10 mOhm cell + 10 mOhm pigtail at 50 nH gives 6.70 V. Adding a 47 uF / 60 mOhm
  polymer at the pads brings that case to 4.98 V.
- Inrush peak is 69-81 A (in range) for a few us. This is normal for BT2.0 and causes connector spark wear.
- The DRV8311 limit (24 V) is never approached.
- Contact bounce cannot exceed the first-contact case: re-closure starts from a charged bank, so the voltage step is
  smaller.

**Verdict: PASS**, with 1.5 V margin to 6.5 V and 1.0 V to the 6.0 V ROC within the stated assumptions.

**Risk note (not a hot-plug result).** If the battery disconnects while the ESCs regenerate (complementary PWM
during deceleration), the bus is pumped by rotor energy. Each rotor holds 0.30 J at full speed. By an energy bound, a
10% slowdown of one rotor into about 66 uF reaches 42 V. See recommendation 8.

---

## 3. Study 2: motor + ESC electrical model (`sim_02_motor_esc.py`)

Plots: `out/s2_hover_waveforms.png`, `out/s2_full_waveforms.png`, `out/s2_inductance_sensitivity.png`,
`out/s2_bemf_and_startup.png`.

**Method.** Switching-level simulation with a 25 ns fixed step, in `simlib.sim_6step`. Energy balance closes within
0.6%. Model contents:
- star motor with sinusoidal BEMF;
- 6-step 120-degree commutation with 15 degree advance;
- DRV8311 3x PWM mode: complementary 48 kHz PWM on the high phase with 500 ns handshake dead time; LS on for the low
  phase; Hi-Z for the floating phase, with body-diode clamping during demagnetisation;
- per-FET RDS = (HS+LS)/2 (Table 12-1 convention).

Speed is held constant per operating point. Duty is solved so that mean electromagnetic torque equals prop torque
plus friction. VM includes battery sag from four identical motors. Default conditions are OCV 3.85 V (mid SoC) and
RDS 25 C typical (0.30 Ohm).

| Operating point (L_LL, f_PWM) | D | rpm | f_e / commut. | I_pk | ripple pk-pk | I_bus avg | P_drv (25 C) |
|---|---|---|---|---|---|---|---|
| Hover 4.25 g (6 uH, 48 kHz) | 0.42 | 24 680 | 2.47 kHz / 14.8 kHz (3.2 PWM per step) | **2.82 A** | 2.98 A | 0.51 A | 0.57 W |
| Hover (3 uH, 48 kHz) | 0.42 | 24 680 | same | **4.04 A** | 4.93 A | 0.76 A | 1.10 W |
| Hover (15 uH, 48 kHz) | 0.47 | 24 680 | same | 1.72 A | 1.32 A | 0.41 A | 0.36 W |
| Hover (6 uH, 96 kHz) | 0.48 | 24 680 | 6.5 PWM per step | 1.94 A | 1.68 A | 0.43 A | 0.42 W |
| 10 g/motor (6 uH, 48 kHz) | 0.77 | 37 850 | 3.79 / 22.7 kHz (2.1 per step) | 2.99 A | 2.32 A | 1.34 A | 1.15 W |
| 10 g/motor (3 uH, 48 kHz) | 0.73 | 37 850 | same | 3.55 A | 4.05 A | 1.51 A | 1.46 W |
| Full throttle, OCV 3.85 V (6 uH) | 1 | 45 530 | 4.55 / 27.3 kHz | 2.54 A | (no PWM) | 2.30 A | 1.76 W |
| Full throttle, OCV 4.35 V (3 uH) | 1 | 50 350 | 5.04 / 30.2 kHz | **3.14 A** | 1.0 A (commut.) | 2.82 A | 2.52 W |

- **OCP / peak current.** In steady flight the worst I_pk is 4.04 A, leaving 0.96 A to the 5 A peak rating and 1.76 A
  to the 5.8 A OCP minimum.
  - A punch-out from hover at 4.35 V on a cold board starts at **4.42 A** (DC-equivalent), only 0.6 A below 5 A.
  - Locked rotor at D = 1 draws 8.1 A. To keep I_pk under 5 A at start-up, duty must stay at or below
    0.24 / 0.39 / 0.48 / 0.52 for L = 3 / 6 / 10 / 15 uH.
  - Firmware must enforce these limits (recommendation 2).
- **Ripple.** With L of 3-6 uH at 48 kHz, the current reverses every PWM period at hover (synchronous rectification).
  Conduction loss is 2-3.9x the ripple-free value (0.24 W at hover). Going to 96 kHz, or a larger L, is the most effective fix.
- **BEMF at the ADC.**
  - Phase BEMF is 26.3 mV peak per krpm.
  - The floating-phase swing at the ADC (10k/10k divider, sampled in on-time around VM/4) is Â±0.24 V at hover
    (Â±332 LSB) and Â±0.45-0.50 V at full throttle.
  - A 100-LSB swing is reached at 7.5 krpm.
  - A 10 mV comparator/divider offset gives about 2.3 degrees electrical error at hover.
  - Demagnetisation clamp time is 1.4-8 us. It is always shorter than the zero-crossing position (25-51 us after
    commutation with 15 degree advance), so the crossing stays visible.
  - The real limit is sampling density: at full throttle there are only 1.6 PWM periods per 60 degree step at 48 kHz.
    Use comparator-based zero-crossing detection, or interpolate ADC samples.

**Verdict: PASS for steady flight; CONDITIONAL** on firmware start-up duty and punch-out current limits.

---

## 4. Study 3: thermal (`sim_03_thermal.py`, `out/s3_driver_losses.png`, `out/s3_temperature_maps.png`, `out/s3_burst_tj.png`)

**Method.**
- Driver losses come from the switching model at RDS(25 C) and RDS(150 C), then interpolated in RDS(Tj). They
  include conduction with ripple, diode loss (dead time + demagnetisation), switching loss IÂ·VÂ·(V/SR)Â·f per Table
  12-1, and standby VMÂ·11 mA.
- Cross-check against TI Table 12-1 with an ideal flat-top current.
- 2.5-D finite-difference board model: 6 copper layers on a 40x40 grid of 0.5 mm cells.
  - [A] Copper fill: L1/L6 45%, L2/L5 90%, L3 35%, L4 75%. Inner copper 15.2 um, outer 35 um.
  - [A] FR4: k_xy 0.8 W/mK, k_z 0.3 W/mK.
  - [A] 0.2 mm vias with 20 um plating: background density 0.3/mmÂ² plus 5 in each exposed pad.
- [A] Convection on both faces with an area factor of 1.2, plus linearised radiation h_rad = 8 W/mÂ²K.
- [A] h_conv = 12 W/mÂ²K (still air), 30 or 80 W/mÂ²K (downwash). A flat-plate laminar estimate at the 5-9 m/s induced
  velocity gives 60-85 W/mÂ²K. A hub sitting directly under the stacked upper props is closer to 80.
- [A] Placement: drivers on L1 at the four quadrants, ESC MCUs under them on L6, FC in the centre, plus LDO, radio and
  VBAT/GND copper IÂ²R (3 mOhm [A]).
- Tj = T(L1 under the pad) + PÂ·(4.8 C/W RthJC(bot) [DS] + 1.0 C/W [A] solder).
- [A] Ambient 25 C, OCV 4.05 V. Implicit Euler at 50 ms.

**Driver loss per DRV8311 at Tj = 100 C:**

| Case | Hover | Full throttle |
|---|---|---|
| Nominal (L 6 uH, RDS typ, 48 kHz) | 0.78 W | 2.27 W |
| Worst (L 3 uH, RDS max, 48 kHz) | 1.54 W | 2.50 W |
| Mitigated (L 6 uH, RDS typ, 96 kHz) | 0.57 W | 2.27 W |
| TI Table 12-1, ideal flat-top current | 0.42 W | 2.48 W |

Board heat capacity is 0.98 J/K. The global time constant at h = 30 is about 27 s.

**Junction temperature (hottest driver, ambient 25 C):**

| Scenario | h = 12 (still air) | h = 30 | h = 80 |
|---|---|---|---|
| Hover steady state, nominal | runaway (above TSD) | **145 C** | **77 C** |
| Hover steady state, worst | runaway | runaway (about 300 C model) | 135 C |
| Hover steady state, 96 kHz | above 200 C | 109 C | 63 C |
| Full throttle steady state, nominal | - | runaway | 199 C, would trip TSD |
| 5 s burst from hover: Tj at 3 s / 5 s, nominal | - | above 190 C | **119 / 130 C** (125 C at 4.1 s) |
| 5 s burst, worst | - | runaway | 163 / 170 C |
| 5 s burst, 96 kHz | - | 160 / 175 C | **110 / 122 C** |

Model values above about 180 C are not physical: the DRV8311 shuts down at TSD (180 C min, [DS] s7.5 p.13).

**Verdict: FAIL vs the 125 C criterion** in still air and at h = 30 W/mÂ²K.
- With strong downwash (h of about 80), hover passes (77 C), but full-throttle bursts must stay at or below about
  4 s. With 96 kHz PWM, 5 s bursts reach 122 C (pass).
- Hover-level power cannot be sustained without airflow. Bench-testing with loaded props in a shroud, or in a closed
  canopy, overheats the board within about 0.5 min (lumped estimate).
- Mitigations: recommendation 1.

---

## 5. Study 4: flight performance (`sim_04_flight.py`, `out/s4_hover_thrust_twr.png`, `out/s4_flight_time.png`, `out/s4_step_response.png`)

**Method.** Hover and full-throttle operating points come from the switching model, which includes ripple losses, at
seven OCVs. [A] RDS uses Tj 80 C at hover and 100 C at full throttle. The discharge integrates SoC with the
OCV/IR-sag model and stops when the loaded voltage reaches 3.30 V. The step response uses the averaged electrical model
with the rotor ODE, all 4 motors stepping together, battery sag included, and optionally a firmware current limit.

| | 260 mAh (AUW 15.3 g) | 300 mAh (AUW 16.0 g) |
|---|---|---|
| Hover duty (4.30 to 3.30 V OCV) | 0.39 to 0.51 | 0.40 to 0.53 |
| Hover battery current | 2.12-2.50 A (mean 2.26 A) | 2.21-2.60 A (mean 2.35 A) |
| Max thrust per motor (isolated) at 4.30 / 3.90 / 3.45 V | 15.7 / 13.6 / 11.4 g | same |
| Max vehicle thrust (x4 x0.9) | 56 / 49 / 41 g | same |
| Thrust-to-weight at 4.30 / 3.45 / 3.30 V | **3.68 / 2.69 / 2.52** | 3.52 / 2.57 / 2.41 |
| Flight time, hover to 3.30 V loaded | **6.7 min** | **7.4 min** |
| Flight time, mixed flying (mean thrust 1.3x hover [A]) | 5.2 min | 5.7 min |

- **Inductance and PWM sensitivity.** Hover bus current per motor at 3.90 V OCV is 0.79 A at 3 uH / 48 kHz,
  0.53 A at 6 uH / 48 kHz, 0.44 A at 6 uH / 96 kHz and 0.42 A at 15 uH. Flight time scales roughly with the inverse.
- **Motor-prop time constant.** Linearised at hover, tau = J / (2Â·kQÂ·w0 + KtÂ·Ke/R) = **46 ms** (31-63 ms for the J
  range).
- **Thrust steps:**
  - hover to 1.2x thrust: t63 48 ms, 10-90% in 101 ms;
  - hover to full throttle: t63 50 ms, 10-90% in 96 ms, peak motor current 3.56 A (warm board, 3.95 V);
  - same from a 4.35 V pack on a cold board: peak **4.42 A**;
  - same with a 3.5 A firmware limit: t63 53 ms, so the limit costs only about 3 ms;
  - deceleration to hover duty with active braking: t63 35 ms, regenerative current -0.34 A.
  - For the attitude loop, the propulsion bandwidth is about 3.5 Hz (1/(2Â·piÂ·46 ms)). The slow time constant comes
    from the high total loop resistance (DRV8311 + motor + battery about 0.6 Ohm), which reduces electrical damping.
- Compared with the vendor's 32 g per motor (on a low-resistance ESC at about 4.2 V), the DRV8311 version reaches
  only about 49% of the thrust at 4.3 V OCV.

**Verdict: PASS** (TWR above 2.4 at every state of charge, 5-7 min flights). This is a moderate-performance whoop, not
a racer.

---

## 6. Study 5: PDN (`sim_05_pdn.py`, `out/s5a_3v0_impedance.png`, `out/s5a_3v0_step.png`, `out/s5b_vbat_ripple.png`)

### 6a. 3V0 rail (updated decoupling set)

The decoupling set includes the design update:
- ESC MCU: 100 nF + 1 uF (0201) each, plus 100 nF on the DRV8311 CSAREF [DS rec.];
- FC: 3x100 nF + 4.7 uF, and VDDA 1 uF + 100 nF behind a ferrite;
- radio: 2x100 nF + 4.7 uF behind a ferrite;
- IMU: 2x100 nF [A];
- LDO output: 2.2 uF.

**Method.** Nodal admittance analysis from 1 kHz to 300 MHz.
- MLCCs are series R-L-C with DC-bias retention at 3 V [A]: 100 nF 0.8-0.9; 1 uF 0201 0.45; 2.2 uF 0.55;
  4.7 uF 0402 0.45. ESL is 0.3 nH (0201) and 0.4 nH (0402).
- [A] Ferrite: R_dc 0.1 Ohm + (0.3 uH || 150 Ohm || 1 pF), about 120 Ohm at 100 MHz. The actual part is unspecified.
- [A] Behavioural LDO: Z = R_ol || (R_a + jwL_a), with R_a = 13 mV/300 mA = 43 mOhm [DS], R_ol = 2 Ohm [A] and
  unity-gain frequency 1 MHz [A].
- [A] Star distribution on the L4 island: 0.6-1.2 nH and 2-4 mOhm per branch.
- Target impedance: step of 48 mA (SX1280 TX at +12.5 dBm in LDO mode, 24 mA x2 [DS]) plus 20 mA (FC wake).
  That gives 1.32 Ohm for 3% (90 mV) and 0.44 Ohm for 1% (ADC reference).

**Results.**
- |Z| maximum below 20 MHz:
  - LDO, FC, ESC and IMU nodes: 0.47 Ohm at 89 kHz. This is the LDO-loop / total-capacitance anti-resonance.
  - Radio: 0.84 Ohm at 88 kHz.
  - VDDA: 2.8 Ohm at 363 kHz. Acceptable for its own 1 mA load, but see below.
- Load step (68 mA, released 80 us later): radio -21.2 / +13.4 mV, FC -12.7 mV, VDDA -14.7 mV, ESC -12.6 mV.
  Static drops are 4-5 mV. The worst case is 0.7% of 3.0 V.
- **VDDA filter.** The FC-to-VDDA voltage transfer peaks at **x5.6 at 333 kHz**: the bead and 1 uF form an
  under-damped LC. At 10 MHz the attenuation is -62 dB.
- The ~90 kHz peak depends on the assumed LDO loop. If it is confirmed by measurement, a larger LDO output capacitor
  (e.g. 10 uF) lowers it (Z0 is proportional to 1/sqrt(C)). That option was not simulated.

**Verdict: PASS** against the 3% target. Slightly over the 1% target (0.47 vs 0.44 Ohm; radio node 0.84 Ohm), but the
time-domain deviation stays within 0.7%. **Fix the VDDA filter** (recommendation 4).

### 6b. VBAT ripple at 48 kHz

**Method.** Bus current per driver comes from the switching model, averaged over many PWM periods into one period
(commutation-rate components are not included). The phasor solution covers the battery (R_path, 40 nH), the pads
(2x22 uF at 3.9 V bias) and four driver nodes (22 uF + 100 nF) through 0.8 nH / 2 mOhm plane segments [A].

| Point | D | I_bus avg / ac rms | Sync pk-pk | Random phase median / 95th pct | Interleaved | Sync + 100 uF polymer |
|---|---|---|---|---|---|---|
| Hover | 0.43 | 0.53 / 0.83 A | **385 mV** | 173 / 275 mV | 48 mV | 231 mV |
| 7 g/motor | 0.60 | 0.90 / 0.97 A | **443 mV** | 202 / 354 mV | 50 mV | 258 mV |
| 10 g/motor | 0.80 | 1.38 / 0.91 A | **407 mV** | 195 / 337 mV | 51 mV | 233 mV |

- Each 22 uF carries 0.51-0.57 A rms ripple, well within 0603 MLCC ratings (part datasheet not checked).
- 3V0 ripple from VBAT ripple at 60 dB PSRR [DS] is under 0.45 mV, unless the LDO is in dropout (study 8).
- The four G071s run from independent clocks, so their relative PWM phase drifts. The ripple then beats between the
  synchronous and interleaved cases at the clock-difference frequency.

**Verdict: PASS** (no limit violated). The ~0.4 V worst-case ripple still matters for BEMF thresholds and EMI, so
interleaving is recommended (recommendation 5).

---

## 7. Study 6: ADC front ends (`sim_06_adc.py`, `out/s6_bemf_adc.png`, `out/s6_csa_vbat.png`)

Configuration: STM32G071, VDDA = VREF+ = 3.0 V [A: VREF+ bonded to VDDA on the 28-pin package], so 1 LSB = 0.73 mV.

**(a) BEMF divider, 10k/10k (5 kOhm Thevenin).**
- DS12232 Table 57 gives the minimum ts: **19.5 cycles (557 ns) for 12-bit**, because 12.5 cycles only allows
  4.7 kOhm. For 10-bit, **12.5 cycles (357 ns)** is enough. Conversion takes 0.91 / 0.66 us.
- A charge-sharing + RC-recovery model (CADC 5 pF [DS], pin 5 pF [DS], PCB 2 pF [A], full-scale channel-to-channel
  step) reproduces this: 0.5 LSB after 490 ns.
- **Adding a pin capacitor makes it worse.** It takes 3.3 us with 100 pF and 18.7 us with 1 nF, because the
  reservoir must recover through 5 kOhm. A 4.7k/4.7k divider allows 12-bit at 12.5 cycles.
- The divider node settles with tau = 35 ns. The earliest valid sample after the MCU PWM edge is 2.23 us: tPD max
  1.5 us [DS], plus slew 0.24 us at 18 V/us min [DS], plus 0.2 us ringing [A], plus settling. Adding ts gives
  **2.8 us**, so on-time sampling needs **D >= 13% at 48 kHz (27% at 96 kHz)**. Below that, use off-time or
  comparator methods.
- Pin range is -0.40 to 2.58 V (2.92 V at the hot-plug peak). The negative clamp current is about 20 uA, far below
  the -5 mA IINJ limit [DS].

**(b) CSA (SOx) into the ADC.**
- 0.25 V/A around 1.5 V gives Â±5 A within the linear range, 2.93 mA/LSB, and an offset of Â±70 mA (Â±24 LSB) [DS],
  which should be calibrated at standstill.
- [A] Driving the pin directly, the CSA must recover the 5 pF kick (CSA tau 0.22 us, from the 1 us to 1% tSET).
  That needs about 1.64 us, i.e. 79.5 cycles.
- **100 Ohm + 1 nF C0G** brings it to 0.31 us, so 12.5 cycles is enough. The filter corner is 1.6 MHz. 220 Ohm /
  2.2 nF is too slow (1.08 us).
- Ripple is 1-5 A pk-pk, so sample the continuously-on low-side phase at the middle of a PWM interval to get the
  mean current.

**(c) VBAT divider, 10k/10k + 100 nF.**
- tau = 0.5 ms (fc = 318 Hz). The 48 kHz ripple is attenuated by -44 dB: 443 mV pk-pk becomes 1.5 mV at the pin.
- Kick-back is 0.2 LSB. Sampling bias is 0.04 mV at 1 kHz and 0.4 mV at 10 kHz. 99% settling takes 2.3 ms.
- Range is 6.0 V at 1.46 mV/LSB.
- Error budget at 4.2 V: divider Â±21 mV (1% parts), VREF Â±63 mV (Â±1.5% LDO tolerance). Calibrate against VREFINT.
- Quiescent drain is 0.22 mA, which empties a stored 260 mAh pack in about 50 days.

**Verdict: PASS** with the settings listed in recommendation 6.

---

## 8. Study 7: RF (`sim_07_rf.py`, `out/s7_cpwg.png`, `out/s7_matching_network.png`)

**(a) CPWG on L6 over L5.**
- [A] Stack: h = 0.0994 mm, er = 4.45, t = 35 um.
- Method: 2-D quasi-static finite-volume Laplace solver on a graded grid (1.5 um at metal edges), half-section with a
  symmetry wall, finite metal thickness. C' comes from the discrete field energy; Z0 = 1/(cÂ·sqrt(C'Â·C'_air)).
- Validation:
  - microstrip (W 0.18 mm, wide gap): FD 47.33 Ohm vs Hammerstad-Jensen 47.52 Ohm (0.4%);
  - grid convergence: 49.68 / 49.79 / 49.87 Ohm (0.2%);
  - GCPW conformal-mapping closed form (t = 0): 2.6-4.3% above FD. That formula loses accuracy for g/h above 1 (its
    g -> infinity limit does not reduce to microstrip), so it is only a sanity check here.

| Gap g (mm) | W for 50 Ohm, bare copper (mm) | eps_eff | Z0 at that W under 15 um mask [A er 3.8] | W for 50 Ohm under mask (mm) |
|---|---|---|---|---|
| 0.100 | 0.1325 | 2.74 | 44.7 | 0.096 |
| 0.125 | 0.1419 | 2.84 | 45.4 | 0.110 |
| **0.150** | **0.1479** | 2.91 | 45.8 | 0.119 |
| 0.200 | 0.1541 | 2.99 | 46.2 | 0.128 |
| 0.250 | 0.1572 | 3.04 | 46.4 | 0.132 |
| 0.300 | 0.1588 | 3.06 | 46.5 | 0.134 |

**Recommended: W = 0.150 mm, g = 0.150 mm with the mask opened over the line**, giving 49.8 Ohm. Under mask it is
45.5 Ohm; use W = 0.12 mm in that case.
- Sensitivity: h Â±10% gives 52.1 / 47.3 Ohm; etch of -20 um on W gives 53.9 Ohm; er 4.2 / 4.6 gives 51.0 / 49.1 Ohm.
- All features are 0.12 mm or more (min 0.09 mm).
- Line is short, so the lumped matching analysis below ignores its phase.

**(b) Matching / harmonic filter** (shunt C0, series L1, shunt C1, series L2||C4, shunt C2, series C3), between
50-Ohm ports.
- [A] Parasitics: L with Q = 30 or 60 at 2.45 GHz (sqrt(f) scaling) and SRF 6-8 GHz; C0G caps with ESR 0.2 Ohm and
  ESL 0.35 nH including the ground via; C3 with 0.4 nH.

| Case | IL 2.4-2.5 GHz | RL (50 Ohm src) | Min atten. 4.8-5.0 GHz | Min atten. 7.2-7.5 GHz |
|---|---|---|---|---|
| Ideal L/C | 0.35-0.46 dB | >= 10.0 dB | 27.1 dB | 30.5 dB |
| L Q=30, SRF 8 GHz | 1.17-1.41 dB | >= 8.7 dB | 33.8 dB | 78 dB (notch) |
| L Q=60, SRF 8 GHz | 0.92-1.14 dB | >= 8.3 dB | 33.8 dB | 78 dB (notch) |
| L Q=30, SRF 6 GHz | 1.48-1.78 dB | >= 7.8 dB | 37.9 dB | 70.6 dB |

- The large 3f0 numbers come from series self-resonance of the 1.2 pF shunt capacitors (about 7.8 GHz with 0.35 nH).
  That notch is narrow and layout-dependent; in practice, assume 40-50 dB is limited by coupling.
- Context: the 2nd harmonic at 4.8-5.0 GHz falls in the FCC 15.205 restricted band. At +12.5 dBm the total
  suppression needed (PA + filter + antenna) is about 54 dB, so the filter supplies 27-34 dB of it.
- **The 50 Ohm source is an assumption.** The SX1280 PA optimum load and LNA input are not 50 Ohm, which is why the
  in-band return loss is only 8-10 dB here. The real match must be tuned on a VNA, and harmonics verified radiated or
  conducted with the real board.

**Verdict: PASS.**

---

## 9. Study 8: LDO dropout / brown-out (`sim_08_ldo_brownout.py`, `out/s8_brownout.png`)

**3V0 load budget (typical / worst):**
- STM32G431 at 170 MHz: 35 / 56 mA (G474 proxy, Table 21);
- 4x STM32G071: 31 / 41 mA (Table 25 + peripherals [A]);
- SX1280 in LDO mode: 12 / 48 mA (RX x2 / TX +12.5 dBm x2);
- IMU: 0.65 mA;
- 4x CSAREF: 6.8 / 12 mA;
- miscellaneous: 3 / 6 mA [A].

That totals **89 / 163 mA**. [DS] Dropout is modelled as R_DO = 140 mV / 300 mA = 0.467 Ohm, so regulation needs
VIN >= **3.04 / 3.08 V**. Worst-case LDO dissipation at 4.35 V is 0.22 W, giving +37 C at the JEDEC RthJA.

**Sag scenarios:** hover (with the ripple-loss correction from study 4), steady full throttle, and the punch-out peak
(hover to D = 1, first instant). [A] RDS is at 25 C, which gives the highest current.

| Pack IR | Full throttle keeps VM >= 3.0 V (ROC) for OCV >= | Punch-out keeps VM >= 2.7 V (UVLO max) for OCV >= | LDO stays in regulation during punch-out for OCV >= | 3V0 minimum (OCV 3.30 V) |
|---|---|---|---|---|
| 25 mOhm | 3.33 V | 3.30 V | 3.63 V | 2.76 V |
| 40 mOhm | 3.45 V | 3.33 V | 3.85 V | 2.61 V |
| 60 mOhm | 3.63 V | 3.58 V | 4.15 V | 2.45 V |

- **Logic rail: PASS.** 3V0 stays at 2.45 V or higher in every case. That is above SX1280 1.8 V, IMU 1.71 V and
  STM32 PDR/BOR0 1.69 V (max). STM32 BOR4 (2.70-2.90 V) would trip, so do not use it.
- During dropout VDDA follows VBAT. The ADC readings then scale: VBAT and BEMF read high, and the CSA reads up to
  about 10% high in counts. Use VREFINT ratio correction or the TPS7A2028 option.
- **Motor drivers: CONDITIONAL.**
  - Late in the flight (OCV 3.35-3.6 V), a punch-out with a 60 mOhm pack drives VM below the DRV8311 UVLO maximum
    (2.7 V, deglitch 5-13 us). That can shut the bridge down mid-flight.
  - At full throttle, VM goes below the 3.0 V ROC minimum for OCV below 3.45-3.63 V.
  - Mitigation: recommendation 3.

---

## 9b. Study 9: PWM frequency x SLEW setting (`sim_09_pwm_slew.py`, `out/s9_pwm_slew.png`)

**Question.** Study 3 showed 96 kHz cuts hover driver loss by 27 %. Is 192 kHz better still, and is a faster
SLEW strap worth its EMI cost? On the DRV8311H the SLEW pin also sets the handshake dead time and the propagation
delay, so the four settings are swept together with frequency.

**Method.** Same switching model as study 2, hover thrust, OCV 3.85 V, L 6 uH, RDS(on) at Tj 100 C. Per SLEW
setting [DS] SLVSFN2B s7.5 p.10-11: AGND 35 V/us, 500 ns tDEAD; 47k 75 V/us, 450 ns; Hi-Z 180 V/us, 425 ns;
AVDD 230 V/us, 425 ns. [A] 12 V datasheet values used at 1S. EMI proxy: trapezoid harmonic envelope at 30 MHz.

**Results (loss per driver at hover, W):**

| SLEW \ f_PWM | 48 kHz | 72 kHz | 96 kHz | 128 kHz | 160 kHz | 192 kHz | 30 MHz envelope @96 kHz |
|---|---|---|---|---|---|---|---|
| AGND (35 V/us) | 0.748 | 0.593 | **0.555** | 0.557 | 0.575 | 0.590 | 57.6 dBuV |
| 47k (75 V/us) | 0.730 | 0.575 | 0.529 | 0.525 | 0.523 | 0.538 | 64.2 dBuV |
| Hi-Z (180 V/us) | 0.720 | 0.565 | 0.517 | 0.505 | 0.509 | 0.512 | 71.8 dBuV |
| AVDD (230 V/us) | 0.719 | 0.564 | 0.515 | 0.503 | 0.506 | 0.509 | 73.9 dBuV |

Above about 100 kHz, ripple conduction loss stops falling while dead-time diode loss (2 x 0.5 us per period at
0.8 V) and switching loss keep rising. 192 kHz is **worse** than 96 kHz at every setting. The fastest strap saves
at most 0.05 W (9 %) but costs 14-16 dB more switching-node harmonics at 30 MHz.

**Decision.** Keep SLEW = AGND (no BOM change, best EMC). Firmware runs the ESC PWM at **96 kHz** (128 kHz is equal
within 0.002 W, at 2.4 % less on-time resolution). Do not use 192 kHz.

---

## 10. Validation status and what to measure first

The models are physics-based but calibrated on vendor marketing data and several unknowns. Measurements that would
most reduce uncertainty, in order of value:
1. **Motor L_LL and R_LL** (LCR meter at 10-100 kHz; 4-wire R). L drives hover loss, ripple, OCP margin and flight
   time by 2-4x across the 3-15 uH range.
2. **Thrust stand with 0702SE II + GF1219S-3 on this board**: thrust, current and rpm vs duty at 4.2 V and 3.7 V.
   This replaces the kT/kQ/I0 calibration and its test-voltage assumption.
3. **Board temperature in hover and 5 s punch-outs** (thermocouple or IR on a driver, or the G071 temperature
   sensors). This calibrates the downwash h, the single most uncertain thermal input.
4. **Pack impedance at 1 kHz and about 80 kHz**, including the BT2.0 connector (hot-plug damping and sag).
5. **3V0 Zout (or a load-step scope shot) and the VDDA node**, to confirm the behavioural LDO model and the ferrite
   resonance.
6. **VNA tuning** of the RF match with the real antenna.

Known simplifications:
- constant speed within each switching simulation;
- no diode reverse recovery or switching-node ringing;
- MLCC models are generic, not part-specific;
- the thermal model ignores heat flow into the frame and motors (conservative) and uses a constant h;
- the VBAT ripple analysis folds each driver's bus current to one PWM period (commutation-rate components excluded);
- the RF match is lumped, with no line sections or coupling.

---

## 11. Files

| File | Content |
|---|---|
| `simlib.py` | shared datasheet values (with citations), assumptions (with ranges), battery/MLCC/motor/prop models, averaged operating point, switching-level 6-step simulator |
| `sim_01_hotplug.py` | study 1, `out/s1_*.png`, `out/study1_hotplug.json` |
| `sim_02_motor_esc.py` | study 2, `out/s2_*.png`, `out/study2_motor_esc.json` |
| `sim_03_thermal.py` | study 3, `out/s3_*.png`, `out/study3_thermal.json` |
| `sim_04_flight.py` | study 4, `out/s4_*.png`, `out/study4_flight.json` |
| `sim_05_pdn.py` | study 5, `out/s5a_*.png`, `out/s5b_*.png`, `out/study5_pdn.json` |
| `sim_06_adc.py` | study 6, `out/s6_*.png`, `out/study6_adc.json` |
| `sim_07_rf.py` | study 7, `out/s7_*.png`, `out/study7_rf.json` |
| `sim_08_ldo_brownout.py` | study 8, `out/s8_brownout.png`, `out/study8_ldo_brownout.json` |
| `sim_09_pwm_slew.py` | study 9 (PWM frequency x SLEW), `out/s9_pwm_slew.png`, `out/study9_pwm_slew.json` |
| `run_all.py` | runs all nine studies |
