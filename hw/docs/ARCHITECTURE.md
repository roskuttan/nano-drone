# Nano drone — hardware architecture (rev 0.3)

A compact, stacked-prop brushless quad whose **PCB is the frame**. Everything below is generated from
code in `hw/` (schematic: `design/nanodrone_sch.py`, layout: `design/nanodrone_pcb.py` +
`design/nanodrone_route.py`, outputs: `design/nanodrone_fab.py`).

## Airframe

| Item | Choice | Why |
|---|---|---|
| Motors | 4 × BetaFPV 0702SE II 23 000 KV (1.38 g, 9N12P, 3×M1.4 EQS on Ø6.6 mm) | lightest brushless motor with published 1S thrust data |
| Props | Gemfan 31 mm (1219S-3 3-blade or 1210-2 2-blade) | smallest commercial brushless props |
| Layout | **Compact stacked X**: adjacent motor spacing **26 mm** (36.8 mm diagonal) → **57 mm prop-tip span** | M1/M4 mounted on top, M2/M3 underneath; adjacent props overlap at different heights |
| Top-motor spacers | 3 mm nylon spacers under M1/M4 | lifts top props ~3 mm above a 6.5 mm battery strapped on top |
| Battery | 1S 260–300 mAh 75–80C, BT2.0 pigtail, along the M2–M3 diagonal on top | the only diagonal free of top props' hubs |
| Antenna | λ/4 wire (~31 mm) from the front edge, **in the board plane** | the board plane is the one place both prop layers never reach |

Coordinates used everywhere: board centre = (0,0), +x right, +y rear. Betaflight numbering:
M1 rear-right (top), M2 front-right (bottom), M3 rear-left (bottom), M4 front-left (top).

## Electronics — distributed architecture

```
 1S LiPo ──┬─────────────── VBAT plane (L4) ───────────┬──────────┬──────────┬──────────┐
           │                                            │          │          │          │
        TPS7A2030 (3.0 V LDO)                       DRV8311H   DRV8311H   DRV8311H   DRV8311H
           │                                         ▲  │BEMF/I  ▲  │       ▲  │       ▲  │
        +3V0 ─┬──────────┬──────────┬────────┐       │  ▼        │  ▼       │  ▼       │  ▼
              │          │          │        │     STM32G071  STM32G071  STM32G071  STM32G071   (one ESC MCU per motor)
        STM32G431 FC ── SPI1 ── LSM6DSV16X IMU      ▲ DShot     ▲          ▲          ▲
              │ ─ SPI3 ── SX1280 2.4 GHz (ELRS-capable) ─ 50 Ω ─ antenna
              └─ TIM2 CH1-4 ─ bidirectional DShot ─┘ + SWD programming bus (4× SWDIO + shared SWCLK/NRST) to the ESC MCUs
```

### Why distributed (and not one big MCU)
The first design used a single STM32G474 in a 0.4 mm-pitch WLCSP-81. Its 61 used signals needed
**23 inner balls** that cannot be escaped with through-vias (two 0.25 mm via pads at 0.4 mm pitch leave
0.15 mm; even JLC's 0.09/0.09 mm rules need ≥ 0.27 mm), forcing HDI (laser microvias + buried vias,
+₹40–70k per batch, new process at JLC). Splitting into one small ESC MCU per motor gives:
same 57 mm drone, standard 6-layer board, ~half the batch cost, hand-reworkable QFNs, and a dedicated
64 MHz controller per motor. Cost of the choice: two firmware images (FC + one ESC image ×4).

### Parts (all in stock at LCSC, 2026-11-08)
| Function | Part | Package | LCSC |
|---|---|---|---|
| Flight controller | STM32G431CBU6 (170 MHz M4F, 128 KB) | UFQFPN-48 7×7 | C529356 |
| ESC MCU ×4 | STM32G071GBU6 (64 MHz M0+, 128 KB, 2.5 Msps ADC) | UFQFPN-28 4×4 | C529347 |
| Motor driver ×4 | TI DRV8311H (integrated FETs, 3–20 V, 5 A pk) | WQFN-24 3×3 | C5218862 |
| Radio | Semtech SX1280 (LoRa/FLRC/GFSK, ELRS hardware) | QFN-24 4×4 | C125969 |
| IMU | ST LSM6DSV16X | LGA-14 2.5×3 | C5267406 |
| LDO | TI TPS7A2030PDQNR (3.0 V, 300 mA) | X2SON 1×1 | C5220164 |
| Crystal | 52 MHz, 10 pF, ±10 ppm (HCI XC21M4) | 2016 | C3034316 |
Passives: see `design/lcsc_map.json` (every line checked in stock).

### DRV8311H configuration (no strap resistors)
`MODE` floating (Hi-Z) → 3x PWM mode, OCP 9 A typ / 5.8 A min; `SLEW`=GND (35 V/µs, gentlest edges);
`GAIN`=GND (0.25 V/A); `CSAREF`=+3V0 (CSA midpoint 1.5 V). `nSLEEP` and `nFAULT` go to that motor's own
ESC MCU. During any MCU reset the driver inputs see their internal pull-downs → `nSLEEP`=0 → bridges asleep and
outputs Hi-Z.

Since rev C, `nFAULT` has no external pull-up; the ESC MCU's PB3 internal pull-up (25-55 kΩ) does the job.
Rev B had one 10 k part per motor, and each sat 3-5 mm away in the hub with its own +3V0 branch. Those four
branches were among the last nets the router could not fit. The worst coupled phase-edge glitch on an `nFAULT` line is
under 20 mV with the 55 kΩ worst case, against the 0.9 V input threshold (P3, 33 dB margin). Firmware requirement
E11 covers the pull-up and a two-read filter.

### ESC MCU pin map (STM32G071GBU6, UFQFPN-28)
| Pin | Port | Net | Function |
|---|---|---|---|
| 15/14/13 | PB1/PB0/PA7 | BEMF A/B/C | ADC1_IN9/IN8/IN7 via 10k/10k dividers (pins at the package corner facing the driver) |
| 12/11 | PA6/PA5 | ISENS A/B | ADC1_IN6/IN5 from DRV8311 SOA/SOB |
| 16/18/19 | PA8/PA9*/PA10* | INH A/B/C | TIM1_CH1..CH3 (*pin 18/19 after SYSCFG PA11/PA12 remap) |
| 17/22/25 | PC6/PA15/PB5 | INL A/B/C | GPIO: 0 = phase floating (BEMF window), 1 = driven |
| 24 | PB4 | DSHOT | TIM3_CH1 + DMA, bidirectional DShot from the FC |
| 28 / 23 | PB8 / PB3 | nSLEEP / nFAULT | driver control / fault input |
| 20/21/5 | PA13/PA14/NRST | SWD | from the FC's programming bus |
Battery voltage is read from the BEMF divider of whichever phase is driven high (VBAT/2), so the ESC
needs no separate VBAT pin.

The BEMF/CSA pins were moved in layout rev A. The first map put them on PA0-PA4, on the package edge facing away
from the driver and right over the neighbouring ESC MCU on the other side, so they could not be routed. None of the
new BEMF pins is a comparator input, so zero-cross detection is ADC-based: continuous sampling of the floating
phase at 1-2 Msps.

### Flight-controller pin map (STM32G431CBU6, UFQFPN-48), layout rev B
| Pins | Function |
|---|---|
| PA0 (8) / PA1 (9) / PA9 (31) / PA10 (32) | TIM2 CH1 / CH2 / CH3 / CH4 → M4 / M3 / M1 / M2 DShot (one timer, DMA, bidirectional capture) |
| PA5/PA6/PA7 (13–15) + PC4/PB0/PB1 | SPI1 → IMU (SCK/MISO/MOSI) + CS/INT1/INT2 |
| PB3/PB4/PB5 (41–43) + PB6/PB7/PB9 + PF1 (6) | SPI3 → SX1280 + NSS/BUSY/DIO1 + NRESET |
| PB10 (22) / PC10 (39) / PA3 (11) / PF0 (5) | ESC1 / ESC2 / ESC3 / ESC4 SWDIO (bit-banged) |
| PB14 (27) / PB15 (28) | shared ESC SWCLK / shared ESC NRST (open-drain) |
| PB12 (25) | VBAT sense, ADC1_IN11 (10k/10k + 100 nF) |
| PC13 (2) | status LED (sinks ≤ 3 mA) |
| PA13/PA14 | own SWD (pogo strip J3: 3V0, SWDIO, SWCLK, NRST, GND) |
No HSE crystal (HSI16 + PLL → 170 MHz), so PF0/PF1 are plain GPIO; the radio has its own 52 MHz crystal.

Rev B pin swap: every FC signal now leaves the package on the side that faces its destination. In rev A, three
DShot lines, three ESC SWDIO lines, VBAT sense and the radio reset left on the far side. They wrapped around the
package and blocked the escape routing; the grid router then left about 18 nets unrouted. All functions are unchanged: DShot is still
TIM2 CH1–CH4 on one timer, and the SWD lines are bit-banged GPIO, so any pin works.

## PCB

| Item | Value |
|---|---|
| Size | 36 × 36 mm outline (22 × 22 mm hub + four Ø10 mm motor discs on 26 mm spacing), 794 mm² |
| Stackup | 6-layer, 0.8 mm, JLC: L1 1 oz / pp 0.0994 / L2 / core 0.16 / L3 / pp 0.13 / L4 / core 0.16 / L5 / pp 0.0994 / L6 1 oz, ENIG |
| Layer roles | L1 signals + GND pour · **L2 solid GND** · L3 signals + GND pour · **L4 VBAT plane on the hub only, GND fill in the motor discs** · **L5 solid GND** · L6 signals + RF feed + GND pour |
| Vias | 0.25/0.15 via-in-pad (filled + capped, IPC-4761 VII) on decoupling and exposed pads; 0.25/0.15 routing vias; 0.3/0.15 stitching. Every hole keeps ≥ 0.2 mm web to every other hole |
| Rules | JLC 6-layer: 0.09 mm track/space min. Digital and DShot nets use 0.09/0.09 (rev B; it was 0.10). Analog 0.10/0.12, phase 0.19/0.15 widened toward 0.5 after routing, RF 0.12/0.15. Every clearance is the larger of the two nets' classes. 0.25 mm copper-to-edge, 0.2 mm hole-to-hole |
| Routing layers | L1, L3 and L6 for all signals. **L4 is a restricted escape layer** (rev B), allowed only for digital, DShot, rail and analog nets, at 2-2.5x routing cost so it is used only where the other layers are full. Phase, RF and crystal nets never use it. No L4 track comes within 0.5 mm of the L5 relief cut-outs under the RF pads, because there L4 is the RF reference. Vias may pass through that ring; the cut-outs themselves stay clear. L4 tracks are striplines between the L3 pour and the L5 GND plane. The VBAT pour flows around them; the post-layout IR-drop and cavity studies (P2, P5) quantify the cuts. The low-current rails route at 0.10 mm, and +3V0 is widened again in the finish stage. Rev C as routed puts these lengths on L4: 122 mm digital/DShot, 14 mm rail, 10 mm analog. P3 checks layer-to-layer coupling between L3 phase tracks and L4 tracks: 15 nets share 16.3 mm, and the worst is M1_BEMFB at 29 dB margin |

### Placement
* FC (U2) at the centre, top. Four ESC clusters, one per hub corner, on the same side as their motor:
  - DRV8311 at (±7.4, ±7.4);
  - G071 at (±1.4, ±8.1);
  - motor wire pads in a column at x = ±10.0, level with the driver's OUT pins.
* Motor wire pads (J11–J14): 0.75 mm pads with 0.4 mm holes at 1.0 mm pitch, for 30 AWG leads, at x = ±10.3 mm,
  level with the driver's output pins. They sit beside the output edge, not at the driver corner. They clear the DRV pins by
  0.4 mm and the motor base by about 0.6 mm, and each phase trace is under 1 mm, so the switching loop is tiny.
* BEMF dividers (10k/10k):
  - Each divider is a compact pair on the side opposite its driver, in the open band between the driver and its
    ESC MCU. Motor 4's pairs sit east of the radio's RF chain, in a column at x = -2.20 / -2.85 mm.
  - Rev C moved that column 0.2 mm east. At rev B its bottom resistors sat right under ESC4's SWD pins (U24 pins
    20/21), so no via could stand there and the SWD pair could not leave the corner.
  - The phase sense line reaches each pair from the motor-pad through-hole, mostly on L3, which is shielded
    between the L2 GND and L4 VBAT planes. Only the divided node runs to the ADC pin.
  - Other spots were tried and failed: at the motor-pad column the BEMF pads were boxed in by the driver's
    back-side exposed pad; at the ESC MCU they sat over the neighbouring ESC MCU.
* Each driver's 100 nF VM capacitor sits on the opposite side, under its VM corner vias.
* Radio (U4) at front-left, bottom. Its rotation is forced so the RFIO pin row faces the antenna pad J2 on the
  front edge.

### RF feed
SX1280 RFIO → 0.6 mm stub straight out of the pin row → Semtech reference network packed into 5.2 mm:
- C22 0.8 pF shunt, L1 3.0 nH;
- C23 1.2 pF shunt, L2 2.5 nH ‖ C24 0.5 pF;
- C25 1.2 pF shunt, then C26 100 pF DC block;
- then a short run of grounded coplanar line to J2.

Each shunt cap's RF pad sits 0.05 mm from its node's series pad, so there is no stub. Its GND pad faces away from
the line and carries a filled via-in-pad. The line is 0.12 mm wide with a 0.15 mm gap on L6 over the L5 plane,
under mask: 50 Ω from the field solver. A GND via fence runs along both sides.

The optional antenna-tuning pi network (0R + 2 DNP shunts) was dropped. Once RFIO faced the antenna, the feed was
only 6.2 mm long and the full chain did not fit. C22/L1 remain the tuning elements.

### EMC measures
* Every signal layer sits next to an unbroken GND plane: L1↔L2, L6↔L5, and L3 between L2 (GND) and L4.
* VBAT plane (L4) only under the hub (±10.2 mm), 0.16 mm from GND (L5):
  - Narrow 1.4 mm lobes reach the few VBAT vias of the driver bulk caps at the hub edge.
  - Trimming it from the full 625 mm² outline to the hub moved the first plane-pair cavity mode from **2.39 GHz**
    (inside the 2.4 GHz ISM band) to **3.14 GHz** (sim_post P5, before L4 carried escape routing).
  - Rev C, as routed: the L4 escape tracks split the plane into smaller cells, which brings modes back to
    1.96 / 2.19 / 2.88 GHz. None falls inside 2.400-2.4835 GHz. The worst VM-port impedance in the band is 81 Ω (P5).
  - The motor discs carry GND on L4 instead.
  - DC drop (P2, rev C) at full throttle: 40 / 62 / 121 / 121 mV to U11 / U12 / U13 / U14.
    - The west drivers sit farthest from the battery pads (J1, east edge), and L4 routing cuts that side of the
      plane, so the verdict is MARGINAL against a 100 mV target.
    - Each ESC reads its own VBAT (E10), so throttle scaling sees the drop.
    - For a respin, feed the battery at the hub centre or keep L4 escapes off the west half.
* Board-edge GND via fence (0.9 mm pitch), return-path GND via within 1 mm of every signal via, extra GND vias
  around each driver.
* Motor drivers:
  - Each driver's two VM pins are joined by a 0.25 mm track around the package corner, with 2–3 VBAT vias there
    (12 in total).
  - Every GND pin ties straight into the exposed pad.
  - The exposed pad carries 9 filled vias.
  - The VM loop closes through the L4/L5 plane pair.
* Netclass separation rules (`kicad/nanodrone.kicad_dru`):
  - phase ↔ analog tracks ≥ 0.20 mm (rev C; 0.30 before);
  - phase ↔ other signal tracks ≥ 0.15 mm;
  - RF ↔ anything except GND ≥ 0.20 mm (by net name).

  A phase net and its own divided BEMF node (Mx_OUTy / Mx_BEMFy) only need the class clearance. They meet at their
  divider resistor, and coupling between them just adds an edge to that phase's own divided voltage, which is
  sampled ≥ 2.8 µs after every edge.

  At 0.20 mm, P3 shows ≥ 18 dB margin on every BEMF line. KiCad applies the last matching custom rule, so the
  file lists the specific phase-to-analog rule after the general phase-to-signal one. In rev B the order was
  reversed, so DRC only checked 0.15 mm there; the router enforced the full spacing anyway.
* Drivers use the slowest slew setting (SLEW = GND, 35 V/µs). Study 9 shows faster straps save ≤ 9 % loss but add
  7–16 dB of switching harmonics.

### Thermal (the main limitation; see `sim_post/POST_LAYOUT_REPORT.md`)
At 1S the DRV8311 has about 0.3 Ω in its bridge (HS + LS), so it dissipates about 0.55 W at hover (96 kHz PWM) and
about 2.3 W at full throttle.
- With prop-wash cooling, hover runs around 70–75 °C junction.
- Full throttle has a burst budget of a few seconds and must be limited in firmware with the compact Foster
  thermal model in the report.
- Do not run the motors at hover power without airflow (bench tests: use props, or limit to < 30 s).

## Firmware notes (Rust, later)
* FC: embassy-stm32 (STM32G431), ELRS-compatible link on the SX1280 (FLRC/LoRa), bidirectional DShot
  (RPM telemetry) to the ESCs, ESC flashing over SWD from the FC.
* ESC: embassy-stm32 (STM32G071): 6-step sensorless, ADC samples the floating phase at PWM centre, phase
  currents from the DRV8311 CSAs, VBAT from the driven-high phase.
* ESC PWM: **96 kHz**, complementary (study 9). Interleave the four ESCs' PWM by 90° to cut VBAT ripple (study 5b).
* Protection the hardware leaves to firmware:
  - start-up duty ≤ 0.24–0.39;
  - cycle-by-cycle current limit at about 3.5–4 A from the CSA;
  - VBAT-based throttle limit;
  - thermal limiting from the Foster model, seeded by the G071 temperature sensor;
  - stop synchronous braking above VBAT 4.6 V.
* ADC sampling time for the CSA outputs is about 1.6 µs, since there is no RC at the pin (study 6).
* IMU sits about 55–60 °C at hover: use gyro bias temperature compensation from the LSM6DSV16X temperature sensor.
