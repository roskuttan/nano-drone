# Firmware requirements from the hardware

These are the things the Rust firmware must do because of how the board is built. Each item cites the simulation
that sets it:
- `sim/` = pre-layout studies 1-9 (`sim/SIMULATION_REPORT.md`);
- `sim_post/` = post-layout studies P1-P6 (`sim_post/POST_LAYOUT_REPORT.md`).

## ESC (STM32G071, one per motor)

| # | Requirement | Value | Why / source |
|---|---|---|---|
| E1 | PWM frequency | **96 kHz** complementary (3x PWM mode, INHx = PWM, INLx = enable) | Hover driver loss 0.555 W vs 0.748 W at 48 kHz. 192 kHz is worse because of dead-time diode loss (study 9) |
| E2 | PWM phase between the 4 ESCs | 0 / 90 / 180 / 270 deg, synchronised from the FC | VBAT ripple about 0.4 V pk-pk drops to about 50 mV (study 5b) |
| E3 | Start-up and locked-rotor duty limit | ≤ 0.24 (if L_LL = 3 uH) … 0.39 (6 uH), measure L first | Locked-rotor current 8 A at D = 1, over the 5 A abs max; OCP only trips at 5.8 A min (study 2) |
| E4 | Cycle-by-cycle current limit | 3.5-4 A from the CSA (0.25 V/A, CSAREF = 3.0 V, midpoint 1.5 V) | Protects the 5 A peak rating that OCP does not cover (study 2) |
| E5 | Thermal limit | Estimate Tj per driver with the Foster model in sim_post P1, seeded by the G071 internal temperature sensor. Start derating at Tj_est 130 C, reach hover-level current at 150 C | Full throttle is time-limited: 125 C within a few seconds from hover (P1). TSD is 180 C min |
| E6 | Synchronous braking | Stop it when VBAT > 4.6 V | A disconnected battery during braking pumps VBAT far above 6.5 V (study 1) |
| E7 | BEMF ADC sampling | ts = 19.5 cycles (12-bit) or 12.5 (10-bit). Sample ≥ 2.8 us after the PWM edge. On-time sampling needs D ≥ 27 % at 96 kHz; below that, sample in the off-time | Divider Thevenin 5 kOhm (study 6). The phase-edge glitch settles within ns (P3) |
| E8 | CSA ADC sampling | ts ≈ 1.6 us (no RC at the pin). Sample the continuously-on low-side phase mid-interval | Ripple is 3-5 A pk-pk (studies 2, 6) |
| E9 | Zero-cross detection at high speed | ADC on the floating phase in continuous mode at 1-2 Msps (BEMF on PB1/PB0/PA7 = ADC1_IN9/IN8/IN7; none is a comparator input since the layout rev A pin swap) | Full throttle has only about 3 PWM periods per commutation step at 96 kHz, so per-PWM sampling is too coarse |
| E10 | VBAT measurement | From the BEMF divider of the phase driven high (VBAT/2), calibrated with VREFINT. Each ESC uses its own reading for duty and limit scaling | No dedicated pin. 3V0 tolerance gives ±63 mV error without calibration (study 6). The routed board drops up to 121 mV to the motor 3/4 drivers at full throttle, against 40-62 mV for motors 1/2 (P2), so a single FC-side VBAT value would be off by that much |
| E11 | Driver enable | nSLEEP (PB8) low during reset/boot. nFAULT (PB3) has no external pull-up since rev C: enable the PB3 internal pull-up (GPIO PUPDR) before nSLEEP goes high. Treat nFAULT as a fault only when two reads at least 2 µs apart are both low. Clear it by toggling nSLEEP | Bridges stay Hi-Z while the MCU resets. A phase-edge glitch on nFAULT stays below 20 mV and settles in under 2 µs with the 55 kΩ worst-case pull-up (P3) |
| E12 | Pin map | BEMF A/B/C = PB1/PB0/PA7, ISENS A/B = PA6/PA5, INH A/B/C = PA8/PA9/PA10 (TIM1 CH1-3, PA11/PA12 remapped), INL A/B/C = PC6/PA15/PB5, DShot = PB4 (TIM3 CH1), SWD = PA13/PA14 | `docs/ARCHITECTURE.md` |
| E13 | Reset pin mode | Option byte NRST_MODE = 01 (reset input only) on every ESC. The FC writes it when it flashes each ESC (F6) | ESC_NRST is one bus shared by all four ESCs. In the default bidirectional mode, an internal reset of one ESC (IWDG, BOR, software reset, option-byte reload) drives NRST low for at least 20 µs and resets the other three in flight |

## Flight controller (STM32G431)

| # | Requirement | Value | Why / source |
|---|---|---|---|
| F1 | Throttle limit vs battery | Keep VM ≥ 3.0 V (DRV8311 ROC). Scale the throttle ceiling with VBAT and pack resistance | Punch-outs below OCV ≈ 3.3-3.6 V brown the drivers out (UVLO 2.5-2.7 V, study 8) |
| F2 | Brown-out | Never enable BOR level 4 on the 3.0 V rail. Use BOR0 | BOR4 falls at 2.70-2.90 V and would reset in normal flight (study 8) |
| F3 | Gyro temperature compensation | Bias vs temperature from the LSM6DSV16X sensor | IMU sits around 55-60 C at hover, 25 C on the bench (P1) |
| F4 | Radio | SX1280 in **LDO mode** (no DC-DC inductor fitted). Budget twice the DC-DC current. 52 MHz crystal, no external load caps (internal trim). Board stray adds ≈ 0.4 pF, about 5 ppm pull (P6) | |
| F5 | RF tuning | Start from the BOM values (Semtech reference). Re-tune C22/L1 on a VNA with the real antenna. The P4 re-tune table is the alternative starting point | |
| F6 | ESC programming | Bit-banged SWD bus. ESC1/2/3/4_SWDIO are on PB10/PC10/PA3/PF0, with shared SWCLK (PB14) and shared ESC_NRST (PB15, open-drain). Flash the ESCs one at a time from the FC | Layout rev B pin swap (`docs/ARCHITECTURE.md`) |
| F8 | FC pin map, rev B | DShot = TIM2 CH1/CH2/CH3/CH4 on PA0/PA1/PA9/PA10, driving M4/M3/M1/M2 (one timer, DMA, bidirectional capture). VBAT_SENSE = PB12 (ADC1_IN11). RF_NRESET = PF1. HSE is unused, so PF0/PF1 are GPIO | Rev B routability swap; every function is unchanged |
| F9 | UCPD dead-battery pull-downs | Set PWR_CR3.UCPD_DBDIS at boot | The STM32G4 enables 5.1 kΩ Rd pull-downs on PB4/PB6 (RF_MISO/RF_NSS) after reset until firmware disables them |
| F7 | Bench testing | Do not hold hover power for more than about 30 s without prop wash (still air: Tj > 150 C, P1). Use props or a fan | |
| F10 | Gyro notch filtering | RPM-locked notches per motor (fundamental and 2nd/3rd harmonic) from the bidirectional-DShot eRPM of F8, plus one dynamic notch | The enclosure's duct rings have their first ovalising mode near 0.5 kHz, inside the motor-frequency sweep (`docs/ENCLOSURE.md`, `mech/enclosure_calcs.py`). Confirm on the blackbox gyro spectrum |
