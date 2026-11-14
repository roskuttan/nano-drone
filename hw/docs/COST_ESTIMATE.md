# Nano drone — cost estimate (Kerala, India)

Snapshot: 8 Nov 2026. Exchange rate assumed **₹96 / US$** (2026 quotes ranged ₹94.7–96.5).
"est" = my estimate; everything else is a listed price at the time of checking.

## 1. PCB batch from JLCPCB (the big one)

JLC only assembles panels ≥ 70 × 70 mm, so the 36 mm board ships as a **2 × 2 panel**.
Minimum assembly order is 2 panels → **8 assembled boards** (+ 12 bare spare boards from the 5-panel fab order).
Assembly must be *Standard PCBA* (0201 parts and 0.4 mm-pitch WQFN rule out *Economic*), double-sided.

### Parts per board (LCSC, distributed architecture rev 0.3)

| Part | Qty | Unit (US$) | Source |
|---|---:|---:|---|
| STM32G431CBU6 flight controller | 1 | in total below | LCSC C529356 |
| STM32G071GBU6 ESC MCU | 4 | in total below | LCSC C529347 |
| DRV8311HRRWR motor driver | 4 | 1.59 | LCSC C5218862 |
| SX1280IMLTRT radio | 1 | 4.64 | LCSC C125969 |
| LSM6DSV16XTR IMU | 1 | 3.39 | LCSC C5267406 |
| TPS7A2030PDQNR LDO | 1 | 0.24 | LCSC C5220164 |
| 52 MHz crystal, RF inductors, ferrites | 4 | ~0.08 avg | est |
| Passives (0201/0402/0603), LED | ~96 | ~0.003 avg | est |
| **Total per board** | 117 parts | **≈ $34.7** | LCSC stock check, 8 Nov 2026 (was $51.1 with the single STM32G474 WLCSP) |

The antenna-tuning pi network (0R + 2 DNP) was dropped in layout rev A, so the BOM is 3 lines shorter.
Rev C removed the four 10 k nFAULT pull-ups; the ESC MCU's internal pull-up replaces them. That is 4 parts fewer
(about $0.03 per board with placement).

### Batch cost (8 assembled boards)

| Item | US$ | ₹ |
|---|---:|---:|
| Components, 8 boards (+ ~$10 attrition) | 288 | 27,600 |
| Assembly: setup $51.12, double-sided stencil $16.42, ~400 joints/board × $0.0016, ~22 extended-part feeders × $1.53 (est), X-ray 88 leadless parts × $0.49, handling $14.93, packing | ~167 | 16,000 |
| PCB fab: 6-layer 0.8 mm, ENIG + epoxy-filled via-in-pad (free on 6-layer), impedance control, 5 panels | ~40 (est) | 3,800 |
| DHL shipping to Kerala | ~20 (est) | 1,900 |
| **Subtotal** | **~515** | **49,400** |
| Indian customs: BCD 10–20 % + IGST 18 % (est. 31–44 % effective) | — | 15,300 – 21,700 |
| **Landed batch cost** | | **≈ 64,700 – 71,100 (mid ≈ 67,900)** |
| **Per assembled board** | | **≈ ₹8,500** |

## 2. Drone hardware (per drone, Indian shops)

| Item | ₹ | Source |
|---|---:|---|
| BetaFPV 0702 motor set (4 pcs, 2026 version) | 5,999 | TUJORC |
| Gemfan 31 mm props, 2 × 8-pack | 652 | Evelta (₹276 + GST each) |
| 1S 300 mAh 75C BT2.0 batteries × 4 | 1,600 | Whoopmasters India ~₹399 each (was out of stock) |
| M1.4 screws, 3 mm nylon spacers, BT2.0 pigtail, antenna wire | 400 | est |
| Shop shipping | 300 | est |
| **Per drone** | **≈ 8,950** | |

## 3. One-time tools

| Item | ₹ | Source |
|---|---:|---|
| STLINK-V3MINIE debugger (for flashing the Rust firmware) | 2,805 | Evelta ₹2,377 + GST |
| BetaFPV 6-port 1S BT2.0 charger | 2,369 | InsideFPV |
| *Optional:* RadioMaster Pocket ELRS (skip if you own an ELRS radio) | 6,300 – 6,900 | Indian Hobby Center / Vega Drones / RCMumbai |

## 4. Totals

| Scenario | ₹ |
|---|---:|
| First flying drone (8-board batch + 1 drone's hardware + tools) | **≈ 82,000** |
| … plus an ELRS radio | ≈ 88,600 |
| Each extra drone after that (board already paid) | ≈ 8,950 |
| Average per drone if all 8 boards are built | ≈ 18,100 |

## 5. Ways to cut it

1. **Already done:** switching from the single STM32G474 WLCSP (only available from LCSC's third-party sellers at $35.82) to a G431 FC plus four G071 ESC MCUs saved about $16 per board and avoided HDI fab.
2. **4-board batch** — if JLC accepts a 1 × 2 panel with rails, landed cost ≈ ₹53,400 (but ₹13,400 per board).
3. **Customs** — small parcels sometimes clear duty-free, but budget for it. Make sure the invoice/HS code is correct for a PCBA.
4. Order a test **bare-board** batch first only if layout risk worries you — fab is cheap (~₹4k), assembly + MCU is where the money goes.

## Sources

- JLCPCB PCBA fees: https://jlcpcb.com/help/article/pcb-assembly-price
- JLCPCB 6-layer pricing / free ENIG + via-in-pad: https://jlcpcb.com/resources/multilayer-pcb , https://jlcpcb.com/6-layer-pcb
- JLC 70 × 70 mm PCBA minimum (help answers / capabilities): https://jlcpcb.com/capabilities/Capabilities
- LCSC parts: https://www.lcsc.com/product-detail/C2055584.html (others by C-number above)
- India import duty discussion (reseller blog, not authoritative): https://zbotic.in/how-to-order-pcb-from-jlcpcb-and-pcbway-india-cost-guide/
- Motors: https://tujorc.com/product/betafpv-0702-brushless-motors-2026-champion-freestyle-for-air-65-ii-whoop/
- Batteries: https://whoopmastersindia.com/product/65mm-whoop-battery-dogcom-1s-300mah-75c-hv-bt2-0/ , https://www.quadkart.in/bt2-0-300mah-1s-30c-battery-8pcs/
- STLINK-V3MINIE: https://evelta.com/stlink-v3minie-debugger-and-programmer
- Charger: https://insidefpv.com/products/betafpv-6-port-1s-charger-board-v2
- ELRS radio: https://indianhobbycenter.com/products/radiomaster-pocket-elrs-radio-controller-transparent
- Exchange rate: https://tradersunion.com/currencies/price/usd-inr/
