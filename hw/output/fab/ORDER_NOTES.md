# JLCPCB order notes: nano drone flight board (rev A)

## PCB
| Option | Value |
|---|---|
| Layers | 6 |
| Thickness | 0.8 mm |
| Stackup | JLC 6-layer 0.8 mm, prepreg 2116 between L1-L2 and L5-L6 (0.0994 mm); see the table below |
| Outer copper | 1 oz finished |
| Inner copper | 0.5 oz |
| Min track / space | 0.09 / 0.09 mm (digital and DShot nets use 0.09 / 0.09; analog 0.10 / 0.12; phase 0.19 / 0.15) |
| Min via | 0.25 mm pad / 0.15 mm drill |
| Via covering | **Epoxy filled and capped (POFV / via-in-pad)**. Pads carry filled vias: DRV8311 exposed pads, decoupling caps, STM32G431 exposed pad |
| Surface finish | ENIG (flat pads for 0.4 mm-pitch WQFN, 0.5 mm-pitch QFN and LGA-14) |
| Impedance control | Request it if JLC offers it for the 0.8 mm 6-layer stackup: RF feed on L6 over L5, grounded coplanar, W 0.12 mm, gap 0.15 mm, under solder mask, 50 ohm. If it is not offered, order without it: over the stackup tolerance the line is 48-53.5 ohm (sim_post P4), which is fine for a 6 mm feed |
| RF pad relief | L5 has rule-area cut-outs under the signal pads of the matching network (zone fill only, no copper removed elsewhere). Keep them: they set the network's parasitics |
| Board outline | Hub 22 x 22 mm + four motor discs (36 x 36 mm envelope), routed edge |
| Solder mask / silk | Any colour. Both sides have components |

### Stackup (as entered in the board file)
| Layer | Role | Thickness |
|---|---|---|
| L1 F.Cu | signals + GND pour | 35 um |
| prepreg 2116 | er 4.45 | 0.0994 mm |
| L2 In1.Cu | GND plane | 15.2 um |
| core | er 4.6 | 0.16 mm |
| L3 In2.Cu | signals + GND pour | 15.2 um |
| prepreg | er 4.45 | 0.13 mm |
| L4 In3.Cu | VBAT plane (hub only) + GND fill | 15.2 um |
| core | er 4.6 | 0.16 mm |
| L5 In4.Cu | GND plane | 15.2 um |
| prepreg 2116 | er 4.45 | 0.0994 mm |
| L6 B.Cu | signals + RF feed + GND pour | 35 um |

## Assembly (PCBA, both sides)
* BOM: nanodrone_BOM_JLC.csv (29 lines, 102 parts). CPL: nanodrone_CPL_JLC.csv (102 placements).
* Check every rotation in JLC's placement preview, especially the QFN/WQFN/LGA parts (U2, U4, U11-U14, U21-U24, U3)
  and the crystal Y1. JLC's library zero orientation sometimes differs from KiCad's.
* Not assembled (hand-soldered by the user): battery pads J1, antenna pads J2, motor wire pads J11-J14, SWD pads J3.
* Fiducials: FID1-FID4 (1 mm mask opening ring), two per side.

## After assembly
* Motor wires solder to J11-J14 (PTH 0.5 mm). Phase order follows the silk A/B/C. Swap two wires or reverse in
  firmware to change direction.
* Antenna: a 2.4 GHz dipole or whip on J2 (pad 1 = feed, pad 2 = GND). Tune C22/L1 on a VNA if the antenna is not 50 ohm.
* First power-up: current-limited bench supply at 3.7 V, 0.3 A limit. Expected idle current is about 0.1 A.
