"""Nano drone flight controller + 4x ESC + radio - schematic as code.

Run with KiCad's Python:
    "C:/Users/angel/AppData/Local/Programs/KiCad/10.0/bin/python.exe" nanodrone_sch.py
Writes the KiCad project into ../kicad/.

Sources for every connection are cited inline (datasheet section / figure).
MCU pin assignment comes from pinmap.json (produced by tools/pinopt.py).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HW, "tools"))
import kisch as K  # noqa: E402

PRJ = os.path.join(HW, "kicad")
LIBDIR = os.path.join(PRJ, "lib")
os.makedirs(LIBDIR, exist_ok=True)
SYMLIB = os.path.join(LIBDIR, "nanodrone.kicad_sym")

# ---------------------------------------------------------------------------
# Footprints
# ---------------------------------------------------------------------------
C0201 = "Capacitor_SMD:C_0201_0603Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
R0201 = "Resistor_SMD:R_0201_0603Metric"
R0402 = "Resistor_SMD:R_0402_1005Metric"
L0402 = "Inductor_SMD:L_0402_1005Metric"
FB0402 = "Inductor_SMD:L_0402_1005Metric"
LED0402 = "LED_SMD:LED_0402_1005Metric"
TP = "TestPoint:TestPoint_Pad_D1.0mm"
# rev B option: per-ESC SWD pogo pads instead of the FC's SWD programming bus (NANODRONE_ESC_TP=1)
ESC_TP = os.environ.get("NANODRONE_ESC_TP") == "1"
# rev C option (NANODRONE_NFAULT_PU=0): no external 10k nFAULT pull-ups; the ESC MCU's PB3 internal pull-up does it
NFAULT_PU = os.environ.get("NANODRONE_NFAULT_PU", "0") == "1"   # rev C default: no external pull-ups
FP_ESC_SWD = "nanodrone:SWD_Pads_1x03_P1.0mm"
FP_DRV = "Package_DFN_QFN:Texas_RRW0024A_WQFN-24-1EP_3x3mm_P0.4mm_EP1.9x1.9mm_ThermalVias"
FP_MCU = "Package_CSP:ST_WLCSP-81_4.02x4.27mm_Layout9x9_P0.4mm"
FP_RF = "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.7x2.7mm_ThermalVias"
FP_IMU = "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
FP_LDO = "Package_SON:Texas_X2SON-4_1x1mm_P0.65mm"
FP_XTAL = "Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm"
FP_MOTOR_PADS = "nanodrone:MotorPads_3x_PTH"
FP_MOTOR_MOUNT = "nanodrone:Motor_0702_Mount"
FP_BAT = "nanodrone:BatteryPads_PTH"
FP_ANT = "nanodrone:AntennaPads_SMD"
FP_FID = "Fiducial:Fiducial_0.75mm_Mask1.5mm"

# ---------------------------------------------------------------------------
# Custom symbols (parts missing from the KiCad library)
# ---------------------------------------------------------------------------
SX1280 = K.make_symbol("SX1280IMLTRT", [
    # Semtech DS_SX1280-1 Rev 3.2, Table 2-1
    ("15", "VBAT", "power_in", "L"), ("11", "VBAT_IO", "power_in", "L"),
    ("2", "VDD_IN", "power_in", "L"), ("12", "DCC_FB", "power_out", "L"),
    ("1", "VR_PA", "power_out", "L"), ("14", "DCC_SW", "passive", "L"),
    ("4", "XTA", "passive", "L"), ("6", "XTB", "passive", "L"),
    ("19", "NSS_CTS", "input", "R"), ("18", "SCK_RTSN", "input", "R"),
    ("17", "MOSI_RX", "input", "R"), ("16", "MISO_TX", "output", "R"),
    ("3", "NRESET", "input", "R"), ("7", "BUSY", "output", "R"),
    ("8", "DIO1", "bidirectional", "R"), ("9", "DIO2", "bidirectional", "R"),
    ("10", "DIO3", "bidirectional", "R"), ("22", "RFIO", "passive", "R"),
    ("5", "GND", "power_in", "B"), ("13", "GND", "power_in", "B"), ("20", "GND", "power_in", "B"),
    ("21", "GND", "power_in", "B"), ("23", "GND", "power_in", "B"), ("24", "GND", "power_in", "B"),
    ("25", "EP", "power_in", "B"),
], ref="U", footprint=FP_RF, datasheet="https://www.semtech.com/products/wireless-rf/lora-connect/sx1280",
    description="2.4 GHz LoRa/FLRC/GFSK transceiver, QFN-24 4x4")

LSM6DSV16X = K.make_symbol("LSM6DSV16X", [
    # ST DS13510 Rev 4, Table 2 (mode 1)
    ("8", "VDD", "power_in", "L"), ("5", "VDDIO", "power_in", "L"),
    ("2", "SDX", "input", "L"), ("3", "SCX", "input", "L"),
    ("10", "OCS_AUX", "passive", "L"), ("11", "SDO_AUX", "passive", "L"),
    ("12", "CS", "input", "R"), ("13", "SCL/SPC", "input", "R"), ("14", "SDA/SDI", "bidirectional", "R"),
    ("1", "SDO/SA0", "output", "R"), ("4", "INT1", "output", "R"), ("9", "INT2", "output", "R"),
    ("6", "GND", "power_in", "B"), ("7", "GND", "power_in", "B"),
], ref="U", footprint=FP_IMU, datasheet="https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf",
    description="6-axis IMU, LGA-14L 2.5x3x0.86")

K.write_symbol_lib(SYMLIB, [SX1280, LSM6DSV16X])
K.CUSTOM_LIBS["nanodrone"] = SYMLIB
K.write_project_files(PRJ, "nanodrone",
                      sym_libs={"nanodrone": "lib/nanodrone.kicad_sym"},
                      fp_libs={"nanodrone": "lib/nanodrone.pretty"})

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_n = {}


def ref(prefix):
    _n[prefix] = _n.get(prefix, 0) + 1
    return "%s%d" % (prefix, _n[prefix])


def C(sheet, a, b, value, fp=C0201, **props):
    return sheet.add(K.Part(ref("C"), "Device:C", {"1": a, "2": b}, value=value, footprint=fp, props=props))


def R(sheet, a, b, value, fp=R0201, **props):
    return sheet.add(K.Part(ref("R"), "Device:R", {"1": a, "2": b}, value=value, footprint=fp, props=props))


def L(sheet, a, b, value, fp=L0402, **props):
    return sheet.add(K.Part(ref("L"), "Device:L", {"1": a, "2": b}, value=value, footprint=fp, props=props))


def TPt(sheet, net, value):
    return sheet.add(K.Part(ref("TP"), "Connector:TestPoint", {"1": net}, value=value, footprint=TP))


# ---------------------------------------------------------------------------
# Distributed architecture (v2): flight controller STM32G431CBU6 + one STM32G071GBU6 per motor.
# Pin maps chosen by hand from ST open pin data (design/STM32G431CxUx_pindata.xml,
# design/STM32G071GxUx_pindata.xml); every function below was checked against those files.
# ---------------------------------------------------------------------------
FP_FC = "Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm"   # EP GND vias added in layout
FP_ESC = "Package_DFN_QFN:QFN-28_4x4mm_P0.5mm"

FC_PINS = {   # STM32G431CBU6 UFQFPN48 pin number -> net
    # Layout rev B pin swap (routability): every pin that leaves the package does so on the side that faces its
    # destination. Package sides: W = pins 1-12 (N->S), S = 13-24 (W->E), E = 25-36 (S->N), N = 37-48 (E->W).
    "1": "+3V0",                      # VBAT (no backup battery)
    "2": "LED_K",                     # PC13 sinks the status LED (<=3 mA)
    "5": "ESC4_SWDIO",                # PF0 (HSE unused: GPIO), W side towards ESC4 (north-west)
    "6": "RF_NRESET",                 # PF1, W side towards the radio
    "7": "NRST",                      # PG10-NRST
    # DShot on TIM2_CH1..CH4 (one timer, DMA, bidirectional capture), each channel on the side of its ESC
    "8": "M4_DSHOT", "9": "M3_DSHOT",                                      # PA0 TIM2_CH1, PA1 TIM2_CH2 (W)
    "31": "M1_DSHOT", "32": "M2_DSHOT",                                    # PA9 TIM2_CH3, PA10 TIM2_CH4 (E)
    "11": "ESC3_SWDIO",               # PA3, W side towards ESC3 (south-west)
    "25": "VBAT_SENSE",               # PB12 ADC1_IN11, E side towards the divider
    "13": "IMU_SCK", "14": "IMU_MISO", "15": "IMU_MOSI",                   # SPI1 PA5/PA6/PA7
    "16": "IMU_CS", "17": "IMU_INT1", "18": "IMU_INT2",                    # PC4, PB0, PB1
    "20": "+3V0A", "21": "+3V0A",     # VREF+, VDDA (via ferrite)
    "22": "ESC1_SWDIO",               # PB10, S side towards ESC1 (south-east)
    "39": "ESC2_SWDIO",               # PC10, N side towards ESC2 (north-east)
    "27": "ESC_SWCLK", "28": "ESC_NRST",                                   # PB14, PB15 (open-drain)
    "23": "+3V0", "35": "+3V0", "48": "+3V0", "49": "GND",
    "36": "SWDIO", "37": "SWCLK",     # PA13, PA14
    "41": "RF_SCK", "42": "RF_MISO", "43": "RF_MOSI",                      # SPI3 PB3/PB4/PB5
    "44": "RF_NSS", "45": "RF_BUSY", "46": "BOOT0", "47": "RF_DIO1",       # PB6, PB7, PB8-BOOT0, PB9
}
if ESC_TP:      # no SWD bus to the ESCs: PF0, PA3, PB10, PC10, PB14, PB15 stay free
    for _pin in ("5", "11", "22", "39", "27", "28"):
        FC_PINS.pop(_pin)

G071_PINS = [   # (pin, name, type, side) - STM32G071GBU6 UFQFPN28 (DS12232 / ST open pin data)
    ("3", "VDD", "power_in", "L"), ("4", "VSS", "power_in", "L"), ("5", "PF2-NRST", "bidirectional", "L"),
    ("6", "PA0", "bidirectional", "L"), ("7", "PA1", "bidirectional", "L"), ("8", "PA2", "bidirectional", "L"),
    ("9", "PA3", "bidirectional", "L"), ("10", "PA4", "bidirectional", "L"), ("11", "PA5", "bidirectional", "L"),
    ("12", "PA6", "bidirectional", "L"), ("13", "PA7", "bidirectional", "L"), ("14", "PB0", "bidirectional", "L"),
    ("15", "PB1", "bidirectional", "L"), ("1", "PC14", "bidirectional", "L"),
    ("16", "PA8", "bidirectional", "R"), ("17", "PC6", "bidirectional", "R"),
    ("18", "PA11[PA9]", "bidirectional", "R"), ("19", "PA12[PA10]", "bidirectional", "R"),
    ("20", "PA13", "bidirectional", "R"), ("21", "PA14-BOOT0", "bidirectional", "R"),
    ("22", "PA15", "bidirectional", "R"), ("23", "PB3", "bidirectional", "R"), ("24", "PB4", "bidirectional", "R"),
    ("25", "PB5", "bidirectional", "R"), ("26", "PB6", "bidirectional", "R"), ("27", "PB7", "bidirectional", "R"),
    ("28", "PB8", "bidirectional", "R"), ("2", "PC15", "bidirectional", "R"),
]
G071 = K.make_symbol("STM32G071GBU6", G071_PINS, ref="U", footprint=FP_ESC,
                     datasheet="https://www.st.com/resource/en/datasheet/stm32g071gb.pdf",
                     description="Arm Cortex-M0+ 64 MHz, 128 KB flash, UFQFPN28 4x4")
K.write_symbol_lib(SYMLIB, [SX1280, LSM6DSV16X, G071])


def esc_pins(m):
    p = "M%d_" % m
    pins = {   # STM32G071GBU6 pin -> net (per motor ESC)
        "3": "+3V0", "4": "GND", "5": "ESC_NRST",
        # BEMF + CSA inputs on the ADC pins of the package corner that faces the driver (routability: the old
        # PA0-PA2 sat on the far edge, over the neighbouring ESC MCU on the other side). Zero-cross detection
        # is ADC-based (2.5 Msps); none of these pins is a comparator input.
        "15": p + "BEMFA", "14": p + "BEMFB", "13": p + "BEMFC",   # PB1 ADC1_IN9, PB0 ADC1_IN8, PA7 ADC1_IN7
        "12": p + "ISENSA", "11": p + "ISENSB",                    # PA6 ADC1_IN6, PA5 ADC1_IN5
        "16": p + "INHA", "18": p + "INHB", "19": p + "INHC",      # TIM1_CH1 PA8, TIM1_CH2 PA9*, TIM1_CH3 PA10*
        "17": p + "INLA", "22": p + "INLB", "25": p + "INLC",      # PC6, PA15, PB5
        "24": p + "DSHOT",                                         # PB4 TIM3_CH1 (DMA, bidirectional DShot)
        "28": p + "nSLEEP", "23": p + "nFAULT",                    # PB8 out, PB3 in (corner next to the driver)
        "20": "ESC%d_SWDIO" % m, "21": "ESC_SWCLK",                # PA13, PA14 (SWD from the FC)
    }   # * pins 18/19 = PA9/PA10 after SYSCFG PA11/PA12 remap
    if ESC_TP:
        # rev B: no SWD bus from the FC (its six board-spanning nets were the routing bottleneck). Each ESC has
        # its own pogo pads for the one-time bootloader flash; updates then go over the DShot wire.
        pins.update({"5": p + "NRST", "20": p + "SWDIO", "21": p + "SWCLK"})
    return pins


d = K.Design("nanodrone", "Nano drone FC + 4x ESC + ELRS radio", rev="0.2", company="")

# ---------------------------------------------------------------------------
# Sheet: power
# ---------------------------------------------------------------------------
pw = d.sheet("power", "Power", notes=[
    "1S LiPo/LiHV (3.0-4.35 V). VBAT feeds the four DRV8311 directly; TPS7A2030 makes +3V0 for logic.",
    "Bulk: 2x22uF here + 22uF at each driver. Hot-plug ringing checked in SPICE (sim/hotplug.cir).",
])
pw.add(K.Part("J1", "Connector_Generic:Conn_01x02", {"1": "VBAT", "2": "GND"}, value="BATTERY 1S",
              footprint=FP_BAT))
C(pw, "VBAT", "GND", "22uF 10V X5R", C0603)
C(pw, "VBAT", "GND", "22uF 10V X5R", C0603)
# TPS7A20 datasheet: CIN >= 1 uF, COUT 1 uF min effective; EN tied to IN
pw.add(K.Part("U1", "Regulator_Linear:TPS7A20xxxDQN",
              {"IN": "VBAT", "EN": "VBAT", "OUT": "+3V0", "GND": "GND", "5": "GND"},
              value="TPS7A2030PDQNR", footprint=FP_LDO, props={"MPN": "TPS7A2030PDQNR"}))
C(pw, "VBAT", "GND", "1uF 10V", C0402)
C(pw, "+3V0", "GND", "2.2uF 6.3V", C0402)
# battery voltage sense: ratio 0.5 (same as BEMF dividers), RC 5k*100n = 0.5 ms
R(pw, "VBAT", "VBAT_SENSE", "10k 1%")
R(pw, "VBAT_SENSE", "GND", "10k 1%")
C(pw, "VBAT_SENSE", "GND", "100nF")
pw.flags = ["VBAT", "GND"]

# ---------------------------------------------------------------------------
# Sheet: MCU
# ---------------------------------------------------------------------------
mcu = d.sheet("mcu", "Flight controller STM32G431", notes=[
    "STM32G431CBU6 UFQFPN48: SPI1 -> IMU, SPI3 -> SX1280, TIM2 CH1-4 -> bidirectional DShot to the 4 ESCs.",
    ("The ESC MCUs have their own SWD pogo pads (one-time bootloader flash); firmware updates go over DShot."
     if ESC_TP else
     "PB10/PC10/PA3/PF0 + PB14/PB15 form an SWD programming bus to the ESC MCUs (own SWDIO each, shared SWCLK "
     "+ NRST)."),
])
mcu.add(K.Part("U2", "MCU_ST_STM32G4:STM32G431CBUx", FC_PINS, value="STM32G431CBU6", footprint=FP_FC,
               props={"MPN": "STM32G431CBU6", "LCSC": "C529356"}))
for _ in range(3):                                          # one per VDD pin (23, 35, 48)
    C(mcu, "+3V0", "GND", "100nF")
C(mcu, "+3V0", "GND", "4.7uF 6.3V", C0402)
# VDDA filter: 10R instead of a ferrite - the bead + 1 uF resonated (x5.6 noise gain at 333 kHz, sim study 5a)
R(mcu, "+3V0", "+3V0A", "10R")
C(mcu, "+3V0A", "GND", "1uF 6.3V")
C(mcu, "+3V0A", "GND", "100nF")
C(mcu, "NRST", "GND", "100nF")
R(mcu, "BOOT0", "GND", "10k")
if not ESC_TP:
    R(mcu, "ESC_NRST", "+3V0", "10k")                       # ESC reset bus pull-up (FC drives open-drain)
R(mcu, "+3V0", "LED_A", "1k")
mcu.add(K.Part(ref("D"), "Device:LED", {"A": "LED_A", "K": "LED_K"}, value="RED", footprint=LED0402))
mcu.add(K.Part("J3", "Connector_Generic:Conn_01x05",
               {"1": "+3V0", "2": "SWDIO", "3": "SWCLK", "4": "NRST", "5": "GND"}, value="SWD",
               footprint="nanodrone:SWD_Pads_1x05_P1.27mm"))
for _ in range(4):
    mcu.add(K.Part(ref("FID"), "Mechanical:Fiducial", {}, value="Fiducial",
                   footprint="nanodrone:Fiducial_0.6mm_Mask1.2mm"))
mcu.flags = ["+3V0A"]          # rail behind ferrite FB: ERC cannot see power through passives

# ---------------------------------------------------------------------------
# Sheet: IMU
# ---------------------------------------------------------------------------
imu = d.sheet("imu", "IMU LSM6DSV16X", notes=[
    "SPI 4-wire, mode 1 (DS13510 fig.28). SDx/SCx tied to GND (analog hub/Qvar off); aux pins NC.",
    "Placed at the centre of the bottom side, under the MCU, at the craft's centre of mass.",
])
imu.add(K.Part("U3", "nanodrone:LSM6DSV16X",
               {"VDD": "+3V0_IMU", "VDDIO": "+3V0", "SDX": "GND", "SCX": "GND", "CS": "IMU_CS",
                "SCL/SPC": "IMU_SCK", "SDA/SDI": "IMU_MOSI", "SDO/SA0": "IMU_MISO",
                "INT1": "IMU_INT1", "INT2": "IMU_INT2", "GND": "GND"},
               value="LSM6DSV16XTR", footprint=FP_IMU, props={"MPN": "LSM6DSV16XTR", "LCSC": "C5267406"}))
R(imu, "+3V0", "+3V0_IMU", "10R")                           # RC filter on the sensor core supply
C(imu, "+3V0_IMU", "GND", "1uF 6.3V")
C(imu, "+3V0_IMU", "GND", "100nF")
C(imu, "+3V0", "GND", "100nF")
imu.flags = ["+3V0_IMU"]

# ---------------------------------------------------------------------------
# Sheet: radio
# ---------------------------------------------------------------------------
rf = d.sheet("radio", "Radio SX1280", notes=[
    "Semtech DS_SX1280-1 Rev 3.2 fig.15-4 (LDO mode, no 15 uH inductor) + 15.1 reference RF network.",
    "RF parts kept 0402 as in the validated reference; antenna pi pads (0R + 2x DNP) for tuning.",
])
rf.add(K.Part("U4", "nanodrone:SX1280IMLTRT",
              {"VBAT": "+3V0_RF", "VBAT_IO": "+3V0_RF", "VDD_IN": "RF_VREG", "DCC_FB": "RF_VREG",
               "VR_PA": "RF_VRPA", "XTA": "RF_XTA", "XTB": "RF_XTB", "NSS_CTS": "RF_NSS",
               "SCK_RTSN": "RF_SCK", "MOSI_RX": "RF_MOSI", "MISO_TX": "RF_MISO", "NRESET": "RF_NRESET",
               "BUSY": "RF_BUSY", "DIO1": "RF_DIO1", "RFIO": "RF_IO", "GND": "GND", "EP": "GND"},
              value="SX1280IMLTRT", footprint=FP_RF, props={"MPN": "SX1280IMLTRT", "LCSC": "C125969"}))
rf.add(K.Part(ref("FB"), "Device:L", {"1": "+3V0", "2": "+3V0_RF"}, value="600R@100MHz", footprint=FB0402))
C(rf, "+3V0_RF", "GND", "100nF", C0402)     # C10 VBAT
C(rf, "+3V0_RF", "GND", "100nF", C0402)     # C8 VBAT_IO
C(rf, "+3V0_RF", "GND", "4.7uF 6.3V", C0402)
C(rf, "RF_VRPA", "GND", "10nF", C0402)      # C5
C(rf, "RF_VREG", "GND", "10nF", C0402)      # C6
C(rf, "RF_VREG", "GND", "470nF", C0402)     # C9
rf.add(K.Part("Y1", "Device:Crystal_GND24", {"1": "RF_XTA", "3": "RF_XTB", "2": "GND", "4": "GND"},
              value="52MHz 10pF 10ppm", footprint=FP_XTAL))
# RF network (reference design 15.1): C0 shunt, L1 series, pi filter C1-L2||C4-C2, C3 DC block
C(rf, "RF_IO", "GND", "0.8pF C0G", C0402)
L(rf, "RF_IO", "RF_N1", "3.0nH")
C(rf, "RF_N1", "GND", "1.2pF C0G", C0402)
L(rf, "RF_N1", "RF_N2", "2.5nH")
C(rf, "RF_N1", "RF_N2", "0.5pF C0G", C0402)
C(rf, "RF_N2", "GND", "1.2pF C0G", C0402)
C(rf, "RF_N2", "ANT", "100pF C0G", C0402)  # C3 DC block straight to the antenna pad
# (the optional antenna-tuning pi - 0R + 2 DNP shunts - was dropped: with RFIO facing the antenna the feed is
#  6.2 mm long and the full chain did not fit; C0/L1 remain the tuning elements, see sim_post p4)
rf.add(K.Part("J2", "Connector_Generic:Conn_01x02", {"1": "ANT", "2": "GND"}, value="ANTENNA 50R",
              footprint=FP_ANT))
rf.flags = ["+3V0_RF"]

# ---------------------------------------------------------------------------
# Sheets: motor 1..4 (Betaflight quad-X order: 1 RR, 2 FR, 3 RL, 4 FL)
# ---------------------------------------------------------------------------
POS = {1: "rear-right (top-mounted)", 2: "front-right (bottom-mounted)",
       3: "rear-left (bottom-mounted)", 4: "front-left (top-mounted)"}
for m in (1, 2, 3, 4):
    s = d.sheet("motor%d" % m, "Motor %d ESC" % m, notes=[
        "ESC MCU STM32G071GBU6 + DRV8311H (3x PWM: MODE Hi-Z => OCP 9 A typ / 5.8 A min, SLEW=GND 35 V/us, "
        "GAIN=GND 0.25 V/A).",
        "BEMF dividers 10k/10k (ratio 0.5); VBAT read from the driven-high phase. CSA A,B -> ADC. "
        "DShot from FC. Motor %d: %s." % (m, POS[m]),
    ])
    p = "M%d_" % m
    s.add(K.Part("U%d" % (20 + m), "nanodrone:STM32G071GBU6", esc_pins(m), value="STM32G071GBU6",
                 footprint=FP_ESC, props={"MPN": "STM32G071GBU6", "LCSC": "C529347"}))
    C(s, "+3V0", "GND", "100nF")                            # ESC MCU VDD decoupling
    C(s, "+3V0", "GND", "1uF 6.3V")
    if ESC_TP:
        s.add(K.Part("J%d" % (20 + m), "Connector_Generic:Conn_01x03",
                     {"1": p + "SWDIO", "2": p + "SWCLK", "3": p + "NRST"}, value="SWD",
                     footprint=FP_ESC_SWD))
        C(s, p + "NRST", "GND", "100nF")                    # NRST filter next to a switching driver (AN5096)
    s.add(K.Part("U%d" % (10 + m), "Driver_Motor:DRV8311H", {
        "~{FAULT}": p + "nFAULT", "CSAREF": "+3V0", "SOA": p + "ISENSA", "SOB": p + "ISENSB",
        "CP": p + "CP", "VIN_AVDD": "VBAT", "VM": "VBAT", "PGND": "GND", "AGND": "GND",
        "OUTA": p + "OUTA", "OUTB": p + "OUTB", "OUTC": p + "OUTC",
        "INHA": p + "INHA", "INHB": p + "INHB", "INHC": p + "INHC",
        "INLA": p + "INLA", "INLB": p + "INLB", "INLC": p + "INLC",
        "AVDD": p + "AVDD", "GAIN": "GND", "SLEW": "GND", "~{SLEEP}": p + "nSLEEP",
    }, value="DRV8311HRRWR", footprint=FP_DRV, props={"MPN": "DRV8311HRRWR", "LCSC": "C5218862"}))
    if NFAULT_PU:
        R(s, p + "nFAULT", "+3V0", "10k")                   # nFAULT open-drain pull-up
    else:
        ref("R")                                            # rev C: PB3 internal pull-up; later refs keep numbers
    C(s, p + "CP", "VBAT", "100nF 10V")                     # CCP between CP and VM
    C(s, p + "AVDD", "GND", "1uF 6.3V")                     # CAVDD 0.7..7 uF
    C(s, "VBAT", "GND", "100nF 10V")                        # CVM1 / CVIN_AVDD1 (pins 7,8 adjacent)
    C(s, "VBAT", "GND", "22uF 10V X5R", C0603)              # CVM2 bulk
    C(s, "+3V0", "GND", "100nF")                            # CCSAREF
    for ph in "ABC":
        R(s, p + "OUT" + ph, p + "BEMF" + ph, "10k 1%")
        R(s, p + "BEMF" + ph, "GND", "10k 1%")
    s.add(K.Part("J%d" % (10 + m), "Connector_Generic:Conn_01x03",
                 {"1": p + "OUTA", "2": p + "OUTB", "3": p + "OUTC"}, value="MOTOR%d 0702" % m,
                 footprint=FP_MOTOR_PADS))
    s.add(K.Part("H%d" % m, "Mechanical:MountingHole", {}, value="0702 mount", footprint=FP_MOTOR_MOUNT))

# ---------------------------------------------------------------------------
warns = d.check()
for w in warns:
    print("WARN", w)
d.write(PRJ)
print("schematic written to", PRJ)
print("parts:", sum(len(s.parts) for s in d.sheets), "nets:", len(d.nets()))
