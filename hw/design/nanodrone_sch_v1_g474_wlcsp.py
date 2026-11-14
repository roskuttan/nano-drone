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
FP_DRV = "Package_DFN_QFN:Texas_RRW0024A_WQFN-24-1EP_3x3mm_P0.4mm_EP1.9x1.9mm_ThermalVias"
FP_MCU = "Package_CSP:ST_WLCSP-81_4.02x4.27mm_Layout9x9_P0.4mm"
FP_RF = "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.7x2.7mm_ThermalVias"
FP_IMU = "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
FP_LDO = "Package_SON:Texas_X2SON-4_1x1mm_P0.65mm"
FP_XTAL = "Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm"
FP_MOTOR_PADS = "nanodrone:MotorPads_3x_PTH"
FP_MOTOR_MOUNT = "nanodrone:Motor_0702_Mount"
FP_BAT = "nanodrone:BatteryPads_PTH"
FP_ANT = "nanodrone:AntennaPads_PTH"
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


pinmap = json.load(open(os.path.join(HERE, "pinmap.json")))
BALL = {row["net"]: row["ball"] for row in pinmap["pins"]}
# spare-pin use (see ARCHITECTURE.md): PC15 (D9) input-only -> IMU INT2, PC6 (E3) -> debug pad
BALL["IMU_INT2"] = "D9"

d = K.Design("nanodrone", "Nano drone FC + 4in1 ESC + ELRS radio", rev="0.1", company="")

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
mcu = d.sheet("mcu", "MCU STM32G474", notes=[
    "STM32G474MEY6 WLCSP81. Pin map from tools/pinopt.py (design/pinmap.json): each motor has its own",
    "timer (TIM8/TIM1/TIM4/TIM5) and its own ADC (ADC5/ADC4/ADC1/ADC2). No HSE: HSI16 + PLL -> 170 MHz.",
])
mnets = {"A1": "+3V0", "A9": "+3V0", "E1": "+3V0", "J1": "+3V0", "J9": "+3V0", "B9": "+3V0",
         "A8": "GND", "B1": "GND", "F1": "GND", "H9": "GND", "J2": "GND", "H6": "GND",
         "J5": "+3V0A", "J6": "+3V0A",                     # VDDA, VREF+ (DS12288 fig.16)
         "D8": "NRST", "B7": "BOOT0", "B2": "SWDIO", "C3": "SWCLK"}
for net, ball in BALL.items():
    mnets[ball] = net
mcu.add(K.Part("U2", "MCU_ST_STM32G4:STM32G474MEYx", mnets, value="STM32G474MEY6TR", footprint=FP_MCU,
               props={"MPN": "STM32G474MEY6TR", "LCSC": "C2055584"}))
for _ in range(5):                                          # one per VDD ball
    C(mcu, "+3V0", "GND", "100nF")
C(mcu, "+3V0", "GND", "4.7uF 6.3V", C0402)
mcu.add(K.Part(ref("FB"), "Device:L", {"1": "+3V0", "2": "+3V0A"}, value="600R@100MHz", footprint=FB0402))
C(mcu, "+3V0A", "GND", "1uF 6.3V")
C(mcu, "+3V0A", "GND", "100nF")
C(mcu, "+3V0A", "GND", "10nF")
C(mcu, "NRST", "GND", "100nF")
R(mcu, "BOOT0", "GND", "10k")
R(mcu, "LED", "LED_A", "470R")
mcu.add(K.Part(ref("D"), "Device:LED", {"A": "LED_A", "K": "GND"}, value="RED", footprint=LED0402))
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
C(rf, "RF_N2", "RF_N3", "100pF C0G", C0402)
# antenna tuning pi (fitted 0R, shunts DNP)
C(rf, "RF_N3", "GND", "DNP", C0402)
R(rf, "RF_N3", "ANT", "0R", R0402)
C(rf, "ANT", "GND", "DNP", C0402)
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
        "DRV8311H, 3x PWM mode (MODE pin Hi-Z => OCP 9 A typ / 5.8 A min). SLEW=GND (35 V/us), "
        "GAIN=GND (0.25 V/A).",
        "BEMF dividers 10k/10k (ratio 0.5). CSA on phases A,B -> MCU ADC. Motor %d: %s." % (m, POS[m]),
    ])
    p = "M%d_" % m
    s.add(K.Part("U%d" % (10 + m), "Driver_Motor:DRV8311H", {
        "~{FAULT}": "MOT_nFAULT", "CSAREF": "+3V0", "SOA": p + "ISENSA", "SOB": p + "ISENSB",
        "CP": p + "CP", "VIN_AVDD": "VBAT", "VM": "VBAT", "PGND": "GND", "AGND": "GND",
        "OUTA": p + "OUTA", "OUTB": p + "OUTB", "OUTC": p + "OUTC",
        "INHA": p + "INHA", "INHB": p + "INHB", "INHC": p + "INHC",
        "INLA": p + "INLA", "INLB": p + "INLB", "INLC": p + "INLC",
        "AVDD": p + "AVDD", "GAIN": "GND", "SLEW": "GND", "~{SLEEP}": "MOT_nSLEEP",
    }, value="DRV8311HRRWR", footprint=FP_DRV, props={"MPN": "DRV8311HRRWR", "LCSC": "C5218862"}))
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
