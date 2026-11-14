"""pinsolve - assign MCU pins for the nano drone from ST open pin data XML.

Requirements (DRV8311H in 3x PWM mode, 6-step sensorless):
  * 4 motors x 3 PWM channels, each motor on its own timer (CH1..CH4, no CHxN)
  * 12 BEMF + 1 VBAT analog inputs (any ADC instance)
  * 12 INLx enable GPIOs
  * IMU SPI (SCK/MISO/MOSI) + CS + INT
  * Radio: SPI (SCK/MISO/MOSI) + N GPIOs, or a UART + N GPIOs
  * nSLEEP, nFAULT, LED GPIOs; SWD on PA13/PA14
Usage: python pinsolve.py <xml> [--radio spi|uart] [--radio-gpio N]
"""
import itertools
import re
import sys
import xml.etree.ElementTree as ET

NS = {"m": "http://dummy.com"}
AVOID = {"PA13", "PA14"}                  # SWD
AVOID_FAMILY = {"STM32G4": {"PB8"}}        # G4: PB8 = BOOT0
INPUT_ONLY = {"PC14", "PC15"}             # backup-domain pins: 3 mA/2 MHz outputs, fine as inputs
INPUT_SLOTS = {"IMU_INT", "MOT_nFAULT", "RF_GPIO1", "RF_GPIO2"}  # RF_GPIO1/2 = BUSY, DIO1


def load(path):
    root = ET.parse(path).getroot()
    pins = {}
    for p in root.findall("m:Pin", NS):
        if p.get("Type") != "I/O":
            continue
        name = re.sub(r"[-_ (].*$", "", p.get("Name"))
        sigs = [s.get("Name") for s in p.findall("m:Signal", NS)]
        pins[name] = dict(pos=p.get("Position"), sigs=sigs)
    return root.get("RefName"), root.get("Package"), pins


def timer_channels(pins):
    """{timer: {ch: [pins]}} for CH1..CH4 outputs (no complementary)."""
    t = {}
    for pn, d in pins.items():
        for s in d["sigs"]:
            m = re.fullmatch(r"(TIM\d+)_CH([1-4])", s)
            if m:
                t.setdefault(m.group(1), {}).setdefault(int(m.group(2)), []).append(pn)
    return t


def bipartite(slots, avail):
    """slots: list of (label, candidate_pins). Returns {label: pin} or None (augmenting paths)."""
    match_pin = {}

    def try_slot(i, seen):
        for p in slots[i][1]:
            if p in seen or p not in avail:
                continue
            seen.add(p)
            if p not in match_pin or try_slot(match_pin[p], seen):
                match_pin[p] = i
                return True
        return False

    for i in range(len(slots)):
        if not try_slot(i, set()):
            return None
    return {slots[i][0]: p for p, i in match_pin.items()}


def solve(path, radio="spi", radio_gpio=4):
    ref, pkg, pins = load(path)
    fam_avoid = set().union(*[v for k, v in AVOID_FAMILY.items() if ref.startswith(k)] or [set()])
    avail = {p for p in pins if p not in AVOID and p not in fam_avoid}
    tims = timer_channels(pins)
    adc = sorted(p for p in avail - INPUT_ONLY if any(re.match(r"ADC\d*_IN", s) for s in pins[p]["sigs"]))
    spis = {}
    for pn in avail:
        for s in pins[pn]["sigs"]:
            m = re.fullmatch(r"(SPI\d)_(SCK|MISO|MOSI)", s)
            if m:
                spis.setdefault(m.group(1), {}).setdefault(m.group(2), []).append(pn)
    uarts = {}
    for pn in avail:
        for s in pins[pn]["sigs"]:
            m = re.fullmatch(r"((?:US|UA|LPUA)RT\d)_(TX|RX)", s)
            if m:
                uarts.setdefault(m.group(1), {}).setdefault(m.group(2), []).append(pn)
    full_spis = [k for k, v in spis.items() if len(v) == 3]
    full_uarts = [k for k, v in uarts.items() if len(v) == 2]
    usable_tims = [t for t, chs in tims.items() if len(chs) >= 3]
    print("%s (%s): %d usable I/O, ADC-capable %d, timers>=3ch %s, SPI %s, UART %s"
          % (ref, pkg, len(avail), len(adc), sorted(usable_tims), sorted(full_spis), sorted(full_uarts)))
    best = None
    pref = ["TIM1", "TIM8", "TIM20", "TIM2", "TIM3", "TIM4", "TIM5"]
    tim_combos = sorted(itertools.combinations(usable_tims, 4),
                        key=lambda c: sum(pref.index(t) if t in pref else 9 for t in c))
    radio_ifaces = full_spis if radio == "spi" else full_uarts
    for tc in tim_combos:
        for chs in itertools.product(*[list(itertools.combinations(sorted(tims[t]), 3)) for t in tc]):
            for imu_spi in full_spis:
                for rif in radio_ifaces:
                    if rif == imu_spi:
                        continue
                    slots = []
                    for mi, (t, cs) in enumerate(zip(tc, chs)):
                        for ph, ch in zip("ABC", cs):
                            slots.append(("M%d_INH%s(%s_CH%d)" % (mi + 1, ph, t, ch), tims[t][ch]))
                    for k in ("SCK", "MISO", "MOSI"):
                        slots.append(("IMU_%s(%s)" % (k, imu_spi), spis[imu_spi][k]))
                    if radio == "spi":
                        for k in ("SCK", "MISO", "MOSI"):
                            slots.append(("RF_%s(%s)" % (k, rif), spis[rif][k]))
                    else:
                        for k in ("TX", "RX"):
                            slots.append(("RF_%s(%s)" % (k, rif), uarts[rif][k]))
                    names = ["M%d_BEMF%s" % (m, ph) for m in range(1, 5) for ph in "ABC"] + ["VBAT_SENSE"]
                    slots += [(n, adc) for n in names]
                    gpio = sorted(avail - INPUT_ONLY)
                    gpio_in = sorted(avail)
                    slots += [("M%d_INL%s" % (m, ph), gpio) for m in range(1, 5) for ph in "ABC"]
                    for n in ["IMU_CS", "IMU_INT", "MOT_nSLEEP", "MOT_nFAULT", "LED"] + \
                             ["RF_GPIO%d" % i for i in range(radio_gpio)]:
                        slots.append((n, gpio_in if n in INPUT_SLOTS else gpio))
                    # most-constrained first
                    slots.sort(key=lambda s: len(s[1]))
                    res = bipartite(slots, avail)
                    if res:
                        spare = sorted(avail - set(res.values()))
                        best = (tc, imu_spi, rif, res, spare)
                        break
                if best:
                    break
            if best:
                break
        if best:
            break
    if not best:
        print("  NO FEASIBLE ASSIGNMENT")
        return None
    tc, imu_spi, rif, res, spare = best
    print("  timers %s, IMU on %s, radio on %s, spare I/O %d: %s" % (tc, imu_spi, rif, len(spare), spare))
    for k in sorted(res, key=lambda s: (s[:2], s)):
        print("   %-22s %-5s %s" % (k, res[k], pins[res[k]]["pos"]))
    return best


if __name__ == "__main__":
    a = sys.argv[1:]
    radio = "uart" if "--radio" in a and a[a.index("--radio") + 1] == "uart" else "spi"
    rg = int(a[a.index("--radio-gpio") + 1]) if "--radio-gpio" in a else 4
    solve(a[0], radio, rg)
