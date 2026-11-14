"""Nano drone - manufacturing outputs for JLCPCB (Gerbers, drill, BOM, CPL, renders, PDFs).

Writes hw/output/fab/: gerbers/ (+ zip), nanodrone_BOM_JLC.csv, nanodrone_CPL_JLC.csv,
ORDER_NOTES.md, renders, schematic PDF, DRC/ERC reports.
"""
import csv
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
PRJ = os.path.join(HW, "kicad")
OUT = os.path.join(HW, "output", "fab")
CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
PCB = os.path.join(PRJ, "nanodrone.kicad_pcb")
SCH = os.path.join(PRJ, "nanodrone.kicad_sch")


def run(args):
    r = subprocess.run([CLI] + args, capture_output=True, text=True)
    if r.returncode:
        print("FAILED:", " ".join(args), r.stdout[-500:], r.stderr[-500:])
    return r


def main():
    os.makedirs(OUT, exist_ok=True)
    gdir = os.path.join(OUT, "gerbers")
    if os.path.exists(gdir):
        shutil.rmtree(gdir)
    os.makedirs(gdir)
    layers = "F.Cu,In1.Cu,In2.Cu,In3.Cu,In4.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
    run(["pcb", "export", "gerbers", "--layers", layers, "--subtract-soldermask", "--use-drill-file-origin",
         "-o", gdir + os.sep, PCB])
    run(["pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "--generate-map",
         "--map-format", "pdf", "-o", gdir + os.sep, PCB])
    zpath = os.path.join(OUT, "nanodrone_gerbers_JLC.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(gdir)):
            z.write(os.path.join(gdir, f), f)
    print("gerber zip:", zpath, len(os.listdir(gdir)), "files")

    # ---- positions (CPL) ----
    pos_csv = os.path.join(OUT, "pos_raw.csv")
    run(["pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--use-drill-file-origin",
         "--exclude-dnp", "-o", pos_csv, PCB])
    lmap = json.load(open(os.path.join(HERE, "lcsc_map.json")))
    skip_vals = set(lmap["not_assembled"])

    def lookup(value, fp):
        fpname = fp.split(":")[-1]
        for p in lmap["parts"]:
            if p["value"] == value and ("fp" not in p or p["fp"] == fpname):
                return p
        return None

    # ---- BOM from the schematic netlist fields (value + footprint) ----
    net_xml = os.path.join(OUT, "bom_raw.csv")
    run(["sch", "export", "bom", "--fields", "Reference,Value,Footprint,${QUANTITY},LCSC,MPN",
         "--labels", "Ref,Value,Footprint,Qty,LCSC,MPN", "--group-by", "Value,Footprint",
         "--ref-range-delimiter", "", "-o", net_xml, SCH])
    rows = list(csv.DictReader(open(net_xml, encoding="utf-8")))
    bom, missing = [], []
    assembled_refs = set()
    for r in rows:
        val, fp = r["Value"], r["Footprint"]
        if val in skip_vals or not fp:
            continue
        refs = [x.strip() for x in re.split(r"[,\s]+", r["Ref"]) if x.strip()]
        p = lookup(val, fp)
        lcsc = (p or {}).get("lcsc") or r.get("LCSC", "")
        mpn = (p or {}).get("mpn") or r.get("MPN", "")
        if not lcsc:
            missing.append((val, fp, refs))
        bom.append({"Comment": val + (" | " + mpn if mpn else ""), "Designator": ",".join(refs),
                    "Footprint": fp.split(":")[-1], "LCSC Part #": lcsc, "Qty": len(refs)})
        assembled_refs.update(refs)
    with open(os.path.join(OUT, "nanodrone_BOM_JLC.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["Comment", "Designator", "Footprint", "LCSC Part #", "Qty"])
        w.writeheader()
        w.writerows(bom)
    print("BOM lines:", len(bom), "parts:", sum(b["Qty"] for b in bom), "missing LCSC:", missing)

    cpl = []
    for r in csv.DictReader(open(pos_csv, encoding="utf-8")):
        ref = r["Ref"]
        if ref not in assembled_refs:
            continue
        cpl.append({"Designator": ref, "Mid X": r["PosX"], "Mid Y": r["PosY"],
                    "Layer": "Top" if r["Side"].lower().startswith("top") else "Bottom",
                    "Rotation": r["Rot"]})
    with open(os.path.join(OUT, "nanodrone_CPL_JLC.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writeheader()
        w.writerows(cpl)
    print("CPL rows:", len(cpl), "(top %d / bottom %d)" % (sum(c["Layer"] == "Top" for c in cpl),
                                                          sum(c["Layer"] == "Bottom" for c in cpl)))

    # ---- renders + documentation ----
    # 3D outputs come from a copy whose three missing library models are swapped for the closest bodies
    # (tools/render3d.py). The renders use the PCB-editor preset, so no F/B.Fab text is drawn.
    sys.path.insert(0, os.path.join(HW, "tools"))
    import pcbnew
    import render3d
    b3 = pcbnew.LoadBoard(PCB)
    render3d.fix_models(b3)
    pcb3 = os.path.join(OUT, "_3d_copy.kicad_pcb")
    pcbnew.SaveBoard(pcb3, b3)
    views = [("top", ["--side", "top", "-w", "1800", "-h", "1800"]),
             ("bottom", ["--side", "bottom", "-w", "1800", "-h", "1800"]),
             ("iso", ["--perspective", "--floor", "--rotate", "-50,0,35", "--zoom", "0.9", "-w", "1800", "-h", "1200"])]
    for name, args in views:
        run(["pcb", "render", "--quality", "high", "--background", "opaque", "--preset", "follow_pcb_editor",
             *args, "-o", os.path.join(OUT, "render_3d_%s.png" % name), pcb3])
    run(["sch", "export", "pdf", "-o", os.path.join(OUT, "nanodrone_schematic.pdf"), SCH])
    run(["pcb", "export", "pdf", "--layers", "F.Cu,F.Fab,Edge.Cuts", "--mode-single",
         "-o", os.path.join(OUT, "assembly_top.pdf"), PCB])
    run(["pcb", "export", "pdf", "--layers", "B.Cu,B.Fab,Edge.Cuts", "--mode-single", "--mirror",
         "-o", os.path.join(OUT, "assembly_bottom.pdf"), PCB])
    run(["pcb", "export", "step", "--subst-models", "-o", os.path.join(OUT, "nanodrone.step"), pcb3])
    for ext in (".kicad_pcb", ".kicad_pro", ".kicad_prl"):
        p_ = os.path.splitext(pcb3)[0] + ext
        if os.path.exists(p_):
            os.remove(p_)
    for f in ("drc.json", "erc.json"):
        if os.path.exists(os.path.join(PRJ, f)):
            shutil.copy(os.path.join(PRJ, f), os.path.join(OUT, f))
    write_order_notes(bom, cpl)


ORDER_NOTES = """# JLCPCB order notes: nano drone flight board (rev A)

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
* BOM: nanodrone_BOM_JLC.csv (%(lines)d lines, %(parts)d parts). CPL: nanodrone_CPL_JLC.csv (%(cpl)d placements).
* Check every rotation in JLC's placement preview, especially the QFN/WQFN/LGA parts (U2, U4, U11-U14, U21-U24, U3)
  and the crystal Y1. JLC's library zero orientation sometimes differs from KiCad's.
* Not assembled (hand-soldered by the user): battery pads J1, antenna pads J2, motor wire pads J11-J14, SWD pads J3.
* Fiducials: FID1-FID4 (1 mm mask opening ring), two per side.

## After assembly
* Motor wires solder to J11-J14 (PTH 0.5 mm). Phase order follows the silk A/B/C. Swap two wires or reverse in
  firmware to change direction.
* Antenna: a 2.4 GHz dipole or whip on J2 (pad 1 = feed, pad 2 = GND). Tune C22/L1 on a VNA if the antenna is not 50 ohm.
* First power-up: current-limited bench supply at 3.7 V, 0.3 A limit. Expected idle current is about 0.1 A.
"""


def write_order_notes(bom, cpl):
    txt = ORDER_NOTES % dict(lines=len(bom), parts=sum(b["Qty"] for b in bom), cpl=len(cpl))
    with open(os.path.join(OUT, "ORDER_NOTES.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    print("wrote ORDER_NOTES.md")


if __name__ == "__main__":
    main()
