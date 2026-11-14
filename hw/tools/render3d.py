"""render3d.py <board.kicad_pcb> <out_dir> [tag] - raytraced 3D renders (top, bottom, 3/4 view) of a board.

Three footprints reference 3D models that KiCad 10's library does not ship. The board is copied, and each of those
models is swapped for the closest available body (same outline and height, minor lead-pitch difference):
  QFN-48 7x7 EP5.6 (STM32G431)  -> QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm
  QFN-28 4x4 P0.5 (STM32G071)   -> QFN-28-1EP_4x4mm_P0.4mm_EP2.4x2.4mm
  X2SON-4 1x1 (TPS7A2030)       -> UDFN-4-1EP_1x1mm_P0.65mm_EP0.48x0.48mm
The copy renders with the PCB-editor appearance preset. The plot-settings preset would also draw the F/B.Fab text.
"""
import os
import subprocess
import sys

import pcbnew

KICAD_CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
D = "${KICAD10_3DMODEL_DIR}/"
SUBST = {
    "QFN-48-1EP_7x7mm_P0.5mm_EP5.6x5.6mm.step": D + "Package_DFN_QFN.3dshapes/QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm.step",
    "QFN-28_4x4mm_P0.5mm.step": D + "Package_DFN_QFN.3dshapes/QFN-28-1EP_4x4mm_P0.4mm_EP2.4x2.4mm.step",
    "Texas_X2SON-4_1x1mm_P0.65mm.step": D + "Package_DFN_QFN.3dshapes/UDFN-4-1EP_1x1mm_P0.65mm_EP0.48x0.48mm.step",
}


def fix_models(board):
    """Swap missing models (index access: iterating the SWIG vector yields copies) and hide the part-number fields
    (MPN/LCSC are BOM data, not silkscreen)."""
    n = 0
    for f in board.GetFootprints():
        ms = f.Models()
        for i in range(len(ms)):
            name = ms[i].m_Filename.split("/")[-1]
            if name in SUBST:
                ms[i].m_Filename = SUBST[name]
                n += 1
        for fld in f.GetFields():
            if fld.GetName() in ("MPN", "LCSC"):
                fld.SetVisible(False)
    return n


def main():
    src, out = sys.argv[1], sys.argv[2]
    tag = sys.argv[3] if len(sys.argv) > 3 else "board"
    os.makedirs(out, exist_ok=True)
    b = pcbnew.LoadBoard(src)
    n = fix_models(b)
    tmp = os.path.join(out, "_render_%s.kicad_pcb" % tag)
    pcbnew.SaveBoard(tmp, b)
    print("render copy: %d models substituted" % n)
    views = [("top", ["--side", "top", "-w", "1600", "-h", "1600"]),
             ("bottom", ["--side", "bottom", "-w", "1600", "-h", "1600"]),
             ("iso", ["--perspective", "--floor", "--rotate", "-50,0,35", "--zoom", "0.9", "-w", "1800", "-h", "1200"])]
    for name, args in views:
        png = os.path.join(out, "%s_%s.png" % (tag, name))
        r = subprocess.run([KICAD_CLI, "pcb", "render", "--quality", "high", "--background", "opaque",
                            "--preset", "follow_pcb_editor", *args, "-o", png, tmp], capture_output=True, text=True)
        print(name, "ok" if os.path.exists(png) and r.returncode == 0 else "FAILED " + r.stderr[-200:])
    os.remove(tmp)


if __name__ == "__main__":
    main()
