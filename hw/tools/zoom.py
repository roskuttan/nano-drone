"""zoom.py <board> <layers> <x0> <x1> <y0> <y1> <out.png> [dpi] - plot a design-coordinate window (mm, board centre
at 0,0 as in the layout scripts) of the given layers to PNG (kicad-cli PDF -> PyMuPDF clip)."""
import os
import subprocess
import sys

KICAD_CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
PYLIBS = r"C:\Users\angel\AppData\Local\Temp\claude\e9d9e4576166a4b9\8404409f-3ecc-4ad5-ba45-8b90dd39ffba\scratchpad\pylibs"
sys.path.insert(0, PYLIBS)
import pymupdf  # noqa: E402

board, layers = sys.argv[1], sys.argv[2]
x0, x1, y0, y1 = (float(v) for v in sys.argv[3:7])
out = sys.argv[7]
dpi = int(sys.argv[8]) if len(sys.argv) > 8 else 1200
pdf = out[:-4] + ".pdf"
r = subprocess.run([KICAD_CLI, "pcb", "export", "pdf", "--layers", layers, "--mode-single", "-o", pdf, board],
                   capture_output=True, text=True)
if r.returncode:
    print(r.stdout, r.stderr)
    sys.exit(1)
d = pymupdf.open(pdf)
page = d[0]
k = 72 / 25.4
clip = pymupdf.Rect((100 + x0) * k, (100 + y0) * k, (100 + x1) * k, (100 + y1) * k)
page.get_pixmap(dpi=dpi, clip=clip).save(out)
print("wrote", out)
