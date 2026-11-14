"""render.py <board.kicad_pcb> <outdir> [dpi] - plot top/bottom assembly views to PNG via kicad-cli + PyMuPDF."""
import os
import subprocess
import sys

KICAD_CLI = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"
PYLIBS = r"C:\Users\angel\AppData\Local\Temp\claude\e9d9e4576166a4b9\8404409f-3ecc-4ad5-ba45-8b90dd39ffba\scratchpad\pylibs"
sys.path.insert(0, PYLIBS)
import pymupdf  # noqa: E402

board, out = sys.argv[1], sys.argv[2]
dpi = int(sys.argv[3]) if len(sys.argv) > 3 else 600
os.makedirs(out, exist_ok=True)
views = {
    "top": "F.Cu,F.Fab,F.Courtyard,Edge.Cuts",
    "bottom": "B.Cu,B.Fab,B.Courtyard,Edge.Cuts",
    "inner": "In1.Cu,In2.Cu,In3.Cu,In4.Cu,Edge.Cuts",
}
if len(sys.argv) > 4:
    views = {k: v for k, v in views.items() if k in sys.argv[4].split(",")}
for name, layers in views.items():
    pdf = os.path.join(out, name + ".pdf")
    args = [KICAD_CLI, "pcb", "export", "pdf", "--layers", layers, "--mode-single", "--include-border-title",
            "-o", pdf, board] if False else \
        [KICAD_CLI, "pcb", "export", "pdf", "--layers", layers, "--mode-single", "-o", pdf, board]
    if name == "bottom":
        args.insert(4, "--mirror")
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        continue
    files = [os.path.join(out, f) for f in os.listdir(out) if f.startswith(name) and f.endswith(".pdf")]
    for f in files:
        d = pymupdf.open(f)
        page = d[0]
        # crop to drawing bbox
        rects = [x["rect"] for x in page.get_drawings()]
        if rects:
            bb = rects[0]
            for rr in rects[1:]:
                bb |= rr
            bb = pymupdf.Rect(bb.x0 - 10, bb.y0 - 10, bb.x1 + 10, bb.y1 + 10)
        else:
            bb = page.rect
        png = f[:-4] + ".png"
        page.get_pixmap(dpi=dpi, clip=bb).save(png)
        print("wrote", png)
