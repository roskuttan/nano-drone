"""Run every post-layout study on the routed board, in order (each script also runs on its own).
    python run_post.py            (board: hw/kicad/nanodrone.kicad_pcb, or $NANODRONE_PCB)
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIES = ["ps1_thermal.py", "ps2_irdrop.py", "ps3_xtalk.py", "ps4_rf.py", "ps5_cavity.py", "ps6_nets.py"]

if __name__ == "__main__":
    ok = True
    for s in STUDIES:
        t0 = time.time()
        r = subprocess.run([sys.executable, os.path.join(HERE, s)], cwd=HERE)
        print("== %s: exit %d, %.0f s" % (s, r.returncode, time.time() - t0), flush=True)
        ok &= r.returncode == 0
    sys.exit(0 if ok else 1)
