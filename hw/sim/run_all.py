"""Run every study in order (each script is also runnable on its own).  python run_all.py"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIES = ["sim_01_hotplug.py", "sim_02_motor_esc.py", "sim_03_thermal.py", "sim_04_flight.py",
           "sim_05_pdn.py", "sim_06_adc.py", "sim_07_rf.py", "sim_08_ldo_brownout.py",
           "sim_09_pwm_slew.py"]

if __name__ == "__main__":
    ok = True
    for s in STUDIES:
        t0 = time.time()
        r = subprocess.run([sys.executable, os.path.join(HERE, s)], cwd=HERE)
        print(f"== {s}: exit {r.returncode}, {time.time() - t0:.0f} s", flush=True)
        ok &= r.returncode == 0
    sys.exit(0 if ok else 1)
