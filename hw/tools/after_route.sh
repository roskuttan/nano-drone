#!/usr/bin/env bash
# after_route.sh <routed board> - the post-routing chain, run unattended once a board is fully routed:
#   1. copy the board to the canonical kicad/nanodrone.kicad_pcb (the project's .kicad_pro / .kicad_dru apply)
#   2. finish stage: widen power/phase, planes, stitching, power islands, DRC
#   3. DRC summary + phase power-path check (motor current on outer copper)
#   4. fab package (Gerbers, drill, BOM/CPL, order notes, renders, STEP)
#   5. post-layout simulations P1-P6 + report
# Each step is logged to output/after_route.log; the chain stops at the first failing step.
set -euo pipefail
HW="$(cd "$(dirname "$0")/.." && pwd)"
PY="/c/Users/angel/AppData/Local/Programs/KiCad/10.0/bin/python.exe"
SRC="$1"
LOG="$HW/output/after_route.log"
mkdir -p "$HW/output"
: > "$LOG"
step() { echo "=== $(date +%H:%M:%S) $*" | tee -a "$LOG"; }
cd "$HW"
step "copy $SRC -> kicad/nanodrone.kicad_pcb"
cp "$SRC" kicad/nanodrone.kicad_pcb
unset NANODRONE_PCB
step "finish stage"
"$PY" -u design/nanodrone_route.py finish 2>&1 | grep -v "image handler\|memory leak" | tee -a "$LOG"
step "DRC summary"
"$PY" tools/drc_summary.py kicad/drc.json 4 2>&1 | tee -a "$LOG"
step "phase power paths"
"$PY" tools/phase_check.py kicad/nanodrone.kicad_pcb 2>&1 | grep -v "image handler\|memory leak" | tee -a "$LOG"
step "fab package"
"$PY" design/nanodrone_fab.py 2>&1 | grep -v "image handler\|memory leak" | tee -a "$LOG"
step "post-layout simulations"
(cd sim_post && "$PY" run_post.py) 2>&1 | grep -v "image handler\|memory leak" | tail -40 | tee -a "$LOG"
(cd sim_post && "$PY" report.py) 2>&1 | grep -v "image handler\|memory leak" | tail -10 | tee -a "$LOG"
step "done"
