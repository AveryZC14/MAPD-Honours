#!/bin/bash
# Restart the thesis sweep (scripts/run_thesis_sweep.py) if it died before
# finishing. Meant to run from cron every 10 minutes; also used to start the
# sweep the first time. Safe to run at any time: the sweep resumes, skipping
# runs that already have an output JSON.
#
# It does nothing if:
#   - outputs/thesis_sweep/STOP exists (touch it before stopping the sweep
#     on purpose, or this will restart it),
#   - the sweep script is already running,
#   - a lifelong process is running (e.g. a run left behind by a killed
#     script; once it finishes and writes its JSON, the next check restarts
#     the sweep, which then skips that run),
#   - the sweep already finished (runner.log ends with its summary line).

REPO=/home/ubuntu/MAPD-Honours
OUT=$REPO/outputs/thesis_sweep
LOG=$OUT/runner.log

mkdir -p "$OUT"
[ -e "$OUT/STOP" ] && exit 0
pgrep -f run_thesis_sweep.py > /dev/null && exit 0
pgrep -x lifelong > /dev/null && exit 0
[ -f "$LOG" ] && tail -n 1 "$LOG" | grep -q "skipped. Summary:" && exit 0

echo "$(date '+%Y-%m-%d %H:%M:%S') sweep not running, starting it" >> "$OUT/watchdog.log"
cd "$REPO" && setsid nohup /usr/bin/python3 -B scripts/run_thesis_sweep.py >> "$LOG" 2>&1 < /dev/null &
