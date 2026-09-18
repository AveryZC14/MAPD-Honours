#!/usr/bin/env bash
# One-off overnight runner for the hierarchical-cascade sweep on IH_mp_2p_01,
# 10000 agents, 7000 timesteps -- matches the rest of
# outputs/IH_mp_2p_01_10000_7000ts_sweep/ (same instance/cache/flags), adding
# --minCascadeLevel/--cascadeLevelStride on top. See ai/hierarchical_matching.md.
#
# Six runs, sequential, safer/cheaper first: level 4, level 6, then "pure
# cascade" (flowSolveLevel = the hierarchy's own top level, 9, so the coarse
# flow solve becomes a vacuous no-op -- see "entirety mode" in the doc) --
# each at cascade_level_stride=1 (matches every level, the "cascade1" runs),
# then the same three configs again at stride=2 ("every second level").
#
# Each run is wrapped in `timeout` so one hung/pathologically slow run (see
# the unexplained level6_cascade1 death from 2026-09-03) can't block the rest
# of the queue overnight -- on timeout we log it and move on. Progress is
# appended to cascade_sweep_summary.csv after EVERY run (not deferred to the
# end), so a hang on a later run never loses bookkeeping for earlier
# completed ones.
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BINARY="$REPO_ROOT/build/lifelong"
INSTANCE="$REPO_ROOT/instances/custom/IH_mp_2p_01/IH_mp_2p_01_10000.json"
CACHE="$REPO_ROOT/hierarchy_cache/IH_mp_2p_01_level9.hierarchy"
OUT_DIR="$REPO_ROOT/outputs/IH_mp_2p_01_10000_7000ts_sweep"
SIM_TIME=7000
PREPROCESS_LIMIT=600000
LOG_DETAIL=3
PER_RUN_TIMEOUT="5h"

RUNNER_LOG="$OUT_DIR/cascade_runner.log"
SUMMARY_CSV="$OUT_DIR/cascade_sweep_summary.csv"

echo "run,flowSolveLevel,minCascadeLevel,cascadeLevelStride,exit_status,wall_clock_s,makespan,tasksFinished,note" > "$SUMMARY_CSV"

log() { echo "[$(date -u '+%Y-%m-%d %H:%M:%S UTC')] $*" | tee -a "$RUNNER_LOG"; }

# args: <name> <flowSolveLevel> <cascadeLevelStride>
run_one() {
    local name="$1" level="$2" stride="$3"
    local out_json="$OUT_DIR/IH_mp_2p_01_10000_solver6_${name}.json"
    local out_log="$OUT_DIR/IH_mp_2p_01_10000_solver6_${name}.log"

    log "START $name (flowSolveLevel=$level minCascadeLevel=1 cascadeLevelStride=$stride)"
    local start_ts
    start_ts=$(date +%s)

    timeout "$PER_RUN_TIMEOUT" "$BINARY" \
        --inputFile "$INSTANCE" \
        -o "$out_json" \
        --scheduleModel 6 \
        --flowSolveLevel "$level" \
        --minCascadeLevel 1 \
        --cascadeLevelStride "$stride" \
        -s "$SIM_TIME" \
        --preprocessTimeLimit "$PREPROCESS_LIMIT" \
        --hierarchyCache "$CACHE" \
        --logDetailLevel "$LOG_DETAIL" \
        > "$out_log" 2>&1
    local exit_code=$?

    local end_ts elapsed
    end_ts=$(date +%s)
    elapsed=$((end_ts - start_ts))

    local status makespan="" tasks="" note=""
    if [ "$exit_code" -eq 124 ]; then
        status="TIMED_OUT"
        note="killed by timeout after ${PER_RUN_TIMEOUT}, see $out_log"
        log "TIMEOUT $name after ${elapsed}s -- $note"
    elif [ "$exit_code" -ne 0 ]; then
        status="FAILED"
        note="exit code $exit_code, see $out_log"
        log "FAILED $name after ${elapsed}s -- $note"
    else
        status="OK"
        if [ -f "$out_json" ]; then
            makespan=$(python3 -c "import json; print(json.load(open('$out_json')).get('makespan',''))" 2>/dev/null)
            tasks=$(python3 -c "import json; print(json.load(open('$out_json')).get('numTaskFinished',''))" 2>/dev/null)
            errs=$(python3 -c "
import json
d = json.load(open('$out_json'))
e = {k: d.get(k,0) for k in ('numPlannerErrors','numScheduleErrors','numEntryTimeouts')}
print(','.join(f'{k}={v}' for k,v in e.items() if v))
" 2>/dev/null)
            if [ -n "$errs" ]; then
                note="nonzero error counts: $errs"
            fi
        else
            status="FAILED"
            note="exit 0 but no output JSON found"
        fi
        log "DONE $name after ${elapsed}s -- makespan=$makespan tasksFinished=$tasks${note:+ ($note)}"
    fi

    echo "${name},${level},1,${stride},${status},${elapsed},${makespan},${tasks},\"${note}\"" >> "$SUMMARY_CSV"
}

log "=== cascade overnight sweep starting ==="
log "binary: $BINARY"
log "instance: $INSTANCE"
log "cache: $CACHE"
log "per-run timeout: $PER_RUN_TIMEOUT"

# stride=1 ("cascade1"): safer/cheaper first, riskiest (pure cascade, top
# level 9) last, so a hang there doesn't cost the cheaper configs anything.
run_one "level4_cascade1"          4 1
run_one "level6_cascade1"          6 1
run_one "level9_cascade1"          9 1

# stride=2 ("every second level"): same order/logic.
run_one "level4_cascade1_stride2"  4 2
run_one "level6_cascade1_stride2"  6 2
run_one "level9_cascade1_stride2"  9 2

log "=== cascade overnight sweep finished, regenerating combined metrics.csv over the whole directory ==="
python3 "$REPO_ROOT/visualisation/compute_throughput_metrics.py" "$OUT_DIR" -o "$OUT_DIR/metrics_all_2026-09-17.csv" >> "$RUNNER_LOG" 2>&1

log "=== all done ==="
