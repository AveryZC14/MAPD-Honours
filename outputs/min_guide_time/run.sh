#!/bin/bash
# --minGuidePathMs test (2026-10-07), about 35 min. One run at a time.
# Question: does guaranteeing guide paths 150 ms per decision beat
# starving them, when the scheduler is slow (scene level 4) or the PIBT
# reserve is large (IH 80k)? Baselines: overnight A_scene_mp_4p_03_10000_
# corridor_L4 and today's R800_IH_mp_2p_01_80000_lift_L4 (both g = 0).
# A memory watchdog kills a run if available memory drops under 1.5 GB.
cd /home/ubuntu/MAPD-Honours-minguide
O=outputs/min_guide_time
I=instances/thesis_benchmarks
mkdir -p $O
START=$(date +%s)
DEADLINE=$((START + 2 * 3600))

declare -A CACHE=(
  [orz900d]=/home/ubuntu/MAPD-Honours/hierarchy_cache/orz900d_full.hierarchy
  [IH_mp_2p_01]=/home/ubuntu/MAPD-Honours/hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy
  [scene_mp_4p_03]=/home/ubuntu/MAPD-Honours/hierarchy_cache/scene_mp_4p_03_full.hierarchy
)

# run NAME MAP AGENTS STEPS SOLVER_LEVEL SOURCE GUIDE_LEVEL
run() {
  local name=$1 map=$2 agents=$3 steps=$4 slevel=$5 src=$6 glevel=$7
  if [ -e $O/$name.json.gz ]; then echo "$name already done"; return; fi
  if [ $(date +%s) -gt $DEADLINE ]; then echo "$name skipped (past 2 h)" | tee -a $O/progress.txt; return; fi
  echo "$(date +%T) start $name" | tee -a $O/progress.txt
  /usr/bin/time -v ./build/lifelong --inputFile $I/$map/${map}_${agents}.json -o $O/$name.json \
    --scheduleModel 6 -s $steps --preprocessTimeLimit 1800000 --logDetailLevel 3 \
    --flowSolveLevel $slevel --hierarchyCache ${CACHE[$map]} --computeGuidePaths false \
    --guidePathSource $src --guidePathLevel $glevel --guidePathTrace $O/$name.trace.csv $EXTRA \
    > $O/$name.log 2> $O/$name.time &
  local pid=$!
  # memory samples (s since start, RSS GB of lifelong, available GB) and watchdog
  local t0=$(date +%s)
  while kill -0 $pid 2>/dev/null; do
    local lp=$(pgrep -P $pid lifelong | head -1)
    local rss=$( [ -n "$lp" ] && awk '/VmRSS/ {print $2}' /proc/$lp/status 2>/dev/null || echo 0)
    local avail=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
    echo "$(( $(date +%s) - t0 )) $(awk "BEGIN {print ${rss:-0}/1048576}") $(awk "BEGIN {print $avail/1048576}")" >> $O/$name.mem
    if [ "$avail" -lt 1572864 ] && [ -n "$lp" ]; then
      echo "$(date +%T) $name killed: available memory under 1.5 GB" | tee -a $O/progress.txt
      kill $lp
    fi
    sleep 10
  done
  wait $pid
  local rc=$?
  gzip -f $O/$name.log $O/$name.trace.csv
  [ -e $O/$name.json ] && gzip -f $O/$name.json
  echo "$(date +%T) end $name exit $rc ($(grep Elapsed $O/$name.time | awk '{print $NF}'))" | tee -a $O/progress.txt
}

EXTRA="--minGuidePathMs 150" run G150_scene_mp_4p_03_10000_corridor_L4 scene_mp_4p_03 10000 1500 4 corridor 4
EXTRA="--minGuidePathMs 150" run G150_IH_mp_2p_01_80000_lift_L4 IH_mp_2p_01 80000 500 8 lift 4

echo "$(date +%T) all done" | tee -a $O/progress.txt
echo done > $O/DONE
