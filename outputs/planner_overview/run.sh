#!/bin/bash
# Overview of the hierarchy guide-path planner (2026-10-01), about 7 h.
# One run at a time on an otherwise idle machine. Plan and reasoning:
# ai/hierarchical_guide_paths_plan.md, "Overview runs".
#   A: does it help?  3 maps x astar/corridor/lift, 10k, 1,500 steps
#   B: scaling        lift, solver 6 level 8, 10k-80k, IH then scene, 500 steps
#   C: level          IH 10k, corridor/lift at guide-path levels 3 and 6, 500 steps
# All runs: margin 0, congestion off, --computeGuidePaths false (the planner
# doesn't use solver 6's lifted paths; it would only take planner time).
# A memory watchdog kills a run if available memory drops under 1.5 GB.
# Runs that would start after 8 h are skipped.
cd /home/ubuntu/MAPD-Honours
O=outputs/planner_overview
I=instances/thesis_benchmarks
mkdir -p $O
START=$(date +%s)
DEADLINE=$((START + 8 * 3600))

declare -A CACHE=(
  [orz900d]=hierarchy_cache/orz900d_full.hierarchy
  [IH_mp_2p_01]=hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy
  [scene_mp_4p_03]=hierarchy_cache/scene_mp_4p_03_full.hierarchy
)

# run NAME MAP AGENTS STEPS SOLVER_LEVEL SOURCE GUIDE_LEVEL
run() {
  local name=$1 map=$2 agents=$3 steps=$4 slevel=$5 src=$6 glevel=$7
  if [ -e $O/$name.json.gz ]; then echo "$name already done"; return; fi
  if [ $(date +%s) -gt $DEADLINE ]; then echo "$name skipped (past 8 h)" | tee -a $O/progress.txt; return; fi
  echo "$(date +%T) start $name" | tee -a $O/progress.txt
  /usr/bin/time -v ./build/lifelong --inputFile $I/$map/${map}_${agents}.json -o $O/$name.json \
    --scheduleModel 6 -s $steps --preprocessTimeLimit 1800000 --logDetailLevel 3 \
    --flowSolveLevel $slevel --hierarchyCache ${CACHE[$map]} --computeGuidePaths false \
    --guidePathSource $src --guidePathLevel $glevel --guidePathTrace $O/$name.trace.csv \
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

# A: does it help? (about 4.2 h)
for map in orz900d IH_mp_2p_01 scene_mp_4p_03; do
  for src in astar corridor lift; do
    run A_${map}_10000_${src}_L4 $map 10000 1500 4 $src 4
  done
done

# B: scaling, lift, solver 6 level 8 (about 1.7 h). Scene last, smallest first.
for map in IH_mp_2p_01 scene_mp_4p_03; do
  for n in 10000 20000 40000 80000; do
    run B_${map}_${n}_lift_L4 $map $n 500 8 lift 4
  done
done

# C: guide-path level (about 0.7 h); level 4 = first 500 steps of the A runs
for src in corridor lift; do
  for gl in 3 6; do
    run C_IH_mp_2p_01_10000_${src}_L${gl} IH_mp_2p_01 10000 500 4 $src $gl
  done
done

echo "$(date +%T) all done" | tee -a $O/progress.txt
echo done > $O/DONE
