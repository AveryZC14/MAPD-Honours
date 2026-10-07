#!/bin/bash
# Quick planner check, 2026-10-07: refine (levels 4 and 6) vs corridor
# (level 4) on scene_mp_4p_03 10k, solver 6 level 4, 300 steps, one at a
# time. Same setup as ../coarse_astar (corridor/lift there). B= overrides
# the binary (these runs used a separate build of this branch).
cd /home/ubuntu/MAPD-Honours
O=outputs/hierarchy_lift_bench/planner_scene_refine
B=${B:-./build/lifelong}
I=instances/thesis_benchmarks
for run in refine:4 refine:6 corridor:4; do
  src=${run%:*}; lvl=${run#*:}
  /usr/bin/time -v $B --inputFile $I/scene_mp_4p_03/scene_mp_4p_03_10000.json \
    -o $O/scene_${src}_L${lvl}.json --scheduleModel 6 -s 300 --preprocessTimeLimit 1800000 --logDetailLevel 3 \
    --flowSolveLevel 4 --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy \
    --guidePathSource $src --guidePathLevel $lvl > $O/scene_${src}_L${lvl}.log 2> $O/scene_${src}_L${lvl}.time
  echo "$src L$lvl exit $?"
done
echo done > $O/DONE
