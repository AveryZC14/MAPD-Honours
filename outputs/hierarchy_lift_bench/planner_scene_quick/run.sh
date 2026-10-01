#!/bin/bash
# Quick planner check, 2026-10-01: corridor vs lift guide paths on
# scene_mp_4p_03 10k, solver 6 level 4, 300 steps, one at a time.
cd /home/ubuntu/MAPD-Honours
O=outputs/hierarchy_lift_bench/planner_scene_quick
for src in corridor lift; do
  /usr/bin/time -v ./build/lifelong --inputFile instances/thesis_benchmarks/scene_mp_4p_03/scene_mp_4p_03_10000.json \
    -o $O/scene_${src}_L4.json --scheduleModel 6 -s 300 --preprocessTimeLimit 1800000 --logDetailLevel 3 \
    --flowSolveLevel 4 --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy \
    --guidePathSource $src --guidePathLevel 4 > $O/scene_${src}_L4.log 2> $O/scene_${src}_L4.time
  echo "$src exit $?"
done
