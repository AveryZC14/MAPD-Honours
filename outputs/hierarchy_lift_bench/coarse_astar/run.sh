#!/bin/bash
# Coarse search as A* with landmark (ALT) + grid heuristic (2026-10-01),
# compared with outputs/hierarchy_lift_bench/corridor/ (Dijkstra) and
# planner_scene_quick/. One at a time on an otherwise idle machine.
cd /home/ubuntu/MAPD-Honours
O=outputs/hierarchy_lift_bench/coarse_astar
B=./build/bench_hierarchy_lift
I=instances/thesis_benchmarks
C=100000000
$B $I/orz900d/orz900d_10000.json --hierarchyCache hierarchy_cache/orz900d_full.hierarchy --levels 1,2,3,4,6 --pairs 300 --maxPathCells $C --csv $O/orz900d.csv > $O/orz900d.txt 2>&1
$B $I/IH_mp_2p_01/IH_mp_2p_01_10000.json --hierarchyCache hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy --levels 1,2,3,4,6 --pairs 300 --maxPathCells $C --csv $O/ih.csv > $O/ih.txt 2>&1
$B $I/scene_mp_4p_03/scene_mp_4p_03_10000.json --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy --levels 2,3,4,5,6 --pairs 200 --maxPathCells $C --csv $O/scene.csv > $O/scene.txt 2>&1
for src in corridor lift; do
  /usr/bin/time -v ./build/lifelong --inputFile $I/scene_mp_4p_03/scene_mp_4p_03_10000.json \
    -o $O/scene_${src}_L4.json --scheduleModel 6 -s 300 --preprocessTimeLimit 1800000 --logDetailLevel 3 \
    --flowSolveLevel 4 --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy \
    --guidePathSource $src --guidePathLevel 4 > $O/scene_${src}_L4.log 2> $O/scene_${src}_L4.time
  echo "$src exit $?"
done
echo done > $O/DONE
