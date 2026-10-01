#!/bin/bash
# Lift vs corridor A* (margin 0 and 1) vs full-map A*, with the planner's
# length cap (no cap in practice). See ai/hierarchical_guide_paths_plan.md,
# "Implementation plan". Run one at a time on an otherwise idle machine.
cd /home/ubuntu/MAPD-Honours
O=outputs/hierarchy_lift_bench/corridor
B=./build/bench_hierarchy_lift
I=instances/thesis_benchmarks
C=100000000
$B $I/orz900d/orz900d_10000.json --hierarchyCache hierarchy_cache/orz900d_full.hierarchy --levels 1,2,3,4,6 --pairs 300 --maxPathCells $C --csv $O/orz900d.csv > $O/orz900d.txt 2>&1
$B $I/IH_mp_2p_01/IH_mp_2p_01_10000.json --hierarchyCache hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy --levels 1,2,3,4,6 --pairs 300 --maxPathCells $C --csv $O/ih.csv > $O/ih.txt 2>&1
$B $I/scene_mp_4p_03/scene_mp_4p_03_10000.json --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy --levels 2,3,4,5,6 --pairs 200 --maxPathCells $C --csv $O/scene.csv > $O/scene.txt 2>&1
echo done > $O/DONE
