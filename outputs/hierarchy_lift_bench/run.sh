#!/bin/bash
# Historical (2026-10-01): produced *_a0.* (lift as it was, only the last
# level anchored) and *_a1.* (every level anchored, then an option) with
# --anchorEveryLevel, which no longer exists: anchoring is now permanent, so
# this script won't run as-is. The *_permanent.txt files are the current
# code with default settings, from:
#   ./build/bench_hierarchy_lift <instance> --hierarchyCache <cache> --levels ... --pairs N
# See ai/hierarchical_guide_paths_plan.md.
cd /home/ubuntu/MAPD-Honours
O=$1
B=./build/bench_hierarchy_lift
I=instances/thesis_benchmarks
for a in 0 1; do
  if [ $a = 0 ]; then C=5000; else C=100000; fi
  $B $I/orz900d/orz900d_10000.json --hierarchyCache hierarchy_cache/orz900d_full.hierarchy --levels 1,2,3,4,6,8,10 --pairs 300 --anchorEveryLevel $a --maxPathCells $C --csv $O/orz900d_a$a.csv > $O/orz900d_a$a.txt 2>&1
  $B $I/IH_mp_2p_01/IH_mp_2p_01_10000.json --hierarchyCache hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy --levels 1,2,3,4,6,8 --pairs 300 --anchorEveryLevel $a --maxPathCells $C --csv $O/ih_a$a.csv > $O/ih_a$a.txt 2>&1
  $B $I/scene_mp_4p_03/scene_mp_4p_03_10000.json --hierarchyCache hierarchy_cache/scene_mp_4p_03_full.hierarchy --levels 2,3,4,5,6,8 --pairs 200 --anchorEveryLevel $a --maxPathCells $C --csv $O/scene_a$a.csv > $O/scene_a$a.txt 2>&1
done
echo done > $O/DONE
