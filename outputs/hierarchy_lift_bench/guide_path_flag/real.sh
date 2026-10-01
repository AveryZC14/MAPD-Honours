#!/bin/bash
cd /home/ubuntu/MAPD-Honours
O=/tmp/claude-1000/-home-ubuntu-MAPD-Honours/24b7984d-574e-48a9-8278-c3c815d4aba7/scratchpad/flag
I=instances/thesis_benchmarks
for g in true false; do
  ./build/lifelong --inputFile $I/IH_mp_2p_01/IH_mp_2p_01_10000.json -o $O/ih_s6_$g.json --scheduleModel 6 -s 100 --preprocessTimeLimit 600000 --logDetailLevel 3 --flowSolveLevel 4 --hierarchyCache hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy --computeGuidePaths $g > $O/ih_s6_$g.log 2>&1; echo "ih s6 $g $?"
done
for g in true false; do
  ./build/lifelong --inputFile $I/orz900d/orz900d_10000.json -o $O/orz_s1_$g.json --scheduleModel 1 -s 20 --preprocessTimeLimit 600000 --logDetailLevel 3 --computeGuidePaths $g > $O/orz_s1_$g.log 2>&1; echo "orz s1 $g $?"
done
