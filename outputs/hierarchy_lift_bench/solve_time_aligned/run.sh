#!/bin/bash
cd /home/ubuntu/MAPD-Honours
O=/tmp/claude-1000/-home-ubuntu-MAPD-Honours/24b7984d-574e-48a9-8278-c3c815d4aba7/scratchpad/align
I=instances/thesis_benchmarks
for sv in 6 7; do
  ./build/lifelong --inputFile $I/orz900d/orz900d_10000.json -o $O/orz_s$sv.json --scheduleModel $sv -s 50 --preprocessTimeLimit 600000 --logDetailLevel 3 --flowSolveLevel 4 --hierarchyCache hierarchy_cache/orz900d_full.hierarchy > $O/orz_s$sv.log 2>&1; echo "orz s$sv $?"
done
./build/lifelong --inputFile $I/IH_mp_2p_01/IH_mp_2p_01_10000.json -o $O/ih_s6.json --scheduleModel 6 -s 50 --preprocessTimeLimit 600000 --logDetailLevel 3 --flowSolveLevel 4 --hierarchyCache hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy > $O/ih_s6.log 2>&1; echo "ih s6 $?"
