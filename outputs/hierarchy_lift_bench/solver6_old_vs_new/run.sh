#!/bin/bash
# Old binary: a copy of the working tree with the lift reverted to anchoring only level 1 (see ai/hierarchical_guide_paths_plan.md).
cd /home/ubuntu/MAPD-Honours
O=/tmp/claude-1000/-home-ubuntu-MAPD-Honours/24b7984d-574e-48a9-8278-c3c815d4aba7/scratchpad/ab
OLD=/tmp/claude-1000/-home-ubuntu-MAPD-Honours/24b7984d-574e-48a9-8278-c3c815d4aba7/scratchpad/oldlift/src/build/lifelong
NEW=./build/lifelong
I=instances/thesis_benchmarks
for m in IH_mp_2p_01:IH_mp_2p_01_fixpoint orz900d:orz900d_full; do
  map=${m%%:*}; cache=${m##*:}
  for v in old new; do
    B=$NEW; [ $v = old ] && B=$OLD
    /usr/bin/time -f "wall %e s peak %M KB" $B --inputFile $I/$map/${map}_10000.json -o $O/${map}_$v.json --scheduleModel 6 -s 300 --preprocessTimeLimit 600000 --logDetailLevel 3 --flowSolveLevel 4 --hierarchyCache hierarchy_cache/$cache.hierarchy > $O/${map}_$v.log 2>&1
    echo "$map $v exit $?"
  done
done
