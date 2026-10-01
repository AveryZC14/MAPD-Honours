
#ifndef search_hpp
#define search_hpp

#include "Types.h"
#include "utils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.h"
#include "const.h"

namespace DefaultPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow

// deadline: if given, the search is abandoned once it passes (checked every
// 1024 expansions); it then returns an empty traj and a node with id -1.
s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline = nullptr);

    s_node multi_goal_astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, unordered_set<int> goals, Neighbors* ns);
}

#endif