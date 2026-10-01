
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

// Limits astar() to cells whose level-L hierarchy node is stamped for this
// search (corridor A*, ai/hierarchical_guide_paths_plan.md). cell_node maps
// each fine cell to its level-L node (-1 for obstacles); a cell is allowed
// if node_stamp[cell_node[cell]] == stamp.
struct SearchCorridor {
    const std::vector<int>* cell_node = nullptr;
    const std::vector<uint32_t>* node_stamp = nullptr;
    uint32_t stamp = 0;
    bool allows(int cell) const {
        const int n = (*cell_node)[cell];
        return n >= 0 && (*node_stamp)[n] == stamp;
    }
};

// deadline: if given, the search is abandoned once it passes (checked every
// 1024 expansions); it then returns an empty traj and a node with id -1.
// corridor: if given, only cells it allows are searched; if the goal can't
// be reached inside it, returns an empty traj and a node with id -1 (instead
// of exiting, as a search without a corridor does).
// expanded_out: if given, receives the number of cells expanded.
s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline = nullptr,
    const SearchCorridor* corridor = nullptr, int* expanded_out = nullptr);

    s_node multi_goal_astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, unordered_set<int> goals, Neighbors* ns);
}

#endif