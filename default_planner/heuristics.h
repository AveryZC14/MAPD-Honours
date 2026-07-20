
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace DefaultPlanner{

void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

// Marks `goal_location`'s heuristic table as most-recently-used and evicts the
// least-recently-used tables once the cache exceeds its memory budget. Every
// call site that ensures a heuristic table exists for a goal (whether newly
// built or already cached) should call this right after, so the cache stays
// bounded regardless of how many distinct goal locations get requested over
// a run -- on large maps with many scattered task locations, caching one
// full-map distance table per distinct goal forever caused unbounded memory
// growth and OOM.
void touch_heuristic_lru(int goal_location, SharedEnvironment* env);

int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

int get_h(SharedEnvironment* env, int source, int target);


void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

}
#endif