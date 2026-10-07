#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"
#include <unordered_map>


typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;
typedef std::unordered_map<int, Task> TaskPool;

class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;

    // Path to a cached, previously-built solver-6 map-coarsening hierarchy
    // (see MapReductionTest::ReducedHierarchy::ensure()). Empty means
    // "don't cache" -- always rebuild in-process and never touch disk.
    std::string hierarchy_cache_path;

    // Hierarchy level (0 = fine map) that solver 6 solves the per-timestep
    // flow assignment on (see MapReductionTest::ReducedHierarchy::
    // compute_reduced_assignment()). Set from --flowSolveLevel; a value
    // out of range for the hierarchy actually built falls back to that
    // function's own default.
    int flow_solve_level = 2;

    // First hierarchy level solver 6's hierarchical/cascaded local matcher
    // (MapReductionTest::ReducedHierarchy::compute_hierarchical_assignment())
    // attempts before handing any leftover to flow_solve_level's usual
    // local-match-then-flow handling. Set from --minCascadeLevel. A value
    // >= flow_solve_level disables cascading entirely (byte-identical to
    // solver 6's pre-cascade behavior) -- this is the default, so cascading
    // is strictly opt-in. See ai/hierarchical_matching.md.
    int min_cascade_level = 999;

    // How many hierarchy levels the cascade climbs between local-match
    // attempts (MapReductionTest::ReducedHierarchy::compute_hierarchical_assignment()).
    // 1 (default) matches at every level from min_cascade_level up to
    // flow_solve_level, exactly as before this field existed. A value of N
    // still climbs one level at a time (to_coarser_node_id only maps one
    // hop), but only attempts a local match every Nth level, carrying
    // unmatched items past the skipped levels untouched -- fewer, larger
    // match_local_node_exact() calls instead of many small ones. Set from
    // --cascadeLevelStride; values < 1 are clamped up to 1. See
    // ai/hierarchical_matching.md.
    int cascade_level_stride = 1;

    // Whether schedulers build guide paths at all. Set from
    // --computeGuidePaths (default true). When false, solvers 6/7 skip the
    // coarse-to-fine lift entirely, solver 1 skips recording the path it
    // walks (the walk itself is still needed to recover the assignment), and
    // no scheduler path reaches the planner (agent_guide_path stays empty,
    // even with --useTraffic or PASS_SCHEDULER_PATHS_TO_PLANNER), so
    // GuidePathLengthSum/GuidePathCostSum/SchedulerGuidePathTime are 0. See
    // ai/hierarchical_guide_paths_plan.md.
    bool compute_guide_paths = true;

    // Where the planner's own guide paths come from (stage 2 of
    // DefaultPlanner::plan). Set from --guidePathSource / --guidePathLevel /
    // --guidePathCorridorMargin / --guidePathCorridorCongestion. See
    // ai/hierarchical_guide_paths_plan.md, "Implementation plan".
    //   GUIDE_SOURCE_ASTAR (default): full-map A*, the planner as before.
    //   GUIDE_SOURCE_LIFT: coarse path at guide_path_level, lifted to the
    //     fine map with solver 6's lift.
    //   GUIDE_SOURCE_CORRIDOR: coarse path at guide_path_level, then A* on
    //     the fine map limited to cells in the coarse path's nodes (plus
    //     guide_path_corridor_margin rings of neighbouring nodes).
    // Both hierarchy sources fall back to full-map A* if they fail.
    // Independent of flow_solve_level.
    enum GuidePathSource { GUIDE_SOURCE_ASTAR = 0, GUIDE_SOURCE_LIFT = 1, GUIDE_SOURCE_CORRIDOR = 2 };
    int guide_path_source = GUIDE_SOURCE_ASTAR;
    int guide_path_level = 4;
    int guide_path_corridor_margin = 0;
    bool guide_path_corridor_congestion = false;
    // --guidePathTrace: CSV file with one row per guide path the planner
    // builds in stage 2 (any source); empty = off.
    std::string guide_path_trace_file;

    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    vector<State> curr_states;

    TaskPool task_pool; // task_id -> Task
    vector<int> new_tasks; // task ids of tasks that are newly revealed in the current timestep
    vector<int> new_freeagents; // agent ids of agents that are newly free in the current timestep
    vector<int> curr_task_schedule; // the current scheduler, agent_id -> task_id

    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    SharedEnvironment(){}

    // vector<pair<int,int>> past_waitings;
    vector<pair<double,double>> past_waitings;
    vector<int> accu_waitings;
};
