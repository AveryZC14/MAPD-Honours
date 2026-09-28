
#ifndef CONST_H
#define CONST_H
namespace DefaultPlanner
{
    // pibt runtime (ms) per 100 agents. 
    // The default planner will use this value to determine how much time to allocate for PIBT action time.
    // The default planner compute the end time for traffic flow assignment by subtracting PIBT action time from the time limit.
    const int PIBT_RUNTIME_PER_100_AGENTS = 1;

    // Traffic flow assignment end time tolerance in ms.
    // The default planner will end the traffic flow assignment phase this many milliseconds before traffic flow assignment end time.
    const int TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE = 60;


    // The default planner timelimit tolerance in ms.
    // The MAPFPlanner will deduct this value from the time limit for default planner.
    const int PLANNER_TIMELIMIT_TOLERANCE = 20;

    // The default scheduler timelimit tolerance in ms.
    // The TaskScheduler will deduct this value from the time limit for default scheduler.
    const int SCHEDULER_TIMELIMIT_TOLERANCE = 20;

    // When true, every heuristic lookup (get_h/get_gp_h/astar) returns raw
    // Manhattan distance instead of the exact BFS-from-goal distance table,
    // and the planner skips building/touching that table entirely (bypasses
    // global_heuristictable and its LRU cache altogether). Also skips
    // building each agent's Dist2Path table (update_dist_2_path, in
    // update_traj() and the scheduler-guide-path-seed branch of
    // planner.cpp) -- that table is per-agent and full-map-sized with no
    // eviction, and nothing reads it once get_gp_h() no longer needs it.
    // Trades heuristic accuracy (Manhattan ignores walls, and frank_wolfe's
    // replan-priority ordering degrades to last-replan-time instead of
    // true path deviation) for zero table-build cost on both fronts.
    const bool USE_MANHATTAN_HEURISTIC = true;

}
#endif