
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

    // Only has an effect when USE_MANHATTAN_HEURISTIC is true. When true,
    // PIBT stops scoring moves by raw Manhattan distance to the goal (which
    // traps agents behind walls) and instead runs a small BFS from each agent
    // to its own guide path, scoring each move by distance to the path plus
    // steps left along it. Also rotates where the guide-path loop starts each
    // step, and re-plans agents pushed too far off their path. When false,
    // the planner is exactly the pure-Manhattan planner. See
    // ai/planner_local_bfs_plan.md. Can also be set at build time with
    // -DPLANNER_USE_LOCAL_PATH_BFS=false, to keep a second pure-Manhattan
    // build around without editing this file.
#ifndef PLANNER_USE_LOCAL_PATH_BFS
#define PLANNER_USE_LOCAL_PATH_BFS true
#endif
    const bool USE_LOCAL_PATH_BFS = PLANNER_USE_LOCAL_PATH_BFS;

    // Local BFS radius cap. An agent with no guide-path cell within this many
    // steps falls back to Manhattan distance for the step and gets a new path.
    const int LOCAL_PATH_BFS_RADIUS = 5;

    // Layers the local BFS keeps searching after the first path cell is
    // found, so it can pick a better path cell nearby.
    const int LOCAL_PATH_BFS_EXTRA_LAYERS = 2;

    // When true, solvers 6 and 7 hand their guide paths to the planner
    // every step, not only with --useTraffic past timestep 100. Off by
    // default: it changes what solver comparisons measure (see the plan doc).
    // Can also be set at build time with -DPLANNER_PASS_SCHEDULER_PATHS=true.
#ifndef PLANNER_PASS_SCHEDULER_PATHS
#define PLANNER_PASS_SCHEDULER_PATHS false
#endif
    const bool PASS_SCHEDULER_PATHS_TO_PLANNER = PLANNER_PASS_SCHEDULER_PATHS;

    // An agent with a goal that hasn't moved for this many consecutive
    // planner decisions counts as stuck in the per-step planner log line.
    const int STUCK_AGENT_THRESHOLD = 20;

}
#endif