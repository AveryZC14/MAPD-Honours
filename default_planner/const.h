
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

    // Worker threads for guide-path A* in stage 2 of plan(). 1 = the original
    // sequential loop. >1 = searches run in parallel against the congestion
    // map as it was at the start of the step, and are committed afterwards
    // (see ai/parallel_guide_paths_plan.md). Only used with
    // USE_MANHATTAN_HEURISTIC (the exact heuristic's tables are filled in
    // lazily by A*, so they can't be shared across threads). Can be set at
    // build time with -DPLANNER_GUIDE_PATH_THREADS=<n>. Default 1 as of
    // 2026-09-30: on orz900d 10k parallel paths gave 11-18% fewer deliveries
    // than sequential ones; they only help on IH together with
    // GUIDE_PATH_IGNORE_CONGESTION. See ai/parallel_guide_paths_plan.md.
#ifndef PLANNER_GUIDE_PATH_THREADS
#define PLANNER_GUIDE_PATH_THREADS 1
#endif
    const int GUIDE_PATH_THREADS = PLANNER_GUIDE_PATH_THREADS;

    // Memory cap for the extra A* search pools the guide-path threads need
    // (one whole-map pool each, 56 bytes per map cell; thread 0 reuses the
    // planner's existing pool). Fewer threads are used when the pools won't
    // fit: 6 on orz900d/IH/warehouseXL, 3 on the scene maps (1 GB per pool on
    // scene_sp_pol_06, which already peaks at about 24 GB).
    const long long GUIDE_PATH_EXTRA_POOL_MB = 2048;

    // Debug checks on guide paths: every new path starts at the agent and
    // ends at its goal with single-cell steps, and every 100 decisions the
    // congestion map is rebuilt from all paths and compared with lns.flow.
    // Exits the program on a mismatch. Off for real runs. Can be set at
    // build time with -DPLANNER_GUIDE_PATH_DEBUG_CHECKS=true.
#ifndef PLANNER_GUIDE_PATH_DEBUG_CHECKS
#define PLANNER_GUIDE_PATH_DEBUG_CHECKS false
#endif
    const bool GUIDE_PATH_DEBUG_CHECKS = PLANNER_GUIDE_PATH_DEBUG_CHECKS;

    // Weight on the Manhattan heuristic in guide-path A* (weighted A*), used
    // only with USE_MANHATTAN_HEURISTIC. 1.0 = plain A*. Above 1, A* heads
    // more directly for the goal instead of exploring everything cheaper
    // than the final route once the congestion map is busy; found paths cost
    // at most this factor times the best (length plus congestion penalties).
    // Can be set at build time with -DPLANNER_ASTAR_HEURISTIC_WEIGHT=<w>.
#ifndef PLANNER_ASTAR_HEURISTIC_WEIGHT
#define PLANNER_ASTAR_HEURISTIC_WEIGHT 1.0
#endif
    const double ASTAR_HEURISTIC_WEIGHT = PLANNER_ASTAR_HEURISTIC_WEIGHT;

    // Diagnostic: when true, the parallel guide-path searches ignore the
    // congestion map (plain shortest paths). Used to measure what the
    // congestion penalties cost; not a decided planner setting. Can be set
    // with -DPLANNER_GUIDE_PATH_IGNORE_CONGESTION=true.
#ifndef PLANNER_GUIDE_PATH_IGNORE_CONGESTION
#define PLANNER_GUIDE_PATH_IGNORE_CONGESTION false
#endif
    const bool GUIDE_PATH_IGNORE_CONGESTION = PLANNER_GUIDE_PATH_IGNORE_CONGESTION;

    // An agent with a goal that hasn't moved for this many consecutive
    // planner decisions counts as stuck in the per-step planner log line.
    const int STUCK_AGENT_THRESHOLD = 20;

}
#endif