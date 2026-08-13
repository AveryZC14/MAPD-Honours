# Project todo list

Running list of follow-up work the user wants tracked across sessions.
Unlike the other `ai/*.md` docs (which record what's already been
investigated/done), this one is forward-looking — check it at the start of a
session for open items, and check items off (move to a "Done" section with
the date and what changed) rather than deleting them outright.

## Open

- [ ] **New output metric: timesteps actually scheduled/planned for, distinct
  from `makespan`.** `makespan` (see `ai/project_context.md` "Makespan vs.
  'timesteps solved'") counts real elapsed simulated ticks, including
  forced-wait ticks replayed while a slow scheduler/planner call was still
  running. The existing `len(timeStepMetrics)` isn't the right thing either —
  it appends 1 entry for a whole burst of catch-up ticks plus 1 more for the
  real move, i.e. up to 2 log rows per *single* underlying planning decision,
  not a count of decisions itself. What's wanted is a clean counter of how
  many times the scheduler+planner were actually invoked and produced a
  fresh decision — i.e. the number of `BaseSystem::simulate()` outer-loop
  iterations / `plan()` calls that returned a real result, as opposed to
  `makespan`'s count of simulated clock ticks (real + forced-wait). Natural
  place to add it: a counter incremented once per outer-loop iteration in
  `BaseSystem::simulate()` (`src/CompetitionSystem.cpp:147`), written to the
  output JSON alongside `makespan` in `saveResults()` (`:258`). Would make it
  possible to directly see, e.g., "solver 1 only got 6 real scheduling
  decisions in during 201 simulated timesteps" instead of inferring it from
  the "steps recorded" workaround used in `ai/auto_benchmarking.md`.

- [ ] **Split `PlannerTime` into its scheduler and path-planner components.**
  Confusing right now: the `PlannerTime` field in each timestep's output
  (`TimeStepMetric::PlannerTime`, `inc/CompetitionSystem.h:19`) isn't just the
  low-level path planner — it's the wall-clock of the *entire*
  `Entry::compute()` call, which runs `scheduler->plan()` (task assignment,
  varies by `--scheduleModel`) followed by `planner->plan()` (low-level
  pathfinding, same code every solver). It's set from a single
  `std::chrono` measurement wrapping both in `BaseSystem::plan()`
  (`src/CompetitionSystem.cpp:162-211`). Meanwhile `SchedulerSolveTime` /
  `SchedulerGuidePathTime` are already captured separately via
  `last_scheduler_timing` (`ScheduleTiming` struct, `default_planner/scheduler.h:18`,
  populated by `set_last_timing`/`set_last_reduced_timing` in
  `default_planner/scheduler.cpp:18-38`) but aren't subtracted out anywhere,
  so it's easy to misread `PlannerTime` as pure path-planning cost when
  solver 1 vs. solver 6 differences are actually dominated by scheduler cost
  (see `ai/auto_benchmarking_IH_mp_2p_01.md`).
  - Rename/keep `PlannerTime` as `TotalPlanTime` (the full `Entry::compute()`
    wall-clock, what's measured today).
  - Add a `PathPlannerTime` field = `TotalPlanTime - SchedulerSolveTime -
    SchedulerGuidePathTime` (or time `planner->plan()` directly for
    precision instead of subtracting, since `planner_wrapper()` in
    `src/CompetitionSystem.cpp:53-70` already calls `scheduler->plan()` and
    `planner->plan()` — wait, actually `scheduler->plan()` and
    `planner->plan()` are called inside `Entry::compute()`
    (`src/Entry.cpp:32,38`), not directly in `planner_wrapper()` — timing
    would need to move into `Entry::compute()` itself, or `Entry` would need
    to expose per-call timings the way the scheduler already does).
  - Keep `SchedulerSolveTime` / `SchedulerGuidePathTime` as-is (already
    correct, just underused).
  - Update `visualisation/compute_throughput_metrics.py` and the two
    `ai/auto_benchmarking_*.md` docs' methodology notes once the new field
    exists, so future sweeps read `PathPlannerTime` instead of misreading
    `PlannerTime`/`TotalPlanTime` as planner-only cost.

- [ ] **(Suggestion, not yet requested) Find the solver-1-vs-solver-6
  crossover map size.** `orz900d` (~978K cells): solver 1 wins. `IH_mp_2p_01`
  (~3.44M cells): solver 6 wins by ~4-7x. Somewhere between those two map
  sizes the relationship flips — a sweep on an intermediate-sized map would
  pin down roughly where, which seems like a genuinely useful data point for
  the thesis's scaling argument. See `ai/auto_benchmarking.md` synthesis
  section. Flagging this because it fell out of the `IH_mp_2p_01` sweep, not
  because it's been asked for.

- [ ] **(Follow-up, not required for the fix below) Guide-path lifting for
  locally-matched pairs.** The new within-coarse-node local matching (see
  "Done" below) never touches the coarse graph, so `compute_reduced_assignment`'s
  Steps 3-4 don't produce a lifted guide path for those agents — they
  currently just fall back to the low-level planner's own seed (`update_traj`/
  `astar` in `flow.cpp`), same as any agent with no `agent_guide_path` entry.
  Deliberately deferred rather than fixed now. Only matters when `--useTraffic`
  is on (that's the only time `agent_guide_path` is consumed at all, see
  `ai/project_context.md` "Guide paths" section) — worth a direct fine-map
  A*/BFS per local pair if/when guide-path completeness under traffic mode is
  actually being evaluated.

- [ ] **`schedulerHierarchyBuildTime`/`schedulerHierarchyLevelNodeCounts`
  came back empty on one solver-6 run for no apparent reason.** During the
  2026-08-12 `scene_mp_4p_03` sweep (`ai/auto_benchmarking_scene_mp_4p_03.md`),
  the `--flowSolveLevel 4` run's JSON reported `schedulerHierarchyBuildTime:
  0.0` and `schedulerHierarchyLevelNodeCounts: []`, while the `1`/`2`/`3`
  runs (identical command shape, same `--hierarchyCache` load path,
  comparable wall-clock) all reported real values. Run completed with 0
  errors and plausible throughput numbers, so not a correctness bug, but the
  metrics gap itself is unexplained — worth a look if these fields matter
  for a future writeup.

- [ ] **scene_mp_4p_03 at 10000/20000 agents.** 5000-agent sweep (4x solver
  6 levels 1-4 + solver 1, see `ai/auto_benchmarking_scene_mp_4p_03.md`) took
  ~43 minutes total wall-clock and found the widest solver-6-wins margin of
  any sweep so far, plus a new "deeper coarsening helps instead of hurting"
  pattern only seen on this map. User asked to hold off on 10000/20000 until
  timing was known; now it is, and both instances already exist
  (`scene_mp_4p_03_10000.json`/`_20000.json`) with a reusable
  `--hierarchyCache` file already built, so a follow-up sweep is cheap to
  run whenever wanted.

## Done

- [x] **2026-08-13: Solver 6 within-coarse-node agent<->task pairing fix
  implemented and validated** (was: "quantified 2026-08-13, fix design
  sketched, not yet implemented"). `compute_reduced_assignment`
  (`map_reduction_test/MapCoarsenV1.cpp:1437`) solves min-cost flow on the
  *coarse* graph only — when an agent's and a task's locations map to the
  same top-level node (`flow_solve_level`), the arc cost between them is 0,
  so the flow solve has no signal to prefer one fine-grained pairing over
  another. Recovery then just pops `top_task_ids[node].front()`
  (`MapCoarsenV1.cpp:1679`) against agents in `flexible_agent_ids` order —
  insertion-order-dependent, uncorrelated with real fine-map distance. The
  existing pin-already-assigned-tasks fix (`scheduler.cpp:996-1010`) only
  stops this pairing from being *re-decided* (churn) every timestep; it does
  nothing for the *first* pairing when new agents/tasks co-locate in a
  coarse node.

  **Quantified on `scene_mp_4p_03`** (new standalone diagnostic,
  `map_reduction_test/analyze_coarse_collisions.cpp` /
  `./build/analyze_coarse_collisions <instance.json> <hierarchy_cache.bin>
  [max_level] [task_cap] [min_level]`) by re-solving the real top-level flow
  at each `--flowSolveLevel` and comparing the actual real (fine-grid
  Manhattan) distance of the resulting assignment against the *exact*
  minimum-cost real-distance matching (LEMON `NetworkSimplex` on true
  distances, not a heuristic) for the same batch of agents/tasks, split by
  whether the pair shared a coarse node or not:

  | level | same-node pairs | same-node waste (total / per-pair) | diff-node pairs | diff-node waste (total / per-pair) |
  |---|---|---|---|---|
  | 1 | 22 | 0 / 0.00 | 4978 | 2210 / 0.44 |
  | 2 | 76 | 0 / 0.00 | 4924 | 2512 / 0.51 |
  | 3 | 317 | 12 / 0.04 | 4683 | 3952 / 0.84 |
  | 4 | 1074 | 58 / 0.05 | 3926 | 7822 / 1.99 |
  | 5 | 2576 | 3542 / 1.38 | 2424 | 14808 / 6.11 |
  | 6 | 3993 | 45938 / 11.50 | 1007 | 13130 / 13.04 |

  Same-node waste is ~0 at shallow levels and explodes from level 4 onward
  (tracks the `ai/auto_benchmarking_scene_mp_4p_03.md` throughput crossover
  at level 4→5) — by level 6 it's the majority of total measured waste. A
  *separate*, smaller-in-aggregate-but-comparable-per-pair effect exists for
  pairs that *don't* share a node (cost estimated only via the coarse
  graph's aggregated inter-node arcs) — real at every level, and roughly
  tied with the same-node effect per-pair by level 6. A same-node fix (below)
  would not address this second effect.

  **Fix design (2026-08-13, implemented 2026-08-13):** before building the
  per-timestep flow graph in `compute_reduced_assignment`'s Step 1, group
  flexible agents/tasks by top-level node as today, but for every node with
  `a` agents and `t` tasks, pull **all** `a` agents and `t` tasks out of the
  flow graph entirely (don't add their supply/demand arcs) and solve one
  small real-distance min-cost bipartite matching over the full `a x t`
  local group. That matching naturally produces exactly `min(a,t)` pairs and
  *implicitly* selects which specific agents/tasks are "local" vs. "surplus"
  as part of minimizing real distance — do not pre-select an arbitrary
  `min(a,t)` subset before matching (an earlier draft of this design did
  that, and it's a real bug: pre-selecting arbitrarily can strand an agent
  with a great local match out in the surplus pile while a worse-positioned
  agent takes its spot, purely because of list order). Whichever `|a-t|`
  agents or tasks the matching leaves unpaired are the ones that get added
  to the flow graph as today's surplus.

  This is provably lossless *to the coarse flow's own cost function*: same-
  node supply/demand arcs cost 0 and real inter-node moves cost >0, so a
  min-cost flow was always going to match `min(a,t)` pairs locally for free
  regardless of which specific agent/task it picked — pulling the whole
  group out and matching it properly doesn't change the flow's optimal
  *coarse-cost* total, it just replaces an arbitrary pairing (and an
  arbitrary local/surplus split) with a distance-informed one. Validation
  below found this framing needs one caveat: it is not lossless on *total
  realized real distance* including the diff-node group, because which
  specific agents become "surplus" changes, and the coarse flow's own
  diff-node routing is itself only cost-optimal in coarse-graph-cost terms,
  not real distance — see "Secondary refinement" below, which turned out to
  matter empirically, not just in theory.

  Secondary refinement, now empirically confirmed (not a hypothetical): the
  local matching only minimizes *local* real distance for the `min(a,t)`
  pairs it keeps — it has no visibility into how good a remote match the
  `|a-t|` surplus agent(s)/task(s) might get from the flow network, so a
  different local/surplus split can do worse end-to-end even though the
  local subset itself is now optimal. Validation found this costs a small
  amount (~0.2-0.5% of total realized distance) at shallow levels where
  there's no same-node waste to offset it, and is dwarfed by the same-node
  gain at the levels that actually matter (5-6) — see "Validation" below.
  Not solvable cleanly without circular dependency on the flow's own output,
  and the coarse flow never had real per-agent remote-cost information to
  offer anyway, so this remains a known, small, accepted cost of the fix,
  not a blocker.

  Guide-path lifting for locally-matched pairs was deliberately deferred,
  not implemented — see the new Open item above ("Guide-path lifting for
  locally-matched pairs"). See conversation 2026-08-12 (original discovery)
  and 2026-08-13 (quantification + fix design + implementation + validation)
  for full analysis.

  **Implementation (2026-08-13):** new `map_reduction_test/LocalNodeMatch.{h,cpp}`
  — `match_local_node_exact()`, exact min-cost bipartite matching (LEMON
  `NetworkSimplex`) on real fine-grid Manhattan distance, exposed as a
  `LocalNodeMatcher` (`std::function`) typedef so the matching strategy is
  swappable without touching the surrounding flow-graph code. Wired into
  `compute_reduced_assignment`'s Step 1 (`MapCoarsenV1.cpp`) exactly per the
  fix design above: agents/tasks are first bucketed by top-level node; nodes
  with both present get matched directly and the pairs go straight into
  `assignments`; only the leftover surplus populates
  `start_supply`/`top_task_ids`/`agent_to_top_node` for the (unchanged) flow
  graph below. `num_workers` (the flow's source/sink supply magnitude) now
  uses the surplus agent count instead of the total flexible agent count,
  mirroring what the original code already did with the full count (so the
  agent/task-count gap fed into the flow is unchanged from before this pass
  existed, not newly introduced by it).

  **Validation (2026-08-13):** extended `analyze_coarse_collisions.cpp`
  (new `fixed_matched`/`fixed_total_dist`/`total_dist_before_fix`/
  `total_waste_eliminated_by_fix` CSV columns, plus a
  `solve_top_level_assignment_with_local_match()` that reproduces the fixed
  Step 1/2 pipeline standalone) to measure total realized real distance
  before vs. after the fix, not just the theoretical same-node-only optimum
  the table above already showed. Found and fixed a real bug in
  `match_local_node_exact` during this pass: it read matched pairs via
  `flow[arc]`, indexing the external `ArcMap` passed to `NetworkSimplex::flowMap()`
  directly — that map is not reliably populated post-solve in this LEMON
  version, so the function silently returned zero matches on every call
  (`ns.run()` itself correctly reported `OPTIMAL`, masking the bug). Fixed
  to read via `ns.flow(arc)` (the solver's own accessor), matching the
  pattern already used elsewhere in this file
  (`run_top_level_flow_and_recover` / `compute_reduced_assignment`). Re-ran
  on `scene_mp_4p_03` (5000 agents/5000 tasks, same batch as the
  quantification table above):

  | level | same-node excess (theoretical max gain) | total waste eliminated by fix |
  |---|---|---|
  | 1 | 0 | -962 |
  | 2 | 0 | -1760 |
  | 3 | 0 | -786 |
  | 4 | 6 | +72 |
  | 5 | 1586 | +4768 |
  | 6 | 28010 | +50596 |

  Confirms the design: net regression (small, <0.6% of total distance) at
  shallow levels 1-3 where there's no same-node waste to begin with (the
  "secondary refinement" cost above), and a large net win at levels 5-6 —
  the same levels where `ai/auto_benchmarking_scene_mp_4p_03.md` found the
  solver-1-vs-6 throughput crossover. At level 6 the realized gain (50596)
  exceeds the naive same-node-only estimate (28010), meaning the fix also
  improves diff-node routing as a side effect, not just the same-node pairs
  directly.

  **Sanity checks (2026-08-13):** `warehouseSmall_100`, solver 6, 200
  timesteps — completed cleanly, 383 tasks finished, no errors. `scene_mp_4p_03_5000`,
  solver 6, `--flowSolveLevel 6`, `--hierarchyCache` (reused the cached
  hierarchy from the quantification run above), 250 timesteps — completed
  cleanly, 1410 tasks finished, no crashes/errors.

- [x] **2026-07-29: Guide-path reconstruction rigor pass + GuidePathLengthSum/
  GuidePathCostSum metric.** Verified solver 6's guide-path output is
  format-interchangeable with solver 1's (same `boost::unordered_map<int,
  list<int>>`, valid start/end/adjacency) via a new standalone tool
  (`./build/guide_path_validator`); found and fixed a real bug where the
  fine-lift could silently return a guide path anchored on the wrong
  sub-component when a coarse parent spans multiple disconnected regions at
  an intermediate hierarchy level. Added `GuidePathLengthSum`/
  `GuidePathCostSum` to `TimeStepMetric` for both solvers, decoupling solver
  6's fine-lift from the traffic-seed gate so the metric is populated on
  every run (verified no OOM/perf regression on `orz900d_5000`, 250
  timesteps, RSS flat ~2.3GB). Found that raw cross-solver totals of this
  metric over a multi-timestep run are only comparable with `--assignNew 1`
  (`new_only=true`) — without it, solver 1's re-offering of already-assigned
  tasks every timestep inflates its total by ~7.7x relative to solver 6's
  pin-once behavior, an artifact of recomputation cadence, not path quality.
  Full writeup: `ai/guide_path_metric.md`.

- [x] **2026-07-31: Guide-path visualisation tooling + solver 6 fine-lift
  fallback bug fix.** Built two standalone tools (`./build/dump_guide_paths`,
  `map_reduction_test/visualisation/plot_guide_paths.py`) that render solver
  1 vs. solver 6 guide paths spatially over the actual map — per-agent
  cropped/zoomed panels and a whole-map overview. While scaling this up
  (`IH_mp_2p_01`, sparse task pools), found solver 6 was silently dropping
  guide paths for many assigned agents (e.g. 15/50 returned vs. solver 1's
  50/50): the coarse-to-fine lift's direct-search fallback
  (`shortest_path_in_graph_local`, `MapCoarsenV1.cpp`) was plain Dijkstra
  with no heuristic, hard-capped at 20000 expansions — fine for its other
  use (tiny bridge hops) but nowhere near enough for an unguided search
  across a ~3.44M-cell map to a goal hundreds of cells away. Fixed with an
  optional A* mode (Manhattan-distance heuristic, exact here since fine
  arcs cost 1.0), applied only to that specific fallback call. Verified via
  `guide_path_validator` (128,062/0 failed) and re-run at every scale from
  50 up to the full 5000/10000/20000-agent instances with zero guide paths
  missing. Full writeup: `ai/guide_path_visualisation.md`.
