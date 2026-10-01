# Project todo list

Running list of follow-up work the user wants tracked across sessions.
Unlike the other `ai/*.md` docs (which record what's already been
investigated/done), this one is forward-looking — check it at the start of a
session for open items, and check items off (move to a "Done" section with
the date and what changed) rather than deleting them outright.

## Open

- [ ] **Run the final thesis sweep (120 runs, 8,000 timesteps each).** Added
  2026-09-29. The full plan (run list, flags, rationale, caveats, runtime of
  about 11.5 days) is in `instances/thesis_benchmarks/README.md` under "Final
  run plan". Pre-sweep checks are done (2026-09-29): the 80k smoke tests
  passed, and the `TasksOpenedThisStep` pickups counter was added and
  verified. The sweep script is `scripts/run_thesis_sweep.py` (tested at 30
  steps). warehouseXL runs last (the user may drop it; the sweep without
  it is 94 runs, about 9 days).

  **Sweep stopped 2026-09-29 16:50 after 4 runs; results unusable.** All 4
  (orz900d 10k, solver 6 levels 2/4/6/8) froze at about step 1,000: e.g.
  level 4 had 10,533 pickups and 619 deliveries by step 1,000, then +0 of
  each for 7,000 steps, with 0 errors and the planner using its full time
  budget every step. The scheduler level made no difference (599-640
  finished), so the cause is below the scheduler. **Confirmed 2026-09-29:
  the Manhattan heuristic** (PIBT ranks moves by raw Manhattan distance and
  ignores the A* guide path, so agents behind walls wait forever). The
  pickup counter, the task format and the scheduler were ruled out; see
  `ai/run_log.md` 2026-09-29. **Next:** fix the planner before rerunning.
  The exact heuristic doesn't freeze but is starved (543 decisions in 1,500
  steps). Plan (2026-09-29, replaces the earlier waypoint idea): keep
  Manhattan distance inside A*, but have PIBT score moves with a small BFS
  from each agent to its guide path (distance to path + steps remaining).
  Full plan, caveats and checks in `ai/planner_local_bfs_plan.md`.
  **Implemented 2026-09-29** (`USE_LOCAL_PATH_BFS`); orz900d 10k 1,500-step
  check passed (5,495 deliveries vs. 617, no freeze). **IH 10k check
  failed (2026-09-29, `ai/run_log.md`):** guide-path A* takes about
  70-300 ms per search there, so about 6,700 agents never get a path, stay on
  Manhattan distance and get trapped (4,661 stuck), and A* overruns the
  step (1,021 decisions in 1,500 steps). Sweep NOT restarted. Next: make
  guide paths cheap enough on big maps (see the plan doc's "IH result"
  section). Plan: build them in parallel, `ai/parallel_guide_paths_plan.md`.
  **2026-09-30: implemented behind flags, default left sequential.** No
  setting works on every map (orz900d best sequential; IH needs parallel +
  congestion ignored; scene_mp_4p_03 fails either way, needs a better A*
  heuristic). **Decision needed**; options in that doc's "Conclusions".
  **2026-10-01: new option, guide paths from the coarsening hierarchy**
  (`ai/hierarchical_guide_paths_plan.md`). Benchmarked, not yet in the
  planner. With every level anchored, the lift gives valid paths on all
  three maps in a few ms each.
  Also fixed a harness bug that cost up to about 650 ms of planner time per
  step on big maps (`ai/run_log.md` 2026-09-30); earlier big-map results
  are affected. Before restarting:
  consider a 20k / larger-map check (PIBT used up to 77 of its 100 ms),
  then delete `outputs/thesis_sweep/STOP`. (The 4 stale runs were moved to
  `outputs/thesis_sweep_junk/2026-09-29_manhattan_freeze/` on 2026-09-29.)
  Also still needed: an analysis script for the
  per-decision solver-1 comparison.

- [ ] **Decide whether to `git rm hierarchy_cache/orz900d.hier`.** Added
  2026-09-29. It is tracked in git (47 MB) and byte-identical to the deleted
  early-stopping `orz900d.hierarchy`, so it stops at level 9; use
  `orz900d_full.hierarchy` instead. It was kept only because it was tracked
  on purpose. orz900d builds in 0.6 s, so no tracked cache is needed. The
  other six superseded caches (about 7.2 GB) were deleted on 2026-09-29, after
  every script and copy-paste command was switched to the full-depth
  replacements.

- [ ] **[HIGH] Solver 6's per-timestep coarse-flow solve re-solves the entire
  backbone graph every call, regardless of how few agents/tasks actually need
  matching — dominant cost of long runs, not a memory leak.** Found
  2026-08-27 while investigating a 30+ minute single-decision stall during a
  `scene_mp_4p_03`, solver 6, `--flowSolveLevel 1`, `-s 5000` long-run sweep
  (see `ai/auto_benchmarking.md`/`ai/auto_benchmarking_scene_mp_4p_03.md` for
  the sweep itself once written up). Two false leads ruled out first, worth
  recording so a future session doesn't re-derive them:
  - **Not task scarcity.** `numTasksReveal 1.5` keeps `ongoing_tasks` topped
    up to `1.5 * num_agents`; since every busy agent locks up exactly one
    task, the busy-agent term cancels out of the leftover-tasks-vs-leftover-
    agents comparison algebraically — genuine system-wide task scarcity is
    structurally impossible whenever the reveal ratio exceeds 1.0.
  - **Not the "coarse-flow all-or-nothing infeasibility" bug** (a different,
    real, still-open item below). Read `schedule_plan_flow_reduced`
    (`default_planner/scheduler.cpp:994-1043`) directly: `flexible_agent_ids`
    is sourced solely from `env->new_freeagents`, which
    `TaskManager::check_finished_tasks` rebuilds from scratch every call —
    so a batch that fails to match doesn't accumulate into a growing
    backlog, it just drops out of consideration. Confirmed directly with
    temporary diagnostic instrumentation (added and reverted the same
    session, not committed): `surplus_agents` *shrank* from 4977 at
    timestep 0 to single digits (1-9) by timestep ~1400 (most agents just
    end up busy on long routes on a map this size), and zero non-`OPTIMAL`
    results occurred across 82 decisions — no infeasibility fired at all.

  **Actual root cause, confirmed by splitting the timing**: across those
  same 82 decisions, `NetworkSimplex::run()` stayed in a flat 6.6-21.5s band
  (median 13.5s) *regardless* of whether the surplus pool was 4977 or 1 —
  pool size doesn't predict solve time at all. Separately timing the graph
  construction (`map_reduction_test/MapCoarsenV1.cpp:1738-1832`, the loop
  that copies `top->g`'s ~1.6M nodes and however many arcs into a fresh
  `ListDigraph` every call) vs. the `ns.run()` call itself: build ~1-2.5s,
  solve ~22s on the first decision — i.e. `NetworkSimplex`'s own
  initialization+pivoting cost, not the graph copy, dominates. This makes
  sense: simplex initialization cost scales with total graph size (V+E),
  not with actual supply/demand magnitude, so a 1-agent match still pays
  the full cost of solving over the entire backbone.
  - **Why this matters specifically for long runs**: surplus pools shrink to
    near-zero for most of a long run (agents settle onto long routes), so
    *most* decisions in a long run pay this fixed backbone-sized cost for
    almost no actual matching work. Directly explains why `scene_mp_4p_03`
    level-1 real-time throughput (`tp/makespan`) dropped from 1.491 (the
    original 500-tick sweep) to 0.286 (extended to 5000 ticks) — a ~5.2x
    drop, not a warm-up effect.
  - **Candidate fix, two complementary parts** (not yet implemented):
    1. **Small-batch short-circuit (primary, highest ROI)**: when
       `surplus_agents`/`surplus_tasks` is below a small threshold (e.g.
       <50, tunable), skip the full-backbone `NetworkSimplex` solve
       entirely and solve a much smaller point-to-point bipartite matching
       instead (a handful of bounded shortest-path calls between just the
       surplus agents/tasks) — same spirit as the existing
       `match_local_node_exact` (`LocalNodeMatch.h`) but extended beyond
       same-coarse-node pairs to genuine cross-node small matching.
    2. **Cache/reuse the static backbone `ListDigraph`** (the 1.6M coarse
       nodes + coarse-to-coarse arcs/costs) across calls instead of
       rebuilding from scratch every timestep — only add/remove the small
       per-call source/sink/surplus arcs. Only ~10% of the measured
       per-call cost on its own (per the build-vs-solve split above), so
       lower value than (1) alone, but free/complementary and helps every
       call including large-surplus ones early in a run.
  - **Explicitly not guaranteed byte-identical, discussed and accepted
    2026-08-27**: total flow cost/optimality is guaranteed unchanged by
    either fix (same graph/costs/capacities/supply always yields the same
    optimal *value*), but the *specific* tie-broken assignment `NetworkSimplex`
    returns among several equally-optimal solutions is not guaranteed to
    match the current from-scratch-rebuild behavior — this repo already has
    a documented instance of arc-order-dependent tie-breaking elsewhere
    (`ai/solver6_preprocessing_efficiency.md`). Caching the backbone graph
    could change effective arc insertion/ID order for the per-call dynamic
    arcs (LEMON's `ListDigraph` reuses freed IDs on erase), and `ns.flowMap(flow)`
    is called before `run()` — the analogous solver-1 code comments this as
    a "warm start," so stale flow values surviving on unchanged backbone
    arcs across calls could genuinely steer which optimal vertex the simplex
    converges to, not just be a passive output buffer. User decided not to
    require a before/after per-decision assignment diff before implementing
    this — flagging here so a future session knows the tradeoff was
    consciously accepted, not overlooked.
  - **Deferred, lower-priority alternatives considered**: investigating
    whether this LEMON version supports genuine `NetworkSimplex` warm-
    starting between calls (uncertain API support, not researched); pruning
    to a bounded relevant subgraph instead of a batch-size threshold (more
    general than the small-batch short-circuit, but riskier — needs care to
    guarantee the pruned region still contains the true optimal route).
  - **Validation plan for whenever this is implemented**: temporarily
    re-add the diagnostic timers (reverted this session, not committed) on
    the same `scene_mp_4p_03` `--flowSolveLevel 1` repro to confirm
    per-decision time actually drops for small-surplus calls; correctness-
    check via `analyze_coarse_collisions` and a clean `warehouseSmall_100`
    re-run, matching the validation pattern used for every prior solver-6
    fix in this repo.
  - Scoped entirely to `compute_reduced_assignment`
    (`map_reduction_test/MapCoarsenV1.cpp`, roughly lines 1738-1832); no
    changes needed elsewhere. Solver 7 has its own separate backbone-reuse
    mechanism already (`lemon::digraphCopy`, built once — see
    `ai/edge_node_representation.md`), so it may not share this problem to
    the same degree, but wasn't specifically re-audited here.

- [ ] **Solver 7 metric instrumentation: two bugs in `SchedulerBackboneBuildTime`
  and `SchedulerSolveTime`, found auditing all of solver 7's per-timestep
  fields for correctness (2026-08-21).** Everything else checked out fine
  (`GuidePathLengthSum`/`GuidePathCostSum`/`SchedulerGuidePathTime`/
  `LocalNodeMatchCount`/`FlowMatchCount` are all correctly scoped, reset per
  call, and computed on solver 7's own actual output — not reused from
  solver 6). Two real problems in the timing fields specifically:

  1. **`SchedulerBackboneBuildTime` always reads 0.0, on every call,
     including the one that actually rebuilds.** Root cause:
     `EdgeAugmentedHierarchy::ensure()` unconditionally resets
     `last_backbone_build_time_ = 0.0` at the top
     (`map_reduction_test/EdgeAugmentedCoarsen.cpp:277`), before doing
     anything else. `ensure()` is called **twice** per scheduler invocation
     with identical `(env, flow_solve_level)` arguments — once at
     `default_planner/scheduler.cpp:1168` (just to check `ready()` for the
     solver-1 fallback branch), and again inside
     `compute_reduced_assignment_edge_augmented` itself at
     `EdgeAugmentedCoarsen.cpp:357`, whose result is what actually gets read
     into the output field. `compute_env_signature` (`MapCoarsenV1.cpp:1018-1028`)
     only hashes static map dimensions/cells, which can't change mid-run, so
     both calls always see the identical signature within one timestep.
     Whichever call does the real rebuild (always the first one, since
     nothing invalidates the cache between the two calls), the *second*
     call's cache-validity check (`EdgeAugmentedCoarsen.cpp:305-306`) finds
     it already valid and returns immediately — but only after line 277 has
     already zeroed the timer back out. Confirmed by direct code read, not
     just the auditing subagent's report.
     - Note this field being *mostly* zero across a run is actually correct
       by design, not itself a bug: the backbone is meant to be built once
       (keyed on map+`--flowSolveLevel`, both fixed for a whole run) and
       reused via `lemon::digraphCopy` every timestep after that (see
       `ai/edge_node_representation.md`), so the healthy shape for this
       field is "one real nonzero value on the first successful call, zeros
       forever after" — confirming the cache is working. The bug is that the
       one nonzero value that should exist is also erased, so there's no way
       to see the real one-time build cost at all, ever.
     - **Candidate fix**: don't call `ensure()` twice per invocation. Either
       have `schedule_plan_flow_reduced_edge` (`scheduler.cpp:1168`) reuse
       the same `ensure()` result that `compute_reduced_assignment_edge_augmented`
       will produce internally (restructure so `ensure()` is called once and
       both the readiness check and the backbone-time capture read off that
       single call), or have the second call at `EdgeAugmentedCoarsen.cpp:357`
       skip resetting/re-running `ensure()` if the first call this timestep
       already confirmed validity. Not yet designed in detail.

  2. **`SchedulerSolveTime` has a wider, undocumented scope than solver 6's
     equivalent field, and silently overlaps `SchedulerLocalMatchTime`.**
     **Fixed 2026-10-01:** both solvers' solve timers now start right after
     local matching and stop before the lift (which is now guide time), so
     solve time includes coarse graph construction (backbone copy for solver
     7) and the three fields are disjoint. See
     `ai/hierarchical_guide_paths_plan.md`, "Guide-path flag and timing
     fields". Item 1 (`SchedulerBackboneBuildTime`) is still open. Original
     description below; line numbers are from before the change.
     Solver 6's `solve_start` timer (`MapCoarsenV1.cpp:1828`) starts
     immediately before `ns.run()` — *after* its own Step 1 local matching
     (starting at line 1627) has already finished, so `SchedulerSolveTime`
     and `SchedulerLocalMatchTime` are disjoint for solver 6 (explicitly
     documented in a comment at `MapCoarsenV1.cpp:1960-1965`). Solver 7's
     `solve_start` timer (`EdgeAugmentedCoarsen.cpp:362`) starts *before* its
     own Step 1 (starting at line 371) — so solver 7's `SchedulerSolveTime`
     includes Step 1 local matching, the backbone `digraphCopy`, edge-node
     arc construction, `ns.run()`, and Step 2 path recovery all bundled
     together, with no comment flagging the divergence from solver 6's
     narrower scope. Confirmed by direct code read (both timer placements
     checked line-by-line against each other).
     - Not a "wrong value" bug — each field measures exactly what its own
       code says — but it means `SchedulerLocalMatchTime` is a *subset* of
       `SchedulerSolveTime` for solver 7 while being a *disjoint sibling* of
       it for solver 6. Any analysis that sums or directly compares these
       two fields across solvers 6 and 7 (e.g. "how much of the timestep was
       spent in the flow solve alone") will silently double-count solver 7's
       local-match time and get numbers that aren't apples-to-apples against
       solver 6's.
     - **Candidate fix**: move solver 7's `solve_start` to immediately before
       `ns.run()` (mirroring solver 6's placement exactly), and account for
       Step 1 local-match time and backbone-copy/arc-construction time
       separately (the former already has its own field,
       `SchedulerLocalMatchTime`; the latter has no field at all right now
       and would either need one or get folded into `SchedulerSolveTime`
       deliberately with a comment saying so, matching solver 6's documented
       reasoning for the timing split it uses).

  Both bugs are scoped entirely to `map_reduction_test/EdgeAugmentedCoarsen.cpp`
  (solver 7's own module) — no solver 6 code needs touching. Worth fixing
  together with the `timeStepMetrics` duplication bug above (Option 1 there
  restructures the same call sites), since both are about making solver 7's
  timing metrics trustworthy enough to actually use for the solver-6-vs-7
  comparison work already underway.

- [ ] **[HIGH] General (non-solver-6) memory ceiling on huge maps — blocks
  `scene_sp_endmaps` entirely.** Found 2026-08-14 while chasing an
  overnight-sweep OOM. `--scheduleModel 1` (which never builds or touches
  solver 6's reduced hierarchy — see `ai/project_context.md` "Solver 6") on
  `scene_sp_endmaps` (30.5M cells, the largest map ever run in this repo)
  peaks at **28.4GB RSS just loading the map and simulating one timestep**
  (`-s 1`), on a 31GB-RAM machine. Solver 6 adds its own hierarchy on top of
  that and goes over into an actual OOM-kill (confirmed via `dmesg`/
  `journalctl -k`, not inferred). This is **not** solver-6/thesis code —
  it's somewhere in the shared `lifelong` map-loading/simulation path
  (`Grid`, `SharedEnvironment`, task pool, or per-timestep planner setup),
  and has never been exercised at this map size before (`scene_mp_4p_03`,
  the previous largest at 13.9M cells / ~2.2x smaller, never came close to
  this on any prior sweep, including higher agent counts than tested here).
  Full bisection (ruled out hierarchy depth, ruled out `--hierarchyCache`
  serialization, isolated to "general, present even with solver 1") in
  `ai/auto_benchmarking_scene_sp_endmaps.md`.
  - **Next step**: profile `--scheduleModel 1 -s 1` on
    `instances/custom/scene_sp_endmaps/scene_sp_endmaps_10000.json` with
    `valgrind --tool=massif` (or similar) to find what's actually
    proportional to map size — needs a real profiler, not more guessing via
    `/usr/bin/time -v` peak-RSS bisection (which is how far this got
    tonight: confirms *that* it happens and roughly *where in the call
    stack* it doesn't happen — build-only via `dump_coarsening`'s
    lightweight loader is fine at 17.3GB even at hierarchy depth 8 — but not
    *which specific allocation*).
  - Until fixed, `scene_sp_endmaps` cannot be benchmarked at all on this
    machine, at any agent count, with any solver. Instances (10000/20000/
    40000/60000/90000 agents, shared 135000-task pool) already exist and are
    valid — only the hierarchy-build/simulation step is blocked.
  - A fix here would help every solver, not just solver 6 — worth
    prioritizing over solver-6-specific efficiency work if bigger maps are
    wanted again.

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
  - Keep `SchedulerSolveTime` / `SchedulerGuidePathTime` (their scopes were
    corrected 2026-10-01: the lift is now guide time, solver 1's flow walk
    is solve time; see `ai/hierarchical_guide_paths_plan.md`).
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

- [ ] **Coarse-flow all-or-nothing infeasibility when leftover tasks < leftover
  agents.** Found 2026-08-13 while explaining solver 6's per-timestep
  assignment behavior. `compute_reduced_assignment`'s Step 1 cross-node flow
  (`MapCoarsenV1.cpp:1450`, after same-coarse-node local matching pulls out
  what it can) builds the flow with **equality** supply/demand:
  `supply[source] = N` (leftover idle agents), `supply[sink] = -N`, and sink
  capacity out of each coarse node capped at that node's leftover task count
  (`MapCoarsenV1.cpp:1619-1651`). Since total supply sums to zero, LEMON's
  `NetworkSimplex` treats this as a hard equality, not a best-effort match:
  if leftover tasks system-wide are fewer than `N`, `ns.run()` returns
  non-`OPTIMAL` and the code immediately does `return assignments;`
  (`MapCoarsenV1.cpp:1687-1688`) — **zero** leftover agents get a new
  assignment that timestep, not just the `N - tasks` shortfall. Same-node
  local matches from Step 1 are unaffected (they never touch this flow).
  Self-healing (nothing is discarded, both pools are rebuilt fresh from
  `env->task_pool` next call), so not a stuck state — but likely bursty
  idle time in any task-scarce-relative-to-agents regime.
  - Solver 1 (`schedule_plan_flow`, `scheduler.cpp:750-751`) has the
    identical equality-supply structure, so this isn't solver-6-specific —
    but solver 6 is fast enough to actually reach this code path every
    timestep, while solver 1 often times out before it does.
  - Suspected live in the `scene_mp_4p_03` sweeps already run (10k/20k
    agents vs. only ~3-7k tasks completed over 500 timesteps, see
    `outputs/scene_mp_4p_03_10k_20k_localmatch_comparison/metrics.md`) — an
    agent-heavy, task-scarce regime that's exactly the trigger condition.
  - **No existing signal to check frequency**: `compute_reduced_assignment`
    prints nothing on the infeasible path today, so there's no way to tell
    from past logs/outputs how often this has actually fired. First step
    before deciding whether to fix: add a counter/print on the infeasible
    branch and re-run one of the large `scene_mp_4p_03` instances to see how
    often it triggers.
  - **Candidate fix** (standard unbalanced-transportation trick, not yet
    designed in detail): add one extra arc directly from source to sink with
    a penalty cost higher than any real coarse-graph path cost and capacity
    = `N`, so the equality constraint is always satisfiable — real
    assignments are still preferred wherever they exist, and only the
    genuinely-unmatchable surplus falls through to the penalty arc instead
    of blocking the whole batch. Scoped to Step 1's flow-graph construction
    only; doesn't touch local matching or Steps 3-4 (guide-path lifting).

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

- [x] **2026-08-26: Per-timestep task-completion count added to
  `timeStepMetrics`.** User asked whether per-timestep throughput (tasks
  completed each timestep, not just a cumulative/aggregate rate) was
  available anywhere — it wasn't: `TaskManager::check_finished_tasks()`
  already computed exactly which tasks finished each call
  (`src/TaskManager.cpp:128-160`) but the result was discarded at its only
  call site, `TaskManager::update_tasks` (`src/TaskManager.cpp:213-219`), and
  neither the JSON output nor `TimeStepMetric` recorded it anywhere.

  **Fix:** `update_tasks` now returns the finished-task count (its signature
  changed `void` -> `int`, `inc/TaskManager.h:27`, `src/TaskManager.cpp`).
  `TimeStepMetric` (`inc/CompetitionSystem.h`) gained two fields:
  `Timestep` (the real simulated timestep this entry corresponds to,
  `simulator.get_curr_timestep()`) and `TasksFinishedThisStep`. In
  `BaseSystem::simulate()` (`src/CompetitionSystem.cpp`), the `update_tasks()`
  call was moved to run *before* the `TimeStepMetric` push (was after) so the
  count lands on the same entry as the rest of that timestep's metrics; both
  new fields are set there and serialized into `timeStepMetrics` in
  `saveResults`.

  **Important caveat carried over from the `len(timeStepMetrics)` fix above:**
  because one `timeStepMetrics` entry can still represent several real
  elapsed timesteps collapsing together is no longer possible post-fix (one
  row per genuine decision, not one per elapsed timestep) — but `Timestep`
  can still jump by more than 1 between consecutive rows whenever a planner
  timeout forces catch-up ticks in between. So a per-timestep throughput
  curve must be plotted against each entry's `Timestep` value, not against
  array index or a naive `TasksFinishedThisStep / 1` per row.

  **Validated** on `instances/custom/tiny/tiny.json` (solver 1, 50
  timesteps): `sum(TasksFinishedThisStep)` across all 50 entries equals
  `numTaskFinished` exactly (4 = 4), and the 4 nonzero entries land on
  distinct, correct real timesteps (15, 20, 40, 41), confirming completions
  aren't smeared onto the wrong entry or double-counted.

  Not done as part of this pass (flagged to the user, not yet requested):
  a plotting/aggregation mode in `visualisation/compute_throughput_metrics.py`
  for per-timestep throughput curves, and a dedicated `ai/*.md` writeup.

- [x] **2026-08-26: `timeStepMetrics` duplicate-row bug fixed and validated**
  (was: "`len(timeStepMetrics)` ('steps') is not a real per-timestep count,
  and it's worse than previously documented"). Root cause (`BaseSystem::simulate`,
  `src/CompetitionSystem.cpp:148`):
  whenever `plan()` times out and forces `timeout_timesteps` catch-up ticks
  before the real move, the loop pushed **exactly 2** `TimeStepMetric` rows
  for that whole burst — one "catch-up" row (`:259-273`) and one "real move"
  row (`:291-305`) — both reading the *same* `last_scheduler_timing`, so
  they were byte-identical duplicates. When `timeout_timesteps == 0` (no
  timeout), only 1 row was pushed. So `len(timeStepMetrics)` was neither the
  number of real elapsed timesteps (`makespan` is) nor the number of genuine
  planning decisions (always 1 per outer-loop iteration, timeout or not) —
  it was an inconsistent mix that happened to equal 2x decisions on a
  timed-out iteration and 1x decisions on a clean one. Purely a logging
  artifact: `time_step_metrics` is write-only (nothing in `simulate()` reads
  it back to drive simulation state), so `makespan`/`numTaskFinished`/
  `tp_makespan` were all unaffected — confirmed by checking that the outer
  `while` loop is gated on `simulator.get_curr_timestep()`, never on this
  list's length.

  **Quantified 2026-08-21** across `outputs/solver_6_solver_7_comparison/`
  (`scene_sp_pol_06`, solvers 6 & 7, `--flowSolveLevel` 2/4/6/8, 10k/20k/60k
  agents): duplication rate tracked timeout frequency exactly, and was
  severe at shallow levels. At 60000 agents: level 2 had only **21 (solver
  7) / 78 (solver 6)** genuine planning decisions across the full ~500-tick
  run — meaning the fleet was idling in forced-wait for the large majority
  of simulated time. Level 6 had **0 of 250** decisions finish without a
  timeout. `orz900d_5000` (smaller/faster map) was much healthier by
  comparison — only 2 of 198 decisions duplicated. Also surfaced a
  possibly-thesis-relevant side finding: **solver 7 has substantially more
  timeouts than solver 6 at the same agent count/level at shallow levels**
  (21 vs. 78 real decisions at 60000/level2) — plausible cause is the
  edge-node backbone subdividing every coarse arc, which costs more in
  absolute terms when the graph is still large (shallow level); the gap
  mostly disappears by level 6/8. Worth a dedicated look now that the metric
  is trustworthy enough to quantify cleanly.

  **User decision (2026-08-26):** `len(timeStepMetrics)` should mean the
  genuine number of times the solver actually solved — i.e. one row per
  `plan()` call / outer-loop iteration, matching the "decisions" framing
  from the quantification above, not one row per real elapsed timestep.
  This ruled out an earlier-considered fix of padding to
  `len(timeStepMetrics) == makespan` with zeroed rows on catch-up ticks
  (that would have made "steps" track simulated time instead of solver
  activity — the opposite of what's wanted). One tradeoff noted and
  accepted: padding would have given a directly-computable "fraction of
  idle timesteps" signal (`zeroed rows / len`); under the fix actually
  taken, the equivalent signal is still recoverable as
  `makespan - len(timeStepMetrics)`, just not read off zeroed rows directly.

  **Implementation:** deleted the "catch-up" `TimeStepMetric`
  construction/push at the old `:259-273` inside the `timeout_timesteps > 0`
  branch — a pure deletion, nothing restructured. The unconditional push
  after the branch (old `:291-305`) is now the only place `time_step_metrics`
  gets appended, so every outer-loop iteration contributes exactly one row,
  regardless of whether it timed out. Also updated the stale comment at
  `:204-213` (previously explained why match-count totals were accumulated
  outside `time_step_metrics` *because* of the double-push; that reasoning no
  longer applies now that the push is single). Did not touch the separate,
  pre-existing `planner_times` double-push in the same branch (`:256-257`
  and `:289`) — that list only feeds the dead `kUseTimeStepMetricsOutput ==
  false` output branch today, so it's latent and out of scope for this fix.

  **Docs updated:** `ai/project_context.md`'s "Makespan vs. 'timesteps
  solved'" section and its "Other gotchas" bullet now describe the new,
  fixed semantics (steps = genuine decisions by design, diverges from
  makespan intentionally, not as a bug). `ai/auto_benchmarking.md` got a note
  flagging that its worked historical numbers (`orz900d` ~26%, `IH_mp_2p_01`
  up to 177.5 tp/steps) predate the fix and are up to 2x inflated in `steps`
  count versus a fresh run — don't compare old and new sweep `steps` columns
  directly. `visualisation/compute_throughput_metrics.py`'s docstring was
  rewritten, and `tp_steps` was re-enabled in both `compute_metrics` and
  `compute_metrics_indexed` (plus both CSV `fieldnames` lists) now that it's
  a meaningful "throughput per genuine decision" metric rather than a
  misleading artifact — `tp_makespan` remains the one to use for real-time
  throughput comparisons.

  **Validation:** `warehouseSmall_100`, solver 6, 200 timesteps — no
  timeouts occurred (solver 6 is fast on this map), so `makespan == 200 ==
  len(timeStepMetrics)` as expected (nothing to collapse when nothing times
  out). `orz900d_5000`, solver 1, 20 timesteps requested — hit 16 logged
  planner timeouts, `makespan` reached 21 (one tick of overshoot, the
  separately-documented pre-existing off-by-one) while `len(timeStepMetrics)`
  was 5 — a real, expected divergence (5 genuine decisions covering 21
  simulated timesteps). Checked all 5 rows pairwise for byte-identical
  duplicates: zero found, confirming the double-push is gone even under
  heavy, repeated timeout pressure (the exact condition that used to produce
  it).

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

- [x] **2026-08-20: New solver 7 (edge-node augmented coarse graph) implemented
  and validated.** New `map_reduction_test/EdgeAugmentedCoarsen.{h,cpp}`
  sibling module (solver 6's hierarchy, bridge caches, and Steps 3/4 lifting
  left untouched); one behavior-preserving extraction out of `MapCoarsenV1.cpp`
  (`ReducedHierarchy::lift_coarse_paths_to_fine`, verified byte-identical
  solver 6 output before/after). Found and fixed a real bug during initial
  testing: `NetworkSimplex::flowMap()` must be called after `run()`, not
  before (latent, harmless copy of the same mistake also sits in solver 6's
  own code, which never reads that map). Structural correctness confirmed
  with a new tool, `./build/edge_augmented_validator` (arc-count, degree,
  cost-reconstruction, and bounding-box invariants — the last cross-checked
  against an independent naive reference implementation) — 10/10 checks pass
  on `tiny` and on `orz900d` at every hierarchy level tested. Scale-tested on
  `orz900d_5000` (~978K cells, 5000 agents, 250 timesteps): completes cleanly
  at every level, comparable-or-better task throughput than solver 6, memory
  flat (non-leaking) throughout. Confirmed `--hierarchyCache` is fully shared
  with solver 6 (same `ReducedHierarchy::ensure()` call). Two non-blocking
  caveats found and documented: solve-time headroom shrinks at shallow
  `--flowSolveLevel` on huge maps (worth checking directly before trusting
  levels 1-2 on anything bigger than `orz900d`), and a ~870MB RSS plateau gap
  vs. solver 6 at level 4 specifically, likely (not yet directly proven)
  explained by the pre-existing ~1GB-capped `global_heuristictable` LRU cache
  responding to different per-agent routing decisions. Full design,
  implementation notes, and validation results: `ai/edge_node_representation.md`.
