# Project context for future Claude sessions

Read this file first. It exists so a fresh session doesn't have to re-read the
whole repo to understand what this is and how the pieces fit together. It
focuses on the path exercised by `./build/lifelong ... --scheduleModel 1` and
`--scheduleModel 6`, since that's been the active area of work.

> **`ai/human_notes/` is off-limits.** It holds standalone notes kept for the
> user's own reference, separate from the maintained doc set this file
> indexes. Do not read, list, or summarize files under `ai/human_notes/`
> unless the user explicitly asks you to open one by name in that specific
> conversation — don't proactively pull it into context otherwise.

## What this repo is

This is a fork of the **League of Robot Runners (LoRR)** "start-kit" —  a C++
benchmark harness for **lifelong Multi-Agent Pickup-and-Delivery (MAPD)**:
a fleet of agents on a 4-connected grid map repeatedly gets assigned tasks
(sequences of goal locations) and must plan collision-free paths to them,
forever, while new tasks keep streaming in. `README.md`,
`Evaluation_Environment.md`, `Input_Output_Format.md`,
`Prepare_Your_Submission.md` are the original upstream competition docs and
are still accurate for the unmodified parts of the system.

This is an **honours thesis project** (MAPD-Honours) built on top of that
start-kit. The thesis-specific work is a from-scratch **hierarchical
map-coarsening scheduler** (`map_reduction_test/MapCoarsenV1.*`, wired in as
`--scheduleModel 6`), intended to make the task-assignment scheduler scale to
much larger maps/agent counts than the stock min-cost-flow scheduler
(`--scheduleModel 1`) by solving assignment on a small coarsened graph instead
of the full fine map every timestep.

**Everything under `map_reduction_test/`, plus the solver-6 plumbing in
`default_planner/scheduler.cpp`, the LRU cache in
`default_planner/heuristics.{h,cpp}`, and the touch-LRU call site in
`default_planner/planner.cpp`, is thesis-authored code, not upstream
start-kit code.** Treat everything else under `src/`, `inc/`, and the rest of
`default_planner/` as the (mostly unmodified) upstream reference
implementation.

Other `ai/*.md` files document specific past investigations in detail (see
"Other docs in `ai/`" below) — this file is the map that tells you when to go
read them.

## Build & run

```shell
./compile.sh                 # cmake configure (Release) + build `lifelong` and `map_reduction_test`
./build/lifelong --inputFile <instance.json> -o outputs/out.json --scheduleModel <1-6>
./build/map_reduction_test <instance.json>   # standalone solver-1-vs-solver-6 comparison harness, see below
```

Key CLI flags (`src/driver.cpp`, `po::options_description`):
- `--inputFile,-i` (required), `--output,-o`
- `--scheduleModel,-m` (default 1): **1**=flow, 2=flow+history edge cost,
  3=matching+dijkstra, 4=matching+lazy heuristic, 5=greedy, **6**=reduced
  hierarchy (thesis scheduler; not in the help string, added later)
- `--simulationTime,-s` (default 5000 timesteps)
- `--planTimeLimit,-t` (ms per timestep, default 1000), `--preprocessTimeLimit,-p` (default 30000)
- `--hierarchyCache <path>` (default `""`, disabled): cache solver 6's map-coarsening
  hierarchy to disk and reuse it on later runs against the same map instead of
  rebuilding from scratch — see `ai/hierarchy_cache.md`
- `--flowSolveLevel` (default 2): solver 6 only — which already-built hierarchy
  level (0 = fine map) to solve the per-timestep flow assignment on, at
  runtime; out-of-range values fall back to the compile-time
  `kDefaultFlowSolveLevel`. Independent of `kDefaultCoarsenLevels` (how deep
  the hierarchy is *built*, still compile-time-only) — the hierarchy must be
  built at least as deep as the requested level. See
  `ai/auto_benchmarking_scene_mp_4p_03.md` for the first sweep using this.
- `--useTraffic,-u` (bool): enables congestion-aware guide-path costing in the scheduler
- `--assignNew,-n` (`new_only`): if true, only ever assign brand-new unassigned tasks (no swapping of already-assigned-but-unopened tasks)
- `--commitWindow,-w`

Example commands actually used during development (`instances/commands.txt`):
```shell
./build/lifelong --inputFile ./instances/warehouseSmall/warehouseSmall_100.json -o outputs/ws100_new.json --scheduleModel 1
./build/lifelong --inputFile ./instances/custom/orz900d/orz900d_5000.json -o outputs/o5000.json -s 250 --scheduleModel 6
./build/map_reduction_test instances/custom/random_10000.json
```

Instances live under `instances/` (`warehouseSmall`, `warehouseLarge`,
`sortationLarge`, `random`, `custom/orz900d` — a large 656x1491 maze map used
as the stress test for solver 6, `custom/IH_mp_2p_01` — a huge 1912x1800
maze/MAPF-benchmark map, `custom/warehouseXL` — a huge 1900x1800 *generated*
structured-warehouse map (pickup stations + storage racks, ~99.4% of
`IH_mp_2p_01`'s cell count) built via the `Benchmark-Archive` warehouse map
generator, see `ai/auto_benchmarking_warehouseXL.md`, `custom/tiny` — tiny
hand-built maps for unit testing). Each instance is a JSON pointing at a
`.map` file + agent/task files; see `Input_Output_Format.md` for the schema.

## End-to-end runtime flow (what happens each timestep)

```
src/driver.cpp: main()
  -> parses CLI, loads Grid + agents + tasks
  -> BaseSystem::simulate(simulationTime)   [src/CompetitionSystem.cpp]
       loop while curr_timestep < simulationTime:
         sync_shared_env()                  // push sim state into SharedEnvironment
         plan(timeout_timesteps)
           -> Entry::compute(time_limit, plan, proposed_schedule)   [src/Entry.cpp]
                1. scheduler->set_flow(planner->get_flow())
                2. scheduler->plan(time_limit, proposed_schedule)   // TaskScheduler::plan, src/TaskScheduler.cpp
                     dispatches on `solver` (1..6) to DefaultPlanner::schedule_plan_* [default_planner/scheduler.cpp]
                3. update_goal_locations(proposed_schedule)          // writes env->goal_locations from task_pool
                4. planner->plan(time_limit, plan)                  // DefaultPlanner::plan [default_planner/planner.cpp]
                     - inits per-goal heuristic tables (touch_heuristic_lru)
                     - builds/updates per-agent guide paths (traffic-flow LNS, Frank-Wolfe)
                     - runs causalPIBT per agent (priority order) -> collision-free next moves
         simulator.move(proposed_actions)   // advances world state, marks tasks complete
```

### Makespan vs. "timesteps solved" — two different counters (`BaseSystem::simulate`, `CompetitionSystem.cpp:147`)

These are computed from different variables and can diverge:

- **`makespan`** (output JSON field, `CompetitionSystem.cpp:271-283`) = the
  max, over all agents, of `solution_costs[a]`. `solution_costs[a]` is
  incremented once per **real elapsed simulated timestep** in which agent `a`
  had a non-empty `goal_locations[a]` — both in the normal path (`:195-199`,
  right before `simulator.move(proposed_actions)`) and in the catch-up path
  described below (`:170-178`). It tracks real simulated time.
- **"timesteps solved" / "steps recorded"** = `len(timeStepMetrics)`, one
  entry appended per *call into the planner*, not per elapsed timestep. The
  outer loop (`simulate()`) calls `plan(timeout_timesteps)` once per
  iteration; `plan()` (`:91-104`) itself loops internally, incrementing
  `timeout_timesteps` once for every additional simulated timestep it has to
  burn waiting for a still-running planner thread. Back in `simulate()`, that
  whole burst of `timeout_timesteps` forced real timestep advances
  (`simulator.move(all_wait_actions)` in a `for` loop, `:170-178`) gets
  **exactly one** `TimeStepMetric` appended for the entire batch (`:183-189`),
  then one more real move + one more metric entry happens for the "actual"
  planned step (`:194-213`). So one outer-loop iteration can advance the
  simulated clock by `timeout_timesteps + 1` real timesteps while only adding
  1-2 rows to `timeStepMetrics`.
- **Net effect**: whenever the planner is slow enough to time out and force
  multiple catch-up timesteps in a single call (solver 1 on `orz900d` does
  this constantly — see "planner timeout" log spam), `len(timeStepMetrics)`
  under-counts real elapsed time, while `makespan` does not. Dividing
  tasks-finished by `len(timeStepMetrics)` therefore **overstates** throughput
  for whichever solver times out more; dividing by `makespan` gives the
  real-time-accurate figure. `visualisation/compute_throughput_metrics.py`
  reports both (`tp/steps` and `tp/makespan`) — prefer `tp/makespan`. Full
  worked example with real numbers in `ai/auto_benchmarking.md`.
- Separately, `makespan` can overshoot the requested `-s N` by 1: `plan()`'s
  internal loop condition (`timestep + timeout_timesteps < simulation_time`)
  stops advancing `timeout_timesteps` once the cap is hit, but `simulate()`
  then unconditionally does one more real `simulator.move()` regardless
  (`:202`) — an off-by-one that only surfaces when the planner is still
  mid-solve exactly as time runs out.

`SharedEnvironment` (`inc/SharedEnv.h`) is the shared blackboard: `map`,
`curr_states` (agent locations), `task_pool` (task_id -> `Task`, see
`inc/Tasks.h`), `new_tasks`, `new_freeagents`, `curr_task_schedule`,
`goal_locations`. A `Task` is a list of `locations` (a multi-stop errand);
`idx_next_loc` tracks which stop is next; a task is "opened" once
`idx_next_loc > 0` (its first stop has been visited), at which point it's
pinned to `agent_assigned` and schedulers must not reassign it.

`TaskScheduler::plan` (`src/TaskScheduler.cpp:33`) is the dispatch point for
the `solver` int (`set_solver()`, driven by `--scheduleModel`):
```cpp
if (solver == 1)      schedule_plan_flow(...);        // full-map min-cost flow
else if (solver == 2) schedule_plan_flow_hist(...);
else if (solver == 3) schedule_plan_matching(...);
else if (solver == 4) schedule_plan_h(...);
else if (solver == 5) schedule_plan_raw(...);
else if (solver == 6) schedule_plan_flow_reduced(...); // thesis: coarsened hierarchy
```
All `schedule_plan_*` functions live in `default_planner/scheduler.cpp` and
share the same signature shape: given `env->task_pool` / `env->new_freeagents`,
produce `proposed_schedule` (agent_id -> task_id, or -1) and optionally
populate the global `agent_guide_path` map (agent_id -> fine-node path) when
`use_traffic && env->curr_timestep >= 100`.

## Solver 1 — `schedule_plan_flow` (`scheduler.cpp:660`)

The baseline / reference scheduler. Every timestep, from scratch:
1. Partitions tasks into already-opened (pinned to their agent) vs.
   "flexible" (unassigned, or assigned-but-not-yet-started when `new_only`
   is false — these get re-offered to the solver every timestep).
2. Builds a **LEMON `ListDigraph`** with **one node per fine map cell**
   (`env->map.size()` nodes), a source connected to each flexible agent's
   current location, a sink connected to each flexible task's location, and
   arcs between every pair of 4-adjacent walkable cells (cost 1, or a
   traffic-congestion-weighted cost if `--useTraffic`).
3. Solves one min-cost flow (`lemon::NetworkSimplex`) over that whole graph.
4. Recovers each agent's assigned task by following positive residual flow
   arcs from the agent's node until a task node is reached (a raw walk over
   the flow solution, not a stored path).

This is correct and simple but **doesn't scale**: the graph has one node per
map cell, so on `orz900d` (~978K walkable cells) solving from scratch every
timestep is extremely slow — slow enough that in practice it starves
`planner.cpp`'s per-timestep time budget before it ever gets to the
traffic-aware guide-path optimization step (see `ai/claude_memleak_fixes.md`,
"Remaining considerations"). It was not the subject of the OOM bugs (see
below) because it's simply too slow to reach the code paths that triggered
them.

## Solver 6 — `schedule_plan_flow_reduced` (`scheduler.cpp:920`) + `MapReductionTest::ReducedHierarchy`

The thesis contribution. Idea: instead of solving assignment on the full fine
map every timestep, build a **persistent multi-level coarsened graph once**
(at hierarchy-build time), solve the tiny top-level flow every timestep, and
only "lift" the result back down to fine-map paths when something actually
needs them (traffic-aware guide paths).

**Hierarchy build only runs for solver 6.** `schedule_initialize()`
(`scheduler.cpp:85`) takes the `solver` int (threaded through from
`TaskScheduler::initialize` via its `solver` member, which is set by
`driver.cpp` before `Entry::initialize`/preprocessing runs) and only calls
`ReducedHierarchy::instance().ensure(env)` when `solver == 6`. This used to
run unconditionally for every solver — solver 1 paid the build cost too even
though it never used the result (fast enough to be invisible on `orz900d`,
~978K cells, but ~300s on the ~2.3x bigger `IH_mp_2p_01`, which meant raising
`--preprocessTimeLimit` well past its 30000ms default for *every* solver in
a sweep, not just solver 6 — see `ai/auto_benchmarking.md`). Gating it on
`solver == 6` removes that cost for solvers 1-5; anything that sweeps
multiple solvers on a huge map no longer needs to inflate
`--preprocessTimeLimit` for the non-6 runs on the hierarchy's account (though
solver 1's own full-map flow solve can still be slow, see "Solver 1" above).

### Data structures (`map_reduction_test/MapCoarsenV1.h`)

- `CoarsenedGraph`: one level of the hierarchy. Wraps a `lemon::ListDigraph`
  plus: `to_coarser_node_id` / `to_finer_node_ids` (inter-level mappings),
  `chosen_finer_node_id` (one arbitrary representative fine node per coarse
  node), `bridge_cache` / `bridge_path_cache` (precomputed fine-level paths
  between adjacent coarse components' representative nodes, keyed by
  `(from_parent, to_parent)`), `internal_directional_arc_*` (aggregated
  arc-weight stats per cardinal direction, used to build coarse arc costs).
- `MultiLevelCoarsenedGraph`: `levels[0]` = fine map, `levels[1..]` =
  successively coarser (`Coarsen()` groups 2x2 blocks of the previous level's
  nodes into connected components, roughly halving grid dimensions each
  time). `kDefaultCoarsenLevels = 2` (`MapCoarsenV1.cpp:31`) → 3 total levels
  for this repo.
- `ReducedHierarchy`: process-lifetime singleton (`instance()`) owning one
  `MultiLevelCoarsenedGraph`. `ensure(env)` builds it once (checked via a
  signature hash) and is cheap to call every timestep after that. If
  `env->hierarchy_cache_path` is set (`--hierarchyCache <path>`), `ensure()`
  tries loading a previously-saved hierarchy from that file before rebuilding,
  and saves a fresh build there afterward — so a second run against the same
  map skips the build entirely. See `ai/hierarchy_cache.md`.

### Per-timestep flow: `compute_reduced_assignment` (`MapCoarsenV1.cpp:1280`)

1. **Step 1** — map every flexible agent's current location and every
   flexible task's location up to the top (coarsest) level, build a small
   `ListDigraph` over just the top-level nodes touched, solve one
   `NetworkSimplex` flow. This is the only per-timestep solve, and it's small
   (bounded by top-level graph size, not fine map size).
2. **Step 2** — recover one coarse-level path per agent from the residual
   flow (same walk-the-flow technique as solver 1, but over the tiny coarse
   graph).
3. **Early return**: if `need_guide_paths` is false (guide paths aren't going
   to be consumed this timestep — see below), return `assignments` here and
   skip steps 3-4 entirely (`MapCoarsenV1.cpp:1519`).
4. **Step 3** — "lift" each coarse path down through the levels to the fine
   map, splicing in the cached `bridge_path_cache` segments between
   neighboring coarse components at each level, down to a concrete
   start→task fine-node path per agent (`current_paths[i]`).
5. **Step 4** — hand `current_paths[i]` to the caller as the agent's guide
   path (a `std::list<int>` view), no further processing.

Caller side, `schedule_plan_flow_reduced` (`scheduler.cpp:920`):
- Tasks already assigned-but-not-yet-opened are **pinned** directly
  (`proposed_schedule[agent] = task`) instead of being re-offered to the
  coarse solver every timestep — see "Bug: reshuffling" below for why.
- `need_guide_paths = use_traffic && env->curr_timestep >= 100` — guide-path
  lifting only runs when traffic-aware guiding is actually enabled and past
  its warm-up window, matching solver 1's own gating.
- Falls back to `schedule_plan_flow` (solver 1) if the hierarchy isn't ready.

### Standalone comparison harness

`map_reduction_test/run.cpp` builds a separate executable
(`./build/map_reduction_test <instance.json>`, CMake target defined in
`CMakeLists.txt:114`) that loads an instance, builds the fine graph +
hierarchy directly (bypassing the full simulation loop), and is used for
quick solver-1-vs-solver-6 experimentation without running the full
`lifelong` binary. Also home to `LGFtoGEXF.py` / `convertLGFtoCSV.py` /
`convertLGFtoDOT.py` (visualization/export of the coarsened graph structure)
and a `visualisation`/`visualisation_csv` output directory.

## Low-level planner (shared by all schedulers): `default_planner/`

Once a scheduler has produced `proposed_schedule` and `Entry::update_goal_locations`
has written `env->goal_locations`, `DefaultPlanner::plan()` (`planner.cpp:119`)
runs regardless of which scheduler solver was used:
1. For every agent's goal location, ensure a `HeuristicTable` exists
   (`trajLNS.heuristics[goal_loc]`, a full-map BFS distance table from that
   goal, `init_heuristic()`), then `touch_heuristic_lru(goal_loc, env)`.
   `trajLNS.heuristics` is a reference alias for the global
   `global_heuristictable` (`TrajLNS():heuristics(global_heuristictable)`,
   `TrajLNS.h:92`) — **every session/agent shares the same cache**, keyed by
   goal location, and it's expensive (`env->map.size() * sizeof(int)` per
   table, so ~3.9MB on a 978K-cell map).
2. For agents whose guide path is stale/missing, recompute it
   (`update_traj`) or adopt the scheduler-provided `agent_guide_path[i]` if
   present (only populated when `use_traffic` and past timestep 100).
3. `frank_wolfe(trajLNS, updated, end_time)` iteratively adjusts guide paths
   to reduce predicted congestion (traffic-flow LNS), time-boxed by
   `end_time` (computed from `time_limit` minus a `PIBT_RUNTIME_PER_100_AGENTS`
   reserve and `TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE`, both in
   `const.h`).
4. Agents are priority-sorted (`p[]`, boosted on new-goal-assignment and on
   dead-end tiles) and `causalPIBT()` (`pibt.cpp`) is run per agent in
   priority order to pick a collision-free next move following the guide
   path.
5. Next-state assignments are converted to `Action`s (`getAction`) —
   turning-in-place is modeled as a delay layered on top of the base plan,
   not a separate action cost.

### Guide paths: which solvers provide them, and are they doing anything

`agent_guide_path` (`scheduler.cpp:13`, a global `unordered_map<int,list<int>>`,
agent id -> fine-node path) is how a scheduler can hand the low-level planner
a ready-made path instead of making it compute one. **Both solver 1 and
solver 6 populate it under the exact same gate**,
`use_traffic && env->curr_timestep >= 100`:
- Solver 1 (`schedule_plan_flow`, `scheduler.cpp:871-872`): the per-agent
  `path` it builds by walking the flow solution (needed anyway, to recover
  which task each agent was assigned) is stored into `agent_guide_path` only
  if that gate is true. The walk itself always happens regardless — it's free.
- Solver 6 (`schedule_plan_flow_reduced`, `scheduler.cpp:991-1000`): computes
  `need_guide_paths = use_traffic && curr_timestep >= 100` and passes it into
  `compute_reduced_assignment`, which **skips Steps 3-4 (coarse-to-fine
  lifting) entirely** when false (early return after Step 2, see "Solver 6"
  above). Unlike solver 1, here the gate isn't just "don't store it" — the
  fine-grained path is never computed at all when traffic is off.

Consequence: **none of the benchmarking done in this repo before the
guide-path metric work below (`ai/claude_memleak_fixes.md` verification runs,
the `orz900d` sweep in `ai/auto_benchmarking.md`) ever passed
`--useTraffic`**, so `agent_guide_path` was empty in every one of those runs
for both solvers, and solver 6's Step 3-4 lifting code specifically never
executed. "Solver 6's guide-path code is barely ever active" was correct as
stated for those runs — it was *never* active, by construction, not just
rare.

**This changed with `ai/guide_path_metric.md`**: solver 6's Step 3-4 lifting
now runs unconditionally every timestep (decoupled from the
`use_traffic && curr_timestep >= 100` gate) so a new `GuidePathLengthSum`/
`GuidePathCostSum` metric can be populated on every run, including ones
without `--useTraffic`. What still respects the original gate is only
*storage into `agent_guide_path`* (the planner-visible seed) — so the
planner-behavior consequence described above (no seed ever reaches the
planner without `--useTraffic`) is still accurate, but the lifting code
itself is no longer dormant outside that window. That change is also what
first exercised Step 3-4 rigorously enough to surface a real bug (wrong
start/end location when a coarse parent spans multiple disconnected
sub-components at an intermediate level) — see `ai/guide_path_metric.md` for
the bug, fix, and a new standalone validator tool
(`./build/guide_path_validator`) that checks solver 1 and solver 6 guide
paths are format-interchangeable.

What happens when `agent_guide_path` has no entry for an agent:
`planner.cpp:227-237` falls back to `update_traj()` (`flow.cpp:162`), which
runs `astar(..., lns.flow, lns.heuristics[goal], ...)` — `lns.flow` is the
low-level planner's **own** internal congestion field (built from other
agents' current trajectories, `get_opened_flow()`, `planner.cpp:33-60`),
independent of the scheduler entirely. So the low-level planner always does
*some* congestion-aware pathing via `update_traj`/`frank_wolfe`
(`flow.cpp:96`) regardless of scheduler solver choice — a scheduler-provided
guide path is only ever a substitute **seed** trajectory (skips the initial
`astar` call, trajectory then still gets iterated on by `frank_wolfe` like
any other agent's), not the only source of congestion-awareness.

Whether that seed is worth anything depends on whether the scheduler's own
edge costs were traffic-weighted when it computed the seed — which is the
same `use_traffic` flag. `heuristics.{h,cpp}`'s distance tables are plain BFS
(topological, unweighted) regardless of `use_traffic`. So: with
`--useTraffic` off (every run so far), there is no dynamically-weighted-edge
information anywhere in the system, and the question of whether solver-
provided guide paths are "useful" doesn't really arise — they're simply never
computed. To exercise/benchmark this feature at all requires
`--useTraffic` and simulating past timestep 100; no code change is needed to
turn it on, only a CLI flag, but Steps 3-4 in solver 6 have never been
exercised under real traffic conditions since the OOM fixes landed (they were
all validated with traffic off) and would be worth a dedicated small-scale
correctness/perf check before trusting them at scale.

`heuristics.{h,cpp}` also owns the **LRU eviction layer** around
`global_heuristictable`: `touch_heuristic_lru()` bounds resident heuristic
tables to a fixed ~1GB budget (`max_tables = max(16, 1GB / bytes_per_table)`),
evicting the least-recently-touched table's `htable`/`open` (freeing its
vector) when the cache exceeds budget. This must be called from **every**
place that ensures a heuristic table exists — both `get_h()` (the "normal"
path, used by e.g. matching-based schedulers) and the direct call site in
`planner.cpp`'s main loop — or the cache silently un-bounds itself again for
whichever call site is missed.

## Memory-leak fix history (now committed)

The fixes below were committed as `b3806cd` ("fix memory leak, set up claude")
and the follow-up readability pass as `97b4a2c` ("clean up mapcoarsen and
other files") — both are on `main` as of this writing. `git log --oneline`
also shows a benchmark sweep (`88ef2a1`, `978af90`) and a bootstrap script
(`722c772`) added afterward; see "Benchmark sweeps" below for what those
benchmarks found. `ai/claude_memleak_fixes.md` remains the
authoritative, detailed writeup of the fixes themselves; short version:

Solver 6 was OOMing on large instances (`orz900d_5000`, 656x1491,
~978K cells) while solver 1 wasn't. Five bugs, found across two rounds:
1. Bridge-path lifting did exact-equality endpoint checks against the
   arbitrary component representative node, almost always failing and
   falling back to whole-map Dijkstra for ~40% of agents/timestep. Fixed by
   splicing bounded intra-component lead-in/lead-out hops instead of failing.
2. Guide-path reconstruction re-derived paths by following shared
   per-arc flow counts with no cycle guard/length cap → pathologically long
   paths. Fixed by reusing the already-bounded path built in Step 3 directly.
3. `build_cached_bridge_path_local`'s one-time BFS (during hierarchy
   construction) allocated full-fine-map-sized vectors (`~978K` ints) per
   call despite only ever touching ≤4-node components — tens of thousands of
   calls made hierarchy *construction* itself balloon to 10+GB transiently.
   Fixed with bounded hash-map state instead.
4. Guide-path lifting (Steps 3-4) ran unconditionally even when nobody
   would read the result (`use_traffic` off, or before timestep 100). Fixed
   with a `need_guide_paths` early-return.
5. **Root cause of the actual OOM**: `global_heuristictable` (see above) is
   never evicted by default — every distinct goal location ever seen builds
   and permanently keeps a ~3.9MB table. Solver 6's speed (vs. solver 1's
   comparative slowness) meant it was the first scheduler fast enough to
   actually route many agents to many distinct locations within the
   per-timestep time budget, exposing ~600 new tables/timestep (~2.3GB/timestep).
   Fixed with the LRU eviction layer described above.

Plus one independent stability fix scoped to solver 6 only: pin
already-assigned-but-unopened tasks instead of re-offering them to the coarse
flow solver every timestep (the coarse solve is blind to fine-grained
distance differences within a coarse node, so re-deciding caused near-random
reassignment churn, each reshuffle feeding bug 5's cache growth).

Verified: `orz900d_5000`, `--scheduleModel 6`, no `--useTraffic`, 250
timesteps — went from OOM-killed at timestep ~15-20 (RSS ~15.7GB) to
completing all 250 timesteps with RSS flat at ~2.3GB.

**If asked to continue this work**: check `git status`/`git diff` first to
see if these changes have since been committed or altered further — this doc
and `ai/claude_memleak_fixes.md` describe the state as of the fixes being
written, not necessarily current HEAD.

### Working-tree cleanup pass (readability only, no behavior change)

After the bug fixes above, a follow-up pass cleaned up and commented the
solver-6-related code (everything in `map_reduction_test/`, the entirety of
`schedule_plan_flow_reduced` in `scheduler.cpp`, and the LRU cache additions
in `heuristics.{h,cpp}`/`planner.cpp`). All changes were verified
behavior-preserving (both targets rebuilt cleanly; solver 6 re-run on
`warehouseSmall_100` and the `map_reduction_test` benchmark harness both
still produce correct output). Notable removals, mostly in
`MapCoarsenV1.cpp`:
- Three fully-unused functions left over from the Bug 2/round-2 fixes:
  `pick_bridge_arc_between_parents_local`, `reconstruct_guide_from_arc_flow_local`
  (the old, buggy, uncapped flow-walk guide reconstruction that Step
  4 replaced but never deleted), and `dump_connected_components` (only ever
  called from a commented-out line).
- A dead block in `ReducedHierarchy::ensure()` that computed `rows`/`cols`
  down to 1 in a loop but never used the result.
- Several "diary of the debugging session" comments (e.g.
  `??!?!??!???`, `FORCE SYSTEM PURGE OF LOCAL CONTAINERS`, numbered
  `1. CHANGE:` edit-instructions) rewritten as normal explanatory comments.
- In `map_reduction_test/run.cpp`, the ~50-line instance-loading logic
  duplicated between `load_fine_graph_from_input` and `run_benchmark` was
  extracted into one shared `populate_env_from_instance` helper.

This shifted line numbers in `MapCoarsenV1.cpp` by roughly -160 (it's ~160
lines shorter); the references elsewhere in this doc have been updated to
match. `heuristics.{h,cpp}` and `mapReductionV0.*` needed no changes — they
were already clean.

## Benchmark sweeps (`ai/auto_benchmarking.md`)

`ai/auto_benchmarking.md` is the index/synthesis doc for all solver-1-vs-
solver-6 sweeps — read it first, then follow its links to the per-sweep
detail file you need. Three sweeps so far:

1. `orz900d` (~978K cells), 15 runs, solver 1 vs. solver 6 at coarsen levels
   1-4 — detail in `ai/auto_benchmarking_orz900d.md`. **Solver 1 wins** on
   real-time throughput at every agent count tested.
2. `IH_mp_2p_01` (~3.44M cells, ~2.3x bigger), 9 runs, solver 1 vs. solver 6
   at coarsen depth 2 and 4 — detail in `ai/auto_benchmarking_IH_mp_2p_01.md`.
   **The result inverts**: solver 6 wins by ~4-7x instead, since solver 1's
   cost scales with map size (not agent count) while solver 6's doesn't.
3. `warehouseXL` (~3.42M cells, ~99.4% of `IH_mp_2p_01`'s size, but a
   *generated, structured* warehouse layout rather than a maze/MAPF-
   benchmark map), 15 runs, solver 1 vs. solver 6 at coarsen levels 1-4 —
   detail in `ai/auto_benchmarking_warehouseXL.md`. Solver 6 still wins at
   every agent count, but the margin is much narrower (~1.5-4.3x, shrinking
   as agent count grows) than `IH_mp_2p_01`'s roughly-constant 4-7x at
   nearly the same cell count — first evidence that map *topology*, not just
   size, affects the crossover (unconfirmed — see the detail file's
   parallel-execution caveat).

Headline pitfalls documented there, worth knowing before touching this again:

- **Solver 1 has its own unbounded-memory-growth bug**, distinct from the
  solver-6 OOM this doc's fixes addressed. It's invisible at the default
  `--planTimeLimit 1000`ms (the tight budget cuts the per-timestep flow solve
  off before the leaking code path completes), but OOM-kills the process
  within ~12 timesteps at `--planTimeLimit 10000`. Never fixed — out of scope
  for that sweep. Anyone raising solver 1's time limit should know this first.
- `makespan` and `len(timeStepMetrics)` ("steps recorded") are **not the same
  number** and can diverge badly — see "Makespan vs. 'timesteps solved'"
  above for the mechanics; always use `tp/makespan`, never `tp/steps`, for
  throughput comparisons (`visualisation/compute_throughput_metrics.py`
  reports both from a result JSON or folder of them).
- The coarsen-level/depth toggle (`kDefaultCoarsenLevels`, a `constexpr int`
  at `MapCoarsenV1.cpp:31`) is compile-time only — there's no CLI flag, so
  comparing levels means editing that line and re-running `./compile.sh` per
  level (both sweeps did this with `sed`, restoring the original value `2`
  at the end).
- Hierarchy build now only happens for solver 6 (see "Solver 6" above) —
  older sweep notes/scripts that inflate `--preprocessTimeLimit` for
  solver-1-only runs on huge maps on the hierarchy's account are no longer
  necessary, though solver 1's own preprocessing/per-timestep cost on huge
  maps can still warrant it independently.

## Known remaining gaps (from `ai/claude_memleak_fixes.md`, not yet acted on)

- The ~1GB LRU cap is a hardcoded constant, not derived from any overall
  memory budget/CLI flag.
- Solver 1 is slow enough on huge maps (full ~978K-node flow every timestep)
  that it may never reach `planner.cpp`'s traffic-aware guide-path
  optimization within its time budget — meaning solver-1-vs-solver-6 quality
  comparisons on huge maps may not be apples-to-apples on path quality, only
  on assignment throughput.

## Other docs in `ai/`

- `ai/claude_memleak_fixes.md` — full narrative of the solver-6 OOM
  investigation above; read it if you need the exact bug mechanics, code
  diffs, or verification methodology rather than the summary here.
- `ai/auto_benchmarking.md` — **index + cross-sweep synthesis + reusable
  methodology** for all solver-1-vs-solver-6 sweeps. Start here, then follow
  its links to the sweep you need:
  - `ai/auto_benchmarking_orz900d.md` — 15-run sweep, solver 1 vs. solver 6
    at coarsen levels 1-4, on `orz900d`.
  - `ai/auto_benchmarking_IH_mp_2p_01.md` — 9-run sweep, solver 1 vs. solver
    6 at coarsen depth 2/4, on `IH_mp_2p_01` (~2.3x bigger map).
  - `ai/auto_benchmarking_warehouseXL.md` — 15-run sweep, solver 1 vs. solver
    6 at coarsen levels 1-4, on `warehouseXL` (a generated, structured
    warehouse map ~the same cell count as `IH_mp_2p_01`).
  - `ai/auto_benchmarking_scene_mp_4p_03.md` — 5-run sweep (5000 agents only
    so far), solver 1 vs. solver 6 at `--flowSolveLevel` 1-4, on
    `scene_mp_4p_03` (~13.9M cells, ~6.3M walkable — the largest map
    benchmarked so far, ~2.8x `IH_mp_2p_01`'s walkable-cell count). First
    sweep using the new `--flowSolveLevel` CLI lever + `--hierarchyCache`
    together (build once, compare levels without recompiling). Widest
    solver-6-wins margin yet (~44-93x), and the first sweep where deeper
    coarsening *increases* throughput instead of costing a little.
  - Each new sweep gets its own `ai/auto_benchmarking_<map>.md` detail file
    (flat, not a subfolder — the user wants to eventually automate
    benchmarking rather than keep hand-writing these); update the master's
    index table and synthesis section to match.
- `ai/todo.md` — forward-looking task list (distinct from the other docs,
  which record completed investigations). Check it at the start of a
  session; add to it instead of losing track of asks that aren't done yet.
- `ai/guide_path_metric.md` — rigor pass on solver 6's guide-path
  reconstruction (found + fixed a real wrong-endpoint bug), the new
  `GuidePathLengthSum`/`GuidePathCostSum` `TimeStepMetric` fields, the new
  `./build/guide_path_validator` tool, and an important caveat: raw
  cross-solver totals of this metric are only comparable with `--assignNew 1`.
- `ai/guide_path_visualisation.md` — two new standalone tools
  (`./build/dump_guide_paths`, `map_reduction_test/visualisation/plot_guide_paths.py`)
  that render solver 1 vs. solver 6 guide paths spatially over the actual
  map, per-agent and whole-map. Also documents a real bug found and fixed
  while scaling this up: solver 6's coarse-to-fine lift fallback
  (`shortest_path_in_graph_local`) was plain, unguided Dijkstra with a fixed
  expansion cap, which silently dropped guide paths for many agents on huge
  maps with long paths — fixed with an A* heuristic. Includes a results
  table across every run done (`orz900d`, `IH_mp_2p_01` at 20 through 20000
  agents). Also documents a second, later tool built on a different
  mechanism: `BaseSystem::kDumpPerTimestepPaths` (`inc/CompetitionSystem.h`,
  off by default) streams scheduler guide paths + every agent's position to
  CSV from a real, live `./build/lifelong` run (not a frozen snapshot), and
  `map_reduction_test/visualisation/plot_timestep_frames.py` renders one PNG
  per timestep from the result.
- `ai/solver6_preprocessing_efficiency.md` — running list of CPU-time and
  memory efficiency findings in solver 6's one-time hierarchy build
  (`ReducedHierarchy::ensure()` → `build_multilevel_from_environment` →
  `Coarsen()`), not the per-timestep `compute_reduced_assignment` path.
  Tracks each finding's status (found vs. fixed) so it survives across
  sessions. Headline finding so far: `collect_connected_components` and
  `collect_internal_directional_arc_samples` both allocate a
  full-fine-map-sized `std::vector<char>` on every 2x2-block/component call
  during `Coarsen()` — the same anti-pattern already fixed once elsewhere in
  this file (`build_cached_bridge_path_local`) but missed in these two,
  and plausibly the dominant cost of hierarchy build time on large maps.
- `ai/coarsening_visualisation.md` — two new standalone tools
  (`./build/dump_coarsening`, `map_reduction_test/visualisation/plot_coarsening.py`)
  that render the map-coarsening hierarchy itself (which fine cells group
  into which coarse node at each level, plus the resulting coarse graph
  topology) spatially over the actual map — distinct from
  `ai/guide_path_visualisation.md`'s tooling, which visualises solver guide
  paths computed on top of an already-built hierarchy, not the coarsening
  structure itself. Includes a `--crop` mode for inspecting coarsening
  detail on large/maze-like maps where a full-map render is too
  fine-grained to read.
- `ai/hierarchy_cache.md` — the `--hierarchyCache <path>` disk cache for
  solver 6's map-coarsening hierarchy (`MapCoarsenSerialize.{h,cpp}`): format,
  cache-validity/invalidation rules, what's deliberately not serialized (and
  why that's safe), the new `./build/hierarchy_cache_validator` structural
  round-trip checker, and build-vs-load timing on `orz900d`/`IH_mp_2p_01`.
- `ai/local_node_matching.md` — **local node matching**: fixes solver 6's
  within-coarse-node agent<->task pairing being arbitrary/insertion-order-
  dependent instead of distance-informed. New `LocalNodeMatch.{h,cpp}`
  (`match_local_node_exact`, exact real-distance bipartite matching), wired
  into `compute_reduced_assignment`'s Step 1, toggleable via
  `kEnableLocalNodeMatching` (`MapCoarsenV1.cpp`). Validated via the
  `analyze_coarse_collisions` diagnostic and full `scene_mp_4p_03` sweeps at
  5000/10000/20000 agents — benefit grows with agent count, up to +7.2% tasks
  finished at level 6/20000 agents, and fixes a "deeper coarsening starts
  hurting" regression at that level.

(Update this list if more `ai/*.md` files are added later.)

## Repo map (quick reference)

```
src/                    upstream competition harness (mostly unmodified)
  driver.cpp              CLI entry point / main()
  Entry.cpp               per-timestep glue: scheduler -> goal_locations -> planner
  TaskScheduler.cpp        solver dispatch (1-6) -> DefaultPlanner::schedule_plan_*
  CompetitionSystem.cpp    BaseSystem::simulate() main loop, metrics, results output
  MAPFPlanner.cpp, Simulator.cpp, ActionModel.cpp, Grid.cpp, TaskManager.cpp, Evaluation.cpp, ...
inc/                     headers for the above + SharedEnvironment, Task, nlohmann json
default_planner/         the "default" reference planner+scheduler implementation
  scheduler.cpp/.h         schedule_plan_flow (1), _hist (2), _matching (3), _h (4), _raw (5), _flow_reduced (6, thesis)
  planner.cpp/.h           low-level PIBT-based multi-agent path planner (shared by all solvers)
  pibt.cpp/.h              causalPIBT collision-avoidance move selection
  heuristics.cpp/.h        global_heuristictable (BFS distance-to-goal cache) + LRU eviction (thesis addition)
  flow.cpp/.h              Frank-Wolfe traffic-flow guide-path optimization
  TrajLNS.h                per-agent trajectory / heuristic state bundle
  utils.*, heap.h, search*.h, Memory.h, Types.h, const.h   supporting utilities/types/tunables
map_reduction_test/      thesis-authored map-coarsening scheduler (solver 6)
  MapCoarsenV1.cpp/.h       CoarsenedGraph, Coarsen(), ReducedHierarchy, compute_reduced_assignment
  MapCoarsenSerialize.cpp/.h  save/load a MultiLevelCoarsenedGraph to/from disk (--hierarchyCache),
                            see ai/hierarchy_cache.md
  mapReductionV0.cpp/.h     earlier/simpler map-reduction prototype (superseded by V1, still compiled in)
  run.cpp                   standalone comparison-harness executable (./build/map_reduction_test)
  instance_loader.cpp/.h    shared instance-JSON loading helper (used by run.cpp, validate_guide_paths.cpp,
                            and dump_guide_paths.cpp)
  validate_guide_paths.cpp  standalone guide-path format/validity checker (./build/guide_path_validator),
                            see ai/guide_path_metric.md
  validate_hierarchy_cache.cpp  standalone hierarchy-cache round-trip checker (./build/hierarchy_cache_validator),
                            see ai/hierarchy_cache.md
  dump_guide_paths.cpp      standalone tool dumping solver 1 vs. solver 6 guide paths to CSV for
                            visualisation (./build/dump_guide_paths), see ai/guide_path_visualisation.md
  dump_coarsening.cpp       standalone tool dumping the map-coarsening hierarchy itself (per-level
                            fine-cell partitions, coarse node positions, coarse graph arcs) to CSV
                            for visualisation (./build/dump_coarsening), see ai/coarsening_visualisation.md
  LGFtoGEXF.py, convertLGFtoCSV.py, convertLGFtoDOT.py   graph export/visualization scripts
  visualisation/            .gexf outputs from the above scripts, plus plot_guide_paths.py
                            (renders dump_guide_paths.cpp's CSV output over the map, see
                            ai/guide_path_visualisation.md) and plot_coarsening.py (renders
                            dump_coarsening.cpp's CSV output over the map, see
                            ai/coarsening_visualisation.md) -- scripts living alongside output
                            artifacts in the same dir, not a clean separation
  visualisation_csv/        CSV output dir for LGFtoGEXF.py etc.
python/                 Python bindings track (pybind11) — separate from the C++ path above; not
                         touched by the solver 1/6 work. set_track.bash selects planner/scheduler/combined.
instances/               benchmark maps+agents+tasks (warehouseSmall/Large, sortationLarge, random,
                         custom/orz900d [656x1491 maze, main stress-test map], custom/tiny)
outputs/                 saved run outputs (JSON), some checked in as test/reference artifacts.
                         Organized per-map/per-purpose (`<map>_guide_paths/`, `<map>_solver_comparison/`,
                         `<map>_per_timestep/`, `coarsening_viz/`); stray/legacy one-off result files
                         not referenced by any doc live in `outputs/old/`.
ai/                      this file + other investigation writeups (claude_memleak_fixes.md)
CMakeLists.txt            build config; note map_reduction_test sources are folded into the main
                          `lifelong` target too (so scheduler.cpp can call into MapCoarsenV1)
compile.sh                cmake configure + build both `lifelong` and `map_reduction_test` targets
```
