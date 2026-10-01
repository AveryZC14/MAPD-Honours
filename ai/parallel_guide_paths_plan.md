# Plan: build guide paths in parallel

Status: **implemented 2026-09-29/30 behind flags; default is sequential
(`GUIDE_PATH_THREADS = 1`).** No setting wins on every map: sequential
congestion-aware paths are best on orz900d, parallel paths that ignore
congestion are the only thing that works on IH, and nothing works yet on
scene_mp_4p_03 (plain A* is too slow there; it needs a better heuristic).
Decision needed from the user; see "Conclusions" at the end. Follows on from
`ai/planner_local_bfs_plan.md` (the planner fix this builds on) and the
2026-09-29 entries in `ai/run_log.md` (all numbers below come from there).

## The overarching problem

The final thesis sweep (`instances/thesis_benchmarks/README.md`, "Final run
plan") can't run until the low-level planner works at 10k-80k agents on
large maps. Three findings so far, in order:

1. **Pure Manhattan freezes.** With `USE_MANHATTAN_HEURISTIC = true`, PIBT
   scored each move by Manhattan distance to the goal. Agents with a wall
   between them and their goal waited forever. The first 4 sweep runs
   (orz900d 10k) stopped delivering at about step 1,000. Those runs are in
   `outputs/thesis_sweep_junk/2026-09-29_manhattan_freeze/`.
2. **The local path BFS fixes the freeze where agents have guide paths.**
   `USE_LOCAL_PATH_BFS` scores moves by a small BFS from the agent to its own
   guide path (distance to the path plus steps left along it). On orz900d
   10k, 1,500 steps: 5,495 deliveries vs. 617 for pure Manhattan, stuck
   agents under 10.
3. **On IH_mp_2p_01 it isn't enough, because most agents have no guide
   path.** IH 10k, 1,500 steps: 2,364 deliveries (pure Manhattan 1,178),
   but 6,740 agents still had no path at decision 1,000, 4,661 were stuck,
   and only 1,021 decisions happened in 1,500 steps.

The stuck-agent diagnosis (IH 10k, 500 steps, new planner) showed that
**every stuck agent was an agent without a guide path**, steering by
Manhattan distance: 1,915 trapped by the score at a wall, 1,146 blocked
with a better move available (most likely by other agents; not proven),
and 0 stuck while using the path BFS. A separate check against the map file
confirmed every "trapped" label. Pictures:
`outputs/planner_stuck_diagnosis/stuck_ih_new.png`.

## The specific problem: not enough guide paths per step

Guide paths are built per agent and kept across steps until the agent's
goal changes. They're built in stage 2 of `DefaultPlanner::plan()`
(`default_planner/planner.cpp`), one A* search at a time, until the
per-step deadline (about 850 ms, minus scheduler time).

**Supply.** One A* search (`default_planner/search.cpp`) takes about 12 ms
on orz900d but about 70-300 ms on IH: IH is a 1,912 × 1,800 maze with paths
of about 1,000-1,250 cells, Manhattan distance is a poor guide around that
many walls, and the congestion penalties in A*'s ordering
(`search_node.h:116`) make it explore further. So IH gets about 14 paths
per step, orz900d about 60-70.

**Demand.** Every agent needs a path to its pickup at the start. Nearly
every agent reaches its pickup within the first 100 steps (10,009 pickups
by step 100 on IH), and each then needs a new path to its delivery. After
that, each delivery and new task adds one more.

**Result.** On orz900d the backlog clears by about step 250 and the planner
keeps up after that. On IH, at 14 per step, the backlog of about 10,000
would take about 700 steps to clear while new requests keep arriving.
Agents waiting in the queue use Manhattan distance and get trapped.

A second, smaller problem: a single A* search can't be interrupted, so a
long one runs past the step's deadline. That's why IH only got about
1,020 decisions in 1,500 steps (pure Manhattan overruns the same way).

## Why the searches are sequential today

1. **The congestion map `lns.flow`.** Each A* search reads it. As soon as a
   path is found, `add_traj()` adds it, so the next agent in the same step
   routes around it. This is deliberate, and it's the only real dependency.
2. **One shared search memory `lns.mem`** (`MemoryPool`, one node per map
   cell, reused by every search). Two searches at once would overwrite each
   other.
3. **`rand()` in A*'s tie-breaking** (`search_node.h:92`, `:122`). glibc's
   `rand()` is thread-safe but takes a shared lock on every call.
4. **Small shared counters** (`lns.soc`, `fw_metrics[i].last_replan_t`),
   updated in `add_traj()`.

Everything else A* touches is per agent (`trajs[i]`, `goal_nodes[i]`,
`path_togo[i]`) or read-only. Manhattan mode reads no shared heuristic
tables.

## Proposed solution

### Setting

`GUIDE_PATH_THREADS` in `default_planner/const.h`, overridable at build time
with `-DPLANNER_GUIDE_PATH_THREADS=<n>` like the other planner flags.

- `1`: the current sequential loop, unchanged.
- `>1`: the parallel version below. Planned default for the sweep: 6 (the
  machine has 8 cores; leaves room for the simulator thread and the OS).

### Each step, in stage 2

1. **Main thread, before searching.** Build the list of agents needing a
   path, in the rotated order (starting from `guide_path_start`). Handle
   scheduler-provided paths here, as now (cheap, and only when
   `PASS_SCHEDULER_PATHS_TO_PLANNER` is on).
2. **Parallel search.** Start the worker threads. Each takes the next agent
   from a shared atomic counter and runs A* against `lns.flow` as it was at
   the start of stage 2. Workers only read `lns.flow`, so no locks are
   needed. Each result (new path and goal node) goes into a per-agent
   buffer; workers don't touch `trajs[i]` or `lns.flow`. A worker stops
   taking new agents once the deadline passes; a search already running
   finishes, as today.
3. **Main thread, commit.** After joining, go through finished agents in
   list order: remove the agent's old path from `lns.flow`, store the new
   one, add it to `lns.flow`, build its `togo` lookup. Removing the old path
   at commit time (not up front) means an agent that didn't get a turn
   keeps its old path, and a path is never taken out of `lns.flow` twice.
4. **Next step.** The rotation starts from the first agent that didn't get a
   turn.

### Shared state changes

- **Search memory:** one `MemoryPool` per thread, created once and reused.
  A search node is about 56 bytes, so a pool is about 55 MB on orz900d,
  190 MB on IH and 780 MB on scene_mp_4p_03 (13.9M cells). With 6 threads:
  about 0.3, 1.1 and 4.7 GB.
- **`rand()` in `search_node.h`:** replaced with a small per-thread
  generator. This changes A*'s tie-breaking even with 1 thread, so results
  won't match earlier runs step for step.
- **Counters:** only touched at commit, on the main thread. No change.

### Trade-off

Agents planned in the same step don't see each other's new paths. They do
see every path committed in earlier steps. With IH getting about 14 paths
per step now, a slightly less congestion-aware path is far better than none.
A later variant could commit in batches (say 64) so batches see each other,
at the cost of threads waiting for the slowest search in each batch.

### Optional second step: stop searches overrunning

Pass the deadline into `astar()` and check it every few thousand
expansions; if it has passed, abandon the search and leave the agent for
next step. Needs a "search abandoned" return, because `astar()` currently
exits the program when it finds no path. Fixes the overruns but wastes the
work of long searches that are cut off. To be done after parallelism, as
its own flag, so its effect can be measured separately.

### Logging

Add the thread count to the `planner stats:` line. `paths_built`,
`bfs_no_path` and the `stuck breakdown:` line already measure the effect.

## Testing

1. **Correctness.**
   - `tiny.json` and the hand-built trap map (see
     `ai/planner_local_bfs_plan.md`) with 1 and 6 threads: 0 errors, trap
     escaped.
   - Debug path check: every committed path starts at the agent's location,
     ends at its goal, and only takes single-cell steps between open cells.
   - Debug congestion-map check: every 100 decisions, rebuild the flow map
     from all agents' paths and compare it with `lns.flow`, to catch paths
     added or removed twice.
   - Both checks go behind a debug flag (`GUIDE_PATH_DEBUG_CHECKS`), off for
     the sweep.
2. **IH 10k, 500 steps, solver 6 level 4,** with the stuck diagnosis,
   compared with the sequential run (366 decisions, 535 deliveries, 1,915
   trapped + 1,146 blocked stuck, over 6,700 without a path). Expected:
   about 6× more paths per step, no-path count near 0 within a few hundred
   steps, stuck count following it down.
3. **orz900d 10k, 1,500 steps:** no loss against 5,495 deliveries; PIBT
   within its 100 ms reserve.
4. **Memory:** a short run on scene_mp_4p_03 80k (largest map and agent
   count) to check peak memory with 6 pools.
5. **If IH looks good:** the full IH 1,500-step check against the go/no-go
   criteria used before the sweep: 0 errors, decisions ≈ steps,
   deliveries rising with no plateau, stuck count in the tens not
   thousands, PIBT under its reserve.

## Caveats

- **Results depend on core count.** They already depend on the machine
  through wall-clock limits; the sweep records CPU and core count
  (`sweep_meta.json`). The planner is shared by all solvers, so the
  comparison between solvers stays fair, but the thesis should say guide
  paths are built in parallel, unlike the upstream planner.
- **Less reproducible.** Which agents get a turn depends on timing. Runs
  already vary for the same reason; this adds a little.
- **Same-step agents don't see each other's paths** (see "Trade-off").
- **Overruns aren't fixed by parallelism alone.** The last searches of a
  step can still run past the deadline; that's the optional second step.
- **No single search gets faster.** A better A* heuristic (landmark
  distances, or distances on the coarsening hierarchy) would cut the cost
  per search, and would multiply with this. Not part of this plan.
- **Memory grows with map size × threads** (see the table above). If
  scene_mp_4p_03 80k runs short, use fewer threads there or a smaller pool
  that grows as needed.

## Alternatives considered

- **A better A* heuristic.** Could help more on big mazes, but the gain is
  harder to predict and it's more work. Worth doing after this.
- **Scheduler paths (`PASS_SCHEDULER_PATHS_TO_PLANNER`).** Tested: no
  effect, because solver 6 matches nearly every agent locally and only
  flow-matched agents get a path (3,489 on IH, nearly all at step 0).
- **Better fallback for agents without a path** (e.g. following the coarse
  hierarchy route). Would reduce trapping while agents wait, but doesn't
  reduce the wait. Could complement this.
- **Parallel PIBT.** PIBT isn't the bottleneck (max 35 ms on IH) and is
  sequential by design (priority order).

## Files to change

- `default_planner/const.h`: `GUIDE_PATH_THREADS`, `GUIDE_PATH_DEBUG_CHECKS`.
- `default_planner/planner.cpp`: the parallel stage 2 and commit; debug
  checks; log field.
- `default_planner/flow.cpp` / `flow.h`: split `update_traj()` into
  "search" and "commit" parts so both paths share them.
- `default_planner/search_node.h`: per-thread random generator.
- `default_planner/TrajLNS.h`: per-thread search pools.
- `scripts/run_thesis_sweep.py`: record `GUIDE_PATH_THREADS` in
  `sweep_meta.json`.

## Implementation and results

### What was built (2026-09-29/30)

All switches are in `default_planner/const.h` and can be overridden at build
time with `-D`, like the other planner flags.

| Constant (build macro) | Default | Meaning |
|---|---|---|
| `GUIDE_PATH_THREADS` (`PLANNER_GUIDE_PATH_THREADS`) | 1 (6 during the tests below; set back to 1 on 2026-09-30, see "Conclusions") | Guide-path worker threads. 1 = the original sequential loop. Parallel mode needs `USE_MANHATTAN_HEURISTIC`. |
| `GUIDE_PATH_EXTRA_POOL_MB` | 2048 | Memory cap for the extra A* search pools (56 bytes per map cell each; thread 0 reuses the planner's own pool). Gives 6 threads on orz900d, IH and warehouseXL, 3 on the scene maps. Added because scene_sp_pol_06 already peaks at about 24 GB of the machine's 31 GB, and one pool there is about 1 GB. |
| `GUIDE_PATH_DEBUG_CHECKS` (`PLANNER_GUIDE_PATH_DEBUG_CHECKS`) | false | Path check on every new path, and congestion-map rebuild-and-compare every 100 decisions. Exits on a mismatch. |
| `ASTAR_HEURISTIC_WEIGHT` (`PLANNER_ASTAR_HEURISTIC_WEIGHT`) | 1.0 (see below) | Weight on A*'s Manhattan heuristic (weighted A*). With a weight above 1, A* ignores a cheaper route to an already-processed cell instead of exiting with "re-expansion" (standard weighted A*; keeps the weight × best bound). |

Code:

- `planner.cpp`: `build_guide_paths_parallel()` (collect agents needing a
  path in rotated order → take scheduler paths on the main thread → workers
  search against the current `lns.flow` → commit finished paths in order);
  `guide_path_threads_used()`; `check_path_or_die()`,
  `check_flow_or_die()`. The `planner stats:` line gained `guide_threads`
  and `abandoned_searches`.
- `flow.cpp`: `commit_traj()` (the commit half of `update_traj()`).
- `search.cpp` / `search.h`: `astar()` takes an optional deadline, checked
  every 1,024 expansions; a search past it returns an empty path and is
  retried next step. The weighted heuristic (`manhattan_h`).
- `search_node.h`: `search_rand()`, a per-thread generator replacing
  `rand()` in A*'s tie-breaking.
- `TrajLNS.h`: `guide_pools`.
- `scripts/run_thesis_sweep.py`: records the new switches in
  `sweep_meta.json`.

### Correctness (debug build, checks on)

`tiny.json` 110 steps and the trap map 60 steps, with 6 and 1 threads; and
orz900d 10k, 300 steps, with 6 threads: no path-check or congestion-map
failures (3 congestion-map checks on orz900d), 0 errors, trap agent
escaped and delivered in both.

### Finding: A* gets much slower as the congestion map fills

Stand-alone benchmark (`astar_bench.cpp`, kept in the session scratchpad):
A* on the real IH map, start/goal pairs 800-1,300 cells apart in the
largest region, congestion map filled with K earlier paths first.

| Paths in congestion map | ms per search, 1 thread | searches/s, 6 threads |
|---|---|---|
| 0 | 15-22 | 190 |
| 1,000 | 66 | 54 |
| 4,000 | 167 | 28 |
| 8,000 | 240 | 17 |

So threads scale well (about 4× with 6), but with about 10k paths
registered each search costs about 250 ms or more. The congestion
penalties are added to A*'s ordering and Manhattan distance says nothing
about them, so A* explores nearly everything cheaper than the final route.

Weighted A* on the same benchmark (8,000 paths, 6 threads):

| Weight | Searches/s | Path length | Path cost (length + congestion) |
|---|---|---|---|
| 1.0 | 17 | 1,096 | 2,107 |
| 1.5 | 25 | 1,095 | 2,163 (+2.7%) |
| 2.0 | 59 | 1,095 | 2,227 (+5.7%) |
| 3.0 | 137 | 1,123 | 2,524 (+20%) |

### Full IH runs (10k agents, 500 steps, solver 6 level 4, one at a time)

| Planner | Deliveries | Decisions | No path at decision 240 | Stuck at 240 |
|---|---|---|---|---|
| Sequential | 535 | 366 | 7,917 | 2,270 |
| 6 threads | 319 | 244 | 5,569 | 1,612 |
| 6 threads + weight 2 | 352 | 250 | 2,809 | 804 |

More agents got paths and fewer got stuck, but stage 2 overran its
deadline (about 1.2 s against about 0.8 s), roughly halving the number of
decisions, so deliveries fell. Searches in the simulation were also slower
than in the benchmark, likely because real traffic concentrates around
pickup and delivery areas. This led to the in-search deadline (above).

### In-search deadline and rotation fix

With the deadline, stage 2 stopped overrunning (499 decisions in 500
steps), but on IH every search took longer than the whole stage-2 window,
so the same 6 searches were abandoned and restarted every step (0-1 paths
built). Fix: the next step starts after the agents tried this step, so an
abandoned agent goes to the back of the queue. IH 10k, 500 steps, one run
at a time:

| Planner | Deliveries | Decisions | Stuck at end |
|---|---|---|---|
| Sequential | 535 | 366 | 3,061 |
| 6 threads + weight 2 + deadline | 764 | 499 | 4,954 |
| same + rotation fix | 1,141 | 499 | 1,190 |
| 6 threads + deadline + rotation, **congestion ignored** (`GUIDE_PATH_IGNORE_CONGESTION`) | **1,299** | 499 | **1** |

### Harness bug found on the way (fixed)

`Entry::compute` built and copied a whole-map "background flow" array every
step for every solver (about 650 ms per step on scene_mp_4p_03), although
it's only read with `--useTraffic`. Fixed without behaviour change; see
`ai/run_log.md` 2026-09-30. All numbers below are after the fix.

### Evidence runs after the fix (one at a time, solver 6 level 4)

| Run | Deliveries | Stuck at end | Agents without a path at end |
|---|---|---|---|
| orz900d 10k, 1,500 steps, **sequential** (congestion on) | **5,492** | 6 | 0 |
| orz900d 10k, 6 threads, congestion on | 4,520 | 9 | 0 |
| orz900d 10k, 6 threads, congestion ignored | 4,901 | 12 | 0 |
| scene_mp_4p_03 10k, 500 steps, 3 threads, congestion on | 203 | 4,620 | 7,909 |
| scene_mp_4p_03 10k, 3 threads, congestion ignored | 214 | 2,886 | 5,605 |
| IH 20k, 500 steps, 6 threads, congestion on | 1,788 | 5,149 | 13,358 |
| **IH 20k, 6 threads, congestion ignored** | **2,431** | **0** | **0** |

0 errors in every run; 468-500 decisions per 500 steps (scene 468-469,
the rest 499-500).

## Conclusions (2026-09-30)

1. **On orz900d, sequential congestion-aware paths are best.** Parallel
   paths give 11-18% fewer deliveries there even though every agent gets a
   path. Likely cause (not proven): agents planned in the same step all
   dodge the same congestion and pile into the same alternative routes.
   Plain shortest paths (congestion ignored) are about 11% worse than
   sequential congestion-aware ones, so congestion-awareness is worth
   something when the planner can keep up.
2. **On IH (10k and 20k), only parallel paths that ignore congestion
   work.** Congestion-aware A* costs about 250 ms or more per search once
   about 10k paths are registered (benchmark above), so the planner can't
   keep up and agents without a path get trapped.
3. **On scene_mp_4p_03, nothing works yet.** Even plain A* takes about
   200-300 ms per search there (very long paths on a 3,728 × 3,728 map), and
   the memory cap allows only 3 threads, so about 8-12 paths per step. This
   needs a better A* heuristic (landmark distances, or distances on the
   coarsening hierarchy). Not started.
4. **Defaults left at the known-good sequential setting.** Everything new is
   behind flags: `GUIDE_PATH_THREADS`, `GUIDE_PATH_IGNORE_CONGESTION`,
   `ASTAR_HEURISTIC_WEIGHT`, `GUIDE_PATH_DEBUG_CHECKS`.

Options for the user to decide between (none tried as a whole yet):

- A per-map setting (sequential on small maps, parallel + congestion
  ignored on big ones). Simple but the planner then differs between maps,
  which the thesis would have to explain.
- Parallel + congestion ignored everywhere (costs about 11% on orz900d vs
  sequential; works on IH; still fails on scene).
- First build a better A* heuristic, then revisit: it could make
  congestion-aware search affordable again and is the only route found so
  far for the scene maps.
