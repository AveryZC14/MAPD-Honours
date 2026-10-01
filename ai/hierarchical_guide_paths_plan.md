# Plan: guide paths from the coarsening hierarchy

Status: **implementation planned, not started (2026-10-01); see
"Implementation plan" at the end.** Two path builders, both behind a flag
that is off by default: the spliced lift, and A* on the fine map limited
to the coarse path's nodes (corridor A*). The level is a run parameter to
sweep and report in the thesis, not a rule. The existing
lift has been benchmarked (see "Results"): as solver 6 ran it, it only
worked from level 1. **Fixed 2026-10-01 (permanent, no switch):** every
level is now anchored to the real start and goal, which was the original
design intent. The lift now gives a valid path for every tested pair on
orz900d, IH and scene_mp_4p_03 in a few ms each, apart from paths longer
than the 5,000-cell cap. Follows on from
`ai/parallel_guide_paths_plan.md` (why guide paths are the planner's
bottleneck) and `ai/planner_local_bfs_plan.md` (why every agent needs a
path).

Related docs on the existing lift:

- `ai/project_context.md`, "Solver 6", "Per-timestep flow": steps 1-4 of
  `compute_reduced_assignment`.
- `ai/claude_memleak_fixes.md`, Round 1: why the lift needs lead-in and
  lead-out hops (bridge segments join representative nodes, not the real
  start and goal).
- `ai/guide_path_metric.md`, "Bug found and fixed": lifts that came back with
  the wrong endpoints, the endpoint check and full-map fallback added for
  them, and `./build/guide_path_validator`.

## The problem

The planner builds a guide path for an agent with one A* search on the fine
map, in stage 2 of `DefaultPlanner::plan()`. On big maps that search is too
slow: about 12 ms on orz900d, 70-300 ms on IH_mp_2p_01, and 200-300 ms on
scene_mp_4p_03 even with congestion ignored. Agents waiting for a path steer
by Manhattan distance and get trapped at walls. Parallel A* helps on IH but
nothing tried so far works on scene_mp_4p_03. Numbers in
`ai/parallel_guide_paths_plan.md`.

## The idea

Find the route on the coarsening hierarchy instead of the fine map, then
turn it into a fine path the way solver 6 already does:

1. Map the agent's cell and its goal up to level `L` (`to_coarser_node_id`).
2. Run A* (or Dijkstra) on level `L`'s graph between those two coarse nodes.
   This graph is far smaller than the fine map.
3. Lift the coarse path down to the fine map with the existing lift
   (`lift_coarse_paths_to_fine`, `MapCoarsenV1.cpp`): at each level, splice
   in the cached bridge segment between adjacent coarse nodes
   (`bridge_path_cache`, built once with the hierarchy), plus a short
   lead-in and lead-out search inside one component where the segment's ends
   don't match. At the last level the endpoints are the real start and goal.

This is close to HPA* (hierarchical path-finding A*).

## How the existing lift works

Solver 6 already lifts coarse paths, but only for agents matched through the
coarse flow, and only for the leg to the pickup. Locally matched agents,
pinned agents and delivery legs get nothing. The paths are built every step
for the `GuidePathLengthSum` metric. They only reach the planner with
`PASS_SCHEDULER_PATHS_TO_PLANNER` or `--useTraffic` past timestep 100, so in
the sweep the planner never uses them.

- **At hierarchy build time:** each coarse node gets one representative
  finer node (`chosen_finer_node_id`, the first in its group). For each pair
  of adjacent coarse nodes, a BFS inside the two components joins their
  representatives (`build_cached_bridge_path_local`), stored in
  `bridge_path_cache`.
- **Lift, one level at a time** (`expand_path_batch_one_level_local`): for
  each consecutive pair of coarse nodes, splice in the cached segment, with a
  lead-in and lead-out search inside one component where needed.
  Intermediate levels use the representatives as endpoints. The last
  expansion (level 1 to fine) uses the real start and goal. A path fails if
  a bridge is missing, a lead-in/out search fails, or it passes 5,000
  cells.
- **Check and fallback:** a path that is empty or doesn't run from the real
  start to the real goal is replaced by A* on the whole fine map
  (`shortest_path_in_graph_local`, Manhattan heuristic, unit costs).

## What is guaranteed

- **Every lifted path is valid.** Each cell comes from a search on the real
  graph (bridge BFS, lead-in/out search), so a successful lift never crosses
  an obstacle. The endpoint check catches wrong endpoints.
- **In theory every lift succeeds.** Coarsening splits each 2×2 block into
  its connected components, so each coarse node is connected on the fine
  map, and a coarse arc exists only where some finer arc crosses between two
  nodes. So a coarse path exists exactly when a fine path does, and every
  bridge and lead-in/out search runs inside a connected region.

## What isn't guaranteed

- **The lift has failed in practice.** `ai/guide_path_metric.md` records
  lifts on `tinyComplex` with wrong endpoints, and blames geometric
  grouping. The benchmark below found the real cause: intermediate levels
  aren't anchored to the real start and goal (see "Results"). Fixed
  permanently on 2026-10-01.
- **The 5,000-cell cap.** Lifted paths longer than 5,000 cells fail. On
  scene_mp_4p_03 (3,728 × 3,728) long paths plus detours may hit it.
- **Path quality.** The path passes through a representative node at every
  level, so it can detour (lead-in, bridge, lead-out) at each one. Coarse
  arc costs are static averages, not real walking distances, so the coarse
  route itself may not be the shortest. PIBT follows the guide path, so
  extra length means extra travel.
- **No congestion.** Every fine arc costs 1. This behaves like
  `GUIDE_PATH_IGNORE_CONGESTION`, which was best on IH but about 11% worse on
  orz900d than sequential congestion-aware paths.

## Improvement to consider: corridor refinement

After the coarse search, run a fine A* restricted to the cells inside the
coarse nodes on the coarse path (perhaps with a one-node margin). That gives
near-shortest paths at a fraction of a full-map search, with the spliced
lift kept as the fallback. Level `L` sets the trade-off: a higher level makes
the coarse search cheaper and the corridor wider. **Now part of the
implementation plan** ("Corridor A*" at the end).

## Where it would live

The aim is a path for every agent and every goal, so it belongs in the
planner (stage 2, replacing `update_traj`'s A*), not in solver 6. Then:

- It applies to every solver, so the hierarchy has to be built for every
  solver. Today `schedule_initialize` only builds it for solver 6; IH takes
  about 300 s to build without a cache.
- The planner would depend on `map_reduction_test/`. The thesis would
  describe the planner change as applying to all solvers.
- The local path BFS (`USE_LOCAL_PATH_BFS`) scores moves against whatever
  path the agent has, so it needs no change.

## Testing the existing lift

Before building anything, measure the existing lift on the paths this plan
would produce: random start/goal pairs, a shortest coarse path at level `L`,
then `lift_coarse_paths_to_fine`. For each map and level:

- how often the lift succeeds, and why it fails (missing bridge, lead-in/out
  search failed, 5,000-cell cap, wrong endpoints);
- lifted path length against the true shortest distance (fine BFS);
- time for the coarse search, the lift, and the fallback;
- that every returned path is valid (walkable, adjacent steps, right
  endpoints).

Maps: orz900d, IH_mp_2p_01, scene_mp_4p_03, using the caches in
`hierarchy_cache/`. Results go in "Results" below.

## Results (2026-10-01)

**Tool:** `./build/bench_hierarchy_lift` (`utils/validation/bench_hierarchy_lift.cpp`).
For random (agent start, task location) pairs from the instance, it finds
the cheapest coarse path at level `L` with Dijkstra on that level's graph
(the arc costs the coarse flow uses), lifts it with
`lift_coarse_paths_to_fine`, checks the result (endpoints, walkable cells,
4-adjacent steps) and compares its length with the true shortest distance
(fine BFS). Raw output and per-pair CSVs: `outputs/hierarchy_lift_bench/`
(`run.sh` reproduces them). 300 pairs on orz900d and IH, 200 on
scene_mp_4p_03; the 10k instances in `instances/thesis_benchmarks/`.

**Changes to the lift** (`MapCoarsenV1.{h,cpp}`):

- `outcomes_out` (optional argument): per-path diagnostics (`LiftOutcome`:
  level and reason a lift failed, wrong endpoints, fallback used and its
  time). Collecting them changes no path.
- **Every level anchored** (permanent since 2026-10-01; first benchmarked as
  the option `anchor_every_level`): at every level, start and end the
  expansion at the nodes containing the real start and goal at the level
  below, instead of the coarse nodes' representatives. Also joins a
  one-node path (start and goal in the same coarse node) with a search
  inside that node. This changes solver 6's guide paths and
  `GuidePathLengthSum`; see "The fix" below.
- `max_path_cells` (optional argument): the length cap, still 5,000 by
  default.

### Finding: solver 6's lift only works from level 1

Intermediate levels walk from representative to representative. The last
expansion (level 1 to fine) uses the real start only if the agent's level-1
node is the first node of the lifted path, which from level 2 up it almost
never is. So the path starts (or ends) in the wrong place, the endpoint
check rejects it, and the full-map fallback runs. That fallback (A* capped
at 200,000 expansions) itself fails on long paths on the big maps.

Lift as solver 6 runs it (no anchoring, cap 5,000), pairs out of 300 (200
on scene):

| Map | Level | Lift ok | Fallback ok | No path at all | Fallback ms (mean) |
|---|---|---|---|---|---|
| orz900d | 1 | 300 | 0 | 0 | – |
| orz900d | 2 | 20 | 280 | 0 | 15 |
| orz900d | 4 | 0 | 300 | 0 | 15 |
| IH | 1 | 300 | 0 | 0 | – |
| IH | 2 | 16 | 161 | 123 | 78 |
| IH | 4 | 0 | 174 | 126 | 75 |
| scene_mp_4p_03 | 2 | 7 | 43 | 150 | 101 |
| scene_mp_4p_03 | 4 | 0 | 45 | 155 | 102 |

This affected real solver 6 runs too: flow-matched agents' paths were
lifted every step (then `need_guide_paths = true`, now
`--computeGuidePaths`, default on), at `--flowSolveLevel` 4 in the sweep,
so most of them paid for the fallback search in scheduler time and the
failed ones were missing from `GuidePathLengthSum`. Measured below
("Runtime effect inside solver 6").

### How the bug happens, step by step

The top-level coarse path is right: it starts at the coarse node containing
the agent and ends at the one containing the goal. Only the last expansion
(level 1 to fine) gets the real start and goal as anchors. Before the fix,
`lift_coarse_paths_to_fine` set them like this:

```cpp
if (level == 1)
{
    preferred_starts.push_back(env->curr_states[agent_ids[i]].location);
    preferred_goals.push_back(env->task_pool[task_ids[i]].locations[0]);
}
// any other level: no anchors
```

With no anchor, `expand_path_batch_one_level_local` starts the expansion
at the coarse node's representative child (`chosen_finer_node_id`, the
first child in its list), and targets the last node's representative.

Example, lifting from level 2. The agent is at fine cell `s`, in level-1
node `a3`, in level-2 node `A`, whose children are `a1` (representative),
`a2`, `a3`.

1. Level-2 path `A → B → C`: right, `A` contains the agent.
2. Expand to level 1, no anchors: the path starts at `a1`. It is in the
   right parent but the wrong child; the agent is in `a3`.
3. Expand to fine, anchor `s`: the anchor is used only if its parent
   matches the path's first node. `s`'s level-1 node is `a3`, the path's
   first node is `a1`, so the anchor is rejected and the expansion starts
   at `a1`'s representative cell.
4. The path is walkable but starts at the wrong cell. The endpoint check
   rejects it and the full-map fallback runs.

The goal end goes wrong the same way. A level-1 lift works because its one
expansion is the anchored one. Lifts from higher levels work only when the
start and goal happen to lie in the representative child at every level.

This is also what `ai/guide_path_metric.md`'s `tinyComplex` bug was. That
doc blamed coarse nodes spanning disconnected regions, but the coarsening
splits each 2×2 block into connected components, so that can't happen.

### The fix (every level anchored, permanent)

**Anchors at every level.** When expanding level `l` to `l-1`, the anchors
are the nodes containing the real start and goal at level `l-1`:

```cpp
for each path i:
    preferred_starts.push_back(map_fine_node_to_level_node_local(hierarchy_, start, level - 1));
    preferred_goals.push_back(map_fine_node_to_level_node_local(hierarchy_, goal, level - 1));
```

At level 1 this gives the real cells, exactly as before.

In the example, step 2 now starts at `a3`. The cached bridge segment from
`A` to `B` starts at some other child of `A`. The existing lead-in search
(inside `A`) joins `a3` to it. At the other end, the existing lead-out
search joins the last segment to the goal's child. No new joining logic is
needed: the anchors the level-1 step already used are supplied at every
level.

**Why it always works.** If the level-`l` path starts at the node
containing the start at level `l`, the anchor at level `l-1` (the start's
node there) has exactly that node as its parent, so it is always accepted,
and the level-`l-1` path starts at the start's node at `l-1`. The coarse
search starts the top-level path correctly, so by induction the fine path
starts exactly at the agent's cell. The goal end is the same.

**One-node paths.** When start and goal are in the same coarse node, the
coarse path is one node and has no pairs to expand, so the old loop never
reaches the goal's child. With the flag on, one search inside that node
joins the start's child to the goal's child. This is common at high levels
and for an agent whose task is in its own coarse node.

**Cost.** One extra lead-in and lead-out per level, at the two ends of each
path. At intermediate levels these searches cover the children of one
coarse node (at most 4, one 2×2 block). Mapping a cell up the levels is a
few array lookups.

**Effect on solver 6.** It was first added as an option
(`anchor_every_level`, off by default) and then made permanent on
2026-10-01, since anchoring at every level was always the intended design.
Solver 6's flow-matched agents now get lifted paths instead of mostly
fallback A* paths, so `GuidePathLengthSum` changes: more agents are counted
(the fallback no longer fails for them), lengths are the lifted paths'
(slightly longer than shortest), and scheduler time drops by the fallback
searches. Solver 6 `GuidePathLengthSum` values from before 2026-10-01 are
not comparable with later ones.

### With every level anchored (cap raised to 100,000)

Every pair got a valid lifted path at every level tested, with no fallback,
on all three maps. Length is lifted path / shortest distance; time is the
coarse Dijkstra plus the lift, per path, means:

| Map | Level | Length mean (p90, max) | Coarse ms | Lift ms |
|---|---|---|---|---|
| orz900d | 1 | 1.015 (1.019, 1.14) | 3.9 | 0.3 |
| orz900d | 2 | 1.054 (1.072, 1.44) | 1.0 | 0.3 |
| orz900d | 3 | 1.13 (1.19, 2.33) | 0.3 | 0.3 |
| orz900d | 4 | 1.30 (1.39, 3.53) | 0.1 | 0.3 |
| orz900d | 6 | 1.79 (2.20, 9.76) | 0.02 | 0.4 |
| IH | 1 | 1.004 (1.008, 1.03) | 210 | 1.6 |
| IH | 2 | 1.013 (1.025, 1.08) | 34 | 1.6 |
| IH | 3 | 1.034 (1.057, 1.15) | 6.4 | 1.1 |
| IH | 4 | 1.094 (1.149, 1.37) | 1.5 | 0.9 |
| IH | 6 | 1.37 (1.57, 2.73) | 0.1 | 1.0 |
| scene_mp_4p_03 | 2 | 1.010 (1.017, 1.11) | 156 | 5.3 |
| scene_mp_4p_03 | 3 | 1.027 (1.042, 1.13) | 27 | 3.8 |
| scene_mp_4p_03 | 4 | 1.051 (1.076, 1.25) | 5.0 | 2.6 |
| scene_mp_4p_03 | 5 | 1.11 (1.18, 1.72) | 1.3 | 2.5 |
| scene_mp_4p_03 | 6 | 1.24 (1.36, 2.18) | 0.4 | 3.2 |

For comparison, the planner's congestion-free fine A* takes about 12 ms
per path on orz900d, 15-22 ms on IH with an empty congestion map (70-300 ms
in real runs with congestion), and 200-300 ms on scene_mp_4p_03.

The 5,000-cell cap has to go for this use: shortest paths on
scene_mp_4p_03 reach 5,915 cells, and with the cap the anchored lift failed
on 10-31 of 200 pairs at levels 2-6 (89 at level 8).

**Reading the numbers:**

- On IH and scene, level 3-4 gives paths within about 3-10% of shortest
  for 2-8 ms each, roughly 10-100 times cheaper than fine A*.
- orz900d's paths stretch faster with level (1.30 at level 4): its open
  areas make the detour through representatives relatively larger. Level 2
  there is 1.05 for about 1.3 ms.
- Detours grow quickly above level 4-5 on every map, with long tails (max
  up to 9.8 on orz900d level 6).
- The coarse search is Dijkstra with no heuristic, so at low levels it
  dominates (210 ms at IH level 1). An A* with a Manhattan heuristic on
  `fine_location`, or a per-map choice of level, would cut that.

### What has been tested (2026-10-01)

After making the anchoring permanent:

- `guide_path_validator` (solver 1 and solver 6 paths: endpoints,
  adjacency, metric sums), with and without `--useTraffic`: `tiny` 57
  checks, `tinyComplex` 95, `warehouseSmall_100` 4,268, `random_2000`
  15,159; 0 failed. (`tiny` had 53 checks before the change: one more agent
  now gets a valid path.)
- Bench with default settings (cap 5,000):
  `outputs/hierarchy_lift_bench/*_permanent.txt`. Same as the anchored
  results below; the only failures are the length cap (scene_mp_4p_03: 10,
  15, 21, 31 of 200 at levels 3, 4, 5, 6; orz900d: 50 and 136 of 300 at
  levels 8 and 10).
- `lifelong` solver 6, 200 steps: `tiny` 14 tasks finished, 0 errors,
  `GuidePathLengthSum` total 204 (202 before; lifted paths replace some
  fallback paths); `tinyComplex` 15 finished, 0 errors, total 192 (same).
- Not yet run: solver 6 on a big map, where the change matters most.

While it was still an option:

- **Anchored lift, in the bench:** every returned path checked for right
  endpoints, walkable cells and 4-adjacent steps: orz900d (300 pairs, levels
  1-10), IH (300, levels 1-8), scene_mp_4p_03 (200, levels 2-8),
  `tinyComplex` (100, levels 1-3, including the one-node top level). 0
  invalid paths, 0 failures with the cap raised.
- **Default behaviour unchanged:** `guide_path_validator` on `tiny` (53
  checks) and `tinyComplex` (95 checks), 0 failed; `lifelong` solver 6,
  200 steps: `tiny` 14 tasks finished (same as before), `tinyComplex` 15,
  0 errors.
- **Not tested:** the anchored lift inside a real solver 6 run (there is no
  switch for it yet, only the function argument), and the plan itself
  (hierarchy paths in planner stage 2).

### Runtime effect inside solver 6 (real runs, 2026-10-01)

Per path, the anchored lift is 20-80 times faster than the old lift plus
fallback at level 2 and above, on the bench's random long pairs (e.g. IH
level 4: 0.9 ms vs 76 ms; scene_mp_4p_03 level 4: 2.6 ms vs 104 ms). At
level 1 the two are the same, so anchoring itself costs nothing measurable.

In real solver 6 runs the effect is small. Solver 6, `--flowSolveLevel 4`,
sweep flags, 300 steps, old lift (a copy of the tree with only level 1
anchored) vs new, run one at a time. Output:
`outputs/hierarchy_lift_bench/solver6_old_vs_new/`. These runs predate the
timing fix below, so "scheduler solve" here still includes the lift
(solve plus guide time in today's fields).

| Run | Decisions | Finished | Flow-matched | Scheduler solve ms total (first step) | `GuidePathLengthSum` per flow-matched agent |
|---|---|---|---|---|---|
| IH 10k, old | 231 | 286 | 3,675 | 8,558 (350) | 24 |
| IH 10k, new | 225 | 278 | 3,660 | 8,112 (140) | 49 |
| orz900d 10k, old | 300 | 1,265 | 411 | 299 (12) | 17 |
| orz900d 10k, new | 300 | 1,277 | 410 | 264 (6) | 41 |

- **Scheduler time:** about 5-12% less in total, and the first step (the
  biggest batch) 2-2.5 times faster. The scheduler is a small part of each
  step (the planner uses about 1 s), so throughput doesn't change;
  deliveries differ by under 3%, within run-to-run noise.
- **Why so small:** flow-matched paths in real runs are short (tens of
  cells; local matching handles same-node pairs, and the flow mostly
  matches neighbouring nodes), so the old fallback A* on them was cheap
  (about 2 ms per agent), unlike the bench's random 1,200-cell pairs.
- **Path length doubled:** lifted paths on short pairs detour through
  representatives far more, relative to their length, than on long ones.
  From the bench (anchored, by shortest distance):

  | Map, level 4 | < 100 cells | 100-300 | 300-800 | ≥ 800 |
  |---|---|---|---|---|
  | orz900d | 2.10 | 1.37 | 1.23 | 1.25 |
  | IH | 1.17 (1 pair) | 1.16 | 1.11 | 1.09 |
  | scene_mp_4p_03 | – | 1.15 | 1.07 | 1.05 |

  So the old metric was mostly fallback A* lengths (shortest paths), and
  the new one is lifted lengths, about 2 times longer for these short
  paths. Solver 6's `GuidePathLengthSum` now measures the hierarchy's paths,
  but it is not comparable with the old values or with solver 1's.
- **For the planner plan:** short requests (local matches, short pickup
  legs) would get badly stretched paths at level 4. Either lift short pairs
  from a lower level (choose the level from the start-goal distance) or add
  the corridor refinement.

### Guide-path flag and timing fields (2026-10-01)

**`--computeGuidePaths`** (default `true`; `env->compute_guide_paths`,
`src/driver.cpp`, recorded as `computeGuidePaths` in the output JSON) gates
all scheduler guide-path work:

- Solvers 6/7: `need_guide_paths` (passed into
  `compute_hierarchical_assignment` / `compute_reduced_assignment_edge_augmented`)
  is the flag; `false` skips Steps 3-4 (the lift) entirely.
- Solver 1: the flow walk always runs (it is how the assignment is
  recovered); with the flag off it doesn't record the cells or arc costs.
- Solver 2 (`schedule_plan_flow_hist`): doesn't store its paths.
- With the flag off nothing reaches `agent_guide_path`, even with
  `--useTraffic` or `PASS_SCHEDULER_PATHS_TO_PLANNER`; a one-time warning
  says so (`warn_if_seed_needs_guide_paths`, `scheduler.cpp`).
- Only flow-matched agents get a path; local and cascade matches still
  don't.

**Timing fields, as of 2026-10-01** (per decision, in `timeStepMetrics`):

| Solver | `SchedulerSolveTime` | `SchedulerGuidePathTime` |
|---|---|---|
| 1 | graph construction, `NetworkSimplex`, and the flow walk that recovers the assignment | storing the walked paths and the length/cost sums (about 0.01 ms per step on orz900d) |
| 6 | from the end of Step 1 local matching: building the coarse flow graph, `NetworkSimplex`, Step 2 (flow decomposition) | Steps 3-4: lift, endpoint check, fallback searches, packaging, sums |
| 7 | same scope as solver 6: from the end of local matching through copying the backbone, adding proxy arcs, `NetworkSimplex`, Step 2 | as solver 6 |

For solvers 6/7, `SchedulerLocalMatchTime`, `SchedulerSolveTime` and
`SchedulerGuidePathTime` are disjoint and in that order. Not covered by any
field: bucketing agents/tasks to coarse nodes before local matching, the
cascade's level climbing, and the scheduler wrapper (pinning assigned
tasks, `ensure()`); all small. Solver 7's one-time backbone build has its
own field (`SchedulerBackboneBuildTime`, still always 0, see `ai/todo.md`).

All three are 0 for guide paths when the flag is off. Before 2026-10-01
the lift (Step 3, including fallback searches) was counted in
`SchedulerSolveTime` for solvers 6/7, and `SchedulerGuidePathTime` was only
Step 4; for solver 1, `SchedulerGuidePathTime` was the whole flow walk. So
both fields from earlier runs are not comparable with later ones. Also
fixed: solvers 6/7 reported 0 solve time when `NetworkSimplex` wasn't
optimal; `compute_reduced_assignment` now zeroes its time outputs on entry;
a dead second `OPTIMAL` check after the lift was removed. Then (same day)
the solve-time start was lined up: solver 6's used to start just before
`NetworkSimplex` (missing the coarse graph build, which no field recorded),
and solver 7's before Step 1 (so it included local matching, which was also
counted in `SchedulerLocalMatchTime`). Both now start right after local
matching.

Check after lining up (`outputs/hierarchy_lift_bench/solve_time_aligned/`,
level 4, 0 errors; first decision, ms):

| Run | Local match | Solve | Guide |
|---|---|---|---|
| IH 10k solver 6 (before lining up) | 24.5 | 22.4 | 143.3 |
| IH 10k solver 6 (after) | 26.3 | 48.1 | 148.2 |
| orz900d 10k solver 6 (after) | 10.8 | 2.4 | 6.4 |
| orz900d 10k solver 7 (after) | 42.3 | 14.3 | 4.7 |

Solver 6's solve time now includes the coarse graph build (about 26 ms on
the first IH decision). Solver 7's solve time no longer contains its local
matching (42 ms there). `guide_path_validator` (4 instances) and
`edge_augmented_validator` (`tinyComplex`): 0 failed.

**Checks:** `guide_path_validator` (4 instances, with and without
`--useTraffic`): 0 failed. `lifelong` on `tiny`, 50 steps, solvers 1/6/7,
flag on and off: 0 errors, same tasks finished, metrics and guide time 0
with the flag off, warning printed with `--useTraffic` and the flag off.
Real maps (`outputs/hierarchy_lift_bench/guide_path_flag/`):

| Run | Solve ms total (first step) | Guide ms total (first step) | Flow-matched |
|---|---|---|---|
| IH 10k solver 6 level 4, 100 steps, flag on | 1,134 (22) | 152 (143) | 3,509 |
| same, flag off | 1,226 (23) | 0 | 3,509 |
| orz900d 10k solver 1, 20 steps, flag on | 8,485 (909) | 0.1 | – |
| same, flag off | 9,256 (782) | 0 | – |

Solve time is the same with the flag on and off (within noise), so it no
longer contains guide-path work. On IH the first decision's lift (the
biggest batch) takes 143 ms; later decisions about 0.1 ms.

### Bench bug fixed on the way

The first version of the bench's BFS left the goal cell marked as visited,
which made later pairs with that goal look unreachable and could act as a
wall in later searches. Found because `tinyComplex` (2 goals) hung. Fixed
and every run redone; the numbers above are from the fixed version and
match the first runs closely.

### Next steps

Superseded by "Implementation plan" below (2026-10-01): the cap is
raised for the planner only, the level is a run parameter, and the corridor
refinement is one of the two path builders.

## Implementation plan (2026-10-01, not started)

### Goal

When the planner needs a guide path for an agent (new goal, or pushed more
than `LOCAL_PATH_BFS_RADIUS` cells off its path), build it from the
hierarchy instead of with a full-map A*:

1. Map the agent's cell and its goal up to level `L` (`to_coarser_node_id`).
2. Find the cheapest path between those two nodes on level `L`'s graph.
3. Turn it into a fine path with one of two builders (a run parameter):
   - **lift:** solver 6's spliced lift (`lift_coarse_paths_to_fine`).
   - **corridor:** A* on the fine map, limited to cells inside the coarse
     path's nodes (plus an optional margin of neighbouring nodes).
4. If that fails, fall back to the planner's existing full-map A*
   (`update_traj`).

With the flag off, the planner is exactly what it is today. Parallel guide
paths (`GUIDE_PATH_THREADS > 1`) stay off and aren't combined with this.

### The level is a parameter, not a rule

The level sets the main trade-off and belongs in the thesis as an
experiment:

- **Higher `L`:** a smaller coarse graph, so a cheaper coarse search. Lift
  paths detour more (bench: IH 1.004× shortest at level 1, 1.094× at 4,
  1.37× at 6). Corridors get wider.
- **Lower `L`:** paths closer to shortest, but the coarse search costs more
  (bench, Dijkstra: IH 210 ms at level 1, 1.5 ms at level 4).
- **Short paths stretch more** with the lift at a high level (orz900d
  level 4: 2.1× under 100 cells, 1.25× over 800). That is a result to
  report. Corridor A* should largely remove it: a short pair spans one or two
  coarse nodes, and A* inside them finds the true shortest path.

So `L` is a CLI flag, recorded in the output JSON, set per run and swept
(for example `L ∈ {2, 3, 4, 6}` on each map). It is **separate from
`--flowSolveLevel`**, so sweeping solver 6's level doesn't change the
planner. Whether the sweep crosses the two or holds one fixed is a
thesis-design decision still to make.

### Corridor A*

**What.** Same A* as today (`astar()`, `search.cpp`), with one extra check:
when generating a neighbour, skip it unless its level-`L` ancestor is on
the coarse path (or within `margin` coarse hops of it).

**Why a path always exists inside the corridor.** By construction of the
coarsening, each coarse node's cells are connected on the fine map (a 2×2
block of finer nodes is split into its connected components, by induction
from level 1), and two consecutive nodes on the coarse path have at least
one fine arc between them (a coarse arc exists only where a finer arc
crosses). So the union of the path's nodes is connected and contains the
start and the goal. With margin 0 the corridor A* can't fail if the coarse
search succeeded; a larger margin only adds cells.

**How the check is made cheap.**
- `ancestor_L`: one array, fine cell → level-`L` node, built once when the
  planner starts (4 bytes per cell: 3.9 MB on orz900d, 14 MB on IH, 56 MB
  on scene_mp_4p_03).
- `corridor_stamp`: one array of `uint32_t` per level-`L` node. For each
  search, increment a counter and stamp the coarse path's nodes (and their
  neighbours up to `margin`). The check is
  `corridor_stamp[ancestor_L[next]] == current_stamp`. No clearing between
  searches.
- `astar()` takes these as an optional argument (null = no limit, today's
  behaviour), next to the existing optional deadline. One `continue` in the
  neighbour loop.

**Expected size (estimate, to be measured).** A level-`L` node covers at
most `4^L` cells, and a coarse path between cells `d` apart has about
`d / 2^L` nodes on open maps (more in mazes), so the corridor is about
`d × 2^L` cells for margin 0. IH, `d ≈ 1,100`, `L = 4`: about 18k cells
(about 50k with margin 1), out of 3.44M. A* explores at most the corridor.

**Congestion.** Corridor A* can read the congestion map (`lns.flow`) or
ignore it, at no code cost (pass `trajLNS.flow` or the zero map, as
`GUIDE_PATH_IGNORE_CONGESTION` does). Congestion made full-map A* explode
on IH (about 250 ms per search with 8k paths); inside a corridor the
explosion is bounded. Default off, to match the lift; worth one test run.

### Step 1: hierarchy side (`map_reduction_test/MapCoarsenV1.{h,cpp}`)

1. **Lift on cells, not agent/task ids.** Move the level loop of
   `lift_coarse_paths_to_fine` into a core function that takes start and
   goal cells. The existing function becomes a wrapper that reads
   `curr_states` / `task_pool`, so solver 6 is unchanged.
2. **Cap and fallback as arguments.** Solver 6 keeps 5,000 and its full-map
   fallback. The planner passes `env->map.size()` (a safety brake no real
   path reaches; the cap was a July 2026 OOM-hunt guard against the old
   unbounded flow walk, and the lift itself can't loop) and no fallback.
3. **Coarse search.** Move `coarse_dijkstra` from
   `utils/validation/bench_hierarchy_lift.cpp` into `ReducedHierarchy`; the
   bench calls the shared copy. Dijkstra is enough at levels 3 and up. If
   levels 1-2 are in the sweep, add a heuristic: Manhattan distance between
   each node's representative fine cell (precomputed per node), scaled to
   the coarse arc-cost units. Check those units first (arc costs are
   averages); an overestimate gives slightly worse routes, not wrong ones.
4. **Public entry points:**
   - `coarse_path(start_cell, goal_cell, L)` → level-`L` node ids (one
     node if both are in the same node).
   - `lift_path(start_cell, goal_cell, L, coarse_path)` → fine path or
     empty.
   - `level_ancestors(L)` → the `ancestor_L` array.

**Check:** `guide_path_validator` on `tiny`, `tinyComplex`,
`warehouseSmall_100`, `random_2000`, with and without `--useTraffic`:
0 failed, same check counts as now. Solver 6 `GuidePathLengthSum` on
`tiny` / `tinyComplex` unchanged (204 / 192).

### Step 2: planner side (`default_planner/`)

1. **Settings.** CLI flags (so a sweep needn't rebuild per value),
   recorded in the output JSON:
   - `--guidePathSource` = `astar` (default, today's planner) / `lift` /
     `corridor`
   - `--guidePathLevel` = `L`
   - `--guidePathCorridorMargin` = coarse hops added around the path
     (default 0)
   - `--guidePathCorridorCongestion` = true/false (default false)
2. **`DefaultPlanner::initialize`:** if the source isn't `astar`, call
   `ReducedHierarchy::instance().ensure(env)` (a no-op for solvers 6/7,
   which already built it; loads from `--hierarchyCache` for the others)
   and build `ancestor_L`.
3. **Stage 2, sequential loop** (`planner.cpp`, the `else update_traj`
   branch): in order, the scheduler's path (only with
   `PASS_SCHEDULER_PATHS_TO_PLANNER`, as now), then the hierarchy path,
   then `update_traj` if that came back empty. A hierarchy path is
   installed with `commit_traj(..., s_node())`, as scheduler paths already
   are. Used for new goals **and for `needs_replan` agents** (decided
   2026-10-01: same code path, and replans become cheap). Confirm nothing reads
   `goal_nodes[i]` in Manhattan mode.
4. **Guard:** source ≠ `astar` with `GUIDE_PATH_THREADS > 1` → one-time
   warning, run sequential.
5. **Logging:** `planner stats:` gains `hier_paths`, `hier_fallbacks`,
   `hier_coarse_ms`, `hier_build_ms` (lift or corridor A*), `hier_cells`
   (sum of path lengths), and for corridor mode `corridor_expanded`.
   `GUIDE_PATH_DEBUG_CHECKS` runs `check_path_or_die` on hierarchy paths
   too.

### Step 3: plumbing

- `scripts/run_thesis_sweep.py`: pass `--hierarchyCache` for every solver
  when the source isn't `astar` (today only solvers 6/7 get it); pass and
  record the new flags; check `--preprocessTimeLimit` covers loading the
  cache for solver 1.
- Memory: solver 1 doesn't hold a hierarchy today. Measure its peak on
  scene_sp_pol_06 (already about 24 GB of 31 GB).

### Step 4: bench before the planner runs

Extend `bench_hierarchy_lift` with corridor A* on the same pairs and
levels: length / shortest, time, cells expanded, corridor size, margin
0 and 1. Cheap, and it gives the lift-vs-corridor and level tables for the
thesis before any long run.

### Step 5: tests in the planner

| # | Run | Pass / compare with |
|---|---|---|
| 1 | `tiny`, `tinyComplex`, trap map; both sources; debug checks on | 0 errors, no check failures, trap agent escapes and delivers |
| 2 | source `astar` on the same instances | identical to the current build |
| 3 | orz900d 10k, 1,500 steps, solver 6 level 4 | 5,492 deliveries (sequential congestion-aware A*) |
| 4 | IH 10k, 1,500 steps | 2,364 deliveries, 4,661 stuck (sequential A*); expect agents without a path near 0 within tens of steps, stuck in the tens, decisions ≈ steps |
| 5 | scene_mp_4p_03 10k, 500 steps | best so far 214 deliveries with about 5,600 agents without a path |
| 6 | IH and scene 20k, then an 80k smoke run | PIBT within its 100 ms reserve, memory fine |
| 7 | solver 1 on one big map | hierarchy loads from cache, memory fine |

Runs 3-5 for both sources at one level first (`L = 4`), then the level
sweep for whichever source looks better. One run at a time; results in
`ai/run_log.md`.

### Thesis notes

- The planner's guide paths come from the hierarchy for **every** solver,
  so the solver comparison stays fair, but it has to be stated.
- The guide-path level and source are experimental parameters with their
  own results (path stretch, time per path, throughput by level).

### Open decisions

- Sweep design: guide-path level crossed with `--flowSolveLevel`, or one
  held fixed.
- Default margin (0 or 1), from the step 4 bench.
- Corridor congestion on or off, from one test run.
