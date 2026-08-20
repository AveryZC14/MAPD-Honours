# Solver 6: local node matching vs. flow solve -- runtime breakdown

Added 2026-08-20. Answers: how much of solver 6's per-timestep scheduling
time is Step 1 (`match_local_node_exact`, see `ai/local_node_matching.md`)
vs. Step 2 (the coarse-graph `NetworkSimplex` flow solve), how that splits
across `--flowSolveLevel`, and whether parallelizing Step 1 is worth doing.

## New metric: `SchedulerLocalMatchTime`

Step 1 (bucketing agents/tasks by top-level node, then calling
`match_local_node_exact` per node) previously had no timing of its own --
`solve_time_out` only starts *after* Step 1 finishes (coarse graph build +
`NetworkSimplex::run()` + Step 2's residual-flow recovery walk + Step 3's
coarse-to-fine lift, all lumped together). Added a `local_match_time_out`
out-param to `compute_reduced_assignment` (`MapCoarsenV1.cpp`/`.h`) that
accumulates wall-clock time spent inside the `match_local_node_exact` calls
only, threaded through the same plumbing as the existing
`LocalNodeMatchCount`/`FlowMatchCount` metrics (`ScheduleTiming` ->
`TimeStepMetric` -> JSON): every `./build/lifelong --scheduleModel 6` run
now reports `SchedulerLocalMatchTime` per timestep in `timeStepMetrics`.
Always 0 for other solvers, which have no local-matching step.

### Bug found + fixed while adding it: field missed one of the two duplicate-push sites

Same double-push mechanism as the `LocalNodeMatchCount` bug in
`ai/local_node_matching.md` (`BaseSystem::plan()` can push two
`TimeStepMetric` entries for one real scheduler call when it's slow enough
to force catch-up timesteps, both copied from the same `last_scheduler_timing`).
The initial edit added `metric.SchedulerLocalMatchTime = ...` after
identical-looking lines at both push sites in `CompetitionSystem.cpp`, but
the two sites have different indentation (one nested inside the catch-up
`for` loop, one not), so an exact-string `replace_all` only matched the
first. Result: the catch-up-push row of a duplicated pair got the real
value, the normal-push row silently stayed at the field's `0.0` default --
caught by comparing the two rows of a known-duplicate pair (identical
`SchedulerSolveTime`/`PlannerTime`/counts, but `SchedulerLocalMatchTime`
0.0718 vs. 0.0) and confirmed fixed by re-running (both rows now read
0.40458374199999986, byte-identical). Fixed by adding the line to the
second site directly.

## Runtime split, by `--flowSolveLevel`

`scene_mp_4p_03` (~13.9M cells, ~6.3M walkable), 20000 agents, `-s 200`,
shared depth-9 `--hierarchyCache` (`outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_level9.hierarchy`,
no rebuild). Per-run totals summed over de-duplicated scheduler calls (the
double-push rows above collapsed to one before summing, to avoid the same
double-counting trap `ai/local_node_matching.md` already found for the
match-count totals):

| level | calls | ΣlocalMatch (s) | Σflow-solve (s)\* | local / (local+flow) | local:flow match-count ratio | tasksFinished |
|---|---|---|---|---|---|---|
| 2 | 41  | 0.010 | 108.913 | 0.0095% | 0.08   | 6170 |
| 4 | 119 | 0.053 | 62.560  | 0.0845% | 0.93   | 6917 |
| 6 | 191 | 0.103 | 2.009   | 4.87%   | 10.35  | 6859 |
| 8 | 183 | 0.514 | 0.671   | 43.37%  | 211.69 | 6798 |

\*"flow-solve" here is the existing `SchedulerSolveTime` metric: coarse
graph build + `NetworkSimplex::run()` + Step 2 recovery + Step 3
coarse-to-fine lift (lifting runs unconditionally since
`ai/guide_path_metric.md`, so this column isn't pure flow-solve time, but
Step 1 is cleanly excluded from it either way).

**Local matching is negligible at shallow levels and comes to rival (then
exceed, in the worst single call) the flow solve at deep levels.** This
mirrors the local:flow *match-count* ratio already documented in
`ai/local_node_matching.md` -- deeper coarsening means bigger top-level
nodes, so more agent/task pairs share a node and get resolved by the local
matcher instead of ever reaching the coarse flow graph, which shrinks
correspondingly. The two trends (time share, count ratio) move together.

Worst single scheduler call per level (always the first/coldest timestep,
where every agent is still flexible):

| level | local match (s) | flow-solve (s) | whole planner step (s) | local as % of planner step |
|---|---|---|---|---|
| 2 | 0.007 | 4.878 | 7.561 | 0.09% |
| 4 | 0.043 | 0.977 | 2.402 | 1.8% |
| 6 | 0.068 | 0.618 | 2.484 | 2.7% |
| 8 | 0.402 | 0.163 | 1.880 | **21.4%** |

At level 8, local matching alone costs more than double the flow solve in
that call, and eats over a fifth of the whole planner timestep -- though
still comfortably inside the default 1000ms `--planTimeLimit` at this agent
count.

## Would parallelizing Step 1 help?

**Mechanically, yes, cleanly** -- `compute_reduced_assignment`'s Step 1
calls `match_local_node_exact` once per top-level node that has both
agents and tasks, in a loop over `agents_by_node`. Each call builds its own
`lemon::ListDigraph` and solves it independently; nothing is read or
written across calls except each node's own slice of `assignments`, so the
per-node loop is embarrassingly parallel (e.g. a `std::async`/thread-pool
`parallel_for` over the map entries, one task per node, no locking needed
beyond appending each node's result pairs afterward).

**Whether it's worth doing depends entirely on level:**
- At shallow levels (2, 4), Step 1 is 0.01-0.05% of the combined
  local+flow time -- the flow solve (108.9s / 62.6s total across the run)
  is overwhelmingly the actual cost driver there. Parallelizing Step 1
  would be imperceptible; the coarse `NetworkSimplex` solve is what would
  need attention instead (out of scope here, see `ai/auto_benchmarking_scene_mp_4p_03.md`
  for why deep coarsening is preferred over shallow anyway).
- At deep levels (6, 8) it's a real, measurable fraction, and checking the
  per-node group-size distribution for level 8's worst call (360 node
  groups making up the 0.407s cold-start local-match time) shows the work
  is well spread, not dominated by one huge blob: agents-per-group ranged
  1-229 (mean 55.8, median 22), and the single largest group only cost
  0.0095s -- **about 2.3% of that call's total local-match time**. That's
  a favorable Amdahl's-law profile: no single serial group caps the
  achievable speedup, so parallelizing across the ~8 cores available on
  this machine could plausibly get close to a proportional (not just
  marginal) speedup on the local-matching portion specifically at these
  levels.
- Net effect at level 8, assuming a rough ~6-8x speedup on Step 1 only: the
  worst single call's local-match cost (0.402s) would drop to roughly
  0.05-0.07s, cutting that call's local+flow total from ~0.57s to
  ~0.21-0.23s, and its share of the whole 1.88s planner step from 21% to
  ~3%. Meaningful for that call, but this map/agent count never came close
  to timing out at 1000ms even unparallelized, so it wouldn't change
  correctness or throughput outcomes here -- it would matter more at
  higher agent counts, deeper levels (level 9 is the deepest built in this
  cache and wasn't measured), or denser/more-clustered maps, where Step
  1's absolute cost would scale up along the same trend this table shows.

**Recommendation**: not worth doing at the levels/scale exercised in this
sweep -- the flow solve (or, at deep levels, just overall headroom) isn't
actually constrained by Step 1 yet. Worth revisiting if a future sweep at
level 8-9 with a much larger agent count (or a more clustered map) shows
Step 1 pushing single calls close to `--planTimeLimit`; the group-size data
above suggests it would parallelize well when that becomes necessary.

## How to reproduce

```shell
./build/lifelong --inputFile instances/custom/scene_mp_4p_03/scene_mp_4p_03_20000.json \
  -o out.json --scheduleModel 6 --flowSolveLevel <2|4|6|8> -s 200 \
  --hierarchyCache outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_level9.hierarchy \
  --preprocessTimeLimit 150000
```

Then read `timeStepMetrics[i].SchedulerLocalMatchTime` /
`.SchedulerSolveTime` from the output JSON, de-duplicating consecutive rows
that share identical `(SchedulerSolveTime, SchedulerLocalMatchTime,
LocalNodeMatchCount, FlowMatchCount, PlannerTime)` before summing (the
catch-up double-push -- see above).
