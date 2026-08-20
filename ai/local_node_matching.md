# Solver 6: local node matching (within-coarse-node distance-informed pairing)

Added 2026-08-13. Fixes the "within-coarse-node agent<->task pairing is
arbitrary, not distance-informed" gap in solver 6's per-timestep assignment
(`ReducedHierarchy::compute_reduced_assignment`, see `ai/project_context.md`'s
"Solver 6" section). See `ai/todo.md`'s Done entry for the full session-by-
session narrative (discovery, quantification, a corrected design draft, the
implementation, and a bug found mid-validation); this doc is the standalone
reference for the mechanism itself.

## Problem

`compute_reduced_assignment`'s Step 1 (`MapCoarsenV1.cpp:1437`) solves one
min-cost flow on the *coarse* graph only. When an agent's and a task's fine
locations map to the same top-level node (`--flowSolveLevel`), the arc
between them costs 0 — the flow solve has no distance signal to prefer one
fine-grained pairing over another among same-node candidates. The old
recovery walk just paired agents with same-node tasks in whatever order a
BFS over residual flow happened to visit them, which tracks insertion order,
not real fine-map distance.

This is distinct from, and doesn't fix, a second effect: pairs that *don't*
share a coarse node are still only scored via the coarse graph's aggregated
inter-node arc costs, not exact fine positions. See the quantification table
below for both effects measured side by side.

## Quantification

Standalone diagnostic `map_reduction_test/analyze_coarse_collisions.cpp`
(`./build/analyze_coarse_collisions <instance.json> <hierarchy_cache.bin>
[max_level] [task_cap] [min_level]`) re-solves the real top-level flow at
each level and compares the actual real (fine-grid Manhattan) distance of
the resulting assignment against the *exact* minimum-cost real-distance
matching (LEMON `NetworkSimplex` on true distances) for the same batch,
split by same-node vs. different-node:

| level | same-node pairs | same-node waste (total / per-pair) | diff-node pairs | diff-node waste (total / per-pair) |
|---|---|---|---|---|
| 1 | 22 | 0 / 0.00 | 4978 | 2210 / 0.44 |
| 2 | 76 | 0 / 0.00 | 4924 | 2512 / 0.51 |
| 3 | 317 | 12 / 0.04 | 4683 | 3952 / 0.84 |
| 4 | 1074 | 58 / 0.05 | 3926 | 7822 / 1.99 |
| 5 | 2576 | 3542 / 1.38 | 2424 | 14808 / 6.11 |
| 6 | 3993 | 45938 / 11.50 | 1007 | 13130 / 13.04 |

(`scene_mp_4p_03`, 5000 agents/5000 tasks.) Same-node waste is ~0 at shallow
levels and explodes from level 4 onward — by level 6 it's the majority of
total measured waste, tracking the throughput crossover found in
`ai/auto_benchmarking_scene_mp_4p_03.md`.

## What was added

- **`map_reduction_test/LocalNodeMatch.{h,cpp}`** — `match_local_node_exact()`:
  exact minimum-cost bipartite matching (LEMON `NetworkSimplex`) on real
  fine-grid Manhattan distance between a small group of agents and tasks.
  Exposed as a `LocalNodeMatcher` (`std::function`) typedef so the matching
  strategy is swappable without touching the surrounding flow-graph code —
  no other implementation is wired in yet, but the seam exists.
- **`compute_reduced_assignment`'s Step 1** (`MapCoarsenV1.cpp:1486` on):
  agents/tasks are bucketed by top-level node (as before), but before any
  supply/demand arc is added to the flow graph, every node with **both**
  agents and tasks present has its whole group pulled out and matched
  directly via `match_local_node_exact`. The resulting pairs go straight
  into `assignments`; only the `|agents - tasks|` leftover at each node
  (never both sides at once, since an exact `min(a,t)`-pair matching is
  used) becomes the surplus that flows into the unchanged coarse flow graph
  below. `num_workers` (the flow's source/sink supply magnitude) now counts
  only that surplus, mirroring what the original code already did with the
  *total* flexible-agent count — the agent/task-count gap fed into the flow
  is unchanged from before this pass existed, not newly introduced by it.
- **`constexpr bool kEnableLocalNodeMatching`** (`MapCoarsenV1.cpp:51`,
  default `true`) — compile-time toggle, same style as
  `kDefaultCoarsenLevels`/`kDefaultFlowSolveLevel` right above it. Flip to
  `false` and rebuild (`./compile.sh`, or `make lifelong` for an incremental
  rebuild) to fall back to the pre-fix behavior for an A/B comparison; no
  other code changes needed, since the matcher call site collapses to
  returning zero pairs and every agent/task falls through to the flow graph
  exactly as it did before.

### Why this is lossless *to the coarse flow's own cost function* (with one caveat)

Same-node supply/demand arcs cost 0 and real inter-node moves cost >0, so a
min-cost flow was always going to match `min(a,t)` pairs locally for free at
each node regardless of which specific agent/task got picked — pulling the
whole group out and matching it properly doesn't change the flow's optimal
*coarse-cost* total, it just replaces an arbitrary pairing (and an arbitrary
local/surplus split) with a distance-informed one.

The one caveat, found empirically during validation (not just theoretical):
this is **not** lossless on *total realized real distance* including the
diff-node group. Which specific agents become "surplus" changes depending on
which ones the local matcher keeps, and the coarse flow's own diff-node
routing is itself only cost-optimal in coarse-graph-cost terms, not real
distance — so a different local/surplus split can occasionally do worse
end-to-end on the diff-node side. Measured impact: small (see below), and
dwarfed by the same-node gain at the levels that matter.

### Deliberately deferred: guide-path lifting for locally-matched pairs

Locally matched pairs never touch the coarse graph, so `compute_reduced_assignment`'s
Steps 3-4 (coarse-to-fine lift) don't produce a guide path for them. They
currently fall back to the low-level planner's own seed (`update_traj`/`astar`
in `flow.cpp`), same as any agent with no `agent_guide_path` entry. This only
matters when `--useTraffic` is on (the only time `agent_guide_path` is
consumed at all — see `ai/project_context.md`'s "Guide paths" section).
Tracked as an open follow-up in `ai/todo.md`, not implemented.

## Validation

### Bug found mid-validation

The first implementation of `match_local_node_exact` read matched pairs via
`flow[arc]`, indexing the external LEMON `ArcMap` passed to
`NetworkSimplex::flowMap()` directly. That map is not reliably populated
post-solve in this LEMON version, so the function silently returned zero
matches on every call — `ns.run()` still correctly reported `OPTIMAL`,
masking the bug (the first validation pass, before this was caught, was
unknowingly comparing the pipeline against itself). Fixed to read via
`ns.flow(arc)` (the solver's own accessor), matching the pattern already
used elsewhere in this codebase (`run_top_level_flow_and_recover` in the
diagnostic tool, and `compute_reduced_assignment` itself).

### Per-timestep realized-distance validation (diagnostic tool)

`analyze_coarse_collisions.cpp` was extended with a
`solve_top_level_assignment_with_local_match()` path that reproduces the
fixed Step 1/2 pipeline standalone, plus new CSV columns
(`fixed_matched`/`fixed_total_dist`/`total_dist_before_fix`/
`total_waste_eliminated_by_fix`) comparing total realized real distance
before vs. after the fix — not just the theoretical same-node-only optimum
in the quantification table above. Re-run on the same `scene_mp_4p_03`
5000-agent/5000-task batch:

| level | same-node excess (theoretical max gain) | total waste eliminated by fix |
|---|---|---|
| 1 | 0 | -962 |
| 2 | 0 | -1760 |
| 3 | 0 | -786 |
| 4 | 6 | +72 |
| 5 | 1586 | +4768 |
| 6 | 28010 | +50596 |

Small regression (<0.6% of total distance) at shallow levels where there's
no same-node waste to offset the diff-node-routing caveat above; large net
win at levels 5-6, where the realized level-6 gain (50596) actually
*exceeds* the naive same-node-only estimate (28010) — the fix also improves
diff-node routing as a side effect there.

### Full-simulation validation

Real `./build/lifelong --scheduleModel 6` runs, `scene_mp_4p_03`, `-s 500`
(or `-s 250` at 5000 agents for the first check), same `--hierarchyCache`
throughout, **zero planner/schedule/timeout errors on every run**. Pre-fix
built via a separate git worktree at the last committed commit (this fix was
never committed mid-A/B-testing); post-fix via `kEnableLocalNodeMatching = true`.

Tasks finished, no tie-breaker vs. with tie-breaker (levels 4-6 only —
levels 1-3 have ~0 same-node waste at every agent count tested, see
quantification table):

| agents | level | no tie-breaker | with tie-breaker | delta | delta% |
|---|---|---|---|---|---|
| 5000 | 6 | 1463 | 1517 | +54 | +3.69% |
| 10000 | 4 | 3182 | 3178 | -4 | -0.13% |
| 10000 | 5 | 3151 | 3166 | +15 | +0.48% |
| 10000 | 6 | 3054 | 3110 | +56 | +1.83% |
| 20000 | 4 | 6971 | 7056 | +85 | +1.22% |
| 20000 | 5 | 6803 | 6972 | +169 | +2.48% |
| 20000 | 6 | 6518 | 6986 | **+468** | **+7.18%** |

(5000-agent level 4/5 deltas not separately re-run at `-s 500` — the
per-timestep diagnostic table above already covers that scale in more
detail than a single full-sim run would add.)

**The benefit grows with agent count**, as expected: more agents packed into
the same coarse nodes means denser same-node collisions. Level 4/5 deltas
that were noise-sized at 5000 agents become clearly positive at 20000.
Level 6 goes from +3.7% (5000 agents) to +7.2% (20000 agents).

Most notable finding: **pre-fix, level 6 was the worst of levels 4-6 at
every agent count tested** — going deeper than level 5 cost throughput, not
just failed to help. Post-fix, level 6 at 20000 agents (6986 tasks) actually
overtakes level 5 (6972) and nearly matches level 4 (7056) — the fix doesn't
just add tasks in isolation, it specifically undoes the "deeper coarsening
starts hurting" regression this map showed pre-fix at its deepest level.

Full sweep outputs (10k/20k agents, both A/B arms, plus calibration probes)
in `outputs/scene_mp_4p_03_10k_20k_localmatch_comparison/`; 5000-agent sweep
outputs in `outputs/scene_mp_4p_03_solver_comparison/` (pre-fix levels 1-6:
`..._level{1..6}.json`; post-fix levels 4-6: `..._level{4,5,6}_postfix.json`).

## Usage

No CLI flag — this is always on by default (`kEnableLocalNodeMatching = true`
in `MapCoarsenV1.cpp`). To disable for an A/B comparison:

```shell
sed -i 's/kEnableLocalNodeMatching = true/kEnableLocalNodeMatching = false/' \
  map_reduction_test/MapCoarsenV1.cpp
./compile.sh   # or: cd build && make lifelong
# ... run comparisons ...
sed -i 's/kEnableLocalNodeMatching = false/kEnableLocalNodeMatching = true/' \
  map_reduction_test/MapCoarsenV1.cpp
./compile.sh
```

Behaves identically to before this feature existed when disabled — the
matcher call site returns zero pairs and every agent/task falls through to
the coarse flow graph unchanged.

## Metrics: local-match vs. flow-match counts

Added 2026-08-14. Every `./build/lifelong --scheduleModel 6` run now reports
how many agents each timestep were assigned via the local matcher above vs.
via the coarse flow solve (Step 2), threaded the same way as
`GuidePathLengthSum`/`GuidePathCostSum` (`compute_reduced_assignment` out-params
-> `ScheduleTiming` -> `TimeStepMetric`, see `ai/project_context.md`'s
"Guide paths" plumbing for the pattern). Solver-6-only — always 0/0 for other
solvers, which have no local-matching step.

Output JSON fields:
- Per-timestep, in each `timeStepMetrics[i]` entry: `LocalNodeMatchCount`,
  `FlowMatchCount` (that timestep's split), plus `LocalNodeMatchCountCumulative`/
  `FlowMatchCountCumulative` (running totals from the start of the run through
  that timestep, inclusive).
- Run-level: `totalLocalNodeMatchCount`/`totalFlowMatchCount` (top-level
  fields, equal to the last timestep's cumulative values).

Note only *flexible* (unassigned / re-offerable) agents ever reach Step 1 each
timestep — already-opened-but-unassigned tasks are pinned directly (see
"Problem" above) and never touch either matcher, so most timesteps after the
first show 0/0 with the totals climbing only as new tasks/agents free up.

### Bug found + fixed while verifying: double-counted running total

The first live sweep (below) surfaced a real bug in the running-total
accumulation, caught before it was trusted: `BaseSystem::plan()`
(`CompetitionSystem.cpp`) can push **two** `TimeStepMetric` entries for one
real scheduler call whenever that call is slow enough to exceed
`--planTimeLimit` — a "catch-up" entry plus a "normal" entry, both copied
from the same `last_scheduler_timing` (see `ai/project_context.md`,
"Makespan vs. 'timesteps solved'"). The running-total accumulator originally
added `LocalNodeMatchCount`/`FlowMatchCount` at *each* push site, so a single
slow call got counted twice. Fixed by moving the accumulation to run exactly
once per `plan()` call (right after it returns, before either push site),
with both push sites now just reading the already-updated running total.
Caught by comparing consecutive `timeStepMetrics` entries with identical
`PlannerTime` (a sure sign of a duplicate push) in a first-draft sweep on
`orz900d`, where the very first timestep's cold hierarchy-cache load pushed
total time past the default 1000ms `--planTimeLimit` on 4 of 6 runs — before
the fix this inflated `totalLocalNodeMatchCount` by as much as ~1.7x on the
worst-affected run.

### Verification sweep (`orz900d`, `outputs/orz900d_local_match_counter_test/`)

2026-08-14. 6 runs, `orz900d` (656x1491, ~978K walkable cells),
`--scheduleModel 6`, `--flowSolveLevel {2,4,6}` x `{10000,20000}` agents,
`-s 200`, shared `--hierarchyCache` (built once at `kDefaultCoarsenLevels =
9`, reused across all 6 — the cache only depends on the map, not agent
count). 0 planner/schedule/timeout errors on every run.

| file | agents | level | tasksFinished | totalLocalMatch | totalFlowMatch | local:flow |
|---|---|---|---|---|---|---|
| orz900d_10000_solver6_level2.json | 10000 | 2 | 3946 | 5880 | 8066 | 0.73 |
| orz900d_10000_solver6_level4.json | 10000 | 4 | 4013 | 10020 | 3988 | 2.51 |
| orz900d_10000_solver6_level6.json | 10000 | 6 | 4117 | 12122 | 1994 | 6.08 |
| orz900d_20000_solver6_level2.json | 20000 | 2 | 8220 | 13386 | 14828 | 0.90 |
| orz900d_20000_solver6_level4.json | 20000 | 4 | 8493 | 21637 | 6850 | 3.16 |
| orz900d_20000_solver6_level6.json | 20000 | 6 | 8601 | 25282 | 3315 | 7.63 |

Sanity-confirms the mechanism the counters are meant to expose: the
local:flow ratio climbs monotonically with `--flowSolveLevel` at both agent
counts (deeper coarsening -> bigger coarse nodes -> more same-node
collisions -> more work absorbed by the local matcher instead of the coarse
flow solve), and `tasksFinished` also climbs with level here — consistent
with the full-simulation validation above (the tie-breaker fix's benefit
growing with agent count/coarsening depth).

See `ai/local_node_matching_runtime.md` for the *runtime* (not just count)
breakdown between local matching and the flow solve — the same
match-count-ratio trend above, mirrored in wall-clock time, and an analysis
of whether parallelizing the local matcher is worth it.

### Second verification sweep (`scene_sp_pol_06`, `outputs/scene_sp_pol_06_local_match_counter_test/`)

2026-08-14. 6 runs, `scene_sp_pol_06` (4192x4328, ~9.94M walkable cells —
~10x `orz900d`), `--scheduleModel 6`, `--flowSolveLevel {4,6,8}` x
`{20000,60000}` agents, `-s 500`, `--preprocessTimeLimit 1800000`, reusing
the pre-built depth-9 hierarchy cache from
`ai/auto_benchmarking_scene_sp_pol_06.md`'s earlier sweep (no rebuild
needed). 0 planner/schedule/timeout errors on every run; total wall-clock
~43 minutes for the 6 runs.

| agents | level | tasksFinished | totalLocalMatch | totalFlowMatch | local:flow |
|---|---|---|---|---|---|
| 20000 | 4 | 6375 | 9601 | 16774 | 0.57 |
| 20000 | 6 | 6264 | 22913 | 3350 | 6.84 |
| 20000 | 8 | 6341 | 26241 | 100 | 262.41 |
| 60000 | 4 | 19834 | 49328 | 30505 | 1.62 |
| 60000 | 6 | 19848 | 77059 | 2789 | 27.63 |
| 60000 | 8 | 19805 | 79731 | 74 | 1077.45 |

Same monotonic-with-level trend as `orz900d`, far more pronounced: by level
8 the coarse flow solve handles almost nothing (74-100 pairs across the
whole 500-timestep run) and the local matcher absorbs nearly everything —
consistent with this map's much larger absolute size meaning even a "small"
top-level coarse node spans a huge region at deep levels, pushing most
agent/task pairs into same-node collisions. Also a much stronger test of the
double-count fix above: this map's heavier per-timestep guide-path lifting
cost means the timeout-catchup duplicate-push path (see "Bug found + fixed"
above) triggered far more often than on `orz900d` — 157-250 duplicated
`timeStepMetrics` entries per run, out of only 300-500 total recorded steps
— yet every run's cumulative-vs-total consistency and monotonicity checks
still passed.
