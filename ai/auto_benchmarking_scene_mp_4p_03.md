# Benchmark run: solver 1 vs solver 6 (coarsen levels 1-4, via new `--flowSolveLevel` lever) on scene_mp_4p_03

> Detail file for one sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index, synthesis, and reusable methodology notes.

Run 2026-08-12, 5000-agent instance only (10000/20000 not yet run — see
"Follow-up" below). First sweep to use the new `--flowSolveLevel` CLI flag
(`src/driver.cpp`, added in commit `5c9ee1a`) instead of the old
`sed` + recompile-per-level workflow every prior sweep used.

## Map + instance

`instances/custom/maps/scene_mp_4p_03.map`, 3728x3728 (13,897,984 total
cells, 6,297,149 walkable) — the largest map benchmarked in this repo so
far: ~4.04x `IH_mp_2p_01`'s total cell count (3,441,600) and ~2.84x its
walkable cell count (2,217,759). Instance:
`instances/custom/scene_mp_4p_03/scene_mp_4p_03_5000.json` (5000 agents,
`numTasksReveal 1.5`, shared 30,000-task pool).

## What changed to make this sweep possible: `kDefaultCoarsenLevels` 4→6

`map_reduction_test/MapCoarsenV1.cpp:31` was bumped from `4` to `6` (7 total
hierarchy levels, 0=fine..6=coarsest) so the hierarchy is built deep enough
to contain every level `--flowSolveLevel` was asked to test (1-4), with
headroom to spare. Rebuilt via `./compile.sh` (~18s, clean).

This is the first sweep where comparing solve-levels did **not** require a
recompile between runs — `--hierarchyCache` (see `ai/hierarchy_cache.md`)
was pointed at one shared file so the hierarchy is built once (first run)
and loaded from disk by the other three, and `--flowSolveLevel <N>` picks
which already-built level to solve the per-timestep flow on. Both levers
landed together in commit `5c9ee1a`, previously unused in a real sweep.

## Commands

All 5 runs, same instance, `-s 500`, `--preprocessTimeLimit 1800000` (sized
very conservatively before knowing actual build time — see below, could be
far smaller next time), default `--planTimeLimit` (1000ms — the
previously-validated-safe regime for solver 1's own unbounded-memory-growth
bug), no `--useTraffic`/`--assignNew`/`--commitWindow` overrides, run
serially (not parallel) on an otherwise-idle 8-core/31GB box:

```shell
CACHE=outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_level6.hierarchy
for LEVEL in 1 2 3 4; do
  ./build/lifelong --inputFile instances/custom/scene_mp_4p_03/scene_mp_4p_03_5000.json \
    -o outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_5000_solver6_level${LEVEL}.json \
    --scheduleModel 6 --flowSolveLevel $LEVEL -s 500 \
    --preprocessTimeLimit 1800000 --hierarchyCache "$CACHE"
done
./build/lifelong --inputFile instances/custom/scene_mp_4p_03/scene_mp_4p_03_5000.json \
  -o outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_5000_solver1.json \
  --scheduleModel 1 -s 500 --preprocessTimeLimit 1800000
```

## Preprocessing time (the thing specifically asked about)

| Run | `schedulerHierarchyBuildTime` | Wall-clock (build/load + 500 timesteps) |
|---|---|---|
| solver 6, level 1 (fresh build) | **67.7s** | 591s |
| solver 6, level 2 (cache load) | 28.5s | 516s |
| solver 6, level 3 (cache load) | 28.4s | 493s |
| solver 6, level 4 (cache load) | 0.0 — see anomaly note below | 467s |
| solver 1 (no hierarchy) | 0.0 (expected — solver 1 doesn't build one) | 528s |

**Fresh hierarchy build at depth 6 on this ~2.8x-bigger-than-anything-tested
map: 67.7 seconds.** Far faster than the conservative 30-minute
`--preprocessTimeLimit` budgeted for it — `IH_mp_2p_01` (smaller, shallower
hierarchy) needed ~300s pre-optimization, but the CPU-efficiency fixes in
`ai/solver6_preprocessing_efficiency.md` (commit `6535248`) evidently more
than absorbed both the bigger map and the deeper hierarchy. Cache load
(level 2/3) costs ~28.5s — cheaper than a fresh build but not free, since
the serialized hierarchy is 1.63GB
(`outputs/scene_mp_4p_03_solver_comparison/scene_mp_4p_03_level6.hierarchy`)
and has to be read + deserialized every process start. Per-level node
counts (levels 0-6, identical across the build and both cache-load runs —
confirms cache round-trip fidelity): `[13897984, 1599811, 412302, 109865,
30627, 9317, 3110]`.

**Anomaly**: the level-4 run's `schedulerHierarchyBuildTime` and
`schedulerHierarchyLevelNodeCounts` fields came back `0.0`/`[]` despite the
run completing normally (`Preprocessing success` in its log, 0 errors, wall
-clock in line with level 2/3). Not investigated further here — doesn't
affect throughput numbers below (those come from `tasksFinished`/`makespan`,
unaffected fields) — but worth a quick look if these fields get relied on
again; added to `ai/todo.md`.

`--preprocessTimeLimit` takeaway for next time on this map: 1.8M ms was
massive overkill against a real cost under 70s; something like 5-10 minutes
would be generous margin without wasting a detach-on-timeout risk window.

## Throughput results

Generated via `python3 visualisation/compute_throughput_metrics.py
outputs/scene_mp_4p_03_solver_comparison/`:

| file | agents | tasksFinished | steps recorded | makespan | tp/steps | tp/makespan |
|---|---|---|---|---|---|---|
| scene_mp_4p_03_5000_solver1.json | 5000 | 17 | 6 | 501 | 2.83 | 0.034 |
| scene_mp_4p_03_5000_solver6_level1.json | 5000 | 747 | 50 | 501 | 14.94 | 1.491 |
| scene_mp_4p_03_5000_solver6_level2.json | 5000 | 1355 | 136 | 501 | 9.96 | 2.705 |
| scene_mp_4p_03_5000_solver6_level3.json | 5000 | 1461 | 194 | 501 | 7.53 | 2.916 |
| scene_mp_4p_03_5000_solver6_level4.json | 5000 | 1580 | 404 | 501 | 3.91 | 3.154 |

`numPlannerErrors`/`numScheduleErrors`/`numEntryTimeouts` all 0 across every
run.

### Reading

- **Solver 6 wins by the widest margin seen in any sweep in this repo**:
  ~44x at level 1 up to ~93x at level 4 on `tp/makespan`, dwarfing
  `IH_mp_2p_01`'s ~4-7x and `warehouseXL`'s ~1.5-4.3x. Solver 1 barely
  functions on a map this size — only 6 real scheduling decisions
  (`len(timeStepMetrics)`) happened across 501 simulated timesteps, finishing
  just 17 tasks, because its from-scratch full-map (6.3M-walkable-node) flow
  solve is so slow it almost never completes within the per-timestep budget.
  Consistent with, and a further data point for, the "solver 6's advantage
  grows with map size" trend from `ai/auto_benchmarking.md`'s synthesis.
- **Coarsen depth is not a second-order effect here — it inverts the
  direction seen in every prior sweep.** `orz900d`/`IH_mp_2p_01`/
  `warehouseXL` all found deeper coarsening costs a small amount of
  throughput (level 1 best, level 4 worst). Here `tp/makespan` **increases
  monotonically** with level: 1.49 → 2.70 → 2.92 → 3.15. On a map this large,
  a shallower top-level graph (level 1: 1.6M nodes) is apparently still
  expensive enough to solve per-timestep that going coarser keeps paying off,
  unlike on the smaller maps tested before where the top-level graph was
  already cheap at level 1. Only one map/agent-count data point for this
  pattern so far — worth confirming at 10000/20000 agents before treating it
  as settled (see Follow-up).

## Follow-up (not done in this session)

- **10000/20000-agent instances for this map** — same instance family
  (`scene_mp_4p_03_10000.json`/`_20000.json` already exist), same shared
  `.hierarchy` cache reusable across all agent counts (cache validity keys
  off map signature only, not agents/tasks — see `ai/hierarchy_cache.md`).
  User asked to hold off pending this 5000-agent run's timing; total wall
  time for the 5-run 5000-agent sweep was ~43 minutes, suggesting 10000/20000
  are very feasible if wanted next.
- Level-4 `schedulerHierarchyBuildTime`/`schedulerHierarchyLevelNodeCounts`
  anomaly above.
- Levels 5-6 were never solved on (only the hierarchy was built that deep) —
  `--flowSolveLevel 5`/`6` would be free to test now given the cache already
  exists, and the monotonic-increase-with-depth finding above makes this a
  more interesting follow-up here than it would've been on any prior map.
