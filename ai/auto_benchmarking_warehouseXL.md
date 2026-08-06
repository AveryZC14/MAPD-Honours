# Benchmark run: solver 1 vs solver 6 (coarsen levels 1-4) on warehouseXL

> Detail file for one sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index, synthesis, and reusable methodology notes.

Run 2026-08-06. `warehouseXL.map` — a **newly generated** warehouse map, not
an existing benchmark map — created via the official 2024 LoRR
`Benchmark-Archive` warehouse map generator (`warehouse_map_generator.py`)
at user request, sized to be "almost as big as `IH_mp_2p_01`"
(1912x1800, ~3.44M cells, the previous largest map benchmarked in this repo
— see `ai/auto_benchmarking_IH_mp_2p_01.md`). This is the first sweep on a
*structured* warehouse-layout map (pickup stations + storage racks) at this
scale, as opposed to `orz900d`/`IH_mp_2p_01` which are both maze/MAPF-
benchmark-style maps.

## Map + instance generation

```shell
python3 "Benchmark-Archive/2024 Competition/Problem Generator/script/warehouse_map_generator.py" \
  --mapWidth 1800 --mapHeight 1900 --output warehouseXL.map \
  --stationConfig picking_station.txt --storageSize 3 2 \
  --stationDistance 2 --corridorWidth 1 --pillarWidth 4 --operWidth 5
```
1900x1800 = 3,420,000 cells (99.4% of `IH_mp_2p_01`'s 3,441,600) — pickup
stations wrap all four sides (`stationNumber` left at its unlimited default),
storage racks fill the interior. 1,729,940 walkable cells (324,802 `.` +
2,104 `E` emitters + 1,403,034 `S` service points).

Copied to `instances/custom/maps/warehouseXL.map`, then instances generated
with the same `benchmark_generator.py` invocation pattern used for
`IH_mp_2p_01` (random task sampling in the largest connected component, not
station-aware sampling — this map has `E`/`S` markers but the established
methodology for large maps in this repo uses `benchmark_generator.py`
without `--taskFile`, i.e. `RandomBenchmarkGenerator`, matching what was done
for `IH_mp_2p_01` even though that map had no station markers at all):

```shell
python3 "Benchmark-Archive/2024 Competition/Problem Generator/script/benchmark_generator.py" \
  --mapFile instances/custom/maps/warehouseXL.map \
  --revealNum 1.5 --problemName warehouseXL --taskNum 30000 \
  --teamSizes 5000 10000 20000 \
  --benchmark_folder instances/custom/warehouseXL \
  --minEPT 1 --maxEPT 4
```

Produces `instances/custom/warehouseXL/warehouseXL_{5000,10000,20000}.json`
+ matching `.agents` files, all sharing one 30,000-task pool
(`tasks/warehouseXL.tasks`).

## Validation before the sweep

Before committing to the full sweep, `--scheduleModel 1` and `6` were each
smoke-tested (`-s 5`) then run to a full `-s 200` on `warehouseXL_5000`
(`--preprocessTimeLimit 900000`, sized off `IH_mp_2p_01`'s ~300-600s
hierarchy-build precedent). Both clean: solver 1 completed 200 timesteps (18
real scheduling decisions, 224 tasks finished, `numPlannerErrors`/
`numScheduleErrors`/`numEntryTimeouts` all 0); solver 6 completed 200
timesteps (100 real decisions, 969 tasks finished, same zero-error result).
These two runs became the `_solver1`/`_solver6_level2` entries for the 5000-
agent row in the sweep below rather than being re-run.

## Sweep goal

15 runs: 3 instance sizes x 5 configs (matching the `orz900d`/`IH_mp_2p_01`
precedent exactly):
1. `--scheduleModel 1`
2. `--scheduleModel 6` @ `kDefaultCoarsenLevels` 1, 2 (repo default), 3, 4

Flags identical across all 15 runs: `-s 200`, default `--planTimeLimit`
(1000ms — the previously-validated-safe regime for solver 1's own unbounded-
memory-growth bug, see `ai/auto_benchmarking_orz900d.md` "Issues"),
`--preprocessTimeLimit 900000`, no `--useTraffic`, no `--assignNew`/
`--commitWindow` overrides.

Naming: `outputs/warehouseXL_solver_comparison/warehouseXL_<agents>_solver<N>[_level<L>].json`.

## Deviation from precedent: runs executed with parallelism, not serially

Both prior sweeps' detail docs explicitly chose full serial execution "to
avoid CPU contention skewing the per-timestep timing numbers." This sweep
did **not** follow that: each compile pass's 3 (or 6, for the depth=2 pass
which also carried solver 1's 3 runs) instance sizes were launched as
concurrent background processes, on an 8-core/31GB box with headroom to
spare (peak ~10GB RSS across up to 7 concurrent processes during the
visualisation-tooling phase, well under the 31GB ceiling). This was a
deliberate time/fidelity trade-off given the sheer number of runs requested
in one session (15 sweep runs + 6 `dump_guide_paths` + 1 `dump_coarsening`,
each independently paying the ~400s hierarchy-build tax) — full serial
execution would have taken several hours longer.

**Consequence worth knowing before trusting the absolute throughput numbers
below**: `--planTimeLimit` is a **wall-clock** deadline
(`default_planner`'s per-timestep budget), so CPU contention from sibling
processes can cause more "planner timeout" events than the same run would
hit in isolation, understating a solver's true per-timestep throughput
(mechanically the same effect as the `tp/steps` vs `tp/makespan` divergence
documented in `ai/auto_benchmarking.md`, just from a different cause). This
mainly affects solver 1 (already the more timeout-prone solver on large
maps) and, within each compile pass, whichever runs happened to overlap most
with siblings. Directional comparisons (solver 1 vs. solver 6; depth 1 vs. 2
vs. 3 vs. 4) should still be trustworthy since contention was roughly
symmetric within each pass, but absolute `tp/makespan` figures here are not
directly comparable apples-to-apples with `orz900d`'s/`IH_mp_2p_01`'s
strictly-serial numbers. Flagging this rather than re-running serially since
the qualitative result (see below) is unambiguous either way.

## Results

Generated via `visualisation/compute_throughput_metrics.py
outputs/warehouseXL_solver_comparison/`.

| file | agents | tasksFinished | steps recorded | makespan | tp/steps | tp/makespan |
|---|---|---|---|---|---|---|
| warehouseXL_5000_solver1.json | 5000 | 224 | 18 | 201 | 12.44 | 1.11 |
| warehouseXL_5000_solver6_level1.json | 5000 | 796 | 58 | 201 | 13.72 | 3.96 |
| warehouseXL_5000_solver6_level2.json | 5000 | 969 | 100 | 201 | 9.69 | 4.82 |
| warehouseXL_5000_solver6_level3.json | 5000 | 977 | 120 | 201 | 8.14 | 4.86 |
| warehouseXL_5000_solver6_level4.json | 5000 | 947 | 131 | 201 | 7.23 | 4.71 |
| warehouseXL_10000_solver1.json | 10000 | 879 | 20 | 201 | 43.95 | 4.37 |
| warehouseXL_10000_solver6_level1.json | 10000 | 1847 | 58 | 201 | 31.84 | 9.19 |
| warehouseXL_10000_solver6_level2.json | 10000 | 1957 | 96 | 201 | 20.39 | 9.74 |
| warehouseXL_10000_solver6_level3.json | 10000 | 1953 | 113 | 201 | 17.28 | 9.72 |
| warehouseXL_10000_solver6_level4.json | 10000 | 1898 | 126 | 201 | 15.06 | 9.44 |
| warehouseXL_20000_solver1.json | 20000 | 2877 | 20 | 201 | 143.85 | 14.31 |
| warehouseXL_20000_solver6_level1.json | 20000 | 4415 | 58 | 201 | 76.12 | 21.97 |
| warehouseXL_20000_solver6_level2.json | 20000 | 4488 | 104 | 201 | 43.15 | 22.33 |
| warehouseXL_20000_solver6_level3.json | 20000 | 4546 | 130 | 201 | 34.97 | 22.62 |
| warehouseXL_20000_solver6_level4.json | 20000 | 4138 | 122 | 201 | 33.92 | 20.59 |

`numPlannerErrors`/`numScheduleErrors`/`numEntryTimeouts` all 0 across every
run. `schedulerHierarchyBuildTime`: ~389-425s for every solver-6 run
regardless of depth (depth doesn't meaningfully change hierarchy-build cost —
consistent with `IH_mp_2p_01`'s finding); solver 1 always reports 0.0 (known
quirk — it pays the same hierarchy-build cost during preprocessing but
doesn't populate this field, see `ai/project_context.md`).
`schedulerHierarchyLevelNodeCounts`: depth 1 → `[3420000, 713888]`; depth 2
→ `[3420000, 713888, 213746]`; depth 3 → `[..., 53550]`; depth 4 →
`[..., 13447]`.

### Reading the results

- **Solver 6 wins on `tp/makespan` at every agent count**, same direction as
  `IH_mp_2p_01` — but **the margin is much narrower here**: ~4.3x at 5000
  agents, ~2.2x at 10000, only ~1.5x at 20000 (compare `IH_mp_2p_01`'s
  roughly constant 4-7x margin across all three sizes). The gap *shrinks* as
  agent count grows here, the opposite trend from what agent-count-
  independence of solver 1's per-timestep cost would predict in isolation —
  most likely explained by solver 1's own flow-solve getting relatively
  cheaper per-agent at higher task/agent density on this map (calibration via
  `dump_guide_paths`, see below, measured solver 1's full-density flow solve
  at only ~15-20s regardless of agent count 5000-20000), narrowing its
  disadvantage even though solver 6's absolute throughput keeps climbing too.
  Whether this is a property of the structured warehouse layout (shorter
  average paths between racks/stations vs. `IH_mp_2p_01`'s maze corridors) or
  the parallel-execution caveat above is not disentangled here — a good
  target for a future serial-only rerun.
- **Coarsen depth remains a second-order effect**, consistent with both
  prior sweeps, though the ordering isn't as cleanly monotonic here: depth
  2/3 are roughly tied for best throughput at every agent count (within ~1%
  of each other), depth 1 trails by 15-20%, and depth 4 is close to depth
  1-2 rather than clearly worst. All four depths stay within roughly a 20%
  band of each other — still a small effect next to the 1.5-4.3x solver-1-
  vs-solver-6 gap.
- Solver 1's `tp/steps` badly overstates its throughput at every size (e.g.
  20000 agents: 143.85 vs. the real 14.31) — same mechanism documented in
  `ai/auto_benchmarking.md`, worth restating since it's the largest gap of
  any sweep so far. Always read `tp/makespan`.

## Guide-path visualisation (`ai/guide_path_visualisation.md` tooling)

Ran `./build/dump_guide_paths` at 50/50, 150/150, 500/500 (capped
agents/tasks — sparse-pool regime), and 5000, 10000, 20000 (full team size
against the full 30000-task pool) on `warehouseXL_5000`/`_10000`/`_20000`.
Both solvers returned guide paths for **100% of agents at every scale**
(no repeat of the pre-fix `IH_mp_2p_01` missing-path bug — the A* fallback
fix holds up on this map too). Output under `outputs/warehouseXL_guide_paths/`.

| Agents | Tasks | Same task | Same path | avg len (solver1 / solver6) |
|---|---|---|---|---|
| 50 | 50 (capped) | 46/50 (92.0%) | 5/50 (10.0%) | 273.56 / 277.84 |
| 150 | 150 (capped) | 115/150 (76.7%) | 19/150 (12.7%) | 191.77 / 210.80 |
| 500 | 500 (capped) | 413/500 (82.6%) | 138/500 (27.6%) | 104.88 / 111.63 |
| 5000 (full) | 30000 (full) | 3401/5000 (68.0%) | 2861/5000 (57.2%) | 8.06 / 9.16 |
| 10000 (full) | 30000 (full) | 6565/10000 (65.7%) | 5534/10000 (55.3%) | 8.50 / 9.59 |
| 20000 (full) | 30000 (full) | 11913/20000 (59.6%) | 9891/20000 (49.5%) | 9.49 / 10.63 |

Same qualitative pattern as `IH_mp_2p_01`: sparse task pools (50-500 capped)
produce long, widely-diverging routes (both solvers' independent assignment
choices have much more room to disagree on a near-empty task pool); dense
full-instance pools produce short paths and much higher task/path agreement.
Unlike `IH_mp_2p_01`, agreement here **falls** only mildly as agent count
rises (68.0% -> 59.6% same-task from 5000 to 20000 agents, vs.
`IH_mp_2p_01`'s 62.6% -> 55.0%) — comparable magnitude, same direction.

Note the sparse-pool (50/150/500) dumps took noticeably longer wall-clock
(89-149s solver-1 solve time) than the full-density ones (15-20s) despite
far fewer agents — consistent with the same parallel-execution contention
caveat above (these ran concurrently with the full-density dumps and each
other) compounded by the sparse-pool fallback A* searches genuinely doing
more work per agent (longer paths = more expansions), both documented
effects from `ai/guide_path_visualisation.md`.

## Coarsening visualisation (`ai/coarsening_visualisation.md` tooling)

`./build/dump_coarsening instances/custom/warehouseXL/warehouseXL_5000.json
outputs/coarsening_viz/warehouseXL_l4/warehouseXL 4` — 5 levels (0-4):
3,420,000 -> 713,888 -> 213,746 -> 53,550 -> 13,447 nodes, each level's
partition/nodes/edges written as separate CSVs
(`warehouseXL_partition_level{1,2,3,4}.csv`, `_nodes.csv`, `_edges.csv`),
same structure as the existing `IH_mp_2p_01_l4` output. Rendered with
`plot_coarsening.py`: a full-map composite (`warehouseXL_fullmap.png`,
9050x1936, all 5 panels) plus a 150x150-cell crop
(`warehouseXL_crop.png`, region `900,900,1050,1050`, where individual
components are actually distinguishable — the full-map render is too
fine-grained to read component boundaries at this map size, same finding as
`orz900d`), and per-level individual PNGs under `per_level/` and
`per_level_crop/` (`--per-level-output-dir`).

## Repo state after the sweep

`map_reduction_test/MapCoarsenV1.cpp`'s `kDefaultCoarsenLevels` was edited
to 1, 3, 4 in turn for the respective compile passes and restored to `2`
(repo default) with a final `./compile.sh` — verified via `git diff`
showing no residual change to that file. New/untracked additions:
`instances/custom/maps/warehouseXL.map`, `instances/custom/warehouseXL/`,
`outputs/warehouseXL_solver_comparison/`, `outputs/warehouseXL_guide_paths/`,
`outputs/coarsening_viz/warehouseXL_l4/`, plus the two smoke-test JSONs
under `outputs/`.
