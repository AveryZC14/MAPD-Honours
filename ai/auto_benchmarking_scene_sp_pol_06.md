# Benchmark run: solver 6 flowSolveLevel sweep (2/4/6/8) on scene_sp_pol_06

> Detail file for one sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index, synthesis, and reusable methodology notes.

Run 2026-08-13/14 (overnight, unattended). First sweep on `scene_sp_pol_06`
(user-imported map, referred to as "pol_06" for short — the file is named
`scene_sp_pol_06.map`, not `scene_mp_pol_06.map` as originally described;
confirmed with the user this is the intended map). **Solver 1 was not run**
— this sweep was solver-6-only by request, unlike every prior
`ai/auto_benchmarking_*.md` sweep, so there's no solver-1 baseline row here.

## Map + instance generation

`instances/custom/maps/scene_sp_pol_06.map`, 4192x4328 (18,142,976 total
cells, 9,939,841 walkable, 54.8%) — bigger than every previously-benchmarked
map except `scene_mp_4p_03` (13.9M cells) and `scene_sp_endmaps` (30.5M
cells, see `ai/auto_benchmarking_scene_sp_endmaps.md`).

Generated with the official 2024 LoRR `benchmark_generator.py`
(`RandomBenchmarkGenerator`, no `--taskFile`), same convention as every prior
sweep's instance generation:

```shell
python3 "Benchmark-Archive/2024 Competition/Problem Generator/script/benchmark_generator.py" \
  --mapFile instances/custom/maps/scene_sp_pol_06.map \
  --revealNum 1.5 --problemName scene_sp_pol_06 --taskNum 135000 \
  --teamSizes 10000 20000 40000 60000 90000 \
  --benchmark_folder instances/custom/scene_sp_pol_06 \
  --minEPT 1 --maxEPT 4
```

`--taskNum 135000` (vs. the `30000` used for `IH_mp_2p_01`/`scene_mp_4p_03`)
was sized to ~1.5x the largest agent count generated here (90000) rather
than reusing the older maps' fixed 30000, since 40000-90000 agents against
only 30000 tasks would put every run at or past the leftover-tasks-<
leftover-agents condition documented in `ai/todo.md` ("Coarse-flow
all-or-nothing infeasibility..."). Only 10000/20000/60000 were actually
*run* this session (see below); 40000/90000 instances exist for future use.
Took ~85 minutes wall-clock to generate all 5 team sizes + the task pool
(dominated by `find_lcc`'s O(n) list-based BFS over 18.1M cells and
`generate_agents`' O(n²) duplicate-start-location check — both from the
unmodified upstream generator script, not something this session's work
touched).

## Hierarchy build + smoke test

`kDefaultCoarsenLevels` is already `9` in the working tree (left over from
prior session's `local_node_matching` work), so no recompile was needed to
reach `--flowSolveLevel 8` — the hierarchy just needed to actually be built
that deep for this map, which happens once and gets cached to disk.

```shell
./build/lifelong --inputFile instances/custom/scene_sp_pol_06/scene_sp_pol_06_10000.json \
  -o outputs/scene_sp_pol_06_solver_comparison/scene_sp_pol_06_smoketest.json \
  --scheduleModel 6 --flowSolveLevel 8 -s 5 \
  --preprocessTimeLimit 1800000 \
  --hierarchyCache outputs/scene_sp_pol_06_solver_comparison/scene_sp_pol_06_level9.hierarchy
```

Clean, `numPlannerErrors`/`numScheduleErrors` 0. **Fresh hierarchy build at
depth 9 (10 levels, 0=fine..9=coarsest): 94.5s**
(`schedulerHierarchyBuildTime`), roughly in line with `scene_mp_4p_03`'s
67.7s at depth 6 on a ~1.3x-smaller map — consistent with the
`ai/solver6_preprocessing_efficiency.md` CPU fixes still holding up at this
scale. Per-level node counts: `[18142976, 2517697, 645370, 169801, 46380,
13405, 4196, 1490, 631, 334]`. Hierarchy cache file:
`outputs/scene_sp_pol_06_solver_comparison/scene_sp_pol_06_level9.hierarchy`
(~2.4GB), **kept on disk** (not deleted after the sweep) so future runs
against this map skip the build entirely.

## Sweep

10000/20000/60000 agents (40000/90000 instances exist but weren't run,
matching what was asked for this sweep) x `--flowSolveLevel` 2/4/6/8, solver
6 only, `-s 500`, `--preprocessTimeLimit 1800000`, all reusing the one
cached hierarchy above (`kEnableLocalNodeMatching = true`, the current
default — see `ai/local_node_matching.md`/`ai/todo.md`):

```shell
for TEAM in 10000 20000 60000; do
  for LEVEL in 2 4 6 8; do
    ./build/lifelong --inputFile instances/custom/scene_sp_pol_06/scene_sp_pol_06_${TEAM}.json \
      -o outputs/scene_sp_pol_06_solver_comparison/scene_sp_pol_06_${TEAM}_solver6_level${LEVEL}.json \
      --scheduleModel 6 --flowSolveLevel $LEVEL -s 500 \
      --preprocessTimeLimit 1800000 \
      --hierarchyCache outputs/scene_sp_pol_06_solver_comparison/scene_sp_pol_06_level9.hierarchy
  done
done
```

All 12 runs completed cleanly (`numPlannerErrors`/`numScheduleErrors`/
`numEntryTimeouts` all 0), total wall-clock for the 12 runs ~1h34m.

### Throughput results

Generated via `python3 visualisation/compute_throughput_metrics.py
outputs/scene_sp_pol_06_solver_comparison/`:

| agents | level | tasksFinished | steps recorded | makespan | tp/makespan |
|---|---|---|---|---|---|
| 10000 | 2 | 2961 | 172 | 500 | 5.922 |
| 10000 | 4 | 3153 | 307 | 501 | 6.293 |
| 10000 | 6 | 3090 | 498 | 500 | 6.180 |
| 10000 | 8 | 3145 | 499 | 500 | 6.290 |
| 20000 | 2 | 6166 | 168 | 501 | 12.307 |
| 20000 | 4 | 6375 | 325 | 501 | 12.725 |
| 20000 | 6 | 6270 | 500 | 501 | 12.515 |
| 20000 | 8 | 6335 | 501 | 501 | 12.645 |
| 60000 | 2 | 19742 | 156 | 501 | 39.405 |
| 60000 | 4 | 19837 | 336 | 501 | 39.595 |
| 60000 | 6 | 19848 | 500 | 501 | 39.617 |
| 60000 | 8 | 19805 | 498 | 501 | 39.531 |

### Reading

- **Throughput scales ~linearly with agent count** (`tp/makespan` ≈ 0.66x
  agent count at every level tested: 5.9-6.3 at 10k, 12.3-12.7 at 20k,
  39.4-39.6 at 60k) — no sign of the coarse-flow saturating or degrading at
  60000 agents against a 135000-task pool.
- **Level has only a small, non-monotonic effect here** — unlike
  `scene_mp_4p_03` (`ai/auto_benchmarking_scene_mp_4p_03.md`), where depth
  helped monotonically (level 4 clearly beat level 1), pol_06's
  `tp/makespan` moves within a ~6% band across levels 2/4/6/8 at every agent
  count, with level 4 usually slightly ahead and level 2 usually slightly
  behind, but no clean monotonic trend in either direction. Plausibly this
  map's larger absolute size (vs. `scene_mp_4p_03`) means even the level-2
  top-level graph is already cheap enough to solve per-timestep that going
  coarser doesn't buy much further — but this is one data point, not
  confirmed against a controlled comparison.
- `numPlannerErrors`/`numScheduleErrors`/`numEntryTimeouts` all 0 across
  every run — no correctness issues surfaced at this scale.
- **No solver-1 comparison** — out of scope for this sweep by request. Given
  `scene_mp_4p_03` (a smaller map) already showed solver 1 barely
  functioning (17 tasks finished in 501 timesteps, 6 real scheduling
  decisions), solver 1 would very likely be even less competitive on this
  ~1.3x-bigger map, but that's not been verified here.

## Coarsening visualisation

`./build/dump_coarsening instances/custom/scene_sp_pol_06/scene_sp_pol_06_10000.json <prefix> 9`
followed by `map_reduction_test/visualisation/plot_coarsening.py` at
`--levels 1,2,3,4,5,6,7,8,9` with `--per-level-output-dir`, all 9 levels,
full-map (no crop — see "Known gaps" below).

Output: `outputs/coarsening_viz/scene_sp_pol_06_l9/scene_sp_pol_06_fullmap.png`
(composite, all 9 levels + fine map) and
`outputs/coarsening_viz/scene_sp_pol_06_l9/per_level/` (one PNG per level).
Per `ai/coarsening_visualisation.md`'s prior finding on large maps, the
full-map render at this cell count is likely too fine-grained to make
individual components legible by eye — no `--crop` region was picked for
this map (would need visual inspection to choose a meaningful sub-rectangle,
not done as part of this unattended run). The intermediate
`_partition_level*.csv`/`_nodes.csv`/`_edges.csv` files (~1GB combined) were
deleted after rendering to protect disk space — they're cheaply
reproducible from the kept `.hierarchy` cache + `dump_coarsening` if needed
again.

## Known gaps / follow-up

- No `--crop` region rendered for legibility (see above) — worth doing if
  the full-map render turns out to be as unreadable as precedent predicts.
- No solver-1 baseline (by request, this sweep).
- 40000/90000-agent instances exist but weren't run.
