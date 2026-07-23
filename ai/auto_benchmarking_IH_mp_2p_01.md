# Benchmark run: solver 1 vs solver 6 (coarsen depth 2 vs 4) on IH_mp_2p_01

> Detail file for one sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index, synthesis, and reusable methodology notes.

Run 2026-07-23. `IH_mp_2p_01.map` (1912x1800, ~3.44M cells, ~2.22M walkable —
roughly 2.3x `orz900d`'s ~978K cells) via user request, to see how the
solver-1-vs-solver-6 comparison changes on a much larger map, using instances
generated with the official 2024 LoRR `benchmark_generator.py` (see below)
rather than the ad hoc script used for `orz900d`.

## Goal

9 runs: 3 instance sizes x 3 configs — `--scheduleModel 1`, `--scheduleModel 6`
with `kDefaultCoarsenLevels = 2` (repo default), `--scheduleModel 6` with
`kDefaultCoarsenLevels = 4`. (Depth 1/3 were not requested for this map, unlike
the `orz900d` sweep which covered levels 1-4.)

## Instance generation

Cloned `https://github.com/MAPF-Competition/Benchmark-Archive.git` into the
repo root (gitignored — not committed) and used its official 2024 problem
generator rather than a bespoke script:

```shell
python3 "Benchmark-Archive/2024 Competition/Problem Generator/script/benchmark_generator.py" \
  --mapFile instances/custom/maps/IH_mp_2p_01.map \
  --revealNum 1.5 --problemName IH_mp_2p_01 --taskNum 30000 \
  --teamSizes 5000 10000 20000 \
  --benchmark_folder instances/custom/IH_mp_2p_01 \
  --minEPT 1 --maxEPT 4
```

Needed `easydict`, `pyyaml`, `numpy`, `tqdm` (`pip install --user
--break-system-packages ...` — the box's Python is externally-managed).
Produces one shared 30,000-task pool plus a separate `.agents` file per team
size, all pointing at the same map/task files — see
`instances/custom/IH_mp_2p_01/IH_mp_2p_01_<agents>.json`.

## Flags used (identical across all 9 runs)

- `-s 200` (matches the `orz900d` sweep).
- `--planTimeLimit` left at the default (1000ms) — same reasoning as the
  `orz900d` sweep: this is the previously-validated-safe regime for solver
  1's own unbounded-memory-growth bug (see `ai/auto_benchmarking_orz900d.md`
  "Issues"). Not re-verified independently on this map, but no OOM was
  observed in any of the 9 runs.
- **`--preprocessTimeLimit 600000`** (10 minutes) — a real deviation from the
  `orz900d` sweep's default 30000ms, forced by a finding below (hierarchy
  build alone takes ~5 minutes on this map, for *every* solver).
- No `--useTraffic`, no `--assignNew`/`--commitWindow` overrides — same as
  `orz900d`.

## Finding: hierarchy build is mandatory for every solver, and it's slow here

`schedule_initialize()` (`default_planner/scheduler.cpp:63`) unconditionally
calls `MapReductionTest::ReducedHierarchy::instance().ensure(env)` regardless
of `--scheduleModel` — **solver 1 pays this cost too**, even though it never
uses the hierarchy. This wasn't visible on `orz900d` (978K cells) because it
was fast enough there to go unnoticed; on this ~2.3x larger map it measured
consistently at **~300-305 seconds** across all 6 solver-6 runs (see
`schedulerHierarchyBuildTime` in each output JSON — this field is only
populated by solver 6's `set_last_reduced_timing`; solver 1 always reports
`0.0` via `set_last_timing` even though it silently pays the same ~5-minute
tax during preprocessing). At the default `--preprocessTimeLimit 30000`, this
made **every single run** (including solver 1) fail immediately with a fatal
"Preprocessing timeout" — confirmed directly on a 50-agent smoke-test
instance before committing to the real sweep. Raising `-p` to 600000 fixed
this uniformly.

This is a real, general repo fact (not sweep-specific) — noted in
`ai/project_context.md`'s "Solver 6" section too. Worth remembering for any
future benchmark on a map bigger than `orz900d`: budget several minutes of
preprocessing per run, for every solver, regardless of which one is actually
being tested.

## Finding: solver 1's per-timestep flow solve is drastically more expensive here

Direct calibration via `./build/map_reduction_test` on the map (bypassing the
full simulation loop) measured a single `schedule_plan_flow` call — solver
1's full-map min-cost-flow solve — at **~35 seconds average, with only 50
agents and 200 tasks**. This is a property of map size, not agent count:
solver 1 builds one node per map cell and one arc per adjacent walkable pair
regardless of how many agents/tasks exist, so the graph is ~2.3x bigger than
`orz900d`'s and the solve is far slower. (For comparison, `compute_reduced_assignment`
in the same harness measured ~24s/call *with* the fine-grained lifting
included — Steps 3-4, which don't run in the real sweep since `--useTraffic`
is off; solver 6's actual per-timestep cost in these runs is just the tiny
top-level flow solve, effectively free by comparison — see results below.)

Practical consequence for interpreting `makespan`/`steps recorded`: solver
1's `plan()` calls take so long that `timeout_timesteps` accumulates rapidly
(polling every default 1000ms), so **each of the 3 solver-1 runs only
recorded 10-12 real scheduling decisions ("steps recorded") to cover the full
201-timestep `makespan`** — i.e. each real decision, once it finally
completes, gets replayed across ~17-20 simulated ticks of "the agents were
just waiting" before the next decision starts. Solver 6 needed 90-124 real
decisions to cover the same 201 ticks — much more responsive, as expected
from a scheduler whose per-timestep cost doesn't scale with map size.

## Results

Generated via `visualisation/compute_throughput_metrics.py
outputs/IH_mp_2p_01_solver_comparison/`.

| file | agents | tasksFinished | steps recorded | makespan | tp/steps | tp/makespan |
|---|---|---|---|---|---|---|
| IH_mp_2p_01_5000_solver1.json | 5000 | 195 | 12 | 201 | 16.250 | 0.970 |
| IH_mp_2p_01_5000_solver6_depth2.json | 5000 | 1436 | 108 | 201 | 13.296 | 7.144 |
| IH_mp_2p_01_5000_solver6_depth4.json | 5000 | 1377 | 98 | 201 | 14.051 | 6.851 |
| IH_mp_2p_01_10000_solver1.json | 10000 | 669 | 12 | 201 | 55.750 | 3.328 |
| IH_mp_2p_01_10000_solver6_depth2.json | 10000 | 3234 | 124 | 201 | 26.081 | 16.090 |
| IH_mp_2p_01_10000_solver6_depth4.json | 10000 | 3023 | 95 | 201 | 31.821 | 15.040 |
| IH_mp_2p_01_20000_solver1.json | 20000 | 1775 | 10 | 201 | 177.500 | 8.831 |
| IH_mp_2p_01_20000_solver6_depth2.json | 20000 | 6851 | 110 | 201 | 62.282 | 34.085 |
| IH_mp_2p_01_20000_solver6_depth4.json | 20000 | 6323 | 90 | 201 | 70.256 | 31.458 |

0 planner errors / 0 schedule errors / 0 entry timeouts across all 9 runs.
`schedulerHierarchyLevelNodeCounts`: depth 2 → `[3441600, 561933, 143752]`
(3 levels); depth 4 → `[3441600, 561933, 143752, 37640, 10201]` (5 levels —
the first 3 levels are identical to depth 2, as expected since coarsening is
the same fixed halving process, `kDefaultCoarsenLevels` just adds more passes
on top).

**Read `tp/makespan`, not `tp/steps`, for the same reason as the `orz900d`
sweep** — solver 1's `tp/steps` here is wildly inflated (up to 177.5 at
20000 agents) purely because "steps recorded" so badly undercounts real
elapsed time when a single decision takes ~35-100s+ to compute; `tp/makespan`
is the real-time-accurate comparison.

### Headline finding: the solver-1-vs-solver-6 relationship inverts vs. orz900d

On `orz900d`, solver 1 beat every solver-6 coarsen level on `tp/makespan` at
every agent count (see `ai/auto_benchmarking_orz900d.md`). **On this ~2.3x
bigger map, solver 6 wins by a wide margin instead**:

- 5000 agents: solver6-depth2 **7.14** vs solver1 **0.97** tasks/timestep (~7.4x)
- 10000 agents: solver6-depth2 **16.09** vs solver1 **3.33** (~4.8x)
- 20000 agents: solver6-depth2 **34.09** vs solver1 **8.83** (~3.9x)

This is exactly the scaling behavior the thesis's map-coarsening approach is
meant to demonstrate: solver 1's cost is tied to total map size (rebuilds a
full-map flow graph every timestep, independent of agent count), while
solver 6's per-timestep cost is tied to the small coarse top-level graph, not
map size. On a small enough map (`orz900d`) that fixed solver-1 cost is still
cheap enough to win; cross some map-size threshold, and it doesn't. This
sweep is the first direct evidence in this repo of that crossover.

Depth 2 vs depth 4, within solver 6: depth 2 consistently beats depth 4
(7.14 vs 6.85, 16.09 vs 15.04, 34.09 vs 31.46) — same direction as `orz900d`'s
level1 > level2 > level3 > level4 pattern, i.e. more coarsening trades a
modest amount of throughput for a (here, unmeasured — see gaps below) cheaper
solve. The depth-2-vs-4 gap (~4-8%) is far smaller than the solver-1-vs-6 gap
(~4-7x), consistent with `orz900d`'s finding that coarsen level is a
second-order effect compared to solver choice.

## Caveats / methodology gaps (not fully apples-to-apples)

- **Task pool differs from `orz900d`'s methodology**: this sweep uses one
  shared 30,000-task pool (`--taskNum 30000`, official generator default
  conventions) across all 3 team sizes; the `orz900d` sweep's ad hoc
  generator (`instances/custom/generate_random_for_orz900d.py`) sized the
  task pool per team size (`max(10000, 1.5*teamSize+100)`). Shouldn't affect
  the within-sweep solver-1-vs-6 comparison (same task file used for all 3
  configs at a given agent count) but means the two sweeps' absolute
  `tasksFinished` numbers aren't directly comparable to each other.
- **Per-timestep scheduler/guide-path solve-time breakdown wasn't captured
  for this sweep** the way it could have been — `SchedulerSolveTime` per
  step is in each run's `timeStepMetrics`, but wasn't pulled into this doc.
  Would let depth-2-vs-4's actual per-timestep cost difference be shown
  directly instead of only inferred from throughput.
- Solver 1's own unbounded-memory-growth bug (`ai/auto_benchmarking_orz900d.md`
  "Issues") was not independently re-verified on this map at an elevated
  `--planTimeLimit` — only the default (safe, per that prior finding) was
  used here.
- Total sweep wall-clock: ~75 minutes for all 9 runs + 2 recompiles
  (`kDefaultCoarsenLevels` 2 → 4 → back to 2), each individual run taking
  475-508 seconds regardless of solver — dominated by the ~300s mandatory
  hierarchy build in every case, not by the actual simulation.
