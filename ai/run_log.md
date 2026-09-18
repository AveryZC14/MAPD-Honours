# Run log

A running ledger of notable `./build/lifelong` runs/sweeps in this repo,
especially ad hoc or overnight ones that don't otherwise land anywhere
indexed. Started 2026-09-17, prompted by the `IH_mp_2p_01_10000_7000ts_sweep`
folder existing on disk with real results in it but no entry in
`ai/project_context.md`, `ai/auto_benchmarking.md`, or any other doc --
nobody looking at "Other docs in `ai/`" would have known it existed.

**This does not replace the tracked-sweep convention.** A sweep run through
`scripts/run_benchmarks.py` already gets `sweep_summary.csv`/`metrics.csv` in
its own `--out-dir`, and a *methodology-focused* sweep (multiple maps/agent
counts/configs, meant to answer a research question) still belongs indexed
in `ai/auto_benchmarking.md`'s sweep table, not just here. This log is for:
(1) logging a run *as soon as it's launched*, not just after it finishes, so
a long/overnight/background run is never only discoverable by noticing a
stray file later; (2) one-off/ad hoc probes that don't fit the
`run_benchmarks.py` batch model and wouldn't otherwise get a CSV at all; (3)
a quick "what's actually running right now / what happened to that run"
answer without reading five docs. Update an entry's Status in place as a run
progresses -- this is not append-only.

## Entries

### 2026-09-17: hierarchical-cascade sweep on `IH_mp_2p_01` (10000 agents, 7000 timesteps)

- **Why**: `ai/hierarchical_matching.md`'s cascade feature had no wall-clock
  timing data yet (the one earlier attempt, `level6_cascade1` on
  2026-09-03, died partway through with no output -- see that doc's
  "Not yet done" section and the folder's `outputs/old/
  IH_mp_2p_01_10000_7000ts_sweep_cascade_probes_2026-09-03/`). Also the
  first real use of the new `--cascadeLevelStride` level-skipping option
  (added this session -- see `ai/hierarchical_matching.md`'s
  "Level-skipping" section).
- **Launched**: 2026-09-17 13:19 UTC, via `scripts/run_cascade_overnight_sweep.sh`
  (background, `nohup` + `disown`, PID at launch 568918), 5h `timeout` cap
  per run so one hang can't block the rest of the queue.
- **Instance/config**: `instances/custom/IH_mp_2p_01/IH_mp_2p_01_10000.json`,
  `-s 7000`, `--preprocessTimeLimit 600000`,
  `--hierarchyCache hierarchy_cache/IH_mp_2p_01_level9.hierarchy` (10 levels,
  0-9; confirmed by reading the cache file's header directly) -- same
  instance/cache/flags as the rest of `outputs/IH_mp_2p_01_10000_7000ts_sweep/`
  (the original 7-run solver-1-vs-6 sweep from 2026-08-29/30, still intact;
  its `sweep_summary.csv`/`metrics.csv`/`metrics.md` were backed up as
  `*_baseline_2026-08-30.*` before this script could touch the directory).
- **Runs** (sequential, safer/cheaper first within each stride group):

  | run | flowSolveLevel | minCascadeLevel | cascadeLevelStride | wall clock | tasksFinished | status |
  |---|---|---|---|---|---|---|
  | `level4_cascade1` | 4 | 1 | 1 | 6290s | 3839 | done |
  | `level6_cascade1` | 6 | 1 | 1 | 6276s | 3716 | done |
  | `level9_cascade1` (intended as "pure cascade" -- turned out **not** to be true entirety mode, see caveat below) | 9 | 1 | 1 | 6254s | 3748 | done |
  | `level4_cascade1_stride2` | 4 | 1 | 2 | 6296s | 3816 | done |
  | `level6_cascade1_stride2` | 6 | 1 | 2 | 6369s | 3735 | done |
  | `level9_cascade1_stride2` | 9 | 1 | 2 | 6374s | 3768 | done |

  All 6 completed cleanly: 0 planner/schedule/timeout errors, no `timeout`
  kills. Total wall clock ~10.5h (2026-09-17 13:19 UTC -> 23:50 UTC).

- **Output**: `outputs/IH_mp_2p_01_10000_7000ts_sweep/IH_mp_2p_01_10000_solver6_<run>.{json,log}`,
  `cascade_runner.log`, `cascade_sweep_summary.csv`,
  `metrics_all_2026-09-17.csv`.
- **Findings**: written up in `ai/hierarchical_matching.md`'s new "Wall-clock
  sweep" section. Two headline results: (1) scheduler cost (flow + local
  match + guide-path lift combined) is ~0.05% of total scheduler+planner
  time on this map/agent-count -- `PlannerTime` dominates so completely that
  no cascade/stride effect on wall-clock is detectable here, regardless of
  whether the sub-additivity argument for the feature is correct in
  isolation; (2) `level9_cascade1` was **not** actually entirety mode --
  `hierarchy_cache/IH_mp_2p_01_level9.hierarchy` (built 2026-08-20, before
  this session's natural-fixpoint change) only has 10 levels because that's
  where the old fixed-depth build stopped, not because it's the map's true
  fixpoint. A fresh build (no cache) shows the real top is level 11 (12
  levels total). `flow_match_count` came out to 5, not 0, as a direct
  consequence -- small in practice (99.96% local) but the run shouldn't be
  cited as proving flow is exactly vacuous.
- **Status**: complete. Results also added to the live throughput dashboard
  (`ai/scene_mp_4p_03_5000ts_sweep_dashboard.md`,
  https://claude.ai/code/artifact/b7877bbb-8313-4351-986b-1ab90907543f,
  `IH_mp_2p_01` dataset) as 6 new configs, so they can be visually compared
  against the non-cascade baselines already there.
