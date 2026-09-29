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

### 2026-09-18: Manhattan-distance-heuristic probe on `IH_mp_2p_01` (10000 agents, 500 timesteps)

- **Why**: while reading the `IH_mp_2p_01_10000_7000ts_sweep` logs (this
  session), found that the low-level planner's guide-path recompute loop and
  `frank_wolfe` traffic-flow LNS both hit their shared per-decision `end_time`
  budget before doing any work at all in every one of `solver6_level4`'s 1562
  logged decisions (`"compute initial stop until 0"` appears 1562/1562 times
  in that log) -- so on this map/agent-count the run is effectively pure
  greedy `causalPIBT`, not the traffic-flow-optimized path planner the
  pipeline is meant to run. Added a new lever, `USE_MANHATTAN_HEURISTIC`
  (`default_planner/const.h`), that swaps every heuristic lookup
  (`get_h`/`get_gp_h`/`astar`) from the exact BFS-from-goal distance table to
  raw Manhattan distance, and skips building/touching that table entirely --
  freeing whatever time the starved run was spending on table upkeep instead
  of on PIBT/A* itself. This probe checks whether that reclaimed time
  actually buys more genuine decisions ("steps") in the same wall-clock
  budget, compared to the first 500 timesteps of the existing
  `IH_mp_2p_01_10000_solver6_level4.json` baseline (exact-heuristic,
  7000-timestep horizon, same instance/cache/flags otherwise).
- **Launched**: 2026-09-18, `nohup` + `disown` (PID at launch 603987),
  foreground-checked build first (`USE_MANHATTAN_HEURISTIC = true`,
  `./compile.sh` exit 0).
- **Instance/config**: `instances/custom/IH_mp_2p_01/IH_mp_2p_01_10000.json`,
  `--scheduleModel 6 --flowSolveLevel 4 -s 500 --preprocessTimeLimit 600000
  --hierarchyCache hierarchy_cache/IH_mp_2p_01_level9.hierarchy
  --logDetailLevel 3` -- same instance/cache/flags as the rest of
  `outputs/IH_mp_2p_01_10000_7000ts_sweep/`, just `-s 500` instead of `7000`
  and built with the Manhattan-heuristic lever on instead of off.
- **Output**: `outputs/IH_mp_2p_01_10000_7000ts_sweep/
  IH_mp_2p_01_10000_solver6_level4_manhattan_500ts.log` (no `.json` -- run
  never reached a clean exit, so `lifelong` never wrote output).
- **Result: OOM-killed at timestep 151/500** (`dmesg`/`journalctl -k`:
  `Out of memory: Killed process 603987 (lifelong) ... anon-rss:30427628kB`,
  2026-09-18 02:41 UTC). **Root cause found, distinct from the Manhattan
  heuristic itself**: `Dist2Path::dist2path` (`default_planner/Types.h:99`)
  is a *per-agent* vector sized `env->map.size()` once built (`d2p` is 4
  ints = 16 bytes/cell -- ~55MB/agent on this ~3.44M-cell map), built by
  `init_dist_2_path()` inside `update_traj()` (`flow.cpp:162`) every time an
  agent gets a fresh guide path, with **no eviction** (unlike
  `global_heuristictable`, which got an LRU cap during the original OOM fix
  work in `ai/claude_memleak_fixes.md`). In the exact-heuristic baseline
  this never mattered because `update_traj()` essentially never got called
  at this scale -- see this session's finding that the guide-path loop was
  starved before touching agent 0 on every one of `solver6_level4`'s 1562
  decisions. The Manhattan lever made `astar()` cheap enough that the loop
  stopped stalling (log shows 400-477 agents/decision processed instead of
  0), so agents started actually getting `Dist2Path` tables built -- ~600
  distinct agents x ~55MB matches the observed ~30.4GB RSS closely. **This
  is a real, previously-latent memory-growth bug in `traj_dists`, not a
  property of the Manhattan heuristic** -- anything that lets the guide-path
  loop actually run at this map/agent-count scale (a faster heuristic, a
  larger time budget, fewer agents, etc.) would hit the same wall. Needs the
  same LRU/bounding treatment `global_heuristictable` already has before
  this probe (or any workload that exercises `update_traj` this heavily) can
  run to completion here.
- **Fix applied**: gated the two `update_dist_2_path()` call sites
  (`flow.cpp:167` inside `update_traj()`, and the scheduler-guide-path-seed
  branch in `planner.cpp:234`) behind `!USE_MANHATTAN_HEURISTIC`, so
  `Dist2Path` is never built at all in Manhattan mode -- nothing reads it
  once `get_gp_h()` short-circuits past it, so building it was pure waste.
  Rebuilt, sanity-checked on `tiny.json` at both flag states, then re-ran
  the same command.
- **Re-run result (2026-09-18, PID 605191)**: completed cleanly to
  `makespan=500`, 0 planner/schedule/timeout errors, RSS held flat around
  4.5GB the whole run (checked every 20s) -- confirms the fix, not just
  "got lucky." Output:
  `outputs/IH_mp_2p_01_10000_7000ts_sweep/IH_mp_2p_01_10000_solver6_level4_manhattan_500ts.{json,log}`.

  | metric (first 500 real timesteps) | baseline (exact heuristic)¹ | Manhattan (fixed) |
  |---|---|---|
  | decisions (steps) | 98 | 376 (3.8x) |
  | tasksFinished | 3276 | 3485 (+209, +6.4%) |

  ¹ sliced from the existing `IH_mp_2p_01_10000_solver6_level4.json` (7000ts
  run) by `Timestep <= 500`, not a fresh `-s 500` re-run -- same
  instance/flags, but subject to the same run-to-run wall-clock jitter
  documented elsewhere in this repo (a few percent on identical configs).
  More decisions translated into more tasks finished in the same window, not
  just more decisions for their own sake -- the Manhattan run actually
  exercises `astar`/`frank_wolfe` now (hundreds of agents/decision through
  the guide-path loop, vs. 0 in the baseline's starved regime), so this
  isn't purely "cheaper heuristic, same behavior" -- it's "the traffic-flow
  planner is functioning again, just against a less accurate distance
  estimate."
- **Not yet done**: a real head-to-head (fresh `-s 500` baseline run rather
  than a slice, and ideally averaged over a couple of runs given the jitter
  caveat above), path-quality/congestion comparison (Manhattan's per-move
  choices are individually worse-informed even though aggregate throughput
  rose), and the same probe at deeper `--flowSolveLevel`s or the full
  7000-timestep horizon to see whether the advantage holds or changes shape
  over a longer run (the backbone-rebuild-cost dynamic documented in
  `ai/todo.md` means short vs. long horizons haven't always agreed on
  solver-6-side questions).
- **Status**: complete. Full writeup (starvation mechanics, the lever, the
  `Dist2Path` bug and fix, code diffs) in `ai/manhattan_heuristic_lever.md`.

### 2026-09-28: 2-location task finding, thesis benchmark set, full-depth hierarchy caches

- **Why**: comparing against the reference paper
  (`thesis/AAMAS_2026_Yue_Camera_ready.pdf`) showed every existing
  `instances/custom/<map>/` task file was generated with the LoRR generator's
  default `--minEPT 1 --maxEPT 4`, so tasks had 1-4 locations, not the paper's
  pickup->delivery pairs. Single-location tasks finish on arrival and dominate
  throughput.
- **Probes** (scratchpad only, not kept in the repo):
  - `orz900d_5000`, 200 steps, original vs. 2-location tasks: solver 1
    1,998 -> 115 tasks finished; solver 6 1,840 -> 219. 95-99% of tasks
    finished on the original instance had 1 location.
  - `IH_mp_2p_01_10000`, 500 steps, `USE_MANHATTAN_HEURISTIC = true`, flags as
    in the 2026-09-18 entry: solver 6 at level 4 finished 3,485 tasks on the
    original instance and 478 on a 2-location instance. Solver 1 finished 0
    on the 2-location instance: about 38 s per full-map flow solve and only
    14 decisions in 500 steps.
  - A hand-rolled 2-location task file put 354 dropoffs in IH's disconnected
    regions. `astar` threw `no path found` and aborted the run, so unreachable
    task locations crash a run rather than being skipped. The official
    generator avoids this by sampling only from the largest connected region.
- **Output**: `instances/thesis_benchmarks/`, fully specified in its
  `README.md`. It covers orz900d at 10k/20k agents and warehouseXL,
  IH_mp_2p_01, scene_mp_4p_03 and scene_sp_pol_06 at 10k/20k/40k/80k. Every
  task has 2 locations, `numTasksReveal` is 1.5, and task files hold 1.5x the
  largest team. All validity checks pass except one orz900d task whose pickup
  and delivery are the same cell (documented in the README).
- **Hierarchy caches**: rebuilt to full depth as
  `hierarchy_cache/{orz900d,warehouseXL,scene_mp_4p_03,scene_sp_pol_06}_full.hierarchy`.
  IH already had `IH_mp_2p_01_fixpoint.hierarchy`. All five match a fresh
  no-cache build level for level (12/12/12/13/14 levels). The older August
  caches stop early and were left in place, unused.
- **Build timings**: clean no-cache timings are in
  `instances/thesis_benchmarks/hierarchy_build_times.csv`, and the README
  explains why `schedulerHierarchyBuildTime` from ordinary runs is not a
  consistent build-time measure. scene_sp_pol_06 peaks at about 25 GB RSS.
- **Status**: complete. Nothing committed. `instances/custom/IH_mp_2p_01_2ept/`
  (an earlier 10k-agent test set) is superseded by the new set and can be
  deleted.
- **Verification (2026-09-29)**:
  - A scratchpad tool reusing `validate_hierarchy_cache.cpp`'s `compare_levels`
    found every full-depth cache identical to a fresh fixpoint build at every
    level (206-240 checks each, 0 failures).
  - Old vs. new caches: identical on every shared level, except the old top
    level's `to_coarser_node_id`, which is expected because the old builds
    stopped there. Past results used the same hierarchy.
  - Real runs, solver 6, 50 steps on each 10k thesis instance, at flow level 4
    and in entirety mode: 10/10 clean. Entirety mode had 0 flow matches on
    every map.
  - Reporting quirk: `schedulerHierarchyNumLevels` and related fields come
    from the last scheduler call only, and are 0 if that call had nothing to
    reassign.
