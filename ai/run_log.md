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

### 2026-09-29: 80k smoke tests and pickups-reached counter

- **Smoke tests**: warehouseXL, IH_mp_2p_01, scene_mp_4p_03 and
  scene_sp_pol_06 at 80k agents, `-s 20`, solver 5, solver 6 level 2 and
  solver 6 hierarchical-only (12 runs, one at a time). All clean: exit 0,
  0 planner/schedule/timeout errors. Peak memory up to 24.0 GB
  (scene_sp_pol_06, solver 6). Outputs were kept in the session scratchpad
  only.
- **Greedy first decision**: solver 5 never finished its first decision
  within 20 steps at 80k. Measured on IH with `-s 400`: 31 s at 40k and 218 s
  at 80k for the first decision, then at most 1.5 s and 0.4 s, 0 errors.
  It's a one-time cost of comparing every free agent with every open task.
- **Counter**: added `TasksOpenedThisStep` / `numTaskOpened` (pickups
  reached). Verified on tiny (4 = 4 = 4) and IH 80k solver 6 level 2
  (72,185 = 72,185 = 72,185 across output total, per-row sum and log
  "opens task" lines). Other fields identical to the pre-change smoke run.
- Details: `instances/thesis_benchmarks/README.md`, "Pre-sweep checks".

### 2026-09-29: thesis sweep stopped; Manhattan heuristic freezes the planner

- **Why**: the final sweep (`outputs/thesis_sweep/`) finished 4 runs
  (orz900d 10k, solver 6 levels 2/4/6/8, 8,000 steps). Every run stopped
  making progress at about step 1,000: 0 pickups and 0 deliveries for the
  remaining 7,000 steps, 0 errors, planner using its full budget each step.
  Sweep stopped at 16:50.
- **Mechanism (from the code)**: PIBT ranks each agent's next cell by
  `get_gp_h` (`default_planner/pibt.cpp`). Under `USE_MANHATTAN_HEURISTIC`
  that returns raw Manhattan distance to the goal and never reads the A*
  guide path, which `update_traj` still computes. An agent with a wall
  between it and its goal scores "wait" best forever. Scheduler guide paths
  can't help: they only reach the planner with `--useTraffic`, and
  Manhattan mode ignores them anyway.
- **Test**: orz900d 10k, thesis 2-location instance, 1,500 steps, solver 6
  level 4 unless noted, 5 runs in parallel (scratchpad only). Cumulative
  deliveries:

  | run | step 500 | step 1,000 | step 1,500 | decisions |
  |---|---|---|---|---|
  | A: current build (Manhattan) | 623 | 628 | 628 | 1,500 |
  | B: exact heuristic | 503 | 654 | 736 | 543 |
  | C: commit 9325d69, before the pickup counter (Manhattan) | 615 | 621 | 621 | 1,500 |
  | D: old 1-4-location tasks (Manhattan) | 4,195 | 4,197 | 4,197 | 1,498 |
  | E: solver 5 (Manhattan) | 611 | 617 | 617 | 1,498 |

- **Result**: the Manhattan heuristic is the cause. The pickup counter (C),
  the task format (D) and the scheduler (E) all freeze the same way; only
  the exact heuristic (B) keeps delivering. B is starved, though: 543
  decisions in 1,500 steps (possibly worse than usual, as 5 runs shared the
  machine). Any Manhattan-mode result past a few hundred steps is affected,
  including the 2026-09-18 IH probe's conclusions if extended.

### 2026-09-29: local-path-BFS planner fix, first tests

- **What**: `USE_LOCAL_PATH_BFS` (`default_planner/const.h`) implemented per
  `ai/planner_local_bfs_plan.md`. Testing it against pure Manhattan.
- **Runs** (scratchpad only, not in `outputs/`): orz900d 10k thesis
  instance, 1,500 steps, 3 in parallel: new planner solver 6 level 4; pure
  Manhattan build (`-DPLANNER_USE_LOCAL_PATH_BFS=false`) solver 6 level 4;
  new planner solver 5.
- **Result**: the freeze is gone. Deliveries (pickups) at step 1,500:
  new s6 L4 5,495 (15,453); pure s6 L4 617 (10,532); new s5 5,530
  (15,508). Pure Manhattan made no deliveries after step 750, and its stuck
  count (agents with a goal that haven't moved for 20+ decisions) rose to
  10,000 of 10,000 by step 750. The new planner delivered about 4 per step all the way
  to 1,500 with no plateau, and its stuck count stayed at 0-8 after step 250.
  All runs made 1,499-1,500 decisions in 1,500 steps, 0 errors. For
  reference, the exact heuristic managed 736 deliveries at 1,500 (2026-09-29
  test B above).
- **Timing** (new planner): PIBT mean 37 ms, p99 52 ms, max 77 ms against a
  100 ms reserve (pure Manhattan: 5 ms). Guide-path A* is the slow part:
  about 12 ms per search on orz900d, so the initial 10k backlog took until
  about step 250 to clear (agents without a path fall back to Manhattan
  meanwhile; 3,679 still without one at step 100). After that, 10-25 new paths per
  step, taking 100-250 ms. 7-21 agents per step were too far from their
  path and got re-planned.
- Not yet tested: `PASS_SCHEDULER_PATHS_TO_PLANNER`, 20k agents, other maps.

### 2026-09-29: local-path-BFS planner on IH, and scheduler path handoff

- **Runs** (scratchpad only), 1,500 steps, solver 6 level 4, 4 in parallel:
  IH_mp_2p_01 10k with the new planner, pure Manhattan, and new planner +
  `PASS_SCHEDULER_PATHS_TO_PLANNER` (build with
  `-DPLANNER_PASS_SCHEDULER_PATHS=true`); orz900d 10k new planner +
  scheduler paths (compare with the 5,495 above). The `planner stats:` line
  now also has `paths_from_scheduler`.
- **Result: IH fails; the sweep was not restarted.** Deliveries at step
  1,500 (decisions in 1,500 steps):

  | Run | Deliveries 500 / 1,000 / 1,500 | Decisions | Stuck at decision 1,000 | Agents without a path at 1,000 |
  |---|---|---|---|---|
  | IH new planner | 541 / 1,355 / 2,364 | 1,021 | 4,661 | 6,740 |
  | IH pure Manhattan | 509 / 937 / 1,178 | 1,053 | 8,776 | – |
  | IH new + scheduler paths | 526 / 1,340 / 2,360 | 1,024 | 4,584 | 6,753 |
  | orz900d new + scheduler paths | 2,050 / 3,978 / 5,646 | 1,500 | 2 | 0 |
  | (earlier) orz900d new | 2,037 / 3,894 / 5,495 | 1,500 | 6 | 0 |

  0 errors in all runs.
- **Why IH fails:** guide-path A* is far too slow on IH. Only 3-14 paths
  per step, with stage 2 using 450-1,050 ms, so roughly 70-300 ms per
  search (orz900d: about 12 ms). About 6,700 of 10,000 agents never get a
  path in 1,000 decisions, so they keep using Manhattan distance and get
  trapped; the stuck count climbs to 4,661. A single long A* search can't be
  interrupted, so it runs past the deadline: only about 1,020 decisions in
  1,500 steps (pure Manhattan overruns the same way, so this is A*, not the
  BFS). The BFS itself is fine: PIBT max 35 ms on IH. The new planner still
  doubles deliveries over pure Manhattan (2,364 vs. 1,178) and they keep
  rising, but it isn't good enough for the thesis sweep.
- **Scheduler paths (`PASS_SCHEDULER_PATHS_TO_PLANNER`):** almost no
  effect. Solver 6 at level 4 matches nearly every agent locally, and only
  flow-matched agents get a path: the planner received 3,489 (IH) and 796
  (orz900d) scheduler paths in total, nearly all at step 0. IH: 2,360 vs.
  2,364 deliveries. orz900d: 5,646 vs. 5,495 (+2.7%, within what
  run-to-run differences with 3-4 runs sharing the machine could explain).
  Leave it off.
- orz900d with scheduler paths peaked at 85 ms PIBT (4 runs in parallel),
  closer to the 100 ms reserve than before.

### 2026-09-29: why IH agents are stuck (diagnosis)

- **What**: added a stuck-agent diagnosis (`stuck breakdown:` and
  `stuck sample:` log lines, see `ai/planner_local_bfs_plan.md`, "Stuck-agent
  diagnosis"). Checked on the hand-built trap map: pure Manhattan labels the
  agent `trapped_manhattan` from decision 20; the new planner has no stuck
  agents.
- **Runs** (scratchpad): IH_mp_2p_01 10k, 500 steps, solver 6 level 4, new
  planner and pure Manhattan in parallel. 0 errors; 366 and 386 decisions.
- **Stuck breakdown at the last decision:**

  | | trapped (Manhattan) | blocked (Manhattan) | trapped / blocked (path BFS) | moved |
  |---|---|---|---|---|
  | New planner | 1,915 | 1,146 | 0 / 0 | 20 |
  | Pure Manhattan | 2,670 | 1,932 | 0 / 0 | 21 |

- **Independent check** (`outputs/planner_stuck_diagnosis/stuck_check.py`,
  map file only, 90 sampled stuck agents per run): every agent the planner
  labelled trapped is a Manhattan dead end on the map (all open neighbours
  are further from the goal); every agent labelled blocked is not. In the new
  planner run, all 90 samples had no guide path. Pictures:
  `outputs/planner_stuck_diagnosis/stuck_ih_{new,pure}.png`.
- **Conclusion**: with the new planner, no agent scored by the path BFS got
  stuck. All stuck agents were ones without a guide path, scored by
  Manhattan distance. About 60% are pinned by the score at a wall. The
  other 40% had a better move but didn't take it, most likely because other
  agents were in the way. Positions of other agents weren't logged, so
  that part is not proven. Fixing IH means getting guide paths to agents
  faster.

### 2026-09-29/30 (overnight): parallel guide paths, and what really limits IH

- **What**: implemented `ai/parallel_guide_paths_plan.md` (guide-path A* on
  6 threads), then followed the evidence: an A* benchmark, weighted A*, an
  in-search deadline, a rotation fix, and a diagnostic that ignores
  congestion. Details and tables: that doc, "Implementation and results".
- **Correctness**: debug builds (path and congestion-map checks) on tiny,
  the trap map and orz900d 10k 300 steps, with 1 and 6 threads: 0 failures,
  0 errors. orz900d's initial path backlog cleared by decision 50 instead of
  about 200.
- **IH 10k, 500 steps, solver 6 level 4, one run at a time:**

  | Planner | Deliveries | Decisions | Stuck at end |
  |---|---|---|---|
  | Sequential (last night) | 535 | 366 | 3,061 |
  | 6 threads | 319 | 244 | – |
  | 6 threads + weight 2 | 352 | 250 | – |
  | 6 threads + weight 2 + deadline | 764 | 499 | 4,954 |
  | same + rotation fix | 1,141 | 499 | 1,190 |
  | **6 threads + deadline + rotation, congestion ignored** | **1,299** | 499 | **1** |

- **Why**: A* gets far slower as the congestion map fills (benchmark on the
  IH map: 15-22 ms per search with no other paths, 240 ms with 8,000). With
  about 10k paths registered, IH searches take longer than the whole
  stage-2 window. Ignoring congestion in the search clears the backlog by
  decision 100 and leaves nobody stuck.
- **Decision left to the user**: whether guide paths should ignore
  congestion (`GUIDE_PATH_IGNORE_CONGESTION`, default false) or use a
  weight. Defaults left unchanged; sweep NOT restarted.

### 2026-09-30 (overnight): parallel local matching, and congestion evidence runs

- **Parallel local matching** (`ai/parallel_local_matching_plan.md`)
  implemented and tested: identical output for 1/2/4/6 threads (validator
  check 5, 35/35 on tiny, tinyComplex, warehouseSmall_200 L4/L6, IH 80k L8,
  scene_mp_4p_03 80k L8); ThreadSanitizer clean. First IH 80k level-8
  decision: 49.3 s → 8.3 s. Full IH 80k L8 run, 50 steps: 0 errors,
  `SchedulerLocalMatchTime` 8.3 s vs `SchedulerLocalMatchCpuTime` 49.6 s on
  the first decision, peak 9.0 GB.
- **Evidence runs for the congestion decision** (scratchpad
  `evidence.sh`, one at a time, solver 6 level 4, `--logDetailLevel 3`):
  default build (congestion on) vs `GUIDE_PATH_IGNORE_CONGESTION=true`, on
  orz900d 10k 1,500 steps, scene_mp_4p_03 10k 500 steps, IH 20k 500 steps.
- **Result**: these first runs (`ev_*`) were affected by the harness bug in
  the next entry (planner got about 190 ms per step on scene_mp_4p_03), so
  they were redone as `ev2_*` after the fix; results there.

### 2026-09-30 (overnight): harness bug, ~650 ms per step lost on big maps

- **Found while checking why scene_mp_4p_03 10k failed** (130 deliveries in
  500 steps, over 9,600 agents without a path, with congestion on or off):
  the planner only got about 190 ms per step (`plan limit`).
- **Cause**: `Entry::compute` (`src/Entry.cpp`) built the whole-map
  "background flow" array (`get_opened_flow`, 32 bytes per cell) every step
  for every solver, then it was copied into the scheduler (`set_flow` by
  value, member copy) and copied again into each `schedule_plan_*` call (by
  value). Measured on scene_mp_4p_03 (13.9M cells): `set_flow` about 395 ms
  and the scheduler call about 395 ms, of which only about 138 ms was the
  solve. The array is only read with `--useTraffic`, which the sweep doesn't
  use.
- **Fix** (no behaviour change): the flow is only built with
  `--useTraffic`; `set_flow` moves instead of copying; the four
  `schedule_plan_*` functions take it by `const&`. `Entry::compute` now
  logs an `entry timing:` line per step (set_flow and scheduler ms).
- **After**: scene_mp_4p_03 10k, 20 steps: `set_flow` 0.002 ms, scheduler
  about 3 ms, planner limit 976 ms (was about 190). 0 errors.
- **Consequence**: every earlier run on the big maps (IH, warehouseXL, the
  scene maps; all solvers) lost roughly 100-650 ms of planner time per step
  to this, growing with map size (about 3 × 32 bytes × cells). orz900d loses
  about 30-60 ms. The evidence runs above were redone with the fix
  (`ev2_*`, below).

### 2026-09-30 (overnight): evidence runs redone after the harness fix

- **Runs** (`ev2_*`, scratchpad, one at a time, 00:00-01:03, solver 6
  level 4): orz900d 10k 1,500 steps sequential / 6 threads / 6 threads
  congestion ignored; scene_mp_4p_03 10k and IH 20k, 500 steps, 6 threads
  (3 on scene, memory cap) with congestion on and ignored.
- **Result**: see the table in `ai/parallel_guide_paths_plan.md`
  ("Evidence runs after the fix"). orz900d: sequential 5,492 (reproduces
  yesterday), parallel 4,520 / 4,901 (congestion ignored). IH 20k:
  congestion ignored 2,431 with 0 stuck vs 1,788 with 5,149 stuck.
  scene_mp_4p_03: 203-214 deliveries in 500 steps, thousands stuck either
  way (plain A* too slow there). 0 errors in every run.
- **Action**: `GUIDE_PATH_THREADS` default set back to 1 (sequential).
  Sweep NOT restarted: no guide-path setting works on all maps; needs the
  user's decision.

### 2026-10-01: benchmark of solver 6's coarse-to-fine lift

- **What**: new tool `./build/bench_hierarchy_lift` lifts shortest coarse
  paths at level L for random (agent start, task) pairs and checks them
  against fine BFS. For `ai/hierarchical_guide_paths_plan.md` (guide paths
  from the hierarchy instead of fine A*). Diagnostics, `anchor_every_level`
  and `max_path_cells` added to `lift_coarse_paths_to_fine`, all off by
  default at first; `guide_path_validator` still passed on `tiny`/`tinyComplex`.
- **Runs**: orz900d, IH_mp_2p_01, scene_mp_4p_03 10k instances, 200-300
  pairs, levels 1-10. Output in `outputs/hierarchy_lift_bench/`.
- **Result**: as solver 6 runs it, the lift only works from level 1. From
  level 2 up almost every path has the wrong endpoints (intermediate levels
  aren't anchored to the real start and goal), so the full-map fallback
  runs, and that fails for about 42% of pairs on IH and 78% on
  scene_mp_4p_03. With every level anchored and the 5,000-cell cap raised,
  every pair got a valid path at every level on all three maps: at level 4,
  1.09× shortest in about 2.5 ms on IH and 1.05× in about 7.5 ms on scene
  (fine A* there: 70-300 ms). Tables in the plan doc.
- **Then (same day): anchoring made permanent** at the user's request (it
  was the original design intent); the option was removed. Solver 6's
  guide paths and `GuidePathLengthSum` change: values from before
  2026-10-01 aren't comparable. Checks: `guide_path_validator` on `tiny`,
  `tinyComplex`, `warehouseSmall_100`, `random_2000`, with and without
  `--useTraffic`, 0 failed; bench with default settings matches the anchored
  results (only the 5,000-cell cap still fails, e.g. 15 of 200 pairs on
  scene_mp_4p_03 level 4); `lifelong` solver 6 200 steps: `tiny` 14
  finished, `tinyComplex` 15, 0 errors. Not yet run on a big map.
- **Runtime, real runs (same day):** solver 6 level 4, 300 steps, old lift
  vs new, IH 10k and orz900d 10k
  (`outputs/hierarchy_lift_bench/solver6_old_vs_new/`). Scheduler solve time
  5-12% lower in total, first step 2-2.5x faster; throughput unchanged
  (planner-bound). Flow-matched paths are short (tens of cells), so the old
  fallback was cheap there. `GuidePathLengthSum` per flow-matched agent
  roughly doubled (IH 24 -> 49, orz900d 17 -> 41): lifted paths stretch much
  more on short pairs (orz900d level 4: 2.1x under 100 cells vs 1.25x over
  800). Details in `ai/hierarchical_guide_paths_plan.md`.

### 2026-10-01: `--computeGuidePaths` flag and guide-path timing fixes

- **What**: new CLI flag `--computeGuidePaths` (default true) gating all
  scheduler guide-path work (solver 6/7 lift, solver 1 path recording,
  solver 2 storage); recorded in the output JSON. Timing scopes corrected:
  the lift moved from `SchedulerSolveTime` into `SchedulerGuidePathTime` for
  solvers 6/7; solver 1's flow walk moved from guide time into solve time.
  Solvers 6/7 no longer report 0 solve time when `NetworkSimplex` fails.
- **Checks**: `guide_path_validator` 4 instances x 2, 0 failed; `tiny`
  solvers 1/6/7 flag on/off, 0 errors, same finished; IH 10k solver 6 level
  4 (100 steps) and orz900d 10k solver 1 (20 steps), flag on/off: solve time
  unchanged by the flag, guide time 0 when off. Output in
  `outputs/hierarchy_lift_bench/guide_path_flag/`. Details and the field
  definitions: `ai/hierarchical_guide_paths_plan.md`.
- **Consequence**: `SchedulerSolveTime` / `SchedulerGuidePathTime` from
  earlier runs are not comparable with later ones.
- **Then (same day): solve time lined up for solvers 6 and 7.** Both timers
  now start right after local matching and stop before the lift, so
  `SchedulerLocalMatchTime`, `SchedulerSolveTime` and
  `SchedulerGuidePathTime` are disjoint, and solver 6's solve time includes
  the coarse graph build (about 26 ms on the first IH 10k decision). Checks:
  validators 0 failed; orz900d 10k solvers 6/7 and IH 10k solver 6, level 4,
  50 steps, 0 errors (`outputs/hierarchy_lift_bench/solve_time_aligned/`).

### 2026-10-01: planner guide paths from the hierarchy (lift / corridor A*)

- **What**: branch `hierarchy-guide-paths`. New `--guidePathSource
  astar|lift|corridor` (default `astar`), `--guidePathLevel`,
  `--guidePathCorridorMargin`, `--guidePathCorridorCongestion`; the planner
  builds guide paths (new goals and `needs_replan`) from a coarse path at
  the chosen level, falling back to full-map A*. Details in
  `ai/hierarchical_guide_paths_plan.md`, "Implementation and results".
- **Checks**: solver 6 unchanged (validators 0 failed with the same check
  counts; `GuidePathLengthSum` 204 / 192 on tiny / tinyComplex; bench
  identical). 25 small-map planner runs with debug checks on (tiny,
  tinyComplex, trap; every source, levels 1-3): 0 errors, 0 check failures,
  0 fallbacks, trap escaped
  (`outputs/hierarchy_lift_bench/corridor/small_checks.txt`).
- **Bench** (`outputs/hierarchy_lift_bench/corridor/`): every lift and
  corridor search valid on orz900d, IH, scene_mp_4p_03. Corridor margin 0
  is 1.000-1.020× shortest at every level; IH level 4: 4.2 ms (full-map A*
  27.9 ms), scene level 4: 16 ms (308 ms). Lift is faster at high levels
  but stretches (IH level 6: 1.37×).
- **Status**: big-map planner runs (plan step 5) not started.

### 2026-10-01: quick planner check on scene_mp_4p_03, corridor vs lift

- **What**: scene_mp_4p_03 10k, solver 6 level 4, 300 steps, guide paths
  from the hierarchy at level 4, corridor then lift, one at a time
  (`outputs/hierarchy_lift_bench/planner_scene_quick/`, `summary.txt`).
- **Result**: 0 errors, 0 hierarchy fallbacks, 18 GB peak, 287 / 289
  decisions. Decisions 1-2 built all 10k pickup paths at 0.1 ms each (short
  legs). Delivery legs (decisions 3-50): corridor 65 paths per decision at
  12.1 ms, lift 125 at 6.2 ms. Agents without a path: corridor peaked at
  6,437 (decision 47) and reached 0 at decision 163, with up to 342 stuck;
  lift peaked after decision 1 and reached 0 at decision 79, 0-1 stuck.
  Mean path 1,375 cells (corridor) vs 1,451 (lift). Finished 93 (corridor)
  vs 73 (lift); too few deliveries in 300 steps to compare.
- **Reading**: in the planner the corridor is about 2x slower per long path
  than the lift (bench: 1.4x), which delays the delivery-leg backlog by
  about 85 steps. After that, every agent has a path with either source.
