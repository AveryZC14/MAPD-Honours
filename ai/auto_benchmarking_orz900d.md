# Benchmark run: solver 1 vs solver 6 (coarsen levels 1-4) on orz900d

> Detail file for one sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index, synthesis, and reusable methodology notes (the
> `tp/steps` vs `tp/makespan` mechanics referenced below live there now).

Started 2026-07-22. This doc is a running log for a Claude-driven benchmark
sweep, kept updated so a future session (or a disconnected/resumed one) can
tell what's done, what's left, and what went wrong. Written to satisfy the
user's request to track process + issues + suggested improvements as we go.

## Goal

15 runs total: 3 instance sizes x 5 configs, to later compute throughput
(tasks finished / timestep) and plot solver-1 vs solver-6-at-various-coarsen-
levels.

- Instances: `instances/custom/orz900d/orz900d_{5000,10000,20000}.json`
  (656x1491 maze map, ~978K walkable cells; only agent/task counts differ)
- Configs per instance:
  1. `--scheduleModel 1` (baseline min-cost-flow scheduler)
  2. `--scheduleModel 6` with `kDefaultCoarsenLevels = 1`
  3. `--scheduleModel 6` with `kDefaultCoarsenLevels = 2`
  4. `--scheduleModel 6` with `kDefaultCoarsenLevels = 3`
  5. `--scheduleModel 6` with `kDefaultCoarsenLevels = 4`
- `kDefaultCoarsenLevels` is a `constexpr int` at
  `map_reduction_test/MapCoarsenV1.cpp:31` (repo default: `2`) — it's baked in
  at compile time, so each distinct level requires editing that line and
  re-running `./compile.sh` before the runs for that level.
- Output: 15 JSON files in `outputs/orz900d_solver_comparison/`.

## Flags decided with the user (must stay identical across all 15 runs)

**REVISED after an OOM investigation mid-run — see "Issues" below for why.**
Final flags:

- `-t 1000 -p 300000` (planTimeLimit/preprocessTimeLimit — **left at
  upstream defaults**, NOT the originally-planned 10000/300000 override)
- `-s 200` (simulationTime timesteps, down from default 5000 — chosen for
  tractable total wall-clock; see history below)
- No `--useTraffic` (left at default `false`) — solver 6's guide-path lifting
  only engages when traffic-aware guiding is on (`need_guide_paths =
  use_traffic && curr_timestep >= 100`, see `ai/project_context.md`), and the
  memleak-fix verification run was also done without it. Since this sweep is
  about scheduler/throughput comparison, not guide-path quality, traffic was
  left off. Flagging this as an assumption in case the user wanted it on.
- No `--assignNew` / `--commitWindow` overrides (left at defaults).

### History of how the flags were decided (kept for context)

1. Originally planned `-s 1000 -t 10000 -p 300000` uniformly (user's initial
   suggestion, to guard against per-timestep timeouts on this large map).
2. First live run (solver 1, 5000 agents, `-t 10000 -p 300000 -s 1000`) was
   **OOM-killed by the kernel at timestep 12**, RSS ~27.7GB on a 27GB
   machine (`dmesg`: `Out of memory: Killed process ... lifelong ...
   anon-rss:27732236kB`). Not a timing problem — a real crash.
3. User asked to shorten `-s` to 200 for a same-day answer, and separately
   asked to investigate the OOM (they observed solver 1 runs fine locally
   without the `-t`/`-p` override).
4. Investigation: re-ran solver 1 (5000 agents, `-s 200`) with **default**
   `-t 1000 -p 30000` (no override). Result: completed cleanly, RSS flat at
   ~1.47-1.49GB throughout, `numTaskFinished: 2163`, 0 planner/schedule
   errors, ~136s wall time for 200 timesteps.
5. Conclusion: raising `-t` to 10000ms lets solver 1's per-timestep flow
   solve run to full completion instead of being cut off by the 1000ms
   timeout — and something in that *complete* computation leaks/balloons
   memory unboundedly. The LRU heuristic-table cap added in
   `ai/claude_memleak_fixes.md` was scoped to solver 6 only; solver 1 has no
   equivalent bound, so a generous time budget exposes an unbounded-growth
   path that a tight time budget normally never reaches. See "Issues" below
   for the suggested follow-up (this is a real, unfixed bug in
   `schedule_plan_flow`, not addressed as part of this benchmarking task).
6. User chose to use **default `-t`/`-p` for all 15 runs** (not split
   per-solver) to keep every run in the sweep directly comparable, since
   defaults are the validated-safe setting for both solvers.

## Naming convention

`outputs/orz900d_solver_comparison/orz900d_<agents>_solver<N>[_level<L>].json`

e.g. `orz900d_5000_solver1.json`, `orz900d_10000_solver6_level3.json`.

## Build/run ordering (minimizing recompiles: 4 total)

Solver 1 doesn't touch `kDefaultCoarsenLevels` at all, so its 3 runs are
bundled into whichever compile pass is most convenient (level=1 pass) rather
than getting a dedicated 5th compile.

1. Set `kDefaultCoarsenLevels = 1`, `./compile.sh`, run:
   solver1 x {5000,10000,20000}, solver6-level1 x {5000,10000,20000} (6 runs)
2. Set `kDefaultCoarsenLevels = 2` (repo default), `./compile.sh`, run:
   solver6-level2 x {5000,10000,20000} (3 runs)
3. Set `kDefaultCoarsenLevels = 3`, `./compile.sh`, run:
   solver6-level3 x {5000,10000,20000} (3 runs)
4. Set `kDefaultCoarsenLevels = 4`, `./compile.sh`, run:
   solver6-level4 x {5000,10000,20000} (3 runs)
5. Restore `kDefaultCoarsenLevels = 2` (original repo state) and do a final
   `./compile.sh` so the working tree is left as found.

## Status log

- [2026-07-22] Plan established, output dir created, this doc started.
- [2026-07-22] User asked (mid-run) to keep this doc updated periodically as
  a resumability aid, plus a running list of issues/suggested improvements.
- [2026-07-22] First attempt at `-s 1000 -t 10000 -p 300000` OOM-crashed
  solver 1 at timestep 12 (see "Issues" below). Investigated, root-caused,
  and revised flags to `-s 200` with default `-t`/`-p`.
- [2026-07-22] Ran the whole matrix as one unattended background script
  (`/tmp/.../scratchpad/run_full_sweep.sh`, not preserved past this session —
  reconstruct from the command template below if resuming). It edited
  `kDefaultCoarsenLevels` via `sed`, ran `./compile.sh`, executed 3 (or 6 for
  the level=1 pass) `./build/lifelong` runs, then moved to the next level,
  finishing with `kDefaultCoarsenLevels` restored to its original value `2`
  and a final rebuild. Total wall time **02:13:41 → 02:58:30 (~45 min)** for
  all 15 runs + 5 compiles combined — much faster than the original
  worst-case estimate (hours), since solver 1 turned out fast at the
  default time limit and `-s` was cut to 200.
- [2026-07-22] **DONE.** All 15 output JSONs present in
  `outputs/orz900d_solver_comparison/`, 0 planner errors / 0 schedule errors
  / 0 entry timeouts across every run. Repo left clean (`kDefaultCoarsenLevels`
  back to `2`, `git status` shows only new/untracked output files, no source
  diff).

Command template used for every run (only `--scheduleModel`, agent count,
and — for solver 6 — `kDefaultCoarsenLevels` at compile time varied):
```shell
./build/lifelong --inputFile instances/custom/orz900d/orz900d_<agents>.json \
  -o outputs/orz900d_solver_comparison/orz900d_<agents>_<label>.json \
  --scheduleModel <1|6> -s 200 --logDetailLevel 3
```

## Results

| file | agents | tasksFinished | steps recorded | makespan | tp/steps | tp/makespan |
|---|---|---|---|---|---|---|
| orz900d_5000_solver1.json | 5000 | 2178 | 148 | 201 | 14.716 | 10.836 |
| orz900d_5000_solver6_level1.json | 5000 | 1900 | 200 | 200 | 9.500 | 9.500 |
| orz900d_5000_solver6_level2.json | 5000 | 1875 | 200 | 200 | 9.375 | 9.375 |
| orz900d_5000_solver6_level3.json | 5000 | 1835 | 200 | 200 | 9.175 | 9.175 |
| orz900d_5000_solver6_level4.json | 5000 | 1806 | 200 | 200 | 9.030 | 9.030 |
| orz900d_10000_solver1.json | 10000 | 4683 | 196 | 201 | 23.893 | 23.299 |
| orz900d_10000_solver6_level1.json | 10000 | 4032 | 200 | 200 | 20.160 | 20.160 |
| orz900d_10000_solver6_level2.json | 10000 | 4030 | 200 | 200 | 20.150 | 20.150 |
| orz900d_10000_solver6_level3.json | 10000 | 3959 | 200 | 200 | 19.795 | 19.795 |
| orz900d_10000_solver6_level4.json | 10000 | 3869 | 200 | 200 | 19.345 | 19.345 |
| orz900d_20000_solver1.json | 20000 | 9845 | 199 | 200 | 49.472 | 49.225 |
| orz900d_20000_solver6_level1.json | 20000 | 8524 | 200 | 200 | 42.620 | 42.620 |
| orz900d_20000_solver6_level2.json | 20000 | 8493 | 200 | 200 | 42.465 | 42.465 |
| orz900d_20000_solver6_level3.json | 20000 | 8303 | 200 | 200 | 41.515 | 41.515 |
| orz900d_20000_solver6_level4.json | 20000 | 8141 | 200 | 200 | 40.705 | 40.705 |

Notes on reading this table:
- "steps recorded" = `len(timeStepMetrics)`, "makespan" = the `makespan`
  field in the output JSON — see `ai/auto_benchmarking.md`'s methodology
  section ("`tp/steps` vs `tp/makespan`") for the general mechanics of why
  these diverge. The orz900d-specific numbers: `makespan` overshoots the
  `-s 200` request by 1 (→201) for the 5000- and 10000-agent solver 1 runs
  (the off-by-one described there), and `tp/steps` overstates solver 1's
  throughput worst at 5000 agents (148 recorded steps vs. the real 201
  elapsed, a ~26% gap: 14.716 vs. the corrected 10.836) — solver 1 hits the
  "planner timeout" divergence constantly on this map even at the default
  1000ms/timestep cap; solver 6 essentially never does, so its steps
  recorded and makespan always agree at exactly 200.
- Directionally, solver 1 still finishes more tasks/(real timestep) than
  any solver 6 coarsen level at every agent count tested even after this
  correction, though its margin at 5000 agents shrinks substantially
  (10.836 vs. solver6-level1's 9.500, not the uncorrected 14.716 vs. 9.500).
  Within solver 6, more coarsening (higher level) trades off a small amount
  of throughput (level 1 > 2 > 3 > 4 consistently) under either metric.
  This tracks with the thesis's expected trade-off (deeper coarsening =
  faster/cheaper per-timestep solve, at some assignment-quality cost).
- Caveat: solver 1 still hits `"planner timeout"` messages regularly even at
  the default 1000ms/timestep cap (confirmed in the investigation run's
  log), i.e. it's still time-budget-starved at these settings, just not
  leaking memory the way it does at the elevated cap. So its throughput
  numbers here may still understate its "true" (unconstrained) assignment
  quality — see `ai/claude_memleak_fixes.md` "Remaining considerations" for
  the same observation made previously. Read the solver-1-vs-6 comparison
  as "under identical, practical time budgets," not "under unconstrained
  compute."
- A `visualisation/compute_throughput_metrics.py` script now generates this
  exact table (file, agents, tasks, steps, makespan, tp/steps, tp/makespan)
  as CSV from either a single result JSON or a folder of them — use it to
  regenerate/extend this table instead of hand-computing for future sweeps.

<!-- Append new entries below if this doc is reused for a future sweep. -->

## Issues / anomalies encountered

- **Solver 1 has an apparent unbounded memory-growth bug that only manifests
  under a generous `--planTimeLimit`.** At the default 1000ms/timestep cap it
  runs stably (verified: 200 timesteps, 5000 agents, RSS flat ~1.5GB). At
  10000ms/timestep it gets OOM-killed by timestep 12 (RSS ~27.7GB). Likely
  cause: the tight default timeout normally cuts off solver 1's per-timestep
  full-map min-cost-flow solve / heuristic init before it can finish: with
  10x more time budget it runs to completion, and *that* code path grows
  memory without bound (unlike solver 6, which got an explicit LRU cap on
  its heuristic table in `ai/claude_memleak_fixes.md` — a fix that was
  scoped to solver 6 only). Not fixed here (out of scope for a benchmarking
  task); worth a real investigation of `schedule_plan_flow`
  (`default_planner/scheduler.cpp:660`) and/or the shared heuristic-table
  code (`default_planner/heuristics.{h,cpp}`) in a future session. Resolved
  for this sweep by using default `-t`/`-p` throughout instead of the
  originally-planned elevated values.
- Default `--logDetailLevel` (1, "show all messages") is extremely verbose
  on this map/agent-count combo — one solver-1 run produced >479KB of stdout
  ("Task N is revealed" etc.) in the first 5 seconds alone. This does **not**
  affect the saved `-o` JSON (that's governed separately by `--outputScreen`,
  left at its default `3`, which already trims task/event/path detail from
  the output file), so it doesn't compromise the benchmark data itself — it's
  purely stdout/log noise. Left as default for this sweep since it doesn't
  affect results, but see suggestion below.

## Suggested improvements for future benchmark runs

- Pass `--logDetailLevel 3` (fatal errors only) and/or redirect stdout to
  `/dev/null` instead of a log file — the default verbosity generates
  hundreds of KB to possibly GBs of log text per run across 1000 timesteps x
  20000 agents, which is wasted disk I/O with no effect on the actual result
  JSON.
- If solver 1 turns out to dominate total wall-clock time (it re-solves a
  ~978K-node min-cost flow from scratch every timestep — see
  `ai/claude_memleak_fixes.md`), consider giving it its own shorter `-s` in
  future sweeps, or note its numbers may be time-budget-starved rather than
  reflective of "true" throughput.
- The 3 instance sizes and 5 configs are all independent processes — could
  run several in parallel (if CPU/RAM headroom allows) instead of fully
  sequential, to cut total wall-clock. Not done here to keep resource usage
  predictable and avoid CPU contention skewing the per-timestep timing
  numbers themselves (this benchmark cares about wall-clock solve time, so
  contention from parallel runs could bias results).
