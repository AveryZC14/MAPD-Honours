# Thesis sweep, first attempt (2026-09-29): pure-Manhattan planner freeze

Moved here from `outputs/thesis_sweep/` on 2026-09-29 so the sweep can be
redone from a clean folder. Kept because the data is useful: it's the
evidence for the freeze and the before/after comparison for the fix.

**Don't use these as thesis results.** They were run with
`USE_MANHATTAN_HEURISTIC = true` before the `USE_LOCAL_PATH_BFS` fix
existed. The planner scored moves by Manhattan distance to the goal, so
agents behind walls waited forever. Every run stopped making progress at
about step 1,000 (0 pickups and 0 deliveries for the last 7,000 steps).

Contents:

- `orz900d_10000_solver6_level{2,4,6,8}.{json,log,time}`: the 4 finished
  runs, 8,000 steps each.
- `orz900d_10000_solver6_level11_hieronly.{log,time}`: the 5th run, stopped
  part-way when the sweep was stopped at 16:50 (no JSON).
- `runner.log`, `sweep_summary.csv`, `sweep_meta.json`,
  `uncommitted_changes_at_start.patch`, `watchdog.log`: the sweep's own
  bookkeeping for this attempt (build, commit and machine it ran on).

See `ai/run_log.md` (2026-09-29), `ai/planner_local_bfs_plan.md` and
`instances/thesis_benchmarks/README.md` ("Sweep status").
