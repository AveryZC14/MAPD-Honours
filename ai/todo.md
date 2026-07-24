# Project todo list

Running list of follow-up work the user wants tracked across sessions.
Unlike the other `ai/*.md` docs (which record what's already been
investigated/done), this one is forward-looking — check it at the start of a
session for open items, and check items off (move to a "Done" section with
the date and what changed) rather than deleting them outright.

## Open

- [ ] **New output metric: timesteps actually scheduled/planned for, distinct
  from `makespan`.** `makespan` (see `ai/project_context.md` "Makespan vs.
  'timesteps solved'") counts real elapsed simulated ticks, including
  forced-wait ticks replayed while a slow scheduler/planner call was still
  running. The existing `len(timeStepMetrics)` isn't the right thing either —
  it appends 1 entry for a whole burst of catch-up ticks plus 1 more for the
  real move, i.e. up to 2 log rows per *single* underlying planning decision,
  not a count of decisions itself. What's wanted is a clean counter of how
  many times the scheduler+planner were actually invoked and produced a
  fresh decision — i.e. the number of `BaseSystem::simulate()` outer-loop
  iterations / `plan()` calls that returned a real result, as opposed to
  `makespan`'s count of simulated clock ticks (real + forced-wait). Natural
  place to add it: a counter incremented once per outer-loop iteration in
  `BaseSystem::simulate()` (`src/CompetitionSystem.cpp:147`), written to the
  output JSON alongside `makespan` in `saveResults()` (`:258`). Would make it
  possible to directly see, e.g., "solver 1 only got 6 real scheduling
  decisions in during 201 simulated timesteps" instead of inferring it from
  the "steps recorded" workaround used in `ai/auto_benchmarking.md`.

- [ ] **Split `PlannerTime` into its scheduler and path-planner components.**
  Confusing right now: the `PlannerTime` field in each timestep's output
  (`TimeStepMetric::PlannerTime`, `inc/CompetitionSystem.h:19`) isn't just the
  low-level path planner — it's the wall-clock of the *entire*
  `Entry::compute()` call, which runs `scheduler->plan()` (task assignment,
  varies by `--scheduleModel`) followed by `planner->plan()` (low-level
  pathfinding, same code every solver). It's set from a single
  `std::chrono` measurement wrapping both in `BaseSystem::plan()`
  (`src/CompetitionSystem.cpp:162-211`). Meanwhile `SchedulerSolveTime` /
  `SchedulerGuidePathTime` are already captured separately via
  `last_scheduler_timing` (`ScheduleTiming` struct, `default_planner/scheduler.h:18`,
  populated by `set_last_timing`/`set_last_reduced_timing` in
  `default_planner/scheduler.cpp:18-38`) but aren't subtracted out anywhere,
  so it's easy to misread `PlannerTime` as pure path-planning cost when
  solver 1 vs. solver 6 differences are actually dominated by scheduler cost
  (see `ai/auto_benchmarking_IH_mp_2p_01.md`).
  - Rename/keep `PlannerTime` as `TotalPlanTime` (the full `Entry::compute()`
    wall-clock, what's measured today).
  - Add a `PathPlannerTime` field = `TotalPlanTime - SchedulerSolveTime -
    SchedulerGuidePathTime` (or time `planner->plan()` directly for
    precision instead of subtracting, since `planner_wrapper()` in
    `src/CompetitionSystem.cpp:53-70` already calls `scheduler->plan()` and
    `planner->plan()` — wait, actually `scheduler->plan()` and
    `planner->plan()` are called inside `Entry::compute()`
    (`src/Entry.cpp:32,38`), not directly in `planner_wrapper()` — timing
    would need to move into `Entry::compute()` itself, or `Entry` would need
    to expose per-call timings the way the scheduler already does).
  - Keep `SchedulerSolveTime` / `SchedulerGuidePathTime` as-is (already
    correct, just underused).
  - Update `visualisation/compute_throughput_metrics.py` and the two
    `ai/auto_benchmarking_*.md` docs' methodology notes once the new field
    exists, so future sweeps read `PathPlannerTime` instead of misreading
    `PlannerTime`/`TotalPlanTime` as planner-only cost.

- [ ] **(Suggestion, not yet requested) Find the solver-1-vs-solver-6
  crossover map size.** `orz900d` (~978K cells): solver 1 wins. `IH_mp_2p_01`
  (~3.44M cells): solver 6 wins by ~4-7x. Somewhere between those two map
  sizes the relationship flips — a sweep on an intermediate-sized map would
  pin down roughly where, which seems like a genuinely useful data point for
  the thesis's scaling argument. See `ai/auto_benchmarking.md` synthesis
  section. Flagging this because it fell out of the `IH_mp_2p_01` sweep, not
  because it's been asked for.

## Done

(none yet)
