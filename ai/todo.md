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
