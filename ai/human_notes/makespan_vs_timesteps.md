# Makespan vs. "timesteps solved" — from the ground up

Two different counters come out of the same simulation loop, and while they
usually look identical, they are not the same thing and can diverge badly.

## The loop they both come from

`BaseSystem::simulate()` (`src/CompetitionSystem.cpp:147`) runs one `while`
loop: one iteration per "round" — sync state, call the planner, move the
world. Everything below happens inside that loop.

## `solution_costs[a]` → `makespan`

This is a per-agent counter. It goes up by exactly 1 every time the
**simulated world clock** actually advances by one tick, for every agent that
currently has a goal. It doesn't matter whether that tick was "productive"
(the planner found a real move) or a forced wait — it counts real simulated
time elapsing. `makespan` (the output JSON field) is just the max of this
counter across all agents.

## `len(timeStepMetrics)` → "steps recorded"

This is a **log entry counter** — one entry per call into the planner, not
one per simulated tick. Here's the mechanism that makes it diverge from
`makespan`:

`plan()` (`CompetitionSystem.cpp:91-104`) hands the actual planning work
(scheduler + low-level planner, run together as one packaged task) to a
background thread and *waits* for it, polling in chunks of
`--planTimeLimit` at a time. If the thread isn't done yet, `plan()` doesn't
give up — it loops again, and each extra loop increments a counter called
`timeout_timesteps`. So if the planner takes, say, 5x the time budget,
`plan()` eventually returns having effectively decided "5 timesteps' worth of
clock time passed while I was still thinking."

Back in `simulate()`, that gets reconciled in two pieces:

1. A `for` loop replays `timeout_timesteps` real simulated moves, all with a
   "do nothing" action (`all_wait_actions`), each one correctly bumping
   `solution_costs[a]` — this is real clock time actually elapsing. But it
   appends only **one** `TimeStepMetric` for that *entire batch*, not one per
   replayed tick.
2. Then, separately, the "actual" planned move happens — one more real tick,
   one more `TimeStepMetric`.

So a single trip through the outer loop can advance the real clock by
`timeout_timesteps + 1` ticks, while only adding 1-2 rows to
`timeStepMetrics`. If the planner never times out, `timeout_timesteps` stays
0 and the two counters march in lockstep — which is exactly what happens for
solver 6 in practice (fast enough that it essentially never times out:
"steps recorded" and `makespan` agree at 200 in every solver-6 row of the
`orz900d` sweep). Solver 1, on that map, times out constantly, so its "steps
recorded" (148 for the 5000-agent run) badly undercounts its real elapsed
time (`makespan` = 201).

## Why this matters practically

If you compute throughput as `tasksFinished / steps_recorded`, you divide by
an undercounted denominator whenever the planner times out a lot — this
makes a slow, timeout-prone scheduler look *faster* than it is.
`tasksFinished / makespan` is the real-time-accurate number. The `orz900d`
sweep found this the hard way (solver 1 looked like 14.7 tasks/step until
corrected to 10.8 tasks/step against real elapsed time). **Always use
`tp/makespan`, never `tp/steps`,** when comparing schedulers that don't time
out at the same rate.

## A secondary off-by-one

`makespan` can also overshoot the requested `-s N` by 1: `plan()`'s internal
loop condition (`timestep + timeout_timesteps < simulation_time`) stops
advancing `timeout_timesteps` once the cap is hit, but `simulate()` then
unconditionally does one more real `simulator.move()` regardless (`:202`) —
an off-by-one that only surfaces when the planner is still mid-solve exactly
as time runs out.

---
*Condensed technical version also lives in `ai/project_context.md` under
"Makespan vs. 'timesteps solved'".*
