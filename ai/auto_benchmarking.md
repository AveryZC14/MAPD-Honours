# Benchmarking: index, cross-sweep synthesis, and reusable methodology

This file is the entry point for all solver-1-vs-solver-6 benchmark sweeps in
this repo. It intentionally does **not** contain full per-sweep methodology
or result tables — those live in one detail file per sweep (see the index
below). This file is for: (1) finding the right detail file, (2) what we
currently believe *across* sweeps, and (3) mechanics/gotchas that would
otherwise get rediscovered or duplicated in every new sweep's detail file.

Kept as flat files under `ai/` (not a subfolder) since the intent is to
eventually automate the benchmarking process itself rather than keep growing
this by hand — see `ai/todo.md`.

## Sweep index

| Sweep | Map | Date | Agent counts | Configs | Takeaway | Detail file |
|---|---|---|---|---|---|---|
| 1 | `orz900d` (656x1491, ~978K cells) | 2026-07-22 | 5000/10000/20000 | solver 1; solver 6 @ coarsen levels 1-4 | Solver 1 wins on `tp/makespan` at every agent count, margin shrinking as agents increase. | `ai/auto_benchmarking_orz900d.md` |
| 2 | `IH_mp_2p_01` (1912x1800, ~3.44M cells, ~2.22M walkable) | 2026-07-23 | 5000/10000/20000 | solver 1; solver 6 @ coarsen depth 2 and 4 | **Inverts sweep 1**: solver 6 wins by ~4-7x on `tp/makespan` at every agent count. | `ai/auto_benchmarking_IH_mp_2p_01.md` |

## Cross-sweep synthesis (what we currently believe)

- **Solver choice matters far more on a bigger map, and the winner flips.**
  On `orz900d` (~978K cells) solver 1 beats every solver-6 coarsen level
  tested, at every agent count. On `IH_mp_2p_01` (~2.3x more cells) that
  reverses hard — solver 6 wins by 4-7x instead. The mechanism is exactly
  what the thesis's coarsening approach targets: solver 1 rebuilds a
  full-map flow graph every timestep (cost scales with *map* size,
  independent of agent count), while solver 6's real per-timestep cost is
  the small coarse top-level graph (cost roughly independent of map size).
  Below some map-size threshold solver 1's fixed cost is cheap enough to
  win anyway; above it, it isn't. **Open question / good next sweep**: find
  that crossover point with an intermediate-sized map — see `ai/todo.md`.
- **Coarsen depth is a second-order effect compared to solver choice.**
  Within solver 6, more coarsening (higher level/depth) consistently costs a
  small amount of throughput (level 1 > 2 > 3 > 4 on `orz900d`; depth 2 >
  depth 4 on `IH_mp_2p_01`) — single-digit percent, not the multi-x gap
  between solver 1 and solver 6.
- **Solver 1's own numbers are probably conservative, not just "worse."** On
  both maps it hits "planner timeout" constantly even at the default
  `--planTimeLimit`, meaning it's time-budget-starved rather than running to
  its own natural completion each timestep. Its `tp/makespan` figures should
  be read as "under a shared, practical time budget," not "at solver 1's true
  unconstrained quality."

## Reusable methodology (read before running or interpreting a new sweep)

### `tp/steps` vs `tp/makespan` — always prefer `tp/makespan`

The simulator's output JSON has two different counters that both look like
"number of timesteps" but aren't:

- **"steps recorded"** (`len(timeStepMetrics)`): one log entry per pass
  through `BaseSystem::simulate()`'s main loop (`src/CompetitionSystem.cpp`).
  When the planner/scheduler call takes long enough to time out and force
  several "wait" timesteps to catch up, that whole multi-timestep batch still
  only adds 1-2 log entries — so this **undercounts** real elapsed simulated
  time whenever a scheduler is slow or times out a lot.
- **`makespan`**: the max, over all agents, of a per-agent counter
  incremented once per *real* elapsed simulator timestep (including forced
  wait-timesteps). This is the accurate measure of how much simulated time
  actually passed.

Dividing `tasksFinished` by "steps" therefore **overstates** throughput for
any run where the scheduler is slow/times out a lot — this is not a small
effect: it inflated solver 1's apparent throughput by ~26% on `orz900d` and
by orders of magnitude on `IH_mp_2p_01` (up to 177.5 vs. the real 8.83
tp/makespan at 20000 agents — solver 1 needed only 10 real scheduling
decisions to cover 201 simulated timesteps there). **Always read
`tp/makespan`, never `tp/steps`,** when solvers being compared don't time
out at the same rate — which, so far, is every sweep. Full mechanical
derivation (why the two counters diverge, worked through from the actual
`simulate()` loop) is in `ai/human_notes/makespan_vs_timesteps.md` and
`ai/project_context.md`.

`visualisation/compute_throughput_metrics.py` generates the full table (file,
agents, tasks, steps, makespan, tp/steps, tp/makespan) as CSV from a single
result JSON or a folder of them — use it instead of hand-computing:

```shell
python3 visualisation/compute_throughput_metrics.py outputs/<folder>/ -o metrics.csv
```

### Hierarchy build is mandatory for *every* solver, not just solver 6

`schedule_initialize()` (`default_planner/scheduler.cpp:63`) unconditionally
builds the reduced hierarchy (`ReducedHierarchy::ensure()`) regardless of
`--scheduleModel` — solver 1 pays this cost too, even though it never uses
the result. On `orz900d` this was fast enough to be invisible; on
`IH_mp_2p_01` (~2.3x more cells) it took ~300s and **the default
`--preprocessTimeLimit 30000` wasn't enough for any solver**, failing every
run with a fatal preprocessing timeout until `-p` was raised to 600000.
**Before benchmarking a new, bigger map: check hierarchy-build time first**
(e.g. via `./build/map_reduction_test <instance.json>`, which builds the
hierarchy directly without running a full simulation) and size
`--preprocessTimeLimit` accordingly, for every solver in the sweep, not just
solver 6.

### `kDefaultCoarsenLevels` is compile-time only

`map_reduction_test/MapCoarsenV1.cpp:31` — a `constexpr int`, repo default
`2`. Comparing coarsen levels/depths means editing that line and
`./compile.sh`-ing between passes; there's no CLI flag. Both sweeps so far
used a `sed` + recompile script, restoring the value to `2` (and rebuilding
again) at the end so the repo is left as found. Minimize recompiles by
grouping all runs that share a level together (solver 1 doesn't touch this
constant at all, so its runs can be bundled into whichever pass is
convenient).

### Other things worth checking before trusting a new sweep's numbers

- `numPlannerErrors` / `numScheduleErrors` / `numEntryTimeouts` should be 0 in
  every output JSON — both sweeps so far are clean across all runs, but this
  is worth checking per-run, not assumed.
- `--logDetailLevel 3` (fatal errors only) — the default (1) is extremely
  verbose (hundreds of KB+ of stdout) and doesn't affect the saved result
  JSON, but wastes disk/time at scale.
- If comparing solver 1 across many agent counts on a large map, its
  per-timestep cost may dominate total sweep wall-clock (it doesn't scale
  down with fewer agents the way solver 6 does — the flow graph is built
  over the whole map regardless). Consider giving it its own shorter `-s` if
  this becomes impractical, though so far (`-s 200` on both maps) it hasn't
  been necessary.
