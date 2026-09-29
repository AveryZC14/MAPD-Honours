# Planner starvation finding + the `USE_MANHATTAN_HEURISTIC` lever

Session: 2026-09-18. Started from a question about which planner backs the
solver-6 runs in `outputs/IH_mp_2p_01_10000_7000ts_sweep/`, and ended up
finding that the low-level planner's traffic-flow optimization is entirely
inert on that map/agent-count, adding a lever to trade heuristic accuracy for
speed, and fixing a real, previously-latent OOM bug the lever exposed.

## Finding: the low-level planner degrades to pure `causalPIBT` on this sweep

`default_planner/planner.cpp`'s `plan()` does three things per decision, in
order, all sharing one `end_time` computed once at function entry
(`start_time + time_limit - PIBT reserve - tolerance`, where `time_limit`
itself is already `1000ms - scheduler time this decision - PLANNER_TIMELIMIT_TOLERANCE`,
see `src/MAPFPlanner.cpp:35`):

1. **Heuristic-table build loop** (`planner.cpp:145-213`) -- iterates every
   agent (no early exit), building `init_heuristic()` for any goal location
   not already cached, but only while `now() < end_time`; once that trips,
   remaining agents just skip table-building for this decision (their other
   bookkeeping -- task assignment, PIBT priority -- still runs regardless).
2. **Guide-path recompute loop** (`planner.cpp:216-239`) -- calls
   `update_traj()` (runs `astar`) for any agent whose path is stale, but
   `break`s out entirely, before touching agent 0, if `now() > end_time`.
3. **`frank_wolfe()`** (`flow.cpp:96-132`, the traffic-flow LNS that
   iteratively re-routes agents against the shared congestion field) --
   shares the same `end_time`; its own `while (now() < timelimit)` loop
   condition is checked strictly after step 2's check already failed, so if
   step 2 found no time left, `frank_wolfe` runs zero iterations too.
4. **`causalPIBT`** -- runs unconditionally every decision, for every agent,
   regardless of the above. It doesn't need a precomputed path: it's a
   greedy one-step move selector, scoring each candidate next-cell with
   `get_gp_h()` (`pibt.cpp:12-23`) and picking the best collision-free one.

**On `IH_mp_2p_01`, 10000 agents, `--scheduleModel 6 --flowSolveLevel 4`**
(the existing `IH_mp_2p_01_10000_solver6_level4.json` / `.log`, 7000-timestep
horizon), step 2 never ran a single iteration in the entire run:
`grep -c "compute initial stop until" ...level4.log` returns **1562/1562** --
every logged decision hit the time check before agent 0. Since step 3 shares
the same expired `end_time`, it never ran either. So this run -- and by
extension the whole `IH_mp_2p_01_10000_7000ts_sweep` folder's solver-6
results, all of which use the same map/agent-count -- is **pure greedy
`causalPIBT`, re-decided from scratch every timestep, with no lookahead and
no traffic-flow optimization**, despite `ai/project_context.md`'s pipeline
description implying the latter always runs. `PlannerTime` dominating total
wall-clock in that sweep (established in `ai/hierarchical_matching.md`'s
wall-clock table) is therefore `causalPIBT` iterating 10000 agents/decision,
not `astar`/`frank_wolfe` cost as might be assumed.

Loop 1 is *not* all-or-nothing the way loop 2 is -- it always finishes all
10000 agents, it just stops building *new* tables partway through once time
runs out (existing agents/goals with an already-cached table still get a
free hit via `global_heuristictable`, keyed by goal location, not by agent).
There's no log line marking where in the 10000-agent list that cutoff lands,
so whether/how many agents are running `get_gp_h()` on Manhattan distance
(its lowest-priority fallback, `pibt.cpp:20`, already present in the code
before this session) purely from loop-1 starvation was inferred, not
directly confirmed, prior to the lever below.

## The lever: `USE_MANHATTAN_HEURISTIC` (`default_planner/const.h`)

```cpp
const bool USE_MANHATTAN_HEURISTIC = false;  // default: off, no behavior change
```

When `true`, every heuristic lookup in the low-level planner returns raw
Manhattan distance instead of the exact BFS-from-goal distance table, and
the code stops building/touching the tables that back that exact distance
entirely (not just falling back when a table happens to be missing --
actively skipping the build). Four gated sites:

- **`heuristics.cpp: get_h()`** -- the funnel solver 3/4 (matching-based
  schedulers) call via `scheduler.cpp:173,304`.
- **`pibt.cpp: get_gp_h()`** -- PIBT's per-move heuristic, the one thing
  that runs unconditionally every decision regardless of starvation.
- **`search.cpp: astar()`** -- both heuristic evaluations inside the
  guide-path A* (this function already had a Manhattan fallback for
  `ht.empty()`, predating this session; the flag now forces the same branch
  unconditionally rather than only when the table happens to be unbuilt).
- **`planner.cpp`'s heuristic-table-build loop** (step 1 above) -- skipped
  entirely under the flag, so the reclaimed time is actually usable by PIBT
  instead of being spent building tables nothing reads.

Verified both flag states build cleanly and complete a run on
`instances/custom/tiny/tiny.json` before doing anything else with it.

## Bug found while testing it: `Dist2Path` is an unbounded per-agent table

First real-scale test (`--scheduleModel 6 --flowSolveLevel 4`,
`IH_mp_2p_01`, 10000 agents, `-s 500`) **OOM-killed at timestep 151/500**,
RSS 30.4GB (`dmesg`/`journalctl -k`: `Out of memory: Killed process ...
(lifelong) ... anon-rss:30427628kB`).

Root cause, **not the Manhattan heuristic itself**: `Dist2Path::dist2path`
(`default_planner/Types.h:99-107`) is a *per-agent* vector sized
`env->map.size()` once built (`d2p` = 4 ints = 16 bytes/cell -- ~55MB/agent
on this ~3.44M-cell map), built by `init_dist_2_path()` inside
`update_traj()` (`flow.cpp:162-168`) every time an agent gets a fresh guide
path, with **no eviction** -- unlike `global_heuristictable`, which got an
LRU cap during the original OOM investigation
(`ai/claude_memleak_fixes.md`). It never surfaced before because
`update_traj()` essentially never ran at this scale under the exact
heuristic (see "Finding" above -- starved before agent 0, every decision).
The Manhattan lever made `astar()` cheap enough that the guide-path loop
stopped stalling (log: 400-1800+ agents/decision processed instead of 0), so
agents started actually accumulating `Dist2Path` tables with nothing
bounding them. ~600 distinct agents x ~55MB lines up closely with the
observed RSS.

**`update_traj()`, step by step** (for anyone reading this cold):
1. `astar(...)` -- searches from the agent's current location to its task's
   goal, biased by `lns.flow` (the shared congestion field: how many other
   agents' paths currently cross each edge). Overwrites `lns.trajs[i]`.
2. `add_traj(lns, i)` (`flow.cpp:36-55`) -- registers the new path into
   `lns.flow`, so *other* agents' subsequent `astar()` calls route around
   it. This is the actual traffic-flow-awareness mechanism.
3. `update_dist_2_path(lns, i)` -> `init_dist_2_path()`
   (`heuristics.cpp:140-156`) -- the OOM culprit. Runs a reverse multi-source
   BFS from every point along the new path, building the table
   `get_dist_2_path()` later uses so `causalPIBT` can cheaply ask "how far
   is agent `i` from its own guide path, and how much further from there to
   the goal" without re-searching every timestep.

**Fix**: gated both places that call `update_dist_2_path()` --
`update_traj()` (`flow.cpp:167`) and the scheduler-guide-path-seed branch in
`planner.cpp:234` (only live with `--useTraffic`, covered for completeness)
-- behind `!USE_MANHATTAN_HEURISTIC`. Safe because its only two consumers
already degrade gracefully when `traj_dists[i]` stays empty:
`get_gp_h()` already short-circuits to Manhattan *before* ever checking
`traj_dists` when the flag is on, so it never reads the table regardless;
`update_fw_metrics()`/`get_deviation()` (`flow.cpp`, used by `frank_wolfe`'s
replan-priority ordering) both explicitly skip agents with an empty
`traj_dists[i]` rather than crashing -- `frank_wolfe` just falls back to
sorting by least-recently-replanned instead of true path deviation.
`get_deviation()` itself turned out to be dead code (never called anywhere)
while checking this.

Rebuilt, re-verified on `tiny.json` at both flag states, then re-ran the
500-timestep probe with RSS watched every 20s throughout: held flat around
4.5GB the entire run, completed cleanly to `makespan=500`, 0 errors.

## Results: 500-timestep `IH_mp_2p_01` probe (post-fix)

| metric (first 500 real timesteps) | baseline (exact heuristic)¹ | Manhattan (fixed) |
|---|---|---|
| decisions (steps) | 98 | 376 (3.8x) |
| tasksFinished | 3276 | 3485 (+209, +6.4%) |

¹ sliced from the existing `IH_mp_2p_01_10000_solver6_level4.json` (7000ts
baseline run) by `Timestep <= 500`, not a fresh `-s 500` re-run of the
baseline -- same instance/flags, but not controlled for the run-to-run
wall-clock jitter this repo has documented elsewhere on identical configs
(a few percent, `ai/hierarchical_matching.md`'s "Validation" section).

More decisions translated into more tasks finished in the same window, not
just more decisions for their own sake -- the Manhattan run is exercising
`astar`/`frank_wolfe` for real now (hundreds to low-thousands of
agents/decision through the guide-path loop, vs. 0 in the baseline's starved
regime), so the improvement isn't purely "cheaper heuristic, same
behavior" -- it's "the traffic-flow planner is functioning again, just
against a less accurate distance estimate than before."

> **Correction (2026-09-29):** the paragraph above is wrong about
> `frank_wolfe`. In Manhattan mode no agent has a `traj_dists` table, and
> `frank_wolfe` skips any agent without one (`flow.cpp:123`), so it runs
> zero re-plans and just spins until the deadline. Only stage 2's first
> A* per new goal ran. PIBT also never reads the guide paths in this mode,
> so the +6.4% came from greedy Manhattan PIBT deciding more often. The
> 500-step window also ended before the freeze that appears around step
> 1,000 (agents trapped behind walls). See `ai/planner_local_bfs_plan.md`.

**Not yet done**:
- A real head-to-head: fresh `-s 500` baseline run rather than a slice, and
  ideally more than one run per side given the jitter caveat.
- Path-quality/congestion comparison -- Manhattan's individual move choices
  are less informed (blind to walls) even though aggregate throughput rose;
  this probe only measured tasksFinished, not path length or collision/wait
  overhead.
- The same probe at deeper `--flowSolveLevel`s or the full 7000-timestep
  horizon, to see whether the advantage holds or changes shape over a longer
  run -- the backbone-rebuild-cost dynamic in `ai/todo.md` means short- and
  long-horizon results haven't always agreed on solver-6-side questions in
  this repo.
- Instrumenting loop 1's actual cutoff point (which agent index the
  heuristic-table build stops at, under the *default* non-Manhattan
  config) -- still inferred, not directly confirmed, for the baseline runs.

## Files changed

- `default_planner/const.h` -- new `USE_MANHATTAN_HEURISTIC` constant.
- `default_planner/heuristics.cpp` -- `get_h()` gate; added `#include "const.h"`, `#include "utils.h"`.
- `default_planner/pibt.cpp` / `pibt.h` -- `get_gp_h()` gate; added `#include "const.h"` to the header.
- `default_planner/search.cpp` / `search.h` -- both `astar()` heuristic-evaluation sites gated; added `#include "const.h"` to the header.
- `default_planner/planner.cpp` -- heuristic-table-build loop gate, plus the scheduler-guide-path-seed `update_dist_2_path()` gate.
- `default_planner/flow.cpp` -- `update_traj()`'s `update_dist_2_path()` gate.

## Output / run log

- `outputs/IH_mp_2p_01_10000_7000ts_sweep/IH_mp_2p_01_10000_solver6_level4_manhattan_500ts.{json,log}` --
  the successful post-fix probe run.
- `ai/run_log.md`'s 2026-09-18 entry has the blow-by-blow (OOM kill details,
  fix, re-run, this same results table) logged as it happened.
