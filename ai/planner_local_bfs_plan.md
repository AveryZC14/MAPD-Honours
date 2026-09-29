# Local BFS to the guide path: fix for the Manhattan-mode planner freeze

Status: **implemented 2026-09-29**, steps 1-5 below. Step 6 (re-enabling
`frank_wolfe`) is not done. This fix is needed before the thesis sweep
(`instances/thesis_benchmarks/README.md`, "Final run plan") can restart.
Background: `ai/manhattan_heuristic_lever.md` (why the Manhattan flag
exists) and `ai/run_log.md` 2026-09-29 (the freeze, the controlled test that
pinned it on the flag, and this fix's first results).

This doc was written as a plan first. "As implemented" below records what
was actually built; the rest is the original plan and reasoning.

## As implemented

**Switches** (`default_planner/const.h`, all compile-time):

| Constant | Default | Meaning |
|---|---|---|
| `USE_MANHATTAN_HEURISTIC` | `true` | Existing lever. The new code only runs when this is `true`. |
| `USE_LOCAL_PATH_BFS` | `true` | The fix. `false` gives exactly the old pure-Manhattan planner (apart from the log line below). Set from the macro `PLANNER_USE_LOCAL_PATH_BFS`, so a second build can override it with `cmake ... -DCMAKE_CXX_FLAGS="-DPLANNER_USE_LOCAL_PATH_BFS=false"` without editing the file. |
| `LOCAL_PATH_BFS_RADIUS` | `5` | BFS radius cap. |
| `LOCAL_PATH_BFS_EXTRA_LAYERS` | `2` | Layers searched after the first path cell is found. |
| `PASS_SCHEDULER_PATHS_TO_PLANNER` | `false` | Solvers 6/7 hand their guide paths to the planner every step (normally only with `--useTraffic` past timestep 100). Untested. Leave off for the sweep. |
| `STUCK_AGENT_THRESHOLD` | `20` | Decisions without moving (while having a goal) before an agent counts as stuck in the log line. |

No scheduler is changed by `USE_LOCAL_PATH_BFS`. It only changes the
low-level planner, which every solver shares, so it applies equally to
solvers 1-7.

**Code:**

- `pibt.cpp`: `local_path_scores()` runs the BFS and returns the scores for
  the agent's neighbours and for waiting. `causalPIBT()` uses them in place
  of `get_gp_h()` when the flag is on and the BFS found a path cell;
  otherwise it falls back to `get_gp_h()` (Manhattan distance to the goal).
- `flow.cpp`: `update_path_togo()` builds the path cell → steps-left map;
  called from `update_traj()` and for scheduler-provided paths. It also
  clears the agent's `needs_replan`.
- `planner.cpp`: stage 2 starts from `guide_path_start` (where it stopped
  last step) instead of agent 0; agents with `needs_replan` are given a new
  path; stuck-agent counting; the `planner stats:` log line.
- `TrajLNS.h`: `path_togo` and `needs_replan` per agent, and the shared BFS
  scratch arrays (`bfs_stamp`, `bfs_dist`, `bfs_moves`, three map-sized
  arrays in total, about 9 bytes per cell).
- `scheduler.cpp`: the `PASS_SCHEDULER_PATHS_TO_PLANNER` condition for
  solvers 6 and 7.
- `scripts/run_thesis_sweep.py`: `sweep_meta.json` records all of the
  switches above.

**Details decided during implementation:**

- **An out-of-date path is not used.** If the agent's path doesn't end at
  its current goal (the goal changed and stage 2 hasn't re-planned it yet),
  the BFS is skipped and the agent uses Manhattan distance for that step.
  Otherwise the old path would pull it back toward its previous goal.
- **Re-planning a stray agent keeps its PIBT priority.** `needs_replan`
  sets `require_guide_path` after the priority reset in stage 1, because it
  isn't a new goal.
- **A cell that appears twice in a path keeps its smaller `togo`.**
- **`compute initial stop until <i>` changes meaning.** With the flag on,
  `<i>` is the agent where the rotated loop stopped, not a count of agents
  processed. Use `paths_built` in the `planner stats:` line instead. (Older
  docs, e.g. `ai/manhattan_heuristic_lever.md`, count "stop until 0" lines
  to detect starvation; that still works only with the flag off.)
- **The stuck count and `planner stats:` line are logged whichever way the
  flag is set,** so pure-Manhattan runs can be compared directly. It's
  printed with `cout` like the existing `compute initial stop until` line,
  so `--logDetailLevel` doesn't hide it. It adds one line (about 250 bytes)
  per decision to each run's log.

**`planner stats:` fields** (one line per decision):

| Field | Meaning |
|---|---|
| `setup_ms` | Stage 1 (per-agent setup). |
| `guide_ms` / `paths_built` | Stage 2: time, and number of agents given a new path. |
| `fw_ms` | Stage 3, `frank_wolfe`. In Manhattan mode it re-plans nothing and fills the time to the deadline. |
| `pibt_ms` / `pibt_reserve_ms` | Stage 4 time, and the time set aside for it (1 ms per 100 agents). If `pibt_ms` goes above the reserve, decisions start running over. |
| `stuck_agents` | Agents with a goal that haven't moved for `STUCK_AGENT_THRESHOLD` decisions. |
| `bfs_no_path` | Agents scored by Manhattan distance because they had no path to their current goal yet. |
| `bfs_too_far` | Agents with no path cell within the radius; they get a new path next step. |
| `bfs_cells` | Total cells the BFS visited this decision. |

**Tests (2026-09-29):**

- `tiny.json`, 200 steps: 14 tasks finished, 0 errors.
- Hand-built trap (15×15 map, agent inside a cup opening north, pickup
  directly south of the cup's bottom wall), 40 steps, solver 5: pure
  Manhattan never moved (stuck from step 20); the new planner walked out and
  reached the pickup.
- orz900d 10k thesis instance, 1,500 steps, 3 runs in parallel:

  | Run | Deliveries at 500 / 1,000 / 1,500 | Stuck at 1,500 |
  |---|---|---|
  | New planner, solver 6 level 4 | 2,037 / 3,894 / 5,495 | 7 |
  | Pure Manhattan, solver 6 level 4 | 614 / 617 / 617 | 10,000 |
  | New planner, solver 5 | 2,062 / 3,943 / 5,530 | 4 |

  All made 1,499-1,500 decisions in 1,500 steps, 0 errors. New planner PIBT
  time: mean 37 ms, p99 52 ms, max 77 ms (reserve 100 ms). Guide-path A*
  takes about 12 ms per search, so the initial 10k backlog took about 250
  steps to clear; until then agents without a path used Manhattan distance
  (3,679 at step 100, 726 stuck). After that, 10-25 new paths per step.

**IH result (2026-09-29): not good enough yet.** IH_mp_2p_01 10k, 1,500
steps: 2,364 deliveries vs. 1,178 for pure Manhattan, but about 6,700
agents never got a guide path, because each A* search takes about
70-300 ms on IH (3-14 paths per step). Those agents fall back to Manhattan
distance and get trapped (4,661 stuck at decision 1,000). A* also overruns
the deadline (1,021 decisions in 1,500 steps). The BFS and PIBT are fine
(max 35 ms). `PASS_SCHEDULER_PATHS_TO_PLANNER` made no difference, since
solver 6 matches nearly every agent locally and those get no scheduler
path. Full numbers in `ai/run_log.md`. Ideas to fix, not yet tried:

- **Better A* heuristic** (option C below: landmark distances, or distances
  on the coarsening hierarchy). Manhattan distance is a poor guide on a big
  maze, so A* expands a large part of the map per search.
- **Keep A* from overrunning:** cap expansions per search, or check the
  deadline inside the search and resume next step.
- **Interim guidance for agents without a path:** e.g. follow the
  coarse-hierarchy route toward the goal until a fine path exists, instead
  of Manhattan distance.
- **Also measure** how much of each search's cost comes from the congestion
  terms in A*'s ordering (`search_node.h:116`), which make it explore more
  than plain A*.

**Still open** (see `ai/todo.md`): 20k agents and larger maps (PIBT's
margin), `PASS_SCHEDULER_PATHS_TO_PLANNER`, step 6 (`frank_wolfe`), and
committing.

## The problem in one paragraph

With `USE_MANHATTAN_HEURISTIC = true` (`default_planner/const.h`), PIBT
scores each candidate move by raw Manhattan distance to the goal
(`get_gp_h`, `pibt.cpp:15-16`). On a 4-connected grid every move changes
Manhattan distance by exactly +1 or -1. An agent in a pocket with a wall
between it and its goal therefore sees every open move score `h+1` and
waiting score `h`. It waits, every step, forever. Agents collect in these
traps until throughput reaches zero: the 2026-09-29 orz900d 10k runs froze
at about step 1,000. The guide paths that would route around the walls are
still computed (stage 2 of `plan()`), but in this mode nothing that moves an
agent reads them.

## How the planner works today (for reference)

Each timestep, `plan()` (`planner.cpp:119`) runs four stages. All stages
before PIBT share one deadline: the time limit, minus 1 ms per 100 agents
for PIBT, minus 60 ms.

1. **Setup** (`planner.cpp:145-213`): sets goals and priorities, and marks
   an agent as needing a new guide path if it has none or its path doesn't
   end at its current goal. With the flag off, it also builds goal distance
   tables.
2. **Guide paths** (`planner.cpp:216-240`): for marked agents, either
   takes the scheduler's path (only with `--useTraffic`, from timestep 100)
   or runs `update_traj` -> `astar`. The A* avoids congestion: it expands
   nodes in order of `f + contra-flow + vertex-flow`
   (`search_node.h:116`), using `lns.flow`, the count of guide paths per
   cell and direction. The loop always starts at agent 0 and stops
   completely at the deadline.
3. **`frank_wolfe`** (`flow.cpp:96`): re-plans existing paths until the
   deadline. It skips agents whose `traj_dists` table is empty
   (`flow.cpp:123`), which is every agent in Manhattan mode, so in that
   mode it spins doing nothing until the deadline.
4. **PIBT** (`pibt.cpp:26`): every agent, in priority order, picks the
   best-scoring free cell among its 4 neighbours and waiting, and can push
   lower-priority agents.

Guide paths persist across timesteps. They are replaced only when the goal
changes (pickup reached, delivery, reassignment), or by `frank_wolfe` when
the flag is off. They are never trimmed as the agent moves, and pushing an
agent off its path doesn't replace it.

**Flag off:** PIBT scores a cell by distance to the agent's guide path plus
the steps left along the path from there (`Dist2Path`, a lazy BFS starting
from every path cell). That respects walls and has no traps in practice. At
10k agents it fails in two ways. Time: stage 2 never reached agent 0 on
IH 10k (1562/1562 decisions), and it managed 543 decisions in 1,500 steps
on orz900d. Memory: `Dist2Path` allocates a whole-map array per agent,
15.6 MB on orz900d, with nothing ever freed.

**Flag on:** stage 1 skips goal tables, A* uses Manhattan distance as its
heuristic (paths are still correct), no `Dist2Path` is built, `frank_wolfe`
does nothing, and PIBT scores by Manhattan distance to the goal. Fast, but
it traps agents.

## The fix

The rule a fix has to satisfy: PIBT's score must be a real walking
distance, so that from any cell other than the goal some neighbour scores
strictly lower than waiting. Estimates that never exceed the real distance
(Manhattan, landmark distances, coarse-hierarchy distances) are fine as an
A* heuristic but create traps in greedy PIBT.

Keep everything that made flag-on mode fast (no goal tables, Manhattan
heuristic in A*). Change only how PIBT scores cells, and do it with one
small BFS per agent per step instead of `Dist2Path`.

### 1. Per-path `togo` lookup

When an agent gets a new guide path (from `update_traj` or from the
scheduler), build a small map from each path cell to its steps remaining to
the goal (`togo`). Cost O(path length), about 700 entries on orz900d. It is
built once per path, not per step.

If a path passes through the same cell twice (it shouldn't, since A* paths
are simple, but scheduler paths are spliced together), keep the smallest
`togo`.

### 2. Local BFS from the agent, scored with the ±1 rule

At the start of PIBT for agent `a`, BFS outward from `a`'s cell up to
radius `R`. For each cell `x` reached, record:

- `dist[x]` = `d(a, x)`
- `first_moves[x]` = a 4-bit mask of which of `a`'s neighbours begin a
  shortest route from `a` to `x` (copied from the parent cell, OR-ed
  together when two routes tie)

The grid is 4-connected, so moves alternate between "black" and "white"
squares like a chessboard. For any neighbour `n` of `a` and any cell `p`,
`d(n, p)` is therefore exactly `d(a, p) - 1` if `n` is in
`first_moves[p]`, and `d(a, p) + 1` otherwise. So the one BFS gives exact
distances from all four neighbours:

```
for each neighbour n of a:
    score(n) = min over path cells p reached by the BFS of
               (first_moves[p] has n ? dist[p] - 1 : dist[p] + 1) + togo[p]
score(wait) = min over p of dist[p] + togo[p]
```

`get_gp_h` returns these precomputed scores instead of Manhattan distance.
`causalPIBT` decides each agent once per step, so each agent's BFS runs at
most once per step.

**Why this doesn't trap.** Let `p*` be the path cell that gives
`score(wait)`. The neighbour that starts a shortest route to `p*` scores
exactly `score(wait) - 1`. So whenever the BFS finds any path cell, some
move scores strictly lower than waiting, except at the goal
(`togo = 0`, distance 0). This is a slightly stronger guarantee than
upstream `Dist2Path`, which uses whichever path cell its BFS reaches first
rather than the best one.

**Stopping rule.** Stop a couple of layers after the first path cell is
found, with a hard cap `R` (start with about 5). The guarantee above only
needs one path cell found. The extra layers let the minimum pick a better
path cell when one is close by.

**Visited bookkeeping.** Use one whole-map `stamp` array shared by all
agents. Increment the stamp for each BFS so nothing needs clearing. The
per-BFS `dist`/`first_moves` can live in a small scratch structure keyed by
cell, or in two more shared whole-map arrays stamped the same way. That's
three map-sized arrays in total, not one per agent.

### 3. Agents too far from their path

If the BFS reaches radius `R` without finding a path cell, the agent has
been pushed well off its path. For this step, score it by Manhattan
distance to the goal, and mark it as needing a new path
(`require_guide_path[i] = true` next step) from where it is now. A new path
costs one A* search. It also removes that agent's old path from `lns.flow`.

### 4. Every agent gets a path quickly

- **Rotate where stage 2 starts.** Today stage 2 always starts at agent 0,
  so when the deadline hits, the same high-numbered agents go without a
  path. Start each step where the previous step stopped.
- **Pass solver 6/7 paths to the planner whenever they exist,** without the
  `use_traffic && curr_timestep >= 100` condition (`scheduler.cpp:1079`,
  `scheduler.cpp:1195`). Solver 6 already computes them every step for
  `GuidePathLengthSum`, so this is free and cuts planner A* searches. See
  the caveats: only flow-matched agents get one, and only for the leg to
  the pickup.
- An agent with no path yet falls back to Manhattan distance to its goal
  for that step.

### 5. Stuck-agent counter (for checking the fix)

Count agents that have a goal and haven't moved for N consecutive steps
(say N = 20), and log the count per step (log line or a `TimeStepMetric`
field). This is how to show the freeze is gone. Today's frozen runs would
show this number climbing to thousands.

### 6. Let `frank_wolfe` work again (optional, second pass)

`frank_wolfe` and `update_fw_metrics` check `traj_dists[a].empty()`
(`flow.cpp:86`, `flow.cpp:123`). Change the skip check to
`trajs[a].empty()`, and either set every agent's deviation to 0 or take it
from the agent's local BFS (distance to nearest path cell). This turns the
traffic re-planning back on. Keep it separate from steps 1-5 so its effect
can be measured on its own.

### Files likely touched

- `default_planner/pibt.cpp` / `pibt.h`: local BFS, ±1 scoring,
  `get_gp_h` change.
- `default_planner/TrajLNS.h`: per-agent `togo` map, shared stamp/scratch
  arrays, stage-2 rotation index, stuck counters.
- `default_planner/flow.cpp`: build the `togo` map in `update_traj`;
  optionally the `frank_wolfe` skip check.
- `default_planner/planner.cpp`: build `togo` for scheduler-provided paths,
  stage-2 rotation, far-off-path replan marking, stuck counter.
- `default_planner/scheduler.cpp`: drop the planner-handoff condition for
  solvers 6/7.
- `default_planner/const.h`: `R`, stuck threshold N, and whether the new
  scoring is on (so the old Manhattan behaviour stays reproducible).

## Caveats

**Guarantees and their limits**

- **No traps only within radius R.** An agent with no path cell within `R`
  falls back to Manhattan distance for a step and can sit in a pocket until
  its new path arrives. That should be one or two steps, but if stage 2 is
  starved it could be longer.
- **Agents without a path at all use Manhattan distance.** This matters
  most at step 0, when all 10k agents need a path at once and stage 2
  spreads them over the first few steps, and at every goal change until
  that agent's A* runs. Stage-2 rotation limits how long any one agent
  waits.
- **Agents can still block each other.** This fixes traps caused by the
  score, not congestion or deadlock between agents. PIBT has the same
  limits it has upstream. Corridors and dead ends can still jam at high
  agent counts. The stuck-agent counter will show both kinds, so a count
  that stays above zero isn't automatically a bug in this fix.
- **Shortcuts across the path.** Taking the minimum over all nearby path
  cells means that where a path winds back near itself, the agent may cut
  across to a later part of it. That's shorter, but it can go through
  cells the congestion-aware A* chose to avoid. `R` limits how big a
  shortcut can be.

**Speed**

- **PIBT's time reserve is fixed at 1 ms per 100 agents** (100 ms at 10k,
  `PIBT_RUNTIME_PER_100_AGENTS` in `const.h`). The BFS runs inside PIBT, so
  if 10k BFSes plus the scoring don't fit in that, decisions start running
  over and throughput drops the way it did with the flag off. Estimate: 25-60
  cells per BFS, about 0.5M cell visits per step, a few ms. That is
  unmeasured. Measure it before anything else; if it's tight, raise the
  reserve or shrink `R`.
- **Far-off-path re-plans can bunch up.** In a jammed area many agents may
  be pushed beyond `R` in the same step, each triggering an A* search.
  They go through stage 2, so they're bounded by the deadline, but they
  compete with new-goal paths. A per-step cap on these re-plans may be
  needed.
- **Hash-map churn.** One `togo` map per path, rebuilt on every goal
  change, means frequent allocations. Probably fine at about 10 goal
  changes per step; if it shows up in profiles, use a pooled flat
  structure.

**Behaviour changes that affect comparisons**

- **Results are not comparable with earlier runs.** Both the old
  flag-on (freezes) and flag-off (starved) results come from a different
  planner. All sweep runs must use the new planner, and the thesis should
  describe it as a change from upstream.
- **Passing scheduler paths to the planner changes solver comparisons.**
  Solver 6/7 paths ignore congestion (every arc costs 1.0, see
  `ai/guide_path_metric.md`). Solver 1's paths account for traffic only
  with `--useTraffic`. Solver 5 produces none. Also, only flow-matched
  agents get a solver 6 path (locally matched agents don't,
  `MapCoarsenV1.cpp:1964-2000` and the cascade), and scheduler paths only
  cover the leg to the pickup. The planner still runs A* for the rest. Say
  which agents got scheduler paths when comparing solvers, or keep the
  handoff off for a clean like-for-like comparison.
- **Turning `frank_wolfe` back on (step 6) changes paths again.** Measure
  it as its own step.

**Existing issues this doesn't fix**

- **Paths are never trimmed,** so `lns.flow` keeps counting cells an agent
  has already passed until its path is replaced. Later A* searches avoid
  congestion that isn't really there. Step 3 (re-plan when far off path)
  and step 6 (`frank_wolfe`) both reduce this but don't remove it.
- **The 2026-09-18 note is wrong about `frank_wolfe`.**
  `ai/manhattan_heuristic_lever.md` says the Manhattan run had "the
  traffic-flow planner functioning again". Stage 2 did run, but
  `frank_wolfe` skipped every agent in that mode. Its +6.4% came from
  greedy Manhattan PIBT running more often, over a 500-step window that
  ended before the freeze.

## Checking it works

1. **Unit check:** `instances/custom/tiny/tiny.json`, both flag states and
   the new scoring, runs to completion with 0 errors.
2. **Trap check:** a tiny hand-built map with a U-shaped pocket between an
   agent and its goal. Manhattan mode should wait forever; the new scoring
   should walk around.
3. **Timing:** add per-stage timing (setup, stage 2 with number of agents
   given paths, `frank_wolfe`, PIBT) and run orz900d 10k for about 300
   steps, current flag-on build vs. new build. Check that PIBT stays within
   its reserve and that decisions ≈ steps.
4. **Freeze check:** orz900d 10k, solver 6 level 4, at least 1,500 steps
   (the freeze showed up around step 1,000). Compare pickups/deliveries per
   step with the frozen run, and check the stuck-agent count stays flat.
   Probes shorter than about 1,000 steps can't show this fix working.
5. Then remove `outputs/thesis_sweep/STOP` and restart the sweep. (The 4
   stale runs were moved to
   `outputs/thesis_sweep_junk/2026-09-29_manhattan_freeze/` on 2026-09-29.)

## Alternatives considered

- **Upstream `Dist2Path` stored sparsely.** Same scoring as the flag-off
  planner, with a hash map instead of a whole-map array per agent. It works,
  but its BFS starts from the whole path, so the first query one cell off
  the path expands about 4 × path length cells, mostly far from the agent.
  The local BFS does less work and has the stronger no-trap guarantee.
- **Waypoint on the path** (Manhattan distance to the cell k steps ahead).
  Cheapest, but can still trap where the path doubles back around a thin
  wall, or when the agent is pushed off its path. A cheaper hybrid (`togo`
  on the path, a fixed penalty off it, no BFS) is the fallback if the BFS
  turns out too slow.
- **Better estimates than Manhattan** (landmark distances, or distances on
  the coarsening hierarchy). Useful for speeding up A*, but they are still
  lower bounds, so they don't make PIBT trap-free.
- **Stuck detection with random walks.** Treats the symptom; kept only as
  the stuck-agent counter for checking.
