# Guide-path visualisation tooling + solver 6 fine-lift fallback bug

Read `ai/project_context.md` first, specifically "Guide paths: which solvers
provide them, and are they doing anything" and "Solver 6 —
`schedule_plan_flow_reduced`" — this doc builds tooling on top of that
mechanism and, along the way, found and fixed a real bug in it. Triggered by
a request to visualise the map and solver 1 vs. solver 6 guide paths
directly (spatially, not just as summary metrics), starting from "what are
my options" through to a working per-agent and whole-map renderer, then
scaling that up until it broke, which surfaced the bug.

## TL;DR

- Two new standalone tools, neither touching core simulation code:
  - `./build/dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all]`
    (`map_reduction_test/dump_guide_paths.cpp`) — runs solver 1
    (`schedule_plan_flow`) and solver 6 (`schedule_plan_flow_reduced`) once
    each on the same starting instance state, with the guide-path gate
    forced open (`use_traffic=true`, `curr_timestep=100` — the only state in
    which either solver's `agent_guide_path` is ever non-empty), and dumps
    both solvers' guide paths + assigned task ids to CSV.
  - `map_reduction_test/visualisation/plot_guide_paths.py` — renders that CSV
    over the actual `.map` file: a grid of per-agent cropped/zoomed panels
    (solver 1 = blue, solver 6 = orange, identical path = purple, start =
    green dot, each panel titled with both solvers' task id + path length),
    and optionally a separate whole-map overview with thicker lines/markers
    covering every agent in the CSV.
- **Found and fixed a real bug**: at moderate-to-large scale on
  `IH_mp_2p_01` (~3.44M cells) with a sparse task pool, solver 6 silently
  returned guide paths for far fewer agents than it had actually assigned
  tasks to (e.g. 15/50, or 15/46 of the agents that even needed it) — solver
  1 returned all of them. Root cause: the coarse-to-fine lift's direct-search
  fallback (`shortest_path_in_graph_local`, `map_reduction_test/MapCoarsenV1.cpp`)
  is plain Dijkstra with no heuristic, hard-capped at 20000 node expansions —
  fine for its other use (tiny intra-component bridge hops) but wrong for
  this one (a potentially long search across the *entire* fine map), where
  undirected search has to expand roughly a full disc of nodes
  (~O(distance²)) before reaching a goal a few hundred cells away. **Fixed**
  by giving that function an optional A* mode (Manhattan-distance heuristic,
  admissible and exact here since every fine-graph arc costs exactly 1.0),
  applied only to that specific fallback call. See "Bug found and fixed"
  below for the full investigation.
- Verified fix: `guide_path_validator` still passes 128,062/128,062 checks
  on `orz900d`; the specific failing case (`IH_mp_2p_01`, 50 agents/50 tasks)
  went from 15/50 to 50/50 guide paths returned; re-verified at 500/500 and
  at the full instance sizes (5000/10000/20000 agents, 30000-task pool) with
  zero missing guide paths at every scale.
- Output for every run so far lives under `outputs/orz900d_guide_paths/` and
  `outputs/IH_mp_2p_01_guide_paths/` — see "Runs done and their results"
  below for the full table and what each file is.
- **Follow-up work, a separate mechanism from the two tools above**: the
  tools above are a frozen-snapshot, offline harness (load one instance,
  force the guide-path gate open, run each solver once). A later request
  asked for the opposite — guide paths and agent positions **as a real
  `lifelong` run actually progresses, timestep by timestep** — which needed
  new instrumentation inside `BaseSystem::simulate()` itself (still no
  behavior change when off, see "Live per-timestep tooling" below) plus a
  new renderer, `map_reduction_test/visualisation/plot_timestep_frames.py`,
  that turns the resulting CSVs into one PNG per timestep.

## Tools built

### `./build/dump_guide_paths` (`map_reduction_test/dump_guide_paths.cpp`)

```
./build/dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all]
```

Loads the instance via the shared `populate_env_from_instance` helper (same
as `run.cpp`/`validate_guide_paths.cpp`), then:

- If `num_tasks` is given, truncates `env.task_pool`/`env.new_tasks` down to
  task ids `< num_tasks`.
- If `num_agents` is less than the instance's full team size, **also**
  shrinks `env.num_of_agents`/`curr_states`/`curr_task_schedule`/
  `new_freeagents` down to the first `num_agents` agents. This isn't just
  about limiting CSV output — it's required for correctness: solver 1's flow
  formulation sets `supply[sink] = -num_workers`, i.e. it requires **at
  least as many flexible tasks as flexible agents**, so capping tasks below
  the *instance's full agent count* (e.g. 50 tasks against a 5000-agent
  instance) makes the assignment infeasible for both solvers (confirmed:
  `schedule_plan_flow` printed "No optimal solution found." and returned 0
  guide paths before this was added). Capping agents to match keeps the
  sub-problem both small and feasible.
- Runs `schedule_plan_flow` on one copy of the env and
  `schedule_plan_flow_reduced` on a second, independent copy (so neither
  solver's decisions leak into the other's run), both with
  `use_traffic=true`, `curr_timestep=100`, `new_only=true`.
- Writes `solver,agent_id,task_id,step,loc,row,col` rows to the output CSV,
  filtered to `agent_id < num_agents`.

Solver 1 and solver 6 decide task assignments **independently** — the same
agent can end up assigned to a different task under each solver, not just
routed differently to the same one. Both are written out as-is (destination
task id included per row) rather than only comparing agents that happen to
match, since the mismatch itself is part of what's being visualised.

### `map_reduction_test/visualisation/plot_guide_paths.py`

```
python3 plot_guide_paths.py <map_file> <paths.csv> -o panels.png \
    [--full-map-output fullmap.png] [--full-map-scale N] \
    [--agents ids] [--full-map-agents ids] [--cols N]
```

Pure `numpy`+`PIL`, no `matplotlib` (not available in this environment).
Parses the `.map` file directly (same `type`/`height`/`width`/`map` format
`Grid.cpp` reads) rather than going through C++.

- **Per-agent panel grid** (`-o`): for each agent, crops the map tightly
  around that agent's own path(s) + a few cells of padding, scales up to a
  target panel size, and draws both solvers' paths. Panel scale is a
  **float**, not an integer pixel-per-cell floor — guide paths here range
  from a couple of cells (dense task pools) to spanning most of the map
  (sparse ones, see "task density controls path length" below), and an
  earlier integer-floor version blew up the whole grid's cell size for one
  oversized panel. Each panel's title includes both solvers' assigned task
  id and path length (edges), and `[same task]`/`[same path]` flags.
- **Whole-map overview** (`--full-map-output`, optional): every agent's
  paths drawn directly on the full, uncropped map, thicker lines/markers
  than the per-agent panels (needed for visibility at 1-2px/cell). Scales
  fine regardless of agent count — the canvas is a fixed size — but at high
  agent density the picture becomes mostly-saturated colored dots (see
  results below), which is an honest reflection of the data (paths are
  short) rather than a rendering problem.
- `--agents` and `--full-map-agents` are **independent** — this was a bug
  the first time: `--agents` accidentally filtered the whole-map render too
  (rendered 20 agents instead of the intended 5000). Fixed by giving the
  whole-map output its own agent-list flag, defaulting to every agent
  present in the CSV.

**Per-agent panel grid does not scale past a few dozen agents.** At 500
agents it produced a 2465×36826px image (100 rows) — technically written,
not practically viewable. At that scale, use `--agents` to render a
representative subset (e.g. first 20, or the ones with the largest task/path
mismatches) alongside the whole-map overview for the full picture.

## Bug found and fixed

### Symptom

Running `dump_guide_paths` on `IH_mp_2p_01` (5000-agent instance file,
capped to 50 agents / 50 tasks): solver 1 returned 50/50 guide paths, solver
6 returned only **15/50**. Confirmed not a CSV/plotting artifact — the
missing 35 agents simply weren't in solver 6's output at all.

### Investigation

Added temporary debug counters through `compute_reduced_assignment`
(`map_reduction_test/MapCoarsenV1.cpp`) at every point that could drop an
agent:

```
[DEBUG compute_reduced_assignment] agents=50 tasks=50 ns_status=1
    no_top_node=0 no_src_remaining=0 bfs_failed=0 no_task_at_sink=0 success=50
[DEBUG lift] needed_fallback=46 fallback_failed=35
```

The top-level flow solve + BFS flow decomposition (Steps 1-2, recovering
each agent's coarse path from the min-cost flow) was **not** the problem —
all 50 agents got a valid coarse-level assignment. The failure was entirely
in the coarse-to-fine lift (Step 3): 46/50 agents needed the direct-search
fallback (because the level-by-level lift produced an empty or
wrong-endpoint path — expected when a coarse parent's chosen representative
doesn't line up with the agent's real location), and **35 of those 46
fallback searches failed outright**, leaving those agents with an empty
`current_paths[i]`, which Step 4 then silently skips (`agent_guide_path`
entry never written — deliberately, per the existing comment there, since a
truly-empty guide path would underflow unsigned arithmetic in
`flow.cpp`'s `add_traj`/`remove_traj`).

### Root cause

The fallback is `shortest_path_in_graph_local` — plain Dijkstra (no
heuristic), hard-capped at `MAX_EXPANSIONS = 20000` node expansions "to
catch disconnected networks safely". That cap is fine for the function's
other call sites (tiny intra-component bridge lead-in/lead-out hops, a
handful of nodes). But the fallback call at the end of Step 3 runs it
**unconstrained over the entire fine graph** (`*fine`, ~3.44M nodes on this
map) with no heuristic guidance. Undirected Dijkstra explores in all
directions equally, so reaching a goal `d` cells away in a largely open
region requires expanding roughly a full disc of nodes — O(d²) — before the
goal is even reached. With only 50 tasks scattered across a huge map,
several agents' nearest available task was hundreds of cells away (solver
1's own path lengths for this instance ranged up to 881 — see the earlier
50/50 comparison panels), and expanding a disc of radius ~300-800 cells
blows past 20000 expansions long before the search frontier reaches the
goal, so the function returns `{}`.

### Fix

`shortest_path_in_graph_local` gained an optional `use_heuristic` parameter
(default `false`, so every existing call site is unaffected). When true, it
becomes A* with a Manhattan-distance heuristic computed from
`graph.fine_location` — admissible *and* exact here, since every fine-graph
arc costs exactly 1.0 on a 4-connected grid, so Manhattan distance never
overestimates true cost-to-go. A* expands roughly along the path itself
instead of a whole disc, so the same (or a larger) expansion budget goes
much further. Only the specific unconstrained whole-fine-map fallback call
opts in:

```cpp
current_paths[i] = shortest_path_in_graph_local(*fine, start_loc, task_loc,
                                                 -1, false, /*use_heuristic=*/true);
```

The tiny constrained bridge-hop calls (`constrain_parent=true`) are
untouched — they were never the problem, and their `graph` argument isn't
always the fine level, so the Manhattan-distance heuristic's assumption
about `fine_location` carrying real map geometry doesn't universally apply
there. As a secondary safety margin, `MAX_EXPANSIONS` is now `200000` (up
from `20000`) specifically when `use_heuristic` is true, and the path-length
"cycle emergency brake" during reconstruction was changed from a hardcoded
`5000` to match `MAX_EXPANSIONS`, so a legitimately long path found after
all that search isn't discarded by an unrelated cap.

### Verification

- Debug counters on the original failing case: `fallback_failed` 35 → 0,
  solver 6 now returns 50/50 guide paths.
- `./build/guide_path_validator instances/custom/orz900d/orz900d_5000.json --useTraffic`:
  128,062 checks run, 0 failed (structural guarantees — correct start/end,
  valid 4-connected adjacency, etc. — still hold everywhere).
- All four build targets (`lifelong`, `map_reduction_test`,
  `guide_path_validator`, `dump_guide_paths`) rebuild clean.
- Re-verified at 500/500, then at the full instance sizes (5000, 10000,
  20000 agents against the full 30000-task pool) — zero agents ever missing
  a guide path at any scale tested. See table below.
- All debug instrumentation was removed after diagnosis; the final diff to
  `MapCoarsenV1.cpp` is +69/-25 lines, comment-heavy, no other behavior
  change.

## Task density controls path length (why some runs look like dots, others like routes)

Recurring pattern across every run: with a **dense** task pool (thousands of
tasks), an agent's nearest available task is almost always a handful of
cells away, so guide paths are short (single digits to low tens of cells) —
the whole-map overview for these runs looks like clustered colored dots, not
visible lines. With a **sparse** task pool (capped to e.g. 50-500 tasks
against thousands of agents), paths get dramatically longer (up to ~880
cells seen on `IH_mp_2p_01`) and the whole-map overview shows real
cross-map routes. Neither is a bug — both are the expected effect of task
density on assignment distance, and it's a genuinely useful lever: capped
runs are what actually exercises interesting solver 1 vs. solver 6 routing
differences (and is what surfaced the fine-lift bug above); full-density
runs are more representative of realistic benchmark conditions
(`ai/auto_benchmarking*.md`).

## Runs done and their results

All runs use `curr_timestep=100`, `use_traffic=true` (the only state in
which either solver's guide paths are non-empty — see
`ai/project_context.md`). "Same task" = both solvers assigned the agent to
the same task id. "Same path" = both solvers additionally produced the
exact same fine-node route (implies same task).

| Map | Agents | Tasks | Same task | Same path | Notes |
|---|---|---|---|---|---|
| `orz900d_5000` | 20 | 10000 (full) | 9/20 | 8/20 | First run, dense pool, short paths |
| `orz900d_5000` | 20 | 50 (capped) | — | — | Sparse pool, long/varied routes, no aggregate stats computed |
| `IH_mp_2p_01_5000` | 20 | 30000 (full) | 13/20 | 6/20 | Before the bug fix |
| `IH_mp_2p_01_5000` | 50 | 30000 (full) | 30/50 | 12/50 | Before the bug fix |
| `IH_mp_2p_01_5000` | 50 | 50 (capped) | — | — | **Bug found here**: solver 6 returned 15/50 guide paths pre-fix, 50/50 post-fix |
| `IH_mp_2p_01_5000` | 500 | 500 (capped) | 372/500 (74%) | 74/500 (15%) | Post-fix, all 500/500 guide paths returned |
| `IH_mp_2p_01_5000` | 5000 (full) | 30000 (full) | 3130/5000 (62.6%) | 1397/5000 (27.9%) | avg len solver1=5.54, solver6=6.35 |
| `IH_mp_2p_01_10000` | 10000 (full) | 30000 (full) | 6111/10000 (61.1%) | 2643/10000 (26.4%) | avg len solver1=5.86, solver6=6.68 |
| `IH_mp_2p_01_20000` | 20000 (full) | 30000 (full) | 10995/20000 (55.0%) | 4595/20000 (23.0%) | avg len solver1=6.90, solver6=7.69 |

Clean trend across the three full-instance runs: as agent count grows
toward the fixed 30000-task pool (1:6 → 2:3 agents:tasks ratio), agreement
between the two solvers drops and average path length rises — more agents
chasing the same task pool means each one's nearest available task is
farther away on average, and the two solvers' independent assignment
choices have more room to diverge.

| `warehouseXL_5000` | 50 | 50 (capped) | 46/50 (92.0%) | 5/50 (10.0%) | avg len solver1=273.56, solver6=277.84 |
| `warehouseXL_5000` | 150 | 150 (capped) | 115/150 (76.7%) | 19/150 (12.7%) | avg len solver1=191.77, solver6=210.80 |
| `warehouseXL_5000` | 500 | 500 (capped) | 413/500 (82.6%) | 138/500 (27.6%) | avg len solver1=104.88, solver6=111.63 |
| `warehouseXL_5000` | 5000 (full) | 30000 (full) | 3401/5000 (68.0%) | 2861/5000 (57.2%) | avg len solver1=8.06, solver6=9.16 |
| `warehouseXL_10000` | 10000 (full) | 30000 (full) | 6565/10000 (65.7%) | 5534/10000 (55.3%) | avg len solver1=8.50, solver6=9.59 |
| `warehouseXL_20000` | 20000 (full) | 30000 (full) | 11913/20000 (59.6%) | 9891/20000 (49.5%) | avg len solver1=9.49, solver6=10.63 |

Zero missing guide paths at every scale on `warehouseXL` too (100% return
rate from both solvers) — the A* fallback fix holds up on a second huge map,
including one with a structured (non-maze) layout. Full sweep context
(instance generation, solver comparison) in
`ai/auto_benchmarking_warehouseXL.md`.

### Output files

- `outputs/orz900d_guide_paths/`: `guide_paths.csv`/`.png`/`_fullmap.png`
  (full 10000-task pool), `guide_paths_50tasks.csv`/`.png`/`_fullmap.png`
  (capped to 50 tasks/20 agents).
- `outputs/IH_mp_2p_01_guide_paths/`: `guide_paths_50agents.*` (30000-task
  pool), `guide_paths_50agents_50tasks.*` (the bug-finding run),
  `guide_paths_500agents_500tasks.*`, `guide_paths_{5000,10000,20000}agents_full.csv`
  + `_fullmap.png` + `_sample20.png` (20-agent representative panel subset,
  since the full panel grid isn't viewable at these agent counts).
- `outputs/warehouseXL_guide_paths/`: `guide_paths_{50,150,500}agents_{50,150,500}tasks.*`
  (full per-agent panel grid + `_fullmap.png` for 50/150, `_sample20.png` +
  `_fullmap.png` for 500), `guide_paths_{5000,10000,20000}agents_full.csv`
  + `_fullmap.png` + `_sample20.png`.

## Live per-timestep tooling: `CompetitionSystem.h` dump switch + `plot_timestep_frames.py`

Distinct mechanism from everything above — no frozen snapshot, no forcing
`curr_timestep=100`/`use_traffic=true`, no running solvers standalone. This
instruments the real `BaseSystem::simulate()` loop (`src/CompetitionSystem.cpp`)
that a normal `./build/lifelong` run already executes, so it captures
whatever a real run's scheduler/planner actually do, timestep by timestep,
under whichever `--scheduleModel`/`--useTraffic`/etc. flags that run used.

### The switch

`BaseSystem::kDumpPerTimestepPaths` (`inc/CompetitionSystem.h`, next to the
pre-existing `kUseTimeStepMetricsOutput` switch) — a `constexpr bool`,
**default `false`**, flipped in code (not a CLI flag) and requires a
rebuild. When `true`, `simulate()` streams two CSVs directly to disk (no
in-memory buffering, so file size is the only cost, not RAM) to the paths in
the adjacent `kGuidePathsCsvPath`/`kAgentPositionsCsvPath` constants
(default `outputs/guide_paths_per_timestep.csv` /
`outputs/agent_positions_per_timestep.csv`):

- `guide_paths_per_timestep.csv` — `timestep,agent_id,task_id,step,loc,row,col`,
  one row per fine-map node in each agent's guide path for that call. Reads
  `DefaultPlanner::get_all_guide_paths()` (`agent_guide_path_all`), **not**
  `get_guide_path()`/`agent_guide_path` — see "Unconditional capture" just
  below for why these are two different maps and why that separation matters.
  Still only non-empty for agents the scheduler actually reassigned that
  call (unassigned/already-pinned agents never get a row) — an empty or
  near-empty file for a given timestep is expected, not a bug.

#### Unconditional capture: a dump-only map, not the planner's seed map

Originally this dump read `agent_guide_path` directly (via the pre-existing
`get_guide_path()`), which meant it inherited that map's gate:
`use_traffic && curr_timestep >= 100`, same as the planner-seed mechanism
described in `ai/project_context.md`. In practice this made the dump far
sparser than the underlying computation actually is — both solvers compute
full guide-path data on **every** call regardless of that gate (it's what
feeds `GuidePathLengthSum`/`GuidePathCostSum`, unconditional since
`ai/guide_path_metric.md`); the gate only decided whether the result got
copied into `agent_guide_path` afterward. First noticed as "the visualization
doesn't seem to be showing any guide paths" on a real run where only 2 of 6
(solver 1) / 3 of 54 (solver 6) real decisions had any dumped data at all.

**Fix, not a change to `agent_guide_path` itself** (deliberately — that map
feeds `DefaultPlanner::plan()`'s per-agent trajectory seed via
`MAPFPlanner.cpp:37`; loosening its gate would change what path every agent's
low-level trajectory is seeded from, on every solver-1/6 run in the repo, not
just ones doing this dump — a real behavior change needing the kind of
re-validation `ai/project_context.md`'s "Known remaining gaps" already flags
as not yet done for that exact mechanism, not something to take on as a side
effect of a visualization ask). Instead, `scheduler.h`/`scheduler.cpp` gained
a **second, parallel global**, `agent_guide_path_all`, populated at the exact
same two call sites as `agent_guide_path` (`schedule_plan_flow` line ~911-914,
`schedule_plan_flow_reduced` line ~1056-1064) but gated only by a new
process-wide bool (`dump_all_guide_paths`, default `false`, set once via
`DefaultPlanner::set_dump_all_guide_paths(kDumpPerTimestepPaths)` at the top
of `simulate()` — so the one existing switch controls this too, no new knob
to remember) instead of `use_traffic`/`curr_timestep`. `get_all_guide_paths()`
returns this map; the dump reads that instead of `get_guide_path()`.
Marginal cost when enabled is one extra `unordered_map` write per already-
computed path (the computation itself was already unconditional) — the
`if (dump_all_guide_paths)` check costs nothing when the feature is off.

**Verified zero behavior change**: ran the same instance/solver/seed with
`kDumpPerTimestepPaths` `true` vs `false` and diffed the output JSON —
`makespan`, `numTaskFinished`, `timeStepMetrics`, and every agent's
`actualPaths` entry were byte-identical. `agent_guide_path` (and therefore
the real planner's seed choice, and therefore the simulated outcome) is
untouched by this feature regardless of whether it's on.

**Result**: guide-path data is now dumped for every real decision the
scheduler makes, not just the ones past the old gate — confirmed on
`warehouseSmall_100`, solver 1, **without** `--useTraffic` at all: previously
this produced an empty guide-path CSV for the entire run (the old gate
requires `--useTraffic`); now it produces a full row for every one of the
first 30 timesteps, since the underlying computation never actually depended
on that flag, only the old storage gate did.
- `agent_positions_per_timestep.csv` — `timestep,agent_id,loc,row,col`, every
  agent's current location, every timestep, regardless of solver or flags
  (`env->curr_states[i].location`, synced every timestep unconditionally).
  Dense: exactly `num_agents` rows per timestep captured.

Both are captured at the same point in the loop, right after
`sync_shared_env()` / right after that timestep's `plan()` call returns — the
state the scheduler and planner actually decided against, not the state
after that timestep's move is applied.

**Known granularity caveat, inherited from the existing timing-metrics code
this piggybacks on**: `simulate()`'s outer loop calls `plan()` once and
captures one dump per *outer-loop iteration*, not one per elapsed simulated
timestep. When the planner/scheduler solve for a timestep is slow enough to
time out (as solver 1 routinely does on huge maps like `IH_mp_2p_01` — see
`ai/auto_benchmarking_IH_mp_2p_01.md`), one outer-loop iteration silently
burns several real simulated timesteps in a catch-up burst
(`ai/project_context.md`, "Makespan vs. 'timesteps solved'") before the dump
for that iteration is written. The dumped `timestep` value is the tick
*before* that burst, not every real tick inside it. Content-wise this is
mostly harmless for positions (agents are executing all-wait actions during
the burst, so they're genuinely stationary the whole time — the recorded
position is accurate for that whole span, just under one timestamp instead
of several), but it means solver 1 runs on large maps produce far fewer
distinct `timestep` values in these CSVs than the `-s` value would suggest
(see the run below: 200 requested timesteps, a low double-digit number of
distinct dumped timesteps for solver 1 vs. most/all of them for solver 6).

### `map_reduction_test/visualisation/plot_timestep_frames.py`

```
python3 plot_timestep_frames.py <map_file> <positions.csv> \
    [--guide-paths <guide_paths.csv>] -o <frames_dir> \
    [--start T0] [--end T1] [--every N] [--crop r0,c0,r1,c1] \
    [--scale N] [--agents ids] [--dot-radius N]
```

Streams both CSVs in one pass, grouped by timestep via `itertools.groupby`
(both are written in increasing-timestep order, so this stays O(rows) total
without ever loading a whole run's worth of rows into memory). Writes one
`frame_<timestep>.png` per (sampled) timestep: every agent as a blue dot,
agents with an active guide path that timestep as a red dot, path drawn as
an orange line, walls/free cells styled the same as the other tools in this
doc. `--crop`/`--every`/`--agents` exist for the same reason they do on
`plot_coarsening.py`/`plot_guide_paths.py` — huge maps and/or long runs
produce more pixels/frames than are practical to render or view at once.

**A marker-visibility side-quest that turned out to be the wrong problem**:
on first look at the `IH_mp_2p_01` full-map frames, red/orange markers were
reported as not showing at all. They technically were (confirmed by counting
exact-color pixels), just at a size low enough on a 3600x3850px canvas to be
practically invisible — two rounds of enlarging the markers were tried (a
size floor, then a halo ring), each verified only by pixel-count diffs, and
each still failed a real look-at-the-image check, the second one caught by
direct user pushback (correctly — "that's shocking. remove the red circle").
**Reverted**: the marker styling is back to a plain same-size colored dot,
no special-casing. The mistake here, worth remembering: verifying a
legibility fix by counting pixels instead of looking at the image at
realistic viewing size proves the marker is *present*, not that it's
*findable* — those are different claims, and only the second one was ever
actually in question.

### The real issue: most timesteps genuinely have no guide-path data

**Update: cause 1 below (the gate) has since been addressed** — see "The
switch" → "Unconditional capture" above, and the re-run numbers in "Run
done" below (2/6 → 5/6 for solver 1, 3/54 → 55/56 for solver 6). Cause 2
(the last-call race condition) has not been fixed and still applies — it's
independent of the gate. Left as-written below since it's the actual
investigation record and cause 2 is still fully accurate.

The marker size was never the problem. Once actually asked directly — "why
do only a couple of frames out of the whole run show any guide path at all"
— direct inspection of the CSVs (not the rendering) gave a precise, evidenced
answer with **two distinct causes**, not one:

**1. The known gate + sparsity (see "The switch" above), which explains most
of it.** `agent_guide_path` only gets populated when `--useTraffic` is on and
`curr_timestep >= 100`, and even then only for agents the scheduler actually
reassigns that call. Direct count from the raw CSVs for the run in
`outputs/IH_mp_2p_01_per_timestep/`: solver 1 dumped guide-path data at only
2 of its 6 real decision timesteps (`t=112`, `t=149` — the only two `>= 100`);
solver 6 at only 3 of its 54 (`t=100`, `t=102`, `t=151`). Every other
timestep's position dump is correct and complete (every agent's location is
unconditional) — it's specifically the guide-path side that's this sparse,
by design.

**2. A second, previously undocumented cause found while checking the first
one: the very last scheduling call in each run never finishes, and that
timestep looks different from "gated off" — it looks like it *should* have
data but doesn't.** First noticed in `result.json`: each run's last
`TimeStepMetric` pair reports a `GuidePathLengthSum` numerically identical to
the *previous* pair (solver 1: last two pairs both `53481.0`; solver 6: last
two pairs both `17.0`) — suspicious on its own, since two different
scheduling calls landing on the exact same sum is unlikely. `PlannerTime`
tells the real story: it's *not* frozen the same way (solver 1's last pair:
`13.0s`, vs. the previous pair's `37.7s`) because it's measured fresh every
iteration straight off the wall clock in `simulate()`, independent of
scheduler state — 13s is almost exactly `simulation_time(200) - 187`, i.e.
the main thread waited only as long as the remaining time budget allowed,
then gave up. `run.log` for both runs confirms this directly and
unambiguously: the scheduler call starting at solver 1's `t=187` (solver 6's
`t=154`) logs `planner timeout` on every single subsequent tick through
`t=199` and **never once logs `planner returns`** — the only such case in
either run. Root cause, in `BaseSystem::plan()` (`src/CompetitionSystem.cpp`,
pre-existing code, not part of this feature): its polling loop
(`while (timestep + timeout_timesteps < simulation_time)`) simply exits and
returns, **without joining the background scheduler/planner thread**, once
the simulation's total time budget (`-s 200` here) is exhausted mid-call —
there's no path in that function that waits for an in-flight call to finish
once the clock runs out. `schedule_plan_flow`/`schedule_plan_flow_reduced`
both call `agent_guide_path.clear()` as their first line; the background
thread had reached that line (or further, into the ~35-47s flow solve) but
never got to repopulate it before `simulate()` gave up and moved on. My dump
code, running on the main thread moments later, reads exactly that
cleared-but-not-yet-repopulated global — hence a real, empty result, not a
merge bug in `plot_timestep_frames.py` (verified separately: the script's
timestep-merge logic produces guide-path content for exactly the CSV rows
that exist, no more, no fewer — checked per-frame by re-deriving which
frames have non-background pixels and diffing against the CSV's own
distinct-timestep list). This is a real characteristic of the existing
`simulate()` loop, not something introduced by this feature, and not fixed
here — flagged as a known gap below instead, since fixing it means changing
core simulation thread-joining behavior, out of scope for a visualization
ask.

All frames in `outputs/IH_mp_2p_01_per_timestep/*/frames*/` were regenerated
with the reverted plain-dot styling.

### Known limitation vs. a true solver comparison

A single run only ever exercises one `--scheduleModel`. Comparing solver 1
against solver 6 with this tooling means two separate runs, and — unlike
`dump_guide_paths`'s frozen-snapshot comparison above, where both solvers see
literally the same starting state — two live runs diverge in simulated world
state from timestep 0 onward as soon as their assignments differ (different
task choices change agent positions, which change what's assignable next,
etc.). So this is for visualising one run's own behavior over time, not a
strict apples-to-apples per-timestep solver diff.

### Run done: `IH_mp_2p_01`, 5000 agents, 200 timesteps, solver 1 vs. solver 6 (depth 2)

Two live `./build/lifelong` runs (`--useTraffic 1 -s 200 --preprocessTimeLimit 600000`,
`kDefaultCoarsenLevels` left at the repo default of 2), each with
`kDumpPerTimestepPaths` on, one per solver — **not** a frozen-state
comparison (see "Known limitation" above), just exercising the new live
tooling at real scale on the map this repo's benchmarking already flagged as
the size where solver 6 starts winning (`ai/auto_benchmarking_IH_mp_2p_01.md`).
Output, fully self-contained: `outputs/IH_mp_2p_01_per_timestep/solver1/` and
`.../solver6_depth2/`, each with `result.json`, `run.log`, both CSVs, a
`frames/` dir (whole-map, `--scale 2`, one PNG per dumped timestep), and a
`frames_detail/` dir (one `--crop 200x200 --scale 6` close-up PNG,
demonstrating the crop option on a real dataset).

**Re-run once, after the "Unconditional capture" fix above landed** — the
numbers and files below are from the second (current) run. The first run
used the still-gated `get_guide_path()` and produced guide-path data for
only 2 of solver 1's 6 real decisions and 3 of solver 6's 54; investigating
why led to the unconditional-capture change documented above, and this
section was regenerated afterward rather than kept as two separate runs.

| | solver 1 | solver 6 (depth 2) |
|---|---|---|
| wall clock | ~8.6 min | ~7.9 min |
| `schedulerHierarchyBuildTime` | paid, unreported (see `ai/project_context.md`) | 302.16s |
| `makespan` | 201 | 201 |
| `numTaskFinished` | 195 | 1445 |
| `timeStepMetrics` entries ("steps recorded") | 12 | 112 |
| distinct dumped timesteps (position CSV) | 6 | 56 |
| distinct dumped timesteps (guide-path CSV) | **5** (was 2) | **55** (was 3) |
| guide-path rows written | 332,612 | 116,655 |

`numTaskFinished`/"steps recorded" land close to, but not exactly on, the
earlier no-traffic sweep's numbers for the same map/agent count/solver
(`ai/auto_benchmarking_IH_mp_2p_01.md`: solver1 195/12 — exact match; solver6
1436/108 there vs. 1445/112 here) — consistent with the run-to-run wall-clock
jitter already characterized in "Verified zero behavior change" above
(decision *content* is deterministic, decision *timing* against the 1000ms
poll isn't), not a regression.

**Now covers almost every real decision, not just the ones past the old
gate**: solver 1 has guide-path data at 5 of its 6 decisions (`t=0, 38, 73,
109, 145`); solver 6 at 55 of its 56 (`t=0` through `t=102` at 2-tick
spacing, plus `146, 149, 194`). In both cases **exactly one** timestep is
still missing — solver 1's `t=182`, solver 6's `t=196` — and it's the same
one, every time: the run's very last scheduling call, which never completes
before the simulation's time budget runs out (confirmed again via `run.log`:
both show `planner timeout` on every tick from that point through `t=199`,
never `planner returns`). This is the pre-existing race condition documented
in "Known gaps" below, not the old use_traffic/curr_timestep gate — there's
simply no data to have captured for that call, at any point in its
execution, so no amount of loosening a gate fixes it.

**A live, concrete confirmation of a cost this repo had previously only
measured in isolation**: solver 6's per-call agent count (see
`agents-with-guide-path` in the raw CSVs) starts at 5000 at `t=0` (every
agent is "flexible" at initialization), then collapses to double digits
within a few calls as most agents get pinned to already-open tasks — and
past `t=100`, once the `use_traffic && curr_timestep >= 100` gate opens and
Steps 3-4 (coarse-to-fine lifting) start actually mattering for real (not
just for the metric), decision spacing itself blows out from every 2 ticks
to gaps of 44+ ticks (`102 → 146 → 149 → 194`). This matches
`ai/auto_benchmarking_IH_mp_2p_01.md`'s isolated `compute_reduced_assignment`
measurement of **~24s/call with lifting included**, vs. effectively free
without it.

Guide paths confirm the "dense task pool → short paths" pattern from the
frozen-snapshot tooling above: solver 1 averages ~12 edges/agent, now visible
as **near-total coverage** of the full-map frames (see
`outputs/IH_mp_2p_01_per_timestep/solver1/frames/frame_000038.png` — almost
every agent dot is red with a short orange path, a handful of stragglers
still blue) rather than the sparse scatter the pre-fix run showed. Solver 6's
per-timestep coverage is still much sparser than solver 1's even with the
fix — by design, not a bug: its pinning logic only re-offers genuinely
flexible agents to the coarse solver each call (`ai/project_context.md`),
so a single timestep's `frames_detail/` crop can legitimately show only one
or two paths in a 200x200 region even though the full run now has guide-path
data for 55 of 56 timesteps instead of 3.

## Known gaps / not yet done

- The per-agent panel grid has no automatic "pick the most interesting
  agents" mode (largest length delta, task mismatches, etc.) — currently
  manual via `--agents`.
- The whole-map overview at 20000 agents is close to visually saturated;
  legible enough to distinguish walls from coverage, but individual routes
  are no longer distinguishable at that density. A density/heatmap mode
  (discussed early on, not built) would read better at very high agent
  counts than discrete dots+lines.
- ~~`dump_guide_paths` always forces `curr_timestep=100`/`use_traffic=true`
  internally... there's no flag to inspect guide-path behavior at other
  timesteps short of the real `lifelong` binary~~ — **done**, see "Live
  per-timestep tooling" above (`kDumpPerTimestepPaths` + `plot_timestep_frames.py`).
- **`BaseSystem::plan()` can abandon an in-flight scheduler/planner call
  without joining it** when the simulation's total time budget runs out
  mid-call (`src/CompetitionSystem.cpp`, the polling loop in `plan()` simply
  returns once `timestep + timeout_timesteps >= simulation_time`, with no
  path that waits for the background thread first). Pre-existing behavior,
  not introduced by this feature, but this feature's per-timestep dump is
  what surfaced a concrete, visible symptom of it: `agent_guide_path` can be
  read mid-clear (real bug-shaped behavior, not just theoretical) exactly
  when this happens, producing an empty guide-path dump for the run's last
  captured timestep even though the scheduler had legitimately started
  computing something for it. See "The real issue" above for the full
  evidence trail. Not fixed here — would mean changing core simulation
  thread-joining behavior near the end of a run, out of scope for a
  visualization tool.
- The live per-timestep dump's timestep granularity is bounded by how often
  the scheduler/planner actually complete a call, not real elapsed time
  (the caveat under "Live per-timestep tooling" above) — on a run dominated
  by planner timeouts (solver 1 on a huge map) this can mean single-digit
  distinct timesteps for the whole run's guide-path data. Capturing inside
  the timeout catch-up loop too, instead of only once per outer-loop
  iteration, would fix this but wasn't needed for the runs done so far.
- `plot_timestep_frames.py` has no animation/GIF output, only individual
  PNGs (a deliberate choice for this pass, see conversation this doc
  originates from) — stitching frames into a GIF/video is a manual follow-up
  if wanted.
