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

## Known gaps / not yet done

- The per-agent panel grid has no automatic "pick the most interesting
  agents" mode (largest length delta, task mismatches, etc.) — currently
  manual via `--agents`.
- The whole-map overview at 20000 agents is close to visually saturated;
  legible enough to distinguish walls from coverage, but individual routes
  are no longer distinguishable at that density. A density/heatmap mode
  (discussed early on, not built) would read better at very high agent
  counts than discrete dots+lines.
- `dump_guide_paths` always forces `curr_timestep=100`/`use_traffic=true`
  internally (the only state guide paths exist in) — there's no flag to
  inspect guide-path behavior at other timesteps short of the real
  `lifelong` binary (discussed and explicitly declined for this pass, see
  "30-timestep guide-path evolution" discussion — self-contained-stepper vs.
  instrumenting-the-real-simulator tradeoff, not pursued).
