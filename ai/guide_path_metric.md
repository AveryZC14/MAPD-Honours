# Guide-path reconstruction rigor pass + GuidePathLengthSum/GuidePathCostSum metric

Read `ai/project_context.md` first, specifically "Guide paths: which solvers
provide them, and are they doing anything" — this doc extends that
investigation. Triggered by a request to (1) rigorously verify solver 6's
guide-path reconstruction produces output solver 1-compatible with the
low-level planner, and (2) add a new per-timestep output metric summing the
"weight" of every guide path built that timestep, for both solvers, as a
proxy for assignment/path quality ("optimality of solutions").

## TL;DR

- Solver 6's guide-path reconstruction **had a real bug**: on a map where a
  single coarse component (parent node) at an intermediate hierarchy level
  actually spans multiple disconnected fine-map sub-components (very
  reachable on small/maze-like maps; plausible but rarer on huge ones), the
  level-by-level lift could silently return a guide path anchored on the
  wrong sub-component's representative node — i.e. a path that does not
  start at the agent's real location or does not end at the assigned task's
  location. **Fixed** (see "Bug found and fixed" below).
- After the fix, exhaustive structural validation across 6 instances (tiny
  hand-built mazes through `orz900d_5000`, ~978K cells) found **zero**
  further violations. Solver 1 and solver 6 write into the exact same global
  (`agent_guide_path`, a `boost::unordered_map<int,list<int>>` — see "the
  container type is boost, not std" below), so they are format-interchangeable
  by construction once the contents are verified well-formed.
- Added `GuidePathLengthSum`/`GuidePathCostSum` to `TimeStepMetric`, computed
  for **both** solvers on **every** timestep now (previously guide paths — and
  for solver 6, all of the fine-grained lifting that produces them — only
  existed at all when `--useTraffic` was on and past timestep 100).
- **Critical caveat before trusting a cross-solver total**: summed across a
  multi-timestep run, `GuidePathLengthSum` is only a fair solver 1 vs. solver
  6 comparison when both are run with `--assignNew 1` (`new_only=true`). See
  "Why `--assignNew` matters" below — without it, solver 1's total is
  inflated by a re-computation-frequency artifact that has nothing to do with
  path quality.
- Solver 6's `GuidePathCostSum` currently always equals its
  `GuidePathLengthSum` — the coarsened hierarchy's fine-graph arc costs are
  fixed at 1.0 at hierarchy-build time and never see `--useTraffic`/
  `background_flow`, unlike solver 1 which rebuilds congestion-weighted costs
  every timestep. See "Solver 6's costs are not traffic-aware" below.

## What changed (code)

- `inc/CompetitionSystem.h`: `TimeStepMetric` gained `GuidePathLengthSum`/
  `GuidePathCostSum`. Populated at both `time_step_metrics.push_back(metric)`
  call sites in `src/CompetitionSystem.cpp` (`BaseSystem::simulate()`, the
  catch-up-timestep branch and the normal branch) from
  `last_scheduler_timing`, and serialized into `timeStepMetrics[].GuidePathLengthSum`/
  `GuidePathCostSum` in `saveResults()`.
- `default_planner/scheduler.h`/`.cpp`: `ScheduleTiming` gained
  `guide_path_length_sum`/`guide_path_cost_sum`; `set_last_timing`/
  `set_last_reduced_timing` gained two new (defaulted) parameters to carry
  them.
- `schedule_plan_flow` (solver 1, `scheduler.cpp`): accumulates length (edges)
  and cost (sum of the `cost[]` ArcMap values of arcs actually walked) while
  reconstructing each agent's path from the flow solution — this walk already
  happened unconditionally before (needed to recover the assignment), so this
  is free and now always reported, not just when the `use_traffic &&
  curr_timestep >= 100` planner-seed gate is open.
- `schedule_plan_flow_reduced` (solver 6, `scheduler.cpp`) /
  `compute_reduced_assignment` (`map_reduction_test/MapCoarsenV1.{h,cpp}`):
  `compute_reduced_assignment` gained two new out-params
  (`guide_path_length_sum_out`/`guide_path_cost_sum_out`), computed in Step 4
  from the already-lifted `current_paths`. The caller now passes
  `need_guide_paths = true` **unconditionally** (previously
  `use_traffic && curr_timestep >= 100`) so the fine-grained lift (and thus
  the metric) always runs — but storage into the global `agent_guide_path`
  (what the low-level planner actually consumes as a seed trajectory) is
  still gated by the original condition, so **no existing run's planner-visible
  behavior changed**, only the metric's availability. New local helper
  `path_cost_on_fine_graph_local` looks up real per-arc costs rather than
  assuming 1.0, so the metric stays correct if the hierarchy's cost model
  ever becomes traffic-aware.
- This is a deliberate, explicit tradeoff: it reintroduces per-timestep
  fine-lift cost for solver 6 that was previously skipped entirely outside
  the traffic-seed window (the whole point of the original `need_guide_paths`
  early-return). See "Perf/regression check" below for the validation this
  didn't reintroduce the original OOM behavior.

## Bug found and fixed

**Symptom** (found by the new `guide_path_validator` tool on
`instances/custom/tiny/tinyComplex.json`, an 8x8 hand-built maze, 2 agents, 2
tasks): solver 6 returned a guide path for one agent whose *last* node wasn't
the assigned task's location, and for another agent whose *first* node wasn't
the agent's real current location.

**Root cause**: `compute_reduced_assignment`'s Step 3
(`expand_path_batch_one_level_local`, `MapCoarsenV1.cpp`) lifts a coarse path
level-by-level. At each intermediate level it walks
representative-node-to-representative-node (`chosen_finer_node_id`), with no
anchor back to the real agent/task location — that correction
(`preferred_starts`/`preferred_goals`, plus lead-in/lead-out bridge splicing)
is only applied at the **final** (level 1 → fine) expansion. This is fine as
long as a coarse parent's chosen representative and the real endpoint's true
sub-node always land in the same connected sub-component — but coarsening
groups nodes geometrically (2x2 blocks), not by connectivity, so a single
coarse parent can legitimately contain **multiple disconnected fine-map
sub-components** (two pockets separated by a wall that both happened to fall
in the same coarsened block). When that happens at an *intermediate* level,
the lift has no way to detect it's walked into the wrong sub-component, and
silently keeps going — producing a structurally valid-looking but
semantically wrong path (right length, wrong location). This is the same
underlying phenomenon as memleak-fix bug #1 (`ai/claude_memleak_fixes.md`,
the bridge-path exact-equality failures), one level further removed from
where that fix was applied.

**Fix** (`MapCoarsenV1.cpp`, end of Step 3): after the level-by-level lift
completes, every path's endpoints are now checked against the real agent
start location and real task location, not just checked for emptiness. Any
path that's empty *or* has a wrong endpoint is discarded and replaced with
the pre-existing direct-Dijkstra fallback
(`shortest_path_in_graph_local(*fine, start_loc, task_loc, -1, false)`),
which is unconstrained by the hierarchy and therefore always correct by
construction. This reuses an existing, already-bounded safety net rather than
adding a new one.

**Verification**: re-ran the validator on the same instance (0 failures,
previously 2), then across `tiny.json`, `warehouseSmall_100/600`,
`random_2000`, and `orz900d_5000` (0 failures on all, ~230K total per-path
checks). A production-scale bug like this is far more likely on small/maze
maps where coarse blocks are geometrically large relative to the
connectivity structure; larger, more open maps (`orz900d`) are less exposed
but not immune, hence keeping the fix general rather than special-casing tiny
maps.

## The container type is boost, not std

`inc/common.h` does `using boost::unordered_map;` at global scope (pulled in
transitively by nearly everything), so every unqualified `unordered_map<K,V>`
in this codebase — including `agent_guide_path`'s declared type in
`scheduler.cpp` and `get_guide_path()`'s return type in `scheduler.h` — is
actually `boost::unordered_map`, not `std::unordered_map`. This doesn't
affect behavior (the two are API-compatible for the operations used here) but
tripped up the validator's first draft (which assumed `std::unordered_map`
and got real compile errors about ambiguous/mismatched types) — worth knowing
if you write code against `get_guide_path()`'s result elsewhere.

## Solver 6's costs are not traffic-aware

`schedule_plan_flow_reduced` takes `background_flow`/`use_traffic` parameters
(matching solver 1's signature), but they are **only** used to decide whether
to fall back to `schedule_plan_flow` when the hierarchy isn't ready yet — the
coarsened hierarchy itself (`build_from_environment`, `MapCoarsenV1.cpp`) is
built once, during preprocessing, straight from `env->map`, with every fine
arc cost hardcoded to `1.0`. It never sees `background_flow`. So:

- Solver 1's arc costs vary timestep-to-timestep under `--useTraffic`
  (congestion-weighted), so `GuidePathCostSum` can and does diverge from
  `GuidePathLengthSum` for solver 1 (confirmed in the integration run below:
  length 361 vs. cost 514 at one sampled timestep).
- Solver 6's arc costs are always exactly 1.0/edge, so `GuidePathCostSum ==
  GuidePathLengthSum` always, confirmed in every test run including the full
  110-timestep integration run (`3644.0 == 3644.0`, `3511.0 == 3511.0`).

This isn't a bug in the new metric — `path_cost_on_fine_graph_local` looks up
real arc costs rather than hardcoding 1.0, so the equality is a faithful
report of the hierarchy's actual (traffic-blind) cost model, not an
implementation shortcut. It does mean `GuidePathCostSum` currently carries no
information beyond `GuidePathLengthSum` for solver 6 — if the hierarchy is
ever made traffic-aware, this metric starts actually meaning something
different for solver 6 too, without further code changes.

## Why `--assignNew` matters for cross-solver totals

Summing `GuidePathLengthSum` across an entire run's timesteps is **not**
directly comparable between solver 1 and solver 6 unless both are run with
`--assignNew 1` (`new_only=true`). Demonstrated on
`warehouseSmall_100.json`, `--useTraffic`, 110 timesteps:

| `--assignNew` | solver 1 total `GuidePathLengthSum` | solver 6 total | ratio |
|---|---|---|---|
| 0 (default) | 27906 | 3644 | 7.7x |
| 1 | 3517 | 3511 | 1.002x |

**Why**: with `new_only=false` (the CLI default), solver 1 re-offers every
already-assigned-but-not-yet-opened task to the flow solver **every
timestep** (`schedule_plan_flow`, the `else` branch that pushes
`task.second.agent_assigned` back into `flexible_agent_ids`/
`flexible_task_ids` when not `new_only`) and rebuilds + re-sums a guide path
for it each time, even though the agent/task pairing usually doesn't change.
Solver 6, by contrast, always pins already-assigned-but-unopened tasks
directly (`proposed_schedule[agent] = task`, skipping the coarse solver
entirely) — a deliberate stability fix from the original memleak
investigation to stop the coarse solver's blindness-to-fine-distance from
causing reassignment churn (`ai/project_context.md`, "Solver 6" section). One
side effect of that fix: solver 6 only ever costs a given task's guide path
**once**, at the timestep it's first assigned, while solver 1 (without
`--assignNew`) keeps re-costing the same in-flight task's guide path on every
subsequent timestep until it's opened. That's a pure re-computation-frequency
artifact with nothing to do with path quality, and it dominates the raw
per-run total.

With `--assignNew 1`, solver 1 also stops re-offering already-assigned tasks
(matching its own `new_only` branch), which brings its recomputation cadence
in line with solver 6's, and the totals converge to within ~0.2% on this
instance/run.

**Recommendation**: always pass `--assignNew 1` for both solvers when using
`GuidePathLengthSum`/`GuidePathCostSum` totals to compare assignment/path
quality across a run. Without it, a lower solver-6 total mostly reflects "pins
more, recomputes less," not "finds shorter paths." The single-snapshot
comparison the standalone validator does (one `compute_reduced_assignment`/
`schedule_plan_flow` call each, no repeated re-offering) sidesteps this issue
entirely and is safe to compare either way.

## New tool: `./build/guide_path_validator`

`utils/validation/validate_guide_paths.cpp` (new CMake target
`guide_path_validator`, built automatically by `./compile.sh` alongside
`lifelong`/`map_reduction_test`). Usage:

```shell
./build/guide_path_validator <instance.json> [--useTraffic]
```

Runs `schedule_plan_flow` and `schedule_plan_flow_reduced` once each on the
same freshly-loaded instance state, in two scenarios (traffic-seed gate open
at timestep 100, and gate closed at timestep 0), and checks every guide path
either solver returns for: non-emptiness, correct start location, correct end
location, 4-connected/walkable adjacency at every step, and that
`GuidePathLengthSum`/`GuidePathCostSum` match a from-scratch recomputation
over the exposed paths. Exit code 0 iff every check passes. `run.cpp`'s
instance-loading logic was extracted into
`map_reduction_test/instance_loader.{h,cpp}` so this second tool could reuse
it without duplicating ~50 lines or linking against `run.cpp`'s `main()`.

## Perf/regression check: solver 6 on `orz900d`, always-on fine lift

Decoupling solver 6's fine-lift from the traffic gate reintroduces
per-timestep lifting cost that was previously skipped entirely without
`--useTraffic` — exactly the code path implicated in the original OOM
(`ai/claude_memleak_fixes.md`). Re-ran the original OOM verification
scenario (`orz900d_5000`, `--scheduleModel 6`, no `--useTraffic`, 250
timesteps) to confirm the now-unconditional lift doesn't reintroduce it.

**Result: no regression.** `/usr/bin/time -v ./build/lifelong --inputFile
instances/custom/orz900d/orz900d_5000.json -o outputs/orz900d_solver6_regression.json
-s 250 -m 6` (no `--useTraffic`, default `--preprocessTimeLimit`/`--planTimeLimit`):

- Completed all 250 timesteps, exit status 0, `numPlannerErrors: 0`,
  `numScheduleErrors: 0`, 1826 tasks finished.
- **Maximum RSS: 2,317,320 KB (~2.32GB)**, sampled continuously throughout the
  run via `ps -o rss=` — flat from timestep ~20 onward, no growth trend. This
  matches the ~2.3GB flat baseline from the original post-fix verification
  (`ai/claude_memleak_fixes.md`, same instance/timestep count) almost exactly,
  despite the fine lift now running on every one of the 250 timesteps instead
  of never running at all in that instance/flag combination before.
- Wall clock 2m24s, 140s CPU time — the always-on lift adds real per-timestep
  cost (this run has no time-budget pressure since it's not competing with a
  tight `--planTimeLimit`-driven early-exit the way the historical throughput
  sweeps were), but nothing runaway.
- `total GuidePathLengthSum` across the run: 448,750 (no `--useTraffic`, so
  this reflects the hierarchy's static distances only, not congestion — see
  "Solver 6's costs are not traffic-aware" above).

Conclusion: the memory-leak-era fixes (bounded bridge-splice search, reused
already-bounded Step 3 path instead of re-deriving from shared flow, and the
heuristic-table LRU cache) hold up under the now-unconditional exposure this
change gives them — this was exactly the scenario (`orz900d`, solver 6, large
map, many agents) that originally triggered the OOM, and it stayed flat.
