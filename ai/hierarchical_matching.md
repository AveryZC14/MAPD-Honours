# Solver 6: hierarchical (cascaded) local matching

Added 2026-09-03. Extends the existing within-coarse-node local matcher
(`ai/local_node_matching.md`) from a single hierarchy level to a bottom-up
cascade across many levels, to cut the cost of `match_local_node_exact` at
deep `--flowSolveLevel` values, where it currently rivals or exceeds the
coarse flow solve itself (`ai/local_node_matching_runtime.md`: 43% of
combined time at level 8 on `scene_mp_4p_03`).

## Motivation

`compute_reduced_assignment`'s Step 1 (`ai/local_node_matching.md`) buckets
every flexible agent/task by their node at a single chosen level
(`--flowSolveLevel`) and exact-matches each bucket on real fine-grid
distance via `match_local_node_exact` (LEMON `NetworkSimplex` over a
complete bipartite graph, cost ~O(agents × tasks) per bucket). This is exact
and correct, but at deep levels the buckets get large (a single node can
span a huge region), and the measured runtime shows this cost growing worse
than linearly with level (0.010s -> 0.514s across levels 2->8 in
`ai/local_node_matching_runtime.md`, on the same agent count).

Key observation: `match_local_node_exact`'s cost is driven by the *product*
of a bucket's agent and task counts, and products are sub-additive under
partitioning -- splitting one big bucket into several smaller ones and
matching each separately is provably cheaper in total than matching the
merged bucket, provided most of the pairing happens within the smaller
groups rather than only balancing out once fully merged. Concretely: for a
229-agent bucket (the largest observed at level 8 in
`ai/local_node_matching_runtime.md`), matching it as ten ~23-agent groups
instead costs roughly `10 * 23^2` vs. `229^2` -- about a 10x reduction --
*if* the finer split still finds most of its pairs locally, which is the
expected case for anything but adversarially clustered agent/task
placement.

This is a distinct problem from the coarse *flow* solve's cost, which is
dominated by the size of the coarse graph at the chosen level (every node
and arc of that level's `top->g` is rebuilt every timestep regardless of
surplus size, `MapCoarsenV1.cpp` ~1800) -- that's a separate, larger cost
driver at *shallow* levels (108s/run at level 2 vs. 0.67s at level 8 on the
same sweep) and is untouched by this work. Hierarchical matching only
targets the local-match side of the cost, which is why it matters
specifically at the deep levels this project's benchmarking has already
identified as the throughput-winning regime.

## Prerequisite: hierarchy now coarsens to a natural fixpoint

Landed first (same session): `ReducedHierarchy::ensure()` used to build a
fixed compile-time number of levels (`kDefaultCoarsenLevels`, was 9). It now
coarsens all the way to the point where a level's 2x2-block grid itself
collapses to a single block (`coarse_rows == coarse_cols == 1`) --
`kMaxCoarsenLevels` (128) is only a generous safety ceiling on that loop,
never expected to bind for any real map. This matters here because the
cascade's "use hierarchical matching in its entirety" mode (below) needs a
real, well-defined top level to cascade up to, not an arbitrary fixed depth.

This also correctly handles maps with disconnected regions: `Coarsen()`
only ever merges nodes that are connected within a 2x2 block
(`collect_connected_components`), so a disconnected map's fixpoint level
ends up with more than one top node (one per component), not artificially
forced to 1 -- verified empirically on a synthetic 8x8 map split into two
disconnected halves by a solid wall: the hierarchy converged to a 3-level
top (`2, 4, then 2` nodes), correctly stopping at 2 top nodes instead of
looping or forcing a merge. See `ai/project_context.md`'s "Solver 6" section
for the fixpoint mechanics and the `schedulerHierarchyNumLevels` output
field this also added.

## The algorithm

State carried through the cascade, per remaining agent/task: its real fine
location (fixed) and its current level's node id (climbs one level at a
time via that level's `to_coarser_node_id`, already built and cached by the
hierarchy -- no repeated top-down remapping needed).

```
level = min_cascade_level
remaining = every flexible agent/task, mapped up to node ids at level `min_cascade_level`
            (one map_fine_node_to_level_node_local() walk per item, done once)

while level < cutoff_level:
    bucket remaining agents/tasks by their node id at this level
    for each node with both agents and tasks present:
        pairs = match_local_node_exact(...)      // unchanged, exact real-distance matching
        record pairs into the final assignment
        leftover (|agents - tasks| at this node) carries forward
    nodes with only agents, or only tasks, carry everyone forward untouched
    climb every surviving item to its parent: node = to_coarser_node_id[node]
    level += 1
    if remaining agents or remaining tasks is now empty: stop early (nothing left to match)

# hand off whatever's left at `cutoff_level` to the existing,
# UNMODIFIED compute_reduced_assignment() -- it does its own local match at
# that level, then flow for anything still left after that.
return cascade_assignments + compute_reduced_assignment(env, leftover_agents, leftover_tasks, ...)
```

`cutoff_level` is `env->flow_solve_level` (the existing `--flowSolveLevel`
control, unchanged) -- the cascade always hands off at exactly the level
flow already runs on, so no new "which level does flow run at" concept is
introduced.

### Why reusing `compute_reduced_assignment` unmodified for the hand-off is correct

The hand-off level's own local match (that function's existing Step 1) is
functionally identical to one more cascade iteration -- bucketing by that
level's node id and exact-matching real distances -- so there is no
special-casing needed at the boundary. This also means:

- **Passing `min_cascade_level >= flow_solve_level` disables cascading
  entirely** -- the `while` loop's range is empty, so the leftover set
  handed to `compute_reduced_assignment` is just the original flexible
  set, unchanged. This is byte-identical to calling
  `compute_reduced_assignment` directly, i.e. today's solver 6 behavior.
  Cascading is strictly opt-in.
- **"A few levels, then flow"**: `min_cascade_level = 1`,
  `flow_solve_level = 6` (say) -- cascade resolves levels 1-5, flow (with
  its own Step 1 local match first) handles level 6.
- **"Hierarchical matching in its entirety"**: set `flow_solve_level` to the
  hierarchy's own top level (from the fixpoint above). The cascade runs
  almost to the top; the final `compute_reduced_assignment` call's own Step
  1 local-matches everyone remaining in that one (or, for a disconnected
  map, few) top node(s) -- which is an exact matching over *every* possible
  remaining pairing, so there is nothing left for the flow step afterward to
  find. Flow becomes a vacuous no-op automatically (either one side of its
  supply/demand is empty, or there are no arcs to route through for a
  disconnected map's separate components) -- no special-casing needed, and
  it's cheap regardless since the top-level graph is tiny by construction.

### What's deliberately *not* new

- Guide-path lifting: cascade-matched pairs never touch the coarse graph,
  so -- exactly like today's flat local matching -- they don't get a lifted
  guide path from Steps 3/4 and fall back to the low-level planner's own
  seed. Same deferred status as `ai/local_node_matching.md`.
- Metrics: cascade match counts/time are folded into the *existing*
  `local_match_count_out`/`local_match_time_out` out-params (so
  `LocalNodeMatchCount`/`SchedulerLocalMatchTime` in the output JSON keep
  meaning "matched without touching the coarse flow graph" without any new
  JSON fields). `flow_match_count_out` continues to mean exactly what it did
  before -- matches recovered from the coarse flow's residual, which can
  now only happen at the hand-off level.

## API

New method on `ReducedHierarchy` (`map_reduction_test/MapCoarsenV1.h/.cpp`):

```cpp
std::unordered_map<int,int> compute_hierarchical_assignment(
    SharedEnvironment* env,
    const std::vector<int>& flexible_agent_ids,
    const std::vector<int>& flexible_task_ids,
    int min_cascade_level,
    std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
    bool need_guide_paths = true,
    double* solve_time_out = nullptr,
    double* guide_time_out = nullptr,
    double* guide_path_length_sum_out = nullptr,
    double* guide_path_cost_sum_out = nullptr,
    int* local_match_count_out = nullptr,
    int* flow_match_count_out = nullptr,
    double* local_match_time_out = nullptr,
    double* cascade_time_out = nullptr);
```

Same shape as `compute_reduced_assignment`, plus `min_cascade_level` (the
first level the cascade attempts, clamped to >= 1 -- level 0 is the
uncoarsened fine map, where matching would only ever catch agents/tasks
already at the exact same fine cell) and `cascade_time_out` (wall-clock
spent in the cascade loop itself, separate from `solve_time_out`/
`local_match_time_out`, which continue to describe the hand-off call as
before).

## Plumbing

- `SharedEnvironment::min_cascade_level` (`inc/SharedEnv.h`) -- new field,
  mirrors `flow_solve_level` exactly. Default disables cascading (>=
  `flow_solve_level`'s own default), so an unmodified CLI invocation behaves
  exactly as before.
- `--minCascadeLevel` CLI flag (`src/driver.cpp`), same pattern as
  `--flowSolveLevel`.
- `schedule_plan_flow_reduced` (solver 6, `default_planner/scheduler.cpp`)
  calls `compute_hierarchical_assignment` instead of
  `compute_reduced_assignment` directly. Solver 6 gains this capability
  in place rather than becoming a new solver number, since it's the same
  underlying hierarchy and the same hand-off machinery, not a structurally
  different coarse graph (unlike solver 7's edge-augmented graph, which
  *is* a new solver for exactly that reason). Disabled by default (no
  behavior change) until `--minCascadeLevel` is passed explicitly.

## Validation

**Full `./build/lifelong` simulation runs are not usable for A/B'ing this
change on this machine.** Two runs of the *identical* command (no code
difference, `--scheduleModel 6 --flowSolveLevel 4`, `warehouseSmall_200`,
`-s 60`) produced different `numTaskFinished` (180 vs. 172) purely from
wall-clock jitter interacting with `--planTimeLimit`-gated planner timeouts
-- this sandbox's per-timestep cost is close enough to the 1000ms default
that outcomes aren't reproducible run-to-run regardless of scheduler logic.
Raising `--planTimeLimit` to remove the jitter made single runs too slow to
finish inside a usable window in-line, but one such run (`-t 20000`, same
command, left running in the background) confirms this isn't just a
tight-timeout artifact: even at a 20x larger budget the two identical runs
still diverged (179 vs. 177 `numTaskFinished`, non-equal `timeStepMetrics`).
So instead of simulation-level A/B, validated directly against the
frozen-batch, no-simulation-loop, exactly-reproducible matching functions
themselves.

### New tool: `./build/hierarchical_matching_validator <instance.json> [flowSolveLevel]`

(`utils/validation/validate_hierarchical_matching.cpp`, same convention as
`guide_path_validator`/`hierarchy_cache_validator`.) Loads an instance once
(all agents free, all tasks revealed -- `populate_env_from_instance`), then
calls the matching functions directly on that one frozen batch: no
simulation loop, no timing dependency, exactly reproducible. Checks:

1. **Disabled cascade == direct call.** `min_cascade_level == flow_solve_level`
   and `min_cascade_level` far above it both must produce an assignment map
   *byte-identical* to calling `compute_reduced_assignment()` directly.
2. **Assignment validity**, for every mode tested (including
   `min_cascade_level` clamped up from a negative input): no agent or task
   appears more than once, every id in the result came from the input
   flexible sets, and `local_match_count + flow_match_count` equals the
   number of pairs returned.
3. **Entirety mode**: `flow_match_count_out == 0` (flow is a vacuous no-op
   once the hand-off level is the hierarchy's real top), and the total
   matched count equals `min(agents, tasks)` exactly.

### Results

- **`instances/custom/tiny/tiny.json`** (2 agents, 2 tasks, 4-level
  hierarchy): 16/16 checks pass.
- **A synthetic disconnected map** (8x8, split into two disconnected halves
  by a solid wall -- same map used to verify the fixpoint-coarsening change):
  16/16 checks pass, including entirety mode correctly matching exactly 2
  (not more) once both a left-side and a right-side task existed. (First
  attempt at this test data used the wrong `.tasks` file format --
  `read_int_vec`'s task-file parser treats every comma-separated value on a
  line as one more *stop* in that task's own multi-stop errand, not
  `task_id,location`; task ids are just the line's index. `0,57` is a
  *two-stop* task `[0, 57]`, not "task 0 at location 57" -- this produced a
  real-looking but spurious failure (only 1 of 2 possible pairs found, at
  distance 0) that traced back to bad test data, not the matching code, once
  the actual locations being matched were checked against the file format
  rather than assumed.)
- **`instances/warehouseSmall/warehouseSmall_200.json`** (200 agents, 2000
  tasks, 7-level hierarchy) at `flowSolveLevel` 4 and 6: 16/16 checks pass
  at both. Total real (Manhattan) distance on this one frozen initial batch:

  | flowSolveLevel | single-level (today) | cascade (min_cascade_level=1) | entirety (top=6) |
  |---|---|---|---|
  | 4 | 1170 (200 matched) | 1207 (193 local / 7 flow) | 1151 (200 local / 0 flow) |
  | 6 | 1112 (200 matched) | 1151 (200 local / 0 flow) | 1151 (200 local / 0 flow) |

  (`flowSolveLevel=6` *is* this map's top level, so "cascade" and "entirety"
  coincide there -- expected, and a useful internal-consistency check that
  they really do produce identical output when they're the same call.)

  The cascade is slightly *worse* than the single-level baseline on this one
  batch at both levels tested (+3.2% at level 4, +3.5% at level 6) -- not a
  bug, the same already-documented caveat from `ai/local_node_matching.md`
  applies here too: locally matching off pairs at fine levels changes which
  agents/tasks are left as "surplus" for the coarser stages, and that
  surplus's own routing is only optimal in *its* stage's cost terms, not
  globally. This is one frozen batch, not a full-simulation average, so it
  should not be read as "cascading regresses quality" -- it's exactly the
  kind of measurement `ai/local_node_matching.md` warns not to skip, and the
  next step (before trusting this in a real sweep) is the same kind of
  before/after realized-distance table that feature was validated with,
  ideally at the deep levels (`ai/local_node_matching_runtime.md`'s level
  6-8 regime) where the *time* saved is the actual point, not distance.

### Not yet done

- No wall-clock runtime comparison yet (the actual point of this feature --
  does cascading measurably cut `SchedulerLocalMatchTime` at deep
  `--flowSolveLevel`s the way the sub-additivity argument predicts). Blocked
  on the same simulation-nondeterminism problem above; would need either a
  dedicated low-jitter benchmarking run (matching this repo's existing
  sweep methodology, `ai/auto_benchmarking.md`) or a synthetic large-batch
  timing harness built the same way as this validator.
- No full-simulation `tasksFinished`/throughput sweep yet -- same blocker.
