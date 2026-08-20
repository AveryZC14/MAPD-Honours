# Solver 7 (planned): edge-node augmented coarse graph

Design doc, written 2026-08-20, before implementation. Captures the design
decided on with the user for a new scheduler variant, solver id **7**, that
is "near-identical to solver 6 except the graph used for the per-timestep
flow solve gets an explicit node inserted on every coarse-to-coarse edge."
This file is the plan; nothing described here is implemented yet (see
"Implementation plan" for what still needs to be written and where).

## Motivation

Solver 6's coarse flow graph (`ReducedHierarchy::compute_reduced_assignment`,
`MapCoarsenV1.cpp:1450`, see `ai/project_context.md`'s "Solver 6" section)
connects every surplus agent to its top-level coarse node with a **cost-0**
arc (`src_arc_by_top`, `MapCoarsenV1.cpp:1646`) and every surplus task to its
node the same way (`sink_arc_by_top`, `:1660`). Cost only enters once flow
crosses a coarse-to-coarse arc (`top->cost[arc]`, an aggregated
average/minimum over the fine arcs that used to cross that boundary, computed
once at hierarchy-build time in `Coarsen()`, `MapCoarsenV1.cpp:865-899`). Two
agents sitting at opposite ends of the same (possibly large, at deep
coarsening levels) coarse node region are treated as equally close to every
neighboring region — the flow solve has no fine-grained signal for "this
agent is actually already near the north exit, that one's near the south
exit." `ai/local_node_matching.md` fixed the analogous problem for
same-node agent/task pairs; this is the same class of information loss, one
level up, for the surplus agents/tasks that *do* reach the coarse flow.

The idea: give the flow solve a real (if approximate) position-aware cost for
"how far is this agent from actually leaving toward neighbor B" instead of a
flat 0, by inserting an explicit node on every coarse-to-coarse edge and
routing agents/tasks through the *specific* edge-nodes adjacent to their own
region rather than through the region node itself.

## Scope decision: flow-solve level only, not the whole hierarchy

Two ways to build this were considered:

- **Every hierarchy level** (rewrite `Coarsen()` itself so every level's
  graph contains edge-nodes as first-class nodes). Rejected: a later
  `Coarsen()` call groups the *previous* level's nodes into 2x2 blocks by
  `coarse_location`; an edge-node has no natural single coarse cell to live
  in (it sits between two), so grouping, bridge caching, serialization, and
  Steps 3/4 lifting would all need new logic to handle a mixed node-kind
  graph recursively. Large, invasive, touches code that's already been
  hardened through several rounds of bug-fixing (`ai/claude_memleak_fixes.md`,
  `ai/guide_path_metric.md`).
- **Flow-solve level only** (chosen). The persistent hierarchy
  (`MultiLevelCoarsenedGraph`, `Coarsen()`, `bridge_cache`,
  `MapCoarsenSerialize`, Steps 3/4 lifting) stays **completely untouched** —
  solver 7 reuses the exact same hierarchy-build path as solver 6. Only the
  per-timestep flow-graph construction (solver 6's Step 1) is replaced: given
  whichever level `top = hierarchy_.level(top_level_idx)` already is, build
  an *augmented* `lemon::ListDigraph` where every existing arc of `top->g` is
  subdivided by one edge-node. This is new code living entirely alongside
  solver 6's, not a modification to it.

This keeps solver 7 an additive, isolated change — same pattern this repo
already uses for `mapReductionV0.*` (superseded but kept, still compiled) and
for solver 1 vs. 6 (both still exist, dispatched by a plain int).

## Structural design

### Topology

For the chosen top level's graph `top->g`, for every unordered pair of
4-adjacent coarse (region) nodes `{A, B}` that have at least one arc between
them in `top->g`:

1. Create one new **edge-node** `E_AB` (one per unordered pair, not per
   direction — a single node sits on the boundary and carries traffic both
   ways).
2. For the existing `A -> B` arc (cost `c_ab`): replace it with `A -> E_AB`
   (cost `c_ab / 2`) and `E_AB -> B` (cost `c_ab / 2`).
3. Symmetrically, for the existing `B -> A` arc (cost `c_ba`, generally
   `!= c_ab` because `Coarsen()`'s directional internal-penalty term is not
   symmetric — see `MapCoarsenV1.cpp:887-894`): replace it with `B -> E_AB`
   (cost `c_ba / 2`) and `E_AB -> A` (cost `c_ba / 2`).
4. **The original direct `A -> B` / `B -> A` arcs are removed, not kept
   alongside.** A path that used to cost `c_ab` going straight A→B now costs
   `c_ab/2 + c_ab/2` going A→E_AB→B — same total, just routed through the new
   node, with no same-cost shortcut that bypasses it.

Capacity on the region↔edge arcs mirrors solver 6's existing (already
effectively-unconstrained) pattern: `capacity = num_workers`, same as
`MapCoarsenV1.cpp:1683` today. Fixing that pre-existing looseness is out of
scope for this change — solver 7 should differ from solver 6 only in the way
described here, not pick up an unrelated capacity-modeling fix as a side
effect.

### Agent/task proxy nodes

Solver 6's Step 1 connects the source to each occupied top node with **one**
arc whose capacity is the count of surplus agents there
(`src_arc_by_top`/`sink_arc_by_top`, `MapCoarsenV1.cpp:1638-1664`) — cost 0,
because all agents at a node were treated as interchangeable. Solver 7 can no
longer do that, because the whole point is to give agents at the same region
node *different* costs depending on where in that region they actually are.
So instead, **after** the local-node-matching pass removes same-node
agent/task pairs (identical to solver 6's Step 1 up through
`MapCoarsenV1.cpp:1602` — unchanged), for every remaining surplus agent/task:

1. Create a dedicated **proxy node** for that agent (or task) — one node per
   agent/task, not shared.
2. `source -> agent_proxy` (cost 0, capacity 1); `task_proxy -> sink` (cost 0,
   capacity 1) — mirrors solver 6's source/sink pattern, just per-unit
   instead of per-node-batched.
3. `agent_proxy -> E` for **every edge-node E adjacent to the agent's own top
   node** (i.e., every neighbor reachable via a single coarse hop — up to 4,
   from 4-adjacency) — **never** an arc to the region node itself. Symmetric
   for `E -> task_proxy` on the sink side.
4. **Do not** additionally connect `agent_proxy` to the plain region node.
   The plain region node stays purely a pass-through hop in the region↔edge
   backbone, used only by other agents' paths that traverse this region
   without originating or ending here (this is guaranteed to be safe: after
   local matching, a node has agent-surplus or task-surplus, never both, so
   an agent's own node never has a task for it to reach with a 0-length hop
   anyway).

### Cost formula for `agent_proxy -> E` (and `E -> task_proxy`)

Chosen formula — **Manhattan distance from the agent's real fine-grid
`(row, col)` to the axis-aligned bounding box of the *neighboring* region's
fine-cell group** (the region on the far side of `E` from the agent's own
region), computed as:

```
dr = max(0, min_row_B - r, r - max_row_B)
dc = max(0, min_col_B - c, c - max_col_B)
cost(agent_proxy -> E_AB) = dr + dc
```

where `(min_row_B, max_row_B, min_col_B, max_col_B)` is region `B`'s
bounding box over **every fine cell** ultimately belonging to it. `dr + dc =
0` whenever `(r, c)` is already inside the box.

Computed with a **bottom-up sweep**, not a per-region top-down recursion
through `to_finer_node_ids` (that was the original idea; a single combined
pass is simpler and does the same total work without redoing it per region):
start every level-0 cell as its own 1x1 box (its own `fine_location`), then
for each level `1..top_level_idx`, walk that level's nodes and use their
existing `to_coarser_node_id` parent pointer (already populated by
`Coarsen()`, `MapCoarsenV1.cpp:461-468`) to fold each node's running box into
its parent's box one level up. One pass per level, total work bounded by the
fine map's cell count, done once per `(hierarchy signature, top_level_idx)`
and cached (see "Structural design" below) — never recomputed per agent or
per timestep.

This is an **O(1)-per-query approximation, not an exact nearest-fine-cell
distance** — chosen deliberately for speed over precision, per discussion:

- It's a lower bound, not the true minimum distance to the nearest actual
  (walkable, connected) cell in `B`'s group: a box corner may not correspond
  to any real cell in an irregularly-shaped or non-convex component, and it
  ignores walls/terrain entirely (no BFS/A* involved).
- Per-agent/task query cost is O(1): two `max(0, ...)` comparisons per axis,
  against the precomputed box.

Alternatives considered (not chosen, recorded per request):

| Approach | Cost per query | Accuracy | Why not chosen |
|---|---|---|---|
| Exact min over every fine cell in B's group | O(\|B\|) per query | Exact (still Manhattan, not path-aware) | Groups can be large at deep coarsen levels; O(\|B\|) per agent per neighbor per timestep doesn't scale the way the bbox does |
| Min over a precomputed boundary-cell subset of B (only cells adjacent to a different region) | O(boundary size) per query, smaller upfront pass | Closer to exact than the bbox for concave shapes, still ignores terrain between agent and boundary | More bookkeeping (need to identify and store the boundary subset per region per level) for a moderate accuracy gain; bbox chosen as the simpler starting point, with this as the natural first upgrade if bbox proves too crude |
| Distance to B's single `chosen_finer_node_id` representative (recursively resolved to level 0) | O(1), no bbox needed at all | Least accurate — one arbitrary point, not even a bounding shape | Already known to be a weak proxy elsewhere in this codebase (it's why local node matching and this change both exist — arbitrary representatives lose real-distance signal) |

If the bbox approximation turns out to be too crude in practice (e.g. an
oddly-shaped region whose bbox is much bigger than the region itself,
making every agent look "close" to it regardless of real walking distance),
the boundary-cell-subset option is the recommended next step, not the exact
per-cell scan.

## Structural design

`ReducedHierarchy` (`MapCoarsenV1.h:227-282`) does **not** grow a second
graph inside it, and is not subclassed — solver 7 gets its own sibling
singleton that *composes* `ReducedHierarchy` rather than extending it. This
section supersedes the "Caching" sketch from the first draft with the actual
shape, including the one place this does reach back into V1.

### New types, in `map_reduction_test/EdgeAugmentedCoarsen.h`/`.cpp`

```cpp
struct RegionBBox { int min_row, max_row, min_col, max_col; };

struct EdgeAugmentedTopGraph
{
    int top_level_idx = -1;
    lemon::ListDigraph g;                       // region-nodes + edge-nodes only; no source/sink/proxies
    lemon::ListDigraph::ArcMap<double> cost;
    lemon::ListDigraph::ArcMap<int> capacity;

    std::vector<lemon::ListDigraph::Node> region_node;   // index: top-level region id -> Node in g
    std::vector<RegionBBox> region_bbox;                 // index: top-level region id -> its fine-cell bbox

    struct AdjacentEdge { lemon::ListDigraph::Node edge_node; int neighbor_region_id; };
    std::vector<std::vector<AdjacentEdge>> region_adjacent_edges; // index: region id -> up to 4 entries
};

std::unique_ptr<EdgeAugmentedTopGraph> build_edge_augmented_top_graph(const CoarsenedGraph& top);

class EdgeAugmentedHierarchy
{
public:
    static EdgeAugmentedHierarchy& instance();

    // Ensures MapReductionTest::ReducedHierarchy::instance().ensure(env) has
    // run (same underlying hierarchy, no separate build), then ensures the
    // cached backbone matches `top_level_idx` and the current map signature
    // -- rebuilds (single slot, not a per-level map) only when either
    // changed. Cheap no-op otherwise, same pattern as ReducedHierarchy::ensure.
    void ensure(const SharedEnvironment* env, int top_level_idx);
    bool ready() const;

    std::unordered_map<int,int> compute_reduced_assignment_edge_augmented(
        SharedEnvironment* env,
        const std::vector<int>& flexible_agent_ids,
        const std::vector<int>& flexible_task_ids,
        std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
        bool need_guide_paths,
        double* solve_time_out, double* guide_time_out,
        double* guide_path_length_sum_out, double* guide_path_cost_sum_out,
        int* local_match_count_out, int* flow_match_count_out,
        double* local_match_time_out,
        double* backbone_build_time_out);   // new -- see "New metric" below

private:
    std::size_t env_signature_ = 0;
    std::unique_ptr<EdgeAugmentedTopGraph> backbone_;
};
```

### Per-timestep flow graph

`compute_reduced_assignment_edge_augmented` builds a fresh `lemon::ListDigraph`
each call via `lemon::digraphCopy(backbone_->g, g).arcMap(...).nodeRef(node_ref).run()`
(LEMON's `DigraphCopy` copy-with-maps utility, declared in `lemon/core.h` —
already pulled in transitively by `lemon/list_graph.h`/`lemon/network_simplex.h`,
which this file already includes, so no new header dependency), then adds
`source`/`sink`/one proxy node per surplus agent/task into the copy,
using `node_ref[backbone_->region_adjacent_edges[region_id][k].edge_node]` to
find each adjacent edge-node's handle *in the copy*. A parallel
`std::vector<int> copy_node_to_region_id` (built at copy time from
`node_ref[backbone_->region_node[region_id]]`, size = copy graph's node count,
`-1` for non-region nodes) gives Step 2's path-filtering an O(1) "is this
node a region node, and which one" lookup. The copy is discarded at the end
of the call — chosen over mutating the persistent backbone in place and
`erase()`-ing proxies afterward, because a missed erase on some early-return
path (there are several: `!ready()`, `ns_status != OPTIMAL`, `!need_guide_paths`)
would silently corrupt the backbone for every following timestep, which is a
much worse failure mode than an extra copy. Solver 6 already rebuilds its own
(smaller) flow graph fresh every timestep, so this isn't a new pattern, just
a bigger copy.

### The one place this reaches into `MapCoarsenV1.{h,cpp}`

Steps 1, 3, and 4 all depend on five helper functions defined `static` (file-
internal linkage) in `MapCoarsenV1.cpp`: `is_valid_graph_node_id_local`,
`map_fine_node_to_level_node_local`, `shortest_path_in_graph_local`,
`path_cost_on_fine_graph_local`, `expand_path_batch_one_level_local`. A
separate translation unit cannot call them as they stand, so **some** change
to V1 is unavoidable — decided as follows:

- **`is_valid_graph_node_id_local`, `map_fine_node_to_level_node_local`**
  (`MapCoarsenV1.cpp:1105-1146`, ~10 lines each, pure/stateless, never
  touched by any past bug fix) — duplicated verbatim into
  `EdgeAugmentedCoarsen.cpp`. Negligible drift risk; not worth a shared
  header for two one-line-bodied functions.
- **`shortest_path_in_graph_local`, `expand_path_batch_one_level_local`,
  `path_cost_on_fine_graph_local`** (`MapCoarsenV1.cpp:1148-1449`, the actual
  lifting logic, and specifically the code `ai/guide_path_metric.md` and
  `ai/guide_path_visualisation.md` each found a real bug in) — **not**
  duplicated. Instead, `ReducedHierarchy` gains one new public method:

  ```cpp
  // Lift an already-decided batch of (agent, task, region-node-only coarse
  // path) triples down to concrete fine-graph guide paths -- exactly what
  // compute_reduced_assignment's own Steps 3/4 do internally, extracted so
  // a second caller (solver 7's edge-augmented flow) can reuse it without a
  // second copy of the lifting logic. `coarse_region_paths[i]` must already
  // have every edge-node/proxy-node filtered out by the caller.
  void lift_coarse_paths_to_fine(SharedEnvironment* env,
                                  int top_level_idx,
                                  const std::vector<int>& agent_ids,
                                  const std::vector<int>& task_ids,
                                  std::vector<std::vector<int>> coarse_region_paths,
                                  std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
                                  double* guide_time_out,
                                  double* guide_path_length_sum_out,
                                  double* guide_path_cost_sum_out);
  ```

  `compute_reduced_assignment`'s own Steps 3/4 (`MapCoarsenV1.cpp:1821-1949`)
  are refactored to call this same method — a behavior-preserving extract,
  not new logic, so solver 6 is unaffected functionally but does get touched
  mechanically (worth a rebuild + rerun on a small instance to confirm
  identical output before moving on, same verification standard the past
  cleanup passes in this file have used).

### New metric: backbone build/copy time

Solver 7 has a cost category solver 6 doesn't: building the persistent
backbone (first call only) and copying it into a fresh graph (every call).
Tracked from the start, mirroring how `local_match_time` was added
(`ai/local_node_matching.md`):

- `ScheduleTiming` (`default_planner/scheduler.h:18-43`) gains
  `double backbone_build_time = 0.0;`.
- `set_last_reduced_timing(...)` (`scheduler.h:50-58`) gains one more
  trailing optional parameter, `double backbone_build_time = 0.0` — default
  keeps solver 6's existing call site (`scheduler.cpp:1088`) unchanged.
- `TimeStepMetric` (`inc/CompetitionSystem.h:15-43`) gains
  `double SchedulerBackboneBuildTime = 0.0;`, wired through both population
  sites in `src/CompetitionSystem.cpp` (mirroring `SchedulerLocalMatchTime`
  at `:270`/`:301`) and the JSON output block (mirroring `:409`).

## Implementation plan (files, not yet written)

Following the sibling-module precedent already in this repo
(`mapReductionV0.*` kept alongside `MapCoarsenV1.*`):

- **New**: `map_reduction_test/EdgeAugmentedCoarsen.h` / `.cpp` — everything
  in "Structural design" above: `EdgeAugmentedTopGraph`,
  `build_edge_augmented_top_graph` (backbone + bbox precompute),
  `EdgeAugmentedHierarchy`, `compute_reduced_assignment_edge_augmented`.
  Step 1 (bucketing + local matching, up to the point the flow graph is
  built) is copied from `MapCoarsenV1.cpp:1493-1602` — this part genuinely
  doesn't change, just needs its own copy since it lives in a different
  function now. Step 2 is new (BFS over the augmented graph, path-filtering
  down to region-node ids). Steps 3/4 call `ReducedHierarchy::instance()
  .lift_coarse_paths_to_fine(...)` (new method, see above) rather than
  reimplementing anything.
- **Edit**: `map_reduction_test/MapCoarsenV1.h`/`.cpp` — add the
  `lift_coarse_paths_to_fine` method (extracted from existing Steps 3/4,
  behavior-preserving); `compute_reduced_assignment` calls it internally too.
  This is the only change to solver 6's own file.
- **Edit**: `CMakeLists.txt` — add
  `list(APPEND SOURCES "map_reduction_test/EdgeAugmentedCoarsen.cpp")` next to
  the existing `MapCoarsenV1.cpp` line (~line 46); the file is not picked up
  by the `src/*.cpp`/`default_planner/*.cpp` glob automatically.
- **Edit**: `default_planner/scheduler.h` / `scheduler.cpp` — add
  `schedule_plan_flow_reduced_edge(...)` with the same signature as
  `schedule_plan_flow_reduced` (scheduler.h:71), calling
  `EdgeAugmentedHierarchy::instance().compute_reduced_assignment_edge_augmented(...)`.
  Also extend `schedule_initialize`'s hierarchy-build gate
  (`scheduler.cpp:101`) from `solver == 6` to `solver == 6 || solver == 7`
  (same underlying hierarchy, so the same `ensure()` call covers both). Add
  the `ScheduleTiming`/`set_last_reduced_timing` changes from "New metric"
  above.
- **Edit**: `src/TaskScheduler.cpp` — add an `else if (solver == 7)` branch
  (`:62-66`) dispatching to `schedule_plan_flow_reduced_edge`, and update the
  fallback error's `"1..6"` message (`:69`) to `"1..7"`.
- **Edit**: `inc/CompetitionSystem.h` / `src/CompetitionSystem.cpp` — add
  `SchedulerBackboneBuildTime` to `TimeStepMetric` and wire it through, per
  "New metric" above.
- **Edit**: `src/driver.cpp` — no new CLI flag is strictly required (solver 7
  reuses `--flowSolveLevel`/`--hierarchyCache` unchanged, since it shares
  solver 6's hierarchy); optionally mention solver 7 in the `--scheduleModel`
  help text (`:57`).
- **Docs**: update `ai/project_context.md`'s solver-dispatch table and repo
  map once solver 7 exists and has been benchmarked, same as every other
  addition documented there.

`MapCoarsenSerialize.*`, `LocalNodeMatch.*`, and all existing
visualization/validator tools are unaffected — the only touch to solver 6's
own code is the `lift_coarse_paths_to_fine` extraction, which is verified
behavior-preserving (rebuild + rerun solver 6 on a small instance, confirm
identical output) before solver 7 is trusted to build on top of it.

## Open items for a later pass (not blocking first implementation)

- Whether the boundary-cell-subset refinement (see cost-formula table above)
  is worth adding, once solver 7 has real benchmark numbers to compare
  against solver 6 (same pattern as `ai/local_node_matching.md`'s
  quantify-then-decide approach).
- Whether the region↔edge backbone's capacity should eventually be derived
  from something real (e.g. count of fine boundary crossings) instead of
  mirroring solver 6's unconstrained `num_workers` placeholder — flagged as
  out of scope above, revisit only if solver 7's results motivate it.
