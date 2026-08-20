# Solver 7: edge-node augmented coarse graph

Design doc, written 2026-08-20; implemented and verified the same day (see
"Implementation status" below). Captures the design decided on with the user
for a new scheduler variant, solver id **7**, that is "near-identical to
solver 6 except the graph used for the per-timestep flow solve gets an
explicit node inserted on every coarse-to-coarse edge."

## Implementation status: done, verified

All of "Implementation plan" below is written and wired in
(`--scheduleModel 7`). Verified:
- **Solver 6 regression**: `lift_coarse_paths_to_fine`'s extraction out of
  `compute_reduced_assignment` (the one planned touch to `MapCoarsenV1.cpp`)
  produces byte-identical output (every field except wall-clock timing) to
  the pre-extraction code, on `tiny` (50 timesteps) -- checked by reverting
  the extraction, capturing a baseline, reapplying it, and diffing.
- **Solver 7 functional**: `tiny` (50 timesteps) -- `numTaskFinished`,
  `totalLocalNodeMatchCount`, `totalFlowMatchCount` all exactly match solver
  6's numbers on the same instance/timesteps.
- **Solver 7 at scale**: `warehouseSmall_100`, 200 timesteps, no
  `--useTraffic` -- completed cleanly (0 planner/schedule errors), 384 tasks
  finished vs. solver 6's 379 on the same run (not a benchmark claim, just a
  sanity check that it's in the same ballpark and doesn't regress). Solve
  time stayed under 3ms/timestep (vs. solver 6's ~1ms), comfortably inside
  the 1000ms default `--planTimeLimit`. `SchedulerBackboneBuildTime` was 0 on
  every timestep, confirming the eager pre-build in `schedule_initialize`
  worked (no runtime rebuild cost).
- **Solver 7 with `--useTraffic`**: `warehouseSmall_100`, 150 timesteps --
  completed cleanly; `GuidePathLengthSum`/`GuidePathCostSum` populated with
  sane (non-negative, non-NaN) values on 121/150 timesteps once past the
  timestep-100 seed gate, confirming Steps 3/4 lifting (via the shared
  `lift_coarse_paths_to_fine`) works correctly through the edge-augmented
  path.
- Full project (`lifelong`, `map_reduction_test`, `guide_path_validator`,
  `hierarchy_cache_validator`, `dump_guide_paths`, `dump_coarsening`,
  `analyze_coarse_collisions`) builds clean with no new warnings.

See "Hiccups encountered during implementation" near the end for two real
bugs (one in this new code, one latent and harmless in solver 6's existing
code) found and fixed while getting to the above.

## Structural + scale validation (2026-08-20, `orz900d`)

Follow-up pass, prompted by "confirm the node hierarchies are built properly
... and that this does properly scale up to much larger maps." The checks
above only establish that solver 7 *runs* and produces plausible assignment
counts -- not that the backbone's actual topology is correct, and only on
small maps. This pass adds a real structural rigor check and a stress test on
`orz900d` (656x1491, ~978K cells, the repo's main large-map stress test).

### New tool: `./build/edge_augmented_validator`

`utils/validation/validate_edge_augmented_graph.cpp`, wired into
`CMakeLists.txt` alongside the other validators. Usage:
`./build/edge_augmented_validator <instance.json> [level] [bbox_sample]`.
Builds the hierarchy and backbone exactly as the runtime path does, then
checks invariants that must hold by construction regardless of map
size/shape:

1. **Arc count**: backbone arc count == 2 × (top-level coarse graph's arc
   count) -- every coarse arc becomes exactly two backbone arcs (in, out of
   its edge-node), never more or fewer.
2. **Adjacency bookkeeping**: sum of every region's `region_adjacent_edges`
   list length == 2 × (edge-node count) -- each edge-node is registered with
   exactly its two endpoint regions, no double-registration or omission.
3. **Per-node degree**: every region node's backbone in/out-degree exactly
   matches its degree in the original top-level coarse graph; every
   edge-node has in-degree exactly 2 and out-degree exactly 2 (one region on
   each side, one arc each direction) -- checked for *every* node in the
   backbone, not sampled.
4. **Cost reconstruction**: for every edge-node, (region→edge cost) +
   (edge→region cost) exactly reconstructs the original coarse arc's cost
   that the edge-node replaced (i.e. nothing was lost or double-counted by
   the halving).
5. **Bounding-box correctness**: `compute_region_bboxes`' fast bottom-up
   sweep (the one actually used at runtime) is cross-checked against a
   deliberately separate, naive, brute-force recursive descent through
   `to_finer_node_ids` all the way to level 0 -- a real independent
   reference implementation, not a second copy of the same algorithm -- for
   a sample of regions (default 500, evenly spaced across the coarse grid;
   exhaustive on small maps).

**Results**: `tiny` (8x8) at levels 1/2 -- 10/10 checks pass; at levels 3+
(map fully collapsed to one region, no edges) the checks that require an
edge to exist correctly no-op rather than false-failing (fixed a validator
over-assertion here, not a backbone bug). `orz900d` at levels 1, 2, 3, 4 --
10/10 checks pass at every level, including the full (non-sampled) degree
check over 20,939 backbone nodes / 53,112 arcs at level 2. Backbone build
time at every level tested: under 70ms, negligible next to the ~0.6s
hierarchy build itself.

### `--hierarchyCache` confirmed shared with solver 6

Solver 7's `EdgeAugmentedHierarchy::ensure()` calls
`ReducedHierarchy::instance().ensure(env)` as its first step (see
"Structural design: solver-side composition" below) -- which is the *only* place `--hierarchyCache`
load/save logic lives (`ReducedHierarchy::ensure`, `MapCoarsenV1.cpp:1039`).
So solver 7 automatically gets the same disk cache for the underlying
hierarchy, with no separate flag or code path needed. Confirmed directly
(not just by reading the call chain) with a temporary debug print at the
`loaded_from_cache` branch point, run on `orz900d_5000`:
`--scheduleModel 7 --hierarchyCache hierarchy_cache/orz900d.hier` →
`loaded_from_cache=1`, same as `--scheduleModel 6` with the same flag; a run
with no `--hierarchyCache` flag at all → `loaded_from_cache=0` (fresh
build), for both solvers. Removed after confirming.

One caveat worth knowing, discovered chasing this down: **the top-level
`schedulerHierarchyBuildTime` JSON field is not a reliable way to check this
indirectly.** It's sourced from whatever `last_scheduler_timing` holds after
the *last* scheduling call of the run, and `schedule_plan_flow_reduced`/
`_edge`'s own "no flexible agents or tasks this timestep" early-return path
calls `set_last_timing(...)` instead of `set_last_reduced_timing(...)`,
which explicitly zeroes `hierarchy_build_time` -- so this field reads 0
whenever the *final* timestep of a run happened to have no flexible work,
regardless of whether the hierarchy was actually built or loaded from cache
at the start. Pre-existing behavior of `ScheduleTiming`/`set_last_timing`,
identical for both solvers, not something solver 7 introduced -- flagging it
here since it's exactly the kind of thing that looks like a caching bug at a
glance and isn't one. The `loaded_from_cache` debug print (or, for a
reusable check, `hierarchy_cache_validator`'s existing round-trip test) is
the trustworthy way to verify this; the JSON summary field isn't.

Separately: solver 7's *own* additional structure -- the edge-node backbone
itself -- has no disk cache of its own (`MapCoarsenSerialize.h` only knows
about `MultiLevelCoarsenedGraph`). Not an oversight: the backbone build is
cheap enough in-memory (under 70ms on `orz900d`, see "New tool" above,
happening once at process start via the eager `schedule_initialize` call)
that a dedicated serialize/deserialize path wasn't worth the added
complexity. If a future map is large enough that this stops being true,
serializing `EdgeAugmentedTopGraph` the same way `MapCoarsenSerialize`
already does for `MultiLevelCoarsenedGraph` would be the natural next step.

### Aside: confirmed pre-existing run-to-run non-determinism, not a regression

While re-verifying solver 6 was still byte-identical after this session's
changes, one comparison against an early-session baseline showed a
one-timestep shift in exactly when a single flow-match event occurred
(`FlowMatchCount`/`GuidePathLengthSum` moving from timestep *N* to *N+1*
between runs). Investigated rather than dismissed, since a real regression
was possible. Ran the identical `--scheduleModel 6` command twice in a row
with zero code changes in between and got two *different* outputs showing
the same one-timestep-shift signature -- confirming this is pre-existing
run-to-run jitter in the simulation itself (already flagged in
`validate_hierarchy_cache.cpp`'s own comments: "run-to-run simulation output
is only weakly deterministic"), not something introduced by any of this
session's changes. Plausible mechanism: wall-clock-timing-dependent
scheduler call budgets (`TaskScheduler::plan`'s `limit = time_limit/2 -
SCHEDULER_TIMELIMIT_TOLERANCE`) occasionally shifting which side of a
near-tie a match resolves to. Noted here so a future session doesn't
re-diagnose the same thing from scratch -- any solver-6-regression check
should tolerate a small number of single-timestep event shifts rather than
requiring byte-for-byte identity, or should compare aggregate/final-state
fields (`numTaskFinished`, `makespan`) instead of per-timestep event timing.

### Scale test: `orz900d_5000` (5000 agents, ~978K cells), full `lifelong` runs

250 timesteps, `--hierarchyCache hierarchy_cache/orz900d.hier` (pre-built,
reused for both solvers so hierarchy-load cost is identical and isolated
from what's actually being tested), solver 6 vs. 7, at `--flowSolveLevel 2`
(default) and `4`, run solo (not in parallel -- see the memory finding
below for why that turned out to matter) with RSS sampled every 2s via
`/proc/<pid>/status`:

| level | solver | numTaskFinished | numPlannerErrors/ScheduleErrors/EntryTimeouts | avg/max SchedulerSolveTime | RSS plateau |
|---|---|---|---|---|---|
| 2 | 6 | 1851 | 0/0/0 | 63ms / 520ms | 2.088 GB |
| 2 | 7 | 1856 | 0/0/0 | 224ms / 845ms | 2.091 GB |
| 4 | 6 | 1886 | 0/0/0 | -- | 2.107 GB |
| 4 | 7 | 1894 | 0/0/0 | 58ms / 599ms | 2.978 GB |

Both solvers complete cleanly at every level tested, no errors, comparable
(solver 7 slightly ahead each time, consistent with the smaller-map results
above, not a formal benchmark claim) task throughput. `SchedulerBackboneBuildTime`
was 0 on every single timestep across all these runs, confirming the eager
build in `schedule_initialize` is working -- no runtime rebuild cost is ever
paid mid-run.

Two things worth flagging, neither a correctness bug:

- **Solve time headroom shrinks at shallow levels on huge maps.** At level 2
  (the default), solver 7's max per-timestep solve time was 845ms against
  the scheduler's own ~980ms budget (`limit = time_limit/2 -
  SCHEDULER_TIMELIMIT_TOLERANCE`, `TaskScheduler::plan`) -- no timeouts
  occurred (`numEntryTimeouts=0`), but there's markedly less margin than
  solver 6's 520ms max at the same level. This tracks the backbone size:
  level 2's backbone has 20,939 nodes/53,112 arcs on `orz900d` (from the
  validator run above), all copied via `lemon::digraphCopy` every timestep,
  plus one proxy node and up to 4 arcs per surplus agent/task -- real,
  structural extra work solver 6 doesn't do. At level 4 the backbone shrinks
  to 2,098 nodes/4,928 arcs and solve time drops to 58ms/599ms, comfortably
  matching solver 6's own headroom. On an even bigger map than `orz900d`
  (e.g. `IH_mp_2p_01` at ~3.44M cells, or `scene_mp_4p_03` at ~13.9M),
  solver 7 at a shallow `--flowSolveLevel` could plausibly start timing out
  where solver 6 wouldn't -- worth checking directly before trusting solver
  7 at level 1-2 on anything bigger than `orz900d`, and a candidate reason
  to prefer deeper `--flowSolveLevel` values with solver 7 specifically.

- **RSS plateau is ~870MB higher for solver 7 at level 4, but flat/non-leaking
  at every level tested, and only material at level 4, not level 2.**
  Confirmed via the same per-2-second RSS sampling used to verify the
  original solver-6 OOM fixes (`ai/claude_memleak_fixes.md`'s methodology):
  after an initial ramp during map/hierarchy loading, RSS is dead flat for
  the entire 250-timestep run at every level -- no growth trend, so this is
  **not** a new leak. At level 2, solo-run RSS is within 3MB between the two
  solvers (an earlier *parallel* run of both solvers together showed a
  ~220MB gap at level 2 that vanished when re-measured solo -- almost
  certainly resource contention between the two concurrent processes, not a
  real difference; solo measurement is the trustworthy number). At level 4,
  the ~870MB gap is real and reproducible solo. Likely explanation, not yet
  directly instrumented to prove: `global_heuristictable`
  (`default_planner/heuristics.cpp`), the full-map BFS distance-table cache
  shared by *every* solver, is already explicitly capped at ~1GB resident
  via its own pre-existing LRU eviction (`ai/claude_memleak_fixes.md`, fix
  #5) -- roughly `max(16, 1GB / ~3.9MB-per-table)` ≈ 256 tables on this map.
  Solver 7's position-aware costing routes some agents differently than
  solver 6 at level 4 (match counts are nearly identical -- 2551 vs 2552 --
  but *which* agent reaches *which* task, and via what path, can differ),
  which could mean more distinct goal locations get touched over the run,
  pushing this shared, already-bounded cache closer to its cap. This is
  speculative pending direct instrumentation (a table-count metric would
  confirm it outright) but is consistent with every observation: bounded by
  a pre-existing ~1GB cap regardless of solver, absent at level 2 (where the
  two solvers' assignment decisions are closer to identical), and flat over
  time rather than growing. Flagged here rather than asserted as proven --
  if this needs to be pinned down exactly (e.g. before a benchmark sweep
  that reports memory), add a resident-table-count field to `heuristics.cpp`
  next to the existing LRU bookkeeping and rerun this same comparison.

## Motivation

Solver 6's coarse flow graph (`ReducedHierarchy::compute_reduced_assignment`,
`MapCoarsenV1.cpp:1584`, see `ai/project_context.md`'s "Solver 6" section)
connects every surplus agent to its top-level coarse node with a **cost-0**
arc (`src_arc_by_top`, `MapCoarsenV1.cpp:1783`) and every surplus task to its
node the same way (`sink_arc_by_top`, `:1797`). Cost only enters once flow
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
`MapCoarsenV1.cpp:1817` today. Fixing that pre-existing looseness is out of
scope for this change — solver 7 should differ from solver 6 only in the way
described here, not pick up an unrelated capacity-modeling fix as a side
effect.

### Agent/task proxy nodes

Solver 6's Step 1 connects the source to each occupied top node with **one**
arc whose capacity is the count of surplus agents there
(`src_arc_by_top`/`sink_arc_by_top`, `MapCoarsenV1.cpp:1770-1800`) — cost 0,
because all agents at a node were treated as interchangeable. Solver 7 can no
longer do that, because the whole point is to give agents at the same region
node *different* costs depending on where in that region they actually are.
So instead, **after** the local-node-matching pass removes same-node
agent/task pairs (identical to solver 6's Step 1, `MapCoarsenV1.cpp:1627-1835`
— unchanged), for every remaining surplus agent/task:

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
and cached (see "Structural design: solver-side composition" below) — never
recomputed per agent or per timestep.

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

## Structural design: solver-side composition

`ReducedHierarchy` (`MapCoarsenV1.h:227`) does **not** grow a second
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

std::unique_ptr<EdgeAugmentedTopGraph> build_edge_augmented_top_graph(
    const MultiLevelCoarsenedGraph& hierarchy, const CoarsenedGraph& top);
// `hierarchy` is needed (not just `top`) so region bounding boxes can be
// folded up from every level 0..top.level_idx, not just the top level itself.

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

### The two places this reaches into `MapCoarsenV1.{h,cpp}`

**Found only while actually writing `EdgeAugmentedCoarsen.cpp`, not
anticipated in the design pass above**: `ReducedHierarchy`'s only public
members were `ensure`, `ready`, `hierarchy_build_time`,
`hierarchy_level_node_counts`, and `compute_reduced_assignment` — there was
no way to read the underlying `MultiLevelCoarsenedGraph` at all. Building the
backbone (needs `hierarchy.level(top_level_idx)`) and computing region
bounding boxes (needs every level from 0 up to `top_level_idx`, via
`to_coarser_node_id`) both require it. Added one small read-only getter:

```cpp
const MultiLevelCoarsenedGraph& hierarchy() const { return hierarchy_; }
```

Trivial and low-risk (a const reference accessor, no behavior change to
anything), but it's a second touch to `ReducedHierarchy`'s public surface
beyond the `lift_coarse_paths_to_fine` one that was actually discussed and
agreed on beforehand — flagged here rather than silently folded in, per "log
hiccups."

Separately, and as anticipated: Steps 1, 3, and 4 all depend on five helper
functions defined `static` (file-internal linkage) in `MapCoarsenV1.cpp`:
`is_valid_graph_node_id_local`, `map_fine_node_to_level_node_local`,
`shortest_path_in_graph_local`, `path_cost_on_fine_graph_local`,
`expand_path_batch_one_level_local`. A separate translation unit cannot call
them as they stand, so **some** change to V1 is unavoidable — decided as
follows:

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
                                  double* expand_time_out,   // Step 3 (expand+fallback) time
                                  double* guide_time_out,    // Step 4 (packaging) time
                                  double* guide_path_length_sum_out,
                                  double* guide_path_cost_sum_out);
  ```

  (Split into two timing out-params, not one, so `compute_reduced_assignment`
  can still report its historical `solve_time_out`/`guide_time_out` split --
  Step 3 counted in `solve_time`, Step 4 in `guide_time` -- unchanged after
  switching to call this method internally; see the call site's own comment
  in `MapCoarsenV1.cpp` for exactly how the two are recombined.)

  `compute_reduced_assignment`'s own Steps 3/4 (previously inline at roughly
  where `lift_coarse_paths_to_fine` -- `MapCoarsenV1.cpp:1450` -- now sits,
  just above it in the file; that inline code no longer exists post-refactor,
  it's what got extracted) are refactored to call this same method — a
  behavior-preserving extract,
  not new logic, so solver 6 is unaffected functionally but does get touched
  mechanically (worth a rebuild + rerun on a small instance to confirm
  identical output before moving on, same verification standard the past
  cleanup passes in this file have used).

### New metric: backbone build/copy time

Solver 7 has a cost category solver 6 doesn't: building the persistent
backbone (first call only) and copying it into a fresh graph (every call).
Tracked from the start, mirroring how `local_match_time` was added
(`ai/local_node_matching.md`):

- `ScheduleTiming` (`default_planner/scheduler.h:18-49`) gains
  `double backbone_build_time = 0.0;`.
- `set_last_reduced_timing(...)` (`scheduler.h:59-68`) gains one more
  trailing optional parameter, `double backbone_build_time = 0.0` — default
  keeps solver 6's existing call site (`scheduler.cpp:1104`) unchanged.
- `TimeStepMetric` (`inc/CompetitionSystem.h:15-47`) gains
  `double SchedulerBackboneBuildTime = 0.0;`, wired through both population
  sites in `src/CompetitionSystem.cpp` (mirroring `SchedulerLocalMatchTime`
  at `:270`/`:302`) and the JSON output block (mirroring `:411`).

## Implementation plan (done — see "Implementation status" above)

Following the sibling-module precedent already in this repo
(`mapReductionV0.*` kept alongside `MapCoarsenV1.*`):

- **New**: `map_reduction_test/EdgeAugmentedCoarsen.h` / `.cpp` — everything
  in "Structural design: solver-side composition" above: `EdgeAugmentedTopGraph`,
  `build_edge_augmented_top_graph` (backbone + bbox precompute),
  `EdgeAugmentedHierarchy`, `compute_reduced_assignment_edge_augmented`.
  Step 1 (bucketing + local matching, up to the point the flow graph is
  built) is copied from `MapCoarsenV1.cpp:1627-1835` — this part genuinely
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
  `schedule_plan_flow_reduced` (scheduler.h:81), calling
  `EdgeAugmentedHierarchy::instance().compute_reduced_assignment_edge_augmented(...)`.
  Also extend `schedule_initialize`'s hierarchy-build gate
  (`scheduler.cpp:106`) from `solver == 6` to `solver == 6 || solver == 7`
  (same underlying hierarchy, so the same `ensure()` call covers both), and
  **additionally** (caught while implementing, not in the original plan
  above) call `EdgeAugmentedHierarchy::instance().ensure(env, env->flow_solve_level)`
  there too when `solver == 7` -- `env->flow_solve_level` is already set from
  `--flowSolveLevel` by this point in `driver.cpp`'s sequence, so there's no
  reason to defer the backbone build to the first per-timestep call. Building
  it lazily instead would land that one-time cost on the first timestep's
  much tighter `--planTimeLimit` budget (default 1000ms) rather than
  preprocessing's `--preprocessTimeLimit` one (default 30000ms) -- exactly
  the problem solver 6's own hierarchy build was already moved into
  `schedule_initialize` to avoid (see `ai/project_context.md`'s "Solver 6"
  section). Add the `ScheduleTiming`/`set_last_reduced_timing` changes from
  "New metric" above.
- **Edit**: `src/TaskScheduler.cpp` — add an `else if (solver == 7)` branch
  (`:67`, right after the existing `solver == 6` branch at `:62`) dispatching
  to `schedule_plan_flow_reduced_edge`, and update the fallback error's
  `"1..6"` message (`:74`) to `"1..7"`.
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

## Hiccups encountered during implementation

- **`ReducedHierarchy` needed a second small public addition beyond
  `lift_coarse_paths_to_fine`** — the `hierarchy()` read-only getter. See
  "The two places this reaches into `MapCoarsenV1.{h,cpp}`" above. Not a
  correctness bug, just an underestimate in the design pass of how much of
  `ReducedHierarchy`'s surface solver 7 would need opened up.

- **`compute_reduced_assignment_edge_augmented`'s backbone capacity can't
  reuse solver 6's exact `num_workers`-based value.** Solver 6 sets a coarse
  arc's capacity to that timestep's total surplus-agent count at graph-build
  time, every timestep, since it rebuilds the whole graph from scratch each
  call. Solver 7's *backbone* (region+edge nodes) is built once and reused
  via `digraphCopy` across many timesteps, so it can't bake in one specific
  timestep's count. Resolved by using a fixed upper bound (total fine-map
  node count, which no per-timestep surplus-agent count can ever exceed) —
  preserves the same "effectively unconstrained" intent without needing
  per-timestep capacity rewrites. Documented inline in
  `build_edge_augmented_top_graph`.

- **Real bug, caught by testing, not by inspection: `ns.flowMap(flow)` was
  called before `ns.run()`, not after.** First functional test on `tiny`
  (50 timesteps) produced `totalFlowMatchCount=0` the whole run (vs. solver
  6's `5` on the identical instance/timesteps) — every agent's flow-graph
  BFS silently found no path to any task, even though `ns.run()` reported
  `OPTIMAL`. Root cause: LEMON's `NetworkSimplex::flowMap()` documents "pre:
  `run()` must be called before using this function" — unlike `costMap()`/
  `upperMap()`/`supplyMap()`, which are pre-solve setup calls, `flowMap()` is
  a post-solve *output* accessor that copies the solved internal flow state
  into the given map at the moment it's called. Called before `run()`, it
  copies the (all-zero) pre-solve state, and `ns_status` gives no indication
  anything is wrong since the solve itself is unaffected by when a caller
  chooses to read `flowMap()`.

  **Solver 6 has this exact same call-ordering** (`ns.flowMap(flow);` before
  `ns.run();`, `MapCoarsenV1.cpp:1826/1829`) but it's latent/harmless there:
  solver 6's own Step 2 never actually reads the `flow` ArcMap it populates
  -- it reads `ns.flow(arc)` (the `NetworkSimplex` object's own live
  accessor, valid any time after `run()`, entirely independent of whether/
  when `flowMap()` was ever called) instead. So the bug has been sitting
  unexercised in solver 6's own code, presumably since whichever earlier
  revision switched Step 2 from reading the `flow` map to reading
  `ns.flow()` directly without removing the now-pointless `flowMap()` call.
  Not fixed in `MapCoarsenV1.cpp` as part of this work (truly inert there,
  and out of scope for an additive change) but worth knowing about if anyone
  ever refactors solver 6's Step 2 to use the `flow` map directly again.
  Fixed in `EdgeAugmentedCoarsen.cpp` by moving the `flowMap()` call to
  immediately after the `OPTIMAL` check, with a comment explaining why.

## Open items for a later pass (not blocking first implementation)

- Whether the boundary-cell-subset refinement (see cost-formula table above)
  is worth adding, once solver 7 has real benchmark numbers to compare
  against solver 6 (same pattern as `ai/local_node_matching.md`'s
  quantify-then-decide approach).
- Whether the region↔edge backbone's capacity should eventually be derived
  from something real (e.g. count of fine boundary crossings) instead of
  mirroring solver 6's unconstrained `num_workers` placeholder — flagged as
  out of scope above, revisit only if solver 7's results motivate it.
- Pin down the ~870MB level-4 RSS plateau gap found during `orz900d` scale
  testing with direct instrumentation (a resident-table-count metric on
  `global_heuristictable`) instead of the current plausible-but-unproven
  explanation. See "Structural + scale validation" above.
- Check solve-time headroom directly (not just extrapolated) on a map bigger
  than `orz900d` at shallow `--flowSolveLevel` values (1-2) before trusting
  solver 7 there — `IH_mp_2p_01` (~3.44M cells) or `scene_mp_4p_03` (~13.9M)
  are the natural next maps, matching the existing solver-1-vs-6 sweep
  progression in `ai/auto_benchmarking.md`. See "Structural + scale
  validation" above.
- A formal solver-6-vs-7 benchmark sweep (throughput/makespan across agent
  counts and `--flowSolveLevel` values), matching the methodology already
  used for solver 1 vs. 6 in `ai/auto_benchmarking*.md` — everything so far
  has been correctness/scale verification, not a benchmark claim.
