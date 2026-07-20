# Solver 6 (reduced-hierarchy scheduler) OOM investigation and fixes

This document records the investigation into why `--scheduleModel 6`
(`schedule_plan_flow_reduced` / `MapReductionTest::ReducedHierarchy`) was
running out of memory on large instances while `--scheduleModel 1`
(`schedule_plan_flow`) was not, and the fixes that were applied. Five
distinct bugs were found and fixed across two rounds of investigation; the
first round explained a moderate slowdown/growth on `warehouseLarge`, the
second explained a much more severe, fast OOM on `orz900d_5000`.

All fixes live in:
- `map_reduction_test/MapCoarsenV1.h` / `.cpp`
- `default_planner/scheduler.cpp`
- `default_planner/heuristics.h` / `.cpp`
- `default_planner/planner.cpp`

## Background

Solver 6 builds a persistent, one-time multi-level "coarsened" version of the
map (`ReducedHierarchy`, built by `Coarsen()` in `MapCoarsenV1.cpp`): level 0
is the full fine-grained map, and each subsequent level groups 2x2 blocks of
the previous level's nodes into connected components, roughly halving the
grid dimensions each time (`kDefaultCoarsenLevels = 2`, so 3 levels total for
this repo's config). Every timestep, `compute_reduced_assignment()`:

1. Maps free agents/tasks up to the coarsest ("top") level and solves one
   small min-cost flow there (LEMON `NetworkSimplex`).
2. Decomposes that flow into one coarse-level path per agent.
3. "Lifts" each coarse path back down through the levels to the fine map,
   splicing in cached bridge paths between neighboring coarse components.
4. Converts the lifted paths into per-agent guide paths for the low-level
   PIBT/traffic-flow planner.

This lifting/guide-path machinery (steps 3-4) is the part whose cost scales
with the size of the *fine* map and the number of agents, rather than the
(tiny) coarse top-level graph, and is where most of the bugs below live.

## Round 1: OOM on `warehouseLarge_4000` (500x140 map, 4000 agents)

### Symptom

Running solver 6 for a while showed RSS growing roughly 35% faster per
timestep than solver 1 on the same instance (≈2.14 MB/timestep vs
≈1.57 MB/timestep), which would eventually OOM on a long enough run.

### Root cause: bridge-path lifting endpoint mismatch → runaway fallback search

`Coarsen()` precomputes, for every pair of adjacent coarse components, a
"bridge path" between each component's arbitrarily-chosen **representative**
fine node (`chosen_finer_node_id`) — this is cached in `bridge_path_cache` so
runtime lifting is just a lookup, not a fresh search.

The bug was in `expand_path_batch_one_level_local()` (`MapCoarsenV1.cpp`),
used for the final lift from the coarsest-but-one level down to the real fine
map. On that final lift, the path's start/end nodes are the agent's *actual*
current location and the task's *actual* location — not the coarse
component's representative node. The code checked for **exact equality**
between the cached bridge segment's endpoint and the real agent/task
location:

```cpp
const std::vector<int>& segment = cached_path_it->second.path;
if (segment.front() != current || segment.back() != target)
{
    failed = true;
    break;
}
```

Since the real agent/task location is essentially never exactly the
arbitrarily-chosen representative node of its component, this check failed
for almost every multi-hop lift, and the caller fell back to a full
Dijkstra search over the *entire* fine map (`shortest_path_in_graph_local`,
capped at 20,000 node expansions) for that agent. Instrumentation showed
this fallback firing for **~1200-1700 agents per timestep** (roughly 40% of
all agents) on `warehouseLarge_4000` — a huge number of large, short-lived
`unordered_map`-based searches every timestep, causing heap churn/
fragmentation far beyond what solver 1 ever does.

### Fix

Instead of failing outright on a mismatch, splice in a short **intra-component**
hop (bounded, since coarse components are provably tiny — at most 4 fine
cells per 2x2-block level) using the existing constrained BFS helper
(`shortest_path_in_graph_local(..., constrain_parent=true)`):

```cpp
std::vector<int> lead_in;
if (segment.front() != current)
{
    // must be in the same coarse parent as this hop's source, or bail
    lead_in = shortest_path_in_graph_local(lower, current, segment.front(), parent_a, true);
    ...
}
// ... splice lead_in + segment + lead_out into path_buffer ...
```

Same idea for the tail end (`lead_out`) when `segment.back() != target`.

### Verified

- Fallback-Dijkstra firings on `warehouseLarge_4000`: **1200-1700/timestep → 0** over 51 timesteps.
- RSS growth rate normalized from **≈2.14 MB/timestep → ≈1.6 MB/timestep**, matching solver 1's own baseline (≈1.57 MB/timestep).

## Round 2: fast OOM on `orz900d_5000` (656x1491 map, ~978K cells, 5000 agents)

The round-1 fix did not resolve this much larger, faster crash (RSS hit
~15GB and got OOM-killed within ~15-20 timesteps). This needed a much deeper
dive, isolating three more independent problems by adding (and then
removing) targeted RSS/counter instrumentation at each candidate site.

### Bug 2: unbounded flow-walk in guide-path reconstruction (Step 4)

`compute_reduced_assignment`'s Step 4 used to re-derive each agent's guide
path by aggregating *every* agent's lifted path into a single shared
per-arc flow-count map, then "following positive residual flow" node by
node from the agent's start to its task — mirroring how `schedule_plan_flow`
(solver 1) has to recover a path from a raw NetworkSimplex flow, since it
never assembles one directly.

```cpp
while (current != task_loc)
{
    for (arc : outgoing arcs of current)
        if (fine_arc_flow_counts[arc] > 0) { current = target(arc); --fine_arc_flow_counts[arc]; break; }
    if (!advanced) break;
}
```

This had **no cycle guard and no length cap**. Because the arc-count map
merges flow contributions from *every* agent's path, a walk could wander
across arcs that "belong" to other agents' routes through a large shared
flow budget before stalling or reaching the goal, producing pathologically
long (sometimes multi-million-node) `std::list<int>` guide paths.

**Fix**: Step 3 already produces a concrete, verified, bounded (≤5000 nodes)
start→task path per agent in `current_paths[i]` (built directly, or via the
existing whole-map Dijkstra fallback). Step 4 now just hands that path to the
caller directly instead of re-deriving it:

```cpp
for (i : agents)
{
    std::list<int> guide(current_paths[i].begin(), current_paths[i].end());
    out_agent_guide_paths[agent_id] = std::move(guide);
}
```

The now-dead `build_arc_flow_counts_local()` helper was removed.

### Bug 3: `build_cached_bridge_path_local` allocating full-map-sized vectors

During the *one-time* hierarchy build, `Coarsen()` calls
`build_cached_bridge_path_local()` once per entry in `bridge_cache` (tens of
thousands of entries for a map this size) to precompute the exact fine-level
walk between each pair of adjacent coarse components' representative nodes.
That function allocated:

```cpp
const int n = static_cast<int>(graph.map_nodes.size()); // ~978,096 for the level0->1 transition!
std::vector<int> prev(n, -1);
std::vector<char> seen(n, 0);
```

— two vectors sized to the **entire finer graph**, even though the BFS is
always confined to two tiny (≤4-node) coarse components via the
`v_parent != parent_a && v_parent != parent_b` filter. With tens of
thousands of calls each allocating and zero-initializing ~4.9MB of state for
what is logically a handful-of-nodes search, this made one-time hierarchy
construction itself extremely expensive in both time and (transient, but
very large) memory.

**Fix**: replaced the `vector<int>`/`vector<char>` pair with
`std::unordered_map<int,int>` (prev) and `std::unordered_set<int>` (seen),
bounded by the actual (tiny) number of nodes visited rather than the whole
graph:

```cpp
std::unordered_map<int, int> prev;
std::unordered_set<int> seen;
```

**Verified**: hierarchy build on `orz900d_5000` went from ballooning to
10+GB (and taking ~10.5s) down to a clean, small profile:
`792MB → 1067MB (fine level) → 1118MB (level 1) → 1118MB (level 2, no
change) → 1118MB (final)` — about 330MB total, one-time, as expected.

### Bug 4: guide-path lifting computed even when nobody will read it

The caller (`schedule_plan_flow_reduced` in `scheduler.cpp`) only ever reads
the computed guide paths when `use_traffic && env->curr_timestep >= 100`:

```cpp
if (use_traffic && env->curr_timestep >= 100)
    agent_guide_path[agent_id] = guide_paths[agent_id];
```

But `compute_reduced_assignment` unconditionally ran the full Step 3/4
lifting pipeline (the only part of the function whose cost scales with the
fine map size and per-agent path length) every timestep regardless — pure
wasted work whenever traffic-aware guiding is off or not yet active.

**Fix**: added a `need_guide_paths` parameter to `compute_reduced_assignment`
(`MapCoarsenV1.h`/`.cpp`); when false, the function solves the (tiny) top-level
flow and returns the agent→task `assignments` immediately after Step 2,
skipping Step 3/4 entirely:

```cpp
if (!need_guide_paths)
{
    // ...timing bookkeeping...
    return assignments;
}
```

Wired up at the call site: `need_guide_paths = use_traffic && env->curr_timestep >= 100;`.

This turned out not to be the dominant factor in the `orz900d_5000` crash
(see Bug 5), but it's a real, unconditional performance/memory win any time
traffic-aware guiding is off — which is the common case in the reproductions
used throughout this investigation (none passed `--useTraffic`).

### Bug 5 (the actual root cause): unbounded `global_heuristictable` cache

After fixing bugs 2-4, `orz900d_5000` **still** crashed the same way — RSS
climbing ~2-2.3GB per timestep and OOMing by timestep ~15-20. Bisecting with
RSS checkpoints at the start/end of the scheduler call and the start of the
low-level planner call showed the growth happened *inside the low-level
planner*, not the scheduler — and a counter on `get_h()` (the "normal" heuristic
lookup path) showed **zero** new heuristic tables being built, which
seemed to rule out heuristics entirely at first.

The real mechanism was a **second, separate call site** that bypasses
`get_h()`. `planner.cpp`'s main per-timestep loop directly initializes a
heuristic table for every agent's current goal location if one doesn't
already exist:

```cpp
if (trajLNS.heuristics.at(goal_loc).empty()){
    init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
}
```

`trajLNS.heuristics` is a reference alias for the same global
`global_heuristictable` (`TrajLNS():heuristics(global_heuristictable)`), but
this call path was never counted by the earlier `get_h()` instrumentation.
`HeuristicTable::htable` is a full per-goal distance array
(`env->map.size()` ints — **~3.9MB** on this map), and **it is never evicted**:
once built for a given goal location, it stays resident for the life of the
process.

Direct instrumentation of this call site showed **~600 new ~3.9MB tables
being built every single timestep** (≈2.3GB/timestep — matching the observed
RSS growth rate almost exactly), for as long as agents kept getting routed to
previously-unseen goal locations, before naturally plateauing once most
distinct locations in the active task set had been touched at least once
(consistent with the observed "grows fast for ~7 timesteps, then flattens
around 15GB" curve, and with the crash happening before the plateau on this
particular instance).

Solver 1's own scheduler is dramatically slower at solving the full
978K-node network flow every timestep, and a side-by-side check showed it
never even reached this heuristic-building code path within its own internal
per-timestep time budget (`"compute initial stop until 0"` / `"planner
timeout"` fired immediately for solver 1's calls) — so it was never exposed
to this problem, not because its assignment was inherently "better", but
because it was comparatively starved of the time needed to trigger it. This
means the underlying `global_heuristictable` cache was never actually safe
against a scheduler that successfully (and quickly) routes many agents to
many distinct locations — it just hadn't been exercised that way before.

**Fix**: added a small LRU cache layer around the heuristic-table cache
(`heuristics.h`/`.cpp`) so resident memory is bounded by a fixed budget
(~1GB) regardless of how many distinct goal locations get requested over a
run:

```cpp
// heuristics.h
void touch_heuristic_lru(int goal_location, SharedEnvironment* env);
```

```cpp
// heuristics.cpp
namespace {
    std::list<int> lru_order;               // most-recently-used at front
    std::unordered_map<int, std::list<int>::iterator> lru_pos;
}

void touch_heuristic_lru(int goal_location, SharedEnvironment* env){
    // move goal_location to the front of lru_order (insert if new)...
    const size_t bytes_per_table = env->map.size() * sizeof(int);
    const size_t target_budget_bytes = 1ull * 1024 * 1024 * 1024; // ~1GB cap
    const size_t max_tables = std::max<size_t>(16, target_budget_bytes / std::max<size_t>(1, bytes_per_table));

    while (lru_order.size() > max_tables){
        int evict = lru_order.back();
        lru_order.pop_back();
        lru_pos.erase(evict);
        // free that table's htable/open, leaving it "empty" so it gets rebuilt if needed again
    }
}
```

Called right after *every* place that ensures a heuristic table exists for a
goal — both `get_h()` and the direct call site in `planner.cpp` — so both
access patterns keep the LRU order (and eviction) correct regardless of
which one actually built or reused the table.

### Additional (independent) stability improvement: pin already-assigned tasks

While investigating Bug 5, `schedule_plan_flow_reduced`'s candidate-building
loop was found to unconditionally re-offer *every* not-yet-picked-up task
(and its currently-assigned agent) back into the flow solver every timestep,
even when neither the agent nor task had actually changed state. Because the
top-level coarse flow is blind to fine-grained distance differences between
agents/tasks that map to the same coarse node, this caused near-arbitrary
reshuffling of already-valid agent↔task pairings between timesteps — each
reshuffle pointing some agent at a "new" goal location and directly feeding
Bug 5's cache growth.

**Fix**: in `schedule_plan_flow_reduced` (`scheduler.cpp`), a task that's
still assigned to an agent who hasn't started it yet is now pinned directly
(`proposed_schedule[agent] = task`) instead of being fed back into the
coarse flow solve for re-decision; only genuinely-unassigned tasks and
newly-freed agents go through the solver:

```cpp
else if (task.second.agent_assigned != -1)
{
    // keep this pairing fixed instead of re-deciding it every timestep
    proposed_schedule[task.second.agent_assigned] = task.first;
}
else
{
    flexible_task_ids.push_back(task.first);
    task_loc_ids[task.second.locations[0]].push_back(task.first);
}
```

This is scoped to solver 6 only (`schedule_plan_flow`/solver 1 was not
touched, since it wasn't reported broken and its own dynamics already tend
toward stable reassignment).

## Verification

`orz900d_5000` (978K cells, 5000 agents), `--scheduleModel 6`, no
`--useTraffic`, run to completion for 250 timesteps:

- **Before**: OOM-killed by the kernel around timestep 15-20, RSS peaking
  at ~15.7GB (confirmed via `dmesg`: `Out of memory: Killed process ...
  anon-rss:15713904kB`).
- **After**: completed all 250 timesteps. RSS stable at **~2.3GB** for the
  entire run (measured every 5s from timestep 6 through 249, essentially
  flat). Output: `numTaskFinished: 1889`, `numPlannerErrors: 0`,
  `numScheduleErrors: 0`.

Regression checks on `warehouseSmall_100` for both `--scheduleModel 1` and
`--scheduleModel 6` completed cleanly with `numPlannerErrors: 0` /
`numScheduleErrors: 0`.

## Files changed

| File | Change |
|---|---|
| `map_reduction_test/MapCoarsenV1.cpp` | Bridge-path lift endpoint splicing (lead-in/lead-out); removed unbounded flow-walk guide reconstruction in favor of reusing `current_paths`; `build_cached_bridge_path_local` uses bounded hash-map state instead of full-graph-sized vectors; `compute_reduced_assignment` gains early-return when guide paths aren't needed |
| `map_reduction_test/MapCoarsenV1.h` | `compute_reduced_assignment` signature gains `bool need_guide_paths = true` |
| `default_planner/scheduler.cpp` | `schedule_plan_flow_reduced` pins already-assigned-but-not-yet-opened tasks instead of re-offering them to the flow solver every timestep; passes `need_guide_paths` through |
| `default_planner/heuristics.h` / `.cpp` | New `touch_heuristic_lru()` bounding `global_heuristictable` to a fixed (~1GB) memory budget via LRU eviction |
| `default_planner/planner.cpp` | Calls `touch_heuristic_lru()` at its direct (previously untracked) heuristic-table access site |

## Remaining considerations (not fixed, noted for future work)

- The LRU cap (~1GB) is a fixed constant; it could instead be derived from
  an overall memory budget passed in from the CLI/config if more precise
  control is ever needed.
- Solver 1's own per-timestep cost on very large maps (solving a full
  ~978K-node/~4M-arc min-cost flow from scratch every timestep) is itself
  expensive enough to starve the low-level planner of its guide-path/
  heuristic-init time budget (`"planner timeout"` / `"compute initial stop
  until 0"` fired for every timestep in testing). This wasn't in scope here
  (solver 1 wasn't reported broken), but it means solver 1's assignment
  quality on huge maps may be worse than it looks, simply because the
  low-level planner rarely gets to do its congestion-aware guide-path
  optimization at all.
