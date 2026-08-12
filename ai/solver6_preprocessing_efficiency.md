# Solver 6 preprocessing efficiency — running list

Read `ai/project_context.md` first, specifically "Solver 6 —
`schedule_plan_flow_reduced`" and its "Data structures
(`map_reduction_test/MapCoarsenV1.h`)" subsection.

**Scope of this doc: the one-time hierarchy build only** —
`ReducedHierarchy::ensure()` → `build_multilevel_from_environment()` →
`build_from_environment()` (level 0) → `Coarsen()` (levels 1..N), all in
`map_reduction_test/MapCoarsenV1.cpp`. This is the part gated to run once
per distinct map (keyed by `compute_env_signature_local`, hashes
`rows`/`cols`/`map` only — agents/tasks never invalidate it). It does
**not** cover the per-timestep path (`compute_reduced_assignment`, called
from `schedule_plan_flow_reduced` every timestep) — that hasn't had the same
line-by-line pass yet and would be a separate doc if/when it does.

This is a **running list**: findings get added as they're found, and each
one's status line gets updated when it's actually fixed.

**As of this writing, findings #1-#5 are fixed in the working tree
(uncommitted — not yet asked to commit).** Finding #6 remains open/deferred.
See "Verification methodology and results" below for how each was checked.

## CPU-time findings

### 1. [HIGH — the dominant cost of hierarchy build, confirmed] Full-fine-map-sized scratch buffers allocated on every 2×2-block / per-component call

**Status: FIXED.** `collect_connected_components` and
`collect_internal_directional_arc_samples` now take caller-owned scratch
buffers (`in_group_scratch`/`visited_scratch`/`in_component_scratch`),
allocated once per `Coarsen()` call (i.e. once per level) instead of once
per block/component, with each function resetting only the indices it
touched before returning.

`Coarsen()`'s outer loop runs once per 2×2 coarse block —
`coarse_rows × coarse_cols` iterations, e.g. ≈860K for the level-0→1 pass on
a map the size of `IH_mp_2p_01` (`MapCoarsenV1.cpp:660-683` pre-fix). Two
helper functions called from inside that loop each allocated a
`std::vector<char>` sized to the **entire fine map** on every call, even
though the actual input is always tiny (at most the handful of fine cells
inside one 2×2 block, or one connected component within it):

- `collect_connected_components` (`MapCoarsenV1.cpp:298-299` pre-fix):
  `std::vector<char> in_group(graph.map_nodes.size(), false);` and a second
  `visited` vector of the same size. Called once per non-empty block.
- `collect_internal_directional_arc_samples` (`MapCoarsenV1.cpp:116`
  pre-fix): `std::vector<char> in_component(graph.map_nodes.size(), false);`.
  Called once per *discovered connected component* (`MapCoarsenV1.cpp:679`,
  so roughly the same call count as above, possibly more since one block
  can yield multiple components).

This was the exact anti-pattern already diagnosed and fixed once elsewhere
in this file, in a third function (`build_cached_bridge_path_local`) — its
own comment names the failure mode: *"A vector<int>/vector<char> pair sized
to that, allocated and zero-initialized once per [call] ... is what made
hierarchy construction on large maps take gigabytes and many seconds"*
(`MapCoarsenV1.cpp:193-202`, fixed there with bounded hash-map state). That
fix had never been applied to these two sibling functions, which had the
identical shape.

**Measured result** (see verification section): **7x wall-clock speedup on
`IH_mp_2p_01`** (124s → 17.9s, `dump_coarsening`, `num_levels=2`), 2.3x on
`orz900d` (2.69s → 1.18s). Hierarchy output confirmed byte-identical
(partition/nodes/edges CSVs) before vs. after.

### 2. [MEDIUM] `std::map` instead of `std::unordered_map` for the inter-component arc bucket

**Status: FIXED.** `inter_component_arc_samples` in `Coarsen()`
(`MapCoarsenV1.cpp:748`) is now
`std::unordered_map<std::pair<int,int>, std::vector<double>,
CoarsenedGraph::PairHash>` instead of `std::map<std::pair<int,int>,
std::vector<double>>`. It's touched once per crossing arc while scanning
**every arc in the entire finer graph** — ~13.77M arcs at the level-0→1
pass. The full scan itself is unavoidable (need to see every arc to find
which ones cross components), but `std::map` meant every bucket
lookup/insert was an `O(log n)` red-black-tree descent with pointer-chasing
instead of an `O(1)`-amortized hash lookup. `CoarsenedGraph::PairHash`
already existed (used for `bridge_cache`/`bridge_path_cache`), so this was a
one-line container-type swap.

**Known, confirmed-benign side effect**: this changes arc *insertion order*
into the coarser level's `ListDigraph` (hash-bucket order instead of sorted
`(src,dst)` order). Verified via `dump_coarsening`'s `edges.csv`: differs
from the pre-fix file byte-for-byte, but is **identical once both are
sorted** (same row count, same multiset of rows) — confirmed on `tiny`,
`orz900d`, and `IH_mp_2p_01`. This reordering can also shift which
equally-optimal solution LEMON's `NetworkSimplex` returns when there are
cost ties in the per-timestep top-level flow solve (downstream in
`compute_reduced_assignment`, not covered by this doc) — see the
end-to-end verification note below for why this is expected and harmless,
not a regression.

## Memory findings

### 3. [HIGH — also a confirmed CPU-time win] `internal_directional_arc_samples` persisted forever but never read back

**Status: FIXED.** The field is gone from `CoarsenedGraph`
(`MapCoarsenV1.h`); `InternalDirectionalArcSamples` remains only as a
transient type (`CompInfo::internal_directional_arc_samples` inside
`Coarsen()`, and the parameter to `populate_new_graph_for_component`, which
now only uses it to compute `internal_directional_arc_metrics` and no
longer also copies it into a permanent per-node array).

It was written once per node during `Coarsen()`
(`populate_new_graph_for_component`) and immediately reduced into
`internal_directional_arc_metrics` — but nothing anywhere in the codebase
ever read `graph.internal_directional_arc_samples[...]` back out
afterward (verified by grep across `map_reduction_test/` before removing
it). `collect_internal_directional_arc_samples` recomputes its input fresh
from raw arcs each time; it never consulted the stored container.

**Measured result**: peak RSS on `IH_mp_2p_01` (`dump_coarsening`) dropped
from 2.42GB to 1.97GB (~18%) with this fix alone (on top of #1). Hierarchy
output confirmed byte-identical before vs. after.

### 4. [MEDIUM] `supply`/`flow` maps kept on the persisted hierarchy but never used after construction

**Status: FIXED.** Both fields removed from `CoarsenedGraph`
(`MapCoarsenV1.h`), along with their constructor-initializer-list entries
and the two `graph.supply[...] = 0` writes in `build_from_environment`.
They were declared "kept because scheduler code expects similar maps," but
the real per-timestep solve in `compute_reduced_assignment` builds its own
fresh temporary `ListDigraph` and copies `cost` into it — it never touched
the persisted `supply`/`flow`. `supply` was written once at build time and
never read again; `flow` was never written or read at all.

**Measured result**: peak RSS on `IH_mp_2p_01` dropped from 1.97GB to
1.91GB (~62MB) with this fix alone (on top of #1+#3). Hierarchy output
confirmed byte-identical before vs. after.

### 5. [LOW-MEDIUM] `to_finer_node_ids` allocated at level 0 despite being structurally guaranteed empty there

**Status: FIXED.** `reserve_fine_map` gained an `is_fine_level` parameter
(default `false`); `build_from_environment` (level 0's builder) now passes
`true`, which skips the `to_finer_node_ids.assign(fine_map_size, ...)`
allocation entirely for level 0 (verified nothing reads
`to_finer_node_ids` unguarded for level 0 specifically — the one read
outside `Coarsen()`, in `dump_coarsening.cpp:143-144`, is bounds-checked).
Level 1+ builds (`Coarsen()`'s call to `reserve_fine_map`) still pass the
default `false`, so their `to_finer_node_ids` — which *is* populated and
read — is unaffected.

**Measured result**: peak RSS on `IH_mp_2p_01` dropped from 1.91GB to
1.78GB with this fix on top of #1+#2+#3+#4 (cumulative). Hierarchy output
confirmed content-identical before vs. after.

### 6. [Bigger effort, deferred] One LEMON node (+ all parallel per-node vectors) allocated per raw grid cell, including walls

**Status: found, deliberately deferred — bigger surgery than 1-5, not
started.**

`build_from_environment` sizes everything to `env->map.size()` (the full
`rows × cols` grid) rather than the walkable-cell count — every
wall/obstacle cell still gets a LEMON node and an entry in every parallel
per-node vector, even though it'll never get an arc. This inflates level-0
memory proportional to the map's wall fraction. On the maze-style maps this
repo mostly benchmarks on (`orz900d`, `IH_mp_2p_01`) the wall fraction is
apparently small, so the win here is probably modest for those specific
maps — but would matter more on a map with large solid blocks (e.g.
`warehouseXL`'s storage racks).

**Fix sketch**: stop treating `loc == graph_id` as an identity mapping;
build a walkable-cell-only id space and translate at the
environment/lookup boundary. Touches more call sites than 1-5 (`map_nodes[loc]`
is assumed to be a trivial index lookup in several places), so this is the
one item here that's a real refactor rather than a local, low-risk change.
Only worth doing if profiling on a wall-heavy map shows level-0
memory/build-time as the bottleneck.

## Verification methodology and results

Every fix (1, 3, 4, 2, 5, applied and verified in that order) went through
the same check before moving to the next one:

1. **Structural correctness**: `./build/dump_coarsening <instance> <prefix> 2`
   (calls `build_multilevel_from_environment` directly, the same function
   `ReducedHierarchy::ensure()` uses) on three instances of increasing size —
   `instances/custom/tiny/tiny.json`, `instances/custom/orz900d/orz900d_5000.json`
   (~978K cells), `instances/custom/IH_mp_2p_01/IH_mp_2p_01_5000.json`
   (~3.44M cells, the map the ~300s hierarchy-build figure came from). Diffed
   the four output CSVs (`*_partition_level{1,2}.csv`, `*_nodes.csv`,
   `*_edges.csv`) plus the printed per-level node counts against a baseline
   captured before any changes. Byte-identical for every fix except #2,
   where `edges.csv` differs only in row order (confirmed via sorted diff —
   same rows, different order, expected from the container-type swap).
2. **Timing/memory**: `/usr/bin/time -v` around each `dump_coarsening` run,
   tracked cumulatively.
3. **Full build**: `cmake --build build -j4` (all targets: `lifelong`,
   `map_reduction_test`, `guide_path_validator`, `dump_guide_paths`,
   `dump_coarsening`) after the final fix, clean.
4. **End-to-end pipeline check**: ran `./build/lifelong --scheduleModel 6`
   on `orz900d_5000` (200 timesteps) and `./build/guide_path_validator` on
   the same instance, before (via `git stash` on the two source files) and
   after all five fixes. `numPlannerErrors`/`numScheduleErrors` were 0 in
   both. `guide_path_validator`'s solver1-vs-solver6 cross-check passed with
   **0 failed checks** both before and after (127950-128062 checks,
   solver1's `GuidePathLengthSum` identical in both since solver 1 is
   untouched by this work).

   `makespan`/`numTaskFinished`/per-timestep `GuidePathLengthSum` differed
   slightly between the before/after runs. **This is not a regression**:
   running the exact same post-fix binary twice on the same input also
   produces different `numTaskFinished` (1805 vs. 1803) and different
   `timeStepMetrics` — `lifelong` is a real-time-budgeted simulator
   (`--planTimeLimit`, Frank-Wolfe time-boxing, visible "planner timeout"
   log lines already present in the baseline run) and is not
   run-to-run-deterministic by construction, independent of anything in this
   doc. Fix #2 additionally has a *known* mechanism for shifting results
   within the space of equally-optimal solutions (arc-order-dependent
   `NetworkSimplex` tie-breaking, see finding #2 above) — also not a
   correctness bug. The only claim actually being verified end-to-end here
   is "no crashes/errors and the solver1-vs-solver6 structural cross-check
   still passes," which held throughout; exact-timestep reproducibility was
   never a property of the live simulator to begin with, only of the
   deterministic `dump_coarsening` structural path, which *is* verified
   byte-for-byte.

## Suggested order of attack

Findings #1, #3, #4, #2, #5 are fixed (in that order, each verified before
moving to the next). Remaining:

1. Finding #6 (walkable-cell-only id space) — only worth doing if profiling
   on a wall-heavy map (e.g. `warehouseXL`) still shows level-0
   memory/build-time as the bottleneck after 1-5.
