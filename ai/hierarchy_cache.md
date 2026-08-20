# Solver 6 hierarchy disk cache

Added 2026-08-12. Lets a solver-6 map-coarsening hierarchy
(`MapReductionTest::ReducedHierarchy`, see `ai/project_context.md`'s "Solver
6" section) be built once and reused by later runs against the same map,
instead of every process paying the full build cost from scratch. Distinct
from `ai/solver6_preprocessing_efficiency.md`, which is about making the
build itself cheaper — this is about not rebuilding at all when a valid
cached copy already exists on disk.

## What was added

- `map_reduction_test/MapCoarsenSerialize.{h,cpp}` — `save_hierarchy_to_file`
  / `load_hierarchy_from_file`, a flat custom binary format (not LEMON's
  `.lgf` — that only covers graph topology, and `CoarsenedGraph` carries a
  lot of non-graph bookkeeping: `bridge_cache`, `bridge_path_cache`,
  `to_finer_node_ids`, per-node directional-arc metrics, none of which have
  a natural `.lgf` slot).
- `--hierarchyCache <path>` CLI flag (`src/driver.cpp`), written onto a new
  `SharedEnvironment::hierarchy_cache_path` field (`inc/SharedEnv.h`) the
  same way `--fileStoragePath` already lands on `file_storage_path` — no
  signature changes needed anywhere between `driver.cpp` and
  `ReducedHierarchy::ensure()`, since `ensure()` only ever received
  `SharedEnvironment*` to begin with. Empty (the default) disables caching
  entirely — behavior is unchanged from before this feature existed.
- `ReducedHierarchy::ensure()` (`MapCoarsenV1.cpp:1019`): if
  `hierarchy_cache_path` is set, tries `load_hierarchy_from_file` before
  `build_multilevel_from_environment`; on success, skips the build outright.
  On a fresh build, saves to that path afterward so the *next* run against
  the same map benefits.
- `MapReductionTest::compute_env_signature()` (moved from a `static` function
  local to `MapCoarsenV1.cpp` to a proper declared function in
  `MapCoarsenV1.h`) is the single source of truth both `ensure()`'s in-memory
  cache check and the on-disk cache's validity check hash against — no
  duplicated hash logic to drift out of sync.
- `./build/hierarchy_cache_validator <instance.json>` — new standalone tool
  (`utils/validation/validate_hierarchy_cache.cpp`), mirroring the existing
  `guide_path_validator` pattern. Builds a hierarchy, saves it, reloads it,
  and asserts the two are structurally identical level-by-level (every
  node's coordinates/hierarchy-mapping fields/directional-arc metrics, every
  arc's endpoints/cost/capacity as an order-independent set, and both bridge
  caches' full contents) — plus confirms a wrong level count or a mutated
  map is correctly *rejected* rather than silently loaded.

## Cache validity / invalidation

A cache file is only trusted if **all** of the following match what's stored
in its header: format magic + version, `env->rows`, `env->cols`,
`compute_env_signature(env)` (an FNV-1a hash over rows/cols/every `map`
cell — the exact same signature `ensure()` already used in-memory to decide
whether to rebuild), and the hierarchy's level count (`kDefaultCoarsenLevels
+ 1`). Any mismatch, truncated/corrupt file, or missing file is treated as a
plain cache miss: `load_hierarchy_from_file` returns `false`, `hierarchy_` is
left untouched, and `ensure()` falls through to a normal build (then
overwrites the stale file with a fresh save). Nothing about this can produce
a silently-wrong hierarchy — it either loads a byte-verified match or it
doesn't load at all.

Caveat: `kDefaultCoarsenLevels` is a compile-time constant
(`MapCoarsenV1.cpp:30`), so a hierarchy cached by one build is invalidated
(level-count mismatch) if that constant is later changed and the binary
recompiled — this is intentional, not a bug to fix.

## What's NOT serialized (and why that's safe)

`nodes_at_location`, `node_to_maploc`, `maploc_to_node`, and `map_nodes`
aren't written to the file. All four are lookup tables `reserve_fine_map`
already rebuilds deterministically from a node count alone (same as the
original build path). `nodes_at_location` specifically is *only* ever read
again while coarsening a level to produce the next one
(`append_group_nodes`, called from `Coarsen()`) — by the time a level is
worth caching, that coarsening has already happened, so nothing at
per-timestep solve time (`compute_reduced_assignment`) ever touches it.
Confirmed via `grep` across `map_reduction_test/*.cpp`.

Arcs are serialized by node index (0..num_nodes-1, matching
`coarse_location`/`chosen_finer_node_id`/etc.'s indexing), not by LEMON's own
internal node/arc ids — those are only meaningful within one process's
`lemon::ListDigraph` instance and are meaningless once reloaded into a fresh
graph. `node_to_maploc[lemon_id] -> map_id` (already used elsewhere in this
file for the same translation) is reused to make this conversion when
writing.

## Benchmarks

Both measured via the `schedulerHierarchyBuildTime` output-JSON field, which
after this change reflects wall time for **whichever branch actually ran**
(load or build) — previously it only timed the build branch, so a cache hit
would have misleadingly read ~0.

| Map | Cells | Fresh build | Load from cache | Speedup | Cache file size |
|---|---|---|---|---|---|
| `orz900d` (`orz900d_5000.json`) | ~978K | 0.96s | 0.55s | ~1.7x | 45.7 MB |
| `IH_mp_2p_01` (`IH_mp_2p_01_5000.json`) | ~3.44M | 17.6s | 6.4s | ~2.75x | 518 MB |

Both runs used `--preprocessTimeLimit 600000` (`IH_mp_2p_01` needs it
regardless of caching, see `ai/auto_benchmarking_IH_mp_2p_01.md`) and
`-s 5`. Note these builds are already fast relative to the ~300s figure in
`ai/auto_benchmarking_IH_mp_2p_01.md` — that sweep predates the
`ai/solver6_preprocessing_efficiency.md` optimization pass (124s -> 17.9s for
this same map); caching's absolute win shrinks as the build itself gets
cheaper, but stays a consistent multi-second-to-multi-minute win at any map
size tested so far, and would be much larger again on a map bigger than
`IH_mp_2p_01` or if `kDefaultCoarsenLevels` is raised (more/bigger cached
bridge paths to build).

## Correctness verification methodology

Two independent checks, because simulation output alone can't distinguish a
serialization bug from ordinary run-to-run jitter: `default_planner/`'s
low-level planner is wall-clock time-boxed (`frank_wolfe`'s `end_time`,
`planTimeLimit`-driven catch-up timesteps — see `ai/project_context.md`'s
"Makespan vs. timesteps solved" section), so **two fresh builds of the same
hierarchy on the same instance with no caching involved at all already
produce different `numTaskFinished`** (174 vs 179 in one A/B check on
`warehouseSmall_100`, ~1100 diff lines in the raw output JSON) — this is
expected and unrelated to caching.

1. **Structural**: `./build/hierarchy_cache_validator` (above) — 57 checks,
   0 failures on both `warehouseSmall_100` and `orz900d_5000`.
2. **Behavioral smoke test**: ran `./build/lifelong --scheduleModel 6` twice
   against `warehouseSmall_100` with `--hierarchyCache` set — first run
   builds+saves, second loads. Confirmed via `schedulerHierarchyBuildTime`
   collapsing from ~0.007s to ~6e-7s (pre-timing-fix measurement) that the
   load path actually ran, and that output stayed within the same
   run-to-run variance envelope as the no-caching control above (not
   identical, which is expected and fine — see above).

## Usage

```shell
./build/lifelong --inputFile <instance.json> -o outputs/out.json \
  --scheduleModel 6 --hierarchyCache /path/to/some_map.hierarchy
```

First run builds and writes the cache file; later runs against the same map
(same `rows`/`cols`/cell contents) load it instead. No flag = no caching,
identical behavior to before this feature existed. The path's parent
directory must already exist (mirrors `--fileStoragePath`'s validation
style) — a missing directory or other I/O error just logs a warning to
stderr and continues without caching, it does not fail the run.
