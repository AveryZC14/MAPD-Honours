# Map-coarsening visualisation tooling

Read `ai/project_context.md` first, specifically "Solver 6 —
`schedule_plan_flow_reduced`" and its "Data structures
(`map_reduction_test/MapCoarsenV1.h`)" subsection — this doc builds tooling
on top of `MultiLevelCoarsenedGraph`/`CoarsenedGraph` to visualise the
coarsening itself (which fine cells get grouped into which coarse node at
each level, and the resulting coarse graph topology), as opposed to
`ai/guide_path_visualisation.md`'s tooling, which visualises solver 1 vs.
solver 6 *guide paths* computed on top of an already-built hierarchy.

## Tools built

### `./build/dump_coarsening` (`map_reduction_test/dump_coarsening.cpp`)

```
./build/dump_coarsening <instance.json> <output_prefix> [num_levels=2]
```

Loads the instance via `populate_env_from_instance` (same helper as
`run.cpp`/`dump_guide_paths.cpp`), builds a hierarchy via
`build_multilevel_from_environment(hierarchy, &env, num_levels)` (same
function `ReducedHierarchy::ensure` uses at solver runtime, just invoked
directly rather than through the singleton), then writes:

- `<prefix>_partition_level<L>.csv` (one per level, `L = 1..num_levels`) — a
  **raster**, one line per map row, comma-separated coarse-node id that each
  fine cell belongs to (`-1` for walls and for "dud" cells that never join
  any component — see `build_from_environment`). Deliberately a raster, not
  a long-format `row,col,id` table, so it loads straight into a 2D numpy
  array. Computed level-by-level via `to_coarser_node_id` chained up from
  level 0 (`O(num_levels * map_size)`, no per-cell tree-walk), so it stays
  fast even at `orz900d` scale (~978K cells, 2 levels: 2.3s total including
  hierarchy build).
- `<prefix>_nodes.csv` — `level,node_id,coarse_row,coarse_col,fine_row,
  fine_col,num_children` for every coarse node at levels `1..num_levels`
  (level 0 skipped — it's one row per fine cell, redundant with the
  walkable-cell mask the `.map` file already encodes). `fine_row`/`fine_col`
  is `chosen_finer_node_id` resolved all the way down to level 0, giving each
  coarse node a real position to plot at.
- `<prefix>_edges.csv` — `level,src_id,dst_id,src_row,src_col,dst_row,
  dst_col,cost` for the coarse graph's arcs at levels `1..num_levels`, with
  both endpoints already resolved to fine-map coordinates.

One subtlety worth noting for anyone extending this: `coarse_location` /
`chosen_finer_node_id` / `to_finer_node_ids` are indexed by **graph id**
(a.k.a. maploc id, `0..num_coarse_nodes-1`), not by LEMON's internal node
id — iterating `lemon::ListDigraph::ArcIt` and calling `graph.g.id(...)`
gives the LEMON id, which must be converted via `node_to_maploc[lemon_id]`
before indexing any of those arrays. `Coarsen()` does this conversion
internally when building coarse arcs; `dump_coarsening.cpp`'s `write_edges`
does the same conversion for the same reason.

### `map_reduction_test/visualisation/plot_coarsening.py`

```
python3 plot_coarsening.py <map_file> <output_prefix> -o out.png \
    [--levels 1,2] [--scale N] [--crop r0,c0,r1,c1] \
    [--no-nodes] [--no-edges] [--per-level-output-dir DIR]
```

Pure `numpy`+`PIL`, same style as `plot_guide_paths.py` (no `matplotlib`).
Renders one panel per level (level 0 = plain walls/free fine map, levels
`1..N` = that level's partition colored per-component with the coarse
graph's nodes/edges overlaid), composed left-to-right into one PNG.

- **Component coloring**: golden-angle hue stepping
  (`hue = (node_id * 0.61803...) % 1.0`) rather than a fixed categorical
  palette — component counts range from single digits (tiny test maps) to
  hundreds of thousands (`orz900d` level 1 has 26384 nodes), far beyond any
  palette with individually distinguishable entries. The goal is only that
  *locally adjacent* components look different, which golden-angle stepping
  gives for free since components discovered close together in the
  row-major coarsening scan get well-separated hues.
- **Node/edge overlay**: coarse nodes drawn as small white-filled,
  dark-outlined dots at their representative fine-map location; arcs as thin
  dark lines between endpoints, deduplicated (the coarse graph stores each
  neighbor pair as two directed arcs, both would otherwise draw the same
  segment twice).
- **`--crop r0,c0,r1,c1`**: full-map renders of large/maze-like maps (e.g.
  `orz900d`, 656x1491) are only meaningful at ~1px/cell, at which point
  individual components are indistinguishable noise — confirmed by actually
  rendering it (see "Runs done" below). `--crop` re-renders a sub-rectangle
  at a legible scale instead; this is the intended way to inspect
  coarsening detail on any map bigger than a few hundred cells per side.
  Partition arrays, node lists, and edge lists are all cropped/offset
  together so panels stay pixel-aligned across levels.
- All levels share the fine map's row/col dimensions (the partition CSVs are
  rasters over fine cells), so unlike `plot_guide_paths.py`'s per-agent
  panels there's no independent per-panel crop/scale — one `--scale`/
  `--crop` applies uniformly across the whole composed image.

## Runs done

- `instances/custom/tiny/tinyComplex.json` (8x8, 2 levels: 64 → 18 → 7
  nodes) — full-map render at auto-scale, legible at a glance, used to sanity
  check the partition/node/edge logic before scaling up.
- `instances/warehouseSmall/warehouseSmall_100.json` (33x57, 3 levels: 1881
  → 417 → 131 → 45 nodes) — full-map render; level 1 (2x2-block granularity)
  is visually dense/busy against the warehouse's narrow aisles, levels 2-3
  read cleanly. Confirms the tool works on a structured (non-maze) map, not
  just tiny hand-built ones.
- `instances/custom/orz900d/orz900d_5000.json` (656x1491, 2 levels: 978096 →
  26384 → 7661 nodes) — `dump_coarsening` runs in ~2.3s including hierarchy
  build. Full-map render at auto-scale (1px/cell) is honest but effectively
  unreadable — a maze map's corridors are only 1-4 cells wide, so component
  colors blend into gray noise at that density (same "looks like dots, not
  useful" pattern `ai/guide_path_visualisation.md` documents for dense
  guide-path whole-map overviews). `--crop` on a ~150x150 corridor region
  fixes this: individual 2x2-block components at level 1 and their level-2
  merges are both clearly visible.
- `instances/custom/warehouseXL/warehouseXL_5000.json` (1900x1800, ~3.42M
  cells, 4 levels: 3,420,000 → 713,888 → 213,746 → 53,550 → 13,447 nodes) —
  first run at `num_levels=4` (previous runs used the 2-level default). Full
  build+dump in one call, no rebuild needed (`num_levels` is a runtime
  `dump_coarsening` argument, not the compile-time
  `kDefaultCoarsenLevels` solver 6 itself uses — see
  `ai/auto_benchmarking_warehouseXL.md` for the separate solver-6 sweep that
  *does* need recompiles). Full-map render (`warehouseXL_fullmap.png`,
  9050x1936, all 5 panels) is honest but too fine-grained to read component
  boundaries, same finding as `orz900d`; a 150x150-cell crop
  (`900,900,1050,1050`) makes individual components legible. Also exercised
  `--per-level-output-dir` for the first time (separate PNG per level,
  `per_level/level{1,2,3,4}.png` and `per_level_crop/level{1,2,3,4}.png`),
  in addition to the combined composite.

Output files under `outputs/coarsening_viz/`.

## Known gaps / not yet done

- No automatic "pick an interesting region" mode for `--crop` on large maps
  — currently manual, same gap `plot_guide_paths.py` has for `--agents` on
  large agent counts.
- `dump_coarsening` always builds a *fresh* hierarchy directly (not through
  `ReducedHierarchy::instance()`), so it doesn't reflect any runtime state
  (e.g. traffic-weighted costs) — it's the same hierarchy solver 6 builds at
  preprocessing time, but computed standalone for visualisation, not pulled
  from a live run.
