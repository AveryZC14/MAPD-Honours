#!/usr/bin/env python3
"""
Render the map-coarsening hierarchy (MapReductionTest::MultiLevelCoarsenedGraph)
spatially over the actual map: each coarse level's connected-component
partition of the fine cells, colored per component, with the coarse graph's
nodes (drawn at their representative fine-map location) and arcs overlaid on
top.

Input is the CSV set produced by
    ./build/dump_coarsening <instance.json> <output_prefix> [num_levels=2]
(<output_prefix>_partition_level<L>.csv, <output_prefix>_nodes.csv,
<output_prefix>_edges.csv), plus the .map file the instance points at.

Usage:
    python3 plot_coarsening.py <map_file> <output_prefix> -o out.png
    python3 plot_coarsening.py <map_file> <output_prefix> -o out.png --levels 1,2 --scale 4
"""

import argparse
import colorsys
import csv
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OBSTACLE_COLOR = (40, 40, 40)
FREE_COLOR = (245, 245, 245)      # walkable but ungrouped ("dud") cell
UNPARTITIONED_COLOR = (225, 225, 225)
NODE_FILL = (255, 255, 255)
NODE_OUTLINE = (20, 20, 20)
EDGE_COLOR = (20, 20, 20)
BG_COLOR = (255, 255, 255)

# Golden-angle hue stepping: consecutive component ids land far apart on the
# hue wheel, so neighboring components (which tend to have nearby ids, since
# components are discovered in row-major scan order) read as visually
# distinct without needing a fixed-size categorical palette -- component
# counts here range from a handful (tiny maps) to hundreds of thousands
# (orz900d), far beyond any palette with individually distinguishable colors.
GOLDEN_ANGLE = 0.6180339887498949


def component_color(node_id):
    hue = (node_id * GOLDEN_ANGLE) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.55, 0.92)
    return (int(r * 255), int(g * 255), int(b * 255))


def parse_map(path):
    with open(path) as f:
        lines = f.read().splitlines()
    idx = 0
    if lines[idx].startswith("type"):
        idx += 1
    height = int(lines[idx].split()[1]); idx += 1
    width = int(lines[idx].split()[1]); idx += 1
    assert lines[idx].strip() == "map"
    idx += 1
    grid = np.zeros((height, width), dtype=np.uint8)
    for r in range(height):
        line = lines[idx + r]
        for c in range(width):
            if line[c] in ("@", "T"):
                grid[r, c] = 1
    return grid


def parse_partition(path):
    with open(path) as f:
        rows = [[int(x) for x in line.split(",")] for line in f.read().splitlines() if line]
    return np.array(rows, dtype=np.int64)


def parse_nodes(path):
    # level -> [(node_id, fine_row, fine_col, num_children), ...]
    nodes = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            level = int(row["level"])
            nodes.setdefault(level, []).append(
                (int(row["node_id"]), int(row["fine_row"]), int(row["fine_col"]), int(row["num_children"]))
            )
    return nodes


def parse_edges(path):
    # level -> [(src_row, src_col, dst_row, dst_col), ...], deduplicated
    # since the coarse graph stores each neighbor arc in both directions.
    edges = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            level = int(row["level"])
            sr, sc = int(row["src_row"]), int(row["src_col"])
            dr, dc = int(row["dst_row"]), int(row["dst_col"])
            key = frozenset({(sr, sc), (dr, dc)})
            edges.setdefault(level, {})[key] = (sr, sc, dr, dc)
    return {level: list(d.values()) for level, d in edges.items()}


def crop_arrays(wall_grid, partition, nodes, edges, crop):
    r0, c0, r1, c1 = crop
    rows, cols = wall_grid.shape
    r0, r1 = max(0, r0), min(rows - 1, r1)
    c0, c1 = max(0, c0), min(cols - 1, c1)

    cropped_wall = wall_grid[r0:r1 + 1, c0:c1 + 1]
    cropped_partition = partition[r0:r1 + 1, c0:c1 + 1] if partition is not None else None

    cropped_nodes = None
    if nodes is not None:
        cropped_nodes = [(nid, fr - r0, fc - c0, n)
                          for nid, fr, fc, n in nodes if r0 <= fr <= r1 and c0 <= fc <= c1]

    cropped_edges = None
    if edges is not None:
        cropped_edges = [(sr - r0, sc - c0, dr - r0, dc - c0)
                          for sr, sc, dr, dc in edges
                          if r0 <= sr <= r1 and c0 <= sc <= c1 and r0 <= dr <= r1 and c0 <= dc <= c1]

    return cropped_wall, cropped_partition, cropped_nodes, cropped_edges


def render_level(wall_grid, partition, nodes, edges, scale, title,
                  draw_nodes=True, draw_edges=True):
    rows, cols = wall_grid.shape
    out_w, out_h = cols * scale, rows * scale

    base = np.where(wall_grid[:, :, None] == 1,
                     np.array(OBSTACLE_COLOR, dtype=np.uint8),
                     np.array(FREE_COLOR, dtype=np.uint8))

    if partition is not None:
        free = wall_grid == 0
        grouped = free & (partition >= 0)
        # Vectorized per-cell coloring: build an RGB image directly from
        # component ids rather than looping over every cell in Python.
        ids = partition[grouped]
        colors = np.array([component_color(int(i)) for i in np.unique(ids)], dtype=np.uint8)
        _, inverse = np.unique(ids, return_inverse=True)
        base[grouped] = colors[inverse]
        base[free & ~grouped] = np.array(UNPARTITIONED_COLOR, dtype=np.uint8)

    img = Image.fromarray(base).resize((out_w, out_h), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    def to_px(r, c):
        return ((c + 0.5) * scale, (r + 0.5) * scale)

    if draw_edges and edges:
        edge_width = max(1, round(scale / 8))
        for sr, sc, dr, dc in edges:
            draw.line([to_px(sr, sc), to_px(dr, dc)], fill=EDGE_COLOR, width=edge_width)

    if draw_nodes and nodes:
        r_marker = max(1.5, scale / 5)
        for _node_id, fr, fc, _num_children in nodes:
            x, y = to_px(fr, fc)
            draw.ellipse([x - r_marker, y - r_marker, x + r_marker, y + r_marker],
                         fill=NODE_FILL, outline=NODE_OUTLINE)

    title_h = 26
    font = ImageFont.load_default(size=14)
    canvas = Image.new("RGB", (out_w, out_h + title_h), BG_COLOR)
    ImageDraw.Draw(canvas).text((4, 4), title, fill=(0, 0, 0), font=font)
    canvas.paste(img, (0, title_h))
    return canvas


def compose_row(panels):
    pad = 10
    cell_w = max(p.width for p in panels) + pad
    cell_h = max(p.height for p in panels) + pad
    canvas = Image.new("RGB", (cell_w * len(panels), cell_h), BG_COLOR)
    for i, p in enumerate(panels):
        canvas.paste(p, (i * cell_w, 0))
    return canvas


def auto_scale(rows, cols, target_px=1100, lo=1, hi=24):
    return max(lo, min(hi, round(target_px / max(rows, cols, 1))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_file")
    ap.add_argument("csv_prefix", help="prefix passed to dump_coarsening (without _partition_level<L>.csv etc.)")
    ap.add_argument("-o", "--output", help="output PNG: one combined panel row, fine map + every level "
                                            "(optional if --per-level-output-dir is given)")
    ap.add_argument("--levels", help="comma-separated levels to render (default: every _partition_level<L>.csv found)")
    ap.add_argument("--scale", type=int, help="pixels per fine cell (default: auto, ~1100px on the longer side)")
    ap.add_argument("--crop", help="r0,c0,r1,c1 fine-cell region to zoom into (inclusive) -- full-map renders of "
                                    "large/maze-like maps are too fine-grained to read at any pixel scale; use "
                                    "this to inspect one area's coarsening in detail")
    ap.add_argument("--no-nodes", action="store_true", help="don't draw coarse-node markers")
    ap.add_argument("--no-edges", action="store_true", help="don't draw coarse-graph arcs")
    ap.add_argument("--per-level-output-dir", help="write one PNG per level into this directory")
    args = ap.parse_args()

    if not args.output and not args.per_level_output_dir:
        print("need at least one of -o/--output or --per-level-output-dir", file=sys.stderr)
        sys.exit(1)

    wall_grid = parse_map(args.map_file)

    crop = None
    if args.crop:
        crop = tuple(int(x) for x in args.crop.split(","))
        if len(crop) != 4:
            print("--crop expects r0,c0,r1,c1", file=sys.stderr)
            sys.exit(1)

    display_wall_grid = wall_grid
    if crop:
        display_wall_grid, _, _, _ = crop_arrays(wall_grid, None, None, None, crop)
    rows, cols = display_wall_grid.shape
    scale = args.scale or auto_scale(rows, cols)

    prefix = Path(args.csv_prefix)
    if args.levels:
        levels = [int(x) for x in args.levels.split(",")]
    else:
        found = sorted(int(p.stem.rsplit("level", 1)[1])
                        for p in prefix.parent.glob(f"{prefix.name}_partition_level*.csv"))
        if not found:
            print(f"no {prefix}_partition_level*.csv files found", file=sys.stderr)
            sys.exit(1)
        levels = found

    nodes_path = Path(f"{prefix}_nodes.csv")
    edges_path = Path(f"{prefix}_edges.csv")
    all_nodes = parse_nodes(nodes_path) if nodes_path.exists() else {}
    all_edges = parse_edges(edges_path) if edges_path.exists() else {}

    panels = [render_level(display_wall_grid, None, None, None, scale,
                            f"level 0 (fine map, {rows}x{cols})",
                            draw_nodes=False, draw_edges=False)]

    per_level_dir = Path(args.per_level_output_dir) if args.per_level_output_dir else None
    if per_level_dir:
        per_level_dir.mkdir(parents=True, exist_ok=True)
        panels[0].save(per_level_dir / "level0.png")

    for level in levels:
        part_path = Path(f"{prefix}_partition_level{level}.csv")
        if not part_path.exists():
            print(f"warning: {part_path} not found, skipping level {level}", file=sys.stderr)
            continue
        partition = parse_partition(part_path)
        level_nodes = all_nodes.get(level, [])
        level_edges = all_edges.get(level, [])
        if crop:
            _, partition, level_nodes, level_edges = crop_arrays(wall_grid, partition, level_nodes, level_edges, crop)
        title = f"level {level}  ({len(level_nodes)} nodes in view)" if crop else f"level {level}  ({len(level_nodes)} coarse nodes)"
        panel = render_level(display_wall_grid, partition, level_nodes, level_edges, scale, title,
                              draw_nodes=not args.no_nodes, draw_edges=not args.no_edges)
        panels.append(panel)
        if per_level_dir:
            panel.save(per_level_dir / f"level{level}.png")
            print(f"wrote {per_level_dir / f'level{level}.png'}")

    if args.output:
        canvas = compose_row(panels)
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        canvas.save(args.output)
        print(f"wrote {args.output} ({canvas.width}x{canvas.height}, {len(panels)} panels)")


if __name__ == "__main__":
    main()
