#!/usr/bin/env python3
"""
Render a per-agent comparison of solver 1 vs. solver 6 guide paths, cropped
and zoomed to each agent's own local neighborhood (guide paths here are a
handful of fine-map cells, not map-spanning routes, so one global map view
would show them as invisible dots -- see ai/project_context.md's guide-path
discussion for why they're this short).

Input is the CSV produced by ./build/dump_guide_paths <instance.json>
<csv> [num_agents], plus the .map file the instance points at. Solver 1 and
solver 6 assign tasks independently, so the same agent can get a different
destination task under each solver -- both are drawn and labelled as-is
(not forced to match), including a per-agent flag for whether the task
assignment and/or exact path matched.

Usage:
    python3 plot_guide_paths.py <map_file> <paths.csv> -o comparison.png
    python3 plot_guide_paths.py <map_file> <paths.csv> -o comparison.png --agents 0,3,5
"""

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

SOLVER1_COLOR = (0, 102, 204)   # blue
SOLVER6_COLOR = (230, 126, 34)  # orange
SAME_PATH_COLOR = (128, 0, 200) # purple, used when solver1 == solver6 exactly
START_COLOR = (34, 139, 34)     # green
OBSTACLE_COLOR = (40, 40, 40)
FREE_COLOR = (245, 245, 245)
GRIDLINE_COLOR = (215, 215, 215)
BG_COLOR = (255, 255, 255)

PANEL_TARGET_PX = 320
CELL_PX_MAX = 28
PANEL_PADDING = 14
TITLE_H = 34
COLS = 5


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


def parse_csv(path):
    # agent_id -> solver -> {"cells": [(row, col), ...], "task": int}
    data = defaultdict(dict)
    with open(path) as f:
        for row in csv.DictReader(f):
            agent_id = int(row["agent_id"])
            solver = row["solver"]
            entry = data[agent_id].setdefault(solver, {"cells": [], "task": row["task_id"]})
            entry["cells"].append((int(row["row"]), int(row["col"])))
    return data


def render_panel(grid, agent_id, paths):
    p1 = paths.get("solver1")
    p6 = paths.get("solver6")
    cells = (p1["cells"] if p1 else []) + (p6["cells"] if p6 else [])
    rows = [r for r, c in cells]
    cols = [c for r, c in cells]
    pad = 3
    r0 = max(0, min(rows) - pad)
    r1 = min(grid.shape[0] - 1, max(rows) + pad)
    c0 = max(0, min(cols) - pad)
    c1 = min(grid.shape[1] - 1, max(cols) + pad)

    crop = grid[r0:r1 + 1, c0:c1 + 1]
    crop_h, crop_w = crop.shape

    # Guide paths here can be anywhere from a couple of cells (dense task
    # pools) to spanning most of the map (sparse task pools -- capping the
    # task count can push an agent's nearest task very far away). A fixed
    # pixels-per-cell floor would blow up the whole grid's cell size for one
    # oversized panel, so scale is a float that can go below 1 px/cell for
    # huge crops instead of clamping.
    raw_scale = PANEL_TARGET_PX / max(crop_h, crop_w, 1)
    scale = max(0.15, min(CELL_PX_MAX, raw_scale))
    out_w = max(1, round(crop_w * scale))
    out_h = max(1, round(crop_h * scale))
    sx, sy = out_w / crop_w, out_h / crop_h

    base_rgb = np.where(crop[:, :, None] == 1,
                         np.array(OBSTACLE_COLOR, dtype=np.uint8),
                         np.array(FREE_COLOR, dtype=np.uint8))
    img = Image.fromarray(base_rgb).resize((out_w, out_h), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    if min(sx, sy) >= 8:
        for r in range(crop_h + 1):
            draw.line([(0, r * sy), (out_w, r * sy)], fill=GRIDLINE_COLOR)
        for c in range(crop_w + 1):
            draw.line([(c * sx, 0), (c * sx, out_h)], fill=GRIDLINE_COLOR)

    def to_px(rc):
        r, c = rc
        return ((c - c0 + 0.5) * sx, (r - r0 + 0.5) * sy)

    same_path = bool(p1 and p6 and p1["cells"] == p6["cells"])
    line_width = max(1, round(min(sx, sy) / 6))
    offset = min(sx, sy) * 0.08

    def draw_path(entry, color, off):
        if not entry or len(entry["cells"]) < 2:
            return
        pts = [to_px(rc) for rc in entry["cells"]]
        pts = [(x + off, y + off) for x, y in pts]
        draw.line(pts, fill=color, width=line_width)

    if same_path:
        draw_path(p1, SAME_PATH_COLOR, 0)
    else:
        draw_path(p1, SOLVER1_COLOR, -offset)
        draw_path(p6, SOLVER6_COLOR, offset)

    r_marker = max(3, min(sx, sy) / 4)
    if p1:
        x, y = to_px(p1["cells"][0])
        draw.ellipse([x - r_marker, y - r_marker, x + r_marker, y + r_marker], fill=START_COLOR)
        x, y = to_px(p1["cells"][-1])
        draw.rectangle([x - r_marker, y - r_marker, x + r_marker, y + r_marker], fill=SOLVER1_COLOR)
    if p6:
        x, y = to_px(p6["cells"][-1])
        draw.rectangle([x - r_marker, y - r_marker, x + r_marker, y + r_marker], fill=SOLVER6_COLOR)

    t1 = p1["task"] if p1 else "?"
    t6 = p6["task"] if p6 else "?"
    len1 = len(p1["cells"]) - 1 if p1 else "?"
    len6 = len(p6["cells"]) - 1 if p6 else "?"
    same_task = bool(p1 and p6 and t1 == t6)
    title = f"agent {agent_id}  task1={t1} (len={len1})  task6={t6} (len={len6})"
    if same_task:
        title += "  [same task]"
    if same_path:
        title += "  [same path]"

    font = ImageFont.load_default(size=13)
    text_w = font.getbbox(title)[2] + 4
    canvas_w = max(img.width, text_w)
    canvas = Image.new("RGB", (canvas_w, img.height + TITLE_H), BG_COLOR)
    ImageDraw.Draw(canvas).text((2, 2), title, fill=(0, 0, 0), font=font)
    canvas.paste(img, ((canvas_w - img.width) // 2, TITLE_H))
    return canvas


def compose_grid(panels, cols=COLS):
    rows = (len(panels) + cols - 1) // cols
    cell_w = max(p.width for p in panels) + PANEL_PADDING
    cell_h = max(p.height for p in panels) + PANEL_PADDING
    legend_h = 26
    canvas = Image.new("RGB", (cell_w * cols, cell_h * rows + legend_h), BG_COLOR)
    for i, p in enumerate(panels):
        x = (i % cols) * cell_w
        y = legend_h + (i // cols) * cell_h
        canvas.paste(p, (x, y))
    font = ImageFont.load_default(size=14)
    legend = (
        "solver1 path/dest = blue   solver6 path/dest = orange   "
        "identical path = purple   start = green dot"
    )
    ImageDraw.Draw(canvas).text((6, 4), legend, fill=(0, 0, 0), font=font)
    return canvas


def render_full_map(grid, data, agent_ids, scale=2, line_width=5, marker_r=7):
    rows, cols = grid.shape
    out_w, out_h = cols * scale, rows * scale
    base_rgb = np.where(grid[:, :, None] == 1,
                         np.array(OBSTACLE_COLOR, dtype=np.uint8),
                         np.array(FREE_COLOR, dtype=np.uint8))
    img = Image.fromarray(base_rgb).resize((out_w, out_h), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    def to_px(rc):
        r, c = rc
        return ((c + 0.5) * scale, (r + 0.5) * scale)

    offset = line_width * 0.6
    for agent_id in agent_ids:
        paths = data.get(agent_id)
        if not paths:
            continue
        p1 = paths.get("solver1")
        p6 = paths.get("solver6")
        same_path = bool(p1 and p6 and p1["cells"] == p6["cells"])

        def draw_path(entry, color, off):
            if not entry or len(entry["cells"]) < 2:
                return
            pts = [to_px(rc) for rc in entry["cells"]]
            pts = [(x + off, y + off) for x, y in pts]
            draw.line(pts, fill=color, width=line_width, joint="curve")

        if same_path:
            draw_path(p1, SAME_PATH_COLOR, 0)
        else:
            draw_path(p1, SOLVER1_COLOR, -offset)
            draw_path(p6, SOLVER6_COLOR, offset)

        if p1:
            x, y = to_px(p1["cells"][0])
            draw.ellipse([x - marker_r, y - marker_r, x + marker_r, y + marker_r], fill=START_COLOR)
            x, y = to_px(p1["cells"][-1])
            draw.rectangle([x - marker_r, y - marker_r, x + marker_r, y + marker_r], fill=SOLVER1_COLOR)
        if p6:
            x, y = to_px(p6["cells"][-1])
            draw.rectangle([x - marker_r, y - marker_r, x + marker_r, y + marker_r], fill=SOLVER6_COLOR)

    legend_h = 30
    font = ImageFont.load_default(size=16)
    canvas = Image.new("RGB", (out_w, out_h + legend_h), BG_COLOR)
    legend = (
        "solver1 path/dest = blue   solver6 path/dest = orange   "
        "identical path = purple   start = green dot"
    )
    ImageDraw.Draw(canvas).text((6, 5), legend, fill=(0, 0, 0), font=font)
    canvas.paste(img, (0, legend_h))
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_file")
    ap.add_argument("paths_csv")
    ap.add_argument("-o", "--output", required=True, help="per-agent cropped panel grid PNG")
    ap.add_argument("--full-map-output", help="also render a separate whole-map overview PNG with all agents' paths")
    ap.add_argument("--full-map-scale", type=int, default=2, help="pixels per cell for the whole-map overview (default: 2)")
    ap.add_argument("--agents", help="comma-separated agent ids for the per-agent panel grid (default: all present in CSV)")
    ap.add_argument("--full-map-agents", help="comma-separated agent ids for the whole-map overview "
                                               "(default: all present in CSV, independent of --agents)")
    ap.add_argument("--cols", type=int, default=COLS)
    args = ap.parse_args()

    grid = parse_map(args.map_file)
    data = parse_csv(args.paths_csv)

    if args.agents:
        wanted = [int(a) for a in args.agents.split(",")]
    else:
        wanted = sorted(data.keys())

    panels = []
    for agent_id in wanted:
        if agent_id not in data:
            print(f"warning: agent {agent_id} not found in {args.paths_csv}", file=sys.stderr)
            continue
        panels.append(render_panel(grid, agent_id, data[agent_id]))

    if not panels:
        print("no agents to render", file=sys.stderr)
        sys.exit(1)

    canvas = compose_grid(panels, cols=args.cols)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output)
    print(f"wrote {args.output} ({canvas.width}x{canvas.height}, {len(panels)} agents)")

    if args.full_map_output:
        if args.full_map_agents:
            full_map_wanted = [int(a) for a in args.full_map_agents.split(",")]
        else:
            full_map_wanted = sorted(data.keys())
        full_map = render_full_map(grid, data, full_map_wanted, scale=args.full_map_scale)
        Path(args.full_map_output).parent.mkdir(parents=True, exist_ok=True)
        full_map.save(args.full_map_output)
        print(f"wrote {args.full_map_output} ({full_map.width}x{full_map.height}, {len(full_map_wanted)} agents)")


if __name__ == "__main__":
    main()
