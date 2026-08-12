#!/usr/bin/env python3
"""
Render one PNG per simulation timestep from the CSVs BaseSystem::simulate()
streams when CompetitionSystem.h's kDumpPerTimestepPaths switch is flipped
on: agent_positions_per_timestep.csv (every agent, every timestep) and,
optionally, guide_paths_per_timestep.csv (the scheduler's guide paths --
sparse, and only ever non-empty with --useTraffic past timestep 100, see
ai/project_context.md, "Guide paths: which solvers provide them, and are
they doing anything").

Both CSVs are streamed in a single pass, grouped by timestep (they're
written in increasing-timestep order by the C++ side, one block of rows per
timestep), rather than loaded into memory -- a long run can produce millions
of position rows.

Usage:
    python3 plot_timestep_frames.py <map_file> <positions.csv> -o frames/
    python3 plot_timestep_frames.py <map_file> <positions.csv> \
        --guide-paths guide_paths.csv -o frames/ \
        --start 100 --end 200 --every 5 --crop 20,20,80,80
"""

import argparse
import csv
import itertools
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OBSTACLE_COLOR = (40, 40, 40)
FREE_COLOR = (245, 245, 245)
AGENT_COLOR = (0, 102, 204)             # blue: agent position this timestep
GUIDE_PATH_COLOR = (230, 126, 34)       # orange: scheduler guide path this timestep
GUIDE_PATH_AGENT_COLOR = (200, 30, 30)  # red: agent whose guide path is active
BG_COLOR = (255, 255, 255)


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


def timestep_groups(csv_path):
    """Stream (timestep:int, [row dicts]) in increasing-timestep order."""
    if csv_path is None:
        return
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for timestep_str, rows in itertools.groupby(reader, key=lambda r: r["timestep"]):
            yield int(timestep_str), list(rows)


def render_frame(grid, positions, guide_paths, crop, scale, timestep, dot_r, line_w):
    r0, c0, r1, c1 = crop
    sub = grid[r0:r1 + 1, c0:c1 + 1]
    h, w = sub.shape
    out_w, out_h = w * scale, h * scale

    base_rgb = np.where(sub[:, :, None] == 1,
                         np.array(OBSTACLE_COLOR, dtype=np.uint8),
                         np.array(FREE_COLOR, dtype=np.uint8))
    img = Image.fromarray(base_rgb).resize((out_w, out_h), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    def to_px(row, col):
        return ((col - c0 + 0.5) * scale, (row - r0 + 0.5) * scale)

    paths_by_agent = {}
    for row in guide_paths:
        paths_by_agent.setdefault(int(row["agent_id"]), []).append(row)

    guided_agents = set()
    for agent_id, rows in paths_by_agent.items():
        guided_agents.add(agent_id)
        rows.sort(key=lambda r: int(r["step"]))
        pts = [(int(r["row"]), int(r["col"])) for r in rows]
        pts_in_view = [p for p in pts if r0 <= p[0] <= r1 and c0 <= p[1] <= c1]
        if len(pts_in_view) >= 2:
            px = [to_px(*p) for p in pts_in_view]
            draw.line(px, fill=GUIDE_PATH_COLOR, width=line_w, joint="curve")

    for row in positions:
        r, c = int(row["row"]), int(row["col"])
        if not (r0 <= r <= r1 and c0 <= c <= c1):
            continue
        x, y = to_px(r, c)
        color = GUIDE_PATH_AGENT_COLOR if int(row["agent_id"]) in guided_agents else AGENT_COLOR
        draw.ellipse([x - dot_r, y - dot_r, x + dot_r, y + dot_r], fill=color)

    legend_h = 26
    font = ImageFont.load_default(size=14)
    canvas = Image.new("RGB", (out_w, out_h + legend_h), BG_COLOR)
    legend = ("timestep %d   agent position = blue   "
              "agent with active guide path = red   guide path = orange line" % timestep)
    ImageDraw.Draw(canvas).text((6, 4), legend, fill=(0, 0, 0), font=font)
    canvas.paste(img, (0, legend_h))
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_file")
    ap.add_argument("positions_csv")
    ap.add_argument("-o", "--output-dir", required=True)
    ap.add_argument("--guide-paths", help="guide_paths_per_timestep.csv (optional -- omit to render positions only)")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, help="inclusive (default: no upper bound, i.e. through the last timestep in the CSV)")
    ap.add_argument("--every", type=int, default=1, help="render every Nth timestep in [start, end] (default: 1, every timestep)")
    ap.add_argument("--crop", help="r0,c0,r1,c1 fine-cell region (inclusive); default: whole map")
    ap.add_argument("--scale", type=int, help="pixels per cell (default: auto, ~900px on the longer side)")
    ap.add_argument("--agents", help="comma-separated agent ids to draw (default: all)")
    ap.add_argument("--dot-radius", type=float, help="agent marker radius in px (default: auto from scale)")
    args = ap.parse_args()

    grid = parse_map(args.map_file)
    rows, cols = grid.shape

    if args.crop:
        crop = tuple(int(x) for x in args.crop.split(","))
        if len(crop) != 4:
            print("--crop expects r0,c0,r1,c1", file=sys.stderr)
            sys.exit(1)
    else:
        crop = (0, 0, rows - 1, cols - 1)
    r0, c0, r1, c1 = crop
    crop_h, crop_w = r1 - r0 + 1, c1 - c0 + 1

    scale = args.scale or max(1, min(24, 900 // max(crop_h, crop_w, 1)))
    dot_r = args.dot_radius if args.dot_radius is not None else max(1.5, scale / 3)
    line_w = max(1, round(scale / 6))

    wanted_agents = set(int(a) for a in args.agents.split(",")) if args.agents else None

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pos_groups = timestep_groups(args.positions_csv)
    guide_groups = timestep_groups(args.guide_paths) if args.guide_paths else iter(())

    # Merge-by-timestep: both streams are sorted ascending by timestep, so a
    # single advancing pointer into guide_groups keeps this O(rows) total
    # even when --start/--every skip most position groups.
    next_guide = next(guide_groups, None)
    n_written = 0
    for timestep, pos_rows in pos_groups:
        if timestep < args.start:
            continue
        if args.end is not None and timestep > args.end:
            break
        if (timestep - args.start) % args.every != 0:
            continue

        while next_guide is not None and next_guide[0] < timestep:
            next_guide = next(guide_groups, None)
        guide_rows = next_guide[1] if next_guide is not None and next_guide[0] == timestep else []

        if wanted_agents is not None:
            pos_rows = [r for r in pos_rows if int(r["agent_id"]) in wanted_agents]
            guide_rows = [r for r in guide_rows if int(r["agent_id"]) in wanted_agents]

        frame = render_frame(grid, pos_rows, guide_rows, crop, scale, timestep, dot_r, line_w)
        frame.save(out_dir / f"frame_{timestep:06d}.png")
        n_written += 1

    print(f"wrote {n_written} frames to {out_dir}")


if __name__ == "__main__":
    main()
