#!/usr/bin/env python3
"""
Compute throughput metrics from lifelong-MAPF result JSON(s).

Background: the simulator's output JSON has two different counters that
both look like "number of timesteps" but are not the same thing (see
ai/auto_benchmarking.md for the full writeup):

  - "steps" (len(timeStepMetrics)): one log entry is appended per pass
    through the main simulation loop (src/CompetitionSystem.cpp,
    BaseSystem::simulate). When the planner times out and the simulator has
    to force several "wait" timesteps to catch up, that whole multi-step
    batch can still only add 1-2 log entries -- so this number can
    *undercount* real elapsed simulated time.
  - "makespan": the max, over all agents, of a per-agent counter that is
    incremented once per *real* elapsed simulator timestep (including
    forced wait-timesteps). This is the more accurate measure of how much
    simulated time actually passed.

Dividing tasksFinished by "steps" therefore overstates throughput for any
run where the planner timed out a lot (observed for --scheduleModel 1 on
the orz900d map). Dividing by "makespan" gives real-time throughput
instead. This script reports both so you can compare/plot either.

Usage:
    python3 compute_throughput_metrics.py <result.json>
    python3 compute_throughput_metrics.py <folder_of_results>/
    python3 compute_throughput_metrics.py <folder_of_results>/ -o metrics.csv
    python3 compute_throughput_metrics.py <folder_of_results>/ --recursive
"""

import argparse
import csv
import json
import re
import sys
from pathlib import Path

# Matches the repo's benchmark naming convention, e.g.:
#   orz900d_5000_solver1.json          -> agents=5000, label=solver1
#   orz900d_10000_solver6_level3.json  -> agents=10000, label=solver6_level3
# Falls back to agents=None, label=<stem> for files that don't match.
FILENAME_RE = re.compile(r"^.+?_(?P<agents>\d+)_(?P<label>.+)$")


def parse_filename(stem: str):
    """Best-effort extraction of (agents, label) from a result filename stem."""
    m = FILENAME_RE.match(stem)
    if m:
        return int(m.group("agents")), m.group("label")
    return None, stem


def compute_metrics(json_path: Path) -> dict:
    """Load one result JSON and compute the file/agents/tasks/steps/makespan/
    throughput row described in ai/auto_benchmarking.md."""
    with open(json_path) as f:
        data = json.load(f)

    agents_from_name, label = parse_filename(json_path.stem)

    tasks = data["numTaskFinished"]
    makespan = data["makespan"]
    steps = len(data.get("timeStepMetrics", []))
    # teamSize is authoritative if present; the filename is just a fallback.
    agents = data.get("teamSize", agents_from_name)

    return {
        "file": json_path.name,
        "label": label,
        "agents": agents,
        "tasks": tasks,
        "steps": steps,
        "makespan": makespan,
        # Guard against div-by-zero on a degenerate/empty run.
        "tp_steps": tasks / steps if steps else float("nan"),
        "tp_makespan": tasks / makespan if makespan else float("nan"),
    }


def find_result_files(path: Path, recursive: bool):
    if path.is_file():
        return [path]
    pattern = "**/*.json" if recursive else "*.json"
    return sorted(path.glob(pattern))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="A result .json file, or a folder containing them")
    parser.add_argument("-o", "--output", type=Path, help="Write CSV here instead of stdout")
    parser.add_argument("--recursive", action="store_true", help="Recurse into subfolders when path is a directory")
    args = parser.parse_args()

    if not args.path.exists():
        sys.exit(f"error: {args.path} does not exist")

    result_files = find_result_files(args.path, args.recursive)
    if not result_files:
        sys.exit(f"error: no .json files found under {args.path}")

    rows = [compute_metrics(f) for f in result_files]
    # Sort by agent count then label so multi-config sweeps plot in a sane order.
    rows.sort(key=lambda r: (r["agents"] if r["agents"] is not None else -1, r["label"]))

    fieldnames = ["file", "agents", "label", "tasks", "steps", "makespan", "tp_steps", "tp_makespan"]
    out = open(args.output, "w", newline="") if args.output else sys.stdout
    try:
        writer = csv.DictWriter(out, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in fieldnames})
    finally:
        if args.output:
            out.close()

    if args.output:
        print(f"wrote {len(rows)} rows to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
