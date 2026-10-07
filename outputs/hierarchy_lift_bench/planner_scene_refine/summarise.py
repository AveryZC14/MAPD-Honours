#!/usr/bin/env python3
"""Summarise the planner stats lines of the quick scene runs (refine vs corridor)."""
import gzip, json, re, sys
from pathlib import Path

D = Path(__file__).resolve().parent


def read(path):
    """Text of path, or of path.gz (the outputs are kept compressed)."""
    if path.exists():
        return path.read_text()
    gz = path.with_name(path.name + ".gz")
    return gzip.open(gz, "rt").read() if gz.exists() else None
KEYS = ["guide_ms", "paths_built", "hier_paths", "hier_fallbacks", "hier_coarse_ms", "hier_build_ms", "hier_refine_ms",
        "hier_cells", "corridor_expanded", "pibt_ms", "stuck_agents", "bfs_no_path", "bfs_too_far"]

for name in ["refine_L4", "refine_L6", "corridor_L4"]:
    src = name.replace("_L", " level ")
    log = read(D / f"scene_{name}.log")
    if log is None:
        continue
    rows = []
    for line in log.splitlines():
        if line.startswith("planner stats:"):
            vals = dict(re.findall(r"(\w+) (-?[\d.]+)", line))
            rows.append({k: float(vals.get(k, 0)) for k in KEYS})
    if not rows:
        print(src, "no planner stats yet"); continue
    paths = sum(r["hier_paths"] for r in rows)
    ms = sum(r["hier_coarse_ms"] + r["hier_refine_ms"] + r["hier_build_ms"] for r in rows)
    cells = sum(r["hier_cells"] for r in rows)
    print(f"== {src}: {len(rows)} decisions")
    print(f"  hierarchy paths {paths:.0f}, fallbacks {sum(r['hier_fallbacks'] for r in rows):.0f}, "
          f"ms per path {ms / max(paths, 1):.2f} (coarse {sum(r['hier_coarse_ms'] for r in rows) / max(paths, 1):.2f}, "
          f"refine {sum(r['hier_refine_ms'] for r in rows) / max(paths, 1):.2f}, "
          f"build {sum(r['hier_build_ms'] for r in rows) / max(paths, 1):.2f}), mean path length {cells / max(paths, 1):.0f} cells")
    print(f"  {'decision':>8} {'paths':>6} {'guide_ms':>9} {'pibt_ms':>8} {'no_path':>8} {'too_far':>8} {'stuck':>6}")
    for i in [0, 1, 2, 4, 9, 24, 49, 99, 149, 199, 249, 299]:
        if i < len(rows):
            r = rows[i]
            print(f"  {i + 1:>8} {r['hier_paths']:>6.0f} {r['guide_ms']:>9.0f} {r['pibt_ms']:>8.0f} "
                  f"{r['bfs_no_path']:>8.0f} {r['bfs_too_far']:>8.0f} {r['stuck_agents']:>6.0f}")
    js = read(D / f"scene_{name}.json")
    if js is not None:
        d = json.loads(js)
        print(f"  finished {d.get('numTaskFinished')}, opened {d.get('numTaskOpened')}, "
              f"planner errors {d.get('numPlannerErrors')}, makespan {d.get('makespan')}")
    t = D / f"scene_{name}.time"
    if t.exists():
        m = re.search(r"Maximum resident set size \(kbytes\): (\d+)", t.read_text())
        if m:
            print(f"  peak RSS {int(m.group(1)) / 1024 / 1024:.1f} GB")
