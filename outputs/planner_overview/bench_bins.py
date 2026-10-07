#!/usr/bin/env python3
"""Lift vs corridor path stretch binned by true (BFS) distance, from the
bench CSVs in outputs/hierarchy_lift_bench/coarse_astar/ (same pairs and
levels as ai/hierarchical_guide_paths_plan.md). Stretch = path length /
shortest; corridor = margin 0. Writes bench_bins.txt next to this file."""
import csv
from pathlib import Path

D = Path(__file__).resolve().parent
SRC = D.parent / "hierarchy_lift_bench" / "coarse_astar"
BINS = [0, 100, 300, 1000, 2000, 4000, 10**9]


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float("nan")


out = ["Path length / shortest by shortest distance (bench pairs). mean (p90) per bin; n = pairs."]
for name in ["orz900d", "ih", "scene"]:
    rows = list(csv.DictReader(open(SRC / f"{name}.csv")))
    for level in sorted({int(r["level"]) for r in rows}):
        out.append(f"\n{name} level {level}")
        out.append(f"  {'distance':>12} {'n':>4} {'lift':>15} {'corridor':>15}")
        for lo, hi in zip(BINS, BINS[1:]):
            b = [r for r in rows if int(r["level"]) == level and lo <= int(r["true_dist"]) < hi]
            if not b:
                continue
            lift = [int(r["final_len"]) / max(1, int(r["true_dist"])) for r in b if r["valid"] == "1"]
            corr = [int(r["c0_len"]) / max(1, int(r["true_dist"])) for r in b if r["c0_ok"] == "1"]
            hi_s = "inf" if hi >= 10**9 else hi
            out.append(f"  {f'{lo}-{hi_s}':>12} {len(b):>4} {mean(lift):7.3f} ({pct(lift, .9):5.2f})"
                       f" {mean(corr):7.3f} ({pct(corr, .9):5.2f})")
(D / "bench_bins.txt").write_text("\n".join(out) + "\n")
print("\n".join(out))
