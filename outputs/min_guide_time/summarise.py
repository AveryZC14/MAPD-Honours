#!/usr/bin/env python3
"""Summarise the planner overview runs (run.sh): per-run table, per-path
distributions from the guide-path traces, distance bins, and a compact
series.json (per-decision / per-step series) for charts.

Standard library only. Reads whatever runs exist, so it can be rerun while
run.sh is still going.

    python3 summarise.py            # writes summary.txt and series.json next to this file
"""
import csv, gzip, io, json, re
from pathlib import Path

D = Path(__file__).resolve().parent
LONG = 500  # paths with start-goal Manhattan distance >= LONG count as long (delivery legs)
BINS = [0, 50, 200, 500, 1000, 2000, 4000, 10**9]


def read(path):
    """Text of path, or of path.gz; None if neither exists."""
    if path.exists():
        return path.read_text()
    gz = path.with_name(path.name + ".gz")
    if gz.exists():
        try:
            return gzip.open(gz, "rt").read()
        except (EOFError, OSError):  # still being written
            return None
    return None


def pct(xs, q):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def parse_log(text):
    """planner stats lines (dicts), entry-timing scheduler ms, stuck breakdown."""
    stats, sched, stuck = [], [], []
    for line in text.splitlines():
        if line.startswith("planner stats:"):
            stats.append({k: float(v) for k, v in re.findall(r"(\w+) (-?[\d.]+(?:e[+-]?\d+)?)", line)})
        elif line.startswith("entry timing:"):
            m = re.search(r"scheduler_ms (-?[\d.]+)", line)
            sched.append(float(m.group(1)) if m else 0.0)
        elif line.startswith("stuck breakdown:"):
            stuck.append({k: int(v) for k, v in re.findall(r"(\w+) (\d+)", line)})
    return stats, sched, stuck


def parse_trace(text):
    rows = []
    for r in csv.DictReader(io.StringIO(text)):
        if r.get("build_ms") in (None, ""):  # last line of a trace still being written
            continue
        rows.append((int(r["timestep"]), r["source"], int(r["manhattan"]), int(r["cells"]), float(r["ms"]),
                     float(r["coarse_ms"]), float(r["build_ms"])))
    return rows


def first_clear(values, limit=0):
    """1-based decision from which values stay <= limit (None if never)."""
    last = None
    for i, v in enumerate(values):
        if v > limit:
            last = i
    if last is None:
        return 1
    return last + 2 if last + 1 < len(values) else None


def summarise_run(name):
    log = read(D / f"{name}.log")
    if log is None:
        return None
    stats, sched, stuck = parse_log(log)
    js = read(D / f"{name}.json")
    js = json.loads(js) if js else None
    trace = read(D / f"{name}.trace.csv")
    trace = parse_trace(trace) if trace else []
    t = D / f"{name}.time"
    rss = elapsed = None
    if t.exists():
        tt = t.read_text()
        m = re.search(r"Maximum resident set size \(kbytes\): (\d+)", tt)
        rss = int(m.group(1)) / 1024 / 1024 if m else None
        m = re.search(r"Elapsed \(wall clock\) time .*: (\S+)", tt)
        elapsed = m.group(1) if m else None

    s = {"name": name, "decisions": len(stats), "done": js is not None, "peak_gb": rss, "elapsed": elapsed}
    if js:
        m = js["timeStepMetrics"]
        s.update(steps=js.get("makespan"), finished=js.get("numTaskFinished"), opened=js.get("numTaskOpened"),
                 errors=js.get("numPlannerErrors"), timeouts=js.get("numEntryTimeouts"),
                 planner_s_mean=mean([x["PlannerTime"] for x in m]))
    col = lambda k: [r.get(k, 0.0) for r in stats]
    no_path = col("bfs_no_path")
    s.update(
        paths=sum(col("paths_built")), fallbacks=sum(col("hier_fallbacks")),
        no_path_peak=max(no_path, default=0),
        # backlog cleared: no-path count stays at or under 1% of the agents
        no_path_clear=first_clear(no_path, 0.01 * (js or {}).get("teamSize", 0)) if stats else None,
        no_path_end=no_path[-1] if no_path else None,
        stuck_max=max(col("stuck_agents"), default=0), stuck_end=col("stuck_agents")[-1] if stats else None,
        guide_ms_mean=mean(col("guide_ms")), fw_ms_mean=mean(col("fw_ms")),
        pibt_ms_mean=mean(col("pibt_ms")), pibt_ms_max=max(col("pibt_ms"), default=0),
        pibt_reserve=stats[0].get("pibt_reserve_ms") if stats else None,
        sched_ms_mean=mean(sched), sched_ms_max=max(sched, default=0),
        abandoned=sum(col("abandoned_searches")),
    )
    if trace:
        ms = [r[4] for r in trace]
        long_ = [r for r in trace if r[2] >= LONG]
        s.update(
            trace_paths=len(trace),
            ms_mean=mean(ms), ms_p50=pct(ms, .5), ms_p90=pct(ms, .9), ms_p99=pct(ms, .99), ms_max=max(ms),
            long_paths=len(long_), long_ms_mean=mean([r[4] for r in long_]), long_ms_p90=pct([r[4] for r in long_], .9),
            long_coarse_ms=mean([r[5] for r in long_]), long_build_ms=mean([r[6] for r in long_]),
            long_cells_mean=mean([r[3] for r in long_]),
            # path length / Manhattan distance (a lower bound on the shortest path), per path, mean
            stretch_m=mean([r[3] / r[2] for r in trace if r[2] > 0]),
            long_stretch_m=mean([r[3] / r[2] for r in long_ if r[2] > 0]),
            sources={src: sum(1 for r in trace if r[1] == src) for src in sorted({r[1] for r in trace})},
        )
        bins = []
        for lo, hi in zip(BINS, BINS[1:]):
            b = [r for r in trace if lo <= r[2] < hi]
            if b:
                bins.append({"lo": lo, "hi": hi, "n": len(b), "ms_mean": mean([r[4] for r in b]),
                             "ms_p90": pct([r[4] for r in b], .9),
                             "stretch_m": mean([r[3] / r[2] for r in b if r[2] > 0])})
        s["bins"] = bins

    # per-decision and per-step series for charts
    series = {"no_path": no_path, "stuck": col("stuck_agents"), "paths": col("paths_built"),
              "guide_ms": col("guide_ms"), "fw_ms": col("fw_ms"), "pibt_ms": col("pibt_ms"),
              "sched_ms": sched[:len(stats)]}
    if js:
        m = js["timeStepMetrics"]
        series["step"] = [x["Timestep"] for x in m]
        series["finished_step"] = [x["TasksFinishedThisStep"] for x in m]
        series["opened_step"] = [x["TasksOpenedThisStep"] for x in m]
    if trace:
        # per timestep: long paths built and their mean ms
        per = {}
        for r in trace:
            if r[2] >= LONG:
                c = per.setdefault(r[0], [0, 0.0])
                c[0] += 1
                c[1] += r[4]
        series["long_by_step"] = [[k, v[0], round(v[1] / v[0], 3)] for k, v in sorted(per.items())]
    mem = D / f"{name}.mem"
    if mem.exists():
        series["mem"] = [[float(x) for x in l.split()] for l in mem.read_text().splitlines() if len(l.split()) == 3]
    for k, v in series.items():
        if v and isinstance(v[0], float):
            series[k] = [round(x, 2) for x in v]
    return s, series


def fmt(v, nd=1):
    if v is None:
        return "-"
    if isinstance(v, float):
        return "nan" if v != v else f"{v:.{nd}f}"
    return str(v)


def main():
    names = sorted({p.name.split(".")[0] for p in D.glob("*.log*")})
    runs, all_series = [], {}
    for n in names:
        r = summarise_run(n)
        if r:
            runs.append(r[0])
            all_series[n] = r[1]
    out = []
    out.append("Planner overview runs (see run.sh). Long path = start-goal Manhattan >= %d. "
               "stretch_m = path cells / Manhattan distance (Manhattan is a lower bound, so this overstates "
               "true stretch; compare between runs, not against 1)." % LONG)
    cols = [("run", "name", 0), ("dec", "decisions", 0), ("steps", "steps", 0), ("fin", "finished", 0),
            ("opened", "opened", 0), ("err", "errors", 0), ("GB", "peak_gb", 1), ("paths", "paths", 0),
            ("fallb", "fallbacks", 0), ("nopath_pk", "no_path_peak", 0), ("clear1%@", "no_path_clear", 0),
            ("nopath_end", "no_path_end", 0), ("stuck_max", "stuck_max", 0), ("stuck_end", "stuck_end", 0),
            ("ms_p50", "ms_p50", 2), ("ms_p90", "ms_p90", 2), ("ms_max", "ms_max", 0),
            ("long_ms", "long_ms_mean", 2), ("long_p90", "long_ms_p90", 2), ("coarse", "long_coarse_ms", 2),
            ("build", "long_build_ms", 2), ("long_n", "long_paths", 0), ("long_cells", "long_cells_mean", 0),
            ("stretch_m", "long_stretch_m", 3), ("guide", "guide_ms_mean", 0), ("fw", "fw_ms_mean", 0),
            ("pibt", "pibt_ms_mean", 0), ("pibt_max", "pibt_ms_max", 0), ("reserve", "pibt_reserve", 0),
            ("sched", "sched_ms_mean", 0), ("sched_max", "sched_ms_max", 0)]
    out.append("")
    out.append("\t".join(c[0] for c in cols))
    for r in runs:
        out.append("\t".join(fmt(r.get(k), nd) for _, k, nd in cols) + ("" if r["done"] else "\t(running)"))
    out.append("")
    out.append("Per-path time and stretch by start-goal Manhattan distance")
    for r in runs:
        if r.get("bins"):
            out.append(f"{r['name']}: sources {r.get('sources')}")
            for b in r["bins"]:
                hi = "inf" if b["hi"] >= 10**9 else b["hi"]
                out.append(f"  {b['lo']:>5}-{hi:<5} n {b['n']:>7}  ms mean {b['ms_mean']:8.2f}  p90 {b['ms_p90']:8.2f}"
                           f"  stretch_m {b['stretch_m']:.3f}")
    (D / "summary.txt").write_text("\n".join(out) + "\n")
    (D / "series.json").write_text(json.dumps({"runs": runs, "series": all_series}, separators=(",", ":")))
    print("\n".join(out))


if __name__ == "__main__":
    main()
