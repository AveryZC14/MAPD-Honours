#!/usr/bin/env python3
"""Write RESULTS.md: tables of the planner overview runs (run.sh).

Reads series.json (run summarise.py first), the guide-path traces and the
bench CSVs in ../hierarchy_lift_bench/coarse_astar/. Standard library only.
"""
import csv, gzip, json
from pathlib import Path

D = Path(__file__).resolve().parent
BENCH = D.parent / "hierarchy_lift_bench" / "coarse_astar"
data = json.load(open(D / "series.json"))
R = {r["name"]: r for r in data["runs"]}
S = data["series"]

MAPS = [("orz900d", "orz900d"), ("IH_mp_2p_01", "IH"), ("scene_mp_4p_03", "scene")]
SRC = ["astar", "corridor", "lift"]
out = []


def w(line=""):
    out.append(line)


def f(v, nd=0):
    if v is None:
        return "-"
    if isinstance(v, float):
        if v != v:
            return "-"
        return f"{v:,.{nd}f}"
    if isinstance(v, int):
        return f"{v:,}"
    return str(v)


def table(head, rows, align=None):
    align = align or ["l"] + ["r"] * (len(head) - 1)
    w("| " + " | ".join(head) + " |")
    w("|" + "|".join(":---" if a == "l" else "---:" for a in align) + "|")
    for r in rows:
        w("| " + " | ".join(r) + " |")
    w()


def cum_at(name, steps):
    s = S[name]
    total, got, i = 0, [], 0
    pairs = list(zip(s["step"], s["finished_step"]))
    for target in steps:
        while i < len(pairs) and pairs[i][0] <= target:
            total += pairs[i][1]
            i += 1
        got.append(total)
    return got


def at_decision(xs, ds):
    return [xs[d - 1] if d - 1 < len(xs) else None for d in ds]


def replans(name):
    last, new, rep = {}, 0, 0
    for r in csv.DictReader(gzip.open(D / f"{name}.trace.csv.gz", "rt")):
        if last.get(r["agent"]) == r["goal"]:
            rep += 1
        else:
            new += 1
        last[r["agent"]] = r["goal"]
    return rep, new + rep


def budget(name, skip=50):
    s = S[name]
    k = len(s["sched_ms"])
    a = min(skip, k)
    m = lambda key: sum(s[key][a:]) / max(1, len(s[key][a:]))
    b = {key: m(key) for key in ["sched_ms", "guide_ms", "fw_ms", "pibt_ms"]}
    return b


w("# Planner overview runs: results tables")
w()
w("Overnight 2026-10-01/02, 21 runs, `run.sh` in this folder. All runs: solver 6 (`--scheduleModel 6`), corridor margin 0,")
w("corridor congestion off, `--computeGuidePaths false`, one run at a time. Every run exited 0 with 0 planner errors and")
w("0 hierarchy fallbacks; the memory watchdog never fired. Run notes: `overnight_notes.txt`. Regenerate with")
w("`python3 summarise.py && python3 make_results_md.py`.")
w()
w("**Terms**")
w()
w("- *Long path*: a guide path whose start-goal Manhattan distance is at least 500 cells (delivery legs, mostly).")
w("- *No path*: agents without a guide path at that decision (they fall back to Manhattan moves).")
w("- *Backlog clear*: first decision after which the no-path count stays at or under 1% of agents.")
w("- *Replans*: paths built for the same goal the agent already had (it was pushed too far off its path).")
w("- *Stretch (M)*: path cells / start-goal Manhattan distance. Manhattan is a lower bound, so compare between runs,")
w("  not against 1. The bench tables (part D) use the true shortest distance instead.")
w("- Times are wall-clock ms on this machine (8 cores, 31 GB).")
w()

# ---------------------------------------------------------------- part A
w("## A. Does it help? 10k agents, 1,500 steps, guide-path level 4, solver 6 level 4")
w()
w("### Outcome")
w()
rows = []
for m, short in MAPS:
    for s in SRC:
        n = f"A_{m}_10000_{s}_L4"
        r = R[n]
        rows.append([f"{short} {s}", f(r["finished"]), f(r["opened"]), f"{r['decisions']:,} / {r['steps']:,}",
                     f(r["errors"]), f(r["fallbacks"]), f(r["peak_gb"], 1), r["elapsed"]])
table(["Run", "Deliveries", "Tasks opened", "Decisions / steps", "Errors", "Fallbacks", "Peak GB", "Wall"], rows)

w("### Agents without a path, and stuck agents")
w()
rows = []
for m, short in MAPS:
    for s in SRC:
        r = R[f"A_{m}_10000_{s}_L4"]
        rows.append([f"{short} {s}", f(r["no_path_peak"]), f(r["no_path_clear"]), f(r["no_path_end"]),
                     f(r["stuck_max"]), f(r["stuck_end"])])
table(["Run", "No path, peak", "Backlog clear (decision)", "No path, end", "Stuck, max", "Stuck, end"], rows)

w("No-path count at selected decisions:")
w()
ds = [1, 2, 10, 25, 50, 100, 200, 500, 1000]
rows = []
for m, short in MAPS:
    for s in SRC:
        n = f"A_{m}_10000_{s}_L4"
        rows.append([f"{short} {s}"] + [f(v) for v in at_decision(S[n]["no_path"], ds)])
table(["Run"] + [f"d{d}" for d in ds], rows)

w("### Guide paths built")
w()
rows = []
for m, short in MAPS:
    for s in SRC:
        n = f"A_{m}_10000_{s}_L4"
        r = R[n]
        rep, tot = replans(n)
        rows.append([f"{short} {s}", f(r["paths"]), f(r["long_paths"]), f(r["ms_p50"], 2), f(r["ms_p90"], 2), f(r["ms_max"], 0),
                     f(r["long_ms_mean"], 2), f(r["long_ms_p90"], 2), f(r["long_coarse_ms"], 2), f(r["long_build_ms"], 2),
                     f(r["long_cells_mean"]), f(r["long_stretch_m"], 3), f"{100 * rep / tot:.1f}%"])
table(["Run", "Paths", "Long paths", "ms p50", "ms p90", "ms max", "Long ms", "Long ms p90", "Long: coarse ms",
       "Long: build ms", "Long cells", "Long stretch (M)", "Replans"], rows)
w("`astar` has no coarse/build split (one full-map A*), so those columns read 0.")
w()

w("### Deliveries over time (cumulative)")
w()
steps = [100, 250, 500, 750, 1000, 1250, 1500]
rows = []
for m, short in MAPS:
    for s in SRC:
        rows.append([f"{short} {s}"] + [f(v) for v in cum_at(f"A_{m}_10000_{s}_L4", steps)])
table(["Run"] + [f"step {x}" for x in steps], rows)

w("### Planner time per decision (means over all decisions)")
w()
rows = []
for m, short in MAPS:
    for s in SRC:
        r = R[f"A_{m}_10000_{s}_L4"]
        rows.append([f"{short} {s}", f(r["sched_ms_mean"]), f(r["sched_ms_max"]), f(r["guide_ms_mean"]), f(r["fw_ms_mean"]),
                     f(r["pibt_ms_mean"]), f(r["pibt_ms_max"]), f(r["pibt_reserve"])])
table(["Run", "Scheduler ms", "Scheduler max", "Guide paths ms", "Frank-Wolfe ms", "PIBT ms", "PIBT max", "PIBT reserve"], rows)
w("Frank-Wolfe fills whatever time is left before the PIBT reserve, so a high value means spare time, not cost.")
w("The scene corridor/lift runs miss about 300 decisions because the scheduler's time grows to about 1 s per decision")
w("late in the run (solver 6 at level 4; known todo item), not because of the planner.")
w()

w("### Per-path time by start-goal distance")
w()
for m, short in MAPS:
    w(f"**{short}**")
    w()
    bins = {s: {b["lo"]: b for b in R[f"A_{m}_10000_{s}_L4"]["bins"]} for s in SRC}
    los = sorted({lo for s in SRC for lo in bins[s]})
    rows = []
    for lo in los:
        b0 = next(bins[s][lo] for s in SRC if lo in bins[s])
        hi = "inf" if b0["hi"] >= 10**9 else f"{b0['hi']:,}"
        row = [f"{lo:,}-{hi}"]
        for s in SRC:
            b = bins[s].get(lo)
            row += [f(b["n"]) if b else "-", f(b["ms_mean"], 2) if b else "-", f(b["stretch_m"], 3) if b else "-"]
        rows.append(row)
    table(["Manhattan distance"] + [f"{s} {c}" for s in SRC for c in ["n", "ms", "stretch (M)"]], rows)

# ---------------------------------------------------------------- part B
w("## B. Scaling: lift, guide-path level 4, solver 6 level 8, 500 steps")
w()
for m, short in MAPS[1:]:
    w(f"### {short}")
    w()
    rows = []
    for n_ in [10000, 20000, 40000, 80000]:
        r = R[f"B_{m}_{n_}_lift_L4"]
        rows.append([f"{n_ // 1000}k", f(r["finished"]), f"{r['decisions']} / {r['steps']}", f(r["no_path_peak"]),
                     f(r["no_path_clear"]), f(r["no_path_end"]), f(r["stuck_max"]), f(r["stuck_end"]), f(r["paths"]),
                     f(r["long_ms_mean"], 2), f(r["long_ms_p90"], 2), f(r["peak_gb"], 1), r["elapsed"]])
    table(["Agents", "Deliveries", "Decisions / steps", "No path, peak", "Backlog clear", "No path, end", "Stuck, max",
           "Stuck, end", "Paths", "Long ms", "Long ms p90", "Peak GB", "Wall"], rows)

    w("Where each 1,000 ms step goes (mean ms per decision, decisions 51 onwards):")
    w()
    rows = []
    for n_ in [10000, 20000, 40000, 80000]:
        n = f"B_{m}_{n_}_lift_L4"
        b = budget(n)
        used = sum(b.values())
        r = R[n]
        rows.append([f"{n_ // 1000}k", f(b["sched_ms"]), f(b["guide_ms"]), f(b["fw_ms"]), f(b["pibt_ms"]),
                     f(r["pibt_ms_max"]), f(r["pibt_reserve"]), f(1000 - used)])
    table(["Agents", "Scheduler", "Guide paths", "Frank-Wolfe", "PIBT used", "PIBT max", "PIBT reserve", "Unused"], rows)
    w("Unused = 1,000 minus the four columns: the unused part of the PIBT reserve plus about 80 ms of fixed tolerances.")
    w()

    w("No-path count at selected decisions:")
    w()
    ds = [1, 10, 25, 50, 100, 200, 300, 400, 490]
    rows = []
    for n_ in [10000, 20000, 40000, 80000]:
        rows.append([f"{n_ // 1000}k"] + [f(v) for v in at_decision(S[f"B_{m}_{n_}_lift_L4"]["no_path"], ds)])
    table(["Agents"] + [f"d{d}" for d in ds], rows)

w("At 80k the 800 ms PIBT reserve (1 ms per 100 agents) leaves stage 2 almost no time; PIBT itself used at most")
w("about 135 ms there, but with most agents on Manhattan moves (cheaper than following a path). At 40k with every")
w("agent on a path it peaked at 268-360 ms, so a linear guess for 80k with paths is about 550-720 ms.")
w()

# ---------------------------------------------------------------- part C
w("## C. Guide-path level: IH 10k, solver 6 level 4, first 500 steps")
w()
w("Level 4 rows are the first 500 steps of the part A runs. Cells and coarse/build split are whole-run means.")
w()
rows = []
for s in ["corridor", "lift"]:
    for lvl in [3, 4, 6]:
        n = f"{'A' if lvl == 4 else 'C'}_IH_mp_2p_01_10000_{s}_L{lvl}"
        se, r = S[n], R[n]
        fin = sum(x for st, x in zip(se["step"], se["finished_step"]) if st <= 500)
        lb = [x for x in se["long_by_step"] if x[0] <= 500]
        cnt = sum(x[1] for x in lb)
        ms = sum(x[1] * x[2] for x in lb) / max(1, cnt)
        rows.append([f"{s} L{lvl}", f(fin), f(cnt), f(ms, 2), f(r["long_coarse_ms"], 2), f(r["long_build_ms"], 2),
                     f(r["long_cells_mean"]), f(r["no_path_clear"]), f(r["stuck_max"])])
table(["Run", "Deliveries by step 500", "Long paths", "Long ms", "Coarse ms", "Build ms", "Long cells",
       "Backlog clear", "Stuck, max"], rows)

# ---------------------------------------------------------------- part D
w("## D. Path length / shortest, by shortest distance (bench, no planner)")
w()
w("From `../hierarchy_lift_bench/coarse_astar/` (random agent-start to task pairs, 300 on orz900d and IH, 200 on")
w("scene). Shortest = BFS on the fine map. Mean per bin, p90 in brackets. Corridor = margin 0.")
w()
BINS = [0, 100, 300, 1000, 2000, 4000, 10**9]


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float("nan")


for name, short in [("orz900d", "orz900d"), ("ih", "IH"), ("scene", "scene")]:
    rows_all = list(csv.DictReader(open(BENCH / f"{name}.csv")))
    levels = sorted({int(r["level"]) for r in rows_all})
    w(f"**{short}**")
    w()
    rows = []
    for lo, hi in zip(BINS, BINS[1:]):
        bin_rows = [r for r in rows_all if lo <= int(r["true_dist"]) < hi]
        if not bin_rows:
            continue
        hi_s = "inf" if hi >= 10**9 else f"{hi:,}"
        n_pairs = len([r for r in bin_rows if int(r["level"]) == levels[0]])
        row = [f"{lo:,}-{hi_s}", f(n_pairs)]
        for lvl in levels:
            b = [r for r in bin_rows if int(r["level"]) == lvl]
            lift = [int(r["final_len"]) / max(1, int(r["true_dist"])) for r in b if r["valid"] == "1"]
            corr = [int(r["c0_len"]) / max(1, int(r["true_dist"])) for r in b if r["c0_ok"] == "1"]
            row.append(f"{sum(lift) / len(lift):.2f} ({pct(lift, .9):.2f}) / {sum(corr) / len(corr):.2f}")
        rows.append(row)
    table(["Shortest distance", "Pairs"] + [f"L{l} lift / corridor" for l in levels], rows)

# ---------------------------------------------------------------- part E
w("## E. Path length: A* vs corridor vs lift on the same pairs (bench, no planner)")
w()
w("Same bench pairs as part D; every builder gets the same start and goal, so the ratios are paired. A* is the")
w("planner's full-map A* with an empty congestion map, so it finds a shortest path (A* / shortest = 1.000 on every")
w("pair). The planner's own A* runs with congestion and takes longer routes; part A's stretch columns show that, but")
w("those runs have different goals, so they can't be paired like this. Ratios: mean, p90 and max over pairs.")
w()
for name, short in [("orz900d", "orz900d"), ("ih", "IH"), ("scene", "scene")]:
    rows_all = list(csv.DictReader(open(BENCH / f"{name}.csv")))
    w(f"**{short}**")
    w()
    rows = []
    for lvl in sorted({int(r["level"]) for r in rows_all}):
        b = [r for r in rows_all if int(r["level"]) == lvl and r["valid"] == "1" and r["c0_ok"] == "1"
             and int(r["fine_len"]) > 0]
        ca = [int(r["c0_len"]) / int(r["fine_len"]) for r in b]
        la = [int(r["final_len"]) / int(r["fine_len"]) for r in b]
        cells = lambda k: sum(int(r[k]) for r in b) / len(b)
        rows.append([f"L{lvl}", f(len(b)), f(cells("fine_len")), f(cells("c0_len")), f(cells("final_len")),
                     f"{sum(ca) / len(ca):.3f}", f"{pct(ca, .9):.2f}", f"{max(ca):.2f}",
                     f"{sum(la) / len(la):.3f}", f"{pct(la, .9):.2f}", f"{max(la):.2f}"])
    table(["Level", "Pairs", "A* cells", "Corridor cells", "Lift cells", "Corridor / A*", "p90", "max",
           "Lift / A*", "p90", "max"], rows)

(D / "RESULTS.md").write_text("\n".join(out))
print(f"wrote RESULTS.md ({len(out)} lines)")
