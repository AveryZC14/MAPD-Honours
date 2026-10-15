#!/usr/bin/env python3
"""Collate the thesis sweep into RESULTS.md (tables only).

Reads sweep_summary.csv (last row per run), each run's JSON, log, guide-path
trace and task trace in this folder. Reuses the per-run summary from
../planner_overview/summarise.py. Rerun any time; runs not yet finished show
as "-".

    python3 make_results.py         # writes RESULTS.md next to this file
"""
import csv, gzip, importlib.util, statistics, sys
from collections import defaultdict
from pathlib import Path

D = Path(__file__).resolve().parent
REPO = D.parent.parent
spec = importlib.util.spec_from_file_location("summarise", D.parent / "planner_overview" / "summarise.py")
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)
S.D = D  # read this folder's runs

MAPS = ["orz900d", "IH_mp_2p_01", "scene_mp_4p_03", "scene_sp_pol_06"]
SHORT = {"orz900d": "orz900d", "IH_mp_2p_01": "IH", "scene_mp_4p_03": "scene_mp", "scene_sp_pol_06": "scene_sp"}
TEAMS = {"orz900d": [10000, 20000]}
DEFAULT_TEAMS = [10000, 40000, 80000]
SOURCES = ["astar", "lift-L4", "corridor-L4", "refine-L4"]
SLICE = ["lift-L3", "lift-L4", "lift-L6", "corridor-L3", "corridor-L4", "corridor-L6", "refine-L4", "refine-L6"]
MIN_GUIDE_MS = 150
SCHEDS = [("s6L2", "level 2"), ("s6L4", "level 4"), ("s6L6", "level 6"), ("s6L8", "level 8"),
          ("s6hier", "hierarchical-only")]


def k(n):
    return f"{n // 1000}k"


def num(v, nd=0):
    if v is None or v == "" or (isinstance(v, float) and v != v):
        return "-"
    v = float(v)
    return f"{v:,.{nd}f}"


def map_cols(map_name):
    for line in (REPO / "instances" / "thesis_benchmarks" / map_name / "maps" / f"{map_name}.map").open():
        if line.startswith("width"):
            return int(line.split()[1])
    raise ValueError(map_name)


def task_metrics(stem, cols):
    """Assignment-quality measures from the task trace (None if missing)."""
    f = D / f"{stem}.tasks.csv.gz"
    if not f.exists():
        return None
    reveal, first_assign, last_assign, pickup_loc = {}, {}, {}, {}
    to_pickup, dist, per_cell, lead, wait = [], [], [], [], []
    assigns = unassigns = 0
    for r in csv.DictReader(gzip.open(f, "rt")):
        t, ev, task = int(r["timestep"]), r["event"], r["task"]
        if ev == "reveal":
            reveal[task] = t
            pickup_loc[task] = int(r["loc"])
        elif ev == "assign":
            assigns += 1
            first_assign.setdefault(task, t)
            # the latest assignment before pickup is the trip that reached it
            last_assign[task] = (t, int(r["loc"]))
        elif ev == "unassign":
            unassigns += 1
        elif ev == "pickup":
            if task in last_assign and task in pickup_loc:
                t0, loc = last_assign[task]
                p = pickup_loc[task]
                m = abs(loc // cols - p // cols) + abs(loc % cols - p % cols)
                to_pickup.append(t - t0)
                dist.append(m)
                if m > 0:
                    per_cell.append((t - t0) / m)
        elif ev == "deliver":
            if task in reveal:
                lead.append(t - reveal[task])
    for task, t in first_assign.items():
        if task in reveal:
            wait.append(t - reveal[task])
    med = lambda xs: statistics.median(xs) if xs else None
    mean = lambda xs: sum(xs) / len(xs) if xs else None
    return {"wait": mean(wait), "dist": mean(dist), "to_pickup": mean(to_pickup),
            "per_cell": med(per_cell), "lead": mean(lead), "lead_n": len(lead),
            "reassign_pct": 100 * unassigns / assigns if assigns else None}


def guide_stats(stem):
    """Paths per decision and the time guide paths had, from the planner stats lines."""
    text = S.read(D / f"{stem}.log")
    if text is None:
        return {}
    stats, sched, _ = S.parse_log(text)
    if not stats:
        return {}
    avail, at_limit = [], 0
    for st, sc in zip(stats, sched):
        # deadline: 1 s - 20 ms tolerance - scheduler - PIBT reserve - 60 ms - setup,
        # but never under the --minGuidePathMs guarantee
        a = max(MIN_GUIDE_MS, 1000 - 20 - sc - st.get("pibt_reserve_ms", 0) - 60 - st.get("setup_ms", 0))
        avail.append(a)
        if st.get("guide_ms", 0) >= 0.97 * a:
            at_limit += 1
    paths = [st.get("paths_built", 0) for st in stats]
    late = paths[-500:]
    return {"paths_dec": sum(paths) / len(paths), "paths_late": sum(late) / len(late),
            "avail": sum(avail) / len(avail), "at_limit": 100 * at_limit / len(avail)}


def main():
    rows = {}
    for r in csv.DictReader(open(D / "sweep_summary.csv")):
        rows[r["run"]] = r  # last row per run wins (reruns)
    summ, tasks = {}, {}
    for stem in rows:
        res = S.summarise_run(stem)
        summ[stem] = res[0] if res else {}
        if stem.startswith("E1_"):
            summ[stem].update(guide_stats(stem))
        print("summarised", stem, file=sys.stderr)
    cols = {m: map_cols(m) for m in MAPS}

    def get(stem, key):
        return summ.get(stem, {}).get(key)

    def tm(stem, m):
        if stem not in tasks:
            tasks[stem] = task_metrics(stem, cols[m]) if stem in rows else None
        return tasks[stem]

    def dec(stem):
        r = rows.get(stem)
        if not r:
            return "-"
        d, s = int(r["decisions"]), int(r["steps"])
        return num(d) if d == s else f"**{d:,}**"

    out = []
    w = out.append
    w("# Thesis sweep results\n")
    w("Generated by `make_results.py` from `sweep_summary.csv` and each run's JSON, log and traces. "
      "Plan and flags: `instances/thesis_benchmarks/README.md`, \"Revised run plan\". Machine: one run at a "
      "time, 1 s per decision; every run uses `--minGuidePathMs 150`.\n")
    w("## Definitions\n")
    w("Each step has 1 s for the scheduler and planner together (a *decision*). Solver 6 is written s6, at "
      "level 6 unless stated. Run-to-run noise is under 0.5% (repeats, below), so differences of 1-2% are "
      "real.\n")
    for line in [
        "**Deliveries**: tasks finished in the run.",
        "**Decisions**: decisions made; **bold** when fewer than the steps, i.e. a decision ran over 1 s and "
        "every agent waited that step.",
        "**Backlog clear**: first decision after which at most 1% of agents lack a guide path for the rest "
        "of the run (`-` = never). Agents without a path make plain Manhattan moves and can get trapped.",
        "**Stuck end**: agents that hadn't moved for 20+ decisions, at the last decision.",
        "**Sched ms**: mean scheduler time per decision.",
        "**Guide ms / avail**: mean time spent building guide paths per decision, against the mean time "
        "available for them (1 s minus scheduler, PIBT reserve of 1 ms per 100 agents, about 80 ms of "
        "margins and setup, but at least the 150 ms guarantee).",
        "**At limit**: share of decisions where guide-path building used up its available time (paths still "
        "needed were left for later decisions).",
        "**Paths/dec**: guide paths built per decision, whole run (last 500 decisions). Agents need one on a "
        "new goal (new task, or pickup -> delivery) or when pushed too far off their path. When time is at "
        "the limit, this is the number that matters.",
        "**PIBT ms (max)**: mean (worst) time to choose every agent's move. PIBT isn't time-limited; near "
        "or above about 800 ms decisions run late.",
        "**Long-path ms / cells**: mean build time and length of one guide path whose start and goal are "
        "500+ cells apart (mostly delivery legs). Compare lengths only within one instance.",
        "**Peak GB**: peak memory of the run (31 GB machine).",
    ]:
        w(f"- {line}")
    w("")
    sup = sorted(p.name.split(".")[0] for p in (D / "superseded").glob("*.json")) if (D / "superseded").exists() else []
    if sup:
        w("Superseded runs (rerun; originals in `superseded/`): " + ", ".join(f"`{s}`" for s in sup) + ".\n")

    # ---- E1
    w("## E1: planner (scheduler s6 level 6, 1,500 steps)\n")
    w("### Deliveries\n")
    w("| Instance | " + " | ".join(SOURCES) + " | Best |")
    w("|---|" + "---:|" * len(SOURCES) + "---|")
    for m in MAPS:
        for t in TEAMS.get(m, DEFAULT_TEAMS):
            vals = {}
            cells = []
            for src in SOURCES:
                r = rows.get(f"E1_{m}_{t}_s6L6_{src}")
                v = int(r["tasksFinished"]) if r else None
                vals[src] = v
                cells.append(num(v))
            best = max((v, s) for s, v in vals.items() if v is not None)[1] if any(vals.values()) else "-"
            w(f"| {SHORT[m]} {k(t)} | " + " | ".join(cells) + f" | {best} |")
    w("\nfull-map A* (`astar`) runs at 10k only.\n")

    w("### Refine and lift relative to corridor (deliveries)\n")
    w("| Instance | refine vs corridor | lift vs corridor |")
    w("|---|---:|---:|")
    for m in MAPS:
        for t in TEAMS.get(m, DEFAULT_TEAMS):
            c = rows.get(f"E1_{m}_{t}_s6L6_corridor-L4")
            if not c:
                continue
            cv = int(c["tasksFinished"])
            rel = []
            for src in ["refine-L4", "lift-L4"]:
                r = rows.get(f"E1_{m}_{t}_s6L6_{src}")
                rel.append(f"{100 * (int(r['tasksFinished']) / cv - 1):+.1f}%" if r else "-")
            w(f"| {SHORT[m]} {k(t)} | " + " | ".join(rel) + " |")
    w("")

    w("### Per run: decisions, backlog, timing, paths, memory\n")
    w("| Run | Decisions | Backlog clear | Stuck end | Sched ms | Guide ms / avail | At limit | "
      "Paths/dec (last 500) | PIBT ms (max) | Long-path ms | Long-path cells | Peak GB |")
    w("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for m in MAPS:
        for t in TEAMS.get(m, DEFAULT_TEAMS):
            for src in SLICE + ["astar"]:
                stem = f"E1_{m}_{t}_s6L6_{src}"
                if stem not in rows:
                    continue
                w(f"| {SHORT[m]} {k(t)} {src} | {dec(stem)} | {num(get(stem, 'no_path_clear'))} | "
                  f"{num(get(stem, 'stuck_end'))} | {num(get(stem, 'sched_ms_mean'))} | "
                  f"{num(get(stem, 'guide_ms_mean'))} / {num(get(stem, 'avail'))} | "
                  f"{num(get(stem, 'at_limit'))}% | {num(get(stem, 'paths_dec'))} "
                  f"({num(get(stem, 'paths_late'))}) | {num(get(stem, 'pibt_ms_mean'))} "
                  f"({num(get(stem, 'pibt_ms_max'))}) | {num(get(stem, 'long_ms_mean'), 2)} | "
                  f"{num(get(stem, 'long_cells_mean'))} | {num(rows[stem]['peak_rss_gb'], 1)} |")
    w("")

    w("### Level slice at 80k (deliveries)\n")
    w("| Instance | " + " | ".join(SLICE) + " |")
    w("|---|" + "---:|" * len(SLICE))
    for m, t in [("IH_mp_2p_01", 80000), ("scene_mp_4p_03", 80000)]:
        cells = []
        for src in SLICE:
            r = rows.get(f"E1_{m}_{t}_s6L6_{src}")
            cells.append(num(r["tasksFinished"]) + ("" if not r or r["decisions"] == r["steps"]
                                                     else f" ({int(r['decisions']):,} dec)") if r else "-")
        w(f"| {SHORT[m]} {k(t)} | " + " | ".join(cells) + " |")
    w("")

    w("### Assignment and travel (task trace)\n")
    w("| Run | Wait for assignment | Distance to pickup | Steps to pickup | Steps per cell | "
      "Lead time (n) | Reassigned |")
    w("|---|---:|---:|---:|---:|---:|---:|")
    for m in MAPS:
        for t in TEAMS.get(m, DEFAULT_TEAMS):
            for src in SOURCES:
                stem = f"E1_{m}_{t}_s6L6_{src}"
                x = tm(stem, m)
                if not x:
                    continue
                w(f"| {SHORT[m]} {k(t)} {src} | {num(x['wait'], 1)} | {num(x['dist'])} | {num(x['to_pickup'])} | "
                  f"{num(x['per_cell'], 2)} | {num(x['lead'])} ({num(x['lead_n'])}) | {num(x['reassign_pct'], 1)}% |")
    w("\nMeans in timesteps or cells. *Wait for assignment*: task appears -> first assigned. *Distance to "
      "pickup*: Manhattan, agent to pickup when assigned. *Steps to pickup*: assigned -> picked up. *Steps "
      "per cell*: median of steps to pickup / Manhattan distance (1 = straight there; walls, detours and "
      "waiting add). *Lead time*: task appears -> delivered, over delivered tasks only (n), so it favours "
      "short tasks in short runs. *Reassigned*: unassignments per 100 assignments.\n")
    w("**Reading this table.** The instances keep 1.5 open tasks per agent, so a free agent almost always "
      "has a task a few cells away: pickup legs are short and nearly straight (median 1.00 steps per cell; "
      "lift's 1.40 at 10k is its detours on short paths), and the distance is in the delivery leg (lead "
      "time). *Wait for assignment* mostly reflects the surplus of tasks over agents. No solver reassigned "
      "a task in E1 (0%). Assignment quality differences should therefore show up in E2 between schedulers "
      "rather than here between planners.\n")

    # ---- E3
    w("## E3: scheduler level x planner (20k, 1,500 steps)\n")
    for m in ["IH_mp_2p_01", "scene_mp_4p_03"]:
        w(f"### {SHORT[m]} 20k\n")
        w("| Scheduler | corridor-L4 deliveries | decisions | sched ms (max) | refine-L4 deliveries | decisions | "
          "sched ms (max) |")
        w("|---|---:|---:|---:|---:|---:|---:|")
        for sc, label in SCHEDS:
            cells = []
            for g in ["corridor-L4", "refine-L4"]:
                stem = f"E3_{m}_20000_{sc}_{g}"
                r = rows.get(stem)
                cells += [num(r["tasksFinished"]) if r else "-", dec(stem),
                          f"{num(get(stem, 'sched_ms_mean'))} ({num(get(stem, 'sched_ms_max'))})"]
            w(f"| {label} | " + " | ".join(cells) + " |")
        w("")

    # ---- repeats
    w("## Repeats (s6 level 6, 10k, 1,500 steps)\n")
    w("| Config | Run 1 (E1) | Run 2 | Run 3 | Spread |")
    w("|---|---:|---:|---:|---:|")
    for m in ["IH_mp_2p_01", "scene_mp_4p_03"]:
        for g in ["corridor-L4", "refine-L4"]:
            stems = [f"E1_{m}_10000_s6L6_{g}", f"R_{m}_10000_s6L6_{g}_r2", f"R_{m}_10000_s6L6_{g}_r3"]
            v = [int(rows[s]["tasksFinished"]) if s in rows else None for s in stems]
            got = [x for x in v if x is not None]
            spread = f"{100 * (max(got) - min(got)) / statistics.mean(got):.2f}%" if len(got) > 1 else "-"
            w(f"| {SHORT[m]} {g} | " + " | ".join(num(x) for x in v) + f" | {spread} |")
    w("\n*Spread*: (max - min) / mean.\n")

    # ---- E2
    e2 = [s for s in rows if s.startswith("E2_")]
    w("## E2: scheduler (4,000 steps)\n")
    w(f"{len(e2)} of 94 runs finished." + ("" if e2 else " Not started: waits for `E2_PLANNER`.") + "\n")

    (D / "RESULTS.md").write_text("\n".join(out) + "\n")
    print("wrote", D / "RESULTS.md", file=sys.stderr)


if __name__ == "__main__":
    main()
