#!/usr/bin/env python3
"""
Run the thesis benchmark sweep: the run lists in
instances/thesis_benchmarks/README.md ("Revised run plan", 2026-10-07).

Experiments (--experiments, default E1 E3 R, plus E2 once its planner is set):
  E1  planner: solver 6 level 6 fixed; guide-path source at level 4 on all 11
      instances, full-map A* at 10k, and a level slice at 80k. 1,500 steps.
  E3  interaction: solver 6 levels 2/4/6/8 + hierarchical-only x corridor L4 /
      refine L4 on IH 20k and scene_mp_4p_03 20k. 1,500 steps.
  R   repeats: corridor L4 and refine L4 (solver 6 level 6) on IH 10k and
      scene_mp_4p_03 10k, two more runs each (E1 has the first). 1,500 steps.
  E2  scheduler: solver 6 levels 2/6/8 + hierarchical-only, Greedy, solver 1
      (10k/20k), solver 7 (2 runs), with the planner chosen from E1. 4,000
      steps. Only planned once <out-dir>/E2_PLANNER exists, holding e.g.
      "refine 4" (source and level), or with --e2-planner; so a watchdog
      restart picks it up without arguments.

Every run: --minGuidePathMs 150, --computeGuidePaths false, corridor margin 0,
congestion off, --hierarchyCache for every solver, --guidePathTrace and
--taskTrace (log and traces gzipped after the run).

Runs are strictly one at a time: results depend on wall-clock planning time.
Each run records other heavy jobs (lifelong / bench processes not its own)
seen while it ran, in sweep_summary.csv (other_jobs_seen), so interference
shows up in the results.

Resumable: a run is skipped if its output JSON already exists and parses.
lifelong writes the JSON only when it finishes, so an interrupted run leaves
no JSON and is redone.

Examples:

  python3 scripts/run_thesis_sweep.py --dry-run          # list the runs
  scripts/sweep_watchdog.sh                              # start / resume, detached
  python3 scripts/run_thesis_sweep.py --experiments E1 --maps IH_mp_2p_01 --teams 10000
  echo "refine 4" > outputs/thesis_sweep/E2_PLANNER      # enable E2 after E1
"""
import argparse
import csv
import json
import os
import re
import signal
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
INSTANCE_ROOT = REPO_ROOT / "instances" / "thesis_benchmarks"

LOG_DETAIL_LEVEL = 3          # fatal errors only; per-task log lines would fill the disk
RUN_TIMEOUT_S = 6 * 3600      # the longest runs (4,000 steps) take ~1.2 h; past 6 h is hung
MIN_FREE_DISK_GB = 2.0
MIN_GUIDE_PATH_MS = 150

# map -> (team sizes, hierarchy cache, --preprocessTimeLimit, top hierarchy level)
MAPS = {
    "orz900d":         ([10000, 20000],               "orz900d_full",          600000,  11),
    "IH_mp_2p_01":     ([10000, 20000, 40000, 80000], "IH_mp_2p_01_fixpoint",  600000,  11),
    "scene_mp_4p_03":  ([10000, 20000, 40000, 80000], "scene_mp_4p_03_full",   1800000, 12),
    "scene_sp_pol_06": ([10000, 20000, 40000, 80000], "scene_sp_pol_06_full",  1800000, 13),
}
EXPERIMENTS = ["E1", "E3", "R", "E2"]

E1_STEPS = E3_STEPS = R_STEPS = 1500
E2_STEPS = 4000
E1_SCHED_LEVEL = 6
E1_TEAMS = {"orz900d": [10000, 20000]}            # other maps: 10k, 40k, 80k
E1_DEFAULT_TEAMS = [10000, 40000, 80000]
E1_SOURCES = [("lift", 4), ("corridor", 4), ("refine", 4)]
E1_SLICE = {("IH_mp_2p_01", 80000), ("scene_mp_4p_03", 80000)}
E1_SLICE_SOURCES = [("lift", 3), ("lift", 6), ("corridor", 3), ("corridor", 6), ("refine", 6)]
E3_INSTANCES = [("IH_mp_2p_01", 20000), ("scene_mp_4p_03", 20000)]
E3_LEVELS = [2, 4, 6, 8]
E3_PLANNERS = [("corridor", 4), ("refine", 4)]
R_INSTANCES = [("IH_mp_2p_01", 10000), ("scene_mp_4p_03", 10000)]
R_PLANNERS = [("corridor", 4), ("refine", 4)]
R_COPIES = [2, 3]                                  # copy 1 is the E1 run
E2_LEVELS = [2, 6, 8]
SOLVER1_TEAMS = {10000, 20000}
SOLVER7_RUNS = [("IH_mp_2p_01", 20000), ("scene_sp_pol_06", 20000)]
SOLVER7_LEVEL = 6


def make_run(exp, map_name, team, steps, sched, guide, copy=None):
    """sched: ("s6", level) | ("hier",) | ("s5",) | ("s1",) | ("s7", level); guide: (source, level)."""
    teams, cache, preprocess, top = MAPS[map_name]
    run = {"experiment": exp, "map": map_name, "team": team, "steps": steps, "cache": cache,
           "preprocess": preprocess, "guide_source": guide[0], "guide_level": guide[1]}
    kind = sched[0]
    if kind == "s6":
        run.update(config="solver6", solver=6, level=sched[1], sched_name=f"s6L{sched[1]}")
    elif kind == "hier":
        run.update(config="solver6_hieronly", solver=6, level=top, min_cascade=1, sched_name="s6hier")
    elif kind == "s5":
        run.update(config="solver5", solver=5, level=None, sched_name="s5")
    elif kind == "s1":
        run.update(config="solver1", solver=1, level=None, sched_name="s1")
    elif kind == "s7":
        run.update(config="solver7", solver=7, level=sched[1], sched_name=f"s7L{sched[1]}")
    gp = guide[0] if guide[0] == "astar" else f"{guide[0]}-L{guide[1]}"
    run["guide_path"] = gp
    run["stem"] = f"{exp}_{map_name}_{team}_{run['sched_name']}_{gp}" + (f"_r{copy}" if copy else "")
    return run


def plan_e1():
    runs = []
    for team in sorted({t for m in MAPS for t in E1_TEAMS.get(m, E1_DEFAULT_TEAMS)}):
        for m in MAPS:
            if team not in E1_TEAMS.get(m, E1_DEFAULT_TEAMS):
                continue
            if team == 10000:
                runs.append(make_run("E1", m, team, E1_STEPS, ("s6", E1_SCHED_LEVEL), ("astar", 0)))
            for g in E1_SOURCES:
                runs.append(make_run("E1", m, team, E1_STEPS, ("s6", E1_SCHED_LEVEL), g))
            if (m, team) in E1_SLICE:
                for g in E1_SLICE_SOURCES:
                    runs.append(make_run("E1", m, team, E1_STEPS, ("s6", E1_SCHED_LEVEL), g))
    return runs


def plan_e3():
    runs = []
    for m, team in E3_INSTANCES:
        for g in E3_PLANNERS:
            for level in E3_LEVELS:
                runs.append(make_run("E3", m, team, E3_STEPS, ("s6", level), g))
            runs.append(make_run("E3", m, team, E3_STEPS, ("hier",), g))
    return runs


def plan_r():
    return [make_run("R", m, team, R_STEPS, ("s6", E1_SCHED_LEVEL), g, copy=c)
            for c in R_COPIES for m, team in R_INSTANCES for g in R_PLANNERS]


def plan_e2(guide):
    runs = []
    for team in sorted({t for m in MAPS for t in MAPS[m][0]}):
        for m in MAPS:
            if team not in MAPS[m][0]:
                continue
            for level in E2_LEVELS:
                runs.append(make_run("E2", m, team, E2_STEPS, ("s6", level), guide))
            runs.append(make_run("E2", m, team, E2_STEPS, ("hier",), guide))
            runs.append(make_run("E2", m, team, E2_STEPS, ("s5",), guide))
            if team in SOLVER1_TEAMS:
                runs.append(make_run("E2", m, team, E2_STEPS, ("s1",), guide))
            if (m, team) in SOLVER7_RUNS:
                runs.append(make_run("E2", m, team, E2_STEPS, ("s7", SOLVER7_LEVEL), guide))
    return runs


def read_e2_planner(out_dir: Path, arg):
    """(source, level) for E2 from --e2-planner or <out-dir>/E2_PLANNER, else None."""
    text = arg
    if text is None:
        f = out_dir / "E2_PLANNER"
        if not f.exists():
            return None
        text = f.read_text()
    parts = text.replace(":", " ").split()
    if len(parts) != 2 or parts[0] not in ("astar", "lift", "corridor", "refine") or not parts[1].isdigit():
        sys.exit(f"error: E2 planner must be '<source> <level>', e.g. 'refine 4', not {text!r}")
    return (parts[0], int(parts[1]))


def plan_runs(experiments, e2_guide):
    runs = []
    for exp in EXPERIMENTS:          # fixed order: E1, E3, R, E2
        if exp not in experiments:
            continue
        if exp == "E1":
            runs += plan_e1()
        elif exp == "E3":
            runs += plan_e3()
        elif exp == "R":
            runs += plan_r()
        elif exp == "E2" and e2_guide is not None:
            runs += plan_e2(e2_guide)
    return runs


def build_command(binary: Path, run: dict, out_dir: Path, sim_time) -> list[str]:
    stem = run["stem"]
    instance = INSTANCE_ROOT / run["map"] / f"{run['map']}_{run['team']}.json"
    cache = str(REPO_ROOT / "hierarchy_cache" / f"{run['cache']}.hierarchy")
    cmd = [str(binary), "--inputFile", str(instance), "-o", str(out_dir / f"{stem}.json"),
           "--scheduleModel", str(run["solver"]),
           "-s", str(sim_time if sim_time else run["steps"]),
           "--preprocessTimeLimit", str(run["preprocess"]),
           "--logDetailLevel", str(LOG_DETAIL_LEVEL),
           # every solver gets the hierarchy: the planner's guide paths use it
           "--hierarchyCache", cache,
           "--computeGuidePaths", "false",
           "--guidePathSource", run["guide_source"],
           "--guidePathLevel", str(max(1, run["guide_level"])),
           "--guidePathCorridorMargin", "0",
           "--guidePathCorridorCongestion", "false",
           "--minGuidePathMs", str(MIN_GUIDE_PATH_MS),
           "--guidePathTrace", str(out_dir / f"{stem}.trace.csv"),
           "--taskTrace", str(out_dir / f"{stem}.tasks.csv")]
    if run["solver"] in (6, 7):
        cmd += ["--flowSolveLevel", str(run["level"])]
    if run.get("min_cascade") is not None:
        cmd += ["--minCascadeLevel", str(run["min_cascade"])]
    return cmd


def other_heavy_jobs(own_pids: set) -> set:
    """PIDs of lifelong / bench processes that aren't this run's."""
    found = set()
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) in own_pids:
            continue
        try:
            name = (d / "comm").read_text().strip()
        except OSError:
            continue
        if name in ("lifelong", "bench_hierarchy") or name.startswith("bench_hierarch"):
            found.add(int(d.name))
    return found


def child_pids(pid: int) -> set:
    """pid and every descendant (the /usr/bin/time wrapper's lifelong)."""
    pids, frontier = {pid}, [pid]
    while frontier:
        p = frontier.pop()
        try:
            kids = Path(f"/proc/{p}/task/{p}/children").read_text().split()
        except OSError:
            continue
        for k in kids:
            pids.add(int(k)); frontier.append(int(k))
    return pids


def gzip_outputs(out_dir: Path, stem: str):
    for suffix in (".log", ".trace.csv", ".tasks.csv"):
        f = out_dir / f"{stem}{suffix}"
        if f.exists():
            subprocess.run(["gzip", "-f", str(f)])


def output_is_complete(out_json: Path) -> bool:
    try:
        json.loads(out_json.read_text())
        return True
    except (OSError, json.JSONDecodeError):
        return False


def read_result(out_json: Path) -> dict:
    try:
        data = json.loads(out_json.read_text())
    except (OSError, json.JSONDecodeError) as e:
        return {"ok": False, "note": f"could not parse output JSON: {e}"}
    errors = {k: data.get(k, 0) for k in ("numPlannerErrors", "numScheduleErrors", "numEntryTimeouts")}
    ok = all(v == 0 for v in errors.values())
    return {
        "ok": ok,
        "note": "" if ok else f"nonzero error counts: {errors}",
        "makespan": data.get("makespan"),
        "decisions": len(data.get("timeStepMetrics", [])),
        "tasksOpened": data.get("numTaskOpened"),
        "tasksFinished": data.get("numTaskFinished"),
    }


def peak_rss_gb(time_file: Path):
    try:
        m = re.search(r"Maximum resident set size \(kbytes\): (\d+)", time_file.read_text())
        return round(int(m.group(1)) / 1048576, 2) if m else None
    except OSError:
        return None


def free_disk_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / 1e9


def git_output(*args) -> str:
    try:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def command_output(*args) -> str:
    try:
        return subprocess.run(list(args), capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def cpu_model() -> str:
    try:
        m = re.search(r"^model name\s*:\s*(.+)$", Path("/proc/cpuinfo").read_text(), re.M)
        return m.group(1) if m else "unknown"
    except OSError:
        return "unknown"


def mem_total_gb():
    try:
        m = re.search(r"^MemTotal:\s*(\d+) kB", Path("/proc/meminfo").read_text(), re.M)
        return round(int(m.group(1)) / 1048576, 1) if m else None
    except OSError:
        return None


def write_meta(out_dir: Path, binary: Path, sim_time, experiments, e2_guide):
    const_h = (REPO_ROOT / "default_planner" / "const.h").read_text()
    def const(pattern):
        m = re.search(pattern, const_h)
        return m.group(1) if m else "unknown"

    meta = {
        "started": datetime.now().isoformat(timespec="seconds"),
        "git_commit": git_output("rev-parse", "HEAD"),
        "git_dirty_files": git_output("status", "--porcelain", "--untracked-files=no").splitlines(),
        "binary_mtime": datetime.fromtimestamp(binary.stat().st_mtime).isoformat(timespec="seconds"),
        "USE_MANHATTAN_HEURISTIC": const(r"USE_MANHATTAN_HEURISTIC\s*=\s*(\w+)"),
        # Planner fix, see ai/planner_local_bfs_plan.md. This is the const.h
        # default; a build with -DPLANNER_USE_LOCAL_PATH_BFS=... would differ,
        # but compile.sh doesn't pass one.
        "USE_LOCAL_PATH_BFS": const(r"#define PLANNER_USE_LOCAL_PATH_BFS\s+(\w+)"),
        "LOCAL_PATH_BFS_RADIUS": const(r"LOCAL_PATH_BFS_RADIUS\s*=\s*(\w+)"),
        "LOCAL_PATH_BFS_EXTRA_LAYERS": const(r"LOCAL_PATH_BFS_EXTRA_LAYERS\s*=\s*(\w+)"),
        "PASS_SCHEDULER_PATHS_TO_PLANNER": const(r"#define PLANNER_PASS_SCHEDULER_PATHS\s+(\w+)"),
        "GUIDE_PATH_THREADS": const(r"#define PLANNER_GUIDE_PATH_THREADS\s+(\w+)"),
        "GUIDE_PATH_DEBUG_CHECKS": const(r"#define PLANNER_GUIDE_PATH_DEBUG_CHECKS\s+(\w+)"),
        "experiments": experiments,
        "e2_planner": e2_guide,
        "min_guide_path_ms": MIN_GUIDE_PATH_MS,
        # Results depend on wall-clock planning time, so record the machine.
        "cpu_model": cpu_model(),
        "cpu_count": os.cpu_count(),
        "mem_total_gb": mem_total_gb(),
        "compiler": command_output("c++", "--version").splitlines()[:1],
        "sim_time_override": sim_time,
        "log_detail_level": LOG_DETAIL_LEVEL,
    }
    meta_path = out_dir / "sweep_meta.json"
    # Keep one entry per sweep start, so a resumed sweep records its build too.
    history = []
    if meta_path.exists():
        try:
            history = json.loads(meta_path.read_text())
        except json.JSONDecodeError:
            pass
    history.append(meta)
    meta_path.write_text(json.dumps(history, indent=2) + "\n")


SUMMARY_FIELDS = ["run", "experiment", "map", "team", "config", "solver", "level", "guide_path", "steps",
                  "other_jobs_seen", "exit_code", "wall_clock_s",
                  "peak_rss_gb", "ok", "note", "makespan", "decisions", "tasksOpened", "tasksFinished",
                  "finished_at"]


def append_summary(summary_path: Path, row: dict):
    new_file = not summary_path.exists()
    with summary_path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        if new_file:
            w.writeheader()
        w.writerow(row)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", default="outputs/thesis_sweep", help="output directory (default: outputs/thesis_sweep)")
    p.add_argument("--experiments", nargs="+", choices=EXPERIMENTS, default=EXPERIMENTS,
                   help="experiments to run (default all; E2 only once its planner is set)")
    p.add_argument("--e2-planner", help="E2's planner as '<source> <level>' (overrides <out-dir>/E2_PLANNER)")
    p.add_argument("--maps", nargs="+", choices=list(MAPS), help="only run these maps")
    p.add_argument("--teams", nargs="+", type=int, help="only run these team sizes")
    p.add_argument("--configs", nargs="+", help="only run these configs (solver6, solver6_hieronly, solver5, solver1, solver7)")
    p.add_argument("--only", nargs="+", help="only run these run names (stems)")
    p.add_argument("--binary", default="build/lifelong", help="path to the lifelong binary")
    p.add_argument("--sim-time", type=int, help="override -s for every run (quick tests only, into a separate --out-dir)")
    p.add_argument("--dry-run", action="store_true", help="print the planned commands and exit")
    args = p.parse_args()

    binary = (REPO_ROOT / args.binary).resolve()
    out_dir = (REPO_ROOT / args.out_dir).resolve()
    e2_guide = read_e2_planner(out_dir, args.e2_planner) if "E2" in args.experiments else None

    runs = plan_runs(args.experiments, e2_guide)
    if args.maps:
        runs = [r for r in runs if r["map"] in args.maps]
    if args.teams:
        runs = [r for r in runs if r["team"] in args.teams]
    if args.configs:
        runs = [r for r in runs if r["config"] in args.configs]
    if args.only:
        runs = [r for r in runs if r["stem"] in args.only]
    stems = [r["stem"] for r in runs]
    assert len(stems) == len(set(stems)), "duplicate run names"

    if args.dry_run:
        hours = 0.0
        for i, run in enumerate(runs, 1):
            cmd = build_command(binary, run, out_dir, args.sim_time)
            print(f"[{i}/{len(runs)}] {run['stem']}\n    {' '.join(cmd)}")
            hours += (run["steps"] + 120) / 3600   # ~1 s per step plus loading
        by_exp = {}
        for r in runs:
            by_exp[r["experiment"]] = by_exp.get(r["experiment"], 0) + 1
        print(f"\n{len(runs)} run(s) {by_exp}, about {hours:.0f} h one at a time")
        if "E2" in args.experiments and e2_guide is None:
            print("E2 not planned: no E2 planner set (write e.g. 'refine 4' to <out-dir>/E2_PLANNER)")
        return

    if not binary.exists():
        sys.exit(f"error: binary not found at {binary} (build it with ./compile.sh)")
    for run in runs:
        cache = REPO_ROOT / "hierarchy_cache" / f"{run['cache']}.hierarchy"
        if not cache.exists():
            sys.exit(f"error: hierarchy cache missing: {cache}")
    time_bin = Path("/usr/bin/time")  # GNU time, for peak memory; skipped if absent
    out_dir.mkdir(parents=True, exist_ok=True)
    write_meta(out_dir, binary, args.sim_time, args.experiments, e2_guide)
    summary_path = out_dir / "sweep_summary.csv"

    print(f"{len(runs)} run(s) planned -> {out_dir}", flush=True)
    if "E2" in args.experiments and e2_guide is None:
        print("E2 not planned yet: no E2_PLANNER file", flush=True)
    done = failed = skipped = 0
    for i, run in enumerate(runs, 1):
        stem = run["stem"]
        out_json = out_dir / f"{stem}.json"
        out_log = out_dir / f"{stem}.log"
        time_file = out_dir / f"{stem}.time"
        tag = f"[{i}/{len(runs)}] {stem}"

        if output_is_complete(out_json):
            print(f"{tag}: skipping, output already exists", flush=True)
            skipped += 1
            continue

        free = free_disk_gb(out_dir)
        if free < MIN_FREE_DISK_GB:
            print(f"{tag}: stopping, only {free:.1f} GB disk free (need {MIN_FREE_DISK_GB} GB)", flush=True)
            break

        cmd = build_command(binary, run, out_dir, args.sim_time)
        if time_bin.exists():
            cmd = [str(time_bin), "-v", "-o", str(time_file)] + cmd
        print(f"{tag}: starting at {datetime.now():%Y-%m-%d %H:%M}", flush=True)

        start = time.monotonic()
        others = set()
        with out_log.open("w") as log_f:
            # Own process group, so a timeout kills lifelong itself and not just the
            # /usr/bin/time wrapper (a leftover lifelong would hold its memory and
            # could make the next scene-map run run out of memory).
            proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT, start_new_session=True)
            exit_code = None
            while exit_code is None:
                try:
                    exit_code = proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    others |= other_heavy_jobs(child_pids(proc.pid))
                    if time.monotonic() - start > RUN_TIMEOUT_S:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
                        exit_code = "timeout"
        elapsed = time.monotonic() - start
        gzip_outputs(out_dir, stem)

        row = {"run": stem, "experiment": run["experiment"], "map": run["map"], "team": run["team"],
               "config": run["config"], "solver": run["solver"], "level": run["level"],
               "guide_path": run["guide_path"], "steps": args.sim_time or run["steps"],
               "other_jobs_seen": len(others), "exit_code": exit_code,
               "wall_clock_s": round(elapsed, 1), "peak_rss_gb": peak_rss_gb(time_file),
               "finished_at": datetime.now().isoformat(timespec="seconds")}
        if exit_code != 0:
            row.update(ok=False, note=f"process exit {exit_code}")
        else:
            row.update(read_result(out_json))
        if others and row["ok"]:
            row["note"] = f"{len(others)} other heavy job(s) ran during this run: timing may be affected"
        append_summary(summary_path, row)

        if row["ok"]:
            done += 1
            extra = f"; WARNING {row['note']}" if others else ""
            print(f"{tag}: ok ({elapsed / 3600:.2f} h, {row.get('tasksFinished')} tasks finished){extra}", flush=True)
        else:
            failed += 1
            print(f"{tag}: PROBLEM ({elapsed / 3600:.2f} h) -- {row['note']}; see {out_log}.gz", flush=True)

    print(f"\n{done} ok, {failed} with problems, {skipped} skipped. Summary: {summary_path}", flush=True)


if __name__ == "__main__":
    main()
