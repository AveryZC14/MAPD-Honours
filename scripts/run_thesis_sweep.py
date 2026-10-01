#!/usr/bin/env python3
"""
Run the final thesis benchmark sweep: the fixed 120-run list in
instances/thesis_benchmarks/README.md ("Final run plan").

Runs are strictly one at a time. Results depend on wall-clock planning time,
so parallel runs would affect each other, and the scene maps need up to
~25 GB of the machine's 31 GB.

Order: all 10k runs first (every map and config), then 20k, 40k, 80k. If the
sweep is stopped early, the finished team sizes are complete and comparable.
warehouseXL (LAST_MAPS) is the exception: all of its runs come after every
other map's, so it can be dropped by stopping the sweep before it starts.

Resumable: a run is skipped if its output JSON already exists and parses.
lifelong writes the JSON only when it finishes, so an interrupted run leaves
no JSON and is redone.

Each finished run appends one row to <out-dir>/sweep_summary.csv, so
progress survives an interrupted sweep. <out-dir>/sweep_meta.json records
the git commit and build settings the sweep was started with.

Examples:

  # show the full run list without running anything
  python3 scripts/run_thesis_sweep.py --dry-run

  # run everything (about 11.5 days), in the background
  nohup python3 scripts/run_thesis_sweep.py > outputs/thesis_sweep/runner.log 2>&1 &

  # run only part of the list
  python3 scripts/run_thesis_sweep.py --maps orz900d IH_mp_2p_01 --teams 10000
  python3 scripts/run_thesis_sweep.py --configs solver1 solver5
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

SIM_TIME = 8000
LOG_DETAIL_LEVEL = 3          # fatal errors only; per-task log lines would fill the disk
RUN_TIMEOUT_S = 6 * 3600      # a run should take ~2.3 h; anything past 6 h is hung
MIN_FREE_DISK_GB = 1.0

# map -> (team sizes, hierarchy cache, --preprocessTimeLimit, top hierarchy level)
MAPS = {
    "orz900d":         ([10000, 20000],               "orz900d_full",          600000,  11),
    "warehouseXL":     ([10000, 20000, 40000, 80000], "warehouseXL_full",      900000,  11),
    "IH_mp_2p_01":     ([10000, 20000, 40000, 80000], "IH_mp_2p_01_fixpoint",  600000,  11),
    "scene_mp_4p_03":  ([10000, 20000, 40000, 80000], "scene_mp_4p_03_full",   1800000, 12),
    "scene_sp_pol_06": ([10000, 20000, 40000, 80000], "scene_sp_pol_06_full",  1800000, 13),
}
# Maps whose runs all go at the very end, after every team size of the others.
LAST_MAPS = ["warehouseXL"]
SOLVER6_LEVELS = [2, 4, 6, 8]
SOLVER1_TEAMS = {10000, 20000}
SOLVER7_RUNS = {("IH_mp_2p_01", 20000), ("scene_sp_pol_06", 20000)}
SOLVER7_LEVEL = 4

CONFIG_NAMES = ["solver6", "solver6_hieronly", "solver5", "solver1", "solver7"]


def plan_runs():
    """Return every run as a dict, in execution order."""
    runs = []
    main_maps = [m for m in MAPS if m not in LAST_MAPS]
    for group in (main_maps, LAST_MAPS):
        all_teams = sorted({t for m in group for t in MAPS[m][0]})
        for team in all_teams:
            for map_name in group:
                runs += map_team_runs(map_name, team)
    return runs


def map_team_runs(map_name: str, team: int) -> list[dict]:
    """Every config for one map at one team size, in execution order."""
    teams, cache, preprocess, top = MAPS[map_name]
    if team not in teams:
        return []
    base = {"map": map_name, "team": team, "cache": cache, "preprocess": preprocess}
    runs = []
    for level in SOLVER6_LEVELS:
        runs.append({**base, "config": "solver6", "solver": 6, "level": level,
                     "stem": f"{map_name}_{team}_solver6_level{level}"})
    runs.append({**base, "config": "solver6_hieronly", "solver": 6, "level": top,
                 "min_cascade": 1,
                 "stem": f"{map_name}_{team}_solver6_level{top}_hieronly"})
    runs.append({**base, "config": "solver5", "solver": 5, "level": None,
                 "stem": f"{map_name}_{team}_solver5"})
    if team in SOLVER1_TEAMS:
        runs.append({**base, "config": "solver1", "solver": 1, "level": None,
                     "stem": f"{map_name}_{team}_solver1"})
    if (map_name, team) in SOLVER7_RUNS:
        runs.append({**base, "config": "solver7", "solver": 7, "level": SOLVER7_LEVEL,
                     "stem": f"{map_name}_{team}_solver7_level{SOLVER7_LEVEL}"})
    return runs


def build_command(binary: Path, run: dict, out_json: Path, sim_time: int) -> list[str]:
    instance = INSTANCE_ROOT / run["map"] / f"{run['map']}_{run['team']}.json"
    cmd = [str(binary), "--inputFile", str(instance), "-o", str(out_json),
           "--scheduleModel", str(run["solver"]),
           "-s", str(sim_time),
           "--preprocessTimeLimit", str(run["preprocess"]),
           "--logDetailLevel", str(LOG_DETAIL_LEVEL)]
    if run["solver"] in (6, 7):
        cmd += ["--flowSolveLevel", str(run["level"]),
                "--hierarchyCache", str(REPO_ROOT / "hierarchy_cache" / f"{run['cache']}.hierarchy")]
    if run.get("min_cascade") is not None:
        cmd += ["--minCascadeLevel", str(run["min_cascade"])]
    return cmd


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


def write_meta(out_dir: Path, binary: Path, sim_time: int):
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
        # Results depend on wall-clock planning time, so record the machine.
        "cpu_model": cpu_model(),
        "cpu_count": os.cpu_count(),
        "mem_total_gb": mem_total_gb(),
        "compiler": command_output("c++", "--version").splitlines()[:1],
        "sim_time": sim_time,
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


SUMMARY_FIELDS = ["run", "map", "team", "config", "solver", "level", "exit_code", "wall_clock_s",
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
    p.add_argument("--maps", nargs="+", choices=list(MAPS), help="only run these maps")
    p.add_argument("--teams", nargs="+", type=int, help="only run these team sizes")
    p.add_argument("--configs", nargs="+", choices=CONFIG_NAMES, help="only run these configs")
    p.add_argument("--binary", default="build/lifelong", help="path to the lifelong binary")
    p.add_argument("--sim-time", type=int, default=SIM_TIME,
                   help=f"-s for every run (default {SIM_TIME}; override only for quick tests, into a separate --out-dir)")
    p.add_argument("--dry-run", action="store_true", help="print the planned commands and exit")
    args = p.parse_args()

    runs = plan_runs()
    if args.maps:
        runs = [r for r in runs if r["map"] in args.maps]
    if args.teams:
        runs = [r for r in runs if r["team"] in args.teams]
    if args.configs:
        runs = [r for r in runs if r["config"] in args.configs]

    binary = (REPO_ROOT / args.binary).resolve()
    out_dir = (REPO_ROOT / args.out_dir).resolve()

    if args.dry_run:
        for i, run in enumerate(runs, 1):
            cmd = build_command(binary, run, out_dir / f"{run['stem']}.json", args.sim_time)
            print(f"[{i}/{len(runs)}] {run['stem']}\n    {' '.join(cmd)}")
        print(f"\n{len(runs)} run(s), about {len(runs) * 2.3:.0f} h one at a time")
        return

    if not binary.exists():
        sys.exit(f"error: binary not found at {binary} (build it with ./compile.sh)")
    for run in runs:
        cache = REPO_ROOT / "hierarchy_cache" / f"{run['cache']}.hierarchy"
        if run["solver"] in (6, 7) and not cache.exists():
            sys.exit(f"error: hierarchy cache missing: {cache}")
    time_bin = Path("/usr/bin/time")  # GNU time, for peak memory; skipped if absent
    out_dir.mkdir(parents=True, exist_ok=True)
    write_meta(out_dir, binary, args.sim_time)
    summary_path = out_dir / "sweep_summary.csv"

    print(f"{len(runs)} run(s) planned -> {out_dir}", flush=True)
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

        cmd = build_command(binary, run, out_json, args.sim_time)
        if time_bin.exists():
            cmd = [str(time_bin), "-v", "-o", str(time_file)] + cmd
        print(f"{tag}: starting at {datetime.now():%Y-%m-%d %H:%M}", flush=True)

        start = time.monotonic()
        with out_log.open("w") as log_f:
            # Own process group, so a timeout kills lifelong itself and not just the
            # /usr/bin/time wrapper (a leftover lifelong would hold its memory and
            # could make the next scene-map run run out of memory).
            proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                exit_code = proc.wait(timeout=RUN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                exit_code = "timeout"
        elapsed = time.monotonic() - start

        row = {"run": stem, "map": run["map"], "team": run["team"], "config": run["config"],
               "solver": run["solver"], "level": run["level"], "exit_code": exit_code,
               "wall_clock_s": round(elapsed, 1), "peak_rss_gb": peak_rss_gb(time_file),
               "finished_at": datetime.now().isoformat(timespec="seconds")}
        if exit_code != 0:
            row.update(ok=False, note=f"process exit {exit_code}")
        else:
            row.update(read_result(out_json))
        append_summary(summary_path, row)

        if row["ok"]:
            done += 1
            print(f"{tag}: ok ({elapsed / 3600:.2f} h, {row.get('tasksFinished')} tasks finished)", flush=True)
        else:
            failed += 1
            print(f"{tag}: PROBLEM ({elapsed / 3600:.2f} h) -- {row['note']}; see {out_log}", flush=True)

    print(f"\n{done} ok, {failed} with problems, {skipped} skipped. Summary: {summary_path}", flush=True)


if __name__ == "__main__":
    main()
