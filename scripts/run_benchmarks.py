#!/usr/bin/env python3
"""
Run a grid of ./build/lifelong benchmark invocations in succession.

Replaces the hand-written `for TEAM in ...; do for LEVEL in ...; do ...`
shell loops used in past sweeps (see ai/auto_benchmarking*.md) with one
reusable, resumable script. Output files are named
`<map>_<agents>_solver<N>[_level<L>][_<label>].json`, matching the
convention `visualisation/compute_throughput_metrics.py` already parses
(FILENAME_RE/LABEL_RE) -- point that script at --out-dir afterwards, or
pass --metrics to do it automatically.

Examples:

  # one map, two solvers, three agent counts, four coarsen levels
  # (levels are only applied to solvers 6/7 -- other solvers run once each)
  python3 scripts/run_benchmarks.py \\
      --map scene_sp_pol_06 --agents 10000 20000 60000 \\
      --solvers 6 7 --levels 2 4 6 8 \\
      --sim-time 500 --preprocess-time-limit 1800000 \\
      --hierarchy-cache hierarchy_cache/{map}.hierarchy \\
      --out-dir outputs/scene_sp_pol_06_solver_comparison --metrics

  # see what would run without running it
  python3 scripts/run_benchmarks.py --map orz900d --agents 5000 10000 \\
      --solvers 1 6 --levels 2 4 --out-dir outputs/orz900d_test --dry-run

  # resume an interrupted sweep, skipping runs whose output already exists
  python3 scripts/run_benchmarks.py ... --skip-existing

  # pass through arbitrary extra lifelong flags
  python3 scripts/run_benchmarks.py ... -- --useTraffic 1 --assignNew 1
"""
import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
LEVEL_SOLVERS = {6, 7}  # solvers that understand --flowSolveLevel / benefit from --hierarchyCache


def find_instance(map_name: str, agents: int) -> Path:
    candidates = [
        REPO_ROOT / "instances" / "custom" / map_name / f"{map_name}_{agents}.json",
        REPO_ROOT / "instances" / map_name / f"{map_name}_{agents}.json",
    ]
    for c in candidates:
        if c.exists():
            return c
    tried = "\n  ".join(str(c) for c in candidates)
    sys.exit(f"error: no instance file found for map={map_name!r} agents={agents}. Tried:\n  {tried}")


def output_stem(map_name: str, agents: int, solver: int, level: int | None, label: str | None) -> str:
    parts = [map_name, str(agents), f"solver{solver}"]
    if level is not None:
        parts.append(f"level{level}")
    if label:
        parts.append(label)
    return "_".join(parts)


def build_command(args, binary: Path, instance: Path, out_json: Path, solver: int, level: int | None) -> list[str]:
    cmd = [
        str(binary),
        "--inputFile", str(instance),
        "-o", str(out_json),
        "--scheduleModel", str(solver),
    ]
    if level is not None and solver in LEVEL_SOLVERS:
        cmd += ["--flowSolveLevel", str(level)]
    if args.sim_time is not None:
        cmd += ["-s", str(args.sim_time)]
    if args.plan_time_limit is not None:
        cmd += ["-t", str(args.plan_time_limit)]
    if args.preprocess_time_limit is not None:
        cmd += ["--preprocessTimeLimit", str(args.preprocess_time_limit)]
    if args.hierarchy_cache and solver in LEVEL_SOLVERS:
        cmd += ["--hierarchyCache", args.hierarchy_cache.format(map=args.map)]
    if args.log_detail_level is not None:
        cmd += ["--logDetailLevel", str(args.log_detail_level)]
    cmd += args.extra
    return cmd


def plan_runs(args):
    runs = []
    for solver in args.solvers:
        levels = args.levels if (args.levels and solver in LEVEL_SOLVERS) else [None]
        for level in levels:
            for agents in args.agents:
                runs.append((solver, level, agents))
    return runs


def check_result(out_json: Path) -> dict:
    try:
        data = json.loads(out_json.read_text())
    except (OSError, json.JSONDecodeError) as e:
        return {"ok": False, "note": f"could not parse output JSON: {e}"}
    errors = {
        k: data.get(k, 0)
        for k in ("numPlannerErrors", "numScheduleErrors", "numEntryTimeouts")
    }
    ok = all(v == 0 for v in errors.values())
    return {
        "ok": ok,
        "note": "" if ok else f"nonzero error counts: {errors}",
        "makespan": data.get("makespan"),
        "tasksFinished": data.get("numTaskFinished"),
    }


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--map", required=True, help="map/instance-set name, e.g. scene_sp_pol_06")
    p.add_argument("--agents", type=int, nargs="+", required=True, help="agent counts to sweep (instance suffixes)")
    p.add_argument("--solvers", type=int, nargs="+", required=True, help="--scheduleModel values to sweep")
    p.add_argument("--levels", type=int, nargs="*", default=None,
                    help="--flowSolveLevel values to sweep; only applied to solvers 6/7, ignored for others")
    p.add_argument("--sim-time", type=int, default=None, help="-s / --simulationTime")
    p.add_argument("--plan-time-limit", type=int, default=None, help="-t / --planTimeLimit (ms)")
    p.add_argument("--preprocess-time-limit", type=int, default=None, help="--preprocessTimeLimit (ms)")
    p.add_argument("--hierarchy-cache", default=None,
                    help="--hierarchyCache path template, only passed for solvers 6/7. "
                         "'{map}' is substituted, e.g. hierarchy_cache/{map}.hierarchy")
    p.add_argument("--log-detail-level", type=int, default=3, help="--logDetailLevel (default 3: fatal only)")
    p.add_argument("--label", default=None, help="extra filename suffix, e.g. nolocalmatch")
    p.add_argument("--out-dir", required=True, help="directory for output JSON + .log files")
    p.add_argument("--binary", default="build/lifelong", help="path to the lifelong binary")
    p.add_argument("--dry-run", action="store_true", help="print planned commands without running them")
    p.add_argument("--skip-existing", action="store_true", help="skip runs whose output JSON already exists")
    p.add_argument("--metrics", action="store_true",
                    help="run visualisation/compute_throughput_metrics.py on --out-dir when the sweep finishes")
    p.add_argument("extra", nargs=argparse.REMAINDER,
                    help="extra args passed through to lifelong verbatim, after a literal '--'")
    args = p.parse_args()
    if args.extra and args.extra[0] == "--":
        args.extra = args.extra[1:]

    binary = (REPO_ROOT / args.binary).resolve()
    if not args.dry_run and not binary.exists():
        sys.exit(f"error: binary not found at {binary} (build it with ./compile.sh, or pass --binary)")

    out_dir = (REPO_ROOT / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = plan_runs(args)
    print(f"planned {len(runs)} run(s) -> {out_dir}")

    summary_rows = []
    for i, (solver, level, agents) in enumerate(runs, 1):
        instance = find_instance(args.map, agents)
        stem = output_stem(args.map, agents, solver, level, args.label)
        out_json = out_dir / f"{stem}.json"
        out_log = out_dir / f"{stem}.log"
        cmd = build_command(args, binary, instance, out_json, solver, level)

        tag = f"[{i}/{len(runs)}] {stem}"
        if args.skip_existing and out_json.exists():
            print(f"{tag}: skipping, output already exists")
            continue

        print(f"{tag}: {' '.join(cmd)}")
        if args.dry_run:
            continue

        start = time.monotonic()
        with out_log.open("w") as log_f:
            proc = subprocess.run(cmd, stdout=log_f, stderr=subprocess.STDOUT)
        elapsed = time.monotonic() - start

        row = {"run": stem, "agents": agents, "solver": solver, "level": level,
               "exit_code": proc.returncode, "wall_clock_s": round(elapsed, 1)}
        if proc.returncode != 0:
            print(f"{tag}: FAILED (exit {proc.returncode}, {elapsed:.1f}s) -- see {out_log}")
            row.update(ok=False, note=f"process exited {proc.returncode}")
        else:
            result = check_result(out_json)
            row.update(result)
            status = "ok" if result["ok"] else "COMPLETED WITH ERRORS"
            print(f"{tag}: {status} ({elapsed:.1f}s)"
                  + (f" -- {result['note']}" if result.get("note") else ""))
        summary_rows.append(row)

    if args.dry_run:
        return

    summary_path = out_dir / "sweep_summary.csv"
    if summary_rows:
        fieldnames = ["run", "agents", "solver", "level", "exit_code", "wall_clock_s",
                      "ok", "note", "makespan", "tasksFinished"]
        with summary_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            w.writeheader()
            w.writerows(summary_rows)
        print(f"wrote {summary_path}")

    failed = [r for r in summary_rows if not r.get("ok", True)]
    print(f"\n{len(summary_rows)} run(s) completed, {len(failed)} with problems.")
    for r in failed:
        print(f"  {r['run']}: {r.get('note', 'unknown problem')}")

    if args.metrics:
        metrics_cmd = [sys.executable, str(REPO_ROOT / "visualisation" / "compute_throughput_metrics.py"),
                        str(out_dir), "-o", str(out_dir / "metrics.csv")]
        print(f"\n{' '.join(metrics_cmd)}")
        subprocess.run(metrics_cmd)


if __name__ == "__main__":
    main()
