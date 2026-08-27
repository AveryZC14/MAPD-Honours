#!/usr/bin/env python3
"""
Compute throughput metrics from lifelong-MAPF result JSON(s).

Background: the simulator's output JSON has two different counters that
both look like "number of timesteps" but measure different things by
design (see ai/auto_benchmarking.md and ai/project_context.md, "Makespan
vs. 'timesteps solved'", for the full writeup):

  - "steps" (len(timeStepMetrics)): one log entry per genuine scheduler
    decision (one entry per pass through the main simulation loop's
    plan() call, src/CompetitionSystem.cpp, BaseSystem::simulate). When
    the planner times out and the simulator has to force several "wait"
    timesteps to catch up, that whole multi-step batch still only adds
    1 log entry -- so this number *undercounts* real elapsed simulated
    time whenever timeouts occur, by design (it's a decision count, not
    a clock).
  - "makespan": the max, over all agents, of a per-agent counter that is
    incremented once per *real* elapsed simulator timestep (including
    forced wait-timesteps). This is the accurate measure of how much
    simulated time actually passed.

Dividing tasksFinished by "steps" gives throughput per genuine solver
decision; dividing by "makespan" gives real-time throughput. These answer
different questions -- use "makespan" for real-time throughput comparisons
across solvers/configs, and "steps" if you want cost-per-decision (e.g.
to compare how much a solver accomplishes each time it actually runs,
independent of how often it times out). This script reports both so you
can compare/plot either.

Usage:
    python3 compute_throughput_metrics.py <result.json>
    python3 compute_throughput_metrics.py <folder_of_results>/
    python3 compute_throughput_metrics.py <folder_of_results>/ -o metrics.csv
    python3 compute_throughput_metrics.py <folder_of_results>/ --recursive

    # Aggregate a curated set of directories (see outputs/dashboard_dirs.txt)
    # into one master CSV/MD -- rerun any time after adding/rerunning sweeps:
    python3 compute_throughput_metrics.py --dirs-file outputs/dashboard_dirs.txt

With -o, a Markdown table is also written alongside the CSV (same path,
.md extension) unless --md gives an explicit path. In --dirs-file mode,
-o/--md default to outputs/dashboard/metrics.csv and .md.

--dirs-file mode also parses map/agents/solver/level/nolocalmatch out of
each file's path and name, using the map names found under instances/
(so it recognizes e.g. "scene_mp_4p_03_10000_solver6_level4.json" without
being confused by the digits inside the map name itself).

Some facts aren't recoverable from the filename at all -- e.g. whether a
run used local node matching, since that's a compile-time flag
(kEnableLocalNodeMatching) that isn't named in most result filenames.
--dirs-file mode also applies outputs/dashboard_overrides.txt (see that
file for format), which lets you manually assert/correct fields per file
or glob of files. Override with --overrides-file, or skip with
--no-overrides.
"""

import argparse
import csv
import fnmatch
import json
import re
import shlex
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Matches the repo's benchmark naming convention, e.g.:
#   orz900d_5000_solver1.json          -> agents=5000, label=solver1
#   orz900d_10000_solver6_level3.json  -> agents=10000, label=solver6_level3
# Falls back to agents=None, label=<stem> for files that don't match.
FILENAME_RE = re.compile(r"^.+?_(?P<agents>\d+)_(?P<label>.+)$")

# Parses a label like "solver6_level4_nolocalmatch" or "solver7_depth2_rerun".
LABEL_RE = re.compile(
    r"^solver(?P<solver>\d+)(?:_(?:level|depth)(?P<level>\d+))?(?P<rest>(?:_[A-Za-z0-9]+)*)$"
)


def parse_filename(stem: str):
    """Best-effort extraction of (agents, label) from a result filename stem."""
    m = FILENAME_RE.match(stem)
    if m:
        return int(m.group("agents")), m.group("label")
    return None, stem


def load_known_maps(repo_root: Path = REPO_ROOT):
    """Map basenames (e.g. "scene_mp_4p_03") from every instances/**/*.map
    file, longest-first so prefix matching picks the most specific map
    (e.g. "orz900d" over a hypothetical "orz9"). Reading this from disk
    instead of hardcoding keeps it correct as maps are added."""
    maps_dir = repo_root / "instances"
    names = {p.stem for p in maps_dir.rglob("*.map")} if maps_dir.is_dir() else set()
    return sorted(names, key=len, reverse=True)


def strip_map_prefix(text: str, known_maps):
    for name in known_maps:
        if text == name or text.startswith(name + "_"):
            return name, text[len(name):].lstrip("_")
    return None, text


def parse_label(label: str):
    """Extract solver/level/nolocalmatch/variant out of a label like
    "solver6_level4_nolocalmatch" (variant = leftover tokens, e.g. "postfix",
    "rerun")."""
    m = LABEL_RE.match(label)
    if not m:
        return {"solver": None, "level": None, "nolocalmatch": False, "variant": label or None}
    tokens = [t for t in (m.group("rest") or "").split("_") if t]
    nolocalmatch = "nolocalmatch" in tokens
    variant_tokens = [t for t in tokens if t != "nolocalmatch"]
    return {
        "solver": int(m.group("solver")),
        "level": int(m.group("level")) if m.group("level") else None,
        "nolocalmatch": nolocalmatch,
        "variant": "_".join(variant_tokens) or None,
    }


def parse_run_info(json_path: Path, known_maps):
    """Best-effort structured parse of map/agents/solver/level/nolocalmatch/
    variant from a result file's path, for --dirs-file mode. Falls back
    gracefully (fields left None) for files that don't follow the naming
    convention, e.g. one-off smoke tests or nested per-timestep dumps."""
    stem = json_path.stem
    map_name, rest = strip_map_prefix(stem, known_maps)
    combo = rest
    if map_name is None:
        # Nested cases like ..._per_timestep/solver1/result.json: the map
        # lives in an ancestor directory name, not the filename itself.
        map_name, _ = strip_map_prefix(json_path.parent.name, known_maps)
        rest = stem
        combo = f"{json_path.parent.name}_{stem}"

    agents = None
    am = re.match(r"^(\d+)_(.*)$", rest)
    if am:
        agents = int(am.group(1))
        label = am.group(2)
    else:
        label = rest

    info = parse_label(label)
    if info["solver"] is None:
        sm = re.search(r"solver(\d+)", combo)
        if sm:
            info["solver"] = int(sm.group(1))
        lm = re.search(r"(?:level|depth)(\d+)", combo)
        if lm:
            info["level"] = int(lm.group(1))
        info["nolocalmatch"] = info["nolocalmatch"] or "nolocalmatch" in combo

    return {
        "map": map_name,
        "agents": agents,
        "solver": info["solver"],
        "level": info["level"],
        "nolocalmatch": info["nolocalmatch"],
        "variant": info["variant"],
        "label": label or None,
    }


def is_result_json(json_path: Path) -> bool:
    """Cheap check so a directory of mixed JSON (metrics.csv exports,
    non-run JSON, etc.) doesn't blow up the scan -- only files with the
    fields this script needs are treated as run results."""
    try:
        with open(json_path) as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        return False
    return isinstance(data, dict) and "makespan" in data and "numTaskFinished" in data


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


def compute_metrics_indexed(json_path: Path, source_dir: Path, known_maps) -> dict:
    """Like compute_metrics, but with the richer map/solver/level/variant
    columns parsed via parse_run_info, plus provenance (which configured
    directory this file came from) -- used by --dirs-file mode."""
    with open(json_path) as f:
        data = json.load(f)

    info = parse_run_info(json_path, known_maps)
    tasks = data["numTaskFinished"]
    makespan = data["makespan"]
    steps = len(data.get("timeStepMetrics", []))
    agents = data.get("teamSize", info["agents"])

    return {
        "dir": str(source_dir),
        "file": str(json_path.relative_to(REPO_ROOT)) if json_path.is_relative_to(REPO_ROOT) else str(json_path),
        "map": info["map"],
        "agents": agents,
        "solver": info["solver"],
        "level": info["level"],
        "nolocalmatch": info["nolocalmatch"],
        # Best-effort default from the "_nolocalmatch" filename convention;
        # this is only reliable when True (a deliberate ablation label) --
        # its absence does NOT mean local matching was on, since that's a
        # compile-time flag most result files don't name at all. Left blank
        # (unknown) rather than guessing True/False otherwise -- annotate
        # the real answer via outputs/dashboard_overrides.txt.
        "local_node_match": False if info["nolocalmatch"] else "",
        "variant": info["variant"],
        "label": info["label"],
        "tasks": tasks,
        "steps": steps,
        "makespan": makespan,
        "tp_steps": tasks / steps if steps else float("nan"),
        "tp_makespan": tasks / makespan if makespan else float("nan"),
    }


def read_dirs_file(dirs_file: Path):
    dirs = []
    for raw_line in dirs_file.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        p = Path(line)
        dirs.append(p if p.is_absolute() else REPO_ROOT / p)
    return dirs


def coerce_override_value(s: str):
    """true/false -> bool, digits -> int, else left as a string."""
    if s.lower() in ("true", "false"):
        return s.lower() == "true"
    if s.lstrip("-").isdigit():
        return int(s)
    return s


def read_overrides_file(overrides_file: Path):
    """Parses lines of the form:
        <glob-pattern> <field>=<value> [<field>=<value> ...]
    Patterns match against each result file's path relative to the repo
    root (fnmatch-style: *, ?, [seq]). Returns (rules, field_names) where
    rules is an ordered list of (pattern, {field: value}) -- applied
    top-to-bottom, later matching rules win on a field-by-field basis --
    and field_names is the ordered set of every field any rule sets, for
    stable CSV column ordering."""
    rules = []
    field_names = []
    for lineno, raw_line in enumerate(overrides_file.read_text().splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            tokens = shlex.split(line)
        except ValueError as e:
            sys.exit(f"error: {overrides_file}:{lineno}: {e}")
        if len(tokens) < 2:
            sys.exit(f"error: {overrides_file}:{lineno}: expected '<pattern> <field>=<value> ...'")
        pattern, assignments = tokens[0], tokens[1:]
        fields = {}
        for a in assignments:
            if "=" not in a:
                sys.exit(f"error: {overrides_file}:{lineno}: '{a}' is not <field>=<value>")
            k, v = a.split("=", 1)
            fields[k] = coerce_override_value(v)
            if k not in field_names:
                field_names.append(k)
        rules.append((pattern, fields))
    return rules, field_names


def apply_overrides(row: dict, rel_file: str, rules):
    for pattern, fields in rules:
        if fnmatch.fnmatch(rel_file, pattern):
            row.update(fields)


def find_result_files(path: Path, recursive: bool):
    if path.is_file():
        return [path]
    pattern = "**/*.json" if recursive else "*.json"
    return sorted(path.glob(pattern))


def format_cell(value):
    if isinstance(value, float):
        return "nan" if value != value else f"{value:.3f}"
    return str(value)


def write_markdown(rows, fieldnames, out):
    out.write("| " + " | ".join(fieldnames) + " |\n")
    out.write("|" + "|".join(["---"] * len(fieldnames)) + "|\n")
    for row in rows:
        out.write("| " + " | ".join(format_cell(row[k]) for k in fieldnames) + " |\n")


def write_outputs(rows, fieldnames, output: Path, md_path: Path):
    out = open(output, "w", newline="") if output else sys.stdout
    try:
        writer = csv.DictWriter(out, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in fieldnames})
    finally:
        if output:
            out.close()
    if output:
        print(f"wrote {len(rows)} rows to {output}", file=sys.stderr)

    if md_path:
        md_path.parent.mkdir(parents=True, exist_ok=True)
        with open(md_path, "w") as f:
            write_markdown(rows, fieldnames, f)
        print(f"wrote {len(rows)} rows to {md_path}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, nargs="?", help="A result .json file, or a folder containing them")
    parser.add_argument(
        "--dirs-file", type=Path,
        help="Aggregate every result JSON found under the directories listed in this file "
             "(one per line, '#' comments allowed -- see outputs/dashboard_dirs.txt) into one "
             "master CSV/MD with map/solver/level/variant columns parsed out.",
    )
    parser.add_argument(
        "--overrides-file", type=Path,
        help="Manual field overrides applied on top of the parsed columns (--dirs-file mode "
             "only). Default: outputs/dashboard_overrides.txt next to --dirs-file, if it exists.",
    )
    parser.add_argument("--no-overrides", action="store_true", help="Skip applying the overrides file")
    parser.add_argument("-o", "--output", type=Path, help="Write CSV here instead of stdout")
    parser.add_argument("--md", type=Path, help="Write Markdown table here (default: -o's path with a .md suffix)")
    parser.add_argument("--recursive", action="store_true", help="Recurse into subfolders when path is a directory")
    args = parser.parse_args()

    if args.dirs_file:
        if args.path:
            sys.exit("error: pass either a path or --dirs-file, not both")
        if not args.dirs_file.exists():
            sys.exit(f"error: {args.dirs_file} does not exist")

        known_maps = load_known_maps()
        included_dirs = read_dirs_file(args.dirs_file)
        if not included_dirs:
            sys.exit(f"error: {args.dirs_file} lists no directories (all blank/commented out?)")

        rows = []
        for d in included_dirs:
            if not d.is_dir():
                print(f"warning: {d} does not exist, skipping", file=sys.stderr)
                continue
            rel_dir = d.relative_to(REPO_ROOT) if d.is_relative_to(REPO_ROOT) else d
            for f in find_result_files(d, recursive=True):
                if is_result_json(f):
                    rows.append(compute_metrics_indexed(f, rel_dir, known_maps))
        if not rows:
            sys.exit("error: no result JSON files found under any listed directory")

        fieldnames = ["dir", "file", "map", "agents", "solver", "level", "nolocalmatch",
                      "local_node_match", "variant", "label", "tasks", "steps", "makespan",
                      "tp_steps", "tp_makespan"]

        overrides_file = args.overrides_file or (args.dirs_file.parent / "dashboard_overrides.txt")
        if not args.no_overrides and overrides_file.exists():
            rules, extra_fields = read_overrides_file(overrides_file)
            for row in rows:
                apply_overrides(row, row["file"], rules)
            for f in extra_fields:
                if f not in fieldnames:
                    fieldnames.append(f)
            for row in rows:
                for f in extra_fields:
                    row.setdefault(f, "")
            print(f"applied {len(rules)} override rule(s) from {overrides_file}", file=sys.stderr)
        elif args.overrides_file and not args.overrides_file.exists():
            sys.exit(f"error: {args.overrides_file} does not exist")

        # "_nolocalmatch" in the filename is a hard fact (a deliberate
        # ablation label), not a guess -- enforce it after overrides so it
        # can never be silently clobbered by an unrelated override rule.
        for row in rows:
            if row["nolocalmatch"]:
                row["local_node_match"] = False

        rows.sort(key=lambda r: (
            r["map"] or "",
            r["agents"] if r["agents"] is not None else -1,
            r["solver"] if r["solver"] is not None else -1,
            r["level"] if r["level"] is not None else -1,
            r["label"] or "",
        ))
        output = args.output or (REPO_ROOT / "outputs" / "dashboard" / "metrics.csv")
        md_path = args.md or output.with_suffix(".md")
        output.parent.mkdir(parents=True, exist_ok=True)
        write_outputs(rows, fieldnames, output, md_path)
        return

    if not args.path:
        sys.exit("error: pass a result .json file/folder, or --dirs-file")
    if not args.path.exists():
        sys.exit(f"error: {args.path} does not exist")

    result_files = find_result_files(args.path, args.recursive)
    if not result_files:
        sys.exit(f"error: no .json files found under {args.path}")

    rows = [compute_metrics(f) for f in result_files]
    # Sort by agent count then label so multi-config sweeps plot in a sane order.
    rows.sort(key=lambda r: (r["agents"] if r["agents"] is not None else -1, r["label"]))

    fieldnames = ["file", "agents", "label", "tasks", "steps", "makespan", "tp_steps", "tp_makespan"]
    md_path = args.md or (args.output.with_suffix(".md") if args.output else None)
    write_outputs(rows, fieldnames, args.output, md_path)


if __name__ == "__main__":
    main()
