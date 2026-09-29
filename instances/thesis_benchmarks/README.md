# Thesis benchmark instances

The final benchmark set for the thesis. All instances live in this folder
(`instances/thesis_benchmarks/`), one subfolder per map. Commit the generated
files and copy them rather than regenerating on each machine: the official
generator has no random seed, so each run of it produces a different instance.

## Why this set exists

The earlier instances under `instances/custom/<map>/` were generated with the
official LoRR 2024 generator's default errand range (`--minEPT 1 --maxEPT 4`),
so each task had 1–4 locations. That does not match the pickup→delivery task
model in the reference paper (Zhang et al., *Flow-Based Task Assignment for
Large-Scale Online MAPD*, AAMAS 2026; a local copy is kept at
`thesis/AAMAS_2026_Yue_Camera_ready.pdf`, which is gitignored),
where every task is exactly one pickup plus one delivery.

It matters for results. A 1-location task finishes the moment its assigned
agent reaches it, and the schedulers send agents to the nearest task, so
throughput on the old instances was dominated by single-location tasks:

- **orz900d, 5k agents, 200 steps:** 95–99% of finished tasks had 1 location.
  Throughput fell from 1,998 to 115 (solver 1) and from 1,840 to 219 (solver 6)
  once every task had 2 locations.
- **IH_mp_2p_01, 10k agents, 500 steps, solver 6 at flow level 4:**
  3,485 → 478 tasks finished.

Runs on the old instances are still valid as a comparison under the LoRR 2024
multi-errand task model, but their throughput numbers are not comparable with
the paper.

## Instance specification

Every instance uses:

- **2 locations per task**, pickup then delivery (`--minEPT 2 --maxEPT 2`).
- **Task pool of 1.5× the team size** (`numTasksReveal: 1.5`), matching the
  paper's LoRR setup.
- **A task file of at least 1.5× the largest team size**, so the starting task
  pool never repeats a task. The simulator cycles through the file
  (`task_id % tasks.size()`, `src/TaskManager.cpp:198`), so a smaller file
  makes the first revealed tasks duplicates of each other.

  This only covers the starting pool. At the top team size the file is
  exactly the pool size (120,000 at 80k, and 30,000 for orz900d at 20k), so
  from the first newly revealed task on, each new task repeats one that may
  still be waiting in the pool. Every run cycles through the file several
  times over 8,000 steps. The simulator allows this and nothing breaks, but
  state it in the thesis.
- **One shared task file per map**, with one `.agents` file per team size,
  all generated in a single call.

### Where agents and tasks can be placed

The generator samples every agent start and every task location uniformly at
random from the map's **largest connected region**. It first runs `find_lcc`,
which marks every free cell outside that region as blocked. Pickups and
deliveries are sampled independently, so a delivery is not placed relative to
its pickup. The generator and the simulator both treat `.`, `E` and `S` as
walkable, and `@` and `T` as obstacles.

Restricting to the largest region matters: IH, scene_mp_4p_03 and
scene_sp_pol_06 all have disconnected regions. If a task location is
unreachable, `astar` throws `no path found` and the whole run aborts rather
than skipping that task.

warehouseXL has `E` (station) and `S` (storage) cells, but is generated with
the same uniform sampler as the other maps (no `--taskFile`), not the
station-aware warehouse task generator. This keeps it consistent with the
earlier warehouseXL instances.

### Maps and team sizes

| Map | Size | Main region (cells) | Teams | Task file | Density at each team size |
|---|---|---|---|---|---|
| orz900d | 1491×656 | 96,603 (one region) | 10k, 20k | 30,000 | 10.4%, 20.7% |
| warehouseXL | 1900×1800 | 1,729,940 (one region) | 10k, 20k, 40k, 80k | 120,000 | 0.58%, 1.2%, 2.3%, 4.6% |
| IH_mp_2p_01 | 1912×1800 | 2,109,360 (of 2,217,759 free, 47 regions) | 10k, 20k, 40k, 80k | 120,000 | 0.47%, 0.95%, 1.9%, 3.8% |
| scene_mp_4p_03 | 3728×3728 | 6,185,043 (of 6,297,149 free, 835 regions) | 10k, 20k, 40k, 80k | 120,000 | 0.16%, 0.32%, 0.65%, 1.3% |
| scene_sp_pol_06 | 4328×4192 | 9,925,937 (of 9,939,841 free, 666 regions) | 10k, 20k, 40k, 80k | 120,000 | 0.10%, 0.20%, 0.40%, 0.81% |

Density is team size divided by the main region's cell count.

### How the team sizes were chosen

- **10k and 20k on every map.** These are the paper's ultra-large-map team
  sizes (Table 3: orz900d and IH at 10k and 20k), and they give one common
  baseline across all maps. Using the same team sizes does not make the
  throughput numbers directly comparable with the paper's: solver 1 re-solves
  on a different schedule here (see "Not included" below), and the paper's IH
  map may not be this one (see the last section).
- **40k and 80k on the four big maps**, for scaling. 60k was dropped as too
  fine a step. 80k replaces the older 90k as a rounder top end.
- **orz900d stays at 10k and 20k.** It is already at 10–21% density, the
  crowded range where the paper's smaller maps start losing throughput.
- **The big maps are deliberately in a low-density regime.** The paper's small
  and medium maps were swept to 7–61% density. Reaching even 7% on the big maps
  would need 150k–700k agents, which this pipeline cannot run. State the
  densities explicitly in the thesis rather than implying the big maps match
  the paper's congestion levels. The paper's IH density is also under 1%.

### Evidence for large team sizes

- **60k has run cleanly.** scene_sp_pol_06 at 60k agents, 500 steps, solver 6
  at flow levels 2/4/6/8 finished with 0 errors or timeouts, and throughput
  scaled roughly linearly (about 19,800 tasks against about 3,100 at 10k). See
  `ai/auto_benchmarking_scene_sp_pol_06.md`.
- **At 60k, shallow flow levels starve the planner.** Level 2 made only 78 real
  planning decisions out of 500 steps (`ai/todo.md`), so agents mostly waited.
  Levels 4 and deeper were fine.
- **80k has only been smoke-tested**, at 20 steps (see "Pre-sweep checks"
  below). No long run has been done at 80k.
- **The known memory limit is about map size, not agent count.**
  scene_sp_endmaps (24.5M cells) runs out of memory on a 31 GB machine just
  loading the map and building the hierarchy
  (`ai/auto_benchmarking_scene_sp_endmaps.md`). That is why it is not in this
  set.

## Generating the instances

### Setup (once per machine)

```shell
git clone https://github.com/MAPF-Competition/Benchmark-Archive.git   # into the repo root, gitignored
pip install --user --break-system-packages easydict pyyaml numpy tqdm
```

### Commands

Run from the repo root. `GEN` is the official LoRR 2024 benchmark generator.

```shell
GEN="Benchmark-Archive/2024 Competition/Problem Generator/script/benchmark_generator.py"
OUT=instances/thesis_benchmarks

python3 "$GEN" --mapFile instances/custom/maps/orz900d.map \
  --problemName orz900d --benchmark_folder $OUT/orz900d \
  --teamSizes 10000 20000 --taskNum 30000 \
  --revealNum 1.5 --minEPT 2 --maxEPT 2

for MAP in warehouseXL IH_mp_2p_01 scene_mp_4p_03 scene_sp_pol_06; do
  python3 "$GEN" --mapFile instances/custom/maps/$MAP.map \
    --problemName $MAP --benchmark_folder $OUT/$MAP \
    --teamSizes 10000 20000 40000 80000 --taskNum 120000 \
    --revealNum 1.5 --minEPT 2 --maxEPT 2
done
```

Generation time is dominated by the generator's pure-Python `find_lcc`, which
reruns for every team size. The set here was generated on 2026-09-28: the four
big maps ran in parallel and took 68 minutes in total.

### Resulting layout

```text
instances/thesis_benchmarks/
  README.md                   # this file
  hierarchy_build_times.csv   # measured build/load times and memory per map
  <map>/
    <map>_<teamSize>.json     # the instance to pass to --inputFile
    agents/<map>_<teamSize>.agents
    tasks/<map>.tasks         # shared across all team sizes for this map
    maps/<map>.map            # copied unchanged from instances/custom/maps/
```

### Checks after generating

For every map, confirm:

- every task has exactly 2 locations, and no task has its pickup and delivery
  on the same cell;
- every task location and agent start is inside the main region;
- agent starts are unique;
- each JSON has `numTasksReveal: 1.5` and the right `teamSize`;
- the copied map is byte-identical to `instances/custom/maps/<map>.map`.

**Result for the set in this folder (2026-09-28):** every check passes on
every map, with one exception. In orz900d, task index 9421 (line 9424 of
`orz900d/tasks/orz900d.tasks`) has the same cell for pickup and delivery
(`905893,905893`). The official generator samples the two locations
independently and does not prevent this. It is 1 of 30,000 tasks and completes
one step after its pickup, so it was left as generated.

## Running the benchmarks

### Settled

- **Hierarchy caches: use the full-depth ones.** The copied maps are
  byte-identical, and the cache is keyed on a hash of the map contents, so the
  caches in `hierarchy_cache/` work for these instances. The hierarchy is built
  from the map alone, so changing agents or tasks never requires a rebuild.

  | Map | Cache | Levels | Top level (nodes) |
  |---|---|---|---|
  | orz900d | `hierarchy_cache/orz900d_full.hierarchy` | 12 | 1 |
  | warehouseXL | `hierarchy_cache/warehouseXL_full.hierarchy` | 12 | 1 |
  | IH_mp_2p_01 | `hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy` | 12 | 33 |
  | scene_mp_4p_03 | `hierarchy_cache/scene_mp_4p_03_full.hierarchy` | 13 | 238 |
  | scene_sp_pol_06 | `hierarchy_cache/scene_sp_pol_06_full.hierarchy` | 14 | 176 |

  Every one of these was verified on 2026-09-29 in two ways.

  - **Structure.** Each cache is identical to a fresh no-cache build with the
    current code at every level. The comparison used the checks from
    `utils/validation/validate_hierarchy_cache.cpp`: node coordinates, level
    mappings, internal arc metrics, every arc's endpoints, cost and capacity,
    and both bridge caches.
  - **Real runs.** Each cache ran solver 6 for 50 steps on the 10k instance,
    once at flow level 4 and once in hierarchical-only mode
    (`--flowSolveLevel <top level> --minCascadeLevel 1`, described under
    "Final run plan"). All 10 runs had 0 planner, schedule and timeout
    errors. Hierarchical-only mode matched every agent locally, with 0 flow matches.

  The old caches are identical to the full-depth ones on every level they
  share, except the top level's pointer to the next level up, which the old
  caches never filled in because they stopped there. Results from earlier runs
  with the old caches therefore used the same hierarchy.

  On maps with disconnected regions, the top level has one node per region
  that survives the final coarsening step, not a single node, which is
  expected.

  **Don't use the older caches.** `orz900d.hierarchy`, `orz900d.hier`, every
  `*_level9.hierarchy` and `scene_mp_4p_03_level6.hierarchy` were built before
  the hierarchy started coarsening all the way to its top level, so they stop
  early. They still load without error, because the cache check covers only
  the format, dimensions and map hash. Runs that stay at or below the old
  cache's last level (9 for the `*_level9` caches, 6 for
  `scene_mp_4p_03_level6.hierarchy`) would be unaffected, but any run that
  cascades higher would stop short.

  `*.hierarchy` files are gitignored, and the five full-depth caches take
  about 5.1 GB in total. On a new machine, copy them over, or let the first run
  build them. Building takes about a minute per map, but see the memory
  figures below.
- **`--preprocessTimeLimit`**: previous sweeps used 600000 (IH), 900000
  (warehouseXL) and 1800000 (scene_mp_4p_03, scene_sp_pol_06).

### Hierarchy build times and memory

Measured on 2026-09-28 on this machine (31 GB RAM), one run per map, with
nothing else running. Full data, including per-level node counts, is in
`hierarchy_build_times.csv` next to this file.

| Map | Pure build (s) | Cache load (s) | Peak memory (GB) |
|---|---|---|---|
| orz900d | 0.56 | 0.85 | 1.1 |
| warehouseXL | 15.4 | 8.2 | 4.8 |
| IH_mp_2p_01 | 13.2 | 9.2 | 4.9 |
| scene_mp_4p_03 | 42.9 | 29.5 | 18.1 |
| scene_sp_pol_06 | 64.5 | 44.2 | 24.5 |

- **Pure build** is from a run with no `--hierarchyCache`, so there is no load
  attempt and no save.
- **Cache load** is from a separate run that loaded the full-depth cache.
  Loading saves only about a third of the build time on IH and the scene
  maps (about half on warehouseXL), and is slower than building on orz900d.
- **Peak memory** is for the whole `lifelong` process with 10,000 agents
  (`/usr/bin/time -v`), not just the hierarchy. Saving a cache adds more: the
  run that built and saved `scene_sp_pol_06_full.hierarchy` peaked at 25.7 GB
  and took 95.9 s, against 64.5 s for the build alone. scene_sp_pol_06 needs a
  machine with well over 26 GB of RAM.
- **These are single runs**, so run-to-run variance is unknown. Repeat them
  before quoting exact figures.

**Why not to use `schedulerHierarchyBuildTime` from ordinary runs.** The field
in the output JSON is not a consistent build-time measure:

- The timer in `ReducedHierarchy::ensure()`
  (`map_reduction_test/MapCoarsenV1.cpp`) covers the cache-load attempt, the
  build, and the save to disk together. With a cache path but no cache file,
  it reports build time plus a multi-GB disk write.
- On a cache hit it reports load time, and the JSON has no field saying
  whether the hierarchy was loaded or built.
- Solver 1 always reports 0, because only solvers 6 and 7 fill the field in
  (`default_planner/scheduler.cpp`), even though every solver builds the
  hierarchy during setup.
- It comes from the last scheduler call only, and a call with nothing to
  reassign resets it to 0 (`set_last_timing` in
  `default_planner/scheduler.cpp`). The same applies to
  `schedulerHierarchyNumLevels` and `schedulerHierarchyLevelNodeCounts`, so a
  run can report 0 levels even though it used a 12-level hierarchy
  throughout.

For build times in the thesis, use the table above, or repeat the same method:
a solver 6 run with no `--hierarchyCache`.

### Final run plan

Agreed 2026-09-29. 120 runs, 8,000 timesteps each.

| Group | Instances | Runs |
|---|---|---|
| Solver 6 at `--flowSolveLevel` 2, 4, 6 and 8 | all 18 | 72 |
| Solver 6, hierarchical matching only (no flow) | all 18 | 18 |
| Solver 5 (Greedy) | all 18 | 18 |
| Solver 1 (full-map flow) | 10k and 20k on all 5 maps | 10 |
| Solver 7 at `--flowSolveLevel 4` | IH_mp_2p_01 20k, scene_sp_pol_06 20k | 2 |
| **Total** | | **120** |

The 18 instances are orz900d at 10k and 20k, plus warehouseXL, IH_mp_2p_01,
scene_mp_4p_03 and scene_sp_pol_06 at 10k, 20k, 40k and 80k.

**Common flags for every run:** `-s 8000`, default `--planTimeLimit` (1000 ms),
default `--assignNew` (false), no `--useTraffic`, and a build with
`USE_MANHATTAN_HEURISTIC = true` (`default_planner/const.h`). Solvers 6 and 7
also get `--hierarchyCache` with the map's full-depth cache from the table
under "Settled".

**Per-map flags:**

| Map | `--preprocessTimeLimit` | Top level (hierarchical-only) |
|---|---|---|
| orz900d | 600000 | 11 |
| warehouseXL | 900000 | 11 |
| IH_mp_2p_01 | 600000 | 11 |
| scene_mp_4p_03 | 1800000 | 12 |
| scene_sp_pol_06 | 1800000 | 13 |

**Hierarchical matching only** means `--flowSolveLevel <top level>
--minCascadeLevel 1`. Matching then cascades up the whole hierarchy and the
flow step has nothing left to do (0 flow matches, confirmed on every map in
the 50-step checks above).

**Example commands** (IH_mp_2p_01, 20k agents):

```shell
I=instances/thesis_benchmarks/IH_mp_2p_01/IH_mp_2p_01_20000.json
C=hierarchy_cache/IH_mp_2p_01_fixpoint.hierarchy
COMMON="-s 8000 --preprocessTimeLimit 600000"

./build/lifelong -i $I -o <out>_solver6_level4.json   --scheduleModel 6 --flowSolveLevel 4 --hierarchyCache $C $COMMON
./build/lifelong -i $I -o <out>_solver6_hieronly.json --scheduleModel 6 --flowSolveLevel 11 --minCascadeLevel 1 --hierarchyCache $C $COMMON
./build/lifelong -i $I -o <out>_solver5.json          --scheduleModel 5 $COMMON
./build/lifelong -i $I -o <out>_solver1.json          --scheduleModel 1 $COMMON
./build/lifelong -i $I -o <out>_solver7_level4.json   --scheduleModel 7 --flowSolveLevel 4 --hierarchyCache $C $COMMON
```

#### Running the sweep

`scripts/run_thesis_sweep.py` holds this exact run list and runs it one run
at a time into `outputs/thesis_sweep/`:

**Current sweep:** started 2026-09-29 07:50 (full 120-run list, detached).
Expected to finish around 2026-10-10 with warehouseXL, or around
2026-10-08 without it.

All commands below are run from the repo root (`cd ~/MAPD-Honours`).

```shell
# --- Start or resume -------------------------------------------------------
python3 scripts/run_thesis_sweep.py --dry-run   # list all 120 commands, run nothing
scripts/sweep_watchdog.sh                       # start (or resume) detached; does nothing if already running
rm -f outputs/thesis_sweep/STOP                 # needed first if you stopped it with STOP (see below)

# --- Check on it -----------------------------------------------------------
tail -n 5 outputs/thesis_sweep/runner.log       # one line per run started/finished; last line = current run
grep -c ': ok' outputs/thesis_sweep/runner.log  # runs finished cleanly so far (out of 120)
grep PROBLEM outputs/thesis_sweep/runner.log    # runs that crashed, timed out or reported errors
column -s, -t < outputs/thesis_sweep/sweep_summary.csv | less -S   # per-run table
pgrep -af 'run_thesis_sweep|build/lifelong'     # is it running? expect the script + one lifelong
ls -t outputs/thesis_sweep/*_solver*.log | head -1 | xargs tail -n 3   # current run's timestep
cat outputs/thesis_sweep/watchdog.log           # when the watchdog (re)started it
df -h / ; free -g                               # disk (stops below 1 GB free) and memory

# --- Stop on purpose -------------------------------------------------------
touch outputs/thesis_sweep/STOP                 # first, so the watchdog won't restart it
pkill -f run_thesis_sweep.py; pkill -x lifelong # the current run is lost and redone on resume

# --- Drop warehouseXL ------------------------------------------------------
# Its 26 runs are last. Once runner.log shows the first warehouseXL run
# starting (run 95/120), stop the sweep as above. Or never start them:
setsid nohup python3 -B scripts/run_thesis_sweep.py --maps orz900d IH_mp_2p_01 scene_mp_4p_03 \
  scene_sp_pol_06 >> outputs/thesis_sweep/runner.log 2>&1 < /dev/null &

# --- Auto-restart after a crash or reboot (optional) ------------------------
crontab -e    # add the line:  */10 * * * * /home/ubuntu/MAPD-Honours/scripts/sweep_watchdog.sh
crontab -l    # check it is installed
crontab -e    # remove that line once the sweep has finished
```

- **Order:** all 10k runs first (every map and config), then 20k, 40k, 80k,
  so a sweep stopped early still has complete team sizes. warehouseXL is the
  exception: its 26 runs (about 2.5 days) all come last, after every team
  size of the other maps, so it can be dropped by stopping the sweep once
  the first warehouseXL run starts. `LAST_MAPS` in the script controls this.
- **Resumable:** rerun the same command after an interruption. Runs with a
  finished output JSON are skipped. lifelong writes its JSON only at the end,
  so a run that was cut off is redone from the start.
- **Stopping:** stop both the script and the current run, e.g.
  `pkill -f run_thesis_sweep.py; pkill -x lifelong`. Killing only the script
  leaves the current `lifelong` run going.
- **Watchdog:** `scripts/sweep_watchdog.sh` starts the sweep, detached, if
  it isn't running, no `lifelong` is running and `runner.log` doesn't end
  with the final summary line. Run it from cron to restart the sweep after a
  crash or reboot:
  `*/10 * * * * /home/ubuntu/MAPD-Honours/scripts/sweep_watchdog.sh`.
  Before stopping the sweep on purpose, `touch outputs/thesis_sweep/STOP`,
  or the watchdog will start it again. Restarts are logged to
  `outputs/thesis_sweep/watchdog.log`.
- **Timeout:** a run still going after 6 h is killed (the whole process
  group, so `lifelong` itself dies too) and recorded as `timeout`.
- **Records:** `sweep_summary.csv` gets one row per finished run (errors,
  wall-clock, peak memory, decisions, pickups, tasks finished).
  `sweep_meta.json` records the git commit, modified files,
  `USE_MANHATTAN_HEURISTIC`, CPU model, core count, total RAM and compiler
  version each time the sweep is started. Quote the machine in the thesis,
  since results depend on wall-clock planning time.
- **Logs** use `--logDetailLevel 3` (fatal errors only). At the default level,
  per-task log lines reach 17 MB in 20 steps at 80k agents, which would fill
  the disk over 120 runs. The script also stops if free disk drops below
  1 GB.
- **Subsets:** `--maps`, `--teams` and `--configs` (`solver6`,
  `solver6_hieronly`, `solver5`, `solver1`, `solver7`) run part of the list.
  `--sim-time` overrides the 8,000 steps for quick tests; point `--out-dir`
  somewhere else when using it.

Tested 2026-09-29 at `--sim-time 30` on orz900d 10k (all 7 configs) and IH
20k solver 7: 8/8 clean, and a rerun skipped all finished runs.

#### Why these choices

- **Solver 6 at several levels on every instance**, not one fixed level: the
  best level is expected to differ by map and team size. Earlier sweeps (on
  the old 1–4-location tasks) showed deeper levels helping steadily on
  scene_mp_4p_03, a flat result across levels 2–8 on scene_sp_pol_06, and
  level 2 starving the planner at 60k agents.
- **Solver 5 is the paper's Greedy baseline.** It assigns only newly free
  agents to their nearest task and never swaps, matching the paper's
  description. With the Manhattan heuristic it matches the paper's
  ultra-large-map setup.
- **Solver 1 is compared per decision, not per timestep.** When a solve runs
  over its time limit, all agents wait, so agents only move, and tasks only
  finish, on real decisions. Solver 1 makes few decisions on the big maps
  (roughly 1 s per solve on orz900d, 10 s on warehouseXL, 36 s on IH, 150 s on
  scene_mp_4p_03), so its N decisions are compared with the first N
  decisions of the other runs. Take the running totals of
  `TasksOpenedThisStep` (pickups reached) and `TasksFinishedThisStep` over
  `timeStepMetrics` rows (one row per decision). Pickups reached is the main
  measure: it is the part of each task the scheduler controls, and it stays
  meaningful even on maps where solver 1 makes too few decisions for any
  deliveries to finish.
  Call the result a percentage of solver 1, not of optimal: solver 1 gives
  the best assignment for each batch, not the best throughput over a run.
- **Solver 1 only at 10k and 20k.** These are the paper's team sizes. Its
  solves already take 10–150 s on the big maps, so 40k and 80k runs would
  make even fewer decisions. How solve time grows with team size has not
  been measured.
- **Solver 7 only as a 2-run appendix check.** It has been worse than solver 6
  everywhere so far. Report only its throughput: its `SchedulerSolveTime` and
  `SchedulerBackboneBuildTime` fields are known to be wrong (`ai/todo.md`).
- **8,000 timesteps.** With 2-location tasks, the mean pickup→delivery
  Manhattan distance is 467 cells on orz900d, about 1,000–1,250 on IH and
  warehouseXL, and about 2,250–2,450 on the two scene maps. A long horizon is
  needed to reach steady throughput, and to show the slow decline of shallow
  levels over time.
- **No repeats, and no cascade or stride variants.** On the IH 7,000-step
  sweep the scheduler was about 0.05% of runtime, and cascade or stride made
  no measurable throughput difference.
- **Not included:** the paper's solver-1 setup, which re-solves only every 10
  steps (orz900d) or 30 steps (IH). Solver 1 here re-solves at every
  decision, so its per-timestep throughput is not comparable with the
  paper's Table 3.

#### Caveats for the write-up

- **Run-to-run variation.** Identical commands give different results by a
  few percent, because the planner is limited by wall-clock time. There are
  no repeats, so treat level-to-level differences of a few percent as ties.
- **Solver 1's planning per decision is probably worse.** A long solve
  likely uses up the planner's time budget for that decision, which would
  favour the other solvers in the per-decision comparison. Not yet checked
  in the code.
- **Few tasks finish within solver 1's decisions on the big maps.** In 8,000
  steps, solver 1 makes only about 50 decisions on scene_mp_4p_03 and even
  fewer on scene_sp_pol_06, so each agent moves at most that many cells and
  almost no deliveries can complete. Tasks finished is a usable per-decision
  measure on orz900d only. It is marginal on warehouseXL (about 800
  decisions against a mean pickup→delivery distance of 1,242) and IH, and
  meaningless on the scene maps. Use pickups reached (`TasksOpenedThisStep`)
  on those maps instead.

#### Runtime

About 1 s of wall-clock per simulated step, so about 2.3 h per run including
loading. That's about **276 h (11.5 days)** run one at a time.

- The sweep runs one at a time. Don't run two at once to save time: results
  depend on wall-clock planning time, so paired runs would affect each
  other, and two copies of the script would share `sweep_summary.csv`.
- scene_mp_4p_03 (about 18 GB) and scene_sp_pol_06 (about 25 GB) runs could
  not be paired anyway on the 31 GB machine.
- Without warehouseXL, the sweep is 94 runs, about 216 h (9 days).
- Output files are 1–2 MB each, so disk space (2.7 GB free) is not a problem.

#### Pre-sweep checks (done 2026-09-29)

- **80k smoke test.** On all four big maps at 80k agents, `-s 20`, with solver
  5, solver 6 at level 2, and solver 6 hierarchical-only: all 12 runs exited
  cleanly, with 0 planner, schedule and timeout errors and no crashes. Peak
  memory: about 3–5 GB on warehouseXL and IH, 12.7–17.8 GB on
  scene_mp_4p_03, and 16.6–24.0 GB on scene_sp_pol_06.
- **Greedy has a slow first decision at large team sizes.** On its first
  decision every agent is free, and solver 5 compares each one with every
  open task. On IH this took 31 s at 40k agents and 218 s at 80k. Later
  decisions took at most 1.5 s (40k) and 0.4 s (80k), with no further
  timeouts. At 80k, agents therefore wait about 218 steps at the start
  (2.7% of an 8,000-step run). Note this when reporting Greedy's throughput
  at 40k and 80k. It does not affect per-decision comparisons.
- **Pickups-reached counter added.** `TasksOpenedThisStep` in each
  `timeStepMetrics` row and `numTaskOpened` in the run totals. Verified on
  `instances/custom/tiny/tiny.json` and on IH at 80k agents: the run total,
  the per-row sum and the number of "opens task" log lines all match (4 and
  72,185), pickups never fall behind deliveries, and every other output
  field matches the run made before the change.

## Known issue in the reference paper

The paper lists IH as 1912×1800 with 6,545,639 free cells. That is impossible:
the grid has only 3,441,600 cells. The IH map here has 2,217,759 free cells.
Oddly, scene_mp_4p_03's 6.30M free cells is close to the paper's figure. Check
which map the paper actually used before citing its IH numbers.
