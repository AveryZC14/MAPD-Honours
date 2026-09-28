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
Large-Scale Online MAPD*, AAMAS 2026, `thesis/AAMAS_2026_Yue_Camera_ready.pdf`),
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
  (`task_id % tasks.size()`, `src/TaskManager.cpp:192`), so a smaller file
  makes the first revealed tasks duplicates of each other.
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
  sizes (Table 3: orz900d and IH at 10k and 20k), so they compare directly
  with it and give one common baseline across all maps.
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
- **80k and 90k have never been run.** Treat 80k as untested. Smoke-test it
  (for example `-s 20`) on each map before queueing long runs.
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
reruns for every team size. IH took about 5 minutes for 3 team sizes, and the
earlier scene_sp_pol_06 generation took about 85 minutes for 5 team sizes.
Expect the full set to take a couple of hours.

### Resulting layout

```text
instances/thesis_benchmarks/
  README.md                   # this file
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
- each JSON has `numTasksReveal: 1.5` and the right `teamSize`.

## Running the benchmarks

### Settled

- **Hierarchy caches.** The copied maps are byte-identical, and the cache is
  keyed on a hash of the map contents, so the existing caches in
  `hierarchy_cache/` work for these instances:

  | Map | Cache |
  |---|---|
  | orz900d | `hierarchy_cache/orz900d.hierarchy` |
  | warehouseXL | `hierarchy_cache/warehouseXL_level9.hierarchy` |
  | IH_mp_2p_01 | `hierarchy_cache/IH_mp_2p_01_level9.hierarchy` (10 levels; the true fixpoint is 12 levels, in `IH_mp_2p_01_fixpoint.hierarchy`) |
  | scene_mp_4p_03 | `hierarchy_cache/scene_mp_4p_03_level9.hierarchy` |
  | scene_sp_pol_06 | `hierarchy_cache/scene_sp_pol_06_level9.hierarchy` |

  `*.hierarchy` files are gitignored and several GB in total. On a new
  machine, copy them over or let the first run build them (IH takes about
  5 minutes, and the scene maps take longer).
- **`--preprocessTimeLimit`**: previous sweeps used 600000 (IH), 900000
  (warehouseXL) and 1800000 (scene_mp_4p_03, scene_sp_pol_06).

### To decide before running

- **Which solvers.** At minimum solver 1 (full-map flow, the paper's method)
  and solver 6 (hierarchy). Possibly solver 7 and the cascade and stride
  variants.
- **Which flow levels for solver 6.** At 60k, level 2 starved the planner, so
  prefer level 4 or deeper at 40k and 80k.
- **Simulation length.** The paper used 2,000 steps for orz900d and 5,000 for
  IH. With 2-location tasks, 500 steps is short: solver 6 finished only 478
  tasks on IH at 10k agents.
- **Heuristic build flag.** `USE_MANHATTAN_HEURISTIC` in
  `default_planner/const.h` is a compile-time switch, currently `true`. The
  paper used a Manhattan heuristic with plain PIBT on its ultra-large maps.
  Record which build each run used.
- **The paper-matched baseline.** On IH the paper ran the flow assignment only
  every 30 steps (every 10 on orz900d). Solver 1 here re-solves at every
  decision. On the 2-location IH instance at 10k agents, that took about 38 s
  per solve, made only 14 decisions in 500 steps and finished 0 tasks. For a
  like-for-like comparison with the paper's Table 3, solver 1 needs a run with
  that 30-step interval.

## Known issue in the reference paper

The paper lists IH as 1912×1800 with 6,545,639 free cells. That is impossible:
the grid has only 3,441,600 cells. The IH map here has 2,217,759 free cells.
Oddly, scene_mp_4p_03's 6.30M free cells is close to the paper's figure. Check
which map the paper actually used before citing its IH numbers.
