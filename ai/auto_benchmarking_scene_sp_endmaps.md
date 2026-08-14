# scene_sp_endmaps: blocked by a general (non-solver-6) memory ceiling

> Detail file for one attempted sweep. See `ai/auto_benchmarking.md` for the
> cross-sweep index. **Unlike every other file in that index, this one has
> no throughput results** — the planned sweep never ran. This documents why,
> with real numbers, so the investigation doesn't have to be repeated.

Attempted 2026-08-13/14 (overnight, unattended), alongside the
`scene_sp_pol_06` sweep (`ai/auto_benchmarking_scene_sp_pol_06.md`, which
completed successfully). The plan was identical: solver 6,
`--flowSolveLevel` 2/4/6/8 at 10000/20000/60000 agents, plus a
`kEnableLocalNodeMatching` on/off ablation at 20000/60000 agents x levels
4/6/8. **None of it ran** — every attempt to build the hierarchy or even
just load the map into `lifelong` at all hit the machine's RAM ceiling
(31GB total, ~29GB usable).

## Map + instance generation

`instances/custom/maps/scene_sp_endmaps.map`, 4760x6400 (30,464,000 total
cells, 24,548,064 walkable, 80.6%) — by far the largest map ever attempted
in this repo (~2.2x `scene_mp_4p_03`'s 13.9M cells, the previous largest).
Generated the same way as `scene_sp_pol_06`
(`ai/auto_benchmarking_scene_sp_pol_06.md`): `benchmark_generator.py`,
`--taskNum 135000`, `--teamSizes 10000 20000 40000 60000 90000`. Took ~1h47m
wall-clock (the slowest step of the night, dominated by the same O(n)/O(n²)
upstream generator inefficiencies noted in the pol_06 doc, worse here
because the map is bigger). All 5 instances + task pool exist and are fine
— the problem is entirely downstream of instance generation, in
hierarchy-build/simulation memory.

## Investigation timeline

**First attempt** (the plan as originally scoped): build the hierarchy to
the compiled-in `kDefaultCoarsenLevels = 9` (10 levels, matching what
`scene_sp_pol_06` used successfully) via `lifelong`'s normal
`--hierarchyCache` smoke test. **OOM-killed** (`journalctl -k`, confirmed
via `dmesg`, not a guess): `anon-rss:32315040kB` at time of kill, `lifelong`
was still climbing when the kernel OOM-killer intervened.

**Working theory 1 (wrong): "level 9 specifically is pathological for this
map's topology."** Node counts shrink normally through the hierarchy (see
below), so the initial hypothesis was that something about the very last
level — not overall map size — was the trigger. Bisected via
`./build/dump_coarsening <instance> <prefix> <N>` (builds the hierarchy
directly with a lightweight loader, no full simulation state, so it isolates
hierarchy-build memory specifically), timed with `/usr/bin/time -v`:

| depth built | peak RSS | wall-clock |
|---|---|---|
| 0 (fine map only) | 8.4GB | 16s |
| 6 | 17.3GB | 3:52 (incl. CSV writing) |
| 8 | **17.3GB** (same as depth 6) | 3:50 (incl. CSV writing) |
| 9 | *(never tested standalone — see below)* | — |

Depth 8 costing the *same* as depth 6 (node counts by then are down to
1148) looked like strong evidence that depth 8 was safe and depth 9 alone
was the problem — since the sweep only needs `--flowSolveLevel` up to 8, a
depth-8 build seemed like a clean fix. **This reasoning turned out to be
incomplete, not wrong about the numbers** — depth 8 build-alone genuinely
is only 17.3GB. The mistake was assuming hierarchy-build memory (measured
via the lightweight `dump_coarsening` loader) was representative of what
`lifelong`'s full simulation setup would cost. It wasn't.

**Second attempt: rebuild with `kDefaultCoarsenLevels` temporarily set to
8** (`map_reduction_test/MapCoarsenV1.cpp:32`, compile-time only, no CLI
flag — same mechanism every prior sweep's depth changes used), then re-ran
the `lifelong --hierarchyCache` smoke test. **OOM-killed again**, in only
~48 seconds this time, at `anon-rss:31435008kB` — essentially the same
magnitude as the depth-9 attempt. This directly falsifies working theory 1:
if depth 9 specifically were the problem, depth 8 should have been fine, and
it wasn't.

**Working theory 2 (wrong): the `--hierarchyCache` save step (disk
serialization) doubles memory.** Re-ran the depth-8 build via `lifelong`
with **no** `--hierarchyCache` flag at all (so `ReducedHierarchy::ensure()`
never calls `save_hierarchy_to_file`). Read `MapCoarsenSerialize.cpp`'s
`write_level()` first and found no evidence of a large intermediate
buffer — it streams directly to `std::ofstream` field-by-field — so this
was already a weak hypothesis before testing, and the test confirmed it:
**OOM-killed anyway**, at `anon-rss:31424548kB`, statistically the same
number as with caching on. Ruled out.

**Isolating test: does this need solver 6 at all?** Ran
`--scheduleModel 1 -s 1` (solver 1 never calls `ReducedHierarchy::ensure()`
— hierarchy build is gated to `solver == 6` only, see
`ai/project_context.md` "Solver 6" section) on the same instance. **This is
the real finding**: solver 1, touching zero hierarchy code, peaked at
**28.4GB** just loading the map and running one timestep — no OOM this
time, but only because it landed under the ~29GB ceiling by a hair (was at
99% of usable RAM). This means:

- The ~28GB cost is **not** solver-6/hierarchy code at all. It's something
  in the general `lifelong` map-loading/simulation path (`Grid`,
  `SharedEnvironment`, task pool, per-timestep planner setup — all
  upstream/shared code, not `map_reduction_test/`) that scales badly with
  raw map size and has simply never been exercised at 30M+ cells before.
  `scene_mp_4p_03` (13.9M cells, ~2.2x smaller) never came close to this on
  any prior sweep, including runs at higher agent counts than tested here.
- Solver 6's hierarchy is only adding a few GB on top of that already
  near-the-ceiling base (~28.4GB solver-1 baseline → ~31-32GB solver-6
  total), not the ~17GB the standalone `dump_coarsening` measurement would
  suggest in isolation — the two costs overlap rather than simply adding,
  consistent with both touching similarly-sized map-scale structures.

## Why this wasn't chased further tonight

Pinning down the exact allocation(s) responsible needs a real memory
profiler (`valgrind --tool=massif` or similar) run against the general
`lifelong` path, and potentially a genuine code fix once found — both are
real development work, not something to attempt blind and unattended
overnight on someone else's uncommitted tree. The machine has 31GB RAM
total (~29GB usable); solver 1 alone is already at 99% of that just to load
this map. There is currently no solver that reliably fits `scene_sp_endmaps`
on this machine, and no remaining compile-flag/CLI lever (unlike the
coarsen-depth dead end above) that changes that — this is a hardware or
code-efficiency ceiling, not a configuration problem.

## State left behind

- `map_reduction_test/MapCoarsenV1.cpp`'s `kDefaultCoarsenLevels` (9) and
  `kEnableLocalNodeMatching` (true) are both back at their committed
  values, rebuilt and confirmed (`git diff` clean on this file) — none of
  the depth-8 experimentation was left in the tree.
- No `.hierarchy` cache file exists for `scene_sp_endmaps` (every build
  attempt was killed before completing).
- All 5 instance files (10000/20000/40000/60000/90000 agents) + the
  135000-task pool exist and are untouched/valid — only the
  hierarchy-build/simulation step is blocked.
- The `kEnableLocalNodeMatching` ablation (`ai/todo.md`) never ran for this
  map — there was never a usable hierarchy cache to reuse.

## Recommended next steps (not done here)

1. Profile `--scheduleModel 1 -s 1` on this instance with
   `valgrind --tool=massif` (or similar) to find what's actually
   proportional to map size in the general loading/simulation path — this
   is upstream/shared code, so a fix would help every solver, not just
   solver 6.
2. Once that's fixed (or if a quicker win turns up), the depth-8
   `kDefaultCoarsenLevels` workaround documented above is still a valid,
   ready-to-reuse lever for keeping solver 6's own hierarchy cost down on
   this map — it just isn't sufficient on its own given finding above.
3. Alternatively: a machine with more RAM would sidestep this without
   needing a code fix, if that's available.
