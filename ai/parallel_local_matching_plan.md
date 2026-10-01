# Plan: run solver 6's local matching in parallel

Status: **implemented 2026-09-30 (steps 0-2; step 3 tuning partly, step 4
solver 7 not done). See "Implementation and results" at the end.** Builds on
`ai/local_node_matching.md` (the matcher), `ai/local_node_matching_runtime.md`
(where its time goes) and `ai/hierarchical_matching.md` (the cascade). Uses
the same threading style as `ai/parallel_guide_paths_plan.md`.

## Why now

`ai/local_node_matching_runtime.md` (2026-08-20) concluded parallelising
local matching wasn't worth it: at 20k agents on scene_mp_4p_03 the worst
single call spent 0.40 s in local matching (level 8), well inside the
1,000 ms step. Two things have changed since:

1. **The final sweep goes to 80k agents** at levels 2/4/6/8 and in
   hierarchical-only mode (`instances/thesis_benchmarks/README.md`, "Final
   run plan"). `match_local_node_exact` costs about `agents × tasks` per
   group, so if group sizes grow with the agent count, 4× the agents means
   about 16× the work per group. A rough extrapolation of the 0.40 s puts the
   80k level-8 worst call at several seconds. Not measured yet (see step 0).
2. **Scheduler time now comes straight out of guide-path time.** The
   planner's stage 2 builds guide paths until the step's deadline, which is
   about 850 ms minus whatever the scheduler used
   (`ai/parallel_guide_paths_plan.md`). On IH, guide paths are the
   bottleneck, so every millisecond the scheduler saves is more A* searches.

## Where the work is

`match_local_node_exact` (`map_reduction_test/LocalNodeMatch.cpp`) is
called once per coarse node that has both agents and tasks, from three
loops:

| Loop | File | Runs when |
|---|---|---|
| Step 1 of `compute_reduced_assignment` | `MapCoarsenV1.cpp` (~line 1700) | every solver 6 call, at `--flowSolveLevel` |
| Cascade loop of `compute_hierarchical_assignment`, once per matched level | `MapCoarsenV1.cpp` (~line 2120) | solver 6 with `--minCascadeLevel` below the flow level (hierarchical-only mode) |
| Step 1 of solver 7's assignment | `EdgeAugmentedCoarsen.cpp` (~line 430) | solver 7 only |

In every loop, each call:

- builds its own LEMON graph and `NetworkSimplex` on the stack and solves it;
- reads only `env.cols` and its own four input vectors;
- returns pairs whose agents and tasks all belong to that node, so no two
  calls can return the same agent or task.

The calls are independent, and LEMON keeps no global state, so they can run
on separate threads without locks. The only shared writes are in the code
around each call (`assignments`, `start_supply`, `top_task_ids`,
`agent_to_top_node`, the `agent_matched`/`task_matched` flags, the counters),
and those can stay on the main thread.

## Proposed solution

### Setting

`SCHEDULER_MATCH_THREADS` in `MapCoarsenV1.cpp`, overridable at build time
with `-DSCHEDULER_MATCH_THREADS=<n>`, the same pattern as
`PLANNER_GUIDE_PATH_THREADS` in `default_planner/const.h`.

- `1`: the current serial loops, unchanged.
- `>1`: the version below. Planned default: 6, the same as the planner. The
  scheduler and the planner run one after the other in `Entry::compute`, so
  their threads never compete.

### One helper, used by all three loops

```cpp
struct LocalMatchJob {
    std::vector<int> agent_ids, agent_locs, task_ids, task_locs;
    std::vector<LocalMatchPair> pairs;   // filled in by the helper
};

// Runs match_local_node_exact on every job, on up to `threads` threads.
// Returns wall-clock time; adds summed per-job time to *cpu_time_out.
double run_local_match_jobs(const SharedEnvironment& env,
                            std::vector<LocalMatchJob>& jobs,
                            int threads, double* cpu_time_out);
```

Each loop is split into three passes:

1. **Collect (main thread).** Walk `agents_by_node` (or `agent_idxs_by_node`)
   exactly as now. Nodes without tasks are handled as now. Every node with
   both sides becomes a `LocalMatchJob`, appended in the order the serial
   loop would have visited it.
2. **Match (worker threads).** `run_local_match_jobs`. Workers take the next
   job from a shared atomic counter and write only to that job's `pairs`.
3. **Merge (main thread).** Walk the jobs in collect order and run the
   existing post-call code unchanged (record pairs, count, mark leftovers as
   surplus or carry them forward).

Because merge order equals the serial loop's order, the output (including
the insertion order into the `assignments` `unordered_map`) is identical to
the serial version for any thread count. This is what makes testing easy
(see below).

### Scheduling details

- **Largest jobs first.** Workers take jobs in order of decreasing
  `agents × tasks` (a sorted index list over `jobs`; the jobs themselves
  stay in collect order for the merge). This stops one big job, picked up
  last, from leaving the other threads idle.
- **Stay serial when there's little work.** If there are fewer than 2 jobs,
  or the total `agents × tasks` over all jobs is below a threshold
  (`kParallelMatchMinWork`, starting guess 20,000, tuned in step 3), run the
  jobs on the calling thread. The cascade calls the helper up to once per
  level (about 11 per step in hierarchical-only mode), and most shallow-level
  calls have many tiny jobs, where starting threads would cost more than it
  saves.
- **Threads: start per call first, pool only if needed.** Start with
  `std::thread` per parallel call, joined before the merge, the same as the
  planner. Starting 5 threads costs tens of microseconds each. If step 3
  shows this matters, replace the helper's inside with a small persistent
  pool; nothing outside the helper changes.

### Metrics

- `SchedulerLocalMatchTime` becomes wall-clock time of the match passes
  (what the scheduler actually spent). With 1 thread this is the same as
  today's value.
- New `SchedulerLocalMatchCpuTime`: summed per-job time, i.e. today's
  meaning. The ratio of the two is the speedup. Plumbed the same way as
  `SchedulerLocalMatchTime` (`ScheduleTiming` → `TimeStepMetric` → JSON).
  Both push sites in `CompetitionSystem.cpp` need it (the catch-up
  duplicate-row bug was fixed on 2026-08-26, but check both sites anyway;
  `ai/local_node_matching_runtime.md` describes how the first attempt at
  `SchedulerLocalMatchTime` missed one).
- Add `scheduler_match_threads` to the output JSON next to
  `schedulerHierarchyNumLevels`, so each result records how it was run.

## Steps

0. **Measure first.** Before writing any threading code, run solver 6 at
   80k on scene_mp_4p_03 and IH, `-s 50`, at level 8 and in
   hierarchical-only mode, and log per call: job count, largest job's
   `agents × tasks`, total `agents × tasks`, `SchedulerLocalMatchTime`, and
   the whole scheduler time. This tells us:
   - whether local matching is a real share of the step at 80k (if it's
     under about 50 ms per call, stop here and record that);
   - the largest job's share of the total, which caps the speedup (below).
1. **Helper + Step 1 of `compute_reduced_assignment`.** Add
   `run_local_match_jobs` to `LocalNodeMatch.{h,cpp}` and switch Step 1 to
   collect / match / merge.
2. **Cascade loop**, same change.
3. **Tune** `kParallelMatchMinWork` and check thread start cost on the
   step-0 runs.
4. **Solver 7** (optional): same change to `EdgeAugmentedCoarsen.cpp`. Only
   2 sweep runs use solver 7, both at level 4, where local matching is
   small; do it only if it's a few lines.

## Testing

1. **Identical output.** `./build/hierarchical_matching_validator` already
   runs the matchers on a frozen batch with no timing dependence. Add a
   check that runs each mode with 1 and with 6 threads and compares the
   assignment maps: they must be identical, not just equally good. Run it
   on `tiny.json`, the disconnected 8×8 map, `warehouseSmall_200` at levels
   4 and 6, and one 80k thesis instance at level 8 and hierarchical-only.
   A difference means the merge order or a shared write is wrong.
2. **Thread sanitiser build** (`-fsanitize=thread`) of the validator, run on
   `warehouseSmall_200`, to catch any shared write missed above.
3. **Speed, frozen batch.** Time the local-match passes in the validator on
   the 80k instance with 1, 2, 4 and 6 threads. This is the clean speedup
   number; full simulation runs are too noisy for it
   (`ai/hierarchical_matching.md`, "Validation").
4. **Full run.** The step-0 runs again with 6 threads: 0 errors,
   `SchedulerLocalMatchTime` down by roughly the frozen-batch speedup, and
   the planner's guide-path count per step not lower than before.

## What speedup to expect

Parallelism is across nodes, not inside one. A call can't finish faster
than its largest job, so the speedup is at most
`total work / largest job`, and at most the thread count.

- **Level 8:** at 20k the largest group was about 2.3% of the call's
  matching time, so 6 threads could give close to 6×.
- **Hierarchical-only mode:** the cascade's shallow levels have many small
  jobs and parallelise well. The hand-off call at the top level has only
  one node (per connected region), so whatever is left there is one job and
  runs serially. How much is left there at 80k is one of the step-0
  questions. If that one job turns out to dominate, parallelism won't help
  it; a faster matcher would (see "Alternatives").
- **Levels 2 and 4:** local matching is already under 0.1% of scheduler
  time there (the flow solve dominates), so expect no visible change.

## Caveats

- **Core count.** Like the planner's threads, results depend on the
  machine. The output records the thread count; the thesis should mention
  it for solver 6 (solvers 1, 5 and 7 are unaffected, except 7 if step 4 is
  done).
- **Memory.** Each running job holds its own complete bipartite graph
  (`agents × tasks` arcs). With up to 6 at once this is 6× the peak of the
  serial version. Fine unless a single group reaches thousands of agents
  and tasks; step 0 will show the largest group size.
- **No change to what is matched.** Same pairs as serial, by construction.
  Any throughput change in full runs comes only from the time saved
  (mainly more guide-path searches in the planner).

## Alternatives considered

- **A faster matcher per group** (reuse one LEMON graph between calls, or a
  Hungarian / auction algorithm on a cost matrix instead of building a
  `ListDigraph` each time). Cuts the per-call fixed cost that
  `cascade_level_stride` was added to avoid, and helps the single big
  top-level job that parallelism can't. More work and changes numerical
  tie-breaking, so output would no longer be identical to today. Worth
  doing after this if step 0 shows the top-level job dominates.
- **Splitting a single big group across threads.** Not possible with an
  exact min-cost matching without a parallel solver; out of scope.
- **Running the scheduler and the planner at the same time.** The planner
  needs the scheduler's goals, so this would mean planning on the previous
  step's assignment. Bigger change to the pipeline; not considered here.

## Implementation and results

### What was built (2026-09-30)

- `map_reduction_test/LocalNodeMatch.{h,cpp}`: `LocalMatchJob`,
  `LocalMatchRunStats`, `run_local_match_jobs()` (largest jobs first, the
  calling thread works too, serial below `kParallelMatchMinWork` = 20,000 or
  with fewer than 2 jobs), `set_local_match_threads()` /
  `get_local_match_threads()` (runtime override, used by the validator), and
  `take_local_match_cpu_time()`. Thread count: `SCHEDULER_MATCH_THREADS`
  (default 6), overridable with `-DSCHEDULER_MATCH_THREADS=<n>`. Defined in
  `LocalNodeMatch.cpp`, not `MapCoarsenV1.cpp` as planned, since the helper
  lives there.
- `MapCoarsenV1.cpp`: Step 1 of `compute_reduced_assignment` and the cascade
  loop of `compute_hierarchical_assignment` are now collect / match / merge.
  Each logs a `local match stats:` line per call (jobs, largest and total
  agents × tasks, wall and CPU ms, threads).
- Metrics: `SchedulerLocalMatchTime` is now wall-clock;
  new `SchedulerLocalMatchCpuTime` per step (`ScheduleTiming.local_match_cpu_time`
  → `TimeStepMetric` → JSON); new top-level `schedulerMatchThreads`.
  There is only one metrics push site in `CompetitionSystem.cpp` now; it
  has both fields.
- Solver 7 (step 4) not done: its Step 1 in `EdgeAugmentedCoarsen.cpp`
  still calls `match_local_node_exact` serially, so its
  `SchedulerLocalMatchCpuTime` is 0 (its `schedulerMatchThreads` still
  reports the setting).
- `utils/validation/validate_hierarchical_matching.cpp`: check 5 runs each
  mode (single-level, cascade 1 → level, hierarchical-only 1 → top) with
  1, 2, 4 and 6 threads, requires identical assignments, and prints the
  frozen-batch timing.

### Tests

1. **Identical output:** check 5 passed (35/35 checks) on `tiny.json`,
   `tinyComplex.json`, `warehouseSmall_200` at levels 4 and 6, and IH 80k at
   level 8 (see below).
2. **ThreadSanitizer** build of the validator on `warehouseSmall_200` level
   4: 0 warnings, 35/35 checks. (Needs `setarch $(uname -m) -R` on this
   kernel, otherwise TSan aborts with "unexpected memory mapping".)
3. **Frozen batch, IH_mp_2p_01 80k, level 8** (all 80k agents and 120k
   tasks unassigned, i.e. the first decision; later decisions are smaller):

   | Mode | 1 thread | 2 | 4 | 6 threads | Speedup at 6 |
   |---|---|---|---|---|---|
   | Single-level, level 8 | 49,256 ms | 24,372 | 12,474 | 8,338 | 5.95× |
   | Cascade 1 → 8 | 546 ms | 254 | 168 | 143 | 2.85× |
   | Hierarchical-only 1 → 11 | 511 ms | 249 | 168 | 144 | 2.83× |

   89 jobs at level 8; the largest is about 9.5M agents × tasks (about
   3,000 × 3,000), 4.4% of the total. Peak memory 5.7 GB.

   **scene_mp_4p_03 80k, level 8** (same setup), 35/35 checks, peak 6.1 GB:

   | Mode | 1 thread | 2 | 4 | 6 threads | Speedup at 6 |
   |---|---|---|---|---|---|
   | Single-level, level 8 | 7,233 ms | 3,839 | 1,985 | 1,338 | 6.0× |
   | Cascade 1 → 8 | 728 ms | 450 | 318 | 281 | 1.9× |
   | Hierarchical-only 1 → 12 | 756 ms | 477 | 339 | 300 | 1.9× |

   304 jobs at level 8, the largest about 1.1M agents × tasks (1.7%).

**Step 0 answer:** local matching is a real cost at 80k: 49 s for the first
level-8 decision single-threaded, so well worth parallelising. Later
decisions are much smaller (only newly free agents and new tasks).
