// LocalNodeMatch.h
//
// Pluggable "within one coarse node" agent<->task matcher. See
// ai/todo.md ("Solver 6: within-coarse-node agent<->task pairing is
// arbitrary, not distance-informed") for the motivating problem: when an
// agent and a task map to the same top-level coarse node, the per-timestep
// coarse flow solve in compute_reduced_assignment (MapCoarsenV1.cpp) gives
// same-node arcs zero cost, so it has no signal to prefer one fine-grained
// pairing over another -- the recovery walk just pairs them in
// insertion/queue order.
//
// A LocalNodeMatcher takes one such group directly (the agents and tasks
// that already share a coarse node, or any other small group the caller
// wants matched on real distance) and returns up to min(na, nt) pairs.
// Whichever ids don't appear in the result are surplus and should be handed
// to whatever broader mechanism (e.g. the coarse flow graph) would otherwise
// have placed them.

#pragma once

#include "../inc/SharedEnv.h"

#include <functional>
#include <vector>

namespace MapReductionTest {

struct LocalMatchPair
{
    int agent_id;
    int task_id;
};

using LocalNodeMatcher = std::function<std::vector<LocalMatchPair>(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs)>;

// Default/reference matcher: exact minimum-cost bipartite matching (LEMON
// NetworkSimplex) on real fine-grid Manhattan distance between agent_locs[i]
// and task_locs[j]. Optimal for this cost metric. Groups passed to this
// function are expected to be small (one coarse node's worth of agents and
// tasks), so the O(na*nt) complete-bipartite-graph construction is cheap in
// practice -- see the per-node group sizes quantified in ai/todo.md.
std::vector<LocalMatchPair> match_local_node_exact(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs);

// ---------------------------------------------------------------------------
// Running many independent matches at once (ai/parallel_local_matching_plan.md)
// ---------------------------------------------------------------------------

// One call's worth of match_local_node_exact input, plus its output.
struct LocalMatchJob
{
    std::vector<int> agent_ids, agent_locs, task_ids, task_locs;
    std::vector<LocalMatchPair> pairs;   // filled in by run_local_match_jobs
    double time_s = 0.0;                 // time spent matching this job
};

struct LocalMatchRunStats
{
    int jobs = 0;
    long long max_work = 0;    // largest job's agents x tasks
    long long total_work = 0;  // sum of agents x tasks over all jobs
    double wall_s = 0.0;       // wall-clock time of the whole call
    double cpu_s = 0.0;        // summed per-job time (the serial cost)
    int threads = 1;           // threads actually used
};

// Runs match_local_node_exact on every job, filling job.pairs. Jobs are
// independent (each builds its own LEMON graph), so they run on up to
// get_local_match_threads() threads, largest first. Runs serially when there
// are fewer than 2 jobs or total work is below kParallelMatchMinWork.
// Output is identical for any thread count: each job's pairs depend only on
// that job's input.
LocalMatchRunStats run_local_match_jobs(const SharedEnvironment& env, std::vector<LocalMatchJob>& jobs);

// Thread count for run_local_match_jobs. Defaults to the build-time
// SCHEDULER_MATCH_THREADS; the validator changes it at runtime to compare
// thread counts in one process.
void set_local_match_threads(int threads);
int get_local_match_threads();

// Summed per-job matching time (seconds) since the last call, then reset.
// Read once per scheduler call for the SchedulerLocalMatchCpuTime metric.
double take_local_match_cpu_time();

} // namespace MapReductionTest
