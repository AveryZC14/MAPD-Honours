// LocalNodeMatch.cpp
// See LocalNodeMatch.h.

#include "LocalNodeMatch.h"

#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <numeric>
#include <thread>

// Threads for run_local_match_jobs (ai/parallel_local_matching_plan.md).
// 1 = serial. Override at build time with -DSCHEDULER_MATCH_THREADS=<n>.
#ifndef SCHEDULER_MATCH_THREADS
#define SCHEDULER_MATCH_THREADS 6
#endif

namespace MapReductionTest {

namespace {

long long manhattan_distance(const SharedEnvironment& env, int loc_a, int loc_b)
{
    const int ra = loc_a / env.cols, ca = loc_a % env.cols;
    const int rb = loc_b / env.cols, cb = loc_b % env.cols;
    return std::abs(ra - rb) + std::abs(ca - cb);
}

} // namespace

std::vector<LocalMatchPair> match_local_node_exact(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs)
{
    using lemon::ListDigraph;
    using lemon::NetworkSimplex;

    std::vector<LocalMatchPair> result;

    const std::size_t na = agent_ids.size();
    const std::size_t nt = task_ids.size();
    if (na != agent_locs.size() || nt != task_locs.size())
        return result;
    const std::size_t k = std::min(na, nt);
    if (k == 0)
        return result;

    ListDigraph g;
    ListDigraph::ArcMap<long long> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g);
    ListDigraph::NodeMap<int> supply(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();
    supply[source] = static_cast<int>(k);
    supply[sink] = -static_cast<int>(k);

    std::vector<ListDigraph::Node> anodes(na), tnodes(nt);
    for (std::size_t i = 0; i < na; ++i)
    {
        anodes[i] = g.addNode();
        supply[anodes[i]] = 0;
        const auto arc = g.addArc(source, anodes[i]);
        capacity[arc] = 1;
        cost[arc] = 0;
    }
    for (std::size_t j = 0; j < nt; ++j)
    {
        tnodes[j] = g.addNode();
        supply[tnodes[j]] = 0;
        const auto arc = g.addArc(tnodes[j], sink);
        capacity[arc] = 1;
        cost[arc] = 0;
    }

    std::vector<std::vector<ListDigraph::Arc>> match_arc(na, std::vector<ListDigraph::Arc>(nt));
    for (std::size_t i = 0; i < na; ++i)
        for (std::size_t j = 0; j < nt; ++j)
        {
            const auto arc = g.addArc(anodes[i], tnodes[j]);
            capacity[arc] = 1;
            cost[arc] = manhattan_distance(env, agent_locs[i], task_locs[j]);
            match_arc[i][j] = arc;
        }

    NetworkSimplex<ListDigraph, int, long long> ns(g);
    ns.costMap(cost).upperMap(capacity).supplyMap(supply).flowMap(flow);
    if (ns.run() != NetworkSimplex<ListDigraph, int, long long>::OPTIMAL)
        return result;

    // Read flow via ns.flow(arc) (the solver's own accessor), not by
    // indexing the external ArcMap passed to flowMap() -- the latter is not
    // reliably populated post-solve in this LEMON version, matching the
    // pattern already used elsewhere in this codebase (e.g.
    // run_top_level_flow_and_recover / compute_reduced_assignment, which
    // read via ns.flow(arc) rather than an external flow map).
    result.reserve(k);
    for (std::size_t i = 0; i < na; ++i)
        for (std::size_t j = 0; j < nt; ++j)
            if (ns.flow(match_arc[i][j]) > 0)
                result.push_back({agent_ids[i], task_ids[j]});

    return result;
}

namespace {

// Below this total agents x tasks, starting threads costs more than it saves.
constexpr long long kParallelMatchMinWork = 20000;

std::atomic<int> g_match_threads{SCHEDULER_MATCH_THREADS};
std::mutex g_cpu_time_mutex;
double g_cpu_time_since_take = 0.0;

void run_job(const SharedEnvironment& env, LocalMatchJob& job)
{
    const auto t0 = std::chrono::steady_clock::now();
    job.pairs = match_local_node_exact(env, job.agent_ids, job.agent_locs, job.task_ids, job.task_locs);
    job.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

void set_local_match_threads(int threads) { g_match_threads = std::max(1, threads); }
int get_local_match_threads() { return g_match_threads; }

double take_local_match_cpu_time()
{
    std::lock_guard<std::mutex> lock(g_cpu_time_mutex);
    const double t = g_cpu_time_since_take;
    g_cpu_time_since_take = 0.0;
    return t;
}

LocalMatchRunStats run_local_match_jobs(const SharedEnvironment& env, std::vector<LocalMatchJob>& jobs)
{
    LocalMatchRunStats stats;
    const auto t0 = std::chrono::steady_clock::now();
    stats.jobs = static_cast<int>(jobs.size());
    std::vector<long long> work(jobs.size());
    for (std::size_t j = 0; j < jobs.size(); ++j)
    {
        work[j] = static_cast<long long>(jobs[j].agent_ids.size()) * static_cast<long long>(jobs[j].task_ids.size());
        stats.max_work = std::max(stats.max_work, work[j]);
        stats.total_work += work[j];
    }

    const int threads = std::min<int>(g_match_threads, static_cast<int>(jobs.size()));
    if (threads < 2 || stats.total_work < kParallelMatchMinWork)
    {
        for (auto& job : jobs)
            run_job(env, job);
    }
    else
    {
        // largest jobs first, so one big job picked up last can't leave the
        // other threads idle; results stay in each job's own slot
        std::vector<std::size_t> order(jobs.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(),
                         [&](std::size_t a, std::size_t b) { return work[a] > work[b]; });
        std::atomic<std::size_t> next(0);
        auto worker = [&]() {
            std::size_t k;
            while ((k = next.fetch_add(1)) < order.size())
                run_job(env, jobs[order[k]]);
        };
        std::vector<std::thread> pool;
        for (int t = 1; t < threads; ++t)
            pool.emplace_back(worker);
        worker();  // the calling thread works too
        for (auto& th : pool)
            th.join();
        stats.threads = threads;
    }

    for (const auto& job : jobs)
        stats.cpu_s += job.time_s;
    stats.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    {
        std::lock_guard<std::mutex> lock(g_cpu_time_mutex);
        g_cpu_time_since_take += stats.cpu_s;
    }
    return stats;
}

} // namespace MapReductionTest
