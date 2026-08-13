// analyze_coarse_collisions.cpp
//
// Diagnostic tool testing two related theories about why solver 6's
// throughput stops improving (and starts regressing) past a certain
// --flowSolveLevel:
//
//   (A) "same coarse node" collisions: at coarse enough levels, many agents
//       and many tasks map to the *same* top-level coarse node.
//       compute_reduced_assignment (MapCoarsenV1.cpp) gives same-node
//       supply/demand arcs zero cost and no distance information at all
//       (see the `cost[arc] = 0.0` lines around its Step 1), and Step 2's
//       flow-recovery walk pairs same-node agents with same-node tasks
//       strictly in iteration/queue order -- so which agent gets which task
//       is effectively arbitrary with respect to their real fine-map
//       positions.
//   (B) "different coarse node, but spatially close" mismatches: even
//       agents/tasks that *don't* share a node still only get matched via
//       the coarse graph's aggregated inter-node arc costs, not their exact
//       fine positions -- so a pair sitting near a shared node boundary can
//       be scored the same as a pair sitting at opposite corners of two
//       adjacent (large) coarse nodes.
//
// This tool re-solves the *actual* top-level min-cost flow + greedy
// flow-recovery (same algorithm as compute_reduced_assignment's Step 1/2)
// directly against the cached hierarchy, for the real instance's agents and
// flexible task pool, at each level. It then reports, split by whether each
// matched pair shared a coarse node or not:
//   - the real (fine-grid Manhattan) distance of the assignment actually
//     produced
//   - the real distance of an achievable-from-real-positions greedy
//     nearest-neighbor pairing computed on the same subset
//   - the gap between them (real distance being left on the table)
// letting (A) and (B)'s contributions be compared directly instead of
// guessed at.
//
// Usage: ./analyze_coarse_collisions <instance.json> <hierarchy_cache.bin> [max_level] [task_cap]

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "MapCoarsenSerialize.h"
#include "LocalNodeMatch.h"
#include "instance_loader.h"

using namespace MapReductionTest;
using lemon::ListDigraph;
using lemon::NetworkSimplex;

namespace {

// Mirrors the static map_fine_node_to_level_node_local() in MapCoarsenV1.cpp
// (not exposed in the header, so re-derived here from the same public
// to_coarser_node_id chain compute_reduced_assignment itself walks).
int map_fine_node_to_level(const MultiLevelCoarsenedGraph& hierarchy, int fine_node_id, int target_level)
{
    if (target_level < 0 || target_level >= hierarchy.num_levels()) return -1;
    int node_id = fine_node_id;
    for (int level = 0; level < target_level; ++level)
    {
        const CoarsenedGraph* graph = hierarchy.level(level);
        if (!graph) return -1;
        if (node_id < 0 || node_id >= static_cast<int>(graph->to_coarser_node_id.size())) return -1;
        node_id = graph->to_coarser_node_id[node_id];
        if (node_id < 0) return -1;
    }
    return node_id;
}

bool is_valid_node(const CoarsenedGraph& g, int node_id)
{
    return node_id >= 0 && node_id < static_cast<int>(g.map_nodes.size()) && g.map_nodes[node_id] != lemon::INVALID;
}

int manhattan(const SharedEnvironment& env, int loc_a, int loc_b)
{
    const int ra = loc_a / env.cols, ca = loc_a % env.cols;
    const int rb = loc_b / env.cols, cb = loc_b % env.cols;
    return std::abs(ra - rb) + std::abs(ca - cb);
}

// Exact minimum-cost bipartite matching on *real* fine-grid Manhattan
// distance, via NetworkSimplex on the complete bipartite graph between
// agent_locs and task_locs (min(agents,tasks) pairs made). This is the true
// optimum achievable if the matcher had full positional information -- not
// a heuristic. An earlier version of this tool used two progressively
// stronger greedy heuristics here (per-agent-in-order nearest-neighbor,
// then sorted-edge greedy) and *both* were beaten by the actual coarse-
// solver assignment on the cross-node subset -- which turned out to mean
// the heuristics were too weak to trust as a baseline, not that coarse
// routing was actually optimal. Solving exactly removes that ambiguity.
long long optimal_matched_distance(const SharedEnvironment& env, const std::vector<int>& agent_locs, const std::vector<int>& task_locs)
{
    const std::size_t na = agent_locs.size(), nt = task_locs.size();
    const std::size_t k = std::min(na, nt);
    if (k == 0) return 0;

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
        capacity[arc] = 1; cost[arc] = 0;
    }
    for (std::size_t j = 0; j < nt; ++j)
    {
        tnodes[j] = g.addNode();
        supply[tnodes[j]] = 0;
        const auto arc = g.addArc(tnodes[j], sink);
        capacity[arc] = 1; cost[arc] = 0;
    }
    for (std::size_t i = 0; i < na; ++i)
        for (std::size_t j = 0; j < nt; ++j)
        {
            const auto arc = g.addArc(anodes[i], tnodes[j]);
            capacity[arc] = 1;
            cost[arc] = manhattan(env, agent_locs[i], task_locs[j]);
        }

    NetworkSimplex<ListDigraph, int, long long> ns(g);
    ns.costMap(cost).upperMap(capacity).supplyMap(supply).flowMap(flow);
    if (ns.run() != NetworkSimplex<ListDigraph, int, long long>::OPTIMAL)
        return -1; // signals infeasible/failed to caller
    return ns.totalCost<long long>();
}

struct FullAssignmentResult
{
    // Parallel arrays over every matched pair.
    std::vector<int> agent_id, task_id, agent_top_node, task_top_node;
};

// Core of ReducedHierarchy::compute_reduced_assignment's Step 1/Step 2
// (MapCoarsenV1.cpp:1437): given agents/tasks already bucketed by top-level
// node, build the compact coarse flow graph, solve one min-cost flow, and
// recover pairs via the same BFS-over-residual-flow walk. Factored out of
// solve_top_level_assignment so solve_top_level_assignment_with_local_match
// (below) can reuse it on just the *surplus* agents/tasks left over after
// local matching, instead of duplicating ~100 lines of flow-graph plumbing.
// `top_task_ids` is taken by value since recovery pops from it.
FullAssignmentResult run_top_level_flow_and_recover(const CoarsenedGraph& top,
                                                     const std::unordered_map<int, int>& agent_to_top_node,
                                                     std::unordered_map<int, std::list<int>> top_task_ids,
                                                     int total_agent_count)
{
    FullAssignmentResult result;

    std::unordered_map<int, int> start_supply;
    for (const auto& kv : agent_to_top_node)
        start_supply[kv.second]++;

    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();

    std::vector<ListDigraph::Node> top_nodes(top.map_nodes.size(), ListDigraph::Node());
    for (int node_id = 0; node_id < static_cast<int>(top.map_nodes.size()); ++node_id)
    {
        if (!is_valid_node(top, node_id)) continue;
        top_nodes[node_id] = g.addNode();
        supply[top_nodes[node_id]] = 0;
    }

    supply[source] = total_agent_count;
    supply[sink] = -total_agent_count;

    std::unordered_map<int, ListDigraph::Arc> src_arc_by_top, sink_arc_by_top;
    std::unordered_map<int, std::unordered_map<int, ListDigraph::Arc>> move_arc_by_endpoints;

    for (const auto& kv : start_supply)
    {
        if (kv.second <= 0 || !is_valid_node(top, kv.first)) continue;
        const auto arc = g.addArc(source, top_nodes[kv.first]);
        capacity[arc] = kv.second;
        cost[arc] = 0.0;
        src_arc_by_top[kv.first] = arc;
    }
    for (const auto& kv : top_task_ids)
    {
        const int count = static_cast<int>(kv.second.size());
        if (count <= 0 || !is_valid_node(top, kv.first)) continue;
        const auto arc = g.addArc(top_nodes[kv.first], sink);
        capacity[arc] = count;
        cost[arc] = 0.0;
        sink_arc_by_top[kv.first] = arc;
    }
    for (ListDigraph::ArcIt arc(top.g); arc != lemon::INVALID; ++arc)
    {
        const int src_lid = top.g.id(top.g.source(arc));
        const int dst_lid = top.g.id(top.g.target(arc));
        if (src_lid < 0 || dst_lid < 0 ||
            src_lid >= static_cast<int>(top.node_to_maploc.size()) ||
            dst_lid >= static_cast<int>(top.node_to_maploc.size()))
            continue;
        const int src_id = top.node_to_maploc[src_lid];
        const int dst_id = top.node_to_maploc[dst_lid];
        if (!is_valid_node(top, src_id) || !is_valid_node(top, dst_id)) continue;
        if (top_nodes[src_id] == lemon::INVALID || top_nodes[dst_id] == lemon::INVALID) continue;
        const auto coarse_arc = g.addArc(top_nodes[src_id], top_nodes[dst_id]);
        capacity[coarse_arc] = total_agent_count;
        cost[coarse_arc] = top.cost[arc];
        move_arc_by_endpoints[src_id][dst_id] = coarse_arc;
    }

    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost).upperMap(capacity).supplyMap(supply).flowMap(flow);
    if (ns.run() != NetworkSimplex<ListDigraph>::OPTIMAL) return result;

    std::unordered_map<int, int> src_remaining, sink_remaining;
    std::unordered_map<int, std::unordered_map<int, int>> move_remaining;
    for (const auto& kv : src_arc_by_top) src_remaining[kv.first] = ns.flow(kv.second);
    for (const auto& kv : sink_arc_by_top) sink_remaining[kv.first] = ns.flow(kv.second);
    for (const auto& src_kv : move_arc_by_endpoints)
        for (const auto& dst_kv : src_kv.second)
            move_remaining[src_kv.first][dst_kv.first] = ns.flow(dst_kv.second);

    for (const auto& akv : agent_to_top_node)
    {
        const int agent_idx = akv.first;
        const int start_top = akv.second;
        if (start_top < 0 || src_remaining[start_top] <= 0) continue;
        src_remaining[start_top]--;

        std::queue<int> q;
        std::unordered_map<int, int> prev;
        prev[start_top] = -1;
        q.push(start_top);
        int reached = -1;
        while (!q.empty())
        {
            const int u = q.front(); q.pop();
            if (sink_remaining[u] > 0) { reached = u; break; }
            if (move_remaining.find(u) == move_remaining.end()) continue;
            for (const auto& nkv : move_remaining[u])
            {
                if (nkv.second <= 0 || prev.count(nkv.first)) continue;
                prev[nkv.first] = u;
                q.push(nkv.first);
            }
        }
        if (reached < 0) continue;

        std::vector<int> path;
        for (int cur = reached; cur != -1; cur = prev[cur]) path.push_back(cur);
        std::reverse(path.begin(), path.end());
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            if (move_remaining[path[i]][path[i + 1]] > 0) move_remaining[path[i]][path[i + 1]]--;
        }
        sink_remaining[reached]--;

        if (top_task_ids[reached].empty()) continue;
        const int task_idx = top_task_ids[reached].front();
        top_task_ids[reached].pop_front();

        result.agent_id.push_back(agent_idx);
        result.task_id.push_back(task_idx);
        result.agent_top_node.push_back(start_top);
        result.task_top_node.push_back(reached);
    }
    return result;
}

// Re-solves the same top-level min-cost flow + greedy BFS flow-recovery as
// ReducedHierarchy::compute_reduced_assignment's Step 1/Step 2
// (MapCoarsenV1.cpp:1437), operating directly on the already-loaded
// hierarchy so this can be run standalone across levels without going
// through the private ReducedHierarchy singleton.
FullAssignmentResult solve_top_level_assignment(const SharedEnvironment& env,
                                                 const MultiLevelCoarsenedGraph& hierarchy,
                                                 const std::vector<int>& agent_locs,
                                                 const std::vector<int>& task_locs,
                                                 int level)
{
    const CoarsenedGraph* top = hierarchy.level(level);
    if (!top) return FullAssignmentResult();

    std::unordered_map<int, int> agent_to_top_node;
    for (std::size_t i = 0; i < agent_locs.size(); ++i)
    {
        const int top_node = map_fine_node_to_level(hierarchy, agent_locs[i], level);
        if (top_node < 0) continue;
        agent_to_top_node[static_cast<int>(i)] = top_node;
    }

    std::unordered_map<int, std::list<int>> top_task_ids; // top_node -> queue of task indices
    for (std::size_t i = 0; i < task_locs.size(); ++i)
    {
        const int top_node = map_fine_node_to_level(hierarchy, task_locs[i], level);
        if (top_node < 0) continue;
        top_task_ids[top_node].push_back(static_cast<int>(i));
    }

    return run_top_level_flow_and_recover(*top, agent_to_top_node, std::move(top_task_ids),
                                           static_cast<int>(agent_locs.size()));
}

// The *fixed* pipeline: same top-level bucketing as solve_top_level_assignment,
// but before touching the flow graph at all, every node with both agents and
// tasks present has its whole group pulled out and matched directly via
// `matcher` (see LocalNodeMatch.h) on real fine-grid distance. Only the
// `|agents - tasks|` leftover at each node (never both sides at once, by
// construction of an exact min(a,t)-pair matching) is fed into the coarse
// flow graph, via the same run_top_level_flow_and_recover() core used above.
// This directly mirrors the fix design in ai/todo.md so its effect on total
// realized distance can be measured before touching compute_reduced_assignment.
FullAssignmentResult solve_top_level_assignment_with_local_match(const SharedEnvironment& env,
                                                                   const MultiLevelCoarsenedGraph& hierarchy,
                                                                   const std::vector<int>& agent_locs,
                                                                   const std::vector<int>& task_locs,
                                                                   int level,
                                                                   const LocalNodeMatcher& matcher)
{
    const CoarsenedGraph* top = hierarchy.level(level);
    if (!top) return FullAssignmentResult();

    std::unordered_map<int, std::vector<int>> agents_by_node, tasks_by_node;
    for (std::size_t i = 0; i < agent_locs.size(); ++i)
    {
        const int top_node = map_fine_node_to_level(hierarchy, agent_locs[i], level);
        if (top_node < 0) continue;
        agents_by_node[top_node].push_back(static_cast<int>(i));
    }
    for (std::size_t i = 0; i < task_locs.size(); ++i)
    {
        const int top_node = map_fine_node_to_level(hierarchy, task_locs[i], level);
        if (top_node < 0) continue;
        tasks_by_node[top_node].push_back(static_cast<int>(i));
    }

    FullAssignmentResult result;
    std::unordered_map<int, int> surplus_agent_to_top_node;
    std::unordered_map<int, std::list<int>> surplus_top_task_ids;

    for (auto& kv : agents_by_node)
    {
        const int node = kv.first;
        std::vector<int>& node_agent_ids = kv.second;
        const auto tasks_it = tasks_by_node.find(node);
        if (tasks_it == tasks_by_node.end() || tasks_it->second.empty())
        {
            // No local counterpart at all -- every agent here is surplus.
            for (int agent_idx : node_agent_ids)
                surplus_agent_to_top_node[agent_idx] = node;
            continue;
        }
        std::vector<int>& node_task_ids = tasks_it->second;

        std::vector<int> agent_node_locs, task_node_locs;
        agent_node_locs.reserve(node_agent_ids.size());
        task_node_locs.reserve(node_task_ids.size());
        for (int agent_idx : node_agent_ids) agent_node_locs.push_back(agent_locs[agent_idx]);
        for (int task_idx : node_task_ids) task_node_locs.push_back(task_locs[task_idx]);

        const std::vector<LocalMatchPair> pairs =
            matcher(env, node_agent_ids, agent_node_locs, node_task_ids, task_node_locs);

        std::unordered_set<int> matched_agents, matched_tasks;
        for (const auto& p : pairs)
        {
            result.agent_id.push_back(p.agent_id);
            result.task_id.push_back(p.task_id);
            result.agent_top_node.push_back(node);
            result.task_top_node.push_back(node);
            matched_agents.insert(p.agent_id);
            matched_tasks.insert(p.task_id);
        }

        for (int agent_idx : node_agent_ids)
            if (!matched_agents.count(agent_idx))
                surplus_agent_to_top_node[agent_idx] = node;
        for (int task_idx : node_task_ids)
            if (!matched_tasks.count(task_idx))
                surplus_top_task_ids[node].push_back(task_idx);
    }
    // Nodes with tasks but no agents at all: every task there is surplus too.
    for (const auto& kv : tasks_by_node)
    {
        if (agents_by_node.find(kv.first) != agents_by_node.end()) continue;
        for (int task_idx : kv.second)
            surplus_top_task_ids[kv.first].push_back(task_idx);
    }

    // total_agent_count here must be the *surplus* agent count fed into this
    // solve, not the original full batch -- run_top_level_flow_and_recover
    // uses it as the source/sink supply magnitude, which must match the
    // actual number of agents routed through it or NetworkSimplex is
    // over/under-constrained relative to the arcs actually built.
    const int surplus_agent_count = static_cast<int>(surplus_agent_to_top_node.size());
    FullAssignmentResult flow_result = run_top_level_flow_and_recover(
        *top, surplus_agent_to_top_node, std::move(surplus_top_task_ids),
        surplus_agent_count);

    result.agent_id.insert(result.agent_id.end(), flow_result.agent_id.begin(), flow_result.agent_id.end());
    result.task_id.insert(result.task_id.end(), flow_result.task_id.begin(), flow_result.task_id.end());
    result.agent_top_node.insert(result.agent_top_node.end(), flow_result.agent_top_node.begin(), flow_result.agent_top_node.end());
    result.task_top_node.insert(result.task_top_node.end(), flow_result.task_top_node.begin(), flow_result.task_top_node.end());
    return result;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "usage: " << argv[0] << " <instance.json> <hierarchy_cache.bin> [max_level] [task_cap] [min_level]\n";
        return 2;
    }

    SharedEnvironment env;
    float num_tasks_reveal = 1.0f;
    try { populate_env_from_instance(argv[1], env, &num_tasks_reveal); }
    catch (const std::exception& e) { std::cerr << "failed to load instance: " << e.what() << "\n"; return 2; }

    MultiLevelCoarsenedGraph hierarchy;
    if (!load_hierarchy_from_file(argv[2], &env, -1, hierarchy))
    {
        std::cerr << "failed to load hierarchy cache from " << argv[2] << "\n";
        return 2;
    }

    const int max_level = (argc >= 4) ? std::atoi(argv[3]) : hierarchy.num_levels() - 1;
    const int team_size = static_cast<int>(env.curr_states.size());
    int task_cap = (argc >= 5) ? std::atoi(argv[4]) : static_cast<int>(num_tasks_reveal * team_size);
    if (task_cap <= 0 || task_cap > static_cast<int>(env.task_pool.size()))
        task_cap = static_cast<int>(env.task_pool.size());

    std::vector<int> agent_locs;
    agent_locs.reserve(env.curr_states.size());
    for (const auto& s : env.curr_states) agent_locs.push_back(s.location);

    std::vector<int> task_locs;
    task_locs.reserve(task_cap);
    for (int t = 0; t < task_cap; ++t) task_locs.push_back(env.task_pool.at(t).locations[0]);

    std::cout << "agents=" << agent_locs.size() << " tasks_used=" << task_locs.size()
              << " (numTasksReveal=" << num_tasks_reveal << ", teamSize=" << team_size
              << "), hierarchy levels=" << hierarchy.num_levels() << "\n\n";

    // Timing probe: cost of a "fine proximity index" lookup pass (map every
    // agent/task down to a fixed fine level via the already-built hierarchy's
    // to_coarser_node_id chain, and bucket them) -- NOT a flow solve, just
    // array lookups + hashing. This is what a boundary-aware fix would cost
    // per timestep if it probed proximity at a fine level independent of
    // flowSolveLevel, as distinct from actually *solving* min-cost flow at
    // that fine level (which the per-level loop below measures via
    // solve_top_level_assignment and prints as "level N solved in ...").
    for (int probe_level : {1, 2})
    {
        if (probe_level >= hierarchy.num_levels()) continue;
        const auto t0 = std::chrono::high_resolution_clock::now();
        std::unordered_map<int, std::vector<int>> agents_at, tasks_at;
        for (int loc : agent_locs)
        {
            const int n = map_fine_node_to_level(hierarchy, loc, probe_level);
            if (n >= 0) agents_at[n].push_back(loc);
        }
        for (int loc : task_locs)
        {
            const int n = map_fine_node_to_level(hierarchy, loc, probe_level);
            if (n >= 0) tasks_at[n].push_back(loc);
        }
        // Also check each populated agent bucket's graph-adjacent buckets
        // (the "adjacent, not just identical, bucket" boundary check) --
        // walk the level's own coarse graph arcs, no flow solve involved.
        const CoarsenedGraph* g = hierarchy.level(probe_level);
        int adjacency_checks = 0;
        if (g)
        {
            for (const auto& kv : agents_at)
            {
                const int node_id = kv.first;
                if (node_id < 0 || node_id >= static_cast<int>(g->map_nodes.size())) continue;
                const auto lemon_node = g->map_nodes[node_id];
                if (lemon_node == lemon::INVALID) continue;
                for (ListDigraph::OutArcIt a(g->g, lemon_node); a != lemon::INVALID; ++a)
                {
                    const int nb_lid = g->g.id(g->g.target(a));
                    if (nb_lid < 0 || nb_lid >= static_cast<int>(g->node_to_maploc.size())) continue;
                    adjacency_checks++;
                }
            }
        }
        const auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "[timing probe] fine-index lookup+bucket+adjacency-scan at level " << probe_level
                  << ": " << std::chrono::duration<double>(t1 - t0).count() << "s"
                  << " (" << agents_at.size() << " agent buckets, " << tasks_at.size()
                  << " task buckets, " << adjacency_checks << " adjacency edges scanned)\n";
    }
    std::cout << "\n";

    std::cout << "level,matched,same_node_pairs,diff_node_pairs,"
                 "same_node_actual_dist,same_node_greedy_dist,same_node_excess,"
                 "diff_node_actual_dist,diff_node_greedy_dist,diff_node_excess,"
                 "same_node_excess_per_pair,diff_node_excess_per_pair,"
                 "fixed_matched,fixed_total_dist,total_dist_before_fix,total_waste_eliminated_by_fix\n";

    const int min_level = (argc >= 6) ? std::atoi(argv[5]) : 1;
    for (int level = min_level; level <= max_level; ++level)
    {
        const auto start = std::chrono::high_resolution_clock::now();
        const FullAssignmentResult res = solve_top_level_assignment(env, hierarchy, agent_locs, task_locs, level);
        const auto end = std::chrono::high_resolution_clock::now();

        std::vector<int> same_agent_locs, same_task_locs, diff_agent_locs, diff_task_locs;
        long long same_actual = 0, diff_actual = 0;
        for (std::size_t i = 0; i < res.agent_id.size(); ++i)
        {
            const int a_loc = agent_locs[res.agent_id[i]];
            const int t_loc = task_locs[res.task_id[i]];
            const int d = manhattan(env, a_loc, t_loc);
            if (res.agent_top_node[i] == res.task_top_node[i])
            {
                same_actual += d;
                same_agent_locs.push_back(a_loc);
                same_task_locs.push_back(t_loc);
            }
            else
            {
                diff_actual += d;
                diff_agent_locs.push_back(a_loc);
                diff_task_locs.push_back(t_loc);
            }
        }

        const long long same_greedy = optimal_matched_distance(env, same_agent_locs, same_task_locs);
        const long long diff_greedy = optimal_matched_distance(env, diff_agent_locs, diff_task_locs);
        const long long same_excess = same_actual - same_greedy;
        const long long diff_excess = diff_actual - diff_greedy;
        const double same_per_pair = same_agent_locs.empty() ? 0.0 : static_cast<double>(same_excess) / same_agent_locs.size();
        const double diff_per_pair = diff_agent_locs.empty() ? 0.0 : static_cast<double>(diff_excess) / diff_agent_locs.size();

        // Fixed pipeline: pull same-node groups out and match them via
        // match_local_node_exact (LocalNodeMatch.h) before the coarse flow
        // graph ever sees them, per ai/todo.md's fix design. Compares total
        // realized distance against the unmodified pipeline above on the
        // exact same agent/task batch.
        const FullAssignmentResult fixed_res = solve_top_level_assignment_with_local_match(
            env, hierarchy, agent_locs, task_locs, level, match_local_node_exact);
        long long fixed_total = 0;
        for (std::size_t i = 0; i < fixed_res.agent_id.size(); ++i)
            fixed_total += manhattan(env, agent_locs[fixed_res.agent_id[i]], task_locs[fixed_res.task_id[i]]);
        const long long total_before = same_actual + diff_actual;
        const long long waste_eliminated = total_before - fixed_total;

        std::cout << level << "," << res.agent_id.size() << ","
                  << same_agent_locs.size() << "," << diff_agent_locs.size() << ","
                  << same_actual << "," << same_greedy << "," << same_excess << ","
                  << diff_actual << "," << diff_greedy << "," << diff_excess << ","
                  << same_per_pair << "," << diff_per_pair << ","
                  << fixed_res.agent_id.size() << "," << fixed_total << ","
                  << total_before << "," << waste_eliminated << "\n";
        std::cerr << "  level " << level << " solved in "
                  << std::chrono::duration<double>(end - start).count() << "s\n";
    }

    return 0;
}
