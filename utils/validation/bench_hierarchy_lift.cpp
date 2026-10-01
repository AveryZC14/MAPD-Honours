// bench_hierarchy_lift.cpp
//
// Measures the existing coarse-to-fine lift (ReducedHierarchy::
// lift_coarse_paths_to_fine) on the kind of path the hierarchical guide-path
// plan would produce (ai/hierarchical_guide_paths_plan.md): for random
// (agent start, task location) pairs from the instance, find the cheapest
// coarse path at level L with Dijkstra on that level's graph (the same arc
// costs the coarse flow uses), lift it, and compare against the true
// shortest distance on the fine map (BFS).
//
// Per level it reports how often the lift succeeds without the full-map
// fallback, why it fails, lifted length / shortest length, and timings. Every
// returned path is checked: right endpoints, walkable cells, 4-adjacent steps.
//
// Usage: ./bench_hierarchy_lift <instance.json> --hierarchyCache <path>
//            [--levels 2,4,6,8] [--pairs 200] [--seed 1] [--csv out.csv]
//            [--maxPathCells 5000] [--coarseHeuristicFactor 0]
//
// --coarseHeuristicFactor f: coarse A*'s heuristic unit is f x 2^L (a
// level-L hop typically costs about 2^L); 0 (default) uses the level's
// cheapest arc cost, which keeps routes as cheap as Dijkstra's.
//
// Also runs the planner's corridor A* (ai/hierarchical_guide_paths_plan.md,
// "Corridor A*"): the planner's A* (no congestion) limited to the coarse
// path's nodes, with margin 0 and 1, and once per pair the same A* on the
// whole map as a reference. Corridor time excludes the coarse search, like
// the lift's.
//
// Results from before every level was anchored (2026-10-01, when this was
// still an option) are in outputs/hierarchy_lift_bench/*_a0.*.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <deque>
#include <list>
#include <map>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "instance_loader.h"
#include "search.h"
#include "TrajLNS.h"

using namespace std;
using MapReductionTest::CoarsenedGraph;
using MapReductionTest::ReducedHierarchy;

namespace {

using Clock = chrono::high_resolution_clock;

double ms_since(Clock::time_point t)
{
    return chrono::duration<double, milli>(Clock::now() - t).count();
}

// Shortest distance start -> goal on the fine map (4-connected, unit cost).
// -1 if unreachable. Scratch arrays are reused across calls.
int fine_bfs_distance(const SharedEnvironment& env, int start, int goal,
                      vector<int>& dist, vector<int>& touched)
{
    for (int c : touched) dist[c] = -1;
    touched.clear();
    if (start == goal) return 0;
    deque<int> q;
    dist[start] = 0;
    touched.push_back(start);
    q.push_back(start);
    const int rows = env.rows, cols = env.cols;
    while (!q.empty())
    {
        const int u = q.front();
        q.pop_front();
        const int r = u / cols, c = u % cols;
        const int nbrs[4] = {r > 0 ? u - cols : -1, r + 1 < rows ? u + cols : -1,
                             c > 0 ? u - 1 : -1, c + 1 < cols ? u + 1 : -1};
        for (int v : nbrs)
        {
            if (v < 0 || env.map[v] == 1 || dist[v] >= 0) continue;
            dist[v] = dist[u] + 1;
            touched.push_back(v);
            if (v == goal) return dist[v];
            q.push_back(v);
        }
    }
    return -1;
}

// Reference: the plain Dijkstra coarse_path used before 2026-10-01's A*
// (hash-map tables, no heuristic), to check A* finds equally cheap routes.
vector<int> reference_dijkstra(const CoarsenedGraph& g, int from, int to, int* expanded_out)
{
    *expanded_out = 0;
    if (from < 0 || to < 0) return {};
    if (from == to) return {from};
    std::unordered_map<int, double> best;
    std::unordered_map<int, int> prev;
    using Item = pair<double, int>;
    priority_queue<Item, vector<Item>, greater<Item>> open;
    best[from] = 0.0;
    open.push({0.0, from});
    while (!open.empty())
    {
        const auto [d, u] = open.top();
        open.pop();
        if (d > best[u]) continue;
        ++*expanded_out;
        if (u == to) break;
        for (lemon::ListDigraph::OutArcIt a(g.g, g.map_nodes[u]); a != lemon::INVALID; ++a)
        {
            const int v_lid = g.g.id(g.g.target(a));
            if (v_lid < 0 || v_lid >= static_cast<int>(g.node_to_maploc.size())) continue;
            const int v = g.node_to_maploc[v_lid];
            if (v < 0) continue;
            const double nd = d + g.cost[a];
            auto it = best.find(v);
            if (it == best.end() || nd < it->second)
            {
                best[v] = nd;
                prev[v] = u;
                open.push({nd, v});
            }
        }
    }
    if (!prev.count(to)) return {};
    vector<int> path{to};
    while (path.back() != from) path.push_back(prev[path.back()]);
    reverse(path.begin(), path.end());
    return path;
}

// Sum of arc costs along a coarse path (-1 if two consecutive nodes aren't joined).
double coarse_path_cost(const CoarsenedGraph& g, const vector<int>& path)
{
    double total = 0.0;
    for (size_t k = 1; k < path.size(); ++k)
    {
        double arc_cost = -1.0;
        for (lemon::ListDigraph::OutArcIt a(g.g, g.map_nodes[path[k - 1]]); a != lemon::INVALID; ++a)
        {
            const int v_lid = g.g.id(g.g.target(a));
            if (v_lid >= 0 && v_lid < static_cast<int>(g.node_to_maploc.size()) && g.node_to_maploc[v_lid] == path[k])
            {
                if (arc_cost < 0.0 || g.cost[a] < arc_cost) arc_cost = g.cost[a];
            }
        }
        if (arc_cost < 0.0) return -1.0;
        total += arc_cost;
    }
    return total;
}

template <class Path>
bool path_is_valid(const SharedEnvironment& env, const Path& path, int start, int goal)
{
    if (path.empty() || path.front() != start || path.back() != goal) return false;
    int prev = -1;
    for (int c : path)
    {
        if (c < 0 || c >= static_cast<int>(env.map.size()) || env.map[c] == 1) return false;
        if (prev >= 0)
        {
            const int dr = abs(c / env.cols - prev / env.cols);
            const int dc = abs(c % env.cols - prev % env.cols);
            if (dr + dc != 1) return false;
        }
        prev = c;
    }
    return true;
}

const char* reason_name(int r)
{
    switch (r)
    {
        case ReducedHierarchy::LIFT_OK: return "ok";
        case ReducedHierarchy::LIFT_NO_START: return "no_start";
        case ReducedHierarchy::LIFT_NO_TARGET: return "no_target";
        case ReducedHierarchy::LIFT_MISSING_BRIDGE: return "missing_bridge";
        case ReducedHierarchy::LIFT_LEAD_IN_WRONG_PARENT: return "lead_in_wrong_parent";
        case ReducedHierarchy::LIFT_LEAD_IN_FAILED: return "lead_in_failed";
        case ReducedHierarchy::LIFT_LEAD_OUT_WRONG_PARENT: return "lead_out_wrong_parent";
        case ReducedHierarchy::LIFT_LEAD_OUT_FAILED: return "lead_out_failed";
        case ReducedHierarchy::LIFT_LENGTH_CAP: return "length_cap";
    }
    return "unknown";
}

double percentile(vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    sort(v.begin(), v.end());
    const size_t idx = min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1) + 0.5));
    return v[idx];
}

double mean(const vector<double>& v)
{
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (double x : v) s += x;
    return s / v.size();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cerr << "usage: " << argv[0] << " <instance.json> --hierarchyCache <path> "
             << "[--levels 2,4,6,8] [--pairs 200] [--seed 1] [--csv out.csv]\n";
        return 2;
    }
    string instance = argv[1], cache_path, csv_path;
    vector<int> levels{2, 4, 6, 8};
    int num_pairs = 200;
    unsigned seed = 1;
    size_t max_cells = 5000;
    double coarse_factor = 0.0;
    for (int i = 2; i + 1 < argc; i += 2)
    {
        const string flag = argv[i], val = argv[i + 1];
        if (flag == "--hierarchyCache") cache_path = val;
        else if (flag == "--pairs") num_pairs = stoi(val);
        else if (flag == "--seed") seed = static_cast<unsigned>(stoul(val));
        else if (flag == "--csv") csv_path = val;
        else if (flag == "--maxPathCells") max_cells = stoul(val);
        else if (flag == "--coarseHeuristicFactor") coarse_factor = stod(val);
        else if (flag == "--levels")
        {
            levels.clear();
            stringstream ss(val);
            string tok;
            while (getline(ss, tok, ',')) levels.push_back(stoi(tok));
        }
        else { cerr << "unknown flag " << flag << "\n"; return 2; }
    }

    SharedEnvironment env;
    populate_env_from_instance(instance, env);
    env.hierarchy_cache_path = cache_path;
    cout << "map " << env.map_name << " " << env.rows << "x" << env.cols
         << ", agents " << env.curr_states.size() << ", tasks " << env.task_pool.size() << "\n";

    const auto build_start = Clock::now();
    ReducedHierarchy& h = ReducedHierarchy::instance();
    h.ensure(&env);
    if (!h.ready()) { cerr << "hierarchy not ready\n"; return 1; }
    const int num_levels = h.hierarchy().num_levels();
    cout << "max path cells " << max_cells << ", coarse heuristic factor " << coarse_factor
         << (coarse_factor > 0.0 ? " (x 2^L)" : " (cheapest arc cost: exact)") << "\n";
    cout << "hierarchy ready in " << fixed << setprecision(1) << ms_since(build_start) / 1000.0
         << " s, " << num_levels << " levels; nodes per level:";
    for (int c : h.hierarchy_level_node_counts()) cout << " " << c;
    cout << "\n";

    // Pairs: a random agent start and a random task's first location, as a
    // real guide-path request would be. Unreachable pairs are redrawn.
    vector<int> starts, task_locs;
    for (const auto& s : env.curr_states) starts.push_back(s.location);
    for (const auto& kv : env.task_pool) task_locs.push_back(kv.second.locations[0]);
    mt19937 rng(seed);
    vector<int> dist(env.map.size(), -1), touched;
    vector<pair<int,int>> pairs;
    vector<int> true_dist;
    double bfs_ms_total = 0.0;
    while (static_cast<int>(pairs.size()) < num_pairs)
    {
        const int s = starts[rng() % starts.size()];
        const int g = task_locs[rng() % task_locs.size()];
        const auto t = Clock::now();
        const int d = fine_bfs_distance(env, s, g, dist, touched);
        bfs_ms_total += ms_since(t);
        if (d < 0) continue;
        pairs.push_back({s, g});
        true_dist.push_back(d);
    }
    cout << "pairs " << pairs.size() << ", shortest distance mean "
         << setprecision(0) << mean(vector<double>(true_dist.begin(), true_dist.end()))
         << ", max " << *max_element(true_dist.begin(), true_dist.end())
         << "; fine BFS mean " << setprecision(1) << bfs_ms_total / pairs.size() << " ms\n\n";

    // The planner's A* pieces: neighbour table, one search pool, and an
    // all-zero congestion map (plain shortest paths, Manhattan heuristic).
    DefaultPlanner::init_heuristics(&env);
    DefaultPlanner::MemoryPool pool(env.map.size());
    vector<DefaultPlanner::Int4> no_flow(env.map.size(), DefaultPlanner::Int4({0, 0, 0, 0}));
    DefaultPlanner::HeuristicTable no_table;

    // Reference: the planner's A* on the whole map, once per pair.
    vector<double> fine_ms(pairs.size()), fine_expanded(pairs.size()), fine_len(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i)
    {
        DefaultPlanner::Traj traj;
        int expanded = 0;
        const auto t = Clock::now();
        DefaultPlanner::astar(&env, no_flow, no_table, traj, pool, pairs[i].first, pairs[i].second,
                              &DefaultPlanner::global_neighbors, nullptr, nullptr, &expanded);
        fine_ms[i] = ms_since(t);
        fine_expanded[i] = expanded;
        fine_len[i] = static_cast<double>(traj.size()) - 1;
    }
    {
        vector<double> ratio;
        for (size_t i = 0; i < pairs.size(); ++i) ratio.push_back(fine_len[i] / max(1, true_dist[i]));
        cout << setprecision(2) << "full-map A* (no congestion): ms mean " << mean(fine_ms)
             << " p90 " << percentile(fine_ms, 0.9) << " max " << percentile(fine_ms, 1.0)
             << "; expanded mean " << setprecision(0) << mean(fine_expanded)
             << "; length / shortest mean " << setprecision(3) << mean(ratio) << "\n\n";
    }

    ofstream csv;
    if (!csv_path.empty())
    {
        csv.open(csv_path);
        csv << "level,pair,start,goal,true_dist,coarse_len,coarse_expanded,coarse_ms,lift_ms,"
               "lifted_cells,fail_level,fail_reason,endpoints_wrong,used_fallback,fallback_ok,"
               "fallback_ms,final_len,valid,"
               "c0_ok,c0_ms,c0_len,c0_expanded,c0_cells,c1_ok,c1_ms,c1_len,c1_expanded,c1_cells,"
               "fine_ms,fine_expanded,fine_len\n";
    }

    // One agent slot and one task slot, rewritten per pair.
    const int agent_id = 0, task_id = -1;
    if (env.curr_states.empty()) env.curr_states.resize(1);

    for (int level : levels)
    {
        if (level < 1 || level >= num_levels)
        {
            cout << "level " << level << ": not in hierarchy, skipped\n";
            continue;
        }
        const CoarsenedGraph* g = h.hierarchy().level(level);
        const double prep_s = h.prepare_coarse_search(level);
        int lift_ok = 0, fallback_ok = 0, failed = 0, invalid = 0, endpoints_wrong = 0;
        map<string, int> reasons;
        map<int, int> fail_levels;
        vector<double> ratio_lift, ratio_final, coarse_ms, lift_ms, fallback_ms, coarse_len;

        // corridor A*: fine cell -> level node, fine cells per node, marks
        const vector<int> cell_node = h.level_ancestors(level);
        vector<int> node_cells(h.level_node_id_count(level), 0);
        for (int n : cell_node) if (n >= 0) ++node_cells[n];
        vector<uint32_t> node_stamp(node_cells.size(), 0);
        uint32_t stamp = 0;
        const int margins[2] = {0, 1};
        int corr_ok[2] = {0, 0}, corr_invalid[2] = {0, 0};
        vector<double> corr_ratio[2], corr_ms[2], corr_expanded[2], corr_cells[2];
        // coarse A* vs the old Dijkstra
        int coarse_same_cost = 0, coarse_compared = 0;
        vector<double> coarse_cost_ratio;
        vector<double> dijkstra_ms, dijkstra_expanded, astar_expanded;

        for (size_t i = 0; i < pairs.size(); ++i)
        {
            const auto [s, goal] = pairs[i];
            env.curr_states[agent_id].location = s;
            env.task_pool[task_id] = Task(task_id, list<int>{goal}, 0);

            int expanded = 0;
            const auto t0 = Clock::now();
            const double unit = coarse_factor > 0.0 ? coarse_factor * (1 << level) : -1.0;
            vector<int> coarse = h.coarse_path(s, goal, level, &expanded, unit);
            const double c_ms = ms_since(t0);
            {
                int d_expanded = 0;
                const auto td = Clock::now();
                const vector<int> ref = reference_dijkstra(*g, h.cell_to_level_node(s, level),
                                                           h.cell_to_level_node(goal, level), &d_expanded);
                dijkstra_ms.push_back(ms_since(td));
                dijkstra_expanded.push_back(d_expanded);
                astar_expanded.push_back(expanded);
                if (!ref.empty() || !coarse.empty())
                {
                    ++coarse_compared;
                    const double ca = coarse_path_cost(*g, coarse), cd = coarse_path_cost(*g, ref);
                    if (ca >= 0.0 && cd >= 0.0 && abs(ca - cd) <= 1e-6 * max(1.0, cd)) ++coarse_same_cost;
                    if (ca >= 0.0 && cd > 0.0) coarse_cost_ratio.push_back(ca / cd);
                }
            }
            coarse_ms.push_back(c_ms);
            coarse_len.push_back(coarse.size());

            std::unordered_map<int, list<int>> out;
            vector<ReducedHierarchy::LiftOutcome> outcomes;
            const auto t1 = Clock::now();
            h.lift_coarse_paths_to_fine(&env, level, {agent_id}, {task_id}, {coarse}, out,
                                        nullptr, nullptr, nullptr, nullptr, &outcomes, max_cells);
            const double l_ms = ms_since(t1);
            const ReducedHierarchy::LiftOutcome o = outcomes.empty() ? ReducedHierarchy::LiftOutcome{} : outcomes[0];

            const auto it = out.find(agent_id);
            const bool have = it != out.end();
            const bool valid = have && path_is_valid(env, it->second, s, goal);
            const int final_len = have ? static_cast<int>(it->second.size()) - 1 : -1;
            const double td = max(1, true_dist[i]);

            if (have && !valid) ++invalid;
            if (o.endpoints_wrong) ++endpoints_wrong;
            if (!o.used_fallback && valid)
            {
                ++lift_ok;
                lift_ms.push_back(l_ms);
                ratio_lift.push_back(final_len / td);
            }
            else
            {
                if (o.used_fallback && o.fallback_ok && valid) ++fallback_ok;
                else ++failed;
                fallback_ms.push_back(o.fallback_ms);
                reasons[o.endpoints_wrong ? "endpoints_wrong" : reason_name(o.fail_reason)]++;
                fail_levels[o.fail_level]++;
            }
            if (valid) ratio_final.push_back(final_len / td);

            struct CorridorResult { bool ok = false; double ms = 0, len = -1, expanded = 0, cells = 0; };
            CorridorResult cr[2];
            for (int m = 0; m < 2 && !coarse.empty(); ++m)
            {
                const auto tc = Clock::now();
                const vector<int> nodes = h.corridor_nodes(level, coarse, margins[m]);
                ++stamp;
                for (int n : nodes) { node_stamp[n] = stamp; cr[m].cells += node_cells[n]; }
                DefaultPlanner::SearchCorridor corridor;
                corridor.cell_node = &cell_node;
                corridor.node_stamp = &node_stamp;
                corridor.stamp = stamp;
                DefaultPlanner::Traj traj;
                int expanded = 0;
                const DefaultPlanner::s_node found = DefaultPlanner::astar(
                    &env, no_flow, no_table, traj, pool, s, goal, &DefaultPlanner::global_neighbors,
                    nullptr, &corridor, &expanded);
                cr[m].ms = ms_since(tc);
                cr[m].expanded = expanded;
                const bool cvalid = found.id != -1 && path_is_valid(env, traj, s, goal);
                if (found.id != -1 && !cvalid) ++corr_invalid[m];
                if (cvalid)
                {
                    cr[m].ok = true;
                    cr[m].len = static_cast<double>(traj.size()) - 1;
                    ++corr_ok[m];
                    corr_ratio[m].push_back(cr[m].len / td);
                    corr_ms[m].push_back(cr[m].ms);
                    corr_expanded[m].push_back(expanded);
                    corr_cells[m].push_back(cr[m].cells);
                }
            }

            if (csv.is_open())
                csv << level << "," << i << "," << s << "," << goal << "," << true_dist[i] << ","
                    << coarse.size() << "," << expanded << "," << c_ms << "," << l_ms << ","
                    << o.lifted_cells << "," << o.fail_level << "," << reason_name(o.fail_reason) << ","
                    << o.endpoints_wrong << "," << o.used_fallback << "," << o.fallback_ok << ","
                    << o.fallback_ms << "," << final_len << "," << valid << ","
                    << cr[0].ok << "," << cr[0].ms << "," << cr[0].len << "," << cr[0].expanded << "," << cr[0].cells << ","
                    << cr[1].ok << "," << cr[1].ms << "," << cr[1].len << "," << cr[1].expanded << "," << cr[1].cells << ","
                    << fine_ms[i] << "," << fine_expanded[i] << "," << fine_len[i] << "\n";
        }

        const int n = static_cast<int>(pairs.size());
        vector<double> arc_costs;
        for (lemon::ListDigraph::ArcIt a(g->g); a != lemon::INVALID; ++a)
        {
            const int u = g->g.id(g->g.source(a)), v = g->g.id(g->g.target(a));
            if (u < 0 || v < 0 || u >= static_cast<int>(g->node_to_maploc.size()) ||
                v >= static_cast<int>(g->node_to_maploc.size()) ||
                g->node_to_maploc[u] < 0 || g->node_to_maploc[v] < 0)
                continue;
            arc_costs.push_back(g->cost[a]);
        }
        cout << setprecision(2) << "level " << level << " arc costs: min " << percentile(arc_costs, 0.0)
             << " p1 " << percentile(arc_costs, 0.01) << " p10 " << percentile(arc_costs, 0.1)
             << " median " << percentile(arc_costs, 0.5) << " mean " << mean(arc_costs)
             << " max " << percentile(arc_costs, 1.0) << " (2^L = " << (1 << level) << ")\n";
        cout << "level " << level << " (" << g->num_coarse_nodes << " nodes): "
             << "lift ok " << lift_ok << "/" << n
             << ", fallback ok " << fallback_ok << ", no path " << failed
             << ", invalid " << invalid << ", endpoints wrong " << endpoints_wrong << "\n";
        cout << setprecision(3)
             << "  length / shortest (lift ok): mean " << mean(ratio_lift)
             << ", p50 " << percentile(ratio_lift, 0.5) << ", p90 " << percentile(ratio_lift, 0.9)
             << ", max " << percentile(ratio_lift, 1.0) << "\n";
        cout << setprecision(2) << "  coarse A* (min arc cost " << h.min_arc_cost(level) << ", landmarks built in "
             << prep_s * 1000.0 << " ms): same cost as Dijkstra "
             << coarse_same_cost << "/" << coarse_compared << " (route cost / Dijkstra's: mean "
             << setprecision(4) << mean(coarse_cost_ratio) << " max " << percentile(coarse_cost_ratio, 1.0)
             << setprecision(2) << "); ms mean " << mean(coarse_ms)
             << " vs Dijkstra " << mean(dijkstra_ms) << "; expanded mean " << setprecision(0) << mean(astar_expanded)
             << " vs " << mean(dijkstra_expanded) << "\n";
        cout << setprecision(2)
             << "  coarse path nodes mean " << mean(coarse_len)
             << "; coarse search ms mean " << mean(coarse_ms) << " p90 " << percentile(coarse_ms, 0.9)
             << "; lift ms (ok) mean " << mean(lift_ms) << " p90 " << percentile(lift_ms, 0.9)
             << " max " << percentile(lift_ms, 1.0) << "\n";
        if (!fallback_ms.empty())
        {
            cout << "  fallback ms mean " << mean(fallback_ms) << " max " << percentile(fallback_ms, 1.0)
                 << "; reasons:";
            for (const auto& kv : reasons) cout << " " << kv.first << "=" << kv.second;
            cout << "; failed at level:";
            for (const auto& kv : fail_levels) cout << " " << kv.first << "=" << kv.second;
            cout << "\n";
        }
        for (int m = 0; m < 2; ++m)
        {
            cout << setprecision(3) << "  corridor margin " << margins[m] << ": ok " << corr_ok[m] << "/" << n
                 << ", invalid " << corr_invalid[m]
                 << "; length / shortest mean " << mean(corr_ratio[m]) << " p90 " << percentile(corr_ratio[m], 0.9)
                 << " max " << percentile(corr_ratio[m], 1.0) << "\n" << setprecision(2)
                 << "    ms mean " << mean(corr_ms[m]) << " p90 " << percentile(corr_ms[m], 0.9)
                 << " max " << percentile(corr_ms[m], 1.0) << setprecision(0)
                 << "; expanded mean " << mean(corr_expanded[m])
                 << "; corridor cells mean " << mean(corr_cells[m]) << " max " << percentile(corr_cells[m], 1.0) << "\n";
        }
    }
    return 0;
}
