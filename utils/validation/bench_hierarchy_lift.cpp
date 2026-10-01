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
//            [--maxPathCells 5000]
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

int to_level(const ReducedHierarchy& h, int fine_loc, int level)
{
    int id = fine_loc;
    for (int l = 0; l < level; ++l)
    {
        const CoarsenedGraph* g = h.hierarchy().level(l);
        if (!g || id < 0 || id >= static_cast<int>(g->to_coarser_node_id.size())) return -1;
        id = g->to_coarser_node_id[id];
    }
    return id;
}

// Cheapest path between two graph ids on one coarse level, by arc cost.
vector<int> coarse_dijkstra(const CoarsenedGraph& g, int from, int to, int* expanded_out)
{
    *expanded_out = 0;
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
        const auto u_node = g.map_nodes[u];
        for (lemon::ListDigraph::OutArcIt a(g.g, u_node); a != lemon::INVALID; ++a)
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

bool path_is_valid(const SharedEnvironment& env, const list<int>& path, int start, int goal)
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
    for (int i = 2; i + 1 < argc; i += 2)
    {
        const string flag = argv[i], val = argv[i + 1];
        if (flag == "--hierarchyCache") cache_path = val;
        else if (flag == "--pairs") num_pairs = stoi(val);
        else if (flag == "--seed") seed = static_cast<unsigned>(stoul(val));
        else if (flag == "--csv") csv_path = val;
        else if (flag == "--maxPathCells") max_cells = stoul(val);
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
    cout << "max path cells " << max_cells << "\n";
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

    ofstream csv;
    if (!csv_path.empty())
    {
        csv.open(csv_path);
        csv << "level,pair,start,goal,true_dist,coarse_len,coarse_expanded,coarse_ms,lift_ms,"
               "lifted_cells,fail_level,fail_reason,endpoints_wrong,used_fallback,fallback_ok,"
               "fallback_ms,final_len,valid\n";
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
        int lift_ok = 0, fallback_ok = 0, failed = 0, invalid = 0, endpoints_wrong = 0;
        map<string, int> reasons;
        map<int, int> fail_levels;
        vector<double> ratio_lift, ratio_final, coarse_ms, lift_ms, fallback_ms, coarse_len;

        for (size_t i = 0; i < pairs.size(); ++i)
        {
            const auto [s, goal] = pairs[i];
            env.curr_states[agent_id].location = s;
            env.task_pool[task_id] = Task(task_id, list<int>{goal}, 0);

            const int cs = to_level(h, s, level), cg = to_level(h, goal, level);
            int expanded = 0;
            const auto t0 = Clock::now();
            vector<int> coarse = (cs >= 0 && cg >= 0) ? coarse_dijkstra(*g, cs, cg, &expanded) : vector<int>{};
            const double c_ms = ms_since(t0);
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

            if (csv.is_open())
                csv << level << "," << i << "," << s << "," << goal << "," << true_dist[i] << ","
                    << coarse.size() << "," << expanded << "," << c_ms << "," << l_ms << ","
                    << o.lifted_cells << "," << o.fail_level << "," << reason_name(o.fail_reason) << ","
                    << o.endpoints_wrong << "," << o.used_fallback << "," << o.fallback_ok << ","
                    << o.fallback_ms << "," << final_len << "," << valid << "\n";
        }

        const int n = static_cast<int>(pairs.size());
        cout << "level " << level << " (" << g->num_coarse_nodes << " nodes): "
             << "lift ok " << lift_ok << "/" << n
             << ", fallback ok " << fallback_ok << ", no path " << failed
             << ", invalid " << invalid << ", endpoints wrong " << endpoints_wrong << "\n";
        cout << setprecision(3)
             << "  length / shortest (lift ok): mean " << mean(ratio_lift)
             << ", p50 " << percentile(ratio_lift, 0.5) << ", p90 " << percentile(ratio_lift, 0.9)
             << ", max " << percentile(ratio_lift, 1.0) << "\n";
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
    }
    return 0;
}
