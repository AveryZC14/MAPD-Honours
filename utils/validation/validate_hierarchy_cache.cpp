// validate_hierarchy_cache.cpp
//
// Standalone rigor check for the hierarchy disk-cache feature
// (MapCoarsenSerialize.{h,cpp} + ReducedHierarchy's load-else-build-then-save
// logic in MapCoarsenV1.cpp). Builds a hierarchy directly from an instance,
// saves it, reloads it into a second in-memory hierarchy, and asserts the
// two are *structurally identical* level-by-level: every node's coordinates,
// hierarchy-mapping fields, and internal-directional-arc metrics; every arc's
// endpoints/cost/capacity (order-independent); and both bridge caches'
// full key/value contents. This is a stronger check than "the simulation
// still runs" (run-to-run simulation output is only weakly deterministic --
// see ai/hierarchy_cache.md -- so behavioral diffs alone can't distinguish a
// serialization bug from ordinary wall-clock-timing jitter).
//
// Usage: ./hierarchy_cache_validator <instance.json>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <lemon/list_graph.h>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "MapCoarsenSerialize.h"
#include "instance_loader.h"

using namespace MapReductionTest;

namespace {

int g_checks_run = 0;
int g_checks_failed = 0;

void check(bool cond, const std::string& msg)
{
    ++g_checks_run;
    if (!cond)
    {
        ++g_checks_failed;
        std::cout << "  [FAIL] " << msg << "\n";
    }
}

struct ArcRecord
{
    int u, v, capacity;
    double cost;
    bool operator<(const ArcRecord& o) const
    {
        if (u != o.u) return u < o.u;
        if (v != o.v) return v < o.v;
        if (capacity != o.capacity) return capacity < o.capacity;
        return cost < o.cost;
    }
};

std::vector<ArcRecord> collect_arcs(const CoarsenedGraph& g)
{
    std::vector<ArcRecord> arcs;
    for (lemon::ListDigraph::ArcIt a(g.g); a != lemon::INVALID; ++a)
    {
        const int u_lid = g.g.id(g.g.source(a));
        const int v_lid = g.g.id(g.g.target(a));
        const int u_id = (u_lid >= 0 && u_lid < static_cast<int>(g.node_to_maploc.size()))
                             ? g.node_to_maploc[u_lid] : -1;
        const int v_id = (v_lid >= 0 && v_lid < static_cast<int>(g.node_to_maploc.size()))
                             ? g.node_to_maploc[v_lid] : -1;
        arcs.push_back({u_id, v_id, g.capacity[a], g.cost[a]});
    }
    std::sort(arcs.begin(), arcs.end());
    return arcs;
}

void compare_levels(int level_idx, const CoarsenedGraph& a, const CoarsenedGraph& b)
{
    const std::string L = "level " + std::to_string(level_idx) + ": ";

    check(a.level_idx == b.level_idx, L + "level_idx mismatch");
    check(a.coarse_rows == b.coarse_rows, L + "coarse_rows mismatch");
    check(a.coarse_cols == b.coarse_cols, L + "coarse_cols mismatch");
    check(a.num_coarse_nodes == b.num_coarse_nodes, L + "num_coarse_nodes mismatch");
    check(a.inter_component_arc_aggregation_policy == b.inter_component_arc_aggregation_policy,
          L + "inter_component_arc_aggregation_policy mismatch");
    check(a.map_nodes.size() == b.map_nodes.size(), L + "map_nodes.size() mismatch");
    check(a.to_coarser_node_id == b.to_coarser_node_id, L + "to_coarser_node_id mismatch");
    check(a.chosen_finer_node_id == b.chosen_finer_node_id, L + "chosen_finer_node_id mismatch");
    check(a.to_finer_node_ids == b.to_finer_node_ids, L + "to_finer_node_ids mismatch");

    const std::size_t n = std::min(a.map_nodes.size(), b.map_nodes.size());
    bool coords_ok = true;
    bool metrics_ok = true;
    for (std::size_t i = 0; i < n; ++i)
    {
        const auto an = a.map_nodes[i];
        const auto bn = b.map_nodes[i];
        if (an == lemon::INVALID || bn == lemon::INVALID) { coords_ok = false; continue; }
        if (a.coarse_location[an] != b.coarse_location[bn]) coords_ok = false;
        if (a.fine_location[an] != b.fine_location[bn]) coords_ok = false;

        const auto& am = a.internal_directional_arc_metrics[i];
        const auto& bm = b.internal_directional_arc_metrics[i];
        for (int d = 0; d < 4; ++d)
        {
            if (am.weights[d].has_value() != bm.weights[d].has_value()) { metrics_ok = false; continue; }
            if (am.weights[d].has_value() && *am.weights[d] != *bm.weights[d]) metrics_ok = false;
        }
    }
    check(coords_ok, L + "coarse_location/fine_location mismatch on at least one node");
    check(metrics_ok, L + "internal_directional_arc_metrics mismatch on at least one node");

    const auto arcs_a = collect_arcs(a);
    const auto arcs_b = collect_arcs(b);
    check(arcs_a.size() == arcs_b.size(), L + "arc count mismatch (" +
          std::to_string(arcs_a.size()) + " vs " + std::to_string(arcs_b.size()) + ")");
    bool arcs_equal = (arcs_a.size() == arcs_b.size());
    if (arcs_equal)
    {
        for (std::size_t i = 0; i < arcs_a.size(); ++i)
        {
            if (!(arcs_a[i].u == arcs_b[i].u && arcs_a[i].v == arcs_b[i].v &&
                  arcs_a[i].capacity == arcs_b[i].capacity && arcs_a[i].cost == arcs_b[i].cost))
            {
                arcs_equal = false;
                break;
            }
        }
    }
    check(arcs_equal, L + "arc set mismatch");

    check(a.bridge_cache.size() == b.bridge_cache.size(), L + "bridge_cache size mismatch");
    bool bridge_cache_ok = true;
    for (const auto& kv : a.bridge_cache)
    {
        auto it = b.bridge_cache.find(kv.first);
        if (it == b.bridge_cache.end() || it->second != kv.second) { bridge_cache_ok = false; break; }
    }
    check(bridge_cache_ok, L + "bridge_cache contents mismatch");

    check(a.bridge_path_cache.size() == b.bridge_path_cache.size(), L + "bridge_path_cache size mismatch");
    bool bridge_path_cache_ok = true;
    for (const auto& kv : a.bridge_path_cache)
    {
        auto it = b.bridge_path_cache.find(kv.first);
        if (it == b.bridge_path_cache.end() ||
            it->second.bridge != kv.second.bridge ||
            it->second.path != kv.second.path)
        {
            bridge_path_cache_ok = false;
            break;
        }
    }
    check(bridge_path_cache_ok, L + "bridge_path_cache contents mismatch");
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <instance.json>\n";
        return 2;
    }

    SharedEnvironment env;
    try
    {
        populate_env_from_instance(argv[1], env);
    }
    catch (const std::exception& e)
    {
        std::cerr << "failed to load instance: " << e.what() << "\n";
        return 2;
    }

    constexpr int kLevelsToAdd = 2; // mirrors kDefaultCoarsenLevels in MapCoarsenV1.cpp
    MultiLevelCoarsenedGraph built;
    build_multilevel_from_environment(built, &env, kLevelsToAdd);
    check(!built.empty(), "fresh build produced a non-empty hierarchy");
    std::cout << "Built hierarchy with " << built.num_levels() << " levels.\n";

    const std::string cache_path = std::string(argv[1]) + ".hierarchy_cache_test.bin";
    check(save_hierarchy_to_file(built, &env, cache_path), "save_hierarchy_to_file succeeded");

    MultiLevelCoarsenedGraph loaded;
    check(load_hierarchy_from_file(cache_path, &env, built.num_levels(), loaded),
          "load_hierarchy_from_file succeeded");
    check(loaded.num_levels() == built.num_levels(), "loaded level count matches built level count");

    const int n = std::min(built.num_levels(), loaded.num_levels());
    for (int i = 0; i < n; ++i)
    {
        const CoarsenedGraph* a = built.level(i);
        const CoarsenedGraph* b = loaded.level(i);
        if (a == nullptr || b == nullptr) { check(false, "level " + std::to_string(i) + " missing"); continue; }
        compare_levels(i, *a, *b);
    }

    // Also verify a mismatched signature/level-count is correctly rejected
    // rather than silently accepted -- the failure-closed behavior every
    // caller (ReducedHierarchy::ensure) depends on.
    MultiLevelCoarsenedGraph rejected;
    check(!load_hierarchy_from_file(cache_path, &env, built.num_levels() + 1, rejected),
          "load rejects a wrong expected_num_levels");

    SharedEnvironment different_env = env;
    if (!different_env.map.empty())
        different_env.map[0] = (different_env.map[0] == 0) ? 1 : 0;
    MultiLevelCoarsenedGraph rejected2;
    check(!load_hierarchy_from_file(cache_path, &different_env, built.num_levels(), rejected2),
          "load rejects a mismatched map signature");

    std::remove(cache_path.c_str());

    std::cout << "\n" << g_checks_run << " checks run, " << g_checks_failed << " failed.\n";
    return g_checks_failed == 0 ? 0 : 1;
}
