// validate_edge_augmented_graph.cpp
//
// Standalone structural rigor check for solver 7's edge-node-augmented
// backbone (EdgeAugmentedCoarsen.{h,cpp}, see ai/edge_node_representation.md).
// Builds the hierarchy exactly as ReducedHierarchy::instance().ensure() does,
// then calls the same build_edge_augmented_top_graph() the runtime path
// uses, and checks structural invariants that must hold by construction
// regardless of map size/shape:
//
//   1. Every backbone arc corresponds 1:1 to a top-level coarse arc, in the
//      same direction, with half its cost -- checked by degree-matching
//      every region node's in/out-degree against top.g's, and by summing
//      forward+backward cost through a sample of edge-nodes against the
//      original top.g arc cost.
//   2. Every edge-node has in-degree exactly 2 and out-degree exactly 2 (one
//      region on each side, one arc each direction).
//   3. Total backbone arc count == 2 * (top.g arc count).
//   4. Each region's fine-cell bounding box (computed by the fast bottom-up
//      sweep in compute_region_bboxes, not directly exposed -- exercised
//      indirectly via build_edge_augmented_top_graph) is cross-checked
//      against an independent, deliberately naive brute-force recursive
//      descent through to_finer_node_ids down to level 0, for a sample of
//      regions (or all of them on a small map).
//
// Usage: ./edge_augmented_validator <instance.json> [level] [bbox_sample]
//   level: hierarchy level to build the backbone at (default: 2, same as
//          MapCoarsenV1.cpp's kDefaultFlowSolveLevel).
//   bbox_sample: max number of regions to run the naive bbox cross-check on
//          (default: 500 -- the cheap structural/degree checks above always
//          run over every region/edge-node regardless).

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <lemon/list_graph.h>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "EdgeAugmentedCoarsen.h"
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

// Deliberately naive reference implementation: recursively expand a node at
// `level` all the way down to level-0 fine cells, independent of (and much
// slower than) compute_region_bboxes' bottom-up sweep in
// EdgeAugmentedCoarsen.cpp -- this is the ground truth the optimized version
// is checked against, not a second copy of the same algorithm.
void collect_fine_descendants(const MultiLevelCoarsenedGraph& hierarchy, int level, int node_id, std::vector<int>& out)
{
    if (level <= 0)
    {
        out.push_back(node_id);
        return;
    }
    const CoarsenedGraph* g = hierarchy.level(level);
    if (!g || node_id < 0 || node_id >= static_cast<int>(g->to_finer_node_ids.size()))
        return;
    for (int child : g->to_finer_node_ids[node_id])
        collect_fine_descendants(hierarchy, level - 1, child, out);
}

RegionBBox naive_region_bbox(const MultiLevelCoarsenedGraph& hierarchy, const CoarsenedGraph& fine, int level, int region_id)
{
    std::vector<int> fine_cells;
    collect_fine_descendants(hierarchy, level, region_id, fine_cells);

    RegionBBox box{};
    bool first = true;
    for (int loc : fine_cells)
    {
        if (loc < 0 || loc >= static_cast<int>(fine.map_nodes.size()) || fine.map_nodes[loc] == lemon::INVALID)
            continue;
        const auto rc = fine.fine_location[fine.map_nodes[loc]];
        if (first)
        {
            box = RegionBBox{rc.first, rc.first, rc.second, rc.second};
            first = false;
        }
        else
        {
            box.min_row = std::min(box.min_row, rc.first);
            box.max_row = std::max(box.max_row, rc.first);
            box.min_col = std::min(box.min_col, rc.second);
            box.max_col = std::max(box.max_col, rc.second);
        }
    }
    return box;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <instance.json> [level] [bbox_sample]\n";
        return 2;
    }

    const int requested_level = argc >= 3 ? std::stoi(argv[2]) : 2;
    const int bbox_sample = argc >= 4 ? std::stoi(argv[3]) : 500;

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

    auto t0 = std::chrono::high_resolution_clock::now();
    ReducedHierarchy::instance().ensure(&env);
    auto t1 = std::chrono::high_resolution_clock::now();
    if (!ReducedHierarchy::instance().ready())
    {
        std::cerr << "hierarchy failed to build\n";
        return 2;
    }
    std::cout << "Hierarchy built in " << std::chrono::duration<double>(t1 - t0).count() << "s, "
              << ReducedHierarchy::instance().hierarchy().num_levels() << " levels.\n";

    const auto& hierarchy = ReducedHierarchy::instance().hierarchy();
    int level = requested_level;
    if (level < 0 || level >= hierarchy.num_levels())
        level = hierarchy.num_levels() - 1;
    const CoarsenedGraph* top = hierarchy.level(level);
    const CoarsenedGraph* fine = hierarchy.fine_graph();
    check(top != nullptr, "requested level exists in the hierarchy");
    check(fine != nullptr, "fine (level 0) graph exists");
    if (!top || !fine)
        return 2;
    std::cout << "Building edge-augmented backbone at level " << level
              << " (top->num_coarse_nodes=" << top->num_coarse_nodes << ")\n";

    auto t2 = std::chrono::high_resolution_clock::now();
    auto backbone = build_edge_augmented_top_graph(hierarchy, *top);
    auto t3 = std::chrono::high_resolution_clock::now();
    check(backbone != nullptr, "build_edge_augmented_top_graph returned non-null");
    if (!backbone)
        return 2;
    const double backbone_build_time = std::chrono::duration<double>(t3 - t2).count();
    std::cout << "Backbone built in " << backbone_build_time << "s.\n";

    // --- Reference counts from top.g itself ---
    int top_arc_count = 0;
    std::unordered_map<int,int> top_out_degree; // region id -> out-degree in top.g
    std::unordered_map<int,int> top_in_degree;  // region id -> in-degree in top.g
    std::unordered_map<std::pair<int,int>, double, CoarsenedGraph::PairHash> top_arc_cost; // (src,dst) -> cost
    for (lemon::ListDigraph::ArcIt a(top->g); a != lemon::INVALID; ++a)
    {
        ++top_arc_count;
        const int src_lid = top->g.id(top->g.source(a));
        const int dst_lid = top->g.id(top->g.target(a));
        if (src_lid < 0 || dst_lid < 0 ||
            src_lid >= static_cast<int>(top->node_to_maploc.size()) ||
            dst_lid >= static_cast<int>(top->node_to_maploc.size()))
            continue;
        const int src_id = top->node_to_maploc[src_lid];
        const int dst_id = top->node_to_maploc[dst_lid];
        top_out_degree[src_id]++;
        top_in_degree[dst_id]++;
        top_arc_cost[{src_id, dst_id}] = top->cost[a];
    }

    // --- Backbone node/arc census ---
    std::unordered_map<int,int> backbone_region_lemon_id; // lemon id in backbone->g -> region id
    for (int region_id = 0; region_id < static_cast<int>(backbone->region_node.size()); ++region_id)
        if (backbone->region_node[region_id] != lemon::INVALID)
            backbone_region_lemon_id[backbone->g.id(backbone->region_node[region_id])] = region_id;

    int backbone_node_count = 0, backbone_arc_count = 0;
    for (lemon::ListDigraph::NodeIt n(backbone->g); n != lemon::INVALID; ++n) ++backbone_node_count;
    for (lemon::ListDigraph::ArcIt a(backbone->g); a != lemon::INVALID; ++a) ++backbone_arc_count;

    const int region_count = static_cast<int>(backbone_region_lemon_id.size());
    const int edge_node_count = backbone_node_count - region_count;
    std::cout << "Backbone: " << backbone_node_count << " nodes (" << region_count << " region + "
              << edge_node_count << " edge), " << backbone_arc_count << " arcs.\n";
    std::cout << "top.g: " << top_arc_count << " arcs.\n";

    check(backbone_arc_count == 2 * top_arc_count,
          "backbone arc count == 2 * top.g arc count (" + std::to_string(backbone_arc_count) +
          " vs 2*" + std::to_string(top_arc_count) + "=" + std::to_string(2 * top_arc_count) + ")");

    // Sum of every region's adjacency-list length should be exactly 2 * edge_node_count
    // (each edge-node is registered in exactly two regions' lists).
    long long total_adjacency_entries = 0;
    for (const auto& v : backbone->region_adjacent_edges)
        total_adjacency_entries += static_cast<long long>(v.size());
    check(total_adjacency_entries == 2LL * edge_node_count,
          "sum(region_adjacent_edges sizes) == 2 * edge_node_count (" +
          std::to_string(total_adjacency_entries) + " vs " + std::to_string(2LL * edge_node_count) + ")");

    // --- Per-node degree checks (every node in the backbone) ---
    int region_degree_mismatches = 0;
    int edge_node_degree_mismatches = 0;
    for (lemon::ListDigraph::NodeIt n(backbone->g); n != lemon::INVALID; ++n)
    {
        int out_deg = 0, in_deg = 0;
        for (lemon::ListDigraph::OutArcIt a(backbone->g, n); a != lemon::INVALID; ++a) ++out_deg;
        for (lemon::ListDigraph::InArcIt a(backbone->g, n); a != lemon::INVALID; ++a) ++in_deg;

        const int lid = backbone->g.id(n);
        const auto region_it = backbone_region_lemon_id.find(lid);
        if (region_it != backbone_region_lemon_id.end())
        {
            const int region_id = region_it->second;
            const int expected_out = top_out_degree.count(region_id) ? top_out_degree[region_id] : 0;
            const int expected_in = top_in_degree.count(region_id) ? top_in_degree[region_id] : 0;
            if (out_deg != expected_out || in_deg != expected_in)
                ++region_degree_mismatches;
        }
        else
        {
            // Edge-node: exactly one arc in from each side, one arc out to each side.
            if (in_deg != 2 || out_deg != 2)
                ++edge_node_degree_mismatches;
        }
    }
    check(region_degree_mismatches == 0,
          "every region node's backbone in/out-degree matches its top.g in/out-degree (" +
          std::to_string(region_degree_mismatches) + " mismatches)");
    check(edge_node_degree_mismatches == 0,
          "every edge-node has in-degree 2 and out-degree 2 (" +
          std::to_string(edge_node_degree_mismatches) + " mismatches)");

    // --- Cost check: forward+backward cost through each edge-node's two
    // directions should reconstruct top.g's original arc costs exactly. ---
    int cost_mismatches = 0;
    int cost_checks = 0;
    for (lemon::ListDigraph::NodeIt n(backbone->g); n != lemon::INVALID; ++n)
    {
        if (backbone_region_lemon_id.count(backbone->g.id(n)))
            continue; // region node, not an edge-node

        // Collect this edge-node's incident region ids via its arcs.
        for (lemon::ListDigraph::OutArcIt out_a(backbone->g, n); out_a != lemon::INVALID; ++out_a)
        {
            const auto dst_lid = backbone->g.id(backbone->g.target(out_a));
            const auto dst_region_it = backbone_region_lemon_id.find(dst_lid);
            if (dst_region_it == backbone_region_lemon_id.end())
                continue;
            const int dst_region = dst_region_it->second;

            for (lemon::ListDigraph::InArcIt in_a(backbone->g, n); in_a != lemon::INVALID; ++in_a)
            {
                const auto src_lid = backbone->g.id(backbone->g.source(in_a));
                const auto src_region_it = backbone_region_lemon_id.find(src_lid);
                if (src_region_it == backbone_region_lemon_id.end())
                    continue;
                const int src_region = src_region_it->second;

                const auto orig_it = top_arc_cost.find({src_region, dst_region});
                if (orig_it == top_arc_cost.end())
                    continue; // this in/out pair doesn't correspond to a real top.g arc (the other direction)

                ++cost_checks;
                const double reconstructed = backbone->cost[in_a] + backbone->cost[out_a];
                if (std::abs(reconstructed - orig_it->second) > 1e-9)
                    ++cost_mismatches;
            }
        }
    }
    // A map coarsened all the way down to a single region (no edges left to
    // check) is a legitimate degenerate case, not a bug -- only require a
    // nonzero check count when the backbone actually has edge-nodes.
    if (edge_node_count > 0)
        check(cost_checks > 0, "at least one edge-node cost reconstruction was checked");
    check(cost_mismatches == 0,
          "forward+backward cost through every checked edge-node reconstructs top.g's original arc cost (" +
          std::to_string(cost_mismatches) + "/" + std::to_string(cost_checks) + " mismatches)");

    // --- Bounding box cross-check against the naive reference implementation ---
    std::vector<int> region_ids;
    for (const auto& kv : backbone_region_lemon_id)
        region_ids.push_back(kv.second);
    std::sort(region_ids.begin(), region_ids.end());
    if (static_cast<int>(region_ids.size()) > bbox_sample)
    {
        // Evenly spaced sample rather than the first N, so a large map's
        // sample isn't biased toward one corner of the coarse grid.
        std::vector<int> sampled;
        const double stride = static_cast<double>(region_ids.size()) / bbox_sample;
        for (int i = 0; i < bbox_sample; ++i)
            sampled.push_back(region_ids[static_cast<std::size_t>(i * stride)]);
        region_ids = std::move(sampled);
    }

    int bbox_mismatches = 0;
    for (int region_id : region_ids)
    {
        if (region_id < 0 || region_id >= static_cast<int>(backbone->region_bbox.size()))
        {
            ++bbox_mismatches;
            continue;
        }
        const RegionBBox fast = backbone->region_bbox[region_id];
        const RegionBBox naive = naive_region_bbox(hierarchy, *fine, level, region_id);
        if (fast.min_row != naive.min_row || fast.max_row != naive.max_row ||
            fast.min_col != naive.min_col || fast.max_col != naive.max_col)
        {
            ++bbox_mismatches;
        }
    }
    check(bbox_mismatches == 0,
          "compute_region_bboxes' fast bottom-up result matches the naive recursive reference on " +
          std::to_string(region_ids.size()) + " sampled regions (" +
          std::to_string(bbox_mismatches) + " mismatches)");

    std::cout << "\n" << g_checks_run << " checks run, " << g_checks_failed << " failed.\n";
    return g_checks_failed == 0 ? 0 : 1;
}
