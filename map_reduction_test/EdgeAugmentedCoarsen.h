// EdgeAugmentedCoarsen.h
//
// Solver 7 (edge-node augmented coarse graph): see
// ai/edge_node_representation.md for the full design writeup. Short version:
// solver 6's per-timestep coarse flow (ReducedHierarchy::compute_reduced_
// assignment, MapCoarsenV1.h/.cpp) gives every surplus agent/task a flat
// cost-0 arc to its own top-level region node regardless of where in that
// (possibly large) region it actually sits. This file builds a second,
// ephemeral graph on top of the *same* hierarchy solver 6 already builds:
// every coarse-to-coarse arc at the chosen flow-solve level is subdivided by
// an explicit "edge-node" (region <-> edge <-> region, cost halved each
// way), and agent/task proxy nodes connect directly to the edge-nodes
// adjacent to their own region -- never the region node itself -- with arc
// cost approximated as the Manhattan distance from the agent/task's real
// fine-grid location to the *neighboring* region's fine-cell bounding box.
//
// Deliberately additive: this file does not modify how the persistent
// hierarchy (MultiLevelCoarsenedGraph, Coarsen(), bridge caches,
// MapCoarsenSerialize) is built -- it only builds a derived structure on top
// of one already-built level, via ReducedHierarchy::instance().

#pragma once

#include "MapCoarsenV1.h"

#include <lemon/list_graph.h>

#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace MapReductionTest {

// Axis-aligned bounding box, in fine-grid (row, col) coordinates, over every
// fine cell that ultimately belongs to one top-level region.
struct RegionBBox
{
    int min_row = 0;
    int max_row = 0;
    int min_col = 0;
    int max_col = 0;
};

// One persistent, solve-time-only augmentation of a single hierarchy level
// ("top_level_idx"): every existing coarse-to-coarse arc in that level's
// graph is subdivided by an edge-node sitting between the two region nodes
// it used to connect directly. Built once per (map signature, level) by
// EdgeAugmentedHierarchy and reused every timestep via lemon::digraphCopy --
// see EdgeAugmentedHierarchy::ensure below.
struct EdgeAugmentedTopGraph
{
    int top_level_idx = -1;

    // Backbone only: region-nodes + edge-nodes and the (halved-cost) arcs
    // between them. No source/sink/proxy nodes -- those are added fresh
    // into a *copy* of this graph every timestep (see
    // EdgeAugmentedHierarchy::compute_reduced_assignment_edge_augmented).
    lemon::ListDigraph g;
    lemon::ListDigraph::ArcMap<double> cost;
    lemon::ListDigraph::ArcMap<int> capacity;

    // index: top-level region id (same indexing as the source CoarsenedGraph's
    // map_nodes) -> Node in g. lemon::INVALID for ids with no node.
    std::vector<lemon::ListDigraph::Node> region_node;

    // index: top-level region id -> its fine-cell bounding box.
    std::vector<RegionBBox> region_bbox;

    struct AdjacentEdge
    {
        lemon::ListDigraph::Node edge_node;
        int neighbor_region_id = -1;
    };
    // index: top-level region id -> every edge-node touching it (<= 4, one
    // per 4-adjacent neighbor with a coarse arc to/from this region).
    std::vector<std::vector<AdjacentEdge>> region_adjacent_edges;

    EdgeAugmentedTopGraph() : cost(g), capacity(g) {}
};

// Build the backbone described above from an already-built hierarchy level
// `top` (its region-nodes + coarse arcs). `hierarchy` is the full multi-level
// structure the region bounding boxes are folded up from (levels 0 through
// top.level_idx).
std::unique_ptr<EdgeAugmentedTopGraph> build_edge_augmented_top_graph(
    const MultiLevelCoarsenedGraph& hierarchy, const CoarsenedGraph& top);

// Process-lifetime singleton, sibling to (and composing) ReducedHierarchy --
// never builds its own hierarchy, only a derived structure on top of
// ReducedHierarchy::instance()'s. Owns one cached EdgeAugmentedTopGraph at a
// time (single slot, not one per level: --flowSolveLevel is fixed for the
// life of a process in practice, so there is no real scenario needing more
// than one cached level at once).
class EdgeAugmentedHierarchy
{
public:
    static EdgeAugmentedHierarchy& instance();

    // Ensures ReducedHierarchy::instance().ensure(env) has run (same
    // underlying hierarchy, no separate build), then ensures the cached
    // backbone matches `top_level_idx` (after the same out-of-range
    // clamping ReducedHierarchy::compute_reduced_assignment applies) and the
    // current map signature -- rebuilds, dropping any previous backbone,
    // only if either changed. Cheap no-op otherwise.
    void ensure(const SharedEnvironment* env, int top_level_idx);
    bool ready() const;

    // Same shape/semantics as ReducedHierarchy::compute_reduced_assignment
    // (see MapCoarsenV1.h for the shared out-param semantics), plus one
    // solver-7-only addition: `backbone_build_time_out` receives the wall-
    // clock time (seconds) this call's internal ensure() spent rebuilding
    // the backbone -- 0 on every call that reused an already-valid cached
    // backbone (the common case; a rebuild only happens on the first call,
    // or after the map/level changes).
    std::unordered_map<int,int> compute_reduced_assignment_edge_augmented(
        SharedEnvironment* env,
        const std::vector<int>& flexible_agent_ids,
        const std::vector<int>& flexible_task_ids,
        std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
        bool need_guide_paths = true,
        double* solve_time_out = nullptr,
        double* guide_time_out = nullptr,
        double* guide_path_length_sum_out = nullptr,
        double* guide_path_cost_sum_out = nullptr,
        int* local_match_count_out = nullptr,
        int* flow_match_count_out = nullptr,
        double* local_match_time_out = nullptr,
        double* backbone_build_time_out = nullptr);

private:
    EdgeAugmentedHierarchy() = default;
    ~EdgeAugmentedHierarchy() = default;
    EdgeAugmentedHierarchy(const EdgeAugmentedHierarchy&) = delete;
    EdgeAugmentedHierarchy& operator=(const EdgeAugmentedHierarchy&) = delete;

    std::size_t env_signature_ = 0;
    std::unique_ptr<EdgeAugmentedTopGraph> backbone_;
    double last_backbone_build_time_ = 0.0;
};

} // namespace MapReductionTest
