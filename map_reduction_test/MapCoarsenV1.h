// MapCoarsenV1.h
// Declarations for the CoarsenedGraph type and helper functions.

#pragma once

#include "../inc/SharedEnv.h"

#include <lemon/list_graph.h>

#include <utility>
#include <vector>

namespace MapReductionTest {

/**
 * A single level of a (possibly multi-level) coarsened graph.
 *
 * This struct holds a LEMON directed graph representing either the fine map
 * (level 0) or a coarsened level (level > 0). The LEMON NodeMap/ArcMap
 * instances are tied to the owning `ListDigraph` and therefore must live in
 * the same struct instance as the graph.
 */
struct CoarsenedGraph
{
    // Dimensions / bookkeeping
    int level_idx = 0;      // 0 for finest, 1 for coarser, etc.
    int coarse_rows = 0;
    int coarse_cols = 0;
    int num_coarse_nodes = 0;

    // Underlying LEMON graph and node/arc maps
    lemon::ListDigraph g;
    lemon::ListDigraph::NodeMap<std::pair<int, int>> coarse_location; // node -> (r,c) in coarse space
    lemon::ListDigraph::NodeMap<std::pair<int, int>> fine_location;   // node -> (r,c) in fine space

    // Flow-network related maps (kept because scheduler code expects similar maps)
    lemon::ListDigraph::NodeMap<int> supply;
    lemon::ListDigraph::ArcMap<double> cost;
    lemon::ListDigraph::ArcMap<int> capacity;
    lemon::ListDigraph::ArcMap<int> flow;

    // Node lookup helpers
    // Index: fine-location index -> Value: List of LEMON nodes at this location
    std::vector<std::vector<lemon::ListDigraph::Node>> map_nodes;
    lemon::ListDigraph::Node source;
    lemon::ListDigraph::Node sink;

    // Efficient ID-based lookups (Replacing unordered_maps where possible)
    std::vector<int> node_to_maploc; // index: lemon node id -> value: fine-index

    // --- Multilevel Hierarchical Mappings ---
    // Upward mapping: This Node ID -> Coarser Node ID (in level + 1)
    std::vector<int> to_coarser_node_id;

    // Downward mapping: This Node ID -> Vector of Finer Node IDs (in level - 1)
    std::vector<std::vector<int>> to_finer_node_ids;

    /**
     * Construct an empty level; optionally pre-allocate storage for `fine_map_size`
     * fine-grid locations.
     */
    explicit CoarsenedGraph(int fine_map_size = 0);
};

/**
 * Recreate the per-level node storage. This clears the LEMON graph and
 * allocates `fine_map_size` nodes (one per fine-grid location). The source
 * and sink nodes are also created to allow the level to be used directly by
 * a flow solver if desired.
 */
void reserve_fine_map(CoarsenedGraph& graph, int fine_map_size);

/**
 * Attach coordinate metadata to a node in the level.
 */
void set_node_coordinates(CoarsenedGraph& graph,
                          int node_index,
                          const std::pair<int, int>& coarse_xy,
                          const std::pair<int, int>& fine_xy);

/**
 * Build the fine (uncoarsened) map graph from `env->map` and associated
 * environment fields. The resulting graph has one node per walkable fine
 * cell and arcs between four-neighbor walkable cells.
 */
void build_from_environment(CoarsenedGraph& graph, const SharedEnvironment* env);

/**
 * Produce the next coarser level from `graph` and return it. The function is
 * intentionally declared here and left for you to implement (it should not be
 * defined in the header).
 */
CoarsenedGraph Coarsen(const CoarsenedGraph& graph);

} // namespace MapReductionTest
