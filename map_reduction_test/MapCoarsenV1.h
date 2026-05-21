// MapCoarsenV1.h
// Declarations for the CoarsenedGraph type and helper functions.

#pragma once

#include "../inc/SharedEnv.h"

#include <lemon/list_graph.h>

#include <string>
#include <utility>
#include <vector>
#include <array>
#include <optional>

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
    // Reduction policy used when aggregating multiple finer arcs into one
    // coarse inter-component arc.
    enum class ArcAggregationPolicy
    {
        Average,
        Minimum
    };

    // Dimensions / bookkeeping
    int level_idx = 0;      // 0 for finest, 1 for coarser, etc.
    int coarse_rows = 0;
    int coarse_cols = 0;
    int num_coarse_nodes = 0;

    // Policy used when creating arcs between coarse connected components.
    ArcAggregationPolicy inter_component_arc_aggregation_policy = ArcAggregationPolicy::Average;

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
    std::vector<lemon::ListDigraph::Node> map_nodes; //graph ID for each node
    lemon::ListDigraph::Node source;
    lemon::ListDigraph::Node sink;

    // Per-node internal directional arc statistics (kept empty for levels
    // that haven't been populated).
    //
    // These containers collect information about INTERNAL edges wholly
    // contained inside a connected component discovered during coarsening.
    // - `InternalDirectionalArcSamples` stores the raw arc weights bucketed
    //   by geometric cardinal direction (Up/Down/Left/Right) for every
    //   coarse node. Keeping the raw samples allows swapping aggregation
    //   policies later without recollecting edges.
    // - `InternalDirectionalArcMetrics` stores the reduced single-value
    //   summaries per direction (e.g., average or minimum). Each entry is
    //   optional to represent the absence of internal edges in that
    //   direction.
    struct InternalDirectionalArcSamples { std::array<std::vector<double>,4> weights; };
    struct InternalDirectionalArcMetrics { std::array<std::optional<double>,4> weights; };
    std::vector<InternalDirectionalArcSamples> internal_directional_arc_samples;
    std::vector<InternalDirectionalArcMetrics> internal_directional_arc_metrics;

    std::vector<std::vector<std::vector<int>>> nodes_at_location; // r,c -> vector of graph IDs at this coarse location

    // lemon id and map id lookups
    std::vector<int> node_to_maploc; // index: lemon node id -> graph id
    std::vector<int> maploc_to_node; // reverse: graph id -> lemon node id

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
CoarsenedGraph* Coarsen(const CoarsenedGraph& graph);

/**
 * Build a compact string summary of the graph fields and bookkeeping sizes.
 */
std::string summarise_graph(const CoarsenedGraph& graph);

} // namespace MapReductionTest
