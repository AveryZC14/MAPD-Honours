#include "MapCoarsenV1.h"

#include <algorithm>
#include <deque>
#include <iostream>

namespace MapReductionTest {

using lemon::INVALID;
using lemon::ListDigraph;

/**
 * Construct the graph-owned LEMON maps and optionally pre-allocate the fine
 * node array.
 */
CoarsenedGraph::CoarsenedGraph(int fine_map_size)
    : coarse_location(g)
    , fine_location(g)
    , supply(g)
    , cost(g)
    , capacity(g)
    , flow(g)
{
    reserve_fine_map(*this, fine_map_size);
}

/**
 * Rebuild the graph's fine-node storage and reset the bookkeeping maps.
 */
void reserve_fine_map(CoarsenedGraph& graph, int fine_map_size)
{
    // Reset the underlying graph before recreating the node layout.
    graph.g.clear();
    graph.node_to_maploc.clear();
    graph.maploc_to_node.clear();
    graph.map_nodes.clear();
    graph.map_nodes.resize(fine_map_size);

    // Create the source and sink nodes first, matching the scheduler pattern.
    graph.source = graph.g.addNode();
    graph.sink = graph.g.addNode();

    // Allocate one node per fine-grid location and store both lookup directions.
    for (int i = 0; i < fine_map_size; ++i)
    {
        graph.map_nodes[i] = graph.g.addNode();
        const int id = ListDigraph::id(graph.map_nodes[i]);
        graph.node_to_maploc[id] = i;
        graph.maploc_to_node[i] = id;
    }
}

/**
 * Attach coarse and fine coordinates to a fine-grid node.
 */
void set_node_coordinates(CoarsenedGraph& graph,
                          int node_index,
                          const std::pair<int, int>& coarse_xy,
                          const std::pair<int, int>& fine_xy)
{
    if (node_index < 0 || node_index >= static_cast<int>(graph.map_nodes.size()))
        return;

    const ListDigraph::Node node = graph.map_nodes[node_index];
    graph.coarse_location[node] = coarse_xy;
    graph.fine_location[node] = fine_xy;
}

/**
 * Build the uncoarsened fine graph directly from the environment map.
 *
 * This creates one node per free map cell, stores the cell coordinates on the
 * node, and connects four-neighbor walkable cells with unit-cost arcs.
 */
void build_from_environment(CoarsenedGraph& graph, const SharedEnvironment* env)
{
    if (env == nullptr)
        return;

    // The level currently being built is the fine graph, so the dimensions
    // mirror the full environment grid.
    graph.coarse_rows = env->rows;
    graph.coarse_cols = env->cols;
    graph.num_coarse_nodes = static_cast<int>(env->map.size());

    // Recreate the backing storage so the graph matches the current map.
    reserve_fine_map(graph, static_cast<int>(env->map.size()));

    // The fine graph is a plain map graph, so the bookkeeping maps are reset to
    // a neutral default state for this level.
    graph.supply[graph.source] = 0;
    graph.supply[graph.sink] = 0;

    // Add the four-neighbor movement edges for every walkable cell.
    const std::vector<int> neighbor_offsets = {-env->cols, 1, env->cols, -1};

    for (int loc = 0; loc < static_cast<int>(env->map.size()); ++loc)
    {
        if (env->map[loc] != 0)
            continue;

        const int row = env->cols > 0 ? loc / env->cols : 0;
        const int col = env->cols > 0 ? loc % env->cols : 0;
        graph.fine_location[graph.map_nodes[loc]] = {row, col};
        graph.coarse_location[graph.map_nodes[loc]] = {row, col};

        for (int i = 0; i < 4; ++i)
        {
            const int neighbor_loc = loc + neighbor_offsets[i];
            if (neighbor_loc < 0 || neighbor_loc >= static_cast<int>(env->map.size()))
                continue;
            if (env->map[neighbor_loc] != 0)
                continue;

            // Prevent wrapping across row boundaries when using +/-1 offsets.
            const int nrow = env->cols > 0 ? neighbor_loc / env->cols : 0;
            const int ncol = env->cols > 0 ? neighbor_loc % env->cols : 0;
            // neighbor must be a direct four-neighbor (manhattan distance == 1)
            if (!((nrow == row && (ncol == col + 1 || ncol == col - 1)) ||
                  (ncol == col && (nrow == row + 1 || nrow == row - 1))))
            {
                continue;
            }

            ListDigraph::Arc arc = graph.g.addArc(graph.map_nodes[loc], graph.map_nodes[neighbor_loc]);
            graph.cost[arc] = 1.0;
            graph.capacity[arc] = 1;
        }
    }
}

CoarsenedGraph Coarsen(const CoarsenedGraph& graph){
    //make the new graph (omg pointers)
    CoarsenedGraph* newGraph = new CoarsenedGraph();

    //int division to get the new num of rows and cols
    newGraph->coarse_rows = (graph.coarse_rows+1) / 2;
    newGraph->coarse_cols = (graph.coarse_cols+1) / 2;
    // newGraph.num_coarse_nodes = 

    
}

} // namespace MapReductionTest
