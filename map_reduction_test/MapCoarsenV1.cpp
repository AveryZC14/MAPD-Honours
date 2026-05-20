#include "MapCoarsenV1.h"

#include <algorithm>
#include <deque>
#include <iostream>
#include <sstream>

namespace MapReductionTest {

using lemon::INVALID;
using lemon::ListDigraph;

namespace {

// Collect the graph IDs stored in the 2x2 coarse cell block anchored at
// (base_row, base_col). This stays local to the current coarse cell so the
// coarsening step only reasons about a small neighborhood at a time.
void append_group_nodes(const CoarsenedGraph& graph,
                        int base_row,
                        int base_col,
                        std::vector<int>& nodes_in_group)
{
    for (int dr = 0; dr < 2; ++dr)
    {
        for (int dc = 0; dc < 2; ++dc)
        {
            const int row = base_row + dr;
            const int col = base_col + dc;

            if (row < 0 || row >= static_cast<int>(graph.nodes_at_location.size()))
                continue;
            if (col < 0 || col >= static_cast<int>(graph.nodes_at_location[row].size()))
                continue;

            nodes_in_group.insert(nodes_in_group.end(),
                                  graph.nodes_at_location[row][col].begin(),
                                  graph.nodes_at_location[row][col].end());
        }
    }
}

// Run a BFS over the induced subgraph formed by nodes_in_group only.
// The search is deliberately restricted to this set so it splits the 2x2
// block into connected pieces without ever leaking into neighboring blocks.
std::vector<std::vector<int>> collect_connected_components(const CoarsenedGraph& graph,
                                                           const std::vector<int>& nodes_in_group)
{
    std::vector<std::vector<int>> connected_components;
    if (nodes_in_group.empty())
        return connected_components;

    // Membership is checked on every expansion so the search never leaves the
    // current 2x2 group.
    std::vector<char> in_group(graph.map_nodes.size(), false);
    std::vector<char> visited(graph.map_nodes.size(), false);

    for (const int node_id : nodes_in_group)
    {
        if (node_id >= 0 && node_id < static_cast<int>(in_group.size()))
            in_group[node_id] = true;
    }

    for (const int start_node_id : nodes_in_group)
    {
        if (start_node_id < 0 || start_node_id >= static_cast<int>(visited.size()))
            continue;
        if (!in_group[start_node_id] || visited[start_node_id])
            continue;

        std::vector<int> component;
        std::deque<int> frontier;
        frontier.push_back(start_node_id);
        visited[start_node_id] = true;

        while (!frontier.empty())
        {
            const int current_id = frontier.front();
            frontier.pop_front();
            component.push_back(current_id);

            const lemon::ListDigraph::Node current_node = graph.map_nodes[current_id];
            if (current_node == lemon::INVALID)
                continue;

            // Explore both outgoing and incoming arcs so the connected
            // component matches the undirected notion used by the coarsener.
            for (lemon::ListDigraph::OutArcIt arc(graph.g, current_node); arc != lemon::INVALID; ++arc)
            {
                const int next_lid = graph.g.id(graph.g.target(arc));
                if (next_lid < 0 || next_lid >= static_cast<int>(graph.node_to_maploc.size()))
                    continue;
                const int next_id = graph.node_to_maploc[next_lid];
                if (next_id < 0 || next_id >= static_cast<int>(visited.size()))
                    continue;
                if (!in_group[next_id] || visited[next_id])
                    continue;

                visited[next_id] = true;
                frontier.push_back(next_id);
            }

            for (lemon::ListDigraph::InArcIt arc(graph.g, current_node); arc != lemon::INVALID; ++arc)
            {
                const int next_lid = graph.g.id(graph.g.source(arc));
                if (next_lid < 0 || next_lid >= static_cast<int>(graph.node_to_maploc.size()))
                    continue;
                const int next_id = graph.node_to_maploc[next_lid];
                if (next_id < 0 || next_id >= static_cast<int>(visited.size()))
                    continue;
                if (!in_group[next_id] || visited[next_id])
                    continue;

                visited[next_id] = true;
                frontier.push_back(next_id);
            }
        }

        connected_components.push_back(std::move(component));
    }

    return connected_components;
}

// Print a detailed view of the connected components discovered for one coarse
// 2x2 block so the BFS result is easy to inspect while debugging.
void dump_connected_components(const CoarsenedGraph& graph,
                               int coarse_row,
                               int coarse_col,
                               const std::vector<std::vector<int>>& connected_components)
{
    std::cout << "[Coarsen] coarse block (" << coarse_row << ", " << coarse_col << ")"
              << " produced " << connected_components.size() << " connected component(s)\n";

    for (size_t component_index = 0; component_index < connected_components.size(); ++component_index)
    {
        const std::vector<int>& component = connected_components[component_index];
        std::cout << "  component " << component_index
                  << " size=" << component.size()
                  << " node_ids=[";

        for (size_t node_index = 0; node_index < component.size(); ++node_index)
        {
            const int node_id = component[node_index];
            std::cout << node_id;

            if (node_id >= 0 && node_id < static_cast<int>(graph.map_nodes.size()))
            {
                const lemon::ListDigraph::Node node = graph.map_nodes[node_id];
                if (node != lemon::INVALID)
                {
                    const std::pair<int, int> coarse_xy = graph.coarse_location[node];
                    const std::pair<int, int> fine_xy = graph.fine_location[node];
                    std::cout << "(coarse=" << coarse_xy.first << "," << coarse_xy.second
                              << "; fine=" << fine_xy.first << "," << fine_xy.second << ")";
                }
            }

            if (node_index + 1 < component.size())
                std::cout << ", ";
        }

        std::cout << "]\n";
    }
}

} // namespace

// Populate `newGraph` for a single discovered connected component.
void populate_new_graph_for_component(CoarsenedGraph* newGraph,
                                     const CoarsenedGraph& oldGraph,
                                     int new_id,
                                     int row,
                                     int col,
                                     const std::vector<int>& nodes)
{
    lemon::ListDigraph::Node n = newGraph->map_nodes[new_id];
    if (n != lemon::INVALID)
    {
        newGraph->coarse_location[n] = {row, col};
    }

    newGraph->to_finer_node_ids[new_id] = nodes;

    if (row >= 0 && row < static_cast<int>(newGraph->nodes_at_location.size()) &&
        col >= 0 && col < static_cast<int>(newGraph->nodes_at_location[row].size()))
    {
        newGraph->nodes_at_location[row][col].push_back(new_id);
    }

    // Update upward mapping on the original (finer) graph so each finer
    // node knows its parent coarse node id. We must cast away constness
    // because the API declares the input graph const.
    CoarsenedGraph &wgraph = const_cast<CoarsenedGraph&>(oldGraph);
    for (int old_node_id : nodes)
    {
        if (old_node_id >= 0 && old_node_id < static_cast<int>(wgraph.to_coarser_node_id.size()))
        {
            wgraph.to_coarser_node_id[old_node_id] = new_id;
        }
    }
}

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
    // Reset the underlying graph and bookkeeping containers.
    graph.g.clear();
    graph.node_to_maploc.clear();
    graph.maploc_to_node.clear();
    graph.to_coarser_node_id.clear();
    graph.to_finer_node_ids.clear();
    graph.map_nodes.clear();
    graph.nodes_at_location.clear();

    // Create the source and sink nodes first, matching the scheduler pattern.
    graph.source = graph.g.addNode();
    graph.sink = graph.g.addNode();

    // Reserve space for one conceptual graph/node per fine map location.
    graph.map_nodes.resize(fine_map_size);
    graph.maploc_to_node.assign(fine_map_size, -1);
    graph.to_coarser_node_id.assign(fine_map_size, -1);
    graph.to_finer_node_ids.assign(fine_map_size, std::vector<int>());

    // If the graph has coarse dimensions set, initialize nodes_at_location
    if (graph.coarse_rows > 0 && graph.coarse_cols > 0)
    {
        graph.nodes_at_location.assign(graph.coarse_rows, std::vector<std::vector<int>>(graph.coarse_cols));
    }

    // Create one LEMON node per fine map location and populate lookup maps.
    std::vector<std::pair<int,int>> id_to_graphid; id_to_graphid.reserve(fine_map_size);
    for (int gid = 0; gid < fine_map_size; ++gid)
    {
        lemon::ListDigraph::Node n = graph.g.addNode();
        graph.map_nodes[gid] = n;                     // graph id -> lemon node
        const int lid = ListDigraph::id(n);
        id_to_graphid.emplace_back(lid, gid);
        graph.maploc_to_node[gid] = lid;

        // If we have 2D coarse location layout, record the graph id at that location
        if (!graph.nodes_at_location.empty())
        {
            const int row = (graph.coarse_cols > 0) ? (gid / graph.coarse_cols) : 0;
            const int col = (graph.coarse_cols > 0) ? (gid % graph.coarse_cols) : gid;
            if (row >= 0 && row < static_cast<int>(graph.nodes_at_location.size()) && col >= 0 && col < graph.coarse_cols)
                graph.nodes_at_location[row][col].push_back(gid);
        }
    }

    // Build reverse map: lemon id -> graph id
    int max_lid = -1;
    for (const auto &p : id_to_graphid) max_lid = std::max(max_lid, p.first);
    if (max_lid >= 0)
    {
        graph.node_to_maploc.assign(max_lid + 1, -1);
        for (const auto &p : id_to_graphid)
            graph.node_to_maploc[p.first] = p.second;
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

    lemon::ListDigraph::Node node = graph.map_nodes[node_index];
    if (node != lemon::INVALID)
    {
        graph.coarse_location[node] = coarse_xy;
        graph.fine_location[node] = fine_xy;
    }
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
    graph.level_idx = 0;
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
        // map_nodes holds one LEMON node per conceptual graph id at this fine location
        lemon::ListDigraph::Node node = graph.map_nodes[loc];
        if (node != lemon::INVALID)
        {
            graph.fine_location[node] = {row, col};
            graph.coarse_location[node] = {row, col};
        }

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

            lemon::ListDigraph::Node from = graph.map_nodes[loc];
            lemon::ListDigraph::Node to = graph.map_nodes[neighbor_loc];
            if (from == lemon::INVALID || to == lemon::INVALID)
                continue;

            ListDigraph::Arc arc = graph.g.addArc(from, to);
            graph.cost[arc] = 1.0;
            graph.capacity[arc] = 1;
        }
    }
}

CoarsenedGraph* Coarsen(const CoarsenedGraph& graph){
    //make the new graph (omg pointers)
    CoarsenedGraph* newGraph = new CoarsenedGraph();

    //int division to get the new num of rows and cols
    newGraph->coarse_rows = (graph.coarse_rows+1) / 2;
    newGraph->coarse_cols = (graph.coarse_cols+1) / 2;
    // newGraph.num_coarse_nodes = 

    // First pass: collect all connected components across 2x2 blocks and
    // remember their coarse coordinates so we can allocate the new graph
    // storage in one go.
    struct CompInfo { int row; int col; std::vector<int> nodes; };
    std::vector<CompInfo> all_components;

    for (int i = 0; i < newGraph->coarse_rows; ++i)
    {
        for (int j = 0; j < newGraph->coarse_cols; ++j)
        {
            std::vector<int> nodes_in_group;
            append_group_nodes(graph, i * 2, j * 2, nodes_in_group);

            if (nodes_in_group.empty())
                continue;

            const std::vector<std::vector<int>> connected_components =
                collect_connected_components(graph, nodes_in_group);

            dump_connected_components(graph, i, j, connected_components);

            for (const auto &comp : connected_components)
            {
                all_components.push_back(CompInfo{i, j, comp});
            }
        }
    }

    // Allocate backing storage for the coarser graph: one conceptual node per
    // connected component discovered.
    const int num_new_nodes = static_cast<int>(all_components.size());
    reserve_fine_map(*newGraph, num_new_nodes);
    newGraph->level_idx = graph.level_idx + 1;
    newGraph->num_coarse_nodes = num_new_nodes;

    // Second pass: populate newGraph bookkeeping and link back to original
    // graph nodes via to_coarser_node_id.
    for (int new_id = 0; new_id < num_new_nodes; ++new_id)
    {
        const CompInfo &ci = all_components[new_id];
        populate_new_graph_for_component(newGraph, graph, new_id, ci.row, ci.col, ci.nodes);
    }

    return newGraph;
    
}

std::string summarise_graph(const CoarsenedGraph& graph)
{
    std::ostringstream out;
    out << "level_idx=" << graph.level_idx << "\n"
        << ", coarse_rows=" << graph.coarse_rows << "\n"
        << ", coarse_cols=" << graph.coarse_cols << "\n"
        << ", num_coarse_nodes=" << graph.num_coarse_nodes << "\n"
        << ", map_nodes.size()=" << graph.map_nodes.size() << "\n"
        << ", nodes_at_location.size()=" << graph.nodes_at_location.size() << "\n"
        << ", node_to_maploc.size()=" << graph.node_to_maploc.size() << "\n"
        << ", maploc_to_node.size()=" << graph.maploc_to_node.size() << "\n"
        << ", to_coarser_node_id.size()=" << graph.to_coarser_node_id.size() << "\n"
        << ", to_finer_node_ids.size()=" << graph.to_finer_node_ids.size() << "\n";

    if (graph.nodes_at_location.empty())
    {
        out << ", nodes_at_location_inner=0";
    }
    else
    {
        out << ", nodes_at_location_inner=" << graph.nodes_at_location.front().size();
    }

    return out.str();
}

} // namespace MapReductionTest
