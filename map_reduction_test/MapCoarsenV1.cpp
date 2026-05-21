#include "MapCoarsenV1.h"

#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <sstream>

namespace MapReductionTest {

using lemon::INVALID;
using lemon::ListDigraph;

namespace {

// Cardinal directions used for bucketing internal edges.
enum class CardinalDirection : std::size_t { Up = 0, Down = 1, Left = 2, Right = 3 };

constexpr std::size_t direction_index(CardinalDirection d) { return static_cast<std::size_t>(d); }

// Reduce a bucket of arc weights according to a selected policy.
std::optional<double> reduce_arc_weight_bucket(const std::vector<double>& bucket,
                                               CoarsenedGraph::ArcAggregationPolicy policy)
{
    if (bucket.empty())
        return std::nullopt;

    if (policy == CoarsenedGraph::ArcAggregationPolicy::Minimum)
    {
        return *std::min_element(bucket.begin(), bucket.end());
    }

    double sum = 0.0;
    for (double value : bucket)
        sum += value;
    return sum / static_cast<double>(bucket.size());
}

// Classify the vector from a -> b into a cardinal direction if possible.
std::optional<CardinalDirection> classify_delta(int dr, int dc)
{
    if (dr == -1 && dc == 0) return CardinalDirection::Up;
    if (dr == 1 && dc == 0) return CardinalDirection::Down;
    if (dr == 0 && dc == -1) return CardinalDirection::Left;
    if (dr == 0 && dc == 1) return CardinalDirection::Right;
    return std::nullopt;
}

// Collect internal arc costs for a connected component and bucket them by
// geometric direction. We canonicalize undirected edges (count once) using
// node id ordering (node_id < neighbor_id) to avoid duplicates.
CoarsenedGraph::InternalDirectionalArcSamples collect_internal_directional_arc_samples(const CoarsenedGraph& graph,
                                                                                     const std::vector<int>& nodes)
{
    CoarsenedGraph::InternalDirectionalArcSamples samples;
    if (nodes.empty()) return samples;

    std::vector<char> in_component(graph.map_nodes.size(), false);
    for (int id : nodes)
        if (id >= 0 && id < static_cast<int>(in_component.size())) in_component[id] = true;

    for (int node_id : nodes)
    {
        if (node_id < 0 || node_id >= static_cast<int>(graph.map_nodes.size())) continue;
        const lemon::ListDigraph::Node node = graph.map_nodes[node_id];
        if (node == lemon::INVALID) continue;

        for (lemon::ListDigraph::OutArcIt arc(graph.g, node); arc != lemon::INVALID; ++arc)
        {
            const int next_lid = graph.g.id(graph.g.target(arc));
            if (next_lid < 0 || next_lid >= static_cast<int>(graph.node_to_maploc.size())) continue;
            const int next_id = graph.node_to_maploc[next_lid];
            if (next_id < 0 || next_id >= static_cast<int>(in_component.size())) continue;

            // Only internal edges
            if (!in_component[next_id]) continue;

            // Canonicalize undirected edges to a single record.
            if (node_id >= next_id) continue;

            const lemon::ListDigraph::Node next_node = graph.map_nodes[next_id];
            if (next_node == lemon::INVALID) continue;

            const auto a = graph.fine_location[node];
            const auto b = graph.fine_location[next_node];
            const int dr = b.first - a.first;
            const int dc = b.second - a.second;

            const std::optional<CardinalDirection> dir = classify_delta(dr, dc);
            if (!dir) continue;

            const double w = graph.cost[arc];
            samples.weights[direction_index(*dir)].push_back(w);
        }
    }

    return samples;
}

// Reduce using average; kept separate so the policy can be swapped later.
CoarsenedGraph::InternalDirectionalArcMetrics reduce_internal_directional_arc_samples(const CoarsenedGraph::InternalDirectionalArcSamples& s)
{
    CoarsenedGraph::InternalDirectionalArcMetrics m;
    for (std::size_t i = 0; i < m.weights.size(); ++i)
    {
        const auto &bucket = s.weights[i];
        m.weights[i] = reduce_arc_weight_bucket(bucket, CoarsenedGraph::ArcAggregationPolicy::Average);
    }
    return m;
}


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
                               const std::vector<std::vector<int>>& connected_components,
                               const std::vector<CoarsenedGraph::InternalDirectionalArcSamples>& internal_samples)
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

        // Print internal arc summary for this component if samples are available
        if (component_index < internal_samples.size())
        {
            const auto &samples = internal_samples[component_index];
            std::cout << "    internal arcs: ";
            const char *names[4] = {"Up","Down","Left","Right"};
            for (size_t d = 0; d < 4; ++d)
            {
                const auto &bucket = samples.weights[d];
                if (d) std::cout << ", ";
                std::cout << names[d] << ":count=" << bucket.size();
                if (!bucket.empty())
                {
                    double sum = 0.0;
                    for (double v : bucket) sum += v;
                    std::cout << ",avg=" << (sum / static_cast<double>(bucket.size()));
                }
            }
            std::cout << "\n";
        }
    }
}

} // namespace

// Populate `newGraph` for a single discovered connected component.
void populate_new_graph_for_component(CoarsenedGraph* newGraph,
                                     const CoarsenedGraph& oldGraph,
                                     int new_id,
                                     int row,
                                     int col,
                                     const std::vector<int>& nodes,
                                     const CoarsenedGraph::InternalDirectionalArcSamples& internal_directional_arc_samples)
{
    lemon::ListDigraph::Node n = newGraph->map_nodes[new_id];
    if (n != lemon::INVALID)
    {
        newGraph->coarse_location[n] = {row, col};
        newGraph->fine_location[n] = {row, col};
    }

    newGraph->to_finer_node_ids[new_id] = nodes;

    // Save raw samples and reduced metrics for this coarse node.
    if (new_id >= 0 && new_id < static_cast<int>(newGraph->internal_directional_arc_samples.size()))
        newGraph->internal_directional_arc_samples[new_id] = internal_directional_arc_samples;
    if (new_id >= 0 && new_id < static_cast<int>(newGraph->internal_directional_arc_metrics.size()))
        newGraph->internal_directional_arc_metrics[new_id] = reduce_internal_directional_arc_samples(internal_directional_arc_samples);

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
    graph.internal_directional_arc_samples.clear();
    graph.internal_directional_arc_metrics.clear();
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
    graph.internal_directional_arc_samples.assign(fine_map_size, CoarsenedGraph::InternalDirectionalArcSamples{});
    graph.internal_directional_arc_metrics.assign(fine_map_size, CoarsenedGraph::InternalDirectionalArcMetrics{});

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
    newGraph->inter_component_arc_aggregation_policy = graph.inter_component_arc_aggregation_policy;

    //int division to get the new num of rows and cols
    newGraph->coarse_rows = (graph.coarse_rows+1) / 2;
    newGraph->coarse_cols = (graph.coarse_cols+1) / 2;
    // newGraph.num_coarse_nodes = 

    // First pass: collect all connected components across 2x2 blocks and
    // remember their coarse coordinates so we can allocate the new graph
    // storage in one go.
    struct CompInfo { int row; int col; std::vector<int> nodes; CoarsenedGraph::InternalDirectionalArcSamples internal_directional_arc_samples; };
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

            for (const auto &comp : connected_components)
            {
                CompInfo info;
                info.row = i;
                info.col = j;
                info.nodes = comp;
                info.internal_directional_arc_samples = collect_internal_directional_arc_samples(graph, comp);
                all_components.push_back(std::move(info));
            }
        }
    }
    // After collecting components for all coarse blocks, print per-block
    // summaries including internal arc statistics grouped by coarse cell.
    if (!all_components.empty())
    {
        // Find unique coarse block coordinates present and dump their components
        std::map<std::pair<int,int>, std::vector<size_t>> index_map;
        for (size_t idx = 0; idx < all_components.size(); ++idx)
        {
            const auto &ci = all_components[idx];
            index_map[{ci.row, ci.col}].push_back(idx);
        }

        for (const auto &entry : index_map)
        {
            const int brow = entry.first.first;
            const int bcol = entry.first.second;
            std::vector<std::vector<int>> comps;
            std::vector<CoarsenedGraph::InternalDirectionalArcSamples> samples;
            for (size_t idx : entry.second)
            {
                comps.push_back(all_components[idx].nodes);
                samples.push_back(all_components[idx].internal_directional_arc_samples);
            }

            dump_connected_components(graph, brow, bcol, comps, samples);
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
        populate_new_graph_for_component(newGraph, graph, new_id, ci.row, ci.col, ci.nodes, ci.internal_directional_arc_samples);
    }

    // Third pass: create coarse arcs between neighboring connected components.
    //
    // For each fine-level arc that crosses from component A to component B,
    // collect the arc cost into bucket (A,B). Then reduce each bucket with the
    // configured policy (average or minimum) and create one coarse arc A->B.
    std::map<std::pair<int, int>, std::vector<double>> inter_component_arc_samples;

    for (lemon::ListDigraph::ArcIt arc(graph.g); arc != lemon::INVALID; ++arc)
    {
        const int src_lid = graph.g.id(graph.g.source(arc));
        const int dst_lid = graph.g.id(graph.g.target(arc));
        if (src_lid < 0 || dst_lid < 0)
            continue;
        if (src_lid >= static_cast<int>(graph.node_to_maploc.size()) ||
            dst_lid >= static_cast<int>(graph.node_to_maploc.size()))
            continue;

        const int src_id = graph.node_to_maploc[src_lid];
        const int dst_id = graph.node_to_maploc[dst_lid];
        if (src_id < 0 || dst_id < 0)
            continue;
        if (src_id >= static_cast<int>(graph.to_coarser_node_id.size()) ||
            dst_id >= static_cast<int>(graph.to_coarser_node_id.size()))
            continue;

        const int coarse_src = graph.to_coarser_node_id[src_id];
        const int coarse_dst = graph.to_coarser_node_id[dst_id];

        // Skip arcs that do not cross components.
        if (coarse_src < 0 || coarse_dst < 0 || coarse_src == coarse_dst)
            continue;

        if (coarse_src >= static_cast<int>(newGraph->map_nodes.size()) ||
            coarse_dst >= static_cast<int>(newGraph->map_nodes.size()))
            continue;

        // Defensive adjacency check. By construction these components should be
        // neighbors in the coarse coordinate grid.
        const lemon::ListDigraph::Node coarse_src_node = newGraph->map_nodes[coarse_src];
        const lemon::ListDigraph::Node coarse_dst_node = newGraph->map_nodes[coarse_dst];
        if (coarse_src_node == lemon::INVALID || coarse_dst_node == lemon::INVALID)
            continue;

        const std::pair<int, int> src_xy = newGraph->coarse_location[coarse_src_node];
        const std::pair<int, int> dst_xy = newGraph->coarse_location[coarse_dst_node];
        const int manhattan_dist = std::abs(src_xy.first - dst_xy.first) +
                                   std::abs(src_xy.second - dst_xy.second);
        if (manhattan_dist != 1)
            continue;

        inter_component_arc_samples[{coarse_src, coarse_dst}].push_back(graph.cost[arc]);
    }

    for (const auto &kv : inter_component_arc_samples)
    {
        const int coarse_src = kv.first.first;
        const int coarse_dst = kv.first.second;
        const std::optional<double> reduced_weight =
            reduce_arc_weight_bucket(kv.second, newGraph->inter_component_arc_aggregation_policy);
        if (!reduced_weight)
            continue;

        const lemon::ListDigraph::Node coarse_src_node = newGraph->map_nodes[coarse_src];
        const lemon::ListDigraph::Node coarse_dst_node = newGraph->map_nodes[coarse_dst];
        if (coarse_src_node == lemon::INVALID || coarse_dst_node == lemon::INVALID)
            continue;

        const ListDigraph::Arc coarse_arc = newGraph->g.addArc(coarse_src_node, coarse_dst_node);
        newGraph->cost[coarse_arc] = *reduced_weight;
        newGraph->capacity[coarse_arc] = 1;
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
