#include "MapCoarsenV1.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <deque>
#include <iostream>
#include <map>
#include <sstream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <lemon/network_simplex.h>

namespace MapReductionTest {

std::size_t CoarsenedGraph::PairHash::operator()(const std::pair<int, int>& p) const noexcept
{
    const std::size_t a = static_cast<std::size_t>(static_cast<std::uint32_t>(p.first));
    const std::size_t b = static_cast<std::size_t>(static_cast<std::uint32_t>(p.second));
    return (a << 32) ^ b;
}

using lemon::INVALID;
using lemon::ListDigraph;

// Default number of coarsening steps to build from the fine graph.
// Increase this to push the hierarchy deeper, or set it to 0 to keep only
// the fine level.
constexpr int kDefaultCoarsenLevels = 2;

// High-level overview:
// This file implements a simple multilevel coarsening pipeline for grid maps
// and a `ReducedHierarchy` controller that exposes a reduced-flow scheduler.
//
// - Coarsening: the fine map is partitioned into 2x2 coarse blocks. Within
//   each block we discover connected components (via BFS restricted to the
//   block). Each connected component becomes a coarse-level node. Internal
//   directional arc statistics are gathered per component to bias coarse
//   inter-component arc costs (so coarse edges respect corridor directions).
//
// - Reduced scheduling: the coarsened hierarchy is cached in
//   `ReducedHierarchy`. For assignment, agents/tasks are mapped up to the
//   topmost coarse level, a compact flow network is built over top-level
//   nodes and solved with LEMON's NetworkSimplex, then the top-level flows
//   are decomposed into agent→task paths and greedily lifted down the
//   hierarchy into fine-level guide paths.
//
// The design aims for: reusing the hierarchy across scheduling calls (via a
// signature), keeping the top-level flow small, and producing quick guide
// paths for downstream planners. The lifting is heuristic (bridge-based)
// and intentionally lightweight.

namespace {

// Cardinal directions used for bucketing internal edges.
enum class CardinalDirection : std::size_t { Up = 0, Down = 1, Left = 2, Right = 3 };

constexpr std::size_t direction_index(CardinalDirection d) { return static_cast<std::size_t>(d); }

// Safely read one coarse node's internal directional metric value.
// Missing metrics (or out-of-range ids) contribute zero to the edge weight.
double get_internal_directional_metric_or_zero(const CoarsenedGraph& graph,
                                               int coarse_node_id,
                                               CardinalDirection direction)
{
    if (coarse_node_id < 0 ||
        coarse_node_id >= static_cast<int>(graph.internal_directional_arc_metrics.size()))
    {
        return 0.0;
    }

    const auto &metrics = graph.internal_directional_arc_metrics[coarse_node_id];
    const std::optional<double> maybe_value = metrics.weights[direction_index(direction)];
    return maybe_value ? *maybe_value : 0.0;
}

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
// geometric direction. Each directed arc is counted once; this preserves the
// true directional balance of the component.
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

// Build a cheapest path inside the union of two parent components. This is
// only used while preprocessing a coarse edge, so the work happens once and
// can be cached on the resulting coarsened graph.
std::vector<int> build_cached_bridge_path_local(const CoarsenedGraph& graph,
                                                int start_id,
                                                int goal_id,
                                                int parent_a,
                                                int parent_b)
{
    const auto valid_node = [&graph](int node_id) {
        return node_id >= 0 &&
               node_id < static_cast<int>(graph.map_nodes.size()) &&
               graph.map_nodes[node_id] != lemon::INVALID;
    };

    std::vector<int> empty;
    if (start_id < 0 || goal_id < 0)
        return empty;
    if (start_id >= static_cast<int>(graph.map_nodes.size()) ||
        goal_id >= static_cast<int>(graph.map_nodes.size()))
        return empty;
    if (!valid_node(start_id) || !valid_node(goal_id))
        return empty;
    if (start_id == goal_id)
        return {start_id};

    // This search is always confined to the union of two adjacent coarse
    // components (each at most a handful of fine nodes, by construction of
    // the 2x2-block coarsening), so it must never allocate state sized to
    // the whole finer graph -- `graph` here is the level being coarsened,
    // which is the *entire* fine map (e.g. ~1M nodes) the first time a level
    // 0->1 transition is built. A vector<int>/vector<char> pair sized to
    // that, allocated and zero-initialized once per bridge_cache entry
    // (there can be tens of thousands of those), is what made hierarchy
    // construction on large maps take gigabytes and many seconds. Use hash
    // maps bounded by the actual (tiny) number of nodes visited instead.
    std::unordered_map<int, int> prev;
    std::unordered_set<int> seen;
    std::deque<int> q;
    seen.insert(start_id);
    q.push_back(start_id);

    while (!q.empty())
    {
        const int u = q.front();
        q.pop_front();
        if (u == goal_id)
            break;

        const lemon::ListDigraph::Node u_node = graph.map_nodes[u];
        if (u_node == lemon::INVALID)
            continue;

        for (lemon::ListDigraph::OutArcIt arc(graph.g, u_node); arc != lemon::INVALID; ++arc)
        {
            const int v_lid = graph.g.id(graph.g.target(arc));
            if (v_lid < 0 || v_lid >= static_cast<int>(graph.node_to_maploc.size()))
                continue;

            const int v = graph.node_to_maploc[v_lid];
            if (!valid_node(v))
                continue;
            if (v >= static_cast<int>(graph.to_coarser_node_id.size()))
                continue;

            const int v_parent = graph.to_coarser_node_id[v];
            if (v_parent != parent_a && v_parent != parent_b)
                continue;

            if (seen.insert(v).second)
            {
                prev[v] = u;
                q.push_back(v);
            }
        }
    }

    const auto goal_it = prev.find(goal_id);
    if (goal_it == prev.end())
        return empty;

    std::vector<int> path;
    for (int cur = goal_id; cur != -1; )
    {
        path.push_back(cur);
        const auto it = prev.find(cur);
        cur = (it != prev.end()) ? it->second : -1;
    }
    std::reverse(path.begin(), path.end());
    return path;
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

    // Store one stable child representative for this coarse node.
    // This is always taken from `to_finer_node_ids[new_id]` so later code can
    // anchor bridge lookup and local lifting without rescanning children.
    if (new_id >= 0 && new_id < static_cast<int>(newGraph->chosen_finer_node_id.size()))
        newGraph->chosen_finer_node_id[new_id] = nodes.empty() ? -1 : nodes.front();

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
    // graph.node_to_maploc.clear();
    // graph.maploc_to_node.clear();
    // graph.to_coarser_node_id.clear();
    // graph.to_finer_node_ids.clear();
    // graph.chosen_finer_node_id.clear();

    // Completely deallocate vectors to free nested capacity leaks
    std::vector<int>().swap(graph.node_to_maploc);
    std::vector<int>().swap(graph.maploc_to_node);
    std::vector<int>().swap(graph.to_coarser_node_id);
    std::vector<std::vector<int>>().swap(graph.to_finer_node_ids); // Releases inner capacities!
    std::vector<int>().swap(graph.chosen_finer_node_id);
    std::vector<lemon::ListDigraph::Node>().swap(graph.map_nodes);
    std::vector<std::vector<std::vector<int>>>().swap(graph.nodes_at_location);

        // Explicitly re-initialize LEMON maps to bind to the fresh graph structure
    graph.cost.~ArcMap(); 
    new (&graph.cost) lemon::ListDigraph::ArcMap<double>(graph.g);

    graph.capacity.~ArcMap();
    new (&graph.capacity) lemon::ListDigraph::ArcMap<int>(graph.g);

    // Do the same for coarse_location if it's a LEMON NodeMap
    graph.coarse_location.~NodeMap();
    new (&graph.coarse_location) lemon::ListDigraph::NodeMap<std::pair<int,int>>(graph.g);


    graph.bridge_cache.clear();
    graph.bridge_path_cache.clear();
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
    graph.chosen_finer_node_id.assign(fine_map_size, -1);
    graph.internal_directional_arc_samples.assign(fine_map_size, CoarsenedGraph::InternalDirectionalArcSamples{});
    graph.internal_directional_arc_metrics.assign(fine_map_size, CoarsenedGraph::InternalDirectionalArcMetrics{});

    // Location buckets are created empty here; the fine builder decides which
    // cells are actually useful enough to be recorded later.
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

    if (graph.coarse_rows > 0 && graph.coarse_cols > 0)
    {
        // Start with an empty coarse-grid table for the fine map.
        graph.nodes_at_location.assign(graph.coarse_rows, std::vector<std::vector<int>>(graph.coarse_cols));
    }

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

        bool is_dud_space = true;
        for (lemon::ListDigraph::OutArcIt arc(graph.g, node); arc != lemon::INVALID; ++arc)
        {
            is_dud_space = false;
            break;
        }

        // Dud spaces stay out of the coarse location table so they can be
        // filtered or reinstated independently from the graph structure.
        if (is_dud_space)
            continue;

        // Only non-dud spaces are recorded in the location table.
        if (graph.nodes_at_location.empty())
            continue;

        if (row >= 0 && row < static_cast<int>(graph.nodes_at_location.size()) &&
            col >= 0 && col < static_cast<int>(graph.nodes_at_location[row].size()))
        {
            graph.nodes_at_location[row][col].push_back(loc);
        }
    }
}

std::unique_ptr<CoarsenedGraph> Coarsen(const CoarsenedGraph& graph) {
    // Automatically cleaned up if something throws or if the caller drops it
    auto newGraph = std::make_unique<CoarsenedGraph>();
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

            // dump_connected_components(graph, brow, bcol, comps, samples);
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
        populate_new_graph_for_component(newGraph.get(), graph, new_id, ci.row, ci.col, ci.nodes, ci.internal_directional_arc_samples);
    }

    // Third pass: create coarse arcs between neighboring connected components.
    //
    // For each fine-level arc crossing from component A to component B:
    // 1) collect base costs into a (A,B) bucket,
    // 2) reduce each bucket with the configured policy (average/minimum),
    // 3) add directional internal penalties:
    //      + 0.5 * A(direction A->B) + 0.5 * B(direction A->B),
    // 4) create the coarse arc A->B with that final weight.
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

        // Remember one representative fine bridge for this coarse neighbor
        // pair so lifting can avoid rescanning the entire fine graph later.
        const std::pair<int, int> bridge_key{coarse_src, coarse_dst};
        const auto bridge_value = std::make_pair(src_id, dst_id);
        const double bridge_cost = graph.cost[arc];
        const auto cache_it = newGraph->bridge_cache.find(bridge_key);
        if (cache_it == newGraph->bridge_cache.end())
        {
            newGraph->bridge_cache.emplace(bridge_key, bridge_value);
        }
        else
        {
            const int cached_src = cache_it->second.first;
            const int cached_dst = cache_it->second.second;
            bool replace_cache = false;
            if (cached_src < 0 || cached_dst < 0 ||
                cached_src >= static_cast<int>(graph.map_nodes.size()) ||
                cached_dst >= static_cast<int>(graph.map_nodes.size()))
            {
                replace_cache = true;
            }
            else
            {
                const lemon::ListDigraph::Node cached_src_node = graph.map_nodes[cached_src];
                const lemon::ListDigraph::Node cached_dst_node = graph.map_nodes[cached_dst];
                if (cached_src_node == lemon::INVALID || cached_dst_node == lemon::INVALID)
                {
                    replace_cache = true;
                }
                else
                {
                    // Keep the cheapest bridge we have seen for this pair.
                    double cached_cost = 0.0;
                    bool cached_found = false;
                    for (lemon::ListDigraph::OutArcIt cached_arc(graph.g, cached_src_node); cached_arc != lemon::INVALID; ++cached_arc)
                    {
                        const int next_lid = graph.g.id(graph.g.target(cached_arc));
                        if (next_lid < 0 || next_lid >= static_cast<int>(graph.node_to_maploc.size())) continue;
                        if (graph.node_to_maploc[next_lid] != cached_dst) continue;
                        cached_cost = graph.cost[cached_arc];
                        cached_found = true;
                        break;
                    }
                    if (!cached_found || bridge_cost < cached_cost) replace_cache = true;
                }
            }
            if (replace_cache) cache_it->second = bridge_value;
        }
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

        // Determine edge direction from coarse source to coarse destination.
        const std::pair<int, int> src_xy = newGraph->coarse_location[coarse_src_node];
        const std::pair<int, int> dst_xy = newGraph->coarse_location[coarse_dst_node];
        const std::optional<CardinalDirection> direction =
            classify_delta(dst_xy.first - src_xy.first, dst_xy.second - src_xy.second);
        if (!direction)
            continue;

        // Add half of the internal directional metric from each endpoint.
        // Example: if A->B moves Right, add 0.5*A(Right) + 0.5*B(Right).
        const double src_internal =
            get_internal_directional_metric_or_zero(*newGraph, coarse_src, *direction);
        const double dst_internal =
            get_internal_directional_metric_or_zero(*newGraph, coarse_dst, *direction);

        const double final_weight = *reduced_weight + 0.5 * src_internal + 0.5 * dst_internal;

        const ListDigraph::Arc coarse_arc = newGraph->g.addArc(coarse_src_node, coarse_dst_node);
        newGraph->cost[coarse_arc] = final_weight;
        newGraph->capacity[coarse_arc] = 1;
    }

    // Precompute the actual bridge path for every cached coarse neighbor pair.
    // The cached path is the exact fine-level walk used when the lift crosses
    // from one coarse parent to the next, so runtime expansion becomes a
    // sequence of hash lookups and vector splices instead of fresh searches.
    for (const auto &kv : newGraph->bridge_cache)
    {
        const int from_parent = kv.first.first;
        const int to_parent = kv.first.second;
        if (from_parent < 0 || to_parent < 0 ||
            from_parent >= static_cast<int>(newGraph->chosen_finer_node_id.size()) ||
            to_parent >= static_cast<int>(newGraph->chosen_finer_node_id.size()))
        {
            continue;
        }

        const int start_id = newGraph->chosen_finer_node_id[from_parent];
        const int goal_id = newGraph->chosen_finer_node_id[to_parent];
        const auto valid_node = [&graph](int node_id) {
            return node_id >= 0 &&
                   node_id < static_cast<int>(graph.map_nodes.size()) &&
                   graph.map_nodes[node_id] != lemon::INVALID;
        };
        if (!valid_node(start_id) || !valid_node(goal_id))
            continue;

        CoarsenedGraph::CachedBridgePath cached_path;
        cached_path.bridge = kv.second;
        cached_path.path = build_cached_bridge_path_local(graph, start_id, goal_id, from_parent, to_parent);
        if (cached_path.path.empty())
            continue;

        newGraph->bridge_path_cache[kv.first] = std::move(cached_path);
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

CoarsenedGraph* MultiLevelCoarsenedGraph::level(int level_idx)
{
    if (level_idx < 0 || level_idx >= static_cast<int>(levels.size()))
        return nullptr;
    return levels[level_idx].get();
}

const CoarsenedGraph* MultiLevelCoarsenedGraph::level(int level_idx) const
{
    if (level_idx < 0 || level_idx >= static_cast<int>(levels.size()))
        return nullptr;
    return levels[level_idx].get();
}

bool append_coarsened_level(MultiLevelCoarsenedGraph& hierarchy)
{
    if (hierarchy.levels.empty())
        return false;

    // 1. Change the type from CoarsenedGraph* to std::unique_ptr<CoarsenedGraph>
    std::unique_ptr<CoarsenedGraph> next_level = Coarsen(*hierarchy.levels.back());
    
    // 2. Check for nullptr using the smart pointer directly
    if (next_level == nullptr)
        return false;

    // 3. Move ownership of the unique_ptr into the vector
    hierarchy.levels.push_back(std::move(next_level));
    return true;
}

void build_multilevel_from_environment(MultiLevelCoarsenedGraph& hierarchy,
                                       const SharedEnvironment* env,
                                       int num_additional_levels)
{
    hierarchy.clear();

    auto fine = std::make_unique<CoarsenedGraph>();
    build_from_environment(*fine, env);
    hierarchy.levels.emplace_back(std::move(fine));

    const int levels_to_add = std::max(0, num_additional_levels);
    for (int i = 0; i < levels_to_add; ++i)
    {
        if (!append_coarsened_level(hierarchy))
            break;
    }
}

} // namespace MapReductionTest

// ReducedHierarchy implementation
namespace MapReductionTest {

using lemon::NetworkSimplex;

// Simple FNV-1a style signature
static std::size_t compute_env_signature_local(const SharedEnvironment* env)
{
    if (env == nullptr) return 0;
    std::size_t h = 1469598103934665603ull;
    auto mix = [&h](std::size_t x){ h ^= x; h *= 1099511628211ull; };
    mix(static_cast<std::size_t>(env->rows));
    mix(static_cast<std::size_t>(env->cols));
    mix(static_cast<std::size_t>(env->map.size()));
    for (int v : env->map) mix(static_cast<std::size_t>(v+1));
    return h;
}

ReducedHierarchy& ReducedHierarchy::instance()
{
    static ReducedHierarchy inst;
    return inst;
}

ReducedHierarchy::ReducedHierarchy() = default;
ReducedHierarchy::~ReducedHierarchy() = default;

void ReducedHierarchy::ensure(const SharedEnvironment* env)
{
    if (env == nullptr) return;
    const std::size_t sig = compute_env_signature_local(env);
    if (ready_ && signature_ == sig && !hierarchy_.empty()) return;

    // Completely wipe the old hierarchy to free memory before rebuilding??!?!??!???
    hierarchy_.levels.clear();
    hierarchy_ = MultiLevelCoarsenedGraph(); 

    const int additional_levels = 0;
    // choose levels heuristically (reduce until map dims collapse)
    int rows = std::max(1, env->rows);
    int cols = std::max(1, env->cols);
    while ((rows > 1 || cols > 1) && additional_levels < 10) { rows = (rows+1)/2; cols=(cols+1)/2; }
    const int levels_to_add = std::max(0, kDefaultCoarsenLevels);

    const auto build_start = std::chrono::high_resolution_clock::now();
    build_multilevel_from_environment(hierarchy_, env, levels_to_add);
    const auto build_end = std::chrono::high_resolution_clock::now();

    signature_ = sig;
    ready_ = !hierarchy_.empty();

    last_hierarchy_build_time_ = std::chrono::duration<double>(build_end - build_start).count();
    last_hierarchy_level_node_counts_.clear();
    last_hierarchy_level_node_counts_.reserve(hierarchy_.num_levels());
    for (const auto& level : hierarchy_.levels)
        last_hierarchy_level_node_counts_.push_back(level ? static_cast<int>(level->map_nodes.size()) : 0);
}

bool ReducedHierarchy::ready() const { return ready_; }

double ReducedHierarchy::hierarchy_build_time() const
{
    return last_hierarchy_build_time_;
}

std::vector<int> ReducedHierarchy::hierarchy_level_node_counts() const
{
    return last_hierarchy_level_node_counts_;
}

static bool is_valid_graph_node_id_local(const CoarsenedGraph& graph, int node_id)
{
    return node_id >= 0 && node_id < static_cast<int>(graph.map_nodes.size()) && graph.map_nodes[node_id] != lemon::INVALID;
}

static int map_fine_node_to_level_node_local(const MultiLevelCoarsenedGraph& hierarchy, int fine_node_id, int target_level)
{
    if (target_level < 0 || target_level >= hierarchy.num_levels()) return -1;
    int node_id = fine_node_id;
    for (int level = 0; level < target_level; ++level)
    {
        const CoarsenedGraph* graph = hierarchy.level(level);
        if (!graph) return -1;
        if (node_id < 0 || node_id >= static_cast<int>(graph->to_coarser_node_id.size())) return -1;
        node_id = graph->to_coarser_node_id[node_id];
        if (node_id < 0) return -1;
    }
    return node_id;
}

// Compute a lowest-cost path (Dijkstra) over the provided `graph` from
// `start_id` to `goal_id`. If `constrain_parent` is true then only nodes
// whose `to_coarser_node_id` equals `required_parent` are considered when
// expanding — this restricts the search to the subgraph belonging to a
// specific coarse parent component.
static std::vector<int> shortest_path_in_graph_local(const CoarsenedGraph& graph,
                                                     int start_id,
                                                     int goal_id,
                                                     int required_parent,
                                                     bool constrain_parent){
    if (!is_valid_graph_node_id_local(graph, start_id) || !is_valid_graph_node_id_local(graph, goal_id))
        return {};
    if (start_id == goal_id)
        return {start_id};

    // Use a hash map instead of massive O(N) state vectors to protect system memory
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;

    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

    dist[start_id] = 0.0;
    open.push({0.0, start_id});

    // Hard-cap the total number of node expansions to catch disconnected networks safely
    int expansions = 0;
    const int MAX_EXPANSIONS = 20000;

    while (!open.empty()) {
        if (++expansions > MAX_EXPANSIONS) {
            return {}; // Bail out if search space is exploding
        }

        auto [cd, u] = open.top();
        open.pop();

        if (cd > dist[u]) continue;
        if (u == goal_id) break;

        const lemon::ListDigraph::Node u_node = graph.map_nodes[u]; 
        if (u_node == lemon::INVALID) continue;

        for (lemon::ListDigraph::OutArcIt arc(graph.g, u_node); arc != lemon::INVALID; ++arc) {
            const int v_lid = graph.g.id(graph.g.target(arc)); 
            if (v_lid < 0 || v_lid >= static_cast<int>(graph.node_to_maploc.size())) continue;

            const int v = graph.node_to_maploc[v_lid]; 
            if (!is_valid_graph_node_id_local(graph, v)) continue;

            if (constrain_parent) { 
                if (v < 0 || v >= static_cast<int>(graph.to_coarser_node_id.size())) continue; 
                if (graph.to_coarser_node_id[v] != required_parent) continue; 
            }

            const double nd = cd + graph.cost[arc]; 
            auto it = dist.find(v);
            if (it == dist.end() || nd + 1e-12 < it->second) { 
                dist[v] = nd; 
                prev[v] = u; 
                open.push({nd, v}); 
            }
        }
    }

    if (prev.find(goal_id) == prev.end())
        return {};

    std::vector<int> path;
    for (int cur = goal_id; cur != -1; ) {
        path.push_back(cur);
        auto it = prev.find(cur);
        cur = (it != prev.end()) ? it->second : -1;

        if(path.size() > 5000) return {}; // Cycle emergency brake
    }
    
    std::reverse(path.begin(), path.end()); 
    return path;
}

// Find a cheapest fine-level directed arc (u->v) in `lower` such that
// `to_coarser_node_id[u] == from_parent` and `to_coarser_node_id[v] == to_parent`.
// The bridge is expected to already be cached on `upper`, so a hit is O(1).
static std::pair<int,int> pick_bridge_arc_between_parents_local(const CoarsenedGraph& lower, const CoarsenedGraph& upper, int from_parent, int to_parent)
{
    std::pair<int,int> best{-1,-1}; double best_cost=1e100;

    const auto cached = upper.bridge_cache.find({from_parent, to_parent});
    if (cached != upper.bridge_cache.end())
        return cached->second;

    // Cache miss means the hierarchy was not preprocessed for this pair.
    // We return an invalid bridge rather than falling back to a scan.
    (void)best_cost;
    return best;
}

// Expand an entire batch of coarse-level paths one level down.
// Every coarse node uses its preselected finer representative, and each
// coarse edge is expanded through the cached fine path recorded at coarsen
// time. The runtime work is intentionally limited to map lookups and vector
// splicing.
// 1. CHANGE: Remove 'const &' and take upper_paths by value so it can accept moved objects
static std::vector<std::vector<int>> expand_path_batch_one_level_local(std::vector<std::vector<int>> upper_paths,
                                                                      const CoarsenedGraph& lower,
                                                                      const CoarsenedGraph& upper,
                                                                      const std::vector<int>& preferred_starts,
                                                                      const std::vector<int>& preferred_goals){
    std::vector<std::vector<int>> lower_paths;
    lower_paths.reserve(upper_paths.size());

    // Reuse a single buffer variable to minimize heap allocations inside the loop
    std::vector<int> path_buffer;
    path_buffer.reserve(128); // Reasonable starting capacity for local segments

    for (std::size_t path_index = 0; path_index < upper_paths.size(); ++path_index)
    {
        const std::vector<int>& upper_path = upper_paths[path_index];
        if (upper_path.empty())
        {
            lower_paths.emplace_back();
            continue;
        }

        path_buffer.clear();
        const int first_parent = upper_path.front();
        int current = -1;

        if (path_index < preferred_starts.size() &&
            is_valid_graph_node_id_local(lower, preferred_starts[path_index]) &&
            preferred_starts[path_index] < static_cast<int>(lower.to_coarser_node_id.size()) &&
            lower.to_coarser_node_id[preferred_starts[path_index]] == first_parent)
        {
            current = preferred_starts[path_index];
        }
        else if (first_parent >= 0 && first_parent < static_cast<int>(upper.chosen_finer_node_id.size()))
        {
            current = upper.chosen_finer_node_id[first_parent];
        }

        if (current < 0)
        {
            lower_paths.emplace_back();
            continue;
        }

        path_buffer.push_back(current);
        bool failed = false;

        for (int step = 0; step + 1 < static_cast<int>(upper_path.size()); ++step)
        {
            const int parent_a = upper_path[step];
            const int parent_b = upper_path[step + 1];

            int target = -1;
            if (step + 1 == static_cast<int>(upper_path.size()) - 1 &&
                path_index < preferred_goals.size() &&
                is_valid_graph_node_id_local(lower, preferred_goals[path_index]) &&
                preferred_goals[path_index] < static_cast<int>(lower.to_coarser_node_id.size()) &&
                lower.to_coarser_node_id[preferred_goals[path_index]] == parent_b)
            {
                target = preferred_goals[path_index];
            }
            else if (parent_b >= 0 && parent_b < static_cast<int>(upper.chosen_finer_node_id.size()))
            {
                target = upper.chosen_finer_node_id[parent_b];
            }

            if (target < 0)
            {
                failed = true;
                break;
            }

            if (current != target)
            {
                const auto cached_path_it = upper.bridge_path_cache.find({parent_a, parent_b});
                if (cached_path_it == upper.bridge_path_cache.end() || cached_path_it->second.path.empty())
                {
                    failed = true;
                    break;
                }

                const std::vector<int>& segment = cached_path_it->second.path;

                // The cached bridge segment connects the two components'
                // chosen representative nodes. On the final (fine-level) lift
                // `current`/`target` are instead the real agent start / task
                // location, which usually differ from those representatives.
                // Splice in a short intra-component hop (cheap: components are
                // tiny) rather than failing the whole path whenever they don't
                // match exactly.
                std::vector<int> lead_in;
                if (segment.front() != current)
                {
                    if (current < 0 || current >= static_cast<int>(lower.to_coarser_node_id.size()) ||
                        lower.to_coarser_node_id[current] != parent_a)
                    {
                        failed = true;
                        break;
                    }
                    lead_in = shortest_path_in_graph_local(lower, current, segment.front(), parent_a, true);
                    if (lead_in.empty())
                    {
                        failed = true;
                        break;
                    }
                }

                std::vector<int> lead_out;
                if (segment.back() != target)
                {
                    if (target < 0 || target >= static_cast<int>(lower.to_coarser_node_id.size()) ||
                        lower.to_coarser_node_id[target] != parent_b)
                    {
                        failed = true;
                        break;
                    }
                    lead_out = shortest_path_in_graph_local(lower, segment.back(), target, parent_b, true);
                    if (lead_out.empty())
                    {
                        failed = true;
                        break;
                    }
                }

                // Protect against an infinitely looping or massive segment in cache
                if (path_buffer.size() + lead_in.size() + segment.size() + lead_out.size() > 5000)
                {
                    failed = true;
                    break;
                }

                if (!lead_in.empty())
                    path_buffer.insert(path_buffer.end(), lead_in.begin() + 1, lead_in.end());

                path_buffer.insert(path_buffer.end(), segment.begin() + 1, segment.end());

                if (!lead_out.empty())
                    path_buffer.insert(path_buffer.end(), lead_out.begin() + 1, lead_out.end());
            }

            current = target;
        }

        if (failed)
        {
            lower_paths.emplace_back();
            continue;
        }

        lower_paths.push_back(path_buffer);
    }

    return lower_paths;
}

// Reconstruct one agent guide from the fine graph and a residual arc-flow map.
// This mirrors `schedule_plan_flow`: start at the agent location, follow a
// positive-flow outgoing arc, decrement that arc's count, and stop when the
// task location is reached.
static std::list<int> reconstruct_guide_from_arc_flow_local(const CoarsenedGraph& fine,
                                                            const std::unordered_map<int, int>& arc_flow_counts,
                                                            int start_loc,
                                                            int task_loc)
{
    std::list<int> guide;
    if (!is_valid_graph_node_id_local(fine, start_loc))
        return guide;

    std::unordered_map<int, int> residual = arc_flow_counts;
    int current = start_loc;
    guide.push_back(current);

    while (current != task_loc)
    {
        if (!is_valid_graph_node_id_local(fine, current))
            break;

        const lemon::ListDigraph::Node current_node = fine.map_nodes[current];
        if (current_node == lemon::INVALID)
            break;

        bool advanced = false;
        for (lemon::ListDigraph::OutArcIt arc(fine.g, current_node); arc != lemon::INVALID; ++arc)
        {
            const int arc_id = fine.g.id(arc);
            auto it = residual.find(arc_id);
            if (it == residual.end() || it->second <= 0)
                continue;

            current = fine.g.id(fine.g.target(arc)) >= 0 ? fine.node_to_maploc[fine.g.id(fine.g.target(arc))] : -1;
            if (current < 0)
                continue;

            it->second--;
            guide.push_back(current);
            advanced = true;
            break;
        }

        if (!advanced)
            break;
    }

    return guide;
}

std::unordered_map<int,int> ReducedHierarchy::compute_reduced_assignment(SharedEnvironment* env,
                                                                        const std::vector<int>& flexible_agent_ids,
                                                                        const std::vector<int>& flexible_task_ids,
                                                                        std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
                                                                        bool need_guide_paths,
                                                                        double* solve_time_out,
                                                                        double* guide_time_out){
    std::unordered_map<int,int> assignments;
    out_agent_guide_paths.clear();

    if (!env)
        return assignments;

    ensure(env);
    if (!ready_)
        return assignments;

    const int top_level_idx = hierarchy_.num_levels() - 1;
    const CoarsenedGraph* top = hierarchy_.level(top_level_idx);
    const CoarsenedGraph* fine = hierarchy_.fine_graph();
    if (!top || !fine)
        return assignments;

    // Step 1: map agents and tasks to the top level and solve one compact
    // min-cost flow on the coarsened graph.
    std::unordered_map<int, int> start_supply;
    std::unordered_map<int, std::list<int>> top_task_ids;
    std::unordered_map<int, int> agent_to_top_node;

    for (int agent_id : flexible_agent_ids)
    {
        const int loc = env->curr_states[agent_id].location;
        const int top_node = map_fine_node_to_level_node_local(hierarchy_, loc, top_level_idx);
        if (top_node < 0)
            continue;
        start_supply[top_node]++;
        agent_to_top_node[agent_id] = top_node;
    }

    for (int task_id : flexible_task_ids)
    {
        const int loc = env->task_pool[task_id].locations[0];
        const int top_node = map_fine_node_to_level_node_local(hierarchy_, loc, top_level_idx);
        if (top_node < 0)
            continue;
        top_task_ids[top_node].push_back(task_id);
    }

    ListDigraph g;
    ListDigraph::NodeMap<int> supply(g);
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();

    std::vector<ListDigraph::Node> top_nodes(top->map_nodes.size(), ListDigraph::Node());
    for (int node_id = 0; node_id < static_cast<int>(top->map_nodes.size()); ++node_id)
    {
        if (!is_valid_graph_node_id_local(*top, node_id))
            continue;
        top_nodes[node_id] = g.addNode();
        supply[top_nodes[node_id]] = 0;
    }

    const int num_workers = static_cast<int>(flexible_agent_ids.size());
    supply[source] = num_workers;
    supply[sink] = -num_workers;

    std::unordered_map<int, ListDigraph::Arc> src_arc_by_top;
    std::unordered_map<int, ListDigraph::Arc> sink_arc_by_top;
    std::unordered_map<int, std::unordered_map<int, ListDigraph::Arc>> move_arc_by_endpoints;

    for (const auto& kv : start_supply)
    {
        const int node_id = kv.first;
        const int count = kv.second;
        if (count <= 0 || node_id < 0 || node_id >= static_cast<int>(top_nodes.size()))
            continue;
        if (top_nodes[node_id] == lemon::INVALID)
            continue;
        const auto arc = g.addArc(source, top_nodes[node_id]);
        capacity[arc] = count;
        cost[arc] = 0.0;
        src_arc_by_top[node_id] = arc;
    }

    for (const auto& kv : top_task_ids)
    {
        const int node_id = kv.first;
        const int count = static_cast<int>(kv.second.size());
        if (count <= 0 || node_id < 0 || node_id >= static_cast<int>(top_nodes.size()))
            continue;
        if (top_nodes[node_id] == lemon::INVALID)
            continue;
        const auto arc = g.addArc(top_nodes[node_id], sink);
        capacity[arc] = count;
        cost[arc] = 0.0;
        sink_arc_by_top[node_id] = arc;
    }

    for (lemon::ListDigraph::ArcIt arc(top->g); arc != lemon::INVALID; ++arc)
    {
        const int src_lid = top->g.id(top->g.source(arc));
        const int dst_lid = top->g.id(top->g.target(arc));
        if (src_lid < 0 || dst_lid < 0 ||
            src_lid >= static_cast<int>(top->node_to_maploc.size()) ||
            dst_lid >= static_cast<int>(top->node_to_maploc.size()))
            continue;

        const int src_id = top->node_to_maploc[src_lid];
        const int dst_id = top->node_to_maploc[dst_lid];
        if (!is_valid_graph_node_id_local(*top, src_id) || !is_valid_graph_node_id_local(*top, dst_id))
            continue;
        if (top_nodes[src_id] == lemon::INVALID || top_nodes[dst_id] == lemon::INVALID)
            continue;

        const auto coarse_arc = g.addArc(top_nodes[src_id], top_nodes[dst_id]);
        capacity[coarse_arc] = num_workers;
        cost[coarse_arc] = top->cost[arc];
        move_arc_by_endpoints[src_id][dst_id] = coarse_arc;
    }

    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    ns.flowMap(flow);

    const auto solve_start = std::chrono::high_resolution_clock::now();
    const int ns_status = ns.run();
    // const auto solve_end = std::chrono::high_resolution_clock::now();
    // if (solve_time_out)
    //     *solve_time_out = std::chrono::duration<double>(solve_end - solve_start).count();
    if (ns_status != NetworkSimplex<ListDigraph>::OPTIMAL)
        return assignments;

    // Step 2: recover one coarse path per agent from the top-level residual
    // flow, exactly as the old code did, but without lifting anything yet.
    std::unordered_map<int, int> src_remaining;
    std::unordered_map<int, int> sink_remaining;
    std::unordered_map<int, std::unordered_map<int, int>> move_remaining;

    for (const auto& kv : src_arc_by_top)
        src_remaining[kv.first] = ns.flow(kv.second);
    for (const auto& kv : sink_arc_by_top)
        sink_remaining[kv.first] = ns.flow(kv.second);
    for (const auto& src_kv : move_arc_by_endpoints)
    {
        for (const auto& dst_kv : src_kv.second)
            move_remaining[src_kv.first][dst_kv.first] = ns.flow(dst_kv.second);
    }

    // 1. Change this tracking vector
    std::vector<int> successful_agent_ids; 
    std::vector<std::vector<int>> coarse_paths;
    std::vector<int> assigned_task_ids;

    coarse_paths.reserve(flexible_agent_ids.size());
    assigned_task_ids.reserve(flexible_agent_ids.size());
    successful_agent_ids.reserve(flexible_agent_ids.size());

    for (int agent_id : flexible_agent_ids)
    {
        const auto agent_it = agent_to_top_node.find(agent_id);
        if (agent_it == agent_to_top_node.end())
            continue;

        const int start_top = agent_it->second;
        if (start_top < 0 || src_remaining[start_top] <= 0)
            continue;

        src_remaining[start_top]--;

        std::queue<int> q;
        std::unordered_map<int, int> prev;
        prev[start_top] = -1;
        q.push(start_top);

        int reached_sink_node = -1;
        while (!q.empty())
        {
            const int u = q.front();
            q.pop();
            if (sink_remaining[u] > 0)
            {
                reached_sink_node = u;
                break;
            }

            if (move_remaining.find(u) == move_remaining.end())
                continue;

            for (const auto& next_kv : move_remaining[u])
            {
                const int v = next_kv.first;
                const int rem = next_kv.second;
                if (rem <= 0)
                    continue;
                if (prev.find(v) != prev.end())
                    continue;
                prev[v] = u;
                q.push(v);
            }
        }

        if (reached_sink_node < 0)
            continue;

        std::vector<int> top_path;
        for (int cur = reached_sink_node; cur != -1; cur = prev[cur])
            top_path.push_back(cur);
        std::reverse(top_path.begin(), top_path.end());

        for (int i = 0; i + 1 < static_cast<int>(top_path.size()); ++i)
        {
            const int u = top_path[i];
            const int v = top_path[i + 1];
            if (move_remaining[u][v] > 0)
                move_remaining[u][v]--;
        }

        sink_remaining[reached_sink_node]--;

        if (top_task_ids[reached_sink_node].empty())
            continue;

        const int task_id = top_task_ids[reached_sink_node].front();
        top_task_ids[reached_sink_node].pop_front();
        
        assignments[agent_id] = task_id;

        successful_agent_ids.push_back(agent_id);
        coarse_paths.push_back(std::move(top_path));
        assigned_task_ids.push_back(task_id);
    }

    if (!need_guide_paths)
    {
        // The caller isn't going to consume guide paths this call (e.g. traffic-aware
        // guiding is disabled, or not active yet this early in the run), so skip the
        // whole-fine-map path lifting in steps 3/4 below entirely: it's the only part
        // of this function whose cost scales with the fine map size and per-agent path
        // length rather than the (tiny) coarse top-level graph, and running it for
        // guide paths nobody reads was both wasted work and, on large/maze-like maps,
        // the actual source of the runaway per-timestep memory growth.
        const auto solve_end = std::chrono::high_resolution_clock::now();
        if (solve_time_out)
            *solve_time_out = std::chrono::duration<double>(solve_end - solve_start).count();
        if (guide_time_out)
            *guide_time_out = 0.0;
        return assignments;
    }

    // Step 3: expand the whole batch of coarse paths level-by-level until the
    // paths live on the fine graph. The last expansion uses the real agent
    // start locations and task locations as endpoint anchors.
    // auto guide_start = std::chrono::high_resolution_clock::now();
     
    std::vector<std::vector<int>> current_paths = std::move(coarse_paths);
    for (int level = top_level_idx; level >= 1; --level)
    {
        const CoarsenedGraph* upper = hierarchy_.level(level);
        const CoarsenedGraph* lower = hierarchy_.level(level - 1);
        if (!upper || !lower)
        {
            current_paths.clear();
            break;
        }

        std::vector<int> preferred_starts;
        std::vector<int> preferred_goals;
        if (level == 1)
        {
            preferred_starts.reserve(current_paths.size());
            preferred_goals.reserve(current_paths.size());
            for (std::size_t i = 0; i < current_paths.size(); ++i)
            {
                const int agent_id = successful_agent_ids[i];
                const int task_id = assigned_task_ids[i];
                preferred_starts.push_back(env->curr_states[agent_id].location);
                preferred_goals.push_back(env->task_pool[task_id].locations[0]);
            }
        }

        current_paths = expand_path_batch_one_level_local(std::move(current_paths),
                                                          *lower,
                                                          *upper,
                                                          preferred_starts,
                                                          preferred_goals);
    }

    // Replace the block "if (current_paths.empty())" with a per-path check:
    for (std::size_t i = 0; i < current_paths.size(); ++i)
    {
        if (current_paths[i].empty())
        {
            const int agent_id = successful_agent_ids[i];
            const int task_id = assigned_task_ids[i];
            const int start_loc = env->curr_states[agent_id].location;
            const int task_loc = env->task_pool[task_id].locations[0];

            current_paths[i] = shortest_path_in_graph_local(*fine, start_loc, task_loc, -1, false);
        }
    }

    //timing
    const auto solve_end = std::chrono::high_resolution_clock::now();
    if (solve_time_out)
        *solve_time_out = std::chrono::duration<double>(solve_end - solve_start).count();
    if (ns_status != NetworkSimplex<ListDigraph>::OPTIMAL)
        return assignments;

    auto guide_start = std::chrono::high_resolution_clock::now();

    // Step 4: each entry in `current_paths` is already a concrete, verified
    // start-to-task path on the fine graph (built directly in Step 3, or via
    // the direct-Dijkstra fallback above) -- just hand it to the caller as
    // the agent's guide. We used to re-derive this by aggregating all paths
    // into a shared arc-flow-count map and "following positive residual
    // flow" node by node (mirroring how `schedule_plan_flow` must recover a
    // path from a raw NetworkSimplex flow, since it never assembles one
    // itself). That reconstruction had no cycle guard or length cap, and
    // since the arc-count map merges flow from every agent's path, a walk
    // could wander across other agents' routes through a large shared flow
    // budget before stalling or reaching the goal -- on large/maze-like maps
    // this produced multi-million-node guide lists and OOM'd the process.
    for (std::size_t i = 0; i < current_paths.size(); ++i)
    {
        if (i >= successful_agent_ids.size() || i >= assigned_task_ids.size())
            break;

        const int agent_id = successful_agent_ids[i];

        std::list<int> guide(current_paths[i].begin(), current_paths[i].end());
        out_agent_guide_paths[agent_id] = std::move(guide);
    }

    // FORCE SYSTEM PURGE OF LOCAL CONTAINERS
    std::vector<std::vector<int>>().swap(current_paths);
    std::vector<int>().swap(successful_agent_ids);
    std::vector<int>().swap(assigned_task_ids);
    std::vector<ListDigraph::Node>().swap(top_nodes);

    if (guide_time_out)
        *guide_time_out = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - guide_start).count();

    return assignments;
}

} // namespace MapReductionTest
