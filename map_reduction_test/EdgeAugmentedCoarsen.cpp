#include "EdgeAugmentedCoarsen.h"
#include "LocalNodeMatch.h"

#include <algorithm>
#include <chrono>
#include <queue>
#include <unordered_set>
#include <utility>

#include <lemon/network_simplex.h>

namespace MapReductionTest {

using lemon::INVALID;
using lemon::ListDigraph;
using lemon::NetworkSimplex;

namespace {

// Mirrors MapCoarsenV1.cpp's kDefaultFlowSolveLevel exactly (both solvers
// share the one --flowSolveLevel CLI flag; this is only the fallback used
// when that value is unset/out of range for the hierarchy actually built).
// Duplicated rather than shared because the original is a file-private
// (implicitly-internal-linkage) constexpr in a different translation unit --
// see ai/edge_node_representation.md's "one place this reaches into
// MapCoarsenV1" for the two helper functions duplicated for the same reason.
constexpr int kDefaultFlowSolveLevel = 2;

// Mirrors MapCoarsenV1.cpp's kEnableLocalNodeMatching (same style, same
// default) -- compile-time toggle for the same-top-node distance-informed
// pairing pass below. Flip to false and rebuild to fall back to routing
// every agent/task straight into the edge-augmented flow graph, for an A/B
// comparison. Kept as a separate constant (not shared) for the same
// file-private-linkage reason as kDefaultFlowSolveLevel above.
constexpr bool kEnableLocalNodeMatching = true;

// Duplicated from MapCoarsenV1.cpp (static there, so not linkable from this
// translation unit) -- both are small, stateless, and have never needed a
// bug fix, unlike the lifting helpers shared via
// ReducedHierarchy::lift_coarse_paths_to_fine instead of duplicated. See
// ai/edge_node_representation.md.
bool is_valid_graph_node_id_local(const CoarsenedGraph& graph, int node_id)
{
    return node_id >= 0 && node_id < static_cast<int>(graph.map_nodes.size()) &&
           graph.map_nodes[node_id] != lemon::INVALID;
}

int map_fine_node_to_level_node_local(const MultiLevelCoarsenedGraph& hierarchy, int fine_node_id, int target_level)
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

// (row, col) of a fine-grid location, or (0, 0) if it isn't a real node --
// only ever called with real agent/task locations, so the fallback should
// never actually fire in practice.
std::pair<int,int> fine_cell_rc(const CoarsenedGraph* fine, int loc)
{
    if (!fine || loc < 0 || loc >= static_cast<int>(fine->map_nodes.size()) ||
        fine->map_nodes[loc] == lemon::INVALID)
    {
        return {0, 0};
    }
    return fine->fine_location[fine->map_nodes[loc]];
}

// Manhattan distance from a fine-grid point to the nearest point of an
// axis-aligned bounding box (0 if the point is already inside). See
// ai/edge_node_representation.md's "Cost formula" section for why this
// approximation (rather than an exact per-cell scan) was chosen.
double bbox_manhattan_distance(const std::pair<int,int>& rc, const RegionBBox& box)
{
    const int dr = std::max({0, box.min_row - rc.first, rc.first - box.max_row});
    const int dc = std::max({0, box.min_col - rc.second, rc.second - box.max_col});
    return static_cast<double>(dr + dc);
}

// Bottom-up sweep: level 0 starts every fine cell as its own 1x1 box, then
// each level's to_coarser_node_id parent pointer folds child boxes into the
// parent's running box one level at a time, up to top_level_idx. One pass
// per level, total work bounded by the fine map's cell count -- see
// ai/edge_node_representation.md's "Cost formula" section.
std::vector<RegionBBox> compute_region_bboxes(const MultiLevelCoarsenedGraph& hierarchy, int top_level_idx)
{
    const CoarsenedGraph* fine = hierarchy.fine_graph();
    if (!fine || top_level_idx < 0 || top_level_idx >= hierarchy.num_levels())
        return {};

    std::vector<RegionBBox> running(fine->map_nodes.size());
    std::vector<char> running_valid(fine->map_nodes.size(), false);

    for (int loc = 0; loc < static_cast<int>(fine->map_nodes.size()); ++loc)
    {
        if (loc >= static_cast<int>(fine->to_coarser_node_id.size()) || fine->to_coarser_node_id[loc] < 0)
            continue; // wall / dud cell, never grouped by Coarsen()
        if (fine->map_nodes[loc] == lemon::INVALID)
            continue;

        const auto rc = fine->fine_location[fine->map_nodes[loc]];
        running[loc] = RegionBBox{rc.first, rc.first, rc.second, rc.second};
        running_valid[loc] = true;
    }

    for (int level = 0; level < top_level_idx; ++level)
    {
        const CoarsenedGraph* graph = hierarchy.level(level);
        const CoarsenedGraph* parent = hierarchy.level(level + 1);
        if (!graph || !parent)
            return {};

        std::vector<RegionBBox> next(parent->num_coarse_nodes);
        std::vector<char> next_valid(parent->num_coarse_nodes, false);

        const int node_count = static_cast<int>(graph->to_coarser_node_id.size());
        for (int node_id = 0; node_id < node_count; ++node_id)
        {
            if (node_id >= static_cast<int>(running_valid.size()) || !running_valid[node_id])
                continue;
            const int parent_id = graph->to_coarser_node_id[node_id];
            if (parent_id < 0 || parent_id >= static_cast<int>(next.size()))
                continue;

            const RegionBBox& child_box = running[node_id];
            if (!next_valid[parent_id])
            {
                next[parent_id] = child_box;
                next_valid[parent_id] = true;
            }
            else
            {
                RegionBBox& box = next[parent_id];
                box.min_row = std::min(box.min_row, child_box.min_row);
                box.max_row = std::max(box.max_row, child_box.max_row);
                box.min_col = std::min(box.min_col, child_box.min_col);
                box.max_col = std::max(box.max_col, child_box.max_col);
            }
        }

        running = std::move(next);
        running_valid = std::move(next_valid);
    }

    return running;
}

} // namespace

// Builds the static region+edge backbone described in EdgeAugmentedCoarsen.h
// from an already-built hierarchy level `top`: one region-node per valid
// coarse node in `top.g`, one edge-node per adjacent region pair (not per
// directed arc -- see the loop below), region bounding boxes for the cost
// formula, and adjacency lists tying it together. Three phases:
//   1. Create region-nodes (mirroring top.g's own valid node set).
//   2. Compute every region's fine-cell bounding box.
//   3. Walk every top.g arc once, creating/reusing the arc's edge-node and
//      wiring the two halved-cost backbone arcs through it.
// Called once per (map signature, --flowSolveLevel) by
// EdgeAugmentedHierarchy::ensure, not per timestep -- see "Per-timestep flow
// graph" in ai/edge_node_representation.md for how this static result gets
// reused every timestep via lemon::digraphCopy.
std::unique_ptr<EdgeAugmentedTopGraph> build_edge_augmented_top_graph(
    const MultiLevelCoarsenedGraph& hierarchy, const CoarsenedGraph& top)
{
    auto result = std::make_unique<EdgeAugmentedTopGraph>();
    result->top_level_idx = top.level_idx;

    // Phase 1: one region-node per valid coarse node id in `top`.
    const int num_regions = static_cast<int>(top.map_nodes.size());
    result->region_node.assign(num_regions, ListDigraph::Node());
    result->region_adjacent_edges.assign(num_regions, {});

    for (int region_id = 0; region_id < num_regions; ++region_id)
    {
        if (!is_valid_graph_node_id_local(top, region_id))
            continue;
        result->region_node[region_id] = result->g.addNode();
    }

    // Phase 2: fine-cell bounding box per region, used by the caller
    // (compute_reduced_assignment_edge_augmented) for the proxy-arc cost
    // formula. Defensively padded to `num_regions` in case a region id has
    // no bbox entry (e.g. an isolated/dud region with no fine descendants) --
    // such a region gets an all-zero box, which just makes its proxy-arc
    // costs look like "already inside," not a crash.
    result->region_bbox = compute_region_bboxes(hierarchy, top.level_idx);
    if (static_cast<int>(result->region_bbox.size()) < num_regions)
        result->region_bbox.resize(num_regions, RegionBBox{});

    // capacity: mirrors solver 6's own coarse-arc capacity, which is set to
    // that timestep's total surplus-agent count -- i.e. effectively
    // unconstrained, never meant to model a real bottleneck (see
    // ai/edge_node_representation.md, "Topology"). This backbone is built
    // once and reused across many timesteps via digraphCopy, so it can't
    // bake in one timestep's agent count; a fixed upper bound (total
    // fine-map node count, which no per-timestep surplus-agent count can
    // ever exceed) preserves the same "unconstrained" intent.
    const CoarsenedGraph* fine = hierarchy.fine_graph();
    const int effectively_unconstrained_capacity =
        fine ? std::max(1, static_cast<int>(fine->map_nodes.size())) : 1000000;

    // Phase 3: walk every top.g arc exactly once. Each arc contributes one
    // edge-node (created on first sight of its unordered {src,dst} pair,
    // reused on the second) and exactly two backbone arcs of its own
    // (region->edge, edge->region, each carrying half this arc's cost) --
    // so a bidirectional pair of top.g arcs (A->B and B->A, the normal case
    // on a symmetric grid) ends up sharing one edge-node with all four
    // arcs (A->E, E->B, B->E, E->A) present.
    std::unordered_map<std::pair<int,int>, ListDigraph::Node, CoarsenedGraph::PairHash> edge_node_by_pair;

    for (ListDigraph::ArcIt arc(top.g); arc != lemon::INVALID; ++arc)
    {
        const int src_lid = top.g.id(top.g.source(arc));
        const int dst_lid = top.g.id(top.g.target(arc));
        if (src_lid < 0 || dst_lid < 0 ||
            src_lid >= static_cast<int>(top.node_to_maploc.size()) ||
            dst_lid >= static_cast<int>(top.node_to_maploc.size()))
            continue;

        const int src_id = top.node_to_maploc[src_lid];
        const int dst_id = top.node_to_maploc[dst_lid];
        if (!is_valid_graph_node_id_local(top, src_id) || !is_valid_graph_node_id_local(top, dst_id))
            continue;
        if (result->region_node[src_id] == lemon::INVALID || result->region_node[dst_id] == lemon::INVALID)
            continue;

        const std::pair<int,int> unordered_key{std::min(src_id, dst_id), std::max(src_id, dst_id)};
        auto it = edge_node_by_pair.find(unordered_key);
        ListDigraph::Node edge_node;
        if (it == edge_node_by_pair.end())
        {
            edge_node = result->g.addNode();
            edge_node_by_pair.emplace(unordered_key, edge_node);
            // Register both endpoints' adjacency lists up front, at
            // creation time -- makes each region's adjacency list "every
            // edge-node touching this region" by construction, rather than
            // depending on the reverse-direction top.g arc also existing
            // (true in practice for this grid-derived graph, but this way
            // it doesn't have to be for the adjacency bookkeeping to be
            // correct).
            result->region_adjacent_edges[unordered_key.first].push_back({edge_node, unordered_key.second});
            result->region_adjacent_edges[unordered_key.second].push_back({edge_node, unordered_key.first});
        }
        else
        {
            edge_node = it->second;
        }

        const double half_cost = top.cost[arc] / 2.0;
        const auto forward_arc = result->g.addArc(result->region_node[src_id], edge_node);
        result->cost[forward_arc] = half_cost;
        result->capacity[forward_arc] = effectively_unconstrained_capacity;
        const auto backward_arc = result->g.addArc(edge_node, result->region_node[dst_id]);
        result->cost[backward_arc] = half_cost;
        result->capacity[backward_arc] = effectively_unconstrained_capacity;
    }

    return result;
}

EdgeAugmentedHierarchy& EdgeAugmentedHierarchy::instance()
{
    static EdgeAugmentedHierarchy inst;
    return inst;
}

void EdgeAugmentedHierarchy::ensure(const SharedEnvironment* env, int top_level_idx)
{
    last_backbone_build_time_ = 0.0;
    if (!env)
        return;

    ReducedHierarchy& base = ReducedHierarchy::instance();
    base.ensure(env);
    if (!base.ready())
        return;

    const MultiLevelCoarsenedGraph& hierarchy = base.hierarchy();

    // Same clamping solver 6's own compute_reduced_assignment applies to
    // env->flow_solve_level: fall back to the compile-time default if the
    // requested level is out of range for the hierarchy actually built, then
    // clamp to the topmost level that exists if even the default doesn't fit
    // (e.g. a tiny map whose hierarchy came out shallower than expected).
    // Doing this here (not just trusting the caller) means `backbone_->
    // top_level_idx` is always the level that actually got used, which the
    // caller reads back rather than re-deriving.
    int clamped_level = top_level_idx;
    if (clamped_level < 0 || clamped_level >= hierarchy.num_levels())
        clamped_level = kDefaultFlowSolveLevel;
    if (clamped_level < 0 || clamped_level >= hierarchy.num_levels())
        clamped_level = hierarchy.num_levels() - 1;
    if (clamped_level < 0)
        return;

    const std::size_t sig = compute_env_signature(env);
    if (backbone_ && sig == env_signature_ && backbone_->top_level_idx == clamped_level)
        return; // cached backbone still valid for this env + level

    const auto build_start = std::chrono::high_resolution_clock::now();

    const CoarsenedGraph* top = hierarchy.level(clamped_level);
    if (!top)
    {
        backbone_.reset();
        return;
    }

    backbone_ = build_edge_augmented_top_graph(hierarchy, *top);
    env_signature_ = sig;
    last_backbone_build_time_ = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - build_start).count();
}

bool EdgeAugmentedHierarchy::ready() const
{
    return backbone_ != nullptr;
}

std::unordered_map<int,int> EdgeAugmentedHierarchy::compute_reduced_assignment_edge_augmented(
    SharedEnvironment* env,
    const std::vector<int>& flexible_agent_ids,
    const std::vector<int>& flexible_task_ids,
    std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
    bool need_guide_paths,
    double* solve_time_out,
    double* guide_time_out,
    double* guide_path_length_sum_out,
    double* guide_path_cost_sum_out,
    int* local_match_count_out,
    int* flow_match_count_out,
    double* local_match_time_out,
    double* backbone_build_time_out)
{
    std::unordered_map<int,int> assignments;
    out_agent_guide_paths.clear();
    if (guide_path_length_sum_out) *guide_path_length_sum_out = 0.0;
    if (guide_path_cost_sum_out) *guide_path_cost_sum_out = 0.0;
    if (local_match_count_out) *local_match_count_out = 0;
    if (flow_match_count_out) *flow_match_count_out = 0;
    if (local_match_time_out) *local_match_time_out = 0.0;
    if (backbone_build_time_out) *backbone_build_time_out = 0.0;
    if (solve_time_out) *solve_time_out = 0.0;
    if (guide_time_out) *guide_time_out = 0.0;

    if (!env)
        return assignments;

    ensure(env, env->flow_solve_level);
    if (backbone_build_time_out) *backbone_build_time_out = last_backbone_build_time_;
    if (!ready())
        return assignments;

    const auto solve_start = std::chrono::high_resolution_clock::now();

    const int top_level_idx = backbone_->top_level_idx;
    ReducedHierarchy& base = ReducedHierarchy::instance();
    const MultiLevelCoarsenedGraph& hierarchy = base.hierarchy();
    const CoarsenedGraph* fine = hierarchy.fine_graph();
    if (!fine)
        return assignments;

    // Step 1: bucket agents/tasks by top-level region, then locally match
    // same-node pairs on real distance -- identical to
    // ReducedHierarchy::compute_reduced_assignment's own Step 1
    // (MapCoarsenV1.cpp): it doesn't change for the edge-augmented flow, so
    // it's duplicated verbatim rather than shared (see
    // ai/edge_node_representation.md).
    double local_match_time_accum = 0.0;
    std::unordered_map<int, std::vector<int>> agents_by_node, tasks_by_node;
    std::unordered_map<int, int> agent_loc_by_id, task_loc_by_id;

    for (int agent_id : flexible_agent_ids)
    {
        const int loc = env->curr_states[agent_id].location;
        const int top_node = map_fine_node_to_level_node_local(hierarchy, loc, top_level_idx);
        if (top_node < 0)
            continue;
        agents_by_node[top_node].push_back(agent_id);
        agent_loc_by_id[agent_id] = loc;
    }

    for (int task_id : flexible_task_ids)
    {
        const int loc = env->task_pool[task_id].locations[0];
        const int top_node = map_fine_node_to_level_node_local(hierarchy, loc, top_level_idx);
        if (top_node < 0)
            continue;
        tasks_by_node[top_node].push_back(task_id);
        task_loc_by_id[task_id] = loc;
    }

    std::unordered_map<int, std::vector<int>> surplus_agents_by_node;
    std::unordered_map<int, std::list<int>> top_task_ids;
    std::unordered_map<int, int> agent_to_top_node;

    for (auto& kv : agents_by_node)
    {
        const int node = kv.first;
        const std::vector<int>& node_agent_ids = kv.second;
        const auto tasks_it = tasks_by_node.find(node);
        if (tasks_it == tasks_by_node.end() || tasks_it->second.empty())
        {
            for (int agent_id : node_agent_ids)
            {
                surplus_agents_by_node[node].push_back(agent_id);
                agent_to_top_node[agent_id] = node;
            }
            continue;
        }
        const std::vector<int>& node_task_ids = tasks_it->second;

        std::vector<int> node_agent_locs, node_task_locs;
        node_agent_locs.reserve(node_agent_ids.size());
        node_task_locs.reserve(node_task_ids.size());
        for (int agent_id : node_agent_ids)
            node_agent_locs.push_back(agent_loc_by_id[agent_id]);
        for (int task_id : node_task_ids)
            node_task_locs.push_back(task_loc_by_id[task_id]);

        const auto local_match_start = std::chrono::high_resolution_clock::now();
        const std::vector<LocalMatchPair> pairs = kEnableLocalNodeMatching
            ? match_local_node_exact(*env, node_agent_ids, node_agent_locs, node_task_ids, node_task_locs)
            : std::vector<LocalMatchPair>();
        local_match_time_accum += std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now() - local_match_start).count();

        std::unordered_set<int> matched_agents, matched_tasks;
        for (const auto& p : pairs)
        {
            assignments[p.agent_id] = p.task_id;
            matched_agents.insert(p.agent_id);
            matched_tasks.insert(p.task_id);
        }
        if (local_match_count_out) *local_match_count_out += static_cast<int>(pairs.size());

        for (int agent_id : node_agent_ids)
        {
            if (matched_agents.count(agent_id))
                continue;
            surplus_agents_by_node[node].push_back(agent_id);
            agent_to_top_node[agent_id] = node;
        }
        for (int task_id : node_task_ids)
        {
            if (!matched_tasks.count(task_id))
                top_task_ids[node].push_back(task_id);
        }
    }
    for (const auto& kv : tasks_by_node)
    {
        if (agents_by_node.find(kv.first) != agents_by_node.end())
            continue;
        for (int task_id : kv.second)
            top_task_ids[kv.first].push_back(task_id);
    }
    if (local_match_time_out) *local_match_time_out = local_match_time_accum;

    // Step 1b: copy the persistent region+edge backbone into a fresh
    // per-timestep graph (see ai/edge_node_representation.md, "Per-timestep
    // flow graph" for why a copy rather than mutate-in-place-and-erase), then
    // add source/sink/one proxy node per surplus agent/task. Unlike solver
    // 6, a proxy connects only to the edge-nodes adjacent to its own region
    // -- never to the region node itself.
    ListDigraph g;
    ListDigraph::ArcMap<double> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g);
    ListDigraph::NodeMap<int> supply(g);

    ListDigraph::NodeMap<ListDigraph::Node> node_ref(backbone_->g);
    lemon::digraphCopy(backbone_->g, g)
        .arcMap(backbone_->cost, cost)
        .arcMap(backbone_->capacity, capacity)
        .nodeRef(node_ref)
        .run();

    for (ListDigraph::NodeIt n(g); n != lemon::INVALID; ++n)
        supply[n] = 0;

    // copy-graph node id -> top-level region id, for Step 2's path
    // filtering (absent for edge-nodes/proxies).
    std::unordered_map<int, int> copy_node_to_region_id;
    for (int region_id = 0; region_id < static_cast<int>(backbone_->region_node.size()); ++region_id)
    {
        if (backbone_->region_node[region_id] == lemon::INVALID)
            continue;
        copy_node_to_region_id[g.id(node_ref[backbone_->region_node[region_id]])] = region_id;
    }

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();
    supply[source] = 0;
    supply[sink] = 0;

    std::unordered_map<int, ListDigraph::Node> agent_proxy_node;
    std::unordered_map<int, ListDigraph::Node> task_proxy_node;
    int num_workers = 0;

    // Agent side: source -> agent_proxy -> {adjacent edge-nodes}. Cost on
    // the second arc is the Manhattan distance from the agent's real
    // location to the *neighboring* region's bbox (ai/edge_node_
    // representation.md, "Cost formula") -- i.e. how close this specific
    // agent already is to actually leaving toward that neighbor.
    for (const auto& kv : surplus_agents_by_node)
    {
        const int region_id = kv.first;
        if (region_id < 0 || region_id >= static_cast<int>(backbone_->region_adjacent_edges.size()))
            continue;
        const auto& adjacent = backbone_->region_adjacent_edges[region_id];

        for (int agent_id : kv.second)
        {
            const auto agent_rc = fine_cell_rc(fine, agent_loc_by_id[agent_id]);
            const ListDigraph::Node proxy = g.addNode();
            supply[proxy] = 0;
            agent_proxy_node[agent_id] = proxy;

            const auto src_arc = g.addArc(source, proxy);
            cost[src_arc] = 0.0;
            capacity[src_arc] = 1;

            for (const auto& e : adjacent)
            {
                if (e.neighbor_region_id < 0 || e.neighbor_region_id >= static_cast<int>(backbone_->region_bbox.size()))
                    continue;
                const auto arc = g.addArc(proxy, node_ref[e.edge_node]);
                cost[arc] = bbox_manhattan_distance(agent_rc, backbone_->region_bbox[e.neighbor_region_id]);
                capacity[arc] = 1;
            }

            ++num_workers;
        }
    }

    // Task side: mirror image of the agent loop above -- {adjacent
    // edge-nodes} -> task_proxy -> sink, cost = Manhattan distance from the
    // task's real location to the neighboring region's bbox.
    for (const auto& kv : top_task_ids)
    {
        const int region_id = kv.first;
        if (region_id < 0 || region_id >= static_cast<int>(backbone_->region_adjacent_edges.size()))
            continue;
        const auto& adjacent = backbone_->region_adjacent_edges[region_id];

        for (int task_id : kv.second)
        {
            const auto task_rc = fine_cell_rc(fine, task_loc_by_id[task_id]);
            const ListDigraph::Node proxy = g.addNode();
            supply[proxy] = 0;
            task_proxy_node[task_id] = proxy;

            const auto sink_arc = g.addArc(proxy, sink);
            cost[sink_arc] = 0.0;
            capacity[sink_arc] = 1;

            for (const auto& e : adjacent)
            {
                if (e.neighbor_region_id < 0 || e.neighbor_region_id >= static_cast<int>(backbone_->region_bbox.size()))
                    continue;
                const auto arc = g.addArc(node_ref[e.edge_node], proxy);
                cost[arc] = bbox_manhattan_distance(task_rc, backbone_->region_bbox[e.neighbor_region_id]);
                capacity[arc] = 1;
            }
        }
    }

    supply[source] = num_workers;
    supply[sink] = -num_workers;

    NetworkSimplex<ListDigraph> ns(g);
    ns.costMap(cost);
    ns.upperMap(capacity);
    ns.supplyMap(supply);
    const int ns_status = ns.run();
    if (ns_status != NetworkSimplex<ListDigraph>::OPTIMAL)
        return assignments;
    // flowMap() must be called after run(), not before -- its own
    // documentation says so ("pre: run() must be called before using this
    // function"), unlike costMap()/upperMap()/supplyMap(), which are pre-run
    // setup calls. Calling it before run() (a bug caught here during initial
    // solver 7 testing -- see ai/edge_node_representation.md) silently
    // copies the all-zero pre-solve internal flow state instead of the
    // solved one, and NetworkSimplex still reports OPTIMAL either way since
    // this call doesn't affect the solve itself, so nothing about ns_status
    // would have caught it.
    ns.flowMap(flow);

    // Step 2: recover one path per agent from residual flow, generalized
    // over arbitrary node kinds (agent-proxy -> edge-node -> region-node ->
    // ... -> edge-node -> task-proxy -> sink) since arcs aren't indexed by
    // small dense region ids the way solver 6's move_arc_by_endpoints is.
    std::unordered_map<int, std::vector<std::pair<int,int>>> outgoing; // lemon node id -> [(dst lemon id, remaining flow)]
    for (ListDigraph::ArcIt arc(g); arc != lemon::INVALID; ++arc)
    {
        const int f = flow[arc];
        if (f <= 0)
            continue;
        outgoing[g.id(g.source(arc))].push_back({g.id(g.target(arc)), f});
    }

    std::unordered_map<int,int> task_proxy_lemon_to_task_id;
    for (const auto& kv : task_proxy_node)
        task_proxy_lemon_to_task_id[g.id(kv.second)] = kv.first;

    std::vector<int> successful_agent_ids;
    std::vector<std::vector<int>> coarse_region_paths;
    std::vector<int> assigned_task_ids;
    const int sink_id = g.id(sink);

    for (const auto& kv : agent_proxy_node)
    {
        const int agent_id = kv.first;
        const int start_node_id = g.id(kv.second);

        std::unordered_map<int,int> prev;
        std::queue<int> q;
        prev[start_node_id] = -1;
        q.push(start_node_id);

        int reached = -1;
        while (!q.empty())
        {
            const int u = q.front();
            q.pop();
            if (u == sink_id)
            {
                reached = u;
                break;
            }

            const auto it = outgoing.find(u);
            if (it == outgoing.end())
                continue;
            for (const auto& edge : it->second)
            {
                if (edge.second <= 0 || prev.count(edge.first))
                    continue;
                prev[edge.first] = u;
                q.push(edge.first);
            }
        }

        if (reached < 0)
            continue;

        std::vector<int> full_path;
        for (int cur = reached; cur != -1; cur = prev[cur])
            full_path.push_back(cur);
        std::reverse(full_path.begin(), full_path.end());

        // Decrement remaining flow along the used arcs so a later agent's
        // BFS in this loop doesn't reuse the same unit of flow.
        for (std::size_t i = 0; i + 1 < full_path.size(); ++i)
        {
            auto it = outgoing.find(full_path[i]);
            if (it == outgoing.end())
                continue;
            for (auto& edge : it->second)
            {
                if (edge.first == full_path[i + 1] && edge.second > 0)
                {
                    edge.second--;
                    break;
                }
            }
        }

        // full_path = [agent_proxy, edge_node, region, ..., edge_node, task_proxy, sink].
        // The shortest possible real path is [agent_proxy, task_proxy, sink]
        // (source-side agent_proxy would never connect straight to a
        // task_proxy by construction, so anything shorter than 3 nodes here
        // would mean the BFS reached `sink` some other way -- treat it as
        // not a real match rather than mis-parsing it below).
        if (full_path.size() < 3)
            continue;
        // The task-proxy is always second-to-last (proxy -> sink is its only
        // outgoing arc), so its lemon id identifies which task this path
        // resolved to.
        const auto task_it = task_proxy_lemon_to_task_id.find(full_path[full_path.size() - 2]);
        if (task_it == task_proxy_lemon_to_task_id.end())
            continue;
        const int task_id = task_it->second;

        // Filter the full path down to just its region-node subsequence --
        // dropping the leading agent_proxy, every edge-node, and the
        // trailing task_proxy/sink -- since that's the only vocabulary
        // Steps 3/4 (lift_coarse_paths_to_fine, unchanged from solver 6)
        // understand. See ai/edge_node_representation.md, "Path recovery
        // and lifting".
        std::vector<int> region_path;
        for (int node_id : full_path)
        {
            const auto rit = copy_node_to_region_id.find(node_id);
            if (rit != copy_node_to_region_id.end())
                region_path.push_back(rit->second);
        }

        assignments[agent_id] = task_id;
        if (flow_match_count_out) (*flow_match_count_out)++;

        successful_agent_ids.push_back(agent_id);
        coarse_region_paths.push_back(std::move(region_path));
        assigned_task_ids.push_back(task_id);
    }

    if (!need_guide_paths)
    {
        if (solve_time_out)
            *solve_time_out = std::chrono::duration<double>(
                std::chrono::high_resolution_clock::now() - solve_start).count();
        return assignments;
    }

    // Steps 3-4: lift the batch of region-node-only coarse paths down to
    // concrete fine-graph guide paths via ReducedHierarchy's shared lifting
    // method (see ai/edge_node_representation.md) -- reuses solver 6's own
    // (previously bug-fixed) lifting logic unmodified rather than a second
    // copy of it.
    const double pre_lift_elapsed = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - solve_start).count();

    double expand_time = 0.0;
    double guide_time = 0.0;
    double guide_path_length_sum = 0.0;
    double guide_path_cost_sum = 0.0;
    base.lift_coarse_paths_to_fine(env, top_level_idx, successful_agent_ids, assigned_task_ids,
                                   std::move(coarse_region_paths), out_agent_guide_paths,
                                   &expand_time, &guide_time,
                                   &guide_path_length_sum, &guide_path_cost_sum);

    if (solve_time_out)
        *solve_time_out = pre_lift_elapsed + expand_time;
    if (guide_time_out)
        *guide_time_out = guide_time;
    if (guide_path_length_sum_out)
        *guide_path_length_sum_out = guide_path_length_sum;
    if (guide_path_cost_sum_out)
        *guide_path_cost_sum_out = guide_path_cost_sum;

    return assignments;
}

} // namespace MapReductionTest
