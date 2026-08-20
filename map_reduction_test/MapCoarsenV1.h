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
#include <memory>
#include <unordered_map>

namespace MapReductionTest {

/**
 * A single level of a (possibly multi-level) coarsened graph.
 *
 * This struct holds a LEMON directed graph representing either the fine map
 * (level 0) or a coarsened level (level > 0). The LEMON NodeMap/ArcMap
 * instances are tied to the owning `ListDigraph` and therefore must live in
 * the same struct instance as the graph.
 */
struct CoarsenedGraph{
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

    // Arc weights, read when copying this level's topology into the
    // per-timestep temporary flow graph built in compute_reduced_assignment.
    // (This struct used to also carry `supply`/`flow` NodeMap/ArcMap fields
    // for the same purpose as the temporary graph's own solve-scoped copies,
    // but nothing ever read them back off the persisted graph -- the
    // per-timestep solve always builds and solves its own fresh ListDigraph
    // -- so they were pure dead weight and have been removed.)
    lemon::ListDigraph::ArcMap<double> cost;
    lemon::ListDigraph::ArcMap<int> capacity;

    // Node lookup helpers
    std::vector<lemon::ListDigraph::Node> map_nodes; //graph ID for each node
    lemon::ListDigraph::Node source;
    lemon::ListDigraph::Node sink;

    // Per-node internal directional arc statistics (kept empty for levels
    // that haven't been populated).
    //
    // `InternalDirectionalArcSamples` stores the raw arc weights bucketed by
    // geometric cardinal direction (Up/Down/Left/Right) for one connected
    // component -- used only transiently while coarsening (to compute the
    // reduced metrics below) and not retained as a per-graph field, since
    // nothing reads it back afterward.
    // `InternalDirectionalArcMetrics` stores the reduced single-value
    // summaries per direction (e.g., average or minimum) for every coarse
    // node, and *is* retained: it's read the next time this graph is itself
    // coarsened, to bias inter-component arc costs by corridor direction.
    // Each entry is optional to represent the absence of internal edges in
    // that direction.
    struct InternalDirectionalArcSamples { std::array<std::vector<double>,4> weights; };
    struct InternalDirectionalArcMetrics { std::array<std::optional<double>,4> weights; };
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

    // Cached representative finer node for each coarse node.
    // This is always chosen from `to_finer_node_ids[node_id]` so later code
    // can use a stable O(1) anchor instead of scanning the child list again.
    std::vector<int> chosen_finer_node_id;

    struct PairHash
    {
        std::size_t operator()(const std::pair<int, int>& p) const noexcept;
    };

    // Cached representative fine bridges for coarse neighbor pairs.
    // Key: (from_parent, to_parent), Value: (fine_u, fine_v).
    using Bridge = std::pair<int, int>;
    std::unordered_map<std::pair<int, int>, Bridge, PairHash> bridge_cache;

    // Cached path between the chosen finer representatives of two coarse
    // neighbors. The path is computed once while building the hierarchy and
    // reused verbatim during lifting, so timesteps only stitch cached pieces.
    struct CachedBridgePath
    {
        Bridge bridge;
        std::vector<int> path;
    };
    std::unordered_map<std::pair<int, int>, CachedBridgePath, PairHash> bridge_path_cache;

    /**
     * Construct an empty level; optionally pre-allocate storage for `fine_map_size`
     * fine-grid locations.
     */
    explicit CoarsenedGraph(int fine_map_size = 0);

        // In MapCoarsenV1.h inside struct CoarsenedGraph:
    ~CoarsenedGraph() {
        // Explicitly clear the graph and clear maps to sever observer bonds
        g.clear(); 
    }
};

/**
 * Owns a stack of coarsened levels.
 *
 * Level 0 is always the finest graph. Each subsequent entry is one call to
 * `Coarsen(...)` applied to the previous level.
 */
struct MultiLevelCoarsenedGraph
{
    std::vector<std::unique_ptr<CoarsenedGraph>> levels;

    // void clear() { levels.clear(); }
    void clear() {
        // This safely drops the unique_ptrs and forces complete destructor chains
        std::vector<std::unique_ptr<CoarsenedGraph>>().swap(levels);
    }
    bool empty() const { return levels.empty(); }
    int num_levels() const { return static_cast<int>(levels.size()); }

    CoarsenedGraph* level(int level_idx);
    const CoarsenedGraph* level(int level_idx) const;

    CoarsenedGraph* fine_graph() { return level(0); }
    const CoarsenedGraph* fine_graph() const { return level(0); }
};

/**
 * Recreate the per-level node storage. This clears the LEMON graph and
 * allocates `fine_map_size` nodes (one per fine-grid location). The source
 * and sink nodes are also created to allow the level to be used directly by
 * a flow solver if desired.
 *
 * `is_fine_level` should be true only when this is level 0 (the finest,
 * uncoarsened level): `to_finer_node_ids` is structurally guaranteed to stay
 * empty at level 0 (it has no finer children to point at), so that vector's
 * allocation is skipped entirely when this is true.
 */
void reserve_fine_map(CoarsenedGraph& graph, int fine_map_size, bool is_fine_level = false);

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
std::unique_ptr<CoarsenedGraph> Coarsen(const CoarsenedGraph& graph);

/**
 * Coarsen the current top level once and append the result.
 *
 * Returns true when a new level was appended.
 */
bool append_coarsened_level(MultiLevelCoarsenedGraph& hierarchy);

/**
 * Build a hierarchy from an environment map.
 *
 * This recreates level 0 as the fine graph, then appends up to
 * `num_additional_levels` coarsened levels.
 */
void build_multilevel_from_environment(MultiLevelCoarsenedGraph& hierarchy,
                                       const SharedEnvironment* env,
                                       int num_additional_levels);

/**
 * Compute a signature over `env`'s grid geometry and cell contents
 * (rows, cols, and every `map` entry). Hierarchy construction reads only
 * these fields (see `build_from_environment`), so two environments with an
 * identical signature always produce the same hierarchy. Used both to
 * decide whether an in-memory hierarchy is still valid for a new `env`, and
 * to validate an on-disk cache file's compatibility with the current map
 * (see `MapCoarsenSerialize.h`).
 */
std::size_t compute_env_signature(const SharedEnvironment* env);

/**
 * Controller that owns a persistent multilevel hierarchy and provides
 * reduced-network operations (top-level simplex + lifting) for schedulers.
 */
class ReducedHierarchy
{
public:
    static ReducedHierarchy& instance();

    // Ensure hierarchy is built for the provided environment (no-op if already valid)
    void ensure(const SharedEnvironment* env);
    bool ready() const;
    double hierarchy_build_time() const;
    std::vector<int> hierarchy_level_node_counts() const;

    // Compute reduced assignment: returns mapping agent_id -> task_id and fills guide paths (fine node ids).
    // Optionally, `solve_time_out` receives the NetworkSimplex solve time in seconds and
    // `guide_time_out` receives the flow-decomposition + path-lifting time in seconds.
    // `need_guide_paths` controls whether the (expensive, whole-fine-map) path lifting in
    // steps 3/4 runs at all -- callers that only need the agent->task assignment (e.g. when
    // traffic-aware guide paths aren't going to be consumed this timestep) should pass false
    // to skip straight to returning `assignments` once the compact top-level flow is solved.
    // `guide_path_length_sum_out`/`guide_path_cost_sum_out`, when steps 3/4 run, receive the
    // sum over every lifted path of (edges) and (sum of that path's fine-graph arc costs)
    // respectively -- 0 if need_guide_paths is false or no agent was successfully matched.
    // `local_match_count_out`/`flow_match_count_out` receive how many agents this call
    // assigned via the within-coarse-node local matcher (Step 1, LocalNodeMatch.h) vs. via
    // the coarse flow solve (Step 2) -- see ai/local_node_matching.md.
    // `local_match_time_out` receives the wall-clock time (seconds) spent inside Step 1's
    // calls to match_local_node_exact() across every same-node agent/task group this call --
    // i.e. just the local-matcher compute, not the bucketing/bookkeeping around it. Distinct
    // from `solve_time_out`, which starts only after Step 1 is entirely done (coarse graph
    // build + NetworkSimplex + Step 2 recovery + Step 3 lift).
    std::unordered_map<int,int> compute_reduced_assignment(SharedEnvironment* env,
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
                                                           double* local_match_time_out = nullptr);

private:
    ReducedHierarchy();
    ~ReducedHierarchy();

    // non-copyable
    ReducedHierarchy(const ReducedHierarchy&) = delete;
    ReducedHierarchy& operator=(const ReducedHierarchy&) = delete;

    MultiLevelCoarsenedGraph hierarchy_;
    bool ready_ = false;
    std::size_t signature_ = 0;
    double last_hierarchy_build_time_ = 0.0;
    std::vector<int> last_hierarchy_level_node_counts_;
};

/**
 * Build a compact string summary of the graph fields and bookkeeping sizes.
 */
std::string summarise_graph(const CoarsenedGraph& graph);

} // namespace MapReductionTest
