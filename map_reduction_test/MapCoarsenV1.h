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

    // Read-only access to the underlying multi-level hierarchy, for a
    // caller building its own structure on top of it without a second
    // hierarchy build -- e.g. solver 7's edge-node-augmented coarse graph
    // (EdgeAugmentedCoarsen.h), which needs every level from 0 up to
    // whichever level it solves on to compute region bounding boxes. Not
    // meaningful before ready() is true.
    const MultiLevelCoarsenedGraph& hierarchy() const { return hierarchy_; }

    // Compute reduced assignment: returns mapping agent_id -> task_id and fills guide paths (fine node ids).
    // Optionally, `solve_time_out` receives the time in seconds from the end of Step 1's
    // local matching to the end of Step 2 (building the coarse flow graph, NetworkSimplex,
    // walking the flow to recover the assignment), and `guide_time_out` the time of
    // Steps 3-4 (the lift: expand, endpoint check, fallback searches, packaging) -- 0 when
    // need_guide_paths is false.
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
    // build + NetworkSimplex + Step 2 recovery).
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

    // Hierarchical ("cascaded") local matching: same contract and same
    // return value as compute_reduced_assignment() above, but instead of
    // local-matching once at env->flow_solve_level, walks the hierarchy
    // bottom-up from `min_cascade_level`, exact-matching same-node
    // agents/tasks (LocalNodeMatch.h, unchanged) at every level in turn and
    // carrying only each level's leftover surplus up to the next level via
    // that level's to_coarser_node_id -- before finally handing whatever's
    // still unmatched at env->flow_solve_level to compute_reduced_assignment()
    // itself, unmodified, for its usual local-match-then-flow handling.
    // See ai/hierarchical_matching.md for the algorithm and rationale.
    //
    // `min_cascade_level` is clamped to >= 1 (level 0 is the uncoarsened
    // fine map -- matching there would only ever catch agents/tasks already
    // at the exact same fine cell) and to <= env->flow_solve_level. Passing
    // a value >= flow_solve_level makes the cascade a no-op: the leftover
    // set handed to compute_reduced_assignment() is just the original
    // flexible set, unchanged, so this is byte-identical to calling
    // compute_reduced_assignment() directly -- cascading is strictly
    // opt-in. Setting env->flow_solve_level to the hierarchy's own top
    // (fixpoint) level makes the cascade run "in its entirety": the final
    // hand-off's own local match then covers every remaining pairing, and
    // flow becomes a vacuous no-op automatically (see
    // ai/hierarchical_matching.md), no special-casing needed here.
    //
    // `env->cascade_level_stride` (>= 1, default 1) skips the match attempt
    // on all but every Nth level in that climb -- items still climb one
    // level at a time every iteration (to_coarser_node_id only maps one
    // hop), but the bucket-and-match step only runs when
    // (level - min_cascade_level) % stride == 0. Trades many small
    // match_local_node_exact() calls (one per node per level) for fewer,
    // larger ones, since each call pays a real fixed LEMON-graph-
    // construction cost independent of how small its bucket is. stride = 1
    // matches every level, unchanged from before this existed.
    //
    // Cascade-level match counts/time are folded into the *same*
    // local_match_count_out/local_match_time_out out-params
    // compute_reduced_assignment() uses, so existing metrics/dashboards
    // keep meaning "matched without touching the coarse flow graph" without
    // any new output fields. `cascade_time_out`, if provided, receives
    // wall-clock time spent in the cascade loop itself (bucketing +
    // matching across every cascade level), separate from solve_time_out/
    // local_match_time_out, which continue to describe only the final
    // hand-off call exactly as they did before this function existed.
    std::unordered_map<int,int> compute_hierarchical_assignment(SharedEnvironment* env,
                                                                 const std::vector<int>& flexible_agent_ids,
                                                                 const std::vector<int>& flexible_task_ids,
                                                                 int min_cascade_level,
                                                                 std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
                                                                 bool need_guide_paths = true,
                                                                 int cascade_level_stride = 1,
                                                                 double* solve_time_out = nullptr,
                                                                 double* guide_time_out = nullptr,
                                                                 double* guide_path_length_sum_out = nullptr,
                                                                 double* guide_path_cost_sum_out = nullptr,
                                                                 int* local_match_count_out = nullptr,
                                                                 int* flow_match_count_out = nullptr,
                                                                 double* local_match_time_out = nullptr,
                                                                 double* cascade_time_out = nullptr);

    // Why one path's level-by-level lift failed (LiftOutcome::fail_reason).
    enum LiftFailReason
    {
        LIFT_OK = 0,
        LIFT_NO_START,              // no valid finer node to start the level from
        LIFT_NO_TARGET,             // no valid finer node for the next coarse node
        LIFT_MISSING_BRIDGE,        // no cached bridge segment for a coarse pair
        LIFT_LEAD_IN_WRONG_PARENT,  // current node isn't inside the coarse node it should be
        LIFT_LEAD_IN_FAILED,        // lead-in search inside one component found nothing
        LIFT_LEAD_OUT_WRONG_PARENT, // target isn't inside the coarse node it should be
        LIFT_LEAD_OUT_FAILED,       // lead-out search inside one component found nothing
        LIFT_LENGTH_CAP,            // path passed max_path_cells (default 5,000)
    };

    // Per-path diagnostics from lift_coarse_paths_to_fine, only filled when a
    // caller asks for them (ai/hierarchical_guide_paths_plan.md). Collecting
    // them doesn't change any path.
    struct LiftOutcome
    {
        int fail_level = -1;             // level being expanded when the lift failed; -1 = didn't fail
        int fail_reason = LIFT_OK;
        bool endpoints_wrong = false;    // lift finished but didn't run start -> goal
        bool used_fallback = false;      // replaced by the full-map search
        bool fallback_ok = false;
        double fallback_ms = 0.0;
        std::size_t lifted_cells = 0;    // cells in the lifted path before any fallback
    };

    // Every level's expansion is anchored to the nodes containing the real
    // start and goal at the level below (ai/hierarchical_guide_paths_plan.md).
    // `max_path_cells`: a lifted path longer than this fails (LIFT_LENGTH_CAP).
    //
    // Lift an already-decided batch of (agent, task, region-node-only coarse
    // path) triples down to concrete fine-graph guide paths -- exactly what
    // compute_reduced_assignment's own Steps 3/4 do internally for its own
    // coarse-flow-derived paths, extracted here so a second caller with its
    // own Step 1/2 (e.g. an edge-node-augmented coarse flow, see
    // ai/edge_node_representation.md) can reuse the same lifting logic
    // without a second copy of it. `coarse_region_paths[i]` must already be
    // a path of plain region-node ids at `top_level_idx` (no edge-nodes or
    // proxy nodes -- the caller filters those out first) matching
    // `agent_ids[i]`/`task_ids[i]`.
    //
    // `expand_time_out` receives the level-by-level expand + endpoint
    // verification/fallback time (what this file calls "Step 3");
    // `guide_time_out` receives the final packaging-into-out_agent_guide_paths
    // time ("Step 4"). Both callers (solvers 6 and 7) report their sum as
    // guide time; they're kept separate here for finer-grained analysis.
    void lift_coarse_paths_to_fine(SharedEnvironment* env,
                                   int top_level_idx,
                                   const std::vector<int>& agent_ids,
                                   const std::vector<int>& task_ids,
                                   std::vector<std::vector<int>> coarse_region_paths,
                                   std::unordered_map<int,std::list<int>>& out_agent_guide_paths,
                                   double* expand_time_out = nullptr,
                                   double* guide_time_out = nullptr,
                                   double* guide_path_length_sum_out = nullptr,
                                   double* guide_path_cost_sum_out = nullptr,
                                   std::vector<LiftOutcome>* outcomes_out = nullptr,
                                   std::size_t max_path_cells = 5000);

    // --- Planner guide paths from the hierarchy ---
    // (ai/hierarchical_guide_paths_plan.md, "Implementation plan"). All take
    // fine cells and work on one path at a time. Only valid once ready().

    // The level-`level` node containing fine cell `cell`; -1 if none.
    int cell_to_level_node(int cell, int level) const;

    // Size of level `level`'s node-id space (ids run 0 .. this - 1).
    int level_node_id_count(int level) const;

    // For every fine cell, its level-`level` node (-1 for obstacles).
    std::vector<int> level_ancestors(int level) const;

    // Cheapest path on level `level`'s graph (the coarse flow's arc costs)
    // between the nodes containing the two cells. One node if both are in
    // the same node; empty if there's no path or level < 1. A* search.
    // Default heuristic (heuristic_unit < 0): the larger of (grid distance to
    // the goal node) x (the level's cheapest arc cost) and the landmark
    // (ALT) bound; both never overestimate, so the route found is as cheap
    // as Dijkstra's. heuristic_unit >= 0 instead uses (grid distance) x
    // heuristic_unit alone, for experiments (a level-L hop typically costs
    // about 2^L; a large unit may return a costlier route). Uses per-level
    // scratch arrays and landmark tables: one search at a time (not
    // thread-safe).
    std::vector<int> coarse_path(int start_cell, int goal_cell, int level, int* expanded_out = nullptr,
                                 double heuristic_unit = -1.0) const;

    // The cheapest arc cost on level `level` (0 if it has no arcs).
    double min_arc_cost(int level) const;

    // Build coarse_path's landmark tables and scratch arrays for `level` now
    // (otherwise the first search on that level does it). Returns the
    // build time in seconds.
    double prepare_coarse_search(int level) const;

    // Lift `coarse` (a path at `level` from the node containing start_cell
    // to the node containing goal_cell) to the fine map, every level
    // anchored, as lift_coarse_paths_to_fine does. No full-map fallback:
    // returns empty if the lift fails or comes back with wrong endpoints.
    std::vector<int> lift_path(int start_cell, int goal_cell, int level, std::vector<int> coarse,
                               std::size_t max_path_cells, LiftOutcome* outcome_out = nullptr) const;

    // The nodes of `coarse` plus every level-`level` node within `margin`
    // arcs of one of them, each once.
    std::vector<int> corridor_nodes(int level, const std::vector<int>& coarse, int margin) const;

private:
    // Level-by-level expansion shared by lift_coarse_paths_to_fine and
    // lift_path: path i runs from start_cells[i] to goal_cells[i]. Returns
    // one fine path per input (empty where the lift failed); fills
    // fail_level / fail_reason in `outcomes_out` if given (same size).
    std::vector<std::vector<int>> lift_cells(int top_level_idx,
                                             std::vector<std::vector<int>> coarse_paths,
                                             const std::vector<int>& start_cells,
                                             const std::vector<int>& goal_cells,
                                             std::vector<LiftOutcome>* outcomes_out,
                                             std::size_t max_path_cells) const;

    ReducedHierarchy();
    ~ReducedHierarchy();

    // non-copyable
    ReducedHierarchy(const ReducedHierarchy&) = delete;
    ReducedHierarchy& operator=(const ReducedHierarchy&) = delete;

    // Resolve which hierarchy level env->flow_solve_level actually refers
    // to for the hierarchy that was built: falls back to
    // kDefaultFlowSolveLevel, and then to the hierarchy's own top level, if
    // env->flow_solve_level is unset/out of range. Shared by
    // compute_reduced_assignment() and compute_hierarchical_assignment() so
    // both always agree on which level is "the" flow/hand-off level.
    // Only valid to call once ready_ is true.
    int resolve_flow_solve_level(const SharedEnvironment* env) const;

    MultiLevelCoarsenedGraph hierarchy_;

    // coarse_path's per-level search state, reused across searches: a
    // node's g and parent are valid only if its stamp equals `current`.
    struct CoarseSearchScratch
    {
        std::vector<double> g;
        std::vector<int> parent;
        std::vector<uint32_t> stamp;
        uint32_t current = 0;
    };
    mutable std::vector<CoarseSearchScratch> coarse_scratch_;

    // ALT landmarks for coarse_path, per level: to[k * n + v] = cost from
    // landmark k to node v, from[k * n + v] = cost from v to landmark k
    // (infinity if unreachable); n = the level's node-id count.
    struct LandmarkTable
    {
        bool built = false;
        int count = 0;
        std::vector<double> to;
        std::vector<double> from;
    };
    mutable std::vector<LandmarkTable> landmarks_;
    const LandmarkTable& landmark_table(int level) const;
    mutable std::vector<double> level_min_arc_cost_; // -1 = not computed yet

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
