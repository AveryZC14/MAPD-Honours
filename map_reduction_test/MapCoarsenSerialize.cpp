#include "MapCoarsenSerialize.h"

#include <lemon/list_graph.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>

namespace MapReductionTest {

namespace {

// 8 bytes, deliberately not NUL-terminated-as-a-C-string: compared with
// memcmp against a fixed-size buffer, not treated as a string anywhere.
constexpr char kMagic[8] = {'M', 'C', 'H', 'I', 'E', 'R', '1', '\n'};
constexpr std::uint32_t kFormatVersion = 1;

template <typename T>
void write_pod(std::ofstream& os, const T& v)
{
    os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool read_pod(std::ifstream& is, T& v)
{
    is.read(reinterpret_cast<char*>(&v), sizeof(T));
    return static_cast<bool>(is);
}

void write_int_vector(std::ofstream& os, const std::vector<int>& v)
{
    write_pod<std::int32_t>(os, static_cast<std::int32_t>(v.size()));
    if (!v.empty())
        os.write(reinterpret_cast<const char*>(v.data()), v.size() * sizeof(int));
}

bool read_int_vector(std::ifstream& is, std::vector<int>& v)
{
    std::int32_t count = 0;
    if (!read_pod(is, count) || count < 0) return false;
    v.resize(static_cast<std::size_t>(count));
    if (count > 0 && !is.read(reinterpret_cast<char*>(v.data()), v.size() * sizeof(int)))
        return false;
    return true;
}

// Writes one level's full state: node coordinates/metrics, arcs (by node
// index, not raw LEMON id -- see the header comment on why that's safe),
// and the two bridge caches. `nodes_at_location`/`node_to_maploc`/
// `maploc_to_node`/`map_nodes` are intentionally NOT written: they're all
// cheap lookup tables that `reserve_fine_map` rebuilds deterministically
// from the node count alone (and `nodes_at_location` specifically is only
// ever read again while *coarsening this level to produce the next one*,
// which has already happened by the time a level is worth caching).
bool write_level(std::ofstream& os, const CoarsenedGraph& g)
{
    write_pod<std::int32_t>(os, g.level_idx);
    write_pod<std::int32_t>(os, g.coarse_rows);
    write_pod<std::int32_t>(os, g.coarse_cols);
    write_pod<std::int32_t>(os, g.num_coarse_nodes);
    write_pod<std::uint8_t>(os, static_cast<std::uint8_t>(g.inter_component_arc_aggregation_policy));

    const std::int32_t num_nodes = static_cast<std::int32_t>(g.map_nodes.size());
    write_pod<std::int32_t>(os, num_nodes);

    for (std::int32_t i = 0; i < num_nodes; ++i)
    {
        const lemon::ListDigraph::Node n = g.map_nodes[i];
        std::pair<int, int> cloc{0, 0};
        std::pair<int, int> floc{0, 0};
        if (n != lemon::INVALID)
        {
            cloc = g.coarse_location[n];
            floc = g.fine_location[n];
        }
        write_pod<std::int32_t>(os, cloc.first);
        write_pod<std::int32_t>(os, cloc.second);
        write_pod<std::int32_t>(os, floc.first);
        write_pod<std::int32_t>(os, floc.second);
        write_pod<std::int32_t>(os, (i < static_cast<std::int32_t>(g.to_coarser_node_id.size()))
                                         ? g.to_coarser_node_id[i] : -1);
        write_pod<std::int32_t>(os, (i < static_cast<std::int32_t>(g.chosen_finer_node_id.size()))
                                         ? g.chosen_finer_node_id[i] : -1);

        static const CoarsenedGraph::InternalDirectionalArcMetrics kEmptyMetrics{};
        const auto& metrics = (i < static_cast<std::int32_t>(g.internal_directional_arc_metrics.size()))
                                   ? g.internal_directional_arc_metrics[i]
                                   : kEmptyMetrics;
        for (int d = 0; d < 4; ++d)
        {
            const auto& opt = metrics.weights[d];
            write_pod<std::uint8_t>(os, opt.has_value() ? 1 : 0);
            if (opt.has_value())
                write_pod<double>(os, *opt);
        }
    }

    // `to_finer_node_ids` is left empty (size 0) at the fine level by
    // construction (see `reserve_fine_map`'s `is_fine_level` parameter) --
    // writing its actual size here (0 for the fine level, num_nodes
    // otherwise) round-trips that distinction without a separate flag.
    const std::int32_t to_finer_count = static_cast<std::int32_t>(g.to_finer_node_ids.size());
    write_pod<std::int32_t>(os, to_finer_count);
    for (std::int32_t i = 0; i < to_finer_count; ++i)
        write_int_vector(os, g.to_finer_node_ids[i]);

    // Arcs, keyed by node index (0..num_nodes-1) rather than raw LEMON arc
    // endpoints -- `node_to_maploc` translates a LEMON node id to that
    // index, giving an identifier that's stable across a save/load cycle
    // (LEMON ids are only stable within one process's `ListDigraph`
    // instance and are meaningless once reloaded into a fresh graph).
    std::int32_t num_arcs = 0;
    for (lemon::ListDigraph::ArcIt a(g.g); a != lemon::INVALID; ++a)
        ++num_arcs;
    write_pod<std::int32_t>(os, num_arcs);
    for (lemon::ListDigraph::ArcIt a(g.g); a != lemon::INVALID; ++a)
    {
        const int u_lid = g.g.id(g.g.source(a));
        const int v_lid = g.g.id(g.g.target(a));
        const int u_id = (u_lid >= 0 && u_lid < static_cast<int>(g.node_to_maploc.size()))
                             ? g.node_to_maploc[u_lid] : -1;
        const int v_id = (v_lid >= 0 && v_lid < static_cast<int>(g.node_to_maploc.size()))
                             ? g.node_to_maploc[v_lid] : -1;
        if (u_id < 0 || v_id < 0)
        {
            // An arc touching a node outside the map-index range (e.g. the
            // unused `source`/`sink` bookkeeping nodes) -- none of the
            // hierarchy-building code ever creates these, but fail loudly
            // rather than silently writing a corrupt/unusable arc.
            return false;
        }
        write_pod<std::int32_t>(os, u_id);
        write_pod<std::int32_t>(os, v_id);
        write_pod<double>(os, g.cost[a]);
        write_pod<std::int32_t>(os, g.capacity[a]);
    }

    write_pod<std::int32_t>(os, static_cast<std::int32_t>(g.bridge_cache.size()));
    for (const auto& kv : g.bridge_cache)
    {
        write_pod<std::int32_t>(os, kv.first.first);
        write_pod<std::int32_t>(os, kv.first.second);
        write_pod<std::int32_t>(os, kv.second.first);
        write_pod<std::int32_t>(os, kv.second.second);
    }

    write_pod<std::int32_t>(os, static_cast<std::int32_t>(g.bridge_path_cache.size()));
    for (const auto& kv : g.bridge_path_cache)
    {
        write_pod<std::int32_t>(os, kv.first.first);
        write_pod<std::int32_t>(os, kv.first.second);
        write_pod<std::int32_t>(os, kv.second.bridge.first);
        write_pod<std::int32_t>(os, kv.second.bridge.second);
        write_int_vector(os, kv.second.path);
    }

    return static_cast<bool>(os);
}

bool read_level(std::ifstream& is, std::unique_ptr<CoarsenedGraph>& out)
{
    std::int32_t level_idx = 0, coarse_rows = 0, coarse_cols = 0, num_coarse_nodes = 0, num_nodes = 0;
    std::uint8_t policy_byte = 0;
    if (!read_pod(is, level_idx) || !read_pod(is, coarse_rows) || !read_pod(is, coarse_cols) ||
        !read_pod(is, num_coarse_nodes) || !read_pod(is, policy_byte) || !read_pod(is, num_nodes))
        return false;
    if (num_nodes < 0) return false;

    auto level = std::make_unique<CoarsenedGraph>();
    level->level_idx = level_idx;
    level->coarse_rows = coarse_rows;
    level->coarse_cols = coarse_cols;
    level->num_coarse_nodes = num_coarse_nodes;
    level->inter_component_arc_aggregation_policy =
        static_cast<CoarsenedGraph::ArcAggregationPolicy>(policy_byte);

    // `reserve_fine_map` reads `coarse_rows`/`coarse_cols` (set above) to
    // size `nodes_at_location`, and (re)creates exactly `num_nodes` LEMON
    // nodes plus `node_to_maploc`/`maploc_to_node`/`map_nodes` -- the same
    // deterministic setup the original build used, just driven by a stored
    // count instead of geometry. `is_fine_level=true` skips allocating
    // `to_finer_node_ids`; the loop below re-allocates it explicitly if the
    // file says this level actually has entries.
    reserve_fine_map(*level, num_nodes, /*is_fine_level=*/true);

    for (std::int32_t i = 0; i < num_nodes; ++i)
    {
        std::int32_t cr = 0, cc = 0, fr = 0, fc = 0, to_coarser = -1, chosen = -1;
        if (!read_pod(is, cr) || !read_pod(is, cc) || !read_pod(is, fr) || !read_pod(is, fc) ||
            !read_pod(is, to_coarser) || !read_pod(is, chosen))
            return false;

        const lemon::ListDigraph::Node n = level->map_nodes[static_cast<std::size_t>(i)];
        level->coarse_location[n] = {cr, cc};
        level->fine_location[n] = {fr, fc};
        level->to_coarser_node_id[static_cast<std::size_t>(i)] = to_coarser;
        level->chosen_finer_node_id[static_cast<std::size_t>(i)] = chosen;

        auto& metrics = level->internal_directional_arc_metrics[static_cast<std::size_t>(i)];
        for (int d = 0; d < 4; ++d)
        {
            std::uint8_t has_value = 0;
            if (!read_pod(is, has_value)) return false;
            if (has_value)
            {
                double v = 0.0;
                if (!read_pod(is, v)) return false;
                metrics.weights[d] = v;
            }
            else
            {
                metrics.weights[d].reset();
            }
        }
    }

    std::int32_t to_finer_count = 0;
    if (!read_pod(is, to_finer_count) || to_finer_count < 0) return false;
    if (to_finer_count > 0)
    {
        level->to_finer_node_ids.assign(static_cast<std::size_t>(to_finer_count), std::vector<int>());
        for (std::int32_t i = 0; i < to_finer_count; ++i)
        {
            if (!read_int_vector(is, level->to_finer_node_ids[static_cast<std::size_t>(i)]))
                return false;
        }
    }

    std::int32_t num_arcs = 0;
    if (!read_pod(is, num_arcs) || num_arcs < 0) return false;
    for (std::int32_t i = 0; i < num_arcs; ++i)
    {
        std::int32_t u_id = -1, v_id = -1, capacity = 0;
        double cost = 0.0;
        if (!read_pod(is, u_id) || !read_pod(is, v_id) || !read_pod(is, cost) || !read_pod(is, capacity))
            return false;
        if (u_id < 0 || u_id >= num_nodes || v_id < 0 || v_id >= num_nodes)
            return false;

        const lemon::ListDigraph::Arc a = level->g.addArc(
            level->map_nodes[static_cast<std::size_t>(u_id)],
            level->map_nodes[static_cast<std::size_t>(v_id)]);
        level->cost[a] = cost;
        level->capacity[a] = capacity;
    }

    std::int32_t bridge_cache_count = 0;
    if (!read_pod(is, bridge_cache_count) || bridge_cache_count < 0) return false;
    for (std::int32_t i = 0; i < bridge_cache_count; ++i)
    {
        std::int32_t k1 = 0, k2 = 0, v1 = 0, v2 = 0;
        if (!read_pod(is, k1) || !read_pod(is, k2) || !read_pod(is, v1) || !read_pod(is, v2))
            return false;
        level->bridge_cache[{k1, k2}] = {v1, v2};
    }

    std::int32_t bridge_path_cache_count = 0;
    if (!read_pod(is, bridge_path_cache_count) || bridge_path_cache_count < 0) return false;
    for (std::int32_t i = 0; i < bridge_path_cache_count; ++i)
    {
        std::int32_t k1 = 0, k2 = 0, b1 = 0, b2 = 0;
        if (!read_pod(is, k1) || !read_pod(is, k2) || !read_pod(is, b1) || !read_pod(is, b2))
            return false;
        CoarsenedGraph::CachedBridgePath cbp;
        cbp.bridge = {b1, b2};
        if (!read_int_vector(is, cbp.path))
            return false;
        level->bridge_path_cache[{k1, k2}] = std::move(cbp);
    }

    out = std::move(level);
    return true;
}

} // namespace

bool save_hierarchy_to_file(const MultiLevelCoarsenedGraph& hierarchy,
                            const SharedEnvironment* env,
                            const std::string& path)
{
    if (env == nullptr || hierarchy.empty() || path.empty()) return false;

    std::ofstream os(path, std::ios::binary | std::ios::trunc);
    if (!os)
    {
        std::cerr << "[MapCoarsenSerialize] failed to open '" << path << "' for writing\n";
        return false;
    }

    os.write(kMagic, sizeof(kMagic));
    write_pod<std::uint32_t>(os, kFormatVersion);
    write_pod<std::int32_t>(os, env->rows);
    write_pod<std::int32_t>(os, env->cols);
    write_pod<std::uint64_t>(os, static_cast<std::uint64_t>(compute_env_signature(env)));
    write_pod<std::int32_t>(os, hierarchy.num_levels());

    for (const auto& level : hierarchy.levels)
    {
        if (!level || !write_level(os, *level))
        {
            std::cerr << "[MapCoarsenSerialize] failed writing hierarchy to '" << path << "'\n";
            return false;
        }
    }

    os.flush();
    if (!os)
    {
        std::cerr << "[MapCoarsenSerialize] I/O error writing hierarchy to '" << path << "'\n";
        return false;
    }
    return true;
}

bool load_hierarchy_from_file(const std::string& path,
                              const SharedEnvironment* env,
                              int expected_num_levels,
                              MultiLevelCoarsenedGraph& out)
{
    if (env == nullptr || path.empty()) return false;

    std::ifstream is(path, std::ios::binary);
    if (!is) return false; // no cache file yet -- not an error, just a cache miss

    char magic[sizeof(kMagic)];
    if (!is.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0)
        return false;

    std::uint32_t version = 0;
    if (!read_pod(is, version) || version != kFormatVersion) return false;

    std::int32_t rows = 0, cols = 0, num_levels = 0;
    std::uint64_t sig = 0;
    if (!read_pod(is, rows) || !read_pod(is, cols) || !read_pod(is, sig) || !read_pod(is, num_levels))
        return false;

    if (rows != env->rows || cols != env->cols) return false;
    if (sig != static_cast<std::uint64_t>(compute_env_signature(env))) return false;
    // A cache with at least as many levels as requested is usable as-is
    // (extra levels are simply unused); only a cache with fewer levels than
    // requested is rejected, falling through to a full rebuild.
    if (expected_num_levels >= 0 && num_levels < expected_num_levels) return false;
    if (num_levels < 0) return false;

    MultiLevelCoarsenedGraph loaded;
    loaded.levels.reserve(static_cast<std::size_t>(num_levels));
    for (std::int32_t i = 0; i < num_levels; ++i)
    {
        std::unique_ptr<CoarsenedGraph> level;
        if (!read_level(is, level))
        {
            std::cerr << "[MapCoarsenSerialize] failed reading level " << i
                      << " from '" << path << "' -- ignoring cache\n";
            return false;
        }
        loaded.levels.push_back(std::move(level));
    }

    out = std::move(loaded);
    return true;
}

} // namespace MapReductionTest
