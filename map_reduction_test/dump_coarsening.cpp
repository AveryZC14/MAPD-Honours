// dump_coarsening.cpp
//
// Standalone tool to export the map-coarsening hierarchy
// (MapReductionTest::MultiLevelCoarsenedGraph, see MapCoarsenV1.h/.cpp) for
// visualisation -- see map_reduction_test/visualisation/plot_coarsening.py.
// Companion to dump_guide_paths.cpp, which dumps solver guide paths instead
// of the coarsening structure itself.
//
// Usage: ./dump_coarsening <instance.json> <output_prefix> [num_levels=2]
//
// num_levels is the number of *additional* coarse levels built on top of the
// fine map (same meaning as kDefaultCoarsenLevels in MapCoarsenV1.cpp / the
// num_additional_levels argument to build_multilevel_from_environment) --
// e.g. 2 produces level 0 (fine) + level 1 + level 2, matching the default
// hierarchy solver 6 actually runs with.
//
// Writes three files (nothing for level 0 -- the fine map is just the
// walkable-cell mask the .map file already encodes, so plotting it doesn't
// need a dump):
//
//   <prefix>_partition_level<L>.csv (L = 1..num_levels)
//     Raster grid, one line per map row, comma-separated coarse-node id that
//     each fine cell belongs to at level L (-1 for walls and for cells with
//     no coarse node -- "dud" isolated cells never get grouped into a
//     component, see build_from_environment). This is a raster, not a
//     long-format table, so it loads directly into a 2D array in Python.
//
//   <prefix>_nodes.csv
//     level,node_id,coarse_row,coarse_col,fine_row,fine_col,num_children
//     One row per coarse node at every level 1..num_levels. fine_row/col is
//     that node's representative fine-map location -- chosen_finer_node_id
//     resolved all the way down to level 0 -- used to plot each coarse node
//     at a real position on the map. num_children is how many finer-level
//     nodes were merged into it (its component size one level down, not
//     recursively down to level 0).
//
//   <prefix>_edges.csv
//     level,src_id,dst_id,src_row,src_col,dst_row,dst_col,cost
//     One row per coarse graph arc at every level 1..num_levels, with both
//     endpoints' representative fine-map locations so edges can be drawn
//     directly on the map.

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <lemon/list_graph.h>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "instance_loader.h"

using namespace std;
using namespace MapReductionTest;

namespace {

// Resolve a node id at `level_idx` down to its representative level-0 (fine
// map) location by following chosen_finer_node_id down through the levels.
// Returns -1 if the chain is broken (shouldn't happen for a real node).
int resolve_to_fine_loc(const MultiLevelCoarsenedGraph& hierarchy, int level_idx, int node_id)
{
    while (level_idx > 0)
    {
        const CoarsenedGraph* g = hierarchy.level(level_idx);
        if (g == nullptr || node_id < 0 || node_id >= static_cast<int>(g->chosen_finer_node_id.size()))
            return -1;
        node_id = g->chosen_finer_node_id[node_id];
        --level_idx;
    }
    return node_id;
}

void write_partitions(const MultiLevelCoarsenedGraph& hierarchy, const SharedEnvironment& env,
                       int num_levels, const string& prefix)
{
    const int size = static_cast<int>(env.map.size());
    vector<int> ancestor(size, -1);
    for (int loc = 0; loc < size; ++loc)
        if (env.map[loc] == 0)
            ancestor[loc] = loc; // level-0 graph id == fine loc

    for (int level_idx = 1; level_idx <= num_levels; ++level_idx)
    {
        const CoarsenedGraph* prev = hierarchy.level(level_idx - 1);
        vector<int> next_ancestor(size, -1);
        for (int loc = 0; loc < size; ++loc)
        {
            const int a = ancestor[loc];
            if (a < 0 || a >= static_cast<int>(prev->to_coarser_node_id.size()))
                continue;
            next_ancestor[loc] = prev->to_coarser_node_id[a];
        }
        ancestor = std::move(next_ancestor);

        const string path = prefix + "_partition_level" + std::to_string(level_idx) + ".csv";
        ofstream out(path);
        if (!out.is_open())
        {
            cerr << "Failed to open " << path << " for writing\n";
            continue;
        }
        for (int r = 0; r < env.rows; ++r)
        {
            for (int c = 0; c < env.cols; ++c)
            {
                if (c > 0)
                    out << ",";
                out << ancestor[r * env.cols + c];
            }
            out << "\n";
        }
        cout << "wrote " << path << "\n";
    }
}

void write_nodes(const MultiLevelCoarsenedGraph& hierarchy, const SharedEnvironment& env,
                  int num_levels, const string& prefix)
{
    const string path = prefix + "_nodes.csv";
    ofstream out(path);
    if (!out.is_open())
    {
        cerr << "Failed to open " << path << " for writing\n";
        return;
    }
    out << "level,node_id,coarse_row,coarse_col,fine_row,fine_col,num_children\n";

    for (int level_idx = 1; level_idx <= num_levels; ++level_idx)
    {
        const CoarsenedGraph* g = hierarchy.level(level_idx);
        if (g == nullptr)
            break;
        for (int node_id = 0; node_id < g->num_coarse_nodes; ++node_id)
        {
            if (node_id >= static_cast<int>(g->map_nodes.size()) || g->map_nodes[node_id] == lemon::INVALID)
                continue;
            const auto& crd = g->coarse_location[g->map_nodes[node_id]];
            const int fine_loc = resolve_to_fine_loc(hierarchy, level_idx, node_id);
            const int frow = (fine_loc >= 0 && env.cols > 0) ? fine_loc / env.cols : -1;
            const int fcol = (fine_loc >= 0 && env.cols > 0) ? fine_loc % env.cols : -1;
            const int num_children = (node_id < static_cast<int>(g->to_finer_node_ids.size()))
                                          ? static_cast<int>(g->to_finer_node_ids[node_id].size())
                                          : 0;
            out << level_idx << "," << node_id << "," << crd.first << "," << crd.second << ","
                << frow << "," << fcol << "," << num_children << "\n";
        }
    }
    cout << "wrote " << path << "\n";
}

void write_edges(const MultiLevelCoarsenedGraph& hierarchy, const string& prefix)
{
    const string path = prefix + "_edges.csv";
    ofstream out(path);
    if (!out.is_open())
    {
        cerr << "Failed to open " << path << " for writing\n";
        return;
    }
    out << "level,src_id,dst_id,src_row,src_col,dst_row,dst_col,cost\n";

    for (int level_idx = 1; level_idx < hierarchy.num_levels(); ++level_idx)
    {
        const CoarsenedGraph* g = hierarchy.level(level_idx);
        if (g == nullptr)
            break;
        for (lemon::ListDigraph::ArcIt a(g->g); a != lemon::INVALID; ++a)
        {
            const int src_lid = g->g.id(g->g.source(a));
            const int dst_lid = g->g.id(g->g.target(a));
            if (src_lid < 0 || src_lid >= static_cast<int>(g->node_to_maploc.size()) ||
                dst_lid < 0 || dst_lid >= static_cast<int>(g->node_to_maploc.size()))
                continue;
            const int src_id = g->node_to_maploc[src_lid];
            const int dst_id = g->node_to_maploc[dst_lid];
            if (src_id < 0 || dst_id < 0)
                continue;

            const int src_loc = resolve_to_fine_loc(hierarchy, level_idx, src_id);
            const int dst_loc = resolve_to_fine_loc(hierarchy, level_idx, dst_id);
            const CoarsenedGraph* fine = hierarchy.fine_graph();
            const int cols = fine != nullptr ? fine->coarse_cols : 0;
            const int srow = (src_loc >= 0 && cols > 0) ? src_loc / cols : -1;
            const int scol = (src_loc >= 0 && cols > 0) ? src_loc % cols : -1;
            const int drow = (dst_loc >= 0 && cols > 0) ? dst_loc / cols : -1;
            const int dcol = (dst_loc >= 0 && cols > 0) ? dst_loc % cols : -1;

            out << level_idx << "," << src_id << "," << dst_id << "," << srow << "," << scol << ","
                << drow << "," << dcol << "," << g->cost[a] << "\n";
        }
    }
    cout << "wrote " << path << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cout << "Usage: dump_coarsening <instance.json> <output_prefix> [num_levels=2]\n";
        return 1;
    }

    const string input_json = argv[1];
    const string prefix = argv[2];
    const int num_levels = argc > 3 ? atoi(argv[3]) : 2;

    SharedEnvironment env;
    try
    {
        populate_env_from_instance(input_json, env);
    }
    catch (const std::exception& e)
    {
        cerr << "Error loading instance: " << e.what() << endl;
        return 1;
    }

    cout << "Instance: " << input_json << "  cells=" << env.map.size()
         << " (" << env.rows << "x" << env.cols << ")\n";

    MultiLevelCoarsenedGraph hierarchy;
    build_multilevel_from_environment(hierarchy, &env, num_levels);
    const int levels_built = hierarchy.num_levels() - 1; // additional levels actually built
    cout << "Built " << levels_built << " coarse level(s) on top of the fine map "
         << "(requested " << num_levels << ")\n";
    for (int i = 0; i < hierarchy.num_levels(); ++i)
        cout << "  level " << i << ": " << hierarchy.level(i)->num_coarse_nodes << " nodes\n";

    write_partitions(hierarchy, env, levels_built, prefix);
    write_nodes(hierarchy, env, levels_built, prefix);
    write_edges(hierarchy, prefix);

    cout << "Done.\n";
    return 0;
}
