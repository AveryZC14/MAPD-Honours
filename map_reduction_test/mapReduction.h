// mapReduction.h
// Header for 2x2 map reduction
#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include "SharedEnv.h"

struct ReducedGraphData
{
    int coarse_rows = 0;
    int coarse_cols = 0;
    int num_coarse_nodes = 0;
    std::vector<int> fine_to_coarse;
    std::vector<std::vector<int>> coarse_to_fine;
    std::vector<int> representative_loc;
    std::vector<std::vector<int>> neighbors;
    std::vector<int> coarse_id_to_block_index;
    std::vector<int> block_index_to_coarse_id;
};

// main reduction function
ReducedGraphData reduce_map_2x2(const SharedEnvironment* env);

// helpers
int fine_loc_to_coarse_node(const ReducedGraphData& reduced, int fine_loc);
int coarse_node_representative_loc(const ReducedGraphData& reduced, int coarse_id);
void print_reduced_graph_summary(const ReducedGraphData& reduced);
