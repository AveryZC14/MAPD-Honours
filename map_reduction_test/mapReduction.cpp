// mapReduction.cpp
//
// Proof-of-concept 2x2 map reduction for a grid map.
// This file reduces the original fine grid into a coarse graph where
// each 2x2 block is represented by a single coarse vertex.
//
// Assumptions:
// - env->map is a flattened row-major grid
// - env->map[loc] == 0 means free, anything else means blocked
// - env->cols exists
//
// This code does NOT try to reconstruct fine paths.
// It only builds the reduced graph and the mappings needed to use it.

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include "mapReduction.h"

namespace
{
    inline int get_row(int loc, int cols)
    {
        return loc / cols;
    }

    inline int get_col(int loc, int cols)
    {
        return loc % cols;
    }

    inline int to_loc(int r, int c, int cols)
    {
        return r * cols + c;
    }

    inline bool in_bounds(int r, int c, int rows, int cols)
    {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }

    inline int block_index(int br, int bc, int coarse_cols)
    {
        return br * coarse_cols + bc;
    }

    // Returns true if there exists at least one valid fine-grid connection
    // across the boundary between two neighboring 2x2 blocks.
    //
    // This is intentionally permissive and simple:
    // - for horizontal neighboring coarse blocks, check if any row in the overlap
    //   has a free cell on the touching columns
    // - for vertical neighboring coarse blocks, check if any column in the overlap
    //   has a free cell on the touching rows
    //
    // This is only a proof-of-concept connectivity rule.
    bool blocks_are_adjacent(
        int br1, int bc1,
        int br2, int bc2,
        const std::vector<int>& map,
        int rows,
        int cols)
    {
        // Only supports cardinal neighboring coarse blocks
        int dr = br2 - br1;
        int dc = bc2 - bc1;

        // Horizontal neighbors
        if (dr == 0 && std::abs(dc) == 1)
        {
            int left_bc  = std::min(bc1, bc2);
            int right_bc = std::max(bc1, bc2);

            int left_col  = left_bc * 2 + 1;   // right edge of left block
            int right_col = right_bc * 2;      // left edge of right block

            int base_row = br1 * 2;

            for (int offset = 0; offset < 2; ++offset)
            {
                int r = base_row + offset;
                if (!in_bounds(r, left_col, rows, cols) || !in_bounds(r, right_col, rows, cols))
                    continue;

                int loc_left = to_loc(r, left_col, cols);
                int loc_right = to_loc(r, right_col, cols);

                if (map[loc_left] == 0 && map[loc_right] == 0)
                    return true;
            }
            return false;
        }

        // Vertical neighbors
        if (dc == 0 && std::abs(dr) == 1)
        {
            int top_br    = std::min(br1, br2);
            int bottom_br = std::max(br1, br2);

            int top_row    = top_br * 2 + 1;   // bottom edge of top block
            int bottom_row = bottom_br * 2;    // top edge of bottom block

            int base_col = bc1 * 2;

            for (int offset = 0; offset < 2; ++offset)
            {
                int c = base_col + offset;
                if (!in_bounds(top_row, c, rows, cols) || !in_bounds(bottom_row, c, rows, cols))
                    continue;

                int loc_top = to_loc(top_row, c, cols);
                int loc_bottom = to_loc(bottom_row, c, cols);

                if (map[loc_top] == 0 && map[loc_bottom] == 0)
                    return true;
            }
            return false;
        }

        return false;
    }

    void add_directed_edge(std::vector<std::vector<int>>& neighbors, int u, int v)
    {
        if (u < 0 || v < 0 || u >= static_cast<int>(neighbors.size()) || v >= static_cast<int>(neighbors.size()))
            return;

        neighbors[u].push_back(v);
    }

    void deduplicate_neighbors(std::vector<std::vector<int>>& neighbors)
    {
        for (auto& adj : neighbors)
        {
            std::sort(adj.begin(), adj.end());
            adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
        }
    }
}

// Main reduction function.
// This collapses each 2x2 block into one coarse node if that block contains
// at least one free fine-grid cell.
ReducedGraphData reduce_map_2x2(const SharedEnvironment* env)
{
    ReducedGraphData reduced;

    if (env == nullptr)
    {
        std::cerr << "[reduce_map_2x2] env is null\n";
        return reduced;
    }

    const int cols = env->cols;
    if (cols <= 0)
    {
        std::cerr << "[reduce_map_2x2] env->cols is invalid\n";
        return reduced;
    }

    const int num_locs = static_cast<int>(env->map.size());
    if (num_locs == 0)
    {
        std::cerr << "[reduce_map_2x2] env->map is empty\n";
        return reduced;
    }

    const int rows = num_locs / cols;
    if (rows * cols != num_locs)
    {
        std::cerr << "[reduce_map_2x2] map size is not divisible by cols\n";
        return reduced;
    }

    reduced.coarse_rows = (rows + 1) / 2;
    reduced.coarse_cols = (cols + 1) / 2;

    reduced.fine_to_coarse.assign(num_locs, -1);
    reduced.block_index_to_coarse_id.assign(reduced.coarse_rows * reduced.coarse_cols, -1);

    // -------------------------------------------------------------------------
    // Step 1: create one coarse node per non-empty 2x2 block
    // -------------------------------------------------------------------------
    for (int br = 0; br < reduced.coarse_rows; ++br)
    {
        for (int bc = 0; bc < reduced.coarse_cols; ++bc)
        {
            std::vector<int> fine_cells_in_block;
            fine_cells_in_block.reserve(4);

            int start_r = br * 2;
            int start_c = bc * 2;

            for (int dr = 0; dr < 2; ++dr)
            {
                for (int dc = 0; dc < 2; ++dc)
                {
                    int r = start_r + dr;
                    int c = start_c + dc;

                    if (!in_bounds(r, c, rows, cols))
                        continue;

                    int loc = to_loc(r, c, cols);
                    if (env->map[loc] == 0)
                    {
                        fine_cells_in_block.push_back(loc);
                    }
                }
            }

            // If no free cells in the 2x2 block, skip creating a coarse node
            if (fine_cells_in_block.empty())
                continue;

            int coarse_id = reduced.num_coarse_nodes++;
            int b_index = block_index(br, bc, reduced.coarse_cols);

            reduced.block_index_to_coarse_id[b_index] = coarse_id;
            reduced.coarse_id_to_block_index.push_back(b_index);
            reduced.coarse_to_fine.push_back(fine_cells_in_block);

            // Pick a simple representative: first free fine cell in the block
            reduced.representative_loc.push_back(fine_cells_in_block.front());

            for (int loc : fine_cells_in_block)
            {
                reduced.fine_to_coarse[loc] = coarse_id;
            }
        }
    }

    reduced.neighbors.assign(reduced.num_coarse_nodes, {});

    // -------------------------------------------------------------------------
    // Step 2: build reduced adjacency
    //
    // We only check right/down and add both directions to avoid duplicate logic.
    // -------------------------------------------------------------------------
    for (int br = 0; br < reduced.coarse_rows; ++br)
    {
        for (int bc = 0; bc < reduced.coarse_cols; ++bc)
        {
            int b_index = block_index(br, bc, reduced.coarse_cols);
            int u = reduced.block_index_to_coarse_id[b_index];
            if (u == -1)
                continue;

            // Right neighbor
            if (bc + 1 < reduced.coarse_cols)
            {
                int right_index = block_index(br, bc + 1, reduced.coarse_cols);
                int v = reduced.block_index_to_coarse_id[right_index];

                if (v != -1 && blocks_are_adjacent(br, bc, br, bc + 1, env->map, rows, cols))
                {
                    add_directed_edge(reduced.neighbors, u, v);
                    add_directed_edge(reduced.neighbors, v, u);
                }
            }

            // Down neighbor
            if (br + 1 < reduced.coarse_rows)
            {
                int down_index = block_index(br + 1, bc, reduced.coarse_cols);
                int v = reduced.block_index_to_coarse_id[down_index];

                if (v != -1 && blocks_are_adjacent(br, bc, br + 1, bc, env->map, rows, cols))
                {
                    add_directed_edge(reduced.neighbors, u, v);
                    add_directed_edge(reduced.neighbors, v, u);
                }
            }
        }
    }

    deduplicate_neighbors(reduced.neighbors);

    return reduced;
}

// Convenience helper:
// returns the coarse node containing a fine location, or -1 if blocked/invalid.
int fine_loc_to_coarse_node(const ReducedGraphData& reduced, int fine_loc)
{
    if (fine_loc < 0 || fine_loc >= static_cast<int>(reduced.fine_to_coarse.size()))
        return -1;
    return reduced.fine_to_coarse[fine_loc];
}

// Convenience helper:
// returns a representative fine location for a coarse node.
int coarse_node_representative_loc(const ReducedGraphData& reduced, int coarse_id)
{
    if (coarse_id < 0 || coarse_id >= static_cast<int>(reduced.representative_loc.size()))
        return -1;
    return reduced.representative_loc[coarse_id];
}

// Optional debugging utility.
void print_reduced_graph_summary(const ReducedGraphData& reduced)
{
    std::cout << "Reduced graph summary:\n";
    std::cout << "  coarse_rows: " << reduced.coarse_rows << "\n";
    std::cout << "  coarse_cols: " << reduced.coarse_cols << "\n";
    std::cout << "  num_coarse_nodes: " << reduced.num_coarse_nodes << "\n";

    for (int u = 0; u < reduced.num_coarse_nodes; ++u)
    {
        std::cout << "  coarse node " << u
                  << " rep=" << reduced.representative_loc[u]
                  << " fine_cells={";

        for (size_t i = 0; i < reduced.coarse_to_fine[u].size(); ++i)
        {
            std::cout << reduced.coarse_to_fine[u][i];
            if (i + 1 < reduced.coarse_to_fine[u].size())
                std::cout << ",";
        }
        std::cout << "} neighbors={";

        for (size_t i = 0; i < reduced.neighbors[u].size(); ++i)
        {
            std::cout << reduced.neighbors[u][i];
            if (i + 1 < reduced.neighbors[u].size())
                std::cout << ",";
        }
        std::cout << "}\n";
    }
}