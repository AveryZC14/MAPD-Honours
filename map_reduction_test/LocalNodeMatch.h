// LocalNodeMatch.h
//
// Pluggable "within one coarse node" agent<->task matcher. See
// ai/todo.md ("Solver 6: within-coarse-node agent<->task pairing is
// arbitrary, not distance-informed") for the motivating problem: when an
// agent and a task map to the same top-level coarse node, the per-timestep
// coarse flow solve in compute_reduced_assignment (MapCoarsenV1.cpp) gives
// same-node arcs zero cost, so it has no signal to prefer one fine-grained
// pairing over another -- the recovery walk just pairs them in
// insertion/queue order.
//
// A LocalNodeMatcher takes one such group directly (the agents and tasks
// that already share a coarse node, or any other small group the caller
// wants matched on real distance) and returns up to min(na, nt) pairs.
// Whichever ids don't appear in the result are surplus and should be handed
// to whatever broader mechanism (e.g. the coarse flow graph) would otherwise
// have placed them.

#pragma once

#include "../inc/SharedEnv.h"

#include <functional>
#include <vector>

namespace MapReductionTest {

struct LocalMatchPair
{
    int agent_id;
    int task_id;
};

using LocalNodeMatcher = std::function<std::vector<LocalMatchPair>(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs)>;

// Default/reference matcher: exact minimum-cost bipartite matching (LEMON
// NetworkSimplex) on real fine-grid Manhattan distance between agent_locs[i]
// and task_locs[j]. Optimal for this cost metric. Groups passed to this
// function are expected to be small (one coarse node's worth of agents and
// tasks), so the O(na*nt) complete-bipartite-graph construction is cheap in
// practice -- see the per-node group sizes quantified in ai/todo.md.
std::vector<LocalMatchPair> match_local_node_exact(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs);

} // namespace MapReductionTest
