// LocalNodeMatch.cpp
// See LocalNodeMatch.h.

#include "LocalNodeMatch.h"

#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>

#include <algorithm>
#include <cstdlib>

namespace MapReductionTest {

namespace {

long long manhattan_distance(const SharedEnvironment& env, int loc_a, int loc_b)
{
    const int ra = loc_a / env.cols, ca = loc_a % env.cols;
    const int rb = loc_b / env.cols, cb = loc_b % env.cols;
    return std::abs(ra - rb) + std::abs(ca - cb);
}

} // namespace

std::vector<LocalMatchPair> match_local_node_exact(
    const SharedEnvironment& env,
    const std::vector<int>& agent_ids, const std::vector<int>& agent_locs,
    const std::vector<int>& task_ids, const std::vector<int>& task_locs)
{
    using lemon::ListDigraph;
    using lemon::NetworkSimplex;

    std::vector<LocalMatchPair> result;

    const std::size_t na = agent_ids.size();
    const std::size_t nt = task_ids.size();
    if (na != agent_locs.size() || nt != task_locs.size())
        return result;
    const std::size_t k = std::min(na, nt);
    if (k == 0)
        return result;

    ListDigraph g;
    ListDigraph::ArcMap<long long> cost(g);
    ListDigraph::ArcMap<int> capacity(g);
    ListDigraph::ArcMap<int> flow(g);
    ListDigraph::NodeMap<int> supply(g);

    const ListDigraph::Node source = g.addNode();
    const ListDigraph::Node sink = g.addNode();
    supply[source] = static_cast<int>(k);
    supply[sink] = -static_cast<int>(k);

    std::vector<ListDigraph::Node> anodes(na), tnodes(nt);
    for (std::size_t i = 0; i < na; ++i)
    {
        anodes[i] = g.addNode();
        supply[anodes[i]] = 0;
        const auto arc = g.addArc(source, anodes[i]);
        capacity[arc] = 1;
        cost[arc] = 0;
    }
    for (std::size_t j = 0; j < nt; ++j)
    {
        tnodes[j] = g.addNode();
        supply[tnodes[j]] = 0;
        const auto arc = g.addArc(tnodes[j], sink);
        capacity[arc] = 1;
        cost[arc] = 0;
    }

    std::vector<std::vector<ListDigraph::Arc>> match_arc(na, std::vector<ListDigraph::Arc>(nt));
    for (std::size_t i = 0; i < na; ++i)
        for (std::size_t j = 0; j < nt; ++j)
        {
            const auto arc = g.addArc(anodes[i], tnodes[j]);
            capacity[arc] = 1;
            cost[arc] = manhattan_distance(env, agent_locs[i], task_locs[j]);
            match_arc[i][j] = arc;
        }

    NetworkSimplex<ListDigraph, int, long long> ns(g);
    ns.costMap(cost).upperMap(capacity).supplyMap(supply).flowMap(flow);
    if (ns.run() != NetworkSimplex<ListDigraph, int, long long>::OPTIMAL)
        return result;

    // Read flow via ns.flow(arc) (the solver's own accessor), not by
    // indexing the external ArcMap passed to flowMap() -- the latter is not
    // reliably populated post-solve in this LEMON version, matching the
    // pattern already used elsewhere in this codebase (e.g.
    // run_top_level_flow_and_recover / compute_reduced_assignment, which
    // read via ns.flow(arc) rather than an external flow map).
    result.reserve(k);
    for (std::size_t i = 0; i < na; ++i)
        for (std::size_t j = 0; j < nt; ++j)
            if (ns.flow(match_arc[i][j]) > 0)
                result.push_back({agent_ids[i], task_ids[j]});

    return result;
}

} // namespace MapReductionTest
