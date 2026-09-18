// validate_hierarchical_matching.cpp
//
// Standalone, timing-independent rigor check for
// MapReductionTest::ReducedHierarchy::compute_hierarchical_assignment()
// (ai/hierarchical_matching.md). Full `./build/lifelong` runs on this
// machine are not reliable for A/B'ing scheduler logic: per-timestep
// planner behavior is gated by wall-clock (`--planTimeLimit`), and this
// sandbox's timestep cost is close enough to that budget that two runs of
// the *identical* command with no code change at all produce different
// `numTaskFinished` -- see ai/hierarchical_matching.md's validation notes.
// This tool instead calls the matching functions directly, once, on a
// frozen agent/task batch (no simulation loop, no timing dependency), so
// results are exactly reproducible.
//
// Checks:
//   1. Disabled cascade == direct call. min_cascade_level >= flow_solve_level
//      must produce a byte-identical assignment map to calling
//      compute_reduced_assignment() directly -- this is the "cascading is
//      strictly opt-in" contract from ai/hierarchical_matching.md.
//   2. A real cascade (min_cascade_level 1, flow_solve_level = the CLI arg)
//      produces a valid assignment: no agent or task appears more than
//      once, every id in the result was actually in the input flexible
//      sets, and local_match_count + flow_match_count == number of pairs
//      returned. Also reports total real (Manhattan) distance vs. the
//      single-level baseline for manual inspection -- not a hard pass/fail,
//      since ai/local_node_matching.md already found this kind of change
//      isn't strictly monotonic on total realized distance.
//   3. "Entirety" mode (flow_solve_level = the hierarchy's own top level):
//      same validity checks, plus flow_match_count_out must be 0 -- the
//      claim from ai/hierarchical_matching.md that flow becomes a vacuous
//      no-op once the cascade's hand-off level is the true top.
//
// Usage: ./hierarchical_matching_validator <instance.json> [flowSolveLevel=4]

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <cmath>
#include <cstdlib>

#include "SharedEnv.h"
#include "instance_loader.h"
#include "MapCoarsenV1.h"

using namespace std;
using namespace MapReductionTest;

namespace {

int g_checks_run = 0;
int g_checks_failed = 0;

void check(bool cond, const string& msg)
{
    ++g_checks_run;
    if (!cond)
    {
        ++g_checks_failed;
        cout << "  [FAIL] " << msg << "\n";
    }
}

int manhattan(const SharedEnvironment& env, int loc_a, int loc_b)
{
    const int ra = loc_a / env.cols, ca = loc_a % env.cols;
    const int rb = loc_b / env.cols, cb = loc_b % env.cols;
    return std::abs(ra - rb) + std::abs(ca - cb);
}

long long total_distance(const SharedEnvironment& env,
                          const std::unordered_map<int,int>& assignments,
                          const std::unordered_map<int,int>& agent_loc,
                          const std::unordered_map<int,int>& task_loc)
{
    long long total = 0;
    for (const auto& kv : assignments)
        total += manhattan(env, agent_loc.at(kv.first), task_loc.at(kv.second));
    return total;
}

// Checks that hold for any valid assignment map regardless of which
// function/parameters produced it: no agent or task id appears twice, and
// every id in the result actually came from the flexible sets fed in.
void check_assignment_validity(const string& label,
                                const std::unordered_map<int,int>& assignments,
                                const vector<int>& flexible_agent_ids,
                                const vector<int>& flexible_task_ids)
{
    const std::unordered_set<int> valid_agents(flexible_agent_ids.begin(), flexible_agent_ids.end());
    const std::unordered_set<int> valid_tasks(flexible_task_ids.begin(), flexible_task_ids.end());
    std::unordered_set<int> seen_tasks;
    bool all_agents_valid = true, all_tasks_valid = true, no_duplicate_tasks = true;
    for (const auto& kv : assignments)
    {
        if (!valid_agents.count(kv.first)) all_agents_valid = false;
        if (!valid_tasks.count(kv.second)) all_tasks_valid = false;
        if (!seen_tasks.insert(kv.second).second) no_duplicate_tasks = false;
    }
    check(all_agents_valid, label + ": every assigned agent id came from flexible_agent_ids");
    check(all_tasks_valid, label + ": every assigned task id came from flexible_task_ids");
    check(no_duplicate_tasks, label + ": no task assigned to more than one agent");
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout << "Usage: hierarchical_matching_validator <instance.json> [flowSolveLevel=4]\n";
        return 1;
    }
    const string input_json = argv[1];
    const int flow_solve_level_arg = argc > 2 ? atoi(argv[2]) : 4;

    SharedEnvironment env;
    try
    {
        populate_env_from_instance(input_json, env);
    }
    catch (const std::exception& e)
    {
        cerr << "failed to load instance: " << e.what() << "\n";
        return 2;
    }

    vector<int> flexible_agent_ids;
    flexible_agent_ids.reserve(env.num_of_agents);
    for (int a = 0; a < env.num_of_agents; ++a) flexible_agent_ids.push_back(a);

    vector<int> flexible_task_ids;
    flexible_task_ids.reserve(env.task_pool.size());
    for (const auto& kv : env.task_pool) flexible_task_ids.push_back(kv.first);

    std::unordered_map<int,int> agent_loc, task_loc;
    for (int a : flexible_agent_ids) agent_loc[a] = env.curr_states[a].location;
    for (int t : flexible_task_ids) task_loc[t] = env.task_pool.at(t).locations[0];

    auto& hierarchy = ReducedHierarchy::instance();
    hierarchy.ensure(&env);
    check(hierarchy.ready(), "hierarchy built successfully");
    const int num_levels = hierarchy.hierarchy().num_levels();
    cout << "Instance: " << input_json << "  agents=" << flexible_agent_ids.size()
         << " tasks=" << flexible_task_ids.size() << " hierarchy levels=" << num_levels << "\n";

    // --- Check 1: disabled cascade == direct call ---------------------
    env.flow_solve_level = flow_solve_level_arg;
    std::unordered_map<int,list<int>> guide_paths_direct, guide_paths_noop;
    const auto assignments_direct = hierarchy.compute_reduced_assignment(
        &env, flexible_agent_ids, flexible_task_ids, guide_paths_direct, true);
    // min_cascade_level == flow_solve_level: empty cascade range.
    const auto assignments_noop_eq = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, flow_solve_level_arg, guide_paths_noop, true);
    check(assignments_direct == assignments_noop_eq,
          "min_cascade_level == flow_solve_level reproduces compute_reduced_assignment() exactly");

    std::unordered_map<int,list<int>> guide_paths_noop2;
    // min_cascade_level way above flow_solve_level: still clamped to a no-op.
    const auto assignments_noop_above = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, flow_solve_level_arg + 50, guide_paths_noop2, true);
    check(assignments_direct == assignments_noop_above,
          "min_cascade_level > flow_solve_level clamps to the same no-op result");

    // min_cascade_level below 1 (e.g. 0 or negative): must clamp to >= 1
    // and still produce a valid (not necessarily identical) assignment,
    // not crash.
    std::unordered_map<int,list<int>> guide_paths_clamped;
    int local_cnt_clamped = 0, flow_cnt_clamped = 0;
    const auto assignments_clamped_low = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, -5, guide_paths_clamped, true, 1,
        nullptr, nullptr, nullptr, nullptr, &local_cnt_clamped, &flow_cnt_clamped);
    check_assignment_validity("min_cascade_level=-5 (clamped to 1)", assignments_clamped_low,
                               flexible_agent_ids, flexible_task_ids);
    check(static_cast<int>(assignments_clamped_low.size()) == local_cnt_clamped + flow_cnt_clamped,
          "min_cascade_level=-5: local_match_count + flow_match_count == number of pairs returned");

    // --- Check 2: a real multi-level cascade ---------------------------
    std::unordered_map<int,list<int>> guide_paths_cascade;
    int local_cnt = 0, flow_cnt = 0;
    const auto assignments_cascade = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, 1, guide_paths_cascade, true, 1,
        nullptr, nullptr, nullptr, nullptr, &local_cnt, &flow_cnt);
    check_assignment_validity("cascade (min_cascade_level=1, flow_solve_level=" + to_string(flow_solve_level_arg) + ")",
                               assignments_cascade, flexible_agent_ids, flexible_task_ids);
    check(static_cast<int>(assignments_cascade.size()) == local_cnt + flow_cnt,
          "cascade: local_match_count + flow_match_count == number of pairs returned");

    const long long dist_direct = total_distance(env, assignments_direct, agent_loc, task_loc);
    const long long dist_cascade = total_distance(env, assignments_cascade, agent_loc, task_loc);
    cout << "  single-level (flowSolveLevel=" << flow_solve_level_arg << "): "
         << assignments_direct.size() << " matched, total Manhattan distance = " << dist_direct << "\n";
    cout << "  cascade (minCascadeLevel=1 -> flowSolveLevel=" << flow_solve_level_arg << "): "
         << assignments_cascade.size() << " matched (" << local_cnt << " local / " << flow_cnt
         << " flow), total Manhattan distance = " << dist_cascade << "\n";

    // --- Check 3: entirety mode -----------------------------------------
    env.flow_solve_level = num_levels - 1;
    std::unordered_map<int,list<int>> guide_paths_entirety;
    int local_cnt_entirety = 0, flow_cnt_entirety = 0;
    const auto assignments_entirety = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, 1, guide_paths_entirety, true, 1,
        nullptr, nullptr, nullptr, nullptr, &local_cnt_entirety, &flow_cnt_entirety);
    check_assignment_validity("entirety mode (flow_solve_level=top=" + to_string(num_levels - 1) + ")",
                               assignments_entirety, flexible_agent_ids, flexible_task_ids);
    check(flow_cnt_entirety == 0,
          "entirety mode: flow_match_count == 0 (flow is a vacuous no-op once the hand-off level is the true top)");
    check(assignments_entirety.size() == std::min(flexible_agent_ids.size(), flexible_task_ids.size()),
          "entirety mode: matched everyone possible (== min(agents,tasks))");
    const long long dist_entirety = total_distance(env, assignments_entirety, agent_loc, task_loc);
    cout << "  entirety (minCascadeLevel=1 -> flowSolveLevel=top=" << (num_levels - 1) << "): "
         << assignments_entirety.size() << " matched (" << local_cnt_entirety << " local / "
         << flow_cnt_entirety << " flow), total Manhattan distance = " << dist_entirety << "\n";

    // --- Check 4: level-skipping (cascade_level_stride) -----------------
    // Disabled cascade must still be a no-op regardless of stride -- the
    // loop range is empty either way, so stride is never even consulted.
    env.flow_solve_level = flow_solve_level_arg;
    std::unordered_map<int,list<int>> guide_paths_noop_stride;
    const auto assignments_noop_stride = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, flow_solve_level_arg, guide_paths_noop_stride, true, 3);
    check(assignments_direct == assignments_noop_stride,
          "disabled cascade (min_cascade_level == flow_solve_level) ignores stride and still no-ops");

    // A real cascade with stride=2: same validity contract as stride=1,
    // just fewer/larger match_local_node_exact() calls internally. Not
    // expected to equal assignments_cascade (stride changes which items get
    // grouped together at which level -- see ai/hierarchical_matching.md's
    // "not lossless" caveat, which applies per-level too), so only the
    // general validity invariants are checked, same as any other cascade.
    std::unordered_map<int,list<int>> guide_paths_stride2;
    int local_cnt_stride2 = 0, flow_cnt_stride2 = 0;
    const auto assignments_stride2 = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, 1, guide_paths_stride2, true, 2,
        nullptr, nullptr, nullptr, nullptr, &local_cnt_stride2, &flow_cnt_stride2);
    check_assignment_validity("cascade stride=2 (min_cascade_level=1, flow_solve_level=" + to_string(flow_solve_level_arg) + ")",
                               assignments_stride2, flexible_agent_ids, flexible_task_ids);
    check(static_cast<int>(assignments_stride2.size()) == local_cnt_stride2 + flow_cnt_stride2,
          "cascade stride=2: local_match_count + flow_match_count == number of pairs returned");

    // Entirety mode with stride=2: flow must still be a vacuous no-op and
    // everyone possible still gets matched -- stride only changes how many
    // match_local_node_exact() calls it takes to get there, not whether the
    // cascade + hand-off together still cover every remaining pairing.
    env.flow_solve_level = num_levels - 1;
    std::unordered_map<int,list<int>> guide_paths_entirety_stride2;
    int local_cnt_entirety_stride2 = 0, flow_cnt_entirety_stride2 = 0;
    const auto assignments_entirety_stride2 = hierarchy.compute_hierarchical_assignment(
        &env, flexible_agent_ids, flexible_task_ids, 1, guide_paths_entirety_stride2, true, 2,
        nullptr, nullptr, nullptr, nullptr, &local_cnt_entirety_stride2, &flow_cnt_entirety_stride2);
    check_assignment_validity("entirety mode stride=2 (flow_solve_level=top=" + to_string(num_levels - 1) + ")",
                               assignments_entirety_stride2, flexible_agent_ids, flexible_task_ids);
    check(flow_cnt_entirety_stride2 == 0,
          "entirety mode stride=2: flow_match_count == 0");
    check(assignments_entirety_stride2.size() == std::min(flexible_agent_ids.size(), flexible_task_ids.size()),
          "entirety mode stride=2: matched everyone possible (== min(agents,tasks))");
    const long long dist_stride2 = total_distance(env, assignments_stride2, agent_loc, task_loc);
    const long long dist_entirety_stride2 = total_distance(env, assignments_entirety_stride2, agent_loc, task_loc);
    cout << "  cascade stride=2 (minCascadeLevel=1 -> flowSolveLevel=" << flow_solve_level_arg << "): "
         << assignments_stride2.size() << " matched (" << local_cnt_stride2 << " local / " << flow_cnt_stride2
         << " flow), total Manhattan distance = " << dist_stride2 << "\n";
    cout << "  entirety stride=2 (minCascadeLevel=1 -> flowSolveLevel=top=" << (num_levels - 1) << "): "
         << assignments_entirety_stride2.size() << " matched (" << local_cnt_entirety_stride2 << " local / "
         << flow_cnt_entirety_stride2 << " flow), total Manhattan distance = " << dist_entirety_stride2 << "\n";

    cout << "\n" << g_checks_run << " checks run, " << g_checks_failed << " failed.\n";
    return g_checks_failed == 0 ? 0 : 1;
}
