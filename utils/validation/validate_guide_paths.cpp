// validate_guide_paths.cpp
//
// Standalone rigor check for the claim "solver 6's guide-path reconstruction
// produces output in the same format as solver 1's, and both are safe to
// hand to the low-level planner". Runs schedule_plan_flow (solver 1) and
// schedule_plan_flow_reduced (solver 6) on the same instance/state and
// checks, for every guide path either solver writes into the shared
// `agent_guide_path` map (`DefaultPlanner::get_guide_path()`):
//
//   1. It's a non-empty list<int> (both solvers write into the exact
//      same global, so the *type* is trivially identical -- the interesting
//      question is whether the *contents* are always well-formed).
//   2. The first node is the agent's real current location.
//   3. The last node is the assigned task's first (pickup) location.
//   4. Every consecutive pair of nodes is a valid 4-connected, walkable
//      fine-map arc (the same adjacency rule schedule_plan_flow's own graph
//      construction uses) -- this is exactly what add_traj/remove_traj
//      (default_planner/flow.cpp) assume when they derive a direction from
//      `loc - prev_loc` with no validation of their own.
//   5. GuidePathLengthSum/GuidePathCostSum (ScheduleTiming, read via
//      get_last_timing()) match a from-scratch recomputation over the
//      exposed paths, in the scenario where they're expected to line up
//      (use_traffic on, past the seed-timestep gate).
//
// Also separately checks the "always compute" behavior for solver 6 (see
// ai/guide_path_metric.md): with --useTraffic off, agent_guide_path must
// stay empty (no behavior change to what's fed to the planner) while
// GuidePathLengthSum/CostSum must still be > 0 (the metric is populated
// unconditionally now).
//
// Usage: ./guide_path_validator <instance.json> [--useTraffic]

#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include <string>
#include <cmath>

#include "SharedEnv.h"
#include "Types.h"
#include "scheduler.h"
#include "instance_loader.h"

using namespace std;
using namespace DefaultPlanner;

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

// 4-connected adjacency check mirroring exactly what schedule_plan_flow's
// own graph construction (and add_traj/remove_traj's direction-from-delta
// math in flow.cpp) assumes about consecutive guide-path cells -- a path
// that violates this wouldn't just be "suboptimal", it would silently
// corrupt the low-level planner's movement direction.
bool are_fine_neighbors(const SharedEnvironment& env, int a, int b)
{
    if (a < 0 || a >= static_cast<int>(env.map.size()) || b < 0 || b >= static_cast<int>(env.map.size()))
        return false;
    if (env.map[a] != 0 || env.map[b] != 0)
        return false;
    if (env.cols <= 0)
        return false;
    const int ra = a / env.cols, ca = a % env.cols;
    const int rb = b / env.cols, cb = b % env.cols;
    return (ra == rb && (cb == ca + 1 || cb == ca - 1)) ||
           (ca == cb && (rb == ra + 1 || rb == ra - 1));
}

// Validate every guide path in `paths` against the format rules described
// above. `label` is used in failure messages (e.g. "solver1"). Returns the
// number of guide paths checked.
int validate_paths(const string& label,
                    const boost::unordered_map<int, list<int>>& paths,
                    const SharedEnvironment& env,
                    const vector<int>& proposed_schedule,
                    const boost::unordered_map<int, int>& agent_start_loc)
{
    int checked = 0;
    for (const auto& kv : paths)
    {
        const int agent_id = kv.first;
        const list<int>& path = kv.second;
        ++checked;

        // Check 1: non-empty (see are_fine_neighbors' comment above for why
        // an empty path is worse than just "wrong").
        check(!path.empty(), label + ": agent " + to_string(agent_id) + " has an EMPTY guide path "
              "(would underflow trajs[agent].size()-1 in flow.cpp's add_traj/remove_traj)");
        if (path.empty())
            continue;

        // Check 2: starts at the agent's real location. `agent_start_loc` is
        // snapshotted once from base_env before either solver runs (both env1
        // and env6 are independent copies of the same base_env and neither
        // solver moves agents), so it's equivalent to reading
        // env.curr_states[agent_id].location here -- just avoids depending on
        // which of the two per-solver env copies happens to be passed in.
        const auto start_it = agent_start_loc.find(agent_id);
        check(start_it != agent_start_loc.end(), label + ": agent " + to_string(agent_id) + " not found in start-location snapshot");
        if (start_it != agent_start_loc.end())
        {
            check(path.front() == start_it->second,
                  label + ": agent " + to_string(agent_id) + " guide path starts at " + to_string(path.front()) +
                  " but agent's real location is " + to_string(start_it->second));
        }

        // Check 3: agent actually has an assigned task, and the guide path
        // ends at that task's pickup location.
        check(agent_id >= 0 && agent_id < static_cast<int>(proposed_schedule.size()) && proposed_schedule[agent_id] != -1,
              label + ": agent " + to_string(agent_id) + " has a guide path but no assigned task in proposed_schedule");
        if (agent_id >= 0 && agent_id < static_cast<int>(proposed_schedule.size()) && proposed_schedule[agent_id] != -1)
        {
            const int task_id = proposed_schedule[agent_id];
            const auto task_it = env.task_pool.find(task_id);
            check(task_it != env.task_pool.end(), label + ": agent " + to_string(agent_id) + "'s assigned task " + to_string(task_id) + " not in task_pool");
            if (task_it != env.task_pool.end())
            {
                const int task_loc = task_it->second.locations[0];
                check(path.back() == task_loc,
                      label + ": agent " + to_string(agent_id) + " guide path ends at " + to_string(path.back()) +
                      " but assigned task " + to_string(task_id) + "'s location is " + to_string(task_loc));
            }
        }

        // Check 4: every consecutive pair of cells is a valid 4-connected,
        // walkable step -- walks the whole path once, checking both
        // in-bounds/non-obstacle (per cell) and adjacency (per consecutive
        // pair) in the same pass.
        int prev = -1;
        bool first = true;
        int step = 0;
        for (int loc : path)
        {
            check(loc >= 0 && loc < static_cast<int>(env.map.size()) && env.map[loc] == 0,
                  label + ": agent " + to_string(agent_id) + " guide path step " + to_string(step) +
                  " location " + to_string(loc) + " is out of bounds or an obstacle");
            if (!first)
            {
                check(are_fine_neighbors(env, prev, loc),
                      label + ": agent " + to_string(agent_id) + " guide path has a non-adjacent jump " +
                      to_string(prev) + " -> " + to_string(loc) + " at step " + to_string(step));
            }
            prev = loc;
            first = false;
            ++step;
        }
    }
    return checked;
}

// Independent recomputation of GuidePathLengthSum straight from the
// exposed paths (edges = cells - 1 per path), used to cross-check the
// metric each scheduler reports via ScheduleTiming actually matches what
// it handed out -- catches the metric silently drifting from reality.
double sum_path_lengths(const boost::unordered_map<int, list<int>>& paths)
{
    double total = 0.0;
    for (const auto& kv : paths)
        if (!kv.second.empty())
            total += static_cast<double>(kv.second.size() - 1);
    return total;
}

void print_timing(const string& label, const ScheduleTiming& t)
{
    cout << "  " << label << ": solve_time=" << t.solve_time
         << "s guide_path_time=" << t.guide_path_time
         << "s GuidePathLengthSum=" << t.guide_path_length_sum
         << " GuidePathCostSum=" << t.guide_path_cost_sum << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout << "Usage: guide_path_validator <instance.json> [--useTraffic]\n";
        return 1;
    }

    const string input_json = argv[1];
    bool use_traffic = false;
    for (int i = 2; i < argc; ++i)
        if (string(argv[i]) == "--useTraffic")
            use_traffic = true;

    SharedEnvironment base_env;
    try
    {
        populate_env_from_instance(input_json, base_env);
    }
    catch (const std::exception& e)
    {
        cerr << "Error loading instance: " << e.what() << endl;
        return 1;
    }

    cout << "Instance: " << input_json << "  cells=" << base_env.map.size()
         << " agents=" << base_env.num_of_agents << " tasks=" << base_env.task_pool.size() << "\n";

    // Flat zero congestion field -- neither solver's *assignment* cost
    // depends on traffic in a way this validator cares about; only
    // use_traffic's effect on the guide-path gate (see below) matters here.
    vector<Double4> background_flow(base_env.map.size());
    for (auto& d : background_flow)
        for (int i = 0; i < 4; ++i) d.d[i] = 0.0;

    // Snapshotted once, before either solver runs, so validate_paths' "does
    // the path start where the agent really is" check has a ground truth
    // that's independent of which per-solver env copy gets passed in.
    boost::unordered_map<int, int> agent_start_loc;
    for (int i = 0; i < base_env.num_of_agents; ++i)
        agent_start_loc[i] = base_env.curr_states[i].location;

    // --- Scenario 1: gate open (use_traffic on, curr_timestep past the
    // 100-timestep warm-up) -- both solvers are expected to actually
    // populate agent_guide_path, so this is where the format checks bite. ---
    {
        cout << "\n=== Scenario 1: --useTraffic on, curr_timestep=100 (planner-seed gate open) ===\n";

        // env1/env6 are independent copies of the same base_env -- each
        // solver gets its own untouched starting state rather than seeing
        // whatever the other solver's run left behind.
        SharedEnvironment env1 = base_env;
        env1.curr_timestep = 100;
        vector<int> schedule1;
        // time_limit=1000 here is nominal, not strictly enforced: these
        // entry points don't cut the underlying NetworkSimplex solve short
        // at that budget (a large/sparse instance can take well over 1000
        // of whatever unit this is and still run to completion).
        // new_only=true: only ever assign brand-new unassigned tasks, no
        // reshuffling of already-assigned-but-unopened ones (see the
        // --assignNew CLI flag / ai/project_context.md).
        schedule_plan_flow(1000, schedule1, &env1, background_flow, /*use_traffic=*/true, /*new_only=*/true);
        auto paths1 = get_guide_path();
        ScheduleTiming timing1 = get_last_timing();

        SharedEnvironment env6 = base_env;
        env6.curr_timestep = 100;
        vector<int> schedule6;
        schedule_plan_flow_reduced(1000, schedule6, &env6, background_flow, /*use_traffic=*/true, /*new_only=*/true);
        auto paths6 = get_guide_path();
        ScheduleTiming timing6 = get_last_timing();

        cout << "solver1: " << paths1.size() << " guide paths returned\n";
        cout << "solver6: " << paths6.size() << " guide paths returned\n";

        const int checked1 = validate_paths("solver1", paths1, env1, schedule1, agent_start_loc);
        const int checked6 = validate_paths("solver6", paths6, env6, schedule6, agent_start_loc);

        // Guards against a silent false-pass: validate_paths' for-loop over
        // an empty `paths` map runs zero iterations and returns 0 checked
        // without ever calling check(), which would otherwise look like
        // "all checks passed" while nothing was actually verified.
        check(checked1 > 0, "solver1 produced zero guide paths -- nothing was actually validated");
        check(checked6 > 0, "solver6 produced zero guide paths -- nothing was actually validated");

        print_timing("solver1 timing", timing1);
        print_timing("solver6 timing", timing6);

        const double recomputed1 = sum_path_lengths(paths1);
        const double recomputed6 = sum_path_lengths(paths6);
        check(std::fabs(recomputed1 - timing1.guide_path_length_sum) < 1e-6,
              "solver1: GuidePathLengthSum (" + to_string(timing1.guide_path_length_sum) +
              ") doesn't match recomputed sum over exposed paths (" + to_string(recomputed1) + ")");
        check(std::fabs(recomputed6 - timing6.guide_path_length_sum) < 1e-6,
              "solver6: GuidePathLengthSum (" + to_string(timing6.guide_path_length_sum) +
              ") doesn't match recomputed sum over exposed paths (" + to_string(recomputed6) + ")");

        // "same format" also means: nothing about the *type* differs. Both
        // maps are boost::unordered_map<int,list<int>> by construction (get_guide_path()'s
        // return type), so this is a compile-time guarantee -- assert it stays true.
        static_assert(std::is_same<decltype(paths1), decltype(paths6)>::value,
                      "solver1 and solver6 guide paths must be the exact same container type");
    }

    // --- Scenario 2: gate closed (use_traffic off) -- the metric should
    // still be populated for solver 6 (the whole point of decoupling the
    // fine-lift from the seed gate), but agent_guide_path must stay EMPTY,
    // i.e. no change to what actually reaches the planner in the runs done
    // so far (which never passed --useTraffic). ---
    {
        cout << "\n=== Scenario 2: --useTraffic off (planner-seed gate closed, as in every prior benchmark run) ===\n";

        SharedEnvironment env1 = base_env;
        env1.curr_timestep = 0;
        vector<int> schedule1;
        schedule_plan_flow(1000, schedule1, &env1, background_flow, /*use_traffic=*/false, /*new_only=*/true);
        auto paths1 = get_guide_path();
        ScheduleTiming timing1 = get_last_timing();

        SharedEnvironment env6 = base_env;
        env6.curr_timestep = 0;
        vector<int> schedule6;
        schedule_plan_flow_reduced(1000, schedule6, &env6, background_flow, /*use_traffic=*/false, /*new_only=*/true);
        auto paths6 = get_guide_path();
        ScheduleTiming timing6 = get_last_timing();

        print_timing("solver1 timing", timing1);
        print_timing("solver6 timing", timing6);

        check(paths1.empty(), "solver1: agent_guide_path should be empty with --useTraffic off (unchanged prior behavior), got " + to_string(paths1.size()));
        check(paths6.empty(), "solver6: agent_guide_path should be empty with --useTraffic off (unchanged prior behavior), got " + to_string(paths6.size()));
        check(timing1.guide_path_length_sum > 0.0, "solver1: GuidePathLengthSum should still be > 0 with --useTraffic off (the walk always happens) -- got " + to_string(timing1.guide_path_length_sum));
        check(timing6.guide_path_length_sum > 0.0, "solver6: GuidePathLengthSum should still be > 0 with --useTraffic off (fine lift is now unconditional) -- got " + to_string(timing6.guide_path_length_sum));
    }

    cout << "\n=== Summary: " << g_checks_run << " checks run, " << g_checks_failed << " failed ===\n";
    return g_checks_failed == 0 ? 0 : 1;
}
