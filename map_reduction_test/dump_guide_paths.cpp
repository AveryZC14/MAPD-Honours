// dump_guide_paths.cpp
//
// Standalone tool to export solver 1 (schedule_plan_flow) and solver 6
// (schedule_plan_flow_reduced) guide paths for a handful of agents, so they
// can be visualised over the map (see map_reduction_test/visualisation/
// plot_guide_paths.py). Companion to validate_guide_paths.cpp, which checks
// the same guide paths for structural validity instead of dumping them.
//
// Both solvers run on the same base instance state and independently decide
// each agent's task assignment -- they are not forced to agree, so an
// agent's path under solver 1 and under solver 6 may end at different task
// locations. Both are written out (with their destination task id) rather
// than only comparing agents that happen to match, since the mismatch
// itself is part of what's being compared.
//
// Guide paths are only populated (agent_guide_path via get_guide_path())
// when use_traffic is on and curr_timestep >= 100 -- see
// ai/project_context.md, "Guide paths: which solvers provide them". This
// tool always runs in that state; there's no other state in which either
// solver produces a non-empty guide path to dump.
//
// Usage: ./dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all] [--match-runtime]
//
// num_tasks, if given, truncates the task pool down to the first num_tasks
// task ids (in the order they appear in the instance's task file) before
// either solver sees them -- both solvers get the same shrunk pool, so this
// shrinks the assignment problem itself (smaller flow graph, fewer flexible
// tasks to offer), not just what gets dumped afterward. When num_tasks
// shrinks the pool below the instance's full team size, the solver's agent
// pool is shrunk to match too (see the feasibility comment in main()).
//
// --match-runtime ignores num_tasks and instead caps the task pool the way
// a real `lifelong` run would at this exact instant: src/TaskManager.cpp's
// reveal_tasks() never lets the *open* task pool (env->task_pool) exceed
// `numTasksReveal * teamSize` tasks, refilling only the small delta that
// closed since the last timestep -- it never loads the whole task file at
// once the way populate_env_from_instance does by default. This mode
// computes that same cap from the instance JSON's own numTasksReveal field
// and, unlike plain num_tasks capping, does NOT shrink the agent pool
// (unless the computed cap is pathologically smaller than the team size) --
// a real run always has the *full* team competing for the capped pool, only
// num_agents (how many get written to the CSV afterward) stays small. This
// is still a single static snapshot, not the real incremental reveal/expiry
// loop -- see ai/guide_path_visualisation.md, "Known gaps".

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "SharedEnv.h"
#include "Types.h"
#include "scheduler.h"
#include "instance_loader.h"

using namespace std;
using namespace DefaultPlanner;

namespace {

void dump_solver(ofstream& out, const string& solver_label,
                  const boost::unordered_map<int, list<int>>& paths,
                  const vector<int>& proposed_schedule,
                  const SharedEnvironment& env,
                  int num_agents)
{
    for (const auto& kv : paths)
    {
        const int agent_id = kv.first;
        if (agent_id < 0 || agent_id >= num_agents)
            continue;

        const int task_id = (agent_id < static_cast<int>(proposed_schedule.size()))
                                 ? proposed_schedule[agent_id] : -1;

        int step = 0;
        for (int loc : kv.second)
        {
            const int row = env.cols > 0 ? loc / env.cols : -1;
            const int col = env.cols > 0 ? loc % env.cols : -1;
            out << solver_label << "," << agent_id << "," << task_id << ","
                << step << "," << loc << "," << row << "," << col << "\n";
            ++step;
        }
    }
}

// Erase every task_pool/new_tasks entry with id >= keep_below.
void truncate_task_pool(SharedEnvironment& env, int keep_below)
{
    for (auto it = env.task_pool.begin(); it != env.task_pool.end(); )
    {
        if (it->first >= keep_below)
            it = env.task_pool.erase(it);
        else
            ++it;
    }
    env.new_tasks.erase(
        std::remove_if(env.new_tasks.begin(), env.new_tasks.end(),
                        [keep_below](int id) { return id >= keep_below; }),
        env.new_tasks.end());
}

// Shrink the solver's agent pool to keep_below agents (already 0..N-1 in
// order, so this is just a resize down).
void truncate_agent_pool(SharedEnvironment& env, int keep_below)
{
    env.num_of_agents = keep_below;
    env.curr_states.resize(keep_below);
    env.curr_task_schedule.resize(keep_below);
    env.new_freeagents.resize(keep_below);
}

} // namespace

int main(int argc, char** argv)
{
    vector<string> args(argv + 1, argv + argc);
    bool match_runtime = false;
    vector<string> positional;
    for (const auto& a : args)
    {
        if (a == "--match-runtime")
            match_runtime = true;
        else
            positional.push_back(a);
    }

    if (positional.size() < 2)
    {
        cout << "Usage: dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all] [--match-runtime]\n";
        return 1;
    }

    const string input_json = positional[0];
    const string output_csv = positional[1];
    const int num_agents = positional.size() > 2 ? atoi(positional[2].c_str()) : 20;
    const int num_tasks = positional.size() > 3 ? atoi(positional[3].c_str()) : -1;

    if (match_runtime && num_tasks >= 0)
        cerr << "Warning: --match-runtime ignores the explicit num_tasks argument (" << num_tasks << ").\n";

    SharedEnvironment base_env;
    float num_tasks_reveal = 1.0f;
    try
    {
        populate_env_from_instance(input_json, base_env, &num_tasks_reveal);
    }
    catch (const std::exception& e)
    {
        cerr << "Error loading instance: " << e.what() << endl;
        return 1;
    }

    const int team_size = base_env.num_of_agents; // before any capping below
    int dump_limit = num_agents;

    if (match_runtime)
    {
        // Matches src/TaskManager.cpp's reveal_tasks() cap on the open task
        // pool: numTasksReveal * teamSize, clamped to however many tasks the
        // instance's task file actually has.
        const int reveal_cap = std::max(1, static_cast<int>(std::lround(num_tasks_reveal * team_size)));
        const int effective_cap = std::min(reveal_cap, static_cast<int>(base_env.task_pool.size()));

        cout << "--match-runtime: numTasksReveal=" << num_tasks_reveal << " * teamSize=" << team_size
             << " -> open task pool capped at " << effective_cap << " tasks\n";

        if (effective_cap < static_cast<int>(base_env.task_pool.size()))
            truncate_task_pool(base_env, effective_cap);

        // Unlike plain num_tasks capping, a real run never shrinks the agent
        // team to fit the task pool -- the whole team stays flexible. Only
        // fall back to shrinking it if the computed cap is pathologically
        // below the team size (numTasksReveal < 1), where solver 1's
        // feasibility requirement (tasks >= agents) would otherwise be
        // violated.
        if (effective_cap < team_size)
        {
            cerr << "Warning: numTasksReveal*teamSize (" << effective_cap << ") < teamSize (" << team_size
                 << ") -- shrinking the solver's agent pool to match for feasibility.\n";
            truncate_agent_pool(base_env, effective_cap);
        }

        dump_limit = std::min(num_agents, base_env.num_of_agents);
    }
    else
    {
        // Solver 1's flow formulation requires at least as many flexible
        // tasks as flexible agents (supply[sink] = -num_workers -- every
        // free agent must be matched to a task, see schedule_plan_flow,
        // scheduler.cpp). So shrinking the task pool below the agent count
        // makes the assignment infeasible for both solvers. Since we only
        // ever visualise the first num_agents agents anyway, shrink the
        // agent count fed to the solver to match -- that keeps the problem
        // small *and* feasible.
        if (num_agents < base_env.num_of_agents)
            truncate_agent_pool(base_env, num_agents);

        if (num_tasks >= 0 && num_tasks < static_cast<int>(base_env.task_pool.size()))
        {
            truncate_task_pool(base_env, num_tasks);

            if (num_tasks < base_env.num_of_agents)
                cerr << "Warning: num_tasks (" << num_tasks << ") < num_agents ("
                     << base_env.num_of_agents << ") -- solver 1's assignment will be infeasible.\n";
        }
    }

    cout << "Instance: " << input_json << "  cells=" << base_env.map.size()
         << " (" << base_env.rows << "x" << base_env.cols << ")"
         << " agents=" << base_env.num_of_agents << " tasks=" << base_env.task_pool.size()
         << "\nDumping guide paths for agents 0.." << (dump_limit - 1) << " to " << output_csv << "\n";

    vector<Double4> background_flow(base_env.map.size());
    for (auto& d : background_flow)
        for (int i = 0; i < 4; ++i) d.d[i] = 0.0;

    ofstream out(output_csv);
    if (!out.is_open())
    {
        cerr << "Failed to open " << output_csv << " for writing\n";
        return 1;
    }
    out << "solver,agent_id,task_id,step,loc,row,col\n";

    cout << "Running solver 1 (schedule_plan_flow)...\n";
    SharedEnvironment env1 = base_env;
    env1.curr_timestep = 100;
    vector<int> schedule1;
    schedule_plan_flow(1000, schedule1, &env1, background_flow, /*use_traffic=*/true, /*new_only=*/true);
    auto paths1 = get_guide_path();
    cout << "  solver 1: " << paths1.size() << " guide paths returned\n";
    dump_solver(out, "solver1", paths1, schedule1, env1, dump_limit);

    cout << "Running solver 6 (schedule_plan_flow_reduced)...\n";
    SharedEnvironment env6 = base_env;
    env6.curr_timestep = 100;
    vector<int> schedule6;
    schedule_plan_flow_reduced(1000, schedule6, &env6, background_flow, /*use_traffic=*/true, /*new_only=*/true);
    auto paths6 = get_guide_path();
    cout << "  solver 6: " << paths6.size() << " guide paths returned\n";
    dump_solver(out, "solver6", paths6, schedule6, env6, dump_limit);

    out.close();
    cout << "Done.\n";
    return 0;
}
