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
// Usage: ./dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all]
//
// num_tasks, if given, truncates the task pool down to the first num_tasks
// task ids (in the order they appear in the instance's task file) before
// either solver sees them -- both solvers get the same shrunk pool, so this
// shrinks the assignment problem itself (smaller flow graph, fewer flexible
// tasks to offer), not just what gets dumped afterward.

#include <algorithm>
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

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cout << "Usage: dump_guide_paths <instance.json> <output.csv> [num_agents=20] [num_tasks=all]\n";
        return 1;
    }

    const string input_json = argv[1];
    const string output_csv = argv[2];
    const int num_agents = argc > 3 ? atoi(argv[3]) : 20;
    const int num_tasks = argc > 4 ? atoi(argv[4]) : -1;

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

    // Solver 1's flow formulation requires at least as many flexible tasks as
    // flexible agents (supply[sink] = -num_workers -- every free agent must
    // be matched to a task, see schedule_plan_flow, scheduler.cpp). So
    // shrinking the task pool below the agent count makes the assignment
    // infeasible for both solvers. Since we only ever visualise the first
    // num_agents agents anyway, shrink the agent count fed to the solver to
    // match -- that keeps the problem small *and* feasible.
    if (num_agents < base_env.num_of_agents)
    {
        base_env.num_of_agents = num_agents;
        base_env.curr_states.resize(num_agents);
        base_env.curr_task_schedule.resize(num_agents);
        base_env.new_freeagents.resize(num_agents); // already 0..N-1 in order
    }

    if (num_tasks >= 0 && num_tasks < static_cast<int>(base_env.task_pool.size()))
    {
        for (auto it = base_env.task_pool.begin(); it != base_env.task_pool.end(); )
        {
            if (it->first >= num_tasks)
                it = base_env.task_pool.erase(it);
            else
                ++it;
        }
        base_env.new_tasks.erase(
            std::remove_if(base_env.new_tasks.begin(), base_env.new_tasks.end(),
                            [num_tasks](int id) { return id >= num_tasks; }),
            base_env.new_tasks.end());

        if (num_tasks < base_env.num_of_agents)
            cerr << "Warning: num_tasks (" << num_tasks << ") < num_agents ("
                 << base_env.num_of_agents << ") -- solver 1's assignment will be infeasible.\n";
    }

    cout << "Instance: " << input_json << "  cells=" << base_env.map.size()
         << " (" << base_env.rows << "x" << base_env.cols << ")"
         << " agents=" << base_env.num_of_agents << " tasks=" << base_env.task_pool.size()
         << "\nDumping guide paths for agents 0.." << (num_agents - 1) << " to " << output_csv << "\n";

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
    dump_solver(out, "solver1", paths1, schedule1, env1, num_agents);

    cout << "Running solver 6 (schedule_plan_flow_reduced)...\n";
    SharedEnvironment env6 = base_env;
    env6.curr_timestep = 100;
    vector<int> schedule6;
    schedule_plan_flow_reduced(1000, schedule6, &env6, background_flow, /*use_traffic=*/true, /*new_only=*/true);
    auto paths6 = get_guide_path();
    cout << "  solver 6: " << paths6.size() << " guide paths returned\n";
    dump_solver(out, "solver6", paths6, schedule6, env6, num_agents);

    out.close();
    cout << "Done.\n";
    return 0;
}
