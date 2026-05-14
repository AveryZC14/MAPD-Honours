#include <iostream>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <list>
#include <unordered_map>
#include <stdexcept>
#include <lemon/graph_to_eps.h>
#include <lemon/lgf_writer.h>

#include "SharedEnv.h"
#include "MapCoarsenV1.h"
#include "Types.h"
#include "scheduler.h"

using namespace std;
using namespace DefaultPlanner;

// forward-declare reduced version (not declared in header)
namespace DefaultPlanner {
    void schedule_plan_flow_reduced(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
}

// Default input graph used when no input file is provided on the command line.
// static const std::string DEFAULT_INPUT_GRAPH = "instances/warehouseLarge/warehouseLarge_16000.json";
static const std::string DEFAULT_INPUT_GRAPH = "instances/random/random_400.json";

/**
 * Load a benchmark input, populate the shared environment, and build the fine
 * graph representation for the map.
 */
void load_fine_graph_from_input(const std::string& input_json,
                                SharedEnvironment& env,
                                MapReductionTest::CoarsenedGraph& fine_graph)
{
    std::ifstream f(input_json);
    if (!f.is_open())
    {
        throw std::runtime_error("Failed to open " + input_json);
    }

    nlohmann::json data;
    try
    {
        data = nlohmann::json::parse(f);
    }
    catch (nlohmann::json::parse_error& e)
    {
        throw std::runtime_error(std::string("Failed to parse ") + input_json + ": " + e.what());
    }

    std::filesystem::path p(input_json);
    std::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (!base_folder.empty() && base_folder.back() != '/')
        base_folder += '/';

    std::string map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    int team_size = read_param_json<int>(data, "teamSize");
    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<std::list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));

    env.rows = grid.rows;
    env.cols = grid.cols;
    env.map = grid.map;
    env.map_name = map_path.substr(map_path.find_last_of("/") + 1);
    env.num_of_agents = team_size;
    env.curr_states.resize(env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; ++i)
    {
        env.curr_states[i] = State(agents[i], 0, 0);
    }
    env.curr_task_schedule.assign(env.num_of_agents, -1);

    env.task_pool.clear();
    for (size_t t = 0; t < tasks.size(); ++t)
    {
        Task task(static_cast<int>(t), tasks[t], 0);
        env.task_pool[static_cast<int>(t)] = task;
    }

    env.new_tasks.clear();
    for (size_t t = 0; t < tasks.size(); ++t)
        env.new_tasks.push_back(static_cast<int>(t));

    env.new_freeagents.clear();
    for (int i = 0; i < env.num_of_agents; ++i)
        env.new_freeagents.push_back(i);

    MapReductionTest::build_from_environment(fine_graph, &env);
}

/**
 * Write the fine graph as an LGF file with node labels and arc labels.
 */
void write_fine_graph_dot(const MapReductionTest::CoarsenedGraph& graph,
                          const std::string& output_path)
{
    using lemon::ListDigraph;
    
    // Create node map for labels
    ListDigraph::NodeMap<std::string> node_label_map(graph.g);
    
    // Create arc map for arc labels (cost)
    ListDigraph::ArcMap<std::string> arc_label_map(graph.g);
    
    // Fill the node label map with a stable node id and coordinate format.
    for (ListDigraph::NodeIt n(graph.g); n != lemon::INVALID; ++n)
    {
        const auto& loc = graph.coarse_location[n];

        bool has_edges = false;
        for (ListDigraph::OutArcIt out_arc(graph.g, n); out_arc != lemon::INVALID; ++out_arc)
        {
            has_edges = true;
            break;
        }
        if (!has_edges)
        {
            for (ListDigraph::InArcIt in_arc(graph.g, n); in_arc != lemon::INVALID; ++in_arc)
            {
                has_edges = true;
                break;
            }
        }

        const std::pair<int, int> display_loc =
            (!has_edges && loc.first == 0 && loc.second == 0)
                ? std::make_pair(-1, -1)
                : loc;

        node_label_map[n] = std::to_string(ListDigraph::id(n)) + ":(" +
                            std::to_string(display_loc.first) + "," +
                            std::to_string(display_loc.second) + ")";
    }
    
    // Fill the arc label map with cost values
    for (ListDigraph::ArcIt a(graph.g); a != lemon::INVALID; ++a)
    {
        arc_label_map[a] = std::to_string(static_cast<int>(graph.cost[a]));
    }
    
    // Write using DigraphWriter
    lemon::digraphWriter(graph.g, output_path)
        .nodeMap("label", node_label_map)
        .arcMap("label", arc_label_map)
        .run();
}

// Benchmark scheduling methods: full flow vs reduced flow
int run_benchmark(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: map_reduction_test <input_json>\n";
        return 1;
    }

    std::string input_json = argv[1];
    std::ifstream f(input_json);
    if (!f.is_open()){
        std::cerr << "Failed to open " << input_json << std::endl;
        return 1;
    }

    nlohmann::json data;
    try{
        data = nlohmann::json::parse(f);
    }
    catch(nlohmann::json::parse_error& e){
        std::cerr << "Failed to parse " << input_json << ": " << e.what() << std::endl;
        return 1;
    }

    // base folder is the parent directory of the input file
    std::filesystem::path p(input_json);
    std::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (base_folder.size() > 0 && base_folder.back() != '/') base_folder += '/';

    // read parameters like driver.cpp
    std::string map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    int team_size = read_param_json<int>(data, "teamSize");
    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<std::list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));

    // populate SharedEnvironment
    SharedEnvironment env;
    env.rows = grid.rows;
    env.cols = grid.cols;
    env.map = grid.map;
    env.map_name = map_path.substr(map_path.find_last_of("/") + 1);
    env.num_of_agents = team_size;
    env.curr_states.resize(env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; ++i)
    {
        env.curr_states[i] = State(agents[i], 0, 0);
    }
    env.curr_task_schedule.assign(env.num_of_agents, -1);

    // populate tasks
    env.task_pool.clear();
    for (size_t t = 0; t < tasks.size(); ++t)
    {
        Task task((int)t, tasks[t], 0);
        env.task_pool[(int)t] = task;
    }

    // mark all tasks as newly revealed and all agents as free
    env.new_tasks.clear();
    for (size_t t = 0; t < tasks.size(); ++t) env.new_tasks.push_back((int)t);
    env.new_freeagents.clear();
    for (int i = 0; i < env.num_of_agents; ++i) env.new_freeagents.push_back(i);

    // prepare a zero background flow vector
    std::vector<Double4> background_flow(env.map.size());
    for (auto &d : background_flow)
        for (int i = 0; i < 4; ++i) d.d[i] = 0.0;

    // proposed schedules
    std::vector<int> schedule_flow;
    std::vector<int> schedule_reduced;

    // Basic diagnostics
    cout << "Map cells: " << env.map.size() << "  Tasks: " << env.task_pool.size() << "  Agents: " << env.num_of_agents << "\n";

    // We'll run multiple trials and alternate order to control for warm-up effects
    const int TRIALS = 5;
    std::vector<double> times_full;
    std::vector<double> times_reduced;

    for (int trial = 0; trial < TRIALS; ++trial)
    {
        bool run_full_first = (trial % 2 == 0);

        if (run_full_first)
        {
            // fresh copy for full
            SharedEnvironment efull = env;
            efull.curr_task_schedule.assign(efull.num_of_agents, -1);
            efull.task_pool = env.task_pool;
            efull.new_tasks = env.new_tasks;
            efull.new_freeagents = env.new_freeagents;

            auto t1 = std::chrono::high_resolution_clock::now();
            DefaultPlanner::schedule_plan_flow(1000, schedule_flow, &efull, background_flow, false, true);
            auto t2 = std::chrono::high_resolution_clock::now();
            times_full.push_back(std::chrono::duration<double>(t2 - t1).count());

            // fresh copy for reduced
            SharedEnvironment ered = env;
            ered.curr_task_schedule.assign(ered.num_of_agents, -1);
            ered.task_pool = env.task_pool;
            ered.new_tasks = env.new_tasks;
            ered.new_freeagents = env.new_freeagents;

            auto t3 = std::chrono::high_resolution_clock::now();
            DefaultPlanner::schedule_plan_flow_reduced(1000, schedule_reduced, &ered, background_flow, false, true);
            auto t4 = std::chrono::high_resolution_clock::now();
            times_reduced.push_back(std::chrono::duration<double>(t4 - t3).count());
        }
        else
        {
            SharedEnvironment ered = env;
            ered.curr_task_schedule.assign(ered.num_of_agents, -1);
            ered.task_pool = env.task_pool;
            ered.new_tasks = env.new_tasks;
            ered.new_freeagents = env.new_freeagents;

            auto t3 = std::chrono::high_resolution_clock::now();
            DefaultPlanner::schedule_plan_flow_reduced(1000, schedule_reduced, &ered, background_flow, false, true);
            auto t4 = std::chrono::high_resolution_clock::now();
            times_reduced.push_back(std::chrono::duration<double>(t4 - t3).count());

            SharedEnvironment efull = env;
            efull.curr_task_schedule.assign(efull.num_of_agents, -1);
            efull.task_pool = env.task_pool;
            efull.new_tasks = env.new_tasks;
            efull.new_freeagents = env.new_freeagents;

            auto t1 = std::chrono::high_resolution_clock::now();
            DefaultPlanner::schedule_plan_flow(1000, schedule_flow, &efull, background_flow, false, true);
            auto t2 = std::chrono::high_resolution_clock::now();
            times_full.push_back(std::chrono::duration<double>(t2 - t1).count());
        }
    }

    auto avg = [&](const std::vector<double>& v){ double s=0; for(auto x:v) s+=x; return s / (v.empty()?1:v.size()); };
    cout << "avg schedule_plan_flow: " << avg(times_full) << " s over " << times_full.size() << " runs\n";
    cout << "avg schedule_plan_flow_reduced: " << avg(times_reduced) << " s over " << times_reduced.size() << " runs\n";
    cout << "Reduced to normal ratio: " << (avg(times_reduced)/avg(times_full)) << "\n\n\n";

    // Done — printed averages above.

    return 0;
}


// Entry point
int main(int argc, char** argv)
{
    try {
        const std::string input_json = (argc >= 2) ? argv[1] : DEFAULT_INPUT_GRAPH;
        const std::string output_dot = (argc >= 3) ? argv[2] : input_json.substr(0, input_json.find_last_of('.')) + ".lgf";
        
        SharedEnvironment env;
        MapReductionTest::CoarsenedGraph fine_graph;
        load_fine_graph_from_input(input_json, env, fine_graph);
        write_fine_graph_dot(fine_graph, output_dot);
        
        std::cout << "Loaded fine graph: " << fine_graph.num_coarse_nodes << " nodes (" << env.rows << "x" << env.cols << ")\n";
        std::cout << "Wrote lgf file: " << output_dot << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading fine graph: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}