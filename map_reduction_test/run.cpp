#include <iostream>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
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

// Default input graph used when no input file is provided on the command line.
// static const std::string DEFAULT_INPUT_GRAPH = "instances/warehouseLarge/warehouseLarge_16000.json";
// static const std::string DEFAULT_INPUT_GRAPH = "instances/warehouseSmall/warehouseSmall_100.json";
// static const std::string DEFAULT_INPUT_GRAPH = "instances/custom/tiny/tiny.json";
static const std::string DEFAULT_INPUT_GRAPH = "instances/custom/tiny/tinyComplex.json";
// static const std::string DEFAULT_INPUT_GRAPH = "instances/random/random_400.json";

/**
 * Parse an instance JSON (map/agent/task file triple, same format as
 * src/driver.cpp reads for the full `lifelong` binary) and populate `env`
 * with its map, agent start locations, and tasks. All agents start free and
 * all tasks start newly-revealed, as they would be at timestep 0 of a real
 * run. Throws std::runtime_error if the file can't be opened or parsed.
 */
void populate_env_from_instance(const std::string& input_json, SharedEnvironment& env)
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
}

/**
 * Load a benchmark input, populate the shared environment, and build the fine
 * graph representation for the map.
 */
void load_fine_graph_from_input(const std::string& input_json,
                                SharedEnvironment& env,
                                MapReductionTest::CoarsenedGraph& fine_graph)
{
    populate_env_from_instance(input_json, env);
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

/**
 * Print the nodes_at_location grid as a simple 2D table.
 * Each cell shows the graph ids stored at that coarse location.
 */
void print_nodes_at_location_table(const MapReductionTest::CoarsenedGraph& graph,
                                   std::ostream& out = std::cout)
{
    if (graph.nodes_at_location.empty())
    {
        out << "nodes_at_location is empty\n";
        return;
    }

    for (size_t row = 0; row < graph.nodes_at_location.size(); ++row)
    {
        out << "row " << row << ": ";
        for (size_t col = 0; col < graph.nodes_at_location[row].size(); ++col)
        {
            const auto& cell = graph.nodes_at_location[row][col];
            out << "[";
            for (size_t i = 0; i < cell.size(); ++i)
            {
                if (i > 0)
                    out << ",";
                out << cell[i];
            }
            out << "]";
            if (col + 1 < graph.nodes_at_location[row].size())
                out << " | ";
        }
        out << '\n';
    }
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

    SharedEnvironment env;
    try
    {
        populate_env_from_instance(input_json, env);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

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

    // Each trial gets a fresh copy of `env` (schedulers mutate agent/task
    // assignment state), so every run starts from the same input.
    auto run_full_trial = [&]() {
        SharedEnvironment efull = env;
        const auto t1 = std::chrono::high_resolution_clock::now();
        DefaultPlanner::schedule_plan_flow(1000, schedule_flow, &efull, background_flow, false, true);
        const auto t2 = std::chrono::high_resolution_clock::now();
        times_full.push_back(std::chrono::duration<double>(t2 - t1).count());
    };
    auto run_reduced_trial = [&]() {
        SharedEnvironment ered = env;
        const auto t1 = std::chrono::high_resolution_clock::now();
        DefaultPlanner::schedule_plan_flow_reduced(1000, schedule_reduced, &ered, background_flow, false, true);
        const auto t2 = std::chrono::high_resolution_clock::now();
        times_reduced.push_back(std::chrono::duration<double>(t2 - t1).count());
    };

    for (int trial = 0; trial < TRIALS; ++trial)
    {
        // Alternate which scheduler runs first each trial, so neither one
        // consistently benefits from (or is penalized by) running second.
        const bool run_full_first = (trial % 2 == 0);
        if (run_full_first)
        {
            run_full_trial();
            run_reduced_trial();
        }
        else
        {
            run_reduced_trial();
            run_full_trial();
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
        const int COARSEN_LEVELS = 2;

        const std::string input_json = (argc >= 2) ? argv[1] : DEFAULT_INPUT_GRAPH;
        const std::string output_dot = (argc >= 3) ? argv[2] : input_json.substr(0, input_json.find_last_of('.')) + ".lgf";
        
        SharedEnvironment env;
        MapReductionTest::CoarsenedGraph fine_graph;
        load_fine_graph_from_input(input_json, env, fine_graph);
        write_fine_graph_dot(fine_graph, output_dot);

        std::cout << summarise_graph(fine_graph);

        std::vector<std::unique_ptr<MapReductionTest::CoarsenedGraph>> coarsened_levels;
        coarsened_levels.reserve(COARSEN_LEVELS);

        print_nodes_at_location_table(fine_graph);

        MapReductionTest::CoarsenedGraph* current_graph = &fine_graph;
        std::cout << "\n";
        std::cout << "\n";

        for (int level = 1; level <= COARSEN_LEVELS; ++level)
        {
            std::unique_ptr<MapReductionTest::CoarsenedGraph> next_graph(Coarsen(*current_graph));
            if (next_graph == nullptr)
            {
                std::cout << "Stopped coarsening at level " << level << " because no further coarsening was produced.\n";
                break;
            }

            std::string coarse_output;
            size_t dot = output_dot.find_last_of('.');
            if (dot == std::string::npos)
                coarse_output = output_dot + "_c" + std::to_string(level);
            else
                coarse_output = output_dot.substr(0, dot) + "_c" + std::to_string(level) + output_dot.substr(dot);

            write_fine_graph_dot(*next_graph, coarse_output);
            std::cout << summarise_graph(*next_graph) << std::endl;
            std::cout << "Wrote coarsened lgf file: " << coarse_output << "\n";
            std::cout << "\n";
            print_nodes_at_location_table(*next_graph);
            std::cout << "\n";
            std::cout << "\n";

            current_graph = next_graph.get();
            coarsened_levels.push_back(std::move(next_graph));
        }

        std::cout << "Loaded fine graph: " << fine_graph.num_coarse_nodes << " nodes (" << env.rows << "x" << env.cols << ")\n";
        std::cout << "Wrote lgf file: " << output_dot << "\n";
        // After preparing the graphs, run the benchmark to compare full vs reduced schedulers
        return run_benchmark(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading fine graph: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}