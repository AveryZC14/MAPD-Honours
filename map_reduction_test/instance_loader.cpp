#include "instance_loader.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "MapCoarsenV1.h"
#include "Types.h"

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
