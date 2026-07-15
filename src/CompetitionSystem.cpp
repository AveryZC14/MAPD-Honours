#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>

using json = nlohmann::ordered_json;






// // This function might not work correctly with small map (w or h <=2)
// bool BaseSystem::valid_moves(vector<State>& prev, vector<Action>& action)
// {
//   return model->is_valid(prev, action);
// }


void BaseSystem::sync_shared_env() 
{
    if (!started)
    {
        env->goal_locations.resize(num_of_agents);
        task_manager.sync_shared_env(env);
        simulator.sync_shared_env(env);

        if (simulator.get_curr_timestep() == 0)
        {
            env->new_freeagents.reserve(num_of_agents); //new free agents are empty in task_manager on initialization, set it after task_manager sync
            for (int i = 0; i < num_of_agents; i++)
            {
                env->new_freeagents.push_back(i);
            }
        }
        //update proposed action to all wait
        proposed_actions.clear();
        proposed_actions.resize(num_of_agents, Action::WA);
        //update proposed schedule to previous assignment
        proposed_schedule = env->curr_task_schedule;
        
    }
    else
    {
        env->curr_timestep = simulator.get_curr_timestep();
    }
}


bool BaseSystem::planner_wrapper()
{
    //I think this is Entry::compute
    planner->compute(plan_time_limit, proposed_actions, proposed_schedule);

    /* Begin capturing scheduler timing for the completed planner step. */
    if (planner != nullptr && planner->scheduler != nullptr)
    {
        last_scheduler_timing = planner->scheduler->get_last_timing();
    }
    else
    {
        last_scheduler_timing = DefaultPlanner::ScheduleTiming();
    }
    /* End capturing scheduler timing for the completed planner step. */

    return true;
}


void BaseSystem::plan(int & timeout_timesteps)
{

    using namespace std::placeholders;
    int timestep = simulator.get_curr_timestep();

    std::packaged_task<bool()> task(std::bind(&BaseSystem::planner_wrapper, this));


    future = task.get_future();
    if (task_td.joinable()){
        task_td.join();
    }
    env->plan_start_time = std::chrono::steady_clock::now();
    task_td = std::thread(std::move(task));

    started = true;

    while (timestep + timeout_timesteps < simulation_time){

        if (future.wait_for(std::chrono::milliseconds(plan_time_limit)) == std::future_status::ready)
            {
                task_td.join();
                started = false;
                auto res = future.get();

                logger->log_info("planner returns", timestep + timeout_timesteps);
                return;
            }
        logger->log_info("planner timeout", timestep + timeout_timesteps);
        timeout_timesteps += 1;
    }

    //
}


bool BaseSystem::planner_initialize()
{
    std::cout << "planner init\n";
    using namespace std::placeholders;
    // this is TaskScheduler::initialize
    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    env->plan_start_time = std::chrono::steady_clock::now();
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::milliseconds(preprocess_time_limit)) == std::future_status::ready)
    {
        init_td.join();
        return true;
    }

    init_td.detach();
    return false;
}


void BaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;
    if (succ)
    {
        logger->log_info("Preprocessing success", simulator.get_curr_timestep());
    } 
    else
    {
        logger->log_fatal("Preprocessing timeout", simulator.get_curr_timestep());
    }
    logger->flush();
}


void BaseSystem::simulate(int simulation_time)
{
    initialize();
    cout<<"finish init\n";

    this->simulation_time = simulation_time;

    vector<Action> all_wait_actions(num_of_agents, Action::NA);

    while (simulator.get_curr_timestep() < simulation_time)
    {
        cout << "current timestep " << simulator.get_curr_timestep() << endl;

        // sync environment and run planner
        sync_shared_env();
        auto start = std::chrono::steady_clock::now();
        int timeout_timesteps = 0;
        plan(timeout_timesteps);
        auto end = std::chrono::steady_clock::now();

        // If planner timed out and produced extra timesteps to advance
        if (timeout_timesteps > 0)
        {
            for (int i = 0; i < timeout_timesteps; ++i)
            {
                simulator.move(all_wait_actions);
                for (int a = 0; a < num_of_agents; a++)
                {
                    if (!env->goal_locations[a].empty())
                        solution_costs[a]++;
                }
            }

            double planner_time = std::chrono::duration<double>(end - start).count();
            planner_times.push_back(planner_time);

            /* Begin storing final per-timestep scheduler and planner metrics. */
            TimeStepMetric metric;
            metric.SchedulerSolveTime = last_scheduler_timing.solve_time;
            metric.SchedulerGuidePathTime = last_scheduler_timing.guide_path_time;
            metric.PlannerTime = planner_time;
            time_step_metrics.push_back(metric);
            /* End storing final per-timestep scheduler and planner metrics. */

            // break; // exit simulation loop after advancing timeout timesteps
        }

        // Normal timestep: count goal-reaching agents before move
        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // Move according to proposed actions and update state
        vector<State> curr_states = simulator.move(proposed_actions);

        double planner_time = std::chrono::duration<double>(end - start).count();
        planner_times.push_back(planner_time);

        /* Begin storing per-timestep scheduler and planner metrics. */
        TimeStepMetric metric;
        metric.SchedulerSolveTime = last_scheduler_timing.solve_time;
        metric.SchedulerGuidePathTime = last_scheduler_timing.guide_path_time;
        metric.PlannerTime = planner_time;
        time_step_metrics.push_back(metric);
        /* End storing per-timestep scheduler and planner metrics. */

        // update tasks using moved states
        task_manager.update_tasks(curr_states, proposed_schedule, simulator.get_curr_timestep());
    }
}


void BaseSystem::initialize()
{
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;

    
    // // bool succ = load_records(); // continue simulating from the records
    // timestep = 0;
    // curr_states = starts;

    int timestep = simulator.get_curr_timestep();

    //planner initilise before knowing the first goals
    bool planner_initialize_success= planner_initialize();
    
    log_preprocessing(planner_initialize_success);
    if (!planner_initialize_success)
        _exit(124);

    // initialize_goal_locations();
    task_manager.reveal_tasks(timestep); //this also intialize env->new_tasks

    sync_shared_env();

    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }

    proposed_actions.resize(num_of_agents, Action::WA);
    proposed_schedule.resize(num_of_agents, -1);
}


void BaseSystem::saveResults(const string &fileName, int screen) const
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF";
    //js["version"] = "2024 LoRR";

    // std::string feasible = fast_mover_feasible ? "Yes" : "No";
    // js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    js["numTaskFinished"] = task_manager.num_of_task_finish;
    int makespan = 0;
    if (num_of_agents > 0)
    {
        makespan = solution_costs[0];
        for (int a = 1; a < num_of_agents; a++)
        {
            if (solution_costs[a] > makespan)
            {
                makespan = solution_costs[a];
            }
        }
    }
    js["makespan"] = makespan;

    js["numPlannerErrors"] = simulator.get_number_errors();
    js["numScheduleErrors"] = task_manager.get_number_errors();

    js["numEntryTimeouts"] = total_timetous;
    js["schedulerHierarchyBuildTime"] = last_scheduler_timing.hierarchy_build_time;
    js["schedulerHierarchyLevelNodeCounts"] = last_scheduler_timing.hierarchy_level_node_counts;

    /* Begin planner timing output selection. */
    if (kUseTimeStepMetricsOutput)
    {
        json time_step_metrics_json = json::array();
        for (const auto &metric : time_step_metrics)
        {
            json step = json::object();
            step["SchedulerSolveTime"] = metric.SchedulerSolveTime;
            step["SchedulerGuidePathTime"] = metric.SchedulerGuidePathTime;
            step["PlannerTime"] = metric.PlannerTime;
            time_step_metrics_json.push_back(step);
        }
        js["timeStepMetrics"] = time_step_metrics_json;
    }
    else
    {
        json planning_times = json::array();
        for (double time: planner_times)
            planning_times.push_back(time);
        js["plannerTimes"] = planning_times;
    }
    /* End planner timing output selection. */

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        js["start"] = simulator.starts_to_json();
    }
    
    if (screen <= 2)
    {
        js["actualPaths"] = simulator.actual_path_to_json();
    }

    if (screen <=1)
    {
        js["plannerPaths"] = simulator.planned_path_to_json();

        // json planning_times = json::array();
        // for (double time: planner_times)
        //     planning_times.push_back(time);
        // js["plannerTimes"] = planning_times;

        // Save errors
        js["errors"] = simulator.action_errors_to_json();

        //actual schedules
        json aschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager.actual_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
            }  
            aschedules.push_back(schedules);
        }

        js["actualSchedule"] = aschedules;

        //planned schedules
        json pschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto schedule : task_manager.planner_schedule[i])
            {
                if (!first)
                {
                    schedules+= ",";
                } 
                else 
                {
                    first = false;
                }

                schedules+=std::to_string(schedule.first);
                schedules+=":";
                int tid = schedule.second;
                schedules+=std::to_string(tid);
                
            }  
            pschedules.push_back(schedules);
        }

        js["plannerSchedule"] = pschedules;

        // Save errors
        json schedule_errors = json::array();
        for (auto error: task_manager.schedule_errors)
        {
            std::string error_msg;
            int t_id;
            int agent1;
            int agent2;
            int timestep;
            std::tie(error_msg,t_id,agent1,agent2,timestep) = error;
            json e = json::array();
            e.push_back(t_id);
            e.push_back(agent1);
            e.push_back(agent2);
            e.push_back(timestep);
            e.push_back(error_msg);
            schedule_errors.push_back(e);
        }

        js["scheduleErrors"] = schedule_errors;

        // Save events
        json event = json::array();
        for(auto e: task_manager.events)
        {
            json ev = json::array();
            int timestep;
            int agent_id;
            int task_id;
            int seq_id;
            std::tie(timestep,agent_id,task_id,seq_id) = e;
            ev.push_back(timestep);
            ev.push_back(agent_id);
            ev.push_back(task_id);
            ev.push_back(seq_id);
            event.push_back(ev);
        }
        js["events"] = event;

        // Save all tasks
        json tasks = task_manager.to_json(map.cols);
        js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;

}


