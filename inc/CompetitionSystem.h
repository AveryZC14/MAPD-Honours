#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "Entry.h"
#include "Logger.h"
#include "TaskManager.h"
#include <pthread.h>
#include <future>
#include "Simulator.h"

/* Begin per-timestep metrics model. */
struct TimeStepMetric
{
    double SchedulerSolveTime = 0.0;
    double SchedulerGuidePathTime = 0.0;
    double PlannerTime = 0.0;
    // Sum, over every guide path the scheduler built this timestep, of path
    // length (edges) and path cost (sum of traversed arc costs -- equals
    // length unless --useTraffic is on, and even then only for solver 1; see
    // ai/guide_path_metric.md). Both solvers populate these unconditionally
    // now, independent of whether traffic-seeding is active, so they're
    // comparable across every run.
    double GuidePathLengthSum = 0.0;
    double GuidePathCostSum = 0.0;
};
/* End per-timestep metrics model. */

class BaseSystem
{
public:
    Logger* logger = nullptr;

    /* Begin planner output format switch. */
    static constexpr bool kUseTimeStepMetricsOutput = true;
    /* End planner output format switch. */

    /* Begin per-timestep guide-path / agent-position CSV dump switch.
     * Flip to true to stream every timestep's scheduler guide paths and
     * every agent's current position (dense -- one row per agent per
     * timestep) to CSV, for later offline visualisation. Guide paths are
     * read from DefaultPlanner::get_all_guide_paths() -- a dump-only
     * capture (agent_guide_path_all, gated only by this same switch via
     * set_dump_all_guide_paths() below), separate from the planner-seed
     * map (agent_guide_path/get_guide_path(), still gated by
     * use_traffic/curr_timestep as before, untouched by this feature -- see
     * ai/guide_path_visualisation.md, "Unconditional capture" for why that
     * separation matters and how it was verified not to change simulation
     * output). Still only non-empty for agents the scheduler actually
     * reassigned that call. Off by default: streaming I/O inside the timed
     * simulation loop would otherwise skew planner/scheduler timing
     * metrics. Does not attempt solver-1-vs-solver-6 comparison -- a single
     * run only ever exercises one scheduler solver, and two separate runs
     * diverge in simulated world state past timestep 0 once assignments
     * differ, so this is for visualising one run's own behaviour over time,
     * not a frozen-state solver comparison (that's what
     * map_reduction_test/dump_guide_paths.cpp is for). */
    static constexpr bool kDumpPerTimestepPaths = false;
    static constexpr const char* kGuidePathsCsvPath = "outputs/guide_paths_per_timestep.csv";
    static constexpr const char* kAgentPositionsCsvPath = "outputs/agent_positions_per_timestep.csv";
    /* End per-timestep guide-path / agent-position CSV dump switch. */

	BaseSystem(Grid &grid, Entry* planner, std::vector<int>& start_locs, std::vector<list<int>>& tasks, ActionModel* model):
      map(grid), planner(planner), env(planner->env),
      task_manager(tasks, start_locs.size()), simulator(grid,start_locs,model)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);

        for (size_t i = 0; i < start_locs.size(); i++)
            {
                if (grid.map[start_locs[i]] == 1)
                    {
                        cout<<"error: agent "<<i<<"'s start location is an obstacle("<<start_locs[i]<<")"<<endl;
                        exit(0);
                    }
                starts[i] = State(start_locs[i], 0, 0);
            }

 //        int task_id = 0;
 // for (auto& task_location: tasks)
 //        {
 //            all_tasks.emplace_back(task_id++, task_location);
 //            task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().locations.front());
 //            //task_queue.emplace_back(task_id++, task_location);
 //        }
 //        num_of_agents = start_locs.size();
 //        starts.resize(num_of_agents);
 //        for (size_t i = 0; i < start_locs.size(); i++)
 //        {
 //            starts[i] = State(start_locs[i], 0, 0);
 //        }
    };

	virtual ~BaseSystem()
    {
        //safely exit: wait for join the thread then delete planner and exit
        if (started)
        {
            task_td.join();
        }
        if (planner != nullptr)
        {
            delete planner;
        }
    };

    void set_num_tasks_reveal(float num){task_manager.set_num_tasks_reveal(num);};
    void set_plan_time_limit(int limit){plan_time_limit = limit;};
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};
    void set_log_level(int level){log_level = level;};
    void set_logger(Logger* logger){
        this->logger = logger;
        task_manager.set_logger(logger);
    }

    void simulate(int simulation_time);
    void plan(int & timeout_timesteps);
    bool planner_wrapper();

    //void saveSimulationIssues(const string &fileName) const;
    void saveResults(const string &fileName, int screen) const;


protected:
    Grid map;
    int simulation_time;

    vector<Action> proposed_actions;
    vector<int> proposed_schedule;

    int total_timetous = 0;


    std::future<bool> future;
    std::thread task_td;
    bool started = false;

    Entry* planner;
    SharedEnvironment* env;

    int preprocess_time_limit=10;

    int plan_time_limit = 3;


    vector<State> starts;
    int num_of_agents;

    int log_level = 1;

    // tasks that haven't been finished but have been revealed to agents;

    vector<list<std::tuple<int,int,std::string>>> events;

    //for evaluation
    vector<int> solution_costs;
    list<double> planner_times; 
    list<TimeStepMetric> time_step_metrics;
    bool fast_mover_feasible = true;

    /* Begin cached scheduler timing for the current planner step. */
    DefaultPlanner::ScheduleTiming last_scheduler_timing;
    /* End cached scheduler timing for the current planner step. */


    void initialize();
    bool planner_initialize();


    TaskManager task_manager;
    Simulator simulator;
    // deque<Task> task_queue;
    virtual void sync_shared_env();

    void move(vector<Action>& actions);
    bool valid_moves(vector<State>& prev, vector<Action>& next);

    void log_preprocessing(bool succ);
    // void log_event_assigned(int agent_id, int task_id, int timestep);
    // void log_event_finished(int agent_id, int task_id, int timestep);

};


// class TaskAssignSystem : public BaseSystem
// {
// public:
// 	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model):
//         BaseSystem(grid, planner, model)
//     {
//         int task_id = 0;
//         for (auto& task_location: tasks)
//         {
//             all_tasks.emplace_back(task_id++, task_location);
//             task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().locations.front());
//             //task_queue.emplace_back(task_id++, task_location);
//         }
//         num_of_agents = start_locs.size();
//         starts.resize(num_of_agents);
//         for (size_t i = 0; i < start_locs.size(); i++)
//         {
//             starts[i] = State(start_locs[i], 0, 0);
//         }
//     };

// 	~TaskAssignSystem(){};


// private:
//     deque<Task> task_queue;

// 	void update_tasks();
// };


