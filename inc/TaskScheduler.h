#pragma once
#include "Tasks.h"
#include "SharedEnv.h"
#include "scheduler.h"


class TaskScheduler
{
    public:
        SharedEnvironment* env;

        TaskScheduler(SharedEnvironment* env): env(env){};
        TaskScheduler(){env = new SharedEnvironment();};
        virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);

        /* Begin scheduler timing snapshot access. */
        DefaultPlanner::ScheduleTiming get_last_timing() const { return last_timing; }
        /* End scheduler timing snapshot access. */

        void set_flow(std::vector<DefaultPlanner::Double4> flow);

        void set_use_traffic(bool use_traffic);
        void set_new_only(bool new_only);
        void set_solver(int solver);
        void set_max_matching_edges(int max_matching_edges);

        std::vector<DefaultPlanner::Double4> background_flow;

        /* Begin scheduler timing snapshot state. */
        DefaultPlanner::ScheduleTiming last_timing;
        /* End scheduler timing snapshot state. */

        bool use_traffic = false;
        bool new_only = false;
        int solver = 1; //1 matching, 2 flow
        int max_matching_edges = INT_MAX;

};