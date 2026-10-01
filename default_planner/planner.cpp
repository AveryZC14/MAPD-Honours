#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include <thread>
#include <atomic>


namespace DefaultPlanner{

    //default planner data
    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    std::mt19937 mt1;
    TrajLNS trajLNS;

    // USE_LOCAL_PATH_BFS: agent the guide-path loop starts from next step
    int guide_path_start = 0;
    // stuck-agent counter: location last decision, and decisions in a row
    // spent there while having a goal
    std::vector<int> last_loc;
    std::vector<int> still_steps;

    // GUIDE_PATH_DEBUG_CHECKS: a new guide path must start at the agent,
    // end at its goal, and move one open cell at a time.
    void check_path_or_die(SharedEnvironment* env, int agent, const Traj& traj, int goal)
    {
        const int start = env->curr_states[agent].location;
        bool ok = !traj.empty() && traj.front() == start && traj.back() == goal;
        for (size_t j = 1; ok && j < traj.size(); j++)
            ok = validateMove(traj[j-1], traj[j], env) && traj[j] != traj[j-1];
        if (!ok)
        {
            cout << "GUIDE PATH CHECK FAILED: agent " << agent << " start " << start
                 << " goal " << goal << " path size " << traj.size() << endl;
            exit(1);
        }
    }

    // GUIDE_PATH_DEBUG_CHECKS: rebuild the congestion map from every agent's
    // current path and compare it with trajLNS.flow, to catch a path added or
    // removed twice.
    void check_flow_or_die(SharedEnvironment* env)
    {
        std::vector<Int4> rebuilt(env->map.size(), Int4({0,0,0,0}));
        for (int a = 0; a < env->num_of_agents; a++)
        {
            const Traj& traj = trajLNS.trajs[a];
            for (size_t j = 1; j < traj.size(); j++)
                rebuilt[traj[j-1]].d[get_d(traj[j] - traj[j-1], env)] += 1;
        }
        for (size_t c = 0; c < rebuilt.size(); c++)
            for (int d = 0; d < 4; d++)
                if (rebuilt[c].d[d] != trajLNS.flow[c].d[d])
                {
                    cout << "FLOW CHECK FAILED: cell " << c << " dir " << d << " rebuilt "
                         << rebuilt[c].d[d] << " flow " << trajLNS.flow[c].d[d] << endl;
                    exit(1);
                }
        cout << "flow check ok" << endl;
    }

    // Guide-path threads actually used: GUIDE_PATH_THREADS, reduced so the
    // extra search pools stay within GUIDE_PATH_EXTRA_POOL_MB.
    int guide_path_threads_used(SharedEnvironment* env)
    {
        const long long pool_mb = std::max<long long>(1, (long long)env->map.size() * (long long)sizeof(s_node) / (1024 * 1024));
        const long long extra = std::min<long long>(GUIDE_PATH_THREADS - 1, GUIDE_PATH_EXTRA_POOL_MB / pool_mb);
        return 1 + (int)std::max<long long>(0, extra);
    }

    // Stage 2 with GUIDE_PATH_THREADS workers (see
    // ai/parallel_guide_paths_plan.md). Scheduler-provided paths are taken on
    // this thread first. The remaining agents that need a path are searched in
    // parallel against trajLNS.flow as it is now (workers only read it), until
    // the deadline; then every finished path is committed here, in order.
    // Returns the number of paths built.
    int build_guide_paths_parallel(SharedEnvironment* env, TimePoint end_time, int start_agent,
                                   unordered_map<int,list<int>>& agent_guide_path, int& paths_from_scheduler,
                                   int& abandoned_searches)
    {
        const int n_agents = env->num_of_agents;
        int paths_built = 0;
        std::vector<int> todo;
        for (int k = 0; k < n_agents; k++)
        {
            const int i = (start_agent + k) % n_agents;
            if (!require_guide_path[i])
                continue;
            auto it = agent_guide_path.find(i);
            if (it != agent_guide_path.end())
            {
                Traj seeded(it->second.begin(), it->second.end());
                if (GUIDE_PATH_DEBUG_CHECKS)
                    check_path_or_die(env, i, seeded, trajLNS.tasks[i]);
                commit_traj(trajLNS, i, seeded, s_node());
                paths_from_scheduler++;
                paths_built++;
            }
            else
                todo.push_back(i);
        }
        if (todo.empty())
            return paths_built;

        const int n_threads = std::min<int>(guide_path_threads_used(env), todo.size());
        // thread 0 uses the planner's own pool, the rest get one each
        while ((int)trajLNS.guide_pools.size() < n_threads - 1)
            trajLNS.guide_pools.push_back(std::make_unique<MemoryPool>(env->map.size()));

        // GUIDE_PATH_IGNORE_CONGESTION: search against an all-zero congestion
        // map (plain shortest paths). Paths are still registered in lns.flow.
        static std::vector<Int4> no_congestion;
        if (GUIDE_PATH_IGNORE_CONGESTION && no_congestion.size() != env->map.size())
            no_congestion.assign(env->map.size(), Int4({0,0,0,0}));

        std::vector<Traj> results(todo.size());
        std::vector<s_node> goal_nodes(todo.size());
        std::atomic<size_t> next(0);
        auto worker = [&](int t)
        {
            MemoryPool& pool = t == 0 ? trajLNS.mem : *trajLNS.guide_pools[t - 1];
            while (std::chrono::steady_clock::now() < end_time)
            {
                const size_t idx = next.fetch_add(1);
                if (idx >= todo.size())
                    break;
                const int i = todo[idx];
                const int goal = trajLNS.tasks[i];
                goal_nodes[idx] = astar(env, GUIDE_PATH_IGNORE_CONGESTION ? no_congestion : trajLNS.flow,
                                        trajLNS.heuristics[goal], results[idx], pool,
                                        env->curr_states[i].location, goal, &(trajLNS.neighbors), &end_time);
            }
        };
        std::vector<std::thread> threads;
        for (int t = 0; t < n_threads; t++)
            threads.emplace_back(worker, t);
        for (auto& th : threads)
            th.join();

        // every index below `next` was taken by a worker; a search that hit
        // the deadline comes back with an empty traj and isn't committed
        const size_t taken = std::min(next.load(), todo.size());
        size_t first_missed = todo.size();
        for (size_t idx = 0; idx < taken; idx++)
        {
            const int i = todo[idx];
            if (results[idx].empty())
            {
                abandoned_searches++;
                first_missed = std::min(first_missed, idx);
                continue;
            }
            if (GUIDE_PATH_DEBUG_CHECKS)
                check_path_or_die(env, i, results[idx], trajLNS.tasks[i]);
            commit_traj(trajLNS, i, results[idx], goal_nodes[idx]);
            paths_built++;
        }
        // Next step starts after the agents tried this step, so an agent whose
        // search was abandoned goes to the back of the queue instead of
        // blocking the same slots every step.
        if (first_missed < todo.size())
        {
            cout << "compute initial stop until " << todo[first_missed] << endl;
            guide_path_start = todo[taken % todo.size()];
        }
        return paths_built;
    }


    // std::vector<Int4> get_flow() 
    // {
    //     return trajLNS.flow;
    // }

    std::vector<Double4> get_opened_flow(SharedEnvironment* env)
    {
        double decay = 1;
        std::vector<Double4> background_flow(env->map.size(),Double4{0,0,0,0});
        //for (int i_task=0 ; i_task < env->task_pool.size() ;i_task++)
        for (auto task: env->task_pool)
        {
            if (task.second.idx_next_loc > 0) //task opened
            {
                int agent = task.second.agent_assigned;
                if (trajLNS.trajs[agent].empty())
                    continue;
                int loc, prev_loc, diff, d;
                double current_cost = 1;
                for (int j = 1; j < trajLNS.trajs[agent].size(); j++)
                {
                    loc = trajLNS.trajs[agent][j];
                    prev_loc = trajLNS.trajs[agent][j-1];
                    diff = loc - prev_loc;
                    d = get_d(diff, env);

                    background_flow[prev_loc].d[d] += current_cost;
                    current_cost *= decay;
                }
            }
        }
        return background_flow;
    }

    /**
     * @brief Default planner initialization
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            // std::cout <<"planner initting\n";
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            last_loc.assign(env->num_of_agents, -1);
            still_steps.assign(env->num_of_agents, 0);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
            // std::cout <<"planner initted\n";
            return;
    };

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit,vector<Action> & actions, SharedEnvironment* env, unordered_map<int,list<int>> agent_guide_path)
    {

        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        cout << "plan limit " << time_limit <<endl;

        // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        // data sturcture for record the previous decision of each agent
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        const bool local_bfs = USE_MANHATTAN_HEURISTIC && USE_LOCAL_PATH_BFS;
        int stuck_agents = 0;
        trajLNS.bfs_no_path = 0;
        trajLNS.bfs_too_far = 0;
        trajLNS.bfs_cells = 0;

        // update the status of each agent and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            //initialise the shortest distance heuristic table for the goal location of the agent
            //skipped entirely under USE_MANHATTAN_HEURISTIC -- nothing reads these tables, so
            //building them would just waste the timestep's time budget for no benefit.
            if ( !USE_MANHATTAN_HEURISTIC && ( std::chrono::steady_clock::now() < end_time) ){
                for(int j=0; j<env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.at(goal_loc).empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                        count++;
                    }
                    // Mark this goal as most-recently-used whether its table was
                    // just built above or already existed, so the LRU eviction
                    // order in touch_heuristic_lru() reflects real usage
                    // regardless of which path filled it in.
                    touch_heuristic_lru(goal_loc, env);
                }
            }
            

            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // check if the agent need a guide path update, when the agent has no guide path or the guide path does not end at the goal location
            require_guide_path[i] = false;

            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
            {
                require_guide_path[i] = true;
            }


            // check if the agent completed the action in the previous timestep
            // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 
            if (decided[i].loc == -1){
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            // if (prev_states[i].location == decided[i].loc){
            //     decided[i].state = DONE::DONE;
            // }
            // if (decided[i].state == DONE::NOT_DONE){
            //     decision.at(decided[i].loc) = i;
            //     next_states[i] = State(decided[i].loc,-1,-1);
            // }
            decided[i].state = DONE::DONE;

            // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
            if(require_guide_path[i])
                p[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            // give priority bonus to the agent if the agent is in a deadend location
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }

            // agent was pushed too far off its path last step: give it a new
            // one from where it is now. Set after the priority reset above on
            // purpose -- this isn't a new goal, so it keeps its priority.
            if (local_bfs && trajLNS.needs_replan[i])
                require_guide_path[i] = true;

            // stuck-agent counter, logged below
            const int loc_now = env->curr_states[i].location;
            if (!env->goal_locations[i].empty() && loc_now == last_loc[i])
                still_steps[i]++;
            else
                still_steps[i] = 0;
            last_loc[i] = loc_now;
            if (still_steps[i] >= STUCK_AGENT_THRESHOLD)
                stuck_agents++;
        }
        TimePoint setup_done = std::chrono::steady_clock::now();

        // compute the congestion minimised guide path for the agents that need guide path update.
        // Under USE_LOCAL_PATH_BFS the loop starts where it stopped last step,
        // so the agents at the end of the list aren't always the ones left
        // without a path when time runs out.
        const int n_agents = env->num_of_agents;
        const bool parallel_paths = USE_MANHATTAN_HEURISTIC && GUIDE_PATH_THREADS > 1;
        const int start_agent = (local_bfs || parallel_paths) ? guide_path_start % n_agents : 0;
        int paths_built = 0;
        int paths_from_scheduler = 0;
        int abandoned_searches = 0;
        if (parallel_paths)
            paths_built = build_guide_paths_parallel(env, end_time, start_agent, agent_guide_path, paths_from_scheduler,
                                                     abandoned_searches);
        else
        for (int k = 0; k < n_agents; k++)
        {
            const int i = (start_agent + k) % n_agents;
            if (std::chrono::steady_clock::now() >end_time)
            {
                cout<<"compute initial stop until "<<i<<endl;
                if (local_bfs)
                    guide_path_start = i;
                break;
            }
            if (require_guide_path[i])
            {
                paths_built++;
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (agent_guide_path.find(i) != agent_guide_path.end())
                {
                    paths_from_scheduler++;
                    trajLNS.trajs[i].clear();
                    trajLNS.trajs[i].insert(trajLNS.trajs[i].end(), agent_guide_path[i].begin(), agent_guide_path[i].end());
                    add_traj(trajLNS,i);
                    // see USE_MANHATTAN_HEURISTIC comment in update_traj() (flow.cpp)
                    if (!USE_MANHATTAN_HEURISTIC)
                        update_dist_2_path(trajLNS,i);
                    else if (USE_LOCAL_PATH_BFS)
                        update_path_togo(trajLNS,i);
                }
                else
                {
                    update_traj(trajLNS, i);
                }
            }
        }
        TimePoint guide_done = std::chrono::steady_clock::now();

        if (GUIDE_PATH_DEBUG_CHECKS)
        {
            static int flow_check_count = 0;
            if (++flow_check_count % 100 == 0)
                check_flow_or_die(env);
        }

        // iterate and recompute the guide path to optimise traffic flow
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated,end_time);
        TimePoint fw_done = std::chrono::steady_clock::now();

        // sort agents based on the current priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        //pibt
        for (int i : ids)
        {
            // if (decided[i].state == DONE::NOT_DONE){
            //     continue;
            // }
            if (next_states[i].location==-1)
            {
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        TimePoint pibt_done = std::chrono::steady_clock::now();

        // Classify stuck agents by why they didn't move (see
        // ai/planner_local_bfs_plan.md, "Stuck-agent diagnosis"):
        // trapped = no neighbour beats waiting on score alone (the score pins
        // it); blocked = a better neighbour existed but the agent still
        // waited this decision (other agents in the way); moved = it moved
        // this decision. Each split by scoring: Manhattan vs. local path BFS.
        int stuck_trapped_manhattan = 0, stuck_trapped_local = 0;
        int stuck_blocked_manhattan = 0, stuck_blocked_local = 0;
        int stuck_moved = 0;
        static int decision_count = 0;
        decision_count++;
        const bool dump_samples = decision_count % 100 == 0;
        int samples = 0;
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (still_steps[i] < STUCK_AGENT_THRESHOLD)
                continue;
            const bool moved = next_states[i].location != prev_states[i].location;
            const bool local = trajLNS.pibt_local[i];
            const char* cls;
            if (moved) { stuck_moved++; cls = "moved"; }
            else if (trajLNS.pibt_trapped[i]) {
                if (local) { stuck_trapped_local++; cls = "trapped_local"; }
                else { stuck_trapped_manhattan++; cls = "trapped_manhattan"; }
            }
            else {
                if (local) { stuck_blocked_local++; cls = "blocked_local"; }
                else { stuck_blocked_manhattan++; cls = "blocked_manhattan"; }
            }
            if (dump_samples && samples < 30)
            {
                samples++;
                const int loc = prev_states[i].location;
                const int goal = trajLNS.tasks[i];
                cout << "stuck sample: decision " << decision_count << " agent " << i
                     << " class " << cls << " still " << still_steps[i]
                     << " loc " << loc / env->cols << " " << loc % env->cols
                     << " goal " << goal / env->cols << " " << goal % env->cols
                     << " has_path " << (!trajLNS.trajs[i].empty() && trajLNS.trajs[i].back() == goal)
                     << endl;
            }
        }
        cout << "stuck breakdown: trapped_manhattan " << stuck_trapped_manhattan
             << " trapped_local " << stuck_trapped_local
             << " blocked_manhattan " << stuck_blocked_manhattan
             << " blocked_local " << stuck_blocked_local
             << " moved " << stuck_moved << endl;

        auto ms = [](TimePoint a, TimePoint b){
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        cout << "planner stats: setup_ms " << ms(start_time, setup_done)
             << " guide_ms " << ms(setup_done, guide_done)
             << " paths_built " << paths_built
             << " paths_from_scheduler " << paths_from_scheduler
             << " guide_threads " << (parallel_paths ? guide_path_threads_used(env) : 1)
             << " abandoned_searches " << abandoned_searches
             << " fw_ms " << ms(guide_done, fw_done)
             << " pibt_ms " << ms(fw_done, pibt_done)
             << " pibt_reserve_ms " << pibt_time
             << " stuck_agents " << stuck_agents
             << " bfs_no_path " << trajLNS.bfs_no_path
             << " bfs_too_far " << trajLNS.bfs_too_far
             << " bfs_cells " << trajLNS.bfs_cells << endl;

        // post processing the targeted next location to turning or moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids)
        {
            //clear the decision table based on which agent has next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;

            if (next_states.at(id).location >=0)
            {
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }

            // post process the targeted next location to turning or moving actions
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }

        // for (int id=0;id < env->num_of_agents ; id++){
        //     if (!checked.at(id) && actions.at(id) == Action::FW){
        //         moveCheck(id,checked,decided,actions,prev_decision);
        //     }
        // }



        prev_states = next_states;
        return;

    };

    void plan_pibt(int time_limit,vector<Action> & actions, SharedEnvironment* env)
    {

        // calculate the time planner should stop optimsing traffic flows and return the plan.
        TimePoint start_time = std::chrono::steady_clock::now();
        //cap the time for distance to goal heuristic table initialisation to half of the given time_limit;
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        //traffic flow assignment end time, leave PIBT_RUNTIME_PER_100_AGENTS ms per 100 agent and TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE ms for computing pibt actions;
        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE); 
        cout << "plan limit " << time_limit <<endl;

        // recrod the initial location of each agent as dummy goals in case no goal is assigned to the agent.
        if (env->curr_timestep == 0){
            dummy_goals.resize(env->num_of_agents);
            for(int i=0; i<env->num_of_agents; i++)
            {
                dummy_goals.at(i) = env->curr_states.at(i).location;
            }
        }

        // data sturcture for record the previous decision of each agent
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        // update the status of each agent and prepare for planning
        int count = 0;
        for(int i=0; i<env->num_of_agents; i++)
        {
            // set the goal location of each agent
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                p[i] = p_copy[i];
            }
            
            // check if the agent completed the action in the previous timestep
            // if not, the agent is till turning towards the action direction, we do not need to plan new action for the agent
            assert(env->curr_states[i].location >=0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i; 
            if (decided[i].loc == -1){
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            decided[i].state = DONE::DONE;

            // reset the pibt priority if the agent reached prvious goal location and switch to new goal location
            if (!env->goal_locations[i].empty())
                p[i] = p[i]+1;

            // give priority bonus to the agent if the agent is in a deadend location
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1){
                p[i] = p[i] + 10;
            }
            
        }

        // sort agents based on the current priority
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return p.at(a) > p.at(b);
            }
        );

        // cout <<"time used: " <<  std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() <<endl;;
        //pibt
        for (int i : ids)
        {
            if (next_states[i].location==-1)
            {
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalPIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }
        
        // post processing the targeted next location to turning or moving actions
        actions.resize(env->num_of_agents);
        for (int id : ids)
        {
            //clear the decision table based on which agent has next_states
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;

            if (next_states.at(id).location >=0)
            {
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }

            // post process the targeted next location to turning or moving actions
            actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;

        }



        prev_states = next_states;
        return;

    };
}