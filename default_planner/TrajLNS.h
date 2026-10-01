#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include "heuristics.h"
#include <iostream>

#include <set>
#include <unordered_map>
#include <cstdint>
#include <memory>

namespace DefaultPlanner{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

extern std::vector<HeuristicTable> global_heuristictable;
extern Neighbors global_neighbors;

struct FW_Metric{
    int id;
    int deviation;
    int last_replan_t;
    int rand;

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

struct FlowHeuristic{
    HeuristicTable* h; 
    int target;
    int origin;
    pqueue_min_of open;
    MemoryPool mem;


    bool empty(){
        return mem.generated() == 0;
    }
    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;

    TimePoint start_time;
    int t_ms=0;

    std::vector<Traj> trajs;

    std::vector<std::pair<int,int>> deviation_agents;

    std::vector<Int4> flow;
    std::vector<HeuristicTable>& heuristics;
    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics;
    Neighbors& neighbors;

    // USE_LOCAL_PATH_BFS state (see ai/planner_local_bfs_plan.md).
    // path_togo[i]: guide-path cell -> steps left to the goal along trajs[i].
    std::vector<std::unordered_map<int,int>> path_togo;
    // needs_replan[i]: agent was found too far from its path last step.
    std::vector<bool> needs_replan;
    // Scratch for the per-agent local BFS, shared by all agents. A cell's
    // bfs_dist/bfs_moves are only valid when bfs_stamp[cell] == bfs_cur_stamp.
    std::vector<uint32_t> bfs_stamp;
    std::vector<int> bfs_dist;
    std::vector<uint8_t> bfs_moves;
    std::vector<int> bfs_queue;
    uint32_t bfs_cur_stamp = 0;
    // One A* search pool per guide-path worker thread (GUIDE_PATH_THREADS > 1),
    // created on first use. unique_ptr because MemoryPool can't be copied.
    std::vector<std::unique_ptr<MemoryPool>> guide_pools;

    // Stuck-agent diagnosis, set by causalPIBT each decision for each agent:
    // pibt_trapped[i]: no neighbour scored strictly better than waiting
    //   (other agents ignored), so the score itself keeps the agent still.
    // pibt_local[i]: scored with the local path BFS (else Manhattan).
    std::vector<uint8_t> pibt_trapped;
    std::vector<uint8_t> pibt_local;
    // Per-step counters for the planner log line.
    int bfs_no_path = 0;   // agent had no usable path, used Manhattan
    int bfs_too_far = 0;   // no path cell within the radius, used Manhattan
    long long bfs_cells = 0;


    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    TrajLNS(SharedEnvironment* env, std::vector<HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(heuristics),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors),
        path_togo(env->num_of_agents), needs_replan(env->num_of_agents, false),
        bfs_stamp(env->map.size(), 0), bfs_dist(env->map.size(), 0),
        bfs_moves(env->map.size(), 0),
        pibt_trapped(env->num_of_agents, 0), pibt_local(env->num_of_agents, 0){
        };


    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};

    

};
}
#endif