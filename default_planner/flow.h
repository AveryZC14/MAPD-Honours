
#ifndef flow_hpp
#define flow_hpp
#include "Types.h"
#include "search.h"
#include "TrajLNS.h"
#include "heuristics.h"

#include <random>
#include <unordered_set>

namespace DefaultPlanner{

//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent);

void add_traj(TrajLNS& lns, int agent);


void get_deviation(TrajLNS& lns);

void update_fw_metrics(TrajLNS& lns);

void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit);

void update_dist_2_path(TrajLNS& lns, int i);

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount);

//update traj and distance table for agent i
void update_traj(TrajLNS& lns, int i);

//replace agent i's traj with an already-searched one and register it in the flow
void commit_traj(TrajLNS& lns, int i, Traj& new_traj, const s_node& goal_node);

//rebuild path cell -> steps-left-to-goal lookup for agent i (USE_LOCAL_PATH_BFS)
void update_path_togo(TrajLNS& lns, int i);


}
#endif