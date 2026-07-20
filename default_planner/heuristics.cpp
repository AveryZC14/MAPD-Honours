
#include "heuristics.h"
#include <queue>
#include <list>
#include <unordered_map>
#include <algorithm>

namespace DefaultPlanner{

std::vector<HeuristicTable> global_heuristictable;
Neighbors global_neighbors;

namespace {
    std::list<int> lru_order; // most-recently-used at the front
    std::unordered_map<int, std::list<int>::iterator> lru_pos;
}

void touch_heuristic_lru(int goal_location, SharedEnvironment* env){
    auto it = lru_pos.find(goal_location);
    if (it != lru_pos.end()){
        lru_order.erase(it->second);
    }
    lru_order.push_front(goal_location);
    lru_pos[goal_location] = lru_order.begin();

    // Bound total resident heuristic-table memory to a fixed budget instead
    // of letting it grow with every distinct goal location ever requested.
    const size_t bytes_per_table = global_heuristictable.empty() ? 1 : env->map.size() * sizeof(int);
    const size_t target_budget_bytes = 1ull * 1024 * 1024 * 1024; // ~1GB resident cap
    const size_t max_tables = std::max<size_t>(16, target_budget_bytes / std::max<size_t>(1, bytes_per_table));

    while (lru_order.size() > max_tables){
        int evict = lru_order.back();
        lru_order.pop_back();
        lru_pos.erase(evict);
        if (evict >= 0 && evict < static_cast<int>(global_heuristictable.size())){
            HeuristicTable& old = global_heuristictable[evict];
            std::vector<int>().swap(old.htable);
            old.open.clear();
        }
    }
}

void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (global_heuristictable.size()==0){
		global_heuristictable.resize(env->map.size());
		init_neighbor(env);
	}

}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.htable.resize(env->map.size(),MAX_TIMESTEP);
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location,0, 0);
	ht.htable[goal_location] = 0;
	ht.open.push_back(root);  // add root to open
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		if (ht.htable[source] < MAX_TIMESTEP) return ht.htable[source];

		std::vector<int> neighbors;
		int cost, diff;
		while (!ht.open.empty())
		{
			HNode curr = ht.open.front();
			ht.open.pop_front();

			
			getNeighborLocs(ns,neighbors,curr.location);

			
			for (int next : neighbors)
			{
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				if (cost >= ht.htable[next] )
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable[next] = cost;
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}

int get_h(SharedEnvironment* env, int source, int target){
	if (global_heuristictable.empty()){
		init_heuristics(env);
	}

	if (global_heuristictable.at(target).empty()){
		init_heuristic(global_heuristictable.at(target),env,target);
	}
	touch_heuristic_lru(target, env);

	return get_heuristic(global_heuristictable.at(target), env, source, &global_neighbors);
}



void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	if (dp.dist2path.empty())
		dp.dist2path.resize(env->map.size(), d2p(0,-1,MAX_TIMESTEP,MAX_TIMESTEP));
	
	dp.open.clear();
	dp.label++;

    int togo = 0;
    for(int i = path.size()-1; i>=0; i--){
        auto p = path[i];
		assert(dp.dist2path[p].label != dp.label || dp.dist2path[p].cost == MAX_TIMESTEP);
		dp.open.emplace_back(dp.label,p,0,togo);
		dp.dist2path[p] = {dp.label,p,0,togo};
		togo++;
    }

}

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	if (dp.dist2path[source].label == dp.label && dp.dist2path[source].cost < MAX_TIMESTEP){
		// std::cout<<dp.dist2path[source].first<<" "<<dp.dist2path[source].second<<std::endl;

		return std::make_pair(dp.dist2path[source].cost, dp.dist2path[source].togo);
	}

	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();



		getNeighborLocs(ns,neighbors,curr.id);

		for (int next_location : neighbors)
		{

			cost = curr.cost + 1;

			if (dp.dist2path[next_location].label == dp.label && cost >= dp.dist2path[next_location].cost )
				continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path[next_location] = {dp.label,next_location,cost,curr.togo};
			
		}
		if (source == curr.id){
			// std::cout<<curr.second.first<<" "<<curr.second.second<<std::endl;
			return std::make_pair(curr.cost, curr.togo);
		}
	}

	return std::make_pair(MAX_TIMESTEP,0);
}

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}



}
