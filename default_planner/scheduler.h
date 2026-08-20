#ifndef SCHEDULER
#define SCHEDULER

#include "Types.h"
#include "SharedEnv.h"
#include "heuristics.h"
#include <random>
#include <thread>
#include <future>
#include <lemon/list_graph.h>
#include <lemon/network_simplex.h>

using namespace lemon;

namespace DefaultPlanner{

/* Begin scheduler timing metrics. */
struct ScheduleTiming
{
	double solve_time = 0.0;
	double guide_path_time = 0.0;
	double hierarchy_build_time = 0.0;
	std::vector<int> hierarchy_level_node_counts;
	// Sum, over every guide path built this scheduler call, of (path length
	// in edges) and (sum of that path's per-edge arc costs). Computed
	// unconditionally every call (not just when use_traffic is on / the
	// planner-seed gate is open) so the two solvers are comparable on every
	// run, including ones that never pass --useTraffic. See
	// ai/guide_path_metric.md for why length and cost currently coincide for
	// solver 6 but can diverge for solver 1.
	double guide_path_length_sum = 0.0;
	double guide_path_cost_sum = 0.0;
	// How many agents this scheduler call assigned via the within-coarse-node
	// local matcher (LocalNodeMatch.h) vs. via the coarse flow solve -- solver
	// 6 only (schedule_plan_flow_reduced); always 0/0 for other solvers,
	// which don't have a local-matching step. See ai/local_node_matching.md.
	int local_node_match_count = 0;
	int flow_match_count = 0;
	// Wall-clock time (seconds) spent inside Step 1's local-matcher calls
	// (match_local_node_exact) -- solver 6 only, 0 for other solvers. See
	// ai/local_node_matching.md.
	double local_match_time = 0.0;
};

void set_last_timing(double solve_time, double guide_path_time,
                      double guide_path_length_sum = 0.0, double guide_path_cost_sum = 0.0);
// Same as set_last_timing, but for solver 6 (schedule_plan_flow_reduced),
// which additionally reports how long the one-time coarsened-hierarchy
// build took and how many nodes each hierarchy level has.
void set_last_reduced_timing(double solve_time,
						     double guide_path_time,
						     double hierarchy_build_time,
						     const std::vector<int>& hierarchy_level_node_counts,
						     double guide_path_length_sum = 0.0,
						     double guide_path_cost_sum = 0.0,
						     int local_node_match_count = 0,
						     int flow_match_count = 0,
						     double local_match_time = 0.0);
ScheduleTiming get_last_timing();
/* End scheduler timing metrics. */

// `solver` gates the reduced-hierarchy build (see MapCoarsenV1.h) to solver 6
// only -- it used to run unconditionally for every solver, which meant
// solvers 1-5 paid its build cost (minutes, on large maps) despite never
// using the result.
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env, int solver);

void schedule_plan_raw(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);
void schedule_plan_matching(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only, int maximum_edges);
void schedule_plan_flow(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
void schedule_plan_flow_reduced(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<Double4> background_flow, bool use_traffic, bool new_only);
void schedule_plan_h(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, bool new_only);

void schedule_plan_flow_hist(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env, std::vector<pair<double,double>>& background_flow, bool new_only);

unordered_map<int,unordered_map<int,int>> compute_heuristics(SharedEnvironment* env, std::vector<Int4> background_flow, vector<pair<int,int>> current_id_assignment, unordered_map<int,list<int>> task_loc_ids, int max_num_tasks);
void printDIMACS(ListDigraph& g, ListDigraph::Node source, ListDigraph::Node sink, vector<ListDigraph::Node>& workers, vector<ListDigraph::Node>& tasks, ListDigraph::ArcMap<int>& capacity, ListDigraph::ArcMap<double>& cost);
bool isTaskNode(ListDigraph::Node node, ListDigraph& g, ListDigraph::Node sink);

unordered_map<int,list<int>> get_guide_path();

/* Begin unconditional guide-path capture for offline dumping.
 * agent_guide_path (above) is the planner-visible seed -- gated by
 * use_traffic/curr_timestep, and touching that gate changes what the real
 * planner actually seeds its trajectories from (see ai/guide_path_visualisation.md,
 * "Live per-timestep tooling"). This is a separate, dump-only capture of the
 * same guide-path data solver 1/6 already compute unconditionally every
 * call (the walk / coarse-to-fine lift, which feeds GuidePathLengthSum
 * regardless of the seed gate) -- populated only when explicitly enabled,
 * so runs that don't ask for it pay no extra cost beyond the boolean check. */
void set_dump_all_guide_paths(bool enable);
unordered_map<int,list<int>> get_all_guide_paths();
/* End unconditional guide-path capture for offline dumping. */

}

#endif