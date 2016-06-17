#include "cbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <cmath>

#include <ctime>

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>

// #include "single_agent_search.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;

#define TIME_LIMIT 30.00

void CBSSearch::printPaths() {
	for (size_t t = 0; t < paths.size(); t++) {
		for (size_t i = 0; i < paths[t]->size(); i++) {
			cout << "TYPE " << t <<"AGENT " << i << " Path: ";
			int timestep = 0;
			for (vector<int>::const_iterator it = paths[t]->at(i).begin(); it != paths[t]->at(i).end(); ++it) {
				int idx = *it;
				Location loc;
				m_ml.idxToLocation(idx, loc);
				std::cout << timestep++ <<":" << idx << "(" << loc.x << "," << loc.y << "," << loc.z << ") ";
			}
			cout << endl;
		}
	}
}

// computes g_val based on current paths
inline double CBSSearch::compute_g_val() {
	double retVal = 0;
	for (int i = 0; i < num_of_types; i++) {
		if (paths[i]->at(0).size() - 1 > retVal) {
			retVal = paths[i]->at(0).size() - 1;
		}
	}
	return retVal;
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBSSearch::updatePaths(CBSNode* curr, CBSNode* root_node) {
	paths = paths_found_initially;
	vector<bool> updated(paths.size());
	/* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
	 * because younger nodes take into account ancesstors' nodes constraints. */
	for (size_t i = 0; i < paths.size(); i++)
		updated.push_back(false);
	while (curr != root_node) {
		if (updated[curr->type_id] == false) {
			paths[curr->type_id] = &(curr->path);
			updated[curr->type_id] = true;
		}
		curr = curr->parent;
	}
}

// Used in the GUI
void CBSSearch::updatePathsForExpTime(int t_exp) {
	if (t_exp > num_expanded || t_exp < 0)
		return;  // do nothing if there's no high-level node for the specified time_expanded

	CBSNode* t_exp_node = NULL;
	for (list < CBSNode* >::iterator it = popped_nodes.begin(); it != popped_nodes.end() && t_exp_node == NULL; it++)
		if ((*it)->time_expanded == t_exp)
		t_exp_node = *it;

	updatePaths(t_exp_node, dummy_start);
	printPaths();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool CBSSearch::updateCBSNode(CBSNode* leaf_node, CBSNode* root_node) {
	// extract all constraints on leaf_node->agent_id
	list < tuple<int, int, int> > constraints;
	int type_id = leaf_node->type_id;
	CBSNode* curr = leaf_node;
	while (curr != root_node) {
		if (curr->type_id == type_id)
			constraints.push_front(curr->constraint);
		curr = curr->parent;
	}

	// calc max_timestep
	int max_timestep = -1;
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) {
		if (get<2>(*it) > max_timestep) {
			max_timestep = get<2>(*it);
		}
	}

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
		cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));

	// find a path w.r.t cons_vec
	int makespan;
	for (makespan = leaf_node->parent->g_val; makespan < max_makespan; ++makespan) {
		if (search_engines[type_id]->findPath(cons_vec, paths, makespan) == true) {
			// update leaf's path to the one found
			leaf_node->path = vector<vector<int>>(*(search_engines[type_id]->getPath()));
			leaf_node->g_val = makespan;
			if (current_makespan < makespan) {
				current_makespan = makespan;
			}
			delete (cons_vec);
			vector<vector<int>>* temp_old_path = paths[type_id];
			paths[type_id] = &(leaf_node->path);
			leaf_node->h_val = computeNumOfCollidingTypes();
			//leaf_node->h_val = search_engines[type_id]->path_cost;
			cout << leaf_node->h_val << endl;
			paths[type_id] = temp_old_path;
			return true;
		}
	}
	return false;
}
////////////////////////////////////////////////////////////////////////////////

/*
  return agent_id's location for the given timestep
  Note -- if timestep is longer than its plan length,
		  then the location remains the same as its last cell)
 */
inline int CBSSearch::getAgentLocation(int type_id, int agent_id, size_t timestep) {
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[type_id]->at(agent_id).size())
		return paths[type_id]->at(agent_id)[paths[type_id]->at(agent_id).size() - 1];
	// otherwise, return its location for that timestep
	return paths[type_id]->at(agent_id)[timestep];
}

/*
  return true iff agent1 and agent2 switched locations at timestep [t,t+1]
 */
inline bool CBSSearch::switchedLocations(int type1_id, int agent1_id, int type2_id, int agent2_id, size_t timestep) {
	// if both agents at their goal, they are done moving (cannot switch places)
	if (timestep >= paths[type1_id]->at(agent1_id).size() && timestep >= paths[type2_id]->at(agent2_id).size())
		return false;
	if (getAgentLocation(type1_id, agent1_id, timestep) == getAgentLocation(type2_id, agent2_id, timestep + 1) &&
		getAgentLocation(type1_id, agent1_id, timestep + 1) == getAgentLocation(type2_id, agent2_id, timestep))
		return true;
	return false;
}

/*
  Emulate agents' paths and returns a vector of collisions
  Note - a collision is a tuple of <int agent1_id, agent2_id, int location1, int location2, int timestep>).
  Note - the tuple's location_2=-1 for vertex collision.
 */
vector< tuple<int, int, int, int, int> >* CBSSearch::extractCollisions(int num_of_types) {
	vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
	earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
	// check for vertex and edge collisions
	for (int t1 = 0; t1 < num_of_types; t1++) {
		for (int t2 = t1 + 1; t2 < num_of_types; t2++) {
			size_t max_path_length = paths[t1]->at(0).size() > paths[t2]->at(0).size() ? paths[t1]->at(0).size() : paths[t2]->at(0).size();
			for (int timestep = 0; timestep < (int)max_path_length; timestep++) {
				for (int a1 = 0; a1 < (int)paths[t1]->size(); a1++) {
					for (int a2 = 0; a2 < (int)paths[t2]->size(); a2++) {
						if (getAgentLocation(t1, a1, timestep) == getAgentLocation(t2, a2, timestep)) {
							cons_found->push_back(make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), -1, timestep));
							if ((int)timestep < std::get<4>(earliest_conflict)) {
								earliest_conflict = make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), -1, timestep);
							}
						}
						if (switchedLocations(t1, a1, t2, a2, timestep)) {
							cons_found->push_back(make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), getAgentLocation(t2, a2, timestep), timestep));
							if ((int)timestep < std::get<4>(earliest_conflict)) {
								earliest_conflict = make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), getAgentLocation(t2, a2, timestep), timestep);
							}
						}
					}
				}
			}
		}
	}
	return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CBSSearch::CBSSearch(MapLoader& ml, const AgentsLoader& al, double e_w, bool tweak_g_val)
	: m_ml(ml)
{
	// cols = ml.cols;
	num_expanded = 0;
	num_of_types = al.num_of_types;
	nums_of_agents = al.nums_of_agents;
	map_size = ml.dimx * ml.dimy * ml.dimz;
	solution_found = false;
	solution_cost = -1;
	current_makespan = 0;
	max_makespan = 1.5 * (ml.dimx + ml.dimy + ml.dimz);
	start_locations = vector<vector<int>>(num_of_types);
	goal_locations = vector<vector<int>>(num_of_types);
	search_engines = vector < SingleTypeSearch* >(num_of_types);

	for (int t = 0; t < num_of_types; t++) {
		vector<int> init_locs_list(nums_of_agents[t]);
		vector<int> goal_locs_list(nums_of_agents[t]);
		for (int i = 0; i < nums_of_agents[t]; i++) {
			int init_loc = ml.locationToIdx(al.initial_locations[t][i]);
			int goal_loc = ml.locationToIdx(al.goal_locations[t][i]);
			init_locs_list[i] = init_loc;
			goal_locs_list[i] = goal_loc;
		}
		start_locations[t] = init_locs_list;
		goal_locations[t] = goal_locs_list;

		//determine starting makespan
		for (int i = 0; i < nums_of_agents[t]; i++) {
			int min_distance_to_a_goal = map_size;
			//vector<int> distances = ml.getDistancesToAllLocations(init_locs_list[i]);
			//ComputeHeuristic ch(goal_locs_list[i], ml.get_map(), ml.rows*ml.cols, ml.actions_offset, e_w, &egr);
			for (int j = 0; j < nums_of_agents[t]; j++) {
				//int manhattan_distance = distances[goal_locs_list[j]];
				Location l1, l2;
				ml.idxToLocation(init_locs_list[i], l1);
				ml.idxToLocation(goal_locs_list[j], l2);

				int manhattan_distance = abs(l1.x - l2.x) + abs(l1.y - l2.y) + abs(l1.z - l2.z);
				min_distance_to_a_goal = (manhattan_distance < min_distance_to_a_goal) ? manhattan_distance : min_distance_to_a_goal;
			}
			current_makespan = (current_makespan < min_distance_to_a_goal) ? min_distance_to_a_goal : current_makespan;
		}
	}

	// initialize paths_found_initially (contain all individual optimal policies)
	paths_found_initially.resize(num_of_types);


	bool all_agents_found_path = true;

	std::clock_t start;
	start = std::clock();

	for (int t = 0; t < num_of_types; t++) {
		paths = paths_found_initially;

		search_engines[t] = new SingleTypeSearch(t, start_locations[t], goal_locations[t],
			NULL,
			ml.my_map, ml, ml.actions_offset,
			e_w,
			tweak_g_val);

		for (; this->current_makespan <= max_makespan; this->current_makespan++) {
			if (search_engines[t]->findPath(paths, current_makespan) == true) {
				paths_found_initially[t] = new vector<vector<int> >(*search_engines[t]->getPath());
				break;
			}
		}
		if (current_makespan == max_makespan) {
			all_agents_found_path = false;
		}

	}

	pre_time = (std::clock() - start);

	if (all_agents_found_path == false) {
		cout << "NO SOLUTION EXISTS";
	}

	paths = paths_found_initially;


	dummy_start = new CBSNode();
	dummy_start->type_id = -1;
	dummy_start->g_val = current_makespan;//compute_g_val(num_of_types);
	dummy_start->open_handle = heap.push(dummy_start);
	current_makespan = dummy_start->g_val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CBSSearch::runCBSSearch() {
	std::clock_t start;
	start = std::clock();

	// start is already in the heap
	while (!heap.empty() && !solution_found) {
		CBSNode* curr = heap.top();
		heap.pop();
		num_expanded++;
		curr->time_expanded = num_expanded;
		//cout << "CBS NODE POPED FOR TP:" << curr->type_id << " Current Makespan: " << current_makespan << endl;
		popped_nodes.push_front(curr);
		updatePaths(curr, dummy_start);  // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start
		vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions(num_of_types);  // check for collisions on updated paths
		/*    for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
		  cout << "A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;    */

		running_time = (std::clock() - start + pre_time) / (double)CLOCKS_PER_SEC;
		if (running_time > TIME_LIMIT) {
			std::cout << "TIMEOUT" << std::endl;
			return solution_found;
		}

		if (collision_vec->size() == 0) {
			solution_found = true;
			solution_cost = curr->g_val;
		}
		else {
			int type1_id, type2_id, location1, location2, timestep;
			tie(type1_id, type2_id, location1, location2, timestep) = earliest_conflict;
			CBSNode* n1 = new CBSNode();
			CBSNode* n2 = new CBSNode();
			n1->type_id = type1_id;
			n2->type_id = type2_id;
			if (location2 == -1) {  // generate vertex constraint
				n1->constraint = make_tuple(location1, -1, timestep);
				n2->constraint = make_tuple(location1, -1, timestep);
			}
			else {  // generate edge constraint
				n1->constraint = make_tuple(location1, location2, timestep);
				n2->constraint = make_tuple(location2, location1, timestep);
			}
			n1->parent = curr;
			n2->parent = curr;

			// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them. Also updates n1's g_val
			if (updateCBSNode(n1, dummy_start) == true) {
				// new g_val equals old g_val plus the new path length found for the agent minus its old path length
				n1->open_handle = heap.push(n1);
			}
			else {
				delete (n1);
			}
			// same for n2
			if (updateCBSNode(n2, dummy_start) == true) {
				n2->open_handle = heap.push(n2);
			}
			else {
				delete (n2);
			}

			//      cout << "It has found the following paths:" << endl;
			//      printPaths();
			//      cout << "First node generated: " << *n1 << "Second node generated: " << *n2 << endl;
		}
		delete (collision_vec);
	}
	if (solution_found) {
		cout << "FOUND COLLISION FREE PATH! Path cost:" << solution_cost << " ; High-level Expanded:" << num_expanded << endl;
		printPaths();
	}
	else {
		cout << "FAILED TO FIND A COLLISION FREE PATH :(" << " ; High-level Expanded:" << num_expanded << endl;
	}
	return solution_found;
}


// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
/*
vector<vector<int>> CBSSearch::getReservationTable(int exclude_type) {
	vector<vector<int>> res_table(this->current_makespan+1, vector<int>(map_size, 0));
	for (size_t timestep = 0; timestep <= this->current_makespan; timestep++) {
		for (int type = 0; type < num_of_types; type++) {
			if (type != exclude_type && paths[type] != NULL)  {
				for (int agent = 0; agent < nums_of_agents[type]; agent++) {
					int loc = getAgentLocation(type, agent, timestep);
					res_table[timestep][loc] += 1;
				}
			}
		}
	}
	return res_table;
}
*/

int CBSSearch::computeNumOfCollidingTypes() {
	int retVal = 0;
	for (int t1 = 0; t1 < num_of_types; t1++) {
		for (int t2 = t1 + 1; t2 < num_of_types; t2++) {
			for (int timestep = 0; timestep < current_makespan; timestep++) {
				for (int a1 = 0; a1 < (int)paths[t1]->size(); a1++) {
					for (int a2 = 0; a2 < (int)paths[t2]->size(); a2++) {
						if (getAgentLocation(t1, a1, timestep) == getAgentLocation(t2, a2, timestep) ||
							switchedLocations(t1, a1, t2, a2, timestep)) {
							retVal++;
							timestep = current_makespan;
							a1 = paths[t1]->size();
							a2 = paths[t2]->size();
						}
					}
				}
			}
		}
	}
	return retVal;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CBSSearch::~CBSSearch() {
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	for (size_t i = 0; i < paths_found_initially.size(); i++)
		delete (paths_found_initially[i]);
	//  for (size_t i=0; i<paths.size(); i++)
	//    delete (paths[i]);
	// clean heap memory and empty heap (if needed, that is if heap wasn't empty when solution found)
	for (heap_open_t::iterator it = heap.begin(); it != heap.end(); it++)
		delete (*it);
	heap.clear();
	// clean up other nodes (the ones that were popped out of the heap)
	//  cout << "Number of CBS nodes expanded: " << popped_nodes.size() << endl;
	while (!popped_nodes.empty()) {
		delete popped_nodes.front();
		popped_nodes.pop_front();
	}
}
