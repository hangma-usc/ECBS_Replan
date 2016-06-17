// Load's agents' init and goal states.
// First line: number of agents
// Second line and onward, (x_init,y_init),(x_goal,y_goal) of each agent (one per line)

#ifndef AGENTSLOADER_H
#define AGENTSLOADER_H

#include <string>
#include <vector>
#include <utility>

#include "types.h"

using namespace std;

class AgentsLoader {
public:
	int num_of_types;
	vector<int> nums_of_agents;
	vector<vector< Location > > initial_locations;
	vector<vector< Location > > goal_locations;

	AgentsLoader(const std::string& fname);
	AgentsLoader();
	// void addAgent(int start_row, int start_col, int goal_row, int goal_col);
	// void printAgentsInitGoal();
	// void saveToFile(std::string fname);
	// pair<int, int> agentStartOrGoalAt(int row, int col);
	// void clearLocationFromAgents(int row, int col);
	~AgentsLoader();
};

#endif
