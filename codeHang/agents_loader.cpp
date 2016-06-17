//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if

using namespace boost;
using namespace std;

AgentsLoader::AgentsLoader(const string& fname) {

	string line;

	ifstream myfile(fname.c_str());

	if (myfile.is_open()) {

		getline(myfile, line);
		char_separator<char> sep(",");

		//num_types
		tokenizer< char_separator<char> > type_tok(line, sep);
		tokenizer< char_separator<char> >::iterator type_beg = type_tok.begin();
		this->num_of_types = atoi((*type_beg).c_str());

		getline(myfile, line);
		//num_agents
		tokenizer< char_separator<char> > agt_tok(line, sep);
		tokenizer< char_separator<char> >::iterator agt_beg = agt_tok.begin();
		for (int t = 0; t < num_of_types; t++) {
			nums_of_agents.push_back(atoi((*agt_beg).c_str()));
			agt_beg++;
		}

		getline(myfile, line);

		for (int t = 0; t < num_of_types; t++) {
			vector< Location > initial_locations_sametype;
			vector< Location > goal_locations_sametype;
			for (int i = 0; i < nums_of_agents[t]; i++) {
				getline(myfile, line);
				tokenizer< char_separator<char> > col_tok(line, sep);
				tokenizer< char_separator<char> >::iterator c_beg = col_tok.begin();
				Location curr_loc;
				curr_loc.x = atoi((*c_beg).c_str());
				c_beg++;
				curr_loc.y = atoi((*c_beg).c_str());
				c_beg++;
				curr_loc.z = atoi((*c_beg).c_str());
				//      cout << "AGENT" << i << ":   START[" << curr_pair.first << "," << curr_pair.second << "] ; ";
				initial_locations_sametype.push_back(curr_loc);
				c_beg++;
				curr_loc.x = atoi((*c_beg).c_str());
				c_beg++;
				curr_loc.y = atoi((*c_beg).c_str());
				c_beg++;
				curr_loc.z = atoi((*c_beg).c_str());
				//      cout << "GOAL[" << curr_pair.first << "," << curr_pair.second << "]" << endl;
				goal_locations_sametype.push_back(curr_loc);
			}
			initial_locations.push_back(initial_locations_sametype);
			goal_locations.push_back(goal_locations_sametype);

		}
		myfile.close();
	}
	else
		cerr << "Agents file not found." << std::endl;
}

// void AgentsLoader::printAgentsInitGoal() {
// 	cout << "TYPES:" << endl;
// 	for (int t = 0; t < num_of_types; t++) {
// 		cout << "Type" << t <<" AGENTS:" << endl;
// 		for (int i = 0; i < nums_of_agents[t]; i++) {
// 			cout << "Agent" << i << " : I=(" << initial_locations[t][i].first << "," << initial_locations[t][i].second << ") ; G=(" <<
// 				goal_locations[t][i].first << "," << goal_locations[t][i].second << ")" << endl;
// 		}
// 		cout << endl;
// 	}
// }

AgentsLoader::~AgentsLoader() {
	// vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
	num_of_types = 0;
}

// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
/*
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
	int f = -1;
	int s = -1;
	for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
		if (it->first == row && it->second == col)
		f = std::distance(initial_locations.begin(), it);
	for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
		if (it->first == row && it->second == col)
		s = std::distance(goal_locations.begin(), it);
	return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
	pair<int, int> idxs = agentStartOrGoalAt(row, col);
	if (idxs.first != -1) {  // remove the agent who's start is at [row,col]
		initial_locations.erase(initial_locations.begin() + idxs.first);
		goal_locations.erase(goal_locations.begin() + idxs.first);
		num_of_agents--;
	}
	idxs = agentStartOrGoalAt(row, col);
	if (idxs.second != -1) {  // remove the agent who's goal is at [row,col]
		initial_locations.erase(initial_locations.begin() + idxs.second);
		goal_locations.erase(goal_locations.begin() + idxs.second);
		num_of_agents--;
	}
}

// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) {
	this->initial_locations.push_back(make_pair(start_row, start_col));
	this->goal_locations.push_back(make_pair(goal_row, goal_col));
	num_of_agents++;
}
*/

// void AgentsLoader::saveToFile(std::string fname) {
// 	ofstream myfile;
// 	myfile.open(fname);
// 	myfile << num_of_types << endl;
// 	for (int t = 0; t < num_of_types; t++) {

// 		for (int i = 0; i < nums_of_agents[t]; i++) {
// 			myfile << initial_locations[t][i].first << "," << initial_locations[t][i].second << ","<< goal_locations[t][i].first << "," << goal_locations[t][i].second << endl;
// 		}
// 	}
// 	myfile.close();
// }
