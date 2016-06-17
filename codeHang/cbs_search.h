// CBS Search (High-level)
#ifndef CBSSEARCH_H
#define CBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <cstring>
#include <climits>
#include <tuple>
#include <string>
#include <vector>
#include <list>
#include "map_loader.h"
#include "agents_loader.h"
#include "single_type_search.h"
#include "cbs_node.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;

class CBSSearch {
 public:
  typedef boost::heap::fibonacci_heap< CBSNode* , boost::heap::compare<CBSNode::compare_node> > heap_open_t;
  typedef dense_hash_map<CBSNode*, CBSNode*, CBSNode::CBSNodeHasher, CBSNode::cbs_eqnode> hashtable_t;

  vector<vector < vector<int> >* > paths;  // agents paths (each entry [t][i] is a vector<int> which specify the locations on the path of agent i)
  vector<vector < vector<int> >* > paths_found_initially;  // contain initial paths found (that is, each with optimal policy)
  bool solution_found;
  double solution_cost;
  double running_time;
  double pre_time;

  list < CBSNode* > popped_nodes;  // used to clean the memory at the end
  CBSNode* dummy_start;
  vector<vector <int> > start_locations;
  vector<vector <int> > goal_locations;

  // int dimx;

  const vector<bool> my_map;
  int map_size;
  int num_of_types;
  int current_makespan;
  int max_makespan;
  vector<int> nums_of_agents;
  const int* actions_offset;

  int num_expanded = 0;

  heap_open_t heap;

  vector < SingleTypeSearch* > search_engines;  // used to find (single) agents' paths

  tuple<int, int, int, int, int> earliest_conflict;

  CBSSearch(MapLoader& ml, const AgentsLoader& al, double e_w, bool tweak_g_val = false);
  inline double compute_g_val();
  inline void updatePaths(CBSNode* curr , CBSNode* root_node);
  inline bool updateCBSNode(CBSNode* leaf_node, CBSNode* root_node);
  bool runCBSSearch();
  inline bool switchedLocations(int type1_id, int agent1_id, int type2_id, int agent2_id, size_t timestep);
  inline int getAgentLocation(int type_id, int agent_id, size_t timestep);
  vector< tuple<int, int, int, int, int> >* extractCollisions(int nums_of_types);
  void printPaths();
  void updatePathsForExpTime(int t_exp);

  vector<vector < vector<int> >* > getReservationTable(int exclude_type);
  int computeNumOfCollidingTypes();

  ~CBSSearch();
private:
  const MapLoader& m_ml;
};

#endif
