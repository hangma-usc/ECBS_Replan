#include <boost/heap/fibonacci_heap.hpp>
#include "compute_heuristic.h"
#include <cstring>
#include <climits>
#include <google/dense_hash_map>
#include "node.h"

using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


ComputeHeuristic::ComputeHeuristic(int goal_location, const bool* my_map, int map_size,
                                   const int* moves_offset, const int* actions_offset,
                                   double e_weight, const EgraphReader* egr) :
    my_map(my_map), moves_offset(moves_offset), actions_offset(actions_offset) {
  this->egr = egr;
  this->e_weight = e_weight;
  this->goal_location = goal_location;
  // this->my_map = my_map; -- our map is constant, so has to be in initialization list
  this->map_size = map_size;
  h_vals = new double[map_size];
  for (int i = 0; i < map_size; i++)
    h_vals[i] = DBL_MAX;
  // generate a heap that can save nodes (and a open_handle)
  boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> > heap;
  boost::heap::fibonacci_heap< Node* , boost::heap::compare<Node::compare_node> >::handle_type open_handle;
  // generate hash_map (key is a node pointer, data is a node handler,
  //                    NodeHasher is the hash function to be used,
  //                    eqnode is used to break ties when hash values are equal)
  dense_hash_map<Node*, fibonacci_heap<Node* , boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode> nodes;
  nodes.set_empty_key(NULL);
  dense_hash_map<Node*, fibonacci_heap<Node* , boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode>::iterator it; // will be used for find()

  Node* goal = new Node (goal_location, MapLoader::WAIT_ACTION, 0, 0, NULL, 0, false);
  goal->open_handle = heap.push(goal);  // add goal to heap
  nodes[goal] = goal->open_handle;       // add goal to hash_table (nodes)
  while ( !heap.empty() ) {
    Node* curr = heap.top();
    heap.pop();
    // cout << endl << "CURRENT node: " << curr << endl;
    for (int direction = 0; direction < 5; direction++) {
      int next_loc = curr->loc + moves_offset[direction];
      if ( !my_map[next_loc] ) {  // if that grid is not blocked
        // compute cost to next_loc via curr node
        double cost = 1;
        if (!(*egr).isEdge(next_loc, curr->loc))
          cost = cost * e_weight;
        double next_g_val = curr->g_val + cost;
        Node* next = new Node (next_loc, MapLoader::WAIT_ACTION, next_g_val, 0, NULL, 0, false);
        it = nodes.find(next);
        if ( it == nodes.end() ) {  // add the newly generated node to heap and hash table
          next->open_handle = heap.push(next);
          nodes[next] = next->open_handle;
        } else {  // update existing node's g_val if needed (only in the heap)
          delete(next);  // not needed anymore -- we already generated it before
          Node* existing_next = (*it).first;
          open_handle = (*it).second;
          if (existing_next->g_val > next_g_val) {
            existing_next->g_val = next_g_val;
            heap.update(open_handle);
          }
        }
      }
    }
  }
  // iterate over all nodes and populate the h_vals
  for (it=nodes.begin(); it != nodes.end(); it++) {
    Node* s = (*it).first;
    h_vals[s->loc] = s->g_val;
    delete (s);
  }
  nodes.clear();
  heap.clear();
}

double* ComputeHeuristic::getHVals() {
  double* retVal = new double[this->map_size];
  memcpy (retVal, this->h_vals, sizeof(double) * this->map_size );
  return retVal;
}

ComputeHeuristic::~ComputeHeuristic() {
  delete[] this->h_vals;
  delete[] my_map;
}
