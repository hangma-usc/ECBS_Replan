#ifndef SINGLETYPESEARCH_H
#define SINGLETYPESEARCH_H

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

#include <vector>
#include <list>
#include <utility>
#include <google/dense_hash_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/type_traits.hpp>

#include "node.h"
#include "map_loader.h"

//using namespace std;
using std::map;
using std::vector;
using std::pair;
using std::shared_ptr;
using std::function;
using google::dense_hash_map;
using std::tr1::hash;


class SingleTypeSearch {
public:

	vector<vector<int>>* path;  // a path that takes the agent from initial to goal location satisying all constraints
	// consider changing path from vector to deque (efficient front insertion)
	int path_cost;

	int type_id;
	vector<int> start_locations;
	vector<int> goal_locations;
	const double* my_heuristic;  // this is the precomputed heuristic for this agent
	const vector<bool> my_map;
	const MapLoader& m_ml;

	int map_size;
	const int* actions_offset;
	bool tweak_g_val;
	double e_weight;

	int num_agents;
	int current_timestep;

	SingleTypeSearch(int type_id, vector<int> s_locs, vector<int> g_locs, const double* heuristic,
		const vector<bool> & my_m, const MapLoader& map_loader, const int* actions,
		double e_w = 1.0, bool tweak = false);
	// note if tweak_g_val is true, the costs are also inflated by e_weight

	const vector<vector<int>>* getPath() { return path; }  // return a pointer to the path found;

	bool findPath();

	bool findPath(vector<vector < vector<int> >* > & res_table, int cost_upperbound);

	bool findPath(const vector < list< pair<int, int> > >* constraints, vector<vector < vector<int> >* > & res_table, int cost_upperbound);

	~SingleTypeSearch();



	/***************************************************************************/
	struct VertexProperties {
		int loc1;
		int loc2;
		bool is_in_node; //in or out node
		bool is_out_node;
		bool is_u_node; //u or v node for each edge in E
		int timestep;
	};

	struct NodeHasher {
		std::size_t operator()(const VertexProperties* n) const {
			size_t loc1_hash = std::hash<int>()(n->loc1);
			size_t loc2_hash = std::hash<int>()(n->loc2);
			size_t time_generated_hash = std::hash<int>()(n->timestep);
			return (loc1_hash ^ (loc2_hash << 1) ^ (time_generated_hash << 2) );
		}
	};

	struct EqNode {
		bool operator()(const VertexProperties* s1, const VertexProperties* s2) const {
			return (s1 == s2) || (s1 && s2 &&
				s1->loc1 == s2->loc1 &&
				s1->loc2 == s2->loc2 &&
				s1->is_in_node == s2->is_in_node &&
				s1->is_out_node == s2->is_out_node &&
				s1->is_u_node == s2->is_u_node &&
				s1->timestep == s2->timestep);
		}
	};

	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperties,
		boost::property < boost::edge_capacity_t, int,
		boost::property < boost::edge_residual_capacity_t, int,
		boost::property < boost::edge_reverse_t, boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>::edge_descriptor,
		boost::property < boost::edge_weight_t, int>
		>
		>
		> > Graph_t;
	typedef Graph_t::vertex_descriptor Graph_Vertex_t;
	typedef Graph_t::edge_descriptor Graph_Edge_t;

	typedef boost::property_map < Graph_t, boost::edge_capacity_t >::type Capacity;
	typedef boost::property_map < Graph_t, boost::edge_residual_capacity_t >::type ResidualCapacity;
	typedef boost::property_map < Graph_t, boost::edge_weight_t >::type Weight;
	typedef boost::property_map < Graph_t, boost::edge_reverse_t>::type Reversed;
	/*typedef boost::labeled_graph<
		Graph_t,
		int
	> Graph_t_labeled;*/

	typedef dense_hash_map<VertexProperties*, Graph_Vertex_t, NodeHasher, EqNode> hashtable_t;

	Capacity  capacities;
	Reversed rev_edges;
	ResidualCapacity residual_capacities;
	Weight weights;

	Graph_t time_expanded_graph;
	hashtable_t vertex_index_map;
	//vector<hashtable_t> vertex_index_maps;

	class EdgeAdder {
	public:
		EdgeAdder(Graph_t & g, Weight & w, Capacity & c, Reversed & rev, ResidualCapacity & residualCapacity)
			: m_g(g), m_w(w), m_cap(c), m_resCap(residualCapacity), m_rev(rev) {}

		void addEdge(Graph_Vertex_t & u, Graph_Vertex_t & v, int weight, int capacity) {
			Graph_Edge_t e, f;
			e = add(u, v, weight, capacity);
			f = add(v, u, -weight, 0);
			//m_rev[e] = f;
			//m_rev[f] = e;
		}

		void updateReversedEdges() {

			Graph_t::edge_iterator ei, e_end;
			for (boost::tie(ei, e_end) = boost::edges(m_g); ei != e_end; ++ei) {
				Graph_Vertex_t u = boost::source(*ei, m_g);
				Graph_Vertex_t v = boost::target(*ei, m_g);
				bool exst;
				Graph_Edge_t reversed_edge;
				boost::tie(reversed_edge, exst) = boost::edge(v, u, m_g);
				m_rev[*ei] = reversed_edge;
				m_rev[reversed_edge] = *ei;
			}
		}

	private:
		Graph_Edge_t add(Graph_Vertex_t & u, Graph_Vertex_t & v, int weight, int capacity) {
			bool b;
			Graph_Edge_t e;
			boost::tie(e, b) = boost::add_edge(u, v, m_g);
			if (!b) {
				std::cerr << "Edge between " << u << " and " << v << " already exists." << std::endl;
				std::abort();
			}
			m_cap[e] = capacity;
			m_w[e] = weight;
			return e;
		}
		Graph_t & m_g;
		Weight & m_w;
		Capacity & m_cap;
		ResidualCapacity & m_resCap;
		Reversed & m_rev;
	};


private:
	//vector<vector<bool>> reachable_locations;

	void setUntillLayer(int layer_time_step); // expand graph until layer_time_step

	Graph_Vertex_t getVertex(Graph_t & g, VertexProperties* v_pt);

	long getVertexIndex(VertexProperties* v_pt);

	//  inline bool checkFutureConstraints(int goal_location, int curr_timestep, const vector< list< pair<int, int> > >* cons)
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< pair<int, int> > >* cons);

	inline bool isDirectionAllowed(int location, int direction);

	inline bool existVertex(Graph_t & g, VertexProperties* v_pt);

	Graph_Vertex_t addVertex(Graph_t & g, VertexProperties* v_pt);

	vector<int> getDistancesToAllLocations(int start);

	void printPaths();

	void updateWeights(Graph_t & g, Weight & w, vector<vector < vector<int> >* > & res_table);

	//void updateReachableSets(int timesteps);

	//void setHashtable(int layer_time_step);


	template <class GraphType> class myVertexWriter {
	public:
		explicit myVertexWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
		template <class VertexOrEdge> void operator()(std::ostream &out, const VertexOrEdge &v) const {
			out << "[loc1=\"" << graphType[v].loc1 << "\", " <<
				"loc2=\"" << graphType[v].loc2 << "\", " <<
				"is_u_node=\"" << graphType[v].is_u_node << "\", " <<
				"label=\"L1:" << graphType[v].loc1 <<
				", L2:" << graphType[v].loc2 << ", T:" << graphType[v].timestep <<
				"\"]";  // for dotty
		}
	private:
		GraphType graphType;
	};

};

#endif
