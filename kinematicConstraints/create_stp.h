// Generate partial order graph

#ifndef CREATESTP_H
#define CREATESTP_H
#define _USE_MATH_DEFINES
#include <cmath>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/property_map/property_map.hpp>
// #include <boost/function.hpp>
// #include <boost/make_shared.hpp>
#include <google/dense_hash_map>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

#include "discretePlan.h"

using std::map;
using std::vector;
using std::pair;
using std::shared_ptr;
using std::function;
using google::dense_hash_map;
using std::tr1::hash;

class CreateSTP {


 public:
  enum OptimizationObjective {
    MinMakeSpan,
    MaxThroughput,
    Hybrid,
  };
private:
  OptimizationObjective _objective;
  bool _disableVminMaximization;


public:

  // Create a struct to hold properties for each vertex
  struct VertexProperties {
    VertexProperties()
      : agent_id(-1)
      , loc1(-1)
      , loc2(-1)
      , isMarker(false)
      , timestep(-1)
      , x(0)
      , y(0)
      , z(0)
      , orientation(discretePlan::ORIENTATION_EAST)
      , vmax(-1)
    {
    }

    VertexProperties(
      int agent_id,
      int loc1,
      int loc2,
      bool isMarker,
      int timestep,
      double x = 0,
      double y = 0,
      double z = 0,
      discretePlan::orientation_t orientation = discretePlan::ORIENTATION_EAST)
        : agent_id(agent_id)
        , loc1(loc1)
        , loc2(loc2)
        , isMarker(isMarker)
        , timestep(timestep)
        , x(x)
        , y(y)
        , z(z)
        , orientation(orientation)
        , vmax(-1)
    {
    }
    // represent agent_id's:
    // loc1 (if loc2=-1 and markers are false)
    // if isMarker==true, the vertex represents delta(loc1,loc2)
    int agent_id;
    int loc1;
    int loc2;
    bool isMarker;
    int timestep;
    // physical location
    double x;
    double y;
    double z;
    discretePlan::orientation_t orientation;

    double vmax; // m/s

    double theta() const {
      double theta = 0;
      switch (orientation) {
      case discretePlan::ORIENTATION_EAST:
        theta = 0;
        break;
      case discretePlan::ORIENTATION_WEST:
        theta = M_PI;
        break;
      case discretePlan::ORIENTATION_NORTH:
        theta = M_PI/2.0f;
        break;
      case discretePlan::ORIENTATION_SOUTH:
        theta = 1.5f * M_PI;
        break;
      default:
        break;
      }
      return theta;
    }

  };

  // Create a struct to hold properties for each edge
  enum EdgeAction {
    EdgeActionMove,
    EdgeActionRotate,
    EdgeActionNone,
  };

  struct EdgeProperties {
    EdgeProperties()
      : lb(0)
      , ub(0)
      , translation(0)
      , rotation(0)
      , action(EdgeActionNone)
    {
    }

    double lb;  // lower bound
    double ub;  // upper bound
    double translation;
    double rotation;
    EdgeAction action;
  };


  struct STPNodeHasher {
    std::size_t operator()(const VertexProperties* n) const {
      size_t agent_id_hash = std::hash<int>()(n->agent_id);
      size_t time_generated_hash = std::hash<int>()(n->timestep);
      return ( agent_id_hash ^ (time_generated_hash << 1) );
    }
  };

  struct stp_eqnode {
    bool operator()(const VertexProperties* s1, const VertexProperties* s2) const {
      return (s1 == s2) || (s1 && s2 &&
                            s1->agent_id == s2->agent_id &&
                            s1->loc1 == s2->loc1 &&
                            s1->loc2 == s2->loc2 &&
                            s1->isMarker == s2->isMarker &&
                            s1->timestep == s2->timestep);
    }
  };


  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperties, EdgeProperties> Graph_t;
  typedef boost::graph_traits<Graph_t>::vertex_descriptor Graph_Vertex_t;
  typedef boost::graph_traits<Graph_t>::edge_descriptor Graph_Edge_t;
  typedef boost::graph_traits<Graph_t>::vertex_iterator vertex_iter_t;
  typedef boost::graph_traits<Graph_t>::edge_iterator edge_iter_t;
  typedef boost::graph_traits<Graph_t> GraphTraits;
  //  typedef boost::adjacency_list<boost::vecS, boost::hash_setS, boost::directedS, VertexDesc> my_digraph;
  typedef dense_hash_map<VertexProperties*, Graph_Vertex_t, STPNodeHasher, stp_eqnode> hashtable_t;


  size_t max_path_length;
  size_t num_of_agents;

  Graph_t stp_graph;
  Graph_Vertex_t X0, XF;

  hashtable_t stpNodes;  // map VertexProperties to their Vertex_Descriptor

  double lambda;
  double delta;
  bool isDeltaInverse;

  struct constrainedPathEntry {
    constrainedPathEntry(int loc, double x, double y, double z, double theta, double arrival)
      : loc(loc)
      , x(x)
      , y(y)
      , z(z)
      , theta(theta)
      , arrival(arrival)
    {
    }

    int loc; //legacy
    double x;
    double y;
    double z;
    double theta;
    double arrival;
  };

  vector< vector< constrainedPathEntry >* > paths_temporal;  // this is used for visualization
  // paths_temporal[i]->at(t) is the <loc,time_event> of agent i at transition (timestep) t

  explicit CreateSTP(
    const discretePlan::discretePlan& discretePlan,
    double delta,
    CreateSTP::OptimizationObjective objective,
    bool disableVminMaximization,
    const std::string& outputFile);

  void addTemporalConstraints(const discretePlan::discretePlan& discretePlan);
  inline std::string getVarName(Graph_Vertex_t vertex);
  inline std::string getEdgeName(Graph_Edge_t e_desc);
  void exampleLP();
  void solveLP(const discretePlan::discretePlan& discretePlan, const std::string& outputFile);
  ~CreateSTP();
  //  void exportSTP(string filename);

  // inner class that is used to "pretty-print" to DOT file (VERTEX) ///////////////////////////////////
  template <class GraphType> class myVertexWriter {
  public:
    explicit myVertexWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
    template <class VertexOrEdge> void operator()(std::ostream &out, const VertexOrEdge &v) const {
      out << "[agent_id=\"" << graphType[v].agent_id << "\", " <<
        "loc1=\"" << graphType[v].loc1  << "\", " <<
        "loc2=\"" << graphType[v].loc2 << "\", " <<
        "isMarker=\"" << graphType[v].isMarker << "\", ";
      out << "label=\"";
        if (graphType[v].isMarker) {
          out << "AG" << graphType[v].agent_id << " m (" << graphType[v].y << ", " << graphType[v].x;
          out << ") T: " << graphType[v].timestep;
          // out << " vmax: " << graphType[v].vmax;
        } else {
          // out << "label=\"AG:" << graphType[v].agent_id << ", L1:" << graphType[v].loc1 <<
          //        ", L2:" << graphType[v].loc2 << ", T:" << graphType[v].timestep;
          if (graphType[v].agent_id == -3) {
            out << "Start";
          } else if (graphType[v].agent_id == -4) {
            out << "Finish";
          } else {
            out << "AG" << graphType[v].agent_id <<
                   " (" << graphType[v].y <<
                   ", " << graphType[v].x << ", ";
            switch (graphType[v].orientation) {
              case discretePlan::ORIENTATION_EAST:
                out << "&#8594;";
                break;
              case discretePlan::ORIENTATION_NORTH:
                out << "&#8593;";
                break;
              case discretePlan::ORIENTATION_WEST:
                out << "&#8592;";
                break;
              case discretePlan::ORIENTATION_SOUTH:
                out << "&#8595;";
                break;
              default:
                break;
            }
            out << ") T: " << graphType[v].timestep;
            // out << " vmax: " << graphType[v].vmax;
          }
        }
        out << "\"]";  // for dotty
    }
  private:
    GraphType graphType;
  };  // end of inner class ///////////////////////////////////////////////////////////////////////
  // inner class that is used to "pretty-print" to DOT file (EDGE) ///////////////////////////////////
  template <class GraphType> class myEdgeWriter {
  public:
    explicit myEdgeWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
    template <class VertexOrEdge> void operator()(std::ostream &out, const VertexOrEdge &e) const {
      auto u = source(e, graphType);
      auto v = target(e, graphType);
      out << "[src_agent_id=\"" << graphType[u].agent_id << "\", dst_agent_id=\"" << graphType[v].agent_id << "\"";
      out << ", label=\"[";
      if (graphType[e].ub == DBL_MAX) {
        out << graphType[e].lb << ",&#8734;";
      } else {
        out << graphType[e].lb << "," << graphType[e].ub;
      }
      out << " t: " << graphType[e].translation << " r: " << graphType[e].rotation;
      out << "]\"]";
    }
  private:
    GraphType graphType;
  };  // end of inner class ///////////////////////////////////////////////////////////////////////

};

#endif
