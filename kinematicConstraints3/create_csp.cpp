#include "create_csp.h"
#include <vector>
#include <utility>
#include <cfloat>
#include <string>
#include "gurobi_c++.h"

#include <Eigen/Core>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace pt = boost::property_tree;

using std::make_pair;
using std::cout;
using std::endl;
using std::to_string;
using std::pair;


size_t getPathsMaxLength(const discretePlan::discretePlan& discretePlan) {
  size_t retVal = 0;
  int num_of_agents = discretePlan.agents.size();
  for (int ag = 0; ag < num_of_agents; ag++)
  {
    if (discretePlan.agents[ag].states.size() > retVal)
    {
      retVal = discretePlan.agents[ag].states.size();
    }
  }
  return retVal;
}  ////////////////////////////////////////////////////////////////////////////////////////////////


inline int getAgentLocation(int agent_id, size_t timestep, const discretePlan::discretePlan& discretePlan) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= discretePlan.agents[agent_id].states.size())
    return discretePlan.agents[agent_id].states.back().locationId;
  // otherwise, return its location for that timestep
  return discretePlan.agents[agent_id].states[timestep].locationId;
}  ////////////////////////////////////////////////////////////////////////////////////////////////


void CreateCSP::addTemporalConstraints(const discretePlan::discretePlan& discretePlan) {
	for (size_t ag1 = 0; ag1 < num_of_agents; ag1++) {
		for (size_t t1 = 0; t1 < discretePlan.agents[ag1].states.size() - 1; t1++) {
			if (getAgentLocation(ag1, t1, discretePlan) != getAgentLocation(ag1, t1 + 1, discretePlan)) {  // skip wait/rotation
				for (size_t ag2 = 0; ag2 < num_of_agents; ag2++) {
					if (ag1 != ag2) {
						for (size_t t2 = t1; t2 < discretePlan.agents[ag2].states.size() - 1; t2++) {
							//if ( getAgentLocation(ag2, t2, discretePlan) != getAgentLocation(ag2, t2+1, discretePlan) ||  // skip wait/rotation
							//     t2 == discretePlan.agents[ag2].states.size()-1 ) {  // unless its the goal location
							if (getAgentLocation(ag2, t2, discretePlan) != getAgentLocation(ag2, t2 + 1, discretePlan)) {
								if (getAgentLocation(ag1, t1, discretePlan) == getAgentLocation(ag2, t2, discretePlan)) {
									// std::cout << "temp constraint: A1: " << ag1 << " t1: " << t1 << " A2: " << ag2 << " t2: " << t2 << endl;
									// find marker right after <ag1,t1> and right before <ag2,t2>
									// cout << "A1:" << ag1 << ",LOC1:" << paths[ag1]->at(t1) << ",LOC2:" << paths[ag1]->at(t1+1) <<
									//    "T:" << t1 << " ; A2:" << ag2 << ",LOC1:" << paths[ag2]->at(t2) <<
									//    ",LOC2:" << paths[ag2]->at(t2-1) << "T:" << t2-1 << endl;
									size_t t2n = t2 - 1;
									for (; getAgentLocation(ag2, t2, discretePlan) == getAgentLocation(ag2, t2n, discretePlan); t2n--)
									{
									}
									t2n++;

									VertexProperties* ag1_t1_plus = new VertexProperties(ag1, discretePlan.agents[ag1].states[t1].locationId, discretePlan.agents[ag1].states[t1 + 1].locationId, true, t1);
									VertexProperties* ag2_t2n_minus = new VertexProperties(ag2, discretePlan.agents[ag2].states[t2n].locationId, discretePlan.agents[ag2].states[t2n - 1].locationId, true, t2n - 1);

									Graph_Vertex_t ag1_t1_plus_desc = cspNodes[ag1_t1_plus];
									Graph_Vertex_t ag2_t2n_minus_desc = cspNodes[ag2_t2n_minus];
									Graph_Edge_t e_desc = (boost::add_edge(ag1_t1_plus_desc, ag2_t2n_minus_desc, csp_graph)).first;
									csp_graph[e_desc].lb = 0;
									csp_graph[e_desc].ub = DBL_MAX;

									//break; //for csp need to consider later visit now


									int t1n = t1; // last time when ag1 in a different location than t1 
									for (; getAgentLocation(ag1, t1, discretePlan) == getAgentLocation(ag1, t1n, discretePlan); t1n--)
									{
									}
									if (t1n > 0) { //need to add reverse edge (allow changing order)
										int t1nn = t1n;
										for (; getAgentLocation(ag1, t1n, discretePlan) == getAgentLocation(ag1, t1nn, discretePlan); )
										{
											t1nn--;
											if (t1nn < 0) {
												break;
											}
										}
										t1nn++;
										if (t1nn > 0 && getAgentLocation(ag2, t2 + 1, discretePlan) != discretePlan.agents[ag2].states[discretePlan.agents[ag2].states.size() - 1].locationId
											&& getAgentLocation(ag1, t1nn, discretePlan) == getAgentLocation(ag2, t2 + 1, discretePlan)) { //allow changing order of visiting a pair of locations
											size_t t2p = t2 + 1;
											for (; getAgentLocation(ag2, t2, discretePlan) == getAgentLocation(ag2, t2p, discretePlan); t2p++)
											{
											}
											VertexProperties* ag1_t1nn_minus = new VertexProperties(ag1, discretePlan.agents[ag1].states[t1nn].locationId, discretePlan.agents[ag1].states[t1nn - 1].locationId, true, t1nn - 1);
											VertexProperties* ag2_t2p_plus = new VertexProperties(ag2, discretePlan.agents[ag2].states[t2p].locationId, discretePlan.agents[ag2].states[t2p + 1].locationId, true, t2p);

											Graph_Vertex_t ag1_t1nn_minus_desc = cspNodes[ag1_t1nn_minus];
											Graph_Vertex_t ag2_t2p_plus_desc = cspNodes[ag2_t2p_plus];
											Graph_Edge_t e_rev_desc = (boost::add_edge(ag2_t2p_plus_desc, ag1_t1nn_minus_desc, csp_graph)).first;
											csp_graph[e_rev_desc].lb = 0;
											csp_graph[e_rev_desc].ub = DBL_MAX;
											edgePairs.push_back(std::make_pair(e_desc, e_rev_desc));
										}
										else { //alow change order of visiting a location
											t1n++;
											VertexProperties* ag1_t1n_minus = new VertexProperties(ag1, discretePlan.agents[ag1].states[t1n].locationId, discretePlan.agents[ag1].states[t1n - 1].locationId, true, t1n - 1);
											VertexProperties* ag2_t2_plus = new VertexProperties(ag2, discretePlan.agents[ag2].states[t2].locationId, discretePlan.agents[ag2].states[t2 + 1].locationId, true, t2);

											Graph_Vertex_t ag1_rev_desc = cspNodes[ag1_t1n_minus];
											Graph_Vertex_t ag2_rev_desc = cspNodes[ag2_t2_plus];
											Graph_Edge_t e_rev_desc = (boost::add_edge(ag2_rev_desc, ag1_rev_desc, csp_graph)).first;
											csp_graph[e_rev_desc].lb = 0;
											csp_graph[e_rev_desc].ub = DBL_MAX;
											edgePairs.push_back(std::make_pair(e_desc, e_rev_desc));
										}
									}
									else { //don't allow change of order
										edgePairs.push_back(std::make_pair(e_desc, e_desc));
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

CreateCSP::CreateCSP(
    const discretePlan::discretePlan& discretePlan,
    double delta,
    CreateCSP::OptimizationObjective objective,
    bool disableVminMaximization,
    const std::string& outputFile)
  :  _objective(objective)
  , _disableVminMaximization(disableVminMaximization)
  , delta(delta)
{
  discretePlan::discretePlan plan = discretePlan;

  // pre-processing: deal with "useless rotations"
  for (auto& agent : plan.agents) {
    auto& path = agent.states;
    auto lastOrientation = path.front().orientation;
    for (size_t i = 1; i < path.size(); ++i) {
      if (   (path[i].action == discretePlan::ACTION_ROTATE_LEFT || path[i].action == discretePlan::ACTION_ROTATE_RIGHT)
          && (path[i-1].action == discretePlan::ACTION_ROTATE_LEFT || path[i-1].action == discretePlan::ACTION_ROTATE_RIGHT)) {
        path[i-1].orientation = lastOrientation;
        path[i-1].action = discretePlan::ACTION_WAIT;
      }
      if (   i > 1
          && (path[i].action == discretePlan::ACTION_ROTATE_LEFT || path[i].action == discretePlan::ACTION_ROTATE_RIGHT)
          && (path[i-1].action == discretePlan::ACTION_WAIT)
          && (path[i-2].action == discretePlan::ACTION_ROTATE_LEFT || path[i-2].action == discretePlan::ACTION_ROTATE_RIGHT)) {
        path[i-1].orientation = lastOrientation;
        path[i-2].orientation = lastOrientation;
        path[i-2].action = discretePlan::ACTION_WAIT;
      }
      if (path[i].action == discretePlan::ACTION_MOVE) {
        lastOrientation = path[i].orientation;
      }
    }
  }


  max_path_length = getPathsMaxLength(plan);
  num_of_agents = plan.agents.size();

  // init hash table
  VertexProperties* empty_node = new VertexProperties(-1, -1, -1, false, -1);
  VertexProperties* deleted_node = new VertexProperties(-2, -2, -2, false, -2);
  cspNodes.set_empty_key(empty_node);
  cspNodes.set_deleted_key(deleted_node);

  // generate X_0 and X_F
  VertexProperties* X0_p = new VertexProperties(-3, -3, -3, false, -3);
  X0 = boost::add_vertex(csp_graph);
  csp_graph[X0] = *X0_p;
  cspNodes[X0_p] = X0;
  VertexProperties* XF_p = new VertexProperties(-4, -4, -4, false, -4);
  XF = boost::add_vertex(csp_graph);  // generate vertex descriptor in graph
  csp_graph[XF] = *XF_p;  // update bundeled properties
  cspNodes[XF_p] = XF;  // update hash table (properties to vertex descriptor)

  Graph_Edge_t e_desc;
  /* DO NOT UNCOMMENT (will add wrong edge to the LP)
  // add edge X0-XF annotated with [0,lambda]
  e_desc = (boost::add_edge(X0, XF, csp_graph)).first;
  csp_graph[e_desc].lb = 0;
  csp_graph[e_desc].ub = -13;
  */
  // generate (basic location and marker) nodes
  for (size_t ag = 0; ag < num_of_agents; ag++) {
    Graph_Vertex_t loc1, marker12, marker21;
    VertexProperties* vp;
    int loc1_id = -1;
    int loc2_id = -1;
    bool first = true;
    for (size_t t = 0; t < plan.agents[ag].states.size()-1; t++) {
      loc1_id = getAgentLocation(ag, t, plan);
      loc2_id = getAgentLocation(ag, t+1, plan);
      if (plan.agents[ag].states[t+1].action != discretePlan::ACTION_WAIT) {  // skip waits
        loc1 = boost::add_vertex(csp_graph);
        float x = plan.agents[ag].states[t].x;
        float y = plan.agents[ag].states[t].y;
        float z = plan.agents[ag].states[t].z;

        vp = new VertexProperties(ag, loc1_id, -1, false, t, x, y, z, plan.agents[ag].states[t].orientation);
        csp_graph[loc1] = *vp;
        cspNodes[vp] = loc1;
        if (first) {  // add X0-loc edge
          e_desc = (boost::add_edge(X0, loc1, csp_graph)).first;
          csp_graph[e_desc].lb = 0;
          csp_graph[e_desc].ub = 0;
          csp_graph[e_desc].translation = 0;
          csp_graph[e_desc].rotation = 0;
          csp_graph[e_desc].action = EdgeActionNone;
          first = false;
        } else {  // add marker21->loc1 for all agents that are not the first
          e_desc = (boost::add_edge(marker21, loc1, csp_graph)).first;
          double translation = sqrt(pow(csp_graph[marker21].x - csp_graph[loc1].x,2) + pow(csp_graph[marker21].y - csp_graph[loc1].y,2) + pow(csp_graph[marker21].z - csp_graph[loc1].z,2));
          double rotation = fabs(atan2(sin(csp_graph[loc1].theta() - csp_graph[marker21].theta()), cos(csp_graph[loc1].theta() - csp_graph[marker21].theta())));
          // std::cout << "x " << csp_graph[marker12].x << " x: " << csp_graph[loc1].x << std::endl;
          std::cout << "T: " << translation << std::endl;
          csp_graph[e_desc].lb = translation / plan.agents[ag].max_v + rotation / plan.agents[ag].max_w;
          csp_graph[e_desc].ub = DBL_MAX;
          csp_graph[e_desc].translation = translation;
          csp_graph[e_desc].rotation = rotation;
          csp_graph[e_desc].action = rotation > 0 ? EdgeActionRotate : EdgeActionMove;
        }

        if (loc1_id != loc2_id) {
          marker12 = boost::add_vertex(csp_graph);
          Eigen::Vector3d v1(plan.agents[ag].states[t].x, plan.agents[ag].states[t].y, plan.agents[ag].states[t].z);
          Eigen::Vector3d v2(plan.agents[ag].states[t+1].x, plan.agents[ag].states[t+1].y, plan.agents[ag].states[t+1].z);
          Eigen::Vector3d marker = v1 + (v2 - v1).normalized() * delta;
          vp = new VertexProperties(ag, loc1_id, loc2_id, true, t, marker.x(), marker.y(), marker.z(), plan.agents[ag].states[t].orientation);
          csp_graph[marker12] = *vp;
          cspNodes[vp] = marker12;
          // add edge between location and safety marker
          e_desc = (boost::add_edge(loc1, marker12, csp_graph)).first;
          csp_graph[e_desc].lb = delta / plan.agents[ag].max_v;
          csp_graph[e_desc].ub = DBL_MAX;
          csp_graph[e_desc].translation = delta;
          csp_graph[e_desc].rotation = 0;
          csp_graph[e_desc].action = EdgeActionMove;

          marker21 = boost::add_vertex(csp_graph);
          marker = v2 + (v1 - v2).normalized() * delta;
          vp = new VertexProperties(ag, loc2_id, loc1_id, true, t, marker.x(), marker.y(), marker.z(), plan.agents[ag].states[t].orientation);
          csp_graph[marker21] = *vp;
          cspNodes[vp] = marker21;
          // add edge between the two safety markers (we assume that the cost of every edge is 1).
          e_desc = (boost::add_edge(marker12, marker21, csp_graph)).first;
          csp_graph[e_desc].lb = (1-2*delta) / plan.agents[ag].max_v;
          csp_graph[e_desc].ub = DBL_MAX;
          csp_graph[e_desc].translation = (1-2*delta);
          csp_graph[e_desc].rotation = 0;
          csp_graph[e_desc].action = EdgeActionMove;
        } else {
          marker21 = loc1;
        }
      }
    }
    // add goal location for the agent
    loc1 = boost::add_vertex(csp_graph);

    vp = new VertexProperties(ag, loc2_id, -1, false, plan.agents[ag].states.size()-1, plan.agents[ag].states.back().x, plan.agents[ag].states.back().y, plan.agents[ag].states.back().z);
    vp->vmax = 0;
    csp_graph[loc1] = *vp;
    cspNodes[vp] = loc1;
    e_desc = (boost::add_edge(marker21, loc1, csp_graph)).first;
    csp_graph[e_desc].lb = delta / plan.agents[ag].max_v;
    csp_graph[e_desc].ub = DBL_MAX;
    csp_graph[e_desc].translation = delta;
    csp_graph[e_desc].rotation = 0;
    csp_graph[e_desc].action = EdgeActionMove;
    // add edge to XF
    e_desc = (boost::add_edge(loc1, XF, csp_graph)).first;
    csp_graph[e_desc].lb = 0;
    csp_graph[e_desc].ub = DBL_MAX;
    csp_graph[e_desc].translation = 0;
    csp_graph[e_desc].rotation = 0;
    csp_graph[e_desc].action = EdgeActionNone;

  }
  addTemporalConstraints(discretePlan);

  // remove all unused markers
  // This is not only a optimization but required as we assume constant velocities between event points
  vertex_iter_t vi, vi_end, next;
  boost::tie(vi, vi_end) = vertices(csp_graph);
  for (next = vi; vi != vi_end; vi = next) {
    ++next;
    // std::cout << "D: " << csp_graph[*vi].isMarker << " " << in_degree(*vi, csp_graph) << " " << out_degree(*vi, csp_graph) << std::endl;
    if (csp_graph[*vi].isMarker && in_degree(*vi, csp_graph) == 1 && out_degree(*vi, csp_graph) == 1) {

      // find (single) in-edge
      GraphTraits::in_edge_iterator in_i, in_end;
      boost::tie(in_i, in_end) = in_edges(*vi,csp_graph);
      Graph_Edge_t inEdge = *in_i;

      // find (single) out-edge
      GraphTraits::out_edge_iterator out_i, out_end;
      boost::tie(out_i, out_end) = out_edges(*vi,csp_graph);
      Graph_Edge_t outEdge = *out_i;

      // combine to new edge
      Graph_Edge_t e_desc = (boost::add_edge(source(inEdge, csp_graph), target(outEdge, csp_graph), csp_graph)).first;
      csp_graph[e_desc].lb = csp_graph[inEdge].lb + csp_graph[outEdge].lb;
      csp_graph[e_desc].ub = DBL_MAX;//csp_graph[inEdge].ub + csp_graph[outEdge].ub;
      csp_graph[e_desc].translation = csp_graph[inEdge].translation + csp_graph[outEdge].translation;
      csp_graph[e_desc].rotation = csp_graph[inEdge].rotation + csp_graph[outEdge].rotation;
      csp_graph[e_desc].action = EdgeActionMove;

      // erase old edges
      remove_edge(inEdge, csp_graph);
      remove_edge(outEdge, csp_graph);

      // erase unused marker
      remove_vertex(*vi, csp_graph);
      --next;
    }
  }


  // Generate DOT file for debug
  myVertexWriter<Graph_t> vw(csp_graph);  // instantiate the writer class
  myEdgeWriter<Graph_t> ew(csp_graph);  // instantiate the writer class
  std::ofstream dotFile("csp.dot");
  boost::write_graphviz(dotFile, csp_graph, vw, ew);
  cout << endl;

  //solveLP(plan, outputFile);

}  ////////////////////////////////////////////////////////////////////////////////////////////////

inline string CreateCSP::getVarName(Graph_Vertex_t vertex) {
  return "V_" + to_string(vertex);
}

inline string CreateCSP::getEdgeName(Graph_Edge_t e_desc) {
  Graph_Vertex_t v_source = boost::source(e_desc, csp_graph);
  Graph_Vertex_t v_target = boost::target(e_desc, csp_graph);
  return getVarName(v_source) + "--" + getVarName(v_target);
  //    + "_[" + to_string(csp_graph[e_desc].lb) + "," + to_string(csp_graph[e_desc].ub) + "]";
}


void CreateCSP::solveLP(
  const discretePlan::discretePlan& discretePlan,
  const std::string& outputFile) {

  // Pass 1: Optimize for RminV
  double RminV_val = 0;
  if (!_disableVminMaximization)
  {
    try {

      GRBEnv env = GRBEnv();
      GRBModel model = GRBModel(env);

      vertex_iter_t vi, vi_end;
      boost::tie(vi, vi_end) = vertices(csp_graph);
      for (; vi != vi_end; vi++) {
        model.addVar(0.0, DBL_MAX, 0.0, GRB_CONTINUOUS, getVarName(*vi));
      }

      GRBVar RminV = model.addVar(0.0, DBL_MAX, 0.0, GRB_CONTINUOUS, "RminV");


      // Integrate new variables
      model.update();

      // Iterate through the edges in the graph
      edge_iter_t ei, ei_end;
      tie(ei, ei_end) = boost::edges(csp_graph);
      for (; ei != ei_end; ++ei) {
        Graph_Edge_t e_desc = *ei;
        Graph_Vertex_t v_source = boost::source(e_desc, csp_graph);
        Graph_Vertex_t v_target = boost::target(e_desc, csp_graph);
        GRBVar grb_v_source = model.getVarByName(getVarName(v_source));
        GRBVar grb_v_target = model.getVarByName(getVarName(v_target));
        model.addConstr(grb_v_target - grb_v_source >= csp_graph[e_desc].lb, getEdgeName(e_desc));
        model.addConstr(grb_v_target - grb_v_source <= csp_graph[e_desc].ub, getEdgeName(e_desc));

        if (csp_graph[e_desc].translation > 0) {
          model.addConstr(grb_v_target - grb_v_source <= csp_graph[e_desc].translation * RminV);
        }
      }
      model.setObjective((GRBLinExpr)RminV, GRB_MINIMIZE);

      // Optimize model
      model.optimize();

      RminV_val = RminV.get(GRB_DoubleAttr_X);

      std::cout << "RminV: " << RminV_val << std::endl;


    } catch(GRBException e) {
      cout << "Error code = " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
    } catch(...) {
      cout << "Exception during optimization" << endl;
    }
  }

  try {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    // create variables
    // hashtable_t::iterator it;  // it.first is VertexProperties*, it.second is Graph_Vertex_t
    // // iterate over all nodes in the hash table (that is, all nodes in the graph)
    // for (it=cspNodes.begin(); it != cspNodes.end(); it++) {
    //   // (lb, ub, 0.0 (added later through setObjective), GRB_CONTINUOUS
    //   //      string v_name = "AG_" + to_string(((*it).first)->agent_id)
    //   model.addVar(0.0, DBL_MAX, 0.0, GRB_CONTINUOUS, getVarName((*it).second));
    // }

    vertex_iter_t vi, vi_end;
    boost::tie(vi, vi_end) = vertices(csp_graph);
    for (; vi != vi_end; vi++) {
      model.addVar(0.0, DBL_MAX, 0.0, GRB_CONTINUOUS, getVarName(*vi));
    }

    // Integrate new variables
    model.update();

    GRBVar grb_X0 = model.getVarByName(getVarName(X0));
    GRBVar grb_XF = model.getVarByName(getVarName(XF));
    GRBLinExpr opt;
    // Iterate through the edges in the graph
    edge_iter_t ei, ei_end;
    tie(ei, ei_end) = boost::edges(csp_graph);
    for (; ei != ei_end; ++ei) {
      Graph_Edge_t e_desc = *ei;
      Graph_Vertex_t v_source = boost::source(e_desc, csp_graph);
      Graph_Vertex_t v_target = boost::target(e_desc, csp_graph);
      GRBVar grb_v_source = model.getVarByName(getVarName(v_source));
      GRBVar grb_v_target = model.getVarByName(getVarName(v_target));
      model.addConstr(grb_v_target - grb_v_source >= csp_graph[e_desc].lb, getEdgeName(e_desc));
      model.addConstr(grb_v_target - grb_v_source <= csp_graph[e_desc].ub, getEdgeName(e_desc));

      if (csp_graph[v_target].agent_id == -4) {
        opt = opt + (grb_v_source - grb_X0);
        if (_objective == Hybrid) {
          model.addConstr(grb_v_source - grb_X0 <= grb_XF - grb_X0);
        }
      }

      if (!_disableVminMaximization && csp_graph[e_desc].translation > 0) {
        model.addConstr(grb_v_target - grb_v_source <= csp_graph[e_desc].translation * RminV_val);
      }

      //      cout << "EDGE between: " << getVarName(&csp_graph[v_source]) << " -- " << getVarName(&csp_graph[v_target])
      //           << " ; WITH CONSTRAINT: [" << csp_graph[e_desc].lb << "," << csp_graph[e_desc].ub << "]" << endl;
    }
    // set objective (minimize makespan)
    switch (_objective)
    {
    case MinMakeSpan:
      {
        std::cout << "minimize makespan" << std::endl;
        model.setObjective(grb_XF - grb_X0, GRB_MINIMIZE);
      }
      break;
    case MaxThroughput:
      {
        std::cout << "maximize throughput" << std::endl;
        model.setObjective(opt, GRB_MINIMIZE);
      }
      break;
    case Hybrid:
      {
        std::cout << "hybrid optimization" << std::endl;
        model.setObjective(opt + 1000 * (grb_XF - grb_X0), GRB_MINIMIZE);
      }
      break;
    // case MaxMinVelocity:
    //   {
    //     std::cout << "hybrid optimization" << std::endl;
    //     model.setObjective(RminV + 1000 * (grb_XF - grb_X0), GRB_MINIMIZE);
    //   }
    //   break;
    }

    // Optimize model
    model.optimize();
    // Print results
    // cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    // update the results in paths_speeds
    paths_temporal.resize(num_of_agents);
    for (size_t ag = 0; ag < num_of_agents; ag++)
      paths_temporal[ag] = new vector< constrainedPathEntry >();

    boost::tie(vi, vi_end) = vertices(csp_graph);
    for (; vi != vi_end; vi++) {
    // for (it=cspNodes.begin(); it != cspNodes.end(); it++) {
      string v_name = getVarName(*vi);  // v_name = AG_{}_L1_{}_L2_{}_T_{}
      GRBVar v_grb = model.getVarByName(v_name);
      double v_value = v_grb.get(GRB_DoubleAttr_X);
      // std::cout << v_name << std::endl;
      // if ( v_name.find("-1") != string::npos ) {  // if there's a -1 in the string (that is, this is a location)
        cout << v_name << " = " << v_value << endl;
        Graph_Vertex_t vertex = atoi(v_name.substr(2).c_str());
        VertexProperties* vp = &csp_graph[vertex];
        if (vp->agent_id >= 0) {
          paths_temporal.at(vp->agent_id)->push_back(constrainedPathEntry(vp->loc1, vp->x, vp->y, vp->z, vp->theta(), v_value)); //make_pair(ag_loc, v_value));
        }
    }
    // sort paths_temporal according to ascending time
    for (size_t ag = 0; ag < num_of_agents; ag++) {
      std::sort(
                paths_temporal[ag]->begin(),
                paths_temporal[ag]->end(),
                [](const constrainedPathEntry &lhs, const constrainedPathEntry &rhs) { return lhs.arrival < rhs.arrival; }
                );
    }
    // print paths_temporal
    for (size_t ag = 0; ag < num_of_agents; ag++) {
      cout << "AG: " << ag;
      for (size_t t = 0; t < paths_temporal[ag]->size(); t++)
        cout << " [" << paths_temporal[ag]->at(t).x
             << "," << paths_temporal[ag]->at(t).y
             << "," << paths_temporal[ag]->at(t).z
             << "," << paths_temporal[ag]->at(t).arrival << "] ";
      cout << endl;
    }

    // double RminV_val = RminV.get(GRB_DoubleAttr_X);
    std::cout << "RminV: " << RminV_val << std::endl;

    double maxv = discretePlan.agents[0].max_v;
    for (size_t ag = 1; ag < discretePlan.agents.size(); ag++) {
      maxv = std::max(maxv, discretePlan.agents[ag].max_v);
    }
	
	std::cout << "Guaranteed safety distance: " << 2*(1-delta) / (RminV_val * maxv) << std::endl;

    {
      using namespace pt;

      ptree pt;
      ptree agents;
      for (size_t ag = 0; ag < num_of_agents; ag++) {
        ptree agent;
        std::stringstream sstr;
        if (ag == 0) {
          sstr << "Quadricopter_target";
        } else {
          sstr << "Quadricopter_target_HASHMARK_" << ag - 1;
        }
        agent.put("name", sstr.str());
        agent.put("group", discretePlan.agents[ag].group);
        ptree path;
        for (size_t t = 0; t < paths_temporal[ag]->size(); t++) {
          ptree pathentry;
          pathentry.put("x", paths_temporal[ag]->at(t).x);
          pathentry.put("y", paths_temporal[ag]->at(t).y);
          pathentry.put("z", paths_temporal[ag]->at(t).z);
          pathentry.put("theta", paths_temporal[ag]->at(t).theta);
          pathentry.put("arrival", paths_temporal[ag]->at(t).arrival);
          path.push_back(std::make_pair("", pathentry));
        }
        agent.add_child("path", path);
        agents.push_back(std::make_pair("", agent));
      }
      pt.add_child("agents", agents);
      write_json(outputFile, pt);

    }

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

}

//vector< vector< pair<int, double> >* >& CreateCSP::getPathsTemporal() {
//  return paths_temporal;
//}


void CreateCSP::exampleLP() {
  try {
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

    // Create variables
    // (lb, ub, 0.0 (added later through setObjective), GRB_CONTINUOUS
    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

    // Integrate new variables

    model.update();

    // Set objective: maximize x + y + 2 z

    model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

    // Add constraint: x + 2 y + 3 z <= 4

    model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

    // Add constraint: x + y >= 1

    model.addConstr(x + y >= 1, "c1");

    // Optimize model

    model.optimize();

    cout << x.get(GRB_StringAttr_VarName) << " "
         << x.get(GRB_DoubleAttr_X) << endl;
    cout << y.get(GRB_StringAttr_VarName) << " "
         << y.get(GRB_DoubleAttr_X) << endl;
    cout << z.get(GRB_StringAttr_VarName) << " "
         << z.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }
}

CreateCSP::~CreateCSP() {
  for (auto path : paths_temporal) {
    delete path;
  }
}
