#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>

#include "boost/program_options.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
namespace pt = boost::property_tree;

#include "discretePlan.h"
#include "temporalgraph.hpp"

#include <Eigen/Core>

#include "Timer.hpp"

using namespace std;

int getAgentLocation(int agent_id, size_t timestep, const discretePlan::discretePlan& discretePlan) {
    // if last timestep > plan length, agent remains in its last location
    if (timestep >= discretePlan.agents[agent_id].states.size())
        return discretePlan.agents[agent_id].states.back().locationId;
    // otherwise, return its location for that timestep
    return discretePlan.agents[agent_id].states[timestep].locationId;
}

int main(int argc, char** argv) {

#if 1
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string agents;
  std::string schedule;
  double delta;
  std::string outputFile;
  desc.add_options()
      ("help", "produce help message")
      ("agents,a", po::value<std::string>(&agents)->required(), "input file for agents (kinematic constraints)")
      ("schedule,s", po::value<std::string>(&schedule)->required(), "input file for discrete schedule")
      ("delta,d", po::value<double>(&delta)->default_value(0.5), "maximum allowed distance")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file for schedule")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
  }

  po::notify(vm);

  discretePlan::discretePlan plan;

  // read schedule
  pt::ptree tree;
  pt::read_json(schedule, tree);

  for (auto& item : tree.get_child("agents")) {
    plan.agents.resize(plan.agents.size() + 1);
    plan.agents.back().group = item.second.get<double>("group");
    for (auto& item2 : item.second.get_child("path")) {
        discretePlan::state state;
        state.position = Eigen::Vector3d(
            item2.second.get<double>("x"),
            item2.second.get<double>("y"),
            item2.second.get<double>("z"));
        state.orientation = (discretePlan::orientation_t)item2.second.get<int>("orientation");
        state.action = (discretePlan::action_t)item2.second.get<int>("action");
        state.locationId = item2.second.get<int>("locationId");
//        state.name = item2.second.get<std::string>("name");
        std::stringstream sstr;
        sstr << "AG: " << plan.agents.size() << " Loc: " << state.locationId << " (" << state.position.x() <<","<<state.position.y() << ")";
        state.name = sstr.str();

        plan.agents.back().states.push_back(state);
    }
  }

  // read kinematic constraints
  pt::ptree tree2;
  pt::read_json(agents, tree2);

  size_t i = 0;
  for (auto& item : tree2.get_child("agents")) {
    if (i < plan.agents.size()) {
      plan.agents[i].max_v = item.second.get<double>("max_v");
      plan.agents[i].max_w = item.second.get<double>("max_w");
    }
    ++i;
  }

  // CreateSTP stp(plan, delta, objective, disableVminMaximization);
#endif

//    double computedDelta = 2 * delta / sqrt(2);
//    if (computedDelta > 1) {
//        std::cout << "Impossible to achieve delta=" << delta << std::endl;
//        return 0;
//    }
//
//    std::cout << "Computed Delta on graph (assuming euclidean space): " << computedDelta << std::endl;
//    delta = computedDelta;

    std::cout << "Assuming unit-edge length and euclidean space: " << delta * sqrt(2) / 2 << std::endl;

    {
      ScopedTimer timer;

    std::map<std::pair<size_t, size_t>, STPSolver::EventId> eventMap;
    std::map<STPSolver::EventId, Eigen::Vector3d> locationMap;
    std::map<STPSolver::EventId, double> thetaMap;

    STPSolver::TemporalGraph<float> tg;
    // add markers and simple temporal constraints
//    for (auto& agent : plan.agents) {
    for (size_t ag = 0; ag < plan.agents.size(); ++ag) {
        const discretePlan::agent& agent = plan.agents[ag];

        STPSolver::EventId previousEvent = tg.startEvent();
//        for (auto& state : agent.states) {
        for (size_t i = 0; i < agent.states.size(); ++i) {
            STPSolver::EventId e = tg.addEvent(agent.states[i].name, i*10, ag * 5);
            eventMap[std::make_pair(ag, i)] = e;
            locationMap[e] = agent.states[i].position;
            thetaMap[e] = agent.states[i].theta();
            if (i > 0) {
                double translation = (agent.states[i].position - locationMap[previousEvent]).norm();
                double rotation = fabs(atan2(sin(agent.states[i].theta() - agent.states[i-1].theta()), cos(agent.states[i].theta() - agent.states[i-1].theta())));
                tg.addConstraint(previousEvent, e, translation/agent.max_v + rotation/agent.max_w, std::numeric_limits<float>::infinity());
            } else {
                tg.addConstraint(previousEvent, e, 0, 0);
            }
            previousEvent = e;
            // add markers if required
            if (i < agent.states.size() - 1) {
                double dist = (agent.states[i+1].position - agent.states[i].position).norm();
                for (size_t j = 1; j < dist/delta; ++j) {
                    std::stringstream sstr;
                    sstr << agent.states[i].name << "_M" << j;

                    STPSolver::EventId e = tg.addEvent(sstr.str(), i * 10 + j * 10 / (dist/delta), ag * 5);
                    locationMap[e] = agent.states[i].position + j * delta * (agent.states[i+1].position - agent.states[i].position).normalized();
                    thetaMap[e] = agent.states[i].theta();
                    tg.addConstraint(previousEvent, e, delta/agent.max_v, std::numeric_limits<float>::infinity());
                    previousEvent = e;
                }
            }

        }
        tg.addConstraint(previousEvent, tg.endEvent(), 0, std::numeric_limits<float>::infinity());
    }

    size_t maxt = 0;
    for (auto& agent : plan.agents) {
        maxt = std::max(maxt, agent.states.size());
    }

    // add temporal constraints
    for (size_t ag1 = 0; ag1 < plan.agents.size(); ag1++) {
        for (size_t t1 = 0; t1 < plan.agents[ag1].states.size()-1; t1++) {
            if (getAgentLocation(ag1, t1, plan) != getAgentLocation(ag1, t1+1, plan)) {  // skip wait/rotation
                for (size_t t2 = t1 + 1; t2 < maxt; t2++) {
                    for (size_t ag2 = 0; ag2 < plan.agents.size(); ag2++) {
                        if ( ag1 != ag2 ) {
                            if ( getAgentLocation(ag1,t1,plan) == getAgentLocation(ag2,t2,plan) ) {

                                size_t t2n = t2-1;
                                for (; getAgentLocation(ag2, t2, plan) == getAgentLocation(ag2, t2n, plan); t2n--)
                                {
                                }
                                t2n++;

                                for (size_t j = 0; j < 2; ++j) {
                                    tg.addConstraint(
                                            eventMap[std::make_pair(ag1, t1)] + j,
                                            eventMap[std::make_pair(ag2, t2n)] - 1 + j,
                                            0,
                                            std::numeric_limits<float>::infinity());
                                }

                                t2n = t2+1;
                                for (; getAgentLocation(ag2, t2, plan) == getAgentLocation(ag2, t2n, plan)
                                        && t2n < plan.agents[ag2].states.size(); t2n++)
                                {
                                }
                                t2n--;
                                if (t2n < plan.agents[ag2].states.size() - 1) {

                                    for (size_t j = 2; j < 1/delta; ++j) {
                                        tg.addConstraint(
                                                eventMap[std::make_pair(ag1, t1)] + j,
                                                eventMap[std::make_pair(ag2, t2n)] - 1 + j,
                                                0,
                                                std::numeric_limits<float>::infinity());
                                    }
                                }


                                // We are done for this ag1 at t1!
                                t2 = maxt;
                                break;
                            }
                        }
                    }

                }
            }
        }
    }


//    STPSolver::EventId e1 = tg.addEvent();
//    STPSolver::EventId e2 = tg.addEvent();
//    STPSolver::EventId e3 = tg.addEvent();
//    STPSolver::EventId e4 = tg.addEvent();
//
//    tg.addConstraint(tg.startEvent(), e2, 1, 2);
//    tg.addConstraint(e2, e4, 2, 5);
//    tg.addConstraint(tg.startEvent(), e4, 8, 100);
//    tg.addConstraint(e4, tg.endEvent(), 0, 0);//std::numeric_limits<float>::infinity());

    tg.solve();

    float lb, ub;
    tg.getResult(tg.endEvent(), lb, ub);
    std::cout << lb << " " << ub << std::endl;

    tg.saveDotFile("stp.dot");

    // write result
    {
        using namespace pt;

        ptree pt;
        ptree agents;
        for (size_t ag = 0; ag < plan.agents.size(); ++ag) {
            ptree agent;
            std::stringstream sstr;
            if (ag == 0) {
                sstr << "Quadricopter_target";
            } else {
                sstr << "Quadricopter_target_HASHMARK_" << ag - 1;
            }
            agent.put("name", sstr.str());
            agent.put("group", plan.agents[ag].group);
            ptree path;

            STPSolver::EventId start = eventMap[std::make_pair(ag, 0)];
            STPSolver::EventId end = eventMap[std::make_pair(ag, plan.agents[ag].states.size() - 1)];

            for (STPSolver::EventId id = start; id <= end; ++id) {
                ptree pathentry;
                const Eigen::Vector3d& loc = locationMap[id];
                float theta = thetaMap[id];
                float lb, ub;
                tg.getResult(id, lb, ub);

                pathentry.put("x", loc.x());
                pathentry.put("y", loc.y());
                pathentry.put("z", loc.z());
                pathentry.put("theta", theta);
                pathentry.put("arrival", lb);
                path.push_back(std::make_pair("", pathentry));
            }

            agent.add_child("path", path);
            agents.push_back(std::make_pair("", agent));
        }
        pt.add_child("agents", agents);
        write_json(outputFile, pt);

    }

  }


}
