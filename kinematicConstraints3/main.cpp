#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "create_csp.h"
#include <cstdlib>
#include <cmath>

#include "boost/program_options.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
namespace pt = boost::property_tree;

#include "Timer.hpp"

using namespace std;

std::istream& operator>>(std::istream& in, CreateCSP::OptimizationObjective& objective)
{
    std::string token;
    in >> token;
    if (token == "MinMakeSpan")
        objective = CreateCSP::MinMakeSpan;
    else if (token == "MaxThroughput")
        objective = CreateCSP::MaxThroughput;
    else if (token == "Hybrid")
        objective = CreateCSP::Hybrid;
//    else throw boost::program_options::validation_error("Invalid unit");
    return in;
}


int main(int argc, char** argv) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  CreateCSP::OptimizationObjective objective;
  std::string agents;
  std::string schedule;
  double delta;
  bool disableVminMaximization;
  std::string outputFile;
  desc.add_options()
      ("help", "produce help message")
      ("agents,a", po::value<std::string>(&agents)->required(), "input file for agents (kinematic constraints)")
      ("schedule,s", po::value<std::string>(&schedule)->required(), "input file for discrete schedule")
      ("delta,d", po::value<double>(&delta)->default_value(0.5), "maximum allowed distance")
      ("objective", po::value<CreateCSP::OptimizationObjective>(&objective)->default_value(CreateCSP::MinMakeSpan)->multitoken(), "optimization objective")
      ("disableVminMaximization", po::bool_switch(&disableVminMaximization)->default_value(false), "Do not try to maximize Vmin first")
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
        state.x = item2.second.get<double>("x");
        state.y = item2.second.get<double>("y");
        state.z = item2.second.get<double>("z");
        state.orientation = (discretePlan::orientation_t)item2.second.get<int>("orientation");
        state.action = (discretePlan::action_t)item2.second.get<int>("action");
        state.locationId = item2.second.get<int>("locationId");

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

  cout << "reading end";

  {
    ScopedTimer timer;
    CreateCSP stp(plan, delta, objective, disableVminMaximization, outputFile);
  }

  return 0;

}
