#include "map_loader.h"
#include "agents_loader.h"
#include "single_type_search.h"
#include "cbs_search.h"
// #include "ecbs_search.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "cbs_node.h"
// #include "ecbs_node.h"
// #include "create_stp.h"
#include <cstdlib>
#include <fstream>

#include "boost/program_options.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace pt = boost::property_tree;

#include "Timer.hpp"

using namespace std;

int main(int argc, char** argv) {


  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string mapFile;
  std::string agentsFile;
  std::string outputFile;
  std::string statFile;
  desc.add_options()
      ("help", "produce help message")
      ("map,m", po::value<std::string>(&mapFile)->required(), "input file for map")
      ("agents,a", po::value<std::string>(&agentsFile)->required(), "input file for agents")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file for schedule")
      ("statFile", po::value<std::string>(&statFile), "output file for statistics")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
  }

  po::notify(vm);

  // read the map file and construct its two-dim array
  MapLoader ml(mapFile);
  // read agents' start and goal locations
  AgentsLoader al = AgentsLoader(agentsFile);

  Timer timer;
  CBSSearch cbs = CBSSearch(ml, al, 0, 0);
  bool res = cbs.runCBSSearch();
  timer.stop();
  std::cout << "Elapsed Time: " << timer.elapsedSeconds() << std::endl;
  if (statFile.size() > 0) {
    std::ofstream stream(statFile);
    stream << timer.elapsedSeconds() << std::endl;
  }

  if (res) {
    cout << "From Driver: Path found" << endl;
  }
  else {
    cout << "From Driver: NO Path found" << endl;
  }

  // write output file
  using namespace pt;

  ptree pt;
  if (res) {
    ptree agents;
    size_t num = 0;
    for (size_t type = 0 ; type < cbs.paths.size(); ++type) {
      for (size_t ag = 0; ag < cbs.paths[type]->size(); ag++) {
        ptree agent;
        std::stringstream sstr;
        sstr << "agent" << num;
        ++num;
        agent.put("name", sstr.str());
        agent.put("group", type);
        ptree path;
        size_t t = 0;
        int lastEntry = 0;
        for (int entry : cbs.paths[type]->at(ag)) {
          ptree pathentry;
          Location loc;
          ml.idxToLocation(entry, loc);
          pathentry.put("x", loc.x);
          pathentry.put("y", loc.y);
          pathentry.put("z", loc.z);
          pathentry.put("locationId", entry);
          pathentry.put("action", (t == 0 || entry == lastEntry) ? 0 : 1);
          pathentry.put("orientation", 1);

          path.push_back(std::make_pair("", pathentry));
          ++t;
          lastEntry = entry;
        }
        agent.add_child("path", path);
        agents.push_back(std::make_pair("", agent));
      }
    }
    pt.add_child("agents", agents);
  }
  write_json(outputFile, pt);

}
