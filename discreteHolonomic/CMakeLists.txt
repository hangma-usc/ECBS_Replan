cmake_minimum_required( VERSION 2.8 )

project( discreteHolonomic )

#set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )

add_definitions(
    -DNDEBUG
)

add_executable( discreteHolonomic
    driver.cpp
    map_loader.cpp
    agents_loader.cpp
    egraph_reader.cpp
    compute_heuristic.cpp
    node.cpp
    single_agent_ecbs.cpp
    ecbs_node.cpp
    ecbs_search.cpp
)

target_link_libraries( discreteHolonomic
    ${Boost_LIBRARIES}
)

target_compile_options( discreteHolonomic PUBLIC -std=c++11 -Wall )
