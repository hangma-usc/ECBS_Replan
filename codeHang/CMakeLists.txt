cmake_minimum_required( VERSION 2.8 )

project( mapf )

# set(CMAKE_BUILD_TYPE Debug)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

FIND_PACKAGE( Boost 1.59 COMPONENTS program_options REQUIRED )

add_definitions(
    -DNDEBUG
)

add_executable( driver
    node.cpp
    cbs_node.cpp
    agents_loader.cpp
    map_loader.cpp
    single_type_search.cpp
    driver.cpp
    cbs_search.cpp
)

target_link_libraries( driver
    ${Boost_LIBRARIES}
)

target_compile_options( driver PUBLIC -std=c++11 -Wall )
