cmake_minimum_required( VERSION 2.8 )

project( kinematicConstraints )

#set(CMAKE_BUILD_TYPE Debug)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
FIND_PACKAGE( Gurobi )
FIND_PACKAGE( Eigen3 )

include_directories(
    ${GUROBI_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_definitions(
    -DNDEBUG
)

add_executable( kinematicConstraints
    main.cpp
    create_stp.cpp
)

target_link_libraries( kinematicConstraints
    ${Boost_LIBRARIES}
    ${GUROBI_LIBRARIES}
)

target_compile_options( kinematicConstraints PUBLIC -std=c++11 -Wall )
