cmake_minimum_required(VERSION 2.8.3)
project(mpl_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(decomp_util REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(mpl_planner_node
  src/MPLPlannerNode.cpp
  src/mpl_planner_node.cpp
)
target_link_libraries(mpl_planner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

add_library(mpl_planner_nodelet SHARED
  src/MPLPlannerNode.cpp
  src/mpl_planner_nodelet.cc)

cs_install()

cs_export()
