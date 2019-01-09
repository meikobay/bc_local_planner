cmake_minimum_required(VERSION 2.8.3)
project(bc_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  tf
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)




 generate_dynamic_reconfigure_options(
   cfg/bc_local.cfg
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bc_local_planner
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy tf
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(bc_planner_node
    src/bc_planner_node.cpp
    src/bc_planner_ros.cpp
    src/occupy_map.cpp
    src/bc_planner_local.cpp
    src/bc_planner_global.cpp
    src/MCLearner.cpp
    include/bc_local_planner/bc_planner_ros.h
    include/bc_local_planner/occupy_map.h
    include/bc_local_planner/bc_planner_local.h
    include/bc_local_planner/bc_planner_global.h
    include/bc_local_planner/MCLearner.h)


 target_link_libraries(bc_planner_node
   ${catkin_LIBRARIES}
 )

