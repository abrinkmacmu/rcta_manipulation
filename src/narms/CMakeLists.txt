cmake_minimum_required(VERSION 2.8.3)
project(narms)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rcta_msgs
  roman_client_ros_utils
  roscpp
  rospy
  rviz
  sbpl_collision_checking
  sbpl_geometry_utils
  trajectory_msgs
  visualization_msgs
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(narms
#   src/${PROJECT_NAME}/narms.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(narms ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ik_test src/test/ik_test.cpp)
target_link_libraries(ik_test 
  ${catkin_LIBRARIES}
)

add_executable(planning_test src/test/planning_test.cpp)
target_link_libraries(planning_test 
  ${catkin_LIBRARIES}
)

add_executable(pr2_planning_test src/test/pr2_planning_test.cpp)
target_link_libraries(pr2_planning_test 
  ${catkin_LIBRARIES}
)

add_executable(roman_planning_test src/test/roman_planning_test.cpp)
target_link_libraries(roman_planning_test 
  ${catkin_LIBRARIES}
)