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
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
add_service_files(
  FILES
  target_pose.srv
  gripper_command.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    moveit_msgs
)


###########
## Build ##
###########

include_directories(include)
include_directories(include/narms)
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



######################################
##  Executables
######################################

add_executable(state_machine 
  src/state_machine.cpp
  utils/narms_utils.cpp
  utils/states.cpp
  )
target_link_libraries(state_machine 
  ${catkin_LIBRARIES}
)
add_dependencies(state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

######################################
##  Nodes and Servers
######################################

add_executable(move_arm_server 
  src/move_arm_server.cpp
  utils/narms_utils.cpp
  )
target_link_libraries(move_arm_server 
  ${catkin_LIBRARIES}
)
add_dependencies(move_arm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gripper_server 
  src/gripper_server.cpp
  )
target_link_libraries(gripper_server 
  ${catkin_LIBRARIES}
)
add_dependencies(gripper_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(object_node 
  src/object_node.cpp
  utils/narms_utils.cpp
  )
target_link_libraries(object_node 
  ${catkin_LIBRARIES}
)
add_dependencies(object_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})






###########################
##  Testing  
###########################

add_executable(ik_test src/test/ik_test.cpp)
target_link_libraries(ik_test 
  ${catkin_LIBRARIES}
)

add_executable(move_arm_server_test 
  src/test/move_arm_server_test.cpp
  utils/narms_utils.cpp
  )
target_link_libraries(move_arm_server_test 
  ${catkin_LIBRARIES}
)
add_dependencies(move_arm_server_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})