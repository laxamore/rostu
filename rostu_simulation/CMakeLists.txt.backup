cmake_minimum_required(VERSION 2.8.3)
project(rostu_simulation)

## add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  rospy
  roscpp
  roslib
  rviz
  move_base_msgs
  tf
  actionlib
)

find_package(OpenCV)
find_package(gazebo)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rostu_simulation
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
)

add_executable(field src/field.cpp)
add_executable(goal_call src/goal_call.cpp)

target_link_libraries(field
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
 ${gazebo_LIBRARIES}
)

target_link_libraries(goal_call
 ${catkin_LIBRARIES}
)
