cmake_minimum_required(VERSION 3.0.2)
project(everybot_odom_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_msgs
  everybot_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES everybot_odom_handler
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf2_msgs everybot_msgs nav_msgs geometry_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/everybot_odom_handler.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/${PROJECT_NAME}/everybot_odom_handler.cpp
  src/${PROJECT_NAME}/everybot_odom_handler_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
