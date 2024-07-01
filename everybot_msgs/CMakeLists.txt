cmake_minimum_required(VERSION 3.0.2)
project(everybot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  Pixart.msg
  ThreeSpinEncoder.msg
  Frontier.msg
)

add_service_files(
  FILES
  Planning.srv
  Explorer.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  tf2_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs std_msgs tf2_msgs message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
