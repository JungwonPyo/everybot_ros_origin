#!/bin/bash

# 1. ROS setup
source /opt/ros/melodic/setup.bash
source /home/ebot/catkin_ws/devel/setup.bash

# 2. Run LiDAR
sleep 1
roslaunch ts800_ros 3i_lidar.launch