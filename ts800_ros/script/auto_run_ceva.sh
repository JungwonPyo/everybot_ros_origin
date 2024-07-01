#!/bin/bash

# 1. check wifi connect
while :
do
    wifi_ip=$(ifconfig wlan0 | grep 'inet ' | awk '{print $2}')

    if [ ${wifi_ip} ]; then
        echo "[auto_run_all.sh] "
        echo "[auto_run_all.sh] wifi_ip : "${wifi_ip}
        break
    fi
    echo "[auto_run_all.sh] wait wifi..."
    sleep 1
done
echo "[auto_run_all.sh] "
echo "[auto_run_all.sh] === wifi connected! ==="
echo "[auto_run_all.sh] "


# 2. ROS setup
source /opt/ros/melodic/setup.bash
source /home/ebot/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://${wifi_ip}:11311
export ROS_IP=${wifi_ip}


# 3. Run Application
sleep 1
roslaunch ts800_ros ceva.launch