#!/bin/bash
#
# SETTING
DEBUG=false # if true, print statement without executing any command
TIME=flase # if true, print statement without executing any command

# 0. lidar control filemode change
chown firefly:firefly /sys/class/pwm/pwmchip1/pwm0/duty_cycle
chmod u+x /sys/class/pwm/pwmchip1/pwm0/duty_cycle

if [ "$TIME" == true ]; then
    start_time=$(date +%s.%N)
fi

# 1. check wifi connect
if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 1. Wifi Check - Start @@@@@"
fi

while :
do
    wifi_ip=$(ifconfig wlan0 | grep 'inet ' | awk '{print $2}')
    if [ ${wifi_ip} ]; then
        if [ "$DEBUG" == true ]; then
            echo "[auto_run_all.sh] wifi_ip : "${wifi_ip}
        fi
        break
    fi
    sleep 1
done

if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 1. Wifi Check - End   @@@@@"
fi

if [ "$TIME" == true ]; then
    second_start_time=$(date +%s.%N)
fi

# 2. ROS setup
if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 2. ROS setup - Start @@@@@"
fi

# cross Compile ros path setup
sed -i 's/\/home\/firefly\/toolchain//g' /home/firefly/catkin_ws/devel/lib/python2.7/dist-packages/ts800_ros/cfg/RobotParameterConfig.py
sed -i 's/\/home\/firefly\/toolchain//g' /home/firefly/catkin_ws/devel/share/ts800_ros/cmake/ts800_rosConfig.cmake
sed -i 's/\/home\/firefly\/toolchain//g' /home/firefly/catkin_ws/devel/_setup_util.py
source /opt/ros/melodic/setup.bash
source /home/firefly/catkin_ws/devel/setup.bash --extend
source /home/firefly/slam_ws/install_isolated/setup.bash --extend
export ROS_MASTER_URI=http://${wifi_ip}:11311
export ROS_IP=${wifi_ip}
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/firefly/catkin_ws/src
export ROS_PACKAGE_PATH=/home/firefly/catkin_ws/src:/home/firefly/slam_ws/install_isolated:$ROS_PACKAGE_PATH

if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 2. ROS setup - End   @@@@@"
fi

if [ "$TIME" == true ]; then
    third_start_time=$(date +%s.%N)
fi

# 3. ROS Master
if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 3. ROS Master - Start @@@@@"
fi

sleep 1
roscore &
until rostopic list > /dev/null 2>&1; do sleep 1;echo "[auto_run_all.sh] waiting ROS master ..."; done

if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 3. ROS Master - End   @@@@@"
fi

if [ "$TIME" == true ]; then
    fourth_start_time=$(date +%s.%N)
fi

# 4. Run Application
if [ "$DEBUG" == true ]; then
    echo "[auto_run_all.sh] @@@@@ 4. ROS Application - Start @@@@@"
fi

if [ "$TIME" == true ]; then
    wifi_check_duration=$(echo "$second_start_time - $start_time" | bc)
fi

if [ "$TIME" == true ]; then
    ros_setup_duration=$(echo "$third_start_time - $second_start_time" | bc)
fi

if [ "$TIME" == true ]; then
    ros_master_duration=$(echo "$fourth_start_time - $third_start_time" | bc)
fi

if [ "$TIME" == true ]; then
    echo "1. Wifi Check Duration: $wifi_check_duration seconds"
fi

if [ "$TIME" == true ]; then
    echo "2. ROS Setup Duration: $ros_setup_duration seconds"
fi

if [ "$TIME" == true ]; then
    echo "3. ROS Master Duration: $ros_master_duration seconds"
fi

sleep 1

if [ "$TIME" == true ]; then
    application_start_time=$(date +%s.%N)
fi

if [ "$TIME" == true ]; then
    echo "application start time: $application_start_time seconds"
fi

roslaunch ts800_ros all_ts800_ws_2_2.launch
