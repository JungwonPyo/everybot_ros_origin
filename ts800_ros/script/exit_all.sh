#!/bin/bash

NC='\033[0m'       # Text Reset
Black='\033[0;30m'        # Black
BGreen='\033[1;32m'       # Green
BYellow='\033[1;33m'      # Yellow

# Kill Process Function
function kill_process {
    echo -e "[exit_all.sh] kill_process(): Search process [${BGreen}"$1"${NC}]..."
    #ppid=$(pgrep $1 | ps aux | grep -i $1 | grep -v grep)
    ppid=$(pgrep $1 | ps aux | grep -i $1 | grep -v grep)

    if [[ -z $ppid ]]; then
        echo -e "[exit_all.sh] kill_process(): ${BYellow}Search process is not running${NC}"
    else
        echo -e "[exit_all.sh] kill_process(): ${BGreen}kill -9 "${ppid}"${NC}"
        kill -9 $ppid
    fi

    while :
    do
        if [[ -z $(pgrep $1) ]]; then
            break
        fi
        echo -e "[exit_all.sh] kill_process(): ${Black}wait to killed...${NC}"
        sleep 1
    done
    echo -e "[exit_all.sh] kill_process(): end"
}

###############################################


# Process Names to killed
process_names="\
ts800_app \
everybot_odom_handler \
everybot_msgs \
static_transform_publisher \
imu_filter_node \
system_monitor_node \
ts800_lidar_filter_node \
cartographer_node \
cartographer_occupancy_grid_node \
roscore \
rosmaster \
rosout\
roslaunch \
"
echo -e "[exit_all.sh] --- process list to kill"
for process_name in $process_names; do
    echo -e "[exit_all.sh]" $process_name
done
echo -e "[exit_all.sh] ------------------------"
###############################################


# Process Names -> kill process()
for process_name in $process_names; do
    kill_process $process_name
    echo -e "[exit_all.sh] ====================="
done
echo -e "[exit_all.sh] exit all.launch end !!"
