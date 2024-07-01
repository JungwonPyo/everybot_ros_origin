#!/bin/bash

NC='\033[0m'       # Text Reset
Black='\033[0;30m'        # Black
BGreen='\033[1;32m'       # Green
BYellow='\033[1;33m'      # Yellow

# check ros log file size
check_ros_log_file() {
    echo -e "${Black}Check Ros Log File ... ##################"

    k_size_log=$(rosclean check | grep K | awk '{print $1}')
    if [ ${k_size_log} ]; then
        echo -e "log size: ${BYellow}${k_size_log}${NC}"
        return 1
    fi

    m_size_log=$(rosclean check | grep M | awk '{print $1}')
    if [ ${m_size_log} ]; then
        echo -e "log size: ${BYellow}${m_size_log}${NC}"
        return 1
    fi

    g_size_log=$(rosclean check | grep G | awk '{print $1}')
    if [ ${g_size_log} ]; then
        echo -e "log size: ${BYellow}${g_size_log}${NC}"
        return 1
    fi
    echo -e "Log Size is not Big... ${BGreen}Good!${NC}"
    return 0
}


while :
do
    check_ros_log_file
    if [ $? -eq 1 ]; then
        rm -rf /home/ebot/.ros/log
        echo -e "--> ${BGreen}rm -rf /home/ebot/.ros/log${NC}"
    fi
    sleep 1h
done