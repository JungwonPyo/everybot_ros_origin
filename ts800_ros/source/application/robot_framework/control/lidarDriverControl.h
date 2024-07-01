/**
 * @file slamControl.h
 * @author yoon
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <ros/ros.h>

#include "commonStruct.h"

#include <mutex>
#include <iostream>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <termio.h>
#include <arpa/inet.h>
#include <thread>
#include "eblog.h"

#define GET_PARSER_DATA             _IOR('k', 1, int)
#define SET_LIDAR_GO                _IOR('k', 2, int)
#define SET_LIDAR_STOP              _IOR('k', 3, int)
#define TS800_LIDAR_DEVICE "/dev/ts800_lidar"
 
class CLidarDriverControl
{
private :
    int m_fd_driver;

public :
    CLidarDriverControl();
    ~CLidarDriverControl();

    int openLidarDriver(void);
    void closeLidarDriver(void);
    int getLidarDriver(void);
    void setEnableLidarMotor(bool set);

private:

};
