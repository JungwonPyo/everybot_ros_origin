/**
 * @file robotSlamPose.h
 * @author yoon
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"

#include <iostream>
#include <mutex>

typedef struct
{
    tPose pauseSlamPose;
    tPose pauseSystemPose;
} tPausePose;

class CRobotSlamPose
{
private:
    /* 로봇의 SLAM/SYSTEM PASUE STATE Last 자기 위치 좌표 */
    tPausePose pausePose;

public:
    CRobotSlamPose();
    ~CRobotSlamPose();

    void setPauseSlamPose(tPose set);
    void setPauseSlamPose(double x, double y, double yaw);

    void setPauseSystemPose(tPose set);
    void setPauseSystemPose(double x, double y, double yaw);

    tPose getPauseSlamPose();
    tPose getPauseSystemPose();
};