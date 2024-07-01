/**
 * @file localization.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"
#include "position_estimator/lowPassFilter.h"
#include "position_estimator/angleExtrapolation.h"
#include "coreData/observer.h"

#include <iostream>
#include <mutex>
#include <sensor_msgs/LaserScan.h>

class CLidar
{
private:
    double lidarDist[LIDAR_DIST_BUFF_SIZE];
    sensor_msgs::LaserScan  rawLidarData;
    std::mutex lidarDataMutex;
    std::mutex rawLidarDataMutex;
    bool lidarinit;

public:
    CLidar(/* args */);
    ~CLidar();
    void setLidarDist(double const *dist);
    void setRawLidarDist(sensor_msgs::LaserScan msg);
    sensor_msgs::LaserScan getRawLidarDist();
    double *getLidarDist();
    double getLidarDist(int idx);
    void getNear(int *idx, double *dist);

    /**
     * @brief Get the Lidar Init object
     * 
     * @return true 
     * @return false 
     */
    bool getLidarInit(void);
};

class CFusionPoint
{
private:
    tPose oldSlam;
    tPose oldSystem;
    tPoint fusion;

public:
    CFusionPoint();
    ~CFusionPoint();
    void updatePose(tPose newSlam, tPose newSystem);
    tPoint getPoint();
};

class CRobotPose
{
private:
    double firstRosTime;
    tPose slamPose;
    tPose sysPose;
    tPose robotPose;
    CLowPassFilter lpFilter;
    CAngleExtrapolation angleExtrapolation; 
    double tempDegAngle; // 누적된 각도를 계산하기 위한 이전 각도
    double accDegAngle; // 누적된 각도 (단위 deg)
    double accRadAngle; // 누적된 각도 (단위 rad)

public:
    CRobotPose();
    ~CRobotPose();

    void setSlamPose(tPose set);
    void setSysPose(tPose set);
    void setAccAngle(double degAngle);
    void setPredictedAngle(double heading, ros::Time timestamp);
    void setRobotPose(tPose externalData);
    tPose getPose();
    tPose getSlamPose();
    tPose getSysPose();
    tPose getLpfPose();
    tPose getLpfInterPose();
    tPose getLpfExtraPose();
    double getAccDegAngle();
    double getAccRadAngle();
};

class CLocalizData : public CObserver
{
private:
    CRobotPose pose;    
    CFusionPoint fusionPoint;

public:
    CLocalizData();
    ~CLocalizData();
    void update(CExternData* pExternData) override;

    CLidar lidar;

    bool getLidarInit();
    double getLidarDist(int idx);
    sensor_msgs::LaserScan getRawLidarDist();
    double *getLidarBuffer();
    void setLidarBuffer(double const *buf);
    void setRawLidarDist(sensor_msgs::LaserScan msg);
    tPose getPose();
    tPose getSysPose();
    void setSlamPose(tPose set);
    tPose getSlamPose();
    tPose getLpfPose();
    tPose getLpfInterpolationPose();
    tPose getLpfExtrapolationPose();
    void setSlamPose(double x, double y, double yaw);    
};
