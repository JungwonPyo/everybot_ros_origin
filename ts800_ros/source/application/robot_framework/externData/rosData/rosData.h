/**
 * @file rosData.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <mutex>

#include "rosInterface.h"
#include "gridmap.h"
#include "trajectory.h"
#include "coreData/serviceData/localization.h"


class CRosData : CRosInterface
{
private:
    
public:
    tPose       slamPose;
    tPose       robotPose;
    tSysIMU     imu;
    CRawGridMap gridMapRaw;
    CLidar      lidar;
    CTrajectory trajectory;
    std::string rosCmd;
    bool bSlamPoseUpdate;
    bool bFilterImuUpdate;
    tUpdateData<tTwist> velocityControl;
    tUpdateData<tPose> targetPose; // debug용 target 위치좌표
    tUpdateData<tPose> rvizGoalPose; // debug용 rviz Navigation Goal 좌표

public:
    CRosData(ros::NodeHandle _nh);
    ~CRosData();
    tPose getSlamPose();
    tPose getRobotPose();
    std::string getRosCmd();
    tSysIMU getFilterImu(); //override;
    bool getUpdateSlamPose() override;
    void setUpdateSlamPose(bool ret) override;
    bool isUpdateFilterImu();
    void setUpdateFilterImu(bool set);

private: //virtual
    void setSlamPose(double x, double y, double yaw) override;
    void setSlamMap(tGridmapInfo info, s8* data) override;
    void setLidarDist(double const *dist) override;
    void setRawLidarDist(sensor_msgs::LaserScan msg) override;
    void setTrajectory(std::list<tPoint> path) override;
    void setKeyCmd(std::string cmd) override;
    void setFilterImu(double r, double p, double y, double ax, double ay, double az) override;

    void setVelocityControl(double linearVelocity, double angularVelocity) override;
    void setTargetPose(tPose targetPose) override;
    void setRvizGoalPose(tPose rvizGoalPose) override;
    void setRobotPose(tPose robotPose) override;
};