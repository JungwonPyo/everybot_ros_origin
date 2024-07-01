#ifndef __EVERYBOT_ODOM_HANDLER_H__
#define __EVERYBOT_ODOM_HANDLER_H__

#include "commonstruct.h"

#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <everybot_msgs/Pixart.h>
#include <everybot_msgs/ThreeSpinEncoder.h>

#include <sstream>
#include <memory>

namespace everybot
{

struct tOdometryData
{
    bool        bUpdate;// 업데이트:true  사용시:false

    uint32_t    seq;    // data 번호
    ros::Time   time;   // data 생성된 시간

    double      x;      // 단위 m
    double      y;      // 단위 m
    geometry_msgs::Quaternion angle;  // 단위 quaternion
};

struct tSlamData
{
    double      x;      // 단위 m
    double      y;      // 단위 m
    double      angle;  // 단위 quaternion
};

struct tOdometryInfo
{
    int pubTimerHz; // Odom Topic 발행주기 (단위 Hz)
    int noDataCnt;  // debug용 no data 카운트

    std::string frameName;
    std::string childFrameName;

    tOdometryData newData;
    tOdometryData oldData;
};

struct tSlamInfo
{
    tSlamData newData;
    tSlamData oldData;
};

class COdomHandler
{
private:
    std::shared_ptr<ros::NodeHandle> nh;
    std::shared_ptr<ros::NodeHandle> pnh;

    std::shared_ptr<ros::Publisher> pubTsOdom;

    std::shared_ptr<ros::Subscriber> subTrackedPose;
    std::shared_ptr<ros::Subscriber> subOdom;

    std::shared_ptr<ros::Timer> pubTsOdomTimer;

    double odomPubLoopRate;

    ros::Time lastTrackedPoseTime;
    nav_msgs::Odometry tsOdom;
    tOdometryInfo info_odom;
    tSlamInfo info_slam;

    void init();
    void trackedPoseCallback(const geometry_msgs::PoseStamped& msgs);
    void odomCallback(const nav_msgs::Odometry& msgs);
    void pubTsOdomCallback(const ros::TimerEvent& event);

    tPose getFusionPose();

public:
    COdomHandler(std::shared_ptr<ros::NodeHandle> nh,
                 std::shared_ptr<ros::NodeHandle> pnh);
    ~COdomHandler();
};
}
#endif