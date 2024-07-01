#pragma once

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>

struct tOdometryData
{
    bool        bUpdate;// 업데이트:true  사용시:false

    uint32_t    seq;    // data 번호
    ros::Time   time;   // data 생성된 시간

    double      x;      // 단위 m
    double      y;      // 단위 m
    double      angle;  // 단위 rad


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

typedef struct _tPose
{
    _tPose() : x{0.0}, y{0.0}, angle{0.0} {}
    _tPose(double _x, double _y, double _angle) : x{_x}, y{_y}, angle{_angle} {}

    double x;   //단위 : m
    double y;   //단위 : m
    double angle;   //단위 : rad or deg.... -> 통일 필요

    bool operator==(const _tPose& __a)
    {
        return __a.x == this->x && __a.y == this->y && __a.angle == this->angle;
    }
}tPose;

class COdometry
{
public:
    COdometry(ros::NodeHandle* _pNh);
    ~COdometry();
    void run();

private:
    ros::NodeHandle*    pNh;
    ros::NodeHandle nh_local;

    tf::TransformBroadcaster Tfbc;
    ros::Publisher      odometryPub;    // SLAM 에서 사용할 Odometry Topic Publisher
    ros::Subscriber     systemCoordSub; // Odometry Topic 작성에 사용될 로봇 System 좌표계 Subscriber
    ros::Subscriber     odometryfilteredSub;
    ros::WallTimer      odometry_publisher_timer;

    tOdometryInfo       info;

    std::mutex mutex;
    tPose sysPoseData;

    void init();
    void initPubSubTimer();
    
    void walltimerProc(const ros::WallTimerEvent&);
    void systemCoordCb(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    nav_msgs::Odometry getOdometryTopic();
    geometry_msgs::TransformStamped getOdometryTF();

public:
    void shutdownWallTimer();
};
