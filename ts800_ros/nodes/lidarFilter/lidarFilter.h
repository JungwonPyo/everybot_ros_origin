#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Int32.h" //
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3Stamped.h"

using namespace std;

class CLidarFilter
{
public:
    CLidarFilter(ros::NodeHandle* _pNh);
    ~CLidarFilter();
    void run();

private:
    ros::NodeHandle*    pNh;
    ros::Subscriber     scan_sub; // ydlidar scan Subscriber
    ros::Subscriber     systemCoordSub; // 로봇 System 좌표계 Subscribe
    ros::Publisher      scan_filtered_pub; // filtered scan Publisher

    double maxDistanceIncreaseThreshold;

    geometry_msgs::Vector3Stamped last_odom_;
    bool first_odom_ = true;

    double linear_threshold_;
    double angular_threshold_;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double total_distance_ = 0.0;

    bool bisAbnormalMotion;

    void init();
    bool isAbnormalMotion(double delta_linear_velocity, double delta_angular_velocity, double delta_distance);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &input_scan);
    void systemCoordCb(const geometry_msgs::Vector3Stamped::ConstPtr &odom_msg);

    sensor_msgs::LaserScan filterSuddenDistanceIncrease(const sensor_msgs::LaserScan& input_scan, double max_distance_increase_threshold);
};