
#include <thread>
#include <mutex>
#include <math.h>

#include "boost/thread.hpp"

#include <iostream>
#include <cstdio> 
#include<stdlib.h>
#include<string.h>

#include "lidarFilter.h"

#define RSF_PI 3.14159265358979323846
#define RAD2DEG(x) ((x)*180./RSF_PI)

std::mutex sys_coord_mutex;
std::mutex scan_filter_mutex;

ros::Time last_odom_time;
double max_allowed_linear_velocity = 1.0; // 최대 허용 선속도 (예시로 1m/s로 설정)
double max_allowed_angular_velocity = 1.0; // 최대 허용 각속도 (예시로 1rad/s로 설정) //57.2958 = 1 rad, 85.9437 = 1.5 rad

CLidarFilter::CLidarFilter(ros::NodeHandle* _pNh) : pNh(_pNh)
{
    ROS_INFO("[Lidar Filter Node] Create");
    
    init();   
    sleep(1);
    ROS_INFO("[Lidar Filter Node] Ready");
}

CLidarFilter::~CLidarFilter()
{
    /* nothing */
}

void CLidarFilter::run()
{
    ROS_INFO("[Lidar Filter Node] Run");
    while( ros::ok() )
    {
        usleep(1000000 / (100*2.0)); // 5000 us = 5ms
        ros::spinOnce();    // ros topic Callback 처리 : 10ms
    }
}

void CLidarFilter::init()
{
    bisAbnormalMotion = false;

    //system pose 갑작스러운 거리 증가 필터링을 위한 임계값
    linear_threshold_ = 0.01;
    angular_threshold_ = 0.01;
    last_odom_time = ros::Time::now();

    //Lidar-> 갑작스러운 거리 증가 필터링을 위한 임계값 (2.0 미터)
    maxDistanceIncreaseThreshold = 2.0; 

    // subscriber
    scan_sub  = pNh->subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&CLidarFilter::scanCallback, this, _1));

/**
 * @brief It is inaccurate -> #if 0
 * 
 */
#if 0
    systemCoordSub  = pNh->subscribe<geometry_msgs::Vector3Stamped>("/q8/system_coord", 1, boost::bind(&CLidarFilter::systemCoordCb, this, _1));
#endif
    // 필터링된 라이다 스캔 데이터 발행
    scan_filtered_pub = pNh->advertise<sensor_msgs::LaserScan>("/scan/filtered", 1);
}

void CLidarFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &input_scan)
{
    std::lock_guard<std::mutex> lock(scan_filter_mutex);
    
    // 필터링된 스캔 데이터 생성 : 2m ???
    sensor_msgs::LaserScan filtered_scan = filterSuddenDistanceIncrease(*input_scan, maxDistanceIncreaseThreshold);

    // 필터링된 스캔 데이터 발행
    scan_filtered_pub.publish(input_scan);
}

/**
 * @brief 갑작스러운 거리 증가가 있는 부분을 필터링하는 함수
 * 
 * @param input_scan 
 * @param max_distance_increase_threshold 
 * @return sensor_msgs::LaserScan 
 */
sensor_msgs::LaserScan CLidarFilter::filterSuddenDistanceIncrease(const sensor_msgs::LaserScan& input_scan, double max_distance_increase_threshold) {
    sensor_msgs::LaserScan filtered_scan = input_scan;

    for (size_t i = 1; i < input_scan.ranges.size(); ++i)
    {
/**
 * @brief NaN 또는 Infinite 값이 아닌 경우에만 유지 
 * 
 */
#if 0
        double range = input_scan.ranges[i];
        if (!std::isnan(range) && std::isfinite(range))
#endif
        {
            double distance_increase = input_scan.ranges[i] - input_scan.ranges[i - 1];

            if (std::fabs(distance_increase) <= max_distance_increase_threshold) 
            {
                // 거리 변화가 임계값 이하인 경우에만 유지
                filtered_scan.ranges[i] = input_scan.ranges[i];
            }
        }
    }

    return filtered_scan;
}

/**
 * @brief 로봇의 비정상적인 움직임을 체크하는 함수
 * 
 * @param delta_linear_velocity 
 * @param delta_angular_velocity 
 * @param delta_distance 
 * @return true 
 * @return false 
 */
bool CLidarFilter::isAbnormalMotion(double delta_linear_velocity, double delta_angular_velocity, double delta_distance)
{
    bool nResult = false;
    //Delta Linear Velocity: 0.00 m/s, Delta Angular Velocity: 0.01 rad/s, Delta Distance : 0.01 : moving and turn
    //Delta Linear Velocity: 0.00 m/s, Delta Angular Velocity: 0.00 rad/s, Delta Distance : 0.01  : moving
    //STOP &  최대 허용 각속도 > 0.05~0.13   : OK   최대 허용 각속도  = 0.01~0.03
    ///double angular_velocity_threshold = 0.009; // 임의의 임계값 설정 -> 최대 허용 각속도 (예시로 1rad/s로 설정) : 57.2958 = 1 rad, 85.9437 = 1.5 rad

    // 선속도 변화량이 일정 값 이상인 경우 비정상적인 움직임으로 간주
    double angular_velocity_threshold = 0.01;
    if (std::abs(delta_angular_velocity) < angular_velocity_threshold && delta_angular_velocity != 0)
    {
        ROS_WARN("Abnormal motion detected-case1!");
        nResult = true;
    }

    // 각속도 변화량이 일정 값 이상인 경우 비정상적인 움직임으로 간주
    double linear_velocity_threshold = 0.01; // 임의의 임계값 설정 -> 최대 허용 선속도 1m/s
    if (std::abs(delta_linear_velocity) < linear_velocity_threshold && delta_linear_velocity != 0) 
    {
       ROS_WARN("Abnormal motion detected-case2!");
       nResult = true;
    }
 
    // 이동 거리가 일정 값 이상인 경우 비정상적인 움직임으로 간주
    //double distance_threshold = 0.06; // 임의의 임계값 설정
    //if (delta_distance > distance_threshold) {
    //    nResult = true;
    //}

    return nResult;
}

void CLidarFilter::systemCoordCb(const geometry_msgs::Vector3Stamped::ConstPtr &odom_msg)
{
    if (first_odom_) {
        last_odom_ = *odom_msg;
        first_odom_ = false;
        return;
    }
 
    // 이전 메시지와 현재 메시지의 선속도 및 각속도 계산
    double last_linear_velocity = std::hypot(last_odom_.vector.x, last_odom_.vector.y);
    double last_angular_velocity = last_odom_.vector.z;
    double current_linear_velocity = std::hypot(odom_msg->vector.x, odom_msg->vector.y);
    double current_angular_velocity = odom_msg->vector.z; //angle

    // 선속도 및 각속도 변화량 계산
    double delta_linear_velocity = current_linear_velocity - last_linear_velocity;
    double delta_angular_velocity = current_angular_velocity - last_angular_velocity;

    // 이동 거리 계산
    double delta_x = odom_msg->vector.x - last_odom_.vector.x;
    double delta_y = odom_msg->vector.y - last_odom_.vector.y;
    double delta_distance = std::hypot(delta_x, delta_y);

    // 변화량 출력
    ROS_WARN("Delta Linear Velocity: %.3f m/s, Delta Angular Velocity: %.3f rad/s, Delta Distance : %.3f ", delta_linear_velocity, delta_angular_velocity, delta_distance);

    // 비정상적인 움직임 체크
    if (isAbnormalMotion(delta_linear_velocity, delta_angular_velocity, delta_distance))
    {
        // 비정상적인 움직임에 대한 추가 작업 수행 가능
        // 예: 경고 메시지 출력, 특정 토픽으로 publish, 로그 파일 기록 등
        bisAbnormalMotion = true;
    }
    else
    {
        bisAbnormalMotion = false;
    }

    // 현재 메시지를 이전 메시지로 업데이트
    last_odom_ = *odom_msg;
}
