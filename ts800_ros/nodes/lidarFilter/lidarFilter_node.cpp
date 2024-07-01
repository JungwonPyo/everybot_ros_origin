
#include "boost/thread.hpp"
#include <iostream>
#include<stdlib.h>
#include<string.h>


#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h" 

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"

// Global variables to store the latest scan and map messages
sensor_msgs::LaserScan::ConstPtr g_scan_msg;
nav_msgs::OccupancyGrid::ConstPtr g_map_msg;
ros::Time slamPoseUpdate;
static double hit_value;
static double scan_matches_threshold;

int submapListSize = 0;
bool isSlamRun = false;
ros::Time trackedPoseTime;
bool bTrackPoseUpdate;

// Utility function to convert LiDAR scan angle and distance to map coordinates
void scanToMapCoordinates(float angle, float distance, float robot_x, float robot_y, float robot_theta,
                          float map_resolution, float map_origin_x, float map_origin_y, int &map_x, int &map_y) {
    float world_x = robot_x + distance * cos(angle + robot_theta);
    float world_y = robot_y + distance * sin(angle + robot_theta);

    map_x = (world_x - map_origin_x) / map_resolution;
    map_y = (world_y - map_origin_y) / map_resolution;
}

// Function to check if LiDAR scan matches the occupancy grid map
bool isScanMatchingMap(const sensor_msgs::LaserScan& scan, const nav_msgs::OccupancyGrid& map, 
                       float robot_x, float robot_y, float robot_theta) {
    int width = map.info.width;
    int height = map.info.height;
    float map_resolution = map.info.resolution;
    float map_origin_x = map.info.origin.position.x;
    float map_origin_y = map.info.origin.position.y;
    
    int hit_count = 0;
    int total_count = 0;
    
    //ROS_WARN("width: %d, height: %d, map_resolution: %f, map_origin_x: %f, map_origin_y: %f", width, height, map_resolution, map_origin_x, map_origin_y);
    //ROS_WARN("robot_x: %f, robot_y: %f, robot_theta: %f", robot_x, robot_y, robot_theta);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max) {
            continue;
        }

        float angle = scan.angle_min + i * scan.angle_increment;
        int map_x, map_y;

        scanToMapCoordinates(angle, range, robot_x, robot_y, robot_theta, map_resolution, map_origin_x, map_origin_y, map_x, map_y);

        if (map_x >= 0 && map_x < width && map_y >= 0 && map_y < height) {
            int map_index = map_y * width + map_x;
            if (map.data[map_index] > hit_value) { // Occupied 확률  이상일 경우 장애물
                hit_count++;
            }
            total_count++;
        }
    }

    float match_ratio = static_cast<float>(hit_count / total_count);
    ROS_WARN("hit_count: %d, total_count: %d, Match Ratio: %f", hit_count, total_count, match_ratio);
    ROS_WARN("hit_value: %f, scan_matches_threshold: %f ", hit_value, scan_matches_threshold);
 
    // Determine if the scan matches the map
    return match_ratio > scan_matches_threshold; // Threshold can be adjusted
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    g_scan_msg = scan_msg;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    g_map_msg = map_msg;
}

void slamTrackedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (msg != nullptr)
    {
        if ( msg->header.stamp > trackedPoseTime )
        {
            //업데이트 시간 업데이트
            trackedPoseTime = msg->header.stamp;
            bTrackPoseUpdate = true;
        }
        else
        {
            bTrackPoseUpdate = false;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ts800_lidar_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); //paramter node handler

    if ( private_nh.getParam("hit_value", hit_value))
    {
        ROS_WARN("hit_value: %f", hit_value);
    }
    else
    {
        ROS_WARN("getParam(hit_value) is error ");
        hit_value = 4.0;
    }

    if ( private_nh.getParam("scan_matches_threshold", scan_matches_threshold))
    {
        ROS_WARN("scan_matches_threshold: %f", scan_matches_threshold);
    }
    else
    {
        ROS_WARN("getParam(scan_matches_threshold) is error ");
        scan_matches_threshold = 4.0;
    }

    // Subscribe to the scan and map topics
    ros::Subscriber lidar_scan_sub = nh.subscribe("/scan", 1, scanCallback);
    ros::Subscriber slam_map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber slam_track_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 5, slamTrackedPoseCallback);  //from of 

    ros::Publisher scan_matchs_result_pub = nh.advertise<std_msgs::Int32>("/scan_matchs/result", 1);

    tf::TransformListener listener;

    ros::Rate rate(5); // Check at 5 Hz
    while (ros::ok()) {
        if (bTrackPoseUpdate)
        {
            if (g_scan_msg && g_map_msg) {
                tf::StampedTransform transform;
                try {
                    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform); //ros::Time::now()

                    float robot_x = transform.getOrigin().x();
                    float robot_y = transform.getOrigin().y();
                    float robot_theta = tf::getYaw(transform.getRotation());
        
                    bool is_matching = isScanMatchingMap(*g_scan_msg, *g_map_msg, robot_x, robot_y, robot_theta);
                    ROS_WARN("Is scan matching the map? %s", (is_matching ? "Yes" : "No"));

                    int match_result = (is_matching ? 1 : 0);

                    //for_scan matchs result publish
                    std_msgs::Int32 msg;
                    msg.data = match_result;
                    scan_matchs_result_pub.publish(msg);
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

#if 0
#include "lidarFilter.h"

static bool test_ = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ts800_lidar_filter_node");
    ros::NodeHandle nh;
    
    CLidarFilter node = CLidarFilter(&nh);
    
    node.run();
    ros::shutdown();
}
#endif