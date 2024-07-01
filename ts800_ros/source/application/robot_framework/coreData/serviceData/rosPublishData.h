/**
 * @file rosPublishData.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

/**
 * @brief ServiceData에 존재하던 rosPublishData.h는 더이상 사용하지 않습니다.
 * ROS Topic Publish 기능은 rosPublisher.h 로 이동하였습니다.
 * 모든 Publish 기능이 rosPublisher.h로 옮겨지지 않아서,
 * 파일은 삭제하지 않고 남겨두었습니다. 필요하지 않으면 삭제해주세요.
 */
#if 0
class CServiceData;

#include <mutex>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>

#include "ts800_ros/Feedback.h"
#include "ts800_ros/RotationVector.h"

#include "obstaclemap.h"
#include "cleanRoom.h"
#include "imgProcessor.h"
#include "commonStruct.h"
#include "gridmap.h"
#include "escapeWall.h"

class CRosPublishData
{
private:
    /* data */
public:
    CRosPublishData(ros::NodeHandle _nh);
    ~CRosPublishData();
    void init();
    void pubLidarScan(sensor_msgs::LaserScan scanMsg);
    void pubLidarTf();
    void pubCleanMap(std::vector <cell> cells,int orgWidth, int orgHeigh);
    void pubCleanMap(cell *cells,int orgWidth, int orgHeigh, int cropSize);
    void pubCleanMap(std::vector <cell> cells);
    void pubLineFittingMap(u8 *pSrc, tGridmapInfo* pGridmapInfo);
    void pubObstaclemap(cell_obstacle* pCell);
    void pubNavigationGoal(double x, double y, double angle);
    void pubCancelGoal();
    void pubPose(geometry_msgs::Vector3Stamped pose);
    void pubOdometry(tSysPose oldData, tSysPose newData);
    void pubExplorer(std::list<tPoint> points);
    void pubExplorerRaw(std::list<tPoint> points);
    void pubDockingChargerPoint(tPose chargerPoint, bool bSet);
    void pubCleanArea(std::list<CCleanRoom> rooms, std::list<tPoint> currentAreaPolygons);
    void pubCleanArea(std::list<CCleanRoom> rooms, int currentRoomId);
    void pubRobotPose(tPose pose);
    void pubLpfRobotPose(tPose pose);
    void pubLpfInterpolationRobotPose(tPose pose);
    void pubLpfExtrapolationRobotPose(tPose pose);
    void pubIMU(tSysIMU systemImu, double angle);
    void pubTillState(int tillState);
    void pubTargetPose(tPose targetPose);
    void pubInitPoseSet(tPose initPose);
    
private:
    ros::NodeHandle nh;
    ros::Publisher pub_scan;
    ros::Publisher pub_pose;
    ros::Publisher pub_odometry;
    ros::Publisher pub_dometry;
    ros::Publisher pub_cleanmap;
    ros::Publisher pub_lineFittingMap;
    ros::Publisher pub_cv_image;
    ros::Publisher pub_dstarMap;
    ros::Publisher pub_obstaclemap;
    ros::Publisher pub_section;
    ros::Publisher pub_navigationGoal;
    ros::Publisher pub_cancelGoal;
    ros::Publisher pub_explorerPoints;
    ros::Publisher pub_explorerRawPoints;
    ros::Publisher pub_chargerPoint;
    ros::Publisher pub_cleanArea;
    ros::Publisher pub_robotPose;
    ros::Publisher pub_robotPose_lpf_raw;
    ros::Publisher pub_robotPose_lpf_inter;
    ros::Publisher pub_robotPose_lpf_extra;
    ros::Publisher pub_serviceMode;
    ros::Publisher pub_imu;
    ros::Publisher pub_tillState;
    ros::Publisher pub_targetPose;
    ros::Publisher pub_escapeWallCell;
    
    ros::Publisher pub_initialPose;
    //odom_node -> ts800_node
    tf::TransformBroadcaster Tfbc;
    uint32_t odometrySeq; // odometry topic과 tf의 seq
    geometry_msgs::TransformStamped lidarTf; // base_link -> lidar 의 Transform 메세지
};

#endif