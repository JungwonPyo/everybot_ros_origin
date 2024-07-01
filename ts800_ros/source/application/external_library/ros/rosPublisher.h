/**
 * @file rosPublisher.h
 * @author jspark
 * @brief
 * ros rviz에서 debug을 위한 클래스 입니다.
 * 사용선 init()을 반드시 처음에 한번 해주세요.
 * @version 0.1
 * @date 2024-01-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include "commonStruct.h"
#include "escapeWall.h"


#include "ts800_ros/SlamControl.h"

#include "ts800_ros/Feedback.h"
#include "ts800_ros/RotationVector.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h" 
#include <sensor_msgs/Imu.h>

#define ROS         CRosPublisher::getInstance()
#define DEBUG_PUB   debug::CRosPublisher::getInstance()

/**
 * @brief Cartographer SLAM을 위한 데이터 publish
 */
class CRosPublisher
{
public:
    struct tOdometry
    {
        std::mutex mutex;
        bool isUpdate;
        geometry_msgs::TransformStamped tf;
        nav_msgs::Odometry topic;
        tSysPose oldData;
    };

    struct tLidar
    {
        std::mutex mutex;
        bool isUpdate;
        sensor_msgs::LaserScan topic;

        geometry_msgs::TransformStamped staticTf;
    };

public:
    static CRosPublisher& getInstance();

private:
    CRosPublisher();
    CRosPublisher(const CRosPublisher& ref);
    CRosPublisher &operator=(const CRosPublisher &ref);
    ~CRosPublisher();
    void initPublisher();
    void initMessage();
    void publishOdometryTopicWithTf(const ::ros::WallTimerEvent& event);
    void publishLidarTopic(const ::ros::WallTimerEvent& event);
    void publishLidarTf(const ::ros::WallTimerEvent& event);

private:
    ros::NodeHandle* pNh;
    tf::TransformBroadcaster tfbc;
    std::vector<::ros::WallTimer> wallTimers;

    ros::Publisher pubOdometry;
    ros::Publisher pubLidar;
    tOdometry odom;
    
    tSysIMU oldSystemImu;
    tLidar lidar;
    double lidarOffset;

    //rbt plus of the ceva(temp)
    std::mutex feedbackMutex;
    std::mutex rotationVectorMutex;
    ros::Publisher pubRBTPlusFeedBack;
    ros::Publisher pubRBTPlusRotationVector;
    //카토그래퍼에서 서브 맵 업데이트 주기를 스위칭 할 수 있는 기능(기능만 만들어 놓고 비활성화 함)
    ros::Publisher pubSlamControl;
    std::mutex slamControlMutex;

    std::mutex submapFilterMutex;
    ros::Publisher pubSubmapFilter;
 

    std::mutex imuMutex;
    ros::Publisher pubImu;
public:
    void init(ros::NodeHandle* pNh);
    void updateOdometryData(tSysPose newData);
    void updateImuData(tSysIMU systemImu);
    void updateLidarData(const sensor_msgs::LaserScan& newScan);
    //카토그래퍼에서 서브 맵 업데이트 주기를 스위칭 할 수 있는 기능(기능만 만들어 놓고 비활성화 함)
    void pubSlamControlFunc(int max_time, int control_type);
    void updateSubmapFilter(int value);

    void updateLidarOffset(double value);

    //rbt plus of the ceva
    void pubRbtPlusFeedBack(double heading, unsigned int mcuTime);
    void pubRbtPlusRotationVector(tSysPose newData, unsigned int mcuTime);
};

/**
 * @brief ROS Rviz 에서 debug하기 위한 데이터는 모두 여기서 처리
 */
namespace debug
{
    class CRosPublisher
    {
    private:
        tf::TransformBroadcaster tfbc;

        CRosPublisher();
        CRosPublisher(const CRosPublisher& ref);
        CRosPublisher &operator=(const CRosPublisher &ref);
        ~CRosPublisher();
        void initPublisher();
        void setCleanAreaMarker();
        void setCleanLineMarker();
        visualization_msgs::Marker cleanAreaMarker;
        visualization_msgs::Marker cleanLineMarker;

        ros::NodeHandle* pNh;
        ros::Publisher pubCvImage;
        ros::Time lastTimeCvImage;
        ros::Publisher pubRawFrontiers;
        ros::Publisher pubTunedFrontiers;
        ros::Publisher pubWfTargetPoint;
        ros::Publisher pubWfStartPoint;
        ros::Publisher pubWfPoints;
        ros::Publisher pubGlobalPathPlan;
        ros::Publisher pubCurrentPathPlan;
        ros::Publisher pubEscapeWallCells;
        ros::Publisher pubCleanArea;
        ros::Publisher pubRobotPath;
        ros::Publisher pubCostMap; // 24.02.21 기준 dstar 장애물 아닙니다. simplifymap을 로봇 주위로 축소시킨 map 입니다.
        ros::Publisher pubDstarMap;
        ros::Publisher pubCleanMap;
        ros::Publisher pubContour;
        ros::Publisher pubContour2;
        ros::Publisher pubShrinkContour;
        ros::Publisher pubContourList;
        ros::Publisher pubCleanLine1;
        ros::Publisher pubCleanLine2;
        ros::Publisher pubDoor;
        ros::Publisher pubCleaningArea;
        ros::Publisher pubCleaningLine;
        ros::Publisher pubForbiddenArea;
        ros::Publisher pubForbiddenLine;      

    public:
        static CRosPublisher& getInstance();
        void init(ros::NodeHandle* pNh);
        void publishSimplifyMapImage(cv::Mat image, int throttleTime);
        void publishRawFrontiers(std::list<tPoint> points);
        void publishTunedFrontiers(std::list<tPoint> points);
        void publishWfTargetPoint(tPoint point);
        void publishWfStartPoint(tPoint point);
        void publishWfPoints(std::list<tPoint> points);
        void publishGlobalPathPlan(std::list<tPoint> path);
        void publishGlobalPathPlan(std::list<tPoint> path, const tPose& robotPose);
        void publishCurrentPathPlan(const std::list<tPoint>& path, const tPose& robotPose);
        void publishDstarMap(u8 *pSrc, tGridmapInfo* pGridmapInfo);
        void publishEscapeWallCells(std::deque<CEscapeWall::CellInfo> cells, std::deque<CEscapeWall::CellInfo> debugCells);
        void publishCleanArea(std::list<CCleanRoom> rooms, int currentRoomId);
        void publishRobotPath(std::list<tPoint> robotPath);
        void publishCostMap(tPose robotPose, std::list<tPoint> debugWallData);
        void publishContour(std::list<tPoint> contour);
        void publishContour2(std::list<tPoint> contour);
        void publishShrinkContour(std::list<tPoint> contour);
        void publishContourList(std::list<std::list<tPoint>> contour);
        void publishCleanLine1(tPose pose, tPoint point);
        void publishCleanLine2(tPose pose, tPoint point);
        void publishDoor(std::list<std::list<tPoint>> doors);
        void publishCleaningArea(tPose curPose);
        void publishCleaningLine(tPose curPose);
        void publishCleanMap(cell *cells, int orgWidth, int orgHeigh, int cropSize);
        void publishForbiddenLine(std::list<tPoint> line);
        void publishForbiddenRect(std::list<tPoint> rect);

    };
};
