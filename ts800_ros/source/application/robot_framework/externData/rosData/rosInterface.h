/**
 * @file rosInterface.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <pthread.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"
#include <dynamic_reconfigure/server.h>
#include <ts800_ros/RobotParameterConfig.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include "gridmap.h"
#include "commonStruct.h"
#include "rosParameter.h"

static bool getLocalization(tf::TransformListener *tf_listener,
    std::string frame_base,
    std::string frame_target,
    geometry_msgs::PoseStamped *pose);

class CRosInterface
{
public:
    CRosInterface(ros::NodeHandle _nh);
    ~CRosInterface();

private:
    ros::NodeHandle nh;    
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_target_sub_;
    ros::Subscriber cmd_dir_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber trajectoryNodeSub;
    ros::Subscriber appControlSub;
    ros::Subscriber lidar_speed_sub_;
    ros::Subscriber rvizGoalSub;
    ros::Subscriber robotPoseSub;

    //slam of the pose topic : publish_tracked_pose(6hz)
    ros::Subscriber slam_track_pose_sub_;

    //walltime of the list	
    std::vector<::ros::WallTimer> wall_timers_;

    ros::Subscriber imu_filter_sub_;
    ros::Subscriber goal_point_sub_;
    pthread_t thRosCallback;

    tf::TransformListener tfListener_;

    tf::TransformListener tf_lstr_;
    dynamic_reconfigure::Server<ts800_ros::RobotParameterConfig>* dsrv_;

    bool flag_cmdvel_update_;           // cmd_vel 값 업데이트 확인 변수

    float linear_velocity, angular_velocity;  // sendMessage로 전달할 cmd_vel 값
    float linearVelocity, angularVelocity;  // sendMessage로 전달할 cmd_vel 값
    int occupancy_grid_idx;

    int laser_scan_inf_count; // 무한대 값을 가지는 데이터 포인트의 개수
    std::mutex lidar_data_mutex;

    double lastTimeLidarData;
    double lastTimeGridmapData;
    bool isMapUpdateDone;
    ros::Time lastGridUpdate;
    ros::Time TrackedPoseUpdateTime;

private: //virtual
    virtual void setSlamPose(double x, double y, double yaw) = 0;
    virtual void setSlamMap(tGridmapInfo info, s8* data) = 0;
    virtual void setLidarDist(double const *dist) = 0;
    virtual void setRawLidarDist(sensor_msgs::LaserScan msg) = 0;
    virtual void setTrajectory(std::list<tPoint> path) = 0;
    virtual void setKeyCmd(std::string cmd) = 0;
    virtual void setFilterImu(double r, double p, double y, double ax, double ay, double az) = 0;
    virtual void setTargetPose(tPose targetPose) = 0;
    virtual void setRvizGoalPose(tPose rvizGoalPose) = 0;
    virtual void setRobotPose(tPose robotPose) = 0;

    virtual void setUpdateSlamPose(bool ret) = 0;
    virtual bool getUpdateSlamPose() = 0;
    
private:/* ros message callback functionros message callback function */
    virtual void setVelocityControl(double linearVelocity, double angularVelocity) = 0;

private: /* ros message callback functionros message callback function */
    void ros_cmdvelCB_(const geometry_msgs::Twist::ConstPtr &msg);
    void ros_cmdtargetCB_(const geometry_msgs::Vector3::ConstPtr &msg);
    void ros_cmddirCB_(const std_msgs::String::ConstPtr &msg);
    void ros_scanCB_(const sensor_msgs::LaserScan::ConstPtr &msg);
    void ros_mapCB_(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void lidar_speedCB_(const std_msgs::Float64::ConstPtr &msg);
    void rvizGoalCb(const geometry_msgs::PoseStampedConstPtr &msg);
    void robotPoseCB(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void>>>& msg_ptr);
    void trajectoryCb(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void walltimerGetTf_(const ros::WallTimerEvent&);
    void missingDataTimer(const ros::WallTimerEvent&);
    void walltimerGetTfBaseLink2Laser(const ros::WallTimerEvent&);
    void imuDataCb(const sensor_msgs::Imu::ConstPtr &imusmsg);
    void goalPointCb(const geometry_msgs::PoseStamped::ConstPtr &goalPoint);
    void slamTrackedPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    int getLaserInfMaxCount();

    static void* threadRosCallbackWrap(void* arg)
    {
        CRosInterface* myRosInterface = static_cast<CRosInterface*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_ROS_CALLBACK);
        myRosInterface->threadRosCallback();
    }
    void threadRosCallback();

    void reconfigureCB(ts800_ros::RobotParameterConfig &config, uint32_t level);
    void parameterInit();

    //FOR_.map topic to png files
    void rosParameterServerInit();
    bool occupancyGridCreateDir();
    void occupancyGridToFile(const nav_msgs::OccupancyGrid::ConstPtr &msg);

public:
    void init();
    bool isRosOk();
};
