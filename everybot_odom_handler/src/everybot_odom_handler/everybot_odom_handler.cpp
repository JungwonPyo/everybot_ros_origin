#include "everybot_odom_handler/everybot_odom_handler.h"

namespace everybot
{

COdomHandler::COdomHandler(std::shared_ptr<ros::NodeHandle> nh,
                           std::shared_ptr<ros::NodeHandle> pnh): nh(nh), pnh(pnh)
{
    init();
    pnh->param("odomPubLoopRate",odomPubLoopRate,100.0);

    pubTsOdom       = std::make_shared<ros::Publisher>(
        nh->advertise<nav_msgs::Odometry>("/ts800_odom", 10)
    );

    subTrackedPose = std::make_shared<ros::Subscriber>(
        nh->subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 1, [this](const geometry_msgs::PoseStamped::ConstPtr& msg){
            this->trackedPoseCallback(*msg);
        })
    );

    subOdom = std::make_shared<ros::Subscriber>(
        nh->subscribe<nav_msgs::Odometry>("/odom", 1, [this](const nav_msgs::Odometry::ConstPtr& msg){
            this->odomCallback(*msg);
        })
    );

    pubTsOdomTimer  = std::make_shared<ros::Timer>(
        nh->createTimer(ros::Duration(1./odomPubLoopRate), std::bind(&COdomHandler::pubTsOdomCallback, this, std::placeholders::_1))
    );
}

COdomHandler::~COdomHandler()
{

}

void COdomHandler::init()
{
    info_odom.pubTimerHz      = 100;   // Odometry Topic 발행 타이머 실행주기
    info_odom.noDataCnt       = 0;
    info_odom.frameName       = "ts800_odom";
    info_odom.childFrameName  = "base_link";
 
    info_odom.newData.seq     = 0;
    info_odom.newData.time    = ros::Time::now();
    info_odom.newData.x       = 0.0;
    info_odom.newData.y       = 0.0;
    info_odom.newData.angle.x = 0.0;
    info_odom.newData.angle.y = 0.0;
    info_odom.newData.angle.z = 0.0;
    info_odom.newData.angle.w = 1.0;
    info_odom.oldData         = info_odom.newData;

    info_slam.newData.x       = 0.0;
    info_slam.newData.y       = 0.0;
    info_slam.newData.angle   = 0.0;
    info_slam.oldData         = info_slam.newData;

    lastTrackedPoseTime = ros::Time::now();
}

void COdomHandler::trackedPoseCallback(const geometry_msgs::PoseStamped& msgs)
{    
    info_slam.newData.x = msgs.pose.position.x;
    info_slam.newData.y = msgs.pose.position.y;
    info_slam.oldData   = info_slam.newData;
    lastTrackedPoseTime = ros::Time::now();
}

void COdomHandler::odomCallback(const nav_msgs::Odometry& msgs)
{
    info_odom.newData.bUpdate = true;
    info_odom.newData.time    = msgs.header.stamp;
    info_odom.newData.x       = msgs.pose.pose.position.x;
    info_odom.newData.y       = msgs.pose.pose.position.y;
    info_odom.newData.angle   = msgs.pose.pose.orientation;
    info_odom.oldData         = info_odom.newData;
}

void COdomHandler::pubTsOdomCallback(const ros::TimerEvent& event)
{
    double elapsed_time_since_last_tracked_pose = (ros::Time::now() - lastTrackedPoseTime).toSec();
    
    if ( info_odom.newData.bUpdate == true )
    {
        double dt = (ros::Time::now() - info_odom.newData.time).toSec();
        
        tsOdom.header.seq       = ++info_odom.newData.seq;
        tsOdom.header.stamp     = info_odom.newData.time;
        tsOdom.header.frame_id  = info_odom.frameName;
        tsOdom.child_frame_id   = info_odom.childFrameName;

        tPose fusionPose        = getFusionPose();

        tsOdom.pose.pose.position.x  = fusionPose.x;
        tsOdom.pose.pose.position.y  = fusionPose.y;
        tsOdom.pose.pose.position.z  = 0.0;
        tsOdom.pose.pose.orientation = info_odom.newData.angle;

        // odom에 속도 정보는 담지 않는다.
        tsOdom.twist.twist.linear.x  = 0.0;
        tsOdom.twist.twist.linear.y  = 0.0;
        tsOdom.twist.twist.angular.z = 0.0;

        info_odom.noDataCnt = 0;
        info_odom.newData.bUpdate = false;

        pubTsOdom->publish(tsOdom);
    }
    else
    {    
        if ( info_odom.noDataCnt >= 10 ) // 10번이상 안들어오면 debug print
        {
            ROS_WARN("[Odometry Node] No System Coordinate from Robot !");
            info_odom.noDataCnt = 0;
        }
        info_odom.noDataCnt++;
    }
}

tPose COdomHandler::getFusionPose()
{
    tPose ret;
    const double epsilon = 1e-6;

    if ((ros::Time::now() - lastTrackedPoseTime).toSec() > 1.0)
    {
        ret.x = info_odom.newData.x;
        ret.y = info_odom.newData.y;
        ROS_INFO_STREAM("Slam odom are not updated during 1 sec. Use Pixart odom :(" << info_odom.newData.x << " , " << info_odom.newData.y << ")");
    }
    else if(std::fabs(info_slam.oldData.x - info_slam.newData.x) > epsilon
         || std::fabs(info_slam.oldData.y - info_slam.newData.y) > epsilon)
    {
        ret.x = info_slam.newData.x;
        ret.y = info_slam.newData.y;
        ROS_INFO_STREAM("Update Slam odom. :(" << info_slam.newData.x << " , " << info_slam.newData.y << ")");
    }
    else
    {
        tPoint delta;
        delta.x = info_odom.newData.x - info_odom.oldData.x;
        delta.y = info_odom.newData.y - info_odom.oldData.y;

        ret.x = info_slam.oldData.x + delta.x;
        ret.y = info_slam.oldData.y + delta.y;
        ROS_INFO_STREAM("Update Fusion odom. :(" << info_slam.oldData.x+delta.x << " , " << info_slam.oldData.y+delta.y << ")");
    }

    return ret;
}

}