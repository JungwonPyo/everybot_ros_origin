#include "odometry.h"

//direct of get wheel + odom + pixart data from the system if 
#include "LibRobotControlInterface.h"
#include "LibRobotDataInterface.h"
#include "LibRobotConfigInterface.h"

extern CRobotControlInterface RoCtrInf;
extern CRobotDataInterface RoDatInf;
extern CRobotConfigInterface RoConfigInf; // hw config 모듈

COdometry::COdometry(ros::NodeHandle* _pNh) : pNh(_pNh)
{
    ROS_INFO("[ts800 Odometry Node] Create");
    init();

    initPubSubTimer();
    sleep(1);   // 초기 시스템 좌표 데이터가 없을 때 publish 방지 코드.
    ROS_INFO("[ts800 Odometry Node] Ready");
}

COdometry::~COdometry()
{

}

void COdometry::run()
{
    ROS_INFO("[ts800 Odometry Node] Run");
	//This node needs to publish at least at a rate of 20hz or navigational stack will complain
    ros::Rate loop_rate(50); // 20ms

    while( ros::ok() )
    {
        //usleep(1000000 / (info.pubTimerHz*2.0)); 5ms
        ros::spinOnce();    // ros topic Callback 처리
        loop_rate.sleep();
    }
}

void COdometry::init()
{
    info.pubTimerHz     = 100;   // Odometry Topic 발행 타이머 실행주기
    info.noDataCnt      = 0;
    info.frameName      = "odom";
    info.childFrameName = "base_link";

    info.newData.seq    = 0;
    info.newData.time   = ros::Time::now();
    info.newData.x      = 0.0;
    info.newData.y      = 0.0;
    info.newData.angle  = 0.0;
    info.oldData        = info.newData;
}

void COdometry::initPubSubTimer()
{
    odometry_publisher_timer = pNh->createWallTimer(ros::WallDuration(1.0/info.pubTimerHz), boost::bind(&COdometry::walltimerProc, this, _1));    
    odometryPub     = pNh->advertise<nav_msgs::Odometry>("odom", 10);
    systemCoordSub  = pNh->subscribe<geometry_msgs::Vector3Stamped>("/q8/system_coord", 10, boost::bind(&COdometry::systemCoordCb, this, _1));
}

/**
 * @brief 주기적으로 실행되는 타이머.
 * @param info.pubTimerHz 를 통해 주기가 결정됨.
 */
void COdometry::walltimerProc(const ros::WallTimerEvent&)
{
    float dt = (ros::Time::now() - info.newData.time).toSec(); // 마지막 시스템 좌표 데이터로 부터 경과된 시간

    if ( info.newData.bUpdate == true )
    {
        //map->odom->base_link -> laser / map -> base_link -> laser
        odometryPub.publish( getOdometryTopic() );  // Odometry Topic 발행       
        Tfbc.sendTransform( getOdometryTF() );      // Odometry TF Topic 발행 tf into the cartograper 
        info.noDataCnt = 0;
        info.newData.bUpdate = false;
    }
    else // 로봇으로 시스템 좌표가 안들어올때 debug print
    {    
        if ( info.noDataCnt >= 10 ) // 10번이상 안들어오면 debug print
        {
            ROS_WARN("[Odometry Node] No System Coordinate from Robot !");
            info.noDataCnt = 0;
        }
        info.noDataCnt++;
    }
}


void COdometry::shutdownWallTimer()
{
	odometry_publisher_timer.stop();
	usleep(200 * 1000);
}

/**
 * @brief 시스템 좌표 Topic 콜백함수.
 * 로봇으로부터 시스템 좌표를 받아야 함.
 * @param msg 
 */
void COdometry::systemCoordCb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    info.newData.bUpdate = true;

    info.newData.time   = msg->header.stamp;
    info.newData.x      = msg->vector.x;
    info.newData.y      = msg->vector.y;
    info.newData.angle  = msg->vector.z;
}

/**
 * @brief Odometry Topic 메세지 생성
 * 
 * @return nav_msgs::Odometry 
 */
nav_msgs::Odometry COdometry::getOdometryTopic()
{
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(info.newData.angle); // angle에 대한 quaternion 값
    double vX = 0.0, vY = 0.0, vAngle = 0.0;
    double dt = info.newData.time.toSec() - info.oldData.time.toSec();
    
    if ( dt != 0.0)
    {
        vX = ( info.newData.x - info.oldData.x ) / dt;
        vY = ( info.newData.y - info.oldData.y ) / dt;
        vAngle = ( info.newData.angle - info.oldData.angle ) / dt;  // TODO: angle이 절대각도 일수있음. 확인필요.
    }

    /* Odometry Topic 생성 */
    nav_msgs::Odometry odom;
    odom.header.seq             = ++info.newData.seq;
    odom.header.stamp           = info.newData.time;
    odom.header.frame_id        = info.frameName;
    odom.child_frame_id         = info.childFrameName;
    
    odom.pose.pose.position.x   = info.newData.x;
    odom.pose.pose.position.y   = info.newData.y;
    odom.pose.pose.position.z   = 0.0;
    odom.pose.pose.orientation  = quat;

    odom.twist.twist.linear.x   = vX;
    odom.twist.twist.linear.y   = vY;
    odom.twist.twist.angular.z  = vAngle;
    
    return odom;
}

/**
 * @brief Odometry TF 메세지 생성
 * 
 * @return geometry_msgs::TransformStamped 
 */
geometry_msgs::TransformStamped COdometry::getOdometryTF()
{
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(info.newData.angle); // angle에 대한 quaternion 값
    
    /* Odometry TF 생성 */
    geometry_msgs::TransformStamped tf;
    tf.header.seq             = info.newData.seq;
    tf.header.stamp           = info.newData.time;
    tf.header.frame_id        = info.frameName;
    tf.child_frame_id         = info.childFrameName;
    
    tf.transform.translation.x  = info.newData.x;
    tf.transform.translation.y  = info.newData.y;
    tf.transform.translation.z  = 0.0;
    tf.transform.rotation       = quat;
    
    return tf;
}
