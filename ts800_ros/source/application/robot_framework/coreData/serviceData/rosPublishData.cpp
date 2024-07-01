/**
 * @brief ServiceData에 존재하던 rosPublishData.h는 더이상 사용하지 않습니다.
 * ROS Topic Publish 기능은 rosPublisher.h 로 이동하였습니다.
 * 모든 Publish 기능이 rosPublisher.h로 옮겨지지 않아서,
 * 파일은 삭제하지 않고 남겨두었습니다. 필요하지 않으면 삭제해주세요.
 */
#if 0

#include "coreData/serviceData/rosPublishData.h"
#include "coreData/serviceData.h"
#include "eblog.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "std_msgs/Int32.h" // 표준 자료형 메시지 패키지 포함
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "actionlib_msgs/GoalID.h"
#include "rosPublishData.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

using namespace cv;

CRosPublishData::CRosPublishData(ros::NodeHandle _nh)
{
    nh = _nh;
    init();
    eblog(LOG_LV, "");
}

CRosPublishData::~CRosPublishData()
{
    eblog(LOG_LV, "");
}

void CRosPublishData::init()
{
    pub_scan            = nh.advertise<sensor_msgs::LaserScan>("scan", 5);
    pub_pose            = nh.advertise<geometry_msgs::Vector3Stamped>("/q8/system_coord", 10);
    pub_odometry        = nh.advertise<nav_msgs::Odometry>("odom", 10);
    pub_cleanmap        = nh.advertise<nav_msgs::OccupancyGrid>("cleanmap/raw", 5);
    pub_lineFittingMap  = nh.advertise<nav_msgs::OccupancyGrid>("simplify_gridmap", 1);

    pub_cv_image  = nh.advertise<sensor_msgs::Image>("simplify_gridImage", 1);

    pub_obstaclemap     = nh.advertise<nav_msgs::OccupancyGrid>("/obstaclemap", 5);
    pub_section         = nh.advertise<std_msgs::Float64MultiArray>("/current_section", 5);
    pub_navigationGoal  = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
    pub_cancelGoal      = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 5);
    pub_explorerPoints  = nh.advertise<visualization_msgs::MarkerArray>("/exploerer/tuned", 5);
    pub_explorerRawPoints = nh.advertise<visualization_msgs::MarkerArray>("/exploerer/raw", 5);
    pub_chargerPoint    = nh.advertise<visualization_msgs::Marker>("/charger/charger_point", 5);
    pub_cleanArea       = nh.advertise<visualization_msgs::MarkerArray>("/q8/clean_area", 5);
    pub_robotPose           = nh.advertise<visualization_msgs::MarkerArray>("/robotPose", 5);
    pub_robotPose_lpf_raw   = nh.advertise<visualization_msgs::MarkerArray>("/robotPose/lowPassFilter/raw", 1);
    pub_robotPose_lpf_inter = nh.advertise<visualization_msgs::MarkerArray>("/robotPose/lowPassFilter/interpolation", 1);
    pub_robotPose_lpf_extra = nh.advertise<visualization_msgs::MarkerArray>("/robotPose/lowPassFilter/extrapolation", 1);
#if 0 //not_used(after uisng???)
    pub_imu            = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 50);
#endif
    pub_targetPose      = nh.advertise<visualization_msgs::Marker>("/motion/targetPose", 5);
    pub_escapeWallCell = nh.advertise<visualization_msgs::MarkerArray>("/escapeWall/cells", 5);
    //로봇 초기 위치 방향
    pub_initialPose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    #if 0 //after 0 -> 1
    pub_tillState      = nh.advertise<std_msgs::Int32>("/till_state", 1);
    #endif

    odometrySeq = 0;

    /* Transform Message 초기화 ("base_link" -> "laser") */
    lidarTf.header.seq             = 0;
    lidarTf.header.stamp           = ros::Time::now();
    lidarTf.header.frame_id        = "base_link";
    lidarTf.child_frame_id         = "laser";
    lidarTf.transform.translation.x  = -0.175;
    lidarTf.transform.translation.y  = 0.0;
    lidarTf.transform.translation.z  = 0.0;
    lidarTf.transform.rotation       = tf::createQuaternionMsgFromYaw(-2.88);

    eblog(LOG_LV, "");
}

void CRosPublishData::pubLidarScan(sensor_msgs::LaserScan scanMsg)
{
    pub_scan.publish(scanMsg);
}

void CRosPublishData::pubLidarTf()
{
    lidarTf.header.seq             = lidarTf.header.seq + 1;
    lidarTf.header.stamp           = ros::Time::now();
    Tfbc.sendTransform(lidarTf);
}

/**
 * @brief 
 * 
 * @param cells 
 * @param orgWidth 
 * @param orgHeigh 
 */
void CRosPublishData::pubCleanMap(std::vector <cell> cells,int orgWidth, int orgHeigh)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cleanmap.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    nav_msgs::OccupancyGrid map;
    map.header.frame_id             = "map";
    map.header.stamp                = ros::Time().now();
    map.info.resolution             = 0.05;
    map.info.width                  = orgWidth;
    map.info.height                 = orgHeigh;
    map.info.origin.position.x      = -25;
    map.info.origin.position.y      = -25;
    map.info.origin.position.z      = 0;
    map.info.origin.orientation.w   = 1;

    for(auto& elem : cells){
        map.data.emplace_back(elem.value);
    }
    
    pub_cleanmap.publish(map);        
}

/**
 * @brief 
 * 
 * @param cells 
 * @param orgWidth 
 * @param orgHeigh 
 * @param cropSize 
 */
void CRosPublishData::pubCleanMap(cell *cells,int orgWidth, int orgHeigh, int cropSize)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cleanmap.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    int idx = 0;    
    nav_msgs::OccupancyGrid map;
    map.header.frame_id             = "map";
    map.header.stamp                = ros::Time().now();
    map.info.resolution             = 0.05;
    map.info.width                  = orgWidth - (cropSize * 2) - 1;
    map.info.height                 = orgHeigh - (cropSize * 2) - 1;
    map.info.origin.position.x      = -25;
    map.info.origin.position.y      = -25;
    map.info.origin.position.z      = 0;
    map.info.origin.orientation.w   = 1;

    // crop
    for (int x = 0; x < CELL_X; x++){
        for (int y = 0; y < CELL_Y; y++){
            if ( x > cropSize && x < (CELL_X - cropSize) ){
                if ( y > cropSize && y < (CELL_Y - cropSize) ){
                    map.data.emplace_back(cells[idx].value);
                }
            }
            idx++;
        }
    }
    pub_cleanmap.publish(map);        
}


void CRosPublishData::pubCleanMap(std::vector <cell> cells)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cleanmap.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    nav_msgs::OccupancyGrid map;
    CImgProcessor imgProc;
    unsigned char src[CELL_SIZE] ={0,};
    int idx = 0;
    for(auto& elem : cells){
        src[idx++] = elem.value;
    }
    
    Mat crop = imgProc.getCleanMapBoundingRect(src);
    
    map.header.frame_id             = "map";
    map.header.stamp                = ros::Time().now();
    map.info.resolution             = 0.05;
    map.info.width                  = crop.rows;
    map.info.height                 = crop.cols;
    map.info.origin.position.x      = -25;
    map.info.origin.position.y      = -25;
    map.info.origin.position.z      = 0;
    map.info.origin.orientation.w   = 1;

    int numOfLines = crop.rows; // number of lines in the image
    int numOfPixels = crop.cols; // number of pixels per a line        
    unsigned char val = 0;
    for( int r = 0; r < numOfLines; r++ )
    {
        for( int c = 0; c < numOfPixels; c++ )
        {            
            val = crop.at<uchar>( r, c );
            map.data.emplace_back(val);
        }
    }        
    
    pub_cleanmap.publish(map);        
}

void CRosPublishData::pubLineFittingMap(u8 *pSrc, tGridmapInfo* pGridmapInfo)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_lineFittingMap.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    if ( pSrc == nullptr || pGridmapInfo == nullptr )
    {
        // ceblog(LOG_LV_NECESSARY, RED, "(pSrc or pGridmapInfo) is nullptr @@@");
        return;
    }

    nav_msgs::OccupancyGrid map;

    //init
    //map.data.resize(pGridmapInfo->width * pGridmapInfo->height);

    int size = pGridmapInfo->width * pGridmapInfo->height;
    int width = pGridmapInfo->width;
    int height = pGridmapInfo->height;
    for( int h=0; h<height; h++ )
    {
        for( int w=0; w<width; w++ )
        {
            map.data.emplace_back( pSrc[w+h*width] );
        }
    }

    map.header.frame_id             = "map";
    map.header.stamp                = ros::Time().now();
    map.info.resolution             = 0.05;
    map.info.width                  = width;
    map.info.height                 = height;
    map.info.origin.position.x      = pGridmapInfo->origin_x;
    map.info.origin.position.y      = pGridmapInfo->origin_y;
    map.info.origin.position.z      = 0;
    map.info.origin.orientation.w   = 1;
    
    pub_lineFittingMap.publish(map);
}

/**
 * @brief 
 * 
 * @param pCell 
 */
void CRosPublishData::pubObstaclemap(cell_obstacle* pCell)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_obstaclemap.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    nav_msgs::OccupancyGrid map;
    map.header.frame_id             = "map";
    map.header.stamp                = ros::Time().now();
    map.info.resolution             = CELL_OBS_RESOLUTION;
    map.info.width                  = CELL_OBS_WIDTH;
    map.info.height                 = CELL_OBS_HEIGHT;
    map.info.origin.position.x      = CELL_OBS_RESOLUTION * (CELL_OBS_WIDTH / 2) * -1;
    map.info.origin.position.y      = CELL_OBS_RESOLUTION * (CELL_OBS_HEIGHT / 2) * -1;
    map.info.origin.position.z      = 0;
    map.info.origin.orientation.w   = 1;

    //double debug_time = SYSTEM_TOOL.getSystemTime();//debug
    size_t data_size = CELL_OBS_SIZE;            
    for(int i=0; i<data_size; i++)  {map.data.emplace_back(pCell[i].value*90);}
    pub_obstaclemap.publish(map);        
}

void CRosPublishData::pubNavigationGoal(double x, double y, double angle)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_navigationGoal.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    geometry_msgs::PoseStamped goalPose;
    goalPose.header.frame_id = "map";
    goalPose.header.stamp = ros::Time::now();
    goalPose.pose.position.x = x;
    goalPose.pose.position.y = y;
    goalPose.pose.orientation.w = 1;
    pub_navigationGoal.publish(goalPose);
}

void CRosPublishData::pubCancelGoal()
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cancelGoal.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    actionlib_msgs::GoalID cancelGoal;
    cancelGoal.id = {};
    cancelGoal.stamp = ros::Time::now();
    pub_cancelGoal.publish(cancelGoal);
}

void CRosPublishData::pubPose(geometry_msgs::Vector3Stamped pose)
{
    /* robot system 좌표 publish */
    CStopWatch __debug_sw;

    pub_pose.publish(pose);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CRosPublishData::pubOdometry(tSysPose oldData, tSysPose newData)
{
    if(oldData.timeStamp == newData.timeStamp)  return;

    //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "RosTime["<<BOLDYELLOW<<newData.timeStamp<<BOLDBLACK<<"]\t"<<BOLDGREEN<<setw(6)<<newData.x<<","<<setw(6)<<newData.y<<","<<setw(6)<<newData.angle);

    odometrySeq++;
    double vX = 0.0, vY = 0.0, vAngle = 0.0;
    double dt = newData.timeStamp.toSec() - oldData.timeStamp.toSec();
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(newData.angle);
    if ( dt != 0.0)
    {
        vX = ( newData.x - oldData.x ) / dt;
        vY = ( newData.y - oldData.y ) / dt;
        vAngle = ( newData.angle - oldData.angle ) / dt;  // TODO: angle이 절대각도 일수있음. 확인필요.
    }

    /* Odometry Topic 생성 */
    nav_msgs::Odometry odom;
    odom.header.seq             = odometrySeq;
    odom.header.stamp           = newData.timeStamp;
    odom.header.frame_id        = "odom";
    odom.child_frame_id         = "base_link";

    odom.pose.pose.position.x   = newData.x;
    odom.pose.pose.position.y   = newData.y;
    odom.pose.pose.position.z   = 0.0;
    odom.pose.pose.orientation  = quat;

    odom.twist.twist.linear.x   = vX;
    odom.twist.twist.linear.y   = vY;
    odom.twist.twist.angular.z  = vAngle;
    pub_odometry.publish(odom);  // Odometry Topic 발행 

    /* Odometry TF 생성 */
    geometry_msgs::TransformStamped tf;
    tf.header.seq             = odometrySeq;
    tf.header.stamp           = newData.timeStamp;
    tf.header.frame_id        = "odom";
    tf.child_frame_id         = "base_link";
    tf.transform.translation.x  = newData.x;
    tf.transform.translation.y  = newData.y;
    tf.transform.translation.z  = 0.0;
    tf.transform.rotation       = quat;
    Tfbc.sendTransform(tf);

}

/**
 * @brief 
 * 
 * @param systemImu 
 */
void CRosPublishData::pubIMU(tSysIMU systemImu, double angle)
{
    //std::cout<<std::fixed<<std::setprecision(4);
#if 0
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_imu.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }
#endif
    CStopWatch __debug_sw;
    sensor_msgs::Imu imu;
    imu.header.seq++;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link"; //"imu_link";

#if 0 //not used of the cartograper, only kindnap of the trigger filter imu
    //ceva imu -> angle -> orientation of the Quaternion
    angle = (angle > 180) ? (angle-360) : angle;
    angle = angle * (3.141592 / 180.0);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angle); // angle에 대한 quaternion 값 <- robot of the direction
    imu.orientation = quat;
#endif 

#if 0 //use of the flat imu : for turtlebot3 of the imu data
    double accelerations_z = static_cast<double>(systemImu.Az);
    imu.linear_acceleration.x = 0.0; 
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 9.8; //accelerations_z/1000.0; //GRVATY = 9.8
 #else
    //systemImu:Accelerations = mm/s^2
    //systemImu:Gyro = radin/s x 1000
    //ros msg = Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    double accelerations_x = static_cast<double>(systemImu.Ax);
    double accelerations_y = static_cast<double>(systemImu.Ay);
    double accelerations_z = static_cast<double>(systemImu.Az);

    imu.linear_acceleration.x = accelerations_x/1000.0; 
    imu.linear_acceleration.y = accelerations_y/1000.0;
    imu.linear_acceleration.z = accelerations_z/1000.0;
#endif
 
    double pitch = static_cast<double>(systemImu.Gpitch);
    double roll = static_cast<double>(systemImu.Groll);
    double yaw = static_cast<double>(systemImu.Gyaw);

    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-acceleration:\t" << accelerations_x << ",\t" << accelerations_y << ",\t" << accelerations_z);
    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-velocity:\t" << pitch  << ",\t" << roll << ",\t" << yaw);

    imu.angular_velocity.x = pitch/1000.0;// roll axis
    imu.angular_velocity.y = roll/1000.0; // pitch axis
    imu.angular_velocity.z = yaw/1000.0; // yaw axis
 
    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-acceleration:\t" << imu.linear_acceleration.x << ",\t" << imu.linear_acceleration.y << ",\t" << imu.linear_acceleration.z);
    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-velocity:\t" << imu.angular_velocity.x  << ",\t" << imu.angular_velocity.y << ",\t" << imu.angular_velocity.z);

    pub_imu.publish(imu);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @param tillState 
 */
void CRosPublishData::pubTillState(int tillState)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_tillState.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }
 
    CStopWatch __debug_sw;
    // UNKNOW,
    // TILTED_UP,      //Tilting UP 완료 상태 
    // TILTED_DOWN,    //Tilting DOWN 완료 상태
    // TILING_UP,      //Tilting UP 중 상태
    // TILING_DOWN,    //Tilting DWON 중 상태
    // TILING_STOP,    //Tilting 중 STOP인 상태
    // TILT_ERROR      //Tilting 오류 있음
    std_msgs::Int32 msg;
    msg.data = tillState;
    pub_tillState.publish(msg);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CRosPublishData::pubExplorer(std::list<tPoint> points)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_explorerPoints.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::SPHERE;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.0;
    tmpMarker.color.g = 255;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.2;
    tmpMarker.scale.y = 0.2;
    tmpMarker.scale.z = 0.05;
    tmpMarker.pose.position.z = 0.0;

    for(tPoint point : points)
    {
        tmpMarker.pose.position.x = point.x;
        tmpMarker.pose.position.y = point.y;
        data.markers.emplace_back(tmpMarker);
        tmpMarker.id++;
    }

    // ROS_INFO("[app publish] system coord: %0.2lf\t%0.2lf\t%0.2lf", sys_coord.vector.x, sys_coord.vector.y, sys_coord.vector.z);
    pub_explorerPoints.publish(data);
}

void CRosPublishData::pubExplorerRaw(std::list<tPoint> points)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_explorerRawPoints.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::SPHERE;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.0;
    tmpMarker.color.g = 127;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.1;
    tmpMarker.scale.y = 0.1;
    tmpMarker.scale.z = 0.05;
    tmpMarker.pose.position.z = 0.0;

    for(tPoint point : points)
    {
        tmpMarker.pose.position.x = point.x;
        tmpMarker.pose.position.y = point.y;
        data.markers.emplace_back(tmpMarker);
        tmpMarker.id++;
    }

    // ROS_INFO("[app publish] system coord: %0.2lf\t%0.2lf\t%0.2lf", sys_coord.vector.x, sys_coord.vector.y, sys_coord.vector.z);
    pub_explorerRawPoints.publish(data);
}

/**
 * @brief 
 * 
 * @param chargerPoint 
 * 
 * @note 연산시간 ms
 * @date 2023-08-31
 * @author hhryu
 */
void CRosPublishData::pubDockingChargerPoint(tPose chargerPoint, bool bSet) // 차저 포인트는 추후 추가 및 position 수정 예정.
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_chargerPoint.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    CStopWatch __debug_sw;
    
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CUBE;
    if (bSet)
    {
        tmpMarker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
        tmpMarker.action = visualization_msgs::Marker::DELETE;
        // ceblog(LOG_LV_DOCKING, GREEN, "맵에서 충전기 위치 정보 삭제");
    }
    tmpMarker.color.r = 0.6;
    tmpMarker.color.g = 1.0;
    tmpMarker.color.b = 0.2;
    tmpMarker.color.a = 0.9;
    tmpMarker.scale.x = 0.35;
    tmpMarker.scale.y = 0.35;
    tmpMarker.scale.z = 0.2;
    tmpMarker.pose.position.x = chargerPoint.x;
    tmpMarker.pose.position.y = chargerPoint.y; 
    tmpMarker.pose.position.z = 0.1;

    pub_chargerPoint.publish(tmpMarker);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CRosPublishData::pubCleanArea(std::list<CCleanRoom> rooms, std::list<tPoint> currentAreaPolygons)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cleanArea.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;


    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::LINE_STRIP;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0;
    tmpMarker.color.g = 128;
    tmpMarker.color.b = 0;
    tmpMarker.color.a = 0.7;
    tmpMarker.scale.x = 0.05;
    tmpMarker.scale.y = 0.0;
    tmpMarker.scale.z = 0.0;
    tmpMarker.pose.orientation.w =1.0;

    geometry_msgs::Point p;
    double centerX = 0;
    double centerY = 0;
    for(CCleanRoom room : rooms)
    {
        for(CCleanArea area : room.getAreas())
        {
            if ( area.getCleanState() == E_CLEAN_STATE::UNCLEAN )
            {
                /* 영역 가운데 점을 기준으로 살짝 축소*/
                const double reduction = 0.1;
                centerX = 0;
                centerY = 0;
                for(tPoint polygon : area.getPolygons())
                {
                    centerX += polygon.x;
                    centerY += polygon.y;
                }
                centerX /= area.getPolygonSize();
                centerY /= area.getPolygonSize();

                for(tPoint polygon : area.getPolygons())
                {
                    if ( polygon.x > centerX)   { p.x=polygon.x-reduction; }
                    else                        { p.x=polygon.x+reduction; }
                    if ( polygon.y > centerY)   { p.y=polygon.y-reduction; }
                    else                        { p.y=polygon.y+reduction; }
                    p.z =0.0;
                    tmpMarker.points.emplace_back(p);
                }
                /* 청소할 영역 - 보라색 */
                tmpMarker.color.r = 128;
                tmpMarker.color.g = 0;
                tmpMarker.color.b = 128;

                tmpMarker.points.emplace_back(tmpMarker.points.front());
                data.markers.emplace_back(tmpMarker);
                tmpMarker.points.clear();
                tmpMarker.points.shrink_to_fit();
                tmpMarker.id++;
            }
            else if ( area.getCleanState() == E_CLEAN_STATE::COMPLETE )
            {
                for(tPoint polygon : area.getPolygons())
                {
                    p.x =polygon.x;
                    p.y =polygon.y;
                    p.z =0.0;
                    tmpMarker.points.emplace_back(p);
                }
                /* 청소 완료 영역 - 초록색 */
                tmpMarker.color.r = 0;
                tmpMarker.color.g = 128;
                tmpMarker.color.b = 0;

                tmpMarker.points.emplace_back(tmpMarker.points.front());
                data.markers.emplace_back(tmpMarker);
                tmpMarker.points.clear();
                tmpMarker.points.shrink_to_fit();
                tmpMarker.id++;
            }
        }
    }

    /* 현재 영역 - 노란색 */
    tmpMarker.color.r = 128;
    tmpMarker.color.g = 128;
    tmpMarker.color.b = 0;
    tmpMarker.color.a = 1.0;
    for(tPoint polygon : currentAreaPolygons)
    {
        p.x =polygon.x;
        p.y =polygon.y;
        p.z =0.0;
        tmpMarker.points.emplace_back(p);
    }
    tmpMarker.points.emplace_back(tmpMarker.points.front());
    data.markers.emplace_back(tmpMarker);

    pub_cleanArea.publish(data);
}


void CRosPublishData::pubCleanArea(std::list<CCleanRoom> rooms, int currentRoomId)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_cleanArea.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;

    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::LINE_STRIP;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0;
    tmpMarker.color.g = 128;
    tmpMarker.color.b = 0;
    tmpMarker.color.a = 0.7;
    tmpMarker.scale.x = 0.05;
    tmpMarker.scale.y = 0.0;
    tmpMarker.scale.z = 0.0;
    tmpMarker.pose.orientation.w =1.0;

    geometry_msgs::Point p;
    double centerX = 0;
    double centerY = 0;
    for(CCleanRoom room : rooms)
    {
        if (room.getId() == currentRoomId)
        {
            for(CCleanArea area : room.getAreas())
            {
                /* 청소할 영역 - 노란색 */
                tmpMarker.color.r = 128;
                tmpMarker.color.g = 128;
                tmpMarker.color.b = 0;
                tmpMarker.color.a = 1.0;
                
                for(tPoint polygon : area.getPolygons())
                {
                    p.x=polygon.x;
                    p.y=polygon.y;
                    p.z =0.0;
                    tmpMarker.points.emplace_back(p);
                }
                tmpMarker.points.emplace_back(tmpMarker.points.front());
                data.markers.emplace_back(tmpMarker);
                tmpMarker.points.clear();
                tmpMarker.points.shrink_to_fit();
                tmpMarker.id++;
            }
        }
        else
        {
            for(CCleanArea area : room.getAreas())
            {
                if ( area.getCleanState() == E_CLEAN_STATE::UNCLEAN )
                {
                    /* 청소할 영역 - 보라색 */
                    tmpMarker.color.r = 128;
                    tmpMarker.color.g = 0;
                    tmpMarker.color.b = 128;
                }
                else if ( area.getCleanState() == E_CLEAN_STATE::COMPLETE )
                {
                    /* 청소 완료 영역 - 초록색 */
                    tmpMarker.color.r = 0;
                    tmpMarker.color.g = 128;
                    tmpMarker.color.b = 0;
                }
                else
                {
                    /* ? 영역 - 하얀색 */
                    tmpMarker.color.r = 10;
                    tmpMarker.color.g = 10;
                    tmpMarker.color.b = 10;
                }

                /* 영역 가운데 점을 기준으로 살짝 축소*/
                const double reduction = 0.05;
                centerX = 0;
                centerY = 0;
                for(tPoint polygon : area.getPolygons())
                {
                    centerX += polygon.x;
                    centerY += polygon.y;
                }
                centerX /= area.getPolygonSize();
                centerY /= area.getPolygonSize();

                for(tPoint polygon : area.getPolygons())
                {
                    if ( polygon.x > centerX)   { p.x=polygon.x-reduction; }
                    else                        { p.x=polygon.x+reduction; }
                    if ( polygon.y > centerY)   { p.y=polygon.y-reduction; }
                    else                        { p.y=polygon.y+reduction; }
                    p.z =0.0;
                    tmpMarker.points.emplace_back(p);
                }
                tmpMarker.points.emplace_back(tmpMarker.points.front());
                data.markers.emplace_back(tmpMarker);
                tmpMarker.points.clear();
                tmpMarker.points.shrink_to_fit();
                tmpMarker.id++;
            }
        }
    }

    pub_cleanArea.publish(data);
}

void CRosPublishData::pubRobotPose(tPose pose)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_robotPose.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CYLINDER;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.7;
    tmpMarker.color.g = 0.7;
    tmpMarker.color.b = 0.7;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.3;
    tmpMarker.scale.y = 0.3;
    tmpMarker.scale.z = 0.3;
    tmpMarker.pose.position.z = 0.0;
    tmpMarker.pose.position.x = pose.x;
    tmpMarker.pose.position.y = pose.y;
    
    data.markers.emplace_back(tmpMarker);

    pub_robotPose.publish(data);
}

void CRosPublishData::pubLpfRobotPose(tPose pose)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_robotPose_lpf_raw.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CYLINDER;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.5;
    tmpMarker.color.g = 0.2;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.2;
    tmpMarker.scale.y = 0.2;
    tmpMarker.scale.z = 0.2;
    tmpMarker.pose.position.z = 0.0;
    tmpMarker.pose.position.x = pose.x;
    tmpMarker.pose.position.y = pose.y;
    
    data.markers.emplace_back(tmpMarker);
    pub_robotPose_lpf_raw.publish(data);
}

void CRosPublishData::pubLpfInterpolationRobotPose(tPose pose)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_robotPose_lpf_inter.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CYLINDER;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.5;
    tmpMarker.color.g = 0.5;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.2;
    tmpMarker.scale.y = 0.2;
    tmpMarker.scale.z = 0.2;
    tmpMarker.pose.position.z = 0.0;
    tmpMarker.pose.position.x = pose.x;
    tmpMarker.pose.position.y = pose.y;
    
    data.markers.emplace_back(tmpMarker);
    pub_robotPose_lpf_inter.publish(data);
}

void CRosPublishData::pubLpfExtrapolationRobotPose(tPose pose)
{
    //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
    if (pub_robotPose_lpf_extra.getNumSubscribers() == 0)
    {
        // No subscribers, so why do any work?
        return;
    }

    visualization_msgs::MarkerArray data;
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CYLINDER;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    tmpMarker.color.r = 0.5;
    tmpMarker.color.g = 1.0;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.8;
    tmpMarker.scale.x = 0.2;
    tmpMarker.scale.y = 0.2;
    tmpMarker.scale.z = 0.2;
    tmpMarker.pose.position.z = 0.0;
    tmpMarker.pose.position.x = pose.x;
    tmpMarker.pose.position.y = pose.y;
    
    data.markers.emplace_back(tmpMarker);
    pub_robotPose_lpf_extra.publish(data);
}

void CRosPublishData::pubTargetPose(tPose targetPose)
{
    CStopWatch __debug_sw;
    
    visualization_msgs::Marker tmpMarker;
    
    tmpMarker.id = 0;
    tmpMarker.header.frame_id = "/map";
    tmpMarker.type = visualization_msgs::Marker::CUBE;
    tmpMarker.action = visualization_msgs::Marker::ADD;
    
    tmpMarker.color.r = 1.0;
    tmpMarker.color.g = 0.0;
    tmpMarker.color.b = 0.0;
    tmpMarker.color.a = 0.9;
    tmpMarker.scale.x = 0.30;
    tmpMarker.scale.y = 0.30;
    tmpMarker.scale.z = 0.2;
    tmpMarker.pose.position.x = targetPose.x;
    tmpMarker.pose.position.y = targetPose.y; 
    tmpMarker.pose.position.z = 0.1;

    pub_targetPose.publish(tmpMarker);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 로봇 초기 위치 방향
 * 
 * @param x 
 * @param y 
 * @param theta 
 */
void CRosPublishData::pubInitPoseSet(tPose initPose)
{
    geometry_msgs::PoseWithCovarianceStamped pose;

    ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| Setting pose:\t" << initPose.x  << ",\t" << initPose.y << ",\t" << initPose.angle);

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = initPose.x;
    pose.pose.pose.position.y = initPose.y;

    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, initPose.angle);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    pose.pose.covariance[6 * 0 + 0] = std::pow(0.5, 2); //X standard deviation for initial pose [m]
    pose.pose.covariance[6 * 1 + 1] = std::pow(0.5, 2); //Y standard deviation for initial pose [m]
    pose.pose.covariance[6 * 5 + 5] = std::pow(M_PI / 12.0, 2); //Theta standard deviation for initial pose [rad]

    pub_initialPose.publish(pose);
}
#endif