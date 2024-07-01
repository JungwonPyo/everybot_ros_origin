#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <cmath>
#include <memory>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>

//for map topic is updated checking, only first checking
#include "control/control.h"
#include "externData/rosData/rosInterface.h"
#include "eblog.h"
#include "utils.h"
#include "MessageHandler.h"

#include "motionPlanner/motionPlanner.h"
#include "motionPlanner/wayPointManager.h"
#include "debugCtr.h"
#include "rosInterface.h"
using namespace cv; 

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief TF topic를 통해 목표의 좌표(pose) 를 얻는다
 * 
 * @param frame_base 기준이 되는 프레임
 * @param frame_target 추적을 하고 싶은 프레임
 * @param pose 확인된 좌표(pose)가 저장될 변수
 * @return tf 확인이 되면 
 * @return false 
 */
static bool getLocalization(
    tf::TransformListener *tf_listener,
    std::string frame_base,
    std::string frame_target,
    geometry_msgs::PoseStamped *pose)
{
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.orientation.w = 1.0;
    target_pose.header.frame_id = frame_target;
    try
    {
        tf_listener->waitForTransform(frame_target, frame_base, ros::Time::now(), ros::Duration(0.1));        
        tf_listener->transformPose(frame_base, target_pose, target_pose);
        // ROS_INFO("x: %.2lf\t\ty: %.2lf", target_pose.pose.position.x, target_pose.pose.position.y);  
        target_pose.header.seq += 1;
        target_pose.header.stamp = ros::Time::now();
        *pose = target_pose;
        return true;
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
        return false;
    }
}

int IsRoccupancyGridFolderExist(const char* path) { return !access(path, F_OK); }

CRosInterface::CRosInterface(ros::NodeHandle _nh)
{
    nh = _nh;

    eblog(LOG_LV_NECESSARY, "");

    isMapUpdateDone = false;
    lastTimeLidarData == -1;
    lastTimeGridmapData = -1;
    laser_scan_inf_count = 0;
    lastGridUpdate = ros::Time(0);
    //카토그래퍼로 부터 발생되는 로봇 추종 위치 시간
    TrackedPoseUpdateTime = ros::Time(0);
}

CRosInterface::~CRosInterface()
{
    pthread_join(thRosCallback, nullptr);

    eblog(LOG_LV_NECESSARY, "");
}

bool CRosInterface::occupancyGridCreateDir() {

  if (!IsRoccupancyGridFolderExist("/home/ebot/map_raw")) {
    int status =
        mkdir("/home/ebot/map_raw", 0777); // or 700 
    if (!status) {
      return true;
    } else {
      return false;
    }
  } else {
    eblog(LOG_LV_NECESSARY, "map_raw");
    return true;
  }
}

/**
 * @brief 속도 제어에 대한 ros 메세지가 날라왔을 때
 * 호출이 되는 콜백함수
 *
 * @param msg 제어 속도에 대한 값이 담겨있는 ros 메세지.
 * msg->linear.x 가 선속도. m/s
 * msg->angular.z 가 각속도.    rad/s
 */
void CRosInterface::ros_cmdvelCB_(const geometry_msgs::Twist::ConstPtr &msg)
{
    setVelocityControl(msg->linear.x, msg->angular.z);
    
    // Debug
    // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[Callback] 콜백함수 - 선속도 " << int(msg->linear.x * 1000) << " mm/s\t 각속도 " << int((msg->angular.z)*180/M_PI) << " deg/s");
}

void CRosInterface::ros_cmdtargetCB_(const geometry_msgs::Vector3::ConstPtr &msg)
{
    setTargetPose( tPose(msg->x, msg->y, msg->z) );
    
    // Debug
    double targetX = msg->x;
    double targetY = msg->y;
    double targetAngle = msg->z;
    ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[리모컨 명령 날라옴]\t타겟위치 (" << targetX << ", " << targetY << ", " << RAD2DEG(targetAngle) << " deg)");
}

/**
 * @brief 방향 제어에 대한 ros 메세지가 날라왔을 때
 * 호출이 되는 콜백함수
 *
 * @param msg 제어 방향에 대한 문자열이 담겨있는 ros 메세지.
 */
void CRosInterface::ros_cmddirCB_(const std_msgs::String::ConstPtr &msg)
{
    setKeyCmd( msg->data );
}

/**
 * 신뢰성에서 라이더 에러 감사할 경우 라이더에 수건 등을 가림.
 * 단 전체 360도 중 어디 부분까지 고려해야 할지는 더 고민을 해야 됨.
 */
int CRosInterface::getLaserInfMaxCount()
{
    return laser_scan_inf_count;
}

void CRosInterface::ros_scanCB_(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(lidar_data_mutex);
    double lidarDist[LIDAR_DIST_BUFF_SIZE] = {0,};
    int idx = 0;
    float degree = 0.0;
    int count = msg->scan_time / msg->time_increment;
    laser_scan_inf_count = 0;
    if(msg->angle_min <= msg->angle_max )
    {
        for(int i = 0; i < count; i++) 
        {
            if (i==0) continue; //hjkim221123 - 라이다 0번 버퍼에 값이 무조건 0으로 들어오는 문제로 0번 버퍼 사용하지 않도록 설정

            degree = RAD2DEG(msg->angle_min + msg->angle_increment*i); // 1, 2. 3. 4----

            idx = (int)(degree+0.5)+LIDAR_SENSOR_HEAD;            
            idx = (idx+360) % 360;
        // std::cout <<"idx: " <<idx <<" dgree :"<< degree << " i :" << i << " range :" << msg->ranges[i] <<"count :" << count << std::endl;
            
            if(idx >=0 && idx <360) 
            {
                lidarDist[idx] = msg->ranges[i];
                //헝겊등 동작 중 라이더 위에 장애물 등이 덮혀져서 라이더 데이터가 무한대가 대부분 됨.
                if (std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i]))
                {
                    laser_scan_inf_count++; // 무한대 또는 NaN 값을 가지는 데이터 포인트 수 증가
                    //eblog(LOG_LV_ERROR, "inf : " << laser_scan_inf_count );
                }
            }
        }

        setRawLidarDist(*msg);
        setLidarDist(lidarDist);
    }
    else
    {
        eblog(LOG_LV_ERROR, "ROS SCAN ANGLE ERROR !! minAngle [" << msg->angle_min  << "]" << "maxAngle [ " << msg->angle_max  << "]");
    }

    //lastTimeLidarData = get_system_time(); // Missing LiDAR 데이터 확인을 위한 마지막 콜백시간.
}

/**
 * @brief 
 * 
 * @param msg
 * 
 * This represents a 2-D grid map, in which each cell represents the probability of occupancy.
 * 
 * Header header 
 * MetaData for the map
 * MapMetaData info
 * The map data, in row-major order, starting with (0,0). 
 * Occupancy  probabilities are in the range [0,100].  Unknown is -1.
 * int8[] data
 *  
 */
void CRosInterface::ros_mapCB_(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if (msg !=nullptr && !msg->data.empty())
    {
        //ros::Time current_time  = ros::Time::now();
        //최근 맵 업데이트 시간과 비교
        if ( msg->header.stamp > lastGridUpdate )
        {
            tGridmapInfo info;
    
            info.resolution     = msg->info.resolution;
            info.width          = msg->info.width;
            info.height         = msg->info.height;
            info.origin_x       = msg->info.origin.position.x;
            info.origin_y       = msg->info.origin.position.y;
            
            //int8_t[] -> u8 of the type cast 
            const std::vector<int8_t>& gridData = msg->data;
            size_t data_size = info.width*info.height;
            s8 * data = new s8[data_size];

            for(int i=0; i<data_size; i++) {
                data[i] = static_cast<signed char>(gridData[i]);
            }

            setSlamMap(info, data);

            //업데이트 시간 업데이트
            lastGridUpdate = msg->header.stamp;
            ROBOT_CONTROL.slam.setSlamGridUpdate(true);
            //double elapse_time = (current_time - msg->header.stamp).toSec();
            //ceblog(LOG_LV_NECESSARY, BOLDRED, " 맵 업데이트 간격 : " << elapse_time  << " 초");

            delete[] data;
        }
        else
        {
            ROBOT_CONTROL.slam.setSlamGridUpdate(false);
        }

        lastTimeGridmapData = get_system_time(); // Missing Gridmap 데이터 확인을 위한 마지막 콜백시간.
    }
}

void CRosInterface::lidar_speedCB_(const std_msgs::Float64::ConstPtr &msg)
{
    double lidarInfo;
    lidarInfo       = msg->data;
    if( lidarInfo < 5.5 ||  lidarInfo > 6.5 )
    {
        ceblog(LOG_LV_NECESSARY, BOLDRED, " lidar speed : " << lidarInfo );
    }
}

void CRosInterface::rvizGoalCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    tPose rvizGoalPose;

    rvizGoalPose.x = msg->pose.position.x;
    rvizGoalPose.y = msg->pose.position.y;

    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, rvizGoalPose.angle);

    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Rviz에서 "<<BOLDWHITE"Goal Pose"<<BOLDBLACK<<"가 들어왔어요. ("
        <<BOLDGREEN<<rvizGoalPose.x<<"\t"<<rvizGoalPose.y<<"\t"<<RAD2DEG(rvizGoalPose.angle)<<BOLDBLACK<<" )");

    setRvizGoalPose(rvizGoalPose);
}

void CRosInterface::robotPoseCB(const boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void>>>& msg_ptr)
{
    const nav_msgs::Odometry_<std::allocator<void>>& msg = *msg_ptr;
    tPose robotPose;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, tf_quat);
    double yaw = tf::getYaw(tf_quat);

    robotPose.x     = msg.pose.pose.position.x;
    robotPose.y     = msg.pose.pose.position.y;
    robotPose.angle = yaw;
    setRobotPose(robotPose);
}

/**
 * @brief occupancyGrid to png files
 * 
 * @param msg 
 */
void CRosInterface::occupancyGridToFile(const nav_msgs::OccupancyGrid::ConstPtr &msg) 
{
    if (!occupancyGridCreateDir()) {
        eblog(LOG_LV, "map_raw dir create success.");
        return;
    }
    
    if (IsRoccupancyGridFolderExist("/home/ebot/map_raw")) 
    {
        occupancy_grid_idx++;
        std::string strPath = "/home/ebot/map_raw/OccupancyGrid_" + std::to_string(occupancy_grid_idx) + "." + "png";    
        Mat img_out = Mat::zeros(cv::Size(msg->info.width,msg->info.height), CV_8UC1);

        for (unsigned int y = 0; y < msg->info.height; y++)
        {
            for (unsigned int x = 0; x < msg->info.width; x++)
            {
                unsigned int i = x + (msg->info.height - y - 1) * msg->info.width;
                int intensity=205;
                
                if (msg->data[i] >= 0 && msg->data[i] <=100)
                    intensity= round((float)(100.0-msg->data[i])*2.55);
                img_out.at<unsigned char>(y, x)=intensity;
            }
        }

        imwrite(strPath.c_str(), img_out);
    }
}

/**
 * @brief trajectory_node_list 토픽 콜백함수
 * 
 * @param msg 
 */
void CRosInterface::trajectoryCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    //std::cout <<"trajectoryCb !!!!!!!!!!!!!!!!!!"  <<std::endl;
    std::list<tPoint> set;
    for ( visualization_msgs::Marker marker : msg->markers)
    {
        for ( geometry_msgs::Point point : marker.points)
        {
            tPoint path(point.x, point.y);                
            set.emplace_back(path);
        }
    }

    std::list<tPoint> optimizedPoints;
    optimizedPoints = utils::cleanmap::optimizePoints(set, 0.05);

    setTrajectory(optimizedPoints);
}

void CRosInterface::walltimerGetTfBaseLink2Laser(const ros::WallTimerEvent&)
{
    tf::StampedTransform tfBaseLink2Laser;
    int cnt = 0;
    try {
        ros::Time now = ros::Time::now();
        tfListener_.waitForTransform("base_link", "laser", now, ros::Duration(2.0)); //ros::Duration(0.01));   
        tfListener_.lookupTransform("base_link", "laser", now, tfBaseLink2Laser);
    } catch (tf::TransformException ex) {
        cnt++;
        if (cnt >= 300) {
            ROS_ERROR("Cannot get the relative pose from the base link to the laser from the tf tree."
                " Did you set the static transform publisher between base_link to laser?");
        }
    }

    tf::Quaternion quatBaseLink2Laser(tfBaseLink2Laser.getRotation().x(),
        tfBaseLink2Laser.getRotation().y(),
        tfBaseLink2Laser.getRotation().z(),
        tfBaseLink2Laser.getRotation().w());
    
    double baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw;
    tf::Matrix3x3 rotMatBaseLink2Laser(quatBaseLink2Laser);
    rotMatBaseLink2Laser.getRPY(baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw);
   
   #if 0
    baseLink2Laser_.setX(tfBaseLink2Laser.getOrigin().x());
    baseLink2Laser_.setY(tfBaseLink2Laser.getOrigin().y());
    baseLink2Laser_.setYaw(baseLink2LaserYaw);
   #endif
    //walltimerGetTfBaseLink2Laser():182: 							walltimerGetTfBaseLink2Laser x: -0.175	y: 0	yaw: -2.88
    ceblog(LOG_LV_SYSTEMINF, CYN, "\t\t\t\t\t\t\twalltimerGetTfBaseLink2Laser" << " x: " << tfBaseLink2Laser.getOrigin().x() << "\ty: " << tfBaseLink2Laser.getOrigin().y()<< "\tyaw: " << baseLink2LaserYaw);
}

/**
 * @brief 
 * 
 * @param msg 
      Enable publishing of tracked pose 
      `geometry_msgs/PoseStamped` to topic "tracked_pose".
      lidar of the duty = 6~5.8 hz (166 ms)
 */
void CRosInterface::slamTrackedPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    CStopWatch __debug_sw;
    //double x, y, yaw;
    //double roll_temp, pitch_temp, yaw_temp;

    //카토그래퍼로 부터 맵 매칭 후 좌표 데이터가 172ms+@ 마다 콜백이 발생함.
    if (msg != nullptr)
    {
        if ( msg->header.stamp > TrackedPoseUpdateTime )
        {
            //업데이트 시간 업데이트
            TrackedPoseUpdateTime = msg->header.stamp;
            //track pose가 업데이트가 발생
            ROBOT_CONTROL.slam.setTrackPoseUpdate(true);
#if 0 //데이터 필요 없다 그냥 트리거 조건만 사용하자.
            // Orientation quaternion
            tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
            // 3x3 Rotation matrix from quaternion
            tf::Matrix3x3 m(q);
            // Roll Pitch and Yaw from rotation matrix
            m.getRPY(roll_temp, pitch_temp, yaw_temp);

            x = msg->pose.position.x;
            y = msg->pose.position.y;
            yaw = yaw_temp;
#endif
            TIME_CHECK_END(__debug_sw.getTime());

            //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| tracked_pose point(x, y)\t" << x << ",\t" << y );
            //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "tracked_pose::robot pose : " <<x<<" , "<<y);
            //ROBOT_CONTROL.slam.setTrackX(x);
            //ROBOT_CONTROL.slam.setTrackY(y);
            //ROBOT_CONTROL.slam.setTrackAngke(yaw);
        }
        else
        {
            //메시지 헤더가 변경이 되지 않을 경우 뭔가 이상이 있다.
            // //track pose가 업데이트가 되지 않음..
            ROBOT_CONTROL.slam.setTrackPoseUpdate(false);
            ceblog(LOG_LV_NECESSARY, BOLDBLUE,"이상 현상 발생:tracked_pose..");
        }
    }
}

/**
 * @brief map - base_link of the tf parser and not duty cycle
 * 
 */
void CRosInterface::walltimerGetTf_(const ros::WallTimerEvent&)
{
    geometry_msgs::PoseStamped pose;
    CStopWatch __debug_sw;

    if(getLocalization(&tf_lstr_, "map", "base_link", &pose)) // update 되었을 때만 publish
    {
        //for slam tf topic is updated checking
        //ceblog(LOG_LV_NECESSARY, BOLDRED, "\t\t\t\t\t\t\t[GetTf-setSlamLocalUpdate-true]");
        ROBOT_CONTROL.slam.setSlamLocalUpdate(true);

        double x, y, yaw;
        double roll_temp, pitch_temp, yaw_temp;
    
        tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll_temp, pitch_temp, yaw_temp);


        x   = pose.pose.position.x;
        y   = pose.pose.position.y;
        yaw = yaw_temp;
        
        TIME_CHECK_END(__debug_sw.getTime());

        //ceblog(LOG_LV_NECESSARY, CYN, "\t" << "| TF point(x, y)\t" << x << ",\t" << y );
        //ceblog(LOG_LV_NECESSARY, CYN, "\t" << "| TF point(angle)\t" << yaw);
        //if ( MOTION.isCheckTargetPoseReached())
        //ceblog(LOG_LV_NECESSARY, CYN, "\t\t\t\t\t\t\t[GetTf]" << " x: " << x<< "\ty: " << y);
        setSlamPose(x, y, yaw);
        setUpdateSlamPose(true);

        if(x < 0.001){x=0.0;}

        #if USE_CEVA_LOG_SAVE
            CEVA_LOG(E_CEVA_TYPE::SLAM_POSE, x, y, yaw);
        #endif
    }
    else
    {
        //for slam tf topic is updated checking
        //if ( MOTION.isCheckTargetPoseReached())
        //    ceblog(LOG_LV_NECESSARY, BOLDRED, "\t\t\t\t\t\t\t[GetTf-setSlamLocalUpdate-false]");
        ROBOT_CONTROL.slam.setSlamLocalUpdate(false);
    }
}

/**
 * @brief Ros 데이터가 제대로 수신되고 있는지
 * 시간을 통해 체크하는 함수.
 * 일정시간 데이터가 들어오지 않으면 에러 메세지 발생.
 * 
 * LiDAR Data   -> 10 초
 * Gridmap Data -> 20 초
 */
void CRosInterface::missingDataTimer(const ros::WallTimerEvent&)
{
#if 1
    //데이터 지연으로는 체크할 경우 사이드가 너무 많음
    //헝겊등으로 라이더에 올려 놓았을때 에러 유무를 확인이 불가능하다
    //현재 라이더 데이터가 커널에서 관리를 하고 있어 에러 발생 부분에 대해서 커널에서 하는 것이 맞음
    //단순히 헝겊으로 덮을 경우 라이더 데이터가 무한대로 300개 이상 발생하는 것으로 아주 간단히 체크함.
    //덮지 않을 경우 장착된 기구물에 의해서 70~80개 정도 무한대 데이터가 발생을 함.
    int laser_max_inf = getLaserInfMaxCount();
    int refer_max_infi = CONFIG.lidar_max_infi; //300
    if (laser_max_inf > refer_max_infi) 
    {
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::MISSING_LIDAR_DATA)));
        ceblog(LOG_LV_ERROR, YELLOW, "Error Detected\t Missing Lidar Data::Inf count = \t  " <<laser_max_inf);
    }
#else
    const double kLidarLimitTime = 5.0; // 5초
    const double kGridmapLimitTime = 20.0; // 10초
    
    if ( (get_system_time(lastTimeLidarData) > kLidarLimitTime) && (lastTimeLidarData != -1) )
    {
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::MISSING_LIDAR_DATA)));
        //ceblog(LOG_LV_ERROR, YELLOW, "Error Detected\t Missing Lidar Data\t "<< get_system_time(lastTimeLidarData) << "sec > " <<kLidarLimitTime);
    }

    //is slam of the running state?
    if ( ROBOT_CONTROL.slam.isSlamRunning()) 
    {
        lastTimeGridmapData = get_system_time();
    } 
    else
    {
        if ( (get_system_time(lastTimeGridmapData) > kGridmapLimitTime) && (lastTimeGridmapData != -1) )
        {
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::MISSING_GRIDMAP_DATA)));
            //ceblog(LOG_LV_ERROR, YELLOW, "Error Detected\t Missing Gridmap Data\t "<< get_system_time(lastTimeGridmapData) << "sec > " <<kGridmapLimitTime);
        }
    }
#endif
}

/**
 * @brief IMU Filter 좌표 Topic 콜백함수.
 *        use of the kidnap trigger 
 * @param msg 
 */
void CRosInterface::imuDataCb(const sensor_msgs::Imu::ConstPtr &imusmsg)
{
    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-angular velocity\t" << imusmsg->angular_velocity.x << ",\t" << imusmsg->angular_velocity.y << ",\t" << imusmsg->angular_velocity.z);
    //ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| IMU-linear acceleration\t" << imusmsg->linear_acceleration.x  << ",\t" << imusmsg->linear_acceleration.y << ",\t" << imusmsg->linear_acceleration.z);
    //set of the data
    setFilterImu(imusmsg->angular_velocity.x, imusmsg->angular_velocity.y, imusmsg->angular_velocity.z, imusmsg->linear_acceleration.x, imusmsg->linear_acceleration.y, imusmsg->linear_acceleration.z);
}

/**
 * @brief from rviz of 2D Nav Goal tool ()
 * 
 * @param goalPoint 
 */
void CRosInterface::goalPointCb(const geometry_msgs::PoseStamped::ConstPtr &goalPoint)
{
    if (goalPoint != nullptr)
    {
        /*
        /move_base_simple/goal
        frame_id: "map"
        pose: 
        position: 
            x: 0.780798733234
            y: 0.118190273643
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.0910552431827
            w: 0.995845842834
        */
        double tx = goalPoint->pose.position.x;
        double ty = goalPoint->pose.position.y;
        double tz = goalPoint->pose.position.z;

        // Orientation quaternion
        tf::Quaternion q(
                    goalPoint->pose.orientation.x,
                    goalPoint->pose.orientation.y,
                    goalPoint->pose.orientation.z,
                    goalPoint->pose.orientation.w);

        // 3x3 Rotation matrix from quaternion
        tf::Matrix3x3 m(q);

        // Roll Pitch and Yaw from rotation matrix
        double roll_temp, pitch_temp, yaw_temp;
        m.getRPY(roll_temp, pitch_temp, yaw_temp);
        
        double yaw = yaw_temp;
        ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| way point(x, y)\t" << tx << ",\t" << ty );
        ceblog(LOG_LV_NECESSARY, BOLDWHITE, "\t" << "| way point(angle)\t" << yaw);

        //move to the robot
        //TODO : move to the robot 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CRosInterface::threadRosCallback()
{
#if 0
    while (/*true*/ ros::ok())
    {
        // 위치이동 Monitor -> rosInterface Timer - jspark 23.06.21
        // spinOnce()시에 10ms delay 발생함.
        ros::spinOnce();
        DEBUG_CTR.isAliveRosCallbackLoop.set(true);
    }
#else
    ros::AsyncSpinner spinner(2);  // Use 2 threads (cpu 코어 수의 절반이 적당)
    spinner.start();
    ros::waitForShutdown();  // Block until shutdown signal (e.g., Ctrl+C)
#endif
}

void CRosInterface::reconfigureCB(ts800_ros::RobotParameterConfig &config, uint32_t level)
{
    tParameter newP = ROS_CONFIG;

    eblog(LOG_LV_NECESSARY, "[ros parameter] motion controller psi :    " << config.motion_control_psi_gain);
    eblog(LOG_LV_NECESSARY, "[ros parameter] motion controller cross 1 :    " << config.motion_control_cross_gain1);
    eblog(LOG_LV_NECESSARY, "[ros parameter] motion controller cross 2 :    " << config.motion_control_cross_gain2);
    eblog(LOG_LV_NECESSARY, "[ros parameter] motion p :    " << config.motion_control_p);
    eblog(LOG_LV_NECESSARY, "[ros parameter] motion i :    " << config.motion_control_i);
    eblog(LOG_LV_NECESSARY, "[ros parameter] motion d :    " << config.motion_control_d);
    eblog(LOG_LV_NECESSARY, "[ros parameter] angle p :     " << config.angle_control_p);
    eblog(LOG_LV_NECESSARY, "[ros parameter] angle i :     " << config.angle_control_i);
    eblog(LOG_LV_NECESSARY, "[ros parameter] angle d :     " << config.angle_control_d);
    eblog(LOG_LV_NECESSARY, "[ros parameter] speed_fwd :   " << config.speed_fwd);
    eblog(LOG_LV_NECESSARY, "[ros parameter] speed_dummy : " << config.speed_dummy);
    eblog(LOG_LV_NECESSARY, "[ros parameter] simplify gridmap - opening : " << config.simplify_gridmap_opening_kernal_size);
    eblog(LOG_LV_NECESSARY, "[ros parameter] simplify gridmap - wall thickness : " << config.simplify_gridmap_wall_thickness);

    newP.angleControl.kP = config.angle_control_p;
    newP.angleControl.kI = config.angle_control_i;
    newP.angleControl.kD = config.angle_control_d;
    newP.lookahead = config.lookahead;
    newP.max_steering_angle = config.max_steering_angle;

    newP.stanleyParam.psi = config.motion_control_psi_gain;
    newP.stanleyParam.cross1 = config.motion_control_cross_gain1;
    newP.stanleyParam.cross2 = config.motion_control_cross_gain2;

    newP.motionControl.kP = config.motion_control_p;
    newP.motionControl.kI = config.motion_control_i;
    newP.motionControl.kD = config.motion_control_d;

    newP.speed_fwd = config.speed_fwd;
    newP.speed_dummy = config.speed_dummy;
    newP.speed_t2 = config.speed_t2;
    newP.speed_t3 = config.speed_t3;

    newP.simplifyGridmap.openingKernalSize = config.simplify_gridmap_opening_kernal_size;
    newP.simplifyGridmap.wallThickness = config.simplify_gridmap_wall_thickness;

    newP.wallTrack_P_gain = config.wallTrack_P_gain;
    newP.wallTrack_I_gain = config.wallTrack_I_gain;

    newP.cleanLineInterval = config.clean_line_interval;

    newP.wallFollowLpfAlpha = config.wall_follow_lpf_alpha;

    SET_ROS_CONFIG(newP);
}

void CRosInterface::parameterInit()
{
    tParameter newP = ROS_CONFIG;

    newP.angleControl.kP = 1.0;
    newP.angleControl.kI = 0.0;
    newP.angleControl.kD = 1200.0;

    newP.motionControl.kP = 40.0;
    newP.motionControl.kI = 0.0;
    newP.motionControl.kD = 1200.0;

    newP.simplifyGridmap.openingKernalSize = 3; // dstar 벽 넓히기 위해 벽확장.
    newP.simplifyGridmap.wallThickness = 2;

    newP.cleanLineInterval = 0.3;

    newP.wallFollowLpfAlpha = 0.2;

    SET_ROS_CONFIG(newP);
}

void CRosInterface::rosParameterServerInit()
{
    ros::NodeHandle private_nh("~"); // parameter 노드핸들러.

    dsrv_ = new dynamic_reconfigure::Server<ts800_ros::RobotParameterConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<ts800_ros::RobotParameterConfig>::CallbackType cb;
    cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
    dsrv_->setCallback(cb);

    parameterInit();
}

void CRosInterface::init()
{
    rosParameterServerInit();

    cmd_vel_sub_    = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&CRosInterface::ros_cmdvelCB_, this, _1));
    cmd_target_sub_ = nh.subscribe<geometry_msgs::Vector3>("/cmd_target", 1, boost::bind(&CRosInterface::ros_cmdtargetCB_, this, _1));
    cmd_dir_sub_    = nh.subscribe<std_msgs::String>("/cmd_dir", 1, boost::bind(&CRosInterface::ros_cmddirCB_, this, _1));
    scan_sub_       = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&CRosInterface::ros_scanCB_, this, _1));
    map_sub_        = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(&CRosInterface::ros_mapCB_, this, _1));
    trajectoryNodeSub   = nh.subscribe<visualization_msgs::MarkerArray>("/trajectory_node_list", 1, boost::bind(&CRosInterface::trajectoryCb, this, _1));
    #if 0 //not_used_external 3i lidar(after using)
    lidar_speed_sub_ = nh.subscribe<std_msgs::Float64>("/3i/lidarSpeed", 1, boost::bind(&CRosInterface::lidar_speedCB_ , this, _1));
    #endif
    rvizGoalSub     = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&CRosInterface::rvizGoalCb , this, _1));
    robotPoseSub    = nh.subscribe<nav_msgs::Odometry>("/ts800_odom", 1, boost::bind(&CRosInterface::robotPoseCB , this, _1));
    
    //실제 카토그래퍼에서 좌표 나오는 트리거로 확인(200ms 마다 콜백이 발생함)
    slam_track_pose_sub_    = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 5, boost::bind(&CRosInterface::slamTrackedPose, this, _1));  //from of the cartograper pose  
    wall_timers_.push_back(nh.createWallTimer(ros::WallDuration(0.01), &CRosInterface::walltimerGetTf_, this)); // pose_publish_period_sec = 0.01 : 10ms
    wall_timers_.push_back(nh.createWallTimer(ros::WallDuration(1.0),&CRosInterface::missingDataTimer, this));
#if 0 //after of the using
    wall_timers_.push_back(nh.createWallTimer(ros::WallDuration(0.01),&CRosInterface::walltimerGetTfBaseLink2Laser, this));
    imu_filter_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, boost::bind(&CRosInterface::imuDataCb , this, _1));
#endif 
    //rviz of 2D Nav Goal
    goal_point_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, boost::bind(&CRosInterface::goalPointCb , this, _1));
 
    pthread_create(&thRosCallback, nullptr, &CRosInterface::threadRosCallbackWrap, this);

    std::cout << " init() : ROS INIT    " << std::endl;
}

bool CRosInterface::isRosOk()
{
    return ros::ok();
}

