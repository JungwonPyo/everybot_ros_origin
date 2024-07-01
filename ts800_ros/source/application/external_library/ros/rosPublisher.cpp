#include "rosPublisher.h"
#include "eblog.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 0.1 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

using namespace std;

CRosPublisher::CRosPublisher() {}
CRosPublisher::CRosPublisher(const CRosPublisher &ref) {}
CRosPublisher &CRosPublisher::operator=(const CRosPublisher &ref)
{
    return *this;
}
CRosPublisher::~CRosPublisher() {}

void CRosPublisher::initPublisher()
{
    pubLidar    = pNh->advertise<sensor_msgs::LaserScan>("scan", 10); //scan & odom time lantecny
    pubOdometry = pNh->advertise<nav_msgs::Odometry>("odom", 10); //scan & odom time lantecny
    pubImu = pNh->advertise<sensor_msgs::Imu>("/imu/data_raw", 10); //scan & odom time lantecny
#if defined (NEW_SLAM_CONTROL) && (NEW_SLAM_CONTROL == 0)
    pubSubmapFilter = pNh->advertise<std_msgs::Int32>("/subMapFilter", 1);
#else
    //카토그래퍼에서 서브 맵 업데이트 주기를 스위칭 할 수 있는 기능(기능만 만들어 놓고 비활성화 함)
    pubSlamControl = pNh->advertise<ts800_ros::SlamControl>("/ts800/slamControl", 1);
#endif
    //request of the ceva
    pubRBTPlusFeedBack = pNh->advertise<ts800_ros::Feedback>("/ts800/feedback", 10);
    pubRBTPlusRotationVector = pNh->advertise<ts800_ros::RotationVector>("/ts800/rotationVector", 10);
}

void CRosPublisher::initMessage()
{
    odom.isUpdate = false;
    odom.topic.header.seq       = 0;
    odom.topic.header.stamp     = ros::Time::now();
    odom.topic.header.frame_id  = "odom";
    odom.topic.child_frame_id   = "base_link";

    odom.tf.header.seq          = 0;
    odom.tf.header.stamp        = ros::Time::now();
    odom.tf.header.frame_id     = "odom";
    odom.tf.child_frame_id      = "base_link";

    lidar.isUpdate = false;
    lidar.topic.header.seq      = 0;
    lidar.topic.header.stamp    = ros::Time::now();

    lidar.staticTf.header.seq               = 0;
    lidar.staticTf.header.stamp             = ros::Time::now();
    lidar.staticTf.header.frame_id          = "base_link";
    lidar.staticTf.child_frame_id           = "laser";
    lidar.staticTf.transform.translation.x  = -0.175;
    lidar.staticTf.transform.translation.y  = 0.0;
    lidar.staticTf.transform.translation.z  = 0.0;
    lidar.staticTf.transform.rotation       = tf::createQuaternionMsgFromYaw(-2.88);
}

CRosPublisher& CRosPublisher::getInstance()
{
    static CRosPublisher s;
    return s;
}

void CRosPublisher::publishOdometryTopicWithTf(const ::ros::WallTimerEvent& event)
{
    if(odom.isUpdate)
    {
        std::lock_guard<std::mutex> guard(odom.mutex);

        pubOdometry.publish(odom.topic);
        tfbc.sendTransform(odom.tf);
        odom.isUpdate = false;
    }
}

void CRosPublisher::publishLidarTopic(const ::ros::WallTimerEvent& event)
{
    if(lidar.isUpdate)
    {
        std::lock_guard<std::mutex> guard(lidar.mutex);

        pubLidar.publish(lidar.topic);
        lidar.isUpdate = false;
    }
}

void CRosPublisher::publishLidarTf(const ::ros::WallTimerEvent& event)
{
    lidar.staticTf.header.seq++;
    lidar.staticTf.header.stamp = ros::Time::now();
    /*
        2-1샘플(-2.97 rad = -170.0 deg)
        3-1샘플(-2.88 rad == -165 deg)
    */
    double angleOffset = DEG2RAD(lidarOffset);
    //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "TF : lidarOffset(radian): "<< angleOffset);
    double angle = -2.88 + angleOffset;
    lidar.staticTf.transform.rotation  = tf::createQuaternionMsgFromYaw(angle);
  
    tfbc.sendTransform(lidar.staticTf);
}

void CRosPublisher::init(ros::NodeHandle *pNh)
{
    int32_t ms = 1000000; // nsec -> ms 변환

    this->pNh = pNh;
    initPublisher();
    initMessage();

#if 0 //delet of the time lantency
    wallTimers.push_back(pNh->createWallTimer(ros::WallDuration(0, 10*ms), &CRosPublisher::publishOdometryTopicWithTf, this)); // 100 Hz
    wallTimers.push_back(pNh->createWallTimer(ros::WallDuration(0, 20*ms), &CRosPublisher::publishLidarTopic, this)); // 50hz
#endif
    wallTimers.push_back(pNh->createWallTimer(ros::WallDuration(0, 25*ms), &CRosPublisher::publishLidarTf, this)); // 40Hz //scan & odom time lantecny
}

void CRosPublisher::updateOdometryData(tSysPose newData)
{
    if(odom.oldData.timeStamp == newData.timeStamp)  return; //set 100% input odom percentage
    // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "RosTime["<<BOLDYELLOW<<newData.timeStamp<<BOLDBLACK<<"]\t"<<BOLDGREEN<<setw(6)<<newData.x<<","<<setw(6)<<newData.y<<","<<setw(6)<<newData.angle);

    std::lock_guard<std::mutex> guard(odom.mutex);

    double dt = newData.timeStamp.toSec() - odom.oldData.timeStamp.toSec();
    double vX = 0.0, vY = 0.0, vAngle = 0.0;
    if ( dt != 0.0)
    {
        vX = (newData.x-odom.oldData.x)/dt;
        vY = (newData.y-odom.oldData.y)/dt;
        vAngle = (newData.angle-odom.oldData.angle)/dt;  // TODO: angle이 절대각도 일수있음. 확인필요.
    }
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(newData.angle);

    /* Odometry Topic 업데이트 */
    odom.topic.header.seq++;
    odom.topic.header.stamp             = newData.timeStamp;
    odom.topic.pose.pose.position.x     = newData.x;
    odom.topic.pose.pose.position.y     = newData.y;
    odom.topic.pose.pose.position.z     = 0.0;
    odom.topic.pose.pose.orientation    = quat;
    odom.topic.twist.twist.linear.x     = vX;
    odom.topic.twist.twist.linear.y     = vY;
    odom.topic.twist.twist.angular.z    = vAngle;
    pubOdometry.publish(odom.topic); //scan_time_shift & lider & odom time lantency

    /* Odometry TF 업데이트 */
    odom.tf.header.seq              = odom.topic.header.seq;
    odom.tf.header.stamp            = newData.timeStamp;
    odom.tf.transform.translation.x = newData.x;
    odom.tf.transform.translation.y = newData.y;
    odom.tf.transform.translation.z = 0.0;
    odom.tf.transform.rotation      = quat;
    tfbc.sendTransform(odom.tf); //scan_time_shift & lider & odom time lantency

    odom.oldData = newData;
    odom.isUpdate = true;
}

void CRosPublisher::updateImuData(tSysIMU systemImu)
{
    std::lock_guard<std::mutex> guard(imuMutex);
    //IMU_INIT		= 0x00,
    //IMU_BUSY		= 0x01,
    //IMU_READY		= 0x02,
    int imu_state = static_cast<int>(systemImu.state);
    if (imu_state == 2)
    {
        if(oldSystemImu.timeStamp == systemImu.timeStamp)  return; //set input imu percentage 100%
    
        sensor_msgs::Imu imu;
        imu.header.seq++;
        imu.header.stamp = systemImu.timeStamp; //ros::Time::now();
        imu.header.frame_id = "imu_link"; //"imu_link";
        //systemImu:Accelerations = mm/s^2
        //systemImu:Gyro = radin/s x 1000
        //ros msg = Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        double accelerations_x = static_cast<double>(systemImu.Ax);
        double accelerations_y = static_cast<double>(systemImu.Ay);
        double accelerations_z = static_cast<double>(systemImu.Az);
    #if 0 //CHECK(sensor_to_tracking->translation().norm() < 1e-5) -> error
        imu.linear_acceleration.x = accelerations_x/1000.0; 
        imu.linear_acceleration.y = accelerations_y/1000.0;
        imu.linear_acceleration.z = 9.8;//accelerations_z/1000.0;
    #else 
        imu.linear_acceleration.x = 0.0; 
        imu.linear_acceleration.y = 0.0;
        imu.linear_acceleration.z = 9.8;
    #endif
        double pitch = static_cast<double>(systemImu.Gpitch);
        double roll = static_cast<double>(systemImu.Groll);
        double yaw = static_cast<double>(systemImu.Gyaw);
        imu.angular_velocity.x = pitch/1000.0;// roll axis
        imu.angular_velocity.y = roll/1000.0; // pitch axis
        imu.angular_velocity.z = yaw/1000.0; // yaw axis
    
        pubImu.publish(imu);
        oldSystemImu = systemImu;
    }
}

void CRosPublisher::updateLidarData(const sensor_msgs::LaserScan& newScan)
{
    //if(lidar.topic.header.stamp==newScan.header.stamp)  return; //scan & odom time lantecny
 
    std::lock_guard<std::mutex> guard(lidar.mutex);
#if 0 //re copy ..
    lidar.topic.header.seq      = newScan.header.seq;
    lidar.topic.header.frame_id = newScan.header.frame_id;
    lidar.topic.header.stamp    = newScan.header.stamp;

    lidar.topic.angle_increment = newScan.angle_increment;
    lidar.topic.angle_max       = newScan.angle_max;
    lidar.topic.angle_min       = newScan.angle_min;
    lidar.topic.range_max       = newScan.range_max;
    lidar.topic.range_min       = newScan.range_min;
    lidar.topic.scan_time       = newScan.scan_time;
    lidar.topic.time_increment  = newScan.time_increment;

    // intensities 깊은 복사
    lidar.topic.intensities.resize(newScan.intensities.size());
    std::copy(newScan.intensities.begin(), newScan.intensities.end(), lidar.topic.intensities.begin());
    
    // ranges 깊은 복사
    lidar.topic.ranges.resize(newScan.ranges.size());
    std::copy(newScan.ranges.begin(), newScan.ranges.end(), lidar.topic.ranges.begin());
    
    pubLidar.publish(lidar.topic);
#else
    pubLidar.publish(newScan);
#endif
    lidar.isUpdate = true;
    // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "LiDAR Data 업데이트 -> "<<BOLDGREEN<<"ros publisher");
}

void CRosPublisher::pubSlamControlFunc(int max_time, int control_type)
{
#if defined (NEW_SLAM_CONTROL) && (NEW_SLAM_CONTROL == 1)
    std::lock_guard<std::mutex> guard(slamControlMutex);

    ts800_ros::SlamControl control_msg;
    //슬램 서브맵 추가 주기
    control_msg.control_max_time = max_time;  
    //슬램 제어(기존 처럼 하면 됨)
    control_msg.control_type = control_type;
    //ceblog(LOG_LV_NECESSARY, RED, "\t" << "| max_time:\t" << max_time  << ",\t" << control_type);
    pubSlamControl.publish(control_msg);
#endif
}

/**
 * @brief 
 * 
 * @param value 
 */
void CRosPublisher::updateSubmapFilter(int value)
{   
    std::lock_guard<std::mutex> guard(submapFilterMutex);

    std_msgs::Int32 subMapFilter_msg;

    subMapFilter_msg.data = value;

    //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "\t" << "| updateSubmapFilter: \t" << value );

    pubSubmapFilter.publish(subMapFilter_msg);
}

void CRosPublisher::updateLidarOffset(double value)
{
    lidarOffset = value;
}
/**
 * @brief 
 * 
 * @param heading 
 * @param mcuTime
    Feedback.msg
    std_msgs/Header header  # ROS time of SLAM measurement being provided to RBT+
    uint32 timestamp        # MCU time assigned to feedback 
    float32 heading         # Feedback to RBT+
 */
void CRosPublisher::pubRbtPlusFeedBack(double heading, unsigned int mcuTime)
{   
    std::lock_guard<std::mutex> guard(feedbackMutex);

    ts800_ros::Feedback feedback_msg;
    feedback_msg.header.stamp = ros::Time::now();  
    feedback_msg.header.frame_id = "feedback";
    feedback_msg.timestamp = mcuTime; //ros::Time::now().toSec(); //MCU time assigned to feedback
    feedback_msg.heading = heading;

    //ceblog(LOG_LV_NECESSARY, RED, "\t" << "| Feedback:\t" << mcuTime  << ",\t" << heading);

    pubRBTPlusFeedBack.publish(feedback_msg);
}

/**
 * @brief 
 * 
 * @param newData 
 * @param mcuTime
    RotationVector.msg
    std_msgs/Header header   # ROS time assigned to RV when used e.g. in odometry 
    geometry_msgs/Quaternion quaternion 
    uint32 timestamp         # MCU timestamp
 */
void CRosPublisher::pubRbtPlusRotationVector(tSysPose newData, unsigned int mcuTime)
{
    std::lock_guard<std::mutex> guard(rotationVectorMutex);

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(newData.angle));

    ts800_ros::RotationVector rotation_vector_msg;
    rotation_vector_msg.header.stamp = ros::Time::now();
    rotation_vector_msg.header.frame_id = "rotationVector";
    rotation_vector_msg.quaternion = quat;
    rotation_vector_msg.timestamp = mcuTime; //ros::Time::now().toSec(); //MCU time assigned to feedback

    pubRBTPlusRotationVector.publish(rotation_vector_msg);
}

namespace debug
{
    CRosPublisher::CRosPublisher() : pNh(nullptr)
    {
        // Initialize other members if needed
    }

    CRosPublisher::CRosPublisher(const CRosPublisher& ref) {}

    CRosPublisher& CRosPublisher::operator=(const CRosPublisher& ref) {}

    CRosPublisher::~CRosPublisher()
    {
        // Clean up resources if needed
    }

    void CRosPublisher::initPublisher()
    {
        pubCvImage          = pNh->advertise<sensor_msgs::Image>("simplify_gridImage", 1);
        
        pubRawFrontiers     = pNh->advertise<visualization_msgs::MarkerArray>("/explorer/frontiers/raw", 5);
        pubTunedFrontiers   = pNh->advertise<visualization_msgs::Marker>("/explorer/frontiers/tuned", 5);

        pubWfTargetPoint    = pNh->advertise<visualization_msgs::MarkerArray>("/wallfollowing/target_pose", 5);
        pubWfStartPoint     = pNh->advertise<visualization_msgs::MarkerArray>("/wallfollowing/start_Pose", 5);
        pubWfPoints         = pNh->advertise<visualization_msgs::MarkerArray>("/wallfollowing/points", 5);

        pubGlobalPathPlan   = pNh->advertise<nav_msgs::Path>("/dstar/path/global", 5);
        pubCurrentPathPlan  = pNh->advertise<nav_msgs::Path>("/dstar/path/current", 5);
        pubDstarMap         = pNh->advertise<nav_msgs::OccupancyGrid>("/dstar/map", 1);

        pubEscapeWallCells  = pNh->advertise<visualization_msgs::MarkerArray>("/escapeWall/cells", 5);

        pubCleanArea        = pNh->advertise<visualization_msgs::MarkerArray>("/clean/areas", 5);

        pubRobotPath        = pNh->advertise<nav_msgs::Path>("/robot/path", 5);

        pubCostMap          = pNh->advertise<nav_msgs::OccupancyGrid>("/local/costmap",5);

        pubContour          = pNh->advertise<visualization_msgs::MarkerArray>("/contour", 5);
        pubContour2         = pNh->advertise<visualization_msgs::MarkerArray>("/contour2", 5);
        pubShrinkContour    = pNh->advertise<visualization_msgs::MarkerArray>("/shrink/contour", 5);
        pubContourList      = pNh->advertise<visualization_msgs::MarkerArray>("/contourList", 5);

        pubCleanLine1       = pNh->advertise<visualization_msgs::Marker>("/clean/line/1", 5);
        pubCleanLine2       = pNh->advertise<visualization_msgs::Marker>("/clean/line/2", 5);

        pubDoor             = pNh->advertise<visualization_msgs::MarkerArray>("/door", 5);

        pubCleaningArea     = pNh->advertise<visualization_msgs::Marker>("/cleaning/area", 5);
        pubCleaningLine     = pNh->advertise<visualization_msgs::Marker>("/cleaning/line", 5);
        pubCleanMap         = pNh->advertise<nav_msgs::OccupancyGrid>("/cleanmap",5);

        pubForbiddenArea    = pNh->advertise<visualization_msgs::MarkerArray>("/forbidRect",1);
        pubForbiddenLine    = pNh->advertise<visualization_msgs::Marker>("/forbidLine",1);
    }

    void CRosPublisher::setCleanAreaMarker()
    {
        cleanAreaMarker.header.frame_id = "map";
        cleanAreaMarker.action = visualization_msgs::Marker::ADD;
        cleanAreaMarker.lifetime = ros::Duration(0);
        cleanAreaMarker.ns = "robot_position";
        cleanAreaMarker.id = 0;
        
        cleanAreaMarker.type = visualization_msgs::Marker::LINE_STRIP;  // 마커 모양 바꿀 수 있음
        cleanAreaMarker.scale.x = 0.28;
        cleanAreaMarker.scale.y = 0.28;
        cleanAreaMarker.scale.z = 0.28;
        cleanAreaMarker.color.a = 0.1;      //투명도
        cleanAreaMarker.color.r = 0.01;
        cleanAreaMarker.color.g = 0.31;
        cleanAreaMarker.color.b = 0.97;

        cleanAreaMarker.pose.orientation.x = 0;
        cleanAreaMarker.pose.orientation.y = 0;
        cleanAreaMarker.pose.orientation.z = 0;
        cleanAreaMarker.pose.orientation.w = 1;
    }

    void CRosPublisher::setCleanLineMarker()
    {
        cleanLineMarker.header.frame_id = "map";
        cleanLineMarker.action = visualization_msgs::Marker::ADD;
        cleanLineMarker.lifetime = ros::Duration(0);
        cleanLineMarker.ns = "robot_position";
        cleanLineMarker.id = 0;
        
        cleanLineMarker.type = visualization_msgs::Marker::LINE_STRIP;  // 마커 모양 바꿀 수 있음
        cleanLineMarker.scale.x = 0.03;
        cleanLineMarker.scale.y = 0.03;
        cleanLineMarker.scale.z = 0.03;
        cleanLineMarker.color.a = 1.0;      //투명도
        cleanLineMarker.color.r = 1.0;
        cleanLineMarker.color.g = 1.0;
        cleanLineMarker.color.b = 1.0;

        cleanLineMarker.pose.orientation.x = 0;
        cleanLineMarker.pose.orientation.y = 0;
        cleanLineMarker.pose.orientation.z = 0;
        cleanLineMarker.pose.orientation.w = 1;
    }

    CRosPublisher& CRosPublisher::getInstance()
    {
        static CRosPublisher s;
        return s;
    }

    void CRosPublisher::init(ros::NodeHandle* pNh)
    {
        this->pNh = pNh;

        lastTimeCvImage = ros::Time::now();
        initPublisher();
        setCleanAreaMarker();
        setCleanLineMarker();
    }

    void CRosPublisher::publishSimplifyMapImage(cv::Mat image, int throttleTime)
    {
        //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
        if (pubCvImage.getNumSubscribers() == 0)    return;
        if (image.empty())  return;
        if ( ros::Time::now().sec-lastTimeCvImage.sec < throttleTime )  return;

        cv_bridge::CvImage cv_image_msg;
        cv_image_msg.encoding = "mono8";
        cv_image_msg.image = image;

        cv_image_msg.header.stamp = ros::Time::now();
        cv_image_msg.header.frame_id = "image_frame";

        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCvImage.publish(cv_image_msg.toImageMsg());
        lastTimeCvImage = ros::Time::now();
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishRawFrontiers(std::list<tPoint> points)
    {
        //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
        if (pubRawFrontiers.getNumSubscribers() == 0)  return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;
        
        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.type = visualization_msgs::Marker::SPHERE;
        tmpMarker.action = visualization_msgs::Marker::ADD;
        tmpMarker.color.r = 0.0;
        tmpMarker.color.g = 1.0;
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

        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubRawFrontiers.publish(data);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishTunedFrontiers(std::list<tPoint> points)
    {
        //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
        if (pubTunedFrontiers.getNumSubscribers() == 0)  return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;
        
        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.type = visualization_msgs::Marker::SPHERE;
        tmpMarker.action = visualization_msgs::Marker::ADD;
        tmpMarker.color.r = 1.0;
        tmpMarker.color.g = 0.0;
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

        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubTunedFrontiers.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishWfTargetPoint(tPoint point)
    {
        if (pubWfTargetPoint.getNumSubscribers()==0)    return;

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
        tmpMarker.scale.x = 0.3;
        tmpMarker.scale.y = 0.3;
        tmpMarker.scale.z = 0.3;
        tmpMarker.pose.position.z = 0.0;
        tmpMarker.pose.position.x = point.x;
        tmpMarker.pose.position.y = point.y;
        
        data.markers.emplace_back(tmpMarker);
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubWfTargetPoint.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishWfStartPoint(tPoint point)
    {
        if (pubWfStartPoint.getNumSubscribers()==0) return;

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
        tmpMarker.scale.x = 0.3;
        tmpMarker.scale.y = 0.3;
        tmpMarker.scale.z = 0.3;
        tmpMarker.pose.position.z = 0.0;
        tmpMarker.pose.position.x = point.x;
        tmpMarker.pose.position.y = point.y;
        
        data.markers.emplace_back(tmpMarker);
            ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubWfStartPoint.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishWfPoints(std::list<tPoint> points)
    {
        if (pubWfPoints.getNumSubscribers()==0) return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;
        
        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.type = visualization_msgs::Marker::SPHERE;
        tmpMarker.action = visualization_msgs::Marker::ADD;
        tmpMarker.color.r = 0.0;
        tmpMarker.color.g = 0.0;
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
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubWfPoints.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishGlobalPathPlan(std::list<tPoint> path)
    {
        if( pubGlobalPathPlan.getNumSubscribers() == 0 )    return;

        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.header.seq = 0;
        poseStamped.header.stamp = ros::Time::now();

        for ( tPoint point : path )
        {
            poseStamped.pose.position.x = point.x;
            poseStamped.pose.position.y = point.y;
            poseStamped.pose.position.z = 0;
            poseStamped.pose.orientation.x = 0;
            poseStamped.pose.orientation.y = 0;
            poseStamped.pose.orientation.z = 0;
            poseStamped.pose.orientation.w = 1;

            msg.poses.push_back(poseStamped);
            poseStamped.header.seq++;
        }
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubGlobalPathPlan.publish(msg);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishGlobalPathPlan(std::list<tPoint> path, const tPose &robotPose)
    {
        if( pubGlobalPathPlan.getNumSubscribers() == 0 )    return;

        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.header.seq = 0;
        poseStamped.header.stamp = ros::Time::now();

        poseStamped.pose.position.x = robotPose.x;
        poseStamped.pose.position.y = robotPose.y;
        poseStamped.pose.position.z = 0;
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 1;

        msg.poses.push_back(poseStamped);
        poseStamped.header.seq++;

        for ( tPoint point : path )
        {
            poseStamped.pose.position.x = point.x;
            poseStamped.pose.position.y = point.y;
            poseStamped.pose.position.z = 0;
            poseStamped.pose.orientation.x = 0;
            poseStamped.pose.orientation.y = 0;
            poseStamped.pose.orientation.z = 0;
            poseStamped.pose.orientation.w = 1;

            msg.poses.push_back(poseStamped);
            poseStamped.header.seq++;
        }
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubGlobalPathPlan.publish(msg);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishCurrentPathPlan(const std::list<tPoint> &path, const tPose& robotPose)
    {
        if( pubCurrentPathPlan.getNumSubscribers() == 0 )   return;

        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.header.seq = 0;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.pose.position.x = robotPose.x;
        poseStamped.pose.position.y = robotPose.y;
        poseStamped.pose.position.z = 0;
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 1;
        msg.poses.push_back(poseStamped);

        for ( tPoint point : path )
        {
            poseStamped.header.seq++;
            poseStamped.pose.position.x = point.x;
            poseStamped.pose.position.y = point.y;
            msg.poses.push_back(poseStamped);
        }
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCurrentPathPlan.publish(msg);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishDstarMap(u8 *pSrc, tGridmapInfo *pGridmapInfo)
    {
        if (pubDstarMap.getNumSubscribers() == 0)           return;
        if ( pSrc == nullptr || pGridmapInfo == nullptr )   return;

        nav_msgs::OccupancyGrid map;

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
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubDstarMap.publish(map);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishEscapeWallCells(std::deque<CEscapeWall::CellInfo> cells, std::deque<CEscapeWall::CellInfo> debugCells)
    {
        //계산 리소스 낭비를 피하기 위해 게시자를 구독하는 node 없는지 확인하는 데 사용
        if (pubEscapeWallCells.getNumSubscribers() == 0)    return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker cellMarker;
        
        cellMarker.id = 0;
        cellMarker.header.frame_id = "/map";
        cellMarker.type = visualization_msgs::Marker::CUBE;
        cellMarker.action = visualization_msgs::Marker::ADD;
        cellMarker.scale.x = 0.33;
        cellMarker.scale.y = 0.33;
        cellMarker.scale.z = 0.33;
        cellMarker.color.a = 0.3;
        cellMarker.pose.position.z = 0.0;
        
        for(auto cell : cells)
        {
            cellMarker.id = cell.index;
            cellMarker.pose.position.x = cell.point.x;
            cellMarker.pose.position.y = cell.point.y;
            if(cell.state == CEscapeWall::E_CELL_STATE::UNKNOWN)
            {
                cellMarker.color.r = 0.5;
                cellMarker.color.g = 0.5;
                cellMarker.color.b = 0.5;
            }
            else
            {
                cellMarker.color.r = 0.0;
                cellMarker.color.g = 1.0;
                cellMarker.color.b = 0.0;
            }
            data.markers.emplace_back(cellMarker);
        }
        for(auto cell : debugCells)
        {
            cellMarker.id = cell.index;
            cellMarker.pose.position.x = cell.point.x;
            cellMarker.pose.position.y = cell.point.y;
            cellMarker.color.r = 1.0;
            cellMarker.color.g = 0.0;
            cellMarker.color.b = 0.0;
            data.markers.emplace_back(cellMarker);
        }
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubEscapeWallCells.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishCleanArea(std::list<CCleanRoom> rooms, int currentRoomId)
    {
        if (pubCleanArea.getNumSubscribers() == 0)  return;
        if (rooms.empty()==true)                    return;

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
                    
                    for(tPoint polygon : area.getPolygon())
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
                    else if ( area.getCleanState() == E_CLEAN_STATE::CLEANED )
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
                    for(tPoint polygon : area.getPolygon())
                    {
                        centerX += polygon.x;
                        centerY += polygon.y;
                    }
                    centerX /= area.getPolygonSize();
                    centerY /= area.getPolygonSize();

                    for(tPoint polygon : area.getPolygon())
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
        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCleanArea.publish(data);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishRobotPath(std::list<tPoint> robotPath)
    {
        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.header.seq = 0;
        poseStamped.header.stamp = ros::Time::now();

        for ( tPoint point : robotPath )
        {
            poseStamped.pose.position.x = point.x;
            poseStamped.pose.position.y = point.y;
            poseStamped.pose.position.z = 0;
            poseStamped.pose.orientation.x = 0;
            poseStamped.pose.orientation.y = 0;
            poseStamped.pose.orientation.z = 0;
            poseStamped.pose.orientation.w = 1;

            msg.poses.push_back(poseStamped);
        }

        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubRobotPath.publish(msg);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishCostMap(tPose robotPose, std::list<tPoint> debugWallData)
    {
        nav_msgs::OccupancyGrid costmap;
        geometry_msgs::TransformStamped tfCostmap;

        tfCostmap.header.frame_id = "map";
        tfCostmap.child_frame_id = "local_costmap";

        tfCostmap.header.stamp = ros::Time::now();

        tfCostmap.transform.translation.x = robotPose.x - 1;
        tfCostmap.transform.translation.y = robotPose.y - 1;
        tfCostmap.transform.translation.z = 0.0;

        tfCostmap.transform.rotation.x = 0.0;
        tfCostmap.transform.rotation.y = 0.0;
        tfCostmap.transform.rotation.z = 0.0;
        tfCostmap.transform.rotation.w = 1.0;

        tfbc.sendTransform(tfCostmap);

        costmap.header.seq = 0;
        costmap.header.stamp = ros::Time::now();
        costmap.header.frame_id = "local_costmap";
        
        costmap.info.height = 40;                   // costmap size 2[m] x 2[m], resolution [m/cell]
        costmap.info.width = 40;
        costmap.info.resolution = 0.05; 
        costmap.info.origin.position.x = 0;
        costmap.info.origin.position.y = 0;
        costmap.info.origin.position.z = 0;

        costmap.info.origin.orientation.w = 1;
        costmap.info.origin.orientation.x = 0;
        costmap.info.origin.orientation.y = 0;
        costmap.info.origin.orientation.z = 0;

        costmap.data.resize(costmap.info.width * costmap.info.height, 0);
      
        for (const auto& wallPoint : debugWallData)
        {
            int cellX = (wallPoint.x - robotPose.x) / costmap.info.resolution + costmap.info.width*0.5;
            int cellY = (wallPoint.y - robotPose.y) / costmap.info.resolution + costmap.info.height*0.5;

            if (cellX >= 0 && cellX < costmap.info.width && cellY >= 0 && cellY < costmap.info.height)
            {
                int index = cellX + cellY * costmap.info.height;
                costmap.data[index] = 254;
            }
        }
        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCostMap.publish(costmap);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }
    
    

    void CRosPublisher::publishContour(std::list<tPoint> contour)
    {
        if (contour.size() <= 0) return;

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


        
        tmpMarker.color.r = 240;
        tmpMarker.color.g = 120;
        tmpMarker.color.b = 40;
        tmpMarker.color.a = 1.0;

        for (auto polygon : contour)
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
        
        
        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubContour.publish(data);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishContour2(std::list<tPoint> contour)
    {
        if (contour.size() <= 0) return;

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
        
        tmpMarker.color.r = 240;
        tmpMarker.color.g = 0;
        tmpMarker.color.b = 0;
        tmpMarker.color.a = 1.0;

        for (auto polygon : contour)
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
        
        
        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubContour2.publish(data);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishShrinkContour(std::list<tPoint> contour)
    {
        if (contour.size() <= 0) return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;

        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.type = visualization_msgs::Marker::LINE_STRIP;
        tmpMarker.action = visualization_msgs::Marker::ADD;
        tmpMarker.scale.x = 0.05;
        tmpMarker.scale.y = 0.0;
        tmpMarker.scale.z = 0.0;
        tmpMarker.pose.orientation.w =1.0;

        geometry_msgs::Point p;
        
        tmpMarker.color.r = 255;
        tmpMarker.color.g = 255;
        tmpMarker.color.b = 0;
        tmpMarker.color.a = 1.0;

        for (auto polygon : contour)
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
        
        
        pubShrinkContour.publish(data);
    }

    void CRosPublisher::publishContourList(std::list<std::list<tPoint>> rooms)
    {
        if (rooms.size() <= 0) return;

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


        
        tmpMarker.color.r = 240;
        tmpMarker.color.g = 10;
        tmpMarker.color.b = 40;
        tmpMarker.color.a = 1.0;

        for(auto room : rooms){
            for (auto polygon : room)
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
        
        
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubContourList.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "<-" );    
    }

    void CRosPublisher::publishCleanLine1(tPose pose, tPoint point)
    {
        if( pubCleanLine1.getNumSubscribers() == 0 )   return;

        visualization_msgs::Marker marker;
        geometry_msgs::Point start_point, goal_point;

        marker.id = 0;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // color: orange
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.12;
        marker.color.a = 1.0;

        marker.scale.x = 0.05;

        start_point.x = pose.x;
        start_point.y = pose.y;
        goal_point.x = point.x;
        goal_point.y = point.y;

        marker.points.push_back(start_point);
        marker.points.push_back(goal_point);
        // ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCleanLine1.publish(marker);
        // ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishCleanLine2(tPose pose, tPoint point)
    {
        if( pubCleanLine2.getNumSubscribers() == 0 )   return;

        visualization_msgs::Marker marker;
        geometry_msgs::Point start_point, goal_point;

        marker.id = 0;
        marker.header.frame_id = "/map";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // color: purple
        marker.color.r = 0.5;
        marker.color.g = 0.12;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.scale.x = 0.05;

        start_point.x = pose.x;
        start_point.y = pose.y;
        goal_point.x = point.x;
        goal_point.y = point.y;

        marker.points.push_back(start_point);
        marker.points.push_back(goal_point);
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubCleanLine2.publish(marker);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishDoor(std::list<std::list<tPoint>> doors)
    {
        if (doors.size() <= 0) return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;

        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.type = visualization_msgs::Marker::LINE_STRIP;
        tmpMarker.action = visualization_msgs::Marker::ADD;

        tmpMarker.scale.x = 0.05;
        tmpMarker.scale.y = 0.0;
        tmpMarker.scale.z = 0.0;
        tmpMarker.pose.orientation.w =1.0;

        geometry_msgs::Point p;


        for (auto id : doors)
        {
            // color: Beige
            tmpMarker.color.r = 1.0;
            tmpMarker.color.g = 0.8;
            tmpMarker.color.b = 0.6;
            tmpMarker.color.a = 1.0;

            for (auto polygon : id)
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
        ceblog(LOG_LV_NECESSARY, GRAY, "->" );    
        pubDoor.publish(data);
        ceblog(LOG_LV_NECESSARY, GRAY, "debug" );
    }

    void CRosPublisher::publishCleaningArea(tPose curPose)
    {
        geometry_msgs::Point robotPoint;
        
        robotPoint.x = curPose.x;
        robotPoint.y = curPose.y;
        robotPoint.z = 0;

        cleanAreaMarker.points.emplace_back(robotPoint);

        pubCleaningArea.publish(cleanAreaMarker);
    }

    void CRosPublisher::publishCleaningLine(tPose curPose)
    {
        geometry_msgs::Point robotPoint;
        
        robotPoint.x = curPose.x;
        robotPoint.y = curPose.y;
        robotPoint.z = 0;

        cleanLineMarker.points.emplace_back(robotPoint);

        pubCleaningLine.publish(cleanLineMarker);
    }

    void CRosPublisher::publishForbiddenLine(std::list<tPoint> line)
    {
        if (line.size() <= 0) return;

        visualization_msgs::Marker data;

        data.id = 0;
        data.header.frame_id = "/map";
        data.header.stamp = ros::Time::now(); // 현재 시간으로 타임스탬프 설정
        data.type = visualization_msgs::Marker::LINE_STRIP;
        data.action = visualization_msgs::Marker::ADD;
        data.scale.x = 0.05;
        data.scale.y = 0.0;
        data.scale.z = 0.0;
        data.pose.orientation.w =1.0;

        geometry_msgs::Point p;
        
        data.color.r = 1.0;
        data.color.g = 0;
        data.color.b = 0;
        data.color.a = 1.0;

        for (auto point : line)
        {
            p.x=point.x;
            p.y=point.y;
            p.z =0.0;
            data.points.emplace_back(p);
        }
        
        pubForbiddenLine.publish(data);
    }

    void CRosPublisher::publishForbiddenRect(std::list<tPoint> rect)
    {
        if (rect.size() <= 0) return;

        visualization_msgs::MarkerArray data;
        visualization_msgs::Marker tmpMarker;

        tmpMarker.id = 0;
        tmpMarker.header.frame_id = "/map";
        tmpMarker.header.stamp = ros::Time::now(); // 현재 시간으로 타임스탬프 설정
        tmpMarker.type = visualization_msgs::Marker::LINE_STRIP;
        tmpMarker.action = visualization_msgs::Marker::ADD;
        tmpMarker.scale.x = 0.05;
        tmpMarker.scale.y = 0.0;
        tmpMarker.scale.z = 0.0;
        tmpMarker.pose.orientation.w =1.0;

        geometry_msgs::Point p;
        
        tmpMarker.color.r = 1.0;
        tmpMarker.color.g = 1.0;
        tmpMarker.color.b = 0;
        tmpMarker.color.a = 1.0;

        for (auto polygon : rect)
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

        pubForbiddenArea.publish(data);
    }
    
    /**
     * @brief 
     * 
     * @param cells 
     * @param orgWidth 
     * @param orgHeigh 
     * @param cropSize 
     */
    void CRosPublisher::publishCleanMap(cell *cells,int orgWidth, int orgHeigh, int cropSize)
    {
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
        pubCleanMap.publish(map);
    }
}


