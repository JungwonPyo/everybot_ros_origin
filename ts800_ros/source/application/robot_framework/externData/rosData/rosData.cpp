#include "externData/rosData/rosData.h"
#include "debugCtr.h"
#include "rosData.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRosData::CRosData(ros::NodeHandle _nh) : CRosInterface(_nh), bSlamPoseUpdate(false), bFilterImuUpdate(false)
{
    init();
}

CRosData::~CRosData()
{

}

tPose CRosData::getSlamPose()
{
    return slamPose;
}

tPose CRosData::getRobotPose()
{
    return robotPose;
}

std::string CRosData::getRosCmd()
{
    std::string ret;
    if(rvizGoalPose.isUpdate())
    {
        ret = std::string("RVIZ_GOAL");
        DEBUG_CTR.rvizGoalPose.set( rvizGoalPose.get() );
    }
    else
    {
        ret = rosCmd;
        rosCmd = std::string("");
    }

    return ret;
}

void CRosData::setSlamPose(double x, double y, double yaw)
{
    slamPose = tPose(x, y, yaw);
    // cevalog(LOG_LV_CEVA_ONLY, x<<", "<<y<<", "<<yaw);
}

void CRosData::setSlamMap(tGridmapInfo info, s8* data)
{
    gridMapRaw.set(data, info);
}

void CRosData::setLidarDist(double const *dist)
{
    lidar.setLidarDist(dist);
}

void CRosData::setRawLidarDist(sensor_msgs::LaserScan msg)
{
    lidar.setRawLidarDist(msg);
}

void CRosData::setTrajectory(std::list<tPoint> path)
{
    trajectory.set(path);
}

void CRosData::setKeyCmd(std::string cmd)
{
    rosCmd = cmd;
}

void CRosData::setUpdateSlamPose(bool ret)
{
    bSlamPoseUpdate = ret;
}

bool CRosData::getUpdateSlamPose()
{
    return bSlamPoseUpdate;
}

void CRosData::setVelocityControl(double linearVelocity, double angularVelocity)
{
    tTwist ds(linearVelocity, angularVelocity);
    velocityControl.set(ds);
}

/**
 * @brief 
 * 
 * @param r 
 * @param p 
 * @param y 
 * @param ax 
 * @param ay 
 * @param az 
 */
void CRosData::setFilterImu(double r, double p, double y, double ax, double ay, double az)
{   
    imu.Ax = ax;
    imu.Ay = ay;
    imu.Az = az;
    imu.Groll = r;
    imu.Gpitch = p;
    imu.Gyaw = y;

    setUpdateFilterImu(true);
}

/**
 * @brief 
 * 
 * @return tSysIMU
 */
tSysIMU CRosData::getFilterImu()
{
    return imu;
}

bool CRosData::isUpdateFilterImu()
{
    return bFilterImuUpdate;
}

void CRosData::setUpdateFilterImu(bool set)
{
    bFilterImuUpdate = set;
}

void CRosData::setTargetPose(tPose targetPose)
{
    this->targetPose.set(targetPose);
}

void CRosData::setRvizGoalPose(tPose rvizGoalPose)
{
    this->rvizGoalPose.set(rvizGoalPose);
}

void CRosData::setRobotPose(tPose input)
{
    robotPose.x     = input.x;
    robotPose.y     = input.y;
    robotPose.angle = input.angle;
}