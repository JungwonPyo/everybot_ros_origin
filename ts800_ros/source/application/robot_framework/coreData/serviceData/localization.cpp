#include <atomic>
#include <limits>
#include "coreData/serviceData/localization.h"
#include "externData/externData.h"
#include "eblog.h"
#include "utils.h"
#include "control/control.h"
#include "localization.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CLidar::CLidar()
{
    //std::cout << "----------->CLidar()" << std::endl;    
    lidarinit = false;
    memset(lidarDist,0,sizeof(double) * LIDAR_DIST_BUFF_SIZE);
    eblog(LOG_LV,  "");
}
CLidar::~CLidar()
{
    eblog(LOG_LV,  "");
}

/**
 * @brief Set the Lidar Dist object
 * 
 * @param dist 
 */
void CLidar::setLidarDist(double const *dist)
{
    CStopWatch __debug_sw;
    lidarDataMutex.lock();
    
    if (dist == NULL)
    {
        lidarinit = false;
    }
    else
    {
        lidarinit = true;
        memcpy(lidarDist, dist, sizeof(double) * LIDAR_DIST_BUFF_SIZE);  
    }

    lidarDataMutex.unlock();
    TIME_CHECK_END(__debug_sw.getTime());
}


void CLidar::setRawLidarDist(sensor_msgs::LaserScan msg)
{
	CStopWatch __debug_sw;
    rawLidarDataMutex.lock();
    
    rawLidarData.angle_increment=msg.angle_increment;
    rawLidarData.angle_max=msg.angle_max;
    rawLidarData.angle_min=msg.angle_min;
    rawLidarData.header=msg.header;
    rawLidarData.range_max=msg.range_max;
    rawLidarData.range_min=msg.range_min;
    rawLidarData.scan_time=msg.scan_time;
    rawLidarData.time_increment=msg.time_increment;

    rawLidarData.ranges.clear();
    for(float range : msg.ranges)
    {
        rawLidarData.ranges.push_back(range);
    }
    rawLidarDataMutex.unlock();
	TIME_CHECK_END(__debug_sw.getTime());
}

sensor_msgs::LaserScan CLidar::getRawLidarDist()
{
    if(rawLidarData.ranges.size()==0)
    {
        // std::cout<<"__ rawLidarData SIZE  ==   0  __"<<std::endl;
    }
    return rawLidarData;
}

/**
 * @brief Get the Lidar Dist object
 * 
 * @return double* 
 */
double* CLidar::getLidarDist()
{
    if (lidarDist == NULL)
    {
        return NULL;
    }
    else
    {
        return lidarDist;
    }
}

double CLidar::getLidarDist(int idx)
{
	CStopWatch __debug_sw;
    double ret = 0;
    
    lidarDataMutex.lock();
    if (lidarDist == NULL)
    {
        ret = -1;
    }
    else 
    {
        ret = lidarDist[idx];
    }
    lidarDataMutex.unlock();
	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 가장 가까운 포인트를 찾는다.
 * 
 * @param idx 
 * @param dist 
 */
void CLidar::getNear(int *idx, double *dist)
{
	CStopWatch __debug_sw;
    double minDistance = std::numeric_limits<double>::max(); // 가능한 최대 double 값으로 초기화
    int minIndex = -1; // 가장 가까운 거리의 인덱스를 저장할 변수

    // 가장 가까운 거리를 찾습니다.
    for (int i = 0; i < 360; ++i) {
        if (lidarDist[i] < minDistance) {
            minDistance = lidarDist[i];
            minIndex = i;
        }
    }

    *idx = minIndex;
    *dist = lidarDist[minIndex];
}

bool CLidar::getLidarInit(void)
{
    return lidarinit;
}

CRobotPose::CRobotPose()
{
    firstRosTime = -1;
}

CRobotPose::~CRobotPose(){}

void CRobotPose::setSlamPose(tPose set)
{
	CStopWatch __debug_sw;
    slamPose = set;

    /* Low Pass Filter 업데이트 */
    lpFilter.updatePoint( tPoint(slamPose.x, slamPose.y) );
	TIME_CHECK_END(__debug_sw.getTime());
}

void CRobotPose::setSysPose(tPose set){
    sysPose = set;
}

void CRobotPose::setAccAngle(double degAngle)
{
    double degDiff = degAngle - tempDegAngle;

    if ( degDiff < -180.0)
    {
        degDiff += 360;
    }
    if ( degDiff > 180.0)
    {
        degDiff -= 360;
    }

    accDegAngle += degDiff;
    accRadAngle += DEG2RAD(degDiff);

    tempDegAngle = degAngle;
}

void CRobotPose::setPredictedAngle(double heading, ros::Time timestamp)
{
    if(firstRosTime == -1)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "############# first ros time : "<<firstRosTime);
        firstRosTime=timestamp.toSec();
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "############# first ros time : "<<firstRosTime);
    }
    double time = (timestamp.toSec()-firstRosTime); //+timestamp.toNSec()/10e9;

    angleExtrapolation.updateAngle(heading, time);
    // eblog(LOG_LV_NECESSARY, "시간[ "<<time<<" ]\t직전 헤딩 각도, " << RAD2DEG(heading) << ", 예측 각도 결과, " << RAD2DEG(predictedAngle));
}

void CRobotPose::setRobotPose(tPose externalData)
{
    robotPose.x     = externalData.x;
    robotPose.y     = externalData.y;
    robotPose.angle = externalData.angle;
}

tPose CRobotPose::getPose()
{
#if 0
    CStopWatch __debug_sw;
    tPose ret;
    ret.x = slamPose.x;
    ret.y = slamPose.y;
    if(CONFIG.bActivePredictAngle)  ret.angle = angleExtrapolation.getPredictedHeading(sysPose.angle, 0.06); // sysPose.angle; 
    else                            ret.angle = sysPose.angle;
#endif
    return robotPose;
}

tPose CRobotPose::getSlamPose(){
    return slamPose;
}

tPose CRobotPose::getSysPose(){
    return sysPose;
}

tPose CRobotPose::getLpfPose()
{
	CStopWatch __debug_sw;
	tPose ret;
	
    tPoint point = lpFilter.getLpfPoint();
	ret = tPose(point.x, point.y, utils::math::deg2rad(sysPose.angle));
	TIME_CHECK_END(__debug_sw.getTime());
	
	return ret;
}

tPose CRobotPose::getLpfInterPose()
{
	CStopWatch __debug_sw;
	tPose ret;
    tPoint point = lpFilter.getInterPolationPoint();
	ret = tPose(point.x, point.y, utils::math::deg2rad(sysPose.angle));
	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

tPose CRobotPose::getLpfExtraPose()
{
	CStopWatch __debug_sw;
	tPose ret;
    tPoint point = lpFilter.getExtraPolationPoint();
	ret = tPose(point.x, point.y, utils::math::deg2rad(sysPose.angle));
    TIME_CHECK_END(__debug_sw.getTime());
	return ret;
}

double CRobotPose::getAccDegAngle()
{
    return accDegAngle;
}

double CRobotPose::getAccRadAngle()
{
    return accRadAngle;
}

CLocalizData::CLocalizData()
{
    eblog(LOG_LV,   "create");
}

CLocalizData::~CLocalizData()
{
    eblog(LOG_LV,   "destroy");
}

void CLocalizData::update(CExternData* pExternData)
{
	CStopWatch __debug_sw;
    /* system pose update */
    if (pExternData->systemData.isUpdateSysPoseData() == true)
    {
        //hjkim230728 - progress 밀림현상 개선을 위한 위치 이동.
        tSysPose sysPose = pExternData->systemData.useSysPoseData();
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
        tPose ceva = tPose(sysPose.x, sysPose.y, sysPose.angle, sysPose.calSn);
#else
        tPose ceva = tPose(sysPose.x, sysPose.y, sysPose.angle);
#endif
        pose.setSysPose(ceva);
        pose.setAccAngle(ceva.angle);
        pose.setPredictedAngle(sysPose.angle, sysPose.timeStamp);
        pose.setRobotPose(pExternData->rosData.getRobotPose());

        fusionPoint.updatePose(pExternData->rosData.getSlamPose(), ceva);
#if 0 // 누적각도 log
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "누적 각도: "<<pose.getAccDegAngle()<<" deg\t"<<pose.getAccRadAngle()<<" rad");
#endif
#if 0 // /* @@@ Debug - ceva와 slam 의 좌표가 차이나는 경우 log 출력 - 23.06.22 jspark @@@ */
        tPose slam = pExternData->rosData.getSlamPose();
        double slamAngle = utils::math::rad2deg(slam.angle);
        double gap = utils::math::calculateMinimumDegreeAngle(ceva.angle, slamAngle);
        if( gap >= 90 && (ROBOT_CONTROL.slam.isSlamRunning() ))
        {
            ceblog(LOG_LV_NECESSARY, YELLOW, "Warning!!! Ceva [" <<ceva.x<<" ] [" <<ceva.y<<" ] [" <<ceva.angle<<" ]\t Slam  [ "<<slam.x<<" ][ "<<slam.y<<" ][ "<<slamAngle<<" ]\t Gap [ "<<gap<<" > 90 ]");
        }
        else
        {
            // ceblog(LOG_LV_NECESSARY, GREEN, "Ceva [ "<<ceva.angle<<" ]\t Slam [ "<<slamAngle<<" ]\t Gap [ "<<gap<<" ]");
        }
        ceblog(LOG_LV_SYSTEMINF, GREEN, "system Pose  x: " << getSysPose().x << "\ty: " << getSysPose().y << "\tz: " << getSysPose().angle);
#endif
    }
    else
    {
        /* mcu로 부터 system pose 가 들어오지 않는 경우 */
    }

    /* slam pose update */
    if (true)
    {
        setSlamPose( pExternData->rosData.getSlamPose() );

        // ceblog(LOG_LV_SYSTEMINF, MAGENTA, "slam Pose  x: " << slamPose.x << "\ty: " << slamPose.y << "\tz: " << slamPose.angle);
    }

    /* lidar data update */
    if (true)
    {
        setLidarBuffer(pExternData->rosData.lidar.getLidarDist());
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

bool CLocalizData::getLidarInit()
{
    return lidar.getLidarInit();
}

double CLocalizData::getLidarDist(int idx)
{
    return lidar.getLidarDist(idx);
}

sensor_msgs::LaserScan CLocalizData::getRawLidarDist()
{
    return lidar.getRawLidarDist();
}

double* CLocalizData::getLidarBuffer()
{
    return lidar.getLidarDist();
}

void CLocalizData::setLidarBuffer(double const *buf)
{
	CStopWatch __debug_sw;
    lidar.setLidarDist(buf);
	TIME_CHECK_END(__debug_sw.getTime());
}

void CLocalizData::setRawLidarDist(sensor_msgs::LaserScan msg)
{
	CStopWatch __debug_sw;
    lidar.setRawLidarDist(msg);
	TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Get the Slam Pose object
 * 
 * @return tPose 
 */
tPose CLocalizData::getPose()
{
#if 0 // ts800_odom topic으로 받기 전 (by hyjoe)
    tPoint fusion = fusionPoint.getPoint();
    double angle = pose.getPose().angle;
    return tPose(fusion.x, fusion.y, angle);
#endif
    return pose.getPose();
}

/**
 * @brief 
 * 
 * @return tPose 단위 m, m, deg
 */
tPose CLocalizData::getSysPose(void)
{
    //std::cout << "getSlamPose()" << std::endl;
    return pose.getSysPose();
}

void CLocalizData::setSlamPose(tPose set)
{
	CStopWatch __debug_sw;
	
    pose.setSlamPose(set);
	TIME_CHECK_END(__debug_sw.getTime());
}

tPose CLocalizData::getSlamPose(void)
{
    //std::cout << "getSlamPose()" << std::endl;
    return pose.getSlamPose();
}

tPose CLocalizData::getLpfPose(void)
{
    return pose.getLpfPose();
}

tPose CLocalizData::getLpfInterpolationPose()
{
    return pose.getLpfInterPose();
}

tPose CLocalizData::getLpfExtrapolationPose()
{
    return pose.getLpfExtraPose();
}

/**
 * @brief [ ros -> monitor ] 로봇의 자기 위치 좌표
 */
void CLocalizData::setSlamPose(double x, double y, double yaw)
{
	CStopWatch __debug_sw;
    tPose rosPose(x,y,yaw);
    pose.setSlamPose(rosPose);
	TIME_CHECK_END(__debug_sw.getTime());
}

CFusionPoint::CFusionPoint()
{
    oldSlam = tPose();
    oldSystem = tPose();
}

CFusionPoint::~CFusionPoint() {}

void CFusionPoint::updatePose(tPose newSlam, tPose newSystem)
{
    // slam 좌표가 업데이트 되는 경우 - slam 좌표를 사용.
    if( oldSlam.x != newSlam.x || oldSlam.y != newSlam.y )
    {
        fusion = tPoint(newSlam.x, newSlam.y);
        oldSlam = newSlam;
        oldSystem = newSystem;
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "SLAM("<<newSlam.x<<", "<<newSlam.y<<"\t "<<BOLDRED<<"Fusion("<<BOLDWHITE<<fusion.x<<", "<<fusion.y<<BOLDBLACK<<")");
    }
    else // slam 좌표가 업데이트 되지 않는 경우 - pixart 변화량을 이용하여 slam 좌표의 공백을 채워줌.
    {
        // 차후에는 회전행렬을 통해 Pixart와 Slam 좌표계사이의 차이를 넣어주세요.
        tPoint delta;
        delta.x = newSystem.x-oldSystem.x;
        delta.y = newSystem.y-oldSystem.y;

        fusion.x = oldSlam.x + delta.x;
        fusion.y = oldSlam.y + delta.y;
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "SLAM("<<newSlam.x<<", "<<newSlam.y<<"\t "<<BOLDGREEN<<"Fusion("<<BOLDWHITE<<fusion.x<<", "<<fusion.y<<BOLDBLACK<<")");
    }
}

tPoint CFusionPoint::getPoint()
{
    return fusion;
}
