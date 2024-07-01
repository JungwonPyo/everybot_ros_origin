

#include "lidarSensor.h"
#include "eblog.h"
#include "utils.h"
#include "control/motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

const int LIFE_MAX = 1; // 50 -> 1 //감지 된 상태에서 장애물이 없어질 때 필터 계수 (높이면 장애물이 없는 경우를 여러번 체크한다)
const int TERM_THRESHOLD = 1; // 20 -> 3 //장애물이 없는 상태에서 장애물이 생겼을 때 감지 계수 (높이면 장애물이 있는 경우를 여러번 체크한다)
const double DIST_THRESHOLD = 0.3; // 0.5 -> 0.35

#define LIDAR_POS_WIDTH_FROM_ROBOT 0.15
#define LIDAR_POS_HIGHT_FROM_ROBOT 0.24


#define S_TURNING_RADIUS  0.38
#define LIDAR_POS_WIDTH_FROM_STURN_SPINDLE 0.08
#define LIDAR_POS_HIGHT_FROM_STURN_SPINDLE 0.16

#define TURNING_RADIUS  0.3//0.18
#define LIDAR_POS_WIDTH_FROM_TURN_SPINDLE 0
#define LIDAR_POS_HIGHT_FROM_TURN_SPINDLE 0.16

#define ERROR_LIMIT 0.001

CLidarSensors::CLidarSensors(/* args */){
    initLidarSensor();
    for (int i = 0; i < LIDAR_POINT_SIZE; i++)
    {
        lidarPoint[i].clear();
        lidarPoint[i].setId(i);
    }
}
CLidarSensors::~CLidarSensors(){

}

void CLidarSensors::initLidarSensor(){
    mask_front.approach.value = 0;
    mask_front.obstacle.value = 0;
    frontIdxRange = getLidarMaskIndex(); 
    avg_idx_count = frontIdxRange/4;
    leftFrontidx = frontIdxRange;
    rightFrontidx = LIDAR_POINT_SIZE-frontIdxRange;
}

LIDAR_RSF_OBSTACLE CLidarSensors::getLidarMask()
{
    return mask_front;
}

LIDAR_RSF_OBSTACLE CLidarSensors::getLidarLeftWallMask()
{
    return mask_left;
}

LIDAR_RSF_OBSTACLE CLidarSensors::getLidarRightWallMask()
{
    return mask_right;
}

/**
 * @brief 로봇 크기에 따른 LIDAR센서 회피 각도 설정 (로봇의 절반 기준 각도 : 좌/우 방향을 고려하여 해당 각도 * 2 를 하면 전방 좌/우 방향에 대한 라이다 감지각도를 구할 수 있다)
 *        로봇의 몸체에서 라이다 센서 장착 위치를 pos로 설정.
 *        라이다의 위치부터 로봇의 정면과 측면 세점을 연결하여 라이다가 감지해야할 각도를 구한다.
 *        라이다 센서의 mask를 8point로 설정하였기 때문에 해당각도를 4등분하여, 각도를 구한다.
 *        라이다센서가 로봇의 가로측 중심부에 위치하기 때문에 좌/우 2방향에 대하여 해당 각도만큼을 전방 회피 각도로 설정한다.
 * @param dist 라이다 거리정보.
 */
int CLidarSensors::getLidarMaskIndex(void)
{
    int ret = 0;
    double pos_width = LIDAR_POS_WIDTH_FROM_ROBOT;
    double pos_hight = LIDAR_POS_HIGHT_FROM_ROBOT;
    double radian = utils::math::getRadianAngle(pos_width,pos_hight);
    double theta = utils::math::rad2deg(radian);
    ret = (int)theta;

    eblog(LOG_LV_NECESSARY,  "getLidarMaskIndex ret :" << (int)ret);
    return ret;
}

double getReferenceRadian(int idx)
{
    CStopWatch __debug_sw;
    double degree; 
    double radian;
    double ret = 0.0;

    if(idx == 0 || idx > 180)   degree = (double)(360-idx);
    else                        degree = (double)idx;    

    radian =  utils::math::deg2rad(degree);

    for (double i = 0; i < M_PI/2 ; i+=utils::math::deg2rad(0.1))
    {
        double height = (TURNING_RADIUS*cos(i)-(TURNING_RADIUS-LIDAR_POS_WIDTH_FROM_ROBOT));
        double width = (TURNING_RADIUS*sin(i)+LIDAR_POS_HIGHT_FROM_ROBOT);
        double error = fabs(tan(radian)-(height/width));
        if(error <= ERROR_LIMIT)
        {
            ret = i;
            //eblog(LOG_LV_NECESSARY,  "getReferenceTheta idx : " << idx << " degree : " << degree << " radian : " << radian << "degree :" << utils::math::rad2deg(i));
            break;
        }
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

double getMovingAxisTurnReference(double radian)
{
    return sqrt(pow(S_TURNING_RADIUS*sin(radian)+LIDAR_POS_HIGHT_FROM_ROBOT,2)+pow(S_TURNING_RADIUS*(cos(radian))-(S_TURNING_RADIUS-LIDAR_POS_WIDTH_FROM_ROBOT),2));
}

double getFixedAxisTurnReference(double radian)
{
    return sqrt(pow(TURNING_RADIUS*sin(radian)+LIDAR_POS_HIGHT_FROM_ROBOT,2)+pow(TURNING_RADIUS*cos(radian),2));
}

void CLidarSensors::updateLidarSensor(double const *lidarBuf) 
{
    CStopWatch __debug_sw;
    mask_front.approach.value = 0;
    mask_front.obstacle.value = 0;
    mask_left.obstacle.value = 0;
    mask_right.obstacle.value = 0;

    for (int i = 0; i < LIDAR_POINT_SIZE; i++)
    {
        lidarPoint[i].check(lidarBuf[i],i);
    }

    checkLidarSensorDetectedCount(0, leftFrontidx);
    checkLidarSensorDetectedCount(leftFrontidx+1, 90);
    checkLidarSensorDetectedCount(270,rightFrontidx-1);
    checkLidarSensorDetectedCount(rightFrontidx,359);
    
    TIME_CHECK_END(__debug_sw.getTime());

}


void CLidarSensors::checkLidarSensorDetectedCount(int startDegree, int endDegree)
{
    CStopWatch __debug_sw;
    startDegree = utils::math::deg2deg(startDegree);
    endDegree = utils::math::deg2deg(endDegree);

    int detectedCount = 0;
    int count = 0;
    int idx = 0;
    int mask_dir = 0;

    for ( idx=startDegree; idx!=endDegree; idx = utils::math::deg2deg(idx+1))
    {
        if(idx >= 0 && idx < 360 )
        {
            if ( lidarPoint[idx].isDetected() == true )
            {
                detectedCount++;
                if( idx <= 90 || idx >= 270)
                {
                    if(idx == 0){
                        if(mask_front.obstacle.b.fleft_center == 0)    mask_front.obstacle.b.fleft_center = 1;
                        if(mask_front.obstacle.b.fright_center == 0)   mask_front.obstacle.b.fright_center = 1;
                    }
                    else
                    {
                        if( idx <= leftFrontidx || idx >= rightFrontidx){
                            if(idx <= leftFrontidx)    mask_dir = 4+(idx-startDegree)/avg_idx_count;
                            else                        mask_dir = (idx-startDegree)/avg_idx_count;

                            if((mask_front.obstacle.value & mask_dir)==0){
                                mask_front.obstacle.value |= (0x01 << mask_dir);
                                // ceblog(LOG_LV_NECESSARY, GREEN, "정면LiDAR 감지 체크 - id "<< idx <<"시작각도 "<<startDegree<<"\t 종료각도 "<<endDegree<<" 총 "<<count <<" 개중 "<< detectedCount<<" 개 감지!" <<
                                // " 장애물 MASK : " << (int)mask_front.obstacle.value << " 방향 : " << (int)mask_dir);
                            }
                        }
                        else if(idx <= 90){
                            mask_dir = (idx-startDegree)/7;
                            if((mask_left.obstacle.value & mask_dir)==0){
                                mask_left.obstacle.value |= (0x01 << mask_dir);
                            }
                            //  ceblog(LOG_LV_NECESSARY, GREEN, "측면 좌측 LiDAR 감지 체크 - 시작각도 "<<startDegree<<"\t 종료각도 "<<endDegree<<" 총 "<<count <<" 개중 "<< detectedCount<<" 개 감지!" <<
                            //  "왼쪽 장애물 : " << (int)mask_left.obstacle.value<< " 장애물 방향 : " << (int)mask_dir);
                        }
                        else// if(idx < 320)
                        {
                            mask_dir = (idx-startDegree)/7;
                            if((mask_right.obstacle.value & mask_dir)==0){
                                mask_right.obstacle.value |= (0x01 << mask_dir);
                            }
                            //  ceblog(LOG_LV_NECESSARY, GREEN, "측면 우측 LiDAR 감지 체크 - 시작각도 "<<startDegree<<"\t 종료각도 "<<endDegree<<" 총 "<<count <<" 개중 "<< detectedCount<<" 개 감지!" <<
                            //  " 오른쪽 장애물 : " << (int)mask_right.obstacle.value << " 장애물 방향 : " << (int)mask_dir);
                        }
                    }
                }
            }
        }
        // else
        // {
        //     ceblog(LOG_LV_NECESSARY, GREEN, "checkLidarSensorDetectedCount IDX ERROR!! IDX : "<< idx );
        // }

        count++;
    }
    TIME_CHECK_END(__debug_sw.getTime());

    //ceblog(LOG_LV_NECESSARY, GREEN, "LiDAR 감지 개수 체크 - 시작각도 "<<startDegree<<"\t 종료각도 "<<endDegree<<" 총 "<<count <<" 개중 "<< detectedCount<<" 개 감지!" <<" 왼쪽 장애물 : " << (int)mask_left.obstacle.value<< " 오른쪽 장애물 : " << (int)mask_right.obstacle.value << " 장애물 방향 : " << (int)mask_dir);
}

CLidarPoint::CLidarPoint()
{
    id = 0;
    bDetected = false;
    life = 0;
    term = 0;
    distanceCriteria = DIST_THRESHOLD;
}

CLidarPoint::~CLidarPoint()
{
    clear();
}

bool CLidarPoint::isDetected()
{
    return bDetected;
}

void CLidarPoint::clear()
{
    bDetected = false;
    life = 0;
    term = 0;
}

/**
 * @brief 라이다 주기에 맞추어 입력 한다.
 * 
 * @param dist 라이다 거리정보.
 */
void CLidarPoint::check(double dist, int idx)
{
    CStopWatch __debug_sw;
    //생명을 감소 시킨다.
    //setId(idx);

    //G4_LIDAR RANGE 0.12m - 16m 
    //S2_LIDAR 0.12~8m
    if(CheckLidarDataValidate(dist))
    {
        if (dist < distanceCriteria) life = LIFE_MAX;
    }

    
    life--;

    //생명이 계속 유지되면 term 을 증가 시킨다.
    if (life >= 0){
        term++;
    }
    else{   //생명 유지를 못하면 모두 초기화 시킨다.
        clear();
    }
    //term이 특정 값 이상이면 detected 된다.
    if (term >= TERM_THRESHOLD)
    {
        // if(idx <= 32 || idx >= 328)
        // {
        //     ceblog(LOG_LV_NECESSARY, GREEN, "LiDAR 감지 - idx : "<< idx <<" dist : "<< dist <<"\t ref : "<<distanceCriteria);
        // }
        bDetected = true;
    }
    TIME_CHECK_END(__debug_sw.getTime());
}

void CLidarPoint::setId(int set)
{
    CStopWatch __debug_sw;
    id = set;
    if(id <= 32 || id >= 328)
    {
        distanceCriteria = DIST_THRESHOLD;//getFixedAxisTurnReference(getReferenceRadian(id));
        //ceblog(LOG_LV_NECESSARY , GREEN,  "LIDAR  IDX :" << id << "distanceCriteria :" << distanceCriteria);
    }
    else if (id <= 90 || id >= 270)
    {
        distanceCriteria = getWallSensingDistancebyAngle(id, DIST_THRESHOLD);
    }
    else
    {
        distanceCriteria = DIST_THRESHOLD;
    }
    TIME_CHECK_END(__debug_sw.getTime());
}

int CLidarPoint::getId()
{
    return id;
}


/**
 * @brief 로봇 크기에 따른 LIDAR센서 회피 각도 설정 (로봇의 절반 기준 각도 : 좌/우 방향을 고려하여 해당 각도 * 2 를 하면 전방 좌/우 방향에 대한 라이다 감지각도를 구할 수 있다)
 *        로봇의 몸체에서 라이다 센서 장착 위치를 pos로 설정.
 *        라이다의 위치부터 로봇의 정면과 측면 세점을 연결하여 라이다가 감지해야할 각도를 구한다.
 *        라이다 센서의 mask를 8point로 설정하였기 때문에 해당각도를 4등분하여, 각도를 구한다.
 *        라이다센서가 로봇의 가로측 중심부에 위치하기 때문에 좌/우 2방향에 대하여 해당 각도만큼을 전방 회피 각도로 설정한다.
 * @param dist 라이다 거리정보.
 */
double CLidarPoint::getSpatternLidarObstacleReference(u16 idx)
{
    CStopWatch __debug_sw;
    double ret = 0,degree = 0,theta = 0;

    if(idx >= 180) degree = (double)360-idx;
    else           degree = (double)idx;
    
    
    if(degree >= 0  && degree <= 45){
        if(degree == 45){
        theta = 0;
        }
        else{
            theta = 70.5-(degree*1.6); 
        }

        double radian = utils::math::deg2rad(theta);
        
        ret = sqrt(pow(S_TURNING_RADIUS*sin(radian)+LIDAR_POS_HIGHT_FROM_STURN_SPINDLE,2)+pow(S_TURNING_RADIUS*cos(radian)-LIDAR_POS_WIDTH_FROM_STURN_SPINDLE,2));

        //eblog(LOG_LV,  " getSpatternLidarObstacleReference degree : " << degree << "idx : " << idx << "theta "  << theta << "sin "  << sin_theta << "cos "  << cos_theta << "x : " << x << "y "  << y << "x2 : " << x2 << "y2 "  << y2 << "reference : " << ret);
 
    }
    else
    {
        eblog(LOG_LV,  "degree range over : " << degree << "idx : " << idx << "theta "  << theta);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;

}

/**
 * @brief 로봇 크기에 따른 LIDAR센서 회피 각도 설정 (로봇의 절반 기준 각도 : 좌/우 방향을 고려하여 해당 각도 * 2 를 하면 전방 좌/우 방향에 대한 라이다 감지각도를 구할 수 있다)
 *        로봇의 몸체에서 라이다 센서 장착 위치를 pos로 설정.
 *        라이다의 위치부터 로봇의 정면과 측면 세점을 연결하여 라이다가 감지해야할 각도를 구한다.
 *        라이다 센서의 mask를 8point로 설정하였기 때문에 해당각도를 4등분하여, 각도를 구한다.
 *        라이다센서가 로봇의 가로측 중심부에 위치하기 때문에 좌/우 2방향에 대하여 해당 각도만큼을 전방 회피 각도로 설정한다.
 * @param dist 라이다 거리정보.
 */

double CLidarPoint::getLidarObstacleReference(u16 idx)
{
    CStopWatch __debug_sw;
    double ret = 0,degree = 0,theta = 0;

    if(idx >= 180) degree = (double)360-idx;
    else           degree = (double)idx;
    
    if(degree >= 0  && degree <= 45){
        if(degree == 45){
            ret = sqrt(pow(LIDAR_POS_HIGHT_FROM_TURN_SPINDLE,2)+pow(LIDAR_POS_HIGHT_FROM_TURN_SPINDLE,2));
        }
        else{
            if(degree == 0)  theta = 90;
            else             theta = 90-(degree*2);

            double radian = utils::math::deg2rad(theta);
            ret = sqrt(pow(TURNING_RADIUS*sin(radian)+LIDAR_POS_HIGHT_FROM_TURN_SPINDLE,2)+pow(TURNING_RADIUS*cos(radian)-LIDAR_POS_WIDTH_FROM_TURN_SPINDLE,2));
        }
    }
    else
    {
        eblog(LOG_LV,  "degree range over : " << degree << "idx : " << idx << "theta "  << theta);
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;

    //eblog(LOG_LV,  " getLidarObstacleReference degree : " << degree << "idx : " << idx << "theta "  << theta << "sin "  << sin_theta << "cos "  << cos_theta << "x : " << x << "y "  << y << "x2 : " << x2 << "y2 "  << y2 << "reference : " << ret);
}

double CLidarPoint::getWallSensingDistancebyAngle(int degreeAngle, double wallDistance)
{
    CStopWatch __debug_sw;
    int angle = degreeAngle;
    
    if (angle > 180)    {angle -= 180;}
    angle = abs(90-angle);
    // ceblog(LOG_LV_NECESSARY, BOLDWHITE, "LiDAR 각도: "<<degreeAngle<<"\t원래거리: "<<wallDistance<<"\t수정거리: "<<wallDistance/cos(angle*M_PI/180));
    TIME_CHECK_END(__debug_sw.getTime());
    return DIST_THRESHOLD/cos(angle*M_PI/180);
}


bool CLidarPoint::CheckLidarDataValidate(double dist)
{
    CStopWatch __debug_sw;
    bool ret = true;
    bool infinity = std::isinf(dist); //hjkim230526 - 라이다 거리값이 무한대인경우 평균값 제외
    //G4_LIDAR RANGE 0.12m - 16m 
    //S2_LIDAR 0.12~8m
    if(dist <= LIDAR_MIN_RANGE || dist >= LIDAR_MAX_RANGE || infinity)
    {
        ret = false;
    }  
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
