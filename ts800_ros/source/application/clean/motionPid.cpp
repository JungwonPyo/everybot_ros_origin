
#include "motionPid.h"
#include "utils.h"
#include "eblog.h"
#include <math.h>
#include "fileMng.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CMotionPid::CMotionPid()
{
    CStopWatch __debug_sw;

    crosstrack_error = 0.0;
    int_crosstrack_error = 0.0;
    diff_crosstrack_error = 0.0;

    max_steering_angle = RSF_PI/4.0;
    max_steering_angle = 50.0;
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;

    errType = AXIS_ANGLE;
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CMotionPid::~CMotionPid()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CMotionPid::init(tPose robotPose, tPoint traget, E_PID_ERROR_TYPE type)
{
    CStopWatch __debug_sw;

    targetPoint = traget;
    errType = type;

    crosstrack_error = 0.0;
    int_crosstrack_error = 0.0;
    diff_crosstrack_error = 0.0;

    crosstrack_error = getError(robotPose, traget);

    kP = ROS_CONFIG.angleControl.kP;
    kI = ROS_CONFIG.angleControl.kI;
    kD = ROS_CONFIG.angleControl.kD;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CMotionPid::init(tPoint startPoint, tPoint targetPoint)
{
    errType = E_PID_ERROR_TYPE::AXIS_SLIP;
    this->startPoint = startPoint;
    this->targetPoint = targetPoint;

    kP = ROS_CONFIG.angleControl.kP;
    kI = ROS_CONFIG.angleControl.kI;
    kD = ROS_CONFIG.angleControl.kD;
}

double CMotionPid::run(tPose robotPose)
{
    CStopWatch __debug_sw;

    kP = ROS_CONFIG.angleControl.kP;
    kI = ROS_CONFIG.angleControl.kI;
    kD = ROS_CONFIG.angleControl.kD;
    max_steering_angle = DEG2RAD(45); // angle error 사용시 70.

    double steer = 0.0;
    diff_crosstrack_error = crosstrack_error*(-1);
    crosstrack_error = getError(robotPose, targetPoint);
    diff_crosstrack_error += crosstrack_error;
    int_crosstrack_error += crosstrack_error;
    steer = (kP * crosstrack_error + kD*diff_crosstrack_error  + kI*int_crosstrack_error); //-kI * int_crosstrack_error

    if (steer > max_steering_angle)
        steer = max_steering_angle;
    if (steer < -max_steering_angle)
        steer = -max_steering_angle;


    //__debug_print(robotPose, targetPoint, steer);

    TIME_CHECK_END(__debug_sw.getTime());

#if 0 // angle 에러 Debug
    ceblog(LOG_LV_CONTROL, BOLDBLACK, "Robot("<<BOLDWHITE<<robotPose.x<<", "<<robotPose.y<<", "<<RAD2DEG(robotPose.angle)<<BOLDBLACK
        <<") -> Target("<<BOLDYELLOW<<targetPoint.x<<", "<<targetPoint.y<<BOLDBLACK
        <<") 에러 각도는: "<<GREEN<<RAD2DEG(crosstrack_error)<<BOLDBLACK
        <<" deg\t -> w(steer): "<<BOLDGREEN<<RAD2DEG(steer)<<" deg/s");
#elif 1 // 휭방향 에러 Debug
    ceblog(LOG_LV_CONTROL, BOLDBLACK, "Robot("<<BOLDWHITE<<robotPose.x<<", "<<robotPose.y<<", "<<RAD2DEG(robotPose.angle)<<BOLDBLACK
        <<") -> Target("<<BOLDYELLOW<<targetPoint.x<<", "<<targetPoint.y<<BOLDBLACK
        <<") 횡방향 에러 거리는: "<<GREEN<<int(crosstrack_error*1000)<<BOLDBLACK
        <<" mm\t -> w(steer): "<<BOLDGREEN<<RAD2DEG(steer)<<" deg/s");
#else
#endif
    return steer;
}

void CMotionPid::stop()
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

double CMotionPid::getError(tPose robotPose, tPoint targetPoint)
{
    CStopWatch __debug_sw;

    lookahead = ROS_CONFIG.lookahead;
    tPose headingPose;
    
    double ret = 0.0;
    switch (errType)
    {
    case E_PID_ERROR_TYPE::AXIS_X :
        ret = getErrorAxisX(robotPose, targetPoint); //std::cout <<" TODO : not yet"<<std::endl;
        break;
    case E_PID_ERROR_TYPE::AXIS_Y :
        ret = getErrorAxisY(robotPose, targetPoint);
        break;
    case E_PID_ERROR_TYPE::AXIS_ANGLE :
        ret = getErrorAngle(robotPose, targetPoint);
        break;
    case E_PID_ERROR_TYPE::AXIS_SLIP:
        headingPose.x = robotPose.x + lookahead*std::cos(robotPose.angle);
        headingPose.y = robotPose.y + lookahead*std::sin(robotPose.angle);
        headingPose.angle = robotPose.angle;
        ret = getErrorSlip(headingPose);
    default:
    std::cout <<" E_PID_ERROR_TYPE not define"<<std::endl;
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief y 축 방향의 error 만 설정.
 * 
 * @param robotPose 
 * @return double 
 */
double CMotionPid::getErrorAxisX(tPose robotPose, tPoint targetPoint)
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return targetPoint.x -  robotPose.x;
}

double CMotionPid::getErrorAxisY(tPose robotPose, tPoint targetPoint)
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return targetPoint.y -  robotPose.y;
}

double CMotionPid::getErrorAngle(tPose robotPose, tPoint targetPoint)
{
    double tempTargetAngle = atan2(targetPoint.y - robotPose.y, targetPoint.x - robotPose.x); // 목표점을 향한 목표각도 (rad)
    double errAngle = utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle);
    return errAngle;
}

double CMotionPid::getErrorSlip(tPose robotPose)
{
    double distErr; // 휭 방향 오차 절대값
    distErr = ((targetPoint.y-startPoint.y)*robotPose.x-(targetPoint.x-startPoint.x)*robotPose.y+targetPoint.x*startPoint.y-targetPoint.y*startPoint.x)/sqrt(pow(targetPoint.y-startPoint.y, 2)+pow(targetPoint.x-startPoint.x, 2));
    // ceblog(LOG_LV_CONTROL, BOLDBLACK, "횡 방향 거리 오차: "<<BOLDGREEN<<int(distErr*1000)<<BOLDBLACK<<" mm");
    return distErr;
}

/**
 * @brief debug 출력.
 * 
 * @param robotPose 
 * @param tragetPose 
 * @param steer 
 */
void CMotionPid::__debug_print(tPose robotPose, tPoint targetPoint, double steer)
{
    CStopWatch __debug_sw;

    std::cout << std::fixed;
    std::cout.precision(4);
    std::cout << "curP[" << robotPose.x << ", " << robotPose.y << "] ";
    std::cout << "tgtP[" << targetPoint.x << ", " << targetPoint.y << "] ";
    std::cout << "steer[" << steer << "] ";
    std::cout << "KP[" << crosstrack_error << "] ";
    std::cout << "KD[" << diff_crosstrack_error << "] ";
    std::cout << "KI[" << int_crosstrack_error << "] ";
    std::cout << "P-ERROR[" << -kP*crosstrack_error << "] ";
    std::cout << "D-ERROR[" << -kD*diff_crosstrack_error << "] ";
    std::cout << "I-ERROR[" << -kI*int_crosstrack_error << "] ";
    
    std::cout << std::endl;
    
    TIME_CHECK_END(__debug_sw.getTime());
}