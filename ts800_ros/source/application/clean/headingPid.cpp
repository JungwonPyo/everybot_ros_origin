#include "headingPid.h"
#include "commonStruct.h"
#include "ros/rosParameter.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CHeadingPid::CHeadingPid(/* args */)
{
    CStopWatch __debug_sw;

    crosstrack_error = 0.0;
    int_crosstrack_error = 0.0;
    diff_crosstrack_error = 0.0;
    max_steering_angle = HEADING_MAX_STEER_SPEED;
    
    kP = 20.0;
    kI = 0.01;
    kD = 3.0;

    targetAngle = 0.0;
    currentAngle = 0.0;
    steer = 0.0;

    bRun = true;

    isRunningControlLooper = true;
    thControlLooper  = std::thread(&CHeadingPid::threadControlLooper, this);
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CHeadingPid::~CHeadingPid()
{
    CStopWatch __debug_sw;
    
    isRunningControlLooper = false;
    thControlLooper.join();

    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CHeadingPid::debug_setpid()
{
    CStopWatch __debug_sw;
    
    kP = ROS_CONFIG.angleControl.kP;
    kI = ROS_CONFIG.angleControl.kI;
    kD = ROS_CONFIG.angleControl.kD;
    max_steering_angle = ROS_CONFIG.max_steering_angle;

    eblog(LOG_LV, "[heading Pid] kP : " << ROS_CONFIG.angleControl.kP);
    eblog(LOG_LV, "[heading Pid] kI : " << ROS_CONFIG.angleControl.kI);
    eblog(LOG_LV, "[heading Pid] kD : " << ROS_CONFIG.angleControl.kD);
    eblog(LOG_LV, "[heading Pid] max : " << ROS_CONFIG.max_steering_angle);
    eblog(LOG_LV, "[heading Pid] speed : "<< ROS_CONFIG.speed_fwd);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CHeadingPid::init(double _currentAngle, double _targetAngle)
{
    CStopWatch __debug_sw;
    
    currentAngle = _currentAngle;
    targetAngle = _targetAngle;
    crosstrack_error = 0.0;
    int_crosstrack_error = 0.0;
    diff_crosstrack_error = 0.0;
    crosstrack_error = getError(currentAngle, targetAngle);
    //debug_setpid();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CHeadingPid::run()
{
    CStopWatch __debug_sw;

    bRun = true;
    eblog(LOG_LV,  "heading pid run"<<std::endl);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CHeadingPid::stop()
{
    CStopWatch __debug_sw;

    bRun = false;
    eblog(LOG_LV,  "heading pid stop"<<std::endl);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

double CHeadingPid::getSteer()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return steer;
}

void CHeadingPid::setCurrentAngle(double set)
{
    CStopWatch __debug_sw;

    currentAngle = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CHeadingPid::setTargetAngle(double set)
{
    CStopWatch __debug_sw;

    targetAngle = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

double CHeadingPid::getError(double currentAngle, double targetAngle)
{
    CStopWatch __debug_sw;

    double error = 0.0;
    s32 s32error = 0;
    //나머지 연산을 위해 360.000 -> 360000
    s32 s32tgtAngle = (s32)(targetAngle * 1000.0);
    s32 s32curAngle = (s32)(currentAngle * 1000.0);

    s32tgtAngle = s32tgtAngle % (s32)360000;
    s32curAngle = s32curAngle % (s32)360000;

    s32error = s32tgtAngle - s32curAngle;

    if ( s32error < -180000)
    {
        s32error = 360000 + s32error;
    }
        
    if (s32error > 180000)
    {
        s32error = s32error - 360000;
    }

    error = (double)s32error / 1000.0;

    TIME_CHECK_END(__debug_sw.getTime());
    return error;
}


void CHeadingPid::threadControlLooper()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "thread_control start");

    int cnt = 0;

    while (isRunningControlLooper)
    {
        if (bRun)
        {
            diff_crosstrack_error = -crosstrack_error;
            crosstrack_error = getError(targetAngle, currentAngle);
            diff_crosstrack_error += crosstrack_error;
            int_crosstrack_error += crosstrack_error;
            steer = (-kP * crosstrack_error-kD*diff_crosstrack_error-kI*int_crosstrack_error); //-kI * int_crosstrack_error

            if (steer > max_steering_angle)
                steer = max_steering_angle;
            if (steer < -max_steering_angle)
                steer = -max_steering_angle;


            // if (cnt++>20)
            // {
            //     std::cout<<"steer : "<<steer<< " P :"<<crosstrack_error<<" I : "<<int_crosstrack_error<<" D :"<< diff_crosstrack_error<<std::endl;
            //     cnt = 0;
            // }

            //__debug_print(robotPose, tragetPoint, steer);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    eblog(LOG_LV,  "thread_control fin");
    
    TIME_CHECK_END(__debug_sw.getTime());
}