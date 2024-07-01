#include <atomic>
#include "robotSlamPose.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRobotSlamPose::CRobotSlamPose()
{
    pausePose.pauseSlamPose.angle = 0.0;
    pausePose.pauseSlamPose.x = 0.0;
    pausePose.pauseSlamPose.y = 0.0;
    
    pausePose.pauseSystemPose.angle = 0.0;
    pausePose.pauseSystemPose.x = 0.0;
    pausePose.pauseSystemPose.y = 0.0;
}

CRobotSlamPose::~CRobotSlamPose(){}

void CRobotSlamPose::setPauseSlamPose(tPose set)
{
    pausePose.pauseSlamPose = set;
}

void CRobotSlamPose::setPauseSlamPose(double x, double y, double yaw)
{
    tPose rosPose(x,y,yaw);

    setPauseSlamPose(rosPose);
}

tPose CRobotSlamPose::getPauseSlamPose()
{
    return pausePose.pauseSlamPose;
}

void CRobotSlamPose::setPauseSystemPose(tPose set)
{
    pausePose.pauseSystemPose = set;
}

void CRobotSlamPose::setPauseSystemPose(double x, double y, double yaw)
{
    tPose rosPose(x,y,yaw);

    setPauseSystemPose(rosPose);
}

tPose CRobotSlamPose::getPauseSystemPose()
{
    return pausePose.pauseSystemPose;
}