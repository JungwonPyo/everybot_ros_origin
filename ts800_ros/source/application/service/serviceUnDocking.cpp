/**
 * @file serviceUnDocking.cpp
 * @author hhryu@everybot.net
 * @brief 충전 중, 청소 or 탐색 or 지정 장소 이동 or 예약 청소 등등...을 위한 CLASS
 * @version 0.1
 * @date 2023-04-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "serviceUnDocking.h"
#include "systemTool.h"
#include "eblog.h"
#include "fileMng.h"
#include "utils.h"
#include "commonStruct.h"
#include "motionPlanner/motionPlanner.h"
//#include "MessageHandler.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief 언~도킹 서비스 관리 및 도킹 클래스 생성
 */
CServiceUnDocking::CServiceUnDocking(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CRobotSlamPose* _pRobotSlamPose)
    : pServiceMacro(_pServiceMacro),pServiceReady(_pServiceReady), pRobotSlamPose(_pRobotSlamPose)
{   
    eblog(LOG_LV_NECESSARY, "new pUnDocking");

    serviceUnDockingSlamPoseInfo.x = 0.0;
    serviceUnDockingSlamPoseInfo.y = 0.0;
    serviceUnDockingSlamPoseInfo.angle = 0.0;

    eblog(LOG_LV, "create");
}

CServiceUnDocking::~CServiceUnDocking()
{    
    eblog(LOG_LV, "");
}

void CServiceUnDocking::serviceInitial()
{
    // TODO : 배터리 부족, 청소 끝나면 시작한다.
    // if(배터리부족or청소끝or홈키)
    // TODO : 값을 초기화!!!
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceUnDocking Class");

    setServiceStatus(E_SERVICE_STATUS::startup); 
}

E_SERVICE_STATUS_STEP CServiceUnDocking::startupStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    ROBOT_CONTROL.reportAwsStatus(2);
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
    eblog(LOG_LV,  "[CServiceClean Class]  - Startup Init");
    
#if 0 //test for code - it is not necessory 23.05.03 by yoon 
    ROBOT_CONTROL.clearSystemLocalization();
#endif    
    completedByForce = false;
    //ROBOT_CONTROL.tilting.tilting(E_TILTING_CONTROL::STOP);
    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);

    return ret;
}
E_SERVICE_STATUS_STEP CServiceUnDocking::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;  
    if(!MOTION.isRunning()){
        if(pServiceReady->runReayServiceStep(E_SERVICE_ID::UNDOCKING))
        {
            std::cout << "serviceStartupRun - from CServiceUnDocking Class" << std::endl;
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            taskUndocking.taskStart();
            setServiceStartTime();
        }
    }
    

    return ret;
}
E_SERVICE_STATUS_STEP CServiceUnDocking::startupStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceUnDocking::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::running);
    return ret; 
}


E_SERVICE_STATUS_STEP CServiceUnDocking::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
	    /* AWS서버 관련*/
    ROBOT_CONTROL.reportAwsStatus(11);

    return ret;
}
E_SERVICE_STATUS_STEP CServiceUnDocking::runningStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    tPose robotPose = ServiceData.localiz.getPose();
    // std::cout << "serviceExecute - from CServiceDocking Class" << std::endl;

    if (taskUndocking.taskRun(robotPose))
    {
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
        std::cout << "CServiceUnDocking::servicerunningStepRun() - completed" << std::endl;
    }

    return ret;
}
E_SERVICE_STATUS_STEP CServiceUnDocking::runningStepWaiting() 
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::runningStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::completed);
    return ret;
}


E_SERVICE_STATUS_STEP CServiceUnDocking::pauseStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    /* AWS서버 관련*/
    ROBOT_CONTROL.reportAwsStatus(11);

    

    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::pauseStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::pauseStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::pauseStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}


E_SERVICE_STATUS_STEP CServiceUnDocking::completedStepReady()
{   
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::completedStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::completedStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceUnDocking::completedStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

