#include "userInterface.h"
#include "serviceDocking.h"
#include "systemTool.h"
#include "fileMng.h"
#include "utils.h"
#include "MessageHandler.h"
#include "motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief 도킹 서비스 관리 및 도킹 클래스 생성
 */
CServiceDocking::CServiceDocking(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CLocation* _pLocation, CKidnap* _pKidnap, CRobotSlamPose* _pRobotSlamPose)
    : pServiceMacro(_pServiceMacro),pServiceReady(_pServiceReady), pLocation(_pLocation), pKidnap(_pKidnap), pRobotSlamPose(_pRobotSlamPose)
{   
    eblog(LOG_LV_NECESSARY, "new pDocking");

    serviceDockinSlamPoseInfo.x = 0.0;
    serviceDockinSlamPoseInfo.y = 0.0;
    serviceDockinSlamPoseInfo.angle = 0.0;

    eblog(LOG_LV, "create");
}

CServiceDocking::~CServiceDocking()
{
    eblog(LOG_LV,  "");
}

void CServiceDocking::serviceInitial()
{
    // TODO : 배터리 부족, 청소 끝나면 시작한다.
    // if(배터리부족or청소끝or홈키)
    // TODO : 값을 초기화!!!
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceDocking Class");

    setServiceStatus(E_SERVICE_STATUS::startup); 
}

E_SERVICE_STATUS_STEP CServiceDocking::startupStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    ROBOT_CONTROL.reportAwsStatus(2);
    MOTION.startStopOnMap(tProfile(),true);
    
    /* 자동도킹 서비스가 시작 될 때 UI 추가 */
    ceblog(LOG_LV_UI, CYN," UI <- Docking Service Start");

    // SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MOVE_TO_CHARGER);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::GO_TO_CHARGER);
    LED_CTR.playDockingServiceStart();
    
    completedByForce = false;
    eblog(LOG_LV,  "[CServiceClean Class]  - startup init");
    
    //ROBOT_CONTROL.tilting.tilting(E_TILTING_CONTROL::STOP);
    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);

    return ret;
}
E_SERVICE_STATUS_STEP CServiceDocking::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    
    if(MOTION.isRunning()){
        
    } 
    else{
        if(pServiceReady->runReayServiceStep(E_SERVICE_ID::DOCKING))
        {
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
            //충전기 위치를 알고 있다면 충전기 위치로 보내기,  충전기 위치를 모른다면, 충전기를 찾아보기
            taskDocking.taskStart();
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            setServiceStartTime();   
        }
        else
        {
            /* 자동도킹 서비스가 실패하는 경우 UI */
        }
    }

    return ret;
}
E_SERVICE_STATUS_STEP CServiceDocking::startupStepWaiting() 
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceDocking::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::running);
    pServiceReady->setRelocalFirstChangeStep(RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP);
    
    return ret;
}


E_SERVICE_STATUS_STEP CServiceDocking::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::READY;

    // if(ROBOT_CONTROL.slam.isSlamRelocalize())
    // {
    //     if(pServiceReady->runRelocalizeServiceStep())
    //     {
    //         ret = E_SERVICE_STATUS_STEP::EXECUTING; 
    //     }
    // }
    // else
    {

        ROBOT_CONTROL.reportAwsStatus(7);       
        ret = E_SERVICE_STATUS_STEP::EXECUTING;
    }
    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::runningStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    tPose robotPose = ServiceData.localiz.getPose();
    /* AWS서버 관련*/

#if USE_KIDNAP
    if(pKidnap->autoKidnapAndEstimateRobotPose() == false)
    {
        status = serviceProc();    
    }
#else
    if(taskDocking.taskRun(robotPose))
    {
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }

#endif


    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::runningStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    
    setServiceStatus(E_SERVICE_STATUS::completed);   // 다음 진행할 서비스 상태 결정.    

    return ret;
}


E_SERVICE_STATUS_STEP CServiceDocking::pauseStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    /* AWS서버 관련*/
    ROBOT_CONTROL.reportAwsStatus(8);
    
        
    /* 탐색 서비스가 시작 될 때 UI 추가 */
    ceblog(LOG_LV_UI, CYN, "UI <-  Service Pause");

    LED_CTR.pause();

    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::pauseStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::pauseStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::pauseStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::completedStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    eblog(LOG_LV_NECESSARY,  "[CServiceDocking Class]");
    MOTION.startStopOnMap(tProfile(),true);
    
    // //ROBOT_CONTROL.tilting(E_TILTING_CONTROL::STOP); // 틸팅 stop 
    
    if(completedByForce)
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
    }
    else
    {
        /* 내부에서 완료 된 경우 */


        /* 자동도킹 서비스가 종료 될 때 UI 추가 */
        ceblog(LOG_LV_UI, CYN," UI <- Docking Service End");
    }


    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::completedStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    
    if(completedByForce)
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
        eblog(LOG_LV_SERVICESTEP,  "completedByForce");
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }
    else
    {
        /* 내부에서 완료 된 경우 */
        eblog(LOG_LV_SERVICESTEP,  "completedByNature");
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }

    return ret;
}

E_SERVICE_STATUS_STEP CServiceDocking::completedStepWaiting() 
{
    return E_SERVICE_STATUS_STEP::WAITING; // hhryu230831 : cppcheck error로 인한 임시 return (미사용 함수)
}

E_SERVICE_STATUS_STEP CServiceDocking::completedStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

