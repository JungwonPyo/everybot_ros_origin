#include "serviceCharging.h"
#include "coreData/serviceData.h"
#include "userInterface.h"
#include "systemTool.h"
#include "utils.h"
#include "define.h"
#include "control/motionPlanner/motionPlanner.h"
#include  "subTask.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CServiceCharging::CServiceCharging(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady)
    : pServiceMacro(_pServiceMacro),pServiceReady(_pServiceReady), dockingAliveTime(0.0)
{
    eblog(LOG_LV, "create");
}

CServiceCharging::~CServiceCharging()
{
    eblog(LOG_LV, "");
}

CTaskCharging* CServiceCharging::getChargingTaskPointer()
{
    return &taskCharging;
}

void CServiceCharging::serviceInitial()
{
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceCharging Class");
    setServiceStatus(E_SERVICE_STATUS::startup); 
}

E_SERVICE_STATUS_STEP CServiceCharging::startupStepReady()
{   
    auto ret = E_SERVICE_STATUS_STEP::READY;

    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
    /* 충전 서비스가 시작 될 때 UI 추가 */
    ceblog(LOG_LV_UI, CYN, "UI <- 충전 서비스 시작");
    eblog((LOG_LV | LOG_LV_NECESSARY), "[CServiceCharging Class]  - startupStepReady");
    completedByForce = false;

    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);    
    ret = E_SERVICE_STATUS_STEP::EXECUTING;

    return ret;
}
    
E_SERVICE_STATUS_STEP CServiceCharging::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    if(!MOTION.isRunning()){
        if(pServiceReady->runReayServiceStep(E_SERVICE_ID::CHARGING)) //( checkReadytoCharge())
        {
            taskCharging.taskStart();
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            setServiceStartTime();
        }
        else
        {
            /* 충전 서비스가 실패할 경우 UI */
        } 
    }
    
    // ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    // setServiceStartTime();

    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::startupStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::running);
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    return ret;
}

/*
 * @brief 충전 서비스 run 함수
 * 1. 도킹을 위한 signal 수신 기능 OFF
 * 2. 충전기와 통신확인을 위한 alive 모니터링
 * 3. 틸팅 상태 모니터링 - 틸팅 Down 상태면 Up 실행
 * 4. MCU 단에 POWER_MODE를 CAHRGE 상태로 변경 실행
 * 5. 걸레 건조 옵션에 따라 건조 기능을 실행한다.
 * 6. KEY 입력에 따른 걸레 건조 기능을 실행한다.
 *   
 * @param /*E_DRY_STATE : 걸레 건조 상태 정보
 * @param START::걸레 건조 기능 시작
 * @param DRY_DRYING::걸레 건조 기능 동작 중
 * @param DRY_COMPLE::걸레 건조 완료 (default time : 6 시간 * 1시간 단위 설정 변경 가능하도록 기능 추가 필요) // 걸레 건조 기능이 완료 되면 다시 충전 display로 변경되어야 한다
 */

E_SERVICE_STATUS_STEP CServiceCharging::runningStepExecuting()
{
    CStopWatch __debug_sw;
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    
    // if(ServiceData.bStartDryMop){
    //     ServiceData.bStartDryMop = false;
    //     taskCharging.dryMopStartStop();
    // } 
    taskCharging.taskRun();
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceCharging::runningStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::completed);
    return ret;
}


E_SERVICE_STATUS_STEP CServiceCharging::pauseStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    eblog(LOG_LV, "[CServiceClean Class]  - pause");

    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::pauseStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::pauseStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::pauseStepTerminaition()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::completedStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    
    ROBOT_CONTROL.dryFanOff(); //hjkim230421 - 충전 서비스가 종료되면 걸레 건조 팬을 중지 시킨다.    

    
    if(completedByForce)
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
		
    }
    else
    {
        /* 내부에서 완료 된 경우 */
		ceblog(LOG_LV_UI, CYN, "UI <- 충전 서비스 종료");

        /* 충전 서비스가 종료 될 때 UI 추가 */
    }
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceCharging::completedStepExecuting()
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

E_SERVICE_STATUS_STEP CServiceCharging::completedStepWaiting()
{
    return E_SERVICE_STATUS_STEP::WAITING; // hhryu230831 : cppcheck error로 인한 임시 return (미사용 함수)
}

E_SERVICE_STATUS_STEP CServiceCharging::completedStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}
