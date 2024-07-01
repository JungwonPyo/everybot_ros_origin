#include "serviceIdle.h"
#include "userInterface.h"
#include "MessageHandler.h"
#include "systemTool.h"
#include "utils.h"
#include "define.h"
#include "motionPlanner/motionPlanner.h"
#include "debugCtr.h"
#include  "subTask.h"
#include "kinematics.h"
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
CServiceIdle::CServiceIdle(CServiceReady* _pServiceReady, CSupplyWater* _pSupplyWater, CLocation* _pLocation, CRobotSlamPose* _pRobotSlamPose,CKidnap* _pKidnap)
    : pServiceReady(_pServiceReady), pSupplyWater(_pSupplyWater), pLocation(_pLocation), pRobotSlamPose(_pRobotSlamPose), pKidnap(_pKidnap)
{
    eblog(LOG_LV, "create");
    
    manualMoving = false;
    avoidStep = MANUAL_MOVING_AVOID_STEP::NONE;
}

CServiceIdle::~CServiceIdle()
{
    eblog(LOG_LV,  "");
}

void CServiceIdle::serviceInitial()
{
    // TODO : 배터리 부족, 청소 끝나면 시작한다.
    // if(배터리부족or청소끝or홈키)
    // TODO : 값을 초기화!!!
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceIdle Class");
    bootingAwsReport = false;
    setServiceStatus(E_SERVICE_STATUS::startup); 
}


E_SERVICE_STATUS_STEP CServiceIdle::startupStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    /* AWS서버 관련*/
    DISPLAY_CTR.startAutoDisplay(true,DISPLAY_CTR.getBlinkEyeImage());
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
    ROBOT_CONTROL.reportAwsStatus(1);
    MOTION.startStopOnMap(tProfile(),true);
    
    manualMoving = false;
    completedByForce = false;
    // 대기상태 슬램 끄기.. 임시 처리 relocalize 정리 후 삭제
    if(ROBOT_CONTROL.slam.isSlamRunning()) ROBOT_CONTROL.slam.exitSlam();
    eblog(LOG_LV_NECESSARY,  "[CServiceIdle Class]  - starttup Step Ready");
    
    PATH_PLANNER->stopMapUpdate(); // hjkim240228 : 프론티어 안꺼지는 현상 수정
    SUB_TASK.waveFrontier.stopUpdate();
    //ROBOT_CONTROL.tilting.tilting(E_TILTING_CONTROL::STOP);
    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);

    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    if(!MOTION.isRunning()){
        LED_CTR.sleepMode(); // 임시: LEDOFF
        debugtick = SYSTEM_TOOL.getSystemTick();
        startTime =  SYSTEM_TOOL.getSystemTime();
        setServiceStartTime();
        eblog(LOG_LV_NECESSARY,  "[CServiceIdle] StartupExecuting EXECUTING -> TERMINAITION");
        pLocation->initLocation();
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }

    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::startupStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
        
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::running);    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    awsTime = SYSTEM_TOOL.getSystemTime();
    
    return ret;
}


E_SERVICE_STATUS_STEP CServiceIdle::runningStepExecuting()
{
    CStopWatch __debug_sw;
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
	
	tPose robotPose = ServiceData.localiz.getPose();

    //버튼 입력과 반복 실행 주기가 일치하는 경우에 대한 버그 해결해야함, 잔수제거하면 딜레이 생김 
    if(pSupplyWater->isActiveWaterDrain())
    {
        pSupplyWater->checkWaterDrainTimeOut();
    }
    if(ServiceData.battery.isUpateState()){
        DISPLAY_CTR.startAutoDisplay(true,DISPLAY_CTR.getBlinkEyeImage());
    } 

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceIdle::runningStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    setServiceStatus(E_SERVICE_STATUS::completed);
    DISPLAY_CTR.stopAutoDisplay();
        
    return ret;
}


E_SERVICE_STATUS_STEP CServiceIdle::pauseStepReady()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::EXECUTING;
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::pauseStepExecuting()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::pauseStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::pauseStepTerminaition()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}


E_SERVICE_STATUS_STEP CServiceIdle::completedStepReady()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::EXECUTING;
    eblog(LOG_LV_NECESSARY, enumToString(getServiceId()));
    MOTION.startStopOnMap(tProfile(),true);
    
        
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::completedStepExecuting()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::completedStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceIdle::completedStepTerminaition()
{
    E_SERVICE_STATUS_STEP ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}




