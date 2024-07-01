
#include "serviceClean.h"
#include "userInterface.h"
#include "systemTool.h"
#include "MessageHandler.h"
#include "fileMng.h"
#include "utils.h"
#include "motionPlanner/motionPlanner.h"
#include  "subTask.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CServiceClean::CServiceClean(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CSupplyWater* _pSupplyWater, CLocation* _pLocation, CKidnap* _pKidnap, CRobotSlamPose* _pRobotSlamPose)
    : pServiceMacro(_pServiceMacro),pServiceReady(_pServiceReady), pSupplyWater(_pSupplyWater), pLocation(_pLocation), pKidnap(_pKidnap), pRobotSlamPose(_pRobotSlamPose)
{
    
    eblog(LOG_LV_NECESSARY, "new  pLineClean");

    // debug demo init
    state = _ready;
    demo_time_cnt = 0;
    demo_cnt = 0;

    eblog(LOG_LV,  "created");
}

CServiceClean::~CServiceClean()
{    
    
    eblog(LOG_LV,  "");
}

void CServiceClean::serviceInitial()
{
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceClean Class");

    // if (_ServiceData.obstaclemap == NULL)
    // {
    //     pLineClean->setObsMapPointer(_ServiceData.obstaclemap);
    // }

    setServiceStatus(E_SERVICE_STATUS::startup); 
}


E_SERVICE_STATUS_STEP CServiceClean::startupStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::READY;
    ROBOT_CONTROL.reportAwsStatus(2);
    MOTION.startStopOnMap(tProfile(),true);
    
    /* 청소 서비스를 시작할 때 UI 추가 */
    ceblog(LOG_LV_NECESSARY, CYN," UI <- Clean Service Start");

    DISPLAY_CTR.startDisplay(E_DisplayImageClass::START_CLEANING);
    DISPLAY_CTR.startAutoDisplay(false,DISPLAY_CTR.getBlinkEyeImage());
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_AUTO_CLEAN_START);
    LED_CTR.playCleanServiceStart();

    completedByForce = false;
    //ROBOT_CONTROL.tilting.tilting(E_TILTING_CONTROL::STOP);
    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);
    ServiceData.robotData.setCleanHistoryStartTime(utils::math::getCurrentTimeString(ServiceData.robotData.getRobotData().country,1));
    ret = E_SERVICE_STATUS_STEP::EXECUTING;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    tPose robotPose = ServiceData.localiz.getPose();
    tPose systempPose = ServiceData.localiz.getSysPose();
    tPose slamtPose = ServiceData.localiz.getSlamPose();
    if(!MOTION.isRunning()){
        if(pServiceReady->runReayServiceStep(E_SERVICE_ID::CLEAN))
        {
    #if defined (USE_SLAM_MAP_LOAD_RELOCAL) && (USE_SLAM_MAP_LOAD_RELOCAL == 1)
            if ( ROBOT_CONTROL.slam.isSlamMapLoading())
            {
                eblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG,  "[CServiceClean Class] start map loading localization");
                taskReLocalMotion.taskStart(slamtPose, systempPose, robotPose);
            }
    #endif 
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            eblog(LOG_LV_NECESSARY,  "[CServiceClean Class] serviceStart complete");
        }
        else
        {
            /* 청소 시작이 실패하는 경우 UI 추가 */
        }
    }
    

    //eblog(LOG_LV,  "[CServiceClean Class]  - fin");

    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::startupStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    tPose robotPose = ServiceData.localiz.getPose();
    tPose systempPose = ServiceData.localiz.getSysPose();
    tPose slamtPose = ServiceData.localiz.getSlamPose();
#if defined (USE_SLAM_MAP_LOAD_RELOCAL) && (USE_SLAM_MAP_LOAD_RELOCAL == 1)
    if ( taskReLocalMotion.taskRun(slamtPose, systempPose, robotPose) )
#endif
    {
        //지도 로딩 시 로봇 리로컬 완료 후 청소 시작
        //청소 중 들림이 발생할 경우에도 taskReLocalMotion 을 사용하세요.
        eblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG,  "[CServiceClean Class::TERMINAITION] start map loading localization");
        ServiceData.obstacle.initObstacleSensor();
        ServiceData.robotMap.robotTrajectory.clearCleanedTrajectory();

        taskClean.taskStart(robotPose);
        setServiceStartTime();
        cleanStartTime = SYSTEM_TOOL.getSystemTime();
        setServiceStatus(E_SERVICE_STATUS::running);
    }

    return ret;
}


E_SERVICE_STATUS_STEP CServiceClean::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::READY;
    tPose robotPose = ServiceData.localiz.getPose();

    //if(taskLocalMotion.taskRun(robotPose))
    {

        ROBOT_CONTROL.reportAwsStatus(3);

        ret = E_SERVICE_STATUS_STEP::EXECUTING;
    }
    
    return ret;
}
E_SERVICE_STATUS_STEP CServiceClean::runningStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    tPose robotPose = ServiceData.localiz.getPose();
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    
    /* AWS에 보내는 청소한 경로데이터 (코드삭제하지마시오) */
    ServiceData.robotMap.robotTrajectory.setCleanedTrajectory(robotPose);

    
    if(taskClean.taskRun(robotPose))
    {
        ServiceData.kidnapData.setCheckingKidnap(false);
        DISPLAY_CTR.stopAutoDisplay();
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }
    if(ServiceData.battery.isUpateState()){
        DISPLAY_CTR.startAutoDisplay(true,DISPLAY_CTR.getBlinkEyeImage());
    } 

    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::runningStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    setServiceStatus(E_SERVICE_STATUS::completed);   // 다음 진행할 서비스 상태 결정.

    return ret;
}



E_SERVICE_STATUS_STEP CServiceClean::pauseStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    
    ROBOT_CONTROL.reportAwsStatus(4);

    
    pSupplyWater->stopWaterSupply();
  
    ceblog(LOG_LV_UI, CYN, "UI <- Clean Service Pause");
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::pauseStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::pauseStepWaiting() 
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::pauseStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::completedStepReady()
{    
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    pSupplyWater->stopWaterSupply();
    MOTION.startStopOnMap(tProfile(),true);
    
    ServiceData.robotData.setCleanHistoryRobotPose(ServiceData.localiz.getPose());
    ServiceData.robotData.setCleanHistoryCleanTime(getServiceStartTime());
    ServiceData.robotData.setCleanHistoryCleanedSize(SUB_TASK.cleanPlan.getCleanedSize());
    ServiceData.robotData.setCleanHistoryAreaInfo("정의필요");
    ServiceData.robotData.setCleanHistoryTraj(ServiceData.robotMap.robotTrajectory.getCleanedTrajectory());
    ServiceData.robotData.setCleanHistoryCradlePose(SUB_TASK.cleanPlan.getNoGoZoneDockingCoord());

    if(completedByForce)
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
        ServiceData.robotData.setCleanHistoryExitReason("cancel");
    }
    else
    {
        /* 내부에서 완료 된 경우 */
        ServiceData.robotData.setCleanHistoryExitReason("complete");

        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CLEAN_COMPLT_TO_CHARGER);
    }
    
    ROBOT_CONTROL.reportCleanHistroy();
    ROBOT_CONTROL.reportOperationArea();
    ROBOT_CONTROL.reportAwsCleanMode();
    ServiceData.robotMap.robotTrajectory.clearCleanedTrajectory();
    

    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::completedStepExecuting()
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

E_SERVICE_STATUS_STEP CServiceClean::completedStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceClean::completedStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    ROBOT_CONTROL.WaterPump(false);
    eblog(LOG_LV_NECESSARY,  "CServiceClean serviceCompleted Class");
    return ret;
}

#if 0 
/*
* return : 
*   SERVICE_STATUS - 리턴값으로 상황에 따라 다음 서비스를 결정 할 수 있다.
*/
E_SERVICE_STATUS CServiceClean::serviceProc()
{
    E_SERVICE_STATUS ret = E_SERVICE_STATUS::running;
    tPose robotPose = ServiceData.localiz.getPose();
    //pSupplyWater->WaterSupplyStateMachine(getServiceStartTick());

 #if TEST_RANDOMPATTERN_FOR_CLEANTIME > 0   
    if(pLineClean->runRandomClean(robotPose))
    {
        cleanTime = get_system_time()-cleanStartTime;
        eblog(LOG_LV_NECESSARY,  "CServiceClean serviceCompleted CleanTime : " << cleanTime);
        ret = E_SERVICE_STATUS::completed;
    }
 #else   
    if ( pLineClean->runLineclean(robotPose) )
    {
        ret = E_SERVICE_STATUS::completed;
    }
#endif

    return ret;
}
#endif

void CServiceClean::setCleanMode(int set)
{
    //pClean->setCleanMode(set);
}


