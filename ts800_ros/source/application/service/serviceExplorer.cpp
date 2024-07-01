#include "serviceExplorer.h"
#include "systemTool.h"
#include "userInterface.h"
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

/**
 * @brief 탐색 서비스 관리 및 탐색 클래스 생성
 */
CServiceExplorer::CServiceExplorer(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CRobotSlamPose* _pRobotSlamPose,CKidnap* _pKidnap)
    : pServiceMacro(_pServiceMacro),pServiceReady(_pServiceReady), pRobotSlamPose(_pRobotSlamPose), pKidnap(_pKidnap)
{    
    pTaskExplorer = new CTaskExplorer();    
    savedFileIndex = 0;
    savedFileTick  = 0;
    eblog(LOG_LV, "create");
}

CServiceExplorer::~CServiceExplorer()
{    
    delete pTaskExplorer;
    eblog(LOG_LV,  "");
}


void CServiceExplorer::serviceInitial()
{
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceExplorer Class");
    setServiceStatus(E_SERVICE_STATUS::startup); 
}

E_SERVICE_STATUS_STEP CServiceExplorer::startupStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::READY;
    DISPLAY_CTR.stopAutoDisplay();
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::MAPPING1);
    std::list<contextDo> doList = pServiceMacro->getDoList();

    while (doList.front().id != E_SERVICE_ID::EXPLORER)
    {
        doList.pop_front();
    }
    
    if(doList.front().id == E_SERVICE_ID::CLEAN) SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MAP_DRAW_FIRST);
    else                                         SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MAP_DRAW_START);

    completedByForce = false;
    ROBOT_CONTROL.reportAwsStatus(2);
    MOTION.startStopOnMap(tProfile(),true);
    
    pServiceReady->setServiceReadyState(E_SERVICE_READY::CHECK_START);
    eblog(LOG_LV_NECESSARY,  "[CServiceExplorer startup]  - Step : Ready");
    ret = E_SERVICE_STATUS_STEP::EXECUTING;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::startupStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    bool checkStart = false; //점검 완료시 TRUE
    ServiceData.__testCheckDstar = true;
    //eblog(LOG_LV,  "serviceStart - from CServiceExplorer Class");
    if(!MOTION.isRunning()){
        if(pServiceReady->runReayServiceStep(E_SERVICE_ID::EXPLORER))
        {
            pTaskExplorer->taskStart();
            savedFileIndex = 0;
            savedFileTick  = 0;
            eblog(LOG_LV_NECESSARY,  "탐색 서비스 준비 완료");
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            setServiceStartTime();
        }
        else
        {
            /* 탐색 서비스가 실패하는 경우 */
        }
    }
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::startupStepWaiting() 
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::startupStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    
    ServiceData.robotMap.robotTrajectory.clearExploredTrajectory();
    setServiceStatus(E_SERVICE_STATUS::running);
    pServiceReady->setRelocalFirstChangeStep(RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP);
    ret = E_SERVICE_STATUS_STEP::READY;
    
    return ret;
} 

E_SERVICE_STATUS_STEP CServiceExplorer::runningStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::READY;

    if(ROBOT_CONTROL.slam.isSlamMapLoading())
    {
        if(pServiceReady->runRelocalizeServiceStep())
        {
            ROBOT_CONTROL.reportAwsStatus(5);
            ret = E_SERVICE_STATUS_STEP::EXECUTING; 
        }
    }
    else
    {
        ROBOT_CONTROL.reportAwsStatus(5);
        ret = E_SERVICE_STATUS_STEP::EXECUTING;
    }
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::runningStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    tPose robotPose = ServiceData.localiz.getPose();
    ServiceData.robotMap.robotTrajectory.setExploredTrajectory(robotPose); // 탐색중일때 청소한 좌표를 저장.
    
    if(pTaskExplorer->taskRun(robotPose))
    {
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
        eblog(LOG_LV_NECESSARY, "CServiceExplorer::runningStepRun() - completed" << std::endl);
    }
    DISPLAY_CTR.runExploreDisplay();
#if 0 //trajectory data is not used..
    if(savedFileTick>100)
    {
        if(ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
        {
            ServiceData.robotMap.simplifyMap.simplifyGridMapLock();

            savedFileTick=1;
            double YAMLSet;
            tGridmapInfo mapInfo;

            mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
            unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
                
            YAMLSet = ServiceData.mapStorage.updateGridMapToYAML(pMap, mapInfo, savedFileIndex);
            YAMLSet = ServiceData.mapStorage.updateTrajectoryToYAML( ServiceData.robotMap.robotTrajectory.getExploredTrajectory(), savedFileIndex);
            
            ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
            savedFileIndex++;
        }
    }
    savedFileTick++;
#endif
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::runningStepTerminaition() 
{
    //play of the sound MAP_DRAWING_WAIT safy time margine
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    setServiceStatus(E_SERVICE_STATUS::completed);

    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;   
}


E_SERVICE_STATUS_STEP CServiceExplorer::pauseStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    /* AWS서버 관련*/
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
    ROBOT_CONTROL.reportAwsStatus(6);

    // 탐색 일시 정지 중에 D* wavefrontier 를 정지 한다.
    PATH_PLANNER->stopMapUpdate();
    SUB_TASK.waveFrontier.stopUpdate();
    
    /* 탐색 서비스 일시정지 UI  */
    LED_CTR.pause();
    
    eblog(LOG_LV_NECESSARY,  "[Service Explorer Pause Step: Ready");

    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::pauseStepExecuting()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
     
    
    //eblog(LOG_LV_NECESSARY,  "[Service Explorer Pause Step: Executing]");

    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::pauseStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::pauseStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    //eblog(LOG_LV_NECESSARY,  "[Service Explorer Pause Step: TERMINAITION]");

    return ret;
}


E_SERVICE_STATUS_STEP CServiceExplorer::completedStepReady()
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    std::list<contextDo> doList = pServiceMacro->getDoList();
    while (doList.front().id == E_SERVICE_ID::EXPLORER)
    {
        doList.pop_front();
    }

    eblog(LOG_LV_NECESSARY,  "[CServiceExplorer Class]");
	ServiceData.__testCheckDstar = false;
    MOTION.startStopOnMap(tProfile(),true);
    
    PATH_PLANNER->stopMapUpdate();
    SUB_TASK.waveFrontier.stopUpdate();    

    ServiceData.robotData.setSaveMapRobotPose(ServiceData.localiz.getPose());
    ServiceData.robotData.setSaveMapAreaInfo("정의필요");
    ServiceData.robotData.setSaveMapUniqueKey("UNIQUE_KEY");
    ServiceData.robotData.setSaveMapName("MAP_NAME");
    ServiceData.robotData.setSaveMapOrder(1);


    if(completedByForce)
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
    }
    else
    {
        /* 내부에서 완료 된 경우 */
        ServiceData.robotData.setSaveMaptraj(ServiceData.robotMap.robotTrajectory.getCleanedTrajectory());
        
        /* 탐색 서비스가 종료 될 때 UI 추가 */    
        LED_CTR.playExplorerServiceEnd();
        if( doList.front().id == E_SERVICE_ID::DOCKING) SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MAP_DONE_MOVE);
        else                                            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
    }
    ROBOT_CONTROL.reportSaveMapInfo(completedByForce);
    ServiceData.robotMap.robotTrajectory.clearExploredTrajectory();
    
    
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::completedStepExecuting()
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

E_SERVICE_STATUS_STEP CServiceExplorer::completedStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
    return ret;
}

E_SERVICE_STATUS_STEP CServiceExplorer::completedStepTerminaition()
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    return ret;
}


