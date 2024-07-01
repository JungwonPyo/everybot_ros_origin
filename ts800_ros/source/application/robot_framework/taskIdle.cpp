#include "taskIdle.h"

#include "utils.h"
#include "eblog.h"
#include "waveFrontier.h"  
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "rosPublisher.h"
#include "kinematics.h"
#include "debugCtr.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/



/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskIdle::CTaskIdle()
{
    CStopWatch __debug_sw;

    runRvizGoalStep = 0;
    runRvizGoal = 0;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskIdle::~CTaskIdle()
{
    CStopWatch __debug_sw;

    
    setidleState(IDLE_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskIdle::setidleState(IDLE_STATE set)
{
    if (set != idleState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[IDLE_STATE] : "<< enumToString(idleState)<<" --> "<< enumToString(set) );
    }
    idleState = set;
}

/**
 * @brief explorer proc
 * jhnoh, 23.01.16
 * @param robotPose     로봇 좌표 
 * @return tExplorerInfo 
 * 
 * @note  연산시간: 2ms ~ 11.0ms 
 * @date  2023-08-28
 */
bool CTaskIdle::taskIdleRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
  
    
    
    // if ( DEBUG_CTR.rvizGoalPose.isUpdate() )
    // {
    //     runRvizGoal = true;
    //     rvizGoal = DEBUG_CTR.rvizGoalPose.get();
    // }

    // if (runRvizGoal)
    // {
    //     goToRvizGoal();
    // }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskIdle::goToRvizGoal()
{
    tPose robotPose = ServiceData.localiz.getPose();
   

    if (runRvizGoalStep == 0)
    {
        PATH_PLANNER->startMapUpdate();
        runRvizGoalStep++;
    }
    else if (runRvizGoalStep == 1)
    {        
        runRvizGoalStep++;
    }
    else if (runRvizGoalStep == 2)
    {        
        {
            runRvizGoalStep++;
        }
    }    
    else if(runRvizGoalStep == 3)
    {
        runRvizGoalStep = 0;
        runRvizGoal = false;
    }
}