#include "taskPathPlan.h"
#include "utils.h"
#include "eblog.h"
#include "waveFrontier.h"  
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "subTask.h"
#include "rosPublisher.h"

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
CTaskPathPlan::CTaskPathPlan()
{
    CStopWatch __debug_sw;
    
    setPathPlanState(PATH_PALN_STATE::NONE);    

    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskPathPlan::~CTaskPathPlan()
{
    CStopWatch __debug_sw;

    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskPathPlan::taskStart(tPoint _goal)
{
    CStopWatch __debug_sw;
    goal = _goal;
    pathPlan.clear();
    setPathPlanState(PATH_PALN_STATE::START);
    eblog(LOG_LV_NECESSARY,  "목적지 : "<< goal.x << " ," << goal.y << "에 대한 경로계획을 시작합니다.");
    TIME_CHECK_END(__debug_sw.getTime());
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
std::list<tPoint> CTaskPathPlan::taskRun(tPose robotPose)
{
    CStopWatch __debug_sw;

    procPathSearch(robotPose);
    TIME_CHECK_END(__debug_sw.getTime());
    return pathPlan;
}

void CTaskPathPlan::setPathPlanState(PATH_PALN_STATE set)
{
    if(state != set){
        // ceblog(LOG_LV_NECESSARY, GREEN, "PATH_PALN_STATE : "<< enumToString(state) << " ----> " << enumToString(set));
    }
    state = set;
}

PATH_PALN_STATE CTaskPathPlan::getPathPlanState()
{
    return state;
}

bool CTaskPathPlan::procPathSearch(tPose robotPose)
{
    bool ret = false;

    switch (getPathPlanState())
    {
    case PATH_PALN_STATE::NONE:
        break;
    case PATH_PALN_STATE::START:
       setPathPlanState(startPathSearching(robotPose));
        break;
    case PATH_PALN_STATE::RUN:
        setPathPlanState(runPathSearching(robotPose));
        break;
    case PATH_PALN_STATE::COMPLETE:
        setPathPlanState(completePathSearching(robotPose));
        ret = true;
        break;
    case PATH_PALN_STATE::FAIL:
        setPathPlanState(failPathSearching(robotPose));
        ret = true;
        break;                      
    default:
        break;
    }

    return ret;
}

PATH_PALN_STATE CTaskPathPlan::startPathSearching(tPose robotPose)
{
    CStopWatch __debug_sw;
    PATH_PALN_STATE ret = PATH_PALN_STATE::START;

    if (PATH_PLANNER->dofindPath(robotPose, goal))
    {
        pathplanStartTime = SYSTEM_TOOL.getSystemTime();
        ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "목적 좌표 : " << goal.x << " , " << goal.y << " 에 대한 경로를 찾아볼게요!!");
        ret = PATH_PALN_STATE::RUN;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY,RED, "누가 경로 검색을 돌려놓고 또 검색을 하고 있습니까?! 기존 검색을 멈추고 검색하세요!!");
    }
    return ret;
}

PATH_PALN_STATE CTaskPathPlan::runPathSearching(tPose robotPose)
{
    CStopWatch __debug_sw;
    PATH_PALN_STATE ret = PATH_PALN_STATE::RUN;
    double runTime = SYSTEM_TOOL.getSystemTime()-pathplanStartTime;
    if ( PATH_PLANNER->isRunFindPath() == false )
    {
        if (PATH_PLANNER->isFindPath())
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로 계획을 성공했습니다!!~~ runTime : " << runTime);
            ret = PATH_PALN_STATE::COMPLETE;    
        }
        else
        {   
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로 계획에 실패 했습니다.. runTime : " << runTime);
            ret = PATH_PALN_STATE::FAIL;
        }
    }
    // else
    // {
    //     ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "경로를 생성하고 있습니다~~ 잠시만 기달려 주세요. 목적 좌표 : " << goal.x << " , " << goal.y << " runTime : " << runTime);
    // }

    return ret;
}

PATH_PALN_STATE CTaskPathPlan::completePathSearching(tPose robotPose)
{
    CStopWatch __debug_sw;
    PATH_PALN_STATE ret = PATH_PALN_STATE::COMPLETE;

    pathPlan = PATH_PLANNER->getPath();
    ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "목적 좌표 : " << goal.x << " , " << goal.y << " 에 대한 경로는 : " << pathPlan.size() << " 개 입니다.");
    
    for(tPoint point : pathPlan)
    {
        ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "경로 : " << point.x << " , " << point.y);
    }
    // ret = PATH_PALN_STATE::NONE;
    return ret;
}

PATH_PALN_STATE CTaskPathPlan::failPathSearching(tPose robotPose)
{
    CStopWatch __debug_sw;
    PATH_PALN_STATE ret = PATH_PALN_STATE::FAIL;
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로계획을 실패하였습니다. 목적지를 줄테니 알아서 가보세요...");
    #if 1 // 경로 실패의 경우 경로를 주지 말아야 하나?...
    pathPlan.push_back(goal);
    #endif
    // ret = PATH_PALN_STATE::NONE;
    return ret;
}