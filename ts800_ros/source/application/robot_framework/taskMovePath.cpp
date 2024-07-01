#include "taskMovePath.h"

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
CTaskMovePath::CTaskMovePath()
{
    CStopWatch __debug_sw;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskMovePath::~CTaskMovePath()
{
    CStopWatch __debug_sw;

    
    setState(MOVEPATH_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskMovePath::setState(MOVEPATH_STATE set)
{
    if (set != state)
    {
        preState = state; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, CYN, "[MOVEPATH_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

tPoint CTaskMovePath::getCurrentTarget()
{
    return currentTarget;
}

std::list<tPoint> CTaskMovePath::getPath()
{
    // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로를 확인합니다. 경로 갯수 : " << currentPath.size());
    // for(tPoint path : currentPath)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로 확인 : " << path.x << " , " << path.y);
    // }
    return currentPath;
}

MOVEPATH_STATE CTaskMovePath::startMoving(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEPATH_STATE ret = MOVEPATH_STATE::START_MOVING;
    
    if( currentPath.empty() )
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로의 모든 점을 이동했어요.");
        eblog(LOG_LV_NECESSARY, "robotPose : "
        <<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);

        MOTION.startStopOnMap(profile,false);
        ret = MOVEPATH_STATE::FINISH;        
    }
    else
    {   
        currentTarget = currentPath.front();
        startPoint = tPoint(robotPose.x,robotPose.y);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로이동을 시작합니다!.robotPose ("<<BOLDYELLOW<<robotPose.x<<", "<<robotPose.y<<BOLDBLACK<<") ");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "첫번째~타겟!! ("<<BOLDYELLOW<<currentTarget.x<<", "<<currentTarget.y<<BOLDBLACK<<") 좌표로 이동할거에요.");
        
        if(currentPath.size() >= 2) bUseMargin = false;
        else                        bUseMargin = true;

        if(fabs(utils::math::getTurnRadAngle(currentTarget,robotPose)) >= DEG2RAD(45)){
            bUseAngularPriorMotion = true;
            MOTION.startLinearAngularPriorToPointOnMap(robotPose, currentTarget, profile);
        }
        else{
            bUseAngularPriorMotion = false;
            MOTION.startLinearToPointOnMap(robotPose, currentTarget, profile);
        }
        
        DEBUG_PUB.publishCurrentPathPlan(currentPath, robotPose);
        startTime = SYSTEM_TOOL.getSystemTime();
        ret = MOVEPATH_STATE::MOVE_TO_TARGET;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEPATH_STATE CTaskMovePath::getTarget(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEPATH_STATE ret = MOVEPATH_STATE::GET_TARGET;
    
    if( currentPath.empty() )
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로의 모든 점을 이동했어요.");
        eblog(LOG_LV_NECESSARY, "robotPose : "
            <<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);

        MOTION.startStopOnMap(profile,false);
        ret = MOVEPATH_STATE::FINISH;        
    }
    else
    { 
        tProfile profile;
        startPoint = tPoint(robotPose.x,robotPose.y);       
        currentTarget = currentPath.front();
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "robotPose ("<<BOLDYELLOW<<robotPose.x<<", "<<robotPose.y<<BOLDBLACK<<") ");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "타겟 ("<<BOLDYELLOW<<currentTarget.x<<", "<<currentTarget.y<<BOLDBLACK<<") 좌표로 이동할거에요.");
        
        if(currentPath.size() >= 2) bUseMargin = false;
        else                        bUseMargin = true;

        bUseAngularPriorMotion = false;
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        MOTION.startLinearToPointOnMap(robotPose, currentTarget, profile);//MOTION.startLinearAngularPriorToPointOnMap(robotPose, currentTarget, profile);
        DEBUG_PUB.publishCurrentPathPlan(currentPath, robotPose);
        startTime = SYSTEM_TOOL.getSystemTime();
        ret = MOVEPATH_STATE::MOVE_TO_TARGET;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}



MOVEPATH_STATE CTaskMovePath::moveTarget(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEPATH_STATE ret = MOVEPATH_STATE::MOVE_TO_TARGET;
        
    bool bUpdateTarget = false;
    double runTime = SYSTEM_TOOL.getSystemTime()-startTime;
    // if(bUseAngularPriorMotion)  
    // else                        

    if(updatePath(robotPose))
    {
        currentPath = newPath;
        DEBUG_PUB.publishCurrentPathPlan(currentPath, robotPose);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로 업데이트 완료. ");
        ret = MOVEPATH_STATE::GET_TARGET;
    }
    if(!MOTION.isRunning())
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "제어기가 멈춰서 종료");
        bUpdateTarget = true;
    }
    else if (MOTION.isOverTargetPoint(robotPose,startPoint,currentTarget)){
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "목표지점을 넘어서 종료");
        bUpdateTarget = true;
    }
    else if(bUseMargin){
        if ( MOTION.isNearTargetPose(robotPose, currentTarget, arriveMargin) ){
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "목표지점 반경 : " << arriveMargin << "도착");
            bUpdateTarget = true;
        } 
    }
    else if ( MOTION.isNearTargetPose(robotPose, currentTarget,0.15)){
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "목표지점 0.15 반경 도착");
        bUpdateTarget = true;

    }else if(runTime >= CONFIG.movePathTimeLimit){
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "목표점에 < " << CONFIG.movePathTimeLimit <<  " > 초 동안 도착하지 못했습니다. 이번 경로는 포기할게요 runTime : " << runTime << " start : " << startTime << " cur :" << SYSTEM_TOOL.getSystemTime());
            ret = MOVEPATH_STATE::FINISH;
    }

    if ( bUpdateTarget ){   
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "타겟으로 이동 완료 robotPose : " << robotPose.x << " , " << robotPose.y << " 목표점 : " << currentTarget.x << " , " << currentTarget.y);
#if 1   // 경로 디버깅
        std::list<tPoint> debugPath = currentPath;
        tPose tpose(robotPose);
        tPoint pt (tpose.x, tpose.y);
        debugPath.push_front(pt);
        DEBUG_PUB.publishRobotPath(debugPath);
#endif
        currentPath.pop_front();        
        ret = MOVEPATH_STATE::GET_TARGET;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


bool CTaskMovePath::finish(tPose robotPose)
{
    bool ret = false;
    
    
    if(!MOTION.isRunning()) ret = true;

    return ret;
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
bool CTaskMovePath::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (state)
    {
    case MOVEPATH_STATE::NONE :
        break;
    case MOVEPATH_STATE::START_MOVING :
        setState(startMoving(robotPose));
        break;    
    case MOVEPATH_STATE::GET_TARGET :
        setState(getTarget(robotPose));
        break;    
    case MOVEPATH_STATE::MOVE_TO_TARGET :
        setState(moveTarget(robotPose));
        break;     
    case MOVEPATH_STATE::FINISH :
        if(finish(robotPose)){
            setState(MOVEPATH_STATE::NONE);
            ret = true;
        }  
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskMovePath::taskStart(std::list<tPoint> &path, double _arriveMargin,tProfile pf)
{
    currentPath.clear();
    currentPath = path;
    arriveMargin = _arriveMargin;
    profile = pf;
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로 이동 시작!! 경로 갯수 : " << currentPath.size() << " 도착 마진 거리 : " << arriveMargin);
    for(tPoint path : currentPath)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "경로 확인 : " << path.x << " , " << path.y);
    }
    setState(MOVEPATH_STATE::START_MOVING);
}

bool CTaskMovePath::updatePath(tPose robotPose)
{
    bool ret = false;

    if(!currentPath.empty())
    {
        if(bSearchPath){
            newPath.clear();
            newPath = taskPathPlan.taskRun(robotPose);
            if(!newPath.empty())
            {
                bSearchPath = false;
                ret = true;
            }
        }
        else if(robotPose.distance(startPoint) > 0.3){
            taskPathPlan.taskStart(currentPath.back());
            bSearchPath = true;
        }
    }
    
    return ret;
}