#include "taskMoveCleanLine.h"

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

const double backDist = -0.025;
const double backMarging = 0.01;
const double goalMargin = 0.05;

/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskMoveCleanLine::CTaskMoveCleanLine()
{
    CStopWatch __debug_sw;
    
    
    pathFailCnt=0;
    waitPathCnt = 0;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskMoveCleanLine::~CTaskMoveCleanLine()
{
    CStopWatch __debug_sw;

    
    setState(MOVECLEANLINE_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskMoveCleanLine::setState(MOVECLEANLINE_STATE set)
{
    if (set != state){
        preState = state; // 이전 상태 저장.
        // ceblog(LOG_LV_NECESSARY, CYN, "[MOVECLEANLINE_STATE] : "
        //     << enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskMoveCleanLine::setGoal(tPoint set)
{
    goal = set;
    // ceblog(LOG_LV_NECESSARY, CYN, "move line goal  x : " <<goal.x<<" , y : "<<goal.y);
}

MOVECLEANLINE_STATE CTaskMoveCleanLine::findPath(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::FIND_PATH;
    curPath.clear();
    taskPathPlan.taskStart(goal);
    ret = MOVECLEANLINE_STATE::WAITING_PATH;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVECLEANLINE_STATE CTaskMoveCleanLine::waittingPath(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::WAITING_PATH;
    curPath = taskPathPlan.taskRun(robotPose);
    if(!curPath.empty())
    {
        movingStartPoint = tPoint(robotPose.x,robotPose.y);
        taskMovePath.taskStart(curPath, CONFIG.sidelineGoalMargin, tProfile());
        ret = MOVECLEANLINE_STATE::MOVE_TO_TARGET;
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


MOVECLEANLINE_STATE CTaskMoveCleanLine::moveTarget(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::MOVE_TO_TARGET;
    CRobotKinematics k;

    bool isCrossOver = utils::math::isRobotCrossLine(robotPose, movingStartPoint, goal);
    bool isNear = MOTION.isNearTargetPose(robotPose, goal, goalMargin);
    if (isCrossOver || isNear)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지와 가까워 종료");
        MOTION.startStopOnMap(tProfile(),true);
        

        ret = MOVECLEANLINE_STATE::FINISH;
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지 하여 벽타기를 실행 하겠음.");
        MOTION.startStopOnMap(tProfile(),true);
        
        // if(updatePath(robotPose))
        // {
        //     if(utils::math::getTurnRadAngle(newPath.front(),robotPose) > 0) taskWallAvoid.taskStart(robotPose,newPath,goal,E_WALLTRACK_DIR::RIGHT);
        //     else                                                            taskWallAvoid.taskStart(robotPose,newPath,goal,E_WALLTRACK_DIR::LEFT);
        //     ret = MOVECLEANLINE_STATE::AVOID_WALLTRACK;
        // }
        
        // // 무한정으로 updatePath() 를 기다리고 있어서 카운트를 달아 놓음.
        // if (waitPathCnt++ >= 200){
            // 일반 회피
            avoidBackStart = robotPose.convertPoint();
            avoidBackTarget = k.translate(robotPose, backDist, 0.0);
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지에 의한 뒤로가기 시작");
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
            MOTION.startBackToPointOnMap(robotPose, avoidBackTarget, tProfile());
            
            ret = MOVECLEANLINE_STATE::AVOID_BACK;
        //}
    }
    else{
        if ( taskMovePath.taskRun(robotPose) ){
            ret = MOVECLEANLINE_STATE::FINISH;
        }
        waitPathCnt = 0;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
        
    
MOVECLEANLINE_STATE CTaskMoveCleanLine::avoidWalltrack(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::AVOID_WALLTRACK;

    if(taskWallAvoid.taskRun(robotPose) || taskWallAvoid.isReturnStartPoint())
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        ret = MOVECLEANLINE_STATE::FIND_PATH;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVECLEANLINE_STATE CTaskMoveCleanLine::avoidBack(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::AVOID_BACK;
    
    if(MOTION.isNearTargetPose(robotPose, avoidBackTarget, backMarging) || 
        MOTION.isOverTargetPoint(robotPose, avoidBackStart, avoidBackTarget))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 후진 도착");
        MOTION.startStopOnMap(tProfile(),true);
        
        ret = MOVECLEANLINE_STATE::FIND_PATH;//MOVECLEANLINE_STATE::FINISH;        
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVECLEANLINE_STATE CTaskMoveCleanLine::avoidTurn(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::AVOID_TURN;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVECLEANLINE_STATE CTaskMoveCleanLine::finish(tPose robotPose)
{
    MOVECLEANLINE_STATE ret = MOVECLEANLINE_STATE::FINISH;
    
    

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
bool CTaskMoveCleanLine::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (state)
    {
    case MOVECLEANLINE_STATE::NONE :
        break;
     case MOVECLEANLINE_STATE::FIND_PATH :
        setState(findPath(robotPose));
        break;
    case MOVECLEANLINE_STATE::WAITING_PATH :
        setState(waittingPath(robotPose));
        break;
    case MOVECLEANLINE_STATE::AVOID_WALLTRACK :
        setState(avoidWalltrack(robotPose));
        break;    
    case MOVECLEANLINE_STATE::AVOID_BACK :
        setState(avoidBack(robotPose));
        break;    
    case MOVECLEANLINE_STATE::AVOID_TURN :
        setState(avoidTurn(robotPose));
        break;
    case MOVECLEANLINE_STATE::MOVE_TO_TARGET :
        setState(moveTarget(robotPose));
        break;     
    case MOVECLEANLINE_STATE::FINISH :
        setState(finish(robotPose));
        ret = true;
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskMoveCleanLine::taskStart(tPoint goal)
{
    pathFailCnt = 0;
    setGoal(goal);
    setState(MOVECLEANLINE_STATE::FIND_PATH);
}



/**
 * @brief 벽타기 방향 결정
 * 
 * @param robotPose 
 */
void CTaskMoveCleanLine::computeWalltrackDir(tPose robotPose, tPoint target)
{
    RSF_OBSTACLE_MASK mask = avoiding.getAvoidMask();
    if(mask.value & 0x0F)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 장애물 감지!! 오른쪽 벽타기로 결정");
        wallDir = E_WALLTRACK_DIR::RIGHT;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 장애물 감지!! 왼쪽 벽타기로 결정");
        wallDir = E_WALLTRACK_DIR::LEFT;
    }
    // CRobotKinematics k;
    // bool bRet = k.computeRotateDir(robotPose, target);
    // //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 벽타기로 결정"); // 좌측 벽타기 제자리 멤돔 문제로 일단 오른쪽 벽타기만 실행 한다.
    // //wallDir = E_WALLTRACK_DIR::RIGHT;
    // if (bRet) {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 벽타기로 결정");
    //     wallDir = E_WALLTRACK_DIR::RIGHT;        
    // } else {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 벽타기로 결정");
    //     wallDir = E_WALLTRACK_DIR::LEFT;
    // }
}


unsigned int CTaskMoveCleanLine::getPathFailCount(){
    return pathFailCnt;
}

bool CTaskMoveCleanLine::updatePath(tPose robotPose)
{
    bool ret = false;

    if(!curPath.empty())
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
        else //if(robotPose.distance(movingStartPoint) >= 1) 
        {
            taskPathPlan.taskStart(goal);
            bSearchPath = true;
        }
    }
    
    return ret;
}