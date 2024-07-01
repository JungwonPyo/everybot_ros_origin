#include "taskMoveRoom.h"

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

/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskMoveRoom::CTaskMoveRoom()
{
    CStopWatch __debug_sw;
    
    goalMargin = 0.05;
    pathFailCnt=0;
    waitPathCnt=0;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskMoveRoom::~CTaskMoveRoom()
{
    CStopWatch __debug_sw;

    
    setState(MOVEROOM_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskMoveRoom::setState(MOVEROOM_STATE set)
{
    if (set != state)
    {
        preState = state; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, CYN, "[MOVEROOM_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

MOVEROOM_STATE CTaskMoveRoom::getState()
{    
    return state;
}

void CTaskMoveRoom::setGoal(tPoint set)
{
    goal = set;
    ceblog(LOG_LV_NECESSARY, CYN, "goal  x : " <<goal.x<<" , y : "<<goal.y);
}

MOVEROOM_STATE CTaskMoveRoom::findPath(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::FIND_PATH;
    curPath.clear();
    taskPathPlan.taskStart(goal);
    ret = MOVEROOM_STATE::WAITING_PATH;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEROOM_STATE CTaskMoveRoom::waittingPath(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::WAITING_PATH;
    curPath = taskPathPlan.taskRun(robotPose);
    
    if (taskPathPlan.getPathPlanState() == PATH_PALN_STATE::FAIL){
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "방이동 경로 획득 실패");
        ret = MOVEROOM_STATE::FAIL;
    }
    else{
        if(!curPath.empty())
        {
            movingStartPoint = tPoint(robotPose.x,robotPose.y);
            taskMovePath.taskStart(curPath,0.2,tProfile());
            ret = MOVEROOM_STATE::MOVE_TO_TARGET;
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


MOVEROOM_STATE CTaskMoveRoom::moveTarget(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::MOVE_TO_TARGET;
    CRobotKinematics k;

    if (MOTION.isNearTargetPose(robotPose, goal, goalMargin) )
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지와 가까워 종료");
        MOTION.startStopOnMap(tProfile(),true);
        

        ret = MOVEROOM_STATE::FINISH;
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지 하여 벽타기를 실행 하겠음.");
        MOTION.startStopOnMap(tProfile(),true);
        
        if (avoiding.getAvoidType() == E_AVOID_TYPE::CLIFF){
            tCliffActionState actionState = ServiceData.obstacle.cliff.getActionState();
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "낙하 처리 시작.");
            MOTION.startStopOnMap(tProfile(), true);
                        
            CRobotKinematics k;
            tPoint cliffpt;
            if (actionState.bLeftDetect){
                cliffpt = k.translate(robotPose, 0.35, 0.15);
                ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
                ceblog(LOG_LV_ERROR, GREEN, "cliff L update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );
                taskWallAvoidAround.taskStart(robotPose,E_WALLTRACK_DIR::LEFT);
            }
            if (actionState.bRightDetect){
                cliffpt = k.translate(robotPose, 0.35, -0.15);
                ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
                ceblog(LOG_LV_ERROR, GREEN, "cliff R update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );
                taskWallAvoidAround.taskStart(robotPose,E_WALLTRACK_DIR::RIGHT);
            }

            ret = MOVEROOM_STATE::AVOID_WALLTRACK_AROUND;
        }
        else{
            // 일반 회피
            avoidBackStart = robotPose.convertPoint();
            avoidBackTarget = k.translate(robotPose, backDist, 0.0);
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지에 의한 뒤로가기 시작");
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
            MOTION.startBackToPointOnMap(robotPose, avoidBackTarget, tProfile());
            
            ret = MOVEROOM_STATE::AVOID_BACK;
        }
    }
    else{
        if ( taskMovePath.taskRun(robotPose) ){
            ret = MOVEROOM_STATE::FINISH;
        }
        waitPathCnt = 0;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
        
    
MOVEROOM_STATE CTaskMoveRoom::avoidWalltrack(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::AVOID_WALLTRACK;

    if(taskWallAvoid.taskRun(robotPose) || taskWallAvoid.isReturnStartPoint())
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        ret = MOVEROOM_STATE::FIND_PATH;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEROOM_STATE CTaskMoveRoom::avoidWalltrackAround(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::AVOID_WALLTRACK_AROUND;

    if(avoiding.checkObstacle(robotPose,true,false)){
        drawCliff(robotPose);
    }

    taskWallAvoidAround.taskRun(robotPose);

    if(taskWallAvoidAround.isReturnStartPoint())
    {
       	MOTION.startStopOnMap(tProfile(),true);
        
        ret = MOVEROOM_STATE::FIND_PATH;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEROOM_STATE CTaskMoveRoom::avoidBack(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::AVOID_BACK;
    
    if(MOTION.isNearTargetPose(robotPose, avoidBackTarget, backMarging) || 
        MOTION.isOverTargetPoint(robotPose, avoidBackStart, avoidBackTarget))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 후진 도착");
        MOTION.startStopOnMap(tProfile(),true);
        
        ret = MOVEROOM_STATE::FIND_PATH;//MOVEROOM_STATE::FINISH;        
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEROOM_STATE CTaskMoveRoom::avoidTurn(tPose robotPose)
{
    CStopWatch __debug_sw;
    MOVEROOM_STATE ret = MOVEROOM_STATE::AVOID_TURN;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

MOVEROOM_STATE CTaskMoveRoom::finish(tPose robotPose)
{
    MOVEROOM_STATE ret = MOVEROOM_STATE::FINISH;
    
    

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
bool CTaskMoveRoom::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (state)
    {
    case MOVEROOM_STATE::NONE :
        break;
     case MOVEROOM_STATE::FIND_PATH :
        setState(findPath(robotPose));
        break;
    case MOVEROOM_STATE::WAITING_PATH :
        setState(waittingPath(robotPose));
        break;
    case MOVEROOM_STATE::AVOID_WALLTRACK :
        setState(avoidWalltrack(robotPose));
        break;    
    case MOVEROOM_STATE::AVOID_WALLTRACK_AROUND :
        setState(avoidWalltrackAround(robotPose));
        break;    
    case MOVEROOM_STATE::AVOID_BACK :
        setState(avoidBack(robotPose));
        break;    
    case MOVEROOM_STATE::AVOID_TURN :
        setState(avoidTurn(robotPose));
        break;
    case MOVEROOM_STATE::MOVE_TO_TARGET :
        setState(moveTarget(robotPose));
        break;     
    case MOVEROOM_STATE::FAIL :        
        ret = true;
        break;
    case MOVEROOM_STATE::FINISH :
        setState(finish(robotPose));
        ret = true;
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskMoveRoom::taskStart(tPoint goal)
{
    pathFailCnt = 0;
    setGoal(goal);
    setState(MOVEROOM_STATE::FIND_PATH);
}



/**
 * @brief 벽타기 방향 결정
 * 
 * @param robotPose 
 */
void CTaskMoveRoom::computeWalltrackDir(tPose robotPose, tPoint target)
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


unsigned int CTaskMoveRoom::getPathFailCount(){
    return pathFailCnt;
}

bool CTaskMoveRoom::updatePath(tPose robotPose)
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


void CTaskMoveRoom::drawCliff(tPose robotPose){
    if (avoiding.getAvoidType() == E_AVOID_TYPE::CLIFF){
        tCliffActionState actionState = ServiceData.obstacle.cliff.getActionState();            
        CRobotKinematics k;
        tPoint cliffpt;
        if (actionState.bLeftDetect){
            cliffpt = k.translate(robotPose, 0.35, 0.15);
            ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
            ceblog(LOG_LV_ERROR, GREEN, "cliff L update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );
            
        }
        if (actionState.bRightDetect){
            cliffpt = k.translate(robotPose, 0.35, -0.15);
            ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
            ceblog(LOG_LV_ERROR, GREEN, "cliff R update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );                
        }
    }
}