#include "taskExplorer.h"
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
CTaskExplorer::CTaskExplorer()
{
    CStopWatch __debug_sw;
    
    invalidCnt = 0;   

    // initial value
    setExplorerState(E_EXPLORER_STATE::NONE);    
    bDebugStatePrint = true;
    bTurnReverse = false;

    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskExplorer::~CTaskExplorer()
{
    CStopWatch __debug_sw;

    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskExplorer::taskStart()
{
    CStopWatch __debug_sw;

    SUB_TASK.waveFrontier.startUpdate();
    PATH_PLANNER->startMapUpdate();
    exploreStartTime = SYSTEM_TOOL.getSystemTime();
    profile = tProfile();
    profile.explorer = true;
    profile.desAngVel = DEG2RAD(25);
    // initial value
    setRecognizeState(E_RECOGNIZE_STATE::RECOGNIZE_START);
    setExplorerState(E_EXPLORER_STATE::RUN_RECOGNIZE);
    newGoalList.clear();
    invalidCnt = 0;    
    bDebugStatePrint = true;
    recognizeCnt = 0;
    reconsiderCnt = 0;
    pathLockCnt = 0;
    bCheckFistWFP = false;
    bStartSearhPath = false;
    ServiceData.robotMap.simplifyMap.cliffPoint.clearCliff();
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
bool CTaskExplorer::taskRun(tPose robotPose)
{
    CStopWatch __debug_sw;
    bool ret = false;

    if(monitorExplorer(robotPose)) // 탐색 종료 조건 및 탐색점 업데이트를 모니터
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "탐색점이 없어 탐색을 종료합니다.");
        MOTION.startStopOnMap(tProfile(),false);
        setExplorerState(E_EXPLORER_STATE::END);
    }

    if (checkPathLock()){
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "탐색 경로가 나오지 않아 탐색을 종료합니다.");
        MOTION.startStopOnMap(tProfile(),false);
        setExplorerState(E_EXPLORER_STATE::END);
    }
    
    // explorer procedure
    switch (getExplorerState())
    {
    case E_EXPLORER_STATE::NONE:
        setExplorerState(procNone(robotPose));
        break;
    case E_EXPLORER_STATE::RUN_RECOGNIZE:
        setExplorerState(procRecognize(robotPose));
        break;      
    case E_EXPLORER_STATE::MOVE_TO_TARGET:
        setExplorerState(procMoveTarget(robotPose));
        break;      
    case E_EXPLORER_STATE::AVOID_WALLTRACK :
        setExplorerState(procAvoidWalltrack(robotPose));
        break;
    case E_EXPLORER_STATE::AVOID_WALLTRACK_AROUND :
        setExplorerState(procAvoidWalltrackAround(robotPose));
        break;        
    case E_EXPLORER_STATE::AVOID_CLIFF :
        setExplorerState(procAvoidCliff(robotPose));
        break;        
    case E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START:
        setExplorerState(procMapReconsiderationMoveStart(robotPose));
        break;
    case E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_WAIT:
        setExplorerState(procMapReconsiderationMoveWait(robotPose));
        break;
    case E_EXPLORER_STATE::AWAY_WALL_MOVE_START:
        setExplorerState(procAwayWallMoveStart(robotPose));
        break;
    case E_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT:
        setExplorerState(procAwayWallMoveWait(robotPose));
        break;
    case E_EXPLORER_STATE::STOP_WAIT:
        setExplorerState(procStopWait(robotPose));
        break;        
    case E_EXPLORER_STATE::END:
        ret = explorerEnd(robotPose);
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

bool CTaskExplorer::monitorExplorer(tPose robotPose)
{
    bool ret =  false;
    // __debug_state_print();

    if(isMapReady())
    {
        if( checkExploerFinish()) {
            ret = true;
        }
    }

    return ret;
}

/**
 * @brief explorer 전 기본 점검
 * jhnoh, 23.02.01
 * @return E_EXPLORER_STATE 
 */
E_EXPLORER_STATE CTaskExplorer::procNone(tPose robotPose)
{
    CStopWatch __debug_sw;

    E_EXPLORER_STATE ret = E_EXPLORER_STATE::NONE;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_EXPLORER_STATE CTaskExplorer::procRecognize(tPose robotPose)
{
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::RUN_RECOGNIZE;

    if(bStartSearhPath)
    {
        curPath = taskPathPlan.taskRun(robotPose);
        if(!curPath.empty())
        {
            bStartSearhPath = false;
            if(taskPathPlan.getPathPlanState() == PATH_PALN_STATE::FAIL){
                pathLockCnt++;                
            }
            else{
                curGoal = newGoal;
                taskMovePath.taskStart(curPath,0.5,profile);
                pathLockCnt = 0;
                ret = E_EXPLORER_STATE::MOVE_TO_TARGET;
            }                                                            
        }
    }
    else if(checkUpdateWaveFrontier(robotPose))
    {
        bStartSearhPath = true;
        taskPathPlan.taskStart(newGoal);
    }

    switch (recognizeState)
    {
    case E_RECOGNIZE_STATE::NONE:
        break;
    case E_RECOGNIZE_STATE::RECOGNIZE_START:
        setRecognizeState(procRecognizeStart(robotPose));
        break;
    case E_RECOGNIZE_STATE::RECOGNIZE_MOTION:
        setRecognizeState(procRecognizeMotion(robotPose));
        break;
    case E_RECOGNIZE_STATE::RECOGNIZE_BACK:
        setRecognizeState(procRecognizeMotionBack(robotPose));
        break;
    case E_RECOGNIZE_STATE::RECOGNIZE_TURN:
        setRecognizeState(procRecognizeMotionTurn(robotPose));
        break;
    case E_RECOGNIZE_STATE::COMPLETE:
        setRecognizeState(procRecognizeComplete(robotPose));
        ret = E_EXPLORER_STATE::END;
        break;            
    default:
        break;
    }

    return ret;
}

E_RECOGNIZE_STATE CTaskExplorer::procRecognizeStart(tPose robotPose)
{
    E_RECOGNIZE_STATE ret = E_RECOGNIZE_STATE::RECOGNIZE_START;
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_SKIP_MOTION_FILTER);  
    taskLocalMotion.taskStart(robotPose);
    ret = E_RECOGNIZE_STATE::RECOGNIZE_MOTION;


    return ret;
}

E_RECOGNIZE_STATE CTaskExplorer::procRecognizeMotion(tPose robotPose)
{
    CStopWatch __debug_sw;

    E_RECOGNIZE_STATE ret = E_RECOGNIZE_STATE::RECOGNIZE_MOTION;

    if(avoiding.checkObstacle(robotPose,true,false)){
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지에 의한 뒤로가기 시작");
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_STOP_LOTATE_INSERT_MAP);
        MOTION.startStopOnMap( profile, true);
        

        CRobotKinematics k;
        targetStart.x = robotPose.x;
        targetStart.y = robotPose.y;
        targetEnd = k.translate(robotPose, -0.15, 0.0);        
        MOTION.startBackToPointOnMap(robotPose, targetEnd, profile);
        
        ret = E_RECOGNIZE_STATE::RECOGNIZE_BACK;
    }
    else{
        if(taskLocalMotion.taskRun(robotPose))
        {
            bDebugStatePrint=true;
            ret = E_RECOGNIZE_STATE::COMPLETE;
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_RECOGNIZE_STATE CTaskExplorer::procRecognizeMotionBack(tPose robotPose)
{
    E_RECOGNIZE_STATE ret = E_RECOGNIZE_STATE::RECOGNIZE_BACK;

    CRobotKinematics k;
    
    if(MOTION.isNearTargetPose(robotPose, targetEnd, 0.1) || 
            MOTION.isOverTargetPoint(robotPose, targetStart, targetEnd))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 후진 도착");        
        targetRad = k.rotation(robotPose, DEG2RAD(50));
        MOTION.startRotation(robotPose, targetRad, profile, E_ROTATE_DIR::CCW);
        ret = E_RECOGNIZE_STATE::RECOGNIZE_TURN;
    }
    else{
             
    }

    return ret;
}

E_RECOGNIZE_STATE CTaskExplorer::procRecognizeMotionTurn(tPose robotPose)
{
    E_RECOGNIZE_STATE ret = E_RECOGNIZE_STATE::RECOGNIZE_TURN;

    if(MOTION.isNearTargetRad(robotPose, targetRad, DEG2RAD(10)))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 회진 도착");        
        ret = E_RECOGNIZE_STATE::RECOGNIZE_START;
    }

    

    return ret;
}

E_RECOGNIZE_STATE CTaskExplorer::procRecognizeComplete(tPose robotPose)
{
    E_RECOGNIZE_STATE ret = E_RECOGNIZE_STATE::COMPLETE;
    
    ceblog(LOG_LV_NECESSARY, YELLOW, " localization finish");
    
    ret = E_RECOGNIZE_STATE::NONE;
    return ret;
}


/**
 * @brief 낙하 발생했을때 처리
 * 
 * @param robotPose 
 * @return E_EXPLORER_STATE 
 */
E_EXPLORER_STATE CTaskExplorer::procAvoidCliff(tPose robotPose){
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::AVOID_CLIFF;
    tCliffActionState actionState = ServiceData.obstacle.cliff.getActionState();
    
    if(actionState.bRobotStop == false && cliffAction.bRun == false){
        ceblog(LOG_LV_NECESSARY, YELLOW, "MCU 회피 정지 완료. 뒤로가기 시작");
        cliffAction.bRun = true;
        cliffAction.backStart = true;        
    }

    if (cliffAction.bRun){   
        if (actionState.bRobotStop){
            ceblog(LOG_LV_NECESSARY, YELLOW, "뒤로 보낼려는데 또 낙하");
            MOTION.startStopOnMap( profile, true);
            
            cliffAction.clear();
        }
        if (cliffAction.backStart){
            ceblog(LOG_LV_NECESSARY, YELLOW, "뒤로 조금 보낼께요.");
            CRobotKinematics k;
            cliffAction.target = k.translate(robotPose, -0.05, 0.0);
            cliffAction.start = robotPose.convertPoint();
            MOTION.startBackToPointOnMap(robotPose, cliffAction.target, tProfile());
            cliffAction.backStart = false;
            cliffAction.backWait = true;
        }
        else if (cliffAction.backWait){
            bool bNear = MOTION.isNearTargetPose(robotPose, cliffAction.target, 0.02);
            bool isOverCross = utils::math::isRobotCrossLine(robotPose, cliffAction.start, cliffAction.target);
            if (bNear || isOverCross){
                ceblog(LOG_LV_NECESSARY, YELLOW, "뒤로 조금 도착.");
                MOTION.startStopOnMap( profile, true);
                cliffAction.backWait = false;
                cliffAction.backEnd = true;
            }
        }
        else if (cliffAction.backEnd){
            if(bStartSearhPath){
                curPath = taskPathPlan.taskRun(robotPose);
                if(!curPath.empty()){
                    bStartSearhPath = false;
                    if(taskPathPlan.getPathPlanState() == PATH_PALN_STATE::FAIL){
                        pathLockCnt++;
                        cliffAction.clear();
                        ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
                    }    
                    else{
                        curGoal = newGoal;
                        taskMovePath.taskStart(curPath,0.5,profile);
                        cliffAction.clear();
                        pathLockCnt = 0;
                        ret = E_EXPLORER_STATE::MOVE_TO_TARGET;                    
                    }                                                            
                }
            }
            else{
                bStartSearhPath = true;
                taskPathPlan.taskStart(curGoal);
            }        
        }
        else{}
    }
    
    

    return ret;
}

E_EXPLORER_STATE CTaskExplorer::procAvoidWalltrack(tPose robotPose)
{
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::AVOID_WALLTRACK;

    if(taskWallAvoid.taskRun(robotPose))
    {
        ret = E_EXPLORER_STATE::AWAY_WALL_MOVE_START;
    }        
    else if(taskWallAvoid.isReturnStartPoint())
    {
        MOTION.startStopOnMap(profile,false);
        tempState = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
        ret = E_EXPLORER_STATE::STOP_WAIT;
    } 

    // if (diff > 1.0)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료 :" << diff);
    //     tempState = E_EXPLORER_STATE::AWAY_WALL_MOVE_START;
    //     ret = E_EXPLORER_STATE::STOP_WAIT;
    //     MOTION.startStopOnMap(tProfile(),false);
    // }

    // if(fabs(wallAngle) >= 360)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 중 제자리멤돔!! 누적 각도 : " << wallAngle);
    //     tempState = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
    //     ret = E_EXPLORER_STATE::STOP_WAIT;
    //     MOTION.startStopOnMap(tProfile(),false);
    // }

    // if(runTime >= CONFIG.avoidWalltrackEndCheckTime && (result < 0.15))
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료 !! 목적 라인 도착 : " << result );
    //     ret = E_EXPLORER_STATE::AWAY_WALL_MOVE_START;
    // }

    return ret;
}


E_EXPLORER_STATE CTaskExplorer::procAvoidWalltrackAround(tPose robotPose)
{
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::AVOID_WALLTRACK_AROUND;

    taskWallAvoidAround.taskRun(robotPose);

    return ret;
}

/**
 * @brief 벽타기 방향 결정
 * 
 * @param robotPose 
 */
void CTaskExplorer::computeWalltrackDir(tPose robotPose, tPoint target)
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
    
    // if (bRet) {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 벽타기로 결정");
    //     wallDir = E_WALLTRACK_DIR::RIGHT;        
    // } else {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 벽타기로 결정");
    //     wallDir = E_WALLTRACK_DIR::LEFT;
    // }
}


E_EXPLORER_STATE CTaskExplorer::procMoveTarget(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::MOVE_TO_TARGET;

    if(avoiding.checkObstacle(robotPose, true, false))
    {
        // 낙하는 벼로 회피루틴을 갖는다.
        if (avoiding.getAvoidType() == E_AVOID_TYPE::CLIFF){
            tCliffActionState actionState = ServiceData.obstacle.cliff.getActionState();
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "낙하 처리 시작.");
            MOTION.startStopOnMap( profile, true);
            
            cliffAction.clear();
            CRobotKinematics k;
            tPoint cliffpt;
            if (actionState.bLeftDetect){
                cliffpt = k.translate(robotPose, 0.35, 0.15);
                ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
                ceblog(LOG_LV_ERROR, GREEN, "cliff L update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );
                // taskWallAvoidAround.taskStart(robotPose,E_WALLTRACK_DIR::LEFT);
            }
            if (actionState.bRightDetect){
                cliffpt = k.translate(robotPose, 0.35, -0.15);
                ServiceData.robotMap.simplifyMap.cliffPoint.updateCliff(cliffpt);
                ceblog(LOG_LV_ERROR, GREEN, "cliff R update x :" <<cliffpt.x<<" , y :"<<cliffpt.y );
                // taskWallAvoidAround.taskStart(robotPose,E_WALLTRACK_DIR::RIGHT);
            }

            ret = E_EXPLORER_STATE::AVOID_CLIFF;
            // ret = E_EXPLORER_STATE::AVOID_WALLTRACK_AROUND;
        }
        else{        
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지 하여 벽타기를 실행 하겠음.");
            MOTION.startStopOnMap(profile,false);

            computeWalltrackDir(robotPose, taskMovePath.getCurrentTarget());
    
            taskWallAvoid.taskStart(robotPose,curPath,curGoal,wallDir);
            tempState = E_EXPLORER_STATE::AVOID_WALLTRACK;
            ret = E_EXPLORER_STATE::STOP_WAIT;
        }
    }
    else{
        if(taskMovePath.taskRun(robotPose) )
        {
            ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
        }
        else if(bStartSearhPath)
        {
            curPath = taskPathPlan.taskRun(robotPose);
            if(!curPath.empty())
            {
                bStartSearhPath = false;
                if(taskPathPlan.getPathPlanState() == PATH_PALN_STATE::FAIL){
                    pathLockCnt++;
                    ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
                }    
                else{
                    curGoal = newGoal;
                    taskMovePath.taskStart(curPath,0.5,profile);
                    pathLockCnt=0;
                }                                                            
            }
        }
        else if(taskMovePath.getPath().size() <= 1 && MOTION.isNearTargetPose(robotPose, curGoal, 0.5))
        {
            if(checkUpdateWaveFrontier(robotPose))
            {
                bStartSearhPath = true;
                taskPathPlan.taskStart(newGoal);
            }
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


E_EXPLORER_STATE CTaskExplorer::procMapReconsiderationMoveStart(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
    
    CRobotKinematics k;

    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_SKIP_MOTION_FILTER);
    if (bTurnReverse){
        //for_test
        // 180도 회전할 경우 지도가 잔상이 심함
        // 45도 회전해도 충분함. 단. MAP_RECONSIDERATION_MOVE_WAIT 단계에서 대기
        //하는 것이 더 효과 적임.
        reconsiderAngle = k.rotation(robotPose, DEG2RAD(350));        
        MOTION.startRotation(robotPose, reconsiderAngle, profile,E_ROTATE_DIR::CCW);
        bTurnReverse = false;
    }
    else{
        //for_test
        reconsiderAngle = k.rotation(robotPose, DEG2RAD(-350));
        MOTION.startRotation(robotPose, reconsiderAngle, profile,E_ROTATE_DIR::CW);
        bTurnReverse = true;
    }

    ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_WAIT;    

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_EXPLORER_STATE CTaskExplorer::procMapReconsiderationMoveWait(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_WAIT;

    
    if (MOTION.isNearTargetRad(robotPose, reconsiderAngle, DEG2RAD(5)))
    {
        if(reconsiderCnt++ >= 2){
            ceblog(LOG_LV_NECESSARY, RED, "비정상적 종료!! 탐색점이 있는데..유효점이 없음..Reconsider 2회 회전 했는데도 탐색점이 안나와 종료!!");
            MOTION.startStopOnMap(profile,false);
            tempState = E_EXPLORER_STATE::END;
            ret = E_EXPLORER_STATE::STOP_WAIT;
        }
        else{
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "이래도 탐색점 안나와? Reconsider 회전 시도 횟수 : " << reconsiderCnt);
            ret = E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START;
        } 
    }

    if(bStartSearhPath)
    {
        curPath = taskPathPlan.taskRun(robotPose);
        if(!curPath.empty())
        {
            bStartSearhPath = false;
            if(taskPathPlan.getPathPlanState() == PATH_PALN_STATE::FAIL){
                pathLockCnt++;                
            }
            else{
                curGoal = newGoal;
                taskMovePath.taskStart(curPath,0.5,profile);
                pathLockCnt = 0;
                ret = E_EXPLORER_STATE::MOVE_TO_TARGET;
            }
        }
    }
    else if(checkUpdateWaveFrontier(robotPose))
    {
        bStartSearhPath = true;
        taskPathPlan.taskStart(newGoal);
    }
    else if (SUB_TASK.waveFrontier.isFinish())
    {
        MOTION.startStopOnMap(profile,false);
        tempState = E_EXPLORER_STATE::END;
        ret = E_EXPLORER_STATE::STOP_WAIT;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


E_EXPLORER_STATE CTaskExplorer::procAwayWallMoveStart(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::AWAY_WALL_MOVE_START;
    
    CRobotKinematics k;
    
    taskPathPlan.taskStart(curGoal);

    //장애물 맞나서 벽타기 할 때애는 지도 업데이트 하지 말자 
    //빨리 돌리고 얼른 나가자
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
    if (wallDir == E_WALLTRACK_DIR::RIGHT){
        reconsiderAngle = k.rotation(robotPose, DEG2RAD(120));        
        MOTION.startRotation(robotPose, reconsiderAngle, profile,E_ROTATE_DIR::CCW);
        bTurnReverse = false;
    }
    else{
        reconsiderAngle = k.rotation(robotPose, DEG2RAD(-120));
        MOTION.startRotation(robotPose, reconsiderAngle, profile,E_ROTATE_DIR::CW);
        bTurnReverse = true;
    }

    ret = E_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT;    

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_EXPLORER_STATE CTaskExplorer::procAwayWallMoveWait(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT;
    curPath = taskPathPlan.taskRun(robotPose);
    
    if(MOTION.isRunning())
    {
        if (MOTION.isNearTargetRad(robotPose, reconsiderAngle, DEG2RAD(5)))
        {
            MOTION.startStopOnMap(profile,false);
            tempState = E_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT;
            ret = E_EXPLORER_STATE::STOP_WAIT;   
        }
    }
    
    if(!curPath.empty())
    {
        taskMovePath.taskStart(curPath,0.5,profile);
        DEBUG_PUB.publishGlobalPathPlan(curPath, robotPose);
        ret = E_EXPLORER_STATE::MOVE_TO_TARGET;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_EXPLORER_STATE CTaskExplorer::procStopWait(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_EXPLORER_STATE ret = E_EXPLORER_STATE::STOP_WAIT;

    //청소로 넘어가기 전 디폴트로 돌리자..
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    
    if(!MOTION.isRunning())
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "정지 대기 완료!! Next State : " << enumToString(tempState));
        ret = tempState;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 탐색 종료 조건을 만족하는지 검사.
 * 
 * @return true 
 * @return false 
 */
bool CTaskExplorer::checkExploerFinish()
{
    bool bRet = false;

    // 탐색 종료 조건 확인
    if(getExplorerState() != E_EXPLORER_STATE::END)
    {
        if ( invalidCnt > 2)
        {
            invalidCnt = 0;
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 움직이며 검사했는데 탐색점 없어요.");
            bRet = true;
        }
        if (SUB_TASK.waveFrontier.isFinish())
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 탐색점이 없어 종료 해요.");
            bRet = true;
        }
    }
    
    return bRet;
}


bool CTaskExplorer::checkPathLock()
{
    bool bRet = false;

    if (pathLockCnt > 10){
        bRet = true;
        pathLockCnt = 0;
    }
    
    return bRet;
}

/**
 * 
 * @brief 탐색 종료 
 * jhnoh, 23.01.30
 */
bool CTaskExplorer::explorerEnd(tPose robotPose)
{
    CStopWatch __debug_sw;
    bool ret = false;
    
    

#if 1   //debug        
    if(ServiceData.debugData.getExplorerDebug().frontiers.empty()){
        ServiceData.debugData.explorerDataClear();
    }
#endif
    // 탐색때 사용했던 클리프는 정리한다.
    ServiceData.robotMap.simplifyMap.cliffPoint.clearCliff();
    
    switch (ROBOT_CONTROL.slam.getMapSaveResult())
    {
    case E_SAVE_MAP_RESULT::NONE:
        PATH_PLANNER->stopMapUpdate();
        SUB_TASK.waveFrontier.stopUpdate();
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MAP_DRAWING_WAIT);
        ROBOT_CONTROL.slam.doSaveSlamMapFile(false);
        break;
    case E_SAVE_MAP_RESULT::COMPLETE:
        if(!SOUND_CTR.isSoundPlaying()){
            ROBOT_CONTROL.slam.setMapSaveResult(E_SAVE_MAP_RESULT::NONE);
            ret = true;
        }        
        break;
    case E_SAVE_MAP_RESULT::SAVING:
        break;
    case E_SAVE_MAP_RESULT::FAIL:
        ROBOT_CONTROL.slam.setMapSaveResult(E_SAVE_MAP_RESULT::NONE);
        break;
    default:
        eblog(LOG_LV_NECESSARY, "ERROR");
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;    
}


/**
 * @brief 탐색 점들 중 가장 가까운 점을 반환
 * jhnoh, 23.01.30
 * @param robotPose                     로봇의 위치
 * @param frontierPoints      프론티어중심 점들 
 * @return tPose 
 */
tPoint CTaskExplorer::getNearDisPose(tPose robotPose, std::list<tPoint> frontierPoints)
{
    CStopWatch __debug_sw;

    tPoint nearestFrontier;
    
    if(frontierPoints.empty())
    {
        nearestFrontier.x =0;
        nearestFrontier.y =0;
    }
    else
    {
        try 
        { 
            nearestFrontier = frontierPoints.back(); 
        } 
        catch(std::bad_alloc& ba) 
        { 
            std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
            std::cerr << " Bad allocation: " << ba.what() << '\n'; 
        } 
        catch(std::exception &e) 
        { 
            std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
            std::cerr << " Unhandled exception: " << e.what() << '\n'; 
        } 
        catch( ... ) 
        { 
            std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
            std::cerr << " Unknown exception: " << '\n'; 
        } 
        
        double frontierMinDist = utils::math::distanceTwoPoint(robotPose, nearestFrontier);    
        // 프론티어 점들 중 가장 가까운 점을 찾는다.
        for (tPoint point : frontierPoints)
        {
            double frontierDist = utils::math::distanceTwoPoint(robotPose, point);
            // if (frontierDist > frontierMinDist) // test로 가장 먼 점을 반환시키게 함
            if (frontierDist < frontierMinDist && frontierDist > 2.0)
            {
                frontierMinDist = frontierDist;
                nearestFrontier = point;
            }
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return nearestFrontier;
}

std::list<tPose> CTaskExplorer::clusterFrontierPoses(tPose robot, tPose nearPose, std::list<tPose> frontiers, double distance)
{
    CStopWatch __debug_sw;
 
    std::list<tPose> poses;
    bool bNearDistance = false;
    double dis;
    poses.push_back(nearPose);
    for(tPose frontier : frontiers )
    {
        bNearDistance = false;
        for(tPose pose : poses)
        {
            dis = utils::math::distanceTwoPoint(pose, frontier);
            if( dis < distance ||  utils::math::distanceTwoPoint(robot, frontier) < 2)
            {
                bNearDistance = true;
                break;
            }
        }

        if(bNearDistance == false)
        {
            poses.push_back(frontier);
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return poses;
}

void CTaskExplorer::getNeighbours(int n_array[], int position, int map_width) 
{
    CStopWatch __debug_sw;
    
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief explorer 상태 디버그 함수
 * jhnoh, 23.02.01
 */
void CTaskExplorer::__debug_state_print()
{
    CStopWatch __debug_sw;

    if(bDebugStatePrint)
    {
        eblog(LOG_LV_EXPLORER,    "[ EXPLORER ]  STATE ---- "<< enumToString(getExplorerState()) << "  ");
        bDebugStatePrint = false;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}


E_EXPLORER_STATE CTaskExplorer::getExplorerState()
{
    return state;
}



void CTaskExplorer::setExplorerState(E_EXPLORER_STATE set)   
{
    if(state != set){
        ceblog(LOG_LV_NECESSARY, GREEN, "탐색 시퀀스 변경 : "<< enumToString(state) << " ----> " << enumToString(set));
    }

    state = set;
}

void CTaskExplorer::setRecognizeState(E_RECOGNIZE_STATE set)   
{
    if(recognizeState != set){
        ceblog(LOG_LV_NECESSARY, GREEN, " RecognizeState 변경 : "<< enumToString(recognizeState) << " ----> " << enumToString(set));
    }

    recognizeState = set;
}

/**
 * @brief goal searching 을 위한 사전 작업.
 * 
 * @param robotPose 
 * @return E_EXPLORER_STATE 
 */
bool CTaskExplorer::checkWaveFrontier(tPose robotPose)
{
    CStopWatch __debug_sw;
    bool ret = false;

    if ( SUB_TASK.waveFrontier.isUpdateWdfPoints() )
    {
        if ( SUB_TASK.waveFrontier.useInvalidNearPoint(curGoal) )
        {
            newGoal = curGoal;
            newGoalList.push_back(newGoal);
            pathSearchStartTime = SYSTEM_TOOL.getSystemTime();
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "탐색 첫번째 목표 좌표를 찾았습니다 : " << curGoal.x << " , " << curGoal.y << " 경로를 검색해 볼게요~~!!");
            ret = true;
        }else{
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "탐색점 있는데 왜 유효성 검사 통과 못하냐!!!");
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}



bool CTaskExplorer::checkUpdateWaveFrontier(tPose robotPose)
{
    CStopWatch __debug_sw;
    bool ret = false;
    double distance = 0;
    if ( SUB_TASK.waveFrontier.isUpdateWdfPoints() )
    {
        if ( SUB_TASK.waveFrontier.useInvalidNearPoint(newGoal) )
        {
            pathSearchStartTime = SYSTEM_TOOL.getSystemTime();
            distance = utils::math::distanceTwoPoint(newGoal,curGoal);

            if(curGoal == newGoal)
            {
                ceblog(LOG_LV_NECESSARY, RED, "오류!! 프론티어에서 이미 탐색한 목적지를 업데이트하고 있어요!!! " << newGoal.x << " , " << newGoal.y << " 기존 목적지 : " << curGoal.x << " ," <<  curGoal.y);
            }
            else
            {
                if(distance < 1)
                {
                    ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "새로운 탐색 좌표가 기존 목적지와 너무 가까워요~~: " << newGoal.x << " , " << newGoal.y << " 기존 목적지 : " << curGoal.x << " ," <<  curGoal.y);
                }
                else
                {
                    newGoalList.push_back(newGoal);
                    ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "탐색할만한 새로운 탐색 좌표를 찾았습니다 : " << newGoal.x << " , " << newGoal.y << " 기존 목적지 : " << curGoal.x << " ," <<  curGoal.y);
                }
            }

            ret = true;
        }
    }

    return ret;
}

bool CTaskExplorer::isMapReady()
{
    bool ret = false; //ServiceData.map.isReady;

    double runTime = SYSTEM_TOOL.getSystemTime()-exploreStartTime;

    if(runTime >= 30 || bCheckFistWFP) ret = true;

    return ret;
}
