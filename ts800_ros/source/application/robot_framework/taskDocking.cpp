/**
 * @file docking.cpp
 * @author hhryu@everybot.net
 * @brief
 * @version 0.1
 * @date 2023~
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "taskDocking.h"
#include "eblog.h"
#include "utils.h"
#include "MessageHandler.h"
#include <memory>
#include "userInterface.h"
#include "control/motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "kinematics.h"

#define SG ServiceData.signal

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0           // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time){printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CTaskDocking::CTaskDocking()
{
    CStopWatch __debug_sw;    
    pFindCharger = new CFindCharger();
    bDebugStart = false;
    eblog(LOG_LV_NECESSARY, "create");
    TIME_CHECK_END(__debug_sw.getTime());
}

CTaskDocking::~CTaskDocking()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, "");

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 도킹상태 set
 *
 * @param state
 *
 * @note 연산시간 < 0.1ms
 * @date 2023-08-22
 * @author hhryu
 */
void CTaskDocking::setDockingState(DOCKING_STATE set)
{
    CStopWatch __debug_sw;

    if (state != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "E_DOCKING_STATE CHANGE " << CYN << "[" << enumToString(state) << "] --> [" << enumToString(set) <<"]");
    }
    state = set;

    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskDocking::setMovingState(MOVE_CHARGER_STATE set)
{
    CStopWatch __debug_sw;

    if (movingState != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "MOVE_CHARGER_STATE CHANGE " << CYN << "[" << enumToString(movingState) << "] --> [" << enumToString(set) <<"]");
    }
    movingState = set;

    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskDocking::setSignalTrackState(SIGNAL_TRACKING_STATE set)
{
    CStopWatch __debug_sw;

    if (trackingState != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "MOVE_CHARGER_STATE CHANGE " << CYN << "[" << enumToString(trackingState) << "] --> [" << enumToString(set) <<"]");
    }
    trackingState = set;

    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskDocking::taskStart()
{
    shortSignalCount = 0;
    bExplorer = false;
    isUpdateTargetPoint = false;
    if(ROBOT_CONTROL.slam.isExistedSlamMap()){
        setMovingState(MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT);
        setDockingState(DOCKING_STATE::MOVE_TO_CHARGER);
    }
    else{
        setDockingState(DOCKING_STATE::START_DOCKING_EXPLORER);
    }
}
bool CTaskDocking::taskRun(tPose robotPose)
{
    bool ret = false;

    // if(state == DOCKING_STATE::SWIMMING)
    // {
    //     if(avoiding.checkObstacle(robotPose,true,false))
    //     {
    //         ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지 하여 벽타기를 실행 하겠음.");
    //         MOTION.startStopOnMap(tProfile(),true);
    //         

    //         computeWalltrackDir(robotPose, taskMovePath.getCurrentTarget());          
    //         taskWallAvoid.taskStart2(robotPose,wallDir);
    //         state = DOCKING_STATE::AVOID_WALLTRACK;
    //     }
    // }
    
    switch (state)
    {
    case DOCKING_STATE::NONE:
         
        break;
    case DOCKING_STATE::START_FIND_SIGNAL:
        setDockingState(procStartFindSiganl(robotPose));
        break;
    case DOCKING_STATE::RUN_FIND_SIGNAL:
        setDockingState(procRunFindSiganl(robotPose));
        break;
    case DOCKING_STATE::START_DOCKING_EXPLORER:
        setDockingState(procStartDockingExplorer(robotPose));
        break;
    case DOCKING_STATE::RUN_DOCKING_EXPLORER:
        setDockingState(procRunDockingExplorer(robotPose));
        break;
    case DOCKING_STATE::SET_SEARCHING_AREA:
        setDockingState(procSetSearchingArea(robotPose));
        break;
    case DOCKING_STATE::SET_TARGET_SEARCHING_POINT:
        setDockingState(procSetTargetSearchingPoint(robotPose));
        break;
    case DOCKING_STATE::GO_SEARCHING_POINT:
        setDockingState(procGoSearchingPoint(robotPose));
        break;
    case DOCKING_STATE::CHECK_SIGNAL_TURN:
        setDockingState(procCheckSignalTurn(robotPose));
        break;        
    case DOCKING_STATE::MOVE_TO_CHARGER:
        setDockingState(procMoveToCharger(robotPose));
        break;
     case DOCKING_STATE::AVOID_WALLTRACK:
        setDockingState(procAvoidWalltrack(robotPose));
        break;
    case DOCKING_STATE::SIGNAL_TRACKING:
        setDockingState(procSignalTrack(robotPose));
    break;    
    case DOCKING_STATE::STOP_READY_MOVE:
        setDockingState(procStopReadyMove(robotPose));
        break;        
    case DOCKING_STATE::COMPLETE:
        setDockingState(DOCKING_STATE::NONE);
        ret = true;
        break;                    
    default:
        break;
    }

    return ret;
}

int CTaskDocking::getCenteringReadyTurnDirection(u16 data)
{
    int ret = 0;

    if(data & IS_SIGNAL_CENTER_SHORT)           ret += 0;
    if(data & IS_SIGNAL_CENTER_LONG)            ret += 0;
    if(data & IS_SIGNAL_RIGHT_CENTER_SHORT)     ret += 1;
    if(data & IS_SIGNAL_RIGHT_CENTER_LONG)      ret += 1;
    if(data & IS_SIGNAL_LEFT_CENTER_SHORT)      ret -= 1;
    if(data & IS_SIGNAL_LEFT_CENTER_LONG)       ret -= 1;
    if(data & IS_SIGNAL_RIGHT_SIDE_SHORT)       ret += 1;
    if(data & IS_SIGNAL_RIGHT_SIDE_LONG)        ret += 1;
    if(data & IS_SIGNAL_LEFT_SIDE_SHORT)        ret -= 1;
    if(data & IS_SIGNAL_LEFT_SIDE_LONG)         ret -= 1;

    return ret;
}

void CTaskDocking::DebugPrintSignalData(u16 data)
{
    switch (data)
    {
    case IS_SIGNAL_CENTER_SHORT :
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_CENTER_SHORT");
        break;
    case IS_SIGNAL_RIGHT_CENTER_SHORT :
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_CENTER_SHORT");
        break;
    case IS_SIGNAL_LEFT_CENTER_SHORT :
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_CENTER_SHORT");
        break;
    case IS_SIGNAL_RIGHT_SIDE_SHORT :
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_SIDE_SHORT");
        break;
    case IS_SIGNAL_LEFT_SIDE_SHORT :
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_SIDE_SHORT");
        break;                
    default:
        if(data != 0)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "UNKOWN DATA : " << (int)data);
        }
        break;
    }
}

void CTaskDocking::DebugPrintSignalData2(u16 data)
{
    if(data & IS_SIGNAL_CENTER_SHORT){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_CENTER_SHORT");
    }
    if(data & IS_SIGNAL_RIGHT_CENTER_SHORT){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_CENTER_SHORT");
    }
    if(data & IS_SIGNAL_LEFT_CENTER_SHORT){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_CENTER_SHORT");
    }
    if(data & IS_SIGNAL_RIGHT_SIDE_SHORT){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_SIDE_SHORT");
    }
    if(data & IS_SIGNAL_LEFT_SIDE_SHORT){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_SIDE_SHORT");
    }
    if(data & IS_SIGNAL_CENTER_LONG){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_CENTER_LONG");
    }
    if(data & IS_SIGNAL_RIGHT_CENTER_LONG){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_CENTER_LONG");
    }
    if(data & IS_SIGNAL_LEFT_CENTER_LONG){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_CENTER_LONG");
    }
    if(data & IS_SIGNAL_RIGHT_SIDE_LONG){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_RIGHT_SIDE_LONG");
    }

    if(data & IS_SIGNAL_LEFT_SIDE_LONG){
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SIGNAL_LEFT_SIDE_LONG");
    }

}


DOCKING_STATE CTaskDocking::monitorFindSignal(DOCKING_STATE curState, tPose robotPose)
{
    DOCKING_STATE ret = curState;

    CRobotKinematics k;

    if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_SHORT)>=2 &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG|IS_SIGNAL_RIGHT_SIDE_SHORT) >= 2)
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        shortSignalCount = 0;
        setSignalTrackState(SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK);
        ret = DOCKING_STATE::SIGNAL_TRACKING;
    }
    else if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING) )
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        setSignalTrackState(SIGNAL_TRACKING_STATE::SWIMMING);
        ret = DOCKING_STATE::SIGNAL_TRACKING;
    }
    else if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG))||
        (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        MOTION.startStopOnMap(tProfile(),false);
        readyTryDockTunrnDir = -1;
        ret = DOCKING_STATE::STOP_READY_MOVE;
        setSignalTrackState(SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN);
        tempState = DOCKING_STATE::SIGNAL_TRACKING;
    }
    else if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)) || 
        (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        MOTION.startStopOnMap(tProfile(),false);
        readyTryDockTunrnDir = 1;
        ret = DOCKING_STATE::STOP_READY_MOVE;
        setSignalTrackState(SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN);
        tempState = DOCKING_STATE::SIGNAL_TRACKING;
    }
    else if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_ANYTHING))
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        MOTION.startStopOnMap(tProfile(),false);
        
        readyTryDockTunrnDir = -1;
        sideReceiverData = SG.receiverDetected(IDX_RECEIVER_SIDE_LEFT);
        targetRad = k.rotation(robotPose, DEG2RAD(90));        

        DebugPrintSignalData2(sideReceiverData);
        ret = DOCKING_STATE::STOP_READY_MOVE;
        tempState = DOCKING_STATE::CHECK_SIGNAL_TURN;
    }
    else if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_ANYTHING) )
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        MOTION.startStopOnMap(tProfile(),false);
        
        sideReceiverData = SG.receiverDetected(IDX_RECEIVER_SIDE_RIGHT);
        DebugPrintSignalData2(sideReceiverData);
        readyTryDockTunrnDir = 1;
        targetRad = k.rotation(robotPose, DEG2RAD(-90));
        ret = DOCKING_STATE::STOP_READY_MOVE;
        tempState = DOCKING_STATE::CHECK_SIGNAL_TURN;
    }

    return ret;
}


MOVE_CHARGER_STATE CTaskDocking::searchChargerPoint(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT;
    tSignalCheckPose signalPose = SG.getSignalCheckPose();
    if(signalPose.bCenter || signalPose.bLeftSide || signalPose.bRightSide)
    {
        if(signalPose.bCenter)          chargerPoint = tPoint(signalPose.centerPose.x,signalPose.centerPose.y);
        else if(signalPose.bLeftSide)   chargerPoint = tPoint(signalPose.leftSidePose.x,signalPose.leftSidePose.y);
        else                            chargerPoint = tPoint(signalPose.rightSidePose.x,signalPose.rightSidePose.y);
    }
    else
    {
        chargerPoint = tPoint(0,0);
    }
    
    taskPathPlan.taskStart(chargerPoint);
    ret = MOVE_CHARGER_STATE::PATH_PLAN;
    return ret;
}

MOVE_CHARGER_STATE CTaskDocking::makePathPlan(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::PATH_PLAN;

    movingPath = taskPathPlan.taskRun(robotPose);

    if(!movingPath.empty()) ret = MOVE_CHARGER_STATE::START_MOVE;

    return ret;
}

MOVE_CHARGER_STATE CTaskDocking::startMove(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::START_MOVE;
    taskMovePath.taskStart(movingPath,0.2,tProfile());
    ret = MOVE_CHARGER_STATE::RUN_MOVING;
    return ret;
}

MOVE_CHARGER_STATE CTaskDocking::runMoving(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::RUN_MOVING;
    CRobotKinematics k;

    if(avoiding.checkObstacle(robotPose,true,false))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지 하여 벽타기를 실행 하겠음.");
        MOTION.startStopOnMap(tProfile(),true);
        

        computeWalltrackDir(robotPose, taskMovePath.getCurrentTarget());          

        taskWallAvoid.taskStart(robotPose,movingPath,chargerPoint,wallDir);
        ret = MOVE_CHARGER_STATE::AVOID_WALLTRACK;
    }
    else if(taskMovePath.taskRun(robotPose)){
        targetRad = k.rotation(robotPose, DEG2RAD(350));
        MOTION.startRotation(robotPose,targetRad,tProfile(),E_ROTATE_DIR::CCW);
        ret = MOVE_CHARGER_STATE::ARRIVED;
    }
    
    return ret;
}

MOVE_CHARGER_STATE CTaskDocking::runAvoidWalltrack(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::AVOID_WALLTRACK;

    // 벽타기 종료조건 수정 필요. 시작점까지 오는 것 안먹힘. 시작점이 계속 변함.
    if(taskWallAvoid.taskRun(robotPose) || taskWallAvoid.isReturnStartPoint())  ret = MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT;

    return ret;
}

MOVE_CHARGER_STATE CTaskDocking::ArrivedTarget(tPose robotPose)
{
    MOVE_CHARGER_STATE ret = MOVE_CHARGER_STATE::ARRIVED;
    
    if(!MOTION.isRunning())
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "충전기 위치 도착!! 할때까지 신호를 못봤어?? ");
        ret = MOVE_CHARGER_STATE::NONE;
    }
    return ret;
}

DOCKING_STATE CTaskDocking::procMoveToCharger(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::MOVE_TO_CHARGER;

    switch (movingState)
    {
    case MOVE_CHARGER_STATE::NONE :
        break;
    case MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT :
        setMovingState(searchChargerPoint(robotPose));
        break;
    case MOVE_CHARGER_STATE::PATH_PLAN :
        setMovingState(makePathPlan(robotPose));
        break;
    case MOVE_CHARGER_STATE::START_MOVE :
        setMovingState(startMove(robotPose));
        break;
    case MOVE_CHARGER_STATE::RUN_MOVING :
        setMovingState(runMoving(robotPose)); 
        break;
    case MOVE_CHARGER_STATE::AVOID_WALLTRACK :
        setMovingState(runAvoidWalltrack(robotPose));
        break;      
    case MOVE_CHARGER_STATE::ARRIVED :
        setMovingState(ArrivedTarget(robotPose));
        ret = DOCKING_STATE::START_FIND_SIGNAL;
        break;                   
    default:
        break;
    }

    if(movingState != MOVE_CHARGER_STATE::ARRIVED && movingState != MOVE_CHARGER_STATE::NONE)//if(utils::math::distanceTwoPoint(robotPose,chargerPoint) <= 1)
    {
        ret = monitorFindSignal(DOCKING_STATE::MOVE_TO_CHARGER,robotPose);
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procSignalTrack(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::SIGNAL_TRACKING;

    SG.debugSignalPrint();

    if(ServiceData.power.getExtPower())
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "충전기 단자 인식되어 시그널 트래킹 종료!! ");
        MOTION.startStopOnMap(tProfile(),true);
        
        return DOCKING_STATE::NONE;
    }
    switch (trackingState)
    {
    case SIGNAL_TRACKING_STATE::CHECK_TURN :
        setSignalTrackState(trackCheckSignalTurn(robotPose));
        break;    
    case SIGNAL_TRACKING_STATE::SWIMMING :
        setSignalTrackState(trackSwimming(robotPose));
        break;
    case SIGNAL_TRACKING_STATE::BALANCE_TURN :
        setSignalTrackState(trackReadyMoveCenterTurn(robotPose));
        break;
    case SIGNAL_TRACKING_STATE::MOVING_CENTER :
        setSignalTrackState(trackMovingCenter(robotPose));
        break;
    case SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN :
        setSignalTrackState(trackTryDockReadyTurn(robotPose));
        break;
    case SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK :
        setSignalTrackState(trackTilDownTryDock(robotPose));
        break;      
    case SIGNAL_TRACKING_STATE::TILUP_TRY_DOCK :
        setSignalTrackState(trackTilUpTryDock(robotPose));
        break;                   
    default:
        break;
    }

    if(!SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING) && !SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING) &&
        !SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_ANYTHING) && !SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_ANYTHING))
    {
        if(((SYSTEM_TOOL.getSystemTime()-noSignalTime >= CONFIG.noSignalTimeOut) || (robotPose.distance(noSignalPoint) >= 1)) && 
        (trackingState == SIGNAL_TRACKING_STATE::SWIMMING || trackingState == SIGNAL_TRACKING_STATE::MOVING_CENTER ||
         trackingState == SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK || trackingState == SIGNAL_TRACKING_STATE::TILUP_TRY_DOCK))
        {
            ret = DOCKING_STATE::START_FIND_SIGNAL;
        }
    }
    else
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        noSignalPoint = tPoint(robotPose.x,robotPose.y);
    }
    
    
    return ret;
}

DOCKING_STATE CTaskDocking::procAvoidWalltrack(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::AVOID_WALLTRACK;
    taskWallAvoid.taskRun2(robotPose);

    ret = monitorFindSignal(DOCKING_STATE::AVOID_WALLTRACK,robotPose);

    return ret;
}

DOCKING_STATE CTaskDocking::procStartFindSiganl(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::START_FIND_SIGNAL;
    CRobotKinematics k;
    tProfile profile;
    profile.desAngVel = DEG2RAD(25);
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);

    if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING))
    {
        ret = monitorFindSignal(DOCKING_STATE::START_FIND_SIGNAL,robotPose);
    }
    else if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_ANYTHING))
    {
        noSignalTime = SYSTEM_TOOL.getSystemTime();
        targetRad = k.rotation(robotPose, DEG2RAD(-350));
        MOTION.startRotation(robotPose,targetRad,profile,E_ROTATE_DIR::CW);
        ret = DOCKING_STATE::RUN_FIND_SIGNAL;
    }
    else
    {
        targetRad = k.rotation(robotPose, DEG2RAD(350));
        MOTION.startRotation(robotPose,targetRad,profile,E_ROTATE_DIR::CCW);
        ret = DOCKING_STATE::RUN_FIND_SIGNAL;
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procRunFindSiganl(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::RUN_FIND_SIGNAL;
    
    if(MOTION.isNearTargetRad(robotPose,targetRad,DEG2RAD(5)))
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, " 충전기 신호를 찾을 수 없어요. 도킹 스테이션 탐색 모션을 진행합니다. ");
        MOTION.startStopOnMap( tProfile(), false);
        if(bExplorer){
            setMovingState(MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT);
            ret = DOCKING_STATE::MOVE_TO_CHARGER;
        }
        else{
            ret = DOCKING_STATE::START_DOCKING_EXPLORER;
        }
    }
    else
    {
        ret = monitorFindSignal(DOCKING_STATE::RUN_FIND_SIGNAL,robotPose);
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procStartDockingExplorer(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::START_DOCKING_EXPLORER;
    
    if(!MOTION.isRunning())
    {
        bExplorer = true;
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, " taskDockingExplorer 스타트! ");
        taskDockingExplorer.taskStart();
        ret = DOCKING_STATE::RUN_DOCKING_EXPLORER;
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procRunDockingExplorer(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::RUN_DOCKING_EXPLORER;
    
    if (taskDockingExplorer.checkDockingSignal())
    {
        PATH_PLANNER->stopMapUpdate();
        SUB_TASK.waveFrontier.stopUpdate();
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, " 도킹 스테이션 탐색 중 신호를 찾아 도킹을 시작합니다. ");
        ret = monitorFindSignal(DOCKING_STATE::RUN_DOCKING_EXPLORER,robotPose);
    }
    else if (!taskDockingExplorer.taskRun(robotPose))
    {
        if (taskDockingExplorer.getRunningTime() >= 120)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " 충전기 신호를 찾을 수 없어요 (탐색 안끝난 상태로 120초 지남) ");
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::DOCKING_FAIL)));
            ret = DOCKING_STATE::NONE;
        }
    }
    else
    {
        if (taskDockingExplorer.getRunningTime() <= 5)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, " 충전기 신호를 찾을 수 없어요 (탐색 끝남, 5초만 기다려 볼게요) ");
        }
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " 충전기 신호를 찾을 수 없어요 (영역 중심점 기준 사방 포인트 5개로 이동하면서 신호가 감지되는지 확인할게요) ");
        ret = DOCKING_STATE::SET_SEARCHING_AREA;
    }

    return ret;
}
DOCKING_STATE CTaskDocking::procSetSearchingArea(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::SET_SEARCHING_AREA;
    bool bMakeMap = false;
    if( ROBOT_CONTROL.slam.isExistedSlamMap() )
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "도킹 탐색 이동 준비: 지도를 불러와 영역을 생성합니다");
        bMakeMap = SUB_TASK.cleanPlan.makeAreaByMap();       
    }
    else
    {
        SUB_TASK.cleanPlan.makeCustomArea(robotPose);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "도킹 탐색 이동 준비: 지도가 없어 임시 영역을 생성합니다");
        bMakeMap = true;
    }

    if(bMakeMap){
        int uncleanedRoomSize = SUB_TASK.cleanPlan.getUnCleanRoomSize();

        if (uncleanedRoomSize > 0)
        {
            SUB_TASK.cleanPlan.makePlan(robotPose);
            dockingExplorerArea = SUB_TASK.cleanPlan.getCurrentAreaPolygons();
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "==============================================");
            for (auto points : dockingExplorerArea)
            {
                ceblog(LOG_LV_NECESSARY, BOLDBLUE, "도킹탐색 영역 좌표 ( X, Y ) : ( " << points.x << " , " << points.y << " )");
            }
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "==============================================");
        }
        dockingTargetSearchingPoints.clear();
        tPoint areaCenter = utils::area::findCentroid(dockingExplorerArea);
        dockingTargetSearchingPoints.push_back(areaCenter);
        dockingTargetSearchingPoints.push_back(tPoint(areaCenter.x + 0.3, areaCenter.y));
        dockingTargetSearchingPoints.push_back(tPoint(areaCenter.x, areaCenter.y + 0.3));
        dockingTargetSearchingPoints.push_back(tPoint(areaCenter.x - 0.3, areaCenter.y));
        dockingTargetSearchingPoints.push_back(tPoint(areaCenter.x, areaCenter.y - 0.3));
        isUpdateTargetPoint = true;
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "==============================================");
        for (auto points : dockingTargetSearchingPoints)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, "도킹탐색 타겟 좌표 ( X, Y ) : ( " << points.x << " , " << points.y << " )");
        }
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "==============================================");
        ret = DOCKING_STATE::SET_TARGET_SEARCHING_POINT;
    }

    return ret;
}
DOCKING_STATE CTaskDocking::procSetTargetSearchingPoint(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::SET_TARGET_SEARCHING_POINT;

    
    if (!MOTION.isRunning())
    {
        if (!dockingTargetSearchingPoints.empty() && isUpdateTargetPoint)
        {
            isUpdateTargetPoint = false;
            targetSearchingPoint = dockingTargetSearchingPoints.front();
            dockingTargetSearchingPoints.pop_front();
            MOTION.startLinearAngularPriorToPointOnMap(robotPose, targetSearchingPoint, tProfile());
            ret = DOCKING_STATE::GO_SEARCHING_POINT;
        }
        else
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " 충전기 신호를 찾을 수 없어요 ");
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::DOCKING_FAIL)));
            ret = DOCKING_STATE::NONE;
        }   
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procGoSearchingPoint(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::GO_SEARCHING_POINT;
    if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING))
    {
        ret = monitorFindSignal(DOCKING_STATE::GO_SEARCHING_POINT,robotPose);
    }
    // if (taskDockingExplorer.checkDockingSignal())
    // {
    //     PATH_PLANNER->stopMapUpdate();
    //     SUB_TASK.waveFrontier.stopUpdate();
    //     ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, " 도킹 스테이션 탐색 중 신호를 찾아 도킹을 시작합니다. ");
    //     ret = monitorFindSignal(DOCKING_STATE::RUN_DOCKING_EXPLORER,robotPose);
    // }

    
    if (MOTION.isNearTargetPose(robotPose, targetSearchingPoint, 0.1))
    {
        isUpdateTargetPoint = true;
        MOTION.startStopOnMap(tProfile(), false);
        ret = DOCKING_STATE::SET_TARGET_SEARCHING_POINT;
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procCheckSignalTurn(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::CHECK_SIGNAL_TURN;
    SUB_TASK.signaltracking.motionCheckSignalTurn(readyTryDockTunrnDir);
    
    if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,sideReceiverData) > 0 && SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,sideReceiverData) > 0) && 
    (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,sideReceiverData) >= 2 || SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,sideReceiverData) >= 2))
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        MOTION.startStopOnMap(tProfile(),false);
        ret = DOCKING_STATE::STOP_READY_MOVE;
        setSignalTrackState(SIGNAL_TRACKING_STATE::SWIMMING);
        tempState = DOCKING_STATE::SIGNAL_TRACKING;
    }
    else if(MOTION.isNearTargetRad(robotPose,targetRad,DEG2RAD(1)))
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        MOTION.startStopOnMap(tProfile(),false);
        ret = DOCKING_STATE::STOP_READY_MOVE;
        setSignalTrackState(SIGNAL_TRACKING_STATE::SWIMMING);
        tempState = DOCKING_STATE::SIGNAL_TRACKING;
    }
    return ret;
}

SIGNAL_TRACKING_STATE CTaskDocking::trackCheckSignalTurn(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::CHECK_TURN;
    SUB_TASK.signaltracking.motionCheckSignalTurn(readyTryDockTunrnDir);
    
    if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,sideReceiverData) > 0 && SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,sideReceiverData) > 0) && 
    (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,sideReceiverData) >= 2 || SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,sideReceiverData) >= 2))
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " 정면에 신호감지 됨!! ");
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        ret = SIGNAL_TRACKING_STATE::SWIMMING;

    }
    else if(MOTION.isNearTargetRad(robotPose,targetRad,DEG2RAD(1)))
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " 신호 감지 없이 제어 끝남!! ");
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        ret = SIGNAL_TRACKING_STATE::SWIMMING;
    }
    return ret;
}


SIGNAL_TRACKING_STATE CTaskDocking::trackSwimming(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::SWIMMING;
    CRobotKinematics k;
    u16 commonData = 0, lRcvData = 0, rRcvData = 0;
    u8 lRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_SHORT_ANYTHING), rRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_SHORT_ANYTHING);

    signalData = getSwimmingRefSignalData();
    SUB_TASK.signaltracking.motionSwimming(signalData);

    if(lRcvDataCount > 0) lRcvData = SG.getReceivedShortSignalData(IDX_RECEIVER_FRONT_LEFT);
    if(rRcvDataCount > 0) rRcvData = SG.getReceivedShortSignalData(IDX_RECEIVER_FRONT_RIGHT);

    if(lRcvData > 0 && rRcvData > 0)
    {
        commonData = lRcvData & rRcvData;
        shortSignalData = commonData;
        if(getCenteringReadyTurnDirection(shortSignalData) == 0)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 TILDOWN TRY-DOCK!! 감지 신호는 ? ");
            ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK; 
        }    
        else
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 MOVING CENTER!! 감지 신호는 ? ");
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
        }
        DebugPrintSignalData(shortSignalData);
    }
    else if(lRcvData > 0)
    {
        shortSignalData = lRcvData;
        if(getCenteringReadyTurnDirection(shortSignalData) == 0)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 TILDOWN TRY-DOCK!! 감지 신호는 ? ");
            ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK; 
        }    
        else
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 MOVING CENTER!! 감지 신호는 ? ");
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
        }
        DebugPrintSignalData(shortSignalData);
    }
    else if(rRcvData > 0)
    {
        shortSignalData = rRcvData;
        if(getCenteringReadyTurnDirection(shortSignalData) == 0)
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 TILDOWN TRY-DOCK!! 감지 신호는 ? ");
            ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK; 
        }    
        else
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "SWIMMING 중 SHORT 신호 감지 MOVING CENTER!! 감지 신호는 ? ");
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
        }
        DebugPrintSignalData(shortSignalData);
    }
    else if( shortSignalCount == 0 && (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING)+SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING) == 0) && 
            (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_ANYTHING) >=2 || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_ANYTHING) >=2 ))
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_ANYTHING) >=2)
        {
            readyTryDockTunrnDir = -1;
            sideReceiverData = SG.receiverDetected(IDX_RECEIVER_SIDE_LEFT);
            targetRad = k.rotation(robotPose, DEG2RAD(90));        
        }
        else
        {
            readyTryDockTunrnDir = 1;
            sideReceiverData = SG.receiverDetected(IDX_RECEIVER_SIDE_RIGHT);
            targetRad = k.rotation(robotPose, DEG2RAD(-90));
        }
        DebugPrintSignalData2(sideReceiverData);                                                                    
        ret = SIGNAL_TRACKING_STATE::CHECK_TURN;
    }
    else if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_SHORT)>=2 &&
            SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG|IS_SIGNAL_RIGHT_SIDE_SHORT) >= 2)
    {
        shortSignalCount = 0;
        ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK;
    }
    // else if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_SHORT)>=2 &&
    // SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_RIGHT_SIDE_SHORT) >= 2)
    // {
    //     ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
    //     ret = DOCKING_STATE::TILUP_TRY_DOCK;
    // }

    return ret;
}

SIGNAL_TRACKING_STATE CTaskDocking::trackReadyMoveCenterTurn(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
    int dir = getCenteringReadyTurnDirection(shortSignalData);
    SUB_TASK.signaltracking.motionCenteringReadyTurn(dir);

    if(dir == 0)
    {
        if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG|IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG >= 2) &&
         SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG >= 2))
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "이미 센터에 와있음 ");
            movingCenterDir = 0;
            ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK;
        }
    }
    else if(dir > 0)
    {
        if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_SIDE_SHORT|IS_SIGNAL_RIGHT_SIDE_LONG|IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) >=2 )
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "오른쪽 감지 ---> 왼쪽으로 가야함 ");
            movingCenterDir = 1;
            ret = SIGNAL_TRACKING_STATE::MOVING_CENTER;
        }
    }
    else
    {
        if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_SIDE_SHORT|IS_SIGNAL_LEFT_SIDE_LONG|IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG) >=2 )
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "왼쪽 감지 ---> 오른쪽으로 가야함 ");
            movingCenterDir = -1;
            ret = SIGNAL_TRACKING_STATE::MOVING_CENTER;
        }
    }

    return ret;
}

DOCKING_STATE CTaskDocking::procStopReadyMove(tPose robotPose)
{
    DOCKING_STATE ret = DOCKING_STATE::STOP_READY_MOVE;
    
    if(!MOTION.isRunning())
    {
        ret = tempState;
    }
    
    return ret;
}

SIGNAL_TRACKING_STATE CTaskDocking::trackMovingCenter(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::MOVING_CENTER;

    SUB_TASK.signaltracking.motionMovingCenter(movingCenterDir);

    if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)) || 
        (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        readyTryDockTunrnDir = 1;
        ret = SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN;
    }
    else if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG))||
        (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        readyTryDockTunrnDir = -1;
        ret = SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN;
    }
    else if(!SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG))
    {
        shortSignalData = IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG;
        ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
    }
    else if(!SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG) &&
    SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG))
    {
        shortSignalData = IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG;
        ret = SIGNAL_TRACKING_STATE::BALANCE_TURN;
    }
    
    return ret;
}

SIGNAL_TRACKING_STATE CTaskDocking::trackTryDockReadyTurn(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN;

    SUB_TASK.signaltracking.motionTryDockReadyTurn(readyTryDockTunrnDir);

    // if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
    //    SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG) && 
    //    SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
    //    SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG))
    if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_SHORT)>=2 &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG|IS_SIGNAL_RIGHT_SIDE_SHORT) >= 2)
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, " CENTER 감지 ");
        shortSignalCount = 0;
        ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK;
    }
    
    return ret;
}


SIGNAL_TRACKING_STATE CTaskDocking::trackTilDownTryDock(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK;
    u16 lRcvData = 0, rRcvData = 0;

    SUB_TASK.signaltracking.motionTilDownTryDock();

    if(SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_CENTER_SHORT) || SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_CENTER_SHORT))
    {
        if(++shortSignalCount >= 10)
        {
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            rRcvData = SG.getReceivedShortSignalData(IDX_RECEIVER_FRONT_RIGHT);
            lRcvData = SG.getReceivedShortSignalData(IDX_RECEIVER_FRONT_LEFT);
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "왼쪽 수신부 SHORT-SIGNAL 은 ? ");
            DebugPrintSignalData(lRcvData);

            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "오른쪽 수신부 SHORT-SIGNAL 은 ? ");
            DebugPrintSignalData(rRcvData);
            ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
            ret = SIGNAL_TRACKING_STATE::TILUP_TRY_DOCK;
        }
    }

    else if(!SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_LEFT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_LONG|IS_SIGNAL_RIGHT_CENTER_LONG|IS_SIGNAL_RIGHT_CENTER_SHORT) && 
    !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_FRONT_RIGHT,IS_SIGNAL_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_LONG|IS_SIGNAL_RIGHT_CENTER_LONG|IS_SIGNAL_LEFT_CENTER_SHORT))
    {
        if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG))||
        (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
        !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_LEFT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
        {
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            readyTryDockTunrnDir = -1;
            ret = SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN;
        }
        else if((SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
                SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
                SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)) || 
                (SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_CENTER_SHORT|IS_SIGNAL_CENTER_LONG) && 
                !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_RIGHT_CENTER_SHORT|IS_SIGNAL_RIGHT_CENTER_LONG) &&
                !SG.countBitSignalDetectedCASE3(IDX_RECEIVER_SIDE_RIGHT,IS_SIGNAL_LEFT_CENTER_SHORT|IS_SIGNAL_LEFT_CENTER_LONG)))
        {
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            readyTryDockTunrnDir = 1;
            ret = SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN;
        }
        else if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_ANYTHING))
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "오른쪽 감지 ---> 왼쪽으로 가야함 ");
            movingCenterDir = 1;
            ret = SIGNAL_TRACKING_STATE::MOVING_CENTER;
        }
        else if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_ANYTHING))
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "왼쪽 감지 ---> 오른쪽으로 가야함 ");
            movingCenterDir = -1;
            ret = SIGNAL_TRACKING_STATE::MOVING_CENTER;
        }
    }
    
    return ret;
}

SIGNAL_TRACKING_STATE CTaskDocking::trackTilUpTryDock(tPose robotPose)
{
    SIGNAL_TRACKING_STATE ret = SIGNAL_TRACKING_STATE::TILUP_TRY_DOCK;
    
    SUB_TASK.signaltracking.motionTilUpTryDock();

    return ret;
}


/**
 * @brief 벽타기 방향 결정
 * 
 * @param robotPose 
 */
void CTaskDocking::computeWalltrackDir(tPose robotPose, tPoint target)
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

u16 CTaskDocking::getSwimmingRefSignalData()
{
    u16 ret = 0;
    u16 lRcvData = 0, rRcvData = 0;
    u8 lRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_LONG_ANYTHING), rRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_LONG_ANYTHING);

    if(lRcvDataCount >= 2) lRcvData = SG.getReceivedLongSignalData(IDX_RECEIVER_FRONT_LEFT);
    if(rRcvDataCount >= 2) rRcvData = SG.getReceivedLongSignalData(IDX_RECEIVER_FRONT_RIGHT);

    if(lRcvData > 0 && rRcvData > 0)    ret = lRcvData & rRcvData;
    else if(lRcvData > 0)               ret = lRcvData;
    else if(rRcvData > 0)               ret = rRcvData;
    else                                ret = SIGNAL_ANYTHING;

    return ret;
}

u16 CTaskDocking::getRefSignalData()
{
    u16 ret = 0;
    u16 lRcvData = 0, rRcvData = 0;
    u8 lRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING), rRcvDataCount = SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING);

    if(lRcvDataCount >= 2) lRcvData = SG.receiverDetected(IDX_RECEIVER_FRONT_LEFT);
    if(rRcvDataCount >= 2) rRcvData = SG.receiverDetected(IDX_RECEIVER_FRONT_RIGHT);

    if(lRcvData > 0 && rRcvData > 0)    ret = lRcvData & rRcvData;
    else if(lRcvData > 0)               ret = lRcvData;
    else if(rRcvData > 0)               ret = rRcvData;
    else                                ret = SIGNAL_ANYTHING;

    return ret;
}
