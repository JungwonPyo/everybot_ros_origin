#include "taskAvoidCharger.h"
#include "utils.h"
#include "eblog.h"
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


CTaskAvoidCharger::CTaskAvoidCharger()
{
    CStopWatch __debug_sw;
    setAvoidChargerState(CHARGER_AVOID_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskAvoidCharger::~CTaskAvoidCharger()
{
    CStopWatch __debug_sw;

    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskAvoidCharger::setAvoidChargerState(CHARGER_AVOID_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskAvoidCharger::taskStart(E_WALLTRACK_DIR dir)
{
    walltrackDir = dir;
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
    setAvoidChargerState(CHARGER_AVOID_STATE::SIGNAL_BALANCE);
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
bool CTaskAvoidCharger::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    if (avoidChagerComplete(robotPose)) setAvoidChargerState(CHARGER_AVOID_STATE::COMPLETE);

    switch (state)
    {
    case CHARGER_AVOID_STATE::NONE:
        break;
    case CHARGER_AVOID_STATE::AVOID_START:
        //setAvoidChargerState();
        break;
    case CHARGER_AVOID_STATE::SIGNAL_BALANCE:
        setAvoidChargerState(procSignalBalanceTurn());
        break;
    case CHARGER_AVOID_STATE::MOVE_CENTER:
        setAvoidChargerState(procMoveSignalCenter());
        break;
    case CHARGER_AVOID_STATE::ACROSS_OPPOSITE:
        setAvoidChargerState(procAcrossOpposite(robotPose));
        break;
    case CHARGER_AVOID_STATE::COMPLETE:
        setAvoidChargerState(procAvoidComplete(robotPose));
        ret = true;
        break;                
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


/**
 * @brief 벽타기중 크래들 회피 시작 조건, 방향에 따라 다른 조건을 가진다.
 * 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
CHARGER_AVOID_STATE CTaskAvoidCharger::procCheckStartAvoid(E_WALLTRACK_DIR dir)
{
    CHARGER_AVOID_STATE ret = CHARGER_AVOID_STATE::NONE;
    u8 shortSignal = 0, longSignal = 0;
    if (dir == E_WALLTRACK_DIR::RIGHT)
    {
        shortSignal = SIGNAL_RIGHT_SIDE_SHORT;
        longSignal = SIGNAL_RIGHT_SIDE_LONG;
    }
    else if (dir == E_WALLTRACK_DIR::LEFT)
    {
        shortSignal = SIGNAL_LEFT_SIDE_SHORT;
        longSignal = SIGNAL_LEFT_SIDE_LONG;
    } ELSE_ERROR

    if (ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, shortSignal)
    + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, shortSignal) >= 2)
    {
        if (ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) == ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, shortSignal) + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, longSignal)
        && ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING) == ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, shortSignal) + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, longSignal))
        {
            ServiceData.signal.debugSignalPrint("크래들 장애물이 감지 되었습니다.");
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
            ret = CHARGER_AVOID_STATE::SIGNAL_BALANCE;
        }
    }
    return ret;
}

/**
 * @brief 크래들을 마주본 상황에서 회피를 위해 회전
 * 
 * @param _curVel 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
CHARGER_AVOID_STATE CTaskAvoidCharger::procSignalBalanceTurn()
{
    CHARGER_AVOID_STATE ret = CHARGER_AVOID_STATE::SIGNAL_BALANCE;
    
    u8 receiver = IDX_RECEIVER_VOID;
    if (walltrackDir == E_WALLTRACK_DIR::RIGHT)
    {
        receiver = IDX_RECEIVER_SIDE_RIGHT;
        SUB_TASK.signaltracking.motionCenteringReadyTurn(1);
    } 
    else if (walltrackDir == E_WALLTRACK_DIR::LEFT)
    {
        receiver = IDX_RECEIVER_SIDE_LEFT;
        SUB_TASK.signaltracking.motionCenteringReadyTurn(0);
    }                                         
    ELSE_ERROR

    if(ServiceData.signal.countSignalDetected(receiver,SIGNAL_SHORT_ANYTHING))
    {
        ret = CHARGER_AVOID_STATE::MOVE_CENTER;
    }

    return ret;
}

/**
 * @brief 중앙으로 이동
 * 
 * @param _curVel 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
CHARGER_AVOID_STATE CTaskAvoidCharger::procMoveSignalCenter()
{
    CHARGER_AVOID_STATE ret = CHARGER_AVOID_STATE::MOVE_CENTER;
    
    motionMovingCenter(walltrackDir);    

    if (isCheckAcross(walltrackDir)) ret = CHARGER_AVOID_STATE::ACROSS_OPPOSITE;

    return ret;
}

/**
 * @brief 중앙을 지나 벽을 향해 가는 중
 * 
 * @param robotPose 
 * @param _curVel 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
CHARGER_AVOID_STATE CTaskAvoidCharger::procAcrossOpposite(tPose robotPose)
{
    CHARGER_AVOID_STATE ret = CHARGER_AVOID_STATE::ACROSS_OPPOSITE;
    motionAcrossOppsite(walltrackDir);

    return ret;
}

/**
 * @brief 크래들 탈출 완료
 * 
 * @param robotPose 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
CHARGER_AVOID_STATE CTaskAvoidCharger::procAvoidComplete(tPose robotPose)
{
    CHARGER_AVOID_STATE ret = CHARGER_AVOID_STATE::COMPLETE;
    
    //충전기 회피가 완료되면 다시 지도 그리자
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);

    
    if(!MOTION.isRunning()) ret = CHARGER_AVOID_STATE::NONE;

    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "충전기 회피 끝!! 멈추는 중..");

    return ret;
}


/**
 * @brief 중앙을 지났음을 체크
 * 
 * @param dir 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
bool CTaskAvoidCharger::isCheckAcross(E_WALLTRACK_DIR dir)
{
    bool bRet = false;
    if (dir == E_WALLTRACK_DIR::RIGHT)
    {
        if(ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_CENTER_LONG) 
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_CENTER_SHORT)
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_LEFT_CENTER_LONG) 
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_LEFT_CENTER_SHORT))
        {
            bRet = true;
        }
    }
    else if(dir == E_WALLTRACK_DIR::LEFT)
    {
        if(ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_CENTER_LONG) 
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_CENTER_SHORT)
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_RIGHT_CENTER_LONG) 
        || ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_RIGHT_CENTER_SHORT))
        {
            bRet = true;
        }
    }
    return bRet;
}

bool CTaskAvoidCharger::avoidChagerComplete(tPose robotPose)
{
    bool bRet = false;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if (pObstacle->bumper.value || pObstacle->front.obstacle.value)
    {
        bRet = true;
        MOTION.startStopOnMap(tProfile(),false);
        ceblog((LOG_LV_OBSTACLE | LOG_LV_WALL | LOG_LV_NECESSARY), MAGENTA, "bumper[" << DEC(pObstacle->bumper.value) 
        << "] ir[" << pObstacle->ir.left.raw_data <<"," << pObstacle->ir.center.raw_data  << "," << pObstacle->ir.right.raw_data << "]");
    }
    return bRet;
}
/**
 * @brief 어떠한 값(속력 등)을 target에 서서히 수렴하고 싶을 때 사용.
 * 위치가 바뀌어야 한다. utils.cpp나 control관련 유틸이 있는 곳 등.
 * 
 * @param speed : 리턴받을 속력 (v,w 모두 가능)
 * @param target : 수렴하고싶은 값
 * @param acceleration : 수렴하고 싶은 값에 도달하는 가속력
 * @return double speed
 * 
 * @note 연산시간 ms
 * @date 2024-04-16
 * @author hhryu
 */
double CTaskAvoidCharger::adjustSpeedToTarget(double currentSpeed, double targetSpeed, double acceleration)
{
    if (abs(currentSpeed - targetSpeed) > std::numeric_limits<double>::epsilon()) // hhryu240416 : currentSpeed != targetSpeed를 부동소수점에 대한 예외처리.
    {
        if (currentSpeed < targetSpeed)
        {
            currentSpeed = std::min(currentSpeed + acceleration, targetSpeed);
        }
        else if (currentSpeed > targetSpeed)
        {
            currentSpeed = std::max(currentSpeed - acceleration, targetSpeed);
        }
    }
    else
    {
        currentSpeed = targetSpeed;
    }
    return currentSpeed;
}
/**
 * @brief 벽타기중 크래들 회피 시작 조건, 방향에 따라 다른 조건을 가진다.
 * 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
u8 CTaskAvoidCharger::getLongSignalMatchedByShortSignal(u8 shortSignal)
{
    u8 ret = 0;
    if (shortSignal == SIGNAL_RIGHT_SIDE_SHORT)       ret = SIGNAL_RIGHT_SIDE_LONG;
    else if (shortSignal == SIGNAL_LEFT_SIDE_SHORT)   ret = SIGNAL_LEFT_SIDE_LONG;
    ELSE_ERROR

    return ret;
}

/**
 * @brief 벽타기중 크래들 회피 시작 조건, 방향에 따라 다른 조건을 가진다.
 * 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
u8 CTaskAvoidCharger::getAvoidCharegerSignalData(E_WALLTRACK_DIR dir)
{
    u8 ret = 0;
    if (dir == E_WALLTRACK_DIR::RIGHT)       ret = SIGNAL_RIGHT_SIDE_SHORT;
    else if (dir == E_WALLTRACK_DIR::LEFT)   ret = SIGNAL_LEFT_SIDE_SHORT;
    ELSE_ERROR

    return ret;
}

/**
 * @brief 벽타기중 크래들 회피 시작 조건, 방향에 따라 다른 조건을 가진다.
 * 
 * @param dir 
 * @return CHARGER_AVOID_STATE 
 * 
 * @note 연산시간 ms
 * @date 2024-04-17
 * @author hhryu
 */
bool CTaskAvoidCharger::isStartChargerAvoid(u8 avoidSignal)
{
    bool ret = false;
    u8 checkLongSignalData = getLongSignalMatchedByShortSignal(avoidSignal);
    if (ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, avoidSignal) + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, avoidSignal) >= 2)
    {
        if (ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) == ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, avoidSignal) + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, checkLongSignalData)
        && ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING) == ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, avoidSignal) + ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, checkLongSignalData))
        {
            ServiceData.signal.debugSignalPrint("크래들 장애물이 감지 되었습니다.");
            ret = true;
        }
    }

    return ret;
}

tTwist CTaskAvoidCharger::getCalulateMoveCenter(E_WALLTRACK_DIR dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    u8 receiver = IDX_RECEIVER_VOID, longSignal = SIGNAL_BLANK;
    if (walltrackDir == E_WALLTRACK_DIR::RIGHT)
    {
        receiver = IDX_RECEIVER_SIDE_RIGHT;
    }
    else if (walltrackDir == E_WALLTRACK_DIR::LEFT)
    {
        receiver = IDX_RECEIVER_SIDE_LEFT;
    } ELSE_ERROR
    // hhryu240416 : 만약 프론트 리시버 or 반대편 사이드 리시버가 신호를 감지한다면 반대로 회전하는 예외 처리를 넣어야함(우선 순위 최상위).
    // 천천히 돌아도 사이드리시버가 신호를 놓칠 수 있기 때문. 사이드가 좀 더 못보는 감이 있다.
    if(ServiceData.signal.countSignalDetected(receiver, SIGNAL_SHORT_ANYTHING)) // 숏을 볼 때 직선주행
    {
        curVel.v += CONFIG.signalTrack_accel_V;
        curVel.w = adjustSpeedToTarget(curVel.w, 0, CONFIG.signalTrack_decel_W);

        if(curVel.v >= CONFIG.signalTrack_max_V) curVel.v = CONFIG.signalTrack_max_V;
    }
    else if (ServiceData.signal.countSignalDetected(receiver, SIGNAL_LONG_ANYTHING)) // 롱만 볼 때 커브
    {
       curVel.v = adjustSpeedToTarget(curVel.v, CONFIG.signalTrack_max_V, CONFIG.signalTrack_accel_V);
       if(walltrackDir == E_WALLTRACK_DIR::LEFT)    curVel.w = adjustSpeedToTarget(curVel.w, CONFIG.signalTrack_max_W, CONFIG.signalTrack_accel_W);
       else                                         curVel.w = adjustSpeedToTarget(curVel.w, -CONFIG.signalTrack_max_W, CONFIG.signalTrack_accel_W);
    }
    // else // 아무 신호도 없을 때 많이 휘는 커브
    // {
    //     curVel.v -= CONFIG.signalTrack_accel_V * 0.5;
    //     if(curVel.v <= 0.065) curVel.v = 0.065;
    //     curVel.w -= CONFIG.signalTrack_accel_W * 1.8;
    //     if (abs(curVel.w) >= 1.3) curVel.w = std::copysign(1.3, curVel.w);
    // }
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "충전기 중앙으로 이동 중.... V : " << curVel.v << " W : " << curVel.w);
    return curVel;
}

tTwist CTaskAvoidCharger::getCalulateAcrossOppsite(E_WALLTRACK_DIR dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    u8 receiver = IDX_RECEIVER_VOID, longSignal = SIGNAL_BLANK;
    if (walltrackDir == E_WALLTRACK_DIR::RIGHT)
    {
        receiver = IDX_RECEIVER_SIDE_RIGHT;
        longSignal = SIGNAL_LEFT_SIDE_LONG;
    }
    else if (walltrackDir == E_WALLTRACK_DIR::LEFT)
    {
        receiver = IDX_RECEIVER_SIDE_LEFT;
        longSignal = SIGNAL_RIGHT_SIDE_LONG;
    } ELSE_ERROR

    if (walltrackDir == E_WALLTRACK_DIR::RIGHT)
    {
        receiver = IDX_RECEIVER_SIDE_RIGHT;
        longSignal = SIGNAL_LEFT_SIDE_LONG;
    }
    else if (walltrackDir == E_WALLTRACK_DIR::LEFT)
    {
        receiver = IDX_RECEIVER_SIDE_LEFT;
        longSignal = SIGNAL_RIGHT_SIDE_LONG;
    } ELSE_ERROR
    // hhryu240416 : 만약 프론트 리시버 or 반대편 사이드 리시버가 신호를 감지한다면 반대로 회전하는 예외 처리를 넣어야함(우선 순위 최상위).
    // 천천히 돌아도 사이드리시버가 신호를 놓칠 수 있기 때문. 사이드가 좀 더 못보는 감이 있다.
    if(ServiceData.signal.countSignalDetected(receiver, SIGNAL_SHORT_ANYTHING)) // 숏을 볼 때 직선주행
    {
        curVel.v += CONFIG.signalTrack_accel_V;
        curVel.w = adjustSpeedToTarget(curVel.w, 0, CONFIG.signalTrack_decel_W);

        if(curVel.v >= CONFIG.signalTrack_max_V) curVel.v = CONFIG.signalTrack_max_V;
    }
    else if (ServiceData.signal.countSignalDetected(receiver, SIGNAL_LONG_ANYTHING)) // 롱만 볼 때 커브
    {
       curVel.v = adjustSpeedToTarget(curVel.v, CONFIG.signalTrack_max_V, CONFIG.signalTrack_accel_V);
       if(walltrackDir == E_WALLTRACK_DIR::LEFT)    curVel.w = adjustSpeedToTarget(curVel.w, CONFIG.signalTrack_max_W, CONFIG.signalTrack_accel_W);
       else                                         curVel.w = adjustSpeedToTarget(curVel.w, -CONFIG.signalTrack_max_W, CONFIG.signalTrack_accel_W);
    }
    // else // 아무 신호도 없을 때 많이 휘는 커브
    // {
    //     curVel.v -= CONFIG.signalTrack_accel_V * 0.5;
    //     if(curVel.v <= 0.065) curVel.v = 0.065;
    //     curVel.w -= CONFIG.signalTrack_accel_W * 1.8;
    //     if (abs(curVel.w) >= 1.3) curVel.w = std::copysign(1.3, curVel.w);
    // }
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "충전기 회피 반대 편으로 이동 중.... V : " << curVel.v << " W : " << curVel.w);
    return curVel;
}

void CTaskAvoidCharger::motionMovingCenter(E_WALLTRACK_DIR dir)
{
    ServiceData.motionInfo.desVel = getCalulateMoveCenter(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CTaskAvoidCharger::motionAcrossOppsite(E_WALLTRACK_DIR dir)
{
    ServiceData.motionInfo.desVel = getCalulateAcrossOppsite(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}