/**
 * @file signaltracking.cpp
 * @author hhryu (hhryu@everybot.net)
 * @brief
 * @version 0.1
 * @date 2023-08-22
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "signaltracking.h"
#include "eblog.h"
#include "systemTool.h"
#include "control/motionPlanner/motionPlanner.h"
#include "MessageHandler.h"
#include "kinematics.h"

#define SG ServiceData.signal

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0           // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)                                     \
    {                                                            \
        printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time); \
    }                                                            \
    0
/******************************************************/

/**
 * @brief Construct a new CSignaltracking::CSignaltracking object
 *
 * @note 연산시간 ms
 * @date 2023-08-22
 * @author hhryu
 */
CSignaltracking::CSignaltracking() : CAvoiding()
{
    CStopWatch __debug_sw;
    eblog(LOG_LV, "");
    signalTrackInit(tPose());
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CSignaltracking::CSignaltracking object
 *
 *
 * @note 연산시간 ms
 * @date 2023-08-22
 * @author hhryu
 */
CSignaltracking::~CSignaltracking()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV, "");

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 시그널트랙킹에서 스텝에 따라 제어함
 *
 * @param location
 * @param robotPose
 * @param signalData
 * @param pObstacle
 * @return true
 * @return false
 *
 * @note 연산시간 0.4ms ~ 2.6ms
 * @date 2023-08-08
 * @author hhryu
 */
bool CSignaltracking::runSignalTracking(tDockingData dockingData_, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    bool ret = false;

    switch (getSignalTrackStep())
    {
    case E_SIGNALTRACK_STEP::VOID:
        setSignalTrackStep(E_SIGNALTRACK_STEP::INIT);
        break;
    case E_SIGNALTRACK_STEP::INIT:
        setSignalTrackStep(signalTrackInit(robotPose)); //(location,signalData)
        break;
    case E_SIGNALTRACK_STEP::SWIMMING:
        setSignalTrackStep(swimmingHandler(dockingData_.location, robotPose, pObstacle));
        break;
    case E_SIGNALTRACK_STEP::MOVING:
        setSignalTrackStep(moveCenterHandler(dockingData_.location, robotPose, pObstacle));
        break;
    case E_SIGNALTRACK_STEP::TRYDOCK:
        setSignalTrackStep(trydockHandler(dockingData_, robotPose, pObstacle));
        break;
    case E_SIGNALTRACK_STEP::COMPLETE:
    case E_SIGNALTRACK_STEP::FAIL:
        ret = true;
        break;
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 시그널 트래킹 시작
 *
 * @param 
 *
 * @note 연산시간 0.8ms
 * @date 2023-08-22
 * @author hhryu
 */
void CSignaltracking::startSignalTracking()
{
    CStopWatch __debug_sw;
    signaltrackData_.bIsSignalTracking = true;
    setSignalTrackStep(E_SIGNALTRACK_STEP::INIT);
    TIME_CHECK_END(__debug_sw.getTime());
    return;
}

/**
 * @brief set SignalTrack Step
 *
 * @param set
 *
 * @note 연산시간 < 0.1ms
 * @date 2023-08-22
 * @author hhryu
 */
void CSignaltracking::setSignalTrackStep(E_SIGNALTRACK_STEP set)
{
    CStopWatch __debug_sw;

    if (signaltrackData_.trackStep != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), WHITE, "E_SIGNALTRACK_STEP Change " << CYN << "[" << enumToString(signaltrackData_.trackStep) << "] --> [" << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
    }
    signaltrackData_.trackStep = set;
    swimingTemp_w = 0;
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief get SignalTrack Step
 *
 * @return E_SIGNALTRACK_STEP
 *
 * @note 연산시간 < 0.1ms
 * @date 2023-08-22
 * @author hhryu
 */
E_SIGNALTRACK_STEP CSignaltracking::getSignalTrackStep(void)
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return signaltrackData_.trackStep;
}

/**
 * @brief 초기화
 *
 * @param location
 * @return E_SIGNALTRACK_STEP
 *
 * @note 연산시간 < 0.1ms
 * @date 2023-08-22
 * @author hhryu
 */
E_SIGNALTRACK_STEP CSignaltracking::signalTrackInit(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_SIGNALTRACK_STEP ret = E_SIGNALTRACK_STEP::INIT;

    signaltrackData_.bSearchSignal = false;
    signaltrackData_.turnTowardCradle = 0; // -32767 ~ 32766 각도 저장용
    signaltrackData_.bCheckAvoidBumperPending = false;
    signaltrackData_.signalCheckShake = false;
    signaltrackData_.lostSignalTick = std::numeric_limits<u32>::max();
    signaltrackData_.bCheckSignal = false;
    signaltrackData_.temporaryMoveCenterStep = 0;
    signaltrackData_.rightSideLastSignalCheckAngle = invalidDoubleValue;
    signaltrackData_.leftSideLastSignalCheckAngle = invalidDoubleValue;
    signaltrackData_.swimmingTimeOut = 0;
    signaltrackData_.trydockTimeOut = 0;
    signaltrackData_.movingTimeOut = 0;
    signaltrackData_.trydockSpeedEqualTick = std::numeric_limits<s32>::max();
    signaltrackData_.bUnknownControl = false;
    signaltrackData_.leftSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);
    signaltrackData_.rightSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);
    signaltrackData_.swimScanCheckCount = 0;
    signaltrackData_.bTrydockEscape = false;
    signaltrackData_.bSignaltrackEscape = false;
    signaltrackData_.tryDockControlTime = invalidDoubleValue;
    signaltrackData_.checkSigStep = E_CHECK_SIG_STEP::IDLE;
    signaltrackData_.bLostSignal = false;
    signaltrackData_.bWallFallowing = false;
    setTrydockStep(E_TRYDOCK_STEP::VOID);
    setMovingCenterStep(E_MOVING_CENTER_STEP::VOID);
    setScanControlStep(E_SCAN_CONTROL_STEP::VOID);
    setSwimmingControlStep(E_SWIMMING_CONTROL_STEP::VOID);

    if((SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)   && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))
    || (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG)    && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG))
    {
        ret = E_SIGNALTRACK_STEP::TRYDOCK;
        SG.debugSignalPrint("Try dock START!!");
    }
    else if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT) 
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_SHORT))
    {
        s16 diff = SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT) - SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT);
        if (diff > 0)
        {
            saveLastSignalPose(robotPose, E_AREA::LEFT);
        }
        else if (diff < 0)
        {
            saveLastSignalPose(robotPose, E_AREA::RIGHT);
        }
        else
        {
            saveLastSignalPose(robotPose, E_AREA::LEFT);
            ceblog(LOG_LV_NECESSARY, RED, "moving으로 보내는 중 양 쪽 사이드에 신호 left side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT) << "] right side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT) <<"]");
        }
        ceblog(LOG_LV_DOCKING, BOLDBLUE, "왼쪽 사이드 숏 횟수 : " << SC<int>(SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT)) << "\t오른쪽 사이드 숏 횟수 : " << SC<int>(SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT)));
        SG.debugSignalPrint("Move Center START!!");
        ret = E_SIGNALTRACK_STEP::MOVING;
    }
    else
    {
        SG.debugSignalPrint("Swimming START!!");
        ret = E_SIGNALTRACK_STEP::SWIMMING;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CSignaltracking::setSwimmingControlStep(E_SWIMMING_CONTROL_STEP set)
{
    CStopWatch __debug_sw;

    if (signaltrackData_.swimmingControlStep != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_DOCKING | LOG_LV_NECESSARY), WHITE, "E_SWIMMING_STEP CHANGE " << CYN << "[" << enumToString(signaltrackData_.swimmingControlStep) << "] --> [" << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
        signaltrackData_.swimmingControlStep = set;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::getSwimmingControlStep()
{
    return signaltrackData_.swimmingControlStep;
}

/**
 * @brief swimming 위치로 판단될 때 타는 함수
 *
 * @param location
 * @param robotPose
 * @param signalData
 * @param pObstacle
 * @return E_SIGNALTRACK_STEP
 *
 * @note 연산시간 0.2ms ~ 5.5ms
 * @date 2023-08-08
 * @author hhryu
 */
E_SIGNALTRACK_STEP CSignaltracking::swimmingHandler(tLocation location, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    E_SIGNALTRACK_STEP ret = E_SIGNALTRACK_STEP::SWIMMING; // COMPLETE면 swimming 종료 --> signaltrack 종료 --> docking에서 다음 무엇을 할지 처리.

    if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT))
    {
        setSwimmingControlStep(E_SWIMMING_CONTROL_STEP::CHANGE_MOVING_CENTER);
        SG.debugSignalPrint("Swimming에서 side short이 들어와서 moving으로 전환");
    }
    else if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG) || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT))
    {
        setSwimmingControlStep(E_SWIMMING_CONTROL_STEP::COMPLETE);
        SG.debugSignalPrint();
    }

    switch (getSwimmingControlStep())
    {
    case E_SWIMMING_CONTROL_STEP::VOID:
        setSwimmingControlStep(swimmingVoidControl(robotPose));
        // 일단 멈춤
        // 벡터 초기화하고
        // 신호 체크
        // 있으면 우회전 하면서 TURNING_TO_LOST_SIDE
        break;
    case E_SWIMMING_CONTROL_STEP::CHECK_SIGNAL:
        if (checkSignalControl(robotPose))
        {
            setSwimmingControlStep(E_SWIMMING_CONTROL_STEP::VOID);
            // setSwimmingControlStep(E_SWIMMING_CONTROL_STEP::FAIL);
        }
        break;
    case E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE:
        setSwimmingControlStep(swimmingTurningToLostSideControl(robotPose));
        // 큐에 신호 없어지면 멈춤.
        // 벡터 초기화 하고
        // 신호 체크
        // 있으면 우회전
        // 없으면 좌회전 하면서 TURNING_TO_REACQUISITION
        break;
    case E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION:
        setSwimmingControlStep(swimmingTurningToReacquisitionControl(robotPose));
        // 큐에 신호 들어오면 멈춤.
        // 벡터 초기화 하고
        // 신호 체크
        // 있으면 직진하면서 FORWARD_TO_SHORT_AREA
        // 없으면 좌회전 하면서 TURNING_TO_REACQUISITION
        break;
    case E_SWIMMING_CONTROL_STEP::FORWARD_TO_SHORT_AREA:
        setSwimmingControlStep(swimmingForwardToShortAreaSideControl(robotPose));
        // 숏 들어오면 stop후 FORWARD_TO_CENTER
        // 큐에 신호가 없어졋으면?
        // 멈춰서 벡터 초기화하고 신호 체크
        // 있으면 직진 재개
        // 그렇게해도 없으면.... 좌회전하면서 TURNING_TO_REACQUISITION
        break;
    // case E_SWIMMING_CONTROL_STEP::CHANGE_TRY_DOCK:
    //     break;
    case E_SWIMMING_CONTROL_STEP::CHANGE_MOVING_CENTER:
        ret = E_SIGNALTRACK_STEP::MOVING;
        break;
    case E_SWIMMING_CONTROL_STEP::COMPLETE:
        ret = E_SIGNALTRACK_STEP::TRYDOCK;
        break;
    case E_SWIMMING_CONTROL_STEP::FAIL:
        ret = E_SIGNALTRACK_STEP::FAIL;
        break;
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::swimmingVoidControl(tPose robotPose)
{
    E_SWIMMING_CONTROL_STEP ret = E_SWIMMING_CONTROL_STEP::VOID;
    CRobotKinematics k;

    if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
    {
        
    }
    else
    {
        /* 일정 시간동안 쌓인 signal 받는 부분. */
        E_CHECK_SIGNAL checkSigState = SG.getCheckSignalState();
        if (checkSigState == E_CHECK_SIGNAL::IDLE)
        {
            processSignalCheck(checkSigState, 0.7);
        }
        else if (checkSigState == E_CHECK_SIGNAL::END)
        {
            if(SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_LONG))
            {
                saveLastSignalPose(robotPose, E_AREA::LEFT);
                tProfile profile;
                profile.desAngVel = DEG2RAD(20);
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn right");
                ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
            }
            else if (SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_LONG)
            || SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_LONG))
            {
                saveLastSignalPose(robotPose, E_AREA::RIGHT);
                tProfile profile;
                profile.desAngVel = DEG2RAD(20);
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn left");
                ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
            }
            else if (SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_ANYTHING))
            {
                ceblog(LOG_LV_DOCKING, RED, "?????");
                ceblog(LOG_LV_DOCKING, RED, "?????");
                ceblog(LOG_LV_DOCKING, RED, "?????");
                ceblog(LOG_LV_DOCKING, RED, "?????");
                ceblog(LOG_LV_DOCKING, RED, "?????");
            }
            else
            {
                ceblog(LOG_LV_DOCKING, GREEN, "시그널 카운트가 안됐어요.");
                setCheckSigStep(E_CHECK_SIG_STEP::IDLE);
                ret = E_SWIMMING_CONTROL_STEP::CHECK_SIGNAL;
            }
            SG.clearSignalVector();
            SG.setCheckSignalState(E_CHECK_SIGNAL::IDLE);
        }
    }
    return ret;
}

void CSignaltracking::setCheckSigStep(E_CHECK_SIG_STEP set)
{
    if (set != signaltrackData_.checkSigStep)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), WHITE, "SIGNAL STEP CHANGE " << CYN << enumToString(signaltrackData_.checkSigStep) << "] --> [" << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
        signaltrackData_.checkSigStep = set;
    }
}

E_CHECK_SIG_STEP CSignaltracking::getCheckSigStep()
{
    return signaltrackData_.checkSigStep;
}

bool CSignaltracking::checkSignalControl(tPose robotPose)
{
    bool bRet = false;
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);

    signalCheck(robotPose);

    switch (getCheckSigStep())
    {
    case E_CHECK_SIG_STEP::IDLE:
        ceblog(LOG_LV_DOCKING, YELLOW, "check signal start");
        setCheckSigStep(E_CHECK_SIG_STEP::START);
        break;
    case E_CHECK_SIG_STEP::START:
        targetAng = k.rotation(robotPose, DEG2RAD(358));
        MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
        setCheckSigStep(E_CHECK_SIG_STEP::FULL_TURN);
        break;
    case E_CHECK_SIG_STEP::LEFT_QUARTER_TURN:
    case E_CHECK_SIG_STEP::RIGHT_QUARTER_TURN:
        if (!MOTION.isRunning())
        {
            setCheckSigStep(E_CHECK_SIG_STEP::START);
        }
        break;
    case E_CHECK_SIG_STEP::FULL_TURN:
        if (!MOTION.isRunning())
        {
            setCheckSigStep(E_CHECK_SIG_STEP::END);
            targetAng = k.rotation(robotPose, DEG2RAD(358));
            MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
        }
        break;
    case E_CHECK_SIG_STEP::END:
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
        }
        else
        {
            ceblog(LOG_LV_DOCKING, YELLOW, "check signal end");
            setCheckSigStep(E_CHECK_SIG_STEP::IDLE);
            bRet = true;
        }
        break;
    }

    return bRet;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::sideLostControl(tPose robotPose)
{
    E_SWIMMING_CONTROL_STEP ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
    E_CHECK_SIGNAL checkSigState = SG.getCheckSignalState();
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);

    if (MOTION.isRunning()
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING)
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
            ceblog(LOG_LV_DOCKING, GREEN, "stop");
            if (checkSigState == E_CHECK_SIGNAL::IDLE)
            {
                processSignalCheck(checkSigState, 0.7);
            }
        }        
        // ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
    }
    else
    {
        if (checkSigState == E_CHECK_SIGNAL::END)
        {
            if (SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) || SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG))
            {
                saveLastSignalPose(robotPose, E_AREA::LEFT);
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn right");
                ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
            }
            else if (SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG) || SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))
            {
                saveLastSignalPose(robotPose, E_AREA::RIGHT);
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn left");
                ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
            }
            else
            {
                ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
                ceblog(LOG_LV_DOCKING, GREEN, "시그널 카운트가 안됐어요.");
            }
            SG.debugTriggerSignalPrint("sideLostControl Signal Check END");
            SG.clearSignalVector();
            SG.setCheckSignalState(E_CHECK_SIGNAL::IDLE);
        }
    }

    return ret;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::swimmingTurningToLostSideControl(tPose robotPose)
{
    E_SWIMMING_CONTROL_STEP ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE;
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);

    if (getLastSignal() == E_AREA::LEFT)
    {
        if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) == 0)
        {
            ret = sideLostControl(robotPose);
        }
        else
        {
            if (!MOTION.isRunning())
            {
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn right, 신호를 잃을 때 까지 우회전.... 돌았는데도 있는 상황.");
            }
        }
    }
    else if (getLastSignal() == E_AREA::RIGHT)
    {
        if (SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG) == 0)
        {
            ret = sideLostControl(robotPose);
        }
        else
        {
            if (!MOTION.isRunning())
            {
                targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                ceblog(LOG_LV_DOCKING, GREEN, "turn left");
                // ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
            }
        }
    }

    return ret;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::swimmingTurningToReacquisitionControl(tPose robotPose)
{
    E_SWIMMING_CONTROL_STEP ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
    
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);

    if (getLastSignal() == E_AREA::LEFT)
    {
        if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG))
        {
            if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
            {
                
                ceblog(LOG_LV_DOCKING, GREEN, "swim중 신호 재탐색, 감지 성공.");
            }
            else
            {
                ret = E_SWIMMING_CONTROL_STEP::FORWARD_TO_SHORT_AREA;
            }
        }
        else
        {
            targetAng = k.rotation(robotPose, DEG2RAD(10));
            MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
            ceblog(LOG_LV_DOCKING, GREEN, "turn left 신호를 다시 찾기 위해");
        }
    }
    else if (getLastSignal() == E_AREA::RIGHT)
    {
        if (SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))
        {
            if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
            {
                
            }
            else
            {
                ret = E_SWIMMING_CONTROL_STEP::FORWARD_TO_SHORT_AREA;
            }
        }
        else
        {
            targetAng = k.rotation(robotPose, DEG2RAD(10));
            MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
            ceblog(LOG_LV_DOCKING, GREEN, "turn right 신호를 다시 찾기 위해");
        }
    }

    return ret;
}

E_SWIMMING_CONTROL_STEP CSignaltracking::swimmingForwardToShortAreaSideControl(tPose robotPose)
{
    E_SWIMMING_CONTROL_STEP ret = E_SWIMMING_CONTROL_STEP::FORWARD_TO_SHORT_AREA;
    tProfile profile;
    profile.desAngVel = DEG2RAD(10);
    if(SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG) 
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT))
    {
        SG.debugTriggerSignalPrint("Center확인, Swimming종료.");
        ret = E_SWIMMING_CONTROL_STEP::COMPLETE;
    }
    else if ((SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_SHORT) 
    + SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_SHORT)
    + SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG)
    + SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT)
    + SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT)
    + SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT))
    >= 2)
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
        }
        else
        {
            ceblog(LOG_LV_DOCKING, MAGENTA, "Swim중 short 감지 됨, move center 실행");
            ret = E_SWIMMING_CONTROL_STEP::CHANGE_MOVING_CENTER;
        }
    }
    else
    {
        if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG)
        || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG)
        || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)
        || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))
        {
            if (MOTION.isRunning())
            {
                ceblog(LOG_LV_DOCKING, GREEN, "숏 영역으로 진입하기 위해 직진중");
            }
            else
            {
                if (MOTION.getControlType() != E_CONTROL_TYPE::LINEAR)
                {
                    // MOTION.startLinearOnVelocity(robotPose, 0.2, tProfile());
                }
                ceblog(LOG_LV_DOCKING, GREEN, "start linear 숏 영역으로 진입하기 위해");
            }
        }
        else
        {
            if (getLastSignal() == E_AREA::LEFT) // 오른쪽 신호가 보이면 반대로 트는 것을 추가하면 좋을 것 같다.
            {
                if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_LONG)
                || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_LONG))
                {
                    SG.debugTriggerSignalPrint("왼쪽 사이드가 오른쪽 신호 확인, Swimming종료.");
                    ret = E_SWIMMING_CONTROL_STEP::COMPLETE;
                }
                else
                {
                    ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
                    // if (MOTION.getControlType() == E_CONTROL_TYPE::LINEAR)
                    // {
                    //     MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                    //
                    //     // MOTION.startCurveVelocity(robotPose, 0.15, DEG2RAD(8), tProfile());
                    // }
                    // else
                    {
                        ceblog(LOG_LV_DOCKING, RED, "신호를 찾기 위해 회전중... ");
                    }
                }
            }
            else if (getLastSignal() == E_AREA::RIGHT)
            {
                if (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG)
                || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG))
                {
                    SG.debugTriggerSignalPrint("오른쪽 사이드가 왼쪽 신호 확인, Swimming종료.");
                    ret = E_SWIMMING_CONTROL_STEP::COMPLETE;
                    // 여기서 SIDE면 MOVING으로 바꿔야하고, CENTER면 TRYDOCK으로 바꿔야함.
                }
                else
                {
                    ret = E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION;
                    // if (MOTION.getControlType() == E_CONTROL_TYPE::LINEAR)
                    // {
                    //     MOTION.startRotateVelocity(robotPose, -profile.desAngVel, profile);
                    //     // MOTION.startCurveVelocity(robotPose, 0.15, DEG2RAD(-8), tProfile());
                    // }
                    // else
                    {
                        ceblog(LOG_LV_DOCKING, RED, "신호를 찾기 위해 회전중... ");
                    }
                }
            }
        }
    }
    return ret;
}

void CSignaltracking::signalCheck(tPose robotPose)
{
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);

    if ((SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) + SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING)) >= 2)
    {
        setCheckSigStep(E_CHECK_SIG_STEP::END);
    }
    else
    {
        if (getCheckSigStep() == E_CHECK_SIG_STEP::START || getCheckSigStep() == E_CHECK_SIG_STEP::FULL_TURN)
        {
            if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
            {
                s16 diff = SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) - SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING);
                if (diff > 0)
                {
                    if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
                    {
                        
                    }
                    else
                    {
                        targetAng = k.rotation(robotPose, DEG2RAD(100));
                        MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
                        setCheckSigStep(E_CHECK_SIG_STEP::LEFT_QUARTER_TURN);
                    }
                }
                else if (diff < 0)
                {
                    if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
                    {
                        
                    }
                    else
                    {
                        targetAng = k.rotation(robotPose, DEG2RAD(100));
                        MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
                        setCheckSigStep(E_CHECK_SIG_STEP::RIGHT_QUARTER_TURN);
                    }
                }
                else
                {
                    ceblog(LOG_LV_NECESSARY, RED, "시그널 체크 중 양 쪽 사이드에 신호 left side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) << "] right side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING) <<"]");
                }
            }
        }
    }

    return;
}

void CSignaltracking::saveLastSignalPose(tPose robotPose, E_AREA area)
{
    switch (area)
    {
        case E_AREA::CENTER:
        case E_AREA::UNKNOWN:
            signaltrackData_.leftSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);
            signaltrackData_.rightSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);        
            break;
        case E_AREA::LEFT:
            signaltrackData_.leftSwimmingPose = robotPose;
            signaltrackData_.rightSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);        
            break;
        case E_AREA::RIGHT:
            signaltrackData_.leftSwimmingPose = tPose(0.0, 0.0, invalidDoubleValue);
            signaltrackData_.rightSwimmingPose = robotPose;        
            break;
        defalut:
            break;
    }
}

E_AREA CSignaltracking::getLastSignal()
{
    E_AREA ret = E_AREA::NONE;

    if (!std::isinf(signaltrackData_.rightSwimmingPose.angle))
    {
        ret = E_AREA::RIGHT;
    }
    else if (!std::isinf(signaltrackData_.leftSwimmingPose.angle))
    {
        ret = E_AREA::LEFT;
    }
    else
    {
        ret = E_AREA::CENTER;
    }

    return ret;    
}
E_TRYDOCK_STEP CSignaltracking::tryDockRotateTowardsCradle(tPose robotPose)
{
    E_TRYDOCK_STEP ret = E_TRYDOCK_STEP::LINE_UP;
    tProfile profile;
    CRobotKinematics k;
    double targetAngle = SG.buildTryDockAngle();
    s16 diff = 0;

    if (utils::isMaxValue(targetAngle) || std::isinf(targetAngle) || fabs(targetAngle) > 30/*40*/) // abs40초과 :리시버 두개 중 하나만 들어와 많이 틀어짐을 의미하는 각도임.)
    {
        diff = SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) - SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING);
        if (utils::isMaxValue(targetAngle) || std::isinf(targetAngle))
        {
            if (diff > 0) // 왼쪽 사이드 감지
            {
                profile.desAngVel = MOTION_CONTROLLER_DESIRED_W;
                //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                ceblog(LOG_LV_DOCKING, CYN, "정면 리시버가 신호를 찾을 때 까지 좌회전. degree:" << profile.desAngVel <<" diff : "<<diff);
            }
            else if (diff < 0) // 오른쪽 사이드 감지
            {
                profile.desAngVel = -MOTION_CONTROLLER_DESIRED_W;
                //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                ceblog(LOG_LV_DOCKING, CYN, "정면 리시버가 신호를 찾을 때 까지 우회전. degree:" << profile.desAngVel <<" diff:" << diff);
            }
            else
            {
                if (!MOTION.isRunning())
                {
                    profile.desAngVel = -MOTION_CONTROLLER_DESIRED_W;
                    //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                }
                ceblog((LOG_LV_DOCKING | LOG_LV_NECESSARY | LOG_LV_ERROR), WHITE, "정면 리시버가 신호를 찾을 때 까지 회전.... 양 쪽 사이드가 다 들어옴 또는 안들어옴 " << -MOTION_CONTROLLER_DESIRED_W <<"degree, LEFT SIDE : " << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING)) << " RIGHT SIDE : " << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)));
                // hhryu240215 : 이 곳에 예외 처리를 꼭 넣어야 한다, 또는 체크시그널으로 변경해야한다.
            }
        }
        else
        {
            if (targetAngle > 0) // 왼쪽 리시버만 감지
            {
                profile.desAngVel = DEG2RAD(8);
                //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                ceblog(LOG_LV_DOCKING, CYN, "정면 리시버가 신호를 찾을 때 까지 좌회전. degree:" << profile.desAngVel <<" diff:"<<diff);
            }
            else if (targetAngle < 0) // 오른쪽 리시버만 감지
            {
                profile.desAngVel = -DEG2RAD(8);
                //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
                ceblog(LOG_LV_DOCKING, CYN, "정면 리시버가 신호를 찾을 때 까지 우회전. degree:" << profile.desAngVel <<" diff:"<<diff);
            }
        }
    }
    else
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
        }
        else
        {
            profile.desAngVel = DEG2RAD(targetAngle);
            MOTION.startRotation(robotPose,targetAngle,profile,E_ROTATE_DIR::CCW);
            ceblog(LOG_LV_DOCKING, CYN, "stop 디버그 용 " << SYSTEM_TOOL.getSystemTime());
            ret = E_TRYDOCK_STEP::CHECK_SLAM;
        }
    }
    ceblog(LOG_LV_DOCKING, MAGENTA, "각도[" << targetAngle << "] diff[" << diff << "] ");
    return ret;
}

/**
 * @brief 크래들 위로 판단될 때 충전 단자로 진입 시도하는 핸들러.
 *
 * @param location
 * @param robotPose
 * @param signalData
 * @param pObstacle
 * @return E_SIGNALTRACK_STEP
 *
 * @note 연산시간 0.1ms ~ 0.5ms
 * @date 2023-08-22
 * @author hhryu
 */
E_SIGNALTRACK_STEP CSignaltracking::trydockHandler(tDockingData dockingData_, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    E_SIGNALTRACK_STEP ret = E_SIGNALTRACK_STEP::TRYDOCK; // COMPLETE면 trydock 종료 --> signaltrack 종료 --> docking에서 다음 무엇을 할지 처리.
    tAction action;

    switch (getTrydockStep())
    {
    case E_TRYDOCK_STEP::VOID:
        setTrydockStep(E_TRYDOCK_STEP::GO_NEAR);
        break;
    case E_TRYDOCK_STEP::LINE_UP:
        setTrydockStep(tryDockRotateTowardsCradle(robotPose));
        break;
    case E_TRYDOCK_STEP::CHECK_SLAM:
        setTrydockStep(tryDockCheckSlam());
        break;        
    case E_TRYDOCK_STEP::CHECK_TILT:
        setTrydockStep(tryDockCheckTilt());
        break;
    case E_TRYDOCK_STEP::GO_NEAR:
        setTrydockStep(tryDockGoNear());
        break;
    case E_TRYDOCK_STEP::ENTRANCE:
        ret = trydockEntrance();
        break;
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_TRYDOCK_STEP CSignaltracking::tryDockGoNear()
{
    E_TRYDOCK_STEP ret = E_TRYDOCK_STEP::GO_NEAR;
    if(SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_SHORT))
    {
        ret = E_TRYDOCK_STEP::LINE_UP;
    }
    else
    {
        trydockEntrance();
    }
    return ret;
}

E_TRYDOCK_STEP CSignaltracking::tryDockCheckSlam()
{
    E_TRYDOCK_STEP ret = E_TRYDOCK_STEP::CHECK_SLAM;
    if(!MOTION.isRunning() || MOTION.getControlType() == E_CONTROL_TYPE::STOP)
    {
        if (ROBOT_CONTROL.slam.isSlamRunning())
        {
            ROBOT_CONTROL.slam.exitSlam();
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), CYN, "도킹 중 충전 및 틸 업 전 EXIT SLAM!!!");
        }
        else
        {
            ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), CYN, "도킹 중 충전 및 틸 업 전 EXIT SLAM이 완료되어 Check tilt로 이동");
            ret = E_TRYDOCK_STEP::CHECK_TILT;
        }
    }
    return ret;
}

E_TRYDOCK_STEP CSignaltracking::tryDockCheckTilt()
{
    CStopWatch __debug_sw;
    E_TRYDOCK_STEP ret = E_TRYDOCK_STEP::CHECK_TILT;
    tAction action;

    if (ROBOT_CONTROL.getTiltStep() == E_TILT_STEP::VOID)
    {
        if(ServiceData.tilting.getStateValue() == E_SYS_TILT_STATE::TILTED_UP)
        {
            ret = E_TRYDOCK_STEP::ENTRANCE;
        }
        else
        {
            ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
            ceblog((LOG_LV_DOCKING), BLUE, "~~TRY DOCK을 위해 TILT UP START~~ state [" << enumToString(ServiceData.tilting.getStateValue()) << "] time["<<SYSTEM_TOOL.getSystemTime()<<"]");
        }
    }
    else if (!MOTION.isRunning())
    {
        ret = E_TRYDOCK_STEP::ENTRANCE;
    }

    return ret;
}

E_TRYDOCK_STEP CSignaltracking::getTrydockStep()
{
    return signaltrackData_.trydockStep;
}

void CSignaltracking::setTrydockStep(E_TRYDOCK_STEP set)
{
    if (signaltrackData_.trydockStep != set)
    {
        SG.debugSignalPrint();
        ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), WHITE, "E_TRYDOCK_STEP CHANGE " << CYN << "[" << enumToString(signaltrackData_.trydockStep) << "-->" << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
        signaltrackData_.trydockStep = set;   
    }
}

E_SIGNALTRACK_STEP CSignaltracking::trydockEntrance()
{
    CStopWatch __debug_sw;
    E_SIGNALTRACK_STEP ret = E_SIGNALTRACK_STEP::TRYDOCK;
    tProfile profile;
    profile.desLinVel = 0.1; // 0.13; // 0.25;
    profile.desAngVel = DEG2RAD(15); //DEG2RAD(20);
    double targetAngle = SG.buildTryDockAngle();
    tPose pose = ServiceData.localiz.getSysPose();
    if (ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILTED_UP && getTrydockStep() != E_TRYDOCK_STEP::GO_NEAR)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_ERROR|LOG_LV_DOCKING), RED, "충전기로 진입 중인데, 틸 업이 안되어있음. ERROR! 상태["<<enumToString(ServiceData.tilting.getStateValue())<<"]");
    }
    if (signaltrackData_.bLostSignal)
    {
        if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) || 
            SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT) || 
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT) ||
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT) || 
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT) || 
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT) ||
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) || 
            SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG) || 
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG) ||
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG) || 
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG) ||
            SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG) || 
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG) ||
            // SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG))
            (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT) && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT)) ||
            (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG) && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG)) ||
            (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT) && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT)) ||
            (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG) && SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG)))
        {
            if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
            {
                
                ceblog(LOG_LV_DOCKING, MAGENTA, "신호를 찾았어요 다시 충전기를 향해 가보자");
                signaltrackData_.bLostSignal = false;
            }
        }
        else if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        {
            rotateTowardsCradle(pose);
        }
    }
    else
    {
        // if 각도가 40도가 넘으면 회전을 시켜주는 방식을 재적용 해보자.
        // 전체적으로 여기 정리를 한번 해야할듯
        // 40도 넘거나 사이드만 보거나 inf max 모두 회전으로 바꾸고
        // 나머지는 그냥 각도에 따라 오로지 커브 제어만 하도록하면 깔끔하지 않을까?
        if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT)
        || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)  || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))
        {
            profile.desLinVel = 0.1;
            //MOTION.startCurveVelocity(pose, profile.desLinVel, DEG2RAD(targetAngle), profile);
        }
        else if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
        {
            if(fabs(targetAngle) >= 40) profile.desLinVel = 0.1;
            else                        profile.desLinVel = 0.15;
            //MOTION.startCurveVelocity(pose, profile.desLinVel, DEG2RAD(targetAngle), profile);
        }
        else if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        {
            rotateTowardsCradle(pose);
        }
        else
        {
            if (std::isinf(targetAngle) || utils::isMaxValue(targetAngle))
            {
                if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
                {
                    
                }
                else
                {
                    ceblog(LOG_LV_DOCKING, MAGENTA, "신호를 잃었어요 신호를 찾아 봅시다");
                    signaltrackData_.bLostSignal = true;
                    profile.desAngVel = DEG2RAD(20);
                    //MOTION.startRotateVelocity(pose,profile.desAngVel,profile);
                }                
            }
            else
            {
                if(fabs(targetAngle) >= 40) profile.desLinVel = 0.15;
                //MOTION.startCurveVelocity(pose, profile.desLinVel, DEG2RAD(targetAngle), profile);
            }
        }
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret; // 양쪽 속력 반환.
}

void CSignaltracking::rotateTowardsCradle(tPose robotPose)
{
    s16 diff = SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING) - SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING);
    tProfile profile;
    profile.desLinVel = 0.25;

    if (diff > 0)
    {
        ceblog(LOG_LV_DOCKING, MAGENTA, "왼쪽 사이드에 신호감지 돌면서 정면 맞추기 left cnt[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING)) << "]\tright cnt[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)) <<"]\tdiff[" << DEC(diff)  << "]");
        signaltrackData_.bLostSignal = true;
        profile.desAngVel = DEG2RAD(13);
        //MOTION.startRotateVelocity(robotPose,profile.desAngVel,profile);
    }
    else if (diff < 0)
    { 
        ceblog(LOG_LV_DOCKING, MAGENTA, "오른쪽 사이드에 신호감지 돌면서 정면 맞추기 left cnt[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING)) << "]\tright cnt[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)) <<"]\tdiff[" << DEC(diff) << "]");
        signaltrackData_.bLostSignal = true;
        profile.desAngVel = DEG2RAD(-13);
        //MOTION.startRotateVelocity(robotPose,profile.desAngVel,profile);
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, RED, "try dock중 양 쪽 사이드에 신호 left side[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING)) << "] right side[" << DEC(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)) <<"]\tdiff[" << DEC(diff) << "]");
    }

    return;
}

/**
 * @brief 사이드에서 센터로 가는 핸들러.
 *
 * @param location
 * @param robotPose
 * @param signalData
 * @param pObstacle
 * @return E_SIGNALTRACK_STEP
 *
 * @note 연산시간 ms
 * @date 2023-08-22
 * @author hhryu
 */
E_SIGNALTRACK_STEP CSignaltracking::moveCenterHandler(tLocation location, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    E_SIGNALTRACK_STEP ret = E_SIGNALTRACK_STEP::MOVING; // COMPLETE면 gotoCenter 종료 --> signaltrack 종료 --> docking에서 다음 무엇을 할지 처리.

    switch (getMovingCenterStep())
    {
    case E_MOVING_CENTER_STEP::VOID:
        setMovingCenterStep(movingCenterVoidControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::TURNING_TO_CENTER:
        setMovingCenterStep(movingCenterTurningToCenterControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::FORWARD_TO_CENTER:
        setMovingCenterStep(movingCenterForwardToCenterControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::TURNING_TO_CHARGER:
        setMovingCenterStep(movingCenterTunringToChargerControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::FORWARD_TO_CHARGER:
        // setMovingCenterStep(E_MOVING_CENTER_STEP::COMPLETE); // 임시
        setMovingCenterStep(movingCenterForwardToChargerControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::SCANNING:
        // setMovingCenterStep(movingCenterScanningControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::FAIL:
        // setMovingCenterStep(movingCenterFailControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::COMPLETE:
        if (!MOTION.isRunning())
        {
            ret = E_SIGNALTRACK_STEP::TRYDOCK;
        }
        // setMovingCenterStep(movingCenterCompleteControl(robotPose));
        break;
    
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 벽타기 도중 크래들을 만났을 시 탈출하는 Handler
 * 
 * @param robotPose 
 * @param pObstacle 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2024-02-29
 * @author hhryu
 */
bool CSignaltracking::wallTrackCradleAvoidHandler(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    bool bRet = false;
    const u16 checkWallIR = 300;
    signaltrackData_.bWallFallowing = true;

    if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_SHORT))
    {
        if((SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG) == SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        || (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_SHORT) == SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        || (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG) == SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        || (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_SHORT) == SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)))
        {
            //MOTION.startCurveVelocity(robotPose, 0.1, DEG2RAD(5), tProfile());
            setMovingCenterStep(E_MOVING_CENTER_STEP::COMPLETE);
            SG.debugSignalPrint("크래들 탈출 완료, 벽면을 향해 가자");
        }
    }
    else
    {
        if(ServiceData.obstacle.getObstacleData()->ir.center.raw_data > checkWallIR
        || ServiceData.obstacle.getObstacleData()->ir.left.raw_data > checkWallIR
        || ServiceData.obstacle.getObstacleData()->ir.right.raw_data > checkWallIR)
        {
            //MOTION.startCurveVelocity(robotPose, 0.1, DEG2RAD(5), tProfile());
            setMovingCenterStep(E_MOVING_CENTER_STEP::COMPLETE);
            SG.debugSignalPrint("크래들 탈출 완료, 벽면을 향해 가자");
        }
    }
    switch (getMovingCenterStep())
    {
    case E_MOVING_CENTER_STEP::VOID:
        setMovingCenterStep(movingCenterVoidControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::TURNING_TO_CENTER:
        setMovingCenterStep(movingCenterTurningToCenterControl(robotPose));
        break;
    case E_MOVING_CENTER_STEP::FORWARD_TO_CENTER:
        setMovingCenterStep(movingCenterForwardToCenterControl(robotPose));
        break;
    default:
        setMovingCenterStep(E_MOVING_CENTER_STEP::VOID);
        signaltrackData_.bWallFallowing = false;
        bRet = true;
        break;
    }

    return bRet;
}

/**
 * @brief void일 때 펜딩이 끝나면 Forward해줌
 *
 * @return E_MOVING_CENTER_STEP
 */
E_MOVING_CENTER_STEP CSignaltracking::movingCenterVoidControl(tPose robotPose)
{
    CStopWatch __debug_sw;

    E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::VOID;

    if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
    {
        
        ceblog(LOG_LV_DOCKING, YELLOW, "stop");
    }
    else
    {
        /* 일정 시간동안 쌓인 signal 받는 부분. */
        E_CHECK_SIGNAL checkSigState = SG.getCheckSignalState();
        if (checkSigState == E_CHECK_SIGNAL::IDLE)
        {
            processSignalCheck(checkSigState, 0.5);
        }
        else if (checkSigState == E_CHECK_SIGNAL::END)
        {
            if (SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT)) // 또는 LC,RC 두개 같이 들어올 때도 추가 필요
            {
                s16 diff = SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT) - SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT);

                if (SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT) 
                || SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))
                {
                    
                    ret = E_MOVING_CENTER_STEP::COMPLETE;
                }
                else if (SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT)
                || SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT))
                {
                    if (diff > 0)
                    {
                        saveLastSignalPose(robotPose, E_AREA::LEFT);
                        ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                    }
                    else if (diff < 0)
                    {
                        saveLastSignalPose(robotPose, E_AREA::RIGHT);
                        ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                    }
                    else
                    {
                        ceblog(LOG_LV_NECESSARY, RED, "move center중 양 쪽 사이드에 신호 left side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT) << "] right side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT) <<"]");
                    }
                }
                // else if (SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT))
                // {
                //     saveLastSignalPose(robotPose, E_AREA::LEFT);
                //     ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                // }
                // else if (SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))
                // {
                //     saveLastSignalPose(robotPose, E_AREA::RIGHT);
                //     ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                // }
            }
            else if (SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG))
            {
                s16 diff = SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG) - SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG);

                if (SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG) 
                || SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG))
                {
                    ret = E_MOVING_CENTER_STEP::COMPLETE;
                }
                else if (SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG)
                || SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))
                {
                    if (diff > 0)
                    {
                        saveLastSignalPose(robotPose, E_AREA::LEFT);
                        ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                    }
                    else if (diff < 0)
                    {
                        saveLastSignalPose(robotPose, E_AREA::RIGHT);
                        ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                    }
                    else
                    {
                        ceblog(LOG_LV_NECESSARY, RED, "move center중 양 쪽 사이드에 신호 left side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG) << "] right side[" << SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG) <<"]");
                    }
                }
                // else if (SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG))
                // {
                //     saveLastSignalPose(robotPose, E_AREA::LEFT);
                //     ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                // }
                // else if (SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))
                // {
                //     saveLastSignalPose(robotPose, E_AREA::RIGHT);
                //     ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
                // }
            }
            else
            {
                u32 leftCenter = SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) 
                + SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT) 
                + SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT)
                + SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT);
                u32 rightCenter = SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT) 
                + SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT) 
                + SG.countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT)
                + SG.countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT);
                s32 diff = std::numeric_limits<s32>::max();
                if (leftCenter || rightCenter)
                {
                    diff = leftCenter - rightCenter;
                }
                if (diff == std::numeric_limits<s32>::max())
                {
                    s16 sideLeft = SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG);
                    s16 sideRight = SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG)
                    + SG.countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG);
                    s16 diffSide = sideLeft - sideRight;

                    if (sideLeft || sideRight)
                    {
                        if (diffSide > 0)
                        {
                            saveLastSignalPose(robotPose, E_AREA::LEFT);
                            ret = E_MOVING_CENTER_STEP::FORWARD_TO_CENTER;
                        }
                        else if (diffSide < 0)
                        {
                            saveLastSignalPose(robotPose, E_AREA::RIGHT);
                            ret = E_MOVING_CENTER_STEP::FORWARD_TO_CENTER;
                        }
                        else
                        {
                            ceblog(LOG_LV_DOCKING, CYN, "");
                        }
                    }
                    else
                    {
                        s16 left = SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_SHORT)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_SHORT)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_LONG)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_SIDE_LONG);
                        s16 right = SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_SHORT)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_SHORT)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_LONG)
                        + SG.countSignalVector(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_SIDE_LONG);
                        if (left)
                        {
                            saveLastSignalPose(robotPose, E_AREA::LEFT);
                        }
                        else if (right)
                        {
                            saveLastSignalPose(robotPose, E_AREA::RIGHT);
                        }
                        else
                        {
                            // 실패요
                            ceblog(LOG_LV_DOCKING | LOG_LV_ERROR, RED, "no signal!!!!  LEFT["<<DEC(left)<<"] RIGHT["<<DEC(right)<<"]");
                        }
                        SG.debugSignalPrint("GO TO TURNING_TO_CENTER");
                        ret = E_MOVING_CENTER_STEP::TURNING_TO_CENTER;
                    }
                }
                else if (diff > 0)
                {
                    saveLastSignalPose(robotPose, E_AREA::LEFT);
                    ret = E_MOVING_CENTER_STEP::TURNING_TO_CENTER;
                }
                else if (diff < 0)
                {
                    saveLastSignalPose(robotPose, E_AREA::RIGHT);
                    ret = E_MOVING_CENTER_STEP::TURNING_TO_CENTER;
                }
                else // diff == 0
                {
                    
                    ret = E_MOVING_CENTER_STEP::COMPLETE;
                }
            }
            SG.clearSignalVector();
            SG.setCheckSignalState(E_CHECK_SIGNAL::IDLE);
            SG.debugTriggerSignalPrint("movingCenterVoidControl");
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 로봇이 Short영역일 때, center를 찾기 위해 회전하는 함수.
 * 
 * @param robotPose 
 * @return E_MOVING_CENTER_STEP 
 * 
 * @note 연산시간 ms
 * @date 2024-04-08
 * @author hhryu
 */
E_MOVING_CENTER_STEP CSignaltracking::movingCenterTurningToCenterControl(tPose robotPose)
{
    E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::TURNING_TO_CENTER;
    E_CHECK_SIGNAL checkSigState = SG.getCheckSignalState();
    tProfile profile;
    profile.desAngVel = DEG2RAD(15);
    switch (getLastSignal()) // 사이드로 볼 때 까지 회전
    {
    case E_AREA::LEFT:
        //MOTION.startRotateVelocity(robotPose, -1 * profile.desAngVel, profile);
        break;
    case E_AREA::RIGHT:
        //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
        break;
    default:
        ceblog((LOG_LV_DOCKING|LOG_LV_ERROR), RED, "미구현 부분, 방향이 없을 때 처리 추가 필요");
        break;
    }
    // 로봇의 사이드로 LC LS RC RS의 short을 감지하면 TURNING_TO_CENTER 종료.
    if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG))
    {
        
        saveLastSignalPose(robotPose, E_AREA::LEFT);
        ret = E_MOVING_CENTER_STEP::FORWARD_TO_CENTER;
    }
    if (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG))
    {
        
        saveLastSignalPose(robotPose, E_AREA::RIGHT);
        ret = E_MOVING_CENTER_STEP::FORWARD_TO_CENTER;
    }
    return ret;
}

void CSignaltracking::cradleEscape(tPose robotPose)
{
    tProfile profile;
    profile.desAngVel = DEG2RAD(20);
    if (getLastSignal() == E_AREA::LEFT 
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING)
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
    {
        profile.desAngVel = DEG2RAD(30);
        profile.desLinVel = 0.15;
        double v = 0.0;
        double w = 0.0;
        if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING))
        {
            v = 0.1;
            w = DEG2RAD(10);     
        }
        else
        {
            v = 0.1;
            w = DEG2RAD(20);
        }
        //MOTION.startCurveVelocity(robotPose, v, w, profile);
        ceblog(LOG_LV_DOCKING, BLUE, "(getLastSignal() == E_AREA::LEFT) " << enumToString(getLastSignal()) << " v:"<<v<<"\tw:"<<w);
    }
    else if (getLastSignal() == E_AREA::RIGHT 
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING)
    && !SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING))
    {
        profile.desAngVel = DEG2RAD(30);
        profile.desLinVel = 0.15;
        double v = 0.0;
        double w = 0.0;
        if (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        {
            v = 0.1;
            w = DEG2RAD(-10);
        }
        else
        {
            v = 0.1;
            w = DEG2RAD(-20);
        }
        //MOTION.startCurveVelocity(robotPose, v, w, profile);
        ceblog(LOG_LV_DOCKING, BLUE, "(getLastSignal() == E_AREA::RIGHT) " << enumToString(getLastSignal()) << " v:"<<v<<"\tw:"<<w);
    }
    else
    {
        SG.debugSignalPrint();
        ceblog(LOG_LV_DOCKING, RED, "센터 숏으로 이동 중인데 위치 정보가 없음.");
    }
}

bool CSignaltracking::isLeftSideReceiverLeftArea()
{
    bool bRet = false;
    if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG) 
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG))
    {
        bRet = true;
    }
    return bRet;
}

bool CSignaltracking::isRightSideReceiverRightArea()
{
    bool bRet = false;
    if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG))
    {
        bRet = true;
    }
    return bRet;
}

bool CSignaltracking::isSideReceiverCenterArea()
{
    bool bRet = false;
    if (getLastSignal() == E_AREA::LEFT)
    {
        if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_LONG)
        || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_LONG))
        {
            bRet = true;
        }
    }
    else if (getLastSignal() == E_AREA::RIGHT)
    {
        if(SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG)
        || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG))
        {
            bRet = true;
        }
    }
    if(SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))
    {
        bRet = true;
    }
    return bRet;
}

bool CSignaltracking::isFrontReceiverCenterArea()
{
    bool bRet = false;
    if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG))
    {
        bRet = true;
    }
    return bRet;
}

void CSignaltracking::forwardToCenterLeftArea(tPose robotPose, tProfile profile)
{
    if (isLeftSideReceiverLeftArea())
    {
        if (MOTION.getControlType() == E_CONTROL_TYPE::ROTATE)
        {
            
        }
        else
        {
            // MOTION.startLinearOnVelocity(robotPose, 0.17, tProfile());
        }
    }
    else if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT))
    {
        //MOTION.startRotateVelocity(robotPose, -profile.desAngVel, profile);
    }
    else
    {
        if (MOTION.getControlType() == E_CONTROL_TYPE::LINEAR)
        {
            //MOTION.startRotateVelocity(robotPose, profile.desAngVel, profile);
        }
    }
    return;
}

void CSignaltracking::forwardToCenterRightArea(tPose robotPose, tProfile profile)
{
    if (isRightSideReceiverRightArea())
    {
        if (MOTION.getControlType() == E_CONTROL_TYPE::ROTATE)
        {
            
        }
        else
        {
            // MOTION.startLinearOnVelocity(robotPose, 0.17, tProfile());
        }
    }
    else if (SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))
    {
        //MOTION.startRotateVelocity(robotPose, -profile.desAngVel, profile);
    }    
    else
    {
        if (MOTION.getControlType() == E_CONTROL_TYPE::LINEAR)
        {        
            //MOTION.startRotateVelocity(robotPose, -1 * profile.desAngVel, profile);
        }
    }
    return;
}

E_MOVING_CENTER_STEP CSignaltracking::movingCenterForwardToCenterControl(tPose robotPose)
{
    E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::FORWARD_TO_CENTER;
    tProfile profile;
    profile.desAngVel = DEG2RAD(6);
    switch (getLastSignal())
    {
    case E_AREA::LEFT:
        forwardToCenterLeftArea(robotPose, profile);
        break;
    case E_AREA::RIGHT:
        forwardToCenterRightArea(robotPose, profile);
        break;        
    default:
        break;
    }
    if (isSideReceiverCenterArea())
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
            SG.debugSignalPrint("충전기 앞으로 판단되어 무빙센터 종료");
        }
        ret = E_MOVING_CENTER_STEP::COMPLETE;
    }
    return ret;
}

E_MOVING_CENTER_STEP CSignaltracking::movingCenterTunringToChargerControl(tPose robotPose)
{
    E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::TURNING_TO_CHARGER;
    tProfile profile;
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(20);    

    if (SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG))
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
            SG.debugSignalPrint("충전기 정면에 도착.");
        }
        else
        {
            targetAng = k.rotation(robotPose, DEG2RAD(90));
            MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CCW);
            ret = E_MOVING_CENTER_STEP::COMPLETE;
        }
    }
    else if (SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
            SG.debugSignalPrint("충전기 정면에 도착.");
        }
        else
        {
            targetAng = k.rotation(robotPose, DEG2RAD(90));
            MOTION.startRotation(robotPose,targetAng,profile,E_ROTATE_DIR::CW);
            ret = E_MOVING_CENTER_STEP::COMPLETE;
        }
    }

    if (SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT))
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
            SG.debugSignalPrint("충전기 정면에 도착.");
        }
        else
        {
            ret = E_MOVING_CENTER_STEP::COMPLETE;
        }
    }

    return ret;
}

E_MOVING_CENTER_STEP CSignaltracking::movingCenterForwardToChargerControl(tPose robotPose)
{
    E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::FORWARD_TO_CHARGER;

    if (SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT)
    || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            // 스탑하고, 100ms 뒤에는 위 if를 안탈 수도 있는데.... 그러면 100ms동안에는 강제로 clear를 하지 말아야하나? list의 관리 주기에 120ms를 더해줘서 클리어되지 않게 해야하나...?????? ㅠㅠㅠㅠㅠㅠㅠ
            
            ceblog(LOG_LV_DOCKING, CYN, "stop 디버그 용 " << SYSTEM_TOOL.getSystemTime());
        }
        else
        {
            ret = E_MOVING_CENTER_STEP::COMPLETE;
            SG.debugSignalPrint("신호 발견해서 무빙 종료 ");
        }
    }

    if (!MOTION.isRunning())
    {
        // if (getLastSignal() == E_AREA::LEFT)
        // {
        //     MOTION.startCurveVelocity(robotPose, 0.15, DEG2RAD(7), tProfile());
        // }
        // else if (getLastSignal() == E_AREA::RIGHT)
        // {
        //     MOTION.startCurveVelocity(robotPose, 0.15, DEG2RAD(-7), tProfile());
        // }
        ret = E_MOVING_CENTER_STEP::COMPLETE;
    }

    return ret;
}

// E_MOVING_CENTER_STEP CSignaltracking::movingCenterScanningControl(tPose robotPose)
// {
//     E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::

//     return ret;
// }

// E_MOVING_CENTER_STEP CSignaltracking::movingCenterFailControl(tPose robotPose)
// {
//     E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::

//     return ret;
// }

// E_MOVING_CENTER_STEP CSignaltracking::movingCenterCompleteControl(tPose robotPose)
// {
//     E_MOVING_CENTER_STEP ret = E_MOVING_CENTER_STEP::

//     return ret;
// }


/**
 * @brief moving center step set
 * @date 23/08/11 hhryu
 * @param set
 */
void CSignaltracking::setMovingCenterStep(E_MOVING_CENTER_STEP set)
{
    CStopWatch __debug_sw;

    if (signaltrackData_.movingCenterStep != set)
    {
        SG.debugSignalPrint();
        ceblog(LOG_LV_DOCKING, WHITE, "E_MOVING_CENTER_STEP CHANGE " << CYN << "[" << enumToString(signaltrackData_.movingCenterStep) << "--> " << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
        signaltrackData_.movingCenterStep = set;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return;
}

/**
 * @brief moving center step get
 *
 * @return E_MOVING_CENTER_STEP
 *
 * @note 연산시간 < 0.1ms
 * @date 2023-08-11
 * @author hhryu
 */
E_MOVING_CENTER_STEP CSignaltracking::getMovingCenterStep(void)
{
    CStopWatch __debug_sw;
    TIME_CHECK_END(__debug_sw.getTime());
    return signaltrackData_.movingCenterStep;
}

/**
 * @brief scan control step set
 * @date 23/08/14 hhryu
 * @param set
 */
void CSignaltracking::setScanControlStep(E_SCAN_CONTROL_STEP set)
{
    CStopWatch __debug_sw;

    if (signaltrackData_.scanControlStep != set)
    {
        SG.debugSignalPrint();
        ceblog(LOG_LV_DOCKING, WHITE, "E_SCAN_CONTROL_STEP CHANGE " << CYN << "[" << enumToString(signaltrackData_.scanControlStep) << "--> " << enumToString(set) << "] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
        signaltrackData_.scanControlStep = set;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return;
}

/**
 * @brief get scan control
 *
 * @return E_SCAN_CONTROL_STEP
 */
E_SCAN_CONTROL_STEP CSignaltracking::getScanControlStep(void)
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());

    return signaltrackData_.scanControlStep;
}

/**
 * @brief Get the Signal Check Max Angle object
 *
 * @param direction
 * @return s16
 */
double CSignaltracking::getSignalCheckLastAngle(E_DIRECTION direction)
{
    CStopWatch __debug_sw;

    double ret = 0;

    if (direction == E_DIRECTION::LEFT)
    {
        ret = signaltrackData_.leftSideLastSignalCheckAngle;
    }
    else if (direction == E_DIRECTION::RIGHT)
    {
        ret = signaltrackData_.rightSideLastSignalCheckAngle;
    }
    else
    {
        ceblog(LOG_LV_ERROR | LOG_LV_DOCKING, RED, "Do not use case;;;");
        ceblog(LOG_LV_ERROR | LOG_LV_DOCKING, RED, "Do not use case;;;");
    }

    TIME_CHECK_END(__debug_sw.getTime());

    return ret;
}

void CSignaltracking::processSignalCheck(E_CHECK_SIGNAL checkSigState, double checkTime)
{
    switch (checkSigState)
    {
    case E_CHECK_SIGNAL::IDLE:
        SG.setCheckSignalState(E_CHECK_SIGNAL::START);
        SG.setCheckSignalEndTime(checkTime); // checkTime초 동안 체크 하겠다. (움직이면서 하든, 멈춰서 하든 무관.)
        break;
    case E_CHECK_SIGNAL::START:
        ceblog(LOG_LV_DOCKING,RED,"???");
        break;
    case E_CHECK_SIGNAL::ING:
        break;
    case E_CHECK_SIGNAL::END:
        break;
    default:
        break;
    }
}


void CSignaltracking::motionSwimming(u16 data)
{
    ServiceData.motionInfo.desVel = getCalculateSwimmingSteer(data);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionCenteringReadyTurn(int dir)
{
    ServiceData.motionInfo.desVel = getCalculateCenteringReadyTurnSteer(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionMovingCenter(int dir)
{
    ServiceData.motionInfo.desVel = getCalculateMoveCenterSteer(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionTilDownTryDock()
{
    ServiceData.motionInfo.desVel = getCalculateTryDockTiltDownSteer();
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionTryDockReadyTurn(int dir)
{
    ServiceData.motionInfo.desVel = getCalculateTryDockReadyTurnSteer(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionCheckSignalTurn(int dir)
{
    ServiceData.motionInfo.desVel = getCalculateCheckSignalTurnSteer(dir);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CSignaltracking::motionTilUpTryDock()
{
    ServiceData.motionInfo.desVel = getCalculateTryDockTiltUpSteer();
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

tTwist CSignaltracking::getCalculateTryDockTiltDownSteer()
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    double targetAngle = SG.buildTryDockAngle();
    double maxV = 0.25;//CONFIG.tryDock_max_V*1.5;
    
    if(fabs(RAD2DEG(targetAngle)) >= 20)
    {
        maxV = CONFIG.tryDock_max_V;

        if(targetAngle < 0) targetAngle = DEG2RAD(-20);
        else                targetAngle = DEG2RAD(20);
    }  

    curVel.v = adjustSpeedToTarget(curVel.v, maxV, CONFIG.tryDock_accel_V);
    curVel.w = adjustSpeedToTarget(curVel.w, targetAngle, CONFIG.tryDock_accel_W);
    // std::string color;
    // if (targetAngle > 0)        {color = BLUE;}
    // else if (targetAngle < 0)   {color = YELLOW;}
    // else                        {color = WHITE;}
    ceblog(LOG_LV_NECESSARY, RED, "Til-DOWN steer v : " << curVel.v << " , W : " << RAD2DEG(curVel.w) << "] targetAngle[" << targetAngle <<"," << RAD2DEG(targetAngle) << "]");
    return curVel;
}

tTwist CSignaltracking::getCalculateTryDockTiltUpSteer()
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    double maxV = CONFIG.tryDock_max_V;
    double targetAngle = SG.buildTryDockAngle();
    if(fabs(RAD2DEG(targetAngle)) >= 20)
    {
        //if(fabs(RAD2DEG(targetAngle)) >= 40) maxV = 0;

        if(targetAngle < 0) targetAngle = DEG2RAD(-20);
        else                targetAngle = DEG2RAD(20);
    }
    
    curVel.v = adjustSpeedToTarget(curVel.v, maxV, CONFIG.tryDock_accel_V);
    curVel.w = adjustSpeedToTarget(curVel.w, targetAngle, CONFIG.tryDock_accel_W);
    // std::string color;
    // if (targetAngle > 0)        {color = BLUE;}
    // else if (targetAngle < 0)   {color = YELLOW;}
    // else                        {color = WHITE;}    
    ceblog(LOG_LV_NECESSARY, GRAY, "Til-UP steer v : " << curVel.v << " , W : " << RAD2DEG(curVel.w) << "] targetAngle[" << targetAngle <<"," << RAD2DEG(targetAngle) << "]");
    return curVel;
}

tTwist CSignaltracking::getCalculateCheckSignalTurnSteer(int dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    if(curVel.v > 0) curVel.v -= CONFIG.signalTrack_decel_V;
    if(curVel.v <= 0)           curVel.v = 0;
    
    if(dir > 0){
        curVel.w -= CONFIG.signalTrack_decel_W;
        if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
        
    }
    else{
        curVel.w += CONFIG.signalTrack_decel_W;
        if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
    }

    //ceblog(LOG_LV_NECESSARY, RED, "CHCECK SIGNAL TURN 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w)  << "dir : " << dir << " acc : " << RAD2DEG(CONFIG.signalTrack_decel_W) << " max_w : " << RAD2DEG(CONFIG.signalTrack_max_W));
    return curVel;
}

tTwist CSignaltracking::getCalculateSwimmingSteer(u16 data)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    double max_v = 0.3;
    if(!SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_LEFT,data) && !SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_RIGHT,data))
    {
        if(curVel.v > max_v/2/*CONFIG.signalTrack_max_V*/) curVel.v -= CONFIG.signalTrack_accel_V; 
        else                                               curVel.v += CONFIG.signalTrack_accel_V;

        if(curVel.v >= max_v/2/*CONFIG.signalTrack_max_V*/) curVel.v = max_v/2;//CONFIG.signalTrack_max_V;

        if(swimingTemp_w != 0)
        {
            if(swimingTemp_w > 0)
            {
                curVel.w -= CONFIG.signalTrack_decel_W;
                if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
            }   
            else
            {
                curVel.w += CONFIG.signalTrack_decel_W;
                if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
            }                    
        }

        ceblog(LOG_LV_NECESSARY, RED, "신호 없음 천천히가면서 확인해보자 v : " << curVel.v << " w : " << curVel.w << "temp_w : " << swimingTemp_w);
    }
    else
    {
        if(SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_LEFT,data) && SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_RIGHT,data))
        {
            if(SG.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_SHORT_ANYTHING) || SG.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_SHORT_ANYTHING))
            {
                if(curVel.v > max_v/2)
                {
                    curVel.v -= CONFIG.signalTrack_accel_V;
                }
                else
                {
                    curVel.v += CONFIG.signalTrack_accel_V;
                    if(curVel.v >= max_v/2/*CONFIG.signalTrack_max_V*/) curVel.v = max_v/2;
                }
            }
            else
            {
                curVel.v += CONFIG.signalTrack_accel_V;
                if(curVel.v >= max_v/*CONFIG.signalTrack_max_V*/) curVel.v = max_v;//CONFIG.signalTrack_max_V;
            }
            
            if(curVel.w != 0)
            {
                if(curVel.w > 0)
                {
                    curVel.w -= CONFIG.signalTrack_decel_W;
                    if(curVel.w <= 0) curVel.w = 0;
                }    
                else
                {
                    curVel.w += CONFIG.signalTrack_decel_W;
                    if(curVel.w >= 0) curVel.w = 0;
                }                
            }
            swimingTemp_w = curVel.w;
        }
        else if(SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_LEFT,data))
        {
            if(curVel.v > max_v/2)
            {
                curVel.v -= CONFIG.signalTrack_accel_V;
            }
            else
            {
                curVel.v += CONFIG.signalTrack_accel_V;
                if(curVel.v >= max_v/2/*CONFIG.signalTrack_max_V*/) curVel.v = max_v/2;
            }
           
            curVel.w += CONFIG.signalTrack_accel_W;
            if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
            swimingTemp_w = curVel.w;
        }
        else if(SG.countBitSignalDetectedCASE1(IDX_RECEIVER_FRONT_RIGHT,data))
        {
            if(curVel.v > max_v/2)
            {
                curVel.v -= CONFIG.signalTrack_accel_V;
            }
            else
            {
                curVel.v += CONFIG.signalTrack_accel_V;
                if(curVel.v >= max_v/2/*CONFIG.signalTrack_max_V*/) curVel.v = max_v/2;
            }
            
            curVel.w -= CONFIG.signalTrack_accel_W;
            if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
            swimingTemp_w = curVel.w;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "신호 뭐야 ? : " << (int)data);
        }
    }

    ceblog(LOG_LV_NECESSARY, RED, "SWIMMING 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w));
    return curVel;
}

tTwist CSignaltracking::getCalculateCenteringReadyTurnSteer(int dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;

    curVel.v -= CONFIG.signalTrack_decel_V;
    

    if(dir > 0)
    {
        curVel.w += CONFIG.signalTrack_decel_W;
        if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
    }
    else
    {
        curVel.w -= CONFIG.signalTrack_decel_W;
        if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
    }
    
    if(curVel.v <= 0)           curVel.v = 0;
    // ceblog(LOG_LV_NECESSARY, RED, "MOVING CENTER READY TURN 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w) << "dir : " << dir);
    return curVel;
}

tTwist CSignaltracking::getCalculateTryDockReadyTurnSteer(int dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    if(curVel.v > 0) curVel.v -= CONFIG.signalTrack_decel_V;
    if(curVel.v <= 0)           curVel.v = 0;
    
    if(dir > 0){
        curVel.w -= CONFIG.signalTrack_decel_W;
        if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
        
    }
    else{
        curVel.w += CONFIG.signalTrack_decel_W;
        if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
    }

    // ceblog(LOG_LV_NECESSARY, GRAY, "TRY-DOCK READY TURN 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w) << "dir : " << dir);
    return curVel;
}

tTwist CSignaltracking::getCalculateMoveCenterSteer(int dir)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    u8 shortCount = 0,longCount = 0, frontCount = ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_LEFT,SIGNAL_ANYTHING)+ServiceData.signal.countSignalDetected(IDX_RECEIVER_FRONT_RIGHT,SIGNAL_ANYTHING);

    if(dir == 0)
    {
        curVel.v = 0;
        curVel.w = 0;
        ceblog(LOG_LV_NECESSARY, RED, "이미 센터에 있어요..얼음!!!");
        return curVel;
    }
    else if(dir > 0)
    {
        shortCount = ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_SHORT_ANYTHING);
        longCount = ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_RIGHT,SIGNAL_LONG_ANYTHING);
        if(frontCount)
        {
            if(curVel.v > 0) curVel.v -= CONFIG.signalTrack_accel_V;
            if(curVel.v <= 0) curVel.v = 0;
            curVel.w += CONFIG.signalTrack_accel_W;
            if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
        }
        else if(shortCount)
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            if(curVel.w != 0)
            {
                if(curVel.w > 0)
                {
                    curVel.w -= CONFIG.signalTrack_decel_W;
                    if(curVel.w <= 0) curVel.w = 0;
                }    
                else
                {
                    curVel.w += CONFIG.signalTrack_decel_W;
                    if(curVel.w >= 0) curVel.w = 0;
                }                
            }
        
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
        }
        else if(longCount)
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            curVel.w -= CONFIG.signalTrack_accel_W;
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
            if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
        }
        else
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
            curVel.w += CONFIG.signalTrack_accel_W;
            if(curVel.w >= 0) curVel.w = 0;//if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
        }
    }
    else
    {
        shortCount = ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_SHORT_ANYTHING);
        longCount = ServiceData.signal.countSignalDetected(IDX_RECEIVER_SIDE_LEFT,SIGNAL_LONG_ANYTHING);

        if(frontCount)
        {
            if(curVel.v > 0) curVel.v -= CONFIG.signalTrack_accel_V;
            if(curVel.v <= 0) curVel.v = 0;
            curVel.w -= CONFIG.signalTrack_accel_W;
            if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
        }
        else if(shortCount)
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            if(curVel.w != 0)
            {
                if(curVel.w > 0)
                {
                    curVel.w -= CONFIG.signalTrack_decel_W;
                    if(curVel.w <= 0) curVel.w = 0;
                }    
                else
                {
                    curVel.w += CONFIG.signalTrack_decel_W;
                    if(curVel.w >= 0) curVel.w = 0;
                }                
            }
        
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
        }
        else if(longCount)
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            curVel.w += CONFIG.signalTrack_accel_W;
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
            if(curVel.w >= CONFIG.signalTrack_max_W) curVel.w = CONFIG.signalTrack_max_W;
        }
        else
        {
            curVel.v += CONFIG.signalTrack_accel_V;
            if(curVel.v >= CONFIG.tryDock_max_V) curVel.v = CONFIG.tryDock_max_V;
            curVel.w -= CONFIG.signalTrack_accel_W;
            if(curVel.w <= 0) curVel.w = 0;//if(curVel.w <= -CONFIG.signalTrack_max_W) curVel.w = -CONFIG.signalTrack_max_W;
        }
    }

    if(dir < 0 ){
        ceblog(LOG_LV_NECESSARY, RED, "왼쪽에서 --> 오른쪽으로 MOVING CENTER STEER 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w));
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "오른쪽에서 --> 왼쪽으로 MOVING CENTER STEER 제어 v : " << curVel.v << " w : " << RAD2DEG(curVel.w));
    }
    
    return curVel;
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
double CSignaltracking::adjustSpeedToTarget(double currentSpeed, double targetSpeed, double acceleration)
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