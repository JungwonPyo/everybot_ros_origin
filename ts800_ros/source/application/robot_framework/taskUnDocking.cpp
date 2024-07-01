#include "taskUnDocking.h"
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

CTaskUnDocking::CTaskUnDocking()
{
    CStopWatch __debug_sw;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskUnDocking::~CTaskUnDocking()
{
    CStopWatch __debug_sw;

    
    setUnDockingState(UNDOCKING_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskUnDocking::setUnDockingState(UNDOCKING_STATE set)
{
    if (set != unDockingState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(unDockingState)<<" --> "<< enumToString(set) );
    }
    unDockingState = set;
}

void CTaskUnDocking::taskStart()
{
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "UN-DOCKING TASK RUN!!");
    profile = tProfile();
    setUnDockingState(UNDOCKING_STATE::CHECK_MODE);
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
bool CTaskUnDocking::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    
    switch (unDockingState)
    {
    case UNDOCKING_STATE::NONE :       
        break;
    case UNDOCKING_STATE::CHECK_MODE :
        setUnDockingState(procCheckMode(robotPose));       
        break;    
    case UNDOCKING_STATE::START_UNDOCKING :
        setUnDockingState(procStartUnDocking(robotPose,profile));
        break;    
    case UNDOCKING_STATE::CHECK_TILT :
        setUnDockingState(procCheckTilt(robotPose,profile));
        break;
    case UNDOCKING_STATE::ESCAPE_DOCKING_STATION :
        setUnDockingState(procEscapeDockingStation(robotPose,profile));
        break;
    case UNDOCKING_STATE::BACK_TO_START_POINT :
        setUnDockingState(procBackToStartPoint(robotPose,profile));
        break;
    case UNDOCKING_STATE::TURN :
        setUnDockingState(procTurnning(robotPose,profile));
        break;
    case UNDOCKING_STATE::COMPLETE :
        setUnDockingState(procComplete(robotPose,profile));
        ret = true;
        break;                
    default:
        break;
    }

    // if( SYSTEM_TOOL.getSystemTime()-imuInitTime <= 3)
    // {
    //     ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," IMU 초기화 데이터 확인 로봇위치 : " << robotPose.x << " , " << robotPose.y << "Angle : " << RAD2DEG(robotPose.angle));
    // }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procCheckMode(tPose robotPose)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::CHECK_MODE;
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();
    E_POWER_STATE powerState = ServiceData.power.getPowerState();

    if(powerState != E_POWER_STATE::ACTIVE)
    {
        modeInitTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.systemModeControl(E_POWER_MODE::MODE_ACTIVE);
        ceblog(LOG_LV_NECESSARY, RED, "ACITVE 모드가 아닙니다. TILT 제어가 불가능함!! POWER MODE : " << enumToString(powerState) << " Tilt State : " << enumToString(tiltState) );
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, RED, "ACITVE 모드 입니다. ");
    }
    ret = UNDOCKING_STATE::START_UNDOCKING;
    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procStartUnDocking(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::START_UNDOCKING;
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();
    E_POWER_STATE powerState = ServiceData.power.getPowerState();

    if(powerState == E_POWER_STATE::ACTIVE)
    {
        if(tiltState == E_SYS_TILT_STATE::TILTED_UP)
        {
            eblog(LOG_LV_NECESSARY,  "START UN-DOCKING !! AREADY TILT-UP ---> ESCAPE_DOCKING_STATION!!");
            startMotionEscape(robotPose,pf);
            ret = UNDOCKING_STATE::ESCAPE_DOCKING_STATION;
        }
        else
        {
            eblog(LOG_LV_NECESSARY,  "START UN-DOCKING !! TILSTATE : " << enumToString(tiltState) <<  " TILT-UP ---> CHECK-TILT STATE!!");
            ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
            ret = UNDOCKING_STATE::CHECK_TILT;
        }
    }
    else if(SYSTEM_TOOL.getSystemTime()-modeInitTime >= 0.1)
    {
        ret = UNDOCKING_STATE::CHECK_MODE;
    }

    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procCheckTilt(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::CHECK_TILT;
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();

    
    if(tiltState == E_SYS_TILT_STATE::TILTED_UP)
    {
        eblog(LOG_LV_NECESSARY,  "CHECK-TILT STATE!!  TILT-UP COMPLETE!! ---> ESCAPE_DOCKING_STATION!!");
        startMotionEscape(robotPose,pf);
        ret = UNDOCKING_STATE::ESCAPE_DOCKING_STATION;
    }
    
    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procEscapeDockingStation(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::ESCAPE_DOCKING_STATION;
    
    if(MOTION.isNearTargetPose(robotPose, targetPoint, fabs(CONFIG.UnDockingEscapeDistance)*0.70) || MOTION.isOverTargetPoint(robotPose, startPoint, targetPoint))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "ESCAPE_DOCKING_STATION COMPLETE!! ---> BACK_TO_START_POINT!!");
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
        ret = UNDOCKING_STATE::BACK_TO_START_POINT;
    }

    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procBackToStartPoint(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::BACK_TO_START_POINT;
    
    
    if(MOTION.isNearTargetPose(robotPose, targetPoint, 0.1) || MOTION.isOverTargetPoint(robotPose, startPoint, targetPoint))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "BACK_TO_START_POINT COMPLETE!!");
        startMotionTurn(robotPose,pf);        
        ret = UNDOCKING_STATE::TURN;
    }
    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procTurnning(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::TURN;
    
    if (MOTION.isNearTargetRad(robotPose, targetAngle, DEG2RAD(5)))
    {  
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "TURNNING COMPLETE!!");
        startMotionStop(robotPose,pf,false);   
        ret = UNDOCKING_STATE::COMPLETE;
    }

    return ret;
}

UNDOCKING_STATE CTaskUnDocking::procComplete(tPose robotPose, tProfile pf)
{
    UNDOCKING_STATE ret = UNDOCKING_STATE::COMPLETE;
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();
    

    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "UN-DOCKING COMPLETE!!");
    SUB_TASK.cleanPlan.enableNogoZoneDocking();
    ret = UNDOCKING_STATE::NONE;

    return ret;
}

void CTaskUnDocking::startMotionEscape(tPose robotPose, tProfile pf)
{
    CRobotKinematics k;
    double backMoving = CONFIG.UnDockingEscapeDistance;
    startPose = robotPose;
    startPoint = tPoint(robotPose.x, robotPose.y);
    targetPoint = k.translate(robotPose, backMoving, 0.0);
    // targetPoint = tPoint(backMoving, 0.0);
    MOTION.startBackToPointOnMap(robotPose,targetPoint,pf);
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," Start UN-DOCKING ESCAPE DISTANCE " << backMoving <<  " MOVING BACK 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << targetPoint.x << " ," << targetPoint.y);
}

void CTaskUnDocking::startMotionTurn(tPose robotPose, tProfile pf)
{
    CRobotKinematics k;
    //targetAngle = DEG2RAD(180);//
    targetAngle = k.rotation(startPose, DEG2RAD(180));
    MOTION.startRotation(robotPose, targetAngle,pf,E_ROTATE_DIR::NONE);
}

void CTaskUnDocking::startMotionStop(tPose robotPose, tProfile pf, bool emergency)
{
    MOTION.startStopOnMap(pf,emergency);
    
}
