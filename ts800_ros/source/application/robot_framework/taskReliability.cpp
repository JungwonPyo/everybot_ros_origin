#include "taskReliability.h"

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
CTaskReliability::CTaskReliability()
{
    CStopWatch __debug_sw;
    
    
    r = 0; l = 0; b = 0;
    bStopCommand = false;
    imuInitTime = SYSTEM_TOOL.getSystemTime();
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskReliability::~CTaskReliability()
{
    CStopWatch __debug_sw;

    
    setReliabilityState(RELIABILITY_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskReliability::setReliabilityState(RELIABILITY_STATE set)
{
    if (set != reliabilityState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(reliabilityState)<<" --> "<< enumToString(set) );
    }
    reliabilityState = set;
    
    if(set != RELIABILITY_STATE::NONE) bStopCommand = false;
}

#if 0
bool CTaskReliability::keyChecker(tPose robotPose)
{
    bool bRet = false;
    E_KEY_FUNCTION keyVal = ServiceData.key.getKeyValue();

    if(reliabilityState != RELIABILITY_STATE::RUN_CHARGE)
    {
        if(keyVal == E_KEY_FUNCTION::POWER_OFF){
        MOTION.startStopOnMap(tProfile(),true);
        MOTION.procStop(robotPose);
        ceblog(LOG_LV_NECESSARY, RED, "POWER OFF @@@");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_POWER, tMsgPowerOff()));
        }   
    }
    
    if(keyVal == E_KEY_FUNCTION::START_STOP || keyVal == E_KEY_FUNCTION::CLEAN_AUTO)
    {
        if(reliabilityState == RELIABILITY_STATE::RUN_CHARGE)
        {
            taskUnDocking.taskStart();
            setReliabilityState(RELIABILITY_STATE::RUN_UNDOCKING);
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_AUTO_CLEAN_START);
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::START_CLEANING);
        }
        else if(reliabilityState != RELIABILITY_STATE::NONE)
        {
            LED_CTR.pause();
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
            setReliabilityState(RELIABILITY_STATE::NONE);
        }
        else
        {
            eblog(LOG_LV_NECESSARY, " Imu 초기화");
            MOTION.startStopOnMap(tProfile(),true);
            MOTION.procStop(robotPose);
            ROBOT_CONTROL.clearSystemLocalization();
            ROBOT_CONTROL.system.initImuSensor();
            imuInitTime = SYSTEM_TOOL.getSystemTime();
            setRandomCleanState(INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR);
            setReliabilityState(RELIABILITY_STATE::RUN_INFINITY);
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_AUTO_CLEAN_START);
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::START_CLEANING);
        }
    }
    else if(keyVal == E_KEY_FUNCTION::HOME || keyVal == E_KEY_FUNCTION::SERVICE_END || keyVal == E_KEY_FUNCTION::MOVING_STOP){
        LED_CTR.pause();
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);        
        setReliabilityState(RELIABILITY_STATE::NONE);
    }
    else if(keyVal == E_KEY_FUNCTION::WATER_DRAIN){
        if(reliabilityState != RELIABILITY_STATE::NONE){
            LED_CTR.pause();
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
            setReliabilityState(RELIABILITY_STATE::NONE);
        }
        else{
            r = 262; l = 262; b = 45;
            setReliabilityState(RELIABILITY_STATE::RUN_NOISE_EVALUATION);
        }
    }

    return bRet;
}
#endif

bool CTaskReliability::isWheelActive()
{
    bool ret = false;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if(pObstacle->wheel.leftEncoder > 0 || pObstacle->wheel.rightEncoder > 0 || pObstacle->wheel.dummyEncoder > 0)
    {
        ret = true;
    }

    return ret;
}

void CTaskReliability::runProcNone(tPose robotPose)
{
    if(!bStopCommand)
    {
        if(isWheelActive()){bStopCommand = true;MOTION.startStopOnMap(tProfile(),true);}
    }

    if(!MOTION.isRunning()) MOTION.sendMessageDrvie(0,0);
    //else                     // 백대원 수석님 요청
}

void CTaskReliability::taskStart()
{
    setRandomCleanState(INFINITY_RANDOM_CLEAN_STATE::CHECK_MODE);
    setReliabilityState(RELIABILITY_STATE::RUN_INFINITY);
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
bool CTaskReliability::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    tPose robotPose = ServiceData.localiz.getPose();
    if(ServiceData.power.getExtPower())
    {
        if(reliabilityState != RELIABILITY_STATE::RUN_CHARGE && reliabilityState != RELIABILITY_STATE::RUN_UNDOCKING)
        {
            eblog(LOG_LV_NECESSARY, "충전상태 아님!! 언도킹도 아닌데!! 단자 접촉 감지 충전모드로 전환");
            MOTION.startStopOnMap(tProfile(),true);
            
            taskCharging.taskStart();
            setReliabilityState(RELIABILITY_STATE::RUN_CHARGE);
        } 
    }
    else  if(reliabilityState == RELIABILITY_STATE::RUN_CHARGE)
    {
        eblog(LOG_LV_NECESSARY, "충전 중 단자 접촉 해제 됨 대기 상태로 전환");
        LED_CTR.pause();
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
        setReliabilityState(RELIABILITY_STATE::NONE);
    }
    
    //keyChecker(robotPose);

    switch (reliabilityState)
    {
    case RELIABILITY_STATE::NONE :
        runProcNone(robotPose);             
        break;
    case RELIABILITY_STATE::RUN_INFINITY :
        setReliabilityState(runRandomCleanInfinity(robotPose));
        break;
    case RELIABILITY_STATE::RUN_NOISE_EVALUATION:
        MOTION.sendMessageDriveOld(1, r, l, b, 0, 0);
        break;
    case RELIABILITY_STATE::RUN_CHARGE :
        runCharging();
        break;
    case RELIABILITY_STATE::RUN_UNDOCKING :
       setReliabilityState(runUnDocking(robotPose));
        break;        
    default:
        break;
    }
       
    

    if( SYSTEM_TOOL.getSystemTime()-imuInitTime <= 3)
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," IMU 초기화 데이터 확인 로봇위치 : " << robotPose.x << " , " << robotPose.y << "Angle : " << RAD2DEG(robotPose.angle));
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


void CTaskReliability::setRandomCleanState(INFINITY_RANDOM_CLEAN_STATE set)
{
    if (set != randomCleanState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(randomCleanState)<<" --> "<< enumToString(set) );
    }
    randomCleanState = set;
}

void CTaskReliability::runCharging()
{
    taskCharging.taskRun();
}
RELIABILITY_STATE CTaskReliability::runUnDocking(tPose robotPose)
{
    
    RELIABILITY_STATE ret = RELIABILITY_STATE::RUN_UNDOCKING;
    if(taskUnDocking.taskRun(robotPose))
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        eblog(LOG_LV_NECESSARY, " Imu 초기화");
        ROBOT_CONTROL.clearSystemLocalization();
        ROBOT_CONTROL.system.initImuSensor();
        imuInitTime = SYSTEM_TOOL.getSystemTime();
        setRandomCleanState(INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR);
        ret = RELIABILITY_STATE::RUN_INFINITY;
    }
    return ret;
}

RELIABILITY_STATE CTaskReliability::runRandomCleanInfinity(tPose robotPose)
{
    RELIABILITY_STATE ret = RELIABILITY_STATE::RUN_INFINITY;
    tProfile profile;

    E_BATTERY_STATE battState = ServiceData.battery.getBatteryState();

    if(battState == E_BATTERY_STATE::BATT_NEED_CHARGE)
    {
        //DISPLAY_CTR.ChargingDisplay(true,battState);
        return RELIABILITY_STATE::NONE;
    } 

    switch (randomCleanState)
    {
    case INFINITY_RANDOM_CLEAN_STATE::NONE:
        break;
    case INFINITY_RANDOM_CLEAN_STATE::CHECK_MODE:
        setRandomCleanState(checkMode());
        break;    
    case INFINITY_RANDOM_CLEAN_STATE::INIT_SENSOR:
        setRandomCleanState(initSensor(robotPose));
        break;    
    case INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR:
        setRandomCleanState(checkSensor(robotPose));
        break;    
    case INFINITY_RANDOM_CLEAN_STATE::START_GOING:
        setRandomCleanState(startRandomGo(robotPose,profile));
        break;
    case INFINITY_RANDOM_CLEAN_STATE::RUN_GOING:
        setRandomCleanState(runRandomGo(robotPose,profile));
        break;
    case INFINITY_RANDOM_CLEAN_STATE::START_BACK_MOVING:
        setRandomCleanState(startRandomBack(robotPose,profile));
        break;
    case INFINITY_RANDOM_CLEAN_STATE::RUN_BACK_MOVING:
       setRandomCleanState(runRandomBack(robotPose));
        break;
    case INFINITY_RANDOM_CLEAN_STATE::START_TURNNING:
        setRandomCleanState(startRandomTurn(robotPose,profile));
        break;
    case INFINITY_RANDOM_CLEAN_STATE::RUN_TURNNING:
        setRandomCleanState(runRandomTurn(robotPose));
        break;                          
    default:
        break;
    }

    return ret;
}


INFINITY_RANDOM_CLEAN_STATE CTaskReliability::checkMode()
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::CHECK_MODE;
    E_POWER_STATE powerState = ServiceData.power.getPowerState();

    if(powerState != E_POWER_STATE::ACTIVE)
    {
        modeInitTime = SYSTEM_TOOL.getSystemTime();
        ROBOT_CONTROL.systemModeControl(E_POWER_MODE::MODE_ACTIVE);
        ceblog(LOG_LV_NECESSARY, RED, "ACITVE 모드가 아닙니다. 제어가 불가능함!! POWER MODE : " << enumToString(powerState) );
        
    }
    else
    {
        ROBOT_CONTROL.clearSystemLocalization();
        ROBOT_CONTROL.system.initImuSensor();
        imuInitTime = SYSTEM_TOOL.getSystemTime();
        ret = INFINITY_RANDOM_CLEAN_STATE::INIT_SENSOR;
    }

    return ret;

}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::initSensor(tPose robotPose)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::INIT_SENSOR;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    double runTime = SYSTEM_TOOL.getSystemTime()-imuInitTime;

    E_POWER_STATE powerState = ServiceData.power.getPowerState();

    if(powerState == E_POWER_STATE::ACTIVE)
    {
        if(pObstacle->imu.state != E_IMU_STATUS::IMU_READY)
        {
            ret = INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR;
        }
        else if(runTime >= 3.5)
        {
            MOTION.startStopOnMap(tProfile(),true);
            
            eblog(LOG_LV_NECESSARY, " Imu 초기화");
            ROBOT_CONTROL.clearSystemLocalization();
            ROBOT_CONTROL.system.initImuSensor();
            imuInitTime = SYSTEM_TOOL.getSystemTime();
        }
    }
    else if(SYSTEM_TOOL.getSystemTime()-modeInitTime >= 0.1)
    {
        ret = INFINITY_RANDOM_CLEAN_STATE::CHECK_MODE;
    }

    return ret;

}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::checkSensor(tPose robotPose)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    double runTime = SYSTEM_TOOL.getSystemTime()-imuInitTime;

    if(pObstacle->imu.state == E_IMU_STATUS::IMU_READY && robotPose.x == 0 && robotPose.y == 0)
    {
        ret = INFINITY_RANDOM_CLEAN_STATE::START_GOING;
    }
    else if(runTime >= 3.5)
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        eblog(LOG_LV_NECESSARY, " Imu 초기화");
        ROBOT_CONTROL.clearSystemLocalization();
        ROBOT_CONTROL.system.initImuSensor();
        imuInitTime = SYSTEM_TOOL.getSystemTime();
        ret = INFINITY_RANDOM_CLEAN_STATE::INIT_SENSOR;
    }

    return ret;

}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::startRandomGo(tPose robotPose, tProfile pf)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::START_GOING;
    CRobotKinematics k;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    double runTime = SYSTEM_TOOL.getSystemTime()-imuInitTime;

    targetPoint = k.translate(robotPose, 10.0, 0.0);
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 랜덤 청소 직진을 시작 합니다. 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << targetPoint.x << " ," << targetPoint.y);
    MOTION.startLinearToPointOnMap(robotPose, targetPoint,pf);
    ret = INFINITY_RANDOM_CLEAN_STATE::RUN_GOING;
    
    return ret;
}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::runRandomGo(tPose robotPose,tProfile pf)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::RUN_GOING;
    bool bCheckObs = avoiding.checkObstacle(robotPose, true, false);
    CRobotKinematics k;
       
    if(MOTION.isNearTargetPose(robotPose, targetPoint, 0.15) || MOTION.isOverTargetPoint(robotPose,startPoint,targetPoint) || bCheckObs)
    {
        if(bCheckObs)
        {
            MOTION.startStopOnMap(pf,true);
            
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "이동 중 장애물을 만나 후진 시작 합니다.");
            ret = startRandomBack(robotPose,pf);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지에 도착했습니다. 회전을 시작 합니다.");
            ret = startRandomTurn(robotPose,pf);
        }
    }
    
    return ret;
}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::startRandomBack(tPose robotPose, tProfile pf)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::START_BACK_MOVING;
    CRobotKinematics k;
    startPoint = tPoint(robotPose.x, robotPose.y);
    targetPoint = k.translate(robotPose, -0.3, 0.0);
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 장애물 감지! 후진 시작!! 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << targetPoint.x << " ," << targetPoint.y);
    MOTION.startBackToPointOnMap(robotPose,targetPoint,pf);
    ret = INFINITY_RANDOM_CLEAN_STATE::RUN_BACK_MOVING;
    return ret;
}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::runRandomBack(tPose robotPose)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::RUN_BACK_MOVING;

    
    if(MOTION.isNearTargetPose(robotPose, targetPoint, 0.15) || MOTION.isOverTargetPoint(robotPose,startPoint,targetPoint) || !MOTION.isRunning() )
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 목적지 도착했습니다. 회전을 시작 합니다.");
        ret = INFINITY_RANDOM_CLEAN_STATE::START_TURNNING;
    }
    
    return ret;
}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::startRandomTurn(tPose robotPose, tProfile pf)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::START_TURNNING;
    CRobotKinematics k;
    RSF_OBSTACLE_MASK mask = avoiding.getAvoidMask();
    int duty = 0;
    E_ROTATE_DIR dir;

    if((mask.b.fleft_side && mask.b.fright_side)|| mask.b.fleft_center || mask.b.fright_center) duty = 120;
    else if(mask.value & 0x0E) duty = 90;
    else if(mask.value & 0x0F) duty = 45;
    else if(mask.value & 0x70) duty = -90;
    else                       duty = -45;

    if(duty >= 0) dir = E_ROTATE_DIR::CCW;
    else          dir = E_ROTATE_DIR::CW;

    targetAngle = k.rotation(robotPose, DEG2RAD(duty));

    MOTION.startRotation(robotPose, targetAngle,pf,dir);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 회전 시작 : " << RAD2DEG(targetAngle) << " duty : " << duty);
    ret = INFINITY_RANDOM_CLEAN_STATE::RUN_TURNNING;
    return ret;
}

INFINITY_RANDOM_CLEAN_STATE CTaskReliability::runRandomTurn(tPose robotPose)
{
    INFINITY_RANDOM_CLEAN_STATE ret = INFINITY_RANDOM_CLEAN_STATE::RUN_TURNNING;
    
    
    if(MOTION.isNearTargetRad(robotPose, targetAngle, DEG2RAD(5)) )
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 회전 도착");
        ret = INFINITY_RANDOM_CLEAN_STATE::START_GOING;
    }
    
    return ret;
}



