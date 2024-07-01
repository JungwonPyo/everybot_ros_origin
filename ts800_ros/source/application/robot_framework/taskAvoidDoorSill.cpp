#include "taskAvoidDoorSill.h"
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

CTaskAvoidDoorSill::CTaskAvoidDoorSill()
{
    CStopWatch __debug_sw;
    
    bPowerOn = false;
    setDoorSillState(DOOR_SILL_STATE::NONE);
    setDoorSillStep(DOORSILL_READY_STEP::NONE);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskAvoidDoorSill::~CTaskAvoidDoorSill()
{
    CStopWatch __debug_sw;

    
    setDoorSillState(DOOR_SILL_STATE::NONE);
    setDoorSillStep(DOORSILL_READY_STEP::NONE);
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskAvoidDoorSill::setDoorSillState(DOOR_SILL_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}
void CTaskAvoidDoorSill::setDoorSillStep(DOORSILL_READY_STEP set)
{
    if (set != step)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[step change] : "<< enumToString(step)<<" --> "<< enumToString(set) );
    }
    step = set;
}

void CTaskAvoidDoorSill::taskStart(DOOR_SILL_STATE startState)
{
    setDoorSillState(startState);
    setDoorSillStep(DOORSILL_READY_STEP::NONE);
}

void CTaskAvoidDoorSill::taskRun(tPose robotPose)
{
    CStopWatch __debug_sw;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    switch (state)
    {
    case DOOR_SILL_STATE::NONE:
        if(!bPowerOn)
        {
            bPowerOn = true;
        }
        
        if(MOTION.isRunning()) MOTION.procDirectPwm();
        if(ROBOT_CONTROL.getTiltStep() != E_TILT_STEP::VOID) ROBOT_CONTROL.procControlTilt();
        else if(ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILTED_DOWN) ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
        break;
    case DOOR_SILL_STATE::GO:
        setDoorSillState(procGoForward(robotPose));
        break;    
    case DOOR_SILL_STATE::MOVE:
        setDoorSillState(procRunMove(robotPose));
        break;        
    case DOOR_SILL_STATE::START_READY:
        setDoorSillState(procStartReadyDoorSill(robotPose));
        break;
    case DOOR_SILL_STATE::RUN_READY:
        setDoorSillState(procRunReadyDoorSill(robotPose));
        break;    
    case DOOR_SILL_STATE::CLIMB:
        setDoorSillState(climbDoorSill(robotPose,pObstacle));
        break;
    case DOOR_SILL_STATE::START_ACROSS:
        setDoorSillState(startAcrossDoorSill(robotPose,pObstacle));
    break;        
    case DOOR_SILL_STATE::RUN_ACROSS:
        setDoorSillState(runAcrossDoorSill(robotPose,pObstacle));
    break;
    case DOOR_SILL_STATE::COMPLETE:
        setDoorSillState(completeAcrossDoorSill(robotPose));
    break;
    case DOOR_SILL_STATE::FAIL:
        setDoorSillState(failAcrossDoorSill(robotPose));
    break;        
    default:
        break;
    }

    // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "TaskDoorSill Running..."<< 
    // BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
    // << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << " ," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << " ,"
    // << SC<int>(pObstacle->imu.Gpitch)<<" ,"<<SC<int>(pObstacle->imu.Groll)<<" ,"<<SC<int>(pObstacle->imu.Gyaw));

    TIME_CHECK_END(__debug_sw.getTime());
}


DOOR_SILL_STATE CTaskAvoidDoorSill::procGoForward(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::GO;
    //TODO : 가장 가까운 벽을 찾는 알고리즘 개발 필요.
    double runTime = SYSTEM_TOOL.getSystemTime()-startTime;

    //if(runTime >= 1) // 신뢰성 시험할 때 버튼 누르면서 라이다 장애물 감지 때문에 추가한 코드임
    {
        CRobotKinematics k;
        originTarget = k.translate(robotPose, 1.5, 0.0);
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 이동 시작 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << originTarget.x << " ," << originTarget.y);
        MOTION.startLinearToPointOnMap(robotPose, originTarget, tProfile());
        ret = DOOR_SILL_STATE::MOVE;
    }
    
    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::procRunMove(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::MOVE;

    
    if(avoiding.checkObstacle(robotPose,false,false))
    {
        MOTION.startStopOnMap(tProfile(),false);
        ret = DOOR_SILL_STATE::START_READY;
    }
    
    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::procStartReadyDoorSill(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::START_READY;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    u8 checkLidarMask = 0x3C;
    //double checkTime = SYSTEM_TOOL.getSystemTime()-avoidStartTime;

     
    if(!MOTION.isRunning())
    {
        setDoorSillStep(DOORSILL_READY_STEP::START_BALANCE);
        ret = DOOR_SILL_STATE::RUN_READY;
    }

    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::procRunReadyDoorSill(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::RUN_READY;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    u8 checkLidarMask = 0x3C;
    //double checkTime = SYSTEM_TOOL.getSystemTime()-avoidStartTime;


    if((pObstacle->lidar.obstacle.value & checkLidarMask) || (pObstacle->lidar.approach.value & checkLidarMask) || 
    pObstacle->front.obstacle.value || /*pObstacle->front.approach.value ||*/ pObstacle->tof.knoll.rangeAvg < 100)  knollclearcount++;
    else                                                                                                            knollclearcount = 0;

    ret = procDoorSillReady(robotPose,pObstacle);

    if(knollclearcount >= 25)//checkTime >= 5 ||
    {
        knollclearcount = 0;
        eblog(LOG_LV_NECESSARY, /*"CheckTime : ["<< checkTime <<"]" << */"bumper : " << (int)pObstacle->bumper.value << "Lidar obs : " << (int)pObstacle->lidar.obstacle.value << "Lidar approach : " << (int)pObstacle->lidar.approach.value );

        //MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,0,tProfile());
        MOTION.startStopOnMap(tProfile(),false);
        ret =  DOOR_SILL_STATE::FAIL;
    }

    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::climbDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::CLIMB;

    int cliffTofVal = 170;
    int standTofVal = 130;
    double diffHeading = RAD2DEG(KnollHeading)-RAD2DEG(robotPose.angle);

    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "climbDoorSill"<< 
    BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
    << BOLDBLACK<< " |KnollHeading : " << WHITE<< RAD2DEG(KnollHeading) << ", RobotHeading : " << RAD2DEG(robotPose.angle) << ", Diff Heading : " << diffHeading
    << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << "," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << ","
    << SC<int>(pObstacle->imu.Gpitch)<<","<<SC<int>(pObstacle->imu.Groll)<<","<<SC<int>(pObstacle->imu.Gyaw));
    //<< "|CServiceIdle sys pose:" << pServiceData->localiz.getSysPose().x << "," << pServiceData->localiz.getSysPose().y<< "," << pServiceData->localiz.getSysPose().angle);

    if(pObstacle->tof.lcliff.rangeAvg >= cliffTofVal && pObstacle->tof.rcliff.rangeAvg >= cliffTofVal && fabs(diffHeading) <= 10)
    {
        MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,false,tProfile());
        eblog(LOG_LV_NECESSARY, "등반 완료 1초 정지 후 낙하 판단!");
        ret = DOOR_SILL_STATE::START_ACROSS;
    }
    else if(pObstacle->tof.lcliff.rangeAvg >= cliffTofVal || pObstacle->tof.rcliff.rangeAvg >= cliffTofVal)
    {
        if(pObstacle->tof.lcliff.rangeAvg >= cliffTofVal && pObstacle->tof.rcliff.rangeAvg >= cliffTofVal)
        {
            if(diffHeading > 0)     MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());
            else                    MOTION.startDriveWheelPwm(robotPose,1,100,0,-20,0,false,tProfile());
        }
        else if(pObstacle->tof.lcliff.rangeAvg >= cliffTofVal)
        {
            MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());
        }
        else
        {
            MOTION.startDriveWheelPwm(robotPose,1,100,0,-20,0,false,tProfile());
        }
    }
    else
    {
        if(pObstacle->tof.rcliff.rangeAvg >= standTofVal && pObstacle->tof.lcliff.rangeAvg < standTofVal)
        {
            if(pObstacle->imu.Ax >= 500)
            {
                if(pObstacle->imu.Ay >= 1500 )  MOTION.startDriveWheelPwm(robotPose,1,100,0,-20,0,false,tProfile());
                else                            MOTION.startDriveWheelPwm(robotPose,1,150,50,-20,0,false,tProfile());
            }    
            //else                                                          MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile());
        }
        else if(pObstacle->tof.rcliff.rangeAvg < standTofVal && pObstacle->tof.lcliff.rangeAvg >= standTofVal)
        {
            if(pObstacle->imu.Ax <= -500)
            {
                if(pObstacle->imu.Ay >= 1500 )  MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());
                else                            MOTION.startDriveWheelPwm(robotPose,1,50,150,20,0,false,tProfile());
            }   
            //else                            MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile());
        }
        else
        {
            MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile());
        }
    }

    MOTION.procDirectPwm();

    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::startAcrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::START_ACROSS;
    completeCnt = 0;
    eblog(LOG_LV_NECESSARY, "안전지대 확인 넘어가자 고고싱!! 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표 좌표 : " << originTarget.x << " ," << originTarget.y);
    MOTION.startLinearToPointOnMap(robotPose, originTarget, tProfile());
    ret = DOOR_SILL_STATE::RUN_ACROSS;
    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::runAcrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::RUN_ACROSS;
    int tUpCliffTofVal = 250;
    int tUpstandTofVal = 150;

    

    if(pObstacle->tof.lcliff.rangeAvg > tUpCliffTofVal || pObstacle->tof.rcliff.rangeAvg > tUpCliffTofVal)
    {
        CRobotKinematics k;
        startPoint = tPoint(robotPose.x, robotPose.y);
        targetPoint = k.translate(robotPose, -0.15, 0.0);
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 등반 중 낙하감지!! 후진~~~ 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << targetPoint.x << " ," << targetPoint.y);
        MOTION.startBackToPointOnMap(robotPose,targetPoint,tProfile());
        ret = DOOR_SILL_STATE::FAIL;
    }
    else
    {
        //틸업 상태 움직이는 중에 평탄한 바닥에서 움직이고 있어요! 기준을 찾아야함
        if(pObstacle->tof.lcliff.rangeAvg <= tUpstandTofVal && pObstacle->tof.rcliff.rangeAvg <= tUpstandTofVal && 
        abs(pObstacle->imu.Ax) <= 1000 && pObstacle->imu.Ay >= 300 && pObstacle->imu.Ay <= 1500 &&  // pObstacle->imu.Az >= CONFIG.standAccelZ  && pObstacle->imu.Az <= CONFIG.limitAccelZ &&
        abs(pObstacle->imu.Groll) <= 100 && abs(pObstacle->imu.Gpitch) <= 100 )
        {
            if(++completeCnt >= 30)
            {
                completeCnt = 0;
                eblog(LOG_LV_NECESSARY, "등반 착지 완료");
                //MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,false,tProfile());
                MOTION.startStopOnMap(tProfile(),false);
                ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
                ret = DOOR_SILL_STATE::COMPLETE;
            }
            else
            {
                ceblog(LOG_LV_NECESSARY, BOLDBLACK, "acrossingDoorSill coount "<< (int)completeCnt << 
                BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
                << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << " ," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << " ,"
                << SC<int>(pObstacle->imu.Gpitch)<<" ,"<<SC<int>(pObstacle->imu.Groll)<<" ,"<<SC<int>(pObstacle->imu.Gyaw));
            }
        }
        else
        {
            completeCnt = 0;
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "acrossingDoorSill count clear-------- "<<  
                BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
                << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << " ," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << " ,"
                << SC<int>(pObstacle->imu.Gpitch)<<" ,"<<SC<int>(pObstacle->imu.Groll)<<" ,"<<SC<int>(pObstacle->imu.Gyaw));
        }
    }

    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::completeAcrossDoorSill(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::COMPLETE;

    
    
    if(!MOTION.isRunning() && ServiceData.tilting.getStateValue() == E_SYS_TILT_STATE::TILTED_DOWN)
    {
        eblog(LOG_LV_NECESSARY, "문턱 등반 완료!! 정지!!");
        ret = DOOR_SILL_STATE::NONE;
    } 
    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::failAcrossDoorSill(tPose robotPose)
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::FAIL;
    

    if(!MOTION.isRunning() || MOTION.isNearTargetPose(robotPose,targetPoint,0.15) || MOTION.isOverTargetPoint(robotPose,startPoint,targetPoint))//if(MOTION.isPwmTargetDistanceArrived(robotPose))
    {
        eblog(LOG_LV_NECESSARY, "문턱 등반 실패 모터 정지 완료!!");
        ret = DOOR_SILL_STATE::NONE;
    }
    
    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::startBalanceDoorSill(tPose robotPose)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::START_BALANCE;

    RSF_OBSTACLE_MASK mask = avoiding.getAvoidMask();
    if(mask.value & 0x0F)   MOTION.startDriveWheelPwm(robotPose,1,100,0,20,0,false,tProfile());
    else                    MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());

    return DOORSILL_READY_STEP::RUN_BALANCE;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::runBalanceDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::RUN_BALANCE;
    u8 checkLidarMask = 0x7E;
    
    double angleDiff = fabs(utils::math::rad2deg(robotPose.angle)-utils::math::rad2deg(poseTemp.angle));

    if(angleDiff < 0.5)     knollTrapcount++;
    else                    knollTrapcount = 0;

    if((knollTrapcount >= 50)) //|| (pObstacle->bumper.b.fleft_side && pObstacle->bumper.b.fright_side)
    {
        MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,false,tProfile());
        KnollHeading = robotPose.angle;
        eblog(LOG_LV_NECESSARY, "문턱과 밸런스 완료 Heading :" << RAD2DEG(KnollHeading));
        knollTrapcount = 0;

        if(pObstacle->lidar.obstacle.value)                         avoiding.setAvoidMask(pObstacle->lidar.obstacle);
        else if(pObstacle->lidar.approach.value & checkLidarMask)   avoiding.setAvoidMask(pObstacle->lidar.approach);
        else if(pObstacle->front.obstacle.value)                    avoiding.setAvoidMask(pObstacle->front.obstacle);
        else                                                        avoiding.setAvoidMask(pObstacle->bumper);
             
        ret = DOORSILL_READY_STEP::START_MOVE_BACK;
    }
                        
    eblog(LOG_LV_NECESSARY, "AngleDiff : ["<< angleDiff <<"]" << "count : " << (int)knollTrapcount);
    poseTemp = robotPose;
    MOTION.procDirectPwm();

    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::startMoveBack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::START_MOVE_BACK;
    
    CRobotKinematics k;
    startPoint = tPoint(robotPose.x, robotPose.y);
    targetPoint = k.translate(robotPose, -0.1, 0.0);
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 틸업을 위한 후진 시작!! 로봇위치 : " << robotPose.x << " , " << robotPose.y << "목표좌표 : " << targetPoint.x << " ," << targetPoint.y);
    MOTION.startBackToPointOnMap(robotPose,targetPoint,tProfile());
    //MOTION.startDriveWheelPwm(robotPose,1,-100,-100,20,50,true,tProfile());
    //MOTION.procDirectPwm();
    ret = DOORSILL_READY_STEP::RUN_MOVE_BACK;
    
    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::runMoveBack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::RUN_MOVE_BACK;

    if(MOTION.isNearTargetPose(robotPose,targetPoint,0.05) || !MOTION.isRunning() || MOTION.isOverTargetPoint(robotPose,startPoint,targetPoint))//if(MOTION.isPwmTargetDistanceArrived(robotPose))
    {
        //MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,false,tProfile());
        MOTION.startStopOnMap(tProfile(),false);
        ret = DOORSILL_READY_STEP::START_TILTUP;
    }

    
    
    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::startTilUp(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::START_TILTUP;

    
    if(!MOTION.isRunning())
    {
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
        ret = DOORSILL_READY_STEP::RUN_TILTUP;
    }
    
    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::runTilUp(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::RUN_TILTUP;
    if(ServiceData.tilting.getStateValue() == E_SYS_TILT_STATE::TILTED_UP)
    {
        ServiceData.obstacle.initObstacleSensor();
        MOTION.startDriveWheelPwm(robotPose,0,0,0,0,0,false,tProfile());
        ret = DOORSILL_READY_STEP::CHECK_TOFCALIB;
    }
    
    return ret;
}

DOORSILL_READY_STEP CTaskAvoidDoorSill::checkTofAccumulate(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    DOORSILL_READY_STEP ret = DOORSILL_READY_STEP::CHECK_TOFCALIB;
    if(ServiceData.obstacle.isCliffAccumulate())
    {
        MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile()); 
        ret = DOORSILL_READY_STEP::COMPLETE;
    }

    return ret;
}

DOOR_SILL_STATE CTaskAvoidDoorSill::procDoorSillReady(tPose robotPose,RSU_OBSTACLE_DATA *pObstacle )
{
    DOOR_SILL_STATE ret = DOOR_SILL_STATE::RUN_READY;
    switch (step)
    {
     case DOORSILL_READY_STEP::NONE :

        break;
    case DOORSILL_READY_STEP::START_BALANCE :
        setDoorSillStep(startBalanceDoorSill(robotPose));
        break;
    case DOORSILL_READY_STEP::RUN_BALANCE :
        setDoorSillStep(runBalanceDoorSill(robotPose,pObstacle));
        break;
    case DOORSILL_READY_STEP::START_MOVE_BACK :
        setDoorSillStep(startMoveBack(robotPose,pObstacle));
        break;
    case DOORSILL_READY_STEP::RUN_MOVE_BACK :
        setDoorSillStep(runMoveBack(robotPose,pObstacle));
        break;    
    case DOORSILL_READY_STEP::START_TILTUP :
        setDoorSillStep(startTilUp(robotPose,pObstacle));
        break;
    case DOORSILL_READY_STEP::RUN_TILTUP :
        setDoorSillStep(runTilUp(robotPose,pObstacle));
        break;
    case DOORSILL_READY_STEP::CHECK_TOFCALIB :
        setDoorSillStep(checkTofAccumulate(robotPose,pObstacle));
        break;
    case DOORSILL_READY_STEP::COMPLETE :
        setDoorSillStep(DOORSILL_READY_STEP::NONE);
        ret = DOOR_SILL_STATE::CLIMB;
        break;                
    default:
        break;
    }

    return ret;
}