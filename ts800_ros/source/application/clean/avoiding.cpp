#include "avoiding.h"
#include "control/control.h"
#include "motionPlanner/motionPlanner.h"
#include <systemTool.h>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CAvoiding::CAvoiding()
{
    CStopWatch __debug_sw;
    
    initAvoiding();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CAvoiding::~CAvoiding()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 장애물 회피 관련 데이터 초기화 함수
 * @param E_AVOID_STATUS    장애물 회피 상태 (대기/시작/회피중/완료) : 제어 알고리즘과 회피 알고리즘을 구분하기 위한 정보
 * @param E_AVOID_TYPE      장애물 회피 종류 (범퍼/둔턱/낙하 등) : 장애물 종류에 따른 회피 방법을 나누기 위한 정보
 * @param E_ESCAPE_STEP      장애물 회피 제어 STEP  : 장애물 회피 중 제어 시퀀스 
 * @param mask              avoid_mask 장애물 회피 방향 정보 : 장애물 감지 방향에 따른 회피 방향을 결정하기 위한 정보
 * @param startMoving       장애물 회피 시작 FLAG : 장애물 초기 회피 제어 보장하기 위한 FLAG
 * @return void
 * 
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim
 */
void CAvoiding::initAvoiding()
{
    CStopWatch __debug_sw;

    setAvoidStatus(E_AVOID_STATUS::CHECKER);
    setAvoidType(E_AVOID_TYPE::NONE);
    setEscapeStep(E_ESCAPE_STEP::NONE);
    setAvoidPose(tPose(0,0,0));
    setStartAvoidType(E_AVOID_TYPE::NONE);
    tempCheckObstacle = false;
    historyPose.clear();
    avoid_mask.value = 0;
    avoidStartTime = SYSTEM_TOOL.getSystemTime();
    bAvodingEndFlag = false; //avoiding 끝난 상태 유지
    isTurnningAvoid = false; 
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 회피 상태를 set하는 함수
 * @param E_AVOID_STATUS  장애물 회피 상태 (대기/시작/회피중/완료) : 제어 알고리즘과 회피 알고리즘을 구분하기 위한 정보
 * @param IDLE            장애물 회피 중이 아닌 상태                                                     -> START
 * @param START           장애물 감지 후 회피 시작 (회피 관련 정보를 SET하고 초기 회피를 시작한다)          -> RUN
 * @param RUN             장애물 회피 중 : 장애물 초기회피 이후 회피 제어 STEP을 RUN한다.                  -> START OR COMPLETE
 * @param COMPLETE         장애물 회피 완료 : 장애물 회피를 종료하고 기존 제어 알고리즘으로 돌아가기 위한 상태 -> IDLE 
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim 
 */
void CAvoiding::setAvoidStatus( E_AVOID_STATUS status )
{
    CStopWatch __debug_sw;
    if(avoidstatus != status)
    {
        std::string strNext = enumToString(status);
        std::string strCur = enumToString(avoidstatus);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "[setAvoidStatus] : "<<WHITE<<strCur<<BOLDBLACK<<" --> "<<CYN<<strNext);
    }
    avoidstatus = status;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 회피 상태를 get하는 함수
 * @param E_AVOID_STATUS  장애물 회피 상태 (대기/시작/회피중/완료) : 제어 알고리즘과 회피 알고리즘을 구분하기 위한 정보
 * @param IDLE            장애물 회피 중이 아닌 상태                                                     -> START
 * @param START           장애물 감지 후 회피 시작 (회피 관련 정보를 SET하고 초기 회피를 시작한다)          -> RUN
 * @param RUN             장애물 회피 중 : 장애물 초기회피 이후 회피 제어 STEP을 RUN한다.                  -> START OR COMPLETE
 * @param COMPLETE         장애물 회피 완료 : 장애물 회피를 종료하고 기존 제어 알고리즘으로 돌아가기 위한 상태 -> IDLE 
 * @return E_AVOID_STATUS
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
E_AVOID_STATUS CAvoiding::getAvoidStatus()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return avoidstatus;
}

/**
 * @brief 장애물 감지 시 장애물 정보를 저장하는 함수, 장애물 감지 우선순위는 아래와 같다.
 *        1.낙하(회피 기준 1순위) 2. 범퍼(감지 정확도 높음) 3. 전방IR(감지 정확도 후순위) 4. 휠트랩(감지하는 시간이 오래걸림)
 * @param E_AVOID_TYPE  장애물 종류 저장
 * @param AvoidPose     장애물 감지 위치 저장
 * @param AvoidMask     장애물 감지 방향 저장
 * @param AvoidTime     ToDo : 장애물 감지 시점 저장 추가 예정
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
void CAvoiding::updateAvoidInfo (tPose robotPose,  bool clean, bool isAvoiding)
{
    CStopWatch __debug_sw;
    u8 checkLidarMask = 0x3C;
    tProfile profile;
    
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    if(pObstacle->cliff.value)
    {
        ceblog(LOG_LV_NECESSARY, GRAY, "cliff obstacle catch !");
        setAvoidMask(pObstacle->cliff);
        setAvoidType(E_AVOID_TYPE::CLIFF);
    }
    else if(pObstacle->bumper.value)
    {
        if(clean 
        || (pObstacle->lidar.obstacle.value & 0x3C) 
        || pObstacle->front.obstacle.value 
        || pObstacle->front.approach.value 
        || pObstacle->tof.knoll.rangeAvg < 150)
        {
            ceblog(LOG_LV_NECESSARY, GRAY, "bumper obstacle catch !");
            setAvoidMask(pObstacle->bumper);
            setAvoidType(E_AVOID_TYPE::BUMPER);
            // profile.desLinVel = 0.2;
            // localizeMotionTarget = kinematics.translate(robotPose, 1.0, 0.0);
            // MOTION.startLinearToDistanceOnRobot(robotPose,-0.15,profile);
            eblog(LOG_LV_NECESSARY, "LIDAR OBS : ["<< SC<int>(pObstacle->lidar.obstacle.value) <<"]" << " APPROACH : ["<< SC<int>(pObstacle->lidar.approach.value) <<"]" << 
            "FRONT IR OBS : ["<< SC<int>(pObstacle->front.obstacle.value) <<"]" << " APPROACH : ["<< SC<int>(pObstacle->front.approach.value) <<"]" << "TOF KNOLL RANGE : ["<< pObstacle->tof.knoll.rangeAvg <<"]");
            eblog(LOG_LV_NECESSARY, "TOF CLIFF RANGE  LEFT : ["<< pObstacle->tof.lcliff.rangeAvg <<"]" << "RIGHT : ["<< pObstacle->tof.rcliff.rangeAvg <<"]" <<
            "Calib Avg  LEFT : ["<<  pObstacle->tof.leftCalibAvg <<
            "]" << "RIGHT : ["<< pObstacle->tof.rightCalibAvg <<"]");
        }
        else
        {
            #if SKIP_CHECKTILT > 0 //hjkim240220 : 둔턱회피 오류로 임시 block
            ceblog(LOG_LV_NECESSARY, GRAY, "knoll obstacle catch ! but Tilting Block Bumper Avoid Start!!!");
            setAvoidMask(pObstacle->bumper);
            // setAvoidType(E_AVOID_TYPE::BUMPER); //임시로 막음
            // MOTION.startLinearToDistanceOnRobot(robotPose,-0.1,tProfile());
            setAvoidType(E_AVOID_TYPE::BUMPER);
            MOTION.startLinearToDistanceOnRobot(robotPose,-0.1,tProfile());
            #else
            ceblog(LOG_LV_NECESSARY, GRAY, "knoll obstacle catch !");
            setAvoidMask(pObstacle->bumper);
            // setAvoidType(E_AVOID_TYPE::BUMPER); //임시로 막음
            // MOTION.startLinearToDistanceOnRobot(robotPose,-0.1,tProfile());
            setAvoidType(E_AVOID_TYPE::KNOLL);
			knollStatus = E_AVOID_KNOLL_STATUS::READY;
            knollStep = E_AVOID_KNOLL_STEP::START_BALANCE;
            #endif

            eblog(LOG_LV_NECESSARY, "LIDAR OBS : ["<< SC<int>(pObstacle->lidar.obstacle.value) <<"]" << " APPROACH : ["<< SC<int>(pObstacle->lidar.approach.value) <<"]" << 
            "FRONT IR OBS : ["<< SC<int>(pObstacle->front.obstacle.value) <<"]" << " APPROACH : ["<< SC<int>(pObstacle->front.approach.value) <<"]" << "TOF KNOLL RANGE : ["<< pObstacle->tof.knoll.rangeAvg <<"]");
            eblog(LOG_LV_NECESSARY, "TOF CLIFF RANGE  LEFT : ["<< pObstacle->tof.lcliff.rangeAvg <<"]" << "RIGHT : ["<< pObstacle->tof.rcliff.rangeAvg <<"]" <<
                "Calib Avg  LEFT : ["<<  pObstacle->tof.leftCalibAvg <<"]" << "RIGHT : ["<< pObstacle->tof.rightCalibAvg <<"]");
        }
    }
    else if(pObstacle->lidar.obstacle.value)
    {
        ceblog(LOG_LV_NECESSARY, GRAY, "Lidar obstacle catch ! : " << (int)pObstacle->lidar.obstacle.value);
        setAvoidMask(pObstacle->lidar.obstacle);
        setAvoidType(E_AVOID_TYPE::LIDAR);
    }

    else if(pObstacle->front.obstacle.value)
    {
        ceblog(LOG_LV_NECESSARY, GRAY, "front obstacle catch !");
        setAvoidMask(pObstacle->front.obstacle);
        setAvoidType(E_AVOID_TYPE::IR);
        profile.desLinVel = 0.2;        
    } 	
    else if(pObstacle->trap.value)
    {
        ceblog(LOG_LV_NECESSARY, GRAY, "trab obstacle catch !");
            setAvoidMask(pObstacle->trap);
            setAvoidType(E_AVOID_TYPE::WHEELTRAP);
            profile.desLinVel = 0.2;
    }
    
    else{
        ceblog(LOG_LV_NECESSARY, RED, "* \n\n\n");
        ceblog(LOG_LV_NECESSARY, RED, "처리할 수 없는 장애물 발견 !!!!!");
        ceblog(LOG_LV_NECESSARY, RED, "* \n\n\n");
    }

    if(getStartAvoidType() == E_AVOID_TYPE::NONE)
    {
        setStartAvoidType(getAvoidType());
    }
    setAvoidPose(robotPose);

    if(getAvoidPoseHistory().empty())
    {
        setAvoidPoseHistory(robotPose);
    }
    avoidStartTime = SYSTEM_TOOL.getSystemTime();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 종류 저장하는 함수
 * @param E_AVOID_TYPE  장애물 종류
 * @param IR        전방 (라이다 감지가 안되는 높이)
 * @param BUMPER    범퍼 (낮은 높이의 장애물)
 * @param CLIFF     낙하
 * @param WHEELTRAP 주행부 걸림
 * @param KNOLL     둔턱
 * @param LIDAR     라이다 -> ToDo : 지도에 정보가 저장되기 때문에 장애물 감지에서는 삭제 예정
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
void CAvoiding::setAvoidType(E_AVOID_TYPE type)
{
    CStopWatch __debug_sw;
    
    if(type == E_AVOID_TYPE::KNOLL) isKnollAvoiding = true;
    else                            isKnollAvoiding = false;
    // if(avoidtype != type)
    {
        std::string strNext = enumToString(type);
        std::string strCur = enumToString(avoidtype);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "[setAvoidType] : "<<WHITE<<strCur<<BOLDBLACK<<" --> "<<CYN<<strNext);
    }
    avoidtype = type;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 종류 RETURN하는 함수, 장애물 회피 제어를 담당하는 함수에서 장애물 TYPE에 따라 제어를 구분하기 위해 사용한다.
 * @param E_AVOID_TYPE  장애물 종류
 * @param IR        전방 (라이다 감지가 안되는 높이)
 * @param BUMPER    범퍼 (낮은 높이의 장애물)
 * @param CLIFF     낙하
 * @param WHEELTRAP 주행부 걸림
 * @param KNOLL     둔턱
 * @param LIDAR     라이다 -> ToDo : 지도에 정보가 저장되기 때문에 장애물 감지에서는 삭제 예정
 * @return E_AVOID_TYPE
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
E_AVOID_TYPE CAvoiding::getAvoidType()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return avoidtype;
}

void CAvoiding::setStartAvoidType(E_AVOID_TYPE type)
{
    CStopWatch __debug_sw;
    
    if(startAvoidtype != type)
    {
        std::string strNext = enumToString(type);
        std::string strCur = enumToString(startAvoidtype);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "[setStartAvoidType] : "<<WHITE<<strCur<<BOLDBLACK<<" --> "<<CYN<<strNext);
    }
    startAvoidtype = type;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 종류 RETURN하는 함수, 장애물 회피 제어를 담당하는 함수에서 장애물 TYPE에 따라 제어를 구분하기 위해 사용한다.
 * @param E_AVOID_TYPE  장애물 종류
 * @param IR        전방 (라이다 감지가 안되는 높이)
 * @param BUMPER    범퍼 (낮은 높이의 장애물)
 * @param CLIFF     낙하
 * @param WHEELTRAP 주행부 걸림
 * @param KNOLL     둔턱
 * @param LIDAR     라이다 -> ToDo : 지도에 정보가 저장되기 때문에 장애물 감지에서는 삭제 예정
 * @return E_AVOID_TYPE
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
E_AVOID_TYPE CAvoiding::getStartAvoidType()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return startAvoidtype;
}
/**
 * @brief 장애물 감지 시 장애물 감지 방향을 저장하는 함수, 총 8가지 방향을 1byte에 각 1bit단위로 저정한다. ToDo : 예전 방식의 장애물 방향정보 관리..좀더 좋은 방법으로 관리가 필요하다.
 * @param RSF_OBSTACLE_MASK  장애물 방향 정보
 * @param fright_side        오른쪽 측면
 * @param fright_Top_side    오른쪽 대각 측면
 * @param fright_Top_center  오른쪽 대각 정면
 * @param fright_center      오른쪽 정면
 * @param fleft_center       왼쪽 정면
 * @param fleft_Top_center   왼쪽 대각 정면
 * @param fleft_Top_side     왼쪽 대각 측면
 * @param fleft_side         왼쪽 측면
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
void CAvoiding::setAvoidMask(RSF_OBSTACLE_MASK mask)
{
    CStopWatch __debug_sw;

    avoid_mask = mask;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 감지 방향을 RETURN하는 함수, 총 8가지 방향을 1byte에 각 1bit단위로 저정한다. ToDo : 예전 방식의 장애물 방향정보 관리..좀더 좋은 방법으로 관리가 필요하다.
 * @param RSF_OBSTACLE_MASK  장애물 방향 정보
 * @param fright_side        오른쪽 측면
 * @param fright_Top_side    오른쪽 대각 측면
 * @param fright_Top_center  오른쪽 대각 정면
 * @param fright_center      오른쪽 정면
 * @param fleft_center       왼쪽 정면
 * @param fleft_Top_center   왼쪽 대각 정면
 * @param fleft_Top_side     왼쪽 대각 측면
 * @param fleft_side         왼쪽 측면
 * @return RSF_OBSTACLE_MASK
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
RSF_OBSTACLE_MASK CAvoiding::getAvoidMask()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return avoid_mask;
}
/**
 * @brief avoiding 끝났다는 1회성 플래그. dispathcher 주기에 의해 다음 실행때 삭제됨. 
 * 
 * @return true : 회피 끝났어요.
 * @return false : 회피 했는지는 잘 모름. true의 의미만 중요.
 */
bool CAvoiding::isAvoidingEnd()
{
    return bAvodingEndFlag;
}

/**
 * @brief Set the Avoiding End object
 * 
 * @param set 
 */
void CAvoiding::setAvoidingEnd(bool set)
{
    bAvodingEndFlag = set;
}

/**
 * @brief 범퍼 동작 시 motionavoid 동작 이후 탈출 루틴 만들기
 *
 * @param mask
 */
void CAvoiding::makeBumperEscapeAction(tPose robotPose,  CWayPoint &wayPoint)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();   
    wayPoint.clearAction();
    tAction action = tAction();

    //action 1 : 뒤로 20cm (장애물 무시)
    // action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
    // action.linear.targetDistance = -0.15;
    // action.needObstacleCheck = false;
    // wayPoint.pushAction(action);
    //action 2 : 우회전 90도
    action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;    
    if(mask.value & 0x0F)  action.rotate.targetAngle = DEG2RAD(90);
    else                   action.rotate.targetAngle = DEG2RAD(-90); 
    action.needObstacleCheck = true;
    wayPoint.pushAction(action);

    // // stop
    // action.type = E_ACTION_TYPE::STOP;
    // action.stop.waitForMs = 300;
    // wayPoint.pushAction(action);

    // //action 3 : 반원 회피
    // action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
    // action.curve.radius = 0.45;
    // action.curve.targetAngle = DEG2RAD(180);
    // action.needObstacleCheck = true;
    // wayPoint.pushAction(action);

    // // stop
    // action.type = E_ACTION_TYPE::STOP;
    // action.stop.waitForMs = 300;
    // wayPoint.pushAction(action);

    // //action 4 : 우회전 90도
    // action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;    
    // action.rotate.targetAngle = DEG2RAD(-90);
    // action.needObstacleCheck = true;
    // wayPoint.pushAction(action);

    ceblog(LOG_LV_NECESSARY, BLUE, "범퍼 회피 waypoint 생성 완료." );
}


/**
 * @brief LiDAR 장애물 발견 시 탈출 루틴 만들기
 * 
 * @param mask 
 */
void CAvoiding::makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();   
    wayPoint.clearAction();
    tAction action = tAction();
   
    //action 2 : 범퍼 방향에 따라 회전
    if(mask.value & 0xF0){
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        action.rotate.targetAngle = (DEG2RAD(10));
        action.needObstacleCheck = true;
        wayPoint.pushAction(action);
    } 
    else
    {
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        action.rotate.targetAngle = (DEG2RAD(-10));
        action.needObstacleCheck = true;
        wayPoint.pushAction(action);
    }

    //action 3 : 전진( 뒤로 후진 간격과 본체 크기만큼을 이동 )
    action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
    action.linear.targetDistance = 0.50; //
    action.needObstacleCheck = true;
    wayPoint.pushAction(action);

    //ceblog(LOG_LV_NECESSARY, RED, "범퍼 회피 mask : " << enumToString(pObstacle->lidar.obstacle) );
    ceblog(LOG_LV_NECESSARY, RED, "범퍼 회피 LiDAR 생성 실험 중... - avoiding" );
}


/**
 * @brief 낙하 동작 시 motionavoid 동작 이후 탈출 루틴 만들기
 * 
 * @param mask 
 */
void CAvoiding::makeCliffEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{   
    RSF_OBSTACLE_MASK mask = getAvoidMask();
    wayPoint.clearAction();
    tAction action = tAction();

    // action 1
    if(mask.b.fleft_side && mask.b.fright_side){
        // 오른쪽으로 90도 회전
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        action.rotate.targetAngle = DEG2RAD(-90.0);        
    } 
    else{
        if(mask.b.fleft_side){
            // 뒤로 좌회전 10 도 만큼 
            //TODO:radius 기능 생기면 변경해야함
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(10.0);
        }
        else{
            // 뒤로 우회전 10 도 만큼
            //TODO:radius 기능 생기면 변경해야함
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;            
            action.rotate.targetAngle = DEG2RAD(-10.0);
        }
    }
    wayPoint.pushAction(action);

    // action 2
    // 우회전 90도
    action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
    action.rotate.targetAngle = DEG2RAD(90.0);
    wayPoint.pushAction(action);

    ceblog(LOG_LV_NECESSARY, BLUE, "낙하 회피 waypoint 생성 완료." );
}


// void CAvoiding::makeIrEscapeAction(tPose robotPose, CWayPoint &wayPoint)
// {
    
//     RSF_OBSTACLE_MASK mask = getAvoidMask();   
//     wayPoint.clearAction();
//     tAction action = tAction();

//     //action 1 : 뒤로 20cm (장애물 무시)
//     action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
//     action.linear.targetDistance = -0.05;
//     action.needObstacleCheck = false;
//     wayPoint.pushAction(action);

//     // // stop
//     // action.type = E_ACTION_TYPE::STOP;
//     // action.stop.waitForMs = 300;
//     // wayPoint.pushAction(action);

//     // //action 2 : 우회전 90도
//     // action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;    
//     // action.rotate.targetAngle = DEG2RAD(90);
//     // action.needObstacleCheck = true;
//     // wayPoint.pushAction(action);
//     ceblog(LOG_LV_NECESSARY, RED, "IR 회피 waypoint 생성 완료" );
// }

void CAvoiding::makeWheelTrapEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    ceblog(LOG_LV_NECESSARY, RED, "WheelTrap 회피 waypoint 아직 안 만듦." );
}

void CAvoiding::makeKnollEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    
    RSF_OBSTACLE_MASK mask = getAvoidMask();
    wayPoint.clearAction();
    tAction action = tAction();

    if(pObstacle->lidar.obstacle.value & 0x3C)
    {
        if(pObstacle->lidar.obstacle.value & 0x0F)
        {
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(45.0);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);

            //action 3 : 반원 회피
            action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
            action.curve.radius = 0.15;
            action.curve.targetAngle = DEG2RAD(-90);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);
        }
        else
        {
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(-45.0);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);

            //action 3 : 반원 회피
            action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
            action.curve.radius = 0.15;
            action.curve.targetAngle = DEG2RAD(90);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);
        }
        ceblog(LOG_LV_NECESSARY, RED, "문턱 등반 불가 옆으로 이동해서 다시 시도해보자 " );
    }
    else
    {
        // action 1 : 가속을 얻기위해 뒤로 후진.
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.needObstacleCheck = false;
        action.linear.targetDistance = -0.1;  // 문지방 길이.
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::TILTING_UP;
        wayPoint.pushAction(action);

        // action 3 : 문턱을 올라가자.
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.needObstacleCheck = true;
        action.linear.targetDistance = 1.5;  // 문지방 길이.
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::TILTING_DOWN;
        wayPoint.pushAction(action);
        ceblog(LOG_LV_NECESSARY, RED, "Knoll 회피 wayPoint 완료" );
    }
}

void CAvoiding::makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();
    wayPoint.clearAction();
    tAction action = tAction();
    double targetHeading = 0.0;

    // action 1
    // action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
    // action.linear.targetDistance = -0.15;
    // action.needObstacleCheck = false;
    // wayPoint.pushAction(action);
    action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;    
    if(mask.value & 0x0F)  action.rotate.targetAngle = DEG2RAD(90);
    else                   action.rotate.targetAngle = DEG2RAD(-90); 
    action.needObstacleCheck = true;
    wayPoint.pushAction(action);

#if TEST_RANDOMPATTERN_FOR_CLEANTIME > 0
    if(mask.value & 0x0F)
    {            
        if(mask.b.fright_center || mask.b.fright_Top_center)//if(walltrack_.wface == wf_left)
        {
            // 오른쪽으로 90도 회전
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(-90.0);
        }
        else
        {
            // 왼쪽으로 90도 회전
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(90.0);             
        }        
    } 
    else 
    {
        if(mask.b.fleft_center || mask.b.fleft_Top_center){
            // 오른쪽으로 90도 회전
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(-90.0);
            
        }
        else{
            // 오른쪽으로 90도 회전
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(90.0);
        }
    }
    wayPoint.pushAction(action);
#else    
    // action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
    // action.rotate.targetAngle = DEG2RAD(90);
    // wayPoint.pushAction(action);    
#endif

    ceblog(LOG_LV_NECESSARY, RED, "front 회피 waypoint 임시 코드..생성. - CAvoiding" );
}

/**
 * @brief 장애물 감지 시 장애물 회피 STEP을 저장하는 함수, 스텝에 순서로만 의미가 부여되어 있음. ToDo : 제어에 따른 스텝이름으로 운영 고민 중
 * @param NONE   스텝 없음
 * @param FIRST  1번째 스텝
 * @param SECOND 2번째 스텝
 * @param THIRD  3번째 스텝
 * @param FORTH  4번째 스텝
 * @param END    마지막 스텝
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
void CAvoiding::setEscapeStep( E_ESCAPE_STEP step )
{
    CStopWatch __debug_sw;
    if(escapeStep != step)
    {
        std::string strNext = enumToString(step);
        std::string strCur = enumToString(escapeStep);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "[setEscapeStep] : "<<BOLDWHITE<<strCur<<BOLDBLACK<<" --> "<<BOLDCYAN<<strNext);
    }
    escapeStep = step;
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 회피 STEP을 반환하는 함수, 스텝에 순서로만 의미가 부여되어 있음. ToDo : 제어에 따른 스텝이름으로 운영 고민 중
 * @param NONE   스텝 없음
 * @param FIRST  1번째 스텝
 * @param SECOND 2번째 스텝
 * @param THIRD  3번째 스텝
 * @param FORTH  4번째 스텝
 * @param END    마지막 스텝
 * @return E_ESCAPE_STEP
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
E_ESCAPE_STEP CAvoiding::getEscapeStep ()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return escapeStep;
}

/**
 * @brief 장애물 감지 시 장애물 감지 위치를 저장하는 함수
 * @param tPose  x,y,angle 정보를 저장
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
void CAvoiding::setAvoidPose(tPose robotPose)
{
    CStopWatch __debug_sw;

    avoidPose = robotPose;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 장애물 감지 시 장애물 감지 위치 반환하는 함수
 * @param tPose  저장된 장애물 감지 위치 x,y,angle 정보를 반환
 * @return tPose
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
tPose CAvoiding::getAvoidPose(void)
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime()); 
    return avoidPose;
}

tPose CAvoiding::getAvoidStartPose()
{
    return startPose;
}

void CAvoiding::setAvoidPoseHistory(tPose robotPose)
{
    historyPose.push_back(robotPose);
}

std::list<tPose> CAvoiding::getAvoidPoseHistory()
{
    return historyPose;
}

/**
 * @brief 장애물 감지를 담당하는 함수, 장애물 감지 시 RETURN true를 반환한다.
 *        장애물 RUN을 시키기 위한 STEP을 초기단계로 설정한다. 필요에 따라 STEP을 다르게 설정할 수 있따.
 *        장애물 초기 회피가 완료되면, 장애물 회피상태를 RUN으로 변경한다. 
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
bool CAvoiding::checkObstacle(tPose robotPose, bool clean, bool isAvoiding)
{
    CStopWatch __debug_sw;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    bool ret = false;
    if( pObstacle->cliff.value ||            //cliff 장애물 발견
        pObstacle->bumper.value ||          //bumper 장애물 발견
        pObstacle->front.obstacle.value ||  //front 장애물 발견
        (pObstacle->lidar.obstacle.value & 0x7E) ||  //lidar 장애물 발견
        pObstacle->trap.value)              //trap 장애물 발견
    {   
        updateAvoidInfo(robotPose, clean, isAvoiding);
        ret = true;
        ceblog(LOG_LV_OBSTACLE, BOLDRED, "OBS check!!!");
        setAvoidingEnd(false);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 장애물 감지 후 초기회피 제어를 담당하는 함수, 장애물 초기 회피시 장애물 감지를 비활성화 한다. (연속회피 방지)
 *        장애물 RUN을 시키기 위한 STEP을 초기단계로 설정한다. 필요에 따라 STEP을 다르게 설정할 수 있따.
 *        장애물 초기 회피가 완료되면, 장애물 회피상태를 RUN으로 변경한다. 
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */
E_AVOID_STATUS CAvoiding::checker(tPose robotPose,  bool clean)
{
    CStopWatch __debug_sw;
    E_AVOID_STATUS ret = E_AVOID_STATUS::CHECKER;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if(checkObstacle(robotPose, clean, false))
    {
        if(clean)
        {
            // if(isTurnningAvoid)
            // {
            //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "청소 회전 중 장애물 감지 회피 시작");
            // }
            // else
            {
                if(getAvoidType() == E_AVOID_TYPE::KNOLL)
                {
                    ret = E_AVOID_STATUS::CHECKKNOLL;
                }   
                else
                {
                    setEscapeStep(E_ESCAPE_STEP::SET_WAYPOINT);
                    ret = E_AVOID_STATUS::MOTION_ESCAPE;
                }
                ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "청소 회피 시작"); 
            }
        }
        else
        {
            if(getAvoidType() == E_AVOID_TYPE::KNOLL)
            {
                ret = E_AVOID_STATUS::CHECKKNOLL;
            }   
            else
            {
                setEscapeStep(E_ESCAPE_STEP::SET_WAYPOINT);
                ret = E_AVOID_STATUS::MOTION_ESCAPE;
            }
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "이동 회피 시작");
        }             
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_AVOID_STATUS CAvoiding::knollchecker(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_AVOID_STATUS ret = E_AVOID_STATUS::CHECKKNOLL;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    switch (knollStatus)
    {
    case E_AVOID_KNOLL_STATUS::READY:
        knollStatus = readyAcrossDoorSill(robotPose);
        break;
     case E_AVOID_KNOLL_STATUS::CLIMB:
        knollStatus = climbDoorSill(robotPose,pObstacle);
        break;    
    case E_AVOID_KNOLL_STATUS::ACROSS:
        knollStatus = acrossDoorSill(robotPose,pObstacle);
    break;
    case E_AVOID_KNOLL_STATUS::COMPLETE:
        knollStatus = completeAcrossDoorSill();
        ret = E_AVOID_STATUS::COMPLETE;
    break;
    case E_AVOID_KNOLL_STATUS::FAIL:
        knollStatus = failAcrossDoorSill();
        ret = E_AVOID_STATUS::COMPLETE;
    break;        
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_AVOID_KNOLL_STATUS CAvoiding::readyAcrossDoorSill(tPose robotPose)
{
    E_AVOID_KNOLL_STATUS ret = E_AVOID_KNOLL_STATUS::READY;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    u8 checkLidarMask = 0x3C;
    //double checkTime = SYSTEM_TOOL.getSystemTime()-avoidStartTime;

    if((pObstacle->lidar.obstacle.value & checkLidarMask) || (pObstacle->lidar.approach.value & checkLidarMask) || 
        pObstacle->front.obstacle.value || /*pObstacle->front.approach.value ||*/ pObstacle->tof.knoll.rangeAvg < 100)  knollclearcount++;
    else                                                                                                                knollclearcount = 0;

    if(knollclearcount >= 25)//checkTime >= 5 ||
    {
        
        knollclearcount = 0;
        eblog(LOG_LV_NECESSARY, /*"CheckTime : ["<< checkTime <<"]" << */"bumper : " << (int)pObstacle->bumper.value << "Lidar obs : " << (int)pObstacle->lidar.obstacle.value << "Lidar approach : " << (int)pObstacle->lidar.approach.value );

        MOTION.startDriveWheelPwm(robotPose,1,-100,-100,20,0,true,tProfile());
        return E_AVOID_KNOLL_STATUS::FAIL;
    }

    switch (knollStep)
    {
    case E_AVOID_KNOLL_STEP::START_BALANCE :
        startBalanceDoorSill(robotPose);
        knollStep = E_AVOID_KNOLL_STEP::CHECK_BALANCE;
        break;
    case E_AVOID_KNOLL_STEP::CHECK_BALANCE :
        if(checkBalanceDoorSill(robotPose,pObstacle))
        {
            KnollHeading = robotPose.angle;
            eblog(LOG_LV_NECESSARY, "문턱과 밸런스 완료 Heading :" << RAD2DEG(KnollHeading));
            knollStep = E_AVOID_KNOLL_STEP::START_TILTUP;
        }
        break;
    case E_AVOID_KNOLL_STEP::START_TILTUP :
        if(!MOTION.isRunning())
        {
            knollStep = E_AVOID_KNOLL_STEP::CHECK_TILTUP;
            ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
        }
        break;
    case E_AVOID_KNOLL_STEP::CHECK_TILTUP :
        if(!MOTION.isRunning())
        {
            if(ServiceData.tilting.getStateValue() == E_SYS_TILT_STATE::TILTED_UP)
            {
                ServiceData.obstacle.initObstacleSensor();
                knollStep = E_AVOID_KNOLL_STEP::CHECK_TOFCALIB;
            }
        }
        break;
    case E_AVOID_KNOLL_STEP::CHECK_TOFCALIB :
        if(ServiceData.obstacle.isCliffAccumulate())
        {
            knollStep = E_AVOID_KNOLL_STEP::NONE;
            
            MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile()); 
            ret = E_AVOID_KNOLL_STATUS::CLIMB;
        }
        break;            
    default:
        break;
    }

    return ret;
}

E_AVOID_KNOLL_STATUS CAvoiding::climbDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    E_AVOID_KNOLL_STATUS ret = E_AVOID_KNOLL_STATUS::CLIMB;

    double diffHeading = RAD2DEG(KnollHeading)-RAD2DEG(robotPose.angle);

    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "climbDoorSill"<< 
    BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
    << BOLDBLACK<< " |KnollHeading : " << WHITE<< RAD2DEG(KnollHeading) << ", RobotHeading : " << RAD2DEG(robotPose.angle) << ", Diff Heading : " << diffHeading
    << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << "," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << ","
    << SC<int>(pObstacle->imu.Gpitch)<<","<<SC<int>(pObstacle->imu.Groll)<<","<<SC<int>(pObstacle->imu.Gyaw));
    //<< "|CServiceIdle sys pose:" << pServiceData->localiz.getSysPose().x << "," << pServiceData->localiz.getSysPose().y<< "," << pServiceData->localiz.getSysPose().angle);

    if(pObstacle->tof.lcliff.rangeAvg >= 230 && pObstacle->tof.rcliff.rangeAvg >= 230 && fabs(diffHeading) <= 2)
    {
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            MOTION.stop(1000);
            eblog(LOG_LV_NECESSARY, "등반 완료 1초 정지 후 낙하 판단!");
            ret = E_AVOID_KNOLL_STATUS::ACROSS;
        }
    }
    else
    {
        if(pObstacle->tof.lcliff.rangeAvg >= 230 && pObstacle->tof.rcliff.rangeAvg >= 230)
        {
            if(diffHeading > 0)     MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());
            else                    MOTION.startDriveWheelPwm(robotPose,1,100,0,-20,0,false,tProfile());
        }
        else if(pObstacle->tof.rcliff.rangeAvg >= 200 && pObstacle->tof.lcliff.rangeAvg < 200)
        {
            if(pObstacle->imu.Ax >= 500)
            {
                if(pObstacle->imu.Ay >= 1500 )  MOTION.startDriveWheelPwm(robotPose,1,100,0,-20,0,false,tProfile());
                else                            MOTION.startDriveWheelPwm(robotPose,1,150,50,-20,0,false,tProfile());
            }    
            //else                                                          MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile());
        }
        else if(pObstacle->tof.rcliff.rangeAvg < 200 && pObstacle->tof.lcliff.rangeAvg >= 200)
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

    return ret;
}


E_AVOID_KNOLL_STATUS CAvoiding::acrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    E_AVOID_KNOLL_STATUS ret = E_AVOID_KNOLL_STATUS::ACROSS;
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "acrossingDoorSill"<< 
    BOLDBLACK<< " |tof LEFT : " << WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", RIGHT : " << SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", KNOLL : " << SC<int>(pObstacle->tof.knoll.rangeAvg)
    << BOLDBLACK << " |imu:" << WHITE << SC<int>(pObstacle->imu.Ax) << " ," << SC<int>(pObstacle->imu.Ay)<<","<< SC<int>(pObstacle->imu.Az) << " ,"
    << SC<int>(pObstacle->imu.Gpitch)<<" ,"<<SC<int>(pObstacle->imu.Groll)<<" ,"<<SC<int>(pObstacle->imu.Gyaw));
    //<< "|CServiceIdle sys pose:" << pServiceData->localiz.getSysPose().x << "," << pServiceData->localiz.getSysPose().y<< "," << pServiceData->localiz.getSysPose().angle);

    if(pObstacle->tof.lcliff.rangeAvg > 300 || pObstacle->tof.rcliff.rangeAvg > 300)
    {
        eblog(LOG_LV_NECESSARY, "등반 중 낙하감지!! 후진");
        MOTION.startDriveWheelPwm(robotPose,1,-100,-100,20,0,true,tProfile());
        ret = E_AVOID_KNOLL_STATUS::FAIL;
    }
    else if(!MOTION.isRunning())
    {
        eblog(LOG_LV_NECESSARY, "등반 성공 안전지대 확인 넘어가자 고고싱");
        MOTION.startDriveWheelPwm(robotPose,1,100,100,20,0,true,tProfile());
    }
    else
    {
        if(pObstacle->tof.lcliff.rangeAvg <= 200 && pObstacle->tof.rcliff.rangeAvg <= 200 && abs(pObstacle->imu.Groll) <= 10 && abs(pObstacle->imu.Gpitch) <= 10 && abs(pObstacle->imu.Gyaw) <= 10 )
        {
            eblog(LOG_LV_NECESSARY, "등반 착지 완료");
            
            ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
            ret = E_AVOID_KNOLL_STATUS::COMPLETE;
        }
    }

    return ret;
}

E_AVOID_KNOLL_STATUS CAvoiding::completeAcrossDoorSill()
{
    E_AVOID_KNOLL_STATUS ret = E_AVOID_KNOLL_STATUS::COMPLETE;
    eblog(LOG_LV_NECESSARY, "문턱 등반 완료");
    return ret;
}

E_AVOID_KNOLL_STATUS CAvoiding::failAcrossDoorSill()
{
    E_AVOID_KNOLL_STATUS ret = E_AVOID_KNOLL_STATUS::FAIL;
    eblog(LOG_LV_NECESSARY, "문턱 등반 실패");
    return ret;
}

void CAvoiding::startBalanceDoorSill(tPose robotPose)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();
    if(mask.value & 0x0F)   MOTION.startDriveWheelPwm(robotPose,1,100,0,20,0,false,tProfile());
    else                    MOTION.startDriveWheelPwm(robotPose,1,0,100,20,0,false,tProfile());
}

bool CAvoiding::checkBalanceDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    bool ret = false;
    u8 checkLidarMask = 0x7E;
    
    double angleDiff = fabs(utils::math::rad2deg(robotPose.angle)-utils::math::rad2deg(poseTemp.angle));

    if(angleDiff < 0.5)     knollTrapcount++;
    else                    knollTrapcount = 0;

    if((knollTrapcount >= 50)) //|| (pObstacle->bumper.b.fleft_side && pObstacle->bumper.b.fright_side)
    {
        
        knollTrapcount = 0;
        if(pObstacle->lidar.obstacle.value)                         setAvoidMask(pObstacle->lidar.obstacle);
        else if(pObstacle->lidar.approach.value & checkLidarMask)   setAvoidMask(pObstacle->lidar.approach);
        else if(pObstacle->front.obstacle.value)                    setAvoidMask(pObstacle->front.obstacle);
        //else if(pObstacle->front.approach.value)                    setAvoidMask(pObstacle->front.approach);
        else                                                        setAvoidMask(pObstacle->bumper);

        
        MOTION.startDriveWheelPwm(robotPose,1,-50,-50,20,50,true,tProfile());                
        ret = true;
    }
                        
    eblog(LOG_LV_NECESSARY, "AngleDiff : ["<< angleDiff <<"]" << "count : " << (int)knollTrapcount);
    poseTemp = robotPose;

    return ret;
}

void CAvoiding::escapeStepRunWayPoint(tPose robotPose,  bool clean)
{    
    // 장애물 감지 체크   
    wayPointMng.runWayPoint();

    if (!wayPointMng.isActive()){
       setEscapeStep(E_ESCAPE_STEP::COMPLETE);
    }
}

/**
 * @brief 장애물 회피 중 회피제어를 담당하는 함수, 장애물 회피 STEP 제어를 하는 제어 함수를 RUN, 장애물 회피 완료를 확인한다. 장애물 회피 중 장애물 감지를 확인한다.
 *        장애물 회피 중 장애물 감지가 되면 장애물 회피 상태를 START로 설정한다.
 *        장애물 회피 중 완료 감지가 되면 장애물 회피 상태를 COMPLETE로 설정한다. 
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
E_AVOID_STATUS CAvoiding::MotionEscape(tPose robotPose,  bool clean)
{
    CStopWatch __debug_sw;

    E_AVOID_STATUS ret = E_AVOID_STATUS::MOTION_ESCAPE;

    switch ( getEscapeStep() )
    {    
    case E_ESCAPE_STEP::NONE :
        break;    
    case E_ESCAPE_STEP::SET_WAYPOINT :
        if(!MOTION.isRunning())
        {
            #if TEST_RANDOMPATTERN_FOR_CLEANTIME > 0
            setEscapeStep(E_ESCAPE_STEP::COMPLETE);
            #else
            startPose = robotPose;
            makeEscapeAction(robotPose, wayPoint);// 어떤 회피 패턴을 쓸지 결정 하자.
            wayPointMng.setWayPoint(wayPoint);
            wayPointMng.startWayPoint();
            ceblog(LOG_LV_NECESSARY, BLUE, "장애물 회피 후진 완료 시작점 저장 : " << robotPose.x << " , " << robotPose.y );
            setEscapeStep(E_ESCAPE_STEP::RUN_WAYPOINT);
            #endif
        }
        break;
    case E_ESCAPE_STEP::RUN_WAYPOINT:
      if( wayPointMng.getCurrentAction().needObstacleCheck == true )
        {   
            if(getAvoidType() == E_AVOID_TYPE::KNOLL)
            {
                if(checkObstacle(robotPose,clean,true))
                {
                    if(getAvoidType() == E_AVOID_TYPE::KNOLL)
                    {
                        ret = E_AVOID_STATUS::CHECKKNOLL;
                    }
                    else
                    {
                        setEscapeStep(E_ESCAPE_STEP::SET_WAYPOINT);
                    }
                    return ret;
                }  
            }
            else
            {
                if(checkObstacle(robotPose,clean,false))
                {
                    if(getAvoidType() == E_AVOID_TYPE::KNOLL)
                    {
                        ret = E_AVOID_STATUS::CHECKKNOLL;
                    }
                    else
                    {
                        setEscapeStep(E_ESCAPE_STEP::SET_WAYPOINT);
                    }
                    return ret;
                } 
            }  
        } 
        escapeStepRunWayPoint(robotPose,clean);        
        break;
    case E_ESCAPE_STEP::COMPLETE:
        ceblog(LOG_LV_OBSTACLE, BLUE, "COMPLETE - 회피동작 끝~" );
        // 회피 동작이 끝났다.
        setEscapeStep(E_ESCAPE_STEP::NONE);
        ret = E_AVOID_STATUS::COMPLETE;
        break;
    default:
        ceblog(LOG_LV_NECESSARY, BLUE, "없는 값 ㅜㅜ : "<<enumToString(getEscapeStep()) );
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;  
}  

/**
 * @brief 장애물 회피 완료를 담당하는 함수, 장애물 정보초기화 및 장애물 회피 상태를 IDLE 설정한다. 제어 알고리즘 상태와 연동을 담당한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
E_AVOID_STATUS CAvoiding::complete(tPose robotPose)
{
    // 회피 전에 했던 내용을 연결. (웨이 포인트 연결등 )

    CStopWatch __debug_sw;
    E_AVOID_STATUS ret = E_AVOID_STATUS::COMPLETE;
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();
    tAction action;

#if SKIP_CHECKTILT > 0
    setAvoidingEnd(true);
    ret = E_AVOID_STATUS::CHECKER;
#else
    eblog(LOG_LV_NECESSARY,  "avoidCompleted TYPE : " << enumToString(avoidtype) << "TiltState : " << (int)ServiceData.tilting.getStateValue());
    if (tiltState == E_SYS_TILT_STATE::TILTED_DOWN)
    {
        setAvoidingEnd(true);
        ret = E_AVOID_STATUS::CHECKER;        
    }
    else
    {
        action.type = E_ACTION_TYPE::TILTING_DOWN;
        MOTION.actionStart(action);
    }
#endif

    
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
    
}

/**
 * @brief 장애물 감지 이후 부터 회피 완료까지 모든 함수를 총괄하는 메인함수. 회피 상태에 따라 START/RUN/COMPLETE 시퀀스로 동작된다.
 * @param IDLE  장애물 감지여부를 선택한다. TODO : 장애물 감지 관련 함수 추가
 * @param START 장애물 초기 회피 제어 담당 : 대부분 후진 제어 처리를 실행한다. START 시점에서의 제어는 보장이 필요한다. (장애물 감지 비활성화)
 * @param RUN   장애물 회피 중 제어 담당 : 초기회피 이후 시점부터는 다시 장애물 감지를 활성하 한다.
 * @param COMPLETE 장애물 회피 완료 : 모든 정보를 초기화 하고, 장애물 회피 상태를 IDLE로 변경한다. 기존 제어 알고리즘과 연동해야한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
bool CAvoiding::avoidRun(tPose robotPose, bool clean)
{
    CStopWatch __debug_sw;
    bool ret = false;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    if(getAvoidStatus() != E_AVOID_STATUS::CHECKER)
    {
        ret = true;
    }
    
    try
    {
        switch ( getAvoidStatus() )        
        {        
        case E_AVOID_STATUS::CHECKER : // 장애물 정보 판단
            setAvoidStatus(checker(robotPose, clean));
            break;
        case E_AVOID_STATUS::CHECKKNOLL:
            setAvoidStatus(knollchecker(robotPose));
            break;        
        case E_AVOID_STATUS::MOTION_ESCAPE : // 회피 탈출, 탈출 중에 장애물을 다시 확인하는 동작이 들어간다.
            setAvoidStatus(MotionEscape(robotPose, clean));
            break;
        case E_AVOID_STATUS::COMPLETE : // 회피 완료.
            setAvoidStatus(complete(robotPose));
            break;        
        default:
            break;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";        
        std::cerr <<"AvoidRun : "<< e.what() << '\n';
    }

    if(getAvoidStatus() != E_AVOID_STATUS::CHECKER)
    {
        //initAvoiding();
        ret = true;
    }
    //ceblog(LOG_LV_NECESSARY, GREEN, "called : " << enumToString(getAvoidStatus()));
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
    
}

void CAvoiding::makeEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    switch (getAvoidType())
    {
    case E_AVOID_TYPE::CLIFF:
        makeCliffEscapeAction(robotPose, wayPoint);
        break;
    case E_AVOID_TYPE::BUMPER:
        makeBumperEscapeAction(robotPose, wayPoint);
        break;
    case E_AVOID_TYPE::IR:
        //makeIrEscapeAction(pObstacle, wayPoint);
        makeFrontEscapeAction(robotPose, wayPoint);
        //makeBumperEscapeAction(robotPose, wayPoint);
        break;
    case E_AVOID_TYPE::WHEELTRAP:
        makeWheelTrapEscapeAction(robotPose, wayPoint);
        break;
    case E_AVOID_TYPE::KNOLL:
        //makeBumperEscapeAction(robotPose,pObstacle, wayPoint);        // 틸팅이 waypoint 에서 동잘 할때 까지는 그냥 사용.
        makeKnollEscapeAction(robotPose, wayPoint);
        break;
    case E_AVOID_TYPE::LIDAR :
        makeLidarEscapeAction(robotPose, wayPoint);
        break;
    default:
        ceblog(LOG_LV_NECESSARY, BLUE, "없는 값 ㅜㅜ : " << enumToString(getAvoidType()));
        break;
    }
}

 bool CAvoiding::isTurnAvoiding()
 {
    return isTurnningAvoid;
 }
    


