
#include "avoidingMotionPlanner.h"
#include "motionPlanner/motionPlanner.h"
#include "kinematics.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CAvoidingMotionPlanner::CAvoidingMotionPlanner()
{
    CStopWatch __debug_sw;

    bIsUpdateObstacleWall = false;
    isRobotInWall = false;

    TIME_CHECK_END(__debug_sw.getTime());
}

CAvoidingMotionPlanner::~CAvoidingMotionPlanner()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

void CAvoidingMotionPlanner::setIsRobotInWall(bool set)
{
    isRobotInWall = set;
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
bool CAvoidingMotionPlanner::checkObstacle(tPose robotPose, bool clean, bool isAvoiding)
{
    CStopWatch __debug_sw;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    bool ret = false;
    if(//pObstacle->cliff.value ||            //cliff 장애물 발견
        pObstacle->bumper.value ||          //bumper 장애물 발견
        pObstacle->front.obstacle.value ||  //front 장애물 발견
        // (pObstacle->lidar.obstacle.value & 0X7E) ||  //lidar 장애물 발견
        pObstacle->trap.value)              //trap 장애물 발견
    {
		
        updateAvoidInfo(robotPose, clean, isAvoiding);        
        ret = true;
        ceblog(LOG_LV_NECESSARY, BOLDRED, "OBS check!!! cliff : " << (int)pObstacle->cliff.value << " bumper : " << (int)pObstacle->bumper.value << " front : " << (int)pObstacle->front.obstacle.value <<
         " trap : " << (int)pObstacle->trap.value);
        setEscapeStep(E_ESCAPE_STEP::SET_WAYPOINT);
        setAvoidingEnd(false);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}




/**
 * @brief LiDAR 장애물 발견 시 탈출 루틴 만들기
 * 
 * @param mask 
 */
void CAvoidingMotionPlanner::makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();   
    wayPoint.clearAction();
    tAction action = tAction();

    if(ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILTED_UP && ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILING_UP)
    {
        makeDstarObstacleWall(robotPose);
    }
    
    // //action 1 : 후진
    // action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
    // action.linear.targetDistance = -0.15;
    // action.needObstacleCheck = false;
    // wayPoint.pushAction(action);

    //action 2 : 그냥 멋있으라고.
    // action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
    // action.rotate.targetAngle = (DEG2RAD(10));
    // action.needObstacleCheck = true;
    // wayPoint.pushAction(action);
 
    // //action 3 : 그냥 멋있으라고.    
    // action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
    // action.rotate.targetAngle = (DEG2RAD(-10));
    // action.needObstacleCheck = true;
    // wayPoint.pushAction(action);
    
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    
    ceblog(LOG_LV_NECESSARY, RED, "mask : " << enumToString(pObstacle->lidar.obstacle) );
    ceblog(LOG_LV_NECESSARY, RED, "LiDAR 생성 실험 중... - CGlobalPathPlanner" );
}


void CAvoidingMotionPlanner::makeBumperEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    CStopWatch __debug_sw;

    RSF_OBSTACLE_MASK mask = getAvoidMask();   
    wayPoint.clearAction();
    tAction action = tAction();
    
    if( isRobotInWall == false)
    {
        if(ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILTED_UP && ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILING_UP)
        {
            makeDstarObstacleWall(robotPose);
        }
        //action 1 : 후진
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = -0.1;
        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "로봇이 벽이 아니에요.\t 0.1만 후진" << enumToString(mask) );
    }
    else
    {
        //action 1 : 후진
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = -0.3;
        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "로봇이 벽입니다. [0.3 후진] 액션 추가");
        if(mask.b.fleft_side && mask.b.fright_side)
        {
            action.type = E_ACTION_TYPE::STOP;
            action.stop.waitForMs = 1;
            wayPoint.pushAction(action);
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "양쪽 범퍼가 들어왔어요. ["<<BOLDRED<<"정지"<<BOLDBLACK<<"] 액션추가");
        }
        else if(mask.b.fleft_side == 1)
        {
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(-45);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "왼쪽 범퍼가 들어왔어요. ["<<BOLDGREEN<<"45도 우회전"<<BOLDBLACK<<"] 액션추가");
        }
        else if(mask.b.fright_side == 1)
        {
            action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
            action.rotate.targetAngle = DEG2RAD(45);
            action.needObstacleCheck = true;
            wayPoint.pushAction(action);
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "오른 범퍼가 들어왔어요. ["<<BOLDGREEN<<"45도 좌회전"<<BOLDBLACK<<"] 액션추가");
        }
        isRobotInWall = false;
    }
    ceblog(LOG_LV_NECESSARY, RED, "범퍼 회피 mask : " << enumToString(mask) );
    ceblog(LOG_LV_NECESSARY, RED, "범퍼 회피 waypoint 실험 중 - CGlobalPathPlanner" );

    TIME_CHECK_END(__debug_sw.getTime()); 
}

void CAvoidingMotionPlanner::makeKnollEscapeAction(tPose robotPose,CWayPoint &wayPoint)
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
        // // action 1 : 가속을 얻기위해 뒤로 후진.
        // action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        // action.needObstacleCheck = false;
        // action.linear.targetDistance = -0.1;  // 문지방 길이.
        // wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::TILTING_UP;
        wayPoint.pushAction(action);

        // action 3 : 문턱을 올라가자.
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.needObstacleCheck = true;
        action.linear.targetDistance = 1.5;  // 문지방 길이.
        action.profile.desLinVel = action.profile.minLinVel;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::TILTING_DOWN;
        wayPoint.pushAction(action);
        ceblog(LOG_LV_NECESSARY, RED, "Knoll 회피 wayPoint 완료" );
    }
}


void CAvoidingMotionPlanner::makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    CStopWatch __debug_sw;

#if 0 // TODO : 아래내용을 잘 정리해서 waypoint 로 만들자.
    E_ESCAPE_STEP ret = getEscapeStep();

    switch (getEscapeStep())
    {
    case E_ESCAPE_STEP::FIRST:
       if(mask.b.fleft_Top_center || mask.b.fleft_center || mask.b.fleft_Top_side || mask.b.fleft_side  )
        {
            // 왼쪽 라이다 감지시 오른쪽 회전 회피 제어
            eblog(LOG_LV_NECESSARY,  " 왼쪽 IR 장애물 감지! ");
            //ROBOT_CONTROL.wheel.DriveWheelControl (E_WHEEL_CONTROL::wheelcontrol_turnright , AVOID_TURN_SPEED, AVOID_TURN_SPEED, 45, CTRL_FLAG_ADJSPEED|CTRL_FLAG_OVERLAP);
        }
        else if (mask.b.fright_center || mask.b.fright_Top_center || mask.b.fright_Top_side || mask.b.fright_side )
        {
            // 오른쪽 라이다 감지시 오른쪽 회전 회피 제어
            eblog(LOG_LV_NECESSARY,  " 오른쪽 IR 장애물 감지!");
            //ROBOT_CONTROL.wheel.DriveWheelControl (E_WHEEL_CONTROL::wheelcontrol_turnleft, AVOID_TURN_SPEED, AVOID_TURN_SPEED, 45, CTRL_FLAG_ADJSPEED|CTRL_FLAG_OVERLAP);
        }
        obstaclePoint.x = ServiceData.localiz.getPose().x;
        obstaclePoint.y = ServiceData.localiz.getPose().y;
        ret = E_ESCAPE_STEP::SECOND;
        break;
    case E_ESCAPE_STEP::SECOND:
        if(!MOTION_PLANNER.isRobotMoving())
        {
            //ROBOT_CONTROL.wheel.DriveWheelControl ( E_WHEEL_CONTROL::wheelcontrol_forward, CLEAN_SPEED, CLEAN_SPEED, 300, CTRL_FLAG_ADJSPEED|CTRL_FLAG_OVERLAP,true);
            ret = E_ESCAPE_STEP::THIRD;
        }
        
        break;
    case E_ESCAPE_STEP::THIRD:
        if(utils::math::distanceTwoPoint(obstaclePoint, ServiceData.localiz.getPose()) > 0.2 || !MOTION_PLANNER.isRobotMoving())
        {
            ret = E_ESCAPE_STEP::END;
            MOTION_PLANNER.robotStop();
            eblog(LOG_LV_PATHPLAN,  " 현재 회피한 거리 : " << utils::math::distanceTwoPoint(obstaclePoint,  ServiceData.localiz.getPose()) << "m , 목표 회피 거리 : 0.2m ");
        }
        /* code */
        break;
    case E_ESCAPE_STEP::FORTH:
        /* code */
        break;
    case E_ESCAPE_STEP::END:
        
        break;	
    default:
        break;
    }

#else
    RSF_OBSTACLE_MASK mask = getAvoidMask();   
    wayPoint.clearAction();
    tAction action = tAction();

    //action 1
    action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
    action.linear.targetDistance = -0.3;
    wayPoint.pushAction(action);

    ceblog(LOG_LV_NECESSARY, RED, "front 회피 waypoint 임시로 만듬 - CGlobalPathPlanner" );

#endif
    TIME_CHECK_END(__debug_sw.getTime());
    
}

bool CAvoidingMotionPlanner::isUpdateObstacleWall()
{
    return bIsUpdateObstacleWall;
}

std::list <tDstarWallPoint> CAvoidingMotionPlanner::useDstarObstacleWall()
{
    bIsUpdateObstacleWall = false;
    return obstacleWall;
}

void CAvoidingMotionPlanner::makeDstarObstacleWall(tPose robotPose)
{
    tDstarWallPoint pt;
    CRobotKinematics rk;
    double deltaX = 0.0, deltaY = 0.0;

    for (int x = 0; x <= 10; x++)
    {
        for(int y=-15; y <= 15; y++)
        {
            deltaX = (double)x *0.01;
            deltaY = (double)y *0.01;
            tPoint getPose = rk.translate(robotPose, deltaX, deltaY);

            utils::coordination::convertCoord2CellCoord(getPose.x, getPose.y, &pt.x, &pt.y);
            pt.costValue = -1;            

            // 특정 좌표가 벡터에 없을 때만 푸시
            //TODO : dstar setObstacleWall() 함수에서 처리하게 변경
            if (std::find(obstacleWall.begin(), obstacleWall.end(), pt) == obstacleWall.end()) {
                obstacleWall.push_back(pt);
                std::cout << "["<<getPose.x <<" , " << getPose.y<<" ], ";     
            }            
        }
        std::cout <<std::endl;  
    }

    bIsUpdateObstacleWall = true;    

    ceblog(LOG_LV_NECESSARY, GREEN, "robot pose : " <<robotPose.x<<" , "<<robotPose.y<<" , "<< RAD2DEG( robotPose.angle ));    
    
}
