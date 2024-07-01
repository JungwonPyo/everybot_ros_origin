#include "taskReLocalMotion.h"

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
#include "taskReLocalMotion.h"
#include "control/control.h"

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
CTaskReLocalMotion::CTaskReLocalMotion()
{
    CStopWatch __debug_sw;
    
    gridMapReadyTime = (u32)-1;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskReLocalMotion::~CTaskReLocalMotion()
{
    CStopWatch __debug_sw;

    
    setState(RE_LOCALMOTION_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskReLocalMotion::setState(RE_LOCALMOTION_STATE set)
{
    if (set != state)
    {
        preState = state; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, CYN, "[RE_LOCALMOTION_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::step_wait(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::STEP_WAIT;
 
    //1초 후 occupancy grid map이 업데이트가 발생을 한다
    //슬램 킨 후 정지 상태(1초)에서 지도 확장을 위해서 잠깐 모두 데이터 다 받자.
    //1초 정도 지연을 주니까 360도 한번만 회전 시켜도 맵이 다 확장이 됨
    //-360도 회전할 필요가 있을지.. 일단 2회 회전 시킴.
    ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_WAIT");
    if (SYSTEM_TOOL.getSystemTick()-gridMapReadyTime >= 1*SEC_1)
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT); 
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_WAIT->STEP_1 ");
        ret = RE_LOCALMOTION_STATE::STEP_1;
    }
    else
    {
        //빠른 지도 매칭이 완료되면 다시 필터 기본으로
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "정지 상태에서 잠시 대기");
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_SKIP_MOTION_FILTER);  
    }

    return ret;
}
RE_LOCALMOTION_STATE CTaskReLocalMotion::step1(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::STEP_1;

    //slam is on 시작
    if (ROBOT_CONTROL.slam.isSlamRunning())
    {
        //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는
        //카토그래퍼에서 모션 필터 동작하지 않고 무조건 서브맵 추가가 됨
        //너무 빠른 회전은 지도 성분이 나빠짐
        //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는

        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_STOP_LOTATE_INSERT_MAP);

        tProfile profile = tProfile();
        profile.desAngVel = DEG2RAD(20);
        profile.desLinVel = profile.minLinVel;
        
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_1 ");
        CRobotKinematics k;
        localizeMotionTargetRad = k.rotation(robotPose, DEG2RAD(350));

        MOTION.startRotation(robotPose, localizeMotionTargetRad, profile, E_ROTATE_DIR::CCW);
        gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
        ret = RE_LOCALMOTION_STATE::STEP_2;
    }

    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::step2(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::STEP_2;

    //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는
    //카토그래퍼에서 모션 필터 동작하지 않고 무조건 서브맵 추가가 됨
    //너무 빠른 회전은 지도 성분이 나빠짐
    //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는
    //타겟 각도 동일한 각도
    //E_ROTATE_DIR::CCW(+) -> E_ROTATE_DIR::CW(-)
    tProfile profile = tProfile();
    profile.desAngVel = DEG2RAD(20);
    profile.desLinVel = profile.minLinVel;

    if (MOTION.isNearTargetRad(robotPose, localizeMotionTargetRad, DEG2RAD(5)))
    {
        //지도 업데이트는 기본으로 한상태에서 반대 방향으로 돌자
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);

        CRobotKinematics k;
        localizeMotionTargetRad = k.rotation(robotPose, DEG2RAD(-350));
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_2->STEP_3 ");    
        MOTION.startRotation(robotPose, localizeMotionTargetRad, profile, E_ROTATE_DIR::CW);
        gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
        ret = RE_LOCALMOTION_STATE::STEP_3;
    }
    else
    {
        //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_2: 슬램 좌표 : " << slamPose.x << " , " << slamPose.y << " , " << slamPose.angle << " ");
        
    }

    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::step3(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::STEP_3;

    if (MOTION.isNearTargetRad(robotPose, localizeMotionTargetRad, DEG2RAD(5)))
    {
        //빠른 지도 매칭이 완료되면 다시 필터 기본으로
        //ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_3->CHECK_END" );

        //라이더 데이터와 gridmap 데이터에 대한 일치 여부를 IPC로 확인하면 간단하게
        //확인하면 되는데.. PCL libarary에서 지원하는 IPC가 좀 무거워서.. 
        //가벼운 방법이 없을까???

        gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
        ret = RE_LOCALMOTION_STATE::CHECK_END;
    }
    else
    {
        //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "STEP_3");
        
    }

    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::checkEnd(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::CHECK_END;

    MOTION.startStopOnMap(tProfile(),false);

    //1초만 일단 대기 - 맵 매칭..
    if (SYSTEM_TOOL.getSystemTick()-gridMapReadyTime >= 1*SEC_1)
    {
        //확실한 종료 조건이 필요하다.. 시간이 아닌.. PCL LIB.를 이용해서 lidar랑 맵매칭 확인할 수 있긴함.
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, YELLOW, " One cycle of rotation has been completed."); 
        ret = RE_LOCALMOTION_STATE::FINISH;
    } 
    else
    {
        ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, YELLOW, "마지막 맵 매칭을 위한 잠시 대기."); 
    }

    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::finish(tPose slamPose, tPose systemPose, tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::FINISH;

    ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, YELLOW, " FINISH."); 
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    ROBOT_CONTROL.slam.setRelocalMotionCompleted(true);

    //stop
    if(MOTION.isRunning())
    {
        MOTION.startStopOnMap(tProfile(),true);
    }

    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::startMove(tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::START_MOVE;

    createPath(robotPose);

    taskMovePath.taskStart(localMotionPath,0.2,tProfile());

    ret = RE_LOCALMOTION_STATE::MOVE_TO_TARGET;
    
    return ret;
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::moveToTarget(tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::MOVE_TO_TARGET;

    if ( taskMovePath.taskRun(robotPose) )
    {
        MOTION.startStopOnMap(tProfile(),false);
        ret = RE_LOCALMOTION_STATE::FINISH;
    }

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
bool CTaskReLocalMotion::taskRun(tPose slamPose, tPose systemPose, tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (state)
    {
    case RE_LOCALMOTION_STATE::NONE :        
        break;
    //extra
    case RE_LOCALMOTION_STATE::START_MOVE :
        setState(startMove(robotPose));
        break;
    case RE_LOCALMOTION_STATE::MOVE_TO_TARGET :
        setState(moveToTarget(robotPose));
        break;
    case RE_LOCALMOTION_STATE::STEP_EXIT_SLAM :        
        //대기 중
        break;
    case RE_LOCALMOTION_STATE::STEP_RUN_SLAM :
        setState(stepRunSlam(robotPose));
        break;
    //extra
    case RE_LOCALMOTION_STATE::STEP_WAIT : //지도 확장을 위해서 잠시 대기 시키자
        setState(step_wait(slamPose, systemPose, robotPose));
        break;
    case RE_LOCALMOTION_STATE::STEP_1 : // + 358
        setState(step1(slamPose, systemPose, robotPose));
        break;
    case RE_LOCALMOTION_STATE::STEP_2 : // -358
        setState(step2(slamPose, systemPose, robotPose));
        break;
    case RE_LOCALMOTION_STATE::STEP_3 : //지도 맵매칭을 위한 잠시 대기 <- 여기 종료 조건이 되는 부분에 대해서 보완이 필요함
        setState(step3(slamPose, systemPose, robotPose));
        break;
    case RE_LOCALMOTION_STATE::CHECK_END :
        setState(checkEnd(slamPose, systemPose, robotPose));
        break;
    case RE_LOCALMOTION_STATE::FINISH :
        setState(finish(slamPose, systemPose, robotPose));
        ret = true;
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskReLocalMotion::taskStart(tPose slamPose, tPose systemPose, tPose robotPose)
{
    gridMapReadyTime = SYSTEM_TOOL.getSystemTick();

    //처음 시작 시 리로컬 완료 여부 플래그 초기화 
    ROBOT_CONTROL.slam.setRelocalMotionCompleted(false);

    startSlamPose = slamPose; //slam pose
    startSystemPose = systemPose; //system pose
    startRobotPose = robotPose; //slam + system fusion pose

    setState(RE_LOCALMOTION_STATE::STEP_WAIT); //STAP_1 => STAP_WAIT
}

void CTaskReLocalMotion::taskExitSlam(tPose robotPose)
{
    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "taskExitSlam");

    ROBOT_CONTROL.slam.exitSlam();
    setState(RE_LOCALMOTION_STATE::STEP_EXIT_SLAM);
}

void CTaskReLocalMotion::taskRunSlam(tPose robotPose)
{
    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "taskRunSlam");

    //로봇 멈추고    
    MOTION.startStopOnMap(tProfile(),true);
    

    //IMU 초기화
    ROBOT_CONTROL.clearSystemLocalization();
    ROBOT_CONTROL.system.initImuSensor();

    //슬램을 키기 위해서 다음 스탬
    setState(RE_LOCALMOTION_STATE::STEP_RUN_SLAM);
}

RE_LOCALMOTION_STATE CTaskReLocalMotion::stepRunSlam(tPose robotPose)
{
    RE_LOCALMOTION_STATE ret = RE_LOCALMOTION_STATE::STEP_RUN_SLAM;
    
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    
    E_IMU_STATUS status = (E_IMU_STATUS)(pObstacle->imu.filteredState);

    eblog(LOG_LV_NECESSARY,"\t"<< "|imu:" << SC<int>(status) );
    eblog(LOG_LV_NECESSARY,"\t"<< "|lift:" << SC<int>(pObstacle->isLift) ); //bool

    if (status == E_IMU_STATUS::IMU_READY)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), GREEN, "IMU Sensor OK ! -> slam on");
        ROBOT_CONTROL.slam.setSlamLocalUpdate(false);
        ROBOT_CONTROL.slam.runSlam();
        ret = RE_LOCALMOTION_STATE::NONE;
    }
    else
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), GREEN, "IMU Sensor NK !");
    }

    return ret;
}

/**
 * @brief 모양 만들기
 * 
 * @param robotPose 
 */
void CTaskReLocalMotion::createPath(tPose robotPose)
{
    CRobotKinematics k;
    tPoint pt;

    localMotionPath.clear();
        

    pt = k.translate(pt , 0.5, 0.5);
    localMotionPath.push_back(pt);

    pt = k.translate(pt , 0.0, -1.0);
    localMotionPath.push_back(pt);

    pt = k.translate(pt , -0.5, -0.5);
    localMotionPath.push_back(pt);
    
}