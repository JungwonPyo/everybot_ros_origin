#include "taskLocalMotion.h"

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
#include "taskLocalMotion.h"

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
CTaskLocalMotion::CTaskLocalMotion()
{
    CStopWatch __debug_sw;
    
    
    
    gridMapReadyTime = (u32)-1;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskLocalMotion::~CTaskLocalMotion()
{
    CStopWatch __debug_sw;

    
    setState(LOCALMOTION_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskLocalMotion::setState(LOCALMOTION_STATE set)
{
    if (set != state)
    {
        preState = state; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, CYN, "[LOCALMOTION_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

LOCALMOTION_STATE CTaskLocalMotion::step1(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::STEP_1;
    //탐색 시작 시점에서 지도 확장을 위해서 모션 필터 제한 품.
    ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_SKIP_MOTION_FILTER);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "STEP_1->STEP_1_WAIT" );

    //TS800에서는 카토에서 submap이 만들어지 진 후 occupancy gridmap 1 초 후 첫번째 grid map이 나옴
    if (SYSTEM_TOOL.getSystemTick()-gridMapReadyTime >= 1*SEC_1)
    {
        gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
        ret = LOCALMOTION_STATE::STEP_1_WAIT;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "STEP_1:정지 상태에서 1초만 대기 시키자.. 지도 업데이트 하게" );
    }

    return ret;
}

LOCALMOTION_STATE CTaskLocalMotion::step1_wait(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::STEP_1_WAIT;

    tProfile profile = tProfile();
    profile.desAngVel = DEG2RAD(20);
    profile.desLinVel = profile.minLinVel;
    CRobotKinematics k;
    
    //submap insert (0.2-> 0.1 초) -> occupancy gridmap 1 초 수신 대기 -> simplify map update is 2초
    //나중에 더 줄일 예정임(일단 안정방).
    //3초만 지나도 업데이트가 되긴 하지만 0.5초 더 주자..
    //if (SYSTEM_TOOL.getSystemTick()-gridMapReadyTime >= 3.5*SEC_1) {
        
    // 20 일 경우에도 됨
    // 최대한 낮추자.. 10일 경우
    localizeMotionTargetRad = k.rotation(robotPose, DEG2RAD(350)); 
    MOTION.startRotation(robotPose, localizeMotionTargetRad, profile, E_ROTATE_DIR::CCW);
    ret = LOCALMOTION_STATE::STEP_2;
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "STEP_1_WAIT->STEP_2" );  
    gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
    //}

    return ret;
}

LOCALMOTION_STATE CTaskLocalMotion::step2(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::STEP_2;

    //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는
    //카토그래퍼에서 모션 필터 동작하지 않고 무조건 서브맵 추가가 됨
    //너무 빠른 회전은 지도 성분이 나빠짐
    //최대한 천천히 회전하자 처음 로봇 주변 지도 확장을 위해서는
    // 20 일 경우에도 됨
    // 최대한 낮추자.. 10일 경우
    if (MOTION.isNearTargetRad(robotPose, localizeMotionTargetRad, DEG2RAD(5)))
    {
        gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
        ret = LOCALMOTION_STATE::CHECK_END;
    }

    
    return ret;
}

LOCALMOTION_STATE CTaskLocalMotion::checkEnd(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::CHECK_END;

    MOTION.startStopOnMap(tProfile(),false);
    ceblog(LOG_LV_NECESSARY, YELLOW, " One cycle of rotation has been completed.");        
    ret = LOCALMOTION_STATE::FINISH;

    return ret;
}

bool CTaskLocalMotion::finish(tPose robotPose)
{
    bool ret = false;
    
    
    if(!MOTION.isRunning()) ret = true;

    return ret;
}

LOCALMOTION_STATE CTaskLocalMotion::startMove(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::START_MOVE;

    createPath(robotPose);

    taskMovePath.taskStart(localMotionPath,0.2,tProfile());

    ret = LOCALMOTION_STATE::MOVE_TO_TARGET;
    
    return ret;
}

LOCALMOTION_STATE CTaskLocalMotion::moveToTarget(tPose robotPose)
{
    LOCALMOTION_STATE ret = LOCALMOTION_STATE::MOVE_TO_TARGET;

    if ( taskMovePath.taskRun(robotPose) )
    {
        MOTION.startStopOnMap(tProfile(),false);
        ret = LOCALMOTION_STATE::FINISH;
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
bool CTaskLocalMotion::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (state)
    {
    case LOCALMOTION_STATE::NONE :        
        break;
    case LOCALMOTION_STATE::START_MOVE :
        setState(startMove(robotPose));
        break;
    case LOCALMOTION_STATE::MOVE_TO_TARGET :
        setState(moveToTarget(robotPose));
        break;
    case LOCALMOTION_STATE::STEP_1 :
        setState(step1(robotPose));
        break;
    case LOCALMOTION_STATE::STEP_1_WAIT :
        setState(step1_wait(robotPose));
        break;    
    case LOCALMOTION_STATE::STEP_2 :
        setState(step2(robotPose));
        break;     
    case LOCALMOTION_STATE::CHECK_END :
        setState(checkEnd(robotPose));        
        break;
    case LOCALMOTION_STATE::FINISH :
        if(finish(robotPose)){
            setState(LOCALMOTION_STATE::NONE);
            ret = true;
        }
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskLocalMotion::taskStart(tPose robotPose)
{    
    //setState(LOCALMOTION_STATE::START_MOVE);
    gridMapReadyTime = SYSTEM_TOOL.getSystemTick();
    setState(LOCALMOTION_STATE::STEP_1);
}

/**
 * @brief 모양 만들기
 * 
 * @param robotPose 
 */
void CTaskLocalMotion::createPath(tPose robotPose)
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