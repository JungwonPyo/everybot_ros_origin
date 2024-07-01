#include "taskWalltrackAvoid.h"
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


CTaskWallTrackAvoid::CTaskWallTrackAvoid()
{
    CStopWatch __debug_sw;
    setWallAvoidState(WALLTRACK_AVOID_STATE::NONE);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskWallTrackAvoid::~CTaskWallTrackAvoid()
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskWallTrackAvoid::setWallAvoidState(WALLTRACK_AVOID_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

bool CTaskWallTrackAvoid::isReturnStartPoint()
{
    return bReturnFromStart;
}


void CTaskWallTrackAvoid::taskStart2(tPose robotPose , E_WALLTRACK_DIR dir)
{
    bReturnFromStart = false;
    bCheckAwayFrontStart = false;
    startPoint = tPoint(robotPose.x,robotPose.y);
    wallDir = dir;
    SUB_TASK.walltracking.startAccumulateAngle(robotPose);
    avoidStartTime = SYSTEM_TOOL.getSystemTime();
    setWallAvoidState(WALLTRACK_AVOID_STATE::RUN_WALLTRACK_AVOID);
}

void CTaskWallTrackAvoid::taskRun2(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    double runTime = SYSTEM_TOOL.getSystemTime()-avoidStartTime;
  
    procWalltrackAvoid(robotPose);

    if(fabs(SUB_TASK.walltracking.getAccumulateAngle(robotPose)) >= 360)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 중 제자리멤돔!");
        bReturnFromStart = true;
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskWallTrackAvoid::taskStart(tPose robotPose, std::list<tPoint> _path, tPoint _goal,E_WALLTRACK_DIR dir)
{
    bReturnFromStart = false;
    bCheckAwayFrontStart = false;
    startPoint = tPoint(robotPose.x,robotPose.y);
    path = _path;
    goal = _goal;
    wallDir = dir;
    SUB_TASK.walltracking.startAccumulateAngle(robotPose);
    avoidStartTime = SYSTEM_TOOL.getSystemTime();
    setWallAvoidState(WALLTRACK_AVOID_STATE::RUN_WALLTRACK_AVOID);
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
bool CTaskWallTrackAvoid::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    double runTime = SYSTEM_TOOL.getSystemTime()-avoidStartTime;
    double result = 0;
    double remainDistance = robotPose.distance(goal);
    double targetRad = utils::math::getTurnRadAngle(goal,robotPose);//utils::math::getRadianAngle(curGoal,robotPose);
    tPoint nearst = searchNearstPointFromPath(robotPose);
    tPoint lineCoffeStartPoint;
    
    if(path.size() >= 2)
    {
        if(nearst == goal)
        {
            lineCoffeStartPoint = startPoint;
        }
        else
        {
            lineCoffeStartPoint = nearst;
        }
    }
    else
    {
        lineCoffeStartPoint = startPoint;
    }

    result = utils::math::calculateDistanceFromLine(tPoint(robotPose.x, robotPose.y), utils::math::calculateLineEquation(lineCoffeStartPoint, goal));
  
    procWalltrackAvoid(robotPose);
    // double dist = utils::math::distanceTwoPoint(tPoint(robotPose.x, robotPose.y), path.front());
    // dist = fabs(dist);

    // // 벽타기 시작할때 보다 거리가 더 멀어지는지 판단 하려고.
    // double diff = fabs(distTarget - dist);

    // // ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 경로 이동 중 장애물 회피 벽타기 종료 조건 확인 : " << result << " 누적각도 : " << wallAngle << " 목표와의 거리 : "<< remainDistance << " 목표와의 각도 : " << RAD2DEG(targetRad) <<
    // // " 라인 시작 좌표 : " << lineCoffeStartPoint.x << " , " << lineCoffeStartPoint.y << " 목표점 : " << curGoal.x << " , " << curGoal.y << " runTime : " << runTime << " 거리 : " << diff);

    // if (diff > 1.0)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료 :" << diff);
    // }

    if(fabs(SUB_TASK.walltracking.getAccumulateAngle(robotPose)) >= 360)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 중 제자리멤돔!");
        bReturnFromStart = true;
    }

    if(runTime >= CONFIG.avoidWalltrackEndCheckTime && (result < 0.15))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료 !! 목적 라인 도착 : " << result );
        ret = true;
    }

    if(runTime >= 10 && fabs(robotPose.distance(goal)-utils::math::distanceTwoPoint(goal,startPoint)) >= 1)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 시작점에서 목표점까지 보다 1m 이상 멀어짐");
        ret = true;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskWallTrackAvoid::procWalltrackAvoid(tPose robotPose)
{
    switch (state)
    {
    case WALLTRACK_AVOID_STATE::NONE :
        break;
    case WALLTRACK_AVOID_STATE::START :
        //setWallAvoidState();
        break;
    case WALLTRACK_AVOID_STATE::RUN_WALLTRACK_AVOID :
        setWallAvoidState(runWalltrackAvoid(robotPose));
        break;
    case WALLTRACK_AVOID_STATE::AVOID_CHARGER :
        setWallAvoidState(runChargerAvoid(robotPose));
        break;
    case WALLTRACK_AVOID_STATE::COMPLETE :
        //setWallAvoidState();
        break;            
    default:
        break;
    }
}

WALLTRACK_AVOID_STATE CTaskWallTrackAvoid::runWalltrackAvoid(tPose robotPose)
{
    WALLTRACK_AVOID_STATE ret = WALLTRACK_AVOID_STATE::RUN_WALLTRACK_AVOID;
    SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);

   if(isStartChargerAvoid(getAvoidCharegerSignalData(wallDir)))
   {
        taskAvoidCharger.taskStart(wallDir);
        ret = WALLTRACK_AVOID_STATE::AVOID_CHARGER;  
   } 
   return ret;
}

WALLTRACK_AVOID_STATE CTaskWallTrackAvoid::runChargerAvoid(tPose robotPose)
{
    WALLTRACK_AVOID_STATE ret = WALLTRACK_AVOID_STATE::AVOID_CHARGER;
    if(taskAvoidCharger.taskRun(robotPose)) ret = WALLTRACK_AVOID_STATE::RUN_WALLTRACK_AVOID;

    return ret;
}

tPoint CTaskWallTrackAvoid::searchNearstPointFromPath(tPose robotPose)
{
    double minDist = std::numeric_limits<double>::max();
    tPoint nearstPoint;
    std::list<tPoint>::iterator closest = path.begin();

    for (auto it = path.begin(); it != path.end(); ++it)
	{
        tPoint pt = tPoint(it->x,it->y);
		double dist = robotPose.distance(pt);

        if(dist < minDist)
        {
            minDist = dist;
            closest = it;
            nearstPoint = pt;
            if(closest != path.begin())
            {
                ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "경로 중에서 현재 위치와 가장 가까운 좌표는 : " << nearstPoint.x << " , " << nearstPoint.y << "로봇좌표 : " << robotPose.x << " ," << robotPose.y << " 로봇과 목표점과 의 거리 : " << minDist);
            }
            break;
        }
	}

    if(closest != path.begin())
    {
        path.erase(path.begin(),--closest);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 경로에서 가까워진 경로를 기반으로 경로를 수정합니다 경로 개수 : " << path.size());

        for(tPoint point : path)
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 수정된 경로  X : " << point.x << " , Y :  " << point.y);
        }
    }

    // ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "경로 중에서 현재 위치와 가장 가까운 좌표는 : " << nearstPoint.x << " , " << nearstPoint.y << "로봇좌표 : " << robotPose.x << " ," << robotPose.y << " 로봇과 목표점과 의 거리 : " << minDist);

    return nearstPoint;
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
u8 CTaskWallTrackAvoid::getLongSignalMatchedByShortSignal(u8 shortSignal)
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
u8 CTaskWallTrackAvoid::getAvoidCharegerSignalData(E_WALLTRACK_DIR dir)
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
bool CTaskWallTrackAvoid::isStartChargerAvoid(u8 avoidSignal)
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