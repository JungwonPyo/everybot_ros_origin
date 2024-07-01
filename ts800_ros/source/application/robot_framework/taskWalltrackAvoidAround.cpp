#include "taskWalltrackAvoidAround.h"
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


CTaskWallTrackAvoidAround::CTaskWallTrackAvoidAround()
{
    CStopWatch __debug_sw;
    setWallAvoidState(WALLTRACK_AVOID_AROUND_STATE::NONE);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskWallTrackAvoidAround::~CTaskWallTrackAvoidAround()
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskWallTrackAvoidAround::setWallAvoidState(WALLTRACK_AVOID_AROUND_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

bool CTaskWallTrackAvoidAround::isReturnStartPoint()
{
    return bReturnFromStart;
}


void CTaskWallTrackAvoidAround::taskStart(tPose robotPose , E_WALLTRACK_DIR dir)
{
    bReturnFromStart = false;
    bCheckAwayFrontStart = false;
    startPoint = tPoint(robotPose.x,robotPose.y);
    wallDir = dir;
    SUB_TASK.walltracking.startAccumulateAngle(robotPose);
    avoidStartTime = SYSTEM_TOOL.getSystemTime();
    setWallAvoidState(WALLTRACK_AVOID_AROUND_STATE::RUN_WALLTRACK_AVOID);
}

void CTaskWallTrackAvoidAround::taskRun(tPose robotPose)
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


void CTaskWallTrackAvoidAround::procWalltrackAvoid(tPose robotPose)
{
    switch (state)
    {
    case WALLTRACK_AVOID_AROUND_STATE::NONE :
        break;
    case WALLTRACK_AVOID_AROUND_STATE::START :
        //setWallAvoidState();
        break;
    case WALLTRACK_AVOID_AROUND_STATE::RUN_WALLTRACK_AVOID :
        setWallAvoidState(runWalltrackAvoid(robotPose));
        break;
    case WALLTRACK_AVOID_AROUND_STATE::AVOID_CHARGER :
        setWallAvoidState(runChargerAvoid(robotPose));
        break;
    case WALLTRACK_AVOID_AROUND_STATE::COMPLETE :
        //setWallAvoidState();
        break;            
    default:
        break;
    }
}

WALLTRACK_AVOID_AROUND_STATE CTaskWallTrackAvoidAround::runWalltrackAvoid(tPose robotPose)
{
    WALLTRACK_AVOID_AROUND_STATE ret = WALLTRACK_AVOID_AROUND_STATE::RUN_WALLTRACK_AVOID;
    SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);

   if(isStartChargerAvoid(getAvoidCharegerSignalData(wallDir)))
   {
        taskAvoidCharger.taskStart(wallDir);
        ret = WALLTRACK_AVOID_AROUND_STATE::AVOID_CHARGER;  
   } 
   return ret;
}

WALLTRACK_AVOID_AROUND_STATE CTaskWallTrackAvoidAround::runChargerAvoid(tPose robotPose)
{
    WALLTRACK_AVOID_AROUND_STATE ret = WALLTRACK_AVOID_AROUND_STATE::AVOID_CHARGER;
    if(taskAvoidCharger.taskRun(robotPose)) ret = WALLTRACK_AVOID_AROUND_STATE::RUN_WALLTRACK_AVOID;

    return ret;
}

tPoint CTaskWallTrackAvoidAround::searchNearstPointFromPath(tPose robotPose)
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
u8 CTaskWallTrackAvoidAround::getLongSignalMatchedByShortSignal(u8 shortSignal)
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
u8 CTaskWallTrackAvoidAround::getAvoidCharegerSignalData(E_WALLTRACK_DIR dir)
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
bool CTaskWallTrackAvoidAround::isStartChargerAvoid(u8 avoidSignal)
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