#include "taskWallClean.h"
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


CTaskWallClean::CTaskWallClean()
{
    CStopWatch __debug_sw;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskWallClean::~CTaskWallClean()
{
    CStopWatch __debug_sw;

    
    setWallCleanState(WALLCLEAN_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskWallClean::setWallCleanState(WALLCLEAN_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state change] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskWallClean::taskStart(tPose robotPose, WALLCLEAN_STATE startState, tPoint _nearWall,std::list<tPoint> areaPath)
{
    bStartWalltrack = false;
    bReturnFromStart = false;
    bCheckAwayFrontStart = false;
    flagCnt = 0;
    nearWall = _nearWall;
    setWallCleanState(startState);
    isInside = utils::area::isInside(tPoint(robotPose.x, robotPose.y), areaPath);
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ============= 원본 Contour 좌표 ============= ");
    for (tPoint point : areaPath)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " [ X, Y : " << point.x << " , " << point.y << " ] ");
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ============================================ ");

    // 청소영역을 10% 확장 (영역이 벽 위에 있는게 가장 좋음)
    utils::area::resizeArea(areaPath, wallCleanAreaPath, CONFIG.shrink_area_scale_factor);
    ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 원본에서 < " << CONFIG.shrink_area_scale_factor << " > 배 확장");
    bool det = utils::math::findClosedLoopDirection(wallCleanAreaPath);
    if(!det)
    {
        wallCleanAreaPath.reverse();
    }
    wallDir = E_WALLTRACK_DIR::RIGHT;
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ============= 가공된 Contour 좌표 ============= ");
    for (tPoint point : wallCleanAreaPath)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " [ X, Y : " << point.x << " , " << point.y << " ] ");
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ========================================================= ");
    DEBUG_PUB.publishShrinkContour(wallCleanAreaPath);

    // 영역 path의 각각의 직선 리스트 세팅
    wallCleanAreaLine = setCleanAreaLine(wallCleanAreaPath);
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ============= 세팅된 라인의 시작점, 끝점 ============= ");
    for (tAreaLine line : wallCleanAreaLine)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " [ Start | End : ( " << line.start.x << " , " << line.start.y << " ) | ( " << line.end.x << " , " << line.end.y << " )");
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ========================================================= ");
    
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
bool CTaskWallClean::taskRun(tPose robotPose, std::list<tPoint> areaPath)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    if(isWallCleanEnd(robotPose)) setWallCleanState(WALLCLEAN_STATE::COMPLETE);

    switch (state)
    {
    case WALLCLEAN_STATE::NONE :
                  
        break;
    case WALLCLEAN_STATE::FIND_NEAR_WALL :
        setWallCleanState(procFindNearWall(robotPose));
        break;
    case WALLCLEAN_STATE::MOVE_TO_NEAR_WALL :
        setWallCleanState(procMoveNearWall(robotPose));
        break;
    case WALLCLEAN_STATE::START_WALLTRACK :
        setWallCleanState(procStartWallTrack(robotPose));
        break;
    case WALLCLEAN_STATE::START_LINETRACK :
        setWallCleanState(procStartLineTrack(robotPose));
        break;
    case WALLCLEAN_STATE::RUN_WALLTRACK :
        setWallCleanState(procRunWallTrack(robotPose));
        break;
    case WALLCLEAN_STATE::RUN_LINETRACK :
        setWallCleanState(procRunLineTrack(robotPose));
        break;
    case WALLCLEAN_STATE::START_COMEBACK_TO_INSIDE :
        setWallCleanState(procStartCombackToInside(robotPose));
        break;
    case WALLCLEAN_STATE::WAIT_COMBACK_PATH :
        setWallCleanState(procWaitComeBackPath(robotPose));
        break;
    case WALLCLEAN_STATE::RUN_COMEBACK_TO_INSIDE :
        setWallCleanState(procRunCombackToInside(robotPose));
        break;
    case WALLCLEAN_STATE::RUN_AVOID_CHARGER :
        setWallCleanState(procAvoidCharger(robotPose));
        break;
    case WALLCLEAN_STATE::COMPLETE:
        setWallCleanState(procCompleteWallTrack(robotPose));
        break;              
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 가까운 벽 찾기, 경로계획 시작, 벽타기 방향 결정
 * 
 * @param robotPose 
 * @param area 
 * @return WALLCLEAN_STATE 
 */
WALLCLEAN_STATE CTaskWallClean::procFindNearWall(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::FIND_NEAR_WALL;

    if (isInside)
    {
        // 가까운 직선을 찾고 거기로 보낸다.
        nearLine = findNearWallLine(robotPose, wallCleanAreaLine);
        nearTargetPoint = findNearWallPoint(robotPose, nearLine);
        MOTION.startLinearAngularPriorToPointOnMap(robotPose, nearTargetPoint, tProfile());
        moveToNearWallTime = SYSTEM_TOOL.getSystemTime();
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 가까운 라인 위의 최소거리 목표점을 찾아 이동을 시작 합니다. 목표 좌표 : [ " << nearTargetPoint.x << " ," << nearTargetPoint.y << " ]");
        ret = WALLCLEAN_STATE::MOVE_TO_NEAR_WALL;
    }
    else
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," FIND_NEAR_WALL 에서 영역 밖이라 일단 영역 안으로 넣어볼게요 -> START_COMEBACK_TO_INSIDE");
        ret = WALLCLEAN_STATE::START_COMEBACK_TO_INSIDE;
    }
    
    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procMoveNearWall(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::MOVE_TO_NEAR_WALL;

    bool bCheckObs = avoiding.checkObstacle(robotPose, true, false);

    bool isRobotNearLine = checkRobotNearLine(robotPose, nearLine, 0.05);

    if (bCheckObs)
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "이동 중 장애물을 만나 벽면주행을 시작 합니다.");
        MOTION.startStopOnMap(tProfile(),false);
        wallTrackStartPoint = tPoint(robotPose.x,robotPose.y);
        ret = WALLCLEAN_STATE::START_WALLTRACK;
    }
    else
    {
        if (isRobotNearLine || robotPose.distance(nearTargetPoint) <= 0.05)
        {
            MOTION.startStopOnMap(tProfile(),false);
            wallTrackStartPoint = tPoint(robotPose.x,robotPose.y);
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, "이동 중 라인 근처에 도착했습니다 영역 벽면주행을 시작 합니다.");
            ret = WALLCLEAN_STATE::START_LINETRACK;
        }
        else
        {
            if (SYSTEM_TOOL.getSystemTime() - moveToNearWallTime >= 15)
            {
                ceblog(LOG_LV_NECESSARY, RED, "가까운 점으로 이동하는데 장애물도 없고, 목표점 도달도 못했네요. 15초가 지나 START_LINETRACK 으로 보낼게요.");
                ret = WALLCLEAN_STATE::START_LINETRACK;
            }
        }
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procStartWallTrack(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::START_WALLTRACK;
    
    if(!MOTION.isRunning())
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "벽타기 시작 포인트 (x, y) : ( " << wallTrackStartPoint.x << " , " << wallTrackStartPoint.y << " )");
        SUB_TASK.walltracking.startAccumulateAngle(robotPose);
        wallCleanStartTime = SYSTEM_TOOL.getSystemTime();
        bStartWalltrack = true;
        ret = WALLCLEAN_STATE::RUN_WALLTRACK;
    }
    
    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procStartLineTrack(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::START_LINETRACK;
    
    if(!MOTION.isRunning())
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "영역 벽타기 시작 포인트 (x, y) : ( " << wallTrackStartPoint.x << " , " << wallTrackStartPoint.y << " )");
        SUB_TASK.walltracking.startAccumulateAngle(robotPose);
        wallCleanStartTime = SYSTEM_TOOL.getSystemTime();
        bStartWalltrack = true;

        nearLine = findNearWallLine(robotPose, wallCleanAreaLine);
        bTargetUpdate = true;
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "영역 벽타기 타겟 포인트 업데이트 (x, y) : ( " << nearLine.end.x << " , " << nearLine.end.y << " )");
        ret = WALLCLEAN_STATE::RUN_LINETRACK;
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procRunWallTrack(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::RUN_WALLTRACK;
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - wallCleanStartTime;

    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    RSF_OBSTACLE_MASK lidarSide = pObstacle->right_wall.obstacle;
    short TofSide = (wallDir == E_WALLTRACK_DIR::RIGHT) ?  pObstacle->tof.rightwall.rangeAvg : pObstacle->tof.leftwall.rangeAvg;
    double targetRad = utils::math::getTurnRadAngle(targetLine.second,robotPose);
    bool isTargetLeft = (targetRad > 0),  isWallonSide = (TofSide <= 100);

    if(!utils::area::isInside(tPoint(robotPose.x, robotPose.y), wallCleanAreaPath))
    {
        if (!checkRobotNearLine(robotPose, nearLine, 0.15))
        {
            MOTION.startStopOnMap(tProfile(),false);
            ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 영역에서 너무 멀리 벗어남!!");
            ret = WALLCLEAN_STATE::START_COMEBACK_TO_INSIDE;
        }
        else
        {
            if(walltrackRunTime >= 5)
            {
                MOTION.startStopOnMap(tProfile(),false);
                ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 영역 밖!  영역 벽타기 시작!!");
                ret = WALLCLEAN_STATE::START_LINETRACK;
            }
            else
            {
                SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);
            }
        }
    }
    else
    {
        SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);
    }

    if(isStartChargerAvoid(getAvoidCharegerSignalData(wallDir))
        && SUB_TASK.cleanPlan.isSetupNogoZoneDocking() == false) //도킹존이 있으면 영역 벽타기로 회피한다.
    {
        MOTION.startStopOnMap(tProfile(),true);
        taskAvoidCharger.taskStart(wallDir);
        ret = WALLCLEAN_STATE::RUN_AVOID_CHARGER;    
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procRunLineTrack(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::RUN_LINETRACK;
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - wallCleanStartTime;

    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    RSF_OBSTACLE_MASK lidarSide = pObstacle->right_wall.obstacle;
    short TofSide = (wallDir == E_WALLTRACK_DIR::RIGHT) ?  pObstacle->tof.rightwall.rangeAvg : pObstacle->tof.leftwall.rangeAvg;
    double targetRad = utils::math::getTurnRadAngle(targetLine.second,robotPose);
    bool isTargetLeft = (targetRad > 0),  isWallonSide = (TofSide <= 100);
    bool bCheckObs = avoiding.checkObstacle(robotPose, true, false);

    if (bCheckObs)
    {
        MOTION.startStopOnMap(tProfile(),false);
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 장애물 감지 벽타기 시작!!");
        ret = WALLCLEAN_STATE::START_WALLTRACK;
    }
    else
    {
        if (robotPose.distance(nearLine.end) <= 0.1)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, "Line Tracking 중 현재 목표점 도달 완료! ( "
            << nearLine.end.x << " , " << nearLine.end.y
            << "), 현재 Line Flag = false, 다음 목표점 서칭을 위해 START_LINETRACK 으로 다시 이동!");
            setNearWallLineFlag(nearLine);
            MOTION.startStopOnMap(tProfile(),false);
            ret = WALLCLEAN_STATE::START_LINETRACK;
        }
        SUB_TASK.walltracking.procAreaWalltracking(robotPose, wallDir, nearLine.start, nearLine.end, bTargetUpdate);
    }

    if(isStartChargerAvoid(getAvoidCharegerSignalData(wallDir))
        && SUB_TASK.cleanPlan.isSetupNogoZoneDocking() == false) //도킹존이 있으면 영역 벽타기로 회피한다.
    {
        MOTION.startStopOnMap(tProfile(),true);
        taskAvoidCharger.taskStart(wallDir);
        ret = WALLCLEAN_STATE::RUN_AVOID_CHARGER;    
    }
    bTargetUpdate = false;
    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procStartCombackToInside(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::START_COMEBACK_TO_INSIDE;
    if (!utils::area::isInside(tPoint(robotPose.x, robotPose.y), wallCleanAreaPath))
    {
        tPoint insideTargetPoint = utils::area::findCentroid(wallCleanAreaPath);
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 영역 중암점으로 이동을 시작 합니다. 목표 좌표 : [ " << insideTargetPoint.x << " ," << insideTargetPoint.y << " ]");
        taskPathPlan.taskStart(insideTargetPoint);
        //MOTION.startLinearAngularPriorToPointOnMap(robotPose, insideTargetPoint, tProfile());
        //ret = WALLCLEAN_STATE::RUN_COMEBACK_TO_INSIDE;
        ret = WALLCLEAN_STATE::WAIT_COMBACK_PATH;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, RED, "뭔가 판단이 이상함, 영역 내부라고 뜨네!!! 알고리즘 확인하시길..., 일단 START_LINETRACK 으로 보냄!");
        ret = WALLCLEAN_STATE::START_LINETRACK;
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procWaitComeBackPath(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::WAIT_COMBACK_PATH;
    std::list<tPoint> path = taskPathPlan.taskRun(robotPose);
    tProfile profile = tProfile();
    if(!path.empty())
    {
        taskMovePath.taskStart(path,0.10,profile);
        ret = WALLCLEAN_STATE::RUN_COMEBACK_TO_INSIDE;
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procRunCombackToInside(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::RUN_COMEBACK_TO_INSIDE;
    bool arrived = taskMovePath.taskRun(robotPose);
    
    if (utils::area::isInside(tPoint(robotPose.x, robotPose.y), wallCleanAreaPath) == true && checkRobotNearLine(robotPose, nearLine, 0.10) == false)
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 영역 안쪽 안전지대로 들어와 다시 벽타기(모션 제어)를 재개합니다. ");
        MOTION.startStopOnMap( tProfile(), false);
        ret = WALLCLEAN_STATE::START_LINETRACK;
    }
    else if(arrived)
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN,"도착했는데 왜 영역안으로 안가졌음?? 오류!!!!");
    }

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procCompleteWallTrack(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::COMPLETE;
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - wallCleanStartTime;
    MOTION.startStopOnMap(tProfile(),false);
    bStartWalltrack = false;
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," WallClean 종료 합니다. 청소 시간 : " << walltrackRunTime );
    ret = WALLCLEAN_STATE::NONE;

    return ret;
}

WALLCLEAN_STATE CTaskWallClean::procAvoidCharger(tPose robotPose)
{
    WALLCLEAN_STATE ret = WALLCLEAN_STATE::RUN_AVOID_CHARGER;
    
    if(taskAvoidCharger.taskRun(robotPose)) ret = WALLCLEAN_STATE::RUN_WALLTRACK;

    return ret;
}

bool CTaskWallClean::isReturnStartPoint()
{
    return bReturnFromStart;
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
u8 CTaskWallClean::getLongSignalMatchedByShortSignal(u8 shortSignal)
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
u8 CTaskWallClean::getAvoidCharegerSignalData(E_WALLTRACK_DIR dir)
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
bool CTaskWallClean::isStartChargerAvoid(u8 avoidSignal)
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

bool CTaskWallClean::isWallCleanEnd(tPose robotPose)
{
    bool ret = false;
   
    if(bStartWalltrack)
    {
        double dist = robotPose.distance(wallTrackStartPoint);
        if (debugCnt >= 1000)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, "벽타기 시작 포인트 (x, y) : ( " << wallTrackStartPoint.x << " , " << wallTrackStartPoint.y << " ) | 현재 떨어진 거리(m) : " << dist);
            debugCnt = 0;
        }
        debugCnt += 1;

        if(!bCheckAwayFrontStart)
        {
            if(dist >= 1)
            {
                bCheckAwayFrontStart = true;
                ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 벽타기 시작 위치에서 1m 멀어졌습니다. 원점 복귀 확인을 시작합니다!");
            } 
        }
        else
        {
            if(dist <= 0.3)    // 이것도 taskMovePath 마진이 0.2라 거기에 맞춤, 나중에 유연하게 바꿀 수 있게 수정해야 함
            {
                bReturnFromStart = true;
                ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN," 로봇이 벽타기로 한바퀴 돌았습니다. 청소 위치로 보냅니다");
                MOTION.startStopOnMap(tProfile(),true);
                
                ret = true;
            }
        }
    }
    
    return ret;
}

/**
 * @brief 영역 포인트를 영역 선분으로 세팅하는 함수 (flag 기본설정: true)
 * 
 * @param areaPoints 
 * @return std::list<tAreaLine> 
 */
std::vector<tAreaLine> CTaskWallClean::setCleanAreaLine(std::list<tPoint> areaPoints)
{
    std::vector<tAreaLine> ret;
    std::vector<tPoint> tempPoints;
    for (tPoint point : areaPoints)
    {
        tempPoints.push_back(point);
    }
    tempPoints.emplace_back(areaPoints.front());

    if (areaPoints.size() < 2)
    {
        ceblog(LOG_LV_NECESSARY, RED, "영역 개수 2개 미만 오류!!");
    }
    else
    {
        while (tempPoints.size() >= 2)
        {
            tPoint point1 = tempPoints[0];
            tPoint point2 = tempPoints[1];
            std::vector<double> coeff = utils::math::calculateLineEquationVector(point1, point2);
            tAreaLine result;
            result.start = point1;
            result.end = point2;
            result.A = coeff[0];
            result.B = coeff[1];
            result.C = coeff[2];
            result.flag = true;

            tempPoints.erase(tempPoints.begin());
            ret.push_back(result);
        }
    }
    return ret;
}

/**
 * @brief 로봇 위치 기준 가장 가까운 영역 라인을 찾는 함수
 * (line.flag = true 인 선분만 서칭)
 * 
 * @param robotPose 
 * @param areaLines 
 * @return tPoint 
 */
tAreaLine CTaskWallClean::findNearWallLine(tPose robotPose, std::vector<tAreaLine> areaLines)
{
    tAreaLine ret = areaLines.front();

    double minDist = std::numeric_limits<double>::max();

    for (const auto& line : areaLines)
    {
        if (line.flag)
        {
            double distance;
            tPoint vecV1 = tPoint(line.end.x - line.start.x, line.end.y - line.start.y);
            tPoint vecV2 = tPoint(line.start.x - line.end.x, line.start.y - line.end.y);
            tPoint vecW1 = tPoint(robotPose.x - line.start.x, robotPose.y - line.start.y);
            tPoint vecW2 = tPoint(robotPose.x - line.end.x, robotPose.y - line.end.y);

            double dotProdV1W1 = vecV1.x*vecW1.x + vecV1.y*vecW1.y;
            double dotProdV2W2 = vecV2.x*vecW2.x + vecV2.y*vecW2.y;
            
            if (dotProdV1W1 < 0 || dotProdV2W2 < 0) //선분 밖이면
            {
                distance = std::min(robotPose.distance(line.start), robotPose.distance(line.end));
            }
            else                                    //선분 안이면
            {
                std::list<double> coeff;
                coeff.push_back(line.A);
                coeff.push_back(line.B);
                coeff.push_back(line.C);
                distance = utils::math::calculateDistanceFromLine(robotPose.convertPoint(),coeff);
            }

            if (distance < minDist)
            {
                minDist = distance;
                ret = line;
            }
        }
    }

    return ret;
}

/**
 * @brief 라인의 flag를 세팅해주는 함수 (영역 라인 중 입력받은 라인의 flag를 false로 세팅)
 * 
 * @param line 
 */
void CTaskWallClean::setNearWallLineFlag(tAreaLine line)
{
    if (flagCnt == 0)
    {
        firstLine = nearLine;
        for (tAreaLine& wallLine : wallCleanAreaLine)
        {
            if (nearLine.start == wallLine.start && nearLine.end == wallLine.end)
            {
                wallLine.flag = false;
                break;
            }
        }
        ceblog(LOG_LV_NECESSARY, BLUE, " firstLine에 최초 라인 저장 start point : ( " << firstLine.start.x << " , " << firstLine.start.y << " )");
    } //firstLine은 이 함수가 처음 호출될 때 한번만 저장되게 하고 싶음

    if (wallCleanAreaLine.size() > 1)
    {
        for (tAreaLine& wallLine : wallCleanAreaLine)
        {
            if (line.start == wallLine.start && line.end == wallLine.end)
            {
                wallLine.flag = false;
                break;
            }
        }
    }
    else
    {
        nearLine = firstLine;
        nearLine.flag = true;
    }

    ++flagCnt;
}

/**
 * @brief 현재 로봇위치 기준 주어진 직선상의 가장 가까운 점을 반환
 * 
 * @param robotPose 
 * @param nearLine 
 * @return tPoint 
 */
tPoint CTaskWallClean::findNearWallPoint(tPose robotPose, tAreaLine nearLine)
{
    tPoint ret;

    double A = nearLine.A;
    double B = nearLine.B;
    double C = nearLine.C;
    double det = A*A + B*B;

    ret.x = (1/det) * (B*B*robotPose.x - A*B*robotPose.y - A*C);
    ret.y = (1/det) * (A*A*robotPose.y - A*B*robotPose.x - B*C);

    return ret;
}

/**
 * @brief 로봇이 선분 근처에 있는지 판단하는 함수 (true: 근처다, false: 아니다)
 * 
 * @return true 
 * @return false 
 */
bool CTaskWallClean::checkRobotNearLine(tPose robotPose, tAreaLine line, double margin)
{
    bool ret;
    std::list<double> coeff;
    coeff.emplace_back(line.A);
    coeff.emplace_back(line.B);
    coeff.emplace_back(line.C);
    double dist = utils::math::calculateDistanceFromLine(robotPose.convertPoint(), coeff);

    if (dist <= margin && line.start.x <= robotPose.x && robotPose.x <= line.end.x && line.start.y <= robotPose.y && robotPose.y <= line.end.y) 
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "로봇이 라인 근처에 있네요!! 로봇위치 : ( " << robotPose.x << " , " << robotPose.y
        << " ) | 라인 시작점 : ( " << line.start.x << " , " << line.start.y << " ) | 라인 끝점 : ( " << line.end.x << " , " << line.end.y
        << " ) | 라인과의 거리 : [ " << dist << " ] | 마진 : [ " << margin << " ]");
        ret = true;
    }
    else
    {
        ret = false;
    }

    return ret;
}
