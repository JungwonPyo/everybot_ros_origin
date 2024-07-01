#include "taskPatternClean.h"
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


/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskPatternClean::CTaskPatternClean()
{
    CStopWatch __debug_sw;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskPatternClean::~CTaskPatternClean()
{
    CStopWatch __debug_sw;

    
    setPatternCleanState(PATTERN_CLEAN_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskPatternClean::setPatternCleanState(PATTERN_CLEAN_STATE set)
{
    if (set != state)
    {
        preState = state; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, CYN, "[PatternCleanState] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskPatternClean::setLineDir(LINE_DIR set)
{
    if (set != lineDir)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[LINE_DIR] : "<< enumToString(lineDir)<<" --> "<< enumToString(set) );
        lineDir = set; // 이전 상태 저장.
    }
}




void CTaskPatternClean::taskStart(PATTERN_CLEAN_STATE startState)
{
    bCleanComplete = false;
    setPatternCleanState(startState);
}


void CTaskPatternClean::findCleanStartPoint(tPose robotPose)
{
    cleanStartPoint.x = 0;
    cleanStartPoint.y = 0;
    MOTION.startLinearToPointOnMap(robotPose, cleanStartPoint, tProfile());
    setPatternCleanState(PATTERN_CLEAN_STATE::GOING_TO_CLEAN_START_POINT);
}

void CTaskPatternClean::goingToCleanStartPoint(tPose robotPose)
{
    if(MOTION.isNearTargetPose(robotPose, targetPoint, 0.15))
    {
        startPatternClean(robotPose);
    }
    else if(avoiding.checkObstacle(robotPose, true, false))
    {
        bakState = PATTERN_CLEAN_STATE::FIND_CLEAN_START_POINT;
        startAvoidWallTrack();
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지하여 벽타기 이동");
    }
    else
    {
                
    }
}


void CTaskPatternClean::startAvoidWallTrack()
{
    walltrackStartTime = SYSTEM_TOOL.getSystemTime();
    setPatternCleanState(PATTERN_CLEAN_STATE::GOING_AVOID_WALLTRACK);
}
void CTaskPatternClean::runAvoidWalltrack(tPose robotPose)
{
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - walltrackStartTime;
    SUB_TASK.walltracking.procWalltracking(robotPose, E_WALLTRACK_DIR::RIGHT);
    
    if(walltrackRunTime >= 3)
    {
        double result = utils::math::calculateDistanceFromLine(tPoint(robotPose.x, robotPose.y), lineCoeff);
        if(fabs(result) <= 0.1 || fabs(result) >= 3.0)
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 회피를 완료 하여이전 상태로 돌립니다. 이전상태 : " <<enumToString(bakState));
            setPatternCleanState(bakState);
        }
    }
}

void CTaskPatternClean::startPatternClean(tPose robotPose)
{
    setLineDir(LINE_DIR::GO_UP);
    maxRange = 3.0;
    startPatternPoint = tPoint(0,0);
    targetPatternPoint = tPoint(maxRange,0);//kinematics.translate(robotPose, maxRange, 0.0);
    NextstartPatternPoint = getNextStartPoint(targetPatternPoint,0.5/*ROS_CONFIG.cleanLineInterval*/);
    NexttargetPatternPoint = getNextTargetPoint(NextstartPatternPoint);
    lineCoeff = utils::math::calculateLineEquation(startPatternPoint, targetPatternPoint);
    lineCoeff2 = utils::math::calculateLineEquation(NextstartPatternPoint, NexttargetPatternPoint);
    MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile()); 
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "패턴 청소 시작!! 라인 : " << enumToString(lineDir) << " 시작 목적지 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
    setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE);
}

void CTaskPatternClean::goToMainLline(tPose robotPose)
{
    if(MOTION.isNearTargetPose(robotPose, targetPatternPoint, 0.1) || MOTION.isOverTargetPoint(robotPose,startPatternPoint,targetPatternPoint))
    {
        lineCoeff = lineCoeff2;
        startPatternPoint = targetPatternPoint;
        targetPatternPoint = getCrossLineTargetPoint(targetPatternPoint,0.5/*ROS_CONFIG.cleanLineInterval*/);
        
        if(targetPatternPoint.y >= 4.5)
        {
            bCleanComplete = true;
            MOTION.startStopOnMap(tProfile(),false);
            setPatternCleanState(PATTERN_CLEAN_STATE::NONE);
        }
        else
        {
            MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile());
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "메인 라인 도착 사이드 라인 시작!! 타켓 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
            setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_SIDE_LINE);
        }
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        startAvoidMainLineWalltrack();
    }
    else
    {
        
    }
}

void CTaskPatternClean::goToSideLine(tPose robotPose)
{
    if(MOTION.isNearTargetPose(robotPose, targetPatternPoint, 0.1) || MOTION.isOverTargetPoint(robotPose,startPatternPoint,targetPatternPoint))
    {
        startPatternPoint = targetPatternPoint;
        targetPatternPoint = getLineTargetPoint(targetPatternPoint);
        NextstartPatternPoint = getNextStartPoint(targetPatternPoint,0.5/*ROS_CONFIG.cleanLineInterval*/);
        NexttargetPatternPoint = getNextTargetPoint(NextstartPatternPoint);
        lineCoeff = utils::math::calculateLineEquation(startPatternPoint, targetPatternPoint);
        lineCoeff2 = utils::math::calculateLineEquation(NextstartPatternPoint, NexttargetPatternPoint);
        MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile());
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "사이드 라인도착!! 메인 라인 전환!! 타켓 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
        setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE);
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        startAvoidSideLineWalltrack();
    }
    else
    {
        
    }
}


void CTaskPatternClean::startAvoidMainLineWalltrack()
{
    if(lineDir == LINE_DIR::GO_UP)          wallDir = E_WALLTRACK_DIR::RIGHT;
    else if(lineDir == LINE_DIR::GO_DOWN)   wallDir = E_WALLTRACK_DIR::LEFT;
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "라인 방향 설정 오류1");
    }
    
    walltrackStartTime = SYSTEM_TOOL.getSystemTime();
    setPatternCleanState(PATTERN_CLEAN_STATE::RUN_MAINLINE_WALLTRCK_AVOID);
}

void CTaskPatternClean::runAvoidMainLineWalltrack(tPose robotPose)
{
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - walltrackStartTime;
    SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);

    double result = utils::math::calculateDistanceFromLine(tPoint(robotPose.x, robotPose.y), lineCoeff);
    double result2 = utils::math::calculateDistanceFromLine(tPoint(robotPose.x, robotPose.y), lineCoeff2);
    if(fabs(result) <= 0.1 && walltrackRunTime >= 10) //|| fabs(result) >= 3.0
    {
        MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile());
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "메인 라인 벽타기 중 원래 목표 메인 라인 이어서 시작!! 타켓 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
        setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE);
    }
    else if(fabs(result2) <= 0.1) //|| fabs(result) >= 3.0
    {
        startPatternPoint = NextstartPatternPoint;
        targetPatternPoint = getLineTargetPoint(startPatternPoint);
        NextstartPatternPoint = getNextStartPoint(targetPatternPoint,0.5/*ROS_CONFIG.cleanLineInterval*/);
        NexttargetPatternPoint = getNextTargetPoint(NextstartPatternPoint);
        lineCoeff = utils::math::calculateLineEquation(startPatternPoint, targetPatternPoint);
        lineCoeff2 = utils::math::calculateLineEquation(NextstartPatternPoint, NexttargetPatternPoint);
        if(targetPatternPoint.y >= 4.5)
        {
            bCleanComplete = true;
            MOTION.startStopOnMap(tProfile(),false);
            setPatternCleanState(PATTERN_CLEAN_STATE::NONE);
        }
        else
        {
            MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile());
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "정면 이동 벽타기 회피 중 다음 목표 도달 메인 라인 타켓 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
            setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE);
        } 
    }
}

void CTaskPatternClean::startAvoidSideLineWalltrack()
{
    if(lineDir == LINE_DIR::GO_UP)          wallDir = E_WALLTRACK_DIR::RIGHT;
    else if(lineDir == LINE_DIR::GO_DOWN)   wallDir = E_WALLTRACK_DIR::LEFT;
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "라인 방향 설정 오류2");
    }
    walltrackStartTime = SYSTEM_TOOL.getSystemTime();
    setPatternCleanState(PATTERN_CLEAN_STATE::RUN_SIDELINE_WALLTRCK_AVOID);
}

void CTaskPatternClean::runAvoidSideLineWalltrack(tPose robotPose)
{
    double walltrackRunTime = SYSTEM_TOOL.getSystemTime() - walltrackStartTime;

    SUB_TASK.walltracking.procWalltracking(robotPose, wallDir);

    double result = utils::math::calculateDistanceFromLine(tPoint(robotPose.x, robotPose.y), lineCoeff);
    if(fabs(result) <= 0.1) //|| fabs(result) >= 3.0
    {
        startPatternPoint = targetPatternPoint;
        targetPatternPoint = getLineTargetPoint(targetPatternPoint);
        NextstartPatternPoint = getNextStartPoint(targetPatternPoint,0.5/*ROS_CONFIG.cleanLineInterval*/);
        NexttargetPatternPoint = getNextTargetPoint(NextstartPatternPoint);
        lineCoeff = utils::math::calculateLineEquation(startPatternPoint, targetPatternPoint);
        lineCoeff2 = utils::math::calculateLineEquation(NextstartPatternPoint, NexttargetPatternPoint);
        if(targetPatternPoint.y >= 4.5)
        {
            bCleanComplete = true;
            MOTION.startStopOnMap(tProfile(),false);
            setPatternCleanState(PATTERN_CLEAN_STATE::NONE);
        }
        else
        {
            MOTION.startLinearToPointOnMap(robotPose,targetPatternPoint,tProfile());
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "측면 이동 벽타기 회피 중 다음 목표 도달 메인 라인 타켓 : " << targetPatternPoint.x << " ," << targetPatternPoint.y);
            setPatternCleanState(PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE);
        }
    }
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
bool CTaskPatternClean::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    double result = 0;
    double result2 = 0;
    double walltrackRunTime = 0;
    CRobotKinematics kinematics;

    switch (state)
    {
    case PATTERN_CLEAN_STATE::NONE :        
        break;
    case PATTERN_CLEAN_STATE::START_PATTERN_CLEAN :
        startPatternClean(robotPose);
        break;       
    case PATTERN_CLEAN_STATE::FIND_CLEAN_START_POINT :        
        findCleanStartPoint(robotPose);
        break;    
    case PATTERN_CLEAN_STATE::GOING_TO_CLEAN_START_POINT :
        goingToCleanStartPoint(robotPose);
        break;
    case PATTERN_CLEAN_STATE::GOING_AVOID_WALLTRACK :
        runAvoidWalltrack(robotPose);
        break;
    case PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE :
        goToMainLline(robotPose);
        break;
    case PATTERN_CLEAN_STATE::GO_TO_SIDE_LINE :
        goToSideLine(robotPose);
        break;
    case PATTERN_CLEAN_STATE::RUN_MAINLINE_WALLTRCK_AVOID :
        runAvoidMainLineWalltrack(robotPose);
        break;
    case PATTERN_CLEAN_STATE::RUN_SIDELINE_WALLTRCK_AVOID :
        runAvoidSideLineWalltrack(robotPose);
        break;    
    default:
        break;
    }

    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

tPoint CTaskPatternClean::getLineTargetPoint(tPoint current)
{
    if(lineDir == LINE_DIR::GO_UP)
    {
        setLineDir(LINE_DIR::GO_DOWN);
        return tPoint(current.x-(maxRange+1),current.y);
    }
    else if(lineDir == LINE_DIR::GO_DOWN)
    {
        setLineDir(LINE_DIR::GO_UP);
        return tPoint(current.x+(maxRange+1),current.y);
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "라인 방향 설정 오류3");
    }
}

tPoint CTaskPatternClean::getCrossLineTargetPoint(tPoint current, double interval)
{
    return tPoint(current.x,current.y+interval);
}

tPoint CTaskPatternClean::getNextStartPoint(tPoint current,double interval)
{
    return tPoint(current.x,current.y+interval);
}

tPoint CTaskPatternClean::getNextTargetPoint(tPoint current)
{
    if(lineDir == LINE_DIR::GO_UP)
    {
        return tPoint(current.x-(maxRange+1),current.y);
    }
    else if(lineDir == LINE_DIR::GO_DOWN)
    {
        return tPoint(current.x+(maxRange+1),current.y);
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "라인 방향 설정 오류4");
    }
}

bool CTaskPatternClean::isCleanComplete()
{
    return bCleanComplete;
}