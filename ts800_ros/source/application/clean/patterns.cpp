#include "patterns.h"
#include "eblog.h"
#include "robotmap.h"
#include "obstaclemap.h"
#include "utils.h"
#include <memory>
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

#define LINE_INVERVAL 0.3
#define ROBOT_RADIUS 0.15
#define CHECK_HEADING_ERROR_DEGREE 5 //라인청소 중 직진 주행 시 각도 오차 기준 (해당 각도기준을 초과하면 보정 제어를 실행하는 기준 값)

CCleanPatterns::CCleanPatterns()
{
    pObsMap = &(ServiceData.obstaclemap); 
    pLinePattern = new CPatternLine();
    pLineTrack   = new CLineTrack();
}

CCleanPatterns::~CCleanPatterns()
{
}

void CCleanPatterns::setPatternState(E_PATTERN_STATE set)
{
    CStopWatch __debug_sw;

    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_PATTERN_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    state = set;
}

E_PATTERN_STATE CCleanPatterns::getPatternState()
{
    return state;
}

E_PATTERN_STATE CCleanPatterns::initCleanPattern(E_CLEAN_PATTERN pattern, tPose robotPose, const std::list<tPoint>& areaPoligons)
{
    E_PATTERN_STATE ret = E_PATTERN_STATE::INIT;
    switch (pattern)
    {
    case E_CLEAN_PATTERN::LINE:
        ret = pLinePattern->initLinePatternStep(robotPose, areaPoligons);
        break;
    case E_CLEAN_PATTERN::RANDOM:
        ret = E_PATTERN_STATE::RUN;
        break;
    case E_CLEAN_PATTERN::WALL:
        //pLineTrack->initLineTrack(pLinePattern->shrinkRectangle(ServiceData.robotMap.getPolygons(), 0.15,0.15,0.15,0.15));
        SUB_TASK.walltracking.initWallTrack(false);
        ret = E_PATTERN_STATE::RUN;
        break;
    case E_CLEAN_PATTERN::CIRCLE:

        break;
    case E_CLEAN_PATTERN::SPOT:

        break;                
    default:
        break;
    }

    return ret;
}

E_PATTERN_STATE CCleanPatterns::runCleanPattern(E_CLEAN_PATTERN pattern, 
    tPose robotPose, cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle)
{
    E_PATTERN_STATE ret = E_PATTERN_STATE::RUN;

    switch (pattern)
    {
    case E_CLEAN_PATTERN::LINE:
        ret = pLinePattern->runLinePatternStep(robotPose,pObsMap,pObstacle);
        break;
    case E_CLEAN_PATTERN::RANDOM:
        pLinePattern->runRandomCleanPattern(robotPose,pObstacle);
        break;
    case E_CLEAN_PATTERN::WALL:
        // if(pLineTrack->runLineTrack())
        // {
        //     ret=E_PATTERN_STATE::COMPLETE;
        // }
        SUB_TASK.walltracking.runWallTrackPattern();
        if(SUB_TASK.walltracking.isReturnStartPoint())
        {
            
            ret = E_PATTERN_STATE::COMPLETE;
        }
        break;
    case E_CLEAN_PATTERN::CIRCLE:
        break;
    case E_CLEAN_PATTERN::SPOT:
        break;                
    default:
        break;
    }

    return ret;
}
bool CCleanPatterns::completeCleanPattern(E_CLEAN_PATTERN pattern,tPose robotPose)
{
    bool ret = false;
    switch (pattern)
    {
    case E_CLEAN_PATTERN::LINE:
        ret = pLinePattern->completeLinePatternStep(robotPose);
        break;
    case E_CLEAN_PATTERN::RANDOM:
        break;
    case E_CLEAN_PATTERN::WALL:
        ret = pLinePattern->completeLinePatternStep(robotPose);
        break;
    case E_CLEAN_PATTERN::CIRCLE:
        break;
    case E_CLEAN_PATTERN::SPOT:
        break;                
    default:
        break;
    }

    return ret;
}

bool CCleanPatterns::controlHandler(E_CLEAN_PATTERN pattern,tPose robotPose,
    const std::list<tPoint>& areaPoligons, cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle)
{
    bool ret = false;
    switch (getPatternState())
    {
    case E_PATTERN_STATE::INIT:
        setPatternState(initCleanPattern(pattern, robotPose, areaPoligons)); 
        break;
    case E_PATTERN_STATE::RUN:
        setPatternState(runCleanPattern(pattern, robotPose, pObsMap, pObstacle));
        break;
    case E_PATTERN_STATE::COMPLETE:
        ret = completeCleanPattern(pattern,robotPose);
        if(ret) setPatternState(E_PATTERN_STATE::INIT);
        break;        
    
    default:
        break;
    }

    return ret;
}


CPatternLine::CPatternLine() : CAvoiding()
{
    pObsMap = &(ServiceData.obstaclemap);
}
CPatternLine::~CPatternLine()
{

}

void CPatternLine::setLinePatternStep(E_PATTERN_LINE_STEP set)
{
    CStopWatch __debug_sw;

    if (set != patternLineData_.step)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "[E_PATTERN_LINE_STEP] setLinePatternStep : "
            <<WHITE<<enumToString(patternLineData_.step)<<BOLDBLACK<<" --> "<<BOLDCYAN<<enumToString(set));
    }
    
    patternLineData_.step = set;
    TIME_CHECK_END(__debug_sw.getTime());
}

E_PATTERN_LINE_STEP CPatternLine::getLinePatternStep()
{
    return patternLineData_.step;
}

void CPatternLine::setCurLine(E_PURPOSE_LINE_CLEAN_INFO set){
    CStopWatch __debug_sw;

    if (set != patternLineData_.curLine)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_PURPOSE_LINE_CLEAN_INFO] setCurLine : "<<enumToString(patternLineData_.curLine)<<" --> "<<enumToString(set));
    }
    patternLineData_.curLine = set;
    TIME_CHECK_END(__debug_sw.getTime());
}
E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getCurLine(){
    //ceblog(LOG_LV_NECESSARY, CYN, "[E_PURPOSE_LINE_CLEAN_INFO] getCurLine : "<<enumToString(patternLineData_.curLine));
    return patternLineData_.curLine;
}

void CPatternLine::setLineTemp(E_PURPOSE_LINE_CLEAN_INFO set){
    CStopWatch __debug_sw;

    if (set != patternLineData_.tempLine)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_PURPOSE_LINE_CLEAN_INFO] setLineTemp : "<< enumToString(patternLineData_.tempLine)<<" --> "<<enumToString(set));
    }
    patternLineData_.tempLine = set;
    TIME_CHECK_END(__debug_sw.getTime());
}
E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getLineTemp(){
    return patternLineData_.tempLine;
}

void CPatternLine::setShortLine(E_PURPOSE_LINE_CLEAN_INFO set){
    CStopWatch __debug_sw;

    if (set != patternLineData_.shortLine)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_PURPOSE_LINE_CLEAN_INFO] setshortLine : "<<enumToString(patternLineData_.shortLine)<<" --> "<<enumToString(set));
    }
    patternLineData_.shortLine = set;
    TIME_CHECK_END(__debug_sw.getTime());
}
E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getShortLine(){
    return patternLineData_.shortLine;
}

void CPatternLine::setNewLine(E_PURPOSE_LINE_CLEAN_INFO set){
    CStopWatch __debug_sw;

    if (set != patternLineData_.newLine)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_PURPOSE_LINE_CLEAN_INFO] setNewLine : "<<enumToString(patternLineData_.newLine)<<" --> "<<enumToString(set));
    }
    patternLineData_.newLine = set;
    TIME_CHECK_END(__debug_sw.getTime());
}

E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getNewLine(){
    return patternLineData_.newLine;
}


void CPatternLine::setAvoidStep(E_AVOID_LINE_STEP set)
{
    if (set != patternLineData_.avoidStep)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_AVOID_LINE_STEP] avoidStep : "<<enumToString(patternLineData_.avoidStep)<<" --> "<<enumToString(set));
    }
    patternLineData_.avoidStep = set;
}
E_AVOID_LINE_STEP CPatternLine::getAvoidStep()
{
    return patternLineData_.avoidStep;
}

void CPatternLine::setDoLineStep(E_DO_LINE_STEP set)
{
    if (set != patternLineData_.doLineStep)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[E_DO_LINE_STEP] DoLineStep : "<<enumToString(patternLineData_.doLineStep)<<" --> "<<enumToString(set));
    }
    patternLineData_.doLineStep = set;
}
E_DO_LINE_STEP CPatternLine::getDoLineStep()
{
    return patternLineData_.doLineStep;
}

// tPose CPatternLine::getCurrentTargetPose()
// {
//     tPose ret;
//     if(patternLineData_.plan.empty())
//     {
//         ret = patternLineData_.lineTargetPose;
//         ceblog(LOG_LV_NECESSARY, CYN, " plan is Empty");
//     }
//     else
//     {
//         ret = patternLineData_.plan.front();
//     }

//     ceblog(LOG_LV_NECESSARY, CYN, " currrent target Pose : " << ret.x << " , "<< ret.y);

//     return ret;
// }

void CPatternLine::makeCliffEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{   
    RSF_OBSTACLE_MASK mask = getAvoidMask();
    wayPoint.clearAction();
    tAction action = tAction();
    action.profile.desAngVel = DEG2RAD(40);
    action.profile.desLinVel = 0.2;
    // action 1

     if (mask.b.fleft_side && mask.b.fright_side)
    {
        //좌회전 90 도 만큼         
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        action.rotate.targetAngle = DEG2RAD(90.0);        
    }
    else
    {
        if (mask.b.fleft_side){
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

    ceblog(LOG_LV_NECESSARY, BLUE, "낙하 회피 waypoint 생성 완료." );
}


/**
 * @brief  라인 청소 중 범퍼 회피 제어를 담당하는 함수 (초기 회피 이후)
 * @param FIRST     : 낙하 지점을 벋어나기 위해 다른 방향으로 회전한다. 초기 회피 시 한쪽 방향만 후진한 경우 회전 반경을 만들기 위해 다른 쪽 방향도 후진한다.
 * @param SECOND    : 초기 회피 시점에서 한쪽 방향만 후진한 경우에만 해당한다. 낙하 지점을 벋어나기 위해 다른 방향으로 회전한다
 * @param THIRD     : 회전 제어에 대한 대기상태를 확인한다. (낙하 지역을 모니터할 수 없기때문에 일정 각도 회전까지 대기한다)
 * @param END       : 회전이 끝나면, 장애물 회피를 종료한다. 라인청소 중 낙하감지가 되면 해당 라인의 종료지점이기 때문에 라인을 업데이트한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
void CPatternLine::makeBumperEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    RSF_OBSTACLE_MASK mask = getAvoidMask();

    wayPoint.clearAction();
    tAction action = tAction();
    action.profile.desAngVel = DEG2RAD(40);
    action.profile.desLinVel = 0.2;
    int avoidCount = (int)getAvoidPoseHistory().size();
    bool complexSpace = false;
    
    if(avoidCount >= 5)
    {
        if(utils::math::distanceTwoPoint(getAvoidStartPose(),robotPose) <= 0.1*avoidCount)
        {
            complexSpace = true;
        }
    }

    patternLineData_.isAvoidTurnLeft = isTurnLeftByLine(getCurLine(),getLineTemp(),getShortLine());
                                    
    if(!patternLineData_.isAvoidTurnLeft)
    {   
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0x0F)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-45.0);
            else                                action.rotate.targetAngle = DEG2RAD(-90.0);
        }   
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-22.5);
            else                                action.rotate.targetAngle = DEG2RAD(-45.0);
        }                    

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);

        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.30;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.3;
        action.curve.targetAngle = DEG2RAD(180);

        wayPoint.pushAction(action);
    }
    else
    {
        #if USE_WALLTRACKAVOIDING > 0
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0xF0)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }        

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        #else
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0xF0)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }                    

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        
        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);
        
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.3;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.15;
        action.curve.targetAngle = DEG2RAD(-180);
        wayPoint.pushAction(action);
        #endif
    }

    ceblog(LOG_LV_NECESSARY, BLUE, "범퍼 회피 waypoint 생성 완료. - patterns" );
    return;
}

/**
 * @brief  라인 청소 중 전방 회피(라이다 센서 감지 불가 높이) 제어를 담당하는 함수 (초기 회피 이후)
 * @param FIRST     : 낙하 지점을 벋어나기 위해 다른 방향으로 회전한다. 초기 회피 시 한쪽 방향만 후진한 경우 회전 반경을 만들기 위해 다른 쪽 방향도 후진한다.
 * @param SECOND    : 초기 회피 시점에서 한쪽 방향만 후진한 경우에만 해당한다. 낙하 지점을 벋어나기 위해 다른 방향으로 회전한다
 * @param THIRD     : 회전 제어에 대한 대기상태를 확인한다. (낙하 지역을 모니터할 수 없기때문에 일정 각도 회전까지 대기한다)
 * @param END       : 회전이 끝나면, 장애물 회피를 종료한다. 라인청소 중 낙하감지가 되면 해당 라인의 종료지점이기 때문에 라인을 업데이트한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
void CPatternLine::makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    CStopWatch __debug_sw;
    
    RSF_OBSTACLE_MASK mask = getAvoidMask();

    wayPoint.clearAction();
    tAction action = tAction();
    action.profile.desAngVel = DEG2RAD(40);
    action.profile.desLinVel = 0.2;
    int avoidCount = (int)getAvoidPoseHistory().size();
    bool complexSpace = false;
    if(avoidCount >= 5)
    {
        if(utils::math::distanceTwoPoint(getAvoidStartPose(),robotPose) <= 0.1*avoidCount)
        {
            complexSpace = true;
        }
    }                                      

    patternLineData_.isAvoidTurnLeft = isTurnLeftByLine(getCurLine(),getLineTemp(),getShortLine());
    if(!patternLineData_.isAvoidTurnLeft)
    {   
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0x0F)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-45.0);
            else                                action.rotate.targetAngle = DEG2RAD(-90.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-22.5);
            else                                action.rotate.targetAngle = DEG2RAD(-45.0);
        }                    

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);

        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.3;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.3;
        action.curve.targetAngle = DEG2RAD(180);
        wayPoint.pushAction(action);
    }
    else
    {
        #if USE_WALLTRACKAVOIDING > 0
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0xF0)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }     

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        #else
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0xF0)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }                    

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        
        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);
        
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.3;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.15;
        action.curve.targetAngle = DEG2RAD(-180);
        wayPoint.pushAction(action);
        #endif
    }
    
    ceblog(LOG_LV_NECESSARY, BLUE, "전방IR 회피 waypoint 생성 완료. - patterns" );
    TIME_CHECK_END(__debug_sw.getTime()); 
    return;   
}

void CPatternLine::makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint)
{
    CStopWatch __debug_sw;
    RSF_OBSTACLE_MASK mask = getAvoidMask();

    wayPoint.clearAction();
    tAction action = tAction();   
    action.profile.desAngVel = DEG2RAD(40);
    action.profile.desLinVel = 0.2;                              
    int avoidCount = (int)getAvoidPoseHistory().size();
    bool complexSpace = false;
    if(avoidCount >= 5)
    {
        if(utils::math::distanceTwoPoint(getAvoidStartPose(),robotPose) <= 0.1*avoidCount)
        {
            complexSpace = true;
        }
    }

    patternLineData_.isAvoidTurnLeft = isTurnLeftByLine(getCurLine(),getLineTemp(),getShortLine());
    if(!patternLineData_.isAvoidTurnLeft)
    {   
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0xF0)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-22.5);
            else                                action.rotate.targetAngle = DEG2RAD(-45.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(-45.0);
            else                                action.rotate.targetAngle = DEG2RAD(-90.0); 
            
        }                    
        action.needObstacleCheck = false;
        wayPoint.pushAction(action);

        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);
        
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.3;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.3;
        action.curve.targetAngle = DEG2RAD(180);
        wayPoint.pushAction(action);
    }
    else
    {
        #if USE_WALLTRACKAVOIDING > 0
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0x0F)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0); 
        }

        action.needObstacleCheck = false;
        wayPoint.pushAction(action);
        #else
        action.type = E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT;
        if(mask.value & 0x0F)
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(22.5);
            else                                action.rotate.targetAngle = DEG2RAD(45.0);
        }
        else
        {
            if(complexSpace || avoidCount < 2)  action.rotate.targetAngle = DEG2RAD(45.0);
            else                                action.rotate.targetAngle = DEG2RAD(90.0); 
        }                    

        action.needObstacleCheck = true;
        wayPoint.pushAction(action);

        // stop
        action.type = E_ACTION_TYPE::STOP;
        action.stop.waitForMs = 50;
        wayPoint.pushAction(action);
        
        action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
        action.linear.targetDistance = 0.3;
        action.needObstacleCheck = true;
        action.profile.isStopAtTarget = false;
        wayPoint.pushAction(action);

        action.type = E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT;
        action.curve.radius = 0.15;
        action.curve.targetAngle = DEG2RAD(-180);
        wayPoint.pushAction(action);
        #endif
    }

    ceblog(LOG_LV_NECESSARY, BLUE, "전방 LIDAR 회피 waypoint 생성 완료. - patterns" );
    TIME_CHECK_END(__debug_sw.getTime()); 
    return; 
}


E_PURPOSE_LINE_CLEAN_INFO checkShortLine(E_PURPOSE_LINE_CLEAN_INFO curLine,tPose start, tPose last)
{
    E_PURPOSE_LINE_CLEAN_INFO ret = E_PURPOSE_LINE_CLEAN_INFO::NONE;
    
    if(curLine == E_PURPOSE_LINE_CLEAN_INFO::UP || curLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
    {
        if(start.y > last.y) ret = E_PURPOSE_LINE_CLEAN_INFO::RIGHT;
        else                 ret = E_PURPOSE_LINE_CLEAN_INFO::LEFT;
    }
    else
    {
        if(start.x > last.x) ret = E_PURPOSE_LINE_CLEAN_INFO::DOWN;
        else                 ret = E_PURPOSE_LINE_CLEAN_INFO::UP;
    }

    return ret;
}
/**
 * @brief 라인 청소 데이터 초기화 함수
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
E_PATTERN_STATE CPatternLine::initLinePatternStep (tPose robotPose, const std::list<tPoint>& areaPoligons)
{  
    CStopWatch __debug_sw;
    E_PATTERN_STATE ret = E_PATTERN_STATE::INIT;
    if(!MOTION.isRunning())
    {
        initAvoiding();
        patternLineData_.tempstartPose = tPose(0,0,0);
        patternLineData_.tempTargetPoint = tPoint(0,0);
        patternLineData_.beforestartPose = tPose(0,0,0);
        patternLineData_.beforeTargetPoint = tPoint(0,0);

        patternLineData_.avoidiPath.clear();
        patternLineData_.checkAvoidingEnd = false;
        patternLineData_.tempSlam = ServiceData.localiz.getSlamPose();
        patternLineData_.debug_slamUpdateTime = SYSTEM_TOOL.getSystemTime();
        patternLineData_.option.crossLine = false;
        patternLineData_.startLine = true;
        patternLineData_.endLine = false;
        patternLineData_.areaMinX = 0;
        patternLineData_.areaMaxX = 0;
        patternLineData_.areaMinY = 0;
        patternLineData_.areaMaxY = 0;
        double minDistance = std::numeric_limits<double>::max(); // 초기 최소 거리를 최대값으로 설정
        double maxDistance = 0.0;
        tPoint startPoint;
        tPoint farthestPoint;
        double distance;
        for(tPoint poligon : areaPoligons)
        {
            if(patternLineData_.areaMaxX == 0)                  patternLineData_.areaMaxX = poligon.x;
            else if(poligon.x > patternLineData_.areaMaxX)      patternLineData_.areaMaxX = poligon.x;

            if(patternLineData_.areaMinX == 0)                  patternLineData_.areaMinX = poligon.x;
            else if(poligon.x < patternLineData_.areaMinX)      patternLineData_.areaMinX = poligon.x;

            if(patternLineData_.areaMaxY == 0)                  patternLineData_.areaMaxY = poligon.y;
            else if(poligon.y > patternLineData_.areaMaxY)      patternLineData_.areaMaxY = poligon.y;

            if(patternLineData_.areaMinY == 0)                  patternLineData_.areaMinY = poligon.y;
            else if(poligon.y < patternLineData_.areaMinY)      patternLineData_.areaMinY = poligon.y;

            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 청소영역 poligon : " << poligon.x << ", "<< poligon.y << "로봇의 현재 좌표 : " << robotPose.x << " , " << robotPose.y);
            
            distance = utils::math::distanceTwoPoint(robotPose, poligon);
            if (distance < minDistance) 
            {
                minDistance = distance;
                startPoint = poligon;
                ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 로봇과 가장 가까운 poligon : " << startPoint.x << ", "<< startPoint.y << " 거리 : " << distance);
            }

            distance = utils::math::distanceTwoPoint(startPoint, poligon);
            if (distance > maxDistance) 
            {
                maxDistance = distance;
                farthestPoint = poligon;
                ceblog(LOG_LV_NECESSARY, BOLDGREEN, "시작점에서 가장 먼 poligon : " << farthestPoint.x << ", "<< farthestPoint.y << " 거리 : " << distance);
            }
        }

        patternLineData_.areaRangeX = fabs(farthestPoint.x - startPoint.x);
        patternLineData_.areaRangeY = fabs(farthestPoint.y - startPoint.y);

        setLineTemp(E_PURPOSE_LINE_CLEAN_INFO::NONE);
        setCurLine(E_PURPOSE_LINE_CLEAN_INFO::NONE);
        setLinePatternStep(E_PATTERN_LINE_STEP::CLEANNING);
        setDoLineStep(E_DO_LINE_STEP::INIT);
        updateStartLine(robotPose);
        ret = E_PATTERN_STATE::RUN;
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CPatternLine::checkObstacleSize(const std::list<tPose>& avoidPathList,tPose *min, tPose *max)
{
    for(tPose avoidPose : avoidPathList)
    {
        if(min->x == 0) min->x = avoidPose.x;
        else if(avoidPose.x < min->x) min->x = avoidPose.x;
        
        if(min->y == 0) min->y = avoidPose.y;
        else if(avoidPose.y < min->y ) min->y = avoidPose.y;

        if(max->x == 0) max->x = avoidPose.x;
        else if(avoidPose.x > max->x) max->x = avoidPose.x;
        
        if(max->y == 0) max->y = avoidPose.y;
        if(avoidPose.y > max->y) max->y = avoidPose.y;
        ceblog(LOG_LV_NECESSARY, GREEN,  " 장애물 위치 : " << avoidPose.x << " , " << avoidPose.y);
    }
    
    ceblog(LOG_LV_NECESSARY, GREEN,  " 장애물 위치 X : " << min->x << " , " << max->x << " 장애물 위치 Y : " << min->y << " , " << max->y);
    return;
}

bool CPatternLine::checkLineAvoidEnd(E_PURPOSE_LINE_CLEAN_INFO curLine,E_PURPOSE_LINE_CLEAN_INFO shortLine,tPose robotPose,bool isAvoiding)
{
    tPose obsMin, obsMax;
    bool ret = false;

    // if(isEndLineInThisArea(robotPose))
    // {  
    //     obsMin = getAvoidStartPose();
    //     obsMax = getAvoidStartPose();
    //     checkObstacleSize(robotPose,&obsMin,&obsMax);
    //     patternLineData_.plan = getPathPlanSwapPair(getEndLineFixedPathPlan(robotPose,patternLineData_.plan,obsMin,obsMax));
    //     ret = true;
    // }
    // else if(isRetunCurrentLine(robotPose,curLine))
    // {
    //     obsMin = getAvoidStartPose();
    //     obsMax = getAvoidStartPose();
    //     checkObstacleSize(robotPose,&obsMin,&obsMax);
    //     patternLineData_.plan = getCurLineFixedPathPlan(robotPose,patternLineData_.plan,obsMin,obsMax);
    //     patternLineData_.plan.push_front(tPose(patternLineData_.lineTargetPose.x,robotPose.y,robotPose.angle));
    //     ret = true;
    // }
    // else if(isArriveNextLine(curLine,patternLineData_.nextTargetPose,robotPose))
    // {
    //     patternLineData_.plan = getNextLineFixedPathPlan(robotPose,patternLineData_.plan,curLine,shortLine);
    //     ret = true;
    // }

    return ret;
}
/**
 * @brief 현재 라인을 청소함.
 *  
 * @param robotPose 현재 로봇의 위치
 * @param pObsMap 장애물 맵 포인터
 * @return E_PATTERN_LINE_STEP
 */
E_PATTERN_STATE CPatternLine::runLinePatternStep(tPose robotPose,cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    E_PATTERN_STATE ret = E_PATTERN_STATE::RUN;
    tPose slamPose = ServiceData.localiz.getSlamPose();
    bool isSlamUpdate = utils::isUpdatePose(patternLineData_.tempSlam,slamPose);
    cleanLineInterval = ROS_CONFIG.cleanLineInterval;
    switch (getLinePatternStep())
    {
    case E_PATTERN_LINE_STEP::CLEANNING:
        ServiceData.robotMap.robotTrajectory.setCleanedTrajectory(robotPose);
        setLinePatternStep(lineCleanStepCleanning(isSlamUpdate,robotPose,pObstacle));
        break;
    case E_PATTERN_LINE_STEP::AVOIDING:
        ServiceData.robotMap.robotTrajectory.setCleanedTrajectory(robotPose);
        setLinePatternStep(lineCleanStepAvoiding(isSlamUpdate,robotPose,pObstacle));
        break;
    case E_PATTERN_LINE_STEP::WALLFOLLOW:
        setLinePatternStep(lineCleanStepWalltrack(isSlamUpdate,robotPose,pObstacle));
        break;
    case E_PATTERN_LINE_STEP::END:
        ret = lineCleanStepComplete(robotPose);
        break;
    default:
        break;
    }

    if(isSlamUpdate)
    {
        double interval = SYSTEM_TOOL.getSystemTime()-patternLineData_.debug_slamUpdateTime;
        patternLineData_.tempSlam = slamPose;
        #if 0
        patternLineData_.debug_slamUpdateTime = SYSTEM_TOOL.getSystemTime();
        if(interval > 0.1)
        {
            ceblog(LOG_LV_NECESSARY, YELLOW,  " Warnning!!! SlamUpdate Interval [ "<< interval << " ]" << "SlamPose [ " << slamPose.x << " ," << slamPose.y << " ]" << "Heading : " << utils::math::rad2deg(slamPose.angle));
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN,  "SlamPose [ " << slamPose.x << " ," << slamPose.y << " ]" << "Heading : " << utils::math::rad2deg(slamPose.angle));
            ceblog(LOG_LV_NECESSARY, GREEN,  " Good~~ SlamUpdate Interval [ "<< interval << " ]" << "SlamPose [ " << slamPose.x << " ," << slamPose.y << " ]" << "Heading : " << utils::math::rad2deg(slamPose.angle));
        }
        #endif
    }

    //__debug_state_print__(robotPose);

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

bool CPatternLine::completeLinePatternStep(tPose robotPose)
{
    CStopWatch __debug_sw;
    bool ret = true;
    //TODO : 미청소 영역이 있는지 확인한다.
    ceblog(LOG_LV_NECESSARY, GREEN,  " completeLinePatternStep : AREA CLEAN END");
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


E_PATTERN_LINE_STEP CPatternLine::lineCleanStepCleanning(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle)
{
    E_PATTERN_LINE_STEP ret = E_PATTERN_LINE_STEP::CLEANNING;
    tProfile profile = tProfile();
    profile.desAngVel = DEG2RAD(60);
    E_PURPOSE_LINE_CLEAN_INFO curLine = getCurLine();
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();
    E_PURPOSE_LINE_CLEAN_INFO newLine;

    double tempTargetAngle = 0;
    bool bAvoiding = avoidRun(robotPose,true);

    if(bAvoiding)
    {
        if(curLine == E_PURPOSE_LINE_CLEAN_INFO::NONE)
        {
            setLineTemp(curLine);
            setCurLine(getNewLine());
            curLine = getCurLine();
        }
        if(bAvoiding)
        {
            setAvoidStep(E_AVOID_LINE_STEP::INIT);
            return E_PATTERN_LINE_STEP::AVOIDING;
        }
    }

    switch (getDoLineStep())
    {
    case E_DO_LINE_STEP::INIT:
        #if USE_LINECLEAN_VELOCITY > 0
        tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
        // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
        #else
        MOTION.startRotateToPointOnMap(robotPose,patternLineData_.TargetPoint,profile);
        #endif
        setDoLineStep(E_DO_LINE_STEP::LINE_UPDATE);
        patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
        break;
    case E_DO_LINE_STEP::TURN:
        if(!MOTION.isRunning())
        {
            // tPoint fixedTarget = updateTargetPoint(curLine,robotPose,patternLineData_.endLine);
            // DEBUG_PUB.publishCleanLine1(robotPose,fixedTarget);
            patternLineData_.endLine = checkEndLine(robotPose);
            patternLineData_.startPose = robotPose;
            patternLineData_.TargetPoint = updateTargetPoint(curLine,robotPose,patternLineData_.endLine);
            DEBUG_PUB.publishCleanLine1(patternLineData_.startPose,patternLineData_.TargetPoint);

            if(curLine == shortLine)
            {
                
                profile.desLinVel = 0.2;
                patternLineData_.beforestartPose = patternLineData_.tempstartPose;
                patternLineData_.beforeTargetPoint = patternLineData_.tempTargetPoint;
                DEBUG_PUB.publishCleanLine2(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint);
                MOTION.startLinearToDistanceOnRobot(robotPose,cleanLineInterval,profile);
                #endif
            }
            else
            {
                MOTION.startLinearToLineOnMap(patternLineData_.startPose,patternLineData_.TargetPoint,profile);
                #endif
            }

            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 라인 직진!! 현재 라인 : " << enumToString(getCurLine()) <<
            "목표좌표 : " << patternLineData_.TargetPoint.x << ", "<< patternLineData_.TargetPoint.y << " 시작좌표 : " << patternLineData_.startPose.x << ", "<< patternLineData_.startPose.y);
            setDoLineStep(E_DO_LINE_STEP::GO);
        }
        
        break;
    case E_DO_LINE_STEP::GO:
        if(!MOTION.isRunning() || isArriveTargetLine(curLine,patternLineData_.TargetPoint,robotPose))
        {
            setDoLineStep(E_DO_LINE_STEP::ARRIVE);
        }
        break;
    case E_DO_LINE_STEP::ARRIVE:
        patternLineData_.tempstartPose = patternLineData_.startPose;
        patternLineData_.tempTargetPoint = patternLineData_.TargetPoint;
        newLine = updateLine(false);
        patternLineData_.startPose = robotPose;
        patternLineData_.TargetPoint = updateTargetPoint(newLine,robotPose,patternLineData_.endLine);
        setNewLine(newLine);

        if(patternLineData_.endLine)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 마지막 목표 도착 : " << patternLineData_.TargetPoint.x << ", "<< patternLineData_.TargetPoint.y);
            ret = E_PATTERN_LINE_STEP::END;
        }      
        else
        {
            #if USE_LINECLEAN_VELOCITY > 0
            double tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
            // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
            #else
            MOTION.startRotateToPointOnMap(robotPose,patternLineData_.TargetPoint,profile);
            #endif
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 다음 목표로 회전!! 다음 목적지 : " << patternLineData_.TargetPoint.x << ", "<< patternLineData_.TargetPoint.y << " 다음 라인 : " << enumToString(newLine));
            setDoLineStep(E_DO_LINE_STEP::LINE_UPDATE);
        }        
        
        break;
    case E_DO_LINE_STEP::LINE_UPDATE:
        if(!MOTION.isRunning())
        {
            newLine = getNewLine();
            setLineTemp(curLine);
            setCurLine(newLine);

            patternLineData_.endLine = checkEndLine(robotPose);
            patternLineData_.startPose = robotPose;
            patternLineData_.TargetPoint = updateTargetPoint(newLine,robotPose,patternLineData_.endLine);
            DEBUG_PUB.publishCleanLine1(patternLineData_.startPose,patternLineData_.TargetPoint);

            if(newLine == shortLine)
            {
                profile.desLinVel = 0.2;
                patternLineData_.beforestartPose = patternLineData_.tempstartPose;
                patternLineData_.beforeTargetPoint = patternLineData_.tempTargetPoint;
                DEBUG_PUB.publishCleanLine2(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint);
                MOTION.startLinearToDistanceOnRobot(robotPose,cleanLineInterval,profile);
                #endif
            }
            else
            {
                MOTION.startLinearToLineOnMap(patternLineData_.startPose,patternLineData_.TargetPoint,profile);
                #endif
            }   

            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 라인 직진!! 현재 라인 : " << enumToString(getCurLine()) <<
            "목표좌표 : " << patternLineData_.TargetPoint.x << ", "<< patternLineData_.TargetPoint.y << " 시작좌표 : " << patternLineData_.startPose.x << ", "<< patternLineData_.startPose.y);
            setDoLineStep(E_DO_LINE_STEP::GO);
        }
        break;                
        
    case E_DO_LINE_STEP::END:
        ret = E_PATTERN_LINE_STEP::END;
        break;            
    default:
        break;
    }

    if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
    {
        ceblog(LOG_LV_NECESSARY, BLUE, "청소 중.... 현재 라인 : " << enumToString(curLine) << " SHORT LINE : " << enumToString(shortLine) << "로봇 좌표 X : " << robotPose.x << " Y : " << robotPose.y);
        if((patternLineData_.startPose.x == patternLineData_.TargetPoint.x) || (patternLineData_.startPose.y == patternLineData_.TargetPoint.y))
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "현재 목표 시작 : " << patternLineData_.startPose.x << " ," << patternLineData_.startPose.y << " 목표 끝 : " << patternLineData_.TargetPoint.x << " ," << patternLineData_.TargetPoint.y);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "현재 목표 시작 : " << patternLineData_.startPose.x << " ," << patternLineData_.startPose.y << " 목표 끝 : " << patternLineData_.TargetPoint.x << " ," << patternLineData_.TargetPoint.y);
        }

        if(patternLineData_.tempstartPose.x == patternLineData_.tempTargetPoint.x || patternLineData_.tempstartPose.y == patternLineData_.tempTargetPoint.y)
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "이전 목표 시작 : " << patternLineData_.tempstartPose.x << " ," << patternLineData_.tempstartPose.y << " 목표 끝  : " << patternLineData_.tempTargetPoint.x << " ," << patternLineData_.tempTargetPoint.y);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "이전 목표 시작 : " << patternLineData_.tempstartPose.x << " ," << patternLineData_.tempstartPose.y << " 목표 끝  : " << patternLineData_.tempTargetPoint.x << " ," << patternLineData_.tempTargetPoint.y);
        }

        if(patternLineData_.beforestartPose.x == patternLineData_.beforeTargetPoint.x || patternLineData_.beforestartPose.y == patternLineData_.beforeTargetPoint.y)
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "과거 LongLine 목표 시작 : " << patternLineData_.beforestartPose.x << " , " << patternLineData_.beforestartPose.y << " 목표 끝 : " << patternLineData_.beforeTargetPoint.x << " , " << patternLineData_.beforeTargetPoint.y );
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "과거 LongLine 목표 시작 : " << patternLineData_.beforestartPose.x << " , " << patternLineData_.beforestartPose.y << " 목표 끝 : " << patternLineData_.beforeTargetPoint.x << " , " << patternLineData_.beforeTargetPoint.y );
        }
        patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
    }

    return ret;
}

/**
* @brief 현재 라인을 청소하는 절차 : 진행
 * - 현재 라인이 끝나기 전까지 직진 주행
 * - 장애물을 만나면 회피 후 직진
 * @param robotPose 
 * @param curLinetype 
 * @return E_PATTERN_LINE_STEP 
 */
E_PATTERN_LINE_STEP CPatternLine::lineCleanStepAvoiding(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;

    E_PATTERN_LINE_STEP ret = E_PATTERN_LINE_STEP::AVOIDING;
    tProfile profile;
    profile.desAngVel = DEG2RAD(60);
    E_PURPOSE_LINE_CLEAN_INFO curLine = getCurLine();
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();

    avoidRun(robotPose,true);

    if(!patternLineData_.endLine)   patternLineData_.endLine = checkEndLine(robotPose);
    if(slamUpdate)                  patternLineData_.avoidiPath.push_back(robotPose);

    switch (getAvoidStep())
    {
    case E_AVOID_LINE_STEP::INIT:
        patternLineData_.tempstartPose = patternLineData_.startPose;
        patternLineData_.tempTargetPoint = patternLineData_.TargetPoint;
        patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
        setAvoidStep(E_AVOID_LINE_STEP::BACK);
        break;
    case E_AVOID_LINE_STEP::BACK:
        if(getEscapeStep() == E_ESCAPE_STEP::RUN_WAYPOINT)
        {
            #if USE_WALLTRACKAVOIDING > 0
            if(patternLineData_.isAvoidTurnLeft || curLine == shortLine)
            {
                setAvoidStep(E_AVOID_LINE_STEP::TURN);
            }
            else if(!MOTION.isRunning())
            {
                setDoLineStep(E_DO_LINE_STEP::ARRIVE);
                setAvoidStep(E_AVOID_LINE_STEP::END);
            } 
            #else
            if(curLine == shortLine)
            {
                patternLineData_.checkAvoidingEnd = false;
                patternLineData_.avoidiPath.push_back(robotPose);
                setAvoidStep(E_AVOID_LINE_STEP::AVOIDING);
                ceblog(LOG_LV_NECESSARY, BLUE, "라인 청소 장애물 회피 시작! 시작 좌표 X : " << robotPose.x << " Y : " << robotPose.y );
            }
            else if(!MOTION.isRunning())
            {
                setDoLineStep(E_DO_LINE_STEP::ARRIVE);
                setAvoidStep(E_AVOID_LINE_STEP::END);
            }
            #endif
        }
        break;
    case E_AVOID_LINE_STEP::TURN:
        #if USE_WALLTRACKAVOIDING > 0
        if(patternLineData_.isAvoidTurnLeft)
        {
            if(isAvoidingEnd())
            {
                initAvoiding();
                patternLineData_.isAvoidTurnLeft = false;
                patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
                SUB_TASK.walltracking.initWallTrack();
                ret = E_PATTERN_LINE_STEP::WALLFOLLOW;
                ceblog(LOG_LV_NECESSARY, BLUE, "청소 중 벽타기 장애물 회피 시작! 시작 좌표 X : " << robotPose.x << " Y : " << robotPose.y );
            }
        }
        else if(curLine == shortLine)
        {
            patternLineData_.checkAvoidingEnd = false;
            patternLineData_.avoidiPath.push_back(robotPose);
            setAvoidStep(E_AVOID_LINE_STEP::AVOIDING);
            ceblog(LOG_LV_NECESSARY, BLUE, "라인 청소 장애물 회피 시작! 시작 좌표 X : " << robotPose.x << " Y : " << robotPose.y );
        }
        #else
        ceblog(LOG_LV_NECESSARY, BLUE, "라인 청소 장애물 회피 회전 중!!! 이로그가 뜨면 오류임!!");
        #endif
        break;
    case E_AVOID_LINE_STEP::AVOIDING:
        if(isEndLineInThisArea(robotPose))
        {
            initAvoiding();
            
            ret = E_PATTERN_LINE_STEP::END;
        }
        if(isClosedEndLine(robotPose,curLine,shortLine))
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 장애물 회피 중 공간 폐쇠 다음 라인으로 갈수 없음!! 청소 종료 ");
            //setAvoidStep(E_AVOID_LINE_STEP::END);
            initAvoiding();
            
            ret = E_PATTERN_LINE_STEP::END;
        }
        else if(isArriveTargetLine(curLine,patternLineData_.TargetPoint,robotPose))
        {
            patternLineData_.startLine = false;
            patternLineData_.avoidiPath.push_back(robotPose);
            E_PURPOSE_LINE_CLEAN_INFO newLine = updateLine(true);

            //patternLineData_.beforestartPose = patternLineData_.tempstartPose;
            //patternLineData_.beforeTargetPoint = patternLineData_.tempTargetPoint;
            //DEBUG_PUB.publishCleanLine2(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint);
            /*patterns.cpp : 1130 line SET PUBLISH LINE OLD-PATH --- Function - PUBLISHER(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint); */
            patternLineData_.startPose = robotPose;
            patternLineData_.TargetPoint = updateTargetPoint(newLine,robotPose,patternLineData_.endLine);
            setNewLine(newLine);
            double tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
            // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
            setDoLineStep(E_DO_LINE_STEP::LINE_UPDATE);
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 장애물 회피 중 다음 라인 도착 --> 새로운 라인 청소시작");
            setAvoidStep(E_AVOID_LINE_STEP::END);
        }
        else if(isArriveNextLine(shortLine,patternLineData_.TargetPoint,robotPose))
        {
            E_PURPOSE_LINE_CLEAN_INFO newLine = updateLine(true);
            // patternLineData_.beforestartPose = patternLineData_.tempstartPose;
            // patternLineData_.beforeTargetPoint = patternLineData_.tempTargetPoint;

            // DEBUG_PUB.publishCleanLine2(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint);
            /*patterns.cpp : 1130 line SET PUBLISH LINE OLD-PATH --- Function - PUBLISHER(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint); */
            patternLineData_.startPose = robotPose;
            patternLineData_.TargetPoint = updateTargetPoint(newLine,robotPose,patternLineData_.endLine);
            setNewLine(newLine);
            initAvoiding();
            double tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
            // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
            setDoLineStep(E_DO_LINE_STEP::LINE_UPDATE);
            setAvoidStep(E_AVOID_LINE_STEP::END);
            //ret = E_PATTERN_LINE_STEP::CLEANNING;
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 벽면주행 회피 중 다음 라인 도착 --> 새로운 라인 청소시작");
        }
        break;
    case E_AVOID_LINE_STEP::END:
        initAvoiding();
        ret = E_PATTERN_LINE_STEP::CLEANNING;
        break;            
    default:
        break;
    }


    if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
    {
        ceblog(LOG_LV_NECESSARY, BLUE, "회피 중.... 현재 라인 : " << enumToString(curLine) << " SHORT LINE : " << enumToString(shortLine) << "로봇 좌표 X : " << robotPose.x << " Y : " << robotPose.y);

        if((patternLineData_.startPose.x == patternLineData_.TargetPoint.x) || (patternLineData_.startPose.y == patternLineData_.TargetPoint.y))
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "현재 목표 시작 : " << patternLineData_.startPose.x << " ," << patternLineData_.startPose.y << " 목표 끝 : " << patternLineData_.TargetPoint.x << " ," << patternLineData_.TargetPoint.y);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "현재 목표 시작 : " << patternLineData_.startPose.x << " ," << patternLineData_.startPose.y << " 목표 끝 : " << patternLineData_.TargetPoint.x << " ," << patternLineData_.TargetPoint.y);
        }

        if(patternLineData_.tempstartPose.x == patternLineData_.tempTargetPoint.x || patternLineData_.tempstartPose.y == patternLineData_.tempTargetPoint.y)
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "이전 목표 시작 : " << patternLineData_.tempstartPose.x << " ," << patternLineData_.tempstartPose.y << " 목표 끝  : " << patternLineData_.tempTargetPoint.x << " ," << patternLineData_.tempTargetPoint.y);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "이전 목표 시작 : " << patternLineData_.tempstartPose.x << " ," << patternLineData_.tempstartPose.y << " 목표 끝  : " << patternLineData_.tempTargetPoint.x << " ," << patternLineData_.tempTargetPoint.y);
        }

        if(patternLineData_.beforestartPose.x == patternLineData_.beforeTargetPoint.x || patternLineData_.beforestartPose.y == patternLineData_.beforeTargetPoint.y)
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "과거 LongLine 목표 시작 : " << patternLineData_.beforestartPose.x << " , " << patternLineData_.beforestartPose.y << " 목표 끝 : " << patternLineData_.beforeTargetPoint.x << " , " << patternLineData_.beforeTargetPoint.y );
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED, "과거 LongLine 목표 시작 : " << patternLineData_.beforestartPose.x << " , " << patternLineData_.beforestartPose.y << " 목표 끝 : " << patternLineData_.beforeTargetPoint.x << " , " << patternLineData_.beforeTargetPoint.y );
        }
        patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
* @brief 현재 라인을 청소하는 절차 : 진행
 * - 현재 라인이 끝나기 전까지 직진 주행
 * - 장애물을 만나면 회피 후 직진
 * @param robotPose 
 * @param curLinetype 
 * @return E_PATTERN_LINE_STEP 
 */
E_PATTERN_LINE_STEP CPatternLine::lineCleanStepWalltrack(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;

    E_PATTERN_LINE_STEP ret = E_PATTERN_LINE_STEP::WALLFOLLOW;
    E_PURPOSE_LINE_CLEAN_INFO curLine = getCurLine();
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();
    tProfile profile;
    profile.desAngVel = DEG2RAD(60);

    if(isClosedEndLine(robotPose,curLine,shortLine))
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 장애물 회피 중 공간 폐쇠 다음 라인으로 갈수 없음!! 청소 종료 ");
        ret = E_PATTERN_LINE_STEP::END;
    }
    else if(isArriveNextLine(shortLine,patternLineData_.TargetPoint,robotPose))
    {
        E_PURPOSE_LINE_CLEAN_INFO newLine = updateLine(true);
        
        // patternLineData_.beforestartPose = patternLineData_.tempstartPose;
        // patternLineData_.beforeTargetPoint = patternLineData_.tempTargetPoint;
        // DEBUG_PUB.publishCleanLine2(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint);
        /*patterns.cpp : 1130 line SET PUBLISH LINE OLD-PATH --- Function - PUBLISHER(patternLineData_.beforestartPose,patternLineData_.beforeTargetPoint); */

        patternLineData_.startPose = robotPose;
        patternLineData_.TargetPoint = updateTargetPoint(newLine,robotPose,patternLineData_.endLine);
        setNewLine(newLine);
        initAvoiding();
        double tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
        // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
        setDoLineStep(E_DO_LINE_STEP::LINE_UPDATE);
        ret = E_PATTERN_LINE_STEP::CLEANNING;
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 벽면주행 회피 중 다음 라인 도착 --> 새로운 라인 청소시작");
    }
    else if(isRetunCurrentLine(robotPose,curLine))
    {
        initAvoiding();
        double tempTargetAngle = atan2(patternLineData_.TargetPoint.y - robotPose.y, patternLineData_.TargetPoint.x - robotPose.x);
        // MOTION.startRotateToAngleOnRobot(robotPose,utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle),profile);
        setDoLineStep(E_DO_LINE_STEP::TURN);
        ret = E_PATTERN_LINE_STEP::CLEANNING;
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 벽면주행 회피 중 기존 라인으로 복귀함 --> 라인 이어서 청소시작");
    }
    else
    {
        SUB_TASK.walltracking.runWallTrackPattern();
        if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "벽타기 장애물 중.... 로봇 좌표 X : " << robotPose.x << " Y : " << robotPose.y );
            patternLineData_.debugWallStartTime = SYSTEM_TOOL.getSystemTime();
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


E_PATTERN_STATE CPatternLine::lineCleanStepComplete(tPose robotPose)
{
    CStopWatch __debug_sw;

    E_PATTERN_STATE ret = E_PATTERN_STATE::COMPLETE;
    
    eblog(LOG_LV_NECESSARY, "do line end!  Robot( " << robotPose.x<< ", " <<  robotPose.y << " , " <<  utils::math::rad2deg(robotPose.angle) << " ) ");
    
    patternLineData_.startPose = tPose(0,0,0);
    patternLineData_.TargetPoint = tPoint(0,0);
    patternLineData_.tempstartPose = tPose(0,0,0);
    patternLineData_.tempTargetPoint = tPoint(0,0);
    patternLineData_.beforestartPose = tPose(0,0,0);
    patternLineData_.beforeTargetPoint = tPoint(0,0);

    if(patternLineData_.option.crossLine && !patternLineData_.doDoubleClean)
    {
        patternLineData_.doDoubleClean = true;
        setLinePatternStep(E_PATTERN_LINE_STEP::CLEANNING);
        ret = E_PATTERN_STATE::RUN;
    }

    /*x
    if(CheckAreaCleanEnd())
    {
        startmoving to LineStartPose
    }
    else
    {
        startmoving to NextArea
    }
    */
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief  청소시간 테스트를 위한 랜덤 청소 RUN함수. 기본 적인 직진주행과 장애물 감지 시 랜덤 각도로 회피하여 주행하는 알고리즘
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
void CPatternLine::runRandomCleanPattern(tPose robotPose,RSU_OBSTACLE_DATA* pObstacle)
{
    tProfile profile;
    if(avoidRun(robotPose, true))
    {
        if(getAvoidStatus() == E_AVOID_STATUS::COMPLETE)
        {
            if(getAvoidType() == E_AVOID_TYPE::LIDAR)
            {
                // MOTION.startLinearOnVelocity(robotPose,profile.desLinVel,profile);
            }
            else
            {
                RSF_OBSTACLE_MASK mask = getAvoidMask();
                if(mask.value)
                {
                    int duty = 0;
                    if((mask.b.fleft_side && mask.b.fright_side)|| mask.b.fleft_center || mask.b.fright_center)
                    {
                        duty = 120;
                    }
                    else if(mask.value & 0x0F)
                    {
                        if(mask.value & 0x0E) duty = 90;
                        else                  duty = 45;
                    }
                    else
                    {
                        
                        if(mask.value & 0x70) duty = -90;
                        else                  duty = -45;
                    }
                    
                    // MOTION.startRotateToAngleOnRobot(robotPose,DEG2RAD(duty),profile);
                }
            }
        }
    }
    else if(!MOTION.isRunning())
    {
        // MOTION.startLinearOnVelocity(robotPose,profile.desLinVel,profile);
    }
}

E_PATTERN_LINE_STEP CPatternLine::checkWallFollowingEnd(tPose robotPose)
{

    E_PATTERN_LINE_STEP ret = E_PATTERN_LINE_STEP::WALLFOLLOW;

    // if(isClosedEndLine(robotPose)){
    //     ret = E_PATTERN_LINE_STEP::END;
    // }
    // else if(isArriveNextLine(robotPose,patternLineData_.nextPose)){//|| isRetunCurrentLine(robotPose) else if(isArriveNextLine(robotPose)){
    //     ret = E_PATTERN_LINE_STEP::READY;
    // }

    return ret; 
}

bool CPatternLine::isTurnLeftByLine(E_PURPOSE_LINE_CLEAN_INFO curLine, E_PURPOSE_LINE_CLEAN_INFO tempLine, E_PURPOSE_LINE_CLEAN_INFO shortLine)
{
    bool ret = false;

    if(curLine == shortLine)
    {
        if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::UP)    ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)    ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::UP)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)    ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)    ret = true;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "라인청소 장애물 회피 방향 결정 라인 타입 오류!! ");
        }

        if(ret)
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "shortLine 장애물 회피 방향 결정 좌회전!! ShortLine : " << enumToString(shortLine) << " tempLine : " << enumToString(shortLine) );
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "shortLine 장애물 회피 방향 결정 우회전!! ShortLine : " << enumToString(shortLine) << " tempLine : " << enumToString(shortLine));
        }
    }
    else
    {
        if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)
        {
            if(curLine == E_PURPOSE_LINE_CLEAN_INFO::UP)    ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)
        {
            if(curLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)  ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::UP)
        {
            if(curLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)  ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
        {
            if(curLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT) ret = true;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "라인청소 장애물 회피 방향 결정 라인 타입 오류!! ");
        }

        if(ret)
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "LonLine 장애물 회피 방향 결정 좌회전!! ShortLine : " << enumToString(shortLine) << " curLine : " << enumToString(curLine) );
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "LonLine 장애물 회피 방향 결정 우회전!! ShortLine : " << enumToString(shortLine) << " curLine : " << enumToString(curLine));
        }
    }

    return ret;
}


E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getCurrentLine(tPose target, tPose start)
{
    E_PURPOSE_LINE_CLEAN_INFO ret = E_PURPOSE_LINE_CLEAN_INFO::NONE;


    if(target.x == start.x)
    {
        if(target.y > start.y)  ret = E_PURPOSE_LINE_CLEAN_INFO::LEFT;
        else                    ret = E_PURPOSE_LINE_CLEAN_INFO::RIGHT;             
    }
    else if(target.y == start.y)
    {
        if(target.x > start.x)  ret = E_PURPOSE_LINE_CLEAN_INFO::UP;
        else                    ret = E_PURPOSE_LINE_CLEAN_INFO::DOWN;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, GREEN, "시작점과 목표점에 대한 X,Y좌표가 모두 달라요..");
        if(fabs(target.x-start.x) >= fabs(target.y-start.y))
        {
            if(target.x > start.x)  ret = E_PURPOSE_LINE_CLEAN_INFO::UP;
            else                    ret = E_PURPOSE_LINE_CLEAN_INFO::DOWN;
        }
        else
        {
            if(target.y > start.y)  ret = E_PURPOSE_LINE_CLEAN_INFO::LEFT;
            else                    ret = E_PURPOSE_LINE_CLEAN_INFO::RIGHT;
        }
    }

    return ret;
}

void CPatternLine::updateStartLine(tPose robotPose)
{

    ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 청소영역 최소 X축 좌표 : " 
        << patternLineData_.areaMinX << " , 최대 X축 좌표 : " 
        << patternLineData_.areaMaxX << " , 최소 Y축 좌표 : " 
        << patternLineData_.areaMinY << " , 최대 Y축 좌표 : " 
        << patternLineData_.areaMaxY << " , 영역의 X축 크기 : " 
        << patternLineData_.areaRangeX << " , 영역의 Y축 크기 : " 
        << patternLineData_.areaRangeY);

    if(patternLineData_.areaRangeX >= patternLineData_.areaRangeY)
    {
        if(fabs(patternLineData_.areaMaxX-robotPose.x) >= fabs(patternLineData_.areaMinX-robotPose.x))      patternLineData_.TargetPoint = tPoint(patternLineData_.areaMaxX,robotPose.y);
        else                                                                                                patternLineData_.TargetPoint = tPoint(patternLineData_.areaMinX,robotPose.y);

        if(patternLineData_.TargetPoint.x >= robotPose.x)    setNewLine(E_PURPOSE_LINE_CLEAN_INFO::UP);
        else                                                 setNewLine(E_PURPOSE_LINE_CLEAN_INFO::DOWN);

        if(fabs(patternLineData_.areaMaxY-robotPose.y) >= fabs(patternLineData_.areaMinY-robotPose.y))  setShortLine(E_PURPOSE_LINE_CLEAN_INFO::LEFT);
        else                                                                                            setShortLine(E_PURPOSE_LINE_CLEAN_INFO::RIGHT);
    }
    else
    {
        if(fabs(patternLineData_.areaMaxY-robotPose.y) >= fabs(patternLineData_.areaMinY-robotPose.y))      patternLineData_.TargetPoint = tPoint(robotPose.x,patternLineData_.areaMaxY);
        else                                                                                                patternLineData_.TargetPoint = tPoint(robotPose.x,patternLineData_.areaMinY);

        if(patternLineData_.TargetPoint.y >= robotPose.y)    setNewLine(E_PURPOSE_LINE_CLEAN_INFO::LEFT);
        else                                                 setNewLine(E_PURPOSE_LINE_CLEAN_INFO::RIGHT);

        if(fabs(patternLineData_.areaMaxX-robotPose.x) >= fabs(patternLineData_.areaMinX-robotPose.x))  setShortLine(E_PURPOSE_LINE_CLEAN_INFO::UP);
        else                                                                                            setShortLine(E_PURPOSE_LINE_CLEAN_INFO::DOWN);
    }
    patternLineData_.startPose = robotPose;
    ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 라인청소 시작!! 목표점으로 회전 목표좌표 : " << patternLineData_.TargetPoint.x << ", "<< patternLineData_.TargetPoint.y << 
    " 시작 라인 : " << enumToString(getNewLine()) << " 짧은 라인 : " << enumToString(getShortLine())); 
}

E_PURPOSE_LINE_CLEAN_INFO CPatternLine::getStartLine(tPose robotPose, tPoint halfPoint)
{
    E_PURPOSE_LINE_CLEAN_INFO ret = E_PURPOSE_LINE_CLEAN_INFO::NONE;

    if(robotPose.x <= halfPoint.x)  ret = E_PURPOSE_LINE_CLEAN_INFO::UP;
    else                            ret = E_PURPOSE_LINE_CLEAN_INFO::DOWN;

    return ret;
}



bool CPatternLine::checkEndLine(tPose robotPose)
{
    bool ret = false;
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();
    
    switch (shortLine)
    {
    case E_PURPOSE_LINE_CLEAN_INFO::UP:
        //if(robotPose.x >= patternLineData_.areaMaxX) ret = true;
        if(robotPose.x+cleanLineInterval >= patternLineData_.areaMaxX) ret = true;
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
        //if(robotPose.x <= patternLineData_.areaMinX)  ret = true;
        if(robotPose.x-cleanLineInterval <= patternLineData_.areaMinX)  ret = true;
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
        //if(robotPose.y >= patternLineData_.areaMaxY)  ret = true;
        if(robotPose.y+cleanLineInterval >= patternLineData_.areaMaxY)  ret = true;
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:
        //if(robotPose.y <= patternLineData_.areaMinY)  ret = true;
        if(robotPose.y-cleanLineInterval <= patternLineData_.areaMinY)  ret = true;
        break;     
    default:
        eblog(LOG_LV_NECESSARY,  "checkEndLine - ERROR LINE TYPE UNKOWN");
        break;
    }

    if(ret)
    {
        eblog(LOG_LV_NECESSARY,  "Check EndLine ShortLine : " << enumToString(shortLine) << "robotPose : " << robotPose.x << " , " << robotPose.y <<
        " areaMinX : " << patternLineData_.areaMinX << "areaMaxX : " << patternLineData_.areaMaxX << " areaMinY : " << patternLineData_.areaMinY << "areaMaxY : " << patternLineData_.areaMaxY);
    } 

    return ret;
}

tPoint CPatternLine::getUpdateShortLineTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine,tPose robotPose, bool endLine)
{
    tPoint ret;
    switch (newLine)
    {
    case E_PURPOSE_LINE_CLEAN_INFO::UP:
        if(endLine)     ret = tPoint(patternLineData_.areaMaxX,robotPose.y);
        else            ret = tPoint(robotPose.x+cleanLineInterval,robotPose.y);//ret = tPoint(patternLineData_.tempTargetPoint.x+cleanLineInterval,robotPose.y);
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
        if(endLine)     ret = tPoint(patternLineData_.areaMinX,robotPose.y);
        else            ret = tPoint(robotPose.x-cleanLineInterval,robotPose.y);//ret = tPoint(patternLineData_.tempTargetPoint.x-cleanLineInterval,robotPose.y);

        break;
    case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
        if(endLine)     ret = tPoint(robotPose.x,patternLineData_.areaMaxY);
        else            ret = tPoint(robotPose.x,robotPose.y+cleanLineInterval);//ret = tPoint(robotPose.x,patternLineData_.tempTargetPoint.y+cleanLineInterval);
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:
        if(endLine)     ret = tPoint(robotPose.x,patternLineData_.areaMinY);
        else            ret = tPoint(robotPose.x,robotPose.y-cleanLineInterval);//ret = tPoint(robotPose.x,patternLineData_.tempTargetPoint.y-cleanLineInterval);
        break;     
    default:
        break;
    }

    return ret;
}

tPoint CPatternLine::getUpdateLongLineTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine, tPose robotPose)
{
    tPoint ret;
    switch (newLine)
    {
    case E_PURPOSE_LINE_CLEAN_INFO::UP:
        ret = tPoint(patternLineData_.areaMaxX,robotPose.y);
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
        ret = tPoint(patternLineData_.areaMinX,robotPose.y);
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
        ret = tPoint(robotPose.x,patternLineData_.areaMaxY);
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:
        ret = tPoint(robotPose.x,patternLineData_.areaMinY);
        break;        
    default:
        break;
    }

    return ret;
}
/**
 * @brief 라인끝 지점을 업데이트 함. update 지점을 찾지못하면 false 리턴
 * 항상 updateLinetype() 후에 진행
 * 
 * @param linetype 라인타입
 * @param area area info
 * @param interval width 방향으로 이동할 간격 (단위, m)
 * @return false: 업데이트 실패
 */
tPoint CPatternLine::updateTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine,tPose robotPose,bool isEndLine)
{
    CStopWatch __debug_sw;

    tPoint ret;
    E_PURPOSE_LINE_CLEAN_INFO curLine = getCurLine();
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();

    if(newLine == shortLine)
    {
        ret = getUpdateShortLineTargetPoint(newLine,robotPose,isEndLine);
    }
    else
    {
        ret = getUpdateLongLineTargetPoint(newLine,robotPose);
    }

    eblog(LOG_LV_NECESSARY,  "[updateTargetPoint]  New nextPoint :" << ret.x << " ," << ret.y);

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


tPoint CPatternLine::checkTargetPointFromAvoidPath(E_PURPOSE_LINE_CLEAN_INFO newLine,tPoint target, const std::list<tPose>& avoidPathList)
{
    tPose avoidMin, avoidMax;
    tPoint fixedTarget = target;

    if(avoidPathList.empty())
    {
        ceblog(LOG_LV_NECESSARY, GREEN,  "AvoidPath is Empty!! :" << enumToString(newLine) << "Origin Target X : " << target.x << " Y : " << target.y);
    }
    else
    {
        checkObstacleSize(avoidPathList,&avoidMin,&avoidMax);
        if(newLine == E_PURPOSE_LINE_CLEAN_INFO::UP || newLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
        {
            if(target.y >= avoidMin.y && target.y <= avoidMax.y)
            {
                if(newLine == E_PURPOSE_LINE_CLEAN_INFO::UP)    fixedTarget.x = avoidMin.x;
                else                                            fixedTarget.x = avoidMax.x;
            }
        }
        else if(newLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT || newLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)
        {
            if(target.x >= avoidMin.x && target.x <= avoidMax.x)
            {
                if(newLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)   fixedTarget.y = avoidMin.y;
                else                                             fixedTarget.y = avoidMax.y;
            }
        }
        ceblog(LOG_LV_NECESSARY, GREEN,  "Fixed Target from AvoidPath :" << enumToString(newLine) << "Avoid Path Min X : " << avoidMin.x << " Y : " << avoidMin.y << "Max X : " << avoidMax.x << " Y : " << avoidMax.y <<
        " Origin Target X : " << target.x << " Y : " << target.y << " Fixed Target X : " << fixedTarget.x << " Y : " << fixedTarget.y);
    }

    return fixedTarget;
    
}

/**
 * @brief 현재 LinePlan에 따라 do clean할 라인종류를 업데이트함.
 * 
 * @return E_PURPOSE_LINE_CLEAN_INFO 바뀐 라인종류
 */
E_PURPOSE_LINE_CLEAN_INFO CPatternLine::updateLine(bool isArriveNextLine)
{
    CStopWatch __debug_sw;
    E_PURPOSE_LINE_CLEAN_INFO newLine;

    E_PURPOSE_LINE_CLEAN_INFO curLine = getCurLine();
    E_PURPOSE_LINE_CLEAN_INFO shortLine = getShortLine();
    E_PURPOSE_LINE_CLEAN_INFO tempLine = getLineTemp();

    if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT || shortLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)
    {
        if(curLine == shortLine)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::UP)          newLine = E_PURPOSE_LINE_CLEAN_INFO::DOWN;
            else if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)   newLine = E_PURPOSE_LINE_CLEAN_INFO::UP;
            else
            {
                ceblog(LOG_LV_NECESSARY, GREEN,  "update TempLine Error!!! :" << enumToString(tempLine) << " Short Line : " << enumToString(shortLine) << "Cur Line : " << enumToString(curLine));
            }
        }
        else
        {
            if(isArriveNextLine)
            {
                if(curLine == E_PURPOSE_LINE_CLEAN_INFO::UP)        newLine = E_PURPOSE_LINE_CLEAN_INFO::DOWN;
                else if(curLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN) newLine = E_PURPOSE_LINE_CLEAN_INFO::UP;
                else
                {
                    ceblog(LOG_LV_NECESSARY, GREEN,  "isArriveNextLine -- update CurLine Error!!! CurLine :" << enumToString(curLine) << " Short Line : " << enumToString(shortLine) << "Temp Line : " << enumToString(tempLine));
                }
            }
            else
            {
                if(curLine == E_PURPOSE_LINE_CLEAN_INFO::UP || curLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)  newLine = shortLine;
                else
                {
                    ceblog(LOG_LV_NECESSARY, GREEN,  "update CurLine Error!!! CurLine :" << enumToString(curLine) << " Short Line : " << enumToString(shortLine) << "Temp Line : " << enumToString(tempLine));
                }
            }
        }
    }
    else
    {
        if(curLine == shortLine)
        {
            if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)         newLine = E_PURPOSE_LINE_CLEAN_INFO::RIGHT;
            else if(tempLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)   newLine = E_PURPOSE_LINE_CLEAN_INFO::LEFT;
            else
            {
                 ceblog(LOG_LV_NECESSARY, GREEN,  "update TempLine Error!!! :" << enumToString(tempLine) << " Short Line : " << enumToString(shortLine) << "Cur Line : " << enumToString(curLine));
            }
        }
        else
        {
            if(isArriveNextLine)
            {
                if(curLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT)         newLine = E_PURPOSE_LINE_CLEAN_INFO::RIGHT;
                else if(curLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)   newLine = E_PURPOSE_LINE_CLEAN_INFO::LEFT;
                else
                {
                    ceblog(LOG_LV_NECESSARY, GREEN,  "isArriveNextLine -- update CurLine Error!!! CurLine :" << enumToString(curLine) << " Short Line : " << enumToString(shortLine) << "Temp Line : " << enumToString(tempLine));
                }
            }
            else
            {
                if(curLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT || curLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)   newLine = shortLine;
                else
                {
                    ceblog(LOG_LV_NECESSARY, GREEN,  "update CurLine Error!!! CurLine :" << enumToString(curLine) << " Short Line : " << enumToString(shortLine) << "Temp Line : " << enumToString(tempLine));
                }
            }
        }
    }

    ceblog(LOG_LV_NECESSARY, GREEN,  "update Line Success!! CurLine :" << enumToString(curLine) << " -----> Next Line : " << enumToString(newLine) << " Temp Line : " << enumToString(tempLine));

    return newLine;
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CPatternLine::isEndLineInThisArea(tPose robotPose)
{
    CStopWatch __debug_sw;

    bool ret = false;

    if(robotPose.x >= patternLineData_.areaMaxX || robotPose.y >= patternLineData_.areaMaxY || robotPose.x <= patternLineData_.areaMinX || robotPose.y <= patternLineData_.areaMinY )
    {
        if(robotPose.x >= patternLineData_.areaMaxX)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN,  "장애물 회피 중 영역 넘음 최대 X축 : " << patternLineData_.areaMaxX << "ROBOT : " << robotPose.x << " , " << robotPose.y);
        }
        if(robotPose.y >= patternLineData_.areaMaxY)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN,  "장애물 회피 중 영역 넘음 최대 Y축 : " << patternLineData_.areaMaxY << "ROBOT : " << robotPose.x << " , " << robotPose.y);
        }
        if(robotPose.x <= patternLineData_.areaMinX)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN,  "장애물 회피 중 영역 넘음 최소 X축 : " << patternLineData_.areaMinX << "ROBOT : " << robotPose.x << " , " << robotPose.y);
        }
        if(robotPose.y <= patternLineData_.areaMinY)
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN,  "장애물 회피 중 영역 넘음 최소 Y축 : " << patternLineData_.areaMinY << "ROBOT : " << robotPose.x << " , " << robotPose.y);
        }

        ret = true;
    }         

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

bool CPatternLine::isArriveTargetLine(E_PURPOSE_LINE_CLEAN_INFO curLine ,tPoint target,tPose robotPose)
{
     CStopWatch __debug_sw;

    bool ret = false;

    switch (curLine)
    {
        case E_PURPOSE_LINE_CLEAN_INFO::UP:
            if(robotPose.x >= target.x) ret = true;         
            break;
        case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
            if(robotPose.x <= target.x) ret = true;               
            break;
        case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:     
            if(robotPose.y <= target.y) ret = true;               
            break;    
        case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
            if(robotPose.y >= target.y) ret = true;               
            break;
        default:
            eblog(LOG_LV_NECESSARY,  "isArriveTargetLine CurLine error!!");
            break;
    }

    if(ret)
    {
        eblog(LOG_LV_NECESSARY, "Arrive TargetLine : " << enumToString(curLine) << "targetPoint : " << target.x << "," << target.y << " RobotPose : "<< robotPose.x << "," << robotPose.y);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

bool CPatternLine::isArriveNextLine(E_PURPOSE_LINE_CLEAN_INFO shortLine ,tPoint target,tPose robotPose)
{
     CStopWatch __debug_sw;

    bool ret = false;

    switch (shortLine)
    {
        case E_PURPOSE_LINE_CLEAN_INFO::UP:
            if(robotPose.x >= target.x+cleanLineInterval) ret = true;         
            break;
        case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
            if(robotPose.x <= target.x-cleanLineInterval) ret = true;               
            break;
        case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:     
            if(robotPose.y <= target.y-cleanLineInterval) ret = true;               
            break;    
        case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
            if(robotPose.y >= target.y+cleanLineInterval) ret = true;               
            break;
        default:
            eblog(LOG_LV_NECESSARY,  "isArriveNextLine CurLine error!!");
            break;
    }

    if(ret)
    {
        eblog(LOG_LV_NECESSARY, "Arrive NextLine : " << enumToString(shortLine) << "targetPoint : " << target.x << "," << target.y << " RobotPose : "<< robotPose.x << "," << robotPose.y);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


// 함수로 직사각형을 축소시키는 함수
std::list<tPoint> CPatternLine::shrinkRectangle(std::list<tPoint> areaPolygons, double upMargin, double downMargin, double leftMargin, double rightMargin) 
{
    // 축소된 좌표를 저장할 벡터
    std::list<tPoint> newCoordinates;
    tPoint centerXY = SUB_TASK.cleanPlan.getAreaCenterPoint();
    
    double newX;
    double newY;
    // 각 꼭짓점을 축소시킴
    for (tPoint polygon : areaPolygons) 
    {
        if(polygon.x < centerXY.x)
        {
            newX = polygon.x + downMargin;
        }
        else
        {
            newX = polygon.x - upMargin;
        }

        if(polygon.y < centerXY.y)
        {
            newY = polygon.y + rightMargin;
        }
        else
        {
            newY = polygon.y - leftMargin;
        }
        
        newCoordinates.emplace_back(newX, newY);
    }

    // 축소된 좌표를 원래 좌표로 대체
    return newCoordinates;
}

// 주어진 점과 3개의 다른 점을 비교하여 가장 먼 점을 찾는 함수
tPoint CPatternLine::findFarthestPoint(tPoint referencePoint, std::list<tPoint> otherPoints) 
{
    double maxDistance = 0.0;
    tPoint farthestPoint = tPoint(0,0);
    for (tPoint other : otherPoints) 
    {
        double distance = utils::math::distanceTwoPoint(referencePoint, other);
        if (distance > maxDistance) 
        {
            maxDistance = distance;
            farthestPoint = other;
        }
    }

    return farthestPoint;
}

tPoint CPatternLine::findClosestPoint(tPoint referencePoint, std::list<tPoint> otherPoints) 
{
    double minDistance = std::numeric_limits<double>::max(); // 초기 최소 거리를 최대값으로 설정
    tPoint closestPoint = tPoint(0, 0);

    for (tPoint other : otherPoints) 
    {
        double distance = utils::math::distanceTwoPoint(referencePoint, other);
        if (distance < minDistance) 
        {
            minDistance = distance;
            closestPoint = other;
        }
    }

    return closestPoint;
}

bool CPatternLine::isClosedEndLine(tPose robotPose, E_PURPOSE_LINE_CLEAN_INFO curLine, E_PURPOSE_LINE_CLEAN_INFO shortLine)
{
    CStopWatch __debug_sw;
    bool ret = false;
    tPoint beforeTarget = patternLineData_.beforeTargetPoint;

    //if(!patternLineData_.startLine)
    {
        if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::UP || shortLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
        {
            if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::UP && robotPose.x <= beforeTarget.x)           ret = true;
            else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN && robotPose.x >= beforeTarget.x)    ret = true;
        }
        else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT || shortLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT)
        {
            if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::LEFT && robotPose.y <= beforeTarget.y)         ret = true;
            else if(shortLine == E_PURPOSE_LINE_CLEAN_INFO::RIGHT && robotPose.y >= beforeTarget.y)   ret = true;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, RED,  "isClosedEndLine 라인 에러!!");
        }
    }
    
    if(ret)
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 장애물 회피 공간 폐쇄됨!! tempTarget : " << beforeTarget.x << " , " << beforeTarget.y  << "RobotPose : "<< robotPose.x << "," << robotPose.y << "ShortLine : " << enumToString(shortLine));
    }
    TIME_CHECK_END(__debug_sw.getTime());

    return ret;
}

bool CPatternLine::isRetunCurrentLine(tPose robotPose, E_PURPOSE_LINE_CLEAN_INFO curLine)
{
    CStopWatch __debug_sw;

    bool ret = false;
    tPose startAvoid = getAvoidStartPose();//patternLineData_.AvoidStartPose;
    tPoint target = patternLineData_.TargetPoint;

    if(curLine == E_PURPOSE_LINE_CLEAN_INFO::UP || curLine == E_PURPOSE_LINE_CLEAN_INFO::DOWN)
    {
        if(patternLineData_.checkAvoidingEnd)
        {
            if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
            {
                ceblog(LOG_LV_NECESSARY, GREEN,  "라인복귀 확인 중 목표 Y : " << target.y << " 로봇 Y : " << robotPose.y << "Y축 기준 거리 : " << fabs(target.y-robotPose.y));
            }
            
            if(fabs(target.y-robotPose.y) <= 0.03)
            {
                ret = true;
                patternLineData_.checkAvoidingEnd = false;
                eblog(LOG_LV_NECESSARY, "라인복귀 확인 목표 라인 : " << target.x << " , " << target.y  << "RobotPose : "<< robotPose.x << "," << robotPose.y);
            } 
        }
        else
        {
            double startAvoidDist = fabs(utils::math::distanceTwoPoint(startAvoid,robotPose));
            if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
            {
                ceblog(LOG_LV_NECESSARY, GREEN,  " 복귀 감지를 확인합니다. 초기 장애물 위치 : " << startAvoid.x << " , " << startAvoid.y << "목표 좌표 : " << target.x << " ," << target.y << 
                " 로봇 : " << robotPose.x << " , " << robotPose.y << "이동거리 : " << startAvoidDist );
            }
            
            if(fabs(startAvoidDist) >= 0.3) //&& fabs(target.y-robotPose.y) >= 0.15
            {
                patternLineData_.checkAvoidingEnd = true;
                ceblog(LOG_LV_NECESSARY, GREEN,  " 복귀 감지 시작!! 초기 장애물 위치 : " << startAvoid.x << " , " << startAvoid.y << " 로봇 : " << robotPose.x << " , " << robotPose.y);
            }
        }
    }
    else
    {
        if(patternLineData_.checkAvoidingEnd)
        {
            if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
            {
                ceblog(LOG_LV_NECESSARY, GREEN,  "라인복귀 확인 중 목표 Y : " << target.y << " 로봇 Y : " << robotPose.y << "Y축 기준 거리 : " << fabs(target.y-robotPose.y));
            }
            
            if(fabs(target.x-robotPose.x) <= 0.03)
            {
                ret = true;
                patternLineData_.checkAvoidingEnd = false;
                eblog(LOG_LV_NECESSARY, "라인복귀 확인 RobotPose : "<< robotPose.x << "," << robotPose.y);
            } 
        }
        else
        {
            double startAvoidDist = fabs(utils::math::distanceTwoPoint(startAvoid,robotPose));
            if(SYSTEM_TOOL.getSystemTime()-patternLineData_.debugWallStartTime >= 1)
            {
                ceblog(LOG_LV_NECESSARY, GREEN,  " 복귀 감지를 확인합니다. 초기 장애물 위치 : " << startAvoid.x << " , " << startAvoid.y << "목표 좌표 : " << target.x << " ," << target.y << 
                " 로봇 : " << robotPose.x << " , " << robotPose.y << "이동거리 : " << startAvoidDist );
            }
            
            if(fabs(startAvoidDist) >= 0.3) //&& fabs(target.x-robotPose.x) >= 0.15
            {
                patternLineData_.checkAvoidingEnd = true;
                ceblog(LOG_LV_NECESSARY, GREEN,  " 복귀 감지 시작!! 초기 장애물 위치 : " << startAvoid.x << " , " << startAvoid.y << " 로봇 : " << robotPose.x << " , " << robotPose.y);
            }
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

//hjkim221222 - 라인청소에서 벽면주행 연동 시 벽면 주행 방향 설정
//hjkim230713 - 라인청소 라인 업데이트 중 장애물 감지 시 벽면주행 방향 설정 오류로 인해 임시로 회전 각도 기준으로 처리해놓았다.
//TODO - 주변 장애물 상황에 따라 벽면방향을 설정해줘야 한다

#if 0
void CPatternLine::getNeighbours(int n_array[], int position, int map_width) 
{
    CStopWatch __debug_sw;

	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CLineClean::checkAreaEnd()
{
    bool ret = false;
    switch (patternLineData_.lineInfo.linetype.side)
    {
    case E_PURPOSE_LINE_CLEAN_INFO::UP:
        if(patternLineData_.lineInfo.nextPose.x >= patternLineData_.areaInfo.max.x) ret = true;
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::DOWN:
        if(patternLineData_.lineInfo.nextPose.x <= patternLineData_.areaInfo.min.x) ret = true;
        break; 
    case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:
        if(patternLineData_.lineInfo.nextPose.y <= patternLineData_.areaInfo.min.y) ret = true;
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
        if(patternLineData_.lineInfo.nextPose.y >= patternLineData_.areaInfo.max.y) ret = true;
        break;
    default:
        eblog(LOG_LV_NECESSARY,  "checkAreaEnd!! style error!!");
        break;
    }

    return ret;
}

//hjkim230713 - 라인청소 장애물 감지(범퍼/IR) 벽면주행 연동 시 벽면 주행 방향 설정
E_WALLFACE_ID CLineClean::getWallfaceLineAvoidToWall(bool turnning,tPose robotPose)
{
    E_WALLFACE_ID ret = wf_void;

    tPoint halfPoint = ServiceData.robotMap.getAreaCenterPoint();

    switch (patternLineData_.lineInfo.linetype.side)
    {
    case E_PURPOSE_LINE_CLEAN_INFO::LEFT:
        if(turnning)
        {
            if(patternLineData_.lineInfo.linetype.temp == E_PURPOSE_LINE_CLEAN_INFO::NONE)
            {
                if(patternLineData_.lineInfo.linetype.main == E_PURPOSE_LINE_CLEAN_INFO::UP)              ret = wf_right;
                else                                                            ret = wf_left;
            }
            else if(patternLineData_.lineInfo.linetype.temp == E_PURPOSE_LINE_CLEAN_INFO::UP)      ret = wf_right;//if(robotPose.x  >= halfPoint.x) ret = wf_right;
            else                                                                ret = wf_left;
        }
        else
        {
            if(patternLineData_.lineInfo.linetype.main == E_PURPOSE_LINE_CLEAN_INFO::UP)                  ret = wf_right;
            else                                                                ret = wf_left;
        }
        break;
    case E_PURPOSE_LINE_CLEAN_INFO::RIGHT:
        if(turnning){
            if(patternLineData_.lineInfo.linetype.temp == E_PURPOSE_LINE_CLEAN_INFO::NONE)
            {
                if(patternLineData_.lineInfo.linetype.main == E_PURPOSE_LINE_CLEAN_INFO::UP)              ret = wf_right;
                else                                                            ret = wf_left;
            }
            else if(patternLineData_.lineInfo.linetype.temp == E_PURPOSE_LINE_CLEAN_INFO::UP)      ret = wf_left;//if(robotPose.x  >= halfPoint.x) ret = wf_left;
            else                                                                ret = wf_right;
        }
        else
        {
            if(patternLineData_.lineInfo.linetype.main == E_PURPOSE_LINE_CLEAN_INFO::UP)                  ret = wf_left;
            else                                                                ret = wf_right;
        }
        
        break;    
    default:
        eblog(LOG_LV_NECESSARY,  "getWallfaceLineToWall style error!!");
        break;
    }

    eblog(LOG_LV_NECESSARY, "Line To WallTrack WallFace : " << (int)ret << " turnning : " << (int)turnning <<" RobotPose : " << robotPose.x << "," << robotPose.y << " lineStyle : " << (int)patternLineData_.lineInfo.linetype.side << " lineType : " << (int)patternLineData_.lineInfo.linetype.main << " BeforeLineType : " << (int)patternLineData_.lineInfo.linetype.temp);
    return ret; 
}

/**
 * @brief do line 에 사용하는 end point 가 known인지 판별.
 * 왼쪽 혹은 오른쪽 라인타입일때만 체크, 연속으로 감지안되면 count 0.
 * 
 * @param linetype 
 * @param endPoint 
 * @return true 
 * @return false 
 */
bool CLineClean::checkEndPointUnknown(E_PURPOSE_LINE_CLEAN_INFO linetype, tPoint endPoint)
{
    /* 4. end point 가 unknown 인 경우 */
    if ( linetype == E_PURPOSE_LINE_CLEAN_INFO::LEFT || linetype == E_PURPOSE_LINE_CLEAN_INFO::RIGHT )
    {
        cell endPointCell = ServiceData.robotMap.checkCellValue(tPoint(endPoint.x, endPoint.y));
        if ( endPointCell.b.known != 1 ) // unknown
        {
            patternLineData_.checkEndPointUnknownCount++;
            eblog(LOG_LV_LINECLEAN, "End Point is Unknown!! count: << " << patternLineData_.checkEndPointUnknownCount);
            return true;
        }
        else
        {
            patternLineData_.checkEndPointUnknownCount = 0;
        }   
    }
    return false;
}

s8 CLineClean::checkEndOfLineObstacle(cell_obstacle* pObsMap)
{
    s8 ret = 0;
    //hjkim220614 - 라이다 센서 감지 기준 거리를 셀로 환산한 값
    u8 offset = 3;//LIDAR_OBS_DISTANCE/(CELL_OBS_RESOLUTION*100);
    u16 start_idx = 5047;//((CELL_OBS_HALFHEIGHT-offset)*CELL_OBS_WIDTH)+(CELL_OBS_HALFWIDTH-offset); //cell 검색 범위 시작 시점.
    u16 last_idx = 5053;//((CELL_OBS_HALFHEIGHT-offset)*CELL_OBS_WIDTH)+(CELL_OBS_HALFWIDTH+offset);

    int idx = 0, obsCount = 0;
    const int startX    = 53;
    const int endX      = 58;   // 56 -> 57 -> 58
    const int startY    = 48;
    const int endY      = 52;
    //printf("\n");
    for(int y=startY; y<=endY; y++)
    {
        for(int x=startX; x<endX; x++)
        {
            idx = x+y*100;
            //printf("%d ", pObsMap[idx].b.lidar);
            if(pObsMap[idx].value != 0)
            {
                // if( pObsMap[idx].b.bumper == 1 || pObsMap[idx].b.cliff == 1 )
                // {
                //     ret = 1;
                //     break; 
                // }

                if( pObsMap[idx].b.lidar == 1)
                {
                    obsCount++;
                    if(obsCount >= 5)
                    {
                        obsCount = 0;
                        ret = 1;
                        eblog(LOG_LV_LINECLEAN, "ObstacleMap check idx[ " << idx << " ] Lidar[ " <<  pObsMap[idx].b.lidar << "]");
                        break;    
                    }
                }
            }
        }
        if(ret == 1) break;
        //printf("\n");
    }
    //printf("\n");

    return ret;
}
#endif

//////////////////////// debug function //////////////////////////////////////


/**
 * @brief debug용 state print 함수
 * @param 
 */
void CPatternLine::__debug_state_print__(tPose robotPose)
{
    CStopWatch __debug_sw;

    if ((SYSTEM_TOOL.getSystemTime()-patternLineData_.debug_Time) >= 1)
    {    
        std::string doLineStateStr = enumToString(patternLineData_.step);
        std::cout.precision(2);

        eblog(LOG_LV_NECESSARY, " [do line Step] : "<< doLineStateStr << " Clean Time : "<< int(get_system_time(patternLineData_.lineCleanStartTime)) << 
        " SlamPose : " << robotPose.x << "," << robotPose.y << " sys_angle : " << utils::math::rad2deg(robotPose.angle) << " slam_angle : " << utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle) );
        //eblog(LOG_LV_NECESSARY, "[line clean] AreaInfo min : " << area.min.x << "," << area.min.y << " max : " << area.max.x << "," << area.max.y << 
        eblog(LOG_LV_NECESSARY, "[line clean] TARGET POSE  : " << patternLineData_.TargetPoint.x << " ," << patternLineData_.TargetPoint.y << " START POSE  : " << patternLineData_.startPose.x << " ," << patternLineData_.startPose.y);
        eblog(LOG_LV_NECESSARY, "[line clean] : LineType : " << (int)patternLineData_.curLine << " LineStyle : " << (int)patternLineData_.tempLine);
    
        patternLineData_.debug_Time = SYSTEM_TOOL.getSystemTime();//SYSTEM_TOOL.getSystemTick();
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}
//////////////////////// debug function //////////////////////////////////////

