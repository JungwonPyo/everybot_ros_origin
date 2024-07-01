#include "lineTrack.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"

CLineTrack::CLineTrack()
{
	lineTrackStep =E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION;
}

CLineTrack::~CLineTrack()
{
}

void CLineTrack::initLineTrack(std::list<tPoint> boundary)
{
    lineTrackStep =E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION;

	if(boundaryPoints.size()!=0) boundaryPoints.clear();
    boundaryPoints = boundary;
}

bool CLineTrack::runLineTrack()
{	
    bool ret = false;
    tPose robotPose = ServiceData.localiz.getPose();
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    switch (getLineTrackState())
    {
    case E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION:
        setLineTrackState(procCheckRobotLocation(robotPose));
        break;
    case E_LINE_TRACK_STEP::GO_STRAIGHT:
        setLineTrackState(procGoStraight(robotPose, pObstacle));
        break;
    case E_LINE_TRACK_STEP::GOAL_CHECK_VALID:
        setLineTrackState(procGoalCheckValid(robotPose));
        break;
    case E_LINE_TRACK_STEP::GO_CORNER:
        setLineTrackState(procGoCorner(robotPose));
        break;
    case E_LINE_TRACK_STEP::SET_LINE:
        setLineTrackState(procSetLine(robotPose));
        break;    
    case E_LINE_TRACK_STEP::LINE_HEADING:
        setLineTrackState(procLineHeading(robotPose));
        break;
    case E_LINE_TRACK_STEP::LINE_TRACK:
        setLineTrackState(procLineTrack(robotPose));
        break;
    case E_LINE_TRACK_STEP::CHECK_LINE_TRACK:
        setLineTrackState(procCheckLineTrack(robotPose, pObstacle));
        break;
    case E_LINE_TRACK_STEP::WALL_TRACK:
        setLineTrackState(procWallTrack(robotPose));
        break;
    case E_LINE_TRACK_STEP::ERROR:
        
        ceblog(LOG_LV_LIDAR, BOLDGREEN, "라인 트레이킹 중 에러");
        break;
    case E_LINE_TRACK_STEP::COMPLETE:
        ret = true;
        break;
    default:
        break;
    }

    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procCheckRobotLocation(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION;
    if(checkRobotNearBoundary(boundaryPoints, robotPose))
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "       라인을 세팅 하러 가!      ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************"); 
        startRobotPose =robotPose;
        ret=E_LINE_TRACK_STEP::SET_LINE;
    }
    else
    {
        if(checkRobotInBoundary(boundaryPoints,robotPose))
        {
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ceblog(LOG_LV_NECESSARY, YELLOW, "       경계선까지  직진해!!     ");
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************"); 
            ret=E_LINE_TRACK_STEP::GO_STRAIGHT;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ceblog(LOG_LV_NECESSARY, YELLOW, "       코너로 경로 이동을 해      ");
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************"); 
            

            if (PATH_PLANNER->dofindNearestPath(robotPose, 
                boundaryPoints, boundaryPoints.size())){
                ret=E_LINE_TRACK_STEP::GOAL_CHECK_VALID;
                ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾습니다.");
            }
            else{
                ceblog(LOG_LV_NECESSARY,  RED, "먼저 경로요청이 있었나봐요. 점검해주세요.");
            }
        }
    }
    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procGoStraight(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::GO_STRAIGHT;

    if(avoiding.checkFrontObs(pObstacle))
    {
        
        SUB_TASK.walltracking.initWallTrack();
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "    장애물 만났어! 벽타기 하러 가 ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");    
        ret = E_LINE_TRACK_STEP::WALL_TRACK;
    }
    else
    {
        if(!MOTION.isRunning())
        {
            tAction action;
            action.profile.isStopAtTarget=false;
            action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
            action.linear.targetDistance = 0.5;
            action.needObstacleCheck = false;
            MOTION.actionStart(action);
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ceblog(LOG_LV_NECESSARY, YELLOW, "       로봇 위치 체크 하러 가     ");
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************"); 
        }
        ret=E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION;
    }
    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procGoalCheckValid(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::GOAL_CHECK_VALID;

    if ( PATH_PLANNER->isRunFindPath() == false )
    {
        if (PATH_PLANNER->isFindPath())
        {            
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾았습니다. ");
            SUB_TASK.navi->start( mtpInput(PATH_PLANNER->getPath().back()) );    
            ret = E_LINE_TRACK_STEP::GO_CORNER;
        }
        else
        {   
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로계획을 실패하였습니다.");
            ret = E_LINE_TRACK_STEP::ERROR;
        }
    }    

    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procGoCorner(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::GO_CORNER;
    
    // if (SUB_TASK.drivePath.isArrivalGoal())
    if( SUB_TASK.navi->isComplete() )
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "       로봇 위치 체크 하러 가     ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");  
        ret = E_LINE_TRACK_STEP::CHECK_ROBOT_LOCATION;
    }

    // if (SUB_TASK.drivePath.requestRePath())  // 재경로 생성 요청 확인
    if( SUB_TASK.navi->getTaskState() == subTaskState::fail )
    {
        // SUB_TASK.drivePath.requestRePathClear();
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "    * * * * * 에러 * * * * *    ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");  
        ret = E_LINE_TRACK_STEP::ERROR;
    }
    
    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procSetLine(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::SET_LINE;
    double minDis=999;
    double curDis=0;
    std::list<tPoint> linePoints =boundaryPoints;
    tPoint prePoint = linePoints.front();
    tPoint firstPoint = prePoint;
    linePoints.pop_front();

    for(tPoint linePoint : linePoints)
    {
        if(utils::math::distanceTwoPoint(robotPose,linePoint) > 0.1)
        {
            curDis = distanceFromRobotToLine(prePoint, linePoint, robotPose);
            if(minDis > curDis)
            {
                minDis = curDis;
                targetCornerPoint = linePoint;
                startCornerPoint  = prePoint;
            }
        }
        prePoint=linePoint;
    }

    curDis = distanceFromRobotToLine(prePoint, firstPoint, robotPose);
    if(minDis > curDis)
    {
        minDis = curDis;
        targetCornerPoint = firstPoint;
        startCornerPoint  = prePoint;
    }

    if(minDis != 999)
    {
        
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "        라인에 헤딩 맞추러 가     ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");  
        ret = E_LINE_TRACK_STEP::LINE_HEADING;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "    * * * * * 에러 * * * * *    ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");   
        ret = E_LINE_TRACK_STEP::ERROR;
    }

    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procLineHeading(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::LINE_HEADING;
    tAction action;
    comparePose = robotPose;
    action.type = E_ACTION_TYPE::ROTATE_TO_POINT_ON_MAP;
    action.rotate.targetPoint = targetCornerPoint;
    MOTION.actionStart(action);
    ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
    ceblog(LOG_LV_NECESSARY, YELLOW, "         라인을 체크하러 가       ");
    ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");    
    ret =E_LINE_TRACK_STEP::LINE_TRACK;
    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procLineTrack(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::LINE_TRACK;
    if(!MOTION.isRunning())
    {
        tAction action;
        action.profile.isStopAtTarget=false;
        action.type = E_ACTION_TYPE::LINEAR_TO_LINE_ON_MAP;
        action.linear.startPose  = tPose(startCornerPoint.x, startCornerPoint.y, robotPose.angle);
        action.linear.targetPoint = targetCornerPoint; 
        MOTION.actionStart(action);
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "         라인을 체크하러 가       ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");    
        ret =E_LINE_TRACK_STEP::CHECK_LINE_TRACK;
    }
    return ret;
}

E_LINE_TRACK_STEP CLineTrack::procCheckLineTrack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::CHECK_LINE_TRACK;
    if(checkEndLineTrack(robotPose) == false)
    {
        if(MOTION.isRunning())
        {
            if(avoiding.checkFrontObs(pObstacle))
            {
                
                SUB_TASK.walltracking.initWallTrack();
                ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
                ceblog(LOG_LV_NECESSARY, YELLOW, "  장애물 만났어! 벽타기 하러 가   ");
                ceblog(LOG_LV_NECESSARY, YELLOW, "********************************"); 
                ret = E_LINE_TRACK_STEP::WALL_TRACK;
            }
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ceblog(LOG_LV_NECESSARY, YELLOW, "       라인을 세팅 하러 가!      ");
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ret = E_LINE_TRACK_STEP::SET_LINE;

        }
    }
    else
    {
        
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "     라인트래킹을 완료하였어!     ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ret=E_LINE_TRACK_STEP::COMPLETE;
    }
    return ret;
}



E_LINE_TRACK_STEP CLineTrack::procWallTrack(tPose robotPose)
{
    E_LINE_TRACK_STEP ret = E_LINE_TRACK_STEP::WALL_TRACK;

    if(checkEndLineTrack(robotPose) == false)
    {
        if(checkRobotNearBoundary(boundaryPoints,robotPose))
        {
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
            ceblog(LOG_LV_NECESSARY, YELLOW, "       라인을 세팅 하러 가!      ");
            ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");    
            ret = E_LINE_TRACK_STEP::SET_LINE;
        }
        else
        {
            SUB_TASK.walltracking.runWallTrackPattern();
        }
    }
    else
    {
        
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "    라인트래킹에 시작점에 도착    ");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ret=E_LINE_TRACK_STEP::COMPLETE;
    }
    return ret;
}



// 함수 정의
double CLineTrack::distanceFromRobotToLine(tPoint point1, tPoint point2, tPose robotPose) 
{
    // 직선의 방정식 계수 계산
    double A = point2.y - point1.y;
    double B = point1.x - point2.x;
    double C = (point2.x - point1.x) * point1.y + (point1.y - point2.y) * point1.x;

    // 거리 계산
    double distance = std::fabs(A * robotPose.x + B * robotPose.y + C) / std::sqrt(A * A + B * B);

    return distance;
}

/**
 * @brief point가 area내부에 있는지 외부에 있는지 판단하는 함수
 * jhnoh, 22.11.07
 * @param point 
 * @param roomId 
 * @param areaId 
 * @return true  - 내부 
 * @return false - 외부
 */
bool CLineTrack::checkRobotInBoundary(std::list<tPoint> boundary, tPose robotPose)
{
    bool bRet = false;
    double margin = 0.0;
    std::list<tPoint> polygons = boundary;
    
    tPoint onePoint = polygons.front();
    tPoint twoPoint = polygons.front();
    for(tPoint polygonsPoint : polygons)
    {
        if(polygonsPoint.x < onePoint.x)
        {
            onePoint.x = polygonsPoint.x;
        }
        if(polygonsPoint.y < onePoint.y)
        {
            onePoint.y = polygonsPoint.y;
        }

        if(polygonsPoint.x > twoPoint.x)
        {
            twoPoint.x = polygonsPoint.x;
        }
        if(polygonsPoint.y > twoPoint.y)
        {
            twoPoint.y = polygonsPoint.y;
        }
    }

    if( (onePoint.x < robotPose.x && robotPose.x < twoPoint.x) && (onePoint.y < robotPose.y && robotPose.y < twoPoint.y))
    {
        bRet = true;
    }
    else
    {
        bRet = false;
    }

    return bRet;
}

bool CLineTrack::checkRobotNearBoundary(std::list<tPoint> boundary, tPose robotPose)
{
    bool bRet = false;
    tPoint prePoint;
    if(boundary.size() !=0)
    {
        prePoint=boundary.front();
        boundary.pop_front();
    }
    for(tPoint point : boundary)
    {
        if(distanceFromRobotToLine(prePoint,point, robotPose) < 0.1 && utils::math::distanceTwoPoint(comparePose, robotPose) > 0.2)
        {
            bRet=true;
            break;
        }
        prePoint = point;
    }

    return bRet;
}

bool CLineTrack::checkEndLineTrack(tPose robotPose)
{
    bool ret =false;
    //ceblog(LOG_LV_NECESSARY, YELLOW, " 로봇과 시작점과의 거리 : " << utils::math::distanceTwoPoint(robotPose, startRobotPose) << " , 시작점과 비교점과의 거리 << " << utils::math::distanceTwoPoint(comparePose, startRobotPose));       

    if(utils::math::distanceTwoPoint(robotPose, startRobotPose) < 0.1 && utils::math::distanceTwoPoint(comparePose, startRobotPose) > 0.2)
    {
        ret = true;
    }

    return ret;
}


void CLineTrack::setLineTrackState(E_LINE_TRACK_STEP ret)
{
    lineTrackStep = ret;
}

E_LINE_TRACK_STEP CLineTrack::getLineTrackState()
{
    return lineTrackStep;
}