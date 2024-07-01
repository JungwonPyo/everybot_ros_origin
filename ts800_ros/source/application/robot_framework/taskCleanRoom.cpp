#include "taskCleanRoom.h"

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

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/




const double backDist = -0.025;
const double backMarging = 0.01;

const double turnMargin = DEG2RAD(5);
/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskCleanRoom::CTaskCleanRoom()
{
    CStopWatch __debug_sw;
    
    

    clearLineInfo();
    clearCheckLineInterval();
    setCleanRoomState(CLEAN_ROOM_STATE::NONE);
    setLineState(LINE_STATE::SIDE_MOVE);

    cleanEndFlag.value = 0;
    globalRotate = 0.0;
    lineCleanCnt = 0;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskCleanRoom::~CTaskCleanRoom()
{
    CStopWatch __debug_sw;

    
    setCleanRoomState(CLEAN_ROOM_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskCleanRoom::setCleanRoomState(CLEAN_ROOM_STATE set)
{
    if (set != cleanRoomState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[CLEAN_ROOM_STATE] : "<< enumToString(cleanRoomState)<<" --> "<< enumToString(set) );
    }
    cleanRoomState = set;
}

CLEAN_ROOM_STATE CTaskCleanRoom::getCleanRoomState()
{
    return cleanRoomState;
}

/**
 * @brief 청소
 * 
 * @param robotPose 
 * @return true  : 청소 완료
 * @return false : 청소 중
 */
tCleanEndFlag CTaskCleanRoom::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
   
    
    tCleanEndFlag ret;
    ret.value = 0;

    // 청소 완료 판단.
    if (checkRoomCleaned(robotPose) == false)
    {
        switch (cleanRoomState)
        {
        case CLEAN_ROOM_STATE::NONE :
            MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);        
            break;
        case CLEAN_ROOM_STATE::START_LINE :
            setCleanRoomState(startLine(robotPose));        
            break;
        case CLEAN_ROOM_STATE::RUNNING_MAIN_LINE :
            setCleanRoomState(runningMainLine(robotPose));
            break;
        case CLEAN_ROOM_STATE::RUNNING_MAIN_LINE_AVOID :
            setCleanRoomState(runningMainLineAvoid(robotPose));
            break;
        case CLEAN_ROOM_STATE::RUNNING_SIDE_LINE :
            setCleanRoomState(runningSideLine(robotPose));
            break;
        case CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_AVOID :
            setCleanRoomState(runningSideLineAvoid(robotPose));
            break;        
        case CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_WALLTRACK :
            setCleanRoomState(runningSideLineWallTrack(robotPose));
            break;
        default:
            break;
        }        
    }
    else{
        ret.value = cleanEndFlag.value;
        cleanEndFlag.value = 0;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 
 * 
 * @param robotPose 
 * @param rotate : 지도 환경에 맞추어 회전시켜야 할 양.(RAD)
 * @return CLEAN_ROOM_STATE 
 */
CLEAN_ROOM_STATE CTaskCleanRoom::startLine(tPose robotPose)
{
    CStopWatch __debug_sw;


    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::START_LINE;

    cleanEndFlag.value = 0;
    lineCleanCnt = 0;
    initTargetLine(robotPose);
    startCheckLineEnd();
    clearCheckLineInterval();
    preSideLineSetup(robotPose);
    

    ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;    

    TIME_CHECK_END(__debug_sw.getTime());

    return ret;
}

/**
 * @brief 메인라인 변경전에 체킹.
 * 
 * @param robotPose 
 */
void CTaskCleanRoom::preMainLineSetup(tPose robotPose){
    CStopWatch __debug_sw;
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "메인라인 변경전 점검 시작.");
    // checkLineInterval(robotPose);
    // calcTargetMainLine(robotPose);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "main.target.end 를 향해 출발: "<<lineInfo.main.target.end.x <<" , "<<lineInfo.main.target.end.y);
    MOTION.startLinearAngularPriorToPointOnMap(robotPose, lineInfo.main.target.end, tProfile());    
    startCheckLineEnd();
    setLineState(LINE_STATE::MAIN_MOVE);
    lineCleanCnt++;
    TIME_CHECK_END(__debug_sw.getTime());
}



/**
 * @brief 나의 위치로 부터 일정 거리 만큼의 위치 이동 포인트 획득
 * 
 * @param robotPose 
 * @return tPoint 
 */
tPoint CTaskCleanRoom::findNextSearchingLinePoint(tPose robotPose,double deltaY)
{
    CStopWatch __debug_sw;


    tPoint ret;
    tPose org;
    CRobotKinematics k;
    double deltaX = 0.0;
   
    org = robotPose;
    org.angle = globalRotate;

    ret = k.translate(org, deltaX, deltaY);

    // eblog(LOG_LV_NECESSARY, "다음 청소 검색 위치 : " <<ret.x<<" , "<<ret.y);
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

// 선분을 생성하는 함수
void CTaskCleanRoom::createSegment(tPoint start, double heading, double length,
    tPoint &segmentStart, tPoint &segmentEnd)
{
    // 끝점의 좌표 계산
    double endX = start.x + length * cos(heading);
    double endY = start.y + length * sin(heading);

    double startX = start.x - length * cos(heading);
    double startY = start.y - length * sin(heading);

    // 끝점 좌표로 선분 구성
    segmentStart = {startX, startY};
    segmentEnd = {endX, endY};

    // eblog(LOG_LV_NECESSARY, "segment Start : " <<segmentStart.x<<" , "<<segmentStart.y
    //     <<" , End : " <<segmentEnd.x<<" , "<<segmentEnd.y);
}



// 두 선분이 겹치는지 여부를 확인하는 함수
bool CTaskCleanRoom::checkIntersection(tPoint segmentStart, tPoint segmentEnd, tPoint lineStart, tPoint lineEnd) {
    double x1 = segmentStart.x, y1 = segmentStart.y;
    double x2 = segmentEnd.x, y2 = segmentEnd.y;
    double x3 = lineStart.x, y3 = lineStart.y;
    double x4 = lineEnd.x, y4 = lineEnd.y;

    if (std::max(x1, x2) < std::min(x3, x4) || std::min(x1, x2) > std::max(x3, x4) ||
        std::max(y1, y2) < std::min(y3, y4) || std::min(y1, y2) > std::max(y3, y4)) {
        return false; // 겹치지 않음
    }
    return true; // 겹침
}

// 두 선분의 교차 여부를 확인하고, 교차한다면 교차점을 구하는 함수
bool CTaskCleanRoom::findIntersection(tPoint segmentStart, tPoint segmentEnd, 
    tPoint lineStart, tPoint lineEnd, tPoint& result) {
        CStopWatch __debug_sw;
	std::cout << "------------------------------------"<< std::endl;
    double x1 = segmentStart.x, y1 = segmentStart.y;
    double x2 = segmentEnd.x, y2 = segmentEnd.y;
    double x3 = lineStart.x, y3 = lineStart.y;
    double x4 = lineEnd.x, y4 = lineEnd.y;
    std::cout << "segmentStart : {" << segmentStart.x << ", " << segmentStart.y << "}"
        << " , segmentEnd : {" << segmentEnd.x << ", " << segmentEnd.y << "}" << std::endl;
    std::cout << "lineStart : {" << lineStart.x << ", " << lineStart.y << "}"
        << " , lineEnd : {" << lineEnd.x << ", " << lineEnd.y << "}" << std::endl;
	std::cout <<"x1~4 : "<< x1 << ",  "<< x2 << ",  " << x3 << ",  "<<x4<< std::endl;
	std::cout <<"y1~4 : "<< y1 << ",  "<< y2 << ",  " << y3 << ",  "<<y4<< std::endl;

    double ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
	std::cout <<"ua : "<< ua <<  std::endl;

    if (ua >= 0 && ua <= 1) {
        double x = x1 + ua * (x2 - x1);
        double y = y1 + ua * (y2 - y1);
        result = {x,y};
        std::cout << "lineStart : {" << lineStart.x << ", " << lineStart.y << "}"
        << " , lineEnd : {" << lineEnd.x << ", " << lineEnd.y << "}"
        << " , Intersection : {" << x << ", " << y << "}" << std::endl;
        return true;
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return false;
}


bool getIntersectionPoint(tPoint p1, tPoint p2, tPoint p3, tPoint p4, tPoint& result)
{
    bool bRet = false;
    double d = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    // If d is zero, there is no intersection
    if (d == 0) return false;

    // Get the x and y
    double pre = (p1.x * p2.y - p1.y * p2.x);
    double post = (p3.x * p4.y - p3.y * p4.x);
    double x = (pre * (p3.x - p4.x) - (p1.x - p2.x) * post) / d;
    double y = (pre * (p3.y - p4.y) - (p1.y - p2.y) * post) / d;

    // Set the point of intersection
    result.x = x;
    result.y = y;
    bRet = true;
    return bRet;
}
// 두 점을 이용하여 직선의 방정식의 계수를 계산하는 함수
Line findLine(tPoint p1, tPoint p2) {
    Line line;
    line.a = p2.y - p1.y;  // b * (y2 - y1)
    line.b = p1.x - p2.x;  // -a * (x2 - x1)
    line.c = p2.x * p1.y - p1.x * p2.y;  // -(x2*y1 - x1*y2)
    return line;
}

// 두 직선의 교점을 찾는 함수
bool CTaskCleanRoom::findIntersection(Line l1, Line l2, tPoint &intersection) {
    // 두 직선의 방정식의 결정자 계산
    double det = l1.a * l2.b - l2.a * l1.b;
    if (det == 0) {  // 결정자가 0이면, 직선은 평행하거나 일치
        return false;  // 교점이 없음
    }

    // Cramer's rule을 사용하여 교점 계산
    intersection.x = (l2.b * -l1.c - l1.b * -l2.c) / det;
    intersection.y = (l1.a * -l2.c - l2.a * -l1.c) / det;
    return true;
}
// 점 C가 선분 AB 내에 있는지 외부에 있는지 판단하는 함수
bool isPointInsideSegment(tPoint A, tPoint B, tPoint C) {
    // 점 C가 선분 AB 위에 있는지 확인 (A, B, C가 일직선상에 있는지는 가정)
   double epsilon = 1e-5;
    
    if ((std::min(A.x, B.x) - epsilon <= C.x && C.x <= std::max(A.x, B.x) + epsilon) &&
        (std::min(A.y, B.y) - epsilon <= C.y && C.y <= std::max(A.y, B.y) + epsilon)) {
        return true; // C는 선분 AB 내부에 위치
    }
    return false; // C는 선분 AB 외부에 위치
}

// 모든 영역과의 교차 검사
std::list<tPoint> CTaskCleanRoom::getIntersections(tPoint org, tPoint segmentStart, 
    tPoint segmentEnd, const std::list<tPoint>& polygon) {

    std::list<std::pair<double, tPoint>> tempIntersections;
    tPoint intersection, areaStart, areaEnd;

    if (!polygon.empty()) {
        for (auto it = polygon.begin(); it != polygon.end(); ++it) {
            auto next_it = (std::next(it) != polygon.end()) ? std::next(it) : polygon.begin();

            areaStart = *it;
            areaEnd = *next_it;

            Line l1 = findLine(segmentStart, segmentEnd);
            Line l2 = findLine(areaStart, areaEnd);

            if (findIntersection(l1, l2, intersection)) {
                if (isPointInsideSegment(segmentStart, segmentEnd, intersection) &&
                    isPointInsideSegment(areaStart, areaEnd, intersection)) {
                    double dist = tPoint::distance(org, intersection);
                    tempIntersections.push_back({dist, intersection});
                }
            }
        }
    }

    // 거리에 따라 정렬하고 가장 가까운 두 교차점만 유지
    tempIntersections.sort([](const std::pair<double, tPoint>& a, const std::pair<double, tPoint>& b) {
        return a.first < b.first;
    });
    std::list<tPoint> closestIntersections;
    // std::cout << "closestIntersections : ";
    auto temp_it = tempIntersections.begin();
    for (int i = 0; i < std::min(2, static_cast<int>(tempIntersections.size())) && temp_it != tempIntersections.end(); ++i, ++temp_it) {
        closestIntersections.push_back(temp_it->second);

        // std::cout << " , (" << temp_it->second.x << ", " << temp_it->second.y << ")";
    }
    // std::cout<< std::endl;

    return closestIntersections;
}

/**
 * @brief 
 * 
 * @param start 
 * @param end 
 */
bool CTaskCleanRoom::findUncleanLine(tPoint start, tPoint end, tPoint& uncleandPoint){
    bool isFind = false;
    const double interval = 0.05; // 간격을 5cm로 설정
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double length = sqrt(dx * dx + dy * dy);
    double unitX = dx / length;
    double unitY = dy / length;

    std::vector<tPoint> points;
    double currentDistance = 0.0;

    bool nogozonecheck = SUB_TASK.cleanPlan.isSetupNogoZoneDocking();

    std::list<tPoint> noGoZone = SUB_TASK.cleanPlan.getNoGoZoneDocking();
    std::list<tPoint> shrinkNoGoZone;

    utils::area::resizeArea(noGoZone, shrinkNoGoZone, 20);

    

    while (currentDistance < length) {
        tPoint point = {
            start.x + unitX * currentDistance,
            start.y + unitY * currentDistance
        };
        if (nogozonecheck){
            bool isNogoZone = utils::area::isInside(point, shrinkNoGoZone);
            if(isNogoZone == false){
                points.push_back(point);
            }else{
            // std::cout << "도킹지역 탈락 : (" << point.x << ", " << point.y << ")" << std::endl; 
            }
        }
        else{
            points.push_back(point);
        }
        
        
        currentDistance += interval;
        // std::cout << "searchpt : (" << point.x << ", " << point.y << ")" << std::endl;
    }

    // 끝점 추가 - 정확한 끝점을 포함하기 위해
    points.push_back(end);


    bool bCleanAndWll = false;
    for (const auto& point : points) {
        // std::cout << "search Point: (" << point.x << ", " << point.y << ")" << std::endl;
        bCleanAndWll = 
            ServiceData.robotMap.cleanMap.checkWallAndCleanBound(point);
        if (bCleanAndWll == false){
            isFind = true;
            uncleandPoint = point;
            break;
        }        
    }

    return isFind;
}



/**
 * @brief 다음 청소라인 시작점과 클린헤딩 을 이용하여 
 * 영역안 중 청소 안한 로봇과 가장 가까운 위치 찾기
 * 
 * @param robotPose 
 * @param searchingPt 
 * @param cleanHeading 
 * @return true 
 * @return false 
 */
bool CTaskCleanRoom::findMainLineStartPoint(tPose robotPose, 
    const std::list<tPoint>& area, tPoint searchingPt, 
    double cleanHeading, tPoint &uncleanPoint){
    CStopWatch __debug_sw;
    bool bRet = false;

    // searchingPt 와 cleanHeading 을 이용하여 영역안 시작점괌 끝점 찾기 
    tPoint segmentStart;
    tPoint segmentEnd;
    createSegment(searchingPt, cleanHeading, 50, segmentStart, segmentEnd);
    std::list<tPoint> line = getIntersections(searchingPt, segmentStart, segmentEnd, area); 
        
    if (line.size() >= 2){
        // 그 라인중 청소 안한 라인 찾기
        tPoint st = line.front();
        tPoint ed = line.back();
        if ( findUncleanLine(st, ed, uncleanPoint) ){            
            eblog(LOG_LV_NECESSARY, "catch - uncleandPoint: (" << uncleanPoint.x << ", " << uncleanPoint.y << ")");
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "searchingPt : "<<searchingPt.x<<" , "<<searchingPt.y);
            bRet = true;
            calcTargetMainLineV2(robotPose, st, ed);
        }
        else{
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "청소 했거나 벽으로 막혀 있음. 검사 위치 : "
                <<searchingPt.x<<" , "<<searchingPt.y);
        }
    }
    else{
        bRet = false;
    }
    TIME_CHECK_END(__debug_sw.getTime());
        
    return bRet;
}

double calculateProjection(const tPoint& point, double heading) {
    return point.x * cos(heading) + point.y * sin(heading);
}

bool CTaskCleanRoom::computeNextMainLinePoint(tPose robotPose, tPoint &uncleanPoint, 
    double cleanHeading){
    
    std::list<tPoint> shrink;
    std::list<tPoint> area = SUB_TASK.cleanPlan.getCurrentAreaPolygons();
    utils::area::resizeArea(area, shrink, -10);

    // Calculate projections and find the extremes based on the robot's heading
    double minProj = std::numeric_limits<double>::infinity();
    double maxProj = -std::numeric_limits<double>::infinity();
    for (const auto& pt : shrink) {
        double proj = calculateProjection(pt, cleanHeading);
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
    }

    tPose setPose= {robotPose.x, robotPose.y, cleanHeading};
    std::list<tPoint> searchPointList;
    CRobotKinematics k;
    double distance = CONFIG.lineInterval;
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, 
        "maxProj - minProj : "<<maxProj - minProj<<
        " , minProj : "<<minProj<<
        " , maxProj : "<<maxProj);
    tPoint centerPt = k.translate(setPose, 0, 0 );
    searchPointList.push_back(centerPt);
    while (distance <= (maxProj - minProj)) {
        tPoint rightPt = k.translate(setPose, 0, distance );
        searchPointList.push_back(rightPt);
        
        tPoint leftPt = k.translate(setPose, 0, -distance );
        searchPointList.push_back(leftPt);
        
        distance += 0.1;
    }

    bool bFind = false;
    for (auto pt : searchPointList){
        bFind = findMainLineStartPoint(robotPose, shrink, pt, cleanHeading, uncleanPoint);
        if (bFind) {
            break;
        }
    }

    return bFind;
}


bool CTaskCleanRoom::preSideLineSetup(tPose robotPose){
    CStopWatch __debug_sw;
    ceblog(LOG_LV_NECESSARY, MAGENTA, "사이드라인 변경전 점검 시작.");
    eblog(LOG_LV_NECESSARY, "robotPose : "
        <<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "globalRotate : "<<globalRotate <<" rad, "<<RAD2DEG(globalRotate)<<" deg");

    bool bRet = true;
    

    //다음 이동점 획득
    tPoint uncleanPoint;
    bool bFind = computeNextMainLinePoint(robotPose, uncleanPoint, globalRotate);    

    if(bFind){
        // target side line 정리. 
        calcTargetSideLine(robotPose, uncleanPoint);
#if 1 // 경로이동으로 보내기
        moveLine.taskStart(lineInfo.side.target.end);
#endif

#if 0 //포인트 이동으로 보내기
        MOTION.startLinearAngularPriorToPointOnMap(
            robotPose, lineInfo.side.target.end, tProfile());
#endif       
        
    }
    else{
        eblog(LOG_LV_NECESSARY, "청소 할 곳을 못 찾아 종료");
        // 이번 패턴 종료
        cleanEndFlag.b.sideMoveError = true;
    }

    setLineState(LINE_STATE::SIDE_MOVE);

    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}


CLEAN_ROOM_STATE CTaskCleanRoom::runningSideLineWallTrack(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_WALLTRACK;
    double result = 0;
    double result2 = 0;
    double walltrackRunTime = 0;
    
    double wallAngle = SUB_TASK.walltracking.getAccumulateAngle(robotPose);
    walltrackRunTime = SYSTEM_TOOL.getSystemTime() - walltrackStartTime;

    double dist = utils::math::distanceTwoPoint(robotPose.convertPoint(),
         lineInfo.side.target.start);

    if(fabs(dist) >= (CONFIG.lineInterval) )
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료,(거리) : "<<dist);

        //회피가 완료가 되면 다시 업데이트 시작
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        MOTION.startStopOnMap(tProfile(),false);
        
        
        //MOTION.startLinearAngularPriorToPointOnMap(
        //    robotPose, lineInfo.side.target.end, tProfile());
        
        bool bCoverage = SUB_TASK.cleanPlan.checkAreaCoverage();
        cleanEndFlag.b.coverageLimit = bCoverage;
        bool bSuccess = preSideLineSetup(robotPose);
        if (bSuccess && bCoverage == false){
            ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
        }
        else{
            ret = CLEAN_ROOM_STATE::NONE;
        } 
    }
    else if(wallAngle >= 360){
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 종료 - 회전 :");
        //회피가 완료가 되면 다시 업데이트 시작
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
        MOTION.startStopOnMap(tProfile(),false);
        
        
        bool bCoverage = SUB_TASK.cleanPlan.checkAreaCoverage();
        cleanEndFlag.b.coverageLimit = bCoverage;
        bool bSuccess = preSideLineSetup(robotPose);
        if (bSuccess && bCoverage == false){
            ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
        }
        else{
            ret = CLEAN_ROOM_STATE::NONE;
        } 
    }
    else{
        SUB_TASK.walltracking.procWalltracking(robotPose, direction);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskCleanRoom::taskStart(tPose robotPose, double rotate)
{
    PATH_PLANNER->startMapUpdate();
    globalRotate = rotate;
    setCleanRoomState(startLine(robotPose));
}

void CTaskCleanRoom::calcTargetMainLine(tPose robotPose)
{
    tPoint org;
    CRobotKinematics k;
    lineInfo.main.pre = lineInfo.main.target;

    //y축 증가량 계산
    double deltaY = pointToLineDistance(lineInfo.main.pre.start, 
        lineInfo.main.pre.end, robotPose);
    deltaY += CONFIG.lineSideIntervalK;   

    org = lineInfo.main.pre.end;
    lineInfo.main.target.start = k.translate(org, 0, deltaY);

    org = lineInfo.main.pre.start;
    lineInfo.main.target.end = k.translate(org, 0, deltaY);

    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "deltaY : "<< deltaY);
    
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "target "
        <<" start x,y: "<< lineInfo.main.target.start.x <<" , " 
        << lineInfo.main.target.start.y
        <<" ,end x,y : "<< lineInfo.main.target.end.x <<" , "
        << lineInfo.main.target.end.y);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "pre "
        <<" start x,y: "<< lineInfo.main.pre.start.x <<" , " 
        << lineInfo.main.pre.start.y
        <<" ,end x,y : "<< lineInfo.main.pre.end.x <<" , "
        << lineInfo.main.pre.end.y);
}


// 두 점 사이의 거리를 계산하는 함수
double distance(tPoint p1, tPoint p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// 두 점을 연결하고 특정 거리만큼 이동한 세 번째 점을 계산하는 함수
tPoint extendLine(tPoint start, tPoint through, double length) {
    // 두 점 사이의 벡터를 계산
    double dx = through.x - start.x;
    double dy = through.y - start.y;

    // 벡터의 길이(거리)를 계산
    double dist = distance(start, through);

    // 벡터를 단위 벡터로 변환
    double ux = dx / dist;
    double uy = dy / dist;

    // 단위 벡터에 길이를 곱하여 세 번째 점 계산
    tPoint thirdPoint;
    thirdPoint.x = start.x + ux * length;
    thirdPoint.y = start.y + uy * length;

    return thirdPoint;
}

void CTaskCleanRoom::calcTargetMainLineV2(tPose robotPose, tPoint start, tPoint end)
{
    tPoint org;
    CRobotKinematics k;
    lineInfo.main.pre = lineInfo.main.target;

    lineInfo.main.target.start = start;
    lineInfo.main.target.end = end;

    // lineInfo.main.target.end = extendLine(start, end, 4);

    ceblog(LOG_LV_NECESSARY, MAGENTA, "robotPose : "
        <<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);    
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "target "
        <<" start x,y: "<< lineInfo.main.target.start.x <<" , " 
        << lineInfo.main.target.start.y
        <<" ,end x,y : "<< lineInfo.main.target.end.x <<" , "
        << lineInfo.main.target.end.y);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "pre "
        <<" start x,y: "<< lineInfo.main.pre.start.x <<" , " 
        << lineInfo.main.pre.start.y
        <<" ,end x,y : "<< lineInfo.main.pre.end.x <<" , "
        << lineInfo.main.pre.end.y);
}


void CTaskCleanRoom::calcTargetSideLine(tPose robotPose, tPoint target)
{
    tPose org;
    CRobotKinematics k;
    double deltaX = 0.0;
    double deltaY = CONFIG.lineInterval;


    lineInfo.side.pre = lineInfo.side.target;
    lineInfo.side.target.start = robotPose.convertPoint();

    lineInfo.side.target.end = target;
    
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "target "
        <<" start x,y: "<< lineInfo.side.target.start.x <<" , " 
        << lineInfo.side.target.start.y
        <<" ,end x,y : "<< lineInfo.side.target.end.x <<" , "
        << lineInfo.side.target.end.y);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "pre "
        <<" start x,y: "<< lineInfo.side.pre.start.x <<" , " 
        << lineInfo.side.pre.start.y
        <<" ,end x,y : "<< lineInfo.side.pre.end.x <<" , "
        << lineInfo.side.pre.end.y);
}

void CTaskCleanRoom::initTargetLine(tPose robotPose)
{
    CRobotKinematics k;
    clearLineInfo();

    tPose initPose = robotPose;
    // initPose.angle = k.rotation(robotPose, globalRotate);
    initPose.angle = globalRotate;

    // 로보락과같이 8m 에어리어 기준.
    lineInfo.main.target.start =  
        k.translate(initPose, CONFIG.lineMaxDist * -1, 0.0);
    lineInfo.main.target.end =  
        k.translate(initPose, CONFIG.lineMaxDist, 0.0);
    
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "lineInfo.main "
        <<" start x,y: "<< lineInfo.main.target.start.x <<" , " 
        << lineInfo.main.target.start.y
        <<" ,end x,y : "<< lineInfo.main.target.end.x <<" , "
        << lineInfo.main.target.end.y);
}

void CTaskCleanRoom::clearLineInfo()
{    
    lineInfo.main.target.start.x = 0.0;
    lineInfo.main.target.start.y = 0.0;
    lineInfo.main.pre.start.x = 0.0;
    lineInfo.main.pre.start.y = 0.0;

    lineInfo.main.target.end.x = 0.0;
    lineInfo.main.target.end.y = 0.0;
    lineInfo.main.pre.end.x = 0.0;
    lineInfo.main.pre.end.y = 0.0;

    lineInfo.side.target.start.x = 0.0;
    lineInfo.side.target.start.y = 0.0;
    lineInfo.side.pre.start.x = 0.0;
    lineInfo.side.pre.start.y = 0.0;

    lineInfo.side.target.end.x = 0.0;
    lineInfo.side.target.end.y = 0.0;
    lineInfo.side.pre.end.x = 0.0;
    lineInfo.side.pre.end.y = 0.0;
}


/**
 * @brief 벽타기 방향 결정
 * 
 * @param robotPose 
 */
void CTaskCleanRoom::computeWalltrackDir(tPose robotPose, tPoint target)
{
    //RSF_OBSTACLE_MASK mask = avoiding.getAvoidMask();
    // if(mask.value & 0x0F)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 장애물 감지!! 오른쪽 벽타기로 결정");
    //     direction = E_WALLTRACK_DIR::RIGHT;
    // }
    // else
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 장애물 감지!! 왼쪽 벽타기로 결정");
    //     direction = E_WALLTRACK_DIR::LEFT;
    // }
    CRobotKinematics k;
    bool bRet = k.computeRotateDir(robotPose, target);
    
    if (bRet) {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 벽타기로 결정");
        direction = E_WALLTRACK_DIR::RIGHT;
    } else {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 벽타기로 결정");
        direction = E_WALLTRACK_DIR::LEFT;
    }
}

tPoint CTaskCleanRoom::getNextPosition(const tPose& robotPose, 
    const tPoint& target, double stepSize) {
    // 목표까지의 벡터 계산
    double dx = target.x - robotPose.x;
    double dy = target.y - robotPose.y;

    // 벡터의 길이 계산
    double distance = sqrt(dx*dx + dy*dy);

    // 단위 벡터 계산
    double unitX = dx / distance;
    double unitY = dy / distance;

    // 다음 위치 계산
    tPoint nextPosition;
    nextPosition.x = robotPose.x + unitX * stepSize;
    nextPosition.y = robotPose.y + unitY * stepSize;

    return nextPosition;
}



bool CTaskCleanRoom::checkNextCell(cell &nextCell, tPose robotPose)
{
    bool bRet = false;
    double step = 0.3;
    tPoint nextPoint;

    if (getLineState() == LINE_STATE::MAIN_MOVE ){        
        nextPoint = getNextPosition(robotPose, lineInfo.main.target.end, step);

        nextCell = ServiceData.robotMap.cleanMap.checkCellValue(nextPoint);
        bRet = true;
    }

    return bRet;
}


unsigned int CTaskCleanRoom::getLineCleanCnt()
{
    return lineCleanCnt;
}

CLEAN_ROOM_STATE CTaskCleanRoom::runningMainLine(tPose robotPose)
{
    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::RUNNING_MAIN_LINE;
    CStopWatch __debug_sw;
    
    bool bNear = MOTION.isNearTargetPose(robotPose, lineInfo.main.target.end, CONFIG.mainlineGoalMargin);
    bool isOverCross = utils::math::isRobotCrossLine(robotPose, lineInfo.main.target.start, lineInfo.main.target.end);
    // bool bOver = false;//MOTION.isOverTargetPoint(robotPose, lineInfo.main.target.start, lineInfo.main.target.end);
    bool bEndLine = checkLineEnd();

    if(bNear || bEndLine || isOverCross)
    {
        ceblog(LOG_LV_NECESSARY, MAGENTA, "메인라인 도착 bNear: "<<bNear<<
            " , bEndlineFlag : " <<bEndLine
            <<" , robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
        
        MOTION.startStopOnMap(tProfile(),false);
        
        
        bool bCoverage = SUB_TASK.cleanPlan.checkAreaCoverage();
        cleanEndFlag.b.coverageLimit = bCoverage;
        bool bSuccess = preSideLineSetup(robotPose);
        if (bSuccess && bCoverage == false){
            ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
        }
        else{
            ret = CLEAN_ROOM_STATE::NONE;
        }
        
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        ceblog(LOG_LV_NECESSARY, MAGENTA, "장애물 감지 하여 옆으로 이동 계획");
        ceblog(LOG_LV_NECESSARY, MAGENTA, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
        MOTION.startStopOnMap(tProfile(),false);
        
        avoidState = MAIN_LINE_AVOID_STATE::BACK_START;
        
        walltrackStartTime = SYSTEM_TOOL.getSystemTime();
        ret = CLEAN_ROOM_STATE::RUNNING_MAIN_LINE_AVOID;
    }
    else
    {
        
    }
    
    TIME_CHECK_END(__debug_sw.getTime());

    return ret;
}

/**
 * @brief 청소 완료 판단
 * 
 * @param robotPose 
 * @return true 
 * @return false 
 */
bool CTaskCleanRoom::checkRoomCleaned(tPose robotPose)
{
    bool bRet = false;

    //구속된거 같아요.
    if(cleanEndFlag.b.lineInterval)
    {
        ceblog(LOG_LV_NECESSARY, MAGENTA, "구속된거 같아 라인 청소 종료");
        bRet = true;
    }

    //회전 구속
    if(cleanEndFlag.b.rotate)
    {
        ceblog(LOG_LV_NECESSARY, MAGENTA, "회전 구속된거 같아 라인 청소 종료");
        bRet = true;
    }

    if (cleanEndFlag.b.coverageLimit){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "커버리지 종료 처리");
        bRet = true;
    }

    if (cleanEndFlag.b.sideMoveError){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "사이드라인 이동 할 곳 없음");
        bRet = true;
    }

    if (cleanEndFlag.b.dockingzone){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "충전기 영역이어서 종료");
        bRet = true;
    }


    return bRet;
}

void CTaskCleanRoom::setEndFlagCleaned(bool bSet)
{
    cleanEndFlag.b.coverageLimit = bSet;
}

LINE_STATE CTaskCleanRoom::getLineState()
{
    return lineState;
}

void CTaskCleanRoom::setEndLineTrig(bool set)
{
    cleanEndFlag.b.endline = set;
}


void CTaskCleanRoom::setDockingZoneTrig(bool set)
{
    cleanEndFlag.b.dockingzone = set;
}

void CTaskCleanRoom::setLineState(LINE_STATE set)
{
    if (lineState != set)
    {
        ceblog(LOG_LV_NECESSARY, GREEN, "라인 변경 됨 : " 
            << enumToString(lineState) << " -> " << enumToString(set));
    }
    lineState = set;
}


CLEAN_ROOM_STATE CTaskCleanRoom::runningMainLineAvoid(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::RUNNING_MAIN_LINE_AVOID;
    CRobotKinematics k;
    switch (avoidState)
    {
    case MAIN_LINE_AVOID_STATE::BACK_START :
        targetStart.x = robotPose.x;
        targetStart.y = robotPose.y;
        targetEnd = k.translate(robotPose, backDist, 0.0);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지에 의한 뒤로가기 시작");
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
        MOTION.startBackToPointOnMap(robotPose, targetEnd, tProfile());
        
        avoidState = MAIN_LINE_AVOID_STATE::BACK_WAIT;
        break;
    case MAIN_LINE_AVOID_STATE::BACK_WAIT :
        if(MOTION.isNearTargetPose(robotPose, targetEnd, backMarging) || 
            MOTION.isOverTargetPoint(robotPose, targetStart, targetEnd))
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 후진 도착");

            bool bCoverage = SUB_TASK.cleanPlan.checkAreaCoverage();
            cleanEndFlag.b.coverageLimit = bCoverage;
            bool bSuccess = preSideLineSetup(robotPose);
            if (bSuccess && bCoverage == false){
                ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
            }
            else{
                ret = CLEAN_ROOM_STATE::NONE;
            }
            avoidState = MAIN_LINE_AVOID_STATE::BACK_START;    
        }
        
        break;
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


CLEAN_ROOM_STATE CTaskCleanRoom::runningSideLine(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
    

    bool bNear = MOTION.isNearTargetPose(robotPose, lineInfo.side.target.end, CONFIG.sidelineGoalMargin);
    bool isOverCross = utils::math::isRobotCrossLine(robotPose, lineInfo.side.target.start, lineInfo.side.target.end);

    if (moveLine.taskRun(robotPose)){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "목적지 이동에의한 사이드 도착");
        preMainLineSetup(robotPose);
        ret = CLEAN_ROOM_STATE::RUNNING_MAIN_LINE;
    }
    else if(bNear || isOverCross)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "사이드 도착 goal margin : "<<CONFIG.sidelineGoalMargin
            <<" , robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);                

        preMainLineSetup(robotPose);
        ret = CLEAN_ROOM_STATE::RUNNING_MAIN_LINE;
        
    }
    else if(avoiding.checkObstacle(robotPose,true,false))
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "사이드라인 이동 중 장애물 발견" );
        eblog(LOG_LV_NECESSARY, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);
        MOTION.startStopOnMap(tProfile(),false);
        

        //청소 중 장애물 회피를 사용하려면 bWall = true
        //신뢰성 테스트 구간에서 일단 확인 시 기존 방식에 비해서 청소 완료가 되는 것은 확인은
        //했으나 추가적인 개선이 필요해서 일단 기존 방식을 유지 시킴.
        bool bWall = false;

        //for_test
        if (bWall){
            //wall follow 로 사이드처리

            //라인 청소 중 너무 밀접한 장애물이 있을 경우 전진/후진 반복이 너무 많음 -> 지도 업데이트를 좀 하지 맣자.
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);

            tPoint target = lineInfo.side.target.end;

            computeWalltrackDir(robotPose, target);
            
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "벽타기 시작"
                <<enumToString(direction) );
                        
            walltrackStartTime = SYSTEM_TOOL.getSystemTime();
            SUB_TASK.walltracking.startAccumulateAngle(robotPose);
            ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_WALLTRACK;            
        }
        else{
            // 회피로 사이드 처리
            avoidSideState = SIDE_LINE_AVOID_STATE::BACK_START;
            ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_AVOID;
        }        
    }    
    else{
        // 
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

//for_test
CLEAN_ROOM_STATE CTaskCleanRoom::runningSideLineAvoid(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEAN_ROOM_STATE ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_AVOID;
    CRobotKinematics k;
    switch (avoidSideState)
    {
    case SIDE_LINE_AVOID_STATE::BACK_START :
        //라인 청소 중 너무 밀접한 장애물이 있을 경우 전진/후진 반복이 너무 많음 -> 지도 업데이트를 좀 하지 맣자.
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        targetStart.x = robotPose.x;
        targetStart.y = robotPose.y;
        targetEnd = k.translate(robotPose, backDist, 0.0);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "장애물 감지에 의한 뒤로가기 시작");
        MOTION.startBackToPointOnMap(robotPose, targetEnd, tProfile());
        avoidSideState = SIDE_LINE_AVOID_STATE::BACK_WAIT;
        break;
    case SIDE_LINE_AVOID_STATE::BACK_WAIT :
        if(MOTION.isNearTargetPose(robotPose, targetEnd, backMarging) || 
            MOTION.isOverTargetPoint(robotPose, targetStart, targetEnd))
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "회피 후진 도착");
            //후진 회피가 완료가 되면 다시 업데이트 시작
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);

            bool bCoverage = SUB_TASK.cleanPlan.checkAreaCoverage();
            cleanEndFlag.b.coverageLimit = bCoverage;
            bool bSuccess = preSideLineSetup(robotPose);
            if (bSuccess && bCoverage == false){
                ret = CLEAN_ROOM_STATE::RUNNING_SIDE_LINE;
            }
            else{
                ret = CLEAN_ROOM_STATE::NONE;
            } 
            avoidSideState = SIDE_LINE_AVOID_STATE::BACK_START;
        }
        
        break;
    case SIDE_LINE_AVOID_STATE::TRUN_START :      
        break;
    case SIDE_LINE_AVOID_STATE::TRUN_WAIT :        
        break;
    default:
        break;
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


void CTaskCleanRoom::clearCheckLineInterval()
{
    cleanEndFlag.b.lineInterval = false;
}

/**
 * @brief //side line 을 이동하는데 계속 같은곳에 머문다면 구속된거다
 * 
 */
void CTaskCleanRoom::checkLineInterval(tPose robotPose)
{    
    ceblog(LOG_LV_NECESSARY, CYN, "사이드 라인 이동거리 검사합니다.");
    double dist = pointToLineDistance(lineInfo.main.target.start, 
        lineInfo.main.target.end, robotPose);
    if (dist < 0.05 )
    {
        ceblog(LOG_LV_NECESSARY, CYN, "사이드라인 이동 거리 오류 : " << dist );
        cleanEndFlag.b.lineInterval = true;
    }
}

void CTaskCleanRoom::startCheckLineEnd(){
    
    if (endLineFlag.trig){
        endLineFlag.preState = true;
    }
    else{
        endLineFlag.preState = false;
    }

    endLineFlag.fallingCnt = 0;
}

/**
 * @brief 메인라인 시작전에 체크 하여 시잘할지 말지 판단 할수 있게 한다.
 * 
 */
bool CTaskCleanRoom::checkLineEnd()
{
    bool bRet = false;
    if (endLineFlag.preState && endLineFlag.trig == false){
        endLineFlag.fallingCnt ++;
    }

    // 범위 안에 확실히 들어왔는지 체크
    if (endLineFlag.fallingCnt  > 10){
        endLineFlag.trig = false;
    }

    if (endLineFlag.trig && endLineFlag.preState == false){
        bRet = true;
    }

    return bRet;
}


// 선분에서 점까지의 최소 거리(수직 이격)를 계산하는 함수
double CTaskCleanRoom::pointToLineDistance(const tPoint& start, 
    const tPoint& end, const tPose& robotPose) 
{
    double lineLength = utils::math::distanceTwoPoint(start, end);  // 선분의 길이
    if (lineLength == 0) 
        return utils::math::distanceTwoPoint(start, robotPose);  // 선분의 시작과 끝이 같은 경우

    // 선분에 대한 벡터 계산
    double dx = end.x - start.x;
    double dy = end.y - start.y;

    // 점과 선분의 시작점에 대한 벡터 계산
    double t = ((robotPose.x - start.x) * dx + (robotPose.y - start.y) * dy) / 
                (lineLength * lineLength);

    // 가장 가까운 점이 선분 상에 위치하는지 확인
    t = std::max(0.0, std::min(1.0, t));
    double nearestX = start.x + t * dx;
    double nearestY = start.y + t * dy;

    // 가장 가까운 점과 주어진 점 사이의 거리를 계산
    return utils::math::distanceTwoPoint(robotPose, {nearestX, nearestY});
}