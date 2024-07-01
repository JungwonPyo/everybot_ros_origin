#include "taskClean.h"

#include "utils.h"
#include "eblog.h"
#include "waveFrontier.h"  
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "debugCtr.h"
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
CTaskClean::CTaskClean()
{
    CStopWatch __debug_sw;    
    
    bSkipWallClean = false;
    // bSkipWallClean = true;
    planRemakeTry = 0;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskClean::~CTaskClean()
{
    CStopWatch __debug_sw;

    
    setState(CLEANTASK_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskClean::setState(CLEANTASK_STATE set)
{
    if (set != cleanLineState)
    {
        preCleanLineState = cleanLineState; // 이전 상태 저장.
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[CLEANTASK_STATE] : "<< enumToString(cleanLineState)<<" --> "<< enumToString(set) );
    }
    cleanLineState = set;
}

CLEANTASK_STATE CTaskClean::planInit(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::PLAN_INIT;
    bool bMakeMap = false;
    ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "청소 시작~~!! 청소 영역을 확인합니다.");
    if( ROBOT_CONTROL.slam.isExistedSlamMap() )
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE,"청소영역 준비: 지도를 불러와 청소영역을 생성합니다");
        bMakeMap = SUB_TASK.cleanPlan.makeAreaByMap();       
    }
    else
    {
        SUB_TASK.cleanPlan.makeCustomArea(robotPose);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, 
            "청소영역 준비: 지도가 없어 임시 청소영역을 생성합니다");
        bMakeMap = true;
    }

    if(bMakeMap){
        rePortCleanAreaInfo();
        debugAwsForbiddenArea();
        if(SUB_TASK.cleanPlan.getRooms().empty())
        {
            ret = CLEANTASK_STATE::FINISH;
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, 
                "청소영역 준비: 청소할 영역이 없어 청소를 종료합니다.");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소 계획 시작!!");
            ret = CLEANTASK_STATE::PLAN_MAKE;
        }    
    }

    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

CLEANTASK_STATE CTaskClean::planMake(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::PLAN_MAKE;

    int roomSize = SUB_TASK.cleanPlan.getRooms().size();
    int cleanedRoomSize = SUB_TASK.cleanPlan.getCleanedRoomSize();
    int uncleanedRoomSize = SUB_TASK.cleanPlan.getUnCleanRoomSize();
    
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소를 계획합니다. " <<
        "전체 방 : " << roomSize <<
        " , cleaned "<< cleanedRoomSize <<
        " , uncleaned "<< uncleanedRoomSize);

    if (uncleanedRoomSize > 0){
        SUB_TASK.cleanPlan.makePlan(robotPose);
         DEBUG_PUB.publishCleanArea(SUB_TASK.cleanPlan.getAllRooms(), 
            SUB_TASK.cleanPlan.getCurrentRoomId());
    
        PATH_PLANNER->startMapUpdate();

        currentAreaList = SUB_TASK.cleanPlan.getCurrentAreaPolygons();        
        currentAreaCentroid = utils::area::findCentroid(currentAreaList);
        #if 1 //for_test _wall_clean_start
        ret = CLEANTASK_STATE::START_MOVING_ROOM;
        #else
        ret = CLEANTASK_STATE::START_WALL_CLEAN;
        #endif
    }
    else{
        ceblog(LOG_LV_NECESSARY, BOLDWHITE, "모든 방을 청소하여 청소를완료");
        // SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CLEAN_COMPLT_TO_CHARGER2);
        ret = CLEANTASK_STATE::FINISH;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 방안에서 청소 안한영역을 다시 찾음.
 * 
 * @return CLEANTASK_STATE 
 */
CLEANTASK_STATE CTaskClean::planReMake(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::PLAN_RE_MAKE;
    ceblog(LOG_LV_NECESSARY, BOLDBLUE,
            "청소영역 재 준비 : 현재 영역중 청소 안한곳을 찾자. planRemakeTry : "<< planRemakeTry);
    bool bRemake = SUB_TASK.cleanPlan.findUncleanArea(robotPose, cleanStartPoint);


    if(planRemakeTry >= 3){
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "planRemakeTry 를 계속해서 이번영역은 종료 합니다. : "<<planRemakeTry);
        planRemakeTry = 0;
        SUB_TASK.cleanPlan.setCurrentAreaCleanComplete(true);
        ret = CLEANTASK_STATE::PLAN_MAKE;
    }
    else
    {
        if (bRemake){
            ret = CLEANTASK_STATE::START_MOVING_START_CLEAN_POINT;

        }
        else{
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "이번 영역은 청소를 완료 하였습니다.");
            SUB_TASK.cleanPlan.setCurrentAreaCleanComplete(true);
            ret = CLEANTASK_STATE::PLAN_MAKE;
        }
    }

    planRemakeTry++;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

CLEANTASK_STATE CTaskClean::planUpdateArea()
{
    CLEANTASK_STATE ret = CLEANTASK_STATE::PLAN_UPDATE_AREA;

   
    return ret;
}

CLEANTASK_STATE CTaskClean::startCleanWall(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::START_WALL_CLEAN;
    cleanStartTime = SYSTEM_TOOL.getSystemTime();    
    tPoint nearWall = utils::area::findNearPoint(robotPose, currentAreaList);
    cleanWall.taskStart(robotPose, WALLCLEAN_STATE::FIND_NEAR_WALL,nearWall,currentAreaList);
    
    TIME_CHECK_END(__debug_sw.getTime());
    return CLEANTASK_STATE::RUN_WALL_CLEAN;
}
#if CLEAN_RVIZ_DEBUG
unsigned int debugCnt1 = 0;
#endif
CLEANTASK_STATE CTaskClean::runCleanWall(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::RUN_WALL_CLEAN;
    
    // 영역 벽타기
    cleanWall.taskRun(robotPose, currentAreaList);
    // 기본 벽타기
    // cleanWall.taskRun(robotPose);
    if(cleanWall.isReturnStartPoint())
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        cleanStartPoint = robotPose.convertPoint();
        ret = CLEANTASK_STATE::START_MOVING_START_CLEAN_POINT;
    }

    waterSupply.WaterSupplyStateMachine(cleanStartTime);
#if CLEAN_RVIZ_DEBUG // 디버깅
    if (debugCnt1 % 50 == 0){        
        DEBUG_PUB.publishCleaningArea(robotPose);
        DEBUG_PUB.publishCleaningLine(robotPose);
#if TEST_FORBIDDEN_AREA
        debugAwsForbiddenArea();
#endif
    }
    debugCnt1++;
#endif
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


double calculateDistance(const tPoint& p1, const tPoint& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

double calculateAngle(const tPoint& p1, const tPoint& p2) {
    double dy = p2.y - p1.y;
    double dx = p2.x - p1.x;
    return atan2(dy, dx); // atan2를 사용하여 선분 각도를 라디안으로 반환
}


double findDesiredCleanHeading(std::list<tPoint>& area){
    double maxDistance = 0;
    tPoint longestStart, longestEnd;

    auto it = area.begin();
    auto next_it = std::next(it);

    // 리스트를 순회하며 인접한 점들 사이의 가장 긴 선분 찾기
    while (next_it != area.end()) {
        double dist = calculateDistance(*it, *next_it);
        if (dist > maxDistance) {
            maxDistance = dist;
            longestStart = *it;
            longestEnd = *next_it;
        }
        it = next_it;
        ++next_it;
    }

    double lineAngle = calculateAngle(longestStart, longestEnd);
    // tPose adjustedPose = findRotationForAlignment(robotPose, lineAngle);

    eblog(LOG_LV_NECESSARY, "가장 긴 선분 좌표 : " 
        << longestStart.x <<", "<<longestStart.y
        <<" , ed : "<< longestEnd.x <<" , "<< longestEnd.y
        << " ,선분 길이 : "<<maxDistance
        << "선분 각도 : " << RAD2DEG (lineAngle ) );
        // << "몇도 더 돌아야해 ?: " << RAD2DEG( adjustedPose.angle) );
    return lineAngle;
}


CLEANTASK_STATE CTaskClean::startCleanRoom(tPose robotPose)
{
    CStopWatch __debug_sw;
    CRobotKinematics k;
    std::list<tPoint> area = SUB_TASK.cleanPlan.getCurrentAreaPolygons();
    double rotate = findDesiredCleanHeading(area);
    eblog(LOG_LV_NECESSARY, "robotPose : " << robotPose.x << "," << robotPose.y << " angle : " << utils::math::rad2deg(robotPose.angle)<<" 도" );
    eblog(LOG_LV_NECESSARY, "rotate : "<<rotate<<" , deg : "<<RAD2DEG(rotate));
   
    cleanRoom.taskStart(robotPose, rotate);
    preCleanPt.x = robotPose.x;
    preCleanPt.y = robotPose.y;
    TIME_CHECK_END(__debug_sw.getTime());
    return CLEANTASK_STATE::RUN_CLEAN_ROOM;
}




void CTaskClean::updateCleaned(tPose robotPose)
{
    curCleanPt = robotPose.convertPoint();    
    ServiceData.robotMap.cleanMap.updateCleaned(preCleanPt, curCleanPt, CONFIG.cleanBoundPixel);
    preCleanPt = curCleanPt;
}

#if CLEAN_RVIZ_DEBUG
unsigned int debugCnt2 = 0;
#endif
CLEANTASK_STATE CTaskClean::runCleanRoom(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::RUN_CLEAN_ROOM;
    bool bSideDetect = false;
    
    // 종료 조건 체크
    bool inside = 
        utils::area::isInside(tPoint(robotPose.x, robotPose.y), currentAreaList);
    if(inside == false){
        cleanRoom.setEndLineTrig(true);
    }
    else{
        cleanRoom.setEndLineTrig(false);
    }

    // if (SUB_TASK.cleanPlan.isSetupNogoZoneDocking()){
    //     std::list<tPoint> noGoZone = SUB_TASK.cleanPlan.getNoGoZoneDocking();
    //     inside = 
    //         utils::area::isInside(tPoint(robotPose.x, robotPose.y), noGoZone);
    //     if(inside && CLEAN_ROOM_STATE::RUNNING_MAIN_LINE == cleanRoom.getCleanRoomState()){
    //         cleanRoom.setDockingZoneTrig(true);
    //     }
    //     else{
    //         cleanRoom.setEndLineTrig(false);
    //     }
    // }
   

    // 라인청소 실행
    tCleanEndFlag endFlag = cleanRoom.taskRun(robotPose);
    if(endFlag.value){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "클린룸 타스크 종료 : "<<endFlag.value);
        SUB_TASK.cleanPlan.setCurrentAreaCleanComplete(true);
        MOTION.startStopOnMap( tProfile(), true);
        

#if 0   // 영역 안에서 신규 컨투어를 개설하는 방법으로 재 계획.
        // 라인청소를 한번이라도 했으면 클리어.
        if (cleanRoom.getLineCleanCnt() > 0){
            planRemakeTry = 0;
        }
        
        ret = CLEANTASK_STATE::PLAN_RE_MAKE;
#else   //타스크클린룸의 서칭을 믿고 그쟝 종료
        SUB_TASK.cleanPlan.setCurrentAreaCleanComplete(true);
        ret = CLEANTASK_STATE::PLAN_MAKE;
#endif
       
    }

    waterSupply.WaterSupplyStateMachine(cleanStartTime);
    // 청소한거 지도에 표시
    updateCleaned(robotPose);    
    ServiceData.robotMap.robotTrajectory.setCleanedTrajectory(robotPose);
    
#if CLEAN_RVIZ_DEBUG // 디버깅
    if (debugCnt2 % 50 == 0){        
        DEBUG_PUB.publishCleaningArea(robotPose);
        DEBUG_PUB.publishCleaningLine(robotPose);
        DEBUG_PUB.publishContour(currentAreaList);
#if TEST_FORBIDDEN_AREA
        debugAwsForbiddenArea();
#endif
        if ( SUB_TASK.cleanPlan.isSetupNogoZoneDocking() )
            DEBUG_PUB.publishContour2(SUB_TASK.cleanPlan.getNoGoZoneDocking());
    }
    debugCnt2++;
#endif
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

CLEANTASK_STATE CTaskClean::startMovingRoom(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::START_MOVING_ROOM;
    
    //tPoint target = findRightPoint();
    tPoint target = SUB_TASK.cleanPlan.getAreaCenterPoint();
    moveRoom.taskStart(target);
    ret = CLEANTASK_STATE::RUN_MOVING_ROOM;

    ceblog(LOG_LV_NECESSARY, BLUE, " [ " << SUB_TASK.cleanPlan.getCurrentRoomId() 
        << " ] 번 방으로 이동합니다.target : "<<target.x<< " , "<<target.y);
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

#if CLEAN_RVIZ_DEBUG
unsigned int debugCnt3 = 0;
#endif
CLEANTASK_STATE CTaskClean::runMovingRoom(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::RUN_MOVING_ROOM;

    bool bNext = false;

    if(SUB_TASK.cleanPlan.checkInArea(robotPose.convertPoint())){
        ceblog(LOG_LV_NECESSARY, MAGENTA, "방안에 있어요.");
        eblog(LOG_LV_NECESSARY, "robotPose : " << robotPose.x << "," << robotPose.y << " angle : " << utils::math::rad2deg(robotPose.angle)<<" 도" );
        bNext = true;
    }
    else if (moveRoom.taskRun(robotPose)){
        if (moveRoom.getState() == MOVEROOM_STATE::FAIL){
            ceblog(LOG_LV_NECESSARY, MAGENTA, "방이동을 할 수 없어요.");
            ret = CLEANTASK_STATE::FINISH;
        }
        else{
            ceblog(LOG_LV_NECESSARY, MAGENTA, "방안에 도착 했어요.");
            bNext = true;        
        }
        
    }

    if (bNext)
    {
        MOTION.startStopOnMap(tProfile(),true);
        
        if (bSkipWallClean){
            ceblog(LOG_LV_NECESSARY, MAGENTA, "벽청소 스킵 !");
            ret = CLEANTASK_STATE::START_CLEAN_ROOM;
            bSkipWallClean = false;
        }
        else{
            ceblog(LOG_LV_NECESSARY, MAGENTA, "벽청소 준비 ~");
            ret = CLEANTASK_STATE::START_WALL_CLEAN;
        }
    }

#if CLEAN_RVIZ_DEBUG // 디버깅
    if (debugCnt3 % 50 == 0){        
        DEBUG_PUB.publishCleaningArea(robotPose);
        DEBUG_PUB.publishCleaningLine(robotPose);
#if TEST_FORBIDDEN_AREA
        debugAwsForbiddenArea();
#endif
    }
    debugCnt3++;
#endif
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

CLEANTASK_STATE CTaskClean::startMovingCleanStartPoint(tPose robotPose)
{
    CLEANTASK_STATE ret = CLEANTASK_STATE::START_MOVING_START_CLEAN_POINT;
    
    // cleanStartPoint = findRightPoint();
    moveRoom.taskStart(cleanStartPoint);
    ret = CLEANTASK_STATE::RUN_MOVING_START_CLEAN_POINT;

    ceblog(LOG_LV_NECESSARY, BLUE, "방청소 시작 getCurrentRoomId :: " 
        << SUB_TASK.cleanPlan.getCurrentRoomId()
        <<"target : "<<cleanStartPoint.x<< " , "<<cleanStartPoint.y);

    return ret;
}

#if CLEAN_RVIZ_DEBUG
unsigned int debugCnt4 = 0;
#endif
CLEANTASK_STATE CTaskClean::runMovingCleanStartPoint(tPose robotPose)
{
    CStopWatch __debug_sw;
    CLEANTASK_STATE ret = CLEANTASK_STATE::RUN_MOVING_START_CLEAN_POINT;
    double goalMargin = 0.10;

    if (MOTION.isNearTargetPose(robotPose, cleanStartPoint, goalMargin) )
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지에 가까워 타스크 무브룸을 종료 시킵니다.");
        MOTION.startStopOnMap(tProfile(),true);
        
        ret = CLEANTASK_STATE::START_CLEAN_ROOM;
    }
    else if (moveRoom.taskRun(robotPose)){
        ret = CLEANTASK_STATE::START_CLEAN_ROOM;
    }
    else if(moveRoom.getPathFailCount() >= 5){
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "경로를 계속 못찾아서 다른 행동으로 넘어갑니다.");        
        ret = CLEANTASK_STATE::PLAN_RE_MAKE;

    }

#if CLEAN_RVIZ_DEBUG // 디버깅
    if (debugCnt4 % 50 == 0){        
        DEBUG_PUB.publishCleaningArea(robotPose);
        DEBUG_PUB.publishCleaningLine(robotPose);
#if TEST_FORBIDDEN_AREA
        debugAwsForbiddenArea();
#endif
    }
    debugCnt4++;
#endif
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskClean::taskStart(tPose robotPose)
{
    ServiceData.robotMap.cleanMap.clear();
    currentAreaList.clear();
    safeCleanZone.clear();
    setState(CLEANTASK_STATE::PLAN_INIT);
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "CLEAN TASK START!!");
}

bool CTaskClean::taskRun(tPose robotPose)
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    switch (cleanLineState)
    {
    case CLEANTASK_STATE::NONE :
        break;
     case CLEANTASK_STATE::START_MOVING_START_CLEAN_POINT :
        setState(startMovingCleanStartPoint(robotPose));
        break;    
    case CLEANTASK_STATE::RUN_MOVING_START_CLEAN_POINT :
        setState(runMovingCleanStartPoint(robotPose));
        break;
     case CLEANTASK_STATE::START_WALL_CLEAN :
        setState(startCleanWall(robotPose));
        break;    
    case CLEANTASK_STATE::RUN_WALL_CLEAN :
        setState(runCleanWall(robotPose));
        break;
     case CLEANTASK_STATE::START_CLEAN_ROOM :
        setState(startCleanRoom(robotPose));
        break;        
    case CLEANTASK_STATE::RUN_CLEAN_ROOM :
        setState(runCleanRoom(robotPose));
        break;
    case CLEANTASK_STATE::START_MOVING_ROOM :
        setState(startMovingRoom(robotPose));
        break;
    case CLEANTASK_STATE::RUN_MOVING_ROOM :
        setState(runMovingRoom(robotPose));
        break;
    case CLEANTASK_STATE::PLAN_INIT :
        setState(planInit(robotPose));
        break;
    case CLEANTASK_STATE::PLAN_MAKE :
        setState(planMake(robotPose));
        break;
    case CLEANTASK_STATE::PLAN_RE_MAKE :
        setState(planReMake(robotPose));
        break;
    case CLEANTASK_STATE::PLAN_UPDATE_AREA :
        setState(planUpdateArea());
        break;
    case CLEANTASK_STATE::FINISH :
        ret = true;
        break;
    case CLEANTASK_STATE::STOP :
           
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CTaskClean::rePortCleanAreaInfo()
{
    tAreaInfoData areaInfo;
    tAreaInfo tempArea;
    std::list<tPoint> polygons;
    std::list<CCleanRoom> tempRoom = SUB_TASK.cleanPlan.getRooms();
    tPointdouble tempPoint;
    areaInfo.areaNumber = tempRoom.size();

    for(int i = 0; i < areaInfo.areaNumber; i++){
        tempArea.id = tempRoom.front().getId();
        polygons = tempRoom.front().getPolygons(tempArea.id);
        tempArea.polyganNum = (short)(polygons.size()*2);
        
        for(int j=0;j < tempArea.polyganNum; j++){
            tempArea.polygon[j] = polygons.front().x;
            tempArea.polygon[++j] = polygons.front().y;
            polygons.pop_front();
        }
        // for(tPoint polygon : polygons){
        //     tempPoint.x = polygon.x;
        //     tempPoint.y = polygon.y;
        //     tempArea.polygon.push_back(tempPoint);
        // }
        areaInfo.info.push_back(tempArea);
        if(!tempRoom.empty()) tempRoom.pop_front();
    }
    ROBOT_CONTROL.reportAreaInfo(areaInfo);
}

void CTaskClean::debugAwsForbiddenArea()
{
    std::list<tForbiddenRect> tempForbidRect = ServiceData.forbiddenArea.getForbiddenRect();
    std::list<tForbiddenLine> tempForbidLine = ServiceData.forbiddenArea.getForbiddenLine();
    std::list<tPoint> forbidAreaRect;
    std::list<tPoint> forbidAreaLine;
    tPoint point;
    forbidAreaRect.clear();
    forbidAreaLine.clear();
    for(tForbiddenRect temp : tempForbidRect){
        point.x = temp.x;
        point.y = temp.y;
        forbidAreaRect.push_back(point);
        point.x = (temp.x+temp.h);
        point.y = temp.y;
        forbidAreaRect.push_back(point);
        point.x = temp.x;
        point.y = (temp.y-temp.w);
        forbidAreaRect.push_back(point);
        point.x = (temp.x+temp.h);
        point.y = (temp.y-temp.w);
        forbidAreaRect.push_back(point);
    }
    if(!forbidAreaRect.empty()){
        for(tPoint point : forbidAreaRect){
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Forbidden Rect X : " << point.x <<  " Y : " << point.y);
        }
        DEBUG_PUB.publishForbiddenRect(forbidAreaRect);
    }
    
    for(tForbiddenLine temp : tempForbidLine){
        point.x = temp.x1;
        point.y = temp.y1;
        forbidAreaLine.push_back(point);
        point.x = temp.x2;
        point.y = temp.y2;
        forbidAreaLine.push_back(point);
    }
    
    if(!forbidAreaLine.empty()){
        for(tPoint point : forbidAreaLine){
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Forbidden Line X : " << point.x <<  " Y : " << point.y);
        }
        DEBUG_PUB.publishForbiddenLine(forbidAreaLine);
    }
}