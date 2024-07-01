#include "cleanplan.h"
#include "utils.h"
#include "eblog.h"
#include <algorithm>
#include "imgProcessor.h"
#include "rosPublisher.h"
#include "coreData/serviceData.h"
#include "kinematics.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 1 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 20.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

using namespace cv;

// map 전체 크기 1000 * 1000

#define ROOM_SIZE           3.0     // 단위 m hjkim221220 - AREA 사이즈를 4M 로 변경
#define PIXEL_RESOLUTION    0.05    // 단위 m


CCleanPlan::CCleanPlan()
{
    CStopWatch __debug_sw;
    
    /* initialize */
    planInfo = tPlanInfo();             //계획 정보 모음.
    cleanPlanId = -1; 
    setNoGoZoneDocking({-0.8, 0.0}, 0.50);
    bNoGoZoneSetup = false;
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CCleanPlan::~CCleanPlan()
{
    CStopWatch __debug_sw;
    
    rooms.clear();
    
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 현제 청소영역에 상태 정보
 * icbaek, 22.10.20
 * @return E_CLEAN_STATE 
 */
E_CLEAN_STATE CCleanPlan::getAreaCleanState()
{
    return getAreaCleanState(planInfo.currentRoomId, planInfo.currentAreaId);
}

/**
 * @brief ID 를 통해 선택된 청소영역 청소상태 정보
 * icbaek, 22.10.20
 * @param roomId 
 * @param areaId 
 * @return E_CLEAN_STATE 
 * @note 0.0 ~ 0.1 msec
 * @date 23.08.18
 */
E_CLEAN_STATE CCleanPlan::getAreaCleanState(int roomId, int areaId)
{
    return findArea(roomId, areaId).getCleanState();
}


/**
 * @brief room, area 정보 초기화 
 * jhnoh, 23.01.02
 */
void CCleanPlan::clearRooms()
{
    CStopWatch __debug_sw;
        
    for(CCleanRoom& room : rooms)
    {        
        room.clearAreaPlan();
        room.clearAreas();
    }
    if(!rooms.empty()) rooms.clear();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 청소해야할 방의 수량
 * 
 * @return int 
 */
int CCleanPlan::getUnCleanRoomSize()
{
    CStopWatch __debug_sw;
    int ret = 0;
        
    for(CCleanRoom& room : rooms)
    {   
        if ( room.getCleanState() == E_CLEAN_STATE::UNCLEAN )
        {
            ret++;
        }        
    }
    return ret;    
    TIME_CHECK_END(__debug_sw.getTime());
}

int CCleanPlan::getCleanedRoomSize()
{
    CStopWatch __debug_sw;
    int ret = 0;
        
    for(CCleanRoom& room : rooms)
    {   
        if ( room.getCleanState() == E_CLEAN_STATE::CLEANED )
        {
            ret++;
        }        
    }
    return ret;    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 모든 room과 그 room 안에 area의 청소 상태 출력 디버그
 * jhnoh, 23.01.02
 */
void CCleanPlan::__debug__cleanState__()
{
    CStopWatch __debug_sw;
    
    eblog(LOG_LV_LINECLEAN,   "||______________________________Clean STATE______________________________||");
    std::cout << std::endl;
    for(CCleanRoom room : rooms)
    {
        switch (room.getCleanState())
        {
        case E_CLEAN_STATE::CLEANED :
            eblog(LOG_LV_LINECLEAN,   " ROOM CLEAN STATE ::  [ roomId ] ::  [ " << room.getId() << " ] = ");
            eblog(LOG_LV_LINECLEAN,  " [ CLEANED ]");
            break;
        case E_CLEAN_STATE::NOTVALID :
            eblog(LOG_LV_LINECLEAN,   " ROOM CLEAN STATE ::  [ roomId ] ::  [ " << room.getId() << " ] = ");
            eblog(LOG_LV_LINECLEAN,   " [ NOTVALID ]");
            break;
        case E_CLEAN_STATE::RUNNING :
            eblog(LOG_LV_LINECLEAN,   " ROOM CLEAN STATE ::  [ roomId ] ::  [ " << room.getId() << " ] = ");
            eblog(LOG_LV_LINECLEAN,   " [ RUNNING ]");
            break;
        case E_CLEAN_STATE::UNCLEAN :
            eblog(LOG_LV_LINECLEAN,   " ROOM CLEAN STATE ::  [ roomId ] ::  [ " << room.getId() << " ] = ");
            eblog(LOG_LV_LINECLEAN,   " [ UNCLEAN ]");
            break;
        default:
            break;
        }

        for(CCleanArea area : room.getAreas())
        {
            switch (area.getCleanState())
            {
            case E_CLEAN_STATE::CLEANED :
                eblog(LOG_LV_LINECLEAN,   " AREA CLEAN STATE   [ roomId  - areaId ] :: [ " << room.getId() << " ---- "  << area.getId() << " ] = ");
                eblog(LOG_LV_LINECLEAN,   " [ CLEANED ]");
                break;
            case E_CLEAN_STATE::NOTVALID :
                eblog(LOG_LV_LINECLEAN,   " AREA CLEAN STATE   [ roomId  - areaId ] :: [ " << room.getId() << " ---- "  << area.getId() << " ] = ");
                eblog(LOG_LV_LINECLEAN,   " [ NOTVALID ]");
                break;
            case E_CLEAN_STATE::RUNNING :
                eblog(LOG_LV_LINECLEAN,   " AREA CLEAN STATE   [ roomId  - areaId ] :: [ " << room.getId() << " ---- "  << area.getId() << " ] = ");
                eblog(LOG_LV_LINECLEAN,   " [ RUNNING ]");
                break;
            case E_CLEAN_STATE::UNCLEAN :
                eblog(LOG_LV_LINECLEAN,   " AREA CLEAN STATE   [ roomId  - areaId ] :: [ " << room.getId() << " ---- "  << area.getId() << " ] = ");
                eblog(LOG_LV_LINECLEAN,   " [ UNCLEAN ]");
                break;
            default:
                break;
            }
        }
        eblog(LOG_LV_LINECLEAN,   "________________________________________________________________________________ ");
        std::cout << std::endl;

    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}
/**
 * @brief 현재 area의 polygon들 리턴
 * jhnoh, 23.01.03
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanPlan::getCurrentAreaPolygons()
{    
    return getPolygon(planInfo.currentRoomId, planInfo.currentAreaId);
}

/**
 * @brief roomId와 areaId를 통해 특정 area의 polygon들 리턴
 * jhnoh, 23.01.03
 * @param roomId 
 * @param areaId 
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanPlan::getPolygon(int roomId, int areaId)
{
    CStopWatch __debug_sw;
    
    std::list<tPoint> test;
    if(roomId == -1 ||areaId == -1)
    {
        eblog(LOG_LV_LINECLEAN, "_____roomId == -1 || areaId == -1____ "  <<std::endl);
        
        TIME_CHECK_END(__debug_sw.getTime());
        return test;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return findArea(roomId, areaId).getPolygon();
}

std::vector<cv::Point> CCleanPlan::getCurrentAreaContour()
{
    return getContour(planInfo.currentRoomId, planInfo.currentAreaId);
}

std::vector<cv::Point> CCleanPlan::getContour(int roomId, int areaId)
{
    CStopWatch __debug_sw;
    
    std::vector<cv::Point> test;
    if(roomId == -1 ||areaId == -1)
    {
        eblog(LOG_LV_LINECLEAN, "_____roomId == -1 || areaId == -1____ "  <<std::endl);
        
        TIME_CHECK_END(__debug_sw.getTime());
        return test;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return findArea(roomId, areaId).getContour();
}

/**
 * @brief 청소 영역 완료 TODO : 외부에서 이함수를 호출하여 청소를 완료했다고 할거다. 그에 맞게 함수를 만들자
 * icbaek, 22.10.20
 * true : 청소 완료
 * false : 청소 실패
 * 
 */
void CCleanPlan::setCurrentAreaCleanComplete(bool bSet)
{
    CStopWatch __debug_sw;
    
    bool bRoomClean;
    if (bSet == true){
        for(CCleanRoom& room : rooms){
            if(room.getId() == planInfo.currentRoomId){
                bRoomClean = true;
                room.setAreaCleanState(planInfo.currentAreaId, E_CLEAN_STATE::CLEANED);
                
                for(CCleanArea area : room.getAreas()){
                    if(area.getCleanState() == E_CLEAN_STATE::UNCLEAN){
                        bRoomClean=false; // 에어리어 중 하나라도 청소가 안되면 룸은 청소 안된거다.
                    }
                }
                if(bRoomClean){
                    room.setCleanState(E_CLEAN_STATE::CLEANED);
                }
            }
        }
    }
    else
    {
         for(CCleanRoom& room : rooms)
        {
            if(room.getId() == planInfo.currentRoomId)
            {
                bRoomClean = true;
                room.setAreaCleanState(planInfo.currentAreaId, E_CLEAN_STATE::NOTVALID);
                
                for(CCleanArea area : room.getAreas())
                {
                    if(area.getCleanState() == E_CLEAN_STATE::UNCLEAN)
                    {
                        bRoomClean=false;
                    }
                }
                if(bRoomClean)
                {
                    room.setCleanState(E_CLEAN_STATE::CLEANED);
                }
            }
        }
    }        
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  area에서 가장 왼쪽 point를 반환
 * icbaek, 22.10.20
 * @param roomId 
 * @param areaId 
 * @param 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaLeftEndPoint()
{
    return getAreaLeftEndPoint(planInfo.currentRoomId,planInfo.currentAreaId);
}


/**
 * @brief area에서 가장 왼쪽 point를 반환
 * icbaek, 22.10.20
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaLeftEndPoint(int roomId, int areaId)
{
    return findArea(roomId, areaId).getAreaLeftEndPoint();
}

/**
 * @brief area에서 가장 오른쪽 point를 반환
 * icbaek, 22.10.20
 * @param
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaRightEndPoint()
{
    return getAreaRightEndPoint(planInfo.currentRoomId,planInfo.currentAreaId);
}

/**
 * @brief area에서 가장 오른쪽 point를 반환
 * icbaek, 22.10.20
 * @param roomId 
 * @param areaId 
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaRightEndPoint(int roomId, int areaId )
{
    return findArea(roomId, areaId).getAreaRightEndPoint();
}

/**
 * @brief area에서 가장 위쪽 point를 반환
 * icbaek, 22.10.20
 * @param 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaUpEndPoint()
{
    return getAreaUpEndPoint(planInfo.currentRoomId,planInfo.currentAreaId);
}
/**
 * @brief area에서 가장 위쪽 point를 반환
 * icbaek, 22.10.20
 * @param roomId 
 * @param areaId 
 * @param 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaUpEndPoint(int roomId, int areaId)
{
    return findArea(roomId, areaId).getAreaUpEndPoint();
}

/**
 * @brief area에서 가장 아래쪽 point를 반환
 * icbaek, 22.10.20
 * @param
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaDownEndPoint()
{
    return getAreaDownEndPoint(planInfo.currentRoomId,planInfo.currentAreaId);
}

/**
 * @brief area에서 가장 아래쪽 point를 반환
 * icbaek, 22.10.20
 * @param roomId 
 * @param areaId 
 * @param  
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaDownEndPoint(int roomId, int areaId)
{
    return findArea(roomId, areaId).getAreaDownEndPoint();
}

/**
 * @brief id 정보를 return
 * jhnoh, 22.12.05
 * @return tPlanInfo 
 */
tPlanInfo CCleanPlan::getIdInfo()
{
    return planInfo;
}

std::list<CCleanRoom> CCleanPlan::getRooms()
{
    return rooms;
}


/**
 * @brief rooms와 청소완료 된 cleanedRooms 를 하나의 리스트로 만들어 반환.
 * 
 * @return std::list<CCleanRoom> 
 */
std::list<CCleanRoom> CCleanPlan::getAllRooms()
{
    CStopWatch __debug_sw;
    
    std::list<CCleanRoom> allRooms;
    allRooms.insert(allRooms.end(), rooms.begin(), rooms.end());
    TIME_CHECK_END(__debug_sw.getTime());
    return allRooms;
}


/**
 * @brief id 를 확인하여 area 값 찾기
 * icbaek, 22.10.20
 * //TODO : area id가 없을 때 예외처리 필요
 * @param roomId 
 * @param areaId 
 */
CCleanArea CCleanPlan::findArea(int roomId, int areaId)
{
    CStopWatch __debug_sw;
    
    E_CLEAN_STATE areaState = E_CLEAN_STATE::UNCLEAN;
    if(roomId == -1 || areaId == -1 ) 
    {
        eblog(LOG_LV_NECESSARY, "/n/n/n/n !! roomId == -1 || areaId == -1 /n/n/n/n");  
    }
    for ( CCleanRoom& room : rooms )
    {
        if (room.getId() == roomId)
        {
            std::list<CCleanArea> areas = room.getAreas();
            for ( CCleanArea& area : areas )
            {
                if (area.getId() == areaId)
                {
                    //eblog(LOG_LV_NECESSARY, " roomId = " << roomId << "areaId = " << areaId);
                    TIME_CHECK_END(__debug_sw.getTime());      
                    return area;
                }
            }
        }
    }

    
    ceblog(LOG_LV_NECESSARY, RED, "!! critcal null return /n/n/n/n");
    
    TIME_CHECK_END(__debug_sw.getTime());
    return CCleanArea(); // 누락된 return 임시로 추가
}

/**
 * @brief id 를 확인하여 room 값 찾기
 * jhnoh, 22.11.30
 *  //TODO : room id가 없을 때 예외처리 필요
 * @param roomId 
 * @return CCleanRoom 
 */
CCleanRoom CCleanPlan::findRoom(int roomId)
{
    CStopWatch __debug_sw;
    
    if(roomId == -1) 
    {
        eblog(LOG_LV, "/n/n/n/n !! roomId == -1 /n/n/n/n");  
    }
    for ( CCleanRoom& room : rooms )
    {
        if (room.getId() == roomId)
        {
            TIME_CHECK_END(__debug_sw.getTime());
            return room;  
        }
        else
        {
        }
    }
    eblog(LOG_LV,  "/n/n/n/n !! critcal null return /n/n/n/n");  
    
    TIME_CHECK_END(__debug_sw.getTime());
    return CCleanRoom(); // 누락된 return 임시로 추가
}

/**
 * @brief room id를 설정
 * 
 * @param roomId 
 * @return CCleanRoom& 
 */
CCleanRoom& CCleanPlan::setRoom(int roomId)
{
    CStopWatch __debug_sw;
    
    for ( CCleanRoom& room : rooms )
    {
        if (room.getId() == roomId)
        {   
            TIME_CHECK_END(__debug_sw.getTime());
            return room;  
        }
        else
        {
        
        }
    }  
    eblog(LOG_LV,  "/n/n/n/n !! critcal null return /n/n/n/n");

    TIME_CHECK_END(__debug_sw.getTime());
    // return // 누락된 return 추가해주세요.
}

/**
 * @brief 탐색 후 청소 시작전 영역을 분할 하기
 * 
 */
bool CCleanPlan::makeAreaByMap()
{
    CStopWatch __debug_sw;
    bool ret = false;
    if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
    {
        ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
        tGridmapInfo info = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
        unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
        makeArea(pMap, info);
        ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
        ret = true;
    }
    return ret;
    TIME_CHECK_END(__debug_sw.getTime());
}



/**
 * @brief 청소 커버리지 
 * 
 */
bool CCleanPlan::checkAreaCoverage()
{
    CStopWatch __debug_sw;
    CImgProcessor imgProc;
    bool bRet = false;

    ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
    
    tGridmapInfo info = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
    unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
    Mat img = imgProc.convertSimplifyGridMap2Mat(pMap, info.width, info.height);
    
    ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
    
    std::vector<cv::Point> contour = getCurrentAreaContour();

    std::list<tPoint> cleanpath = 
        ServiceData.robotMap.robotTrajectory.getCleanedTrajectory();

    std::list<cv::Point> cleanpathCv =
        imgProc.convertRobotpathToCvpath(cleanpath, info);
    std::pair<double, double> result = imgProc.getAreaCoverage(img, contour, cleanpathCv , 5);
    double ratio = result.first;
    cleanedSize = result.second;

    eblog(LOG_LV_NECESSARY, "청소 비율 : "<<ratio);

    if (ratio > 90){
        eblog(LOG_LV_NECESSARY, "청소 종료 트리그 !!!! : "<<ratio);
        bRet = true;
    }

    return bRet;
}
double CCleanPlan::getCleanedSize(){return cleanedSize;}

/**
 * @brief 청소 중인 영역에서 임시로 청소가 안된 영역을 생성한다
 * 
 */
bool CCleanPlan::findUncleanArea(tPose robotPose, tPoint& cleanPoint )
{
    CStopWatch __debug_sw;
    CImgProcessor imgProc;
    bool bRet = false;

    ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
    
    tGridmapInfo info = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
    unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
    Mat img = imgProc.convertSimplifyGridMap2Mat(pMap, info.width, info.height);
    
    ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
    
    std::vector<cv::Point> contour = getCurrentAreaContour();

    std::list<tPoint> cleanpath = 
        ServiceData.robotMap.robotTrajectory.getCleanedTrajectory();

    std::list<cv::Point> cleanpathCv =
        imgProc.convertRobotpathToCvpath(cleanpath, info);

    std::vector<std::vector<cv::Point>> uncleandContours;    
    imgProc.fillAreaCleanPath(img, contour, cleanpathCv , 5, uncleandContours);

    std::list<std::list<tPoint>> uncleanedPolygon;
    for (auto& contour :uncleandContours )
    {
        double area = cv::contourArea(contour);
        if (area > 100){
            eblog(LOG_LV_NECESSARY, "면적 통과 : "<<area);
            std::list<tPoint> org = imgProc.convertContourToPolygon(contour, info);
            std::list<tPoint> shrink;
            utils::area::resizeArea(org, shrink, -20);
            uncleanedPolygon.push_back(shrink);
        }
        else{
            // eblog(LOG_LV_NECESSARY, "면적 탈락 : "<<area);
        }
    }
    DEBUG_PUB.publishContourList(uncleanedPolygon);
    eblog(LOG_LV_NECESSARY, "uncleanedPolygon : "<<uncleanedPolygon.size());

    //가장 가까운 포인트 찾기.
    double minDistance = std::numeric_limits<double>::max();
    

    std::list<tPoint> center;
    //각 에어리어 중 center 찾아 놓고
    for (auto polygon : uncleanedPolygon) {
        tPoint pt = utils::area::findCentroid(polygon);
        center.push_back(pt);
        bRet = true;
    }
    // 그중 가장 가까운것.
    cleanPoint = utils::area::findNearPoint(robotPose, center);

    eblog(LOG_LV_NECESSARY, "cleanPoint : "<<cleanPoint.x <<" , " <<cleanPoint.y);
    TIME_CHECK_END(__debug_sw.getTime());

    return bRet;
}

/**
 * @brief 매번 check 하여 플랜을 결정 한다.
 * 
 * @param src 
 * @param dest : area 그려지는 장소.
 * @param width 
 * @param height 
 * @note 24.80 ms : src 크기에 따라 달라짐.
 * @date 23.08.18
 * 
 * 폴리곤 인덱싱.
    3-----------2
    |           |
    |           |
    |           |
    0-----------1
 */ 
void CCleanPlan::makeArea(unsigned char *src, tGridmapInfo info)
{
    CStopWatch __debug_sw;
    int makeId = 0, areaId = 0;  
    CImgProcessor imgProc;

    Mat img = imgProc.convertSimplifyGridMap2Mat(src, info.width, info.height);
    
    //area를 만들기 전에 rooms 를 정리
    clearRooms();
        
    //이미지 전체를 검색하여 방들을 구분.     
    std::vector<std::vector<cv::Point>> contours = imgProc.fakeBspV4(img);
    eblog(LOG_LV_NECESSARY,"이미지처리후 방개수(컨투어) : "<<contours.size());
    
    // 이미지 위치를 좌표 위치로 변환
    for (auto& contour : contours) {
        
        //일단 룸은 무조건 생성 한다. 
        CCleanRoom room; 
        room.setId(makeId); 
        room.setCleanEnable(false); 
        room.setCleanState(E_CLEAN_STATE::NOTVALID); 

        CCleanArea area(areaId, E_CLEAN_STATE::UNCLEAN); // 미청소 에리어 생성

        room.addArea(area);
        room.setCleanEnable(true);
        room.setCleanState(E_CLEAN_STATE::UNCLEAN);

        std::list<tPoint> set = imgProc.convertContourToPolygon(contour, info);
        // 좌표 변환후 폴리곤 등록
        room.setPolygon(areaId, set);
        // cv 콘투어도 등록
        room.setContour(areaId, contour);

        eblog(LOG_LV_NECESSARY, "contour : "<<contour.size());

        rooms.emplace_back(room);
        areaId++;        
        makeId++; 
    }

#if 0 //debug
    for(CCleanRoom room : rooms) 
    { 
        for(CCleanArea area : room.getAreas() ) 
        { 
            eblog(LOG_LV_NECESSARY, "room ID :: [ " <<room.getId() << " ]  " 
                << " area ID :: [ " << area.getId() << " ]  ");
            for(tPoint polygon : area.getPolygon()) 
            { 
                eblog(LOG_LV_NECESSARY, "polygon :: ( " << polygon.x << " , "
                    << polygon.y << " ) "); 
            } 
        }        
    } 
#endif
    
    TIME_CHECK_END(__debug_sw.getTime());
}


void CCleanPlan::makeCustomArea(tPose robotPose)
{
    CStopWatch __debug_sw;
    
    bool bRet = true;    
    clearRooms();
    
    tPoint poygonPt[4];    
    int makeId = 0;
    double offsetX = 0.0;
    double offsetY = 0.0;
    
    
    // make part 
    // room 하나에 area 하나 

    int partNumY = 1;
    int partNumX = 1;
    for (int quadrant = 0 ; quadrant < 1; quadrant++)
    {
        for ( int i = 0; i < partNumY; i++)
        {
            for ( int j = 0; j < partNumX; j++)
            {   
                CCleanRoom room;
                CCleanArea area;

                room.setCleanEnable(true);  // 일단 모두 청소 가능하게 설정.                 
                room.setCleanState(E_CLEAN_STATE::UNCLEAN);// 당연하게 플랜 짤때는 미 청소.
                room.setId(makeId);

                area.setId(makeId); //area 먼저 add icbaek, 22.10.20
                room.addArea(area);
                

                // 1사분면 폴리곤 좌표를 만든다.
                offsetX = (j * ROOM_SIZE);
                offsetY = (i * ROOM_SIZE);

                // poygonPt[0] = tPoint(0              + offsetX, 0 + offsetY);
                // poygonPt[1] = tPoint(ROOM_SIZE + offsetX, 0 + offsetY);
                // poygonPt[2] = tPoint(ROOM_SIZE + offsetX, ROOM_SIZE + offsetY);
                // poygonPt[3] = tPoint(0              + offsetX, ROOM_SIZE + offsetY);

                poygonPt[0] = tPoint(robotPose.x + (offsetX)*cos(robotPose.angle), robotPose.y + (offsetY)*sin(robotPose.angle));
                poygonPt[1] = tPoint(robotPose.x + (ROOM_SIZE + offsetX)*cos(robotPose.angle), robotPose.y + (ROOM_SIZE +offsetY)*sin(robotPose.angle));
                poygonPt[2] = tPoint(robotPose.x + (ROOM_SIZE*sqrt(2) + offsetX)*cos(robotPose.angle+M_PI_4), robotPose.y + (ROOM_SIZE*sqrt(2) + offsetY)*sin(robotPose.angle+M_PI_4));
                poygonPt[3] = tPoint(robotPose.x + (ROOM_SIZE + offsetX)*cos(robotPose.angle+M_PI_2), robotPose.y + (ROOM_SIZE + offsetY)*sin(robotPose.angle + M_PI_2));
                
                for (int i = 0; i < 4; i++)
                {
                    room.addPolygon(makeId, poygonPt[i]);
                }
                
                rooms.emplace_back(room);
            }
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 청소해야할 방을 선택 한다.
 * 
 */
void CCleanPlan::updateCurrentId()
{
    int minPriority = INT_MAX; // 가능한 가장 큰 정수로 초기화
    int minPriorityRoomId = -1; // 유효하지 않은 ID 값으로 초기화

    for(CCleanRoom& room : rooms)
    {
        if (room.getCleanState() == E_CLEAN_STATE::UNCLEAN)
        {
            int roomPriority = room.getCleanPriority(); // 방의 우선순위 가져오기
            if (roomPriority < minPriority) // 현재 방의 우선순위가 가장 낮은 것보다 낮은지 확인
            {
                minPriority = roomPriority; // 가장 낮은 우선순위 업데이트
                minPriorityRoomId = room.getId(); // 해당 방의 ID 저장
            }
        }        
    }

    if (minPriorityRoomId != -1) // 유효한 방 ID가 발견된 경우
    {
        for(CCleanRoom room : rooms) {
            if (room.getId() == minPriorityRoomId)
            {
                for (CCleanArea area : room.getAreas()) {

                    planInfo.currentRoomId = minPriorityRoomId; // 현재 방 ID 업데이트
                    planInfo.currentAreaId = room.getId(); // 현재 방 ID 업데이트
                    break;
                }
                eblog(LOG_LV_NECESSARY, "update room ID : " << planInfo.currentRoomId  
                    << " , area ID : " << planInfo.currentAreaId );
                break;
            }
        }
    }
    
}



/**
 * @brief robot 위치로부터 우선순위 정렬
 * 
 * @param robotPose 
 */
void CCleanPlan::sortPriorityByRobotPose(tPose robotPose)
{
    using RoomAreaDistanceTuple = std::tuple<double, int, int>;
    std::vector<RoomAreaDistanceTuple> distances;

    for(CCleanRoom room : rooms) {
        for (CCleanArea area : room.getAreas()) {
            double tmpDis = utils::math::distanceTwoPoint(robotPose, 
                getAreaCenterPoint(room.getId(), area.getId()));
            distances.emplace_back(tmpDis, room.getId(), area.getId());
        }
    }

    std::sort(distances.begin(), distances.end(), 
        [](const RoomAreaDistanceTuple& a, const RoomAreaDistanceTuple& b) {
        return std::get<0>(a) < std::get<0>(b); // 거리를 기준으로 비교
    });

    int priorityScore = 1; // 우선순위 점수 초기값
    for (auto& dist : distances) {        
        int roomID = std::get<1>(dist);
        int areaID = std::get<2>(dist);

        for(CCleanRoom room : rooms) {
            if ( room.getId() == roomID ) {                
                room.setCleanPriority(priorityScore);
                break;
            }
        }

        priorityScore++;
    }

    
    
#if 1   //debug
    for(CCleanRoom room : rooms) {
        ceblog(LOG_LV_NECESSARY, BLUE, "roomId : " << room.getId() 
            << " , 우선 순위 : " << room.getCleanPriority()
            << " , 영역 수량 : "  <<  room.getAreaSize());
    }    
#endif
}

void CCleanPlan::setNoGoZoneDocking(tPoint org, double bound)
{
    CRobotKinematics k;

    noGoZoneDockingCoord = org;
    
    noGoZoneDocking.push_back(k.translate(org, bound       , bound));
    noGoZoneDocking.push_back(k.translate(org, bound * -1  , bound));
    noGoZoneDocking.push_back(k.translate(org, bound * -1  , bound * -1));
    noGoZoneDocking.push_back(k.translate(org, bound       , bound * -1));
}

void CCleanPlan::enableNogoZoneDocking()
{
    bNoGoZoneSetup = true;
}

void CCleanPlan::disableNogoZoneDocking()
{
    bNoGoZoneSetup = false;
}

std::list<tPoint> CCleanPlan::getNoGoZoneDocking(){
    return noGoZoneDocking;
}

tPoint CCleanPlan::getNoGoZoneDockingCoord(){
    return noGoZoneDockingCoord;
}

bool CCleanPlan::isSetupNogoZoneDocking(){
    return bNoGoZoneSetup;
}

/**
 * @brief 지도가 완성된 후 청소 계획을 만드는 함수
 * 
 * 폴리곤 인덱싱.
    3-----------2
    |           |
    |           |
    |           |
    0-----------1
 * true  : plan 성공
 * false : plan 없음
 */
void CCleanPlan::makePlan(tPose robotPose)
{    
    CStopWatch __debug_sw;
    eblog(LOG_LV_NECESSARY,  " 청소영역계획 시작");

    // robot 위치로 부터 우선 순위 정렬
    sortPriorityByRobotPose(robotPose);

    updateCurrentId();

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 해당 room안에 area 중에 point에서 가까운 area ID를 반환한다.
 * jhnoh, 22.12.01
 * @param point 
 * @return int 
 */
int CCleanPlan::getNearestAreaId(tPoint point, int roomId)
{
    CStopWatch __debug_sw;
    
    double dis = 50000.0;
    double tmpDis;
    int id = -1;

    // room안에 area의 중심과의 거리를 계산하여 비교.
    for (CCleanArea area : findRoom(roomId).getAreas())
    {
        tmpDis = utils::math::distanceTwoPoint(point, getAreaCenterPoint(roomId, area.getId()));
        
        if( dis > tmpDis)
        {
            dis = tmpDis;
            id  = area.getId();
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return id;
}
/**
 * @brief 몆적을 구하는 함수
 * icbaek, 22.10.20
 * TODO : 폴리곤을 입력 받아 면적이 나오도록 수정 해야함.
 * @return double 면적 값. 단위 : 제곱 미터
 */
double CCleanPlan::calcRoomAreaSize()
{
    return ROOM_SIZE * ROOM_SIZE;
}


bool CCleanPlan::checkInArea(tPoint point)
{
    int roomId = planInfo.currentRoomId;
    int areaId = planInfo.currentAreaId;

    std::list<tPoint> polygons = getPolygon(roomId, areaId);
    bool bInside = utils::area::isInside(point, polygons);

    return bInside;
}



/**
 * @brief getNearConerPoint warper
 * jhnoh, 23.01.03
 * @param robotPose  
 * @return tPoint 
 */
tPoint CCleanPlan::getNearConerPoint(tPose robotPose, double margin)
{
    return getNearConerPoint(robotPose, planInfo.currentRoomId, planInfo.currentAreaId, margin);
}


/**
 * @brief getNearConerPoint warper
 * jhnoh, 23.01.03
 * @param robotPose  
 * @return tPoint 
 */
tPoint CCleanPlan::getNearConerPoint(tPose robotPose, double margin, const std::list<int> &checkPoints)
{
    return getNearConerPoint(robotPose, planInfo.currentRoomId, planInfo.currentAreaId, margin, checkPoints);
}

/**
 * @brief 로봇 위치에서 해당 area의 4개의 폴리건 중에 가장 가까운 포인트 반환.
 *        (checkPoints)를 통해 다녀간 곳의 폴리건은 목적지 계산 중제외 한다.
 *        margin > 0  :: area 확장
 *        margin < 0  :: area 축소
 * jhnoh, 23.01.03
 * @param robotPose  
 * @return tPoint 
 */
tPoint CCleanPlan::getNearConerPoint(tPose robotPose, int roomId, int areaId, double margin,const std::list<int> &checkPoints)
{
    CStopWatch __debug_sw;
    
    tPoint nearPoint;
    double dis;
    std::list<tPoint> polygons = getAreaMorphology(roomId, areaId, margin);
    if(checkPoints.empty()) 
    {
        eblog(LOG_LV_LINECLEAN, " checkPoints.size() == 0 ");
    }
    int cnt = checkPoints.front();
    bool bPassPoint = false;

    // 확인할 포인트의 첫번째 포인트로 폴리건을 세팅.
    for(int i =0; i <= checkPoints.front(); i++)
    {
        nearPoint = polygons.front();
        dis = utils::math::distanceTwoPoint(polygons.front(), robotPose);
        polygons.pop_front();
    }
    
    // 확인할 포인트들 중의 가장 가까운 거리를 구한다.
    for(tPoint polygon : polygons)
    {
        cnt++;
        double aD = utils::math::distanceTwoPoint(polygon, robotPose);
        
        // 확인할 포인트들만 확인.
        for(int num : checkPoints)
        {
            if(num == cnt)
            {
                bPassPoint = true;
            }
        }
        if(bPassPoint)
        {
            if(dis>aD)
            {
                dis = aD;
                nearPoint = polygon;
            }
            bPassPoint = false;
        }
    }


    TIME_CHECK_END(__debug_sw.getTime());
    return nearPoint;
}

/**
 * @brief 현재 로봇 위치에서 해당 area의 4개의 폴리건 중에 가장 가까운 포인트 반환.
 *        margin > 0  :: area 확장
 *        margin < 0  :: area 축소
 * jhnoh, 23.01.03
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanPlan::getNearConerPoint(tPose robotPose, int roomId, int areaId, double margin)
{
    CStopWatch __debug_sw;
    
    tPoint nearPoint;
    std::list<tPoint> polygons = getAreaMorphology(roomId, areaId, margin);

    double dis = utils::math::distanceTwoPoint(polygons.front(), robotPose);
    nearPoint = polygons.front();
    polygons.pop_front();
    
    // 폴리건 중 가장 가까운 포인트를 구한다.
    for(tPoint polygon : polygons)
    {
        double aD = utils::math::distanceTwoPoint(polygon, robotPose);
        
        if(dis>aD)
        {
            dis = aD;
            nearPoint = polygon;
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return nearPoint;
}

/**
 * @brief 로봇이 현재 청소중인 AREA에 polygons을 기준으로 중심 point를 반환 ( getAreaCenterPoint warper )
 * jhnoh, 23.01.03
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaCenterPoint()
{
    return getAreaCenterPoint(planInfo.currentRoomId, planInfo.currentAreaId);
}

int CCleanPlan::getCurrentRoomId()
{
    return planInfo.currentRoomId;
}

/**
 * @brief 로봇이 현재 청소중인 AREA에 polygons을 기준으로 중심 point를 반환
 * jhnoh, 23.01.03
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaCornerPoint(int idx)
{
    return findArea(planInfo.currentRoomId,planInfo.currentAreaId).getAreaCornerPoint(idx);
}

/**
 * @brief area에서 polygons의 중심 point를 계산 후 반환
 * jhnoh, 22.11.08
 * 
 * @param robotPose 
 * @param roomId 
 * @param areaId 
 * @return tPoint 
 */
tPoint CCleanPlan::getAreaCenterPoint(int roomId, int areaId)
{
    CStopWatch __debug_sw;
    
    tPoint centerPoint;
    centerPoint.x=0;
    centerPoint.y=0;
    if( roomId == -1 || areaId == -1)
    {
        eblog(LOG_LV_NECESSARY, " roomId == -1 || areaId == -1 ");
        return centerPoint;
    }
    std::list<tPoint> polygons = getPolygon(roomId,areaId);
    for(tPoint polygon : polygons)
    {
        centerPoint.x += polygon.x;
        centerPoint.y += polygon.y;
    }
    centerPoint.x = centerPoint.x / polygons.size();
    centerPoint.y = centerPoint.y / polygons.size();
    
    TIME_CHECK_END(__debug_sw.getTime());
    return centerPoint;
}

/**
 * @brief getAreaMorphology warper 함수
 * jhnoh, 22.12.05
 * @param margin 
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanPlan::getAreaMorphology(double margin)
{
    return getAreaMorphology(planInfo.currentRoomId, planInfo.currentAreaId, margin);
}

/**
 * @brief area에서 일정 길이만큼 area의 범위를 증가시키거나 감소 (polygon들의 좌표를 이동시켜 반환.)
 *        | margin > 0  :: area 확장 |
 *        | margin < 0  :: area 축소 |
 * jhnoh, 22.12.05
 * @param roomId room의 고유 Id
 * @param areaId area의 고유 Id
 * @param margin 모폴리지 마진
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanPlan::getAreaMorphology(int roomId, int areaId, double margin)
{
    CStopWatch __debug_sw;
    
    std::list<tPoint> test;
    if( roomId == -1 || areaId == -1)
    {
        eblog(LOG_LV_LINECLEAN, " roomId  == -1 || areaId == -1 ");
        
        TIME_CHECK_END(__debug_sw.getTime());
        return test;
    }

    std::list<tPoint> changedPolygons;
    std::list<tPoint> polygons = getPolygon(roomId, areaId);
    tPoint centerPoint = getAreaCenterPoint(roomId, areaId);
    int cnt = 0;

    for (tPoint polygon : polygons)
    {
        if(polygon.x > centerPoint.x)
        {
            polygon.x += margin;
        }
        else
        {
            polygon.x -= margin;
        }

        if(polygon.y > centerPoint.y)
        {
            polygon.y += margin;
        }
        else
        {
            polygon.y -= margin;
        }
        changedPolygons.emplace_back(polygon);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return changedPolygons;
}


