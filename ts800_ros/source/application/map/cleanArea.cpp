#include "cleanArea.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CCleanArea::CCleanArea() :
    id(-1),
    cleanState(E_CLEAN_STATE::NOTVALID),
    bCleanEnable(false),
    offsetDeg(0.0)
{
    CStopWatch __debug_sw;
    
    // id = -1;
    // cleanState = E_CLEAN_STATE::NOTVALID;
    // bCleanEnable = false;
    // offsetDeg = 0;
    //eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CCleanArea::CCleanArea(int _id, E_CLEAN_STATE _state)
{
    CStopWatch __debug_sw;
    
    setId(_id);
    setCleanState(_state);
    bCleanEnable = false;
    offsetDeg = 0;
    //eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CCleanArea::~CCleanArea()
{
    CStopWatch __debug_sw;
    
    polygon.clear();
    //eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Get the polygon 개수 
 * icbaek, 22.10.20
 * @return int : polygon 개수
 */
int CCleanArea::getPolygonSize(){
    return polygon.size();
}

/**
 * @brief area를 구성하는 폴리곤 좌표를 추가한다. 
 * area를 구성하기위해서는 최소 3개의 포인트가 필요하다.
 * icbaek, 22.10.20
 * @param add 
 */
void CCleanArea::addPolygon(tPoint add){
    polygon.emplace_back(add);
}

void CCleanArea::setPolygon(std::list<tPoint> &set)
{
    polygon = set;

}

void CCleanArea::setContour(std::vector<cv::Point> &set)
{
    contour = set;
}


std::vector<cv::Point> CCleanArea::getContour()
{
    return contour;
}

/**
 * @brief Get the Id object area의 고유 아이디 반환
 * icbaek, 22.10.20
 * @return int 
 */
int CCleanArea::getId(){
    return id;
}

/**
 * @brief Set the Id object
 * icbaek, 22.10.20
 * @param set 
 */
void CCleanArea::setId(int set)
{
    CStopWatch __debug_sw;
    
    id = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Get the Polygon object
 * icbaek, 22.10.20
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanArea::getPolygon(){
    return polygon;
}

/**
 * @brief Set the Robot Begin Pose object
 * icbaek, 22.10.20
 * @param set 
 */
void CCleanArea::setRobotBeginPose(tPose set)
{
    CStopWatch __debug_sw;
    
    robotBeginPos = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Get the Robot Begin Pose object
 * icbaek, 22.10.20
 * @return tPose 
 */
tPose CCleanArea::getRobotBeginPose()
{
    CStopWatch __debug_sw;
    
    tPoint point;
    point =polygon.front();
    tPose pose;
    pose.x = point.x;
    pose.y = point.y;

    TIME_CHECK_END(__debug_sw.getTime());
    return pose;
}

/**
 * @brief Get the Clean State object
 * icbaek, 22.10.20
 * @return E_CLEAN_STATE 
 */
E_CLEAN_STATE CCleanArea::getCleanState()
{
    return cleanState;
}

/**
 * @brief Set the Clean State object
 * icbaek, 22.10.20
 * @param set 
 */
void CCleanArea::setCleanState(E_CLEAN_STATE set)
{
    CStopWatch __debug_sw;
    
    cleanState = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 폴리건 중 가장 왼쪽 y 좌표값.
 * icbaek, 22.10.20
 * @return double 
 */
double CCleanArea::getLeftYcoord()
{
    CStopWatch __debug_sw;
    
    std::vector<double> tempv;
    for ( tPoint& point : polygon )
        tempv.emplace_back(point.x);

    double minX  = *std::min_element(tempv.begin(), tempv.end());

    for ( tPoint& point : polygon )
    {
        if (point.x == minX)
            return point.y;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return 0.0;
}

/**
 * @brief 폴리건 중 가장 오른쪽 y 좌표값.
 * icbaek, 22.10.20
 * @return double 
 */
double CCleanArea::getrightYcoord()
{
    CStopWatch __debug_sw;
    
    std::vector<double> tempv;
    for ( tPoint& point : polygon )        
        tempv.emplace_back(point.x);

    double maxX  = *std::max_element(tempv.begin(), tempv.end());

    for ( tPoint& point : polygon )
    {
        if (point.x == maxX)
            return point.y;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return 0.0;
}

/**
 * @brief Get the Area Up End Point object
 * 라인클린중 area의 끝 좌표
 * icbaek, 22.10.20
 * TODO: 로봇의 좌표를 이용하여 area의 끝 좌표를 만들어야 함
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanArea::getAreaUpEndPoint()
{
    CStopWatch __debug_sw;
    
    tPoint endPoint;

    TIME_CHECK_END(__debug_sw.getTime());
    return endPoint;
}

/**
 * @brief Get the Area Down End Point object
 * 라인클린중 area의 끝 좌표
 * icbaek, 22.10.20
 * TODO: 로봇의 좌표를 이용하여 area의 끝 좌표를 만들어야 함
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanArea::getAreaDownEndPoint()
{
    CStopWatch __debug_sw;
    
    tPoint endPoint;

    TIME_CHECK_END(__debug_sw.getTime());
    return endPoint;
}

/**
 * @brief Get the Area Left End Point object
 * 라인클린중 area의 끝 좌표
 * icbaek, 22.10.20
 * TODO: 로봇의 좌표를 이용하여 area의 끝 좌표를 만들어야 함
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanArea::getAreaLeftEndPoint()
{
    CStopWatch __debug_sw;
    
    tPoint endPoint;

    TIME_CHECK_END(__debug_sw.getTime());
    return endPoint;
}

/**
 * @brief Get the Area Right End Point object
 * 라인클린중 area의 끝 좌표
 * icbaek, 22.10.20
 * TODO: 로봇의 좌표를 이용하여 area의 끝 좌표를 만들어야 함
 * @param robotPose 
 * @return tPoint 
 */
tPoint CCleanArea::getAreaRightEndPoint()
{
    CStopWatch __debug_sw;
    
    tPoint endPoint;

    TIME_CHECK_END(__debug_sw.getTime());
    return endPoint;
}

tPoint CCleanArea::getAreaCornerPoint(int idx)
{
    CStopWatch __debug_sw;
    
    //TODO : 예외 처리 만들어야함.   
    //std::list<tPoint> coners = polygon;

    tPoint targetPoint(0,0);
    int i = 0;
    for ( tPoint& point : polygon )
    {
        if ( i == idx)
        {
            targetPoint = point;
            break;
        }
        i++;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return targetPoint;    
}


void CCleanArea::clearPolygon()
{
    CStopWatch __debug_sw;
    
    if(!polygon.empty()) polygon.clear();
    
    TIME_CHECK_END(__debug_sw.getTime());
}
