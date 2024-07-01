
#include "cleanRoom.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CCleanRoom::CCleanRoom()
{
    CStopWatch __debug_sw;
    
    cleanState = E_CLEAN_STATE::UNCLEAN;
    bCleanEnable = false;
    cleanedAreaSize = 0.0;
    id = -1;
    
    TIME_CHECK_END(__debug_sw.getTime());
}
CCleanRoom::~CCleanRoom()
{
    CStopWatch __debug_sw;
    
    areas.clear();
    areaPlan.clear();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCleanRoom::setAreaCleanState(int areaId, E_CLEAN_STATE set)
{
    CStopWatch __debug_sw;
    
    for(CCleanArea& area : areas)
    {
        if(areaId == area.getId())
        {
            area.setCleanState(set);
        }
    }
    eblog(LOG_LV,  "");

    TIME_CHECK_END(__debug_sw.getTime());
}
void CCleanRoom::clearAreas()
{
    CStopWatch __debug_sw;
    
    for(CCleanArea& area : areas)
    {
        area.clearPolygon();
    }
    if(!areas.empty()) areas.clear();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCleanRoom::clearAreaPlan()
{
    CStopWatch __debug_sw;
    
    if(!areaPlan.empty()) areaPlan.clear();
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Get the Area 개수 
 * icbaek, 22.10.20
 * @return int : area 개수
 */
int CCleanRoom::getAreaSize()
{
    return areas.size();
}

/**
 * @brief Get the Id object
 * icbaek, 22.10.20
 * @return int : id
 */
int CCleanRoom::getId()
{
    return id;
}

/**
 * @brief Set the Id object
 * icbaek, 22.10.20
 * id 0 부터 시작 -1 은 할당 안됨.
 * @param set 
 */
void CCleanRoom::setId(int set)
{
    CStopWatch __debug_sw;
    
    id = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCleanRoom::setCleanPriority(int set)
{
    CStopWatch __debug_sw;
    
    cleanPriority = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

int CCleanRoom::getCleanPriority()
{
    return cleanPriority;
}

/**
 * @brief area의 청소계획을 설정
 * jhnoh, 22.12.05
 * @param areaId 
 */
void CCleanRoom::setPlan(int areaId)
{
    CStopWatch __debug_sw;
    
    areaPlan.emplace_back(areaId);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
* @brief  area 청소 계획을 반환
* jhnoh, 22.12.05
* @return std::list<int> 
*/
std::list<int> CCleanRoom::getPlanList()
{
    return areaPlan;
}

/**
 * @brief Set the Clean Enable object
 * icbaek, 22.10.20
 * @param bSet 
 */
void CCleanRoom::setCleanEnable(bool bSet){
    bCleanEnable = bSet;
}

/**
 * @brief Get the Clean State object
 * icbaek, 22.10.20
 * @return E_CLEAN_STATE 
 */
E_CLEAN_STATE CCleanRoom::getCleanState(){
    return cleanState;
}

/**
 * @brief Set the Clean State object
 * icbaek, 22.10.20
 * @param set 
 */
void CCleanRoom::setCleanState(E_CLEAN_STATE set)
{
    CStopWatch __debug_sw;
    
    cleanState = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief area를 추가한다.
 * icbaek, 22.10.20
 * @param area 
 */
void CCleanRoom::addArea(CCleanArea area)
{
    CStopWatch __debug_sw;
    
    areas.emplace_back(area);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 폴리곤 추가
 * icbaek, 22.10.20
 * @param targetAreaId 
 * @param polygon 
 * @return true : 추가 됨.
 * @return false : 추가 안됨.
 */
bool CCleanRoom::addPolygon(int targetAreaId, tPoint polygon)
{
    CStopWatch __debug_sw;
    
    bool bRet = false;        
    for ( CCleanArea& area : areas )
    {
        if ( area.getId() == targetAreaId )
        {
            area.addPolygon(polygon);
            bRet = true;
            break;
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

bool CCleanRoom::setPolygon(int targetAreaId, std::list<tPoint> &set)
{
    CStopWatch __debug_sw;
    
    bool bRet = false;        
    for ( CCleanArea& area : areas )
    {
        if ( area.getId() == targetAreaId )
        {
            area.setPolygon(set);
            bRet = true;
            break;
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

bool CCleanRoom::setContour(int targetAreaId, std::vector<cv::Point> &set)
{
    CStopWatch __debug_sw;
    
    bool bRet = false;        
    for ( CCleanArea& area : areas )
    {
        if ( area.getId() == targetAreaId )
        {
            area.setContour(set);
            bRet = true;
            break;
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}


/**
 * @brief Get the Areas object
 * icbaek, 22.10.20
 * @return std::list<CCleanArea> 
 */
std::list<CCleanArea> CCleanRoom::getAreas()
{
    return areas;
}

/**
 * @brief Get the Polygons object
 * icbaek, 22.10.20
 * @param targetAreaId 
 * @return std::list<tPoint> 
 */
std::list<tPoint> CCleanRoom::getPolygons(int targetAreaId)
{
    CStopWatch __debug_sw;
    
    for ( CCleanArea& area : areas )  // 
    {
        if ( area.getId() == targetAreaId )
        {
            return area.getPolygon();
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

