#include "pathFinderInterface.h"
#include "coreData/serviceData.h"
#include "ebtime.h"
#include "debugCtr.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 1 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 300.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CPathFinderInterface::CPathFinderInterface()
{
    bisRunMapUpdate = false;
    bisNeedMapUpdate = false;
    isRunfindNearestPath = false;
}

/**
 * @brief 소멸자에서 호출 해 줄 것.
 * 
 */
void CPathFinderInterface::destroyPathFinderInterface()
{

}

/**
 * @brief 가장 가까운 경로를 찾는다.
 * 
 * @param robotPose 
 * @param searchingPoints 
 * @param searchingCnt 
 * @return true 
 * @return false 
 */
bool CPathFinderInterface::dofindNearestPath(tPose robotPose, 
    std::list<tPoint> searchingPoints, int searchingCnt )
{
    bool bRet = false;

    if (isRunFindNearestPath() == false)
    {
        clearPath();    
        std::thread(&CPathFinderInterface::findNearestPath, this, 
            robotPose, searchingPoints, searchingCnt).detach();  
        bRet = true;
    }
    else
    {
        bRet = false;
    }   
    
    return bRet;
}

/**
 * @brief 특정 위치의 경로를 찾는다.
 * 
 * @param robotPose 
 * @param searchingPoint 
 * @return true 
 * @return false 
 */
bool CPathFinderInterface::dofindPath(tPose robotPose, tPoint searchingPoint)
{
    std::list<tPoint> searchingPoints;
    searchingPoints.push_back(searchingPoint);
    return dofindNearestPath(robotPose, searchingPoints, 1);
}

/**
 * @brief 찾기 알고리즘이 돌고 있는 중 인가 ?
 * 
 * @return true 
 * @return false 
 */
bool CPathFinderInterface::isRunFindPath()
{
    return isRunFindNearestPath();
}

/**
 * @brief 외부에서 map update 를 할지 말지 판단 하는 flag
 * 
 * @return true 
 * @return false 
 */
bool CPathFinderInterface::isFindPath()
{
    bool bRet = false;

    if ( isRunFindNearestPath() == false &&  //스레딩이 정리 됐고..
        getFindNearSuccess() == true)       // 경로를 찾았다.
    {
        bRet = true;
    }
    return bRet;
}

/**
 * @brief D* map update 상태를 확인한다.
 * 
 * @return true 
 * @return false 
 */
bool CPathFinderInterface::isNeedMapUpdate()
{
    return bisNeedMapUpdate;
}

/**
 * @brief map update 시작
 * 
 */
void CPathFinderInterface::startMapUpdate()
{
    ceblog(LOG_LV_NECESSARY, YELLOW,  "경로 이동 D* 에 map update를 시작 합니다.");
    bisNeedMapUpdate = true;
}

/**
 * @brief map update 종료.
 * 
 */
void CPathFinderInterface::stopMapUpdate()
{
    ceblog(LOG_LV_NECESSARY, YELLOW,  "경로 이동 D* 에 map update를 중지 합니다.");
    bisNeedMapUpdate = false;
}


bool CPathFinderInterface::doUpdateDstarMap(){
    bool bRet = false;

    if (isNeedMapUpdate())
    {
        if (bisRunMapUpdate == false){   
            // ceblog(LOG_LV_NECESSARY, BOLDMAGENTA, "D* 업데이트"); 
            std::thread(&CPathFinderInterface::updateDstarMap, this).detach();  
            bRet = true;
        }
        else{
            ceblog(LOG_LV_NECESSARY, BOLDRED, "D* 업데이트 중이어서 이번 텀은 포기");
            bRet = false;
        }
    }
    
    return bRet;
}


void CPathFinderInterface::updateDstarMap(){
    bisRunMapUpdate = true;
    CStopWatch __debug_sw;
    u8 *pGridmap = nullptr;
    auto mapSet = false;    
    tGridmapInfo mapInfo;
    
    std::cout<<std::fixed<<std::setprecision(4);   

    try
    {   
        // grid map 으로 부터 wall point 를 만든다.
        if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
        {
            ServiceData.robotMap.simplifyMap.simplifyGridMapLock();

            mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
            pGridmap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();

            tPose robotPose = ServiceData.localiz.getPose();
            makeWallListFromGridMap(robotPose, mapInfo, pGridmap);

            ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
            
        }
        updateGridMapWall();            
    }
    catch(const std::exception& e)
    {
        ceblog(LOG_LV_ERROR, BOLDYELLOW, "[Try-Exception] " << e.what());
    }
    TIME_CHECK_END(__debug_sw.getTime());
    
    
    bisRunMapUpdate = false;
}

#if 0 // 업데이트 됐을때반 돌게 수정 하여 안쓸것 같음. 23.04.30, icbaek
/**
 * @brief 경로이동 벽 업데이트 thread
 * jhnoh, 23.08.14
 * 
 */
void CPathFinderInterface::threadPathMapUpdate()
{
    unsigned int milliSecCnt = 0;
    u8 *pGridmap = nullptr;
    auto mapSet = false;    
    tGridmapInfo mapInfo;
    tPose robotPose;
    
    
    std::cout<<std::fixed<<std::setprecision(4);

    while (true)
    {
        CStopWatch __debug_sw;

        try
        {
            // 1. map copy
            if(milliSecCnt%50 == 0)
            {
                if(isNeedMapUpdate())
                {
                    // grid map 으로 부터 wall point 를 만든다.
                    if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
                    {
                        ServiceData.robotMap.simplifyMap.simplifyGridMapLock();

                        mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
                        pGridmap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();

                        robotPose = ServiceData.localiz.getPose();
                        makeWallListFromGridMap(robotPose, mapInfo, pGridmap);

                        ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
                        
                    }
                }
            }
            updateGridMapWall();            
        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, BOLDYELLOW, "[Try-Exception] " << e.what());
        }
        TIME_CHECK_END(__debug_sw.getTime());
        
        milliSecCnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        DEBUG_CTR.isAliveDstarWallUpdate.set(true);    
    }
    
}
#endif