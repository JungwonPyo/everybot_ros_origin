#include <cmath>

#include "robotmap.h"
#include "externData/externData.h"
#include "eblog.h"
#include "utils.h"
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRobotTrajectory::CRobotTrajectory() {}
CRobotTrajectory::~CRobotTrajectory() {}

// get
std::list<tPoint> CRobotTrajectory::getCleanedTrajectory()
{
    return cleanedTrajectory;
}

/**
 * @brief 청소된 좌표들 클리어
 * 
 */
void CRobotTrajectory::clearCleanedTrajectory()
{
    cleanedTrajectory.clear();
    eblog(LOG_LV_NECESSARY,  "Cleaned Trajectory clear");
}

/**
 * @brief 청소된 좌표들 저장.
 * 
 * @param cleanedPoint 
 */
void CRobotTrajectory::setCleanedTrajectory(tPoint cleanedPoint)
{
    tPoint lastPoint = cleanedTrajectory.back();

    if (utils::math::distanceTwoPoint(lastPoint, cleanedPoint) > 0.05)
    {
        cleanedTrajectory.push_back(cleanedPoint);
    }
}

/**
 * @brief 청소된 좌표들 저장.
 * 
 * @param cleanedPose 
 */
void CRobotTrajectory::setCleanedTrajectory(tPose cleanedPose)
{
    tPoint lastPoint = cleanedTrajectory.back();
    tPoint cleanedPoint = tPoint(cleanedPose.x, cleanedPose.y);

    if (utils::math::distanceTwoPoint(lastPoint, cleanedPoint) > 0.1)
    {
        cleanedTrajectory.push_back(cleanedPoint);
    }
}
//get
std::list<tPoint> CRobotTrajectory::getExploredTrajectory()
{
    return exploredTrajectory;
}

/**
 * @brief 탐색한 좌표들 클리어
 * 
 */
void CRobotTrajectory::clearExploredTrajectory()
{
    exploredTrajectory.clear();
}
/**
 * @brief 탐색한 좌표들 저장.
 * 
 * @param cleanedPose 
 */
void CRobotTrajectory::setExploredTrajectory(tPoint exploredPoint)
{
    tPoint lastPoint = exploredTrajectory.back();

    if (utils::math::distanceTwoPoint(lastPoint, exploredPoint) > 0.05)
    {
        exploredTrajectory.push_back(exploredPoint);
    }
}
/**
 * @brief 탐색한 좌표들 저장.
 * 
 * @param cleanedPose 
 */
void CRobotTrajectory::setExploredTrajectory(tPose exploredPose)
{
    tPoint lastPoint = exploredTrajectory.back();
    tPoint exploredPoint = tPoint(exploredPose.x, exploredPose.y);

    if (utils::math::distanceTwoPoint(lastPoint, exploredPoint) > 0.05)
    {
        exploredTrajectory.push_back(exploredPoint);
    }
}

CRobotMap::CRobotMap()
{
    debug_showRoom = false;
}

CRobotMap::~CRobotMap()
{
    
}

void CRobotMap::update(CExternData* pExternData)
{
    CStopWatch __debug_sw;
    
    s8 *pGridMapRaw = NULL;    
    tGridmapInfo gInfo;

    // grid map 업데이트
    if ( pExternData->rosData.gridMapRaw.isUpdate() )
    {
#if USE_SAVEDMAP
        if(pExternData->rosData.gridmap.copySavedGridMapFromYAML(tempCells, &gInfo))
        {
            cells.updateCells(tempCells, gInfo);
            gridMap.set(tempCells, gInfo);
            pExternData->rosData.gridmap.setUpdateState(false);
        }
        else if( pExternData->rosData.gridmap.copyGridMap(tempCells, &gInfo) )
        {
            cells.updateCells(tempCells, gInfo);
            gridMap.set(tempCells, gInfo);
            pExternData->rosData.gridmap.setUpdateState(false);
        }
#else        
        if( pExternData->rosData.gridMapRaw.copyGridMap(pGridMapRaw, &gInfo))
        {
            //심플리파이 업데이트
            simplifyMap.set(pGridMapRaw, gInfo);           
       
            
            pExternData->rosData.gridMapRaw.setUpdateState(false);
        }
        else
        {
            eblog(LOG_LV,  "Cannot Copy GridMap to Cleanmap "); //gridMap Copy Fail
        }
#endif
    }

    if ( pExternData->rosData.trajectory.isUpdate())
    {
        robotTrajectory.trajectory.set(pExternData->rosData.trajectory.get());
        pExternData->rosData.trajectory.setUpdateState(false);
    }

    if (pGridMapRaw != NULL)
        delete[] pGridMapRaw;

    TIME_CHECK_END(__debug_sw.getTime());
}


