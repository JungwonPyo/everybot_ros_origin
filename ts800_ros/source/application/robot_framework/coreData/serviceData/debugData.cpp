#include "coreData/serviceData/debugData.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CDebugData::CDebugData()
{
    pidErrorData.dummy1 = 0;
    pidErrorData.dummy2 = 0;
    pidErrorData.errP   = 0;
    pidErrorData.errI   = 0;
    pidErrorData.errD   = 0;

    
    bUpdateExplorer = false;
    eblog(LOG_LV,  ""); 
}

CDebugData::~CDebugData()
{
    eblog(LOG_LV,  "");
}

/*---------------------------------- explorer ----------------------------------*/
void CDebugData::explorerDataClear()
{
    explorerData.frontiers.clear();
}

void CDebugData::setExplorerDebug(const tExplorerDebug &set)
{ 
    explorerDataClear();
    explorerData = set;
}

tExplorerDebug CDebugData::getExplorerDebug()
{
    return explorerData;
}




void CDebugData::setUpdateExplorer(const bool set){
    bUpdateExplorer = set;
}



/*----------------------------------- docking -----------------------------------*/

// void CDebugData::setDockingDebug(const tPathplanDebug &set)
// { 
//     dockingDataclear();
//     dockingData = set;
// }

// tPathplanDebug CDebugData::getDokingDebug()
// {
//     return dockingData;
// }

bool CDebugData::getUpdateDocking()
{
    return bUpdateDocking;
}

void CDebugData::setUpdateDocking(bool set)
{
    bUpdateDocking = set;
}


/*----------------------------------- area -----------------------------------*/
void CDebugData::cleanAreaDataclear()
{
    for(CCleanRoom& room : cleanAreaData.rooms)
    {        
        room.clearAreas();
    }
    cleanAreaData.currentAreaPolygons.clear();
    cleanAreaData.rooms.clear();
}

void CDebugData::setCleanAreaDebug(const tCleanAreaDebug &set)
{ 
    cleanAreaDataclear();
    cleanAreaData = set;
}

tCleanAreaDebug CDebugData::getCleanAreaDebug()
{
    return cleanAreaData;
}

void CDebugData::setCleanPathDebug(const std::list<tPose> &set)
{
    if(cleanPathData.empty()== false)
    {
        cleanPathData.clear();
    }
    cleanPathData = set;

}
std::list<tPose> CDebugData::getCleanPathDebug()
{
    return cleanPathData;
}

/*----------------------------------- pid error -----------------------------------*/
void CDebugData::clearPidErrorData()
{
    pidErrorData = tPidErrorDebug();
}

void CDebugData::setPidErrorData(const tPidErrorDebug &set)
{ 
    clearPidErrorData();
    pidErrorData = set;
}

tPidErrorDebug CDebugData::getPidErrorData()
{
    return pidErrorData;
}