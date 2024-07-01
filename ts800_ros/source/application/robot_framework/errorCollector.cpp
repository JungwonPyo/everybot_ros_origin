#include "errorCollector.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

tErrorInfo::tErrorInfo()
{
    bTrapObstacle = false;
    bBumperCurrent = false;
    bBatteryAdapter = false;
    bBatteryNoset = false;
    bBatteryError = false;
    bWifi = false;
    bLiftCliff = false;
    bCommunication = false;
    bWheel = false;

    bTilt = false;
    bDockingFail = false;

    bMissingLidarData = false;
    bMissingGridmapData = false;
}

CErrorCollector::CErrorCollector() {}

CErrorCollector::~CErrorCollector() {}

void CErrorCollector::setErrorInfo(message_t* pMsg)
{
    bCheckError = true;
    tMsgError* arg = boost::get<tMsgError>(&pMsg->arg);
   
    switch (arg->type)
    {
    case E_ERROR_TYPE::TRAP_OBSTACLE:           info.bTrapObstacle = true;break;
    case E_ERROR_TYPE::BUMPER_CURRENT:          info.bBumperCurrent = true;break;
    case E_ERROR_TYPE::BATTERY_ADAPTER:         info.bBatteryAdapter = true;break;
    case E_ERROR_TYPE::BATTERY_NOSET:           info.bBatteryNoset = true;break;
    case E_ERROR_TYPE::BATTERY_ERROR:           info.bBatteryError = true;break;
    case E_ERROR_TYPE::WIFI:                    info.bWifi = true;break;
    // case E_ERROR_TYPE::ADC_CRITICAL:        info.errorType1 = true;break;
    // case E_ERROR_TYPE::MCU_CRITICAL:        info.errorType1 = true;break;
    // case E_ERROR_TYPE::INTERRUPT_CRITICAL:  info.errorType1 = true;break;
    case E_ERROR_TYPE::COMMUNICATION:           info.bCommunication = true;break;

    // case E_ERROR_TYPE::UNDEFINED:           info.errorType1 = true;break;
    case E_ERROR_TYPE::LIFT_CLIFF:              info.bLiftCliff = true;break;
    // case E_ERROR_TYPE::GYRO_INIT_FAIL:      info.errorType1 = true;break;
    // case E_ERROR_TYPE::TRAP_CLIFF:          info.errorType1 = true;break;
    case E_ERROR_TYPE::WHEEL_CURRENT:           info.bWheel = true;break;
    // case E_ERROR_TYPE::WHEEL_WRONG_MOP:     info.errorType1 = true;break;
    // case E_ERROR_TYPE::NO_MOP_OR_TOF_OBS:   info.errorType1 = true;break;
    case E_ERROR_TYPE::TILT:                    info.bTilt = true;break;

    case E_ERROR_TYPE::DOCKING_FAIL:            info.bDockingFail = true;break;
    // case E_ERROR_TYPE::BATTERY_LOW_CANNOT_MOVE: info.errorType1 = true;break;
    
    case E_ERROR_TYPE::MISSING_LIDAR_DATA:      info.bMissingLidarData = true;break;
    case E_ERROR_TYPE::MISSING_GRIDMAP_DATA:    info.bMissingGridmapData = true;break;

    default:
        ceblog(LOG_LV_ERROR, YELLOW, "Wrong Error Type ["<< enumToString(arg->type) <<"] @@@");
        break;
    }
    ceblog(LOG_LV_ERROR, GREEN, "Error Type ["<< enumToString(arg->type) <<"] @@@");        
}

void CErrorCollector::clearErrorInfo()
{
    info = tErrorInfo();
} 

tErrorInfo CErrorCollector::getErrorInfo()
{
    tErrorInfo ret = info;
    clearErrorInfo();
    return ret;
}

bool CErrorCollector::isErrorDetected()
{
    return bCheckError;
}