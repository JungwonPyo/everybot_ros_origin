#include "errorHandler.h"
#include "service/service.h"
#include "eblog.h"
#include "control/control.h"
#include "define.h"
#include "utils.h"
#include "MessageHandler.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CErrorHandler::CErrorHandler(/* args */)
{
    lastErrorType = E_ERROR_HANDLING_TYPE::NONE;
    bErrorActive = false;

    __debugPrintCnt = 0;
}

CErrorHandler::~CErrorHandler() {}

bool CErrorHandler::procError(E_ERROR_HANDLING_TYPE errCmd, E_KEY_TYPE key, bool extPower)
{
    bool ret = false;
    CStopWatch __debug_sw;
    E_SoundClass errorSound = SOUND_CTR.getDisireSound();
    if (isErrorActive() == true) // 에러가 활성화 되어 있는 경우 -> 에러해제를 위한 키를 확인함.
    {
        if(MOTION.isRunning()){
            MOTION.startStopOnMap(tProfile(),true);
            
        }
        if (__debugPrintCnt++%1500 == 0){
            ceblog(LOG_LV_ERROR, RED, "@@@@@  Plz Push Button  for  Deactivate Error  @@@@@");
        }
        if (__debugPrintCnt % 3000 == 0){ // 음성 등등 재알림 대기 시간.. count에서 시간으로 바꿀 예정
            ceblog(LOG_LV_ERROR, RED, "@@@@@  에러 알림~~~  @@@@@");
            SOUND_CTR.soundPlay(true,errorSound);
        }

        if(checkKeyForErrorDeactivate(key,extPower)) ret = true;
    }
    else // 에러가 비활성화 되어 있는 경우 -> 에러 처리 및 에러 활성화.
    {
        /* 치명적인 에러 -> 휠 멈추고, 서비스 종료 */
        if(errCmd != E_ERROR_HANDLING_TYPE::NONE){
            ceblog(LOG_LV_ERROR, RED, "@@@@@ DETECT ERROR!! TYPE : " << enumToString(errCmd));
            ROBOT_CONTROL.reportAwsStatus(14);
            if(MOTION.isRunning()){
                MOTION.startStopOnMap(tProfile(),true);
            }
        }
        switch (errCmd)
        {
        case E_ERROR_HANDLING_TYPE::CONTINUOUS_CLIFF:
            handleContinuousCliffError(); //exit
            handleExitSlam();
#if defined (USE_SLAM_MAP_LOAD_RELOCAL) && (USE_SLAM_MAP_LOAD_RELOCAL == 1) //cheol , 24.04.03 리프팅 됐을때 지도가 틀어져서 해결방법 전까지는 지운다. //for_check
            //현재 지도 데이터는 실시간으로 저장하지 않음.
            if ( ROBOT_CONTROL.slam.isExistedSlamMap())
            {
                ServiceData.mapStorage.resetChargerPose();
                //removeSlamMapFile-> arg is true : temp map remove & false : saved map remove
                bool nResult = ROBOT_CONTROL.slam.removeSlamMapFile(false);                
            } 
            else
            {
                eblog(LOG_LV_NECESSARY, " 지도가 존재하지 않습니다");
            }
#endif
            activateError(errCmd);
            break;
        case E_ERROR_HANDLING_TYPE::TILT:
            handleTiltError(); //???
            activateError(errCmd);
            break;
        case E_ERROR_HANDLING_TYPE::WHEEL_ENCODER:
            handleWheelError();
            activateError(errCmd);

            //TRAP_OBSTACLE 이거 에러도 만들어야함

            break;
        case E_ERROR_HANDLING_TYPE::TRAP_OBSTACLE:
            handleTrapError();
            activateError(errCmd);

            //TRAP_OBSTACLE 이거 에러도 만들어야함
            break;
        case E_ERROR_HANDLING_TYPE::MISSING_ROS_DATA:
            handleMissingRosDataError(); //???
            activateError(errCmd);
            break;
        case E_ERROR_HANDLING_TYPE::COMMUNICATION:
            handleCommunicationError();
            activateError(errCmd);
            break;    
        case E_ERROR_HANDLING_TYPE::DOCKING_FIND:
            // 충전기를 찾을 수 없습니다. 충전기로 옮겨주세요.
            // 김지민 : 충전기를 찾으러 갔는데 충전기 어댑터가 꽂혀있지 않은 경우 충전기가 불량인 경우, 충전기 주변에 장애물이 있어 막힌 경우
            // break;
        case E_ERROR_HANDLING_TYPE::DOCKING_FAIL:
            // 충전기로 이동할 수 없습니다. 충전기로 옮겨주세요.
            handleDockingFailError(); //exit
            handleExitSlam();
            activateError(errCmd);
            break;

        // bDockingFail
        case E_ERROR_HANDLING_TYPE::NONE:
            /* handle error */
            activateError(errCmd);
            break;
        default:
            break;
        }
    }
    TIME_CHECK_END(__debug_sw.getTime());

    return ret;
}

/**
 * @brief 에러가 활성화 되어 있는지 확인.
 */
bool CErrorHandler::isErrorActive()
{
    return bErrorActive;
}

/**
 * @brief 에러가 활성화.
 */
void CErrorHandler::activateError(E_ERROR_HANDLING_TYPE type)
{
     // 에러발생시 즉시 로봇의 움직임을 멈춘다.
    bErrorActive = true;
    lastErrorType = type;
}

/**
 * @brief 에러가 비활성화.
 */
void CErrorHandler::deactivateError()
{
    bErrorActive = false;
    __debugPrintCnt = 0;
    LED_CTR.pause(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_START);
}

/**
 * @brief 연속 낙하 감지 에러 조건
 * 
 * @param errInf 감지 에러 종류
 * @param pContext 서비스 상태를 알기 위함.
 */
bool CErrorHandler::isContinuousCliffError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bLiftCliff && ((serviceId == E_SERVICE_ID::CLEAN) || (serviceId == E_SERVICE_ID::DOCKING) \
    || (serviceId == E_SERVICE_ID::EXPLORER) || (serviceId == E_SERVICE_ID::UNDOCKING) \
    || (serviceId == E_SERVICE_ID::REDOCKING)))
    { 
        ret = true;
    }

    return ret;
}

/**
 * @brief 통신 불가 에러. 시스템 에러 이므로 서비스와 관계 없이 에러를 출력한다.
 * 사용자 해결 불가 에러.
 * 
 * @param errInf 
 * @param pContext 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2023-01-26
 * @author hhryu
 */
bool CErrorHandler::isCommunicationError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bCommunication)
    { 
        ret = true;
    }

    return ret;
}
bool CErrorHandler::isWheelError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bWheel)
    { 
        ret = true;
    }

    return ret;
}
bool CErrorHandler::isTrapError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bTrapObstacle)
    { 
        ret = true;
    }

    return ret;
}
/**
 * @brief 틸팅 에러 조건
 * 
 * @param errInf 감지 에러 종류
 * @param pContext 서비스 상태를 알기 위함.
 */
bool CErrorHandler::isTiltError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bTilt)
    { 
        ret = true;
    }

    return ret;
}

bool CErrorHandler::isDockinFailError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();

    if (errInf.bDockingFail)
    { 
        ret = true;
    }

    return ret;
}

/**
 * @brief ROS 데이터 미수신 에러 조건
 * 
 * @param errInf 
 * @param pContext
 */
bool CErrorHandler::isMissingRosDataError(tErrorInfo errInf, CRsuContext *pContext)
{
    bool ret = false;
    E_SERVICE_ID serviceId = pContext->getServiceId();
    
    if ((errInf.bMissingLidarData || errInf.bMissingGridmapData) && ((serviceId == E_SERVICE_ID::CLEAN) || (serviceId == E_SERVICE_ID::DOCKING) || (serviceId == E_SERVICE_ID::EXPLORER)
    || (serviceId == E_SERVICE_ID::REDOCKING)))
    {
        if(pContext->getServiceReadyState() == E_SERVICE_READY::COMPLETE)
        {
            //lidar를 생활 물품으로 가릴 경우 무한대 데이터 발생 기준으로 발생(장착에 의한 무한대 데이터 70~90 정도는 발생함)
            if (errInf.bMissingLidarData)
            {
                ceblog(LOG_LV_ERROR, YELLOW, "Missing Ros Lidar Data Error !!! - Service Id ["<< enumToString(serviceId)<<"]");
                ret = true;
            }
            #if 0 //not used
            else if (errInf.bMissingGridmapData)
            {
                ceblog(LOG_LV_ERROR, YELLOW, "Missing Ros Gridmap Data Error !!! - Service Id ["<< enumToString(serviceId)<<"]");
                ret = true;
            }
            #endif
        }
    }

    return ret;
}

/**
 * @brief 특정 키 값이 들어오는 경우, 애러를 해제한다.
 * ( 우선 모든 키에 대해서 에러를 해제)
 * @param key 
 */
bool CErrorHandler::checkKeyForErrorDeactivate(E_KEY_TYPE key, bool extPower)
{
    bool ret = false;
    if(key != E_KEY_TYPE::VOID || extPower)
    {
        ceblog(LOG_LV_ERROR, GREEN, "Error Deactivate by key[" << enumToString(key) << "]");
        deactivateError();
        ret = true;
    }
    return ret;
}

void CErrorHandler::handleExitSlam()
{
    if ( ROBOT_CONTROL.slam.isSlamRunning()) 
    {
        ceblog(LOG_LV_NECESSARY, RED, "handleExitSlam::SLAM 종료 @@@");
        ROBOT_CONTROL.slam.exitSlam();
    }
}
