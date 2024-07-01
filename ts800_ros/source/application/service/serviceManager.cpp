
#include <unistd.h>
#include <sys/stat.h>
#include <math.h>

#include "serviceManager.h"
#include "MessageHandler.h"
#include "systemTool.h"
#include "eblog.h"
#include "userInterface.h"
#include "utils.h"
#include "motionPlanner/motionPlanner.h"
#include "debugCtr.h"
#include "subTask.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 2.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

int IsMapFileExist(const char* path) { return !access(path, F_OK); }
u32 slamPauseEndtick = (u32)-1;



CServiceManager::CServiceManager()
{
    CStopWatch __debug_sw;
    
    //로봇의 PASUE STATE 자기 위치 좌표
    pRobotSlamPose = new CRobotSlamPose();

    pLocation   = new CLocation();    
    pKidnap     = new CKidnap(pLocation);

    pSupplyWater        = new CSupplyWater();
    pServiceReady       = new CServiceReady();
    pServcieIdle        = new CServiceIdle(pServiceReady, pSupplyWater, pLocation, pRobotSlamPose, pKidnap);
    pServiceClean       = new CServiceClean(&svcMacro,pServiceReady, pSupplyWater, pLocation ,pKidnap, pRobotSlamPose);
    pServiceExplore     = new CServiceExplorer(&svcMacro,pServiceReady, pRobotSlamPose, pKidnap);
    pServiceDocking     = new CServiceDocking(&svcMacro,pServiceReady, pLocation ,pKidnap, pRobotSlamPose);
    pServiceUnDocking   = new CServiceUnDocking(&svcMacro,pServiceReady, pRobotSlamPose);
    pServiceCharging    = new CServiceCharging(&svcMacro,pServiceReady);
    pServiceWifi        = new CServiceWifi(pServiceReady);

    pContext = new CRsuContext(pServcieIdle,pServiceReady);
    pAwsDataManager     = new CAwsDataManager(pContext);
    pKeyHandler = new CKeyHanler(&svcMacro,pSupplyWater,pServiceCharging->getChargingTaskPointer());
    pDeviceHandler = new CDeviceHandler(&svcMacro);
    bRunPowerOff = false;
    bRunFirmwareUpdate = false;

    checkTime.check = 0;
    checkTime.start = 0;

    checkSlamTime.check = 0;
    checkSlamTime.start = 0;

    bootService();
    eblog(LOG_LV, "");

    TIME_CHECK_END(__debug_sw.getTime());
}

CServiceManager::~CServiceManager()
{
    delete pServiceWifi;
    delete pServiceCharging;
    delete pServiceUnDocking;
    delete pServiceDocking;
    delete pServiceExplore;
    delete pServiceClean;
    delete pServcieIdle;
    delete pSupplyWater;
    delete pLocation;    
    delete pKidnap;
    eblog(LOG_LV, "");
}

void CServiceManager::bootService()
{
    CStopWatch __debug_sw;
    eblog(LOG_LV,  "service booting init");
    clearAllService();
    svcMacro.bootIdle();

    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::clearAllService()
{
    eblog(LOG_LV,  "service change init");
    pServcieIdle->initializeSvc(E_SERVICE_ID::IDLE);
    pServiceClean->initializeSvc(E_SERVICE_ID::CLEAN);
    pServiceExplore->initializeSvc(E_SERVICE_ID::EXPLORER);
    pServiceDocking->initializeSvc(E_SERVICE_ID::DOCKING);
    pServiceUnDocking->initializeSvc(E_SERVICE_ID::UNDOCKING);
    pServiceCharging->initializeSvc(E_SERVICE_ID::CHARGING);
    pServiceWifi->initializeSvc(E_SERVICE_ID::WIFI);
}

void CServiceManager::initService(E_SERVICE_ID id)
{
    CStopWatch __debug_sw;

    switch (id)
    {
    case E_SERVICE_ID::IDLE:
        pServcieIdle->initializeSvc(E_SERVICE_ID::IDLE);
        break;
    case E_SERVICE_ID::CLEAN:
        pServiceClean->initializeSvc(E_SERVICE_ID::CLEAN);
        break;
    case E_SERVICE_ID::CHARGING:
        pServiceCharging->initializeSvc(E_SERVICE_ID::CHARGING);
        break;
    case E_SERVICE_ID::DOCKING:
        pServiceDocking->initializeSvc(E_SERVICE_ID::DOCKING);
        break;
    case E_SERVICE_ID::EXPLORER:
        pServiceExplore->initializeSvc(E_SERVICE_ID::EXPLORER);
        break;
    case E_SERVICE_ID::UNDOCKING:
        pServiceUnDocking->initializeSvc(E_SERVICE_ID::UNDOCKING);
        break;
    case E_SERVICE_ID::REDOCKING:
        // pContext->setService(); 
        break;
    case E_SERVICE_ID::WIFI:
        pDeviceHandler->skipConnectCheck();
        pServiceWifi->initializeSvc(E_SERVICE_ID::WIFI);
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::serviceSelector(E_SERVICE_ID id)
{
    CStopWatch __debug_sw;

    switch (id)
    {
    case E_SERVICE_ID::IDLE:
        pContext->setService(pServcieIdle);
        break;
    case E_SERVICE_ID::CLEAN:
        pContext->setService(pServiceClean);
        break;
    case E_SERVICE_ID::CHARGING:
        pContext->setService(pServiceCharging);
        break;
    case E_SERVICE_ID::DOCKING:
        pContext->setService(pServiceDocking);
        break;
    case E_SERVICE_ID::EXPLORER:
        pContext->setService(pServiceExplore);
        break;
    case E_SERVICE_ID::UNDOCKING:
        pContext->setService(pServiceUnDocking);
        break;
    case E_SERVICE_ID::REDOCKING:
        // pContext->setService(); 
        break;
    case E_SERVICE_ID::WIFI:
        pContext->setService(pServiceWifi);
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::serviceStart(E_SERVICE_ID doId, E_SERVICE_STATUS doStatus)
{
    serviceSelector(doId);
    if(doStatus == E_SERVICE_STATUS::startup){
        //initService(doId);
        pContext->serviceControl(E_SERVICE_CTR::START);
    }
    else if(doStatus == E_SERVICE_STATUS::running){
        pContext->serviceControl(E_SERVICE_CTR::RUN);
    }
   
    svcMacro.popDolist();

    eblog(LOG_LV_NECESSARY, "Service Start id : " << enumToString(doId));
}

void CServiceManager::serviceRun()
{
    if( pContext->getServiceStatus() == E_SERVICE_STATUS::completed && pContext->getServiceStep(E_SERVICE_STATUS::completed) == E_SERVICE_STATUS_STEP::TERMINAITION){
        svcMacro.popDolist();
        eblog(LOG_LV_NECESSARY, "Service Complete id : " << enumToString(pContext->getServiceId()));
    }
}

void CServiceManager::servicePause()
{
    pContext->serviceControl(E_SERVICE_CTR::PAUSE);
    svcMacro.popDolist();
    eblog(LOG_LV_NECESSARY, "Service Pause id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
}

void CServiceManager::ServiceCancle()
{
    if(pContext->getServiceStatus() != E_SERVICE_STATUS::completed){
        pContext->serviceControl(E_SERVICE_CTR::CANCEL);
    }else if(pContext->getServiceStep(E_SERVICE_STATUS::completed) == E_SERVICE_STATUS_STEP::TERMINAITION){
        svcMacro.popDolist();
    }
    
    eblog(LOG_LV_NECESSARY, "Service Cancle id : " << enumToString(pContext->getServiceId()));
}

void CServiceManager::runServiceMacro(const std::list<contextDo>& doList)
{
    E_CONTEXT_DO doAction = doList.front().what;
    E_SERVICE_ID doId = doList.front().id;
    E_SERVICE_STATUS doStatus = doList.front().status;
    switch (doAction)
    {
    case E_CONTEXT_DO::doIt:
        serviceStart(doId,doStatus);
        break;
    case E_CONTEXT_DO::doUntil:
        serviceRun();
        break;
    case E_CONTEXT_DO::doCancle:
        ServiceCancle();
        break;
    case E_CONTEXT_DO::doPause:
        servicePause();
        break;
    case E_CONTEXT_DO::doWait:

        break;              
    default:
        break;
    }
}

void CServiceManager::procAwsReport()
{
#if 0 // 디버깅 및 사용법 코드
    std::string temp = utils::math::getCurrentTimeString(1,2);
    eblog(LOG_LV_NECESSARY,"현재 시간 : " << temp);
    if (temp == "2024-06-06")
    {
        eblog(LOG_LV_NECESSARY,"비교 되네 " );
    }
    else {
        eblog(LOG_LV_NECESSARY,"비교 안되네 " );
    }
#endif
    if(ServiceData.rsBridge.getconnect())
    {
        // if(ServiceData.awsData.isAreaInfoUpdate()){
        // ROBOT_CONTROL.reportAreaInfo();
        // }
        if(ServiceData.awsData.isCleanModeUpdate()){
            ROBOT_CONTROL.reportAwsCleanMode();
        }
        if(ServiceData.awsData.isErrorDataUpdate()){
            ROBOT_CONTROL.reportAwsCleanMode();
        }
        if(ServiceData.awsData.isDryMopDataUpdate()){
            ROBOT_CONTROL.reportAwsDryMopData();
        }
        if(ServiceData.awsData.isWaterLevelUpdate()){
            ROBOT_CONTROL.reportAwsWaterLevel();
        }
        if(ServiceData.awsData.isCountryDataUpdate()){
            ROBOT_CONTROL.reportAwsCountry();
        }
        if(ServiceData.awsData.isSettingDataUpdate()){
            ROBOT_CONTROL.reportRobotSettings();
        }
        if(ServiceData.awsData.isSoundVolumeUpdate()){
            ROBOT_CONTROL.reportAwsSoundLevel();
        }
        if(ServiceData.awsData.isDistruptDataUpdate()){
            ROBOT_CONTROL.reportDistruptMode();
        }
        if(ServiceData.awsData.isLanguageDataUpdate()){
            ROBOT_CONTROL.reportAwsLanguage();
        }
        if(ServiceData.awsData.isRsvCleanDataUpdate()){
            ROBOT_CONTROL.reportReservationClean();
        }
        if(ServiceData.awsData.isForbiddenAreaUpdate()){
            ROBOT_CONTROL.reportForbiddenArea();
        }
        if(ServiceData.awsData.isOperationAreaUpdate()){
            ROBOT_CONTROL.reportOperationArea();
        }
        if(ServiceData.awsData.isBatteryUpdate()){
            ROBOT_CONTROL.reportAwsBattery();
        }
        if(ServiceData.awsData.isDivideAreaUpdate()){
            ROBOT_CONTROL.reportDivideArea();
        }
        if(ServiceData.awsData.isCombineAreaUpdate()){
            ROBOT_CONTROL.reportCombineArea();
        }
        if(ServiceData.awsData.isOtaDataUpdate()){
            ROBOT_CONTROL.reportOtaVersion();
        }
        /* 주기적 report */
        E_SERVICE_ID curId = pContext->getServiceId();
        if((curId == E_SERVICE_ID::EXPLORER || curId == E_SERVICE_ID::CLEAN || curId == E_SERVICE_ID::DOCKING) && pAwsDataManager->checkSendMapTime()){
            ROBOT_CONTROL.reportMapData(curId);
        }
    }

}
void CServiceManager::updateServiceManager(tErrorInfo errInf)
{
    CStopWatch __debug_sw;
    E_ERROR_HANDLING_TYPE err = makeError(errInf);
    E_SERVICE_ID curId = pContext->getServiceId();
    E_SERVICE_STATUS curStatus = pContext->getServiceStatus();
    bool bNeedCharge = ServiceData.battery.isBatteryNeedCharge();
    bool bKeyPowerOff = false, bKeyFirmwareUpdate = false; 

    if(bRunPowerOff){taskPowerOff.taskRun();return;}
    if(bRunFirmwareUpdate){
        taskFwUpdate.taskRun();
        return;
        }

    pKeyHandler->keyChecker(curId,curStatus,bNeedCharge,&bKeyPowerOff,&bKeyFirmwareUpdate);
    if(bKeyPowerOff){bRunPowerOff = true;taskPowerOff.taskStart();return;}
    if(bKeyFirmwareUpdate){bRunFirmwareUpdate = true;taskFwUpdate.taskStart();return;}
    pDeviceHandler->powerChecker(curId,curStatus);
    if ( err != E_ERROR_HANDLING_TYPE::NONE || isErrorActive() == true)
    {
        if(procError(err, ServiceData.key.getKeyValue(),ServiceData.power.getExtPower())){
            if(ServiceData.power.getTerminalState() == E_TERMINAL_STATE::DOCKED)    svcMacro.macroServiceCharge(curId);
            else                                                                    svcMacro.bootIdle();
        }
    }else{
        procService();
    }
    
#if USE_AWS_APP_INTERFACE
    procAwsReport();
#endif   
    timeProcess(curId,curStatus,1*SEC_1);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::procService()
{
    CStopWatch __debug_sw;
    std::list<contextDo> doList = svcMacro.getDoList();
    if(doList.empty()){
        eblog(LOG_LV_NECESSARY, "더 실행할 서비스가 없어요!! 충전기로 이동합니다.");
        //svcMacro.bootIdle();
        svcMacro.macroServiceConnector(E_SERVICE_ID::DOCKING);
    }
    else{
        runServiceMacro(doList);
    }
    pContext->serviceRun();


    //현재 실행되고 있는 서비스 ID 저장
    //NONE =0 , IDLE  = 1, CLEAN = 2, CHARGING, DOCKING = 4, EXPLORER = 5, UNDOCKING, REDOCKING, WIFI,
    ServiceData.tempRunServceID = static_cast<int>(pContext->getServiceId());
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief time 간격으로 무조건 수행하는 함수. 로그 등을 출력하기 위함
 * 
 * @param time : time 간격마다 실행
 * @param bTimeCheck : false면 time체크 안함. (20ms마다 실행)
 */
void CServiceManager::timeProcess(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus,u16 time)
{
    CStopWatch __debug_sw;
    
    checkTime.check = SYSTEM_TOOL.getSystemTick() - checkTime.start;

    if (checkTime.check >= time)
    {
        checkTime.start = SYSTEM_TOOL.getSystemTick();
        pDeviceHandler->batteryChecker(curId,curStatus);
        #if USE_AWS_APP_INTERFACE
        if(curId != E_SERVICE_ID::WIFI){
            pDeviceHandler->wifiConnectChecker();   
        }
        
        #endif
        systemInterfaceErrorCheck();
        __debugPrint();
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Collector에서 set된 에러 종류, 서비스 상태, 우선 순위 등을 고려하여 최종적으로 에러 출력하는 단계.
 * @date 2023/05/23
 * @author hhryu, jspark
 * @param errInf
 * @return E_ERROR_HANDLING_TYPE
 */
E_ERROR_HANDLING_TYPE CServiceManager::makeError(tErrorInfo errInf)
{
    CStopWatch __debug_sw;

    /**
     * @brief 우선 순위가 높을 수록 상단 if로 작성해주세요.
     * ServiceData, pContext->getService(), 
     * 채워야 함.
     */
    E_ERROR_HANDLING_TYPE ret = E_ERROR_HANDLING_TYPE::NONE;
    
    if (isCommunicationError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::COMMUNICATION;
    }
    else if (isTiltError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::TILT;
    }
    /* 연속 낙하 감지 에러 조건 */
    else if (isContinuousCliffError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::CONTINUOUS_CLIFF;
    }
    else if (isWheelError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::WHEEL_ENCODER;
    }
    else if (isTrapError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::TRAP_OBSTACLE;
    }
     /**
     * 신뢰성 구속 테스트 중 라이더를 생활 품(수건,헝겊.옷, 등)으로 
     * 가릴 경우 에러 발생
     */    
    else if ( isMissingRosDataError(errInf, pContext))
    {
         ret = E_ERROR_HANDLING_TYPE::MISSING_ROS_DATA;
    }
    else if (isDockinFailError(errInf, pContext))
    {
        ret = E_ERROR_HANDLING_TYPE::DOCKING_FAIL;
    }

#if TODO_LIST
    if (어떤 체크함수(errInf))
    {
        ret = E_ERROR_HANDLING_TYPE::어떤 체크;
    }
    .
    .
    .
#endif
    if(ret != E_ERROR_HANDLING_TYPE::NONE){
        ServiceData.awsData.setSendError(ret);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 연속 낙하 에러 처리.
 */
void CServiceManager::handleContinuousCliffError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */
    LED_CTR.sensorError(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_RESTART_ON_NORMAL_FLOOR_ERROR);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::FLAT_FLOOR);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Tilt 에러 처리.
 */
void CServiceManager::handleTiltError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */
    LED_CTR.sensorError(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_NEED_TO_TEST_CENTER_REQUEST_ERROR);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::MOTOR_ERROR);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::handleWheelError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */
    LED_CTR.sensorError(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CHECKING_MOP_FLOOR);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::ROTATING_BOARD_PROBLEM);
    
    TIME_CHECK_END(__debug_sw.getTime());
}
void CServiceManager::handleTrapError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */
    LED_CTR.sensorError(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CHECKING_AREA);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::UNABLE_TO_MOVE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief ROS 데이터 미수신 에러 처리.
 */
void CServiceManager::handleMissingRosDataError()
{
    CStopWatch __debug_sw;

    ceblog(LOG_LV_NECESSARY, BOLDRED , "\t" << "라이더 데이터들이 대부분 가려져 있어요.. 라이더를 확인 좀 해주세요" );
    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */
    LED_CTR.sensorError(); 
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CHECKING_LIDAR);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::LIDAR_SENSOR_ERROR);
    
    //일단 로봇 중지만 시키자 
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 시스템 통신 에러.
 * 
 * 
 * @note 연산시간 ms
 * @date 2023-01-26
 * @author hhryu
 */
void CServiceManager::handleCommunicationError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */

    LED_CTR.sensorError();
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_NEED_TO_TEST_CENTER_REQUEST_ERROR);
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::CHECK_THE_SYSTEM);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 도킹 실패 에러..
 */
void CServiceManager::handleDockingFailError()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, " Service end , id : " << enumToString(pContext->getServiceId()) << " , status : " << enumToString(pContext->getServiceStatus()));
    //doList = svcMacro.deactivateServiceFlow(pContext->getServiceId());

    /* UI */

    LED_CTR.chargingMoveError();
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CANT_MOVE_TO_CHARGER);
    //case E_SoundClass::SOUND_CANT_MOVE_TO_CHARGER               : m_pSoundClass[nSoundClassNumber].strName = "/usr/local/data/sound/'29. 충전기를 찾을 수 없습니다. 충전기로 옮겨주세요..mp3'"; break;
    //case E_SoundClass::SOUND_CANT_MOVE_TO_DOCKING_FAIL_ERROR    : m_pSoundClass[nSoundClassNumber].strName = "/usr/local/data/sound/'30. 충전기로 이동할 수 없습니다. 충전기로 옮겨주세요..mp3'"; break;
    //case E_SoundClass::SOUND_CANT_FIND_CHARGER_CHECK_AREA       : m_pSoundClass[nSoundClassNumber].strName = "/usr/local/data/sound/'33. 충전기를 찾을 수 없습니다. 충전기 주변에 장애물이 있는지 확인해 주세요..mp3'"; break;
    //case E_SoundClass::SOUND_LOW_BATT_CANT_MOVE_TO_CHARGER      : m_pSoundClass[nSoundClassNumber].strName = "/usr/local/data/sound/'25. 배터리가 부족하여 충전기로 이동할 수 없습니다..mp3'"; break
    // DISPLAY_CTR.startDisplay(E_DisplayImageClass::CHARGER_NOT_FOUND2); 기존
    DISPLAY_CTR.startAutoDisplay(true, E_DisplayImageClass::CHARGER_NOT_FOUND1);
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 디버깅 프린트 하는 곳.
 * 
 */
void CServiceManager::__debugPrint()
{
    CStopWatch __debug_sw;

    printSysState();
    printSysData();
    // printAppData();
    // printPoseData();
    printThreadAlive();
    //eblog((LOG_LV_NECESSARY|LOG_LV_MOTION),"\t" << "|ServiceData.tempRunServceID :" << ServiceData.tempRunServceID);
    //printLidarData();
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION),"\t" << "|SERVICE ID :" << enumToString(pContext->getServiceId()) << "|STATUS : " << enumToString(pContext->getServiceStatus()) << "|STEP : " << enumToString(pContext->getServiceStep(pContext->getServiceStatus())));
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION|LOG_LV_POSE_DEBUG),"\t" << "|SlamPose:" << SC<double>(ServiceData.localiz.getSlamPose().x) << ", " << SC<double>(ServiceData.localiz.getSlamPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION|LOG_LV_POSE_DEBUG),"\t" << "|CevaPose:" << SC<int>(ServiceData.localiz.getSysPose().calSn) << ", "<< SC<double>(ServiceData.localiz.getSysPose().x) << ", " << SC<double>(ServiceData.localiz.getSysPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSysPose().angle)));
#else
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION|LOG_LV_POSE_DEBUG),"\t" << "|CevaPose:" << SC<double>(ServiceData.localiz.getSysPose().x) << ", " << SC<double>(ServiceData.localiz.getSysPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSysPose().angle)));
#endif   
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION|LOG_LV_POSE_DEBUG),"\t" << "|RobotPose:" << SC<double>(ServiceData.localiz.getPose().x) << ", " << SC<double>(ServiceData.localiz.getPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getPose().angle)));
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::systemInterfaceErrorCheck()
{
    CStopWatch __debug_sw;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    /* 시스템 인터페이스 이상시, 빠른 확인을 위해 추가함. */
    // if (pObstacle->tof.lcliff.rangeAvg == 0 && pObstacle->tof.rcliff.rangeAvg == 0 && pObstacle->tof.knoll.rangeAvg == 0 && pObstacle->tof.rightwall.rangeAvg == 0)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDRED, "\n\n\n\n\t[ 경고 ]\n\n\t시스템 인터페이스가 이상합니다. 정상적인지 확인 해주세요.\n"
    //                                           << "\t(ToF 데이터가 0, 0, 0, 0 입니다.)\n\n\n");
    //     SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::COMMUNICATION))); // 인터페이스 에러!!
    // }

    TIME_CHECK_END(__debug_sw.getTime());
    return;
}

/**
 * @brief 배터리 디버깅 로그를 띄울 때 색상을 변경해주는 함수.
 * 
 * @return std::string 
 * 
 * @note 연산시간 ms
 * @date 2024-04-09
 * @author hhryu
 */
std::string CServiceManager::batteryColor()
{
    std::string color;
    switch (ServiceData.battery.getBatteryPercent())
    {
    case 80 ... 100:
        color = WHITE;
        break;
    case 50 ... 79:
        color = BLUE;
        break;
    case 0 ... 49:
        color = YELLOW;
        break;
    case 101 ... std::numeric_limits<u8>::max(): // 퍼센트가 100이 넘으면... 잘못된 것!
        color = RED;
    default:
        color = RED;
        break;
    }
    return color;
}

void CServiceManager::printSysState()
{
    CStopWatch __debug_sw;
    
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    std::string color;
    color = batteryColor();
    ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), color.c_str(), "|BATTERY STATE : " << enumToString(ServiceData.battery.getBatteryState()) << "|Percent : " << SC<int>(ServiceData.battery.getBatteryPercent()) <<"|Volt :"<< ServiceData.battery.getBatteryVolt());
    eblog((LOG_LV_NECESSARY|LOG_LV_MOTION),  "|SYSTEM-MODE : " << enumToString(ServiceData.power.getPowerState()) << "|TERMINAL STATE : " << enumToString(ServiceData.power.getTerminalState()) << "|EXT-POWER : " << SC<int>(ServiceData.power.getExtPower()));

#if 0
    eblog(LOG_LV_SERVICESTEP,// "\t|slamPoseTfUpdate: " << SC<int>(ROBOT_CONTROL.slam.isSlamLocalUpdate())
        //<< 
        "|TOF: L[" << SC<int>(pObstacle->tof.lcliff.rangeStatus) << "," << SC<int>(pObstacle->tof.lcliff.deviceState)<<"]\tR[" << SC<int>(pObstacle->tof.rcliff.rangeStatus) << "," << SC<int>(pObstacle->tof.rcliff.deviceState)<<"]\tKN["<< SC<int>(pObstacle->tof.knoll.rangeStatus) << "," << SC<int>(pObstacle->tof.knoll.deviceState)<<"]\tW["<< SC<int>(pObstacle->tof.leftwall.rangeStatus) << "," << SC<int>(pObstacle->tof.leftwall.deviceState) << "," << SC<int>(pObstacle->tof.rightwall.rangeStatus) << "," << SC<int>(pObstacle->tof.rightwall.deviceState)<<"]\t"
        << "|IMU:" << SC<int>(pObstacle->imu.filteredState) 
        << "\t|TILT:" << /*E_SYS_TILT_STATE_STR*[*/SC<int>(ServiceData.tilting.getStateValue())/*]*/ << "|POWER:" << SC<int>(ServiceData.power.getPowerState()) << "|TERMINAL:" << SC<int>(ServiceData.power.getTerminalState())
        << "\t|time:" << SYSTEM_TOOL.getSystemTime()
        );
#endif
    TIME_CHECK_END(__debug_sw.getTime());
}
void CServiceManager::printSysData()
{
    CStopWatch __debug_sw;
    
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDBLACK, "\t"
        <<BOLDBLACK<< "|tof: " <<WHITE<< SC<int>(pObstacle->tof.lcliff.rangeAvg) << ", " <<SC<int>(pObstacle->tof.rcliff.rangeAvg) << ", " << SC<int>(pObstacle->tof.knoll.rangeAvg) << ", " << SC<int>(pObstacle->tof.rightwall.rangeAvg) <<" " << SC<int>(pObstacle->tof.leftwall.rangeAvg) <<" "
        <<BOLDBLACK<< "|imu:" <<WHITE<< SC<int>(pObstacle->imu.Ax)<<","<<SC<int>(pObstacle->imu.Ay)<<","<<SC<int>(pObstacle->imu.Az)<<","<<SC<int>(pObstacle->imu.Gpitch)<<","<<SC<int>(pObstacle->imu.Groll)<<","<<SC<int>(pObstacle->imu.Gyaw)
        // <<BOLDBLACK<< "|CrIR:" <<WHITE<< SC<int>(ServiceData.signal.getReceiver().receivers->b.sideLeft) << "," << SC<int>(ServiceData.signal.getReceiver().receivers->b.centerLeft) << "," << SC<int>(ServiceData.signal.getReceiver().receivers->b.centerShort) << "," << SC<int>(ServiceData.signal.getReceiver().receivers->b.centerRight) << "," << SC<int>(ServiceData.signal.getReceiver().receivers->b.sideRight) << ","
        <<BOLDBLACK<< "|bump:" <<WHITE<< SC<int>(pObstacle->bumper.b.fleft_side)<<","<<SC<int>(pObstacle->bumper.b.fright_side)
        <<BOLDBLACK<< "|front:"<<WHITE<< pObstacle->ir.left.raw_data << "," <<pObstacle->ir.center.raw_data << "," <<pObstacle->ir.right.raw_data << ","
        );

    //ServiceData.signal.debugSignalPrint();

    TIME_CHECK_END(__debug_sw.getTime());
}
void CServiceManager::printAppData()
{
    CStopWatch __debug_sw;

    ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), GREEN, "\t"
        << "|slamPoseTfUpdate: " << SC<int>(ROBOT_CONTROL.slam.isSlamLocalUpdate())
        << "|slamPose:" << SC<double>(ServiceData.localiz.getSlamPose().x) << ", " << SC<double>(ServiceData.localiz.getSlamPose().y) << ", " << SC<double>(ServiceData.localiz.getSlamPose().angle)
        // << "|CrIR:" << SC<int>(ServiceData.signal.getDetectedReceiver().detector.b.leftLong) << "," << SC<int>(ServiceData.signal.getDetectedReceiver().detector.b.leftCenter) << "," << SC<int>(ServiceData.signal.getDetectedReceiver().detector.b.shortCenter) << "," << SC<int>(ServiceData.signal.getDetectedReceiver().detector.b.rightCenter) << "," << SC<int>(ServiceData.signal.getDetectedReceiver().detector.b.rightLong) << ","
        //<< "|Trap:" << SC<int>(pObstacle->trap.value) << "," << SC<int>(pObstacle->backtrap) << "," << SC<int>(pObstacle->wheeltrap)
        << "|LiDAR:" << ServiceData.localiz.getLidarDist(0) << ","<< ServiceData.localiz.getLidarDist(90) << ","<< ServiceData.localiz.getLidarDist(180) << ","<< ServiceData.localiz.getLidarDist(270)
        );
    
    TIME_CHECK_END(__debug_sw.getTime());
}


#define LIDAR_DEBUG_WALL_BALANCE_ANGLE 15 //라이다 센서 데이터 디버깅 용 디파인
#define LDIAR_DEBUG_CHECK_AROUND_IDX    3 //라이다 센서 디버깅용 기준 idx 기준 출력을 원하는 주변 idx개수

int debug_lidar_idx = 0;

void CServiceManager::printLidarData()
{
    CStopWatch __debug_sw;

    ceblog(LOG_LV_NECESSARY, BLUE, "print Lidar Data - st");
#if 1 // 모든 포인트 다 찍을 때!!!
    

	while(debug_lidar_idx <= 359)
    { 
        double lidarDistance = ServiceData.localiz.getLidarDist(debug_lidar_idx);

        if(std::isfinite(lidarDistance)) //(std::isinf(lidarDistance))isfinite
        {
           if (lidarDistance <= LIDAR_MIN_RANGE)
            {
                ceblog(LOG_LV_NECESSARY, RED, "dist[" << debug_lidar_idx << "] :" << lidarDistance);
            }
            else
            {
                if(lidarDistance < 0.5)
                {
                    ceblog(LOG_LV_NECESSARY, GREEN, "dist[" << debug_lidar_idx << "] :" << lidarDistance);
                }
            }
        }
        debug_lidar_idx++;
    }
    debug_lidar_idx = 0;
    ceblog(LOG_LV_NECESSARY, BLUE, "print Lidar Data - ed");
#else
    int start_idx = 360-(int)(LDIAR_DEBUG_CHECK_AROUND_IDX/2);
    int idx = 0;
    int last_idx = 359;

    while(true)
    {
        if(start_idx > last_idx) start_idx-=360;
        idx = start_idx;

        for (int i = 0; i < LDIAR_DEBUG_CHECK_AROUND_IDX; i++){
            if(idx > last_idx) idx-=360; 
            ceblog(LOG_LV_NECESSARY, GREEN, "idx["<<idx<<"]" "dist["<<ServiceData.localiz.getLidarDist(idx)<<"]");
            idx++;
        }

        if(idx == 360-start_idx+1){
            start_idx = 90-LIDAR_DEBUG_WALL_BALANCE_ANGLE;
        }
        else if(idx >= 90+LIDAR_DEBUG_WALL_BALANCE_ANGLE && idx <= 180){
            start_idx += 90-LIDAR_DEBUG_WALL_BALANCE_ANGLE;
        }                                     
        else{//if(start_idx < 90+LIDAR_DEBUG_WALL_BALANCE_ANGLE)
            start_idx += LIDAR_DEBUG_WALL_BALANCE_ANGLE;
        } 

        if(idx >= 270+LIDAR_DEBUG_WALL_BALANCE_ANGLE && idx < 360-LIDAR_DEBUG_WALL_BALANCE_ANGLE) break;   
    }
#endif
    ceblog(LOG_LV_NECESSARY, BLUE, "print Lidar Data - ed");

    TIME_CHECK_END(__debug_sw.getTime());
}

//FOR IMU STATE, IMU POSE DATA, SLAM POSE DATA
void CServiceManager::printPoseData()
{
    CStopWatch __debug_sw;

    if ( ROBOT_CONTROL.slam.isSlamRunning())
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDWHITE, "\t" << "| sysPose:\t" << (ServiceData.localiz.getSysPose().x) << ",\t" << (ServiceData.localiz.getSysPose().y) << ",\t" << utils::math::rad2deg(ServiceData.localiz.getSysPose().angle));
        ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDGREEN, "\t"<< "|slamPose:\t" << (ServiceData.localiz.getSlamPose().x) << ",\t" << (ServiceData.localiz.getSlamPose().y) << ",\t" << utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle));
        double __debugGap = utils::math::calculateMinimumDegreeAngle( utils::math::rad2deg(ServiceData.localiz.getSysPose().angle), utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle));
        if ( __debugGap >= 30 )
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "\t"<< "| 각도차 :\t" << __debugGap << "\t> 30도");
        }
        tPose sl = ServiceData.localiz.getSlamPose();
        tPose sy = ServiceData.localiz.getSysPose();
        eblog(LOG_LV_CEVA_ONLY, "slamPose, "<<sl.x<<",\t" <<sl.y<<",\t"<<utils::math::rad2deg(sl.angle)<<",\tsysPose, "<<
            sy.x<<",\t"<<sy.y<<",\t"<<utils::math::rad2deg(sy.angle));
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceManager::printThreadAlive()
{    
    // bool b2 = DEBUG_CTR.isAliveDstarWallUpdate.isUpdate()?false:true;DEBUG_CTR.isAliveDstarWallUpdate.get();
    bool b3 = DEBUG_CTR.isAliveImgProc.isUpdate()?false:true;DEBUG_CTR.isAliveImgProc.get();
    // bool b4 = DEBUG_CTR.isAliveMessage.isUpdate()?false:true;DEBUG_CTR.isAliveMessage.get();
    //bool b5 = DEBUG_CTR.isAliveMotionController.isUpdate()?false:true;DEBUG_CTR.isAliveMotionController.get();
    // bool b6 = DEBUG_CTR.isAliveRosCallbackLoop.isUpdate()?false:true;DEBUG_CTR.isAliveRosCallbackLoop.get();
    bool b7 = DEBUG_CTR.isAliveRosDataPub.isUpdate()?false:true;DEBUG_CTR.isAliveRosDataPub.get();
    bool b8 = DEBUG_CTR.isAliveSystemInterface.isUpdate()?false:true;DEBUG_CTR.isAliveSystemInterface.get();
    bool b9 = DEBUG_CTR.isAliveSystemWhatch.isUpdate()?false:true;DEBUG_CTR.isAliveSystemWhatch.get();
    // bool b10 = DEBUG_CTR.isAliveMotionPlanner.isUpdate()?false:true;DEBUG_CTR.isAliveMotionPlanner.get();
    // bool b11 = DEBUG_CTR.isAliveWaveFrontier.isUpdate()?false:true;DEBUG_CTR.isAliveWaveFrontier.get();
#if 0 // 주기적으로 생존 여부 모니터링 출력
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"==============================");
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDCYAN<<"\tThread 살아있나?");
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, MAGENTA<<"------------------------------");    
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" DstarWallUpdate"<<"\t"<<(b2?BOLDRED:BOLDGREEN)<<(b2?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" ImgProc"<<"\t"<<(b3?BOLDRED:BOLDGREEN)<<(b3?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" Message"<<"\t"<<(b4?BOLDGREEN:BOLDYELLOW)<<(b4?"살아있음":"살아있을듯?"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" MotionController"<<"\t"<<(b5?BOLDRED:BOLDGREEN)<<(b5?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" RosCallbackLoop"<<"\t"<<(b6?BOLDRED:BOLDGREEN)<<(b6?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" RosDataPub"<<"\t"<<(b7?BOLDRED:BOLDGREEN)<<(b7?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" SystemInterface"<<"\t"<<(b8?BOLDRED:BOLDGREEN)<<(b8?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" SystemWhatch"<<"\t"<<(b9?BOLDRED:BOLDGREEN)<<(b9?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" MotionPlanner"<<"\t"<<(b10?BOLDRED:BOLDGREEN)<<(b10?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" WaveFrontier"<<"\t"<<(b11?BOLDRED:BOLDGREEN)<<(b11?"살아있음":"죽음"));
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"==============================");
#else // 죽은 thread만 출력
    if(b3|b7|b8|b9)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"==============================");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDCYAN<<"\tThread 죽은것들");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, MAGENTA<<"------------------------------");    
        // if(b2)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" DstarWallUpdate"<<"\t"<<BOLDRED<<("죽음"));
        if(b3)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" ImgProc"<<"\t"<<BOLDRED<<"죽음");
        // if(b4)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" Message"<<"\t"<<BOLDYELLOW<<"살아있을듯?");
        //if(b5)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" MotionController"<<"\t"<<BOLDRED<<"죽음");
        // if(b6)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" RosCallbackLoop"<<"\t"<<BOLDRED<<"죽음");
        if(b7)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" RosDataPub"<<"\t"<<BOLDRED<<"죽음");
        if(b8)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" SystemInterface"<<"\t"<<BOLDRED<<"죽음");
        if(b9)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" SystemWhatch"<<"\t"<<BOLDRED<<"죽음");
        // if(b10)  ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" motionplanner"<<"\t"<<BOLDRED<<"죽음");
        // if(b11) ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDBLACK<<BOLDWHITE<<setw(20)<<" WaveFrontier"<<"\t"<<BOLDRED<<"죽음");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"===================");
    }
#endif
}

