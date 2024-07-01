#include "control/control.h"
#include "control/motionPlanner/motionPlanner.h"
#include "systemTool.h"
#include "eblog.h"
#include "MessageHandler.h"
#include "subTask.h"

#define TILT_CONTORL_TIMEOUT 7

CControl::CControl() {}
CControl::~CControl() {}

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CControl& CControl::getInstance()
{
    CStopWatch __debug_sw;
    
    static CControl s;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return s;
}

void CControl::reportAwsAction(short action)
{
    if(ServiceData.rsBridge.getconnect()){
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_ACTION, tMsgReportAction(action)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION ACTION-REPROT FAIL! : " << action);
    }
    
}

void CControl::reportAwsStatus(int state)
{
    if(ServiceData.rsBridge.getconnect()){
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_STATUS, tMsgReportStatus(state)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION STATE-REPROT FAIL! : " << state);
    }
}

void CControl::reportAwsWaterLevel()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::WATER_LEVEL;
    data.waterLv.value = ServiceData.awsData.getSendWaterLv();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT WaterLevel : " << data.waterLv.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! WaterLevel : " );
    }
}
void CControl::reportAwsSoundLevel()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::SOUND_LEVEL;
    data.soundLv.value = ServiceData.awsData.getSendSoundLv();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT SoundLevel : " << data.soundLv.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! SoundLevel : " );
    }
}
void CControl::reportAwsCleanMode()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::CLEAN_MODE;
    data.cleanMode.value = ServiceData.awsData.getSendCleanMode();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT cleanMode : " << data.cleanMode.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! cleanMode : " );
    }
}
void CControl::reportAwsLanguage()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::LANGUAGE;
    data.language.value = ServiceData.awsData.getSendLanguage();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT Language : " << data.language.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! Language : " );
    }
}
void CControl::reportAwsCountry()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::COUNTRY;
    data.country.value = ServiceData.awsData.getSendCountry();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT Country : " << data.country.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! Country : " );
    }
}

void CControl::reportAwsBattery()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::BATTERY;
    data.battery.value = ServiceData.awsData.getSendBattery();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT Battery : " << data.battery.value);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! Battery : " );
    }
}
void CControl::reportAwsDryMopData()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::DRY_MOP;
    data.dryData.dryEnabled = ServiceData.awsData.getSendDryEnabled();
    data.dryData.dryHours = ServiceData.awsData.getSendDryHours();
    data.dryData.dryPower = ServiceData.awsData.getSendDryPower();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT DryMopData");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! DryMopData : " );
    }
}
void CControl::reportAwsError()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::ERROR;
    data.error.errorCode = ServiceData.awsData.getSendErrorCode();
    data.error.errorDesc = ServiceData.awsData.getSendErrorDesc();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT Error : " << data.error.errorCode);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! Error : " );
    }
}
void CControl::reportRobotSettings()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::SETTTING;

    data.setting.apVersion = ServiceData.awsData.getSendApVersion();
    data.setting.mcuVersion = ServiceData.awsData.getSendMcuVersion();

    if(ServiceData.rsBridge.getconnect()){
        ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT Settings apVersion : " << data.setting.apVersion << " mcuVersion : " <<  data.setting.mcuVersion);
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION Settings-REPROT FAIL! : ");
    }
}
void CControl::reportOperationArea()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::OPERATION_AREA;
    data.operationArea.type = ServiceData.awsData.getOperationAreaType();
    data.operationArea.all = ServiceData.awsData.getSendAreaAll();
    data.operationArea.spot = ServiceData.awsData.getSendAreaSpot();
    data.operationArea.spotNumber = ServiceData.awsData.getSendAreaSpotNum();
    data.operationArea.room = ServiceData.awsData.getSendAreaRoom();
    data.operationArea.roomNumber = ServiceData.awsData.getSendAreaRoomNum();
    data.operationArea.custom = ServiceData.awsData.getSendAreaCustom();
    data.operationArea.customNumber = ServiceData.awsData.getSendAreaCustomNum();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT OperationArea : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! OperationArea : " );
    }
}
void CControl::reportForbiddenArea()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::FORBIDDEN_AREA;
    data.forbiddenArea.type = ServiceData.awsData.getSendForbiddenType();
    data.forbiddenArea.line = ServiceData.awsData.getSendForbiddenLine();
    data.forbiddenArea.lineNumber = ServiceData.awsData.getSendForbiddenLineNum();
    data.forbiddenArea.rect = ServiceData.awsData.getSendForbiddenRect();
    data.forbiddenArea.rectNumber = ServiceData.awsData.getSendForbiddenRectNum();

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT ForbiddenArea : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! ForbiddenArea : " );
    }
}
void CControl::reportDistruptMode()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::DISTRUBT_MODE;
    data.distruptData.status = ServiceData.awsData.getSendDontDisturbStatus();
    data.distruptData.startTime = ServiceData.awsData.getSendDontDisturbStartTime();
    data.distruptData.endTime = ServiceData.awsData.getSendDontDisturbEndTime();

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT DistruptMode : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! DistruptMode : " );
    }
}
void CControl::reportReservationClean()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::RESERVATION_CLEAN;
    data.rsvCleanData.schedule = ServiceData.awsData.getSendCleaningSchedule();
    data.rsvCleanData.scheduleNumber = ServiceData.awsData.getSendCleanScheduleNum();

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT ReservationClean : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! ReservationClean : " );
    }
}

void CControl::reportSaveMapInfo(bool completedByForce)
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::SAVE_MAP_INFO;
    
    if(completedByForce == false ){
        ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
        data.saveMapInfo.mapData = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
        data.saveMapInfo.mapInfo = ServiceData.robotMap.simplifyMap.getInfo();
        ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
        data.saveMapInfo.traj = ServiceData.awsData.getSendSaveMap().traj;
    } else {
        data.saveMapInfo.mapData = nullptr;
    }

    data.saveMapInfo.robotPose = ServiceData.awsData.getSendSaveMap().robotPose;
    data.saveMapInfo.areaInfo = ServiceData.awsData.getSendSaveMap().areaInfo;
    data.saveMapInfo.mapName = ServiceData.awsData.getSendSaveMap().mapName;
    data.saveMapInfo.order = ServiceData.awsData.getSendSaveMap().order;
    data.saveMapInfo.uniqueKey = ServiceData.awsData.getSendSaveMap().uniqueKey;

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT SaveMapInfo : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! SaveMapInfo : " );
    }
}

void CControl::reportMapData(E_SERVICE_ID id)
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::MAP_DATA;
    if (id == E_SERVICE_ID::CLEAN && id == E_SERVICE_ID::EXPLORER)
    {
        ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
        data.mapData.mapData = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
        data.mapData.mapInfo = ServiceData.robotMap.simplifyMap.getInfo();
        ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
    }
    else if (id == E_SERVICE_ID::DOCKING)
    {
        data.mapData.mapData = nullptr;
    }
    
    data.mapData.cradleCoord = SUB_TASK.cleanPlan.getNoGoZoneDockingCoord();
    data.mapData.robotPose = ServiceData.localiz.getPose();
    if (id == E_SERVICE_ID::CLEAN)
    {
        data.mapData.traj = ServiceData.robotMap.robotTrajectory.getCleanedTrajectory();
        data.mapData.cleanSize = SUB_TASK.cleanPlan.getCleanedSize(); 
    }
    else if (id == E_SERVICE_ID::EXPLORER)
    {
        data.mapData.traj = ServiceData.robotMap.robotTrajectory.getExploredTrajectory();
        data.mapData.cleanSize = 0.0; 
    }
        
    //data.mapData.mapData = serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT MapData : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! MapData : " );
    }
}
void CControl::reportAreaInfo(tAreaInfoData areaInfo)
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::AREA_INFO;
    data.areaInfo.areaNumber = areaInfo.areaNumber;
    data.areaInfo.info = areaInfo.info;
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT AreaInfo : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! AreaInfo : " );
    }
}
void CControl::reportDivideArea()
{
    tMsgReportData msg;
    msg.sort = E_AWS_MSG_SORT::DIVIDE_AREA;
    msg.divideArea.data = ServiceData.areaInfo.getDivideAreaData();
    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT DivideArea : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(msg)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! DivideArea : " );
    }
}
void CControl::reportCombineArea()
{
    tMsgReportData msg;
    msg.sort = E_AWS_MSG_SORT::COMBINE_AREA;
    msg.combineArea.data = ServiceData.areaInfo.getCombineAreaData();

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT CombineArea : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(msg)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! CombineArea : " );
    }
}
void CControl::reportCleanHistroy()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::CLEAN_HISTORY;

    ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
    data.cleanHistory.mapData = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
    data.cleanHistory.mapInfo = ServiceData.robotMap.simplifyMap.getInfo();
    ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
    data.cleanHistory.areaInfo = ServiceData.awsData.getSendCleanHistory().areaInfo;
    data.cleanHistory.cleanedSize = ServiceData.robotData.getCleanHistory().cleanedSize;
    data.cleanHistory.cleanStartTime = ServiceData.awsData.getSendCleanHistory().cleanStartTime;
    data.cleanHistory.cleanTime = ServiceData.awsData.getSendCleanHistory().cleanTime;
    data.cleanHistory.cradlePose = ServiceData.awsData.getSendCleanHistory().cradlePose;
    data.cleanHistory.exitReason = ServiceData.awsData.getSendCleanHistory().exitReason;
    data.cleanHistory.robotPose = ServiceData.awsData.getSendCleanHistory().robotPose;
    data.cleanHistory.traj = ServiceData.awsData.getSendCleanHistory().traj;

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," AWS REPROT CleanHistory : ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! CleanHistory : " );
    }
}

void CControl::reportFactoryReset()
{
    tMsgFactoryReset data;

    std::string currentTime = utils::math::getCurrentTimeString(1,1);
    std::string discript = "test Factory Reset";

    strncpy(data.startTime, currentTime.c_str(), sizeof(data.startTime) - 1);
    // 널 문자를 수동으로 추가하여 문자열을 종료
    data.startTime[sizeof(data.startTime) - 1] = '\0';

    strncpy(data.descript, discript.c_str(), sizeof(data.descript) - 1);
    data.descript[sizeof(data.descript) - 1] = '\0';

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," reportFactoryReset StartTime : " << data.startTime << "descript : " << data.descript);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_FACTORY_RESET, tMsgFactoryReset(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! AreaInfo : " );
    }
}

void CControl::reportRobotPose()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::ROBOT_POSE;
    data.robotPose.robotPose = ServiceData.localiz.getPose();

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," robot pose X : " << data.robotPose.robotPose.x << " robot pose Y : " << data.robotPose.robotPose.y);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! robot Pose : " );
    }
}

void CControl::reportOtaVersion()
{
    tMsgReportData data;
    data.sort = E_AWS_MSG_SORT::OTA_VERSION;
    std::strcpy(data.otaInfo.version,ServiceData.awsData.getSendOtaVersion());

    if(ServiceData.rsBridge.getconnect()){
    ceblog(LOG_LV_NECESSARY, CYN," OTA-VERSION : " << data.otaInfo.version);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, tMsgReportData(data)));
    }else{
        ceblog(LOG_LV_NECESSARY, CYN," LOST AWS CONNECTION REPROT FAIL! OTA-VERSION " );
    }
}

void CControl::robotSystemReset()
{
    ceblog(LOG_LV_NECESSARY, CYN," robotSystemReset");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_MCU_RESET));
}

void CControl::robotPowerOff()
{
    ceblog(LOG_LV_NECESSARY, CYN," robotPowerOff");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_POWER));
}

void CControl::systemOpenCloseOta(bool on)
{
    tMsgOtaCmd msg = {};
    msg.type = E_SYSTEM_OTA_CMD::OPEN_CLOSE;
    msg.bOpen = on;
    ceblog(LOG_LV_NECESSARY, CYN," OTA ON_OFF : " <<  (int)msg.bOpen);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_OTA_COMMAND, msg));
}

void CControl::systemStartOta(){
    tMsgOtaCmd msg = {};
    
    msg.type = E_SYSTEM_OTA_CMD::START;
    std::strcpy(msg.path,ServiceData.Ota.getOtaPath());

    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_OTA_COMMAND, msg));
    ceblog(LOG_LV_NECESSARY, CYN," OTA PATH : " <<  msg.path);
}

void CControl::sendMessageTilting(E_TILTING_CONTROL control)
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_TILT;
    tMsgCtrTilt tiltMsg = {};

    tiltMsg.control = control;

    SEND_MESSAGE(message_t(type, tiltMsg));
}

void CControl::WaterPump(bool on)
{
    CStopWatch __debug_sw;
    
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_PUMP;
    tMsgCtrPump pumpMsg = {};

    pumpMsg.on = on;

    SEND_MESSAGE(message_t(type, pumpMsg));
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CControl::dryFanOn(E_DRYFAN_LEVEL lv)
{
    CStopWatch __debug_sw;
    int FanRPM = 0;
    if(lv == E_DRYFAN_LEVEL::DRY_LEVEL_0)       FanRPM = 0;  
    else if(lv == E_DRYFAN_LEVEL::DRY_LEVEL_1)  FanRPM = 60;  
    else if(lv == E_DRYFAN_LEVEL::DRY_LEVEL_2)  FanRPM = 80;
    else                                        FanRPM = 100;

    SEND_MESSAGE( message_t(E_MESSAGE_TYPE_CONTROL_DRYFAN, tMsgCtrDryfan(true, FanRPM))); // 팬 속도는 인자로 받아서 조절할 수 있도록 변경 해야 함.
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CControl::dryFanOff()
{
    CStopWatch __debug_sw;
    
    SEND_MESSAGE( message_t(E_MESSAGE_TYPE_CONTROL_DRYFAN, tMsgCtrDryfan(false, 0) ));
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CControl::systemModeControl(u8 type)
{
    CStopWatch __debug_sw;
    
    if(type == 0) 	SEND_MESSAGE( message_t(E_MESSAGE_TYPE_ACTIVE_MODE));
    else			SEND_MESSAGE( message_t(E_MESSAGE_TYPE_CHARGE_MODE));	
    
    TIME_CHECK_END(__debug_sw.getTime());
}


void CControl::clearSystemLocalization(void)
{
    CStopWatch __debug_sw;

    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_SYSTEM_CTR;

    printf("clearSystemLocalization \n\r");

    SEND_MESSAGE(message_t(type, tMsgCtrSystem(E_SYSTEM_CTR_CLEAR_LOCALIZTION)));
    
    TIME_CHECK_END(__debug_sw.getTime());
}


u16 CControl::getPumpCurrent(void)	{
    return pumpcurrent;
}

void CControl::startSlam()
{
    CStopWatch __debug_sw;
    
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_SLAM;
    
    printf("startSlam \n\r");

    tMsgCtrSlam slamlMsg = {};
    slamlMsg.state = 4;
    
    SEND_MESSAGE(message_t(type, slamlMsg));
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CControl::startTiltingUp()
{
    if(ServiceData.tilting.getStateValue() != E_SYS_TILT_STATE::TILTED_UP)
    {
        tInfo.bRunning = true;
        tInfo.cmd = E_TILTING_CONTROL::UP;
        tInfo.step = E_TILT_STEP::WAIT;
        tInfo.startTime = SYSTEM_TOOL.getSystemTime();
        sendMessageTilting(E_TILTING_CONTROL::UP);
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN,"Tilt State is Aready Up!! skip Command");
    }
}


E_TILT_STEP CControl::getTiltStep()
{
    return tInfo.step;
}

void CControl::startTiltingStop()
{
    tInfo.bRunning = true;
    tInfo.cmd = E_TILTING_CONTROL::STOP;
    tInfo.step = E_TILT_STEP::WAIT;
    tInfo.startTime = SYSTEM_TOOL.getSystemTime();
    sendMessageTilting(E_TILTING_CONTROL::STOP);
}

void CControl::setTiltStep(E_TILT_STEP set)
{
    if(tInfo.step != set){
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "TiltStep Change!! now step : " << enumToString(tInfo.step) << " new Step : " << enumToString(set));
    }
    tInfo.step = set;
}

bool CControl::isTiltRunning()
{
    return tInfo.bRunning;
}
double CControl::getTiltCtrlStartTime()
{
    return tInfo.startTime;
}

void CControl::startTilt(E_TILTING_CONTROL cmd)
{
    E_SYS_TILT_STATE state = ServiceData.tilting.getStateValue();
    tInfo.bRunning = true;
    tInfo.retryCount = 0;
    tInfo.cmd = cmd;
    tInfo.step = E_TILT_STEP::WAIT;
    tInfo.startTime = SYSTEM_TOOL.getSystemTime();
    tInfo.ctrlTime = SYSTEM_TOOL.getSystemTime();
    if(checkTiltState(state)){
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "skip Command : " << enumToString(tInfo.cmd) << " Tilt State is Aready : " << enumToString(state));
    }else{
        sendMessageTilting(tInfo.cmd);
    }
}

void CControl::reStartTilt(E_SYS_TILT_STATE state)
{
    tInfo.bRunning = true;
    tInfo.retryCount++;
    tInfo.step = E_TILT_STEP::WAIT;
    tInfo.ctrlTime = SYSTEM_TOOL.getSystemTime();
    sendMessageTilting(tInfo.cmd);
    ceblog(LOG_LV_NECESSARY, BOLDGREEN, "retry Command : " << enumToString(tInfo.cmd) << " Tilt State : " << enumToString(state));
}

bool CControl::checkTiltState(E_SYS_TILT_STATE state)
{
    bool ret = false;

    switch (tInfo.cmd)
    {
    case E_TILTING_CONTROL::STOP:
        if(state == E_SYS_TILT_STATE::TILING_STOP) ret = true;
        break;
    case E_TILTING_CONTROL::UP:
        if(state == E_SYS_TILT_STATE::TILTED_UP) ret = true;
        break;
    case E_TILTING_CONTROL::DOWN:
        if(state == E_SYS_TILT_STATE::TILTED_DOWN) ret = true;
        break;   
    default:
        startTilt(E_TILTING_CONTROL::STOP);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " wrong command!! command : "<< enumToString(tInfo.cmd) << "state : "<< enumToString(state));
        break;
    }
    return ret;
}

E_TILT_STEP CControl::waitCheckTilt(E_SYS_TILT_STATE state)
{
    E_TILT_STEP ret = E_TILT_STEP::WAIT;
    double runTime = SYSTEM_TOOL.getSystemTime()-tInfo.ctrlTime;
    
    if(runTime >= TILT_CONTORL_TIMEOUT) ret = E_TILT_STEP::RETRY;
    if(checkTiltState(state)) ret = E_TILT_STEP::COMPLETE;

    return ret;
}
E_TILT_STEP CControl::retryTilt(E_SYS_TILT_STATE state)
{
    E_TILT_STEP ret = E_TILT_STEP::RETRY;
    double runTime = SYSTEM_TOOL.getSystemTime()-tInfo.ctrlTime;
    if(runTime >= TILT_CONTORL_TIMEOUT){
        if(tInfo.retryCount >= 3) ret = E_TILT_STEP::ERROR;
        else                      reStartTilt(state);
    }else if(checkTiltState(state)){
        ret = E_TILT_STEP::COMPLETE;
    }
    return ret;
}
E_TILT_STEP CControl::completeTilt()
{
    double ctrlTime = SYSTEM_TOOL.getSystemTime()-tInfo.ctrlTime;
    double runTime = SYSTEM_TOOL.getSystemTime()-tInfo.startTime;
    E_TILT_STEP ret = E_TILT_STEP::COMPLETE;
    tInfo.bRunning = false;
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "TILT COMMAND : " << enumToString(tInfo.cmd) << "\tE_TILT_STEP::COMPLETE -> retryCount : " << tInfo.retryCount <<
            "제어 대기 시간 : " << ctrlTime << "\t전체 대기 시간 : " <<  runTime);
    ret = E_TILT_STEP::VOID;
    return ret;
}

E_TILT_STEP CControl::errorTilt()
{
    E_TILT_STEP ret = E_TILT_STEP::COMPLETE;
    tInfo.bRunning = false;
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::TILT)));
    ceblog(LOG_LV_ERROR, RED, "틸 에러" );
    ret = E_TILT_STEP::VOID;
    return ret;
}


void CControl::procControlTilt()
{
    E_SYS_TILT_STATE state = ServiceData.tilting.getStateValue();
    E_POWER_STATE power = ServiceData.power.getPowerState();

    if(power != E_POWER_STATE::ACTIVE){
        ROBOT_CONTROL.systemModeControl(E_POWER_MODE::MODE_ACTIVE);
        ceblog(LOG_LV_NECESSARY, RED, "ACITVE 모드가 아닙니다. TILT 제어가 불가능함!! POWER MODE : " << (int)power << " Tilt State : " << enumToString(state) );
        return;
    }

    switch (tInfo.step)
    {
        case E_TILT_STEP::VOID:
            break;
        case E_TILT_STEP::WAIT:
            setTiltStep(waitCheckTilt(state));
            break;
        case E_TILT_STEP::RETRY:
            setTiltStep(retryTilt(state));
            break;
        case E_TILT_STEP::COMPLETE:
            setTiltStep(completeTilt());
            break;
        case E_TILT_STEP::ERROR:
            setTiltStep(errorTilt());
            break;                    
        default:
            break;
    }
}
