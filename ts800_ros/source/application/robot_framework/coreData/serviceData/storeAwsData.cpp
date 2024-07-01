#include "storeAwsData.h"
#include "robotServerBridge.h"
#include "coreData/serviceData.h"

CStoreAwsData::CStoreAwsData() {
    
}
CStoreAwsData::~CStoreAwsData() {}

E_AWS_MSG_SORT CStoreAwsData::getDataSort()
{
    return data.sort;
}

void CStoreAwsData::setSendWaterLv(E_WATER_PUMP_STEP lv)
{
    if(lv == E_WATER_PUMP_STEP::WATER_DRY)               data.waterLv.value = 1;
    else if (lv == E_WATER_PUMP_STEP::WATER_DISPOSABLE)  data.waterLv.value = 2;
    else if (lv == E_WATER_PUMP_STEP::WATER_LEVEL_1)     data.waterLv.value = 3;
    else if (lv == E_WATER_PUMP_STEP::WATER_LEVEL_2)     data.waterLv.value = 4;
    eblog((LOG_LV_NECESSARY|LOG_LV_AWS), "SET SEND AWS WATER_LV :" << data.waterLv.value <<" ROBOT WATER LV : "<< enumToString(lv));
    data.waterLv.bUpdate = true;
    data.sort = E_AWS_MSG_SORT::WATER_LEVEL;
}
short CStoreAwsData::getSendWaterLv() {return data.waterLv.value;eblog((LOG_LV_NECESSARY|LOG_LV_AWS), "GET SEND - AWS WATER_LV :" << data.waterLv.value);}
bool CStoreAwsData::isWaterLevelUpdate()
{
    bool ret = false; 
    if(data.waterLv.bUpdate){
        ret = data.waterLv.bUpdate;
        data.waterLv.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendSoundLv(short lv){data.soundLv.value = lv;data.soundLv.bUpdate = true;}
short CStoreAwsData::getSendSoundLv() {return data.soundLv.value;}
bool CStoreAwsData::isSoundVolumeUpdate()
{
    bool ret = false; 
    if(data.soundLv.bUpdate){
        ret = data.soundLv.bUpdate;
        data.soundLv.bUpdate = false;
    }
    return ret;
}
#if 0
void CStoreAwsData::setSendDryOption(short _dryEnabled, short _dryHours, E_DRYFAN_LEVEL _dryPower)
{
    robotData.dryEnabled = _dryEnabled;
    robotData.dryHours = _dryHours;
    if (_dryPower == E_DRYFAN_LEVEL::DRY_LEVEL_1) 
        robotData.dryPower = 60;
    else if (_dryPower == E_DRYFAN_LEVEL::DRY_LEVEL_2) 
        robotData.dryPower = 80;
    else if (_dryPower == E_DRYFAN_LEVEL::DRY_LEVEL_3) 
        robotData.dryPower = 100;
    else if (_dryPower == E_DRYFAN_LEVEL::DRY_LEVEL_0) 
        robotData.dryPower = 0;
}
short CStoreAwsData::getSendDryOption(short sort)
{
    if (sort == 1) return robotData.dryEnabled;
    else if (sort == 2) return robotData.dryHours;
    else if (sort == 3) return robotData.dryPower;
    else return 0;

}
#endif
void CStoreAwsData::setSendDryEnabled(short dryEnabled){data.dryData.dryEnabled = dryEnabled;data.dryData.bUpdate = true;}
short CStoreAwsData::getSendDryEnabled(){return data.dryData.dryEnabled;}
void CStoreAwsData::setSendDryPower(short dryPower){    data.dryData.dryPower = dryPower;data.dryData.bUpdate = true;}
short CStoreAwsData::getSendDryPower(){return data.dryData.dryPower;}
void CStoreAwsData::setSendDryHours(short dryHours){data.dryData.dryHours = dryHours;data.dryData.bUpdate = true;}
short CStoreAwsData::getSendDryHours() {return data.dryData.dryHours;}
bool CStoreAwsData::isDryMopDataUpdate()
{
    bool ret = false; 
    if(data.dryData.bUpdate){
        ret = data.dryData.bUpdate;
        data.dryData.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendCleanMode(short mode){data.cleanMode.value = mode;data.cleanMode.bUpdate = true;}
short CStoreAwsData::getSendCleanMode() {return data.cleanMode.value;}
bool CStoreAwsData::isCleanModeUpdate()
{
    bool ret = false; 
    if(data.cleanMode.bUpdate){
        ret = data.cleanMode.bUpdate;
        data.cleanMode.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendLanguage(short language) {data.language.value = language;data.language.bUpdate = true;}
short CStoreAwsData::getSendLanguage() {return data.language.value;}
bool CStoreAwsData::isLanguageDataUpdate()
{
    bool ret = false; 
    if(data.language.bUpdate){
        ret = data.language.bUpdate;
        data.language.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendCountry(short country) {data.country.value = country;data.country.bUpdate = true;}
short CStoreAwsData::getSendCountry() {return data.country.value;}
bool CStoreAwsData::isCountryDataUpdate()
{
    bool ret = false; 
    if(data.country.bUpdate){
        ret = data.country.bUpdate;
        data.country.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendError(E_ERROR_HANDLING_TYPE errType)
{
    if(errType == E_ERROR_HANDLING_TYPE::CONTINUOUS_CLIFF)
    {
        data.error.errorCode = "Robot Lifting";
        data.error.errorDesc = "평평한 바닥에 다시 시작해주세요.";
    }
    else if(errType == E_ERROR_HANDLING_TYPE::TILT)
    {
        data.error.errorCode = "Tilting Error";
        data.error.errorDesc = "점검이 필요합니다. 서비스 센터에 문의해주세요.";
    }
    else if(errType == E_ERROR_HANDLING_TYPE::MISSING_ROS_DATA)
    {
        data.error.errorCode = "LiDAR Data Error";
        data.error.errorDesc = "라이다 센서를 확인해주세요.";
    }
    else if(errType == E_ERROR_HANDLING_TYPE::DOCKING_FAIL)
    {
        data.error.errorCode = "Homing Fail";
        data.error.errorDesc = "충전기로 이동 할 수 없습니다. 충전기로 옮겨주세요.";
    }
    else 
    {
        data.error.errorCode = "no error";
        data.error.errorDesc = "no error";
        /* 다른 에러들 */
    }
    if(errType != E_ERROR_HANDLING_TYPE::NONE){
        data.error.bUpdate = true;
    }
    
}
std::string CStoreAwsData::getSendErrorCode() {return data.error.errorCode;}
std::string CStoreAwsData::getSendErrorDesc() {return data.error.errorDesc;}
bool CStoreAwsData::isErrorDataUpdate()
{
    bool ret = false; 
    if(data.error.bUpdate){
        ret = data.error.bUpdate;
        data.error.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendBattery(short _battery){data.battery.value = _battery; data.battery.bUpdate = true;}
short CStoreAwsData::getSendBattery() {return data.battery.value;}
bool CStoreAwsData::isBatteryUpdate()
{
    bool ret = false; 
    if(data.battery.bUpdate){
        ret = data.battery.bUpdate;
        data.battery.bUpdate = false;
    }
    return ret;
}

/* Robot Info */

void CStoreAwsData::setSendApVersion(std::string apVersion) {data.setting.apVersion = apVersion; data.setting.bUpdate = true;}
std::string CStoreAwsData::getSendApVersion() {return data.setting.apVersion;}
void CStoreAwsData::setSendMcuVersion(std::string mcuVersion) {data.setting.mcuVersion = mcuVersion; data.setting.bUpdate = true;}
std::string CStoreAwsData::getSendMcuVersion() {return data.setting.mcuVersion;}

bool CStoreAwsData::isSettingDataUpdate()
{
    bool ret = false; 
    if(data.setting.bUpdate){
        ret = data.setting.bUpdate;
        data.setting.bUpdate = false;
    }
    return ret;
}

/* operation Area */
short CStoreAwsData::getOperationAreaType(){return data.operationArea.type;}

void CStoreAwsData::setSendAreaAll()
{
    data.operationArea.bUpdate = true;
    data.operationArea.type = 1;
    data.operationArea.all = ServiceData.rsBridge.getAreaAll();
}

short CStoreAwsData::getSendAreaAll(){return data.operationArea.all;}

void CStoreAwsData::setSendAreaSpot()
{
    data.operationArea.bUpdate = true;
    data.operationArea.type = 2;
    tSpot* spotPtr = ServiceData.rsBridge.getAreaSpot().first;
    if(!data.operationArea.spot.empty()){
        data.operationArea.spot.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        data.operationArea.spot.push_back(spotPtr[i]);
        eblog(LOG_LV_AWS," Spot "<<"[" << i << "] : x = " << data.operationArea.spot.back().x
                                <<"[" << i << "] : y = " << data.operationArea.spot.back().y);
    }
    // operationArea.spot = ServiceData.rsBridge.getAreaSpot().first;
    if (ServiceData.rsBridge.getAreaSpot().second == 0xff){
        data.operationArea.spotNumber = 0;
    }
    else    data.operationArea.spotNumber = ServiceData.rsBridge.getAreaSpot().second;
}
std::list<tSpot> CStoreAwsData::getSendAreaSpot(){return data.operationArea.spot;}
short CStoreAwsData::getSendAreaSpotNum(){return data.operationArea.spotNumber;}

void CStoreAwsData::setSendAreaRoom()
{
    data.operationArea.bUpdate = true;
    data.operationArea.type = 3;
    tRoom* roomPtr = ServiceData.rsBridge.getAreaRoom().first;
    if(!data.operationArea.room.empty()){
        data.operationArea.room.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        data.operationArea.room.push_back(roomPtr[i]);
        eblog(LOG_LV_AWS," Room "<<"[" << i << "] : x = " << data.operationArea.room.back().x 
                                <<"[" << i << "] : y = " << data.operationArea.room.back().y
                                <<"[" << i << "] : w = " << data.operationArea.room.back().w
                                <<"[" << i << "] : h = " << data.operationArea.room.back().h);
    }
    if (ServiceData.rsBridge.getAreaRoom().second == 0xff){
        data.operationArea.roomNumber = 0;
    }
    else    data.operationArea.roomNumber = ServiceData.rsBridge.getAreaRoom().second;    
}
std::list<tRoom> CStoreAwsData::getSendAreaRoom(){return data.operationArea.room;}
short CStoreAwsData::getSendAreaRoomNum(){return data.operationArea.roomNumber;}

void CStoreAwsData::setSendAreaCustom()
{
    data.operationArea.bUpdate = true;
    data.operationArea.type = 4;
    tCustom* customPtr = ServiceData.rsBridge.getAreaCustom().first;
    if(!data.operationArea.custom.empty()){
        data.operationArea.custom.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        data.operationArea.custom.push_back(customPtr[i]);
        eblog(LOG_LV_AWS," Custom "<<"[" << i << "] : x = " << data.operationArea.custom.back().x 
                                <<"[" << i << "] : y = " << data.operationArea.custom.back().y
                                <<"[" << i << "] : w = " << data.operationArea.custom.back().w
                                <<"[" << i << "] : h = " << data.operationArea.custom.back().h);
    }
    if (ServiceData.rsBridge.getAreaCustom().second == 0xff){
        data.operationArea.customNumber = 0;
    }
    else    data.operationArea.customNumber = ServiceData.rsBridge.getAreaCustom().second;
    eblog(LOG_LV_AWS," service customNumber : "<<data.operationArea.customNumber);
}
std::list<tCustom> CStoreAwsData::getSendAreaCustom(){return data.operationArea.custom;}
short CStoreAwsData::getSendAreaCustomNum(){return data.operationArea.customNumber;}
bool CStoreAwsData::isOperationAreaUpdate()
{
    bool ret = false; 
    if(data.operationArea.bUpdate){
        ret = data.operationArea.bUpdate;
        data.operationArea.bUpdate = false;
    }
    return ret;
}

/* forbiddenArea */
short CStoreAwsData::getSendForbiddenType()
{
    return data.forbiddenArea.type;
}

void CStoreAwsData::setSendForbiddenLine()
{
    data.forbiddenArea.bUpdate = true;
    data.forbiddenArea.type = 1;
    tForbiddenLine* linePtr = ServiceData.rsBridge.getForbiddenLine().first;
    if(!data.forbiddenArea.line.empty()){
        data.forbiddenArea.line.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        data.forbiddenArea.line.push_back(linePtr[i]);
        eblog(LOG_LV_AWS," ForbiddenLine "<<"[" << i << "] : x1 = " << data.forbiddenArea.line.back().x1 
                                <<"[" << i << "] : y1 = " << data.forbiddenArea.line.back().y1
                                <<"[" << i << "] : x2 = " << data.forbiddenArea.line.back().x2
                                <<"[" << i << "] : y2 = " << data.forbiddenArea.line.back().y2);
    }
    if (ServiceData.rsBridge.getForbiddenLine().second == 0xff){
        data.forbiddenArea.lineNumber = 0;
    }
    else    data.forbiddenArea.lineNumber = ServiceData.rsBridge.getForbiddenLine().second;
    eblog(LOG_LV_AWS," service customNumber : "<<data.forbiddenArea.lineNumber);
}

std::list<tForbiddenLine> CStoreAwsData::getSendForbiddenLine()
{
    return data.forbiddenArea.line;
}
short CStoreAwsData::getSendForbiddenLineNum()
{
    return data.forbiddenArea.lineNumber;
}

void CStoreAwsData::setSendForbiddenRect()
{
    data.forbiddenArea.bUpdate = true;
    data.forbiddenArea.type = 2;
    tForbiddenRect* rectPtr = ServiceData.rsBridge.getForbiddenRect().first;
    if(!data.forbiddenArea.rect.empty()){
        data.forbiddenArea.rect.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        data.forbiddenArea.rect.push_back(rectPtr[i]);
        eblog(LOG_LV_AWS," ForbiddenRect "<<"[" << i << "] : x = " << data.forbiddenArea.rect.back().x 
                                <<"[" << i << "] : y = " << data.forbiddenArea.rect.back().y
                                <<"[" << i << "] : w = " << data.forbiddenArea.rect.back().w
                                <<"[" << i << "] : h = " << data.forbiddenArea.rect.back().h); 
    }
    if (ServiceData.rsBridge.getForbiddenRect().second == 0xff){
        data.forbiddenArea.rectNumber = 0;
    }
    else    data.forbiddenArea.rectNumber = ServiceData.rsBridge.getForbiddenRect().second;
    eblog(LOG_LV_AWS," service customNumber : "<<data.forbiddenArea.rectNumber);
}
std::list<tForbiddenRect> CStoreAwsData::getSendForbiddenRect()
{
    return data.forbiddenArea.rect;
}
short CStoreAwsData::getSendForbiddenRectNum()
{
    return data.forbiddenArea.rectNumber;   
}
bool CStoreAwsData::isForbiddenAreaUpdate()
{
    bool ret = false; 
    if(data.forbiddenArea.bUpdate){
        ret = data.forbiddenArea.bUpdate;
        data.forbiddenArea.bUpdate = false;
    }
    return ret;
}

/* areaInfo */
void CStoreAwsData::setSendAreaInfo()
{
    data.areaInfo.bUpdate =true;
    // tAreaInfo* areaInfoPtr = ServiceData.rsBridge.getAreaInfo().first;
    // if(!data.areaInfo.info.empty())
    // {
    //     data.areaInfo.info.clear();
    // }
    // for (int i = 0; i < MAXLINE; ++i) {
    //     data.areaInfo.info.push_back(areaInfoPtr[i]);
    //     eblog(LOG_LV_AWS," areaInfo.info "<<"[" << i << "] : id = " << data.areaInfo.info.back().id 
    //                             <<"[" << i << "] : x1 = " << data.areaInfo.info.back().x1
    //                             <<"[" << i << "] : y1 = " << data.areaInfo.info.back().y1
    //                             <<"[" << i << "] : x2 = " << data.areaInfo.info.back().x2
    //                             <<"[" << i << "] : y2 = " << data.areaInfo.info.back().y2
    //                             <<"[" << i << "] : roomName = " << data.areaInfo.info.back().name
    //                             <<"[" << i << "] : color = " << data.areaInfo.info.back().color);
    // }

    // data.areaInfo.areaNumber = ServiceData.rsBridge.getAreaInfo().second;
    // eblog(LOG_LV_AWS,"areaInfo.areaNumber : "<<data.areaInfo.areaNumber);    
}
std::list<tAreaInfo> CStoreAwsData::getSendAreaInfo(){return data.areaInfo.info;}
short CStoreAwsData::getSendAreaInfoNum(){return data.areaInfo.areaNumber;}
bool CStoreAwsData::isAreaInfoUpdate()
{
    bool ret = false; 
    if(data.areaInfo.bUpdate){
        ret = data.areaInfo.bUpdate;
        data.areaInfo.bUpdate = false;
    }
    return ret;
}

void CStoreAwsData::setSendDivideArea(tDivideArea *pData)
{
    data.divideArea.bUpdate = true;
    data.divideArea.data.id = pData->id;
    for(int i = 0; i < 4; i++){
        data.divideArea.data.point[i] = pData->point[i];
    }
    
}
bool CStoreAwsData::isDivideAreaUpdate()
{
    bool ret = false; 
    if(data.divideArea.bUpdate){
        ret = true;
        data.divideArea.bUpdate = false;
    }
    return ret;
}
void CStoreAwsData::setSendCombineArea(tCombieArea *pData)
{
    data.combineArea.bUpdate = true;
    for(int i = 0; i < 2; i++){
        data.combineArea.data.point[i] = pData->point[i];
    }
}
bool CStoreAwsData::isCombineAreaUpdate()
{
    bool ret = false; 
    if(data.combineArea.bUpdate){
        ret = true;
        data.combineArea.bUpdate = false;
    }
    return ret;
}


/* 방해금지 */
void CStoreAwsData::setSendDontDisturbStatus(){ data.distruptData.status = ServiceData.rsBridge.getDontDisturbStatus(); data.distruptData.bUpdate = true;}
short CStoreAwsData::getSendDontDisturbStatus(){return data.distruptData.status;}

void CStoreAwsData::setSendDontDisturbStartTime(){data.distruptData.startTime = ServiceData.rsBridge.getDontDisturbStartTime(); data.distruptData.bUpdate = true;
if(data.distruptData.startTime != nullptr){
    eblog(LOG_LV_AWS,"distruptData.startTime : "<<data.distruptData.startTime);
}

}
char* CStoreAwsData::getSendDontDisturbStartTime(){return data.distruptData.startTime;}

void CStoreAwsData::setSendDontDisturbEndTime(){data.distruptData.endTime = ServiceData.rsBridge.getDontDisturbEndTime(); data.distruptData.bUpdate = true;
if(data.distruptData.endTime != nullptr){
    eblog(LOG_LV_AWS,"distruptData.endTime : "<<data.distruptData.endTime);}
}

char* CStoreAwsData::getSendDontDisturbEndTime(){return data.distruptData.endTime;}
bool CStoreAwsData::isDistruptDataUpdate()
{
    bool ret = false; 
    if(data.distruptData.bUpdate){
        ret = data.distruptData.bUpdate;
        data.distruptData.bUpdate = false;
    }
    return ret;
}

/* 예약청소 */
void CStoreAwsData::setSendCleanSchedule()
{
    data.rsvCleanData.bUpdate = true;

    tCleanSchedule* cleaningSchedulePtr = ServiceData.rsBridge.getCleanSchedule().first;
    data.rsvCleanData.scheduleNumber = ServiceData.rsBridge.getCleanSchedule().second;
    
    if(!data.rsvCleanData.schedule.empty()){
        data.rsvCleanData.schedule.clear();
    }
    for (int i = 0; i < data.rsvCleanData.scheduleNumber; ++i)
    {
        data.rsvCleanData.schedule.push_back(cleaningSchedulePtr[i]);
        eblog(LOG_LV_AWS," CleanSchedule "<<"[" << i << "] : time = " << data.rsvCleanData.schedule.back().time 
        <<"[" << i << "] : active Day = " << data.rsvCleanData.schedule.back().weeks
        <<"[" << i << "] : clean mode = " << data.rsvCleanData.schedule.back().mode
        <<"[" << i << "] : water Lv = " << data.rsvCleanData.schedule.back().waterLevel
        <<"[" << i << "] : areas = " << data.rsvCleanData.schedule.back().areas
        <<"[" << i << "] : isEnabled = " << data.rsvCleanData.schedule.back().isEnabled
        <<"[" << i << "] : isValid = " << data.rsvCleanData.schedule.back().isValid);
    }
}
#if 1

std::list<tCleanSchedule> CStoreAwsData::getSendCleaningSchedule(){return data.rsvCleanData.schedule;}
short CStoreAwsData::getSendCleanScheduleNum(){return data.rsvCleanData.scheduleNumber;}
bool CStoreAwsData::isRsvCleanDataUpdate()
{
    bool ret = false; 
    if(data.rsvCleanData.bUpdate){
        ret = data.rsvCleanData.bUpdate;
        data.rsvCleanData.bUpdate = false;
    }
    return ret;
}
#endif
/* OTA */

void CStoreAwsData::setSendOtaForce(){data.otaInfo.force = ServiceData.rsBridge.getOtaForce(); data.otaInfo.bUpdate = true;}
short CStoreAwsData::getSendOtaForce() {return data.otaInfo.force;}

void CStoreAwsData::setSendOtaName(){strcpy(data.otaInfo.name, ServiceData.rsBridge.getOtaName()); data.otaInfo.bUpdate = true;}
char* CStoreAwsData::getSendOtaName() {return data.otaInfo.name;}

void CStoreAwsData::setSendOtaVersion(){strcpy(data.otaInfo.version, ServiceData.rsBridge.getOtaVersion()); data.otaInfo.bUpdate = true;}
char* CStoreAwsData::getSendOtaVersion(){return data.otaInfo.version;}

void CStoreAwsData::setSendOtaScheduled(){data.otaInfo.scheduled = ServiceData.rsBridge.getOtaScheduled(); data.otaInfo.bUpdate = true;}
short CStoreAwsData::getSendOtaScheduled(){return data.otaInfo.scheduled;}

void CStoreAwsData::setSendOtaScheduleTime(){strcpy(data.otaInfo.scheduleTime, ServiceData.rsBridge.getOtaScheduleTime()); data.otaInfo.bUpdate = true;}
char* CStoreAwsData::getSendOtaScheduleTime(){return data.otaInfo.scheduleTime;}
bool CStoreAwsData::isOtaDataUpdate()
{
    bool ret = false; 
    if(data.otaInfo.bUpdate){
        ret = data.otaInfo.bUpdate;
        data.otaInfo.bUpdate = false;
    }
    return ret;
}


/* clean history */
void CStoreAwsData::setSendCleanAreaInfo(std::string areaInfo)
{
    data.cleanHistory.areaInfo = areaInfo; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanTraj(std::list<tPoint> traj)
{
    data.cleanHistory.traj = traj; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanRobotPose(tPose robotPose)
{
    data.cleanHistory.robotPose = robotPose; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanCradlePose(tPoint cradlePose)
{
    data.cleanHistory.cradlePose = cradlePose; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanStartTime(std::string cleanStartTime)
{
    data.cleanHistory.cleanStartTime = cleanStartTime; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanExitReason(std::string exitReason)
{
    data.cleanHistory.exitReason = exitReason; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanedSize(int cleanedSize)
{
    data.cleanHistory.cleanedSize = cleanedSize; data.cleanHistory.bUpdate = true;
}
void CStoreAwsData::setSendCleanTime(int cleanTime)
{
    data.cleanHistory.cleanTime = cleanTime; data.cleanHistory.bUpdate = true;
}
tAwsCleanHistory CStoreAwsData::getSendCleanHistory(){return data.cleanHistory;}

bool CStoreAwsData::isCleanHistoryUpdate()
{
    bool ret = false; 
    if(data.cleanHistory.bUpdate){
        ret = data.cleanHistory.bUpdate;
        data.cleanHistory.bUpdate = false;
    }
    return ret;
}


/* SaveMap */
void CStoreAwsData::setSendSaveMapRobotPose(tPose robotPose){data.saveMapInfo.robotPose = robotPose; data.saveMapInfo.bUpdate = true;}
void CStoreAwsData::setSendSaveMapTraj(std::list<tPoint> traj){data.saveMapInfo.traj = traj; data.saveMapInfo.bUpdate = true;}
void CStoreAwsData::setSendSaveMapUniqueKey(std::string key){data.saveMapInfo.uniqueKey = key; data.saveMapInfo.bUpdate = true;}
void CStoreAwsData::setSendSaveMapName(std::string mapName){data.saveMapInfo.mapName = mapName; data.saveMapInfo.bUpdate = true;}
void CStoreAwsData::setSendSaveMapOrder(int order){data.saveMapInfo.order = order; data.saveMapInfo.bUpdate = true;}
void CStoreAwsData::setSendSaveMapAreaInfo(std::string areaInfo){data.saveMapInfo.areaInfo = areaInfo; data.saveMapInfo.bUpdate = true;}

tAwsSaveMapInfo CStoreAwsData::getSendSaveMap(){return data.saveMapInfo;}
bool CStoreAwsData::isSaveMapUpdate()
{
    bool ret = false; 
    if(data.saveMapInfo.bUpdate){
        ret = data.saveMapInfo.bUpdate;
        data.saveMapInfo.bUpdate = false;
    }
    return ret;
}