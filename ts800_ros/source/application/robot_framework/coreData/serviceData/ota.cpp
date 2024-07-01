#include "coreData/serviceData/ota.h"
#include "coreData/serviceData.h"
#include "eblog.h"
#include "externData/externData.h"

COta::COta(){
    bRunning = false;
    const char* pPath = "/userdata/ota/";
    std::strcpy(path, pPath);
}
COta::~COta(){}

void COta::update(CExternData* pExternData)
{
    if ( pExternData->systemData.isUpdateOtaData() == true )
    {
        updateSysOtaData( pExternData->systemData.useOtaData() );		
    }
}

void COta::updateSysOtaData(tSysOta _sysData)
{
    sysData.bOpen = _sysData.bOpen;
    sysData.state = _sysData.pecent;
    sysData.pecent = _sysData.state;
    //eblog(LOG_LV_NECESSARY, "SYS - OTA DTA UPDATE : ");
}

void COta::setAppData()
{
    if(ServiceData.rsBridge.getOtaForce() != 0){
        appData.force = ServiceData.rsBridge.getOtaForce();
        ServiceData.awsData.setSendOtaForce();
        
        eblog(LOG_LV_NECESSARY, "OTA Force : "<<appData.force);
    }
    if(ServiceData.rsBridge.getOtaName() != nullptr){
        strcpy(appData.name, ServiceData.rsBridge.getOtaName());
        ServiceData.awsData.setSendOtaName();
        if(appData.name != nullptr){
            eblog(LOG_LV_NECESSARY, "OTA NAME : "<<appData.name);
        }
   
        std::strcpy(otaPath, path);
        std::strcat(otaPath, appData.name); // 또는 strcat로 b를 이어붙일 수도 있습니다.
    }
    if(ServiceData.rsBridge.getOtaVersion() != nullptr){
        strcpy(appData.version, ServiceData.rsBridge.getOtaVersion());
        ServiceData.awsData.setSendOtaVersion();
        if(appData.version != nullptr){
            eblog(LOG_LV_NECESSARY, "OTA Version : "<< appData.version);
        }
        
    }
    if(ServiceData.rsBridge.getOtaScheduled() != 0){
        appData.scheduled = ServiceData.rsBridge.getOtaScheduled();
        ServiceData.awsData.setSendOtaScheduled();
        eblog(LOG_LV_NECESSARY, "OTA Scheduled : "<< appData.scheduled);
    }
    if(ServiceData.rsBridge.getOtaScheduleTime() != nullptr){
        strcpy(appData.scheduleTime, ServiceData.rsBridge.getOtaScheduleTime());
        ServiceData.awsData.setSendOtaScheduleTime();
        if(appData.scheduleTime != nullptr){
            eblog(LOG_LV_NECESSARY, "OTA ScheduleTime : "<< appData.scheduleTime);
        }
    }
}

tappOta COta::getAppData()
{
    return appData;
}

tSysOta COta::getSysData()
{
    return sysData;
}

void COta::setFirmWareUpdateFlag(bool set)
{
    if(set){
        eblog(LOG_LV_NECESSARY, "FirmWareUpdate Start!");
    }
    else{
        eblog(LOG_LV_NECESSARY, "FirmWareUpdate Stop!");
    }
    bRunning = set;
}
bool COta::isRunning()
{
    return bRunning;
}

char* COta::getOtaPath()
{
    return otaPath;
}