#include "deviceHandler.h"
#include "userInterface.h"
#include "control/control.h"
#include "eblog.h"

CDeviceHandler::CDeviceHandler(CServiceMacro* _pServiceMacro) : pServiceMacro(_pServiceMacro)
{
    if (AWS_MODE==0)
        setWifiState(WIFI_CONNECT_STATE::START_STATION_CONNECT);
    else if (AWS_MODE==1)
        setWifiState(WIFI_CONNECT_STATE::START_AWS_CONNECT);
}

CDeviceHandler::~CDeviceHandler()
{
    setWifiState(WIFI_CONNECT_STATE::VOID);
}

void CDeviceHandler::setWifiState(WIFI_CONNECT_STATE set)
{
    if (set != wifiState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[WIFI_CONNECT_STATE] : "<< enumToString(wifiState)<<" --> "<< enumToString(set) );
    }
    wifiState = set;
}

WIFI_CONNECT_STATE CDeviceHandler::getWifiState(){
    return wifiState;
}
void CDeviceHandler::powerChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curState)
{
    E_TERMINAL_STATE terminalState = ServiceData.power.getTerminalState();
    
    if(curId != E_SERVICE_ID::WIFI)
    {
        if(curId == E_SERVICE_ID::CHARGING && curState != E_SERVICE_STATUS::completed)
        {
            if(terminalState != E_TERMINAL_STATE::DOCKED)   pServiceMacro->macroServiceStop(curId);
        }
        else if(curId != E_SERVICE_ID::UNDOCKING && curState != E_SERVICE_STATUS::completed)
        {
            if(terminalState == E_TERMINAL_STATE::DOCKED)   pServiceMacro->macroServiceCharge(curId);
        }
    }
}

void CDeviceHandler::batteryChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curState)
{
    bool bStateChanged = ServiceData.battery.isUpateState();
    bool bNeedCharge = ServiceData.battery.isBatteryNeedCharge();
    short percent = (short)ServiceData.battery.getBatteryPercent();
    E_BATTERY_STATE battState = ServiceData.battery.getBatteryState();

    if(bNeedCharge)
    {
        if(curId == E_SERVICE_ID::IDLE && curState != E_SERVICE_STATUS::completed  && SYSTEM_TOOL.getSystemTime()-ServiceData.battery.getStateChangeTime() >= 60*5){
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_LOW_BATT_MOVE_TO_CHARGER);
            pServiceMacro->macroServiceStart(curId, E_SERVICE_ID::DOCKING);
        }
        else if((curId == E_SERVICE_ID::CLEAN || curId == E_SERVICE_ID::EXPLORER) && (curState != E_SERVICE_STATUS::completed)){
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_LOW_BATT_MOVE_TO_CHARGER);
            pServiceMacro->macroServiceStart(curId, E_SERVICE_ID::DOCKING);
        }
    }

    if(bStateChanged){
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "setSendBattery Percent : " << percent);
        ServiceData.awsData.setSendBattery(percent);
    }
}

WIFI_CONNECT_STATE CDeviceHandler::startStationConnect(){
    WIFI_CONNECT_STATE ret = WIFI_CONNECT_STATE::START_STATION_CONNECT;
    ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Start Station Connect!!");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_STATION)));
    ret = WIFI_CONNECT_STATE::STATION_CONNECTING;
    return ret;
}

WIFI_CONNECT_STATE CDeviceHandler::runStationConnecting(){
    WIFI_CONNECT_STATE ret = WIFI_CONNECT_STATE::STATION_CONNECTING;
    if(ServiceData.rsBridge.getStationConnection() == E_AP_STAION_CONNECTION_STATE::COMPLETE ){
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Station Connect Success!!");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION)));
        ret = WIFI_CONNECT_STATE::START_AWS_CONNECT;
    }else{
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Station Connecting......");
    }

    return ret;
}

WIFI_CONNECT_STATE CDeviceHandler::startAwsConnect(){
    WIFI_CONNECT_STATE ret = WIFI_CONNECT_STATE::START_AWS_CONNECT;
    ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Start Aws Connect Complete!!");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_AWS)));
    ret = WIFI_CONNECT_STATE::COMPLETE;
    return ret;
}
WIFI_CONNECT_STATE CDeviceHandler::runAwsConnecting(){
    WIFI_CONNECT_STATE ret = WIFI_CONNECT_STATE::COMPLETE;

    if (ServiceData.rsBridge.getconnect()){
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Aws Connect Good~~");
    }else{
        ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Aws lost-Connect....");
    }

    return ret;
}

void CDeviceHandler::wifiConnectChecker()
{
    switch (wifiState)
    {
    case WIFI_CONNECT_STATE::VOID:

        break;
    case WIFI_CONNECT_STATE::START_STATION_CONNECT:
        setWifiState(startStationConnect()); 
        break;
    case WIFI_CONNECT_STATE::STATION_CONNECTING:
        setWifiState(runStationConnecting());
        break;
    case WIFI_CONNECT_STATE::START_AWS_CONNECT:
        setWifiState(startAwsConnect());
        break;
    case WIFI_CONNECT_STATE::COMPLETE:
        setWifiState(runAwsConnecting());
        break;                                     
    default:
        break;
    }

    return;
} 

void CDeviceHandler::skipConnectCheck()
{
    ceblog(LOG_LV_NECESSARY, BOLDCYAN, "Skip WiFi connect");
    setWifiState(WIFI_CONNECT_STATE::COMPLETE);
}