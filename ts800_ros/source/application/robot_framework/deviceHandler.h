#pragma once

#include "commonStruct.h"
#include "coreData/serviceData.h"
#include "serviceMacro.h"
#include "service.h"

enum class WIFI_CONNECT_STATE
{
    VOID,      //부팅 연결 처음 단계
    START_STATION_CONNECT,//부팅 연결 Station 연결
    STATION_CONNECTING,
    START_AWS_CONNECT,       //부팅 연결 station 연결 실패
    COMPLETE,   //부팅 연결 station 연결 성공
};

static std::string enumToString(WIFI_CONNECT_STATE value) {
    static const std::unordered_map<WIFI_CONNECT_STATE, std::string> enumToStringMap = {
        { WIFI_CONNECT_STATE::VOID, "VOID," },
        { WIFI_CONNECT_STATE::START_STATION_CONNECT, "START_STATION_CONNECT," },
        { WIFI_CONNECT_STATE::STATION_CONNECTING, "STATION_CONNECTING," },
        { WIFI_CONNECT_STATE::START_AWS_CONNECT, "START_AWS_CONNECT," },
        { WIFI_CONNECT_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CDeviceHandler
{

private:
    CServiceMacro   *pServiceMacro;
    WIFI_CONNECT_STATE wifiState;
public:
    CDeviceHandler(CServiceMacro* _pServiceMacro);
    ~CDeviceHandler();

    void powerChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curState);
    void batteryChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curState);
    void wifiConnectChecker();
    WIFI_CONNECT_STATE getWifiState();
    void skipConnectCheck();
private:
    void setWifiState(WIFI_CONNECT_STATE set);
    WIFI_CONNECT_STATE startStationConnect();
    WIFI_CONNECT_STATE runStationConnecting();
    WIFI_CONNECT_STATE startAwsConnect();
    WIFI_CONNECT_STATE runAwsConnecting();
};