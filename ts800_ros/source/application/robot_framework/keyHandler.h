#pragma once

#include "commonStruct.h"
#include "coreData/serviceData.h"
#include "coreData/serviceData/keyState.h"
#include "supplyWater.h"
#include "taskCharging.h"
#include "serviceMacro.h"
#include "service.h"
#include "userInterface.h"

enum class E_KEY_VALUE
{
    NONE,
    POWER_OFF,
    
    STOP,
    
    CLEAN_START,
    CLEAN_PAUSE,
    CLEAN_RESUME,

    HOME_START,
    HOME_PAUSE,
    HOME_RESUME,

    EXPLORER_START,
    EXPLORER_PAUSE,
    EXPLORER_RESUME,

    START_STOP_DRY_MOP,
    START_STOP_DRAIN_WATER,
    
    START_APP_CONNECT,
    START_INIT_USERSET,
    START_RESERVAITON_CLEAN, //예약청소 시작
    START_RESERVATION_STOP,  //방해금지 시작

    START_FW_UPDATE,
    START_FW_RECOVERY,
    START_FACTORY_RESET,

    DELETE_MAP,

    WATER_OPTION,

//test key

    MOVING_UP,
    MOVING_UPLEFT,
    MOVING_UPRIGHT,
    MOVING_DOWN,
    MOVING_DOWNLEFT,
    MOVING_DOWNRIGHT,
    MOVING_LEFT,
    MOVING_RIGHT,
    MOVING_STOP,

    TILTING_UP,
    TILTING_DONW,

    SLAM_ON,
    SAVE_MAP,
    SLAM_OFF,
    IMU_INIT,
    TOF_INIT,

    AP_RESET,
    MCU_RESET,
};

static std::string enumToString(E_KEY_VALUE value) {
    static const std::unordered_map<E_KEY_VALUE, std::string> enumToStringMap = {
        { E_KEY_VALUE::NONE, "NONE," },
        { E_KEY_VALUE::POWER_OFF, "POWER_OFF," },
        { E_KEY_VALUE::STOP, "STOP," },
        { E_KEY_VALUE::CLEAN_START, "CLEAN_START," },
        { E_KEY_VALUE::CLEAN_PAUSE, "CLEAN_PAUSE," },
        { E_KEY_VALUE::CLEAN_RESUME, "CLEAN_RESUME," },
        { E_KEY_VALUE::HOME_START, "HOME_START," },
        { E_KEY_VALUE::HOME_PAUSE, "HOME_PAUSE," },
        { E_KEY_VALUE::HOME_RESUME, "HOME_RESUME," },
        { E_KEY_VALUE::DELETE_MAP, "DELETE_MAP," },
        { E_KEY_VALUE::EXPLORER_START, "EXPLORER_START," },
        { E_KEY_VALUE::EXPLORER_PAUSE, "EXPLORER_PAUSE," },
        { E_KEY_VALUE::EXPLORER_RESUME, "EXPLORER_RESUME," },
        { E_KEY_VALUE::START_STOP_DRY_MOP, "START_STOP_DRY_MOP," },
        { E_KEY_VALUE::START_STOP_DRAIN_WATER, "START_STOP_DRAIN_WATER," },
        { E_KEY_VALUE::START_APP_CONNECT, "START_APP_CONNECT," },
        { E_KEY_VALUE::START_INIT_USERSET, "START_INIT_USERSET," },
        { E_KEY_VALUE::START_RESERVAITON_CLEAN, "START_RESERVAITON_CLEAN," },
        { E_KEY_VALUE::START_RESERVATION_STOP, "START_RESERVATION_STOP," },
        { E_KEY_VALUE::START_FW_UPDATE, "START_FW_UPDATE," },
        { E_KEY_VALUE::START_FW_RECOVERY, "START_FW_RECOVERY," },
        { E_KEY_VALUE::START_FACTORY_RESET, "START_FACTORY_RESET," },
        { E_KEY_VALUE::DELETE_MAP, "DELETE_MAP," },
        { E_KEY_VALUE::WATER_OPTION, "WATER_OPTION," },

        { E_KEY_VALUE::MOVING_UP, "MOVING_UP," },
        { E_KEY_VALUE::MOVING_DOWN, "MOVING_DOWN," },
        { E_KEY_VALUE::MOVING_LEFT, "MOVING_LEFT," },
        { E_KEY_VALUE::MOVING_RIGHT, "MOVING_RIGHT," },
        { E_KEY_VALUE::MOVING_STOP, "MOVING_STOP," },
        { E_KEY_VALUE::TILTING_UP, "TILTING_UP," },
        { E_KEY_VALUE::TILTING_DONW, "TILTING_DONW," },
        { E_KEY_VALUE::SLAM_ON, "SLAM_ON," },
        { E_KEY_VALUE::SAVE_MAP, "SAVE_MAP," },
        { E_KEY_VALUE::SLAM_OFF, "SLAM_OFF," },
        { E_KEY_VALUE::IMU_INIT, "IMU_INIT," },
        { E_KEY_VALUE::TOF_INIT, "TOF_INIT," },

        { E_KEY_VALUE::AP_RESET, "AP_RESET," },
        { E_KEY_VALUE::MCU_RESET, "MCU_RESET," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CKeyHanler
{

private:
    CServiceMacro   *pServiceMacro;
    CSupplyWater    *pSupplyWater;
    CTaskCharging   *pTaskCharging;

    bool    bServiceUdate;
    
public:
    CKeyHanler(CServiceMacro* _pServiceMacro, CSupplyWater* _pSupplyWater, CTaskCharging* _pTaskCharging);
    ~CKeyHanler();

    bool isUpdateKeyService();
    void keyChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus, bool bNeedCharge, bool *powerOff, bool *FirmwareUpdate);
private:
    E_KEY_VALUE convertKeyValue(E_KEY_TYPE type, E_SERVICE_ID curId, E_SERVICE_STATUS curState);
    
    bool isKeyValidate(E_KEY_VALUE keyVel, E_SERVICE_ID svcId, E_SERVICE_STATUS svcStatus, bool battLow);
    void keyProc(E_KEY_VALUE keyVel, E_SERVICE_ID curService ,bool *powerOff, bool *FirmwareUpdate);
    void monitorAppOption();
    E_SERVICE_ID getDoingServiceId(E_KEY_TYPE type,E_SERVICE_ID curId);
    E_SERVICE_STATUS getDoingServiceState(E_SERVICE_ID doId, E_SERVICE_ID curId, E_SERVICE_STATUS curState);
    E_KEY_VALUE getServiceKeyValue(E_SERVICE_ID doId,E_SERVICE_STATUS doState);
    E_KEY_VALUE getServicePauseKey(E_SERVICE_ID doId);
    E_KEY_VALUE getServiceResumeKey(E_SERVICE_ID doId);
    E_KEY_VALUE getServiceStartKey(E_SERVICE_ID doId);
    void keyPowerOff();
    void keySoundVolume(short _soundLv);
    E_KEY_VALUE checkTriggerOption(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus);
};