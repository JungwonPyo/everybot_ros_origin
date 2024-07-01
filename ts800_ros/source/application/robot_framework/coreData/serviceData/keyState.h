/**
 * @file keyState.h
 * @author jspark
 * @brief keyState 의 가공된 데이터를 저장하고 있음.
 * @date 2023-05-09
 */
#pragma once

#include "coreData/observer.h"
#include "coreData/serviceData/rosKey.h"
#include "coreData/serviceData/button.h"
#include "coreData/serviceData/appKey.h"


enum class E_KEY_TYPE
{
    VOID,
    POWER_OFF,

    CLEAN,
    EXPLORER,
    HOME,
    STOP,
    DRY_MOP,
    DRAIN_WATER,
    APP_CONNECT,
    DELETE_MAP,

    INIT_USER_OPTION,
    RESERVAITON_CLEAN, //예약청소 시작
    RESERVATION_STOP,  //방해금지 시작

    FW_UPDATE,
    FW_RECOVERY,
    FACTORY_RESET,

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

static std::string enumToString(E_KEY_TYPE value) {
    static const std::unordered_map<E_KEY_TYPE, std::string> enumToStringMap = {
        { E_KEY_TYPE::VOID, "VOID," },
        { E_KEY_TYPE::POWER_OFF, "POWER_OFF," },
        { E_KEY_TYPE::CLEAN, "CLEAN," },
        { E_KEY_TYPE::EXPLORER, "EXPLORER," },
        { E_KEY_TYPE::HOME, "HOME," },
        { E_KEY_TYPE::STOP, "STOP," },
        { E_KEY_TYPE::DRY_MOP, "DRY_MOP," },
        { E_KEY_TYPE::DRAIN_WATER, "DRAIN_WATER," },
        { E_KEY_TYPE::APP_CONNECT, "APP_CONNECT," },
        { E_KEY_TYPE::DELETE_MAP, "DELETE_MAP," },
        { E_KEY_TYPE::INIT_USER_OPTION, "INIT_USER_OPTION," },
        { E_KEY_TYPE::RESERVAITON_CLEAN, "RESERVAITON_CLEAN," },
        { E_KEY_TYPE::FW_UPDATE, "FW_UPDATE," },
        { E_KEY_TYPE::FW_RECOVERY, "FW_RECOVERY," },
        { E_KEY_TYPE::FACTORY_RESET, "FACTORY_RESET," },
        { E_KEY_TYPE::WATER_OPTION, "WATER_OPTION," },

        { E_KEY_TYPE::MOVING_UP, "MOVING_UP," },
        { E_KEY_TYPE::MOVING_DOWN, "MOVING_DOWN," },
        { E_KEY_TYPE::MOVING_LEFT, "MOVING_LEFT," },
        { E_KEY_TYPE::MOVING_RIGHT, "MOVING_RIGHT," },
        { E_KEY_TYPE::MOVING_STOP, "MOVING_STOP," },
        { E_KEY_TYPE::TILTING_UP, "TILTING_UP," },
        { E_KEY_TYPE::TILTING_DONW, "TILTING_DONW," },
        { E_KEY_TYPE::SLAM_ON, "SLAM_ON," },
        { E_KEY_TYPE::SAVE_MAP, "SAVE_MAP," },
        { E_KEY_TYPE::SLAM_OFF, "SLAM_OFF," },
        { E_KEY_TYPE::IMU_INIT, "IMU_INIT," },
        { E_KEY_TYPE::TOF_INIT, "TOF_INIT," },

        { E_KEY_TYPE::AP_RESET, "AP_RESET," },
        { E_KEY_TYPE::MCU_RESET, "MCU_RESET," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef struct _tAppOption
{
    bool bForbiddenLine;
    bool bForbiddenRect;
    bool bOperationAll;
    bool bOperationSpot;
    bool bOperationRoom;
    bool bOperationCustom;
    bool bAreaInfo;
    bool bDivideArea;
    bool bCombineArea;
    bool bWaterLevel;
    bool bSoundVolume;
    bool bDryMop;
    bool bScheduleClean;
    bool bDistruptMode;
    bool bCleanMode;
    bool bLanguage;
    bool bCountry;
    bool bOta;

}tAppOption;

class CKeyState : public CObserver
{
private:
    E_KEY_TYPE key; // 가공된 최종 key 상태.
    CButton button;
    CRosKey rosCmd;
    CAppKey appCmd;
    tAppOption appOption;


    void initKeyState();

public:
    CKeyState();
    ~CKeyState();
    void update(CExternData* pExternData) override;
    void setKeyValue(E_KEY_TYPE newKey);    
    E_KEY_TYPE getKeyValue(void);

    void convertButtonKey(E_FUNCTION_BUTTON button);
    void convertAwsActionKey(E_ACTION_KEY trigger);
    void convertRosKey(E_ROS_KEY rosKey);
    void setAppOption(CExternData* pExternData);
    
    bool isUpdatForbiddenLine();
    bool isUpdateForbiddenRect();
    bool isUpdatOperationAll();
    bool isUpdatOperationSpot();
    bool isUpdatOperationRoom();
    bool isUpdatOperationCustom();
    bool isUpdatDivideArea();
    bool isUpdatCombineArea();
    bool isUpdatAreaInfo();
    bool isUpdatWaterLevel();
    bool isUpdatSoundVolume();
    bool isUpdatDryMop();
    bool isUpdatScheduleClean();
    bool isUpdatDistruptMode();
    bool isUpdatCleanMode();
    bool isUpdatLanguage();
    bool isUpdatCountry();
    bool isUpdatOta();


public: /* 쓰는 것만 올려주세요 */
    tKey keys; // 키 입력 상태 정보
    tKey old_keys;
    s8 old_key_count;    
    s8 old_remocon_count = 0;
    
    u32 double_key_display_time = (u32)-1;
    u32 double_key_Check_time = (u32)-1;
    bool double_key_On_WifiRest = false;
    bool double_key_On_FactoryRest = false;
    bool FactoryResetMode = false;

//debug key
    tUpdateData<tPose> targetPose; // debug용 target 위치좌표
    tUpdateData<tTwist> velocityControl;

};
