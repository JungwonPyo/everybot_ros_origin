/**
 * @file rosKey.h
 * @author jspark
 * @brief debug용 ros key 데이터 가공 클래스.
 * @date 2023-05-09
 */
#pragma once

#include "commonStruct.h"

enum class E_ROS_KEY
{
    NONE,
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

    WATER_OPTION,               //물공급 설정

    //test key
    MOVING_UP,
    MOVING_DOWN,
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

static std::string enumToString(E_ROS_KEY value) {
    static const std::unordered_map<E_ROS_KEY, std::string> enumToStringMap = {
        { E_ROS_KEY::NONE, "NONE," },
        { E_ROS_KEY::POWER_OFF, "POWER_OFF," },
        { E_ROS_KEY::CLEAN, "CLEAN," },
        { E_ROS_KEY::EXPLORER, "EXPLORER," },
        { E_ROS_KEY::HOME, "HOME," },
        { E_ROS_KEY::STOP, "STOP," },
        { E_ROS_KEY::DRY_MOP, "DRY_MOP," },
        { E_ROS_KEY::DRAIN_WATER, "DRAIN_WATER," },
        { E_ROS_KEY::APP_CONNECT, "APP_CONNECT," },
        { E_ROS_KEY::DELETE_MAP, "DELETE_MAP," },
        { E_ROS_KEY::INIT_USER_OPTION, "INIT_USER_OPTION," },
        { E_ROS_KEY::RESERVAITON_CLEAN, "RESERVAITON_CLEAN," },
        { E_ROS_KEY::FW_UPDATE, "FW_UPDATE," },
        { E_ROS_KEY::FW_RECOVERY, "FW_RECOVERY," },
        { E_ROS_KEY::FACTORY_RESET, "FACTORY_RESET," },

        { E_ROS_KEY::WATER_OPTION, "WATER_OPTION," },

        { E_ROS_KEY::MOVING_UP, "MOVING_UP," },
        { E_ROS_KEY::MOVING_DOWN, "MOVING_DOWN," },
        { E_ROS_KEY::MOVING_LEFT, "MOVING_LEFT," },
        { E_ROS_KEY::MOVING_RIGHT, "MOVING_RIGHT," },
        { E_ROS_KEY::MOVING_STOP, "MOVING_STOP," },
        { E_ROS_KEY::TILTING_UP, "TILTING_UP," },
        { E_ROS_KEY::TILTING_DONW, "TILTING_DONW," },
        { E_ROS_KEY::SLAM_ON, "SLAM_ON," },
        { E_ROS_KEY::SAVE_MAP, "SAVE_MAP," },
        { E_ROS_KEY::SLAM_OFF, "SLAM_OFF," },
        { E_ROS_KEY::IMU_INIT, "IMU_INIT," },
        { E_ROS_KEY::TOF_INIT, "TOF_INIT," },

        { E_ROS_KEY::AP_RESET, "AP_RESET," },
        { E_ROS_KEY::MCU_RESET, "MCU_RESET," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CRosKey
{
private:
    E_ROS_KEY rosKey;

public:
    CRosKey();
    ~CRosKey();
    void initRosKey();
    void setRosKey(std::string cmd);
    E_ROS_KEY getRosKey();
};