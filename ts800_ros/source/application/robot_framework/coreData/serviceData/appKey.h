/**
 * @file actionKey.h
 * @author jmk1
 * @brief debug용 app Key 데이터 가공 클래스.
 * @date 2023-08-22
 */
#pragma once
#include "Message.h"
#include "commonStruct.h"
#include "interfaceStruct.h"

enum class E_ACTION_KEY
{
    NONE,
    CLEAN,
    HOMING,
    EXPLORER,
    STOP,

    START_STOP_DRY_MOP,
    START_STOP_DRAIN_WATER,

    START_INIT_USERSET,
    START_FW_UPDATE,
    START_FW_RECOVERY,
    START_FACTORY_RESET,

    DELETE_MAP,
};

static std::string enumToString(E_ACTION_KEY value) {
    static const std::unordered_map<E_ACTION_KEY, std::string> enumToStringMap = {
        { E_ACTION_KEY::NONE, "NONE," },
        { E_ACTION_KEY::CLEAN, "CLEAN," },
        { E_ACTION_KEY::HOMING, "HOMING," },
        { E_ACTION_KEY::EXPLORER, "EXPLORER," },
        { E_ACTION_KEY::STOP, "STOP," },
        { E_ACTION_KEY::START_STOP_DRY_MOP, "START_STOP_DRY_MOP," },
        { E_ACTION_KEY::START_STOP_DRAIN_WATER, "START_STOP_DRAIN_WATER," },
        { E_ACTION_KEY::START_INIT_USERSET, "START_INIT_USERSET," },
        { E_ACTION_KEY::START_FW_UPDATE, "START_FW_UPDATE," },
        { E_ACTION_KEY::START_FW_RECOVERY, "START_FW_RECOVERY," },
        { E_ACTION_KEY::START_FACTORY_RESET, "START_FACTORY_RESET," },
        { E_ACTION_KEY::DELETE_MAP, "DELETE_MAP," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CAppKey
{
private:
    E_ACTION_KEY actionKey;

public:
    CAppKey();
    ~CAppKey();
    void initAppKey();
    void setActionKey(short action);
    E_ACTION_KEY getActionKey();
};