/**
 * @file powerState.h
 * @author jspark
 * @brief power 관련 데이터를 가공하여 저장하고 있음.
 * @date 2023-05-09
 */
#pragma once

#include "coreData/observer.h"
#include "interfaceStruct.h"
#include "ebtypedef.h"
#include "define.h"
#include "commonStruct.h"

enum E_TERMINAL_STATE
{
    NOT_WORK,
    NORMAL,
    DOCKED,
    DCJACK_IN,
};

static std::string enumToString(E_TERMINAL_STATE value) {
    static const std::unordered_map<E_TERMINAL_STATE, std::string> enumToStringMap = {
        { E_TERMINAL_STATE::NOT_WORK, "NOT_WORK," },
        { E_TERMINAL_STATE::NORMAL, "NORMAL," },
        { E_TERMINAL_STATE::DOCKED, "DOCKED," },
        { E_TERMINAL_STATE::DCJACK_IN, "DCJACK_IN," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CPowerState : public CObserver
{
private :
    double power_checkTime;
    bool extpower;
    E_TERMINAL_STATE state;
    E_TERMINAL_STATE preState;
    E_POWER_STATE powerState;

    bool bChangeState;

public :
    CPowerState();
    ~CPowerState();
    void update(CExternData* pExternData) override;
    E_TERMINAL_STATE getTerminalState();
    E_TERMINAL_STATE getTerminalPreState();
    E_POWER_STATE getPowerState();
    bool getExtPower();

    void updateTerminalState(tSysPower power);
};
