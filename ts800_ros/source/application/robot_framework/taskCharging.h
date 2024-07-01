/**
 * @file explorer.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "commonStruct.h"
#include "taskDryMop.h"


enum class CHARGING_STATE
{
    NONE,
    START_DRY_MOP,
    RUN_DRY_MOP,
    RUN_CHARGING,
    CHARGING_COMPLETE,
};

static std::string enumToString(CHARGING_STATE value) {
    static const std::unordered_map<CHARGING_STATE, std::string> enumToStringMap = {
        { CHARGING_STATE::NONE, "NONE," },
        { CHARGING_STATE::START_DRY_MOP, "START_DRY_MOP," },
        { CHARGING_STATE::RUN_DRY_MOP, "RUN_DRY_MOP," },
        { CHARGING_STATE::RUN_CHARGING, "RUN_CHARGING," },
        { CHARGING_STATE::CHARGING_COMPLETE, "CHARGING_COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class RUN_CHARGING_STATE
{
    NONE,
    MODE_CHECK,
    START,
    RUN,
    COMPLETE,
};

static std::string enumToString(RUN_CHARGING_STATE value) {
    static const std::unordered_map<RUN_CHARGING_STATE, std::string> enumToStringMap = {
        { RUN_CHARGING_STATE::NONE, "NONE," },
        { RUN_CHARGING_STATE::MODE_CHECK, "MODE_CHECK," },
        { RUN_CHARGING_STATE::START, "START," },
        { RUN_CHARGING_STATE::RUN, "RUN," },
        { RUN_CHARGING_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskCharging
{
private: /* not use! */
    CTaskCharging(const CTaskCharging& other) = default; // Copy constructor
    CTaskCharging& operator=(const CTaskCharging& other) = default; // Assignment operator
public:
    CTaskCharging();
    ~CTaskCharging();
    void taskStart();
    bool taskRun();
    void dryMopStartStop();
    void dryMopOptionUpdate();
    bool isDryMopActive();
    

private:
    CTaskDryMop taskDryMop;
    double startTime;
    double debugStartTime;
    double modeInitTime;
    double totalChargingTime;

    E_BATTERY_STATE startBattState;

    u8 startPercentage;
    double startVolt;

    u8 tempPercentage;

    void setChargingState(CHARGING_STATE set);
    
    CHARGING_STATE  state;
    RUN_CHARGING_STATE runChargeState;

    void setRunChargingState(RUN_CHARGING_STATE set);

    CHARGING_STATE startDryMop();
    CHARGING_STATE runDryMop(E_BATTERY_STATE battState);
    CHARGING_STATE runCharging(E_BATTERY_STATE battState,u8 percent,double volt,double runTime);
    CHARGING_STATE completeCharging(E_BATTERY_STATE battState, u8 percentage, double volt,double runTime);

    RUN_CHARGING_STATE modeCheck();
    RUN_CHARGING_STATE chargingStart(E_BATTERY_STATE battState, u8 percentage, double volt);
    RUN_CHARGING_STATE chargingRun(double runTime,E_BATTERY_STATE battState, u8 percentage, double volt);
    

    void debugPrint(double runTime, E_BATTERY_STATE battState, u8 percentage, double volt, bool extPower);
};