/**
 * @file taskError.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

// C++ system header
#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"

enum class POWER_OFF_STATE
{
    NONE,
    START,
    RUN,
    COMPLETE,
};

static std::string enumToString(POWER_OFF_STATE value) {
    static const std::unordered_map<POWER_OFF_STATE, std::string> enumToStringMap = {
        { POWER_OFF_STATE::NONE, "NONE," },
        { POWER_OFF_STATE::START, "START," },
        { POWER_OFF_STATE::RUN, "RUN," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskPowerOff
{
private: /* not use! */
    CTaskPowerOff(const CTaskPowerOff& other) = default; // Copy constructor
    CTaskPowerOff& operator=(const CTaskPowerOff& other) = default; // Assignment operator

public:
    CTaskPowerOff();
    ~CTaskPowerOff();
    
    void taskStart();
    bool taskRun();
private:
    POWER_OFF_STATE state;
    double startTime;
private:
    void setState(POWER_OFF_STATE set);
    POWER_OFF_STATE startPowerOff();
    POWER_OFF_STATE runPowerOff();
    POWER_OFF_STATE completePowerOff();

    int processShutDownProcessor(bool isKillTS800);
};