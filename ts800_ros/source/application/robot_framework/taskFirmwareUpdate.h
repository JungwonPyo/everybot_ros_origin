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

enum class OTA_STATE
{
    NONE,
    START,
    RUN,
    RECOVERY,
    COMPLETE,
    REBOOT,
};

static std::string enumToString(OTA_STATE value) {
    static const std::unordered_map<OTA_STATE, std::string> enumToStringMap = {
        { OTA_STATE::NONE, "NONE," },
        { OTA_STATE::START, "START," },
        { OTA_STATE::RUN, "RUN," },
        { OTA_STATE::RECOVERY, "RECOVERY," },
        { OTA_STATE::COMPLETE, "COMPLETE," },
        { OTA_STATE::REBOOT, "REBOOT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskFirmwareUpdate
{
private: /* not use! */
    CTaskFirmwareUpdate(const CTaskFirmwareUpdate& other) = default; // Copy constructor
    CTaskFirmwareUpdate& operator=(const CTaskFirmwareUpdate& other) = default; // Assignment operator

public:
    CTaskFirmwareUpdate();
    ~CTaskFirmwareUpdate();
    
    void taskStart();
    bool taskRun();
private:
    OTA_STATE state;
    double startTime;
    double debugTime;
private:
    void setState(OTA_STATE set);
    OTA_STATE startUpdate();
    OTA_STATE runUpdate(int percent, double runTime);
    OTA_STATE completeUpdate();
    OTA_STATE reBoot();
    int processShutDownProcessor(bool isKillTS800);
    
};