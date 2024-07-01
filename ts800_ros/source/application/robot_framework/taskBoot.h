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

enum class BOOT_STATE
{
    NONE,
    START,
    RUN,
    COMPLETE,
};

static std::string enumToString(BOOT_STATE value) {
    static const std::unordered_map<BOOT_STATE, std::string> enumToStringMap = {
        { BOOT_STATE::NONE, "NONE," },
        { BOOT_STATE::START, "START," },
        { BOOT_STATE::RUN, "RUN," },
        { BOOT_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskBoot
{
private: /* not use! */
    CTaskBoot(const CTaskBoot& other) = default; // Copy constructor
    CTaskBoot& operator=(const CTaskBoot& other) = default; // Assignment operator

public:
    CTaskBoot();
    ~CTaskBoot();
    
    void taskStart();
    bool taskRun();
    BOOT_STATE getState();
private:
    BOOT_STATE state;
    double startTime;
private:
    void setState(BOOT_STATE set);
    
    BOOT_STATE bootStart();
    BOOT_STATE bootRun();
    BOOT_STATE bootComplete();

};