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

enum class ERROR_STATE
{
    NONE,
    START,
    RUN,
    POWER_OFF,
    COMPLETE,
};

static std::string enumToString(ERROR_STATE value) {
    static const std::unordered_map<ERROR_STATE, std::string> enumToStringMap = {
        { ERROR_STATE::NONE, "NONE," },
        { ERROR_STATE::START, "START," },
        { ERROR_STATE::RUN, "RUN," },
        { ERROR_STATE::POWER_OFF, "POWER_OFF," },
        { ERROR_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskError
{
private: /* not use! */
    CTaskError(const CTaskError& other) = default; // Copy constructor
    CTaskError& operator=(const CTaskError& other) = default; // Assignment operator

public:
    CTaskError();
    ~CTaskError();
    
    void taskStart();
    bool taskRun();
private:
    ERROR_STATE state;
    double startTime;
private:
    void setState(ERROR_STATE set);
    
};