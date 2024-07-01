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

// C++ system header
#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"
#include "avoiding.h"
#include "walltracking.h"
#include "taskCharging.h"
#include "taskUnDocking.h"


enum class RELIABILITY_STATE
{
    NONE,
    MODE_CHECK,
    RUN_INFINITY,
    RUN_NOISE_EVALUATION,
    RUN_CHARGE,
    RUN_UNDOCKING,
};

static std::string enumToString(RELIABILITY_STATE value) {
    static const std::unordered_map<RELIABILITY_STATE, std::string> enumToStringMap = {
        { RELIABILITY_STATE::NONE, "NONE," },
        { RELIABILITY_STATE::MODE_CHECK, "MODE_CHECK," },
        { RELIABILITY_STATE::RUN_INFINITY, "RUN_INFINITY," },
        { RELIABILITY_STATE::RUN_NOISE_EVALUATION, "RUN_NOISE_EVALUATION," },
        { RELIABILITY_STATE::RUN_CHARGE, "RUN_CHARGE," },
        { RELIABILITY_STATE::RUN_UNDOCKING, "RUN_UNDOCKING," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class INFINITY_RANDOM_CLEAN_STATE
{
    NONE,
    CHECK_MODE,
    INIT_SENSOR,
    CHECK_SENSOR,
    START_GOING,
    RUN_GOING,
    START_BACK_MOVING,
    RUN_BACK_MOVING,
    START_TURNNING,
    RUN_TURNNING,
};

static std::string enumToString(INFINITY_RANDOM_CLEAN_STATE value) {
    static const std::unordered_map<INFINITY_RANDOM_CLEAN_STATE, std::string> enumToStringMap = {
        { INFINITY_RANDOM_CLEAN_STATE::NONE, "NONE," },
        { INFINITY_RANDOM_CLEAN_STATE::CHECK_MODE, "CHECK_MODE," },
        { INFINITY_RANDOM_CLEAN_STATE::INIT_SENSOR, "INIT_SENSOR," },
        { INFINITY_RANDOM_CLEAN_STATE::CHECK_SENSOR, "CHECK_SENSOR," },
        { INFINITY_RANDOM_CLEAN_STATE::START_GOING, "START_GOING," },
        { INFINITY_RANDOM_CLEAN_STATE::RUN_GOING, "RUN_GOING," },
        { INFINITY_RANDOM_CLEAN_STATE::START_BACK_MOVING, "START_BACK_MOVING," },
        { INFINITY_RANDOM_CLEAN_STATE::RUN_BACK_MOVING, "RUN_BACK_MOVING," },
        { INFINITY_RANDOM_CLEAN_STATE::START_TURNNING, "START_TURNNING," },
        { INFINITY_RANDOM_CLEAN_STATE::RUN_TURNNING, "RUN_TURNNING," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskReliability
{
private: /* not use! */
    CTaskReliability(const CTaskReliability& other) = default; // Copy constructor
    CTaskReliability& operator=(const CTaskReliability& other) = default; // Assignment operator

    CTaskCharging taskCharging;
    CTaskUnDocking taskUnDocking;
    CAvoiding avoiding;
public:
    CTaskReliability();
    ~CTaskReliability();
    
    bool taskRun();
    void taskStart();

    INFINITY_RANDOM_CLEAN_STATE randomCleanState;
    RELIABILITY_STATE reliabilityState;
    
private:

    int r = 262, l = 262, b = 45;
    bool bStopCommand;
    bool isWheelActive();
    void runProcNone(tPose robotPose);

    double imuInitTime;
    double modeInitTime;

    tPoint targetPoint;
    tPoint startPoint;
    double targetAngle;
    
    bool keyChecker(tPose robotPose);

    void setReliabilityState(RELIABILITY_STATE set);
    void setRandomCleanState(INFINITY_RANDOM_CLEAN_STATE set);

    INFINITY_RANDOM_CLEAN_STATE checkMode();
    INFINITY_RANDOM_CLEAN_STATE initSensor(tPose robotPose);
    INFINITY_RANDOM_CLEAN_STATE checkSensor(tPose robotPose);
    INFINITY_RANDOM_CLEAN_STATE startRandomGo(tPose robotPose, tProfile pf);
    INFINITY_RANDOM_CLEAN_STATE runRandomGo(tPose robotPose,tProfile pf);
    INFINITY_RANDOM_CLEAN_STATE startRandomBack(tPose robotPose, tProfile pf);
    INFINITY_RANDOM_CLEAN_STATE runRandomBack(tPose robotPose);
    INFINITY_RANDOM_CLEAN_STATE startRandomTurn(tPose robotPose, tProfile pf);
    INFINITY_RANDOM_CLEAN_STATE runRandomTurn(tPose robotPose);

    RELIABILITY_STATE runRandomCleanInfinity(tPose robotPose);
    RELIABILITY_STATE runUnDocking(tPose robotPose);
    void runCharging();
};