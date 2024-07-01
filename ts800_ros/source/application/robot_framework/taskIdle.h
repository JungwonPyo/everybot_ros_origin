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


enum class IDLE_STATE
{
    NONE,
    START_MOVE_TO_POINT,
    RUN_MOVE_TO_POINT,
    START_AVOID,
    RUN_AVOID,
    WALLTRACK_AVOID,
    START_WALLTRACK,
    RUN_WALLTRACK,
};

static std::string enumToString(IDLE_STATE value) {
    static const std::unordered_map<IDLE_STATE, std::string> enumToStringMap = {
        { IDLE_STATE::NONE, "NONE," },
        { IDLE_STATE::START_MOVE_TO_POINT, "START_MOVE_TO_POINT," },
        { IDLE_STATE::RUN_MOVE_TO_POINT, "RUN_MOVE_TO_POINT," },
        { IDLE_STATE::START_AVOID, "START_AVOID," },
        { IDLE_STATE::RUN_AVOID, "RUN_AVOID," },
        { IDLE_STATE::WALLTRACK_AVOID, "WALLTRACK_AVOID," },
        { IDLE_STATE::START_WALLTRACK, "START_WALLTRACK," },
        { IDLE_STATE::RUN_WALLTRACK, "RUN_WALLTRACK," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskIdle
{
private: /* not use! */
    CTaskIdle(const CTaskIdle& other) = default; // Copy constructor
    CTaskIdle& operator=(const CTaskIdle& other) = default; // Assignment operator

public:
    CTaskIdle();
    ~CTaskIdle();
    CAvoiding avoiding;
    bool runRvizGoal;
    int runRvizGoalStep;
    tPose rvizGoal;
    
    
    // proc
    bool taskIdleRun(tPose robotPose);
    void setidleState(IDLE_STATE set);

    IDLE_STATE idleState;
    E_WALLTRACK_DIR wallDir;
    tPose startPose;
    
    tPoint localizeMotionTarget;
    double localizeMotionTargetRad;
    bool bAvoiding;
    tPoint targetPoint;
    double walltrackStartTime;
private:
    void goToRvizGoal();
    
};