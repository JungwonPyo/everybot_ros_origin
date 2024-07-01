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
#include "control/control.h"
#include "signaltracking.h"

enum class CHARGER_AVOID_STATE
{
    NONE,
    AVOID_START,
    SIGNAL_BALANCE,
    MOVE_CENTER,
    ACROSS_OPPOSITE,
    COMPLETE,
};

static std::string enumToString(CHARGER_AVOID_STATE value) {
    static const std::unordered_map<CHARGER_AVOID_STATE, std::string> enumToStringMap = {
        { CHARGER_AVOID_STATE::NONE, "NONE," },
        { CHARGER_AVOID_STATE::AVOID_START, "AVOID_START," },
        { CHARGER_AVOID_STATE::SIGNAL_BALANCE, "SIGNAL_BALANCE," },
        { CHARGER_AVOID_STATE::MOVE_CENTER, "MOVE_CENTER," },
        { CHARGER_AVOID_STATE::ACROSS_OPPOSITE, "ACROSS_OPPOSITE," },
        { CHARGER_AVOID_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskAvoidCharger
{
private: /* not use! */
    CTaskAvoidCharger(const CTaskAvoidCharger& other) = default; // Copy constructor
    CTaskAvoidCharger& operator=(const CTaskAvoidCharger& other) = default; // Assignment operator

public:
    CTaskAvoidCharger();
    ~CTaskAvoidCharger();
    
    void taskStart(E_WALLTRACK_DIR dir);
    bool taskRun(tPose robotPose);
    
private:

    E_WALLTRACK_DIR walltrackDir;
    CHARGER_AVOID_STATE state;
    void setAvoidChargerState(CHARGER_AVOID_STATE set);

    CHARGER_AVOID_STATE procCheckStartAvoid(E_WALLTRACK_DIR dir);
    CHARGER_AVOID_STATE procSignalBalanceTurn();
    CHARGER_AVOID_STATE procMoveSignalCenter();
    CHARGER_AVOID_STATE procAcrossOpposite(tPose robotPose);
    CHARGER_AVOID_STATE procAvoidComplete(tPose robotPose);
    
    bool avoidChagerComplete(tPose robotPose);
    bool isCheckAcross(E_WALLTRACK_DIR dir);
    tTwist getCalulateMoveCenter(E_WALLTRACK_DIR dir);
    tTwist getCalulateAcrossOppsite(E_WALLTRACK_DIR dir);
    void motionMovingCenter(E_WALLTRACK_DIR dir);
    void motionAcrossOppsite(E_WALLTRACK_DIR dir);
    double adjustSpeedToTarget(double currentSpeed, double targetSpeed, double acceleration);
    u8 getLongSignalMatchedByShortSignal(u8 shortSignal);
    u8 getAvoidCharegerSignalData(E_WALLTRACK_DIR dir);
    bool isStartChargerAvoid(u8 avoidSignal);

};