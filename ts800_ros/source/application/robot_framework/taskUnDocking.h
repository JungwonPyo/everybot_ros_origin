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
#include "avoiding.h"

enum class UNDOCKING_STATE
{
    NONE,
    CHECK_MODE,
    START_UNDOCKING,
    CHECK_TILT,
    ESCAPE_DOCKING_STATION,
    BACK_TO_START_POINT,
    TURN,
    COMPLETE,
};

static std::string enumToString(UNDOCKING_STATE value) {
    static const std::unordered_map<UNDOCKING_STATE, std::string> enumToStringMap = {
        { UNDOCKING_STATE::NONE, "NONE," },
        { UNDOCKING_STATE::CHECK_MODE, "CHECK_MODE," },
        { UNDOCKING_STATE::START_UNDOCKING, "START_UNDOCKING," },
        { UNDOCKING_STATE::CHECK_TILT, "CHECK_TILT," },
        { UNDOCKING_STATE::ESCAPE_DOCKING_STATION, "ESCAPE_DOCKING_STATION," },
        { UNDOCKING_STATE::BACK_TO_START_POINT, "BACK_TO_START_POINT," },
        { UNDOCKING_STATE::TURN, "TURN," },
        { UNDOCKING_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskUnDocking
{
private: /* not use! */
    CTaskUnDocking(const CTaskUnDocking& other) = default; // Copy constructor
    CTaskUnDocking& operator=(const CTaskUnDocking& other) = default; // Assignment operator

public:
    CTaskUnDocking();
    ~CTaskUnDocking();
    
    void setUnDockingState(UNDOCKING_STATE set);
    void taskStart();
    bool taskRun(tPose robotPose);
    

private:
    UNDOCKING_STATE unDockingState;
    tProfile profile;
    double imuInitTime;
    double modeInitTime;
    tPoint targetPoint;
    tPoint startPoint;
    tPose startPose;
    double targetAngle;
    
    UNDOCKING_STATE procCheckMode(tPose robotPose);
    UNDOCKING_STATE procStartUnDocking(tPose robotPose, tProfile pf);
    UNDOCKING_STATE procCheckTilt(tPose robotPose, tProfile pf);
    UNDOCKING_STATE procEscapeDockingStation(tPose robotPose, tProfile pf);
    UNDOCKING_STATE procBackToStartPoint(tPose robotPose, tProfile pf);
    UNDOCKING_STATE procTurnning(tPose robotPose, tProfile pf);
    UNDOCKING_STATE procComplete(tPose robotPose, tProfile pf);

    void startMotionEscape(tPose robotPose, tProfile pf);
    void startMotionTurn(tPose robotPose, tProfile pf);
    void startMotionStop(tPose robotPose, tProfile pf, bool emergency); 
};