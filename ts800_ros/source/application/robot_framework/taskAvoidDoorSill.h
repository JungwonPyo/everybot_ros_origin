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

enum class DOOR_SILL_STATE
{
    NONE,
    GO,
    MOVE,
    START_READY,
    RUN_READY,
    CLIMB,
    START_ACROSS,
    RUN_ACROSS,
    COMPLETE,
    FAIL,
};

static std::string enumToString(DOOR_SILL_STATE value) {
    static const std::unordered_map<DOOR_SILL_STATE, std::string> enumToStringMap = {
        { DOOR_SILL_STATE::NONE, "NONE," },
        { DOOR_SILL_STATE::GO, "GO," },
        { DOOR_SILL_STATE::MOVE, "MOVE," },
        { DOOR_SILL_STATE::START_READY, "START_READY," },
        { DOOR_SILL_STATE::RUN_READY, "RUN_READY," },
        { DOOR_SILL_STATE::CLIMB, "CLIMB," },
        { DOOR_SILL_STATE::START_ACROSS, "START_ACROSS," },
        { DOOR_SILL_STATE::RUN_ACROSS, "RUN_ACROSS," },
        { DOOR_SILL_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return std::string("Unknown : ");
    }
}

enum class DOORSILL_READY_STEP
{
    NONE,
    START_BALANCE,
    RUN_BALANCE,
    START_MOVE_BACK,
    RUN_MOVE_BACK,
    START_TILTUP,
    RUN_TILTUP,
    CHECK_TOFCALIB,
    COMPLETE,
};

static std::string enumToString(DOORSILL_READY_STEP value) {
    static const std::unordered_map<DOORSILL_READY_STEP, std::string> enumToStringMap = {
        { DOORSILL_READY_STEP::NONE, "NONE," },
        { DOORSILL_READY_STEP::START_BALANCE, "START_BALANCE," },
        { DOORSILL_READY_STEP::RUN_BALANCE, "RUN_BALANCE," },
        { DOORSILL_READY_STEP::START_MOVE_BACK, "START_MOVE_BACK," },
        { DOORSILL_READY_STEP::RUN_MOVE_BACK, "RUN_MOVE_BACK," },
        { DOORSILL_READY_STEP::START_TILTUP, "START_TILTUP," },
        { DOORSILL_READY_STEP::RUN_TILTUP, "RUN_TILTUP," },
        { DOORSILL_READY_STEP::CHECK_TOFCALIB, "CHECK_TOFCALIB," },
        { DOORSILL_READY_STEP::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return std::string("Unknown : ");
    }
}

class CTaskAvoidDoorSill
{
private: /* not use! */
    CTaskAvoidDoorSill(const CTaskAvoidDoorSill& other) = default; // Copy constructor
    CTaskAvoidDoorSill& operator=(const CTaskAvoidDoorSill& other) = default; // Assignment operator

    CAvoiding avoiding;
public:
    CTaskAvoidDoorSill();
    ~CTaskAvoidDoorSill();
    
    void setDoorSillState(DOOR_SILL_STATE set);
    void setDoorSillStep(DOORSILL_READY_STEP set);
    void taskStart(DOOR_SILL_STATE startState);
    void taskRun(tPose robotPose);
    

private:
    DOOR_SILL_STATE state;
    DOORSILL_READY_STEP step;

    tPoint originTarget;
    tPoint targetPoint;
    tPoint startPoint;
    tPose poseTemp;
    u8 knollTrapcount;
    u8 knollclearcount;
    double KnollHeading;
    u8 completeCnt;
    double startTime;
    bool bPowerOn;

    DOOR_SILL_STATE procGoForward(tPose robotPose);
    DOOR_SILL_STATE procRunMove(tPose robotPose);
    DOOR_SILL_STATE procStartReadyDoorSill(tPose robotPose);
    DOOR_SILL_STATE procRunReadyDoorSill(tPose robotPose);
    DOOR_SILL_STATE climbDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOOR_SILL_STATE startAcrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOOR_SILL_STATE runAcrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOOR_SILL_STATE completeAcrossDoorSill(tPose robotPose);
    DOOR_SILL_STATE failAcrossDoorSill(tPose robotPose);
    
    DOOR_SILL_STATE procDoorSillReady(tPose robotPose,RSU_OBSTACLE_DATA *pObstacle );
    DOORSILL_READY_STEP startBalanceDoorSill(tPose robotPose);
    DOORSILL_READY_STEP runBalanceDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOORSILL_READY_STEP startMoveBack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOORSILL_READY_STEP runMoveBack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOORSILL_READY_STEP startTilUp(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOORSILL_READY_STEP runTilUp(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    DOORSILL_READY_STEP checkTofAccumulate(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
};