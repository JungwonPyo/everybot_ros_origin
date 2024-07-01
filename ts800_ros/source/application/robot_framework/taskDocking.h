/**
 * @file docking.h
 * @author hhryu
 * @brief
 * @date 2023-08-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <iostream>
#include <unordered_map>
#include "ebtypedef.h"
#include "signaltracking.h"
#include "findCharger.h"
#include "utils.h"
#include "robotSlamPose.h"


#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"
#include "avoiding.h"
#include "taskPathPlan.h"
#include "taskMovePath.h"
#include "taskWalltrackAvoid.h"
#include "taskDockingExplorer.h"

/**
 * @brief   로봇이 충전 스테이션에 도킹하는 상태
 * @author  hhryu
 * @date    2022/10/25
 */
enum class DOCKING_STATE
{
    NONE,                   // VOID, 기본 도킹을 하고 있지 않는 상태.
    START_FIND_SIGNAL,
    RUN_FIND_SIGNAL,
    START_DOCKING_EXPLORER,
    RUN_DOCKING_EXPLORER,
    SET_SEARCHING_AREA,
    SET_TARGET_SEARCHING_POINT,
    GO_SEARCHING_POINT,
    CHECK_SIGNAL_TURN,
    MOVE_TO_CHARGER,
    AVOID_WALLTRACK,
    STOP_READY_MOVE,
    SIGNAL_TRACKING,
    COMPLETE,
};

static std::string enumToString(DOCKING_STATE value) {
    static const std::unordered_map<DOCKING_STATE, std::string> enumToStringMap = {
        { DOCKING_STATE::NONE, "NONE," },
        { DOCKING_STATE::START_FIND_SIGNAL, "START_FIND_SIGNAL," },
        { DOCKING_STATE::RUN_FIND_SIGNAL, "RUN_FIND_SIGNAL," },
        { DOCKING_STATE::START_DOCKING_EXPLORER, "START_DOCKING_EXPLORER," },
        { DOCKING_STATE::RUN_DOCKING_EXPLORER, "RUN_DOCKING_EXPLORER," },
        { DOCKING_STATE::SET_SEARCHING_AREA, "SET_SEARCHING_AREA," },
        { DOCKING_STATE::SET_TARGET_SEARCHING_POINT, "SET_TARGET_SEARCHING_POINT," },
        { DOCKING_STATE::GO_SEARCHING_POINT, "GO_SEARCHING_POINT," },
        { DOCKING_STATE::CHECK_SIGNAL_TURN, "CHECK_SIGNAL_TURN," },
        { DOCKING_STATE::MOVE_TO_CHARGER, "MOVE_TO_CHARGER," },
        { DOCKING_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { DOCKING_STATE::STOP_READY_MOVE, "STOP_READY_MOVE," },
        { DOCKING_STATE::SIGNAL_TRACKING, "SIGNAL_TRACKING," },
        { DOCKING_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class SIGNAL_TRACKING_STATE
{
    CHECK_TURN,
    SWIMMING,                   // 초기화
    BALANCE_TURN,
    MOVING_CENTER,           // 충전기로 이동한다...
    READY_TRY_DOCK_TURN,
    STOP_READY_TRY_DOCK,
    TILDOWN_TRY_DOCK,            // 찾는다 충전기!
    TILUP_TRY_DOCK,
};

static std::string enumToString(SIGNAL_TRACKING_STATE value) {
    static const std::unordered_map<SIGNAL_TRACKING_STATE, std::string> enumToStringMap = {
        { SIGNAL_TRACKING_STATE::CHECK_TURN, "CHECK_TURN," },
        { SIGNAL_TRACKING_STATE::SWIMMING, "SWIMMING," },
        { SIGNAL_TRACKING_STATE::BALANCE_TURN, "BALANCE_TURN," },
        { SIGNAL_TRACKING_STATE::MOVING_CENTER, "MOVING_CENTER," },
        { SIGNAL_TRACKING_STATE::READY_TRY_DOCK_TURN, "READY_TRY_DOCK_TURN," },
        { SIGNAL_TRACKING_STATE::TILDOWN_TRY_DOCK, "TILDOWN_TRY_DOCK," },
        { SIGNAL_TRACKING_STATE::TILUP_TRY_DOCK, "TILUP_TRY_DOCK," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

/**
 * @brief   로봇이 충전 스테이션에 도킹하는 상태
 * @author  hhryu
 * @date    2022/10/25
 */
enum class MOVE_CHARGER_STATE
{
    NONE,                   // VOID, 기본 도킹을 하고 있지 않는 상태.
    SEARCH_CHARGER_POINT,
    PATH_PLAN,
    START_MOVE,
    RUN_MOVING,
    AVOID_WALLTRACK,
    ARRIVED,
};

static std::string enumToString(MOVE_CHARGER_STATE value) {
    static const std::unordered_map<MOVE_CHARGER_STATE, std::string> enumToStringMap = {
        { MOVE_CHARGER_STATE::NONE, "NONE," },
        { MOVE_CHARGER_STATE::SEARCH_CHARGER_POINT, "SEARCH_CHARGER_POINT," },
        { MOVE_CHARGER_STATE::PATH_PLAN, "PATH_PLAN," },
        { MOVE_CHARGER_STATE::START_MOVE, "START_MOVE," },
        { MOVE_CHARGER_STATE::RUN_MOVING, "RUN_MOVING," },
        { MOVE_CHARGER_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { MOVE_CHARGER_STATE::ARRIVED, "ARRIVED," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class MOVING_CENTER_STATE
{
    SIGNAL_TRACK,
    START_AVOID_WALLTRACK,
    RUN_AVOID_WALLTRACK,
};

static std::string enumToString(MOVING_CENTER_STATE value) {
    static const std::unordered_map<MOVING_CENTER_STATE, std::string> enumToStringMap = {
        { MOVING_CENTER_STATE::SIGNAL_TRACK, "SIGNAL_TRACK," },
        { MOVING_CENTER_STATE::START_AVOID_WALLTRACK, "START_AVOID_WALLTRACK," },
        { MOVING_CENTER_STATE::RUN_AVOID_WALLTRACK, "RUN_AVOID_WALLTRACK," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskDocking
{

private: /* not use! */
CTaskDocking(const CTaskDocking& other) = default; // Copy constructor
CTaskDocking& operator=(const CTaskDocking& other) = default; // Assignment operator



private: // 클래스    
    CFindCharger *pFindCharger;

private: // 변수
    CAvoiding avoiding;
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskWallTrackAvoid taskWallAvoid;
    CTaskDockingExplorer taskDockingExplorer;
    DOCKING_STATE state;
    DOCKING_STATE tempState;
    MOVE_CHARGER_STATE movingState;
    SIGNAL_TRACKING_STATE trackingState;
    MOVING_CENTER_STATE movingCenterState;
    tDockingData dockingData_;
    E_CHECK_SIG_STEP mStep;
    tSignalArray signalArray;

    u16 sideReceiverData;
    double noSignalTime;
    tPoint noSignalPoint;

    bool bExplorer;
    
public:
    CTaskDocking();
    ~CTaskDocking();

public:

    s32 tryDockAngle; // test
    void taskStart();
    bool taskRun(tPose robotPose);

private:
    
    bool bDebugStart;
    bool isUpdateTargetPoint;
    double debugTime;
    std::list<tPoint> movingPath;
    std::list<tPoint> dockingTargetSearchingPoints;
    std::list<tPoint> dockingExplorerArea;

    E_WALLTRACK_DIR wallDir;
    int readyTryDockTunrnDir;
    int movingCenterDir;
    u16 shortSignalData;
    u16 signalData;
    int shortSignalCount;
    
    tPoint chargerPoint;
    tPoint startWallPoint;
    tPoint targetPoint;
    tPoint targetSearchingPoint;
    
    double targetRad;

    double walltrackStartTime;

    void setMovingState(MOVE_CHARGER_STATE set);
    void setSignalTrackState(SIGNAL_TRACKING_STATE set);
    void setDockingState(DOCKING_STATE set);
    
    MOVE_CHARGER_STATE searchChargerPoint(tPose robotPose);
    MOVE_CHARGER_STATE makePathPlan(tPose robotPose);
    MOVE_CHARGER_STATE startMove(tPose robotPose);
    MOVE_CHARGER_STATE runMoving(tPose robotPose);
    MOVE_CHARGER_STATE runAvoidWalltrack(tPose robotPose);
    MOVE_CHARGER_STATE ArrivedTarget(tPose robotPose);

    SIGNAL_TRACKING_STATE trackCheckSignalTurn(tPose robotPose);
    SIGNAL_TRACKING_STATE trackSwimming(tPose robotPose);
    SIGNAL_TRACKING_STATE trackReadyMoveCenterTurn(tPose robotPose);
    SIGNAL_TRACKING_STATE trackStopReadyMoveCenter(tPose robotPose);
    SIGNAL_TRACKING_STATE trackMovingCenter(tPose robotPose);
    SIGNAL_TRACKING_STATE trackTryDockReadyTurn(tPose robotPose);
    SIGNAL_TRACKING_STATE trackTilDownTryDock(tPose robotPose);
    SIGNAL_TRACKING_STATE trackTilUpTryDock(tPose robotPose);
    
    DOCKING_STATE monitorFindSignal(DOCKING_STATE curState,tPose robotPose);

    DOCKING_STATE procStartFindSiganl(tPose robotPose);
    DOCKING_STATE procRunFindSiganl(tPose robotPose);
    DOCKING_STATE procStartDockingExplorer(tPose robotPose);
    DOCKING_STATE procRunDockingExplorer(tPose robotPose);
    DOCKING_STATE procSetSearchingArea(tPose robotPose);
    DOCKING_STATE procSetTargetSearchingPoint(tPose robotPose);
    DOCKING_STATE procGoSearchingPoint(tPose robotPose);
    DOCKING_STATE procCheckSignalTurn(tPose robotPose);
    DOCKING_STATE procMoveToCharger(tPose robotPose);
    DOCKING_STATE procAvoidWalltrack(tPose robotPose);
    DOCKING_STATE procSignalTrack(tPose robotPose);
    DOCKING_STATE procStopReadyMove(tPose robotPose);
   
    
    u16 getSwimmingRefSignalData();
    u16 getRefSignalData();
    u16 getRefReceiver();
    int getCenteringReadyTurnDirection(u16 data);
    void computeWalltrackDir(tPose robotPose, tPoint target);

    void DebugPrintSignalData(u16 data);
    void DebugPrintSignalData2(u16 data);
};

