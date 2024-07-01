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
#include "taskWallClean.h"
#include "taskCleanRoom.h"
#include "taskMoveRoom.h"
#include "supplyWater.h"



enum class CLEANTASK_STATE
{
    NONE,
    START_MOVING_START_CLEAN_POINT,
    RUN_MOVING_START_CLEAN_POINT,
    START_WALL_CLEAN,
    RUN_WALL_CLEAN,
    START_CLEAN_ROOM,
    RUN_CLEAN_ROOM,
    START_MOVING_ROOM,
    RUN_MOVING_ROOM,
    PLAN_INIT,          // 계획 수립 전 영역 분할
    PLAN_MAKE,          // 전체 청소 계획을 수립
    PLAN_RE_MAKE,       // 방안에서 재계획. 
    PLAN_UPDATE_AREA,
    FINISH,
    STOP,
};

static std::string enumToString(CLEANTASK_STATE value) {
    static const std::unordered_map<CLEANTASK_STATE, std::string> enumToStringMap = {
        { CLEANTASK_STATE::NONE, "NONE," },
        { CLEANTASK_STATE::START_MOVING_START_CLEAN_POINT, "START_MOVING_START_CLEAN_POINT," },
        { CLEANTASK_STATE::RUN_MOVING_START_CLEAN_POINT, "RUN_MOVING_START_CLEAN_POINT," },
        { CLEANTASK_STATE::START_WALL_CLEAN, "START_WALL_CLEAN," },
        { CLEANTASK_STATE::RUN_WALL_CLEAN, "RUN_WALL_CLEAN," },
        { CLEANTASK_STATE::START_CLEAN_ROOM, "START_CLEAN_ROOM," },
        { CLEANTASK_STATE::RUN_CLEAN_ROOM, "RUN_CLEAN_ROOM," },
        { CLEANTASK_STATE::START_MOVING_ROOM, "START_MOVING_ROOM," },
        { CLEANTASK_STATE::RUN_MOVING_ROOM, "RUN_MOVING_ROOM," },
        { CLEANTASK_STATE::PLAN_INIT, "PLAN_INIT," },
        { CLEANTASK_STATE::PLAN_MAKE, "PLAN_MAKE," },
        { CLEANTASK_STATE::PLAN_RE_MAKE, "PLAN_RE_MAKE," },
        { CLEANTASK_STATE::PLAN_UPDATE_AREA, "PLAN_UPDATE_AREA," },
        { CLEANTASK_STATE::FINISH, "FINISH," },
        { CLEANTASK_STATE::STOP, "STOP," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}




class CTaskClean
{
private: /* not use! */
    CTaskClean(const CTaskClean& other) = default; // Copy constructor
    CTaskClean& operator=(const CTaskClean& other) = default; // Assignment operator

    CTaskCleanRoom cleanRoom;
    CTaskWallClean cleanWall;
    CTaskMoveRoom moveRoom;
    CSupplyWater waterSupply;

    std::list<tPoint> currentAreaList;  // 현재 선택된 에어리어 
    std::list<tPoint> safeCleanZone;    // 선택된 에어리어 축소 버전 
    tPoint currentAreaCentroid;

    double awayWallAngle;
    tPoint cleanStartPoint;
    double cleanStartTime;

    unsigned int planRemakeTry; //재플랜 여러번해도 못하면 포기하게

    CLEANTASK_STATE planInit(tPose robotPose);
    CLEANTASK_STATE planMake(tPose robotPose);
    CLEANTASK_STATE planReMake(tPose robotPose);
    CLEANTASK_STATE planUpdateArea();

    CLEANTASK_STATE startMovingCleanStartPoint(tPose robotPose);
    CLEANTASK_STATE runMovingCleanStartPoint(tPose robotPose);
    CLEANTASK_STATE startCleanWall(tPose robotPose);
    CLEANTASK_STATE runCleanWall(tPose robotPose);
    CLEANTASK_STATE startCleanRoom(tPose robotPose);
    CLEANTASK_STATE runCleanRoom(tPose robotPose);
    CLEANTASK_STATE startMovingRoom(tPose robotPose);
    CLEANTASK_STATE runMovingRoom(tPose robotPose);

    void rePortCleanAreaInfo();
    void debugAwsForbiddenArea();

public:
    CTaskClean();
    ~CTaskClean();

    void taskStart(tPose robotPose);
    bool taskRun(tPose robotPose);

    CLEANTASK_STATE cleanLineState;
    CLEANTASK_STATE preCleanLineState;
    CLEANTASK_STATE bakCleanLineState;  // 이전상태로 대돌릴 버퍼

private:
    //func
    void setState(CLEANTASK_STATE set);
    void updateCleaned(tPose robotPose);

    tPoint curCleanPt;
    tPoint preCleanPt;
    bool bSkipWallClean;
    
};