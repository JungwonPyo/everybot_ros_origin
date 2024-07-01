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
#include "taskMovePath.h"
#include "taskWalltrackAvoid.h"
#include "taskWalltrackAvoidAround.h"
#include "taskPathPlan.h"


enum class MOVEROOM_STATE
{
    NONE,
    FIND_PATH,
    WAITING_PATH,
    MOVE_TO_TARGET,
    AVOID_WALLTRACK,
    AVOID_WALLTRACK_AROUND, // 그냥 한바퀴 돌아보기
    AVOID_BACK,
    AVOID_TURN,
    FAIL,
    FINISH,
};

static std::string enumToString(MOVEROOM_STATE value) {
    static const std::unordered_map<MOVEROOM_STATE, std::string> enumToStringMap = {
        { MOVEROOM_STATE::NONE, "NONE," },
        { MOVEROOM_STATE::FIND_PATH, "FIND_PATH," },
        { MOVEROOM_STATE::WAITING_PATH, "WAITING_PATH," },
        { MOVEROOM_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },
        { MOVEROOM_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { MOVEROOM_STATE::AVOID_WALLTRACK_AROUND, "AVOID_WALLTRACK_AROUND," },
        { MOVEROOM_STATE::AVOID_BACK, "AVOID_BACK," },
        { MOVEROOM_STATE::AVOID_TURN, "AVOID_TURN," },
        { MOVEROOM_STATE::FAIL, "FAIL," },
        { MOVEROOM_STATE::FINISH, "FINISH," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskMoveRoom
{
private: /* not use! */
    CTaskMoveRoom(const CTaskMoveRoom& other) = default; // Copy constructor
    CTaskMoveRoom& operator=(const CTaskMoveRoom& other) = default; // Assignment operator
    
    //action
    MOVEROOM_STATE findPath(tPose robotPose);
    MOVEROOM_STATE waittingPath(tPose robotPose);
    MOVEROOM_STATE moveTarget(tPose robotPose);
    MOVEROOM_STATE avoidWalltrack(tPose robotPose);
    MOVEROOM_STATE avoidWalltrackAround(tPose robotPose);
    MOVEROOM_STATE avoidBack(tPose robotPose);
    MOVEROOM_STATE avoidTurn(tPose robotPose);
    MOVEROOM_STATE finish(tPose robotPose);

    //func
    void setState(MOVEROOM_STATE set);
    void computeWalltrackDir(tPose robotPose, tPoint target);

    bool updatePath(tPose robotPose);
    void drawCliff(tPose robotPose);
    

public:
    CTaskMoveRoom();
    ~CTaskMoveRoom();

    bool taskRun(tPose robotPose);    
    void taskStart(tPoint goal);
    void setGoal(tPoint set);
    unsigned int getPathFailCount();
    MOVEROOM_STATE getState();
    

private:
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskWallTrackAvoid taskWallAvoid;
    CTaskWallTrackAvoidAround taskWallAvoidAround;
    CAvoiding avoiding;
    tPoint movingStartPoint;
    tPoint goal;
    double goalMargin;
    tPoint startWallPoint;    
    MOVEROOM_STATE state;
    MOVEROOM_STATE preState;
    MOVEROOM_STATE bakState;  // 이전상태로 대돌릴 버퍼
    E_WALLTRACK_DIR wallDir;
    tPoint avoidBackTarget;
    tPoint avoidBackStart;

    std::list<double> lineCoeff;

    double distTarget;      //비교를 위한 타겟 과의 거리
    double walltrackStartTime;
    unsigned int pathFailCnt;

    unsigned int waitPathCnt;
    bool bSearchPath;
    std::list<tPoint> curPath;
    std::list<tPoint> newPath;

};