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
#include "taskPathPlan.h"


enum class MOVECLEANLINE_STATE
{
    NONE,
    FIND_PATH,
    WAITING_PATH,
    MOVE_TO_TARGET,
    AVOID_WALLTRACK,
    AVOID_BACK,
    AVOID_TURN,
    FINISH,
};

static std::string enumToString(MOVECLEANLINE_STATE value) {
    static const std::unordered_map<MOVECLEANLINE_STATE, std::string> enumToStringMap = {
        { MOVECLEANLINE_STATE::NONE, "NONE," },
        { MOVECLEANLINE_STATE::FIND_PATH, "FIND_PATH," },
        { MOVECLEANLINE_STATE::WAITING_PATH, "WAITING_PATH," },
        { MOVECLEANLINE_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },
        { MOVECLEANLINE_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { MOVECLEANLINE_STATE::AVOID_BACK, "AVOID_BACK," },
        { MOVECLEANLINE_STATE::AVOID_TURN, "AVOID_TURN," },
        { MOVECLEANLINE_STATE::FINISH, "FINISH," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskMoveCleanLine
{
private: /* not use! */
    CTaskMoveCleanLine(const CTaskMoveCleanLine& other) = default; // Copy constructor
    CTaskMoveCleanLine& operator=(const CTaskMoveCleanLine& other) = default; // Assignment operator
    
    //action
    MOVECLEANLINE_STATE findPath(tPose robotPose);
    MOVECLEANLINE_STATE waittingPath(tPose robotPose);
    MOVECLEANLINE_STATE moveTarget(tPose robotPose);
    MOVECLEANLINE_STATE avoidWalltrack(tPose robotPose);
    MOVECLEANLINE_STATE avoidBack(tPose robotPose);
    MOVECLEANLINE_STATE avoidTurn(tPose robotPose);
    MOVECLEANLINE_STATE finish(tPose robotPose);

    //func
    void setState(MOVECLEANLINE_STATE set);
    void computeWalltrackDir(tPose robotPose, tPoint target);

    bool updatePath(tPose robotPose);
    

public:
    CTaskMoveCleanLine();
    ~CTaskMoveCleanLine();

    bool taskRun(tPose robotPose);    
    void taskStart(tPoint goal);
    void setGoal(tPoint set);
    unsigned int getPathFailCount();
    

private:
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskWallTrackAvoid taskWallAvoid;
    CAvoiding avoiding;
    tPoint movingStartPoint;
    tPoint goal;
    tPoint startWallPoint;    
    MOVECLEANLINE_STATE state;
    MOVECLEANLINE_STATE preState;
    MOVECLEANLINE_STATE bakState;  // 이전상태로 대돌릴 버퍼
    E_WALLTRACK_DIR wallDir;
    tPoint avoidBackTarget;
    tPoint avoidBackStart;

    std::list<double> lineCoeff;

    double distTarget;      //비교를 위한 타겟 과의 거리
    double walltrackStartTime;
    unsigned int pathFailCnt;

    unsigned int waitPathCnt = 0;
    bool bSearchPath;
    std::list<tPoint> curPath;
    std::list<tPoint> newPath;

};