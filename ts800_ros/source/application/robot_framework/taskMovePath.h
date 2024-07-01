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
#include "taskPathPlan.h"



enum class MOVEPATH_STATE
{
    NONE,
    START_MOVING,
    GET_TARGET,
    MOVE_TO_TARGET,
    FINISH,
};

static std::string enumToString(MOVEPATH_STATE value) {
    static const std::unordered_map<MOVEPATH_STATE, std::string> enumToStringMap = {
        { MOVEPATH_STATE::NONE, "NONE," },
        { MOVEPATH_STATE::START_MOVING, "START_MOVING," },
        { MOVEPATH_STATE::GET_TARGET, "GET_TARGET," },
        { MOVEPATH_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },                
        { MOVEPATH_STATE::FINISH, "FINISH," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskMovePath
{
private: /* not use! */
    CTaskMovePath(const CTaskMovePath& other) = default; // Copy constructor
    CTaskMovePath& operator=(const CTaskMovePath& other) = default; // Assignment operator

    CTaskPathPlan taskPathPlan;
    //action
    MOVEPATH_STATE startMoving(tPose robotPose);
    MOVEPATH_STATE getTarget(tPose robotPose);
    MOVEPATH_STATE moveTarget(tPose robotPose);    
    bool finish(tPose robotPose);

    void setState(MOVEPATH_STATE set);
    bool updatePath(tPose robotPose);
    

public:
    CTaskMovePath();
    ~CTaskMovePath();

    bool taskRun(tPose robotPose);    
    void taskStart(std::list<tPoint> &path, double _arriveMargin,tProfile Pf);
    tPoint getCurrentTarget();
    std::list<tPoint> getPath();

private:
    std::list<tPoint> currentPath;
    std::list<tPoint> newPath;
    tProfile profile;
    tPoint currentTarget;
    tPoint startPoint;
    MOVEPATH_STATE state;
    MOVEPATH_STATE preState;
    MOVEPATH_STATE bakState;  // 이전상태로 대돌릴 버퍼
    double arriveMargin;
    bool bUseMargin;
    bool bUseAngularPriorMotion;

    bool bSearchPath;
    double startTime;
    

    
};