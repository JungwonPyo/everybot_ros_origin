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
#include "motionPlanner/motionPlanner.h"

enum class PATH_PALN_STATE
{
    NONE,                       // 초기 세팅
    START,
    RUN,
    COMPLETE,
    FAIL,
};
static std::string enumToString(PATH_PALN_STATE value) {
    static const std::unordered_map<PATH_PALN_STATE, std::string> enumToStringMap = {
        { PATH_PALN_STATE::NONE, "NONE," },
        { PATH_PALN_STATE::START, "START," },
        { PATH_PALN_STATE::RUN, "RUN," },
        { PATH_PALN_STATE::COMPLETE, "COMPLETE," },
        { PATH_PALN_STATE::FAIL, "FAIL," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskPathPlan
{
private: /* not use! */
    CTaskPathPlan(const CTaskPathPlan& other) = default; // Copy constructor
    CTaskPathPlan& operator=(const CTaskPathPlan& other) = default; // Assignment operator

public:
    CTaskPathPlan();
    ~CTaskPathPlan();
    
    // proc
    std::list<tPoint> taskRun(tPose robotPose);
    void taskStart(tPoint _goal);
    PATH_PALN_STATE getPathPlanState();
    
private:
    PATH_PALN_STATE  state;   

    double pathplanStartTime;
    tPoint goal;
    std::list<tPoint> pathPlan;

    void setPathPlanState(PATH_PALN_STATE set);

    PATH_PALN_STATE startPathSearching(tPose robotPose);
    PATH_PALN_STATE runPathSearching(tPose robotPose);
    PATH_PALN_STATE completePathSearching(tPose robotPose);
    PATH_PALN_STATE failPathSearching(tPose robotPose);
    
    bool procPathSearch(tPose robotPose);
};