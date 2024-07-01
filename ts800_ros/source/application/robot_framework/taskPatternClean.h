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
#include "walltracking.h"

enum class PATTERN_CLEAN_STATE
{
    NONE,
    FIND_CLEAN_START_POINT,
    GOING_TO_CLEAN_START_POINT,

    START_AVOID_WALLTRACK,
    GOING_AVOID_WALLTRACK,

    START_PATTERN_CLEAN,
    GO_TO_MAIN_LINE,
    GO_TO_SIDE_LINE,
    RUN_MAINLINE_WALLTRCK_AVOID,
    RUN_SIDELINE_WALLTRCK_AVOID,    
};

static std::string enumToString(PATTERN_CLEAN_STATE value) {
    static const std::unordered_map<PATTERN_CLEAN_STATE, std::string> enumToStringMap = {
        { PATTERN_CLEAN_STATE::NONE, "NONE," },
        { PATTERN_CLEAN_STATE::START_AVOID_WALLTRACK, "START_AVOID_WALLTRACK," },
        { PATTERN_CLEAN_STATE::GOING_AVOID_WALLTRACK, "GOING_AVOID_WALLTRACK," },
        { PATTERN_CLEAN_STATE::FIND_CLEAN_START_POINT, "FIND_CLEAN_START_POINT," },
        { PATTERN_CLEAN_STATE::GOING_TO_CLEAN_START_POINT, "GOING_TO_CLEAN_START_POINT," },
        { PATTERN_CLEAN_STATE::GO_TO_MAIN_LINE, "GO_TO_MAIN_LINE," },
        { PATTERN_CLEAN_STATE::GO_TO_SIDE_LINE, "GO_TO_SIDE_LINE," },
        { PATTERN_CLEAN_STATE::RUN_MAINLINE_WALLTRCK_AVOID, "RUN_MAINLINE_WALLTRCK_AVOID," },
        { PATTERN_CLEAN_STATE::RUN_SIDELINE_WALLTRCK_AVOID, "RUN_SIDELINE_WALLTRCK_AVOID," },
        { PATTERN_CLEAN_STATE::START_PATTERN_CLEAN, "START_PATTERN_CLEAN," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskPatternClean
{
private: /* not use! */
    CTaskPatternClean(const CTaskPatternClean& other) = default; // Copy constructor
    CTaskPatternClean& operator=(const CTaskPatternClean& other) = default; // Assignment operator

public:
    CTaskPatternClean();
    ~CTaskPatternClean();
    CAvoiding avoiding;
    
    void taskStart(PATTERN_CLEAN_STATE startState);
    bool taskRun(tPose robotPose);    

    //action
    void findCleanStartPoint(tPose robotPose);
    void goingToCleanStartPoint(tPose robotPose);
    void startAvoidWallTrack();
    void runAvoidWalltrack(tPose robotPose);
    void startPatternClean(tPose robotPose);
    void goToMainLline(tPose robotPose);
    void goToSideLine(tPose robotPose);
    void startAvoidMainLineWalltrack();
    void runAvoidMainLineWalltrack(tPose robotPose);
    void startAvoidSideLineWalltrack();
    void runAvoidSideLineWalltrack(tPose robotPose);
    

    //func
    void setPatternCleanState(PATTERN_CLEAN_STATE set);
    void setLineDir(LINE_DIR set);

    PATTERN_CLEAN_STATE state;
    PATTERN_CLEAN_STATE preState;
    PATTERN_CLEAN_STATE bakState;  // 이전상태로 대돌릴 버퍼
    E_WALLTRACK_DIR wallDir;
    tPose startPose;
    
    tPoint localizeMotionTarget;
    double localizeMotionTargetRad;
    bool bAvoiding;
    tPoint targetPoint;
    double walltrackStartTime;
    tPoint startWallPoint;

    bool reverse = false;
    LINE_DIR lineDir;
    double maxRange;
    tPoint startPatternPoint;
    tPoint targetPatternPoint;
    tPoint NextstartPatternPoint;
    tPoint NexttargetPatternPoint;
    tPoint getLineTargetPoint(tPoint current);
    tPoint getCrossLineTargetPoint(tPoint current, double interval);
    tPoint getNextStartPoint(tPoint current,double interval);
    tPoint getNextTargetPoint(tPoint current);
    bool isCleanComplete();

    bool bCleanComplete;
private:
    tPoint nearWall;
    tPoint cleanStartPoint;
    std::list<double> lineCoeff;
    std::list<double> lineCoeff2;

    
};