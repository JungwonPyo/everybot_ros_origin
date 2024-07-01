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


enum class LOCALMOTION_STATE
{
    NONE,
    START_MOVE,
    MOVE_TO_TARGET,

    STEP_1,
    STEP_1_WAIT,
    STEP_2,
    CHECK_END,              // end
    FINISH,
};

static std::string enumToString(LOCALMOTION_STATE value) {
    static const std::unordered_map<LOCALMOTION_STATE, std::string> enumToStringMap = {
        { LOCALMOTION_STATE::NONE, "NONE," },
        { LOCALMOTION_STATE::START_MOVE, "START_MOVE," },
        { LOCALMOTION_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },
        { LOCALMOTION_STATE::STEP_1, "STEP_1," },
        { LOCALMOTION_STATE::STEP_1_WAIT, "STEP_1_WAIT," }, //simplify map update
        { LOCALMOTION_STATE::STEP_2, "STEP_2," },
        { LOCALMOTION_STATE::CHECK_END, "CHECK_END," },
        { LOCALMOTION_STATE::FINISH, "FINISH," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskLocalMotion
{
private: /* not use! */
    CTaskLocalMotion(const CTaskLocalMotion& other) = default; // Copy constructor
    CTaskLocalMotion& operator=(const CTaskLocalMotion& other) = default; // Assignment operator
    
    //action
    LOCALMOTION_STATE step1(tPose robotPose);
    LOCALMOTION_STATE step1_wait(tPose robotPose);
    LOCALMOTION_STATE step2(tPose robotPose);

    LOCALMOTION_STATE goFwd(tPose robotPose);
    LOCALMOTION_STATE turnLeft(tPose robotPose);
    LOCALMOTION_STATE trunRight(tPose robotPose);
    LOCALMOTION_STATE checkEnd(tPose robotPose);
    bool finish(tPose robotPose);
    
    LOCALMOTION_STATE startMove(tPose robotPose);
    LOCALMOTION_STATE moveToTarget(tPose robotPose);

    //func
    void setState(LOCALMOTION_STATE set);
    void createPath(tPose robotPose);

public:
    CTaskLocalMotion();
    ~CTaskLocalMotion();

    bool taskRun(tPose robotPose);
    void taskStart(tPose robotPose);

private:
    CAvoiding avoiding;
    CTaskMovePath taskMovePath;
    std::list<tPoint> localMotionPath;
    tPoint localizeMotionTarget;
    double localizeMotionTargetRad;    
    LOCALMOTION_STATE state;
    LOCALMOTION_STATE preState;
    LOCALMOTION_STATE bakState;  // 이전상태로 대돌릴 버퍼
    u32 gridMapReadyTime;
    
};