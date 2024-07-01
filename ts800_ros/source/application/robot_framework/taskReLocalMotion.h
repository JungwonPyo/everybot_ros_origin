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

#include <iostream>
#include <unordered_map>
#include "ebtypedef.h"
#include "commonStruct.h"
#include "service.h"

enum class RE_LOCALMOTION_STATE
{
    NONE,
    //extra step
    START_MOVE,
    MOVE_TO_TARGET,
    STEP_EXIT_SLAM,
    STEP_RUN_SLAM,

    //relocal step
    STEP_WAIT,
    STEP_1,
    STEP_2,
    STEP_3,
    CHECK_END,              // end
    FINISH,
};

static std::string enumToString(RE_LOCALMOTION_STATE value) {
    static const std::unordered_map<RE_LOCALMOTION_STATE, std::string> enumToStringMap = {
        { RE_LOCALMOTION_STATE::NONE, "NONE," },
        { RE_LOCALMOTION_STATE::STEP_EXIT_SLAM, "STEP_EXIT_SLAM," },
        { RE_LOCALMOTION_STATE::STEP_RUN_SLAM, "STEP_RUN_SLAM," },
        { RE_LOCALMOTION_STATE::START_MOVE, "START_MOVE," },
        { RE_LOCALMOTION_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },
        { RE_LOCALMOTION_STATE::STEP_WAIT, "STEP_WAIT," }, //시작 시 모든 필터 풀고.. 중지 시점에서 gridmap 업데이트 목적
        { RE_LOCALMOTION_STATE::STEP_1, "STEP_1," },
        { RE_LOCALMOTION_STATE::STEP_2, "STEP_2," },
        { RE_LOCALMOTION_STATE::STEP_3, "STEP_3," },
        { RE_LOCALMOTION_STATE::CHECK_END, "CHECK_END," },
        { RE_LOCALMOTION_STATE::FINISH, "FINISH," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskReLocalMotion
{
private: /* not use! */
    CTaskReLocalMotion(const CTaskReLocalMotion& other) = default; // Copy constructor
    CTaskReLocalMotion& operator=(const CTaskReLocalMotion& other) = default; // Assignment operator
    
    //action
    RE_LOCALMOTION_STATE stepRunSlam(tPose robotPose);

    RE_LOCALMOTION_STATE step_wait(tPose slamPose, tPose systemPose, tPose robotPose);
    RE_LOCALMOTION_STATE step1(tPose slamPose, tPose systemPose, tPose robotPose);
    RE_LOCALMOTION_STATE step2(tPose slamPose, tPose systemPose, tPose robotPose);
    RE_LOCALMOTION_STATE step3(tPose slamPose, tPose systemPose, tPose robotPose);
    RE_LOCALMOTION_STATE checkEnd(tPose slamPose, tPose systemPose, tPose robotPose);
    RE_LOCALMOTION_STATE finish(tPose slamPose, tPose systemPose, tPose robotPose);
    
    RE_LOCALMOTION_STATE startMove(tPose robotPose);
    RE_LOCALMOTION_STATE moveToTarget(tPose robotPose);

    //func
    void setState(RE_LOCALMOTION_STATE set);
    void createPath(tPose robotPose);

public:
    CTaskReLocalMotion();
    ~CTaskReLocalMotion();

    bool taskRun(tPose slamPose, tPose systemPose, tPose robotPose);
    void taskStart(tPose slamPose, tPose systemPose, tPose robotPose);
    void taskExitSlam(tPose robotPose);
    void taskRunSlam(tPose robotPose);

private:
    CAvoiding avoiding;
    CTaskMovePath taskMovePath;
    std::list<tPoint> localMotionPath;
    tPoint localizeMotionTarget;
    double localizeMotionTargetRad;    
    RE_LOCALMOTION_STATE state;
    RE_LOCALMOTION_STATE preState;
    RE_LOCALMOTION_STATE bakState;  // 이전상태로 대돌릴 버퍼
    u32 gridMapReadyTime;
    tPose startRobotPose; //relocal 발생 시점에 로봇 시작 위치, 필요할까>
    tPose startSystemPose; //relocal 발생 시점에 로봇 시작 위치, 필요할까>
    tPose startSlamPose; //relocal 발생 시점에 로봇 시작 위치, 필요할까>
};