/**
 * @file rsaDispatcher.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/serviceData.h"
#include "externData/externData.h"
#include "coreData/serviceData/keyState.h"
#include "coreData/serviceData.h"
#include "rsfMonitor.h"
#include "serviceManager.h"
#include "control/control.h"
#include "coreData/serviceData/obstacle.h"
#include "taskBoot.h"

#include "taskDemoRBTPlus.h"
#include "taskCleanRoom.h"
#include "taskReliability.h"
#include "taskClean.h"
#include "taskLocalMotion.h"
#include "taskDocking.h"
#include "taskUnDocking.h"
#include "taskAvoidDoorSill.h"
#include "taskWallClean.h"

enum class INIT_SENSOR
{
    CHECK_START,
    INIT_LOCAL,
    CHECK_INIT_LOCAL,
    CHECK_LOCAL,
    INIT_IMU,
    CHECK_INIT_IMU,
    CHECK_IMU,
    COMPLETE,
};


enum class MOTION_STATE
{
    NONE,
    START_MOVE_TO_POINT,
    MOVE_TO_BACK_POINT,
    START_ROTATE_TO_POINT,
    START_TO_STOP,
    RUN_MOVE_TO_POINT,
    START_AVOID,
    RUN_AVOID,
    WALLTRACK_AVOID,
    START_WALLTRACK,
    RUN_WALLTRACK,
};

static std::string enumToString(MOTION_STATE value) {
    static const std::unordered_map<MOTION_STATE, std::string> enumToStringMap = {
        { MOTION_STATE::NONE, "NONE," },
        { MOTION_STATE::START_MOVE_TO_POINT, "START_MOVE_TO_POINT," },
        { MOTION_STATE::MOVE_TO_BACK_POINT, "MOVE_TO_BACK_POINT," },
        { MOTION_STATE::START_ROTATE_TO_POINT, "START_ROTATE_TO_POINT," },
        { MOTION_STATE::START_TO_STOP, "START_TO_STOP," },
        { MOTION_STATE::RUN_MOVE_TO_POINT, "RUN_MOVE_TO_POINT," },
        { MOTION_STATE::START_AVOID, "START_AVOID," },
        { MOTION_STATE::RUN_AVOID, "RUN_AVOID," },
        { MOTION_STATE::WALLTRACK_AVOID, "WALLTRACK_AVOID," },
        { MOTION_STATE::START_WALLTRACK, "START_WALLTRACK," },
        { MOTION_STATE::RUN_WALLTRACK, "RUN_WALLTRACK," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CRsaDispatcher
{
private:
    CRsfMonitor*        pRsfMonitor;
    CServiceManager*    pServiceMng;
    CTaskBoot           taskBoot;

    CTaskDemoRBTPlus taskDemoRBTPlus;
    CTaskCleanRoom taskCleanRoom;
    CTaskReliability taskReliability;
    CTaskClean taskClean;
    CTaskLocalMotion taskLocalMotion;
    CTaskUnDocking taskUndock;
    CTaskWallClean taskWallClean;
    CTaskAvoidDoorSill taskDoorSill;
    CTaskDocking taskDocking;

private: /* not use! */
    CRsaDispatcher(const CRsaDispatcher &other);
    CRsaDispatcher &operator=(const CRsaDispatcher &other);

public:
    CRsaDispatcher(ros::NodeHandle _nh);
    ~CRsaDispatcher();
    
    bool init();
    bool loadService();
    void updateMonitor(void);
    void rsaDispatcherExecute();
    void callServiceHandler();
    void controlHandler();
    void errorChecker();
    void debugDispacher();

};