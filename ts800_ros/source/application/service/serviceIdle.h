/**
 * @file serviceIdle.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>


#include "service.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "location.h"
#include "supplyWater.h"
#include "robotSlamPose.h"
#include "avoiding.h"
#include "serviceReady.h"
#include "kidnap.h"
#include "taskExplorer.h"

#include "taskIdle.h"
#include "taskAvoidDoorSill.h"
#include "taskClean.h"
#include "taskReLocalMotion.h"

#include "taskIdle.h"
#include "taskAvoidDoorSill.h"
#include "taskClean.h"
#include "taskReLocalMotion.h"

enum class MANUAL_MOVING_AVOID_STEP
{
    NONE,
    BACK,
    TRUN,
    WALLTRACK,
    RETURN,
};

class CServiceIdle : public service , public CAvoiding
{
private:

    CTaskIdle taskIdle;
    CTaskAvoidDoorSill taskDoorSill;
    CTaskClean taskClean;
    CTaskCleanRoom taskCleanRoom;
    CServiceReady* pServiceReady;
    CKidnap* pKidnap;
    CSupplyWater   *pSupplyWater;  
    CLocation  *pLocation;
    CTaskReLocalMotion taskReLocalMotion;

    //로봇의 PASUE STATE 자기 위치 좌표
    CRobotSlamPose* pRobotSlamPose;

    u32 debugtick;
    double startTime;

    tPose startPose;
    tPoint targetPoint;
    bool manualMoving;
    MANUAL_MOVING_AVOID_STEP avoidStep;

    bool optionReset;
    bool bootingAwsReport;
    
    //for_test
    bool bPreLift;
    bool bStart;
public:
    CServiceIdle(CServiceReady* _pServiceReady, CSupplyWater* _pSupplyWater, CLocation* _pLocation, CRobotSlamPose* _pRobotSlamPose,  CKidnap* _pKidnap);
    ~CServiceIdle();    

private: 
    void serviceInitial() override;
    
    E_SERVICE_STATUS_STEP startupStepReady() override;
    E_SERVICE_STATUS_STEP startupStepExecuting() override;
    E_SERVICE_STATUS_STEP startupStepWaiting() override;
    E_SERVICE_STATUS_STEP startupStepTerminaition() override;

    E_SERVICE_STATUS_STEP runningStepReady() override;
    E_SERVICE_STATUS_STEP runningStepExecuting() override;
    E_SERVICE_STATUS_STEP runningStepWaiting() override;
    E_SERVICE_STATUS_STEP runningStepTerminaition() override;

    E_SERVICE_STATUS_STEP completedStepReady() override;
    E_SERVICE_STATUS_STEP completedStepExecuting() override;
    E_SERVICE_STATUS_STEP completedStepWaiting() override;
    E_SERVICE_STATUS_STEP completedStepTerminaition() override;

    E_SERVICE_STATUS_STEP pauseStepReady() override;
    E_SERVICE_STATUS_STEP pauseStepExecuting() override;
    E_SERVICE_STATUS_STEP pauseStepWaiting() override;
    E_SERVICE_STATUS_STEP pauseStepTerminaition() override;

    void goToRvizGoal();
    
private:

};
