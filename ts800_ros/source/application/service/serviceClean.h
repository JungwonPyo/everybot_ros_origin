/**
 * @file serviceClean.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <thread>

#include "coreData/serviceData.h"
#include "define.h"
#include "service.h"
#include "supplyWater.h"
#include "control/control.h"
#include "location.h"
#include "kidnap.h"
#include "robotSlamPose.h"
#include "serviceReady.h"
#include "taskClean.h"
#include "taskLocalMotion.h"
#include "taskReLocalMotion.h"
#include "serviceMacro.h"
class CServiceClean : public service
{
private:
    CServiceMacro* pServiceMacro;
    CServiceReady* pServiceReady;
    double readyStartTime;
    
    CSupplyWater *pSupplyWater;
    CLocation* pLocation;
    CKidnap* pKidnap;


    CTaskClean taskClean;
    CTaskLocalMotion taskLocalMotion;
    CTaskReLocalMotion taskReLocalMotion;

    //로봇의 PASUE STATE 자기 위치 좌표
    CRobotSlamPose* pRobotSlamPose;

    double cleanStartTime;
    double cleanTime;

public:
    CServiceClean(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CSupplyWater* _pSupplyWater, CLocation* _pLocation, CKidnap* _pKidnap, CRobotSlamPose* _pRobotSlamPose);
    ~CServiceClean();    
    
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

    E_SERVICE_STATUS_STEP pauseStepReady() override;
    E_SERVICE_STATUS_STEP pauseStepExecuting() override;
    E_SERVICE_STATUS_STEP pauseStepWaiting() override;
    E_SERVICE_STATUS_STEP pauseStepTerminaition() override;

    E_SERVICE_STATUS_STEP completedStepReady() override;
    E_SERVICE_STATUS_STEP completedStepExecuting() override;
    E_SERVICE_STATUS_STEP completedStepWaiting() override;
    E_SERVICE_STATUS_STEP completedStepTerminaition() override;

    /*********************************/
    //debug 추후 서비스 방향에 따라 수정 및 삭제
    enum demo_state
    {
        _ready,
        _go_straight,
        _left_turn,
        _demo_end,
    };
    int demo_time_cnt;
    int demo_cnt;
    demo_state state;
    /*********************************/
public:/* serviceClean -> Dispatcher */    
    void setCleanMode(int set);
    
};
