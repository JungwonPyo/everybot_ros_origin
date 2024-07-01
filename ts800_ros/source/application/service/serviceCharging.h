/**
 * @file serviceCharging.h
 * @author hhryu
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
#include "serviceReady.h"
#include "taskCharging.h"
#include "serviceMacro.h"

#define DRY_FAN_TIMEOUT 600.0
#define BATTERY_DISPLAY_INTERVAL 5.0
#define DRY_FAN_DISPLAY_INTERVAL 5.0
#define DOCKING_ALIVE_LIFE_TIME 1.0
class CServiceCharging : public service
{
private:
    CServiceMacro* pServiceMacro;
    CServiceReady* pServiceReady;
    CTaskCharging taskCharging;

private:
    double dockingAliveTime;
    
public:
    CServiceCharging(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady);
    ~CServiceCharging();    
    bool checkReadytoCharge();
    CTaskCharging* getChargingTaskPointer();
    
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
};