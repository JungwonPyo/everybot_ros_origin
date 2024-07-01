/**
 * @file serviceWifi.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <thread>

#include "define.h"
#include "coreData/serviceData.h"
#include "service.h"
#include "serviceReady.h"
#include "wifi.h"

class CServiceWifi : public service
{
private:
    double readyStartTime;
    double exitStartTime;
    CServiceReady* pServiceReady;    
    CWifi* pWifi;

    E_POWER_MODE powerMode;
    bool connectToggle;
    bool isSendMsg;


public:
    CServiceWifi(CServiceReady* _pServiceReady);
    ~CServiceWifi();    

    bool checkReadytoWifi();
    
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
