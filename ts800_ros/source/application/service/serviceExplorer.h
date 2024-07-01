/**
 * @file serviceExplorer.h
 * @author jhnoh
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

#include "define.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "service.h"
#include "taskExplorer.h"
#include "rsuContext.h"
#include "robotSlamPose.h"
#include "serviceReady.h"
#include "kidnap.h"
#include "serviceMacro.h"
typedef struct
{
    double x;   //단위 : m
    double y;   //단위 : m
    double angle;   //단위 : rad
}tServiceExplorerPoseInfo;


class CServiceExplorer : public service
{
private:
    CServiceMacro* pServiceMacro;
    CServiceReady* pServiceReady;
    CTaskExplorer *pTaskExplorer;    
    CKidnap* pKidnap;

    //로봇의 PASUE STATE 자기 위치 좌표
    CRobotSlamPose* pRobotSlamPose;
    int savedFileIndex;
    int savedFileTick;
    
public:
    CServiceExplorer(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CRobotSlamPose* _pRobotSlamPose, CKidnap* _pKidnap);
    ~CServiceExplorer();
    bool checkReadytoExplore();

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


    
private:
};
