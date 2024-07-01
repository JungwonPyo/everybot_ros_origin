/**
 * @file serviceUnDocking.h
 * @author hhryu@everybot.net
 * @brief 충전 중, 청소 or 탐색 or 지정 장소 이동 or 예약 청소 등등...을 위한 CLASS
 * @version 0.1
 * @date 2023-04-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>

#include "define.h"
#include "service.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "robotSlamPose.h"
#include "serviceReady.h"
#include "taskUnDocking.h"
#include "serviceMacro.h"
typedef struct
{
    double x;   //단위 : m
    double y;   //단위 : m
    double angle;   //단위 : rad
}tServiceUnDockingPoseInfo;

class CServiceUnDocking : public service
{
private:
    CServiceMacro* pServiceMacro;
    CServiceReady* pServiceReady;    
    CTaskUnDocking taskUndocking;

    //로봇의 PASUE STATE 자기 위치 좌표
    CRobotSlamPose* pRobotSlamPose;

public:
    CServiceUnDocking(CServiceMacro* _pServiceMacro,CServiceReady* _pServiceReady, CRobotSlamPose* _pRobotSlamPose);
    ~CServiceUnDocking();
    E_SERVICE_STATUS serviceProc();
    bool checkReadytoUndocking();

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

    tServiceUnDockingPoseInfo serviceUnDockingSlamPoseInfo;

};