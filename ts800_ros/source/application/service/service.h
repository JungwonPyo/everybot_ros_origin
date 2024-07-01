/**
 * @file service.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <unordered_map>
#include "ebtypedef.h"
#include "commonStruct.h"


//서비스 컨트롤러. 서비스를 어떻게 처리할지 명령 한다.
enum class E_SERVICE_CTR
{    
    IDLE,       //서비스 활성화 명령
    START, 
    RUN,        //서비스 실행 명령
    PAUSE,      //서비스 일시정지 명령.
    CANCEL,     //서비스 취소 명령    
};
static std::string enumToString(E_SERVICE_CTR value) {
    static const std::unordered_map<E_SERVICE_CTR, std::string> enumToStringMap = {
        { E_SERVICE_CTR::IDLE, "IDLE,"},
        { E_SERVICE_CTR::START, "START,"},
        { E_SERVICE_CTR::RUN, "RUN,"},
        { E_SERVICE_CTR::PAUSE, "PAUSE,"},
        { E_SERVICE_CTR::CANCEL, "CANCEL,"}
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_SERVICE_STATUS
{    
    status_void      = 0,
    initializing      = 1,
    startup   = 2,
    running   = 3,
    paused    = 4,    
    completed = 5,    
};
static std::string enumToString(E_SERVICE_STATUS value) {
    static const std::unordered_map<E_SERVICE_STATUS, std::string> enumToStringMap = {
        { E_SERVICE_STATUS::status_void, "status_void" },
        { E_SERVICE_STATUS::initializing, "initializing" },
        { E_SERVICE_STATUS::startup, "startup" },
        { E_SERVICE_STATUS::running, "running" },
        { E_SERVICE_STATUS::paused, "paused" },
        { E_SERVICE_STATUS::completed, "completed" }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum class E_SERVICE_STATUS_STEP
{
    READY,       // 스테터스를 수행 하기위한 초기 단계 (한번만 실행)   
    EXECUTING,        // 스테터스를 수행 단계(반복 실행)
    WAITING,    // 스테터스 대기 단계
    TERMINAITION,       // 스테터스 종료 단계    
};
static std::string enumToString(E_SERVICE_STATUS_STEP value) {
    static const std::unordered_map<E_SERVICE_STATUS_STEP, std::string> enumToStringMap = {
        { E_SERVICE_STATUS_STEP::READY, "READY," },
        { E_SERVICE_STATUS_STEP::EXECUTING, "EXECUTING," },
        { E_SERVICE_STATUS_STEP::WAITING, "WAITING," },
        { E_SERVICE_STATUS_STEP::TERMINAITION, "TERMINAITION," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

struct tServiceStatusStep
{
    E_SERVICE_STATUS_STEP startup;
    E_SERVICE_STATUS_STEP running;
    E_SERVICE_STATUS_STEP paused;
    E_SERVICE_STATUS_STEP completed;
};


class service
{
private:
    /**
    * @brief 크게 3가지 정보가 존재함 (1.서비스 ID, 2.서비스 상태, 3.서비스 상태 단계)
    * 1. 서비스 ID : 가장 큰 개념이다. 어떤 서비스를 실행하지 정하는 정보
    * 2. 서비스 상태 : 각 서비스에는 절차가 존재한다.(initializing, startup, running, pause, complete)
    * 3. 서비스 상태 단계 : 각 상태마다 실행되는 절차가 존재한다.(INIT, RUN, WAITING, STOP)
    * 번외 : 예전 서비스 ready 상태가 존재했으며 삭제했으나 잔재가 있을 수 있다.
    */  
    E_SERVICE_ID id;
    E_SERVICE_STATUS preSvcStatus;
    E_SERVICE_STATUS svcstatus;
    tServiceStatusStep svcStep;

    double starttime;
    u32 starttick;
public:
    bool completedByForce = false;

    double awsTime;
    
    service();
    ~service();
    E_SERVICE_STATUS getPreSvcStatus();
    
    virtual void serviceInitial() = 0;

    void serviceStartup();
    virtual E_SERVICE_STATUS_STEP startupStepReady() = 0;
    virtual E_SERVICE_STATUS_STEP startupStepExecuting() = 0;
    virtual E_SERVICE_STATUS_STEP startupStepWaiting() = 0;
    virtual E_SERVICE_STATUS_STEP startupStepTerminaition() = 0;

    void serviceRunning();
    virtual E_SERVICE_STATUS_STEP runningStepReady() = 0;
    virtual E_SERVICE_STATUS_STEP runningStepExecuting() = 0;
    virtual E_SERVICE_STATUS_STEP runningStepWaiting() = 0;
    virtual E_SERVICE_STATUS_STEP runningStepTerminaition() = 0;

    void servicePause();
    virtual E_SERVICE_STATUS_STEP pauseStepReady() = 0;
    virtual E_SERVICE_STATUS_STEP pauseStepExecuting() = 0;
    virtual E_SERVICE_STATUS_STEP pauseStepWaiting() = 0;
    virtual E_SERVICE_STATUS_STEP pauseStepTerminaition() = 0;
    
    void serviceCompleted();
    virtual E_SERVICE_STATUS_STEP completedStepReady() = 0;
    virtual E_SERVICE_STATUS_STEP completedStepExecuting() = 0;
    virtual E_SERVICE_STATUS_STEP completedStepWaiting() = 0;
    virtual E_SERVICE_STATUS_STEP completedStepTerminaition() = 0;
    
    void serviceControl(E_SERVICE_CTR ctr);
    void serviceCtrlStart(E_SERVICE_STATUS status, E_SERVICE_CTR ctr);
    void serviceCtrlRun(E_SERVICE_STATUS status, E_SERVICE_CTR ctr);
    void serviceCtrlPause(E_SERVICE_STATUS status, E_SERVICE_CTR ctr);
    void serviceCtrlCancel(E_SERVICE_STATUS status, E_SERVICE_CTR ctr);
    void setServiceStatus ( E_SERVICE_STATUS status );
    void setServiceStep ( E_SERVICE_STATUS status, E_SERVICE_STATUS_STEP step);
    E_SERVICE_STATUS getServiceStatus ();
    E_SERVICE_STATUS_STEP getServiceStep(E_SERVICE_STATUS status);
        
    void serviceRun();    
    void initializeSvc(E_SERVICE_ID set);
    E_SERVICE_ID getServiceId();
    void setServiceStartTime();
    double getServiceStartTime();
    u32 getServiceStartTick();
};
