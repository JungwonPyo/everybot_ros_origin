/**
 * @file rsuContext.h
 * @author icbaek
 * @brief Application Context 관리 모듈
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/serviceData/powerState.h"
#include "commonStruct.h"
#include "service.h"
#include "serviceReady.h"

enum class E_CONTEXT_DO {
    doIt,           // 이거     하세요.
    doPause,        // 일시정지 하세요.
    doWait,         // 대기     하세요.
    doUntil,        // 끝날때까지 진행하세요.
    doCancle,       // 취소     하세요.
};

//해야할일 
struct contextDo
{
    contextDo(E_CONTEXT_DO _what, E_SERVICE_ID _id, E_SERVICE_STATUS _status) : 
        what{_what}, id{_id}, status{_status} {}
    E_CONTEXT_DO what;
    E_SERVICE_ID id;
    E_SERVICE_STATUS status;
};


class CRsuContext //: public CRsfCore
{
private:
    service* pService;
    CServiceReady* pServiceReady;
    E_TERMINAL_STATE preTerminalState;

public:
    CRsuContext(service* _service, CServiceReady* _pServiceReady);
    ~CRsuContext();
        
    void setService(service* _service);
    void serviceRun();
    void serviceControl(E_SERVICE_CTR ctr);        
        
    E_SERVICE_STATUS getServiceStatus();
    E_SERVICE_STATUS_STEP getServiceStep(E_SERVICE_STATUS status);
    E_SERVICE_READY getServiceReadyState();
    E_SERVICE_ID getServiceId();    
    int  getServiceTime ( void );
};


