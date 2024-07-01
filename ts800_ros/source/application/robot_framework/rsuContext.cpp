
#include "rsuContext.h"
#include "systemTool.h"
#include "eblog.h"
#include <sys/time.h>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRsuContext::CRsuContext(service* _service,CServiceReady* _pServiceReady)
{
    pService= _service;
    pServiceReady = _pServiceReady;
    preTerminalState = E_TERMINAL_STATE::NOT_WORK;
    eblog(LOG_LV, "");
}

CRsuContext::~CRsuContext()
{
    eblog(LOG_LV,  "");
}


void CRsuContext::setService(service* _service)
{
    if (pService != _service)
    {
        eblog(LOG_LV,  "pre id : "<<enumToString(pService->getServiceId())<< 
            " set id:"<<enumToString(_service->getServiceId()));
    }    
    pService= _service;
}

void CRsuContext::serviceRun()
{
    pService->serviceRun();
}

void CRsuContext::serviceControl(E_SERVICE_CTR ctr)
{
    pService->serviceControl(ctr);
}

//-----------------------------------------------------------------------------
// 어플리케이션 동작 시간 정보를 구한다.
//
//-----------------------------------------------------------------------------

int  CRsuContext::getServiceTime ( void )
{    
    return ( SYSTEM_TOOL.getSystemTick() - pService->getServiceStartTick() ) ;
}

E_SERVICE_STATUS CRsuContext::getServiceStatus()
{
    return pService->getServiceStatus();
}

E_SERVICE_STATUS_STEP CRsuContext::getServiceStep(E_SERVICE_STATUS status)
{
    return pService->getServiceStep(status);
}

E_SERVICE_READY CRsuContext::getServiceReadyState()
{
    return pServiceReady->getServiceReadyState();//pService->getServiceReadyState();
}

E_SERVICE_ID CRsuContext::getServiceId()
{
    return pService->getServiceId();
}
