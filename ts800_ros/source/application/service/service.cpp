#include "service.h"
#include "eblog.h"
#include "systemTool.h"
#include "MessageHandler.h"
#include "motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

service::service() {
    svcstatus = E_SERVICE_STATUS::status_void;
    id = E_SERVICE_ID::IDLE;
    starttime = 0.0;
    starttick = 0;
}

service::~service() {}

//-----------------------------------------------------------------------------
// 프로그램 로딩 시점에 서비스 관련 구조체 정보를 초기화한다.
//-----------------------------------------------------------------------------
void service::initializeSvc(E_SERVICE_ID set)
{
    // 서비스 처리 정보 초기화    
    svcStep.startup = E_SERVICE_STATUS_STEP::READY;
    svcStep.running = E_SERVICE_STATUS_STEP::READY;
    svcStep.paused = E_SERVICE_STATUS_STEP::READY;
    svcStep.completed = E_SERVICE_STATUS_STEP::READY;
    
    // 부팅으로 인한 호출
    if(svcstatus == E_SERVICE_STATUS::status_void){
        svcstatus = E_SERVICE_STATUS::initializing;
        id = set;
    }else{
        svcstatus = E_SERVICE_STATUS::startup;
        id = set;
    }
    
}

/**
 * @brief stratup의 step에 맞는 함수를 실행 시켜주는 함수 
 * 
 */
void service::serviceStartup()
{
     E_SERVICE_STATUS_STEP next;

    switch (getServiceStep(E_SERVICE_STATUS::startup))
    {
    case E_SERVICE_STATUS_STEP::READY:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : STARTUP READY");
        next = startupStepReady();
        setServiceStep(E_SERVICE_STATUS::startup, next);
        break;
    case E_SERVICE_STATUS_STEP::EXECUTING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : STARTUP EXECUTING");
        next = startupStepExecuting();
        setServiceStep(E_SERVICE_STATUS::startup, next);
        break;
    case E_SERVICE_STATUS_STEP::WAITING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : STARTUP WAITING");
        next = startupStepWaiting();
        setServiceStep(E_SERVICE_STATUS::startup, next);
        break;
    case E_SERVICE_STATUS_STEP::TERMINAITION:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : STARTUP TERMINAITION");
        next = startupStepTerminaition();
        setServiceStep(E_SERVICE_STATUS::startup, next);
        break;
    default:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS : " << enumToString(getServiceStep(E_SERVICE_STATUS::startup)));
        break;
    }
}
/**
 * @brief running의 step에 맞는 함수를 실행 시켜주는 함수 
 * 
 */
void service::serviceRunning()
{
    E_SERVICE_STATUS_STEP next;

    switch (getServiceStep(E_SERVICE_STATUS::running))
    {
    case E_SERVICE_STATUS_STEP::READY:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : RUNNING READY");
        next = runningStepReady();
        setServiceStep(E_SERVICE_STATUS::running, next);
        break;
    case E_SERVICE_STATUS_STEP::EXECUTING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : RUNNING EXECUTING");
        next = runningStepExecuting();
        setServiceStep(E_SERVICE_STATUS::running, next);
        break;
    case E_SERVICE_STATUS_STEP::WAITING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : RUNNING WAITING");
        next = runningStepWaiting();
        setServiceStep(E_SERVICE_STATUS::running, next);
        break;
    case E_SERVICE_STATUS_STEP::TERMINAITION:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : RUNNING TERMINAITION");
        next = runningStepTerminaition();
        setServiceStep(E_SERVICE_STATUS::running, next);
        break;
    default:
        break;
    }  
}
/**
 * @brief Pause의 step에 맞는 함수를 실행 시켜주는 함수 
 * 
 */
void service::servicePause()
{
    auto next = E_SERVICE_STATUS_STEP::READY;

    switch (getServiceStep(E_SERVICE_STATUS::paused))
    {
    case E_SERVICE_STATUS_STEP::READY:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : Pause READY");
        next = pauseStepReady();
        setServiceStep(E_SERVICE_STATUS::paused, next);
        break;
    case E_SERVICE_STATUS_STEP::EXECUTING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : Pause EXECUTING");
        next = pauseStepExecuting();
        setServiceStep(E_SERVICE_STATUS::paused, next);
        break;
    case E_SERVICE_STATUS_STEP::WAITING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : Pause WAITING");
        next = pauseStepWaiting();
        setServiceStep(E_SERVICE_STATUS::paused, next);
        break;
    case E_SERVICE_STATUS_STEP::TERMINAITION:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : Pause TERMINAITION");
        next = pauseStepTerminaition();
        setServiceStep(E_SERVICE_STATUS::paused, next);
        break;
    default:
        break;
    }  
}
/**
 * @brief completed의 step에 맞는 함수를 실행 시켜주는 함수 
 * 
 */
void service::serviceCompleted()
{
    E_SERVICE_STATUS_STEP next;

    switch (getServiceStep(E_SERVICE_STATUS::completed))
    {
    case E_SERVICE_STATUS_STEP::READY:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : completed READY");
        next = completedStepReady();
        setServiceStep(E_SERVICE_STATUS::completed, next);
        break;
    case E_SERVICE_STATUS_STEP::EXECUTING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : completed EXECUTING");
        next = completedStepExecuting();
        setServiceStep(E_SERVICE_STATUS::completed, next);
        break;
    case E_SERVICE_STATUS_STEP::WAITING:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : completed WAITING");
        next = completedStepWaiting();
        setServiceStep(E_SERVICE_STATUS::completed, next);
        break;
    case E_SERVICE_STATUS_STEP::TERMINAITION:
        eblog(LOG_LV_SERVICESTEP,  "E_SERVICE_STATUS_STEP : completed TERMINAITION");
        next = completedStepTerminaition();
        setServiceStep(E_SERVICE_STATUS::completed, next);
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
// 어플리케이션 동작 모드의 상태 정보를 확인한다.
//
//-----------------------------------------------------------------------------
E_SERVICE_ID service::getServiceId()
{
    return id;
}

E_SERVICE_STATUS service::getServiceStatus ( void )
{
    return svcstatus;
}

E_SERVICE_STATUS service::getPreSvcStatus()
{
    return preSvcStatus;
}


E_SERVICE_STATUS_STEP service::getServiceStep(E_SERVICE_STATUS status)
{
    E_SERVICE_STATUS_STEP ret;

    switch ( status )
    {
    case E_SERVICE_STATUS::startup : 
        ret = svcStep.startup;
        break;
    case E_SERVICE_STATUS::running :
        ret = svcStep.running;
        break;
    case E_SERVICE_STATUS::paused :
        ret = svcStep.paused;
        break;
    case E_SERVICE_STATUS::completed :
        ret = svcStep.completed;
        break;        
    default:
        eblog(LOG_LV,  "############## default");
        break;
    }

    return ret;
}

//-----------------------------------------------------------------------------
// 어플리케이션 동작 모드의 상태 정보를 갱신한다.
//
//-----------------------------------------------------------------------------
void service::setServiceStatus ( E_SERVICE_STATUS status )
{
    if (svcstatus != status)
    {
        setServiceStep(status, E_SERVICE_STATUS_STEP::READY);
        eblog(LOG_LV_NECESSARY,  "current : "<<enumToString(svcstatus)<<" set : " <<enumToString(status));
    }
    svcstatus = status;
}

/**
 * @brief Set the Service Status Step object
 * 
 * @param status 
 * @param step 
 */
void service::setServiceStep(E_SERVICE_STATUS status, E_SERVICE_STATUS_STEP step)
{
    switch ( status )
    {
    case E_SERVICE_STATUS::startup : 
        if (svcStep.startup != step)
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"STATUS : "<<WHITE<<enumToString(status)<<BOLDBLACK<< " ,STEP : "<<enumToString(svcStep.startup)<<BOLDBLACK<<" ==> "<<BOLDCYAN<<enumToString(step)<<BOLDBLACK<<" id : "<<BOLDCYAN<<enumToString(id));
        svcStep.startup = step;
        break;
    case E_SERVICE_STATUS::running :
        if (svcStep.running != step)
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"STATUS : "<<WHITE<<enumToString(status)<<BOLDBLACK<< " ,STEP : "<<enumToString(svcStep.running)<<BOLDBLACK<<" ==> "<<BOLDCYAN<<enumToString(step)<<BOLDBLACK<<" id : "<<BOLDCYAN<<enumToString(id));
        svcStep.running = step;
        break;
    case E_SERVICE_STATUS::paused :
        if (svcStep.paused != step)
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"STATUS : "<<WHITE<<enumToString(status)<<BOLDBLACK<< " ,STEP : "<<enumToString(svcStep.paused)<<BOLDBLACK<<" ==> "<<BOLDCYAN<<enumToString(step)<<BOLDBLACK<<" id : "<<BOLDCYAN<<enumToString(id));
        svcStep.paused = step;
        break;
    case E_SERVICE_STATUS::completed :
        if (svcStep.completed != step)
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"STATUS : "<<WHITE<<enumToString(status)<<BOLDBLACK<< " ,STEP : "<<enumToString(svcStep.completed)<<BOLDBLACK<<" ==> "<<BOLDCYAN<<enumToString(step)<<BOLDBLACK<<" id : "<<BOLDCYAN<<enumToString(id));
        svcStep.completed = step;
        break;        
    default:
        eblog(LOG_LV_NECESSARY,  "############## default");
        break;
    }
}


void service::serviceRun()
{    
    try
    {
        switch ( getServiceStatus() )
        {
        case E_SERVICE_STATUS::initializing :
            serviceInitial();
            break;
        case E_SERVICE_STATUS::startup     : 
            //eblog(LOG_LV, "svc_status_startup : " << E_SERVICE_ID_STR[(int)id]);
            serviceStartup();
            preSvcStatus = E_SERVICE_STATUS::startup;
            break;
        case E_SERVICE_STATUS::running     :
            serviceRunning();
            preSvcStatus = E_SERVICE_STATUS::running;
            break;
        case E_SERVICE_STATUS::paused      :
            //eblog(LOG_LV, "svc_status_paused : " << E_SERVICE_ID_STR[(int)id]);  
            servicePause();
            preSvcStatus = E_SERVICE_STATUS::paused;
            break;
        case E_SERVICE_STATUS::completed   :  
            //eblog(LOG_LV, ebcout("svc_status_completed"));            
            serviceCompleted();
            preSvcStatus = E_SERVICE_STATUS::completed;
            break;        
        default:
            eblog(LOG_LV,  "############## default");
            break;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";        
        std::cerr <<"ServiceRun : "<< e.what() << '\n';
    }
    
}


/**
 * @brief 외부로부터 온 서비스 제어명령 중 service 제어를 처리한다. 현재 서비스 상태에 따라 제어가 무시 될 수 있다.
 * @param ctr : 변경되어야하는 서비스 명령
 * @param status : 현재 서비스 상태
 */

void service::serviceControl(E_SERVICE_CTR ctr)
{
    E_SERVICE_STATUS status = getServiceStatus();

    // 디버그용 로그 
    ceblog(LOG_LV_SERVICESTEP, BLUE, "serviceControl status:"<< enumToString(status));


    switch (ctr)
    {    
    case E_SERVICE_CTR::IDLE :
        break;
    case E_SERVICE_CTR::START :
        serviceCtrlStart(status, ctr);
        break;    
    case E_SERVICE_CTR::RUN :
        serviceCtrlRun(status, ctr);
        break;
    case E_SERVICE_CTR::PAUSE :
        serviceCtrlPause(status, ctr);
        break;
    case E_SERVICE_CTR::CANCEL : 
        serviceCtrlCancel(status, ctr);
        break;  
    default:
        break;
    }
}

void service::serviceCtrlStart(E_SERVICE_STATUS status, E_SERVICE_CTR ctr)
{
    setServiceStatus(E_SERVICE_STATUS::startup);
}

void service::serviceCtrlRun(E_SERVICE_STATUS status, E_SERVICE_CTR ctr)
{
    if( status == E_SERVICE_STATUS::paused){
        setServiceStatus(E_SERVICE_STATUS::running);
    }else{
        setServiceStatus(E_SERVICE_STATUS::startup);
    }          
}

void service::serviceCtrlPause(E_SERVICE_STATUS status, E_SERVICE_CTR ctr)
{
    if ( status != E_SERVICE_STATUS::paused ) 
    {
        setServiceStatus(E_SERVICE_STATUS::paused);
        ceblog(LOG_LV_NECESSARY, BLUE, "Now E_SERVICE_CTR : "<< enumToString(ctr)<<" Service Status Change : "<< enumToString(E_SERVICE_STATUS::paused));
    }
}

void service::serviceCtrlCancel(E_SERVICE_STATUS status, E_SERVICE_CTR ctr)
{
    //if ( status == E_SERVICE_STATUS::running || status == E_SERVICE_STATUS:: paused || status == E_SERVICE_STATUS:: startup) 
    {
        completedByForce = true;
        setServiceStatus(E_SERVICE_STATUS::completed);
        ceblog(LOG_LV_NECESSARY, BLUE, "Now E_SERVICE_CTR : "<< enumToString(ctr)<<" Service Status Change : "<< enumToString(E_SERVICE_STATUS::completed));
    }
    //else    ceblog(LOG_LV_NECESSARY, RED, "Now status:"<< enumToString(status)<<"-> So Service Control Cannot Change to 'completed' Status");
}

//hjkim230428 서비스 시작 시점 저장 및 서비스 동작시간 관리함수 추가
u32 service::getServiceStartTick()
{
    return starttick;
}
double service::getServiceStartTime()
{
    return SYSTEM_TOOL.getSystemTime()-starttime;
}
void service::setServiceStartTime()
{
    starttime = SYSTEM_TOOL.getSystemTime();
    starttick = SYSTEM_TOOL.getSystemTick();
}
