
#include "serviceMacro.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 2.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CServiceMacro::CServiceMacro() {}
CServiceMacro::~CServiceMacro() {}

std::list<contextDo> CServiceMacro::getDoList()
{
    return doList;
}

void CServiceMacro::macroServiceConnector(E_SERVICE_ID doId)
{
    // 새로운 서비스 실행
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        doId,
        E_SERVICE_STATUS::startup));

    // 새로운 서비스 완료 대기
    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        doId,
        E_SERVICE_STATUS::completed));
    eblog(LOG_LV_NECESSARY, " CONNECT ID : " << enumToString(doId));
}
// 이전 서비스 취소 -> 새로운 서비스 실행 -> 새로운 서비스 완료 -> 기본 IDLE 서비스로 전환
void CServiceMacro::macroServiceStart(E_SERVICE_ID curId, E_SERVICE_ID doId)
{
    CStopWatch __debug_sw;
    clearDolist();
    // 이전 서비스 취소
    doList.push_back(contextDo(
        E_CONTEXT_DO::doCancle,
        curId,
        E_SERVICE_STATUS::completed));

    // 새로운 서비스 실행
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        doId,
        E_SERVICE_STATUS::startup));

    // 새로운 서비스 완료 대기
    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        doId,
        E_SERVICE_STATUS::completed));
    eblog(LOG_LV_NECESSARY, " CANCLE ID : " << enumToString(curId) << " ACTIVE ID : " << enumToString(doId));
    TIME_CHECK_END(__debug_sw.getTime());
}

// 이전 서비스 취소 하고 idle 서비스로 전환
void CServiceMacro::macroServiceStop(E_SERVICE_ID curId)
{
    CStopWatch __debug_sw;
    clearDolist();
    // 이전 서비스 취소
    doList.push_back(contextDo(
        E_CONTEXT_DO::doCancle,
        curId,
        E_SERVICE_STATUS::completed));

    // 기본 idle 서비스로 전환.
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        E_SERVICE_ID::IDLE,
        E_SERVICE_STATUS::startup));
    
    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        E_SERVICE_ID::IDLE,
        E_SERVICE_STATUS::completed));    
    eblog(LOG_LV_NECESSARY, " STOP ID : " << enumToString(curId) << " --> STANDBY");
    TIME_CHECK_END(__debug_sw.getTime());
}

// 서비스 일시정지
void CServiceMacro::macroServicePause(E_SERVICE_ID curId)
{
    CStopWatch __debug_sw;
    clearDolist();
    // 일시 정지
    doList.push_back(contextDo(
        E_CONTEXT_DO::doPause,
        curId,
        E_SERVICE_STATUS::paused));
    doList.push_back(contextDo(
        E_CONTEXT_DO::doWait,
        curId,
        E_SERVICE_STATUS::paused));    
        eblog(LOG_LV_NECESSARY, " Pause ID : " << enumToString(curId));

    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceMacro::macroServiceResume(E_SERVICE_ID curId)
{
    CStopWatch __debug_sw;
    clearDolist();

    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        curId,
        E_SERVICE_STATUS::running));
    // 새로운 서비스 완료 대기
    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        curId,
        E_SERVICE_STATUS::completed));

    // 기본 idle 서비스로 전환.
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        E_SERVICE_ID::IDLE,
        E_SERVICE_STATUS::startup));
        eblog(LOG_LV_NECESSARY, " Resume ID : " << enumToString(curId));
    
    TIME_CHECK_END(__debug_sw.getTime());
}

// 부팅해서 idle 서비스 가는 로직.
void CServiceMacro::bootIdle()
{
    CStopWatch __debug_sw;
    clearDolist();
    // 새로운 서비스 실행
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        E_SERVICE_ID::IDLE,
        E_SERVICE_STATUS::startup));

    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        E_SERVICE_ID::IDLE,
        E_SERVICE_STATUS::completed));    
    eblog(LOG_LV_NECESSARY, " BOOT LIDE --> STANDBY");
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceMacro::macroServiceCharge(E_SERVICE_ID curId)
{
    CStopWatch __debug_sw;
    clearDolist();
    // 이전 서비스 취소
    doList.push_back(contextDo(
        E_CONTEXT_DO::doCancle,
        curId,
        E_SERVICE_STATUS::completed));

    // 새로운 서비스 실행
    doList.push_back(contextDo(
        E_CONTEXT_DO::doIt,
        E_SERVICE_ID::CHARGING,
        E_SERVICE_STATUS::startup));

    doList.push_back(contextDo(
        E_CONTEXT_DO::doUntil,
        E_SERVICE_ID::CHARGING,
        E_SERVICE_STATUS::completed));    

    eblog(LOG_LV_NECESSARY, " START CHARGE  CUR ID : " << enumToString(curId));
    TIME_CHECK_END(__debug_sw.getTime());
}

// 언 도킹 -> 입력된 서비스 활성화 -> 도킹 서비스 활성화
void CServiceMacro::macroProcess(E_SERVICE_ID curId, E_SERVICE_ID doId)
{
    CStopWatch __debug_sw;
    E_TERMINAL_STATE terminal = ServiceData.power.getTerminalState();

    switch (doId)
    {
    case E_SERVICE_ID::IDLE:
        macroRunOnDocked(curId, doId);
        break;
    case E_SERVICE_ID::DOCKING:
        if (terminal == E_TERMINAL_STATE::DOCKED || terminal == E_TERMINAL_STATE::DCJACK_IN)
        {
            eblog(LOG_LV_NECESSARY, " DOCKING SERVICE NOT ACTIVE  - already docked");
        }
        else 
        {
            // ROBOT_CONTROL.system.sensor(); // 충전 후 데이터가 이상하기 때문에 센서를 초기화한다.
            // ROBOT_CONTROL.system.moving(); // 충전 후 데이터가 이상하기 때문에 moving data 를 초기화한다.
           macroServiceStart(curId, doId);
        }
        break;
    case E_SERVICE_ID::EXPLORER:
        if (terminal == E_TERMINAL_STATE::DOCKED || terminal == E_TERMINAL_STATE::DCJACK_IN)
        {
           macroRunOnDocked(curId, doId);
        }
        else
        {
            macroRunOnStanby(curId, doId);
        }
        break;
    case E_SERVICE_ID::CLEAN:
        if (terminal == E_TERMINAL_STATE::DOCKED || terminal == E_TERMINAL_STATE::DCJACK_IN)
        {
            macroRunOnDocked(curId, doId);
        }
        else
        {
            macroRunOnStanby(curId, doId);
        }
        break;
    case E_SERVICE_ID::WIFI:
        macroServiceStart(curId, doId);
        macroServiceConnector(E_SERVICE_ID::IDLE);
        break;    
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
}
// 입력된 서비스 활성화 -> 도킹 서비스 활성화
void CServiceMacro::macroRunOnDocked(E_SERVICE_ID curId, E_SERVICE_ID doId)
{
    CStopWatch __debug_sw;

    macroServiceStart(curId, E_SERVICE_ID::UNDOCKING);
    if(doId == E_SERVICE_ID::CLEAN)
    {
        #if 0
        if(ROBOT_CONTROL.slam.isExistedSlamMap())    // (임시) 지도 있어도 탐색 하도록 수정 2024.06.10
        {
            macroServiceConnector(doId);
        }
        else
        #endif
        {
           macroServiceConnector(E_SERVICE_ID::EXPLORER);
           macroServiceConnector(doId);
        }     
    }
    else
    {
       macroServiceConnector(doId);
    }

    macroServiceConnector(E_SERVICE_ID::DOCKING);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

// 입력된 서비스 활성화 -> 도킹 서비스 활성화
void CServiceMacro::macroRunOnStanby(E_SERVICE_ID curId, E_SERVICE_ID doId)
{
    CStopWatch __debug_sw;

    if(doId == E_SERVICE_ID::CLEAN)
    {
        #if 0
        if(ROBOT_CONTROL.slam.isExistedSlamMap())    // (임시) 지도 있어도 탐색 하도록 수정 2024.06.10
        {
            macroServiceStart(curId,doId);
        }
        else
        #endif
        {
            macroServiceStart(curId,E_SERVICE_ID::EXPLORER);
            macroServiceConnector( doId);
        }
    }
    else
    {
        macroServiceStart(curId,doId);
    }

    macroServiceConnector(E_SERVICE_ID::DOCKING);

    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceMacro::popDolist()
{
    CStopWatch __debug_sw;
    doList.pop_front();
    TIME_CHECK_END(__debug_sw.getTime());
}

void CServiceMacro::clearDolist()
{
    CStopWatch __debug_sw;

    if (doList.empty() == false){
        eblog(LOG_LV_NECESSARY, " Service DoList Clear!!");
        doList.clear();
    }else{
        eblog(LOG_LV_NECESSARY, " Service DoList Aready Empty!!");
    }

    TIME_CHECK_END(__debug_sw.getTime());
}
