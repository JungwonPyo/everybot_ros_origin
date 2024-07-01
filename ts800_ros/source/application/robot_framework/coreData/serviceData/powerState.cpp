#include "coreData/serviceData/powerState.h"
#include "externData/externData.h"
#include "ebtime.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CPowerState::CPowerState() {
    power_checkTime = 0;
    bChangeState = false;
    state = E_TERMINAL_STATE::NORMAL;
}

CPowerState::~CPowerState() {}

void CPowerState::update(CExternData* pExternData)
{
    if ( pExternData->systemData.isUpdatePowerData() )
    {
        updateTerminalState( pExternData->systemData.usePowerData() );
    }
}

E_TERMINAL_STATE CPowerState::getTerminalState()
{
    return state;
}

E_TERMINAL_STATE CPowerState::getTerminalPreState()
{
    return preState;
}

E_POWER_STATE CPowerState::getPowerState()
{
    return powerState;
}

bool CPowerState::getExtPower()
{
    return extpower;
}

void CPowerState::updateTerminalState(tSysPower power)
{
    double checkTime = SYSTEM_TOOL.getSystemTime()- power_checkTime;
    preState = state;
    powerState = (E_POWER_STATE)power.state;
    extpower = power.adaptor_in;
    //ceblog(LOG_LV_NECESSARY,BLUE, "updateTerminalState  adaptor : "<<(int)power.adaptor_in<<" , terminal : "<<(int)state << " checkTime : " << checkTime);
    if(state == E_TERMINAL_STATE::DOCKED)
    {
        if(!power.adaptor_in)//if(state != E_POWER_STATE::CHARGE)
        {
            if(!bChangeState){
                bChangeState = true;
                power_checkTime = SYSTEM_TOOL.getSystemTime();
            }
            else if(checkTime >= 1)
            {
                bChangeState = false;
                state = E_TERMINAL_STATE::NORMAL;
                ceblog(LOG_LV_NECESSARY,GREEN, "updateTerminalState adaptor Dis-Connet : "<<(int)power.adaptor_in<<" , terminal : "<<(int)state);
            }
        }
    }
    else if(state == E_TERMINAL_STATE::NORMAL)
    {
        if(power.adaptor_in)//if(state == E_POWER_STATE::CHARGE)
        {
            if(!bChangeState){
                bChangeState = true;
                power_checkTime = SYSTEM_TOOL.getSystemTime();
            }
            else if(checkTime >= 0.1)
            {
                bChangeState = false;
                state = E_TERMINAL_STATE::DOCKED;
                ceblog(LOG_LV_NECESSARY,GREEN, "updateTerminalState adaptor Connet : "<<(int)power.adaptor_in<<" , terminal : "<<(int)state); //충전이 아닌 상태에서 단자 인식이 되면 로그 출력됨 - 1초 유지 시 docked 상태로 전환
            }                      
        }
    }
    else{
        bChangeState = false;
        if(!power.adaptor_in) state = E_TERMINAL_STATE::NORMAL;
        else                  state = E_TERMINAL_STATE::DOCKED;  
    }
}