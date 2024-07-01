#include "coreData/serviceData/tiltingState.h"
#include "externData/externData.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CTiltingState::CTiltingState()
{
    state = E_SYS_TILT_STATE::UNKNOW;
}

CTiltingState::~CTiltingState() {}

void CTiltingState::update(CExternData* pExternData)
{
    if ( pExternData->systemData.isUpdateTiltingData() == true )
    {
        setStateValue( pExternData->systemData.useTiltingData().state );
        // ceblog(LOG_LV_SYSTEMINF, GREEN, "tilting state : " << SC<int>(getStateValue()) );
    }
}

void CTiltingState::setStateValue(E_SYS_TILT_STATE newState)
{
#if 1 //debug
    if (state != newState)
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "틸팅 상태 변경 : "<< enumToString(state)
            <<"  ->  "<<enumToString(newState));
#endif
    state = newState;
}

E_SYS_TILT_STATE CTiltingState::getStateValue()
{
    return state;
}