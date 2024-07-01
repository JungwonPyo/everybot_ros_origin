#include "coreData/serviceData/appKey.h"
#include "eblog.h"
#include "control/control.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/


CAppKey::CAppKey() {}
CAppKey::~CAppKey() {}

void CAppKey::initAppKey()
{
    actionKey = E_ACTION_KEY::NONE;
}

void CAppKey::setActionKey(short action)
{
    switch (action)
    {
        case 0: { actionKey = E_ACTION_KEY::NONE;break; }
        case 1: { actionKey = E_ACTION_KEY::CLEAN;break; }
        case 2: { actionKey = E_ACTION_KEY::EXPLORER;break; } 
        case 3: { actionKey = E_ACTION_KEY::HOMING;break; }
        case 4: { actionKey = E_ACTION_KEY::STOP;break; } 
        case 5: { actionKey = E_ACTION_KEY::DELETE_MAP;break; }
        case 6: { ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "위치알림 기능 없음");break; } // 위치알림 기능은 미정
        case 7: { actionKey = E_ACTION_KEY::START_STOP_DRY_MOP;break; }
        case 8: { actionKey = E_ACTION_KEY::START_STOP_DRAIN_WATER;break; }
        case 9: { actionKey = E_ACTION_KEY::START_FW_UPDATE;break; }
        case 10: { actionKey = E_ACTION_KEY::START_FW_RECOVERY;break; }
        case 11: { actionKey = E_ACTION_KEY::START_INIT_USERSET;break; }
        case 12: { actionKey = E_ACTION_KEY::START_FACTORY_RESET;break; }

        default:
            actionKey = E_ACTION_KEY::NONE;
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "actionKey["<<action<<"] is not valid.");
            break;
    }
    
    if(actionKey != E_ACTION_KEY::NONE)
    {
        { ceblog(LOG_LV_AWS, CYN, " actionKey[" << action << "]");}
        { ceblog(LOG_LV_SYSTEMINF, CYN, " actionKey[" << action << "]");}
        ROBOT_CONTROL.reportAwsAction(action);
    }   
}

E_ACTION_KEY CAppKey::getActionKey()
{
    E_ACTION_KEY ret = actionKey;
    actionKey = E_ACTION_KEY::NONE;
    return ret;
}