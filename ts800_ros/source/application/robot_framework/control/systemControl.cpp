#include "control/systemControl.h"
#include "MessageHandler.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CSystemControl::CSystemControl(){}
CSystemControl::~CSystemControl(){}

void CSystemControl::initTofSensor()
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_INIT_SENSOR;
    SEND_MESSAGE(message_t(type));
}

void CSystemControl::initImuSensor()
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_INIT_MOVING;
    SEND_MESSAGE(message_t(type));
}