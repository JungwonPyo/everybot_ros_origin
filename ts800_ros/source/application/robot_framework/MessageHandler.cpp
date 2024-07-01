
#include <iostream>
#include <thread>
#include "MessageHandler.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CMessageHandler::CMessageHandler()
{
    mMessageQueue.clear();
    eblog(LOG_LV,  "");
}

CMessageHandler::~CMessageHandler()
{
    eblog(LOG_LV,  "");
}

CMessageHandler& CMessageHandler::getInstance()
{
    static CMessageHandler s;
    return s;
}

void CMessageHandler::sendMessage(message_t message)
{        
    try
    {
        #if USE_SHARED_PTR == 1
            if ( message == nullptr)    return;
        #endif

        mMessageQueue.enqueue(message);            
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
}

message_t CMessageHandler::dequeue()
{
    return mMessageQueue.dequeue();
}

void CMessageHandler::clear()
{
    mMessageQueue.clear();
}

void CMessageHandler::destory() 
{        
    mMessageQueue.destory();
}
