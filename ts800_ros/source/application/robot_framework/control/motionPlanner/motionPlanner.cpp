#include <thread>
#include "coordinate.h"
#include "control/control.h"
#include "motionPlanner/motionPlanner.h"
#include "motionController.h"
#include "eblog.h"
#include "motionPlanner.h"
#include "MessageHandler.h"
#include "debugCtr.h"
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CPathPlanner::CPathPlanner()
{
    pPathFinder = new CDstar();    
    
    ceblog(LOG_LV_NECESSARY, GREEN, " called  "); 
}

CPathPlanner::~CPathPlanner()
{
    delete []pPathFinder;    
    ceblog(LOG_LV_NECESSARY, GREEN, " called  "); 
}

CPathPlanner &CPathPlanner::getInstance()
{
    CStopWatch __debug_sw;
    
    static CPathPlanner s;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return s;
}

