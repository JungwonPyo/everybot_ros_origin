#include "taskError.h"
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "control/control.h"
#include "motionController.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/



/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskError::CTaskError()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskError::~CTaskError()
{
    CStopWatch __debug_sw;

    setState(ERROR_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskError::setState(ERROR_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[ERROR_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskError::taskStart()
{
    setState(ERROR_STATE::START);
    startTime = SYSTEM_TOOL.getSystemTime();
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
}

bool CTaskError::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    switch (state)
    {
    case ERROR_STATE::NONE :
        /* code */
        break;
    case ERROR_STATE::START :
         
        break;
    case ERROR_STATE::RUN :
        /* code */
        break;
    case ERROR_STATE::POWER_OFF :
        /* code */
        break;
    case ERROR_STATE::COMPLETE :
        /* code */
        break;            
    default:
        break;
    }
  
    
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}