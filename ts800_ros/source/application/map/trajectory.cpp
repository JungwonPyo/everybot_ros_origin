#include "trajectory.h"
#include "eblog.h"
#include "systemTool.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CTrajectory::CTrajectory()
{
    CStopWatch __debug_sw;
    
    bIsUpdate = false;
    pthread_mutex_init(&mutex, nullptr);
    eblog(LOG_LV,   "create");
    
    TIME_CHECK_END(__debug_sw.getTime());
}
CTrajectory::~CTrajectory()
{
    CStopWatch __debug_sw;

    pthread_mutex_destroy(&mutex);
    
    eblog(LOG_LV,   "destroy");    
    TIME_CHECK_END(__debug_sw.getTime());    
}

bool CTrajectory::isUpdate(){
    return bIsUpdate;
}

void CTrajectory::setUpdateState(bool set)
{
    CStopWatch __debug_sw;
    
    bIsUpdate = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief trajectory를 복사한다.
 * 
 * @return std::list<tPoint> 
 */
std::list<tPoint> CTrajectory::get(void)
{
    CStopWatch __debug_sw;
        
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);    
    
    std::list<tPoint>temp = trajectory;
    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");
    
    TIME_CHECK_END(__debug_sw.getTime());
    return temp;
}

/**
 * @brief trajectory를 업데이트 한다.
 * 
 * @param set 
 */
void CTrajectory::set(std::list<tPoint> set)
{
    CStopWatch __debug_sw;
    
    
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);
    
    
    trajectory.clear();//set 하기 전에 모두 클리어 한다. ( 메모리 증가 방지)
    for (tPoint path : set)
    {			
        trajectory.emplace_back(path);
    }
    bIsUpdate = true;
    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief trajectory를 지운다.
 * 
 */
void CTrajectory::clear(void)
{
    CStopWatch __debug_sw;
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);    
    
    trajectory.clear();
    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");
    
    TIME_CHECK_END(__debug_sw.getTime());
}