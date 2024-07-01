#include "ebtime.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief ap부팅후 현재 시간(초)을 얻음
 * 
 * @return sec_t (double)
 */
sec_t get_system_time()
{
    timespec tp;
    try
    {
        clock_gettime(CLOCK_MONOTONIC, &tp);
        return tp.tv_sec + (tp.tv_nsec/(NANO_SEC/TIME_RESOLUTION_HZ))*(1.0/TIME_RESOLUTION_HZ);
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
        return -1;
    }
    
}

/**
 * @brief ap부팅후 현재 시간(초)을 얻음
 * 
 * @param _start_sec 기준이 될 시간
 * @return sec_t (double)
 */
sec_t get_system_time(sec_t _start_sec)
{
    timespec tp;
    try
    {
        clock_gettime(CLOCK_MONOTONIC, &tp);
        return tp.tv_sec + (tp.tv_nsec/(NANO_SEC/TIME_RESOLUTION_HZ))*(1.0/TIME_RESOLUTION_HZ) - _start_sec;
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
        return -1;
    }
}

/**
 * @brief ap부팅후 현재 system tick을 얻음
 * tick 단위는 TICK_RESOLUTION_HZ 를 따름.
 * 현재는 10ms
 * @return tick_t (int64_t)
 */
tick_t get_system_tick(void)
{
    return (int64_t)(get_system_time()*TIME_RESOLUTION_HZ) / TICK_RESOLUTION_HZ;
}

/**
 * @brief 특정 시간 이후로 현재 system tick을 얻음
 * tick 단위는 TICK_RESOLUTION_HZ 를 따름.
 * 현재는 10ms
 * @param _start_sec 기준이 될 시간 
 * @return tick_t 
 */
tick_t get_system_tick(sec_t _start_sec)
{
    return (int64_t)((get_system_time()-_start_sec)*TIME_RESOLUTION_HZ) / TICK_RESOLUTION_HZ;
}

CStopWatch::CStopWatch()
{
    this->unit = E_UNIT::MILLI_SEC;
    startTime = get_system_time();
}

CStopWatch::CStopWatch(CStopWatch::E_UNIT unit)
{
    this->unit = unit;
    startTime = get_system_time();
}

CStopWatch::~CStopWatch() {}

double CStopWatch::getTime()
{
    switch (unit)
    {
    case CStopWatch::E_UNIT::SEC:       return get_system_time(startTime);
    case CStopWatch::E_UNIT::MILLI_SEC: return get_system_time(startTime)*1000;
    case CStopWatch::E_UNIT::MICRO_SEC: return get_system_time(startTime)*1000000;
    default:                            return -1;
    }
}

std::string CStopWatch::getUnit()
{
    switch (unit)
    {
    case CStopWatch::E_UNIT::SEC:       return "s";
    case CStopWatch::E_UNIT::MILLI_SEC: return "ms";
    case CStopWatch::E_UNIT::MICRO_SEC: return "us";
    default:                            return "??";
    }
}