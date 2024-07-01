/**
 * @file ebtime.h
 * @brief ap 프로그램에서 시간과 관련이 된 함수들.
 * 
 * TIME_RESOLUTION_HZ : 사용할 시간(초) 분해능
 * TICK_RESOLUTION_HZ : 사용할 tick 분해능
 * 
 * @author jspark
 * @date 2022-04-18
 * 
 * @copyright Copyright (c) 2022
 */
#pragma once
#include <sys/time.h>
#include <iostream>

#define NANO_SEC 1000000000
#define TIME_RESOLUTION_HZ 10000    //TODO: 시간 분해능 설정
#define TICK_RESOLUTION_HZ 100      //TODO: tick 분해능 설정
#define sec_t double
#define tick_t int64_t
//시간단위 해석.
#define SEC_1	TICK_RESOLUTION_HZ // 1 sec
#define MIN_1   60*SEC_1 // 1 min

sec_t get_system_time(void);
sec_t get_system_time(sec_t _start_sec);
tick_t get_system_tick(void);
tick_t get_system_tick(sec_t _start_sec);

// 시간 측정 클래스.
// 
// 사용예시)
// CStopWatch sw( E_UNIT::MILLI_SEC ); // 시간 측정 클래스 생성.
// (소스코드) // 측정할 소스코드
// sw.getTime() // 측정한 시간 리턴.
class CStopWatch
{
    enum E_UNIT
    {
        SEC, // 초
        MILLI_SEC, // 밀리초 (0.001초)
        MICRO_SEC, // 마이크로초 (0.000001초)
    };

private:
    CStopWatch::E_UNIT unit;
    double startTime;

public:
    CStopWatch();
    CStopWatch(CStopWatch::E_UNIT unit);
    ~CStopWatch();
    double getTime();
    std::string getUnit();
};