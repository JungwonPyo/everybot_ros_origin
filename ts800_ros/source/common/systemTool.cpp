#include "systemTool.h"
#include <limits>
#include "utils.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0           // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)                                     \
    {                                                            \
        printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time); \
    }                                                            \
    0
/******************************************************/

CSystemTool::CSystemTool() { init(); }
CSystemTool::CSystemTool(const CSystemTool &ref) { init(); }
CSystemTool::~CSystemTool() {}

void CSystemTool::init()
{
    init_sec = get_system_time();
}

CSystemTool &CSystemTool::getInstance()
{
    static CSystemTool s;
    return s;
}

int CSystemTool::getSystemTick()
{
    return get_system_tick(init_sec);
}

double CSystemTool::getSystemTime()
{
    return get_system_time(init_sec);
}

/**
 * @brief 시간 체크하는 함수!
 * 사용할 곳에서 전역 변수 double(startTime)을 생성 초기화 값은 std::numeric_limits<double>::infinity() 또는 std::numeric_limits<double>::max()
 * checkTime에 경과 시간을 입력
 * startTime으로부터 checkTime만큼이 경과되지 않으면 false.
 * 경과 되면 true를 반환.
 * 
 * @param startTime 
 * @param checkTime 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2023-12-20
 * @author hhryu
 */
bool CSystemTool::checkTime(double &startTime, double checkTime)
{
    bool bRet = false;

    if (utils::isMaxValue(startTime) || std::isinf(startTime))
    {
        bRet = false;
        startTime = getSystemTime();
        ceblog(LOG_LV_TILTING, BLUE, "check time start");
    }
    else
    {
        if (getSystemTime() - startTime < checkTime)
        {
            bRet = false;
            ceblog(LOG_LV_TILTING, BLUE, "time checking 시작시간 : " << startTime << "\t체크 할 시간 : " << checkTime << "\t경과된 시간 : " << (getSystemTime() - startTime));
        }
        else
        {
            ceblog(LOG_LV_TILTING, BLUE, "check time end");
            bRet = true;
            startTime = std::numeric_limits<double>::infinity();
            // startTime = getSystemTime();
        }
    }

    return bRet;
}