#include "rosParameter.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 0.1 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRosParameter::CRosParameter() { parameter = tParameter(); }
CRosParameter::CRosParameter(const CRosParameter &ref) { parameter = tParameter(); }
CRosParameter &CRosParameter::operator=(const CRosParameter &ref)
{
    if (this != &ref) { parameter = ref.parameter; }
    return *this;
}
CRosParameter::~CRosParameter() {}

/**
 * @brief 파라매터들을 항상 getParameter 하여 사용.
 * 
 * @return tParameter 
 */
tParameter CRosParameter::getParameter()
{
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return parameter;
}

/**
 * @brief rqt 콜백함수에서 사용할 파라매터 값 변경 함수.
 * 
 * @param inputParameter 
 */
void CRosParameter::setParameter(tParameter inputParameter)
{
    CStopWatch __debug_sw;
    
    parameter = inputParameter;
    
    TIME_CHECK_END(__debug_sw.getTime());
}