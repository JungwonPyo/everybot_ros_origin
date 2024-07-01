#include "position_estimator/lowPassFilter.h"
#include "eblog.h"
#include "lowPassFilter.h"

#include <cmath>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 0.1 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief LPF 필터와 보간법, 보외법 계산
 *
 * @details
 * 1. 1차 저주파 통과 필터 계산식.
 *
 * x[k] = a * x[k-1] + ( 1- a ) * x[k]
 * a 값의 범위는 0 < a < 1
 *
 * 2. 보간법.
 * 3. 보외법 (예측값)
 *
 * @note 연산시간: 0.1ms ~ 1.7ms (멀티스레드 고려가 안되있습니다.)
 * @date 2023-08-10
 */
void CLowPassFilter::filter()
{
    CStopWatch __debug_sw;

    /* 1차 저주파 필터 */
    lpfPoint.x = lpfPrePoint.x * alpha + newPoint.x * (1.0 - alpha);
    lpfPoint.y = lpfPrePoint.y * alpha + newPoint.y * (1.0 - alpha);

    /* 보간법 */
    interPoint.x = (lpfPrePoint.x+lpfPoint.x)/2.0;
    interPoint.y = (lpfPrePoint.y+lpfPoint.y)/2.0;

    /* 보외법 */
    const double kPredictTime = 0.040; // 0.04초 (40ms) 예측
    extraPoint = predictPoint(kPredictTime);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 위치를 예측하여 보외법에 적용.
 *
 * 시간의 변화량이 적을 때는 예측을 하지않고 lpfPoint 리턴.
 *
 * @param predictTime 예측에 사용할 시간. (단위: 초)
 * @return tPoint 예측된 좌표
 *
 * @note 연산시간: 0.1ms (멀티스레드 고려가 안되있습니다.)
 * @date 2023-08-10
 */
tPoint CLowPassFilter::predictPoint(double predictTime)
{
    CStopWatch __debug_sw;

    tPoint predictPoint;
    double deltaTime = curTime - preTime;

    if ( deltaTime < 0.0001 )
    {
        predictPoint = lpfPoint;
    }
    else
    {
        double deltaX = lpfPoint.x - lpfPrePoint.x;
        double deltaY = lpfPoint.y - lpfPrePoint.y;

        predictPoint.x = lpfPoint.x + deltaX*(predictTime/deltaTime);
        predictPoint.y = lpfPoint.y + deltaY*(predictTime/deltaTime);
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return predictPoint;
}

/**
 * @brief 생성자. init()을 호출합니다.
 *
 * @note 연산시간: 0.1 ms 이하
 * @date 2023-08-10
 */
CLowPassFilter::CLowPassFilter()
{
    CStopWatch __debug_sw;

    type = E_FILTER_TYPE::EXTRAPOLATION; // 저주파 필터 타입.
    init(tPoint(0.0, 0.0), 0.7);    // alpha 는 0.7

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 소멸자.
 */
CLowPassFilter::~CLowPassFilter() {}

/**
 * @brief 초기 위치와 데이터 민감도를 입력.
 *
 * @details
 * alpha 값이 작으면, 노이즈가 많고 변화에 민감.
 * alpha 값이 크면, 노이즈가 적고 변화에 둔감.
 *
 * @param initPoint LPF 필터 초기화시 사용.
 * @param alpha 민감도 (클수록 노이즈 적음)
 *
 * @note 연산시간: 0.1 ms 이하
 * @date 2023-08-10
 */
void CLowPassFilter::init(tPoint initPoint, double alpha)
{
    CStopWatch __debug_sw;

    this->newPoint = initPoint;
    
    this->lpfPrePoint = initPoint;
    this->lpfPoint = initPoint;

    this->interPrePoint = initPoint;
    this->interPoint = initPoint;
    
    this->extraPoint = initPoint;
    
    this->preTime = get_system_time();
    this->curTime = get_system_time();

    this->alpha = alpha;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 필터처리가 된 위치 좌표를 리턴.
 */
tPoint CLowPassFilter::getCurrentPoint()
{
    switch (this->type)
    {
    case E_FILTER_TYPE::RAW:            return lpfPoint;
    case E_FILTER_TYPE::INTERPOLATION:  return interPoint;
    case E_FILTER_TYPE::EXTRAPOLATION:  return extraPoint;
    default:                            return tPoint(-1, -1);
    }
}

/**
 * @brief LPF 필터 좌표를 리턴.
 */
tPoint CLowPassFilter::getLpfPoint()            { return lpfPoint; }

/**
 * @brief LPF + 보간법 좌표를 리턴.
 */
tPoint CLowPassFilter::getInterPolationPoint()  { return interPoint; }

/**
 * @brief LPF + 보간법 + 보외법(예측값) 좌표를 리턴.
 */
tPoint CLowPassFilter::getExtraPolationPoint()  { return extraPoint; }

/**
 * @brief 최신 좌표 데이터로 업데이트 해주세요.
 *
 * @details 업데이트와 동시에 LPF 필터 및 보간법, 보외법 계산을 함.
 * @param point 업데이트할 최신 좌표. 단위:(m, m)
 *
 * @note 연산시간: 0.1ms ~ 4.9ms (멀티스레드 고려가 안되있습니다.)
 * @date 2023-08-10
 */
void CLowPassFilter::updatePoint(tPoint point)
{
    CStopWatch __debug_sw;

    lpfPrePoint = lpfPoint;
    interPrePoint = interPoint;

    preTime = curTime;
    curTime = get_system_time();

    newPoint = point;
    filter();

    TIME_CHECK_END(__debug_sw.getTime());
}