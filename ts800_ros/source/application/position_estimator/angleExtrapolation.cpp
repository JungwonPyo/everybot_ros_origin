#include "angleExtrapolation.h"
#include "utils.h"
#include "eblog.h"

CAngleExtrapolation::CAngleExtrapolation() : maximumDataSize(30), alpha(1), deltaT(0.02)
{
}

CAngleExtrapolation::~CAngleExtrapolation()
{
}

/**
 * @brief LPF
 * 
 * @param currentValue, previousValue, alpha
 * @return alpha * currentValue + (1 - alpha) * previousValue
 */
double CAngleExtrapolation::lowPassFilter(double currentValue, double previousValue, double alpha)
{
    return alpha * currentValue + (1 - alpha) * previousValue;
}

/**
 * @brief 로봇 헤딩 각, 시간, 각속도 세팅
 * 
 * @param heading, timestamp
 */
void CAngleExtrapolation::setData(double heading, double timestamp)
{
    // 데이터 추가 (최대 30개로 제한)
    if (pastHeadings.size() >= maximumDataSize)
    {
        pastHeadings.erase(pastHeadings.begin());
        pastTimestamps.erase(pastTimestamps.begin());
        pastAngularVelocities.erase(pastAngularVelocities.begin());
    }
    pastHeadings.push_back(heading);
    pastTimestamps.push_back(timestamp);

    // 각속도 데이터 계산
    if (pastHeadings.size() >= 2 && pastTimestamps.size() >= 2)
    {
        double angularVelocity = (pastHeadings.back() - pastHeadings.at(pastHeadings.size() - 2)) / static_cast<double>((pastTimestamps.back() - pastTimestamps.at(pastTimestamps.size() - 2)));
        pastAngularVelocities.push_back(angularVelocity);
    }

    // LPF 필터링
    if (pastAngularVelocities.size() > 1)
    {
        pastAngularVelocities.back() = lowPassFilter(pastAngularVelocities.back(), pastAngularVelocities.at(pastAngularVelocities.size() - 2), alpha);
    }
}

/**
 * @brief 각속도 기반 외삽법(보외법) 결과를 예측 각도로 변환
 */
void CAngleExtrapolation::extrapolateHeading()
{
    if (!pastAngularVelocities.empty())
    {
        double extrapolatedAngularVel = pastAngularVelocities.back();
        predictedHeading = extrapolatedAngularVel * deltaT + pastHeadings.back();
    }
}

/**
 * @brief 계산된 결과를 반환
 * 
 * @return predictedHeading
 */
double CAngleExtrapolation::getPredictedHeading()
{
    return predictedHeading;
}

double CAngleExtrapolation::getPredictedHeading(double robotAngle, double deltaTime)
{
    double ret = utils::math::rad2rad(robotAngle+deltaTime*angleVelocity);
#if 0 // debug 코드
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "현재 각도: "<<BOLDWHITE<<RAD2DEG(robotAngle)<<BOLDBLACK<<
        "\t 속도: "<<BOLDWHITE<<RAD2DEG(angleVelocity)<<BOLDBLACK<<" deg/s \t 예측시간: "<<int(deltaTime*1000)<<" ms\t 예측 각도: "<<BOLDGREEN<<RAD2DEG(ret));
#endif
    return ret;
}

/**
 * @brief 각도 예측 실행 함수
 * 
 * @param heading, timestamp
 */
void CAngleExtrapolation::updateAngle(double newAngle, double newTime)
{
    pastHeadings.reserve(maximumDataSize);
    pastTimestamps.reserve(maximumDataSize);
    pastAngularVelocities.reserve(maximumDataSize);
    predictedHeading = 0.0;
    

    angleVelocity = (newTime==oldTime) ? 0.0 : utils::math::getTurnRadAngle(newAngle, oldAngle)/(newTime-oldTime);
    oldAngle = newAngle;
    oldTime = newTime;

    setData(newAngle, newTime);

    extrapolateHeading();
    // ceblog(LOG_LV_NECESSARY, BLUE, "각도 예측 완료 !! 예측 결과 : " << predictedHeading);
}