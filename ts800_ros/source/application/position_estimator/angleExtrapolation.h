#pragma once
#include <iostream>
#include <vector>

class CAngleExtrapolation
{
private:
    double oldAngle;
    double oldTime;
    double angleVelocity;

    std::vector<double> pastHeadings; // 과거 heading 데이터
    std::vector<double> pastTimestamps; // 과거 heading 데이터가 기록된 시간
    std::vector<double> pastAngularVelocities; // 과거 각속도 데이터
    double extrapolationTime; // 미래 예측 시간
    double predictedHeading; // 예측된 미래 heading 각도

    double lowPassFilter(double currentValue, double previousValue, double alpha);
    void setData(double heading, double timestamp);
    void extrapolateHeading();

public:
    // 나중에, 이 3개 파라미터는 동적으로 변경시키며 디버그 할 수 있게 따로 빼기
    int maximumDataSize;
    double alpha;
    double deltaT;

    CAngleExtrapolation();
    ~CAngleExtrapolation();
    double getPredictedHeading();
    double getPredictedHeading(double robotAngle, double deltaTime);
    void updateAngle(double heading, double timestamp);
};