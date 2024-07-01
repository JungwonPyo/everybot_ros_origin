/**
 * @file lowPassFilter.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"

// 1차 저주파 통과 필터.
//
// 로봇의 x, y 좌표의 노이즈를 제거하기 위해 사용.
class CLowPassFilter
{
    /** @brief 저주파 필터 타입. (일반, 보간법, 보외법) */
    enum E_FILTER_TYPE
    {
        RAW,            // 저주파 필터만 적용.
        INTERPOLATION,  // 저주파 필터에 보간법 적용.
        EXTRAPOLATION,  // 저주파 필터에 보외법을 통한 예측값 적용.
    };

private:
    double alpha; // alpha가 클수록 노이즈가 적고 변화에 둔감.

    double preTime;
    double curTime;
    tPoint newPoint;
    tPoint lpfPrePoint;
    tPoint lpfPoint;
    tPoint interPrePoint;
    tPoint interPoint;
    tPoint extraPoint;

    CLowPassFilter::E_FILTER_TYPE type; // 사용할 필터 처리 방식.

    void filter();
    tPoint predictPoint(double predictTime);

public:
    CLowPassFilter();
    ~CLowPassFilter();
    tPoint getCurrentPoint();
    tPoint getLpfPoint();
    tPoint getInterPolationPoint();
    tPoint getExtraPolationPoint();
    void init(tPoint initPoint, double alpha);
    void updatePoint(tPoint point);
};