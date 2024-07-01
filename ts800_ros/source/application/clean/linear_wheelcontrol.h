/**
 * @file linear_wheelcontrol.h
 * @author 담당자 미정
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"

#define LINEAR_MAX_SPEED 120 		//최대 속도.
#define LINEAR_MIN_SPEED 50
#define WHEEL_TO_WHEEL   230	// 휠과 휠사이 거리 mm

class CLinearWheelControl
{
private:
    double io_linear_xd = 0, io_linear_yd = 0; //1차 미분 저장 공간.
    double dtTerm = 0; // 미분 시간  저장 공간.
    
public:
    CLinearWheelControl(/* args */);
    ~CLinearWheelControl();
    void getFreq();
    void rsuInitLinearCtrl();
    double rsuLinearController ( tPoint currentPoint, double currentAngleRad, tPoint targetPoint, double freq );

private:    
    void rsuLineIoLinearization (double *v, double *w, double xd, double yd, double xdd, double ydd, double currentAngleRad, tPoint currentPoint);
    void rsuGetLineIoDesiredPos (tPoint currentPoint, double *xd, double *yd );    
};