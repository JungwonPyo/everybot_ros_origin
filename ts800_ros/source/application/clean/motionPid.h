/**
 * @file motionPid.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "commonStruct.h"

typedef enum
{
    INIT,
    RUN,
    STOP,
}E_PID_STATE;


typedef enum
{
    AXIS_X,
    AXIS_Y,
    AXIS_ANGLE,
    AXIS_SLIP, // 라인 횡방향 에러 PID
}E_PID_ERROR_TYPE;

class CMotionPid
{
private:

    double crosstrack_error;
    double int_crosstrack_error;
    double diff_crosstrack_error;

    double max_steering_angle;

    double kP, kI, kD;
    double lookahead;

    tPoint startPoint;
    tPoint targetPoint;
    E_PID_ERROR_TYPE errType;
    
public:
    CMotionPid();
    ~CMotionPid();
    void init(tPose robotPose, tPoint traget, E_PID_ERROR_TYPE type);
    void init(tPoint startPoint, tPoint targetPoint); // 휭 방향 Error 에러시 사용
    double run(tPose robotPose);
    void stop();

private:
    double getError(tPose robotPose, tPoint targetPoint);
    double getErrorAxisX(tPose robotPose, tPoint targetPoint);
    double getErrorAxisY(tPose robotPose, tPoint targetPoint);
    double getErrorAngle(tPose robotPose, tPoint targetPoint);
    double getErrorSlip(tPose robotPose);
    void __debug_print(tPose robotPose, tPoint targetPoint, double steer);
};
