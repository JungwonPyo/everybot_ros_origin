/**
 * @file headingPid.h
 * @author hjkim
 * @brief
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "eblog.h"

class CHeadingPid
{
private:
    double crosstrack_error;
    double int_crosstrack_error;
    double diff_crosstrack_error;
    double max_steering_angle;
    double kP, kI, kD;
    double targetAngle, currentAngle;
    double steer;
    bool bRun;

    bool isRunningControlLooper;
    std::thread thControlLooper;    
    
public:
    CHeadingPid(/* args */);
    ~CHeadingPid();
    void debug_setpid();
    void init(double _currentAngle, double _targetAngle);
    void run();
    void stop();
    double getSteer();
    void setCurrentAngle(double set);
    void setTargetAngle(double set);
    
private:
    double getError(double currentAngle, double targetAngle);
    void threadControlLooper();
};










