/**
 * @file rosParameter.h
 * @author jspark
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "eblog.h"

#define ROS_CONFIG              CRosParameter::getInstance().getParameter()
#define SET_ROS_CONFIG(param)   CRosParameter::getInstance().setParameter(param)

struct tPID
{
    double kP;
    double kI;
    double kD;
};

struct tStanleyParameter
{
    double psi;
    double cross1;
    double cross2;
};

struct tSimplifyGridmapTuningInfo
{
    int openingKernalSize;
    int wallThickness;
};

/**
 * @brief 모든 ros 파라매터들 모음.
 */
struct tParameter
{
    tStanleyParameter stanleyParam;
    tPID motionControl;
    tPID angleControl;
    double max_steering_angle;
    double speed_fwd;
    double speed_dummy;
    double speed_t2;
    double speed_t3;
    tSimplifyGridmapTuningInfo simplifyGridmap;

    double wallTrack_P_gain;
    double wallTrack_I_gain;

    double lookahead;
    double cleanLineInterval;

    double wallFollowLpfAlpha;
};

class CRosParameter
{
private:
    CRosParameter();
    CRosParameter(const CRosParameter& ref);
    CRosParameter &operator=(const CRosParameter &ref);
    ~CRosParameter();

    tParameter parameter; // rqt 로 조정 가능한 parameter 들
public:

    static CRosParameter& getInstance() {
        static CRosParameter s;
        return s;
    }
    tParameter getParameter();
    void setParameter(tParameter inputParameter);
};