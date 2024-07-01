/**
 * @file interfaceStruct.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <list>
#include <ros/time.h>
#include <unordered_map>

#include "define.h"

struct tSysTofInfo
{
    tSysTofInfo() : deviceState{4}, rangeStatus{4}, rangeAvg{0} {}

    unsigned short deviceState; // TOF 센서의 상태 정보   정상 : 0, 초기화 안됨 : 4 , 에러 : 99 
    unsigned short rangeStatus;    
    unsigned short rangeAvg;

};

struct tTof_q8
{
    tSysTofInfo rightSide;
};

struct tTof_ts800
{
    tSysTofInfo leftSide;
    tSysTofInfo rightSide;	
    tSysTofInfo knollCenter; //hjkim230120 - W/S둔턱 용 tof센서 추가   
};

#if ROBOT_MODEL == TS800_WS
typedef tTof_ts800 tTof;
#elif ROBOT_MODEL == Q8_3I
typedef tTof_q8 tTof;
#endif

struct tSysCliff
{
    tSysTofInfo left;
    tSysTofInfo right;

    tSysTofInfo fleft_side;
    tSysTofInfo fleft_center;
    tSysTofInfo fright_center;
    tSysTofInfo fright_side;     
};

struct tCliffActionState
{
    bool bRobotStop;    // 로봇이 멈춘상태
    bool bLeftDetect;   // 왼쪽 감지
    bool bRightDetect;  // 오른쪽 감지
    bool bTimeOut;      // 후진 타임아웃 (현재는 후진 안함.)
};

typedef struct _tWheelEncoder
{
    int leftCnt;
    int rightCnt;
    int backCnt;

}tSysWheelEncoder;


typedef struct _tSysBumper
{
    _tSysBumper() : right{false}, left{false} {}

    bool right;
    bool left;

}tSysBumper;


typedef struct _tBattery
{
    _tBattery() : battVolt{-1}, temperature{-1}, percent{-1}, current{-1} {}

    short battVolt;
    short temperature;
    short percent;
    short current;

}tSysBattery;

typedef struct _tPower
{
    _tPower() : state{0}, adaptor_in{0}, aptVolt{0} {}

    unsigned char state;
    unsigned char adaptor_in;
    short aptVolt;

}tSysPower;
//dequeue<Battery> q_battery;

typedef struct _tSysRemote
{
    _tSysRemote() : remoteKey{0} {}

    unsigned char remoteKey;

}tSysRemote;

typedef struct _tSysButton
{
    _tSysButton() : buttonStart{false}, buttonHome{false}, buttonFunc{false} {}

    bool buttonStart;
    bool buttonHome;
    bool buttonFunc;

}tSysButton;

struct tSysFront
{
    tSysFront() : left{0}, center{0}, right{0} {}

    unsigned short left;
    unsigned short center;
    unsigned short right;

};

typedef struct _tSysChargerIR
{
    _tSysChargerIR() : Data{0,}, count{0}, timeStamp{0,} {}

	unsigned char Data[4096];		//*SYS_Q_LENGTH*/
    unsigned int count;
    double timeStamp[4096];

} tSysChargerIR; // tSysSignal;

typedef struct
{
    tSysChargerIR ChargerIR[4]; // SIDE 추가. //[2];

} tSysSignal;

struct tSysPose
{
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    tSysPose() : timeStamp{0}, x{0}, y{0}, angle{0}, calSn{0} {}
#else
    tSysPose() : timeStamp{0}, x{0}, y{0}, angle{0} {}
#endif
    ros::Time timeStamp;
    double x; // 단위(m)
    double y; // 단위(m)
    double angle; // 단위(radian)
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)  //RBT_PLUS 인 경우 calibration num++
    unsigned int calSn;
#endif
};

//IMU 정보.
typedef struct _tSysIMU
{
    _tSysIMU() : timeStamp{0}, state{0}, filteredState{0}, Groll{0}, Gpitch{0}, Gyaw{0}, Ax{0}, Ay{0}, Az{0} {}

    ros::Time timeStamp;

    unsigned char state;
    uint8_t filteredState;
#if 1 // system interface
    // short Groll;
    // short Gpitch;
    // short Gyaw;
    int Groll;
    int Gpitch;
    int Gyaw;
    
    short Ax;
    short Ay;
    short Az;

#else
    s32 Groll;
    s32 Gpitch;
    s32 Gyaw;

    s16 Ax;
    s16 Ay;
    s16 Az;
#endif
}tSysIMU;


//wheel 정보.
typedef struct _tSysWheelMotor_ts800
{
    _tSysWheelMotor_ts800() : actuatorStatus{0}, leftEncoder{0}, leftCurrent{0}, rightEncoder{0}, rightCurrent{0}, dummyEncoder{0}, dummyCurrent{0}, leftEncAccumCnt{0}, rightEncAccumCnt{0}, dummyEncAccumCnt{0} {}
    
    unsigned short actuatorStatus;
    
    int leftCurrent;
    int rightCurrent;
    int dummyCurrent;
    
    int leftEncoder;
    int rightEncoder;
    int dummyEncoder;
    
    int leftGain;
    int rightGain;
    int dummyGain;

    /* ceva 요청 누적 엔코더 데이터 */
    int leftEncAccumCnt;    // 왼쪽 엔코더 누적 카운트
    int rightEncAccumCnt;   // 오른쪽 엔코더 누적 카운트    
    int dummyEncAccumCnt;

    int lineVelocity;
    int angularVelocity;

}tSysWheelMotor_ts800;


typedef struct _tSysWheelMotor_q8
{
    _tSysWheelMotor_q8() : actuatorStatus{0}, leftEncoder{0}, leftCurrent{0}, rightEncoder{0}, rightCurrent{0} , leftEncAccumCnt{0}, rightEncAccumCnt{0} {}

    unsigned short actuatorStatus;
    
    int leftCurrent;
    int rightCurrent;
    
    int leftEncoder;
    int rightEncoder;
    
    int leftGain;
    int rightGain;

    /* ceva 요청 누적 엔코더 데이터 */
    int leftEncAccumCnt;    // 왼쪽 엔코더 누적 카운트
    int rightEncAccumCnt;   // 오른쪽 엔코더 누적 카운트
}tSysWheelMotor_q8;

#if ROBOT_MODEL == TS800_WS
typedef tSysWheelMotor_ts800 tSysWheelMotor;
#elif ROBOT_MODEL == Q8_3I
typedef tSysWheelMotor_q8 tSysWheelMotor;
#endif

typedef enum
{
    UNKNOW,
    TILTED_UP,      //Tilting UP 완료 상태 
    TILTED_DOWN,    //Tilting DOWN 완료 상태
    TILING_UP,      //Tilting UP 중 상태
    TILING_DOWN,    //Tilting DWON 중 상태
    TILING_STOP,    //Tilting 중 STOP인 상태
    TILT_ERROR      //Tilting 오류 있음

}E_SYS_TILT_STATE;

static std::string enumToString(E_SYS_TILT_STATE value)
{
    static const std::unordered_map<E_SYS_TILT_STATE, std::string> enumToStringMap = {
        {E_SYS_TILT_STATE::UNKNOW, "UNKNOW"},
        {E_SYS_TILT_STATE::TILTED_UP, "TILTED_UP"},
        {E_SYS_TILT_STATE::TILTED_DOWN, "TILTED_DOWN"},
        {E_SYS_TILT_STATE::TILING_UP, "TILING_UP"},
        {E_SYS_TILT_STATE::TILING_DOWN, "TILING_DOWN"},
        {E_SYS_TILT_STATE::TILING_STOP, "TILING_STOP"},
        {E_SYS_TILT_STATE::TILT_ERROR, "TILT_ERROR"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end())
    {
        return it->second;
    }
    else
    {
        return "Unknown";
    }
}




typedef enum
{
    COMMAND_TILT_STOP = 0, //틸팅 멈춤 제어
    COMMAND_TILT_UP   = 1, //틸팅 UP 제어
    COMMAND_TILT_DOWN = 2, //틸팅 DOWN 제어

}E_SYS_TILT_COMMAND;

typedef struct _tSysTilting
{
    _tSysTilting() : state{E_SYS_TILT_STATE::UNKNOW} {}

    E_SYS_TILT_STATE state;

}tSysTilting;

typedef struct _tSysControlTilt
{
    _tSysControlTilt() : cmd{E_SYS_TILT_COMMAND::COMMAND_TILT_STOP}, speed{0} {}

    E_SYS_TILT_COMMAND cmd;
    int speed;
    
    /* data */
}tSysControlTilt;

typedef struct _tSysPump
{
    _tSysPump() : isActive{false} {}

    /* data */
    bool isActive;

}tSysPump;

typedef struct _tSysOta
{
    _tSysOta() : bOpen{false},state{99}, pecent{0}{}
    
    bool bOpen; 
    int state;
    int pecent;

}tSysOta;
