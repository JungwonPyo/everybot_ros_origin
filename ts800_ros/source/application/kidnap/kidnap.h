/**
 * @file kidnap.h
 * @author jhnoh
 * @brief 
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once


// user defined header
#include "location.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "externData/externData.h"

enum class RELOCAL_STEP
{
    RELOCAL_FIRST_STEP = 0,
    RELOCAL_SECOND_STEP,
    RELOCAL_THIRD_STEP,
    RELOCAL_FORTH_STEP,
    RELOCAL_FIFTH_STEP,
    RELOCAL_SIXTH_STEP,
    RELOCAL_SEVENTH_STEP,
    RELOCAL_END_STEP,
};

enum class E_KIDNAPROBOT_STATE
{
    ROTATING,                // 회전 중
    MOVE_LAST_POSE,      // 마지막 위치로 이동
    END,
};

// explorer PROC 상태 
enum class E_KIDNAP_STATE
{
    NONE,                    // 초기 세팅
    READY_LOCATION,          // 위치 추정 준비
    LOCATION,                // 위치 추정 중
    MOVE_LAST_POSITION,      // 마지막 위치로 이동
    ANGLE_SETTING,           // 원하는 각도를 세팅한다.
    END,
};
static std::string enumToString(E_KIDNAP_STATE value) {
    static const std::unordered_map<E_KIDNAP_STATE, std::string> enumToStringMap = {
        { E_KIDNAP_STATE::NONE, "NONE," },
        { E_KIDNAP_STATE::READY_LOCATION, "READY_LOCATION," },
        { E_KIDNAP_STATE::LOCATION, "LOCATION," },
        { E_KIDNAP_STATE::MOVE_LAST_POSITION, "MOVE_LAST_POSITION," },
        { E_KIDNAP_STATE::ANGLE_SETTING, "ANGLE_SETTING," },
        { E_KIDNAP_STATE::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


typedef struct  _tKidnapPaceData
{   
    _tKidnapPaceData() : slamX{0.0}, slamY{0.0}, slamAngle{0.0}, 
                        sysX{0.0}, sysY{0.0}, sysAngle{0.0},
                        imuRoll{0.0}, imuPitch{0.0} {}

    double slamX;
    double slamY;
    double slamAngle;

    double sysX;
    double sysY;
    double sysAngle;

    double imuRoll;
    double imuPitch;

}tKidnapPaceData;


class CKidnap
{
public:
    CKidnap(CLocation* _pLocation);
    ~CKidnap();

private:

    E_KIDNAP_STATE info;                 // kidnap 정보
    E_KIDNAPROBOT_STATE kidnapInfo;                 // kidnap 정보
    std::list<tPoint> lastPoseList;
    tPose estimatedPose;                 // RMCL을 통해 추정된 Pose
    bool bKidnapGeneration;              // 키드냅 발생 여부
    bool bSlamResume;
    bool bAngleSetting;    /* data */

    RELOCAL_STEP mStep;

    std::deque<tPose>       curSlamPoses; // slam pose
    std::deque<tPose>       curSysPoses;  // sys pose
    std::deque<tSysIMU>     curImuDatas;  // imu data
    std::deque<double>      curTimeDatas; // time data

    std::deque<tPose>       preSlamPoses; // 이전 slam pose
    std::deque<tPose>       preSysPoses;  // 이전 sys pose
    std::deque<tSysIMU>     preImuDatas;  // 이전 imu data
    std::deque<double>      preTimeDatas; // 이전 time data

    tKidnapPaceData paceData;
    tKidnapPaceData prePaceData;
    
    CLocation           *pLocation;
    
public:
    bool  autoKidnapAndEstimateRobotPose();

    void kidnapRobotProc();

    E_KIDNAPROBOT_STATE getKidnapRobotState();
    void setKidnapRobotState(E_KIDNAPROBOT_STATE set);
    //proc
    E_KIDNAPROBOT_STATE procRotating();
    E_KIDNAPROBOT_STATE procMoveLastPose();
    E_KIDNAPROBOT_STATE procEnd();

private:

    void kidnapProc();
    //proc
    E_KIDNAP_STATE procNone();
    E_KIDNAP_STATE procReadyLocation();
    E_KIDNAP_STATE procLocation();
    E_KIDNAP_STATE procMoveLastPosition();
    E_KIDNAP_STATE procAngleSetting(double degAngleGoal);

    void initKidnap();

    //set
    void setEstimatedPose(tPose pose);
    
    void updateData(double checkTime, tPose slamPose, tPose sysPose, tSysIMU imuData);
    void updateData();
    void updatePaceData();

    void updatePrePaceData();

    //get
    tPose getEstimatedPose();

    //check (bool)
    bool checkKidnap();

    // state 
    void setKidnapState(E_KIDNAP_STATE ret);
    E_KIDNAP_STATE getKidnapState();

    /* Debug용  */
    void __debug_kidnap_state_print();    
    std::string enumToString(E_KIDNAP_STATE state);
    bool bDebugStatePrint;



};

