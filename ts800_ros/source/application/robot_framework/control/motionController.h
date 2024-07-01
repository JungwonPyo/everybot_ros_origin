/**
 * @file motionController.h
 * @author jspark
 * @brief 모션 컨트롤러 클래스 입니다.
 * 목표점에 도달하기 위해 선속도와 각속도를
 * 적절한 프로파일에 따라 생성해줍니다.
 * 
 * @version 0.1
 * @date 2023-10-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <pthread.h>
#include <iostream>
#include <unordered_map>
#include "commonStruct.h"
#include "utils.h"
#include "define.h"
#include "coreData/serviceData.h"
#include "motionPlanner/wayPoint.h"
#include "motionPid.h"
#include <atomic>

#define MOTION CMotionController::getInstance()
#define TILT_UP_TIME_OUT 10     // 틸 업 타임아웃 10SEC
#define TILT_DOWN_TIME_OUT 7    // 틸 다운 타임아웃 10SEC
#define TILT_TIME_OUT 10    // 틸 업이나 다운 타임아웃 10SEC

enum class E_CONTROL_TYPE
{
    STOP,           // 정지
    LINEAR,         // 직선 이동
    ROTATE,         // 회전 이동
    BACK,           // 후진
    ANGULAR_PRIOR,  // 회전우선 제어
};

static std::string enumToString(E_CONTROL_TYPE value)
{
    static const std::unordered_map<E_CONTROL_TYPE, std::string> enumToStringMap = {
        { E_CONTROL_TYPE::STOP, "STOP 제어" },
        { E_CONTROL_TYPE::LINEAR, "Linear 제어" },
        { E_CONTROL_TYPE::ROTATE, "Rotate 제어" },
        { E_CONTROL_TYPE::BACK, "Back 제어" },
        { E_CONTROL_TYPE::ANGULAR_PRIOR, "ANGULAR_PRIOR 제어" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_STATE
{
    NOTHING,
    RUNNING,                    // 제어 진행중
};

static std::string enumToString(E_STATE value)
{
    static const std::unordered_map<E_STATE, std::string> enumToStringMap = {
        { E_STATE::NOTHING, "NOTHING" },
        { E_STATE::RUNNING, "RUNNING" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_ROTATE_DIR
{
    NONE,
    CW,                     // 우회전
    CCW,                    // 좌회전
};

static std::string enumToString(E_ROTATE_DIR value)
{
    static const std::unordered_map<E_ROTATE_DIR, std::string> enumToStringMap = {
        { E_ROTATE_DIR::NONE, "NONE" },
        { E_ROTATE_DIR::CW, "CW" },
        { E_ROTATE_DIR::CCW, "CCW" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_ANGULAR_PRIOR_CONTROL_STEP
{
    V_DECEL,
    ROTATE,
    MOVE_FORWARD,
};

static std::string enumToString(E_ANGULAR_PRIOR_CONTROL_STEP value)
{
    static const std::unordered_map<E_ANGULAR_PRIOR_CONTROL_STEP, std::string> enumToStringMap = {
        { E_ANGULAR_PRIOR_CONTROL_STEP::V_DECEL, "V_DECEL 스텝" },
        { E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE, "ROTATE 스텝" },
        { E_ANGULAR_PRIOR_CONTROL_STEP::MOVE_FORWARD, "MOVE_FORWARD 스텝" }, 
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CMotionController
{
public:
    struct tDrivingInfo
    {
        tProfile pf;                // motion 프로파일
    
        tPose startPose;            // 시작 좌표
        
        tPoint targetPoint;         // 움직일 목표 좌표
        tPoint previousPoint;       // 이전 로봇위치

        double movementDistance;    // 움직인 거리
        double targetDistance;      // 움직일 목표 거리
        double targetAngle;         // 움직일 목표 각도
        double previousAngle;       // 이전 로봇각도

        bool bStopEmergency;        // 긴급정지 플래그
    };
    
    struct tPwmInfo
    {
        int direction;
        int lspeed;
        int rspeed;
        int bspeed;
        int duty;
        
        bool pid;
    };

private:
    bool testMutexFlag;                     //2024.04.01 안정화 전까지 나둠. 기간 2주
    bool bThMotion;
    bool bObstacleSpeedDownFlag;            // 장애물 감지시 감속 flag
    bool pwmControl;                        // pwm제어 flag
    bool isRobotCrossEndLine;               // 로봇의 목표지점 넘어감 flag

    int msgSeq;
    int stopTime;                           // [정지 제어 후 대기] 정지시간 (단위 ms)
    int waitingTime;                        // [정지 제어 후 대기] 대기할 시간 (단위 ms)
    int bufferSize          = 10;

    double desiredV;                        // desired 선속도 (m/s)
    double desiredW;                        // desired 각속도 (rad/s)
    double curLinVel;                       // 현재 제어 선속도 (m/s)
    double curAngVel;                       // 현재 제어 각속도 (rad/s)
    double previousAngle;                   // 이전 각도
    double targetAngle;                     // 움직일 각도
    double tempTargetAngle;                 // 제어기 내에서 사용하는 타겟 각도
    double angleError;                      // 각도 오차
    double lineDisErr;                      // 거리 오차
    double nomalizedLineDisErr;             // 거리 오차 정규화 (m->rad)
    double relativePosition;                // 로봇 위치 부호 결정 변수
    double disRobot2Goal;                   // 로봇과 목표점 사이의 거리
    double debugCnt;
    double encCnt;
    double angleIError      = 0.0;          // 현재 각도 누적 오차
    double prevAangleError  = 0.0;          // 이전 각도 오차

    std::deque<double> errorBuffer;

    pthread_t thMotionHandler;
    pthread_mutex_t  mutexPointControl;    // motion point 보호용
    tDrivingInfo DrvInfo;
    tPwmInfo pwmInfo;   // [휠 pwm 제어]
    tPoint targetPoint; // 타겟 포인트 좌표
    tTwist combineTwist; // 합성된 최종 속도

    E_STATE state;
    E_ROTATE_DIR rotateDir;
    E_CONTROL_TYPE controlType;
    E_ANGULAR_PRIOR_CONTROL_STEP currentStep;

    std::list<tPoint> pointLists;
    std::list<double> lineCoeff;
    bool isErrorEncoder;

public:
    CMotionController();
    ~CMotionController();
    static CMotionController& getInstance();

    E_CONTROL_TYPE getControlType();
    E_ANGULAR_PRIOR_CONTROL_STEP getAngularPriorControlType();

    void init();

    void startStopOnMap(tProfile prof, bool emergency);
    void startLinearToPointOnMap(const tPose &robotPose, tPoint targetPoint, tProfile prof);
    void startLinearAngularPriorToPointOnMap(const tPose &robotPose, tPoint targetPoint, tProfile prof);
    void startBackToPointOnMap(const tPose &robotPose, tPoint targetPoint, tProfile prof);
    void startRotation(const tPose& robotPose, double targetAngle, tProfile prof, E_ROTATE_DIR dir = E_ROTATE_DIR::NONE);
    void startDriveWheelPwm(const tPose& robotPose, int direction, int lspeed, int rspeed, int bspeed, int duty, bool pid, tProfile prof);

    void proc();
    void procDirectPwm();

    void sendMessageDrvie(double linearVelocity, double angularVelocity);
    void sendMessageDriveOld(int direction, int lspeed, int rspeed, int bspeed, int duty, bool pid);	
    void actionStart(tAction action);
    void stop(int sleepMs);

    bool isErrorEncoderStop() {return isErrorEncoder; }
    bool isRunning();
    bool isRotate();
    bool isPwmTargetDistanceArrived(tPose robotPose);
    bool isNearTargetPose(const tPose &robotPose, tPoint target, double distanceMargin);
    bool isNearTargetRad(const tPose& robotPose, double targetRad, double marginRad);
    bool isOverTargetPoint(const tPose &robotPose, tPose startPose);
    bool isOverTargetPoint(const tPose &robotPose, tPoint startPoint,tPoint targetPoint);

    tTwist getControlVelocity();

private:
    static void* threadMotionWrap(void* arg)
    {
        CMotionController* myMotionCtrl = static_cast<CMotionController*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_ROS_CALLBACK);
        myMotionCtrl->threadMotion();
    }
    
    E_STATE getState();

    void threadMotion();

    void setState(E_STATE state);
    void setPointLists(tPose startPose, tPoint targetPoint, double point2pointDist);
    void updatePointLists(tPose robotPose, double point2robotDist);
    
    void procStop();
    void procLinearToPointOnMap(const tPose& robotPose);
    void procLinearToPointOnMapCheckObs(const tPose &robotPose);
    void procLinearAngularPriorToPointOnMap(const tPose &robotPose);
    void procBackToPointOnMap(const tPose &robotPose);
    void procRotation(const tPose &robotPose);
    void initControlValues();
    void setControlValues(tPose robotPose);
    void setRotateControlValues(tPose robotPose);
    void setNormalControlValues(tPose robotPose);
    void clearVelocity();
    void debugPrintInfo(tPose robotPose, double curVelv, double curVelw);

    bool isEncoderStop();

    double getLinearDesiredVel(double curVelW);
    double getAngularDesiredVel();
    double getNormalAngularDesiredVel();
    double getAngularCmdVel(double curVelW, double desiredW);
    double getLinearCmdVel(double curVelV, double desiredV);
    double getBackCmdVel(double curVelV, double desiredV);
    double getStopCmdVel(double curVel, bool linear);
    double getFrontLidarData();
    double getBackLidarData();
    double normalLineFunction(double x, double y);
    double applyRotateDeadzone(double curVelw, double angleError);

    tTwist getCalculatedStopTwist();
    tTwist getCalculatedEmergencyStopTwist();
    tTwist getCalculatedTwist(tPose robotPose);
    tTwist getCalculatedTwistCheckObs(tPose robotPose);
    tTwist getCalculatedAngularPriorTwist(tPose robotPose);
    tTwist getCalculatedReverseTwist(tPose robotPose);
    tTwist getCalculatedReverseTwistCheckObs(tPose robotPose);
    tTwist getCalculatedRotateTwist(tPose robotPose);
    tTwist getCalculatedCWRotateTwist(tPose robotPose);
    tTwist getCalculatedCCWRotateTwist(tPose robotPose);

    tPwmSpeed getCalculatedPwmSpeed();
};
