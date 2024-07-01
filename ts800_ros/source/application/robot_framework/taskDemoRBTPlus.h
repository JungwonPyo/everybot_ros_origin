/**
 * @file explorer.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

// C++ system header
#include <cstdint>
#include <utility>
#include <future>
#include <ctime>
//#include <time.h>
#include <iostream>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"
//#include "avoiding.h"
//#include "walltracking.h"
#include "commonStruct.h"
#include "supplyWater.h"

#include "motionPlanner/wayPointManager.h"

enum class DEMORBTPLUS_STATE
{
    NONE,
    WAIT,
    START_WATER_SUPPLY,

    //imu 특성 고려해서 가장 간단한 패턴
    //단 2시간 정도 하면 30도 정도 오차가 날 가능성 있음
    DRIVE_TEST_WAIT,
    DRIVE_RESTART,
    DRIVE_GO_2_0, // 2,0
    DRIVE_LEFT1_90,
    DRIVE_GO_2_0_5, //2,0.5
    DRIVE_LEFT2_90,
    DRIVE_GO_0_0_5, //0,0.5
    DRIVE_RIGHT1_90,
    DRIVE_GO_0_1_0, //0, 1.0
    DRIVE_RIGH2_90,
    DRIVE_GO_2_1, // 2,1
    DRIVE_LEFT3_90,
    DRIVE_GO_2_1_5, // 2, 1.5
    DRIVE_LEFT4_90,
    DRIVE_GO_0_1_5, //0,1.5
    DRIVE_RIGHT3_90, //
    DRIVE_GO_0_0, //0,0
    //주행 성능 테스트 패튼 주행 "ㄹ"
    WAIT_DRIVE_S,
    DRIVE_S,
    START_S_MOVE_TO_POINT,
    RUN_S_MOVE_TO_POINT,
};

static std::string enumToString(DEMORBTPLUS_STATE value) {
    static const std::unordered_map<DEMORBTPLUS_STATE, std::string> enumToStringMap = {
        { DEMORBTPLUS_STATE::NONE, "NONE," },
        { DEMORBTPLUS_STATE::START_WATER_SUPPLY, "START_WATER_SUPPLY," },
        { DEMORBTPLUS_STATE::WAIT, "WAIT," },
        ////IMU 특성을 고려한 패턴
        { DEMORBTPLUS_STATE::DRIVE_TEST_WAIT, "DRIVE_TEST_WAIT," },
        { DEMORBTPLUS_STATE::DRIVE_RESTART, "DRIVE_RESTART," },
        { DEMORBTPLUS_STATE::DRIVE_GO_2_0, "DRIVE_GO_2_0," }, 
        { DEMORBTPLUS_STATE::DRIVE_LEFT1_90, "DRIVE_LEFT1_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_2_0_5, "DRIVE_GO_2_0_5," }, 
        { DEMORBTPLUS_STATE::DRIVE_LEFT2_90, "DRIVE_LEFT2_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_0_0_5, "DRIVE_GO_0_0_5," }, 
        { DEMORBTPLUS_STATE::DRIVE_RIGHT1_90, "DRIVE_RIGHT1_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_0_1_0, "DRIVE_GO_0_1_0," }, 
        { DEMORBTPLUS_STATE::DRIVE_RIGH2_90, "DRIVE_RIGH2_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_2_1, "DRIVE_GO_2_1," }, 
        { DEMORBTPLUS_STATE::DRIVE_LEFT3_90, "DRIVE_LEFT3_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_2_1_5, "DRIVE_GO_2_1_5," }, 
        { DEMORBTPLUS_STATE::DRIVE_LEFT4_90, "DRIVE_LEFT4_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_0_1_5, "DRIVE_GO_0_1_5," },
        { DEMORBTPLUS_STATE::DRIVE_RIGHT3_90, "DRIVE_RIGHT3_90," }, 
        { DEMORBTPLUS_STATE::DRIVE_GO_0_0, "DRIVE_GO_0_0," }, 

        //주행 성능 측정 : 속도 등 (신뢰성 요구 사항)
        { DEMORBTPLUS_STATE::WAIT_DRIVE_S, "WAIT_DRIVE_S," },
        { DEMORBTPLUS_STATE::DRIVE_S, "DRIVE_S," },
        { DEMORBTPLUS_STATE::START_S_MOVE_TO_POINT, "START_S_MOVE_TO_POINT," },
        { DEMORBTPLUS_STATE::RUN_S_MOVE_TO_POINT, "RUN_S_MOVE_TO_POINT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskDemoRBTPlus
{
private: /* not use! */
    CTaskDemoRBTPlus(const CTaskDemoRBTPlus& other) = default; // Copy constructor
    CTaskDemoRBTPlus& operator=(const CTaskDemoRBTPlus& other) = default; // Assignment operator
public:
    CTaskDemoRBTPlus();
    ~CTaskDemoRBTPlus();
    //CAvoiding avoiding;
    
    // proc
    bool taskDemoRBTPlusRun(tPose robotPose, tPose slamPose, tPose cevaPose, double batteryVolt, RSU_OBSTACLE_DATA *pObstacle);
    void setDemoRBTPlusState(DEMORBTPLUS_STATE set);
    bool testKeyChecker(tPose robotPose);

    void debugPrint(u16 time, tPose slamPose, tPose cevaPose, tPose robotPose, double batteryVolt, RSU_OBSTACLE_DATA *pObstacle);
    void checkLowBattery(u16 time, tPose robotPose, double batVolt );
    void runWaterPump(bool enable );

    int processShutDownProcessor(bool isKillTS800);
    void funcPowerOff();
    void funcSlamOnOFF(bool on);
    void funcDriveTestInit();
    void funcDriveTestExit();
    void funcDriveTestRun(bool squareType);

    std::list<tPoint> setDriverTestSquarePattern();
    std::list<tPoint> setDriverTestSquarePattern2();
    std::list<tPoint> setDRiverTestPattern();

    timespec getCurrentTime();

    //무한 "ㅁ 패턴 - 주행 시간 측정 및 RBT PLUS YAW 측정"
    std::list<tPoint> patternList1; //오른쪽 방향
    std::list<tPoint> patternList2; //왼쪽 방향

    //주행 속도 측정 패턴(신뢰성 요구 사항)
    std::list<tPoint> patternList3;

    DEMORBTPLUS_STATE demoPlusState;
    tPose startPose;

    tPoint targetPoint1; //오른쪽 반향
    tPoint targetPoint2;//왼쪽 방향
    tPoint targetPoint3;//ㄹ 3회 운동

    tPoint originUpTarget;
    tPoint originDownTarget;
private:
    CSupplyWater waterSupply;
    double waterSupplyStartTime;
    struct timespec         currTime;
    tCheckTime checkTime;
    tCheckTime checkBattTime;
    
    u32 movingErrorTime;

    int currCalSN;
    int preCalSN;
    tPoint targetEnd;
    tPoint originTarget;
    bool alert_lowbatt;
    int lowBatteryLevel;
    bool bRunPump;

    tPoint startPoint1;
    tPoint startPoint2;
    tPoint startPoint3;

    tPoint startPointUp;
    tPoint startPointDown;
    double motionTargetRad0;
    double motionTargetRad;
    double motionTargetRad1;

    double        seconds;
    double        nanoseconds;
    double        elapsed;

    tPoint moveTarget0;

    tPoint moveTarget1;
    tPoint moveTarget2;
    tPoint startPointMOVE;
 
    tPoint move1Target;
    tPoint move2Target;

    bool slamOn;
    //소요 시간 측정
    struct timespec driveStartTime;
    struct timespec driveEndTime;
    bool open_;
    bool isSave;
};