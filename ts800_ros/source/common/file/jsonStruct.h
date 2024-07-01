/**
 * @file jsonStruct.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>

struct tLogSetting
{
    tLogSetting() : bUse(true), bTime(true), bFileName(false), bFuncName(true), bLineNum(true) {}

    bool bUse;          // 로그 사용 유무
    bool bTime;         // 시간정보 유무
    bool bFileName;     // 파일이름 유무
    bool bFuncName;     // 함수이름 유무
    bool bLineNum;      // 라인수 유무
};

//////////////////////////////////////////////
/****** log.json 파일 구조체 ******************/
struct tLogSettingJsonData
{
    tLogSettingJsonData() : debug(tLogSetting()), file(tLogSetting()) {}

    tLogSetting debug;  // 디버그용 - print 로그 설정
    tLogSetting file;   // 파일 저장용 - 로그 설정
};
/********************************************/
//////////////////////////////////////////////


/**
 * @brief PID control 인자 p,i,d
 * 
 */
struct tPidControl
{
    tPidControl() : kP(0.0), kI(0.0), kD(0.0) {}
 
    double kP;
    double kI;
    double kD;
};

/**
 * @brief 경로계획(D*) 관련 설정값
 * 
 */
struct tDstarSetting
{
    tDstarSetting() : morphology(0), defaultSpeed(0.0), defaultTurnSpeed(0.0),
        slowSpeed(0.0), slowTurnSpeed(0.0), tiltingUpSpeed(0.0), tiltingDownSpeed(0.0),
        speedGain(0.0), bDebugState(false), bDebugWall(false), bDebugPath(false), bDebugWheel(false) {}

    int morphology;             // 모폴로지 값

    double defaultSpeed;        // 기본 주행 속도
    double defaultTurnSpeed;    // 기본 회전 속도
    double slowSpeed;           // 느린 주행 속도
    double slowTurnSpeed;       // 느린 회전 속도
    double tiltingUpSpeed;      // 틸링 업 속도
    double tiltingDownSpeed;    // 틸링 다운 속도
    double speedGain;               // p gain

    bool bDebugState;           // state 정보 print 여부
    bool bDebugWall;            // wall 정보 print 여부
    bool bDebugPath;            // path 정보 print 여부
    bool bDebugWheel;           // wheel 정보 print 여부
};

/**
 * @brief Clean 관련 설정들 구조체
 * 
 */
struct tCleanSetting
{
    tCleanSetting() : speed(0), dummyForwardSpeed(0), 
        dummyTurnSpeed(0), approachSpeed(0), slowSpeed(0), verySlowSpeed(0),
        turnSpeed(0), slowTurnSpeed(0), minSpeed(0), minTurnSpeed(0),
        tiltingUpSpeed(0.0), tiltingDownSpeed(0.0),  cradleMoveSpeed(0.0), CradleTrunSpeed(0.0),
        DummySlowSpeed(0.0), DummySlowTurnSpeed(0.0) {}

    int speed;
    int dummyForwardSpeed;
    int dummyTurnSpeed;
    int approachSpeed;

    int slowSpeed;
    int verySlowSpeed;

    int turnSpeed;
    int slowTurnSpeed;
    int minSpeed;
    int minTurnSpeed;

    double tiltingUpSpeed;      // 틸링 업 속도
    double tiltingDownSpeed;    // 틸링 다운 속도
    //up
    double cradleMoveSpeed;      // cradle move 속도
    double CradleTrunSpeed;    // cradle turn 속도
    double DummySlowSpeed;
    double DummySlowTurnSpeed;

    // clean area
    double updownMarginFromWall; // 청소영역과 벽 사이의 위아래 간격
    double leftrightMarginFromwall; // 청소영역과 벽 사이의 좌우 간격
    double spaceOfCleaningLines; // 청소라인의 간격
};

//////////////////////////////////////////////
/****** config.json 파일 구조체 ***************/
struct tSwConfigJsonData
{
    tSwConfigJsonData() : pid(tPidControl()), dstar(tDstarSetting()), clean(tCleanSetting()),
        manualMoveForwardSpeed(0), manualMoveTurnSpeed(0), avoidBackSpeed(0), avoidTurnSpeed(0), rosPublishDebug(0), adjuestHeadingSpeed(0),
        maxTimeSeconds(0.0), maxDistanceMeters(0.0), maxAngleRadians(0.0), maxDefaultTimeSeconds(0.0), maxDefaultDistanceMeters(0.0), maxDefaultAngleRadians(0.0) {}

    tPidControl pid;            // PID 설정값
    tDstarSetting dstar;        // Dstar 설정값
    tCleanSetting clean;        // Clean 설정값

    double lineInterval = 0.5;    // 라인간격
    double lineSideIntervalK = 0.1; // 사이드라인 살짝덜가게 하는 미세 파라미터
    double lineMaxDist = 2.0;     // 메인 라인 최대 거리
    double lineXdelta = 0.3;      //사선으로 내리는 거
    double mainlineGoalMargin = 0.10;
    double sidelineGoalMargin = 0.05;
    double cleanBoundPixel = 6;    

    /* 아직 카테고리를 못정함 */
    int manualMoveForwardSpeed; // 수동제어 전진속도
    int manualMoveTurnSpeed;    // 수동제어 회전속도
    int avoidBackSpeed;         // 회피시 후진속도
    int avoidTurnSpeed;         // 회피시 TURN SPEED

    int adjuestHeadingSpeed;

    double maxTimeSeconds;
    double maxDistanceMeters;
    double maxAngleRadians;

    double maxDefaultTimeSeconds;
    double maxDefaultDistanceMeters;
    double maxDefaultAngleRadians;

    int rosPublishDebug; //FOR_ROS_PBUBLISH ENABLE or DISABLE

    double motionCtrDesiredV;
    double motionCtrDesiredW;
    /***********************/

    double wallTrack_v2_decel_V;
    double wallTrack_v2_accel_V;
    double wallTrack_v2_decel_Dis;
    double wallTrack_v2_accel_decel_W;
    double wallTrack_v2_left_accel_decel_W;
    double wallTrack_v2_W_Weight;
    double wallTrack_v2_left_W_Weight;
    double wallTrack_v2_left_V_weight;
    double wallTrack_v2_slow_W;
    double wallTrack_v2_left_slow_W;
    double wallTrack_v2_rotate_accel_V;
    double wallTrack_v2_rotate_accel_W;
    double wallTrack_v2_left_rotate_accel_V;
    double wallTrack_v2_left_rotate_accel_W;
    double wallTrack_v2_left_rotate_V_max;
    double wallTrack_v2_left_rotate_W_max;
    double wallTrack_v2_oppback_time;
    double wallTrack_v2_back_time;
    double wallTrack_v2_rotate_time;
    double wallTrack_v2_oppback_accel_V;
    double wallTrack_v2_oppback_accel_W;
    double wallTrack_v2_back_V;
    double wallTrack_v2_back_W;

    double pointControl_angle_P_gain;
    double pointControl_angle_I_gain;
    double pointControl_angle_D_gain;
    double pointControl_Line_P_gain;
    double pointControl_accel_V;
    double pointControl_decel_dis;
    double pointControl_max_V;
    double pointControl_max_W;
    double pointControl_decel_V;
    double pointControl_accel_W;
    double pointControl_decel_W;
    double pointControlReverse_angle_P_gain;
    double pointControlReverse_Line_P_gain;
    double pointControlReverse_accel_V;
    double pointControlReverse_decel_V;
    double pointControlRotate_deadzone_w;
    double stopControl_decel_V;
    double stopControl_decel_W;

    bool bActivePredictAngle;
    
	double signalTrack_decel_V;
    double signalTrack_accel_V;
    double signalTrack_accel_W;
    double signalTrack_decel_W;
    double signalTrack_rotate_W;
    double signalTrack_max_V;
    double signalTrack_max_W;

    double tryDock_accel_V;
    double tryDock_accel_W;
    double tryDock_max_V;

    double noSignalTimeOut;

    double UnDockingEscapeDistance;
    double avoidWalltrackEndCheckTime;

    int fullChargePercentage;
	
	std::string robot_ip;

    //odom & lidar & mcu time latency 조정
    double lateOfodomSec;
    double lateOfLidarSec;
    int lateOfMcuMs;

    int limitAccelX;
    int limitAccelY;
    int limitAccelZ;
    int limitRoll;
    int limitPitch;
    int limitYaw;
    int limitCliff;
    int limitWall;

    int standAccelX;
    int standAccelY;
    int tilUpstandAccelY;
    int standAccelZ;
    int standRoll;
    int standPitch;

    int pwmAccel;
    int pwmDeccel;

    double shrink_area_scale_factor;
    double motionController_waypoint_dist;
    //물공급 예상 청소 시간
    int waterSupplyCleanTime;
    //물 공급 간격(ontime per cycle)
    int waterSupplyInterval;
    int ota_timeout;
    int standWheelTrap;
    int lidar_max_infi;
    int movePathTimeLimit; //sec
    int sysWheelErrorCount;
};
/********************************************/
