/**
 * @file signaltracking.h
 * @author hhryu
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "ebtypedef.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "coreData/serviceData/signals.h"
#include "robotSlamPose.h"
#include "avoiding.h"
#include "utils.h"

#define TRY_DOCK_FORWARD_ANGLE 15/*12*/                      // Trydock - 로봇과 크래들의 각도 판단 후, 몇 도가 벗어나야 직진하지 않을지 설정하는 define.
#define TRY_DOCK_MAX_TURN_ANGLE (TRY_DOCK_FORWARD_ANGLE * 4) // Trydock - 로봇과 크래들의 각도 판단 후, 몇 도가 벗어나야 보정제어 하지않을지 정한다.
#define TRY_DOCK_FORWARD_SPEED 75                            // Trydock - 정렬되어있음으로 판단할 때 직진 speed
#define TRY_DOCK_FORWARD_DECEL_SPEED 25                      // Trydock - 직진 중 감속
#define TRY_DOCK_LEFT_MIN_SPEED 50                           // Trydock - TRY_DOCK_FORWARD_ANGLE의 각도만큼 틀어져있을 때, 왼쪽바퀴속력
#define TRY_DOCK_LEFT_MAX_SPEED 70                           // Trydock - TRY_DOCK_MAX_TURN_ANGLE에서의 속력
#define TRY_DOCK_RIGHT_MIN_SPEED 50                          // Trydock - TRY_DOCK_FORWARD_ANGLE의 각도만큼 틀어져있을 때, 왼쪽바퀴속력
#define TRY_DOCK_RIGHT_MAX_SPEED 70                          // Trydock - TRY_DOCK_MAX_TURN_ANGLE에서의 속력
#define TRY_DOCK_LEFT_MIN_BACK_SPEED (-15)                   // Trydock - TRY_DOCK_FORWARD_ANGLE의 각도만큼 틀어져있을 때, 뒤로가는 왼쪽바퀴속력
#define TRY_DOCK_LEFT_MAX_BACK_SPEED (-20)                   // Trydock - TRY_DOCK_MAX_TURN_ANGLE에서의 뒤로가는 속력
#define TRY_DOCK_RIGHT_MIN_BACK_SPEED (-10)                  // Trydock - TRY_DOCK_FORWARD_ANGLE의 각도만큼 틀어져있을 때, 뒤로가는 왼쪽바퀴속력
#define TRY_DOCK_RIGHT_MAX_BACK_SPEED (-20)                  // Trydock - TRY_DOCK_MAX_TURN_ANGLE에서의 뒤로가는 속력
#define STOP_SPEED 0
#define SWIMMING_SCAN_CHECK_CNT 4
#define TEST_MODE_CONTROL_BLOCK 0//1 // 활성화 시 제어코드를 비활성화 함. 테스트 용

/**
 * @brief   로봇이 충전 스테이션에 도킹하는 상태
 * @author  hhryu
 * @date    2022/10/25
 */
enum class E_DOCKING_STATE
{
    NONE,                   // VOID, 기본 도킹을 하고 있지 않는 상태.
    INIT,                   // 초기화
    MOVE_POINT,           // 충전기로 이동한다...
    FIND_CHARGER,            // 찾는다 충전기!
    // MOVE_CENTER_FROM_SIDE, // 로봇이 사이드에서 트랙킹을 위해 중앙으로 가는 중.
    SWIMMING,
    MOVING_CENTER,
    TRY_DOCK,
    TRACK_SIGNAL,          // 신호를 발견하여 충전기로 이동 // hhryu230125 : 이상태가 아니면 E_SIGNALTRACK_STEP을 강제 VIOD???
    NO_SIGNAL,              // 시그널 찾기 실패, FAIL....
    DOCKED,                // 로봇이 충전기에 붙은 것으로 판단... --> CHARGER로 이관
};

static std::string enumToString(E_DOCKING_STATE value) {
    static const std::unordered_map<E_DOCKING_STATE, std::string> enumToStringMap = {
        { E_DOCKING_STATE::NONE, "NONE" },
        { E_DOCKING_STATE::INIT, "INIT" },
        { E_DOCKING_STATE::MOVE_POINT, "MOVE_POINT" },
        { E_DOCKING_STATE::FIND_CHARGER, "FIND_CHARGER" },
        // { E_DOCKING_STATE::MOVE_CENTER_FROM_SIDE, "MOVE_CENTER_FROM_SIDE" },
        { E_DOCKING_STATE::SWIMMING, "SWIMMING" },
        { E_DOCKING_STATE::MOVING_CENTER, "MOVING_CENTER" },
        { E_DOCKING_STATE::TRY_DOCK, "TRY_DOCK" },
        { E_DOCKING_STATE::TRACK_SIGNAL, "TRACK_SIGNAL" },
        { E_DOCKING_STATE::NO_SIGNAL, "NO_SIGNAL" },
        { E_DOCKING_STATE::DOCKED, "DOCKED" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_AREA
{
    NONE,
    LEFT,
    RIGHT,
    CENTER,
    UNKNOWN,
};

static std::string enumToString(E_AREA value) {
    static const std::unordered_map<E_AREA, std::string> enumToStringMap = {
        { E_AREA::NONE, "NONE" },
        { E_AREA::LEFT, "LEFT" },
        { E_AREA::RIGHT, "RIGHT" },
        { E_AREA::CENTER, "CENTER" },
        { E_AREA::UNKNOWN, "UNKNOWN" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_LOCATION_FROM_CHARGER
{
    NONE,
    IDLE,

    CENTER,
    LEFT_CENTER,
    RIGHT_CENTER,
    LEFT,
    RIGHT,
    LEFT_SIDE,
    RIGHT_SIDE,

    CENTER_LONG,
    LEFT_CENTER_LONG,
    RIGHT_CENTER_LONG,
    LEFT_LONG,
    RIGHT_LONG,
    LEFT_SIDE_LONG,
    RIGHT_SIDE_LONG,

    ERROR,
};
static std::string enumToString(E_LOCATION_FROM_CHARGER value) {
    static const std::unordered_map<E_LOCATION_FROM_CHARGER, std::string> enumToStringMap = {
        { E_LOCATION_FROM_CHARGER::NONE, "NONE" },
        { E_LOCATION_FROM_CHARGER::IDLE, "IDLE" },
        { E_LOCATION_FROM_CHARGER::CENTER, "CENTER" },
        { E_LOCATION_FROM_CHARGER::CENTER_LONG, "CENTER_LONG" },
        { E_LOCATION_FROM_CHARGER::LEFT_CENTER, "LEFT_CENTER" },
        { E_LOCATION_FROM_CHARGER::RIGHT_CENTER, "RIGHT_CENTER" },
        { E_LOCATION_FROM_CHARGER::LEFT, "LEFT" },
        { E_LOCATION_FROM_CHARGER::RIGHT, "RIGHT" },
        { E_LOCATION_FROM_CHARGER::LEFT_SIDE, "LEFT_SIDE" },
        { E_LOCATION_FROM_CHARGER::RIGHT_SIDE, "RIGHT_SIDE" },

        { E_LOCATION_FROM_CHARGER::CENTER_LONG, "CENTER_LONG" },
        { E_LOCATION_FROM_CHARGER::LEFT_CENTER_LONG, "LEFT_CENTER_LONG" },
        { E_LOCATION_FROM_CHARGER::RIGHT_CENTER_LONG, "RIGHT_CENTER_LONG" },
        { E_LOCATION_FROM_CHARGER::LEFT_LONG, "LEFT_LONG" },
        { E_LOCATION_FROM_CHARGER::RIGHT_LONG, "RIGHT_LONG" },
        { E_LOCATION_FROM_CHARGER::LEFT_SIDE_LONG, "LEFT_SIDE_LONG" },
        { E_LOCATION_FROM_CHARGER::RIGHT_SIDE_LONG, "RIGHT_SIDE_LONG" },

        { E_LOCATION_FROM_CHARGER::ERROR, "ERROR" }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


// typedef enum
enum class E_SIGNALTRACK_STEP
{
    VOID,
    INIT,
    MOVING,
    SWIMMING,
    TRYDOCK,
    COMPLETE,
    FAIL, 
};
static std::string enumToString(E_SIGNALTRACK_STEP value) {
    static const std::unordered_map<E_SIGNALTRACK_STEP, std::string> enumToStringMap = {
        { E_SIGNALTRACK_STEP::VOID, "VOID," },
        { E_SIGNALTRACK_STEP::INIT, "INIT," },
        { E_SIGNALTRACK_STEP::MOVING, "MOVING," },
        { E_SIGNALTRACK_STEP::SWIMMING, "SWIMMING," },
        { E_SIGNALTRACK_STEP::TRYDOCK, "TRYDOCK," },
        { E_SIGNALTRACK_STEP::COMPLETE, "COMPLETE," },
        { E_SIGNALTRACK_STEP::FAIL, "FAIL," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_MOVING_CENTER_STEP
{
    VOID,
    TURNING_TO_CENTER,
    FORWARD_TO_CENTER,
    TURNING_TO_CHARGER,
    FORWARD_TO_CHARGER,
    SCANNING, // --> FORWARD_TO_CENTER
    FAIL,
    ESCAPE_TO_CHARGER,
    COMPLETE, // 완료는 곧 try dock.
};
static std::string enumToString(E_MOVING_CENTER_STEP value) {
    static const std::unordered_map<E_MOVING_CENTER_STEP, std::string> enumToStringMap = {
        { E_MOVING_CENTER_STEP::VOID, "VOID" },
        { E_MOVING_CENTER_STEP::TURNING_TO_CENTER, "TURNING_TO_CENTER" },
        { E_MOVING_CENTER_STEP::FORWARD_TO_CENTER, "FORWARD_TO_CENTER" },
        { E_MOVING_CENTER_STEP::TURNING_TO_CHARGER, "TURNING_TO_CHARGER" },
        { E_MOVING_CENTER_STEP::FORWARD_TO_CHARGER, "FORWARD_TO_CHARGER" },
        { E_MOVING_CENTER_STEP::SCANNING, "SCANNING" },
        { E_MOVING_CENTER_STEP::FAIL, "FAIL" },
        { E_MOVING_CENTER_STEP::COMPLETE, "COMPLETE" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_SCAN_CONTROL_STEP
{
    VOID,
    NORMAL_ROTATION,
    REVERSE_ROTATION,
    COME_BACK_ROTATION,
    FORWARD,
    COMPLELT,
};
static std::string enumToString(E_SCAN_CONTROL_STEP value) {
    static const std::unordered_map<E_SCAN_CONTROL_STEP, std::string> enumToStringMap = {
        { E_SCAN_CONTROL_STEP::VOID, "VOID" },
        { E_SCAN_CONTROL_STEP::NORMAL_ROTATION, "NORMAL_ROTATION" },
        { E_SCAN_CONTROL_STEP::REVERSE_ROTATION, "REVERSE_ROTATION" },
        { E_SCAN_CONTROL_STEP::COME_BACK_ROTATION, "COME_BACK_ROTATION" },
        { E_SCAN_CONTROL_STEP::FORWARD, "FORWARD" },
        { E_SCAN_CONTROL_STEP::COMPLELT, "COMPLELT" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_SWIMMING_CONTROL_STEP
{
    VOID,
    CHECK_SIGNAL,
    TURNING_TO_LOST_SIDE,
    TURNING_TO_REACQUISITION,
    FORWARD_TO_SHORT_AREA,
    CHANGE_TRY_DOCK,
    CHANGE_MOVING_CENTER,
    COMPLETE,
    FAIL,
};
static std::string enumToString(E_SWIMMING_CONTROL_STEP value) {
    static const std::unordered_map<E_SWIMMING_CONTROL_STEP, std::string> enumToStringMap = {
        { E_SWIMMING_CONTROL_STEP::VOID, "VOID" },
        { E_SWIMMING_CONTROL_STEP::CHECK_SIGNAL, "CHECK_SIGNAL" },
        { E_SWIMMING_CONTROL_STEP::TURNING_TO_LOST_SIDE, "TURNING_TO_LOST_SIDE" },
        { E_SWIMMING_CONTROL_STEP::TURNING_TO_REACQUISITION, "TURNING_TO_REACQUISITION" },
        { E_SWIMMING_CONTROL_STEP::FORWARD_TO_SHORT_AREA, "FORWARD_TO_SHORT_AREA" },
        { E_SWIMMING_CONTROL_STEP::CHANGE_TRY_DOCK, "CHANGE_TRY_DOCK" },
        { E_SWIMMING_CONTROL_STEP::CHANGE_MOVING_CENTER, "CHANGE_MOVING_CENTER" },
        { E_SWIMMING_CONTROL_STEP::COMPLETE, "COMPLETE" },
        { E_SWIMMING_CONTROL_STEP::FAIL, "FAIL" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_TRYDOCK_STEP
{
    VOID,
    LINE_UP,
    CHECK_SLAM,
    CHECK_TILT,
    GO_NEAR,
    ENTRANCE,
    // MISPLACE
};
static std::string enumToString(E_TRYDOCK_STEP value) {
    static const std::unordered_map<E_TRYDOCK_STEP, std::string> enumToStringMap = {
        { E_TRYDOCK_STEP::VOID, "VOID" },
        { E_TRYDOCK_STEP::LINE_UP, "LINE_UP" },
        { E_TRYDOCK_STEP::CHECK_SLAM, "CHECK_SLAM" },
        { E_TRYDOCK_STEP::CHECK_TILT, "CHECK_TILT" },
        { E_TRYDOCK_STEP::GO_NEAR, "GO_NEAR" },
        { E_TRYDOCK_STEP::ENTRANCE, "ENTRANCE" },
        // { E_TRYDOCK_STEP::MISPLACE, "MISPLACE" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_MOVE_POINT_STEP
{
    IDLE,
    CHECK_SIGNAL,
    FIND,
    RUN,
    RE_PLAN,
    COMPLETE,
};
static std::string enumToString(E_MOVE_POINT_STEP value)
{
    static const std::unordered_map<E_MOVE_POINT_STEP, std::string> enumToStringMap = {
        {E_MOVE_POINT_STEP::IDLE, "IDLE"},
        {E_MOVE_POINT_STEP::CHECK_SIGNAL, "CHECK_SIGNAL"},
        {E_MOVE_POINT_STEP::FIND, "FIND"},
        {E_MOVE_POINT_STEP::RUN, "RUN"},
        {E_MOVE_POINT_STEP::RE_PLAN, "RE_PLAN"},
        {E_MOVE_POINT_STEP::COMPLETE, "COMPLETE"},
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

typedef struct
{
    E_LOCATION_FROM_CHARGER bySigTemp; //신호가 있었던 경우 이전 위치 정보 (bySig 업데이트 전에 정보를 저장한다)
    E_LOCATION_FROM_CHARGER bySig;     //같은 신호가 1초동안 유지되는 경우에 대한 위치정보(메인정보)
    E_LOCATION_FROM_CHARGER bySigLive; //실시간 신호정보를 통한 위치정보 -> 1초간 동일 신호 유지하면 bySig 업데이트
    E_LOCATION_FROM_CHARGER byPose;    //신호가 없을 때 이용하기 위한 위치정보 (Center신호가 있었던 좌표 정보를 기반으로 충전기로 부터 로봇위치를 추정한다) // 신호가 한번도 없었다면 충전기 좌표 기준으로 셋팅한다.
    
    bool bOnCradle;                     // 충전기 SHORT 신호가 감지 될때 셋팅 된다.
    tPose centerSigPose;                // 센터신호를 수신할때마다 update한다.
    tPose shortSigPose;                 // 센터신호가 있고, short신호가 있을 때 업데이트한다. (틸팅UP 시작위치 부터 도킹될때까지 중심점)   
    tPose leftSideSigPose;              // side 신호를 봤을 때
    tPose rightSideSigPose;             // side 신호를 봤을 때
    tPose leftCenterSigPose;              // side 신호를 봤을 때
    tPose rightCenterSigPose;             // side 신호를 봤을 때

    bool aroundCradle;

    s32 tryDockAngle;

}tLocation;

typedef struct
{
    E_DOCKING_STATE dockingState;      // 도킹 상태
    tLocation location;                // 신호에 의해 추정한 로봇 위치
    tPoint chargerPoint;                // 충전기 위치
    E_MOVE_POINT_STEP movePointStep;
    
    double chargerLocationUpdateTime;
    double onCradleUpdateTime;

    s8 replanCount;                    // 경로 계획 재설정 횟수.

    double debugTime;

    bool bCheckSignalTime;             // 회전 등을 하는 경우, sig와 sigLive가 다를 때 잠시 멈춰주기 위한 변수. 
    s32 tryDockAngle;
    double logTime;

} tDockingData;

typedef struct
{
    /* data */
    E_SIGNALTRACK_STEP trackStep;
    E_MOVING_CENTER_STEP movingCenterStep;
    E_SWIMMING_CONTROL_STEP swimmingControlStep;
    E_TRYDOCK_STEP trydockStep;
    bool bWallFallowing;
    bool bIsSignalTracking;

    // E_SIGNAL_CHECK_ANGLE setSignalCheckAngle;
    // s32 robotLeftLCAngle;
    bool bSearchSignal;
    s16 turnTowardCradle;          // 크래들을 향해 돌 각도를 저장하는 변수 -32767 ~ 32766 각도 저장용
    bool bCheckAvoidBumperPending; // 범퍼 펜딩 중인가?
    u8 signalCheckShake;           // 시그널을 잃었을 때, 까딱이며 신호를 찾는 step

    u32 lostSignalTick;

    bool bCheckSignal;
    double checkSignalTime;

    u8 temporaryMoveCenterStep;     // 센터로 가는 중 신호가 없을 때 step

    double rightSideLastSignalCheckAngle;
    double leftSideLastSignalCheckAngle;

    E_SCAN_CONTROL_STEP scanControlStep;

    s16 swimmingTimeOut;
    s32 trydockSpeedEqualTick;
    s16 trydockTimeOut;
    s16 movingTimeOut;

    bool bTrydockEscape;
    bool bSignaltrackEscape;

    // 수영 시작
    bool bUnknownControl;
    tPose leftSwimmingPose;
    tPose rightSwimmingPose;
    u8 swimScanCheckCount;
    // 수영 끝

    // 도킹 시도 시작
    double tryDockControlTime;
    bool bFinishTurn;
    // 도킹 시도 끝
    E_CHECK_SIG_STEP checkSigStep;
    bool bLostSignal;

} tSignaltrackData;

typedef struct
{
    double deg;
    double headingError;
    bool bTurnDirection;
    tPoint max;
    tPoint min;

} tSignalCheck;

// class CSignaltracking :
class CSignaltracking : public CAvoiding
{
private:
    tSignaltrackData signaltrackData_;
    u8 lastReceiver;
    const double invalidDoubleValue = std::numeric_limits<double>::infinity();
    double swimingTemp_w;
    double targetAng;
    E_SIGNALTRACK_STEP signalTrackInit(tPose robotPose);

    void signalTrackAvoidObstacleProc(void);
    
    void setSignalTrackStep(E_SIGNALTRACK_STEP set);

    void setCheckSigStep(E_CHECK_SIG_STEP set);
    E_CHECK_SIG_STEP getCheckSigStep();

    E_SIGNALTRACK_STEP trydockHandler(tDockingData dockingData_, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_TRYDOCK_STEP tryDockCheckSlam();
    E_TRYDOCK_STEP tryDockCheckTilt();
    E_TRYDOCK_STEP tryDockGoNear();
    E_SIGNALTRACK_STEP swimmingHandler(tLocation location, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_SIGNALTRACK_STEP moveCenterHandler(tLocation location, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_SIGNALTRACK_STEP signaltrackComplete();

    /*  move center start   */
    void sideAreaProcedure(E_DIRECTION area, tLocation location);
    void sideLongAreaProcedure(E_DIRECTION area, tLocation location);
    void movingCenterProcedure(tLocation location, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    void processSignalCheck(E_CHECK_SIGNAL checkSigState, double checkTime);
    bool isDiffTargetArrived(tPose robotPose);
    void clearSignalCheckLastAngle(void);
    double getSignalCheckLastAngle(E_DIRECTION direction);
    void setMovingCenterStep(E_MOVING_CENTER_STEP set);
    E_MOVING_CENTER_STEP getMovingCenterStep(void);
    void setScanControlStep(E_SCAN_CONTROL_STEP set);
    E_SCAN_CONTROL_STEP getScanControlStep();
    E_MOVING_CENTER_STEP movingCenterVoidControl(tPose robotPose);
    E_MOVING_CENTER_STEP movingCenterForwardToChargerControl(tPose robotPose);
    E_MOVING_CENTER_STEP movingCenterTurningToCenterControl(tPose robotPose);
    E_MOVING_CENTER_STEP movingCenterForwardToCenterControl(tPose robotPose);
    E_MOVING_CENTER_STEP movingCenterTunringToChargerControl(tPose robotPose);
    void cradleEscape(tPose robotPose);
    void forwardToCenterLeftArea(tPose robotPose, tProfile profile);
    void forwardToCenterRightArea(tPose robotPose, tProfile profile);
    /*  move center end     */

    /*  swimming start   */
    void setSwimmingControlStep(E_SWIMMING_CONTROL_STEP set);
    E_SWIMMING_CONTROL_STEP sideLostControl(tPose robotPose);
    E_SWIMMING_CONTROL_STEP getSwimmingControlStep();
    void signalCheck(tPose robotPose);
    void saveLastSignalPose(tPose robotPose, E_AREA area);
    E_SWIMMING_CONTROL_STEP swimmingVoidControl(tPose robotPose);
    E_SWIMMING_CONTROL_STEP swimmingTurningToLostSideControl(tPose robotPose);
    E_SWIMMING_CONTROL_STEP swimmingTurningToReacquisitionControl(tPose robotPose);
    E_SWIMMING_CONTROL_STEP swimmingForwardToShortAreaSideControl(tPose robotPose);    
    /*  swimming end     */

    /*  try dock ready start  */
    E_TRYDOCK_STEP tryDockRotateTowardsCradle(tPose robotPose);
    E_SIGNALTRACK_STEP trydockEntrance();
    void rotateTowardsCradle(tPose robotPose);
    E_TRYDOCK_STEP getTrydockStep();
    void setTrydockStep(E_TRYDOCK_STEP set);    
    /*  try dock end    */


    E_AREA getLastSignal();
    s32 reTrySignalCheck();

public:
    CSignaltracking();
    ~CSignaltracking();

    E_SIGNALTRACK_STEP getSignalTrackStep(void);
    bool checkSignalControl(tPose robotPose);
    
    void startSignalTracking();
    bool runSignalTracking(tDockingData dockingData_, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle);
    bool wallTrackCradleAvoidHandler(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    
    // 신호 조건 정리
    bool isLeftSideReceiverLeftArea();
    bool isRightSideReceiverRightArea();
    bool isSideReceiverCenterArea();
    bool isFrontReceiverCenterArea();


    void motionCheckSignalTurn(int dir);
    void motionSwimming(u16 data);
    void motionCenteringReadyTurn(int dir);
    void motionMovingCenter(int dir);
    void motionTryDockReadyTurn(int dir);
    void motionTilDownTryDock();
    void motionTilUpTryDock();

    tTwist getCalculateCheckSignalTurnSteer(int dir);
    tTwist getCalculateTryDockTiltDownSteer();
    tTwist getCalculateTryDockTiltUpSteer();
    tTwist getCalculateSwimmingSteer(u16 data);
    tTwist getCalculateCenteringReadyTurnSteer(int dir);
    tTwist getCalculateTryDockReadyTurnSteer(int dir);
    tTwist getCalculateMoveCenterSteer(int dir);
    double adjustSpeedToTarget(double currentSpeed, double targetSpeed, double acceleration);
};