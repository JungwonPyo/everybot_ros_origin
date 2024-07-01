/**
 * @file control.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/serviceData/localization.h"
#include "coreData/serviceData/obstacle.h"
#include "coreData/serviceData/areaInfo.h"
#include "control/slamControl.h"
#include "control/systemControl.h"
#include "control/lidarDriverControl.h"

#define ROBOT_CONTROL CControl::getInstance()

enum class E_TILT_STEP
{
    VOID,
    WAIT,
    RETRY,
    COMPLETE,
    ERROR,                    // 제어 진행중
};

static std::string enumToString(E_TILT_STEP value)
{
    static const std::unordered_map<E_TILT_STEP, std::string> enumToStringMap = {
        { E_TILT_STEP::VOID, "VOID" },
        { E_TILT_STEP::WAIT, "WAIT" },
        { E_TILT_STEP::RETRY, "RETRY" },
        { E_TILT_STEP::COMPLETE, "COMPLETE" },
        { E_TILT_STEP::ERROR, "ERROR" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

struct tTiltInfo//tTimeOut
    {
        E_TILTING_CONTROL cmd;
        E_TILT_STEP step;
        bool bRunning;
        double ctrlTime;
        double startTime;
        int retryCount;
    };

class CControl 
{
public:
	CSlamControl slam;
    CLidarDriverControl lidar;

	CSystemControl system;	// initial system

private: /* not use! */
    CControl();
	~CControl();
    CControl(const CControl& other) = default; // Copy constructor
    CControl& operator=(const CControl& other) = default; // Assignment operator
	tTiltInfo  tInfo;   // [틸팅 제어]
public:
    static CControl& getInstance();
	#if SKIP_CHECKTILT > 0
	void tilting(E_TILTING_CONTROL control);
	#endif

    void reportAwsAction(short action);
    void reportAwsStatus(int state);

    
    void reportAwsWaterLevel();
    void reportAwsSoundLevel();
    void reportAwsCleanMode();
    void reportAwsLanguage();
    void reportAwsCountry();
    void reportAwsBattery();
    void reportAwsDryMopData();
    void reportAwsError();
    void reportRobotSettings();
    void reportOperationArea();
    void reportForbiddenArea();
    void reportDistruptMode();
    void reportReservationClean();
    void reportSaveMapInfo(bool completedByForce);
    void reportMapData(E_SERVICE_ID id);
    void reportAreaInfo(tAreaInfoData areaInfo);
    void reportDivideArea();
    void reportCombineArea();
    void reportCleanHistroy();
    void reportFactoryReset();
    void reportRobotPose();
    void reportOtaVersion();

    void robotSystemReset();
    void robotPowerOff();
    void systemOpenCloseOta(bool on);
    void systemStartOta();

	void WaterPump(bool on);
	void dryFanOn(E_DRYFAN_LEVEL lv);
	void dryFanOff();

	void systemModeControl(u8 type);
	void clearSystemLocalization(void);	
	void startSlam(void);
    u16 getPumpCurrent(void);

    void setTiltStep(E_TILT_STEP set);
    bool checkTiltState(E_SYS_TILT_STATE state);
	void startTiltingUp();

    void startTiltingStop();
    void startTilt(E_TILTING_CONTROL control);
    void reStartTilt(E_SYS_TILT_STATE state);
    E_TILT_STEP waitCheckTilt(E_SYS_TILT_STATE state);
    E_TILT_STEP retryTilt(E_SYS_TILT_STATE state);
    E_TILT_STEP completeTilt();
    E_TILT_STEP failTilt();
    E_TILT_STEP errorTilt();

	void procControlTilt();
    bool isTiltRunning();
    double getTiltCtrlStartTime();
	void sendMessageTilting(E_TILTING_CONTROL control);
	E_TILT_STEP getTiltStep();
private:
	u16 pumpcurrent;
    u16 pumpcurMax; // danielhongL210311 : average 대신 max 값 기준으로 활용
};
