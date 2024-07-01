/**
 * @file walltracking.h
 * @author hjkim
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
#include "fileMng.h"
#include "MessageHandler.h"


#define WALLTRACK_TOF_TRUST_VALUE	200		// 벽면 TOP 신뢰 가능한 길이
#define WALLTRACK_CLIFF_TOF			95


enum class E_WALLTRACK_PATTERN_FLAG
{
	NONE,
	STOP,
	ROTATE,
	OPP_BACK,
	BACK
};

enum class E_AREA_WALLTRACK_CONTROL_STEP
{
    V_DECEL,
    ROTATE,
    MOVE_FORWARD,
};

static std::string enumToString(E_AREA_WALLTRACK_CONTROL_STEP value) {
    static const std::unordered_map<E_AREA_WALLTRACK_CONTROL_STEP, std::string> enumToStringMap = {
        { E_AREA_WALLTRACK_CONTROL_STEP::V_DECEL, "V_DECEL," },
        { E_AREA_WALLTRACK_CONTROL_STEP::ROTATE, "ROTATE," },
        { E_AREA_WALLTRACK_CONTROL_STEP::MOVE_FORWARD, "MOVE_FORWARD," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CWalltracking
{
public:
	explicit CWalltracking();
	~CWalltracking();
private:
	CWalltracking(const CWalltracking& other) = default; // Copy constructor
    CWalltracking& operator=(const CWalltracking& other) = default; // Assignment operator

	double obsChecktime;
	bool isRotate;
	bool isOppBack;
	bool isBack;

	int preSideTof;
	int preFrontIR;
	int preFrontTof;

	int sideTof;
    int cliffTof;
	int  frontTof;
	bool isWallBump;
	bool isOppBump;
	bool isFrontObs;
	bool isLidarObs;
	bool isNeedCurvOut;
	int rotateDir;
	double obsTime;

	double debugCnt;

	double accumulateAngle;
	double tempAngle;

	tPoint preTargetPoint;
	bool isOutOfArea;
	int distErrorFromPath;
	bool bAvoiding;
	double avoidStartTime;

	E_AREA_WALLTRACK_CONTROL_STEP areaControlStep;

public:    
    void procWalltracking(tPose robotPose, E_WALLTRACK_DIR direction);
	void procAreaWalltracking(tPose robotPose, E_WALLTRACK_DIR direction, tPoint startPoint, tPoint targetPoint, bool update);
	void startAccumulateAngle(tPose robotPose);
	double getAccumulateAngle(tPose robotPose);

private:
    void setSensorData(E_WALLTRACK_DIR direction);


    bool isObsDetectForStop();
    bool isObsDetectForSlowdown();
	bool isObsDetectForTurning();	
    
    tPoint getCalculatedTargetPoint(tPose robotPose, tPoint targetPoint, double waypoint);

    tTwist runWallTrackPattern(E_WALLTRACK_DIR direction, tPose robotPose);
	tTwist runAreaWallTrackPattern(E_WALLTRACK_DIR direction, tPose robotPose, tPoint startPoint, tPoint targetPoint, bool update);
	
	tTwist driveWallTrack(tTwist cur, tPose robotPose, E_WALLTRACK_DIR direction);
    tTwist driveOppCurveBack(tTwist cur, tPose robotPose);
	tTwist driveCurveBack(tTwist cur, tPose robotPose, E_WALLTRACK_DIR direction);
	tTwist rotateWallTrack(tTwist cur,   tPose robotPose, E_WALLTRACK_DIR direction);
	tTwist driveAreaLineTrack(tTwist cur, tPose robotPose, tPoint startPoint, tPoint targetPoint);
    tTwist driveEscapeWall(tPose robotPose, tTwist curVel, E_WALLTRACK_DIR direction, bool isOppBack, bool isBack, bool isRotate);

    void setPatternFlags(E_WALLTRACK_PATTERN_FLAG flag, double time = 0.0);
    bool checkPatternTime();
	bool areaWallTrackStateChecker(tPose robotPose, E_AREA_WALLTRACK_CONTROL_STEP curStep);

	
};
