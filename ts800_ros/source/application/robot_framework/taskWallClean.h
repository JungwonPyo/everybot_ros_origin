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

#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"
#include "avoiding.h"
#include "walltracking.h"
#include "taskPathPlan.h"
#include "taskMovePath.h"
#include "taskAvoidCharger.h"

enum class WALLCLEAN_STATE
{
    NONE,
    FIND_NEAR_WALL,
    MOVE_TO_NEAR_WALL,
    START_WALLTRACK,
    START_LINETRACK,
    RUN_WALLTRACK,
    RUN_LINETRACK,
    START_COMEBACK_TO_INSIDE,
    WAIT_COMBACK_PATH,
    RUN_COMEBACK_TO_INSIDE,
    RUN_AVOID_CHARGER,
    COMPLETE,
};

static std::string enumToString(WALLCLEAN_STATE value) {
    static const std::unordered_map<WALLCLEAN_STATE, std::string> enumToStringMap = {
        { WALLCLEAN_STATE::NONE, "NONE," },
        { WALLCLEAN_STATE::FIND_NEAR_WALL, "FIND_NEAR_WALL," },
        { WALLCLEAN_STATE::MOVE_TO_NEAR_WALL, "MOVE_TO_NEAR_WALL," },
        { WALLCLEAN_STATE::START_WALLTRACK, "START_WALLTRACK," },
        { WALLCLEAN_STATE::START_LINETRACK, "START_LINETRACK," },
        { WALLCLEAN_STATE::RUN_WALLTRACK, "RUN_WALLTRACK," },
        { WALLCLEAN_STATE::RUN_LINETRACK, "RUN_LINETRACK," },
        { WALLCLEAN_STATE::START_COMEBACK_TO_INSIDE, "START_COMEBACK_TO_INSIDE," },
        { WALLCLEAN_STATE::WAIT_COMBACK_PATH, "WAIT_COMBACK_PATH," },
        { WALLCLEAN_STATE::RUN_COMEBACK_TO_INSIDE, "RUN_COMEBACK_TO_INSIDE," },
        { WALLCLEAN_STATE::RUN_AVOID_CHARGER, "RUN_AVOID_CHARGER," },
        { WALLCLEAN_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

struct tAreaLine{
	double A;
    double B;
    double C;
    tPoint start;
    tPoint end;
    bool flag;
    bool operator==(const tAreaLine& other) const
    {
        return (A == other.A && B == other.B && C == other.C && start == other.start && end == other.end);
    }
};

class CTaskWallClean
{
private: /* not use! */
    CTaskWallClean(const CTaskWallClean& other) = default; // Copy constructor
    CTaskWallClean& operator=(const CTaskWallClean& other) = default; // Assignment operator
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskAvoidCharger taskAvoidCharger;
    CAvoiding avoiding;
public:
    CTaskWallClean();
    ~CTaskWallClean();
    
    E_WALLTRACK_DIR wallDir;
    void setWallCleanState(WALLCLEAN_STATE set);
    void taskStart(tPose robotPose, WALLCLEAN_STATE startState, tPoint _nearWall,std::list<tPoint> areaPath);
    bool taskRun(tPose robotPose, std::list<tPoint> areaPath);
    bool isReturnStartPoint();
    

private:
    std::list<tPoint> wallCleanAreaPath;
    std::vector<tAreaLine> wallCleanAreaLine;
    std::vector<bool> wallCleanAreaLineFlag;
    pair<tPoint,tPoint> targetLine;
    WALLCLEAN_STATE state;
    tPoint wallTrackStartPoint;
    tPoint nearWall;
    tPoint nearWallPoint;
    tPoint nearTargetPoint;
    tAreaLine nearLine;
    tAreaLine preNearLine;
    tAreaLine firstLine;
    bool bStartWalltrack;
    bool bCheckAwayFrontStart;
    bool bReturnFromStart;
    bool bTargetUpdate;
    bool isInside;
    double moveToNearWallTime;
    double wallCleanStartTime;
    double lastSideWallTime;
    unsigned int flagCnt;
    unsigned int debugCnt = 0;
    std::list<tPoint> dstarPath;

    WALLCLEAN_STATE procFindNearWall(tPose robotPose);
    WALLCLEAN_STATE procMoveNearWall(tPose robotPose);
    WALLCLEAN_STATE procStartWallTrack(tPose robotPose);
    WALLCLEAN_STATE procStartLineTrack(tPose robotPose);
    WALLCLEAN_STATE procRunWallTrack(tPose robotPose);
    WALLCLEAN_STATE procRunLineTrack(tPose robotPose);
    WALLCLEAN_STATE procStartCombackToInside(tPose robotPose);
    WALLCLEAN_STATE procWaitComeBackPath(tPose robotPose);
    WALLCLEAN_STATE procRunCombackToInside(tPose robotPose);
    WALLCLEAN_STATE procCompleteWallTrack(tPose robotPose);
    WALLCLEAN_STATE procAvoidCharger(tPose robotPose);    

    u8 getLongSignalMatchedByShortSignal(u8 shortSignal);
    u8 getAvoidCharegerSignalData(E_WALLTRACK_DIR dir);
    bool isStartChargerAvoid(u8 avoidSignal);
    bool isWallCleanEnd(tPose robotPose);
    std::vector<tAreaLine> setCleanAreaLine(std::list<tPoint> areaPoints);
    tAreaLine findNearWallLine(tPose robotPose, std::vector<tAreaLine> areaLines);
    void setNearWallLineFlag(tAreaLine line);
    tPoint findNearWallPoint(tPose robotPose, tAreaLine nearLine);
    bool checkRobotNearLine(tPose robotPose, tAreaLine line, double margin);
};