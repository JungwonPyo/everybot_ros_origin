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




// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"
#include "walltracking.h"
#include "taskAvoidCharger.h"
#include "avoiding.h"

enum class WALLTRACK_AVOID_AROUND_STATE
{
    NONE,
    START,
    RUN_WALLTRACK_AVOID,    
    AVOID_CHARGER,
    COMPLETE,
};

static std::string enumToString(WALLTRACK_AVOID_AROUND_STATE value) {
    static const std::unordered_map<WALLTRACK_AVOID_AROUND_STATE, std::string> enumToStringMap = {
        { WALLTRACK_AVOID_AROUND_STATE::NONE, "NONE," },
        { WALLTRACK_AVOID_AROUND_STATE::START, "START," },
        { WALLTRACK_AVOID_AROUND_STATE::RUN_WALLTRACK_AVOID, "RUN_WALLTRACK_AVOID," },        
        { WALLTRACK_AVOID_AROUND_STATE::AVOID_CHARGER, "AVOID_CHARGER," },        
        { WALLTRACK_AVOID_AROUND_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskWallTrackAvoidAround
{
private: /* not use! */
    CTaskWallTrackAvoidAround(const CTaskWallTrackAvoidAround& other) = default; // Copy constructor
    CTaskWallTrackAvoidAround& operator=(const CTaskWallTrackAvoidAround& other) = default; // Assignment operator

    CTaskAvoidCharger taskAvoidCharger;
    CAvoiding avoiding;
public:
    CTaskWallTrackAvoidAround();
    ~CTaskWallTrackAvoidAround();
    
    void taskStart(tPose robotPose,E_WALLTRACK_DIR dir);    
    void taskRun(tPose robotPose);
    
    bool isReturnStartPoint();
    

private:

    WALLTRACK_AVOID_AROUND_STATE state;
    E_WALLTRACK_DIR wallDir;
    tPoint startPoint;

    bool bCheckAwayFrontStart;
    bool bReturnFromStart;
    double avoidStartTime;

    std::list<tPoint> path;
    tPoint goal;    

    void setWallAvoidState(WALLTRACK_AVOID_AROUND_STATE set);
    void procWalltrackAvoid(tPose robotPose);

    WALLTRACK_AVOID_AROUND_STATE runWalltrackAvoid(tPose robotPose);
    WALLTRACK_AVOID_AROUND_STATE runChargerAvoid(tPose robotPose);

    tPoint searchNearstPointFromPath(tPose robotPose);
    u8 getLongSignalMatchedByShortSignal(u8 shortSignal);
    u8 getAvoidCharegerSignalData(E_WALLTRACK_DIR dir);
    bool isStartChargerAvoid(u8 avoidSignal);

};