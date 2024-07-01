#pragma once

#include "wayPoint.h"

enum class E_WAYPOINT_STEP
{
    NONE,
    ACTION_POP,         //wayPoint action 한개를 가지고 온다.    
    ACTION_WAIT,        //wayPoint action 한개 수행 완료를  기다린다.
    COMPLETE,           //완료.
};
static std::string enumToString(E_WAYPOINT_STEP value) {
    static const std::unordered_map<E_WAYPOINT_STEP, std::string> enumToStringMap = {
        { E_WAYPOINT_STEP::NONE, "NONE" },
        { E_WAYPOINT_STEP::ACTION_POP, "ACTION_POP" },
        { E_WAYPOINT_STEP::ACTION_WAIT, "ACTION_WAIT" },
        { E_WAYPOINT_STEP::COMPLETE, "COMPLETE" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CWayPointManager
{
private:
    CWayPoint wayPoint;
    E_WAYPOINT_STEP step;
    tAction currentAction;

    void setWayPointStep(E_WAYPOINT_STEP set);
    
public:
    CWayPointManager(/* args */);
    ~CWayPointManager();

    bool isWayPointComplete();
    bool isActive();
    void setWayPoint(CWayPoint& set);
    void startWayPoint();
    tAction getCurrentAction();
    E_WAYPOINT_STEP getWayPointStep();
    E_WAYPOINT_STEP runWayPoint();
    E_WAYPOINT_STEP stepNone();
    E_WAYPOINT_STEP stepActionPop();
    E_WAYPOINT_STEP stepActionWait();
    E_WAYPOINT_STEP stepComplete();

public: // Debug
    std::list<tPoint> debugCurrentPath; // debug용 현재 경로 (wayPoint에서 tPoint list를 적절하게 추출함.)
};

