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

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "control/control.h"

#include "taskLocalMotion.h"
#include "taskPathPlan.h"
#include "taskMovePath.h"
#include "taskWalltrackAvoid.h"

// explorer PROC 상태 
enum class E_DOCKING_EXPLORER_STATE
{
    NONE,                       // 초기 세팅
    RUN_RECOGNIZE,           // 좌우 회전 
    MOVE_TO_TARGET,
    AVOID_WALLTRACK,
    MAP_RECONSIDERATION_MOVE_START,   // 지도 확장을 위해 움직여 본다.
    MAP_RECONSIDERATION_MOVE_WAIT,    // 지도 확장을 위해 움직임을 대기한다.
    AWAY_WALL_MOVE_START,   // 지도 확장을 위해 움직여 본다.
    AWAY_WALL_MOVE_WAIT,    // 지도 확장을 위해 움직임을 대기한다.
    STOP_WAIT,
    END,                        // 탐색 완료
};
static std::string enumToString(E_DOCKING_EXPLORER_STATE value) {
    static const std::unordered_map<E_DOCKING_EXPLORER_STATE, std::string> enumToStringMap = {
        { E_DOCKING_EXPLORER_STATE::NONE, "NONE," },
        { E_DOCKING_EXPLORER_STATE::RUN_RECOGNIZE, "RUN_RECOGNIZE," },  
        { E_DOCKING_EXPLORER_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },       
        { E_DOCKING_EXPLORER_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { E_DOCKING_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START, "MAP_RECONSIDERATION_MOVE_START," },      
        { E_DOCKING_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_WAIT, "MAP_RECONSIDERATION_MOVE_WAIT," },
        { E_DOCKING_EXPLORER_STATE::AWAY_WALL_MOVE_START, "AWAY_WALL_MOVE_START," },      
        { E_DOCKING_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT, "AWAY_WALL_MOVE_WAIT," },
        { E_DOCKING_EXPLORER_STATE::STOP_WAIT, "STOP_WAIT," },     
        { E_DOCKING_EXPLORER_STATE::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_DOCKING_EXPLORER_RECOGNIZE_STATE
{
    NONE,                       // 초기 세팅
    RECOGNIZE_START,
    RECOGNIZE_MOTION,           // 좌우 회전 
    RECOGNIZE_BACK,
    RECOGNIZE_TURN,
    REQUEST_PATH,
    COMPLETE,                        // 탐색 완료
};
static std::string enumToString(E_DOCKING_EXPLORER_RECOGNIZE_STATE value) {
    static const std::unordered_map<E_DOCKING_EXPLORER_RECOGNIZE_STATE, std::string> enumToStringMap = {
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::NONE, "NONE," },
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::RECOGNIZE_START, "RECOGNIZE_START," },
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::RECOGNIZE_MOTION, "RECOGNIZE_MOTION," },
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::RECOGNIZE_BACK, "RECOGNIZE_BACK," },
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::RECOGNIZE_TURN, "RECOGNIZE_TURN," },
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::REQUEST_PATH, "REQUEST_PATH," },     
        { E_DOCKING_EXPLORER_RECOGNIZE_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


class CTaskDockingExplorer
{
private: /* not use! */
    CTaskDockingExplorer(const CTaskDockingExplorer& other) = default; // Copy constructor
    CTaskDockingExplorer& operator=(const CTaskDockingExplorer& other) = default; // Assignment operator

public:
    CTaskDockingExplorer();
    ~CTaskDockingExplorer();
    
    E_DOCKING_EXPLORER_STATE getExplorerState();

    bool taskRun(tPose robotPose);
    bool checkDockingSignal();

    double getRunningTime();

    void taskStart();
    
private:
    CTaskLocalMotion taskLocalMotion;
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskWallTrackAvoid taskWallAvoid;
    CAvoiding avoiding;

    E_DOCKING_EXPLORER_STATE state;                // EXPLORER proc 상태
    E_DOCKING_EXPLORER_STATE tempState;
    E_DOCKING_EXPLORER_RECOGNIZE_STATE recognizeState;

    E_WALLTRACK_DIR wallDir;

    tProfile profile;

    tPoint curGoal;
    tPoint newGoal;
    tPoint targetEnd;
    tPoint targetStart;
    tPoint startWallPoint;

    bool bTurnReverse = false;
    bool bCheckFistWFP;
    bool bStartSearhPath;
    bool bDockingSignal;

    unsigned int invalidCnt;    //유효하지 않은 탐색 점 횟수
    unsigned int recognizeCnt;
    unsigned int reconsiderCnt;

    double targetRad;
    double reconsiderAngle;
    double walltrackStartTime;
    double distTarget;      //비교를 위한 타겟 과의 거리
    double pathSearchStartTime;
    double exploreStartTime;

    std::list<tPoint> curPath;
    std::list<tPoint> newGoalList;
    std::list<tPoint> historyList;

    E_DOCKING_EXPLORER_STATE procNone(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procRecognize(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procMoveTarget(tPose robotPose);   
    E_DOCKING_EXPLORER_STATE procAvoidWalltrack(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procMapReconsiderationMoveStart(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procMapReconsiderationMoveWait(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procAwayWallMoveStart(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procAwayWallMoveWait(tPose robotPose);
    E_DOCKING_EXPLORER_STATE procStopWait(tPose robotPose);

    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeStart(tPose robotPose);
    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeMotion(tPose robotPose);
    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeMotionBack(tPose robotPose);
    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeMotionTurn(tPose robotPose);
    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeRequestPath(tPose robotPose);
    E_DOCKING_EXPLORER_RECOGNIZE_STATE procRecognizeComplete(tPose robotPose);
    
    bool monitorExplorer(tPose robotPose);
    bool checkWaveFrontier(tPose robotPose);
    bool checkUpdateWaveFrontier(tPose robotPose);
    bool checkExploerFinish();
    bool explorerEnd(tPose robotPose);

    void computeWalltrackDir(tPose robotPose, tPoint target);

    // state    
    void setExplorerState(E_DOCKING_EXPLORER_STATE set);
    void setRecognizeState(E_DOCKING_EXPLORER_RECOGNIZE_STATE set);

    // getting 
    tPoint getNearDisPose(tPose robotPose, std::list<tPoint> frontierPoints);
    std::list<tPose> clusterFrontierPoses(tPose robot, tPose nearPose, std::list<tPose> frontiers, double distance);
    void  getNeighbours(int n_array[], int position, int map_width);

    // (DEBUG)
    void __debug_state_print();        
    bool bDebugStatePrint;
    bool isMapReady();
};