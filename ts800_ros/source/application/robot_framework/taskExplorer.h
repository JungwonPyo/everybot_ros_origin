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
#include "taskWalltrackAvoidAround.h"

// explorer PROC 상태 
enum class E_EXPLORER_STATE
{
    NONE,                       // 초기 세팅
    RUN_RECOGNIZE,           // 좌우 회전 
    MOVE_TO_TARGET,
    AVOID_WALLTRACK,
    AVOID_WALLTRACK_AROUND,
    AVOID_CLIFF,
    MAP_RECONSIDERATION_MOVE_START,   // 지도 확장을 위해 움직여 본다.
    MAP_RECONSIDERATION_MOVE_WAIT,    // 지도 확장을 위해 움직임을 대기한다.
    AWAY_WALL_MOVE_START,   // 지도 확장을 위해 움직여 본다.
    AWAY_WALL_MOVE_WAIT,    // 지도 확장을 위해 움직임을 대기한다.
    STOP_WAIT,
    END,                        // 탐색 완료
};
static std::string enumToString(E_EXPLORER_STATE value) {
    static const std::unordered_map<E_EXPLORER_STATE, std::string> enumToStringMap = {
        { E_EXPLORER_STATE::NONE, "NONE," },
        { E_EXPLORER_STATE::RUN_RECOGNIZE, "RUN_RECOGNIZE," },  
        { E_EXPLORER_STATE::MOVE_TO_TARGET, "MOVE_TO_TARGET," },       
        { E_EXPLORER_STATE::AVOID_WALLTRACK, "AVOID_WALLTRACK," },
        { E_EXPLORER_STATE::AVOID_WALLTRACK_AROUND, "AVOID_WALLTRACK_AROUND," },
        { E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_START, "MAP_RECONSIDERATION_MOVE_START," },      
        { E_EXPLORER_STATE::MAP_RECONSIDERATION_MOVE_WAIT, "MAP_RECONSIDERATION_MOVE_WAIT," },
        { E_EXPLORER_STATE::AWAY_WALL_MOVE_START, "AWAY_WALL_MOVE_START," },      
        { E_EXPLORER_STATE::AWAY_WALL_MOVE_WAIT, "AWAY_WALL_MOVE_WAIT," },
        { E_EXPLORER_STATE::STOP_WAIT, "STOP_WAIT," },     
        { E_EXPLORER_STATE::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_RECOGNIZE_STATE
{
    NONE,                       // 초기 세팅
    RECOGNIZE_START,
    RECOGNIZE_MOTION,           // 좌우 회전 
    RECOGNIZE_BACK,
    RECOGNIZE_TURN,
    REQUEST_PATH,
    COMPLETE,                        // 탐색 완료
};
static std::string enumToString(E_RECOGNIZE_STATE value) {
    static const std::unordered_map<E_RECOGNIZE_STATE, std::string> enumToStringMap = {
        { E_RECOGNIZE_STATE::NONE, "NONE," },
        { E_RECOGNIZE_STATE::RECOGNIZE_START, "RECOGNIZE_START," },
        { E_RECOGNIZE_STATE::RECOGNIZE_MOTION, "RECOGNIZE_MOTION," },
        { E_RECOGNIZE_STATE::RECOGNIZE_BACK, "RECOGNIZE_BACK," },
        { E_RECOGNIZE_STATE::RECOGNIZE_TURN, "RECOGNIZE_TURN," },
        { E_RECOGNIZE_STATE::REQUEST_PATH, "REQUEST_PATH," },     
        { E_RECOGNIZE_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

/**
 * @brief 회피동작을 처리하기위한 스탭 구조체
 * 
 */
typedef struct _tCliffAction
{    
    _tCliffAction():
    bRun(false), backStart(false), backWait(false), backEnd(false) {}

    bool bRun;
    bool backStart;
    bool backWait;
    bool backEnd;
    tPoint start;
    tPoint target;

    void clear() {
        bRun = false;
        backStart = false;
        backWait = false;
        backEnd = false;
    }
    
}tCliffAction;


class CTaskExplorer
{
private: /* not use! */
    CTaskExplorer(const CTaskExplorer& other) = default; // Copy constructor
    CTaskExplorer& operator=(const CTaskExplorer& other) = default; // Assignment operator

public:
    CTaskExplorer();
    ~CTaskExplorer();
    
    // proc
    bool taskRun(tPose robotPose);
    void taskStart();
    // state
    E_EXPLORER_STATE getExplorerState();
    
private:
    tCliffAction cliffAction;
    CTaskLocalMotion taskLocalMotion;
    CTaskPathPlan taskPathPlan;
    CTaskMovePath taskMovePath;
    CTaskWallTrackAvoid taskWallAvoid;    
    CTaskWallTrackAvoidAround taskWallAvoidAround;
    E_EXPLORER_STATE state;                // EXPLORER proc 상태    
    E_EXPLORER_STATE tempState;
    E_RECOGNIZE_STATE recognizeState;
    tProfile profile;
    unsigned int invalidCnt;    //유효하지 않은 탐색 점 횟수
    unsigned int recognizeCnt;

    double reconsiderAngle;
    bool bTurnReverse = false;
    CAvoiding avoiding;
    double walltrackStartTime;
    double distTarget;      //비교를 위한 타겟 과의 거리
    E_WALLTRACK_DIR wallDir;
    tPoint targetEnd;
    tPoint targetStart;
    double targetRad;

    tPoint curGoal;
    tPoint newGoal;

    double pathSearchStartTime;

    unsigned int reconsiderCnt;
    unsigned int pathLockCnt;   //이동을 못하고 경로획득만 주구장창 하면 구속이다.

    tPoint startWallPoint;
    std::list<tPoint> curPath;
    std::list<tPoint> newGoalList;
    std::list<tPoint> historyList;

    bool bCheckFistWFP;
    double exploreStartTime;
    bool bStartSearhPath;
    
    bool monitorExplorer(tPose robotPose);

    E_EXPLORER_STATE procNone(tPose robotPose);
    E_EXPLORER_STATE procRecognize(tPose robotPose);
    E_EXPLORER_STATE procMoveTarget(tPose robotPose);   
    E_EXPLORER_STATE procAvoidWalltrack(tPose robotPose);
    E_EXPLORER_STATE procAvoidWalltrackAround(tPose robotPose);
    E_EXPLORER_STATE procAvoidCliff(tPose robotPose);
    E_EXPLORER_STATE procMapReconsiderationMoveStart(tPose robotPose);
    E_EXPLORER_STATE procMapReconsiderationMoveWait(tPose robotPose);
    E_EXPLORER_STATE procAwayWallMoveStart(tPose robotPose);
    E_EXPLORER_STATE procAwayWallMoveWait(tPose robotPose);
    E_EXPLORER_STATE procStopWait(tPose robotPose);

    E_RECOGNIZE_STATE procRecognizeStart(tPose robotPose);
    E_RECOGNIZE_STATE procRecognizeMotion(tPose robotPose);
    E_RECOGNIZE_STATE procRecognizeMotionBack(tPose robotPose);
    E_RECOGNIZE_STATE procRecognizeMotionTurn(tPose robotPose);    
    E_RECOGNIZE_STATE procRecognizeComplete(tPose robotPose);
    
    bool checkWaveFrontier(tPose robotPose);
    bool checkUpdateWaveFrontier(tPose robotPose);    
    bool checkExploerFinish();
    bool explorerEnd(tPose robotPose);
    void computeWalltrackDir(tPose robotPose, tPoint target);
    bool checkPathLock();

    // state    
    void setExplorerState(E_EXPLORER_STATE set);
    void setRecognizeState(E_RECOGNIZE_STATE set);

    // getting 
    tPoint getNearDisPose(tPose robotPose, std::list<tPoint> frontierPoints);
    std::list<tPose> clusterFrontierPoses(tPose robot, tPose nearPose, std::list<tPose> frontiers, double distance);
    void  getNeighbours(int n_array[], int position, int map_width);

    // (DEBUG)
    void __debug_state_print();        
    bool bDebugStatePrint;
    bool isMapReady();
};