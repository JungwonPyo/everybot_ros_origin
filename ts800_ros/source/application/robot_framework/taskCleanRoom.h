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
#include "avoiding.h"
#include "walltracking.h"
#include "taskMovePath.h"
#include "taskMoveRoom.h"
#include "taskMoveCleanLine.h"
enum class CLEAN_ROOM_STATE
{
    NONE,
    START_LINE,
    RUNNING_MAIN_LINE,
    RUNNING_MAIN_LINE_AVOID,
    RUNNING_SIDE_LINE,
    RUNNING_SIDE_LINE_AVOID,
    RUNNING_SIDE_LINE_WALLTRACK,    
};

static std::string enumToString(CLEAN_ROOM_STATE value) {
    static const std::unordered_map<CLEAN_ROOM_STATE, std::string> enumToStringMap = {
        { CLEAN_ROOM_STATE::NONE, "NONE," },                
        { CLEAN_ROOM_STATE::RUNNING_MAIN_LINE, "RUNNING_MAIN_LINE," },
        { CLEAN_ROOM_STATE::RUNNING_MAIN_LINE_AVOID, "RUNNING_MAIN_LINE_AVOID," },
        { CLEAN_ROOM_STATE::RUNNING_SIDE_LINE, "RUNNING_SIDE_LINE," },
        { CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_AVOID, "RUNNING_SIDE_LINE_AVOID," },
        { CLEAN_ROOM_STATE::RUNNING_SIDE_LINE_WALLTRACK, "RUNNING_SIDE_LINE_WALLTRACK," },        
        { CLEAN_ROOM_STATE::START_LINE, "START_LINE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum class MAIN_LINE_AVOID_STATE
{
    NONE,

    BACK_START,
    BACK_WAIT,
    TRUN_START,
    TRUN_WAIT,
};

static std::string enumToString(MAIN_LINE_AVOID_STATE value) {
    static const std::unordered_map<MAIN_LINE_AVOID_STATE, std::string> enumToStringMap = {
        { MAIN_LINE_AVOID_STATE::NONE, "NONE," },        
        { MAIN_LINE_AVOID_STATE::BACK_START, "BACK_START," },
        { MAIN_LINE_AVOID_STATE::BACK_WAIT, "BACK_WAIT," },
        { MAIN_LINE_AVOID_STATE::TRUN_START, "TRUN_START," },
        { MAIN_LINE_AVOID_STATE::TRUN_WAIT, "TRUN_WAIT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class SIDE_LINE_AVOID_STATE
{
    NONE,

    BACK_START,
    BACK_WAIT,
    TRUN_START,
    TRUN_WAIT,    
};

static std::string enumToString(SIDE_LINE_AVOID_STATE value) {
    static const std::unordered_map<SIDE_LINE_AVOID_STATE, std::string> enumToStringMap = {
        { SIDE_LINE_AVOID_STATE::NONE, "NONE," },        
        { SIDE_LINE_AVOID_STATE::BACK_START, "BACK_START," },
        { SIDE_LINE_AVOID_STATE::BACK_WAIT, "BACK_WAIT," },
        { SIDE_LINE_AVOID_STATE::TRUN_START, "TRUN_START," },
        { SIDE_LINE_AVOID_STATE::TRUN_WAIT, "TRUN_WAIT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}



enum class LINE_STATE
{
    MAIN_MOVE,
    SIDE_MOVE,
};

static std::string enumToString(LINE_STATE value) {
    static const std::unordered_map<LINE_STATE, std::string> enumToStringMap = {
        { LINE_STATE::MAIN_MOVE, "MAIN_MOVE," },
        { LINE_STATE::SIDE_MOVE, "SIDE_MOVE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}



union tCleanEndFlag
{
    u8 value;
    struct
    {
        u8 coverageLimit    : 1;    //커버리지 다됨.
        u8 sideMoveError    : 1;    // 사이드 이동할곳 없음
        u8 lineInterval     : 1;    // 사이드 이동간격이 너무 적음
        u8 rotate           : 1;         // 회전 구속
        u8 endline          : 1;         // 라인 종료
        u8 dockingzone      : 1;
        u8 reserve1         : 1;
    }b;
};



typedef struct tEndAreaFlag
{
    bool trig;  //외부 입력 트리거
    bool preState;  //이전상태
    unsigned int fallingCnt;    //이전상태 유지
}tEndAreaFlag;

    

typedef struct tLine
{
    tPoint start;
    tPoint end;
}tLine;

typedef struct tLineHistory
{
    tLine pre;    // 이전 라인
    tLine target;    // 수행해야할 라인    
}tLineHistory;

typedef struct tLineInfo
{
    tLineHistory main;
    tLineHistory side;
}tLineInfo;
// 직선의 방정식을 저장하기 위한 구조체
struct Line {
    double a;  // x의 계수
    double b;  // y의 계수
    double c;  // 상수항
};
class CTaskCleanRoom
{
private: /* not use! */
    CTaskCleanRoom(const CTaskCleanRoom& other) = default; // Copy constructor
    CTaskCleanRoom& operator=(const CTaskCleanRoom& other) = default; // Assignment operator

public:
    CTaskCleanRoom();
    ~CTaskCleanRoom();    
    
    // proc
    tCleanEndFlag taskRun(tPose robotPose);
    void taskStart(tPose robotPose, double rotate);
    LINE_STATE getLineState();
    void setEndLineTrig(bool set);
    void setEndFlagCleaned(bool bSet);
    void setDockingZoneTrig(bool set);
    CLEAN_ROOM_STATE getCleanRoomState();
    bool checkNextCell(cell &nextCell, tPose robotPose);
    unsigned int getLineCleanCnt();


private:
    //action
    CLEAN_ROOM_STATE startLine(tPose robotPose);    
    CLEAN_ROOM_STATE runningMainLine(tPose robotPose);
    CLEAN_ROOM_STATE runningMainLineAvoid(tPose robotPose);
    CLEAN_ROOM_STATE runningSideLine(tPose robotPose);
    CLEAN_ROOM_STATE runningSideLineAvoid(tPose robotPose);
    CLEAN_ROOM_STATE runningSideLineWallTrack(tPose robotPose);
    
    

    //func
    void setCleanRoomState(CLEAN_ROOM_STATE set);    
    void calcTargetMainLine(tPose robotPose);
    void calcTargetMainLineV2(tPose robotPose, tPoint start, tPoint end);
    void calcTargetSideLine(tPose robotPose, tPoint target);
    void initTargetLine(tPose robotPose);
    void clearLineInfo();    
    void computeWalltrackDir(tPose robotPose, tPoint target);
    bool checkRoomCleaned(tPose robotPose);
    void setLineState(LINE_STATE set);
    void clearCheckLineInterval();
    void checkLineInterval(tPose robotPose);
    void startCheckLineEnd();
    bool checkLineEnd();
    void preMainLineSetup(tPose robotPose);
    bool preSideLineSetup(tPose robotPose);
    tPoint getNextPosition(const tPose& robotPose, const tPoint& target, double stepSize);
    double pointToLineDistance(const tPoint &start, const tPoint &end, const tPose &robotPose);
    tPoint findNextSearchingLinePoint(tPose robotPose, double deltaY);
    void createSegment(tPoint start, double heading, double length, tPoint &segmentStart, tPoint &segmentEnd);
    bool checkIntersection(tPoint segmentStart, tPoint segmentEnd, tPoint lineStart, tPoint lineEnd);
    bool findIntersection(tPoint segmentStart, tPoint segmentEnd, tPoint lineStart, tPoint lineEnd, tPoint& result);
    bool findIntersection(Line l1, Line l2, tPoint &intersection);
    std::list<tPoint> getIntersections(tPoint org, tPoint segmentStart, tPoint segmentEnd, const std::list<tPoint>& polygon);
    bool findUncleanLine(tPoint start, tPoint end, tPoint &uncleandPoint);
    bool findMainLineStartPoint(tPose robotPose,const std::list<tPoint>& area, tPoint searchingPt, double cleanHeading, tPoint &uncleanPoint);
    bool computeNextMainLinePoint(tPose robotPose, tPoint &uncleanPoint, double cleanHeading);

    tLineInfo lineInfo;
    CLEAN_ROOM_STATE cleanRoomState;
    CLEAN_ROOM_STATE preDemoBState;
    CLEAN_ROOM_STATE bakCleanRoomState;  // 이전상태로 대돌릴 버퍼
    MAIN_LINE_AVOID_STATE avoidState;
    SIDE_LINE_AVOID_STATE avoidSideState;

    CTaskMoveCleanLine moveLine;
    
    double startAngle;
    double endAngle;
    tPose startPose;
    tPoint targetStart;
    tPoint targetEnd;    
    double walltrackStartTime;
    LINE_STATE lineState;
    E_WALLTRACK_DIR direction;
    CAvoiding avoiding;

    bool bPahtSearching;    //경로 획득중 플래그
    tCleanEndFlag cleanEndFlag;
    tEndAreaFlag endLineFlag;   // 라인 종료 판단 플래그
    unsigned int lineCleanCnt;  // 라인클린 횟수

    double globalRotate;
    
};