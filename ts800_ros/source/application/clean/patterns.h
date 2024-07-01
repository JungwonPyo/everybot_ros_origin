/**
 * @file line_clean.h
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
#include "avoiding.h"
#include "motionPlanner/wayPoint.h"
#include "lineTrack.h"
#include "rosPublisher.h"

enum class E_CLEAN_PATTERN
{
    LINE,
    CIRCLE,
    SPOT,
    RANDOM, //for test
    WALL,   //for test
};

static std::string enumToString(E_CLEAN_PATTERN value) {
    static const std::unordered_map<E_CLEAN_PATTERN, std::string> enumToStringMap = {
        { E_CLEAN_PATTERN::LINE, "LINE," },
        { E_CLEAN_PATTERN::CIRCLE, "CIRCLE," },
        { E_CLEAN_PATTERN::SPOT, "SPOT," },
        { E_CLEAN_PATTERN::RANDOM, "RANDOM," },
        { E_CLEAN_PATTERN::WALL, "WALL," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_PATTERN_STATE
{
    INIT,
    RUN,
    COMPLETE,
};

static std::string enumToString(E_PATTERN_STATE value) {
    static const std::unordered_map<E_PATTERN_STATE, std::string> enumToStringMap = {
        { E_PATTERN_STATE::INIT, "INIT," },
        { E_PATTERN_STATE::RUN, "RUN," },
        { E_PATTERN_STATE::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

// enum class E_PURPOSE_LINE_CLEAN_INFO
// {
//     NONE,
//     UP,
//     DOWN,
//     LEFT,
//     RIGHT,
// };
// static std::string enumToString(E_PURPOSE_LINE_CLEAN_INFO value) {
//     static const std::unordered_map<E_PURPOSE_LINE_CLEAN_INFO, std::string> enumToStringMap = {
//         { E_PURPOSE_LINE_CLEAN_INFO::NONE, "NONE," },
//         { E_PURPOSE_LINE_CLEAN_INFO::UP, "UP," },
//         { E_PURPOSE_LINE_CLEAN_INFO::DOWN, "DOWN," },
//         { E_PURPOSE_LINE_CLEAN_INFO::LEFT, "LEFT," },
//         { E_PURPOSE_LINE_CLEAN_INFO::RIGHT, "RIGHT," },
//     };

//     auto it = enumToStringMap.find(value);
//     if (it != enumToStringMap.end()) {
//         return it->second;
//     } else {
//         return "Unknown";
//     }
// }
// Do Line Step 정리
enum class E_PATTERN_LINE_STEP
{
    CLEANNING,
    AVOIDING,
    WALLFOLLOW,  // line이 끝나지 않았지만 별도의 avoid 처리.
    END,    // line이 end가 되면, do line을 종료. update line이 완료가 되면 step을 READY로 바꾼다.
    
};

static std::string enumToString(E_PATTERN_LINE_STEP value) {
    static const std::unordered_map<E_PATTERN_LINE_STEP, std::string> enumToStringMap = {
        { E_PATTERN_LINE_STEP::CLEANNING, "CLEANNING," },
        { E_PATTERN_LINE_STEP::AVOIDING, "AVOIDING," },
        { E_PATTERN_LINE_STEP::WALLFOLLOW, "WALLFOLLOW," },
        { E_PATTERN_LINE_STEP::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum class E_DO_LINE_STEP
{
    INIT,
    TURN,
    GO,  // line이 끝나지 않았지만 별도의 avoid 처리.
    LINE_UPDATE,
    ARRIVE,
    END,    // line이 end가 되면, do line을 종료. update line이 완료가 되면 step을 READY로 바꾼다.
    
};

static std::string enumToString(E_DO_LINE_STEP value) {
    static const std::unordered_map<E_DO_LINE_STEP, std::string> enumToStringMap = {
        { E_DO_LINE_STEP::INIT, "INIT," },
        { E_DO_LINE_STEP::TURN, "TURN," },
        { E_DO_LINE_STEP::GO, "GO," },
        { E_DO_LINE_STEP::ARRIVE, "ARRIVE," },
        { E_DO_LINE_STEP::LINE_UPDATE, "LINE_UPDATE," },
        { E_DO_LINE_STEP::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_AVOID_LINE_STEP
{
    INIT,
    BACK,
    TURN,  // line이 끝나지 않았지만 별도의 avoid 처리.
    AVOIDING,
    END,    // line이 end가 되면, do line을 종료. update line이 완료가 되면 step을 READY로 바꾼다.
    
};

static std::string enumToString(E_AVOID_LINE_STEP value) {
    static const std::unordered_map<E_AVOID_LINE_STEP, std::string> enumToStringMap = {
        { E_AVOID_LINE_STEP::INIT, "INIT," },
        { E_AVOID_LINE_STEP::BACK, "BACK," },
        { E_AVOID_LINE_STEP::TURN, "TURN," },
        { E_AVOID_LINE_STEP::AVOIDING, "AVOIDING," },
        { E_AVOID_LINE_STEP::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum class E_PATTERN_PATHRUN_STEP
{
    CHECK_WAYPOINT,       //way Point 를 획득한다.
    ACTION_POP,         //wayPoint action 한개를 가지고 온다.    
    ACTION_WAIT,        //wayPoint action 한개 수행 완료를  기다린다.
    END,    // line이 end가 되면, do line을 종료. update line이 완료가 되면 step을 READY로 바꾼다.
    
};

static std::string enumToString(E_PATTERN_PATHRUN_STEP value) {
    static const std::unordered_map<E_PATTERN_PATHRUN_STEP, std::string> enumToStringMap = {
        { E_PATTERN_PATHRUN_STEP::CHECK_WAYPOINT, "CHECK_WAYPOINT," },
        { E_PATTERN_PATHRUN_STEP::ACTION_POP, "ACTION_POP," },
        { E_PATTERN_PATHRUN_STEP::ACTION_WAIT, "ACTION_WAIT," },
        { E_PATTERN_PATHRUN_STEP::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

// Do Line Step 정리
enum class E_LINE_CLEAN_TYPE
{
    E_NORMAL_CLEAN,   // 긴 ㄹ 자 청소.
    E_REVERSE_CLEAN,  // 짧은 ㄹ자 청소
    E_GRID_CLEAN,     // 격자 청소
};

typedef struct
{
    bool crossLine;
    bool compactLine;

}tPatternLineOption;


class CPatternLine : public CAvoiding
{
private:

    typedef struct
    {
        //std::list<tPose> plan;
        E_DO_LINE_STEP  doLineStep;
        E_AVOID_LINE_STEP avoidStep;
        std::list<tPose> avoidiPath;
        tPatternLineOption option;

        E_PURPOSE_LINE_CLEAN_INFO tempLine;
        E_PURPOSE_LINE_CLEAN_INFO curLine;
        E_PURPOSE_LINE_CLEAN_INFO shortLine;
        E_PURPOSE_LINE_CLEAN_INFO newLine;

        E_PATTERN_LINE_STEP     step;     // do line 단계
        E_PATTERN_LINE_STEP     nextStep;
        //E_PATTERN_PATHRUN_STEP  pathRunStep;

        tPose               startPose;
        tPoint              TargetPoint;

        tPose              tempstartPose;
        tPose              beforestartPose;

        tPoint              tempTargetPoint;
        tPoint              beforeTargetPoint;
        tPose               AvoidStartPose;
        tPose               tempSlam;

        double              areaMinX;
        double              areaMaxX;
        double              areaMinY;
        double              areaMaxY;

        double              areaRangeX;
        double              areaRangeY;

        bool                isAvoidTurnLeft;
        bool                startLine;
        bool                endLine;
        bool                doDoubleClean;
        bool                checkAvoidingEnd;
        double              tempSlamAngle;

        //for debug
        double              lineCleanStartTime;
        double              debug_Time;
        int                 debugNaviPrintCnt;    
        int                 areaPrintCnt;
        bool                debugEnd;
        double              debug_slamUpdateTime;

        double  debugWallStartTime;

        ////////////////////////////////
    }tPatternLineData;

    tPatternLineData  patternLineData_; 
    CObstaclemap *pObsMap;     
    CWayPoint wayPoint; //

    double cleanLineInterval = ROS_CONFIG.cleanLineInterval;

    //CMotionPid pid;   //icbaek, 230323, MCU 에서 제어 하기로 결정.

public:
    CPatternLine();
    ~CPatternLine();

    //bool checkObstacle(tPose robotPose, bool clean) override;
    void makeCliffEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeBumperEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    
    E_PURPOSE_LINE_CLEAN_INFO getCurrentLine(tPose target, tPose start);
    void updateStartLine(tPose robotPose);
    E_PURPOSE_LINE_CLEAN_INFO getStartLine(tPose robotPose, tPoint halfPoint);
    E_PURPOSE_LINE_CLEAN_INFO getShortLineByLongLine(tPose robotPose, E_PURPOSE_LINE_CLEAN_INFO longLine);

    void setLinePatternStep(E_PATTERN_LINE_STEP set);
    E_PATTERN_LINE_STEP getLinePatternStep();

    void setCurLine(E_PURPOSE_LINE_CLEAN_INFO set);
    E_PURPOSE_LINE_CLEAN_INFO getCurLine();

    void setLineTemp(E_PURPOSE_LINE_CLEAN_INFO set);
    E_PURPOSE_LINE_CLEAN_INFO getLineTemp();

    void setShortLine(E_PURPOSE_LINE_CLEAN_INFO set);
    E_PURPOSE_LINE_CLEAN_INFO getShortLine();

    void setNewLine(E_PURPOSE_LINE_CLEAN_INFO set);
    E_PURPOSE_LINE_CLEAN_INFO getNewLine();

    void setAvoidStep(E_AVOID_LINE_STEP set);
    E_AVOID_LINE_STEP getAvoidStep();

    void setDoLineStep(E_DO_LINE_STEP set);
    E_DO_LINE_STEP getDoLineStep();

    E_PATTERN_STATE initLinePatternStep ( tPose robotPose,const std::list<tPoint>& areaPoligons);
    E_PATTERN_STATE runLinePatternStep(tPose robotPose,cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle);
    bool completeLinePatternStep(tPose robotPose);
    
    E_PATTERN_LINE_STEP lineCleanStepCleanning(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle);
    E_PATTERN_LINE_STEP lineCleanStepAvoiding(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle);
    E_PATTERN_LINE_STEP lineCleanStepWalltrack(bool slamUpdate, tPose robotPose,RSU_OBSTACLE_DATA *pObstacle);
    E_PATTERN_STATE lineCleanStepComplete(tPose robotPose);

    void checkObstacleSize(const std::list<tPose>& avoidPathList,tPose *min, tPose *max);
    tPoint checkTargetPointFromAvoidPath(E_PURPOSE_LINE_CLEAN_INFO newLine,tPoint target, const std::list<tPose>& avoidPathList);
    bool checkLineAvoidEnd(E_PURPOSE_LINE_CLEAN_INFO curLine,E_PURPOSE_LINE_CLEAN_INFO shortLine,tPose robotPose,bool isAvoiding);
    void runRandomCleanPattern(tPose robotPose,RSU_OBSTACLE_DATA* pObstacle);
    
    E_PURPOSE_LINE_CLEAN_INFO updateLine(bool isArriveNextLine);
    bool checkEndLine(tPose robotPose);
    tPoint getUpdateShortLineTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine,tPose robotPose, bool endLine);
    tPoint getUpdateLongLineTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine, tPose robotPose);
    tPoint updateTargetPoint(E_PURPOSE_LINE_CLEAN_INFO newLine,tPose robotPose,bool isEndLine);
   
    bool isEndLineInThisArea(tPose robotPose); 
    

    //========================================================================using for stepWallTrack ===========================================================
    E_PATTERN_LINE_STEP checkWallFollowingEnd(tPose robotPose);
    E_WALLFACE_ID getWallfaceLineToWall(bool turnning,tPose robotPose);
    bool isClosedEndLine(tPose robotPose, E_PURPOSE_LINE_CLEAN_INFO curLine, E_PURPOSE_LINE_CLEAN_INFO shortLine);
    bool isRetunCurrentLine(tPose robotPose,E_PURPOSE_LINE_CLEAN_INFO curLine);
    bool isArriveTargetLine(E_PURPOSE_LINE_CLEAN_INFO curLine ,tPoint target,tPose robotPose);
    bool isArriveNextLine(E_PURPOSE_LINE_CLEAN_INFO shortLine ,tPoint target,tPose robotPose);

    //========================================================================utils for LinePattern ===========================================================
    bool isTurnLeftByLine(E_PURPOSE_LINE_CLEAN_INFO curLine, E_PURPOSE_LINE_CLEAN_INFO tempLine, E_PURPOSE_LINE_CLEAN_INFO shortLine);

    std::list<tPoint> shrinkRectangle(std::list<tPoint> areaPolygons, double upMargin, double downMargin, double leftMargin, double rightMargin);
    tPoint findFarthestPoint(tPoint referencePoint, std::list<tPoint> otherPoints);
    tPoint findClosestPoint(tPoint referencePoint, std::list<tPoint> otherPoints);

    //tPose getCurrentTargetPose();

public: //debug 모음
    void __debug_state_print__(tPose robotPose);//debug
    
private:
    //procedure 함수 기능 모음
};


class CCleanPatterns //: public CAvoiding
{
private:
    E_PATTERN_STATE state;
    CObstaclemap *pObsMap;     
    
    E_PATTERN_STATE initCleanPattern(E_CLEAN_PATTERN pattern, tPose robotPose, 
        const std::list<tPoint>& areaPoligons);
    E_PATTERN_STATE runCleanPattern(E_CLEAN_PATTERN pattern,tPose robotPose, 
        cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle);
    bool completeCleanPattern(E_CLEAN_PATTERN pattern, tPose robotPose);

    
public:
    CCleanPatterns();
    ~CCleanPatterns();
    CPatternLine *pLinePattern;
    CLineTrack   *pLineTrack;

    void setPatternState(E_PATTERN_STATE set);
    E_PATTERN_STATE getPatternState();

    bool controlHandler(E_CLEAN_PATTERN pattern, tPose robotPose, const std::list<tPoint>& areaPoligons,cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle);
    bool patternAvoidProc(E_CLEAN_PATTERN pattern,tPose robotPose,RSU_OBSTACLE_DATA *pObstacle);
    
    E_PATTERN_STATE runLineCleanPattern(tPose robotPose,cell_obstacle* pObsMap, RSU_OBSTACLE_DATA *pObstacle);
    
};