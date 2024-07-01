/**
 * @file clean.h
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
#include "patterns.h"

#define CLEANAREA_UP_MARGIN_FROM_WALL       0.3     // 0.25 -> 0.4
#define CLEANAREA_DOWN_MARGIN_FROM_WALL     0.3     // 0.8  -> 1.0
#define CLEANAREA_LEFT_MARGIN_FROM_WALL     0.3     // 0.25 -> 0.4
#define CLEANAREA_RIGHT_MARGIN_FROM_WALL    0.3     // 0.25 -> 0.4

enum class E_CLEAN_MODE
{
    AUTO,
    FAST,
    WALL,   //for test
    // AREASELECT,
    // ROOM,
    // RESERVATION,
    // CIRCLE,
    // SPOT,
    // RANDOM, //for test
};

static std::string enumToString(E_CLEAN_MODE value) {
    static const std::unordered_map<E_CLEAN_MODE, std::string> enumToStringMap = {
        { E_CLEAN_MODE::AUTO, "AUTO," },
        { E_CLEAN_MODE::FAST, "FAST," },
        { E_CLEAN_MODE::WALL, "WALL," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum class E_CLEAN_STEP
{  
    INIT,                   //
    PLAN_AREA,          // 청소할 영역에 대한 준비한다.
    CHECK_MOVE,
    MOVE_AREA,          // 현재 area 에서 라인 청소를 진행하기 전 준비 상태
    RE_PLAN_AREA,       // 이미 선택된 area 로 다시 경로를 생성 하여 이동 한다.
    CLEAN_AREA,           // 현재 area 에서 라인 청소를 진행하는 상태
    CHECK_CLEAN_LIST,     // 청소 패턴을 체크한다.
    UPDATE_AREA,
    CLEAN_END,              // 영역 라인청소, 벽면청소 모두 끝난상태
};
static std::string enumToString(E_CLEAN_STEP value) {
    static const std::unordered_map<E_CLEAN_STEP, std::string> enumToStringMap = {
        { E_CLEAN_STEP::INIT, "INIT," },
        { E_CLEAN_STEP::PLAN_AREA, "PLAN_AREA," },
        { E_CLEAN_STEP::CHECK_MOVE, "CHECK_MOVE," },
        { E_CLEAN_STEP::MOVE_AREA, "MOVE_AREA," },
        { E_CLEAN_STEP::RE_PLAN_AREA, "RE_PLAN_AREA," },
        { E_CLEAN_STEP::CLEAN_AREA, "CLEAN_AREA," },
        { E_CLEAN_STEP::CHECK_CLEAN_LIST, "CHECK_CLEAN_LIST," },
        { E_CLEAN_STEP::UPDATE_AREA, "UPDATE_AREA," },
        { E_CLEAN_STEP::CLEAN_END, "CLEAN_END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}



typedef struct
{
    E_CLEAN_MODE        mode;
    std::list<E_CLEAN_PATTERN>     patterns;
    E_CLEAN_STEP        cleanStep;     // 라인청소 상태 // 라인청소 준비 단계
    std::list<tPoint>   arrangedPolygons;
    std::list<tPoint>   polygonPoints;    
    int                 cleanedRoom;
    tPose               lineTargetPose;
   
}tCleanData;


class CClean
{
private:
    CObstaclemap *pObsMap; 
    CCleanPatterns *pCleanPatterns;
    tCleanData     cleanData_;
public:
    CClean();
    ~CClean();

    void initCleanning(void);
    bool runCleanning (tPose robotPose);
    void stopCleanning(void);

    void setCleanMode(int set);

    void setCleanStep( E_CLEAN_STEP set );
    E_CLEAN_STEP getCleanStep();

    void addCleanPattern(E_CLEAN_PATTERN set);
    void clearCleanPattern();
    E_CLEAN_PATTERN getCurrentPattern();    
    int getCurrentPatternSize();
    
    void popCurrentPattern();


    E_CLEAN_STEP procInit(tPose robotPose);
    E_CLEAN_STEP procPlanArea(tPose robotPose);
    E_CLEAN_STEP procRePlanArea(tPose robotPose);
    E_CLEAN_STEP procCheckMove();
    E_CLEAN_STEP procMoveArea(tPose robotPose);
    E_CLEAN_STEP procRunClean(tPose robotPose);
    E_CLEAN_STEP procCheckCleanList(tPose robotPose);
    E_CLEAN_STEP procUpdateArea(tPose robotPose);
    bool procCleanEnd();

    std::list<tPose> getSortByNearest(tPose robotPose, const std::list<tPoint>& targetList);
};