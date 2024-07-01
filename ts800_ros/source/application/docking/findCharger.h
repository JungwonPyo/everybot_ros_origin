/**
 * @file findCharger.h
 * @author hhryu (hhryu@everybot.net)
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <iostream>
// #include <unordered_map>
#include "ebtypedef.h"
// #include "signaltracking.h"
// #include "findCharger.h"
#include "coreData/serviceData.h"
#include "control/control.h"
#include "utils.h"
#include "robotSlamPose.h"
#include "avoiding.h"


enum class E_FINDCHARGER_STEP
{
    NONE,
    INIT,
    WITH_MAP,
    WITHOUT_MAP,
    MOVE_POINT,
    COMPLELT,
    FAIL,
};

static std::string enumToString(E_FINDCHARGER_STEP value)
{
    static const std::unordered_map<E_FINDCHARGER_STEP, std::string> enumToStringMap = {
        {E_FINDCHARGER_STEP::NONE, "NONE"},
        {E_FINDCHARGER_STEP::INIT, "INIT"},
        {E_FINDCHARGER_STEP::WITH_MAP, "WITH_MAP"},
        {E_FINDCHARGER_STEP::WITHOUT_MAP, "WITHOUT_MAP"},
        {E_FINDCHARGER_STEP::MOVE_POINT, "MOVE_POINT"},
        {E_FINDCHARGER_STEP::COMPLELT, "COMPLELT"},
        {E_FINDCHARGER_STEP::FAIL, "FAIL"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end())
    {
        return it->second;
    }
    else
    {
        return "Unknown";
    }
}



typedef struct
{
    bool bMovePoint;
    bool bRequestPath;
    E_FINDCHARGER_STEP findChargerStep;
    E_CHECK_SIG_STEP checkSigStep;

    // setCheckedRegion
    tPose* checkedRegionPoint;  // 체크 완료 영역의 중심 좌표 배열
    int checkedRegionCount;      // 현재 저장된 중심 좌표의 개수
    int checkedRegionCapacity;   // 현재 할당된 배열의 용량
    bool bChecking; // 체크 중임을 나타내는 변수
    tPoint nextPoint;

} tFindChargerData;

static const int INITIAL_CAPACITY = 10;  // checkedRegion 초기 동적 배열 용량    

class CFindCharger : public CAvoiding
{
private:    
    tFindChargerData findChargerData_;
    /* avoid */
    // bool findChargerAvoidProc(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    // bool findChargerCheckObstacle(void);
    // void findChargerAvoidObstacleProc(void);
    /* avoid */
private:
    void setFindChargerStep(E_FINDCHARGER_STEP set);
    E_FINDCHARGER_STEP initProcedure();
    E_FINDCHARGER_STEP withMapProcedure(tPoint chargerPoint, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_FINDCHARGER_STEP withoutMapProcedure(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    bool isMovingPoint();
    bool getCheckedRegion(tPoint checkPoint                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     );
    void setCheckedRegion(tPose robotPose);
    tPoint nextCheckPoint(tPose robotPose);
    void setMovingPoint(bool bSet);
    bool findChargerEnd();
    bool nextPoint();
    bool runExplorerCharger(tPose robotPose);
    double targetAng;

public:
    CFindCharger();
    ~CFindCharger();
    /* avoid */
    // bool startAvoidMoving(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    // E_AVOID_STEP cliffAvoiding(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    // E_AVOID_STEP bumperAvoiding(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    // E_AVOID_STEP knollAvoiding(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    // E_AVOID_STEP wheeltrapAvoiding(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    // E_AVOID_STEP frontAvoiding(tPose robotPose, RSF_OBSTACLE_MASK mask, RSU_OBSTACLE_DATA *pObstacle) override;
    /* avoid */
    void init();
    E_FINDCHARGER_STEP getFindChargerStep(void);
    void startFindCharger(tSignalData singalData);
    E_FINDCHARGER_STEP runFindChargerProcedure(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle, tPoint chargerPoint);
    E_FINDCHARGER_STEP finishProcedure(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    void stopFindCharger(void);
    bool checkSignal(tPose robotPose);
    bool movePointHandler(tPose robotPose, tPoint targetPoint, RSU_OBSTACLE_DATA *pObstacle);
    void setNextPoint(tPoint);
    tPoint getNextPoint();
};