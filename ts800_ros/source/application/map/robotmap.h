/**
 * @file robotmap.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <list>
#include <vector>

#include "coreData/observer.h"
#include "ebtypedef.h"
#include "commonStruct.h"
#include "cleanplan.h"
#include "gridmap.h"
#include "trajectory.h"
#include "cells.h"

#define IMSI_USE_AREA_IMG_PRC 1

typedef enum
{
    ROBOT_DIR_LEFT   = 0,
    ROBOT_DIR_FLEFT  = 1,
    ROBOT_DIR_FRONT  = 2,
    ROBOT_DIR_FRIGHT = 3,
    ROBOT_DIR_RIGHT  = 4,
    ROBOT_DIR_BACK   = 5,
    ROBOT_DIR_BLEFT  = 6,
    ROBOT_DIR_BRIGHT = 7
}E_ROBOT_DIR;

class CRobotTrajectory
{
private :
    std::list<tPoint> cleanedTrajectory; // 청소중에 저장되는 로봇의 궤적.
    std::list<tPoint> exploredTrajectory; // 탐색중에 저장되는 로봇의 궤적.


public:
    CRobotTrajectory();
    ~CRobotTrajectory();

    //청소
    std::list<tPoint> getCleanedTrajectory();
    void clearCleanedTrajectory();
    void setCleanedTrajectory(tPoint cleanedPoint);
    void setCleanedTrajectory(tPose cleanedPose);
    
    //탐색
    std::list<tPoint> getExploredTrajectory();
    void clearExploredTrajectory();
    void setExploredTrajectory(tPoint exploredPoint);
    void setExploredTrajectory(tPose exploredPose);

    CTrajectory trajectory;
};

class CRobotMap : public CObserver
{
public:
    CRobotMap();
    ~CRobotMap();
    bool debug_showRoom;
    CSimplifyGridMap simplifyMap;    
    CRobotTrajectory robotTrajectory;
    CCells cleanMap;
private:
    void update(CExternData* pExternData) override;
};
