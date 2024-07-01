/**
 * @file avoiding.h
 * @author hjkim
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"
#include "coreData/serviceData.h"
#include "motionPlanner/wayPointManager.h"
#include "avoiding.h"
#include "dstar.h"


class CAvoidingMotionPlanner : public CAvoiding
{
private:
    bool bIsUpdateObstacleWall;
    std::list <tDstarWallPoint> obstacleWall;
    bool isRobotInWall; // 이거 꼭 있어야 하냐ㅏ???

public:

    CAvoidingMotionPlanner();
    ~CAvoidingMotionPlanner();

    void setIsRobotInWall(bool set);
    bool isUpdateObstacleWall();
    std::list <tDstarWallPoint> useDstarObstacleWall();

    bool checkObstacle(tPose robotPose, bool clean, bool isAvoiding) override;

    // avoding
    void makeBumperEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeKnollEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;    
    void makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint) override;
    void makeDstarObstacleWall(tPose robotPose);    

};

