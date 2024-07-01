

#pragma once

#include "coordinate.h"
#include "math.h"

class CRobotKinematics {
public:
    CRobotKinematics();
    ~CRobotKinematics();
    
    tPoint translate(tPose org, double deltaX, double deltaY);
    tPoint translate(tPoint org, double deltaX, double deltaY);
    double rotation(tPose org, double deltaRad);
    tPose translateAndRotate(tPose org, double deltaX, double deltaY, double deltaTheta);
    tPoint globalToLocal(const tPose& pose, const tPoint& globalPoint);
    tPoint localToGlobal(const tPose& pose, const tPoint& localPoint);
    


    bool computeRotateDir(tPose robotPose, tPoint target);

private:
    
};