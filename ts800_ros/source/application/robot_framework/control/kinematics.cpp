

#include "kinematics.h"
#include "eblog.h"

CRobotKinematics::CRobotKinematics(){}
CRobotKinematics::~CRobotKinematics(){}
    
tPoint CRobotKinematics::translate(tPose org, double deltaX, double deltaY)
{
    tPoint ret;

    ret.x = org.x + (deltaX * cos(org.angle) - deltaY * sin(org.angle));
    ret.y = org.y + (deltaX * sin(org.angle) + deltaY * cos(org.angle));

    return ret;   
}

tPoint CRobotKinematics::translate(tPoint org, double deltaX, double deltaY) {
    tPoint ret;

    ret.x = org.x + deltaX;
    ret.y = org.y + deltaY;

    return ret;
}

double CRobotKinematics::rotation(tPose org, double deltaRad) {
     
    double newAngle = org.angle + deltaRad;

    // 정규화.
    newAngle = fmod(newAngle + M_PI, 2 * M_PI);
    if (newAngle < 0)
        newAngle += 2 * M_PI;
    newAngle -= M_PI;

    
    return newAngle;
}

tPose CRobotKinematics::translateAndRotate(tPose org, double deltaX, 
    double deltaY, double deltaTheta) {
        tPose ret;
        // 먼저 이동을 적용
        ret.x = org.x + (deltaX * cos(org.angle) - deltaY * sin(org.angle));
        ret.y = org.y + (deltaX * sin(org.angle) + deltaY * cos(org.angle));
        // 회전 적용
        ret.angle = rotation(org, deltaTheta);
        return ret;
}

// 글로벌 좌표에서 로컬 좌표로 변환
tPoint CRobotKinematics::globalToLocal(const tPose& pose, const tPoint& globalPoint) {
    double cosTheta = cos(pose.angle);
    double sinTheta = sin(pose.angle);
    tPoint local;
    local.x = cosTheta * (globalPoint.x - pose.x) + sinTheta * (globalPoint.y - pose.y);
    local.y = -sinTheta * (globalPoint.x - pose.x) + cosTheta * (globalPoint.y - pose.y);
    return local;
}

// 로컬 좌표에서 글로벌 좌표로 변환
tPoint CRobotKinematics::localToGlobal(const tPose& pose, const tPoint& localPoint) {
    double cosTheta = cos(pose.angle);
    double sinTheta = sin(pose.angle);
    tPoint global;
    global.x = cosTheta * localPoint.x - sinTheta * localPoint.y + pose.x;
    global.y = sinTheta * localPoint.x + cosTheta * localPoint.y + pose.y;
    return global;
}



/**
 * @brief 타겟을 가기위한 가까운 회전 방향
 * 
 * @param robotPose 
 * @param target
 * @return true : 왼쪽, false 오른쪽 
 */

bool CRobotKinematics::computeRotateDir(tPose robotPose, tPoint target)
{
    bool bRet = false;

    // Calculate the angle to the target
    double targetAngle = atan2(target.y - robotPose.y, target.x - robotPose.x);

    // Calculate the smallest angle difference
    double angleDifference = targetAngle - robotPose.angle;

    // Normalize the angle to be within -pi to pi
    while (angleDifference > M_PI) angleDifference -= 2 * M_PI;
    while (angleDifference < -M_PI) angleDifference += 2 * M_PI;

    // Determine the direction based on the sign of the angle difference
    if (angleDifference > 0) {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "왼쪽 회전이 유리하다 판단");
        bRet = true;
    } else {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "오른쪽 회전이 유리하다 판단");     
        bRet = false;
    }
    return bRet;
}