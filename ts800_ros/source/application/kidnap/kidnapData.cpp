#include "kidnapData.h"
#include "eblog.h"
#include <cmath>

CKidnapData::CKidnapData(/* args */)
{
    bCheckingKidnap =false;
    bOccurDisError  =false;
}

CKidnapData::~CKidnapData()
{
}

void CKidnapData::setRobotPoseForDisError(tPose robotPose)
{
    lastPoseList.emplace_back(robotPose);
    if(lastPoseList.size() > 6)
    {
        lastPoseList.pop_front();
    }
}
tPose CKidnapData::getLastRobotPose()
{
    return lastPoseList.front();
}


void CKidnapData::setOccurDisError(bool set)
{
    bOccurDisError =set;
}
bool CKidnapData::getOccurDisError()
{
    return bOccurDisError;
}

void CKidnapData::setCheckingKidnap(bool set)
{
    if(set == true)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 키드냅 검사를 시작하겠습니다.");
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 키드냅 검사를 종료하겠습니다.");
    }
    bCheckingKidnap =set;
}
bool CKidnapData::getCheckingKidnap()
{
    return bCheckingKidnap;
}

void CKidnapData::watchKidnap(tPose sysPose, tPose slamPose)
{
    if( getCheckingKidnap()&& getOccurDisError() == false)
        {
            double curPoseDiff = getDiffSlamSysPose(sysPose, preSysPose);
            double avgPoseDiff = addMeasurement(curPoseDiff, 10);
            double triggerDiff = fabs(curPoseDiff - avgPoseDiff);
            setRobotPoseForDisError(slamPose);
            if( curPoseDiff > 0.25)
            {
                ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 좌표 차이 : " << curPoseDiff << "  ");
                ceblog(LOG_LV_NECESSARY, BOLDRED, " 키드냅 발생 !! " );
                setOccurDisError(true);
            }
            else
            {
                //ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 좌표 차이 : " << curPoseDiff << "  ");
            }
        }
        
        preSysPose = sysPose;
}

double CKidnapData::addMeasurement(double value, int windowSize) 
{
    measurements.push_back(value);
    if (measurements.size() > windowSize) 
    {
        measurements.erase(measurements.begin());
    }

    double sum = 0.0;
    for (const double& val : measurements) 
    {
        sum += val;
    }

    return sum / measurements.size();
}

double CKidnapData::getDiffSlamSysPose(tPose slamPose, tPose sysPose)
{
    double deltaX = slamPose.x - sysPose.x;
    double deltaY = slamPose.y - sysPose.y;
    double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    return distance;
}
