
// #include "control/control.h"
#include "coordinate.h"

class CKidnapData
{
private:
    /* data */
    bool bCheckingKidnap;
    bool bOccurDisError;
    std::list<tPose> lastPoseList;
    std::list<double> measurements;
    tPose preSysPose;
public:
    CKidnapData(/* args */);
    ~CKidnapData();

    void setOccurDisError(bool set);
    bool getOccurDisError();
    void setCheckingKidnap(bool set);
    bool getCheckingKidnap();
    void watchKidnap(tPose sysPose, tPose slamPose);
    void setRobotPoseForDisError(tPose robotPose);
    tPose getLastRobotPose();
    double addMeasurement(double value, int windowSize);

    double getDiffSlamSysPose(tPose slamPose, tPose sysPose);
};


