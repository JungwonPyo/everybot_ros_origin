

#pragma once

#include "commonStruct.h"
#include "motionPlanner/wayPoint.h"
#define LIDAR_POINT_SIZE 360

#define LIDAR_DETECT_COUNT  5

#define LIDAR_MIN_RANGE  0.012
#define LIDAR_MAX_RANGE  16



enum class LIDAR_OSBATALCE_STATE
{
    NOTHING = 0,
    APPROACH = 1,
    OBSTACLE = 2
};


class CLidarPoint{
public:
    CLidarPoint();
    ~CLidarPoint();

    bool isDetected();
    void clear();
    void check(double dist, int idx);
    void setId(int set);
    int getId();

private:
    bool bDetected;
    int life;
    int term;
    int id;
    double distanceCriteria;
private:
    double getWallSensingDistancebyAngle(int degreeAngle, double wallDistance);
    double getLidarObstacleReference(u16 idx);
    double getSpatternLidarObstacleReference(u16 idx);
    bool CheckLidarDataValidate(double dist);
};

class CLidarSensors
{
private:
    CWayPoint wayPoint;
    LIDAR_RSF_OBSTACLE mask_front; //0~32 /328~360
    LIDAR_RSF_OBSTACLE mask_left; //33~90
    LIDAR_RSF_OBSTACLE mask_right; //329~270
    u8 avg_idx_count;
    int frontIdxRange;
    CLidarPoint lidarPoint[LIDAR_POINT_SIZE];
    int leftFrontidx;
    int rightFrontidx;

    bool CheckLidarDataValidate(double dist);
    int getLidarMaskIndex(void);
public:
    
    CLidarSensors(/* args */);
    ~CLidarSensors();

    void initLidarSensor();
    LIDAR_RSF_OBSTACLE getLidarMask();
    LIDAR_RSF_OBSTACLE getLidarLeftWallMask();
    LIDAR_RSF_OBSTACLE getLidarRightWallMask();
    
    void updateLidarSensor(double const *lidarBuf);
    void checkLidarSensorDetectedCount(int startDegree, int endDegree);
};