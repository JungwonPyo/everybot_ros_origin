/**
 * @file obstacle.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/observer.h"
#include "commonStruct.h"
#include "sensorCliff.h"
#include "interfaceStruct.h"
#include "lidarSensor.h"


#define TOF_MIN_RANGE 0
#define TOF_MAX_RANGE 1200
#define TOF_ERROR_TILT_DOWN_RANGE 230
#define TOF_ERROR_TILT_UP_RANGE 400


#define TOF_CENTER_APPROACH 200  //hjkim230504 - 약 250mm 기준 / 300mm 이상에서는 1200 데이터 나옴

#define IR_FRONT_OPENVALUE 300
#define IR_FRONT_OBSTACLE_SLOPE 30
#define IR_FRONT_CHECK_MAXSLOPE 30

#define LPF_SUM 1000
#define LPF_RAW_NEW 200
#define LPF_RAW_OLD 800

#define LPF_SLOPE_NEW 500
#define LPF_SLOPE_OLD 500

#define LPF_RAW(OLD, NEW)  (NEW * LPF_RAW_NEW + OLD * LPF_RAW_OLD) / LPF_SUM
#define LPF_SLOPE(OLD, NEW)  (NEW * LPF_SLOPE_NEW + OLD * LPF_SLOPE_OLD) / LPF_SUM



class CImuSensors
{
private:
    
public:
    CImuSensors(/* args */);
    ~CImuSensors();

    void initImuSensor();
    void updateImuSensor(tSysIMU _set);
    tSysIMU getImuSensor();
    u8 getImuState();

    tSysIMU data;

};


class CTofSensors
{
private:
    
    int continuousCliffErrorCount;
    u32 errorLogTime;

public:
    
    CTofSensors(/* args */);
    ~CTofSensors();

    void initTofSensor();

    void updateTofSensorData(tTof tof, tSysCliff cliff);

    void checkAndSendErrorMessage(tSysTilting tilt); // @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
    tSysTofInfo getTofLeftWall();
    tSysTofInfo getTofRightWall();
    tSysTofInfo getTofKnoll();
    tSysTofInfo getLeftCliff();
    tSysTofInfo getRightCliff();
    tSysTofInfo leftwall;
    tSysTofInfo rightwall;
    tSysTofInfo knoll;
    tSysTofInfo left;
    tSysTofInfo right;

private:
    // @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
    void tiltUpCheckTofError();
    void tiltDownCheckTofError();

};


class CBumperSensors
{
private:
   
public:
    RSF_OBSTACLE_MASK mask;
    RSF_OBSTACLE_MASK pre_mask;
    CBumperSensors(/* args */);
    ~CBumperSensors();
    void initBumperSensor();
    RSF_OBSTACLE_MASK getBumperMask();
    void updateBumperSensor(tSysBumper bumper);
};

class CFrontSensors
{
private:
    /* data */
    tFrontIr frontIr;
protected :
    FRONT_OSBATALCE_STATE frontStateMachine(tFrontData frontData);
    void setFrontSensor(tSysFront sysFrontIr);

public:
    LIDAR_RSF_OBSTACLE mask;
    RSF_OBSTACLE_MASK pre_mask;
    CFrontSensors(/* args */);
    ~CFrontSensors();

    void initFrontSensor();
    tFrontIr getFrontSensor();

    RSF_OBSTACLE_MASK getFrontApproachMask();
    RSF_OBSTACLE_MASK getFrontObstacleMask();

    void updateFrontSensor(tSysFront sysFrontIr);
};

class CWheeltrap
{
private:
    unsigned int errorCount;
    unsigned int trapDetectionCount;
    unsigned int lineartrapCount;
    unsigned int angulartrapCount;
    double poseChangeThreshold;
    int trapDetectionThreshold;
    double ctrlStartTime = 0;
    double movingStartTime = 0;
    double movingDistance;
    double calculateDistance(const tSysPose& pose1, const tSysPose& pose2);
public:
    RSF_OBSTACLE_MASK mask;
    RSF_OBSTACLE_MASK pre_mask;    
    tSysWheelMotor lastWheelData;
    tSysPose lastSysPose;
    
    u8 type;
    CWheeltrap(/* args */);
    ~CWheeltrap();

    void initWheeltrapSensor();
    void updateWheelTrapData(tSysWheelMotor sysWheelData, tSysPose sysPose);
    RSF_OBSTACLE_MASK getWheelTrapMask();
    
    
};

class CObstacle : public CObserver
{
private:
    /*system 데이터로 부터 raw data를 parsing 하여 각 class가 데이터를 가공하여 데이터를 저장하고,
	 저장 된 데이터를 기반으로 buildObstacleData() 함수를 통해 서비스에 전달 할 장애물 정보를 update한다  */
    CLidarSensors       lidar;    
    CBumperSensors      bumper;
    CTofSensors         tof;    
    CFrontSensors       front;
    CImuSensors         imu;
    CWheeltrap          wheeltrap;
    RSF_OBSTACLE_MASK   acc;

    RSU_OBSTACLE_DATA obstacle; //service에 넘겨줄 장애물 가공 데이터 struct
    tSysIMU			    filterImu;
    double updateLidarTime;

    unsigned int suspectCnt;
    unsigned int suspectCnt2;
    unsigned int safeCnt;

    int minAccelX;
    int minAccelY;
    int minAccelZ;
    int minRoll;
    int minPitch;
    int minYaw;
    int minCliff;
    int minWall;

    int maxAccelX;
    int maxAccelY;
    int maxAccelZ;
    int maxRoll;
    int maxPitch;
    int maxYaw;
    int maxCliff;
    int maxWall;

public:
    CObstacle();
    ~CObstacle();    
    void update(CExternData* pExternData) override;
    RSU_OBSTACLE_DATA *getObstacleData();
    tSysIMU            getFilterImuData();
    bool getApproach();
    void initObstacleSensor();
    int checkLidarSensorDetectedCount(int startDegree, int endDegree);
    bool isCliffAccumulate();
    void initCliffAccumulate();

    CCliffSensors       cliff;

private:
    void checkRobotLiftUpDown(tSysTilting tilt);
    bool isRobotLifting(tSysTilting tilt);
    bool isRobotOntheFloor(tSysTilting tilt);
    void buildObstacleData();
};
