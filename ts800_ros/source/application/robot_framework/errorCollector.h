/**
 * @file errorCollector.h
 * @author jspark
 * @brief 메세지를 통하여 수집된 에러들이 저장되는 곳 입니다.
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "MessageHandler.h"

struct tErrorInfo
{
    tErrorInfo();

    bool bTrapObstacle;
    bool bBumperCurrent;
    bool bBatteryAdapter;
    bool bBatteryNoset;
    bool bBatteryError;
    bool bWifi;

    bool bCommunication;
    
    bool bLiftCliff;
    bool bTilt;
    
    bool bWheel;

    /* 진행 불가 에러 */
    bool bDockingFail;

    /* ros data 가 안들어오는 경우 */
    bool bMissingLidarData; // ros의 lidar 데이터가 들어오지 않는 경우.
    bool bMissingGridmapData;   // ros의 gridmap 데이터가 들어오지 않는 경우.
};


class CErrorCollector
{
private:
    tErrorInfo info;
    bool bCheckError;

protected:
    void setErrorInfo(message_t* pMsg);
    void clearErrorInfo();

public:
    CErrorCollector();
    ~CErrorCollector();
    tErrorInfo getErrorInfo();
    bool isErrorDetected();
};