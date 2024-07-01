/**
 * @file serviceData.h
 * @author jspark
 * @brief externData 를 가공하여 생성된 
 * 서비스에서 사용될 serviceData 입니다.
 * 
 * @date 2023-05-17
 */

#pragma once

#include "ros/ros.h"
#include "coreData/serviceData/batteryState.h"
#include "coreData/serviceData/keyState.h"
#include "coreData/serviceData/powerState.h"
#include "coreData/serviceData/tiltingState.h"
#include "coreData/serviceData/obstacle.h"
#include "coreData/serviceData/rosPublishData.h"
#include "coreData/serviceData/signals.h"
#include "coreData/serviceData/localization.h"
#include "coreData/serviceData/tiltingState.h"
#include "coreData/serviceData/error.h"
#include "robotmap.h"
#include "obstaclemap.h"
#include "coreData/serviceData/debugData.h"
#include "mapStorage.h"
#include "waveFrontier.h"
#include "kidnapData.h"
#include "coreData/serviceData/robotServerBridge.h"
#include "coreData/serviceData/storeAwsData.h"
#include "coreData/serviceData/robotData.h"
#include "coreData/serviceData/robotInfo.h"
#include "coreData/serviceData/areaInfo.h"
#include "coreData/serviceData/cleaningSchedule.h"
#include "coreData/serviceData/doNotDisturb.h"
#include "coreData/serviceData/forbiddenArea.h"
#include "coreData/serviceData/operationArea.h"
#include "coreData/serviceData/ota.h"

#define ServiceData     singleton::CServiceData::getInstance().getServiceData()

class CServiceData
{
public:
    /* Service 데이터 */
    CObstacle           obstacle;
    CObstaclemap        obstaclemap;
    CLocalizData        localiz;
    CSignal             signal;
    CRobotMap           robotMap;
    CKidnapData         kidnapData;
    
    /* State 데이터 */
    CBatteryState       battery;    // battery state 데이터
    CKeyState           key;        // key state 데이터
    CPowerState         power;      // power state 데이터
    CTiltingState       tilting;    // tilting state 데이터

    CDebugData          debugData;
    CMapStorage         mapStorage; // 위치 고민 필요

    CError              error;

    CRobotData          robotData;
    CRobotInfo          robotInfo;
    COperationArea      operationArea;
    CAreaInfo           areaInfo;
    CForbiddenArea      forbiddenArea; 
    CDoNotDisturb       doNotDisturb; 
    CCleaningSchedule   cleaningSchedule;
    COta                Ota;

    
    /* 서버데이터 */
    CRobotServerBridge  rsBridge;   //수신단(aws->로봇)
    CStoreAwsData       awsData;    //현재 로봇 설정상태

    double tempLidarSpeed; // 임시로 저장할 Lidar 속도 데이터
    int tempDutyCycle; // 임시로 저장할 Lidar Duty Cycle값

    //임시로 서비스가 무엇인지 저장하자.
    //NONE =0 , IDLE, CLEAN, CHARGING, DOCKING, EXPLORER, UNDOCKING, REDOCKING, WIFI,  
    int tempRunServceID;

    tTestMotionInfo motionInfo;
    tPwmDriveInfo pwmDriveInfo;
    

public:
    CServiceData(ros::NodeHandle _nh);
    ~CServiceData();

public:
    bool __testCheckDstar; // 디버그용 플레그
};

namespace singleton
{
    /**
     * @brief 모든 작업에서 Monitor의 ServiceData 데이터에 접근이 가능하도록 함.
     */
    class CServiceData
    {
    private:
        CServiceData();
        CServiceData(const CServiceData& ref);
        CServiceData &operator=(const CServiceData &ref);
        ~CServiceData();
        ::CServiceData* pServiceData;

    public:
        static CServiceData& getInstance();
        void init(::CServiceData* pServiceData);
        ::CServiceData& getServiceData();
    };
};