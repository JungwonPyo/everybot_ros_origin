/**
 * @file rsfMonitor.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <atomic>
#include <mutex>
#include <pthread.h>
#include <ctime>

#include "coreData/serviceData.h"
#include "coreData/subject.h"
#include "externData/externData.h"
#include "control/control.h"
#include "coreData/serviceData/debugData.h"
#include "externData/systemData.h"
#include "MessageHandler.h"
#include "errorCollector.h"
#include "pthreadLockGuard.h"

class CRsfMonitor : CSubject, public CErrorCollector
{
private:
    CExternData         externData;
    CServiceData        serviceData;

private:
    bool isRunningMessage;
    bool isRunningRosDebug;
    //for_requst of the ceva
    bool isRunningRosDebugRbtPlusRotationVector;
    bool isRunningRosDebugRbtPlusFeedBack;

    bool isRunningRosDataPub;
    bool isRunningSystemWatch;
    bool isRunningDstarWallUpdate;
    bool isRunningWaveFrontier;    
    bool isRunningLidar;

    std::thread thMessage;
    pthread_t thRosDebug;
    pthread_t thRosDataPub;
    pthread_t thRosImuDataPub;

    //for_request of the ceva
    pthread_t thRosDebugPlusRotationVector;
    pthread_t thRosDebugRbtPlusFeedBack;

    pthread_t thRosDebugSimplifyGridmap;
    pthread_t thSystemWatch;
    pthread_t thLidar;
    pthread_t thLidarTf;
    
    //카토그래퍼에서 서브 맵 업데이트 주기를 스위칭 할 수 있는 기능(기능만 만들어 놓고 비활성화 함)
    pthread_t thSlamControl;
    pthread_t thSubMapFilter;
 
    pthread_t thCleanMapProc;

    //청소 중 임시 라이더 지도 저장 쓰레드
    pthread_t thTemporarySaveSlamMap;
    //RBT_PLUS			
    struct timespec currTime;
    struct timespec preTime;
public:
    CRsfMonitor(ros::NodeHandle _nh);
    ~CRsfMonitor();
    void update();
    void enroll();
    CServiceData* getServiceDataPointer();
    CExternData* getExternDataPointer();

private:

    //control initData
    void messageTypeInitSensor();
    void messageTypeInitMoving();

    //control system Mode
    void messageTypeActiveMode();
    void messageTypeChargeMode();

    //control wheelMotor    
    void messageTypeControlWheel(message_t* pMsg);

    /*water Pump motor contorl msg add*/
    void messageTypeControlPump(message_t* pMsg);

	/*tilting motor contorl msg add*/
    void messageTypeControlTilt(message_t* pMsg);
    /*Speaker contorl msg add*/
    void messageTypePlaySound(message_t* pMsg);
	/*LCD contorl msg add*/
    void messageTypePlayDisplay(message_t* pMsg);

    /*LED contorl msg add*/
    void messageTypePlayLed(message_t* pMsg);
	/* dry mop fanMotor contorl msg add*/
    void messageTypeControlDryFan(message_t* pMsg);

    void messageTypeClaerLocalization(void);
    void messageTypeSystemCtr(message_t* pMsg);
    void messageTypePowerOff();
    void messageTypeMcuReset();

    /* AWS  */
    void messageTypeReportAction( message_t* pMsg);
    void messageTypeReportStatus( message_t* pMsg);
    void messageTypeReportFactoryReset(message_t* pMsg);
    void messageTypeReportAwsData( message_t* pMsg);
    void messageTypePhoneInterface( message_t* pMsg);

    void messageTypeOtaCommand( message_t* pMsg);


    void threadMessage();
    
    //RBT_PLUS
    timespec getCurrentTime();
    long long getTimeDifference(const timespec& start, const timespec& end);
 
    static void* threadRosDebugSubWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_ROS_DEBUG_SUB);
        myRsfMonitor->threadRosDebugSub();
    }
    void threadRosDebugSub();

    static void* threadRosDebugRbtPlusRotationVectorWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_RBT_PLUS_RV);
        myRsfMonitor->threadRosDebugRbtPlusRotationVector();
    }
    void threadRosDebugRbtPlusRotationVector();

    static void* threadRosDebugRbtPlusFeedBackWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_RBT_PLUS_FB);
        myRsfMonitor->threadRosDebugRbtPlusFeedBack();
    }
    void threadRosDebugRbtPlusFeedBack();

    static void* threadSystemWatchWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_SYSTEM_WATCH);
        myRsfMonitor->threadSystemWatch();
    }
    void threadSystemWatch();

    void sendFakeBsp();

    static void* threadRosDataPubWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_ROS_DATA_PUB);
        myRsfMonitor->threadRosDataPub();
    }
    void threadRosDataPub();
    
    static void* threadRosDataImuPubWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "ROS IMU Data pub");
        myRsfMonitor->threadRosImuDataPub();
    }
    void threadRosImuDataPub();

    static void* threadLidarWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "Lidar Interface");
        myRsfMonitor->threadLidar();
    }
    void threadLidar();
    
    static void* threadLidarTfPubWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "Lidar TF pub");
        myRsfMonitor->threadLidarTfPub();
    }
    void threadLidarTfPub();

    //TS800 -> TOPIC -> CARTOGRAPER OF THE ENABLE OR DISABLE INSERT SUBMAP
    static void* threadRosInsertSubMapFilterWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "Submap Filter");
        myRsfMonitor->threadRosInsertSubMapFilter();
    }
    void threadRosInsertSubMapFilter();
    
    //카토그래퍼에서 서브 맵 업데이트 주기를 스위칭 할 수 있는 기능(기능만 만들어 놓고 비활성화 함)
    static void* threadRosSlamControlWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "slam control");
        myRsfMonitor->threadRosSlamControl();
    }
    void threadRosSlamControl();

    static void* threadCleanMapProcWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), "cleanmapproc");
        myRsfMonitor->threadCleanMapProc();
    }
    void threadCleanMapProc();

    static void* threadTemporarySaveSlamMapWrap(void* arg)
    {
        CRsfMonitor* myRsfMonitor = static_cast<CRsfMonitor*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_TEMP_SLAM_MAP_SAVE);
        myRsfMonitor->threadTemporarySaveSlamMap();
    }
    void threadTemporarySaveSlamMap();

    void sendServiceDataToSystemInterface();
    void sendSlamPoseData();
};
