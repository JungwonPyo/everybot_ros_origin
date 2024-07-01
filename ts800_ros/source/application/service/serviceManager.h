/**
 * @file serviceManager.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "coreData/serviceData.h"
#include "serviceClean.h"
#include "serviceCharging.h"
#include "serviceDocking.h"
#include "serviceUnDocking.h"
#include "serviceExplorer.h"
#include "serviceIdle.h"
#include "serviceWifi.h"
#include "commonStruct.h"
#include "control/control.h"
#include "kidnap.h"
#include "location.h"
#include "robotSlamPose.h"
#include "errorCollector.h"
#include "errorHandler.h"
#include "serviceReady.h"
#include "keyHandler.h"
#include "deviceHandler.h"
#include "awsDataManager.h"
#include "taskError.h"
#include "taskFirmwareUpdate.h"
#include "taskPowerOff.h"


#define FIRST_AWS 0 // 부팅연결 시퀀스 옵션 0: station -> aws, 1: aws ->station

typedef enum
{
    POWEROFF_VOID,
    POWEROFF_INIT,
    POWEROFF_READY,
    POWEROFF_READY_DONE,

} E_POWEROFF_STEP;


class CServiceManager : public CErrorHandler
{
private:
    /* data */
    CRsuContext*    pContext;
    
    CLocation    *pLocation;
    CKidnap      *pKidnap;    

    CServiceIdle*       pServcieIdle;
    CServiceClean*      pServiceClean;
    CServiceExplorer*   pServiceExplore;
    CServiceDocking*    pServiceDocking;
    CSupplyWater*       pSupplyWater;    
    CServiceUnDocking*  pServiceUnDocking;
    CServiceCharging*   pServiceCharging;
    CServiceWifi*       pServiceWifi;

    CServiceReady*      pServiceReady;

    CTaskError          taskError;
    CTaskFirmwareUpdate taskFwUpdate;

    //로봇의 PASUE STATE 자기 위치 좌표
    CRobotSlamPose* pRobotSlamPose;
    CKeyHanler* pKeyHandler;
    CDeviceHandler* pDeviceHandler;

    CAwsDataManager* pAwsDataManager;
    CServiceMacro svcMacro;
    CTaskPowerOff taskPowerOff;
    
    bool bRunPowerOff;
    bool bRunFirmwareUpdate;

    tCheckTime checkTime;
    tCheckTime checkSlamTime;
    
public:
    CServiceManager();
    ~CServiceManager();

    void updateServiceManager(tErrorInfo errInf);
    void timeProcess(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus,u16 time);

private:
    void procService();
    void procAwsReport();
    //void keyCheckerPattern();

private: /* Error Handler 오버라이딩 */
    E_ERROR_HANDLING_TYPE makeError(tErrorInfo errInf) override;
    void handleContinuousCliffError() override; // 연속낙하 감지 처리
    void handleTiltError() override;
    void handleMissingRosDataError() override;
    void handleCommunicationError() override;
    void handleDockingFailError() override;
    void handleWheelError() override;
    void handleTrapError() override;

private: 
    
/*디버깅 로그 함수 __debugPrint {상태, 데이터} */
    void __debugPrint();
    void printSysState();
    void printSysData();
    void printAppData();
    void printLidarData();
    void printPoseData();//FOR IMU STATE, IMU POSE DATA, SLAM POSE DATA
    void printThreadAlive();
    
    std::string batteryColor();
/**********************/

    // 시스템 체크
    void systemInterfaceErrorCheck();

    //부팅 초기화
    void bootService();
    // 서비스 전환 초기화
    void clearAllService();
    void initService(E_SERVICE_ID id);
    void serviceSelector(E_SERVICE_ID id);
    void runServiceMacro(const std::list<contextDo>& doList);
    void serviceStart(E_SERVICE_ID doId, E_SERVICE_STATUS doStatus);
    void serviceRun();
    void servicePause();
    void ServiceCancle();
};
