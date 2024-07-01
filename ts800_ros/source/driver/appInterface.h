/**
 * @file appInterface.h
 * @author jspark
 * @brief app interface 입니다.
 * 
 * @version 0.1
 * @date 2023-08-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <iostream>
#include <thread>
#include "interfaceStruct.h"
#include "commonStruct.h"
#include "gridmap.h"

struct tTrajectorySendBuf
{
    tTrajectorySendBuf() : x(0.0), y(0.0) {}
    double x = 0.0, y = 0.0;
};

typedef struct _ConnectData
{
    char ssid[128];
    char pw[128];    
    char ip[64]; 
    char router[64]; 
    char dns1[64]; 
    char dns2[64];
    char netmask[64];
    char gateway[64];
    char network[64];
    char broadcast[64];
}ConnectData;
typedef enum 
{
    NO_DELETE,

    MAP_TRAJ,
    HISTORY_TRAJ,
    SAVEMAP_TRAJ,

    OPERATION_SPOT,
    OPERATION_ROOM,
    OPERATION_CUSTOM,

    FORBIDDEN_LINE,
    FORBIDDEN_RECT,

    AREA_INFO,

}E_DELETE_MEMORY_SORT;

class CAppInterface
{
private:
    short length1, length2;
    char code[256], desc[256], ap[128], robotIp[64];
    char cpyCleanStartTime[256], cpyExitReason[128], cpyCleanAreaInfo[128];
    char cpyUniqueKey[128], cpyMapName[128], cpyExplorerAreaInfo[128]; 
    double robotPoseX=0.0, robotPoseY=0.0;   
    double trajPoint[1024];
    u32 mapCount = 0;
    u32 trajCount = 0;
    u32 poseCount = 0;
    u32 mapPoseTraj = 0;
    tTrajectorySendBuf* transMapSendTrajData;
    bool bDeltransMapSendTraj;
    bool bDeltransHistoryTraj;

    char sendStartTime[16], sendEndTime[16];

    char factroyResetStartTime[256], factoryResetDiscript[256];

    //디버깅용 변수
    int i=0, debug = 0;

    tSpot spot[MAXLINE];
    tRoom room[MAXLINE];
    tCustom custom[MAXLINE];

    tForbiddenRect rect[MAXRECT];
    tForbiddenLine line[MAXLINE];
    tAreaInfo infoArea[MAXRECT];
    tDivideArea divideArea[MAXRECT];
    tCombieArea combineArea[MAXRECT];
    //tAreaInfo sendInfoArea[MAXRECT];
    // 아직
    tCleanSchedule scheduleClean[MAXRECT];

    //OTA
    short isForce;
    char name[256];
    char version[256];
    short isScheduled;
    char scheduleTime[256];

    ConnectData wifiConnectData;

    char apValue;
    std::pair<char, char> apValues;
    char stationValue;
    std::pair<char, char> stationValues;

    bool bThreadRunning;
    std::thread thLoop;
    void threadLoop();
    void receiveData();
    void receiveOtaData();

    bool boottingAwsTogle;
    double boottingStartTime;
    bool apStationExitTrigger;
    float tick=0 ;
    float startTime =0;

public:
    tUpdateData<short> action;
    tUpdateData<short> status;
    tUpdateData<short> sound;
    tUpdateData<short> dryEnabled;
    tUpdateData<short> dryHours;
    tUpdateData<short> dryPower;
    tUpdateData<short> waterLv;
    tUpdateData<short> cleanMode;
    tUpdateData<short> cleanDuration;
    tUpdateData<short> tilt;
    tUpdateData<short> language;
    tUpdateData<short> country;

    tUpdateData<short> operationAreaAll;
    tUpdatePairData<tSpot> operationAreaSpot;
    tUpdatePairData<tRoom> operationAreaRoom;
    tUpdatePairData<tCustom> operationAreaCustom;

    tUpdatePairData<tForbiddenLine> forbiddenLine;
    tUpdatePairData<tForbiddenRect> forbiddenRect;

    tUpdatePairData<tAreaInfo> areaInfo;
    tUpdatePairData<tDivideArea> divideAreaInfo;
    tUpdatePairData<tCombieArea> combineAreaInfo;

    tUpdateData<short> dontDisturbStatus;
    tUpdateBuffer<char,16> dontDisturbStartTime;
    tUpdateBuffer<char,16> dontDisturbEndTime;

    tUpdatePairData<tCleanSchedule> cleanSchedule;
    
    //OTA
    tUpdateData<short> otaForce;
    tUpdateBuffer<char,256> otaName;
    tUpdateBuffer<char,256> otaVersion;
    tUpdateData<short> otaScheduled;
    tUpdateBuffer<char,256> otaScheduleTime;

public:
    CAppInterface();
    ~CAppInterface();
    void bootingConnect();
    void initConnectData();
    
//-------------------device_connection.h-------------------
    void transPhoneInterface(E_PHONE_INTERFACE_TYPE type);

    // AP 관련 (phone - robot)
    void connectAP();
    void disconnectAP();
    bool isConnectingAP(); //

    void apState(void); //fw쪽 상태(ap, station)와 phone과의 연결 상태를 리턴해주는 함수
    void apStates(void); //리턴 타입만 다른것 아마도?
    
    // Station 관련(HomeAp 공유기 - robot)
    void connectStation(E_CONNECT_SORT connect);
    void disconnectStation();
    bool isConnectingStation(); //객체를 생성 성공여부 리턴
    
    void stationState(void);
    void stationStates(void);
    bool connection(void);      //station 연결여부 리턴

    //AP와 Station 공통 부분
    char mode(void);
    E_AP_STAION_CONNECTION_STATE checkApConnection();
    E_AP_STAION_CONNECTION_STATE checkStationConnection();
    bool delApStationPtr();
    void checkMemory();

    void handleInterruptedConnection();

//-----------Robot-Server bridge에서 update하는 데이터--------
    char getApValue();
    std::pair<char, char> getApValues();
    char getStationValue();
    std::pair<char, char> getStationValues();

    bool checkApExitState();
    bool checkStationExitState();
//------------------device_aws_iot.h-----------------------
    //AWS 관련
    void connectAWS();
    void disconnectAWS();
    bool isConnectingAWS();

    //상태 및 명령에 대한 보고
    void transAction(short actionValue);
    void transStatus(short statusValue);
    void transFactoryReset(char* startTime, char* descript);
    void transWaterLevel(short waterLv);
    void transSoundLevel( short soundLv);
    void transDryMopData(short dryEnabled, short dryHours, short dryPower);
    void transCleanMode(short cleanMode);
    void transLanguage(short language);
    void transCountry(short country);
    void transErrorInfo(std::string errorCode, std::string errorDesc);
    void transBatteryPercent(short battery);
    void transSetting(std::string apVersion, std::string mcuVersion);
  
    void transMapData(u8* mapData, tGridmapInfo mapInfo);
    void transRobotPose(tPose robotPose);
    void transTrajectory(std::list<tPoint> traj);
    void transCradlePose(tPoint cradleCoord);
    void transTotalMapInfo(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, tPoint cradleCoord, double cleanSize);
        
    void transSavedMapData(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, std::string uniqueKey, std::string mapName, int order, std::string areaInfo);
    void transHistoryData(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, tPoint cradlePose, std::string cleanStartTime, std::string exitReason, double cleanedSize, int cleanTime, std::string areaInfo);



    void transCleanAll(short all);
    void transCleanSpot(std::list<tSpot> spot, short spotNumber);
    void transCleanRoom(std::list<tRoom> room, short roomNumber);
    void transCleanCustom(std::list<tCustom> custom, short customNumber);

    void transOperationArea(short cleanType, short all, std::list<tSpot> spot, short spotNumber, std::list<tRoom> room, short roomNumber, std::list<tCustom> custom, short customNumber);
    
    void transForbiddenLine(std::list<tForbiddenLine> line, short lineNumber);
    void transForbiddenRect(std::list<tForbiddenRect> rect, short rectNumber);

    void transForbiddenArea(short forbiddenType, std::list<tForbiddenLine> line, short lineNumber, std::list<tForbiddenRect> rect, short rectNumber);

    void transAreaInfo(std::list<tAreaInfo>info, short areaNum);
    void transDivideArea(tDivideArea data);
    void transCombineArea(tCombieArea data);
    void transDoNotDisturb(short status, char* startTime, char* endTime);
    void transCleaningSchedule(std::list<tCleanSchedule> schedule, short  scheduleNum);

    void transOtaVersion(char * _version);

};
