#pragma once

#include <iostream>
#include <string>
#include "eblog.h" 
#include "commonStruct.h"
#include "gridmap.h"

typedef struct _tAwsWaterLv
{
    _tAwsWaterLv() {}
    _tAwsWaterLv(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsWaterLv;

typedef struct _tAWSSoundLv
{
    _tAWSSoundLv() {}
    _tAWSSoundLv(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsSoundLv;

typedef struct _tAwsCleanMode
{
    _tAwsCleanMode() {}
    _tAwsCleanMode(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsCleanMode;

typedef struct _tAwsLanuage
{
    _tAwsLanuage() {}
    _tAwsLanuage(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsLanuage;

typedef struct _tAwsCountry
{
    _tAwsCountry() {}
    _tAwsCountry(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsCountry;

typedef struct _tAwsBattery
{
    _tAwsBattery() {}
    _tAwsBattery(bool _bUpdate,short _value) : bUpdate(_bUpdate),value(_value){}
    
    bool bUpdate;
    short value;

}tAwsBattery;

typedef struct _tAwsDryOption
{
    _tAwsDryOption() {}
    _tAwsDryOption(bool _bUpdate, short _dryEnabled, short _dryHours, short _dryPower) : bUpdate(_bUpdate),dryEnabled(_dryEnabled), dryHours(_dryHours), dryPower(_dryPower) {}

    bool bUpdate;  
    short dryEnabled;
    short dryHours;
    short dryPower;
    
}tAwsDryOption;

typedef struct _tAwsError
{
    _tAwsError() {}
    _tAwsError(bool _bUpdate, std::string _errorCode, std::string _errorDesc) : bUpdate(_bUpdate), errorCode(_errorCode), errorDesc(_errorDesc) {}

    bool bUpdate; 
    std::string errorCode;
    std::string errorDesc;
    
}tAwsError;

typedef struct _tAwsSettings
{
    _tAwsSettings() {}
    _tAwsSettings(bool _bUpdate, std::string _apVersion,std::string _mcuVersion) : 
    bUpdate(_bUpdate), apVersion(_apVersion),mcuVersion(_mcuVersion){}

    bool bUpdate;
    // std::string ssid;
    // short rssi;
    // std::string serialNum;
    std::string apVersion;
    std::string mcuVersion;
    //std::string wifiVersion;
    
}tAwsSettings;

typedef struct _tAwsOperationArea
{
    _tAwsOperationArea() {}
    _tAwsOperationArea(bool _bUpdate, short _all, std::list<tSpot> _spot, short _spotNumber,std::list<tRoom> _room, short _roomNumber,std::list<tCustom> _custom, short _customNumber) : 
    bUpdate(_bUpdate),all(_all), spot(_spot), spotNumber(_spotNumber), room(_room),roomNumber(_roomNumber),custom(_custom),customNumber(_customNumber){}
    
    bool bUpdate;
    short type;
    short all;
    std::list<tSpot> spot;
    short spotNumber;
    std::list<tRoom> room;
    short roomNumber;
    std::list<tCustom> custom;
    short customNumber;

}tAwsOperationArea;

typedef struct _tAwsForbiddenArea
{
    _tAwsForbiddenArea() {}
    _tAwsForbiddenArea(bool _bUpdate, std::list<tForbiddenLine> _line, short _lineNumber,std::list<tForbiddenRect> _rect, short _rectNumber) : 
    bUpdate(_bUpdate),line(_line), lineNumber(_lineNumber), rect(_rect), rectNumber(_rectNumber){}
    
    bool bUpdate;
    short type;
    std::list<tForbiddenLine> line;
    short lineNumber;
    std::list<tForbiddenRect> rect;
    short rectNumber;

}tAwsForbiddenArea;

typedef struct _tAwsDoNotDistrupt
{
    _tAwsDoNotDistrupt() {}
    _tAwsDoNotDistrupt(bool _bUpdate, short _status) : bUpdate(_bUpdate),status(_status) {}
    
    bool bUpdate;
    short status;
    char* startTime;
    char* endTime;

}tAwsDoNotDistrupt;
typedef struct _tAwsReservationClean
{
    _tAwsReservationClean() {}
    _tAwsReservationClean(bool _bUpdate, std::list<tCleanSchedule> _schedule, short _scheduleNumber) :
    bUpdate(_bUpdate),schedule(_schedule), scheduleNumber(_scheduleNumber) {}
  
    bool bUpdate;
    std::list<tCleanSchedule> schedule;
    short scheduleNumber;

}tAwsReservationClean;

typedef struct _tAwsMapData
{
    _tAwsMapData() {}
    _tAwsMapData(bool _bUpdate, u8* _mapData, tGridmapInfo _mapInfo, tPose _robotPose, std::list<tPoint> _traj, tPoint _cradleCoord, int _cleanSec, double _cleanSize) :
    bUpdate(_bUpdate),mapData(_mapData), mapInfo(_mapInfo), robotPose(_robotPose),traj(_traj), cradleCoord(_cradleCoord), cleanSec(_cleanSec), cleanSize(_cleanSize){}
  
    bool bUpdate;
    u8* mapData;
    tGridmapInfo mapInfo;
    tPose robotPose;
    std::list<tPoint> traj;
    tPoint cradleCoord;
    int cleanSec;
    double cleanSize;

}tAwsMapData;

typedef struct _tAwsAreaInfo
{
    _tAwsAreaInfo() {}
    _tAwsAreaInfo(bool _bUpdate, std::list<tAreaInfo> _info, short _areaNumber) : bUpdate(_bUpdate),info(_info), areaNumber(_areaNumber){}
    bool bUpdate;
    std::list<tAreaInfo> info;
    short areaNumber;
}tAwsAreaInfo;

typedef struct _tAwsDivideArea
{
    _tAwsDivideArea() {}
    _tAwsDivideArea(bool _bUpdate, tDivideArea _data) : bUpdate(_bUpdate),data(_data){}
    bool bUpdate;
    tDivideArea data;

}tAwsDivideArea;

typedef struct _tAwsCombineArea
{
    _tAwsCombineArea() {}
    _tAwsCombineArea(bool _bUpdate, tCombieArea _data) : bUpdate(_bUpdate),data(_data){}
    bool bUpdate;
    tCombieArea data;
}tAwsCombineArea;

typedef struct _tAwsSaveMapInfo
{
    _tAwsSaveMapInfo() {}
    _tAwsSaveMapInfo(bool _bUpdate, u8* _mapData, tGridmapInfo _mapInfo, tPose _robotPose, std::list<tPoint> _traj, std::string _uniqueKey, std::string _mapName, int _order, std::string _areaInfo) :
    bUpdate(_bUpdate),mapData(_mapData), mapInfo(_mapInfo), robotPose(_robotPose), traj(_traj), uniqueKey(_uniqueKey), mapName(_mapName), order(_order), areaInfo(_areaInfo){}
    
    bool bUpdate;
    u8* mapData;
    tGridmapInfo mapInfo;
    tPose robotPose;
    std::list<tPoint> traj;
    std::string uniqueKey;
    std::string mapName;
    int order;
    std::string areaInfo;
}tAwsSaveMapInfo;

typedef struct _tAwsCleanHistory
{
    _tAwsCleanHistory() {}
    _tAwsCleanHistory(bool _bUpdate, u8* _mapData, tGridmapInfo _mapInfo, tPose _robotPose, std::list<tPoint> _traj, tPoint _cradlePose, std::string _cleanStartTime, std::string _exitReason, int _cleanedSize, int _cleanTime, std::string _areaInfo) :
    bUpdate(_bUpdate),mapData(_mapData), mapInfo(_mapInfo), robotPose(_robotPose), traj(_traj), cradlePose(_cradlePose), cleanStartTime(_cleanStartTime), cleanedSize(_cleanedSize), cleanTime(_cleanTime), areaInfo(_areaInfo){}

    bool bUpdate;
    u8* mapData;
    tGridmapInfo mapInfo;
    tPose robotPose;
    std::list<tPoint> traj;
    tPoint cradlePose;
    std::string cleanStartTime;
    std::string exitReason;
    int cleanedSize;
    int cleanTime;
    std::string areaInfo;
}tAwsCleanHistory;

typedef struct _tAwsOta
{
    _tAwsOta() {}

    bool bUpdate;
    short force;
    char name[256];
    char version[256];
    short scheduled;
    char scheduleTime[256];
    bool isUpdate;

}tAwsOta;
typedef struct _tAwsRobotPose
{
    tPose robotPose;
}tAwsRobotPose;
typedef struct _tAwsData
{
  E_AWS_MSG_SORT sort;  
  tAwsWaterLv waterLv;
  tAwsSoundLv soundLv;
  tAwsCleanMode cleanMode;
  tAwsLanuage language;
  tAwsCountry country;
  tAwsBattery battery;
  tAwsDryOption dryData;
  tAwsError error;
  tAwsSettings setting;
  tAwsOperationArea operationArea;
  tAwsForbiddenArea forbiddenArea;
  tAwsDoNotDistrupt distruptData;
  tAwsReservationClean rsvCleanData;
  tAwsSaveMapInfo saveMapInfo;
  tAwsMapData mapData;
  tAwsRobotPose robotPose;
  tAwsAreaInfo areaInfo;
  tAwsDivideArea divideArea;
  tAwsCombineArea combineArea;
  tAwsCleanHistory cleanHistory;
  tAwsOta otaInfo;

}tAwsData;


class CStoreAwsData
{
private:

    tAwsData   data;

public:
    CStoreAwsData();
    ~CStoreAwsData();

    bool mapTrigger;
    E_AWS_MSG_SORT getDataSort();
    /* RobotData */
    void setSendWaterLv(E_WATER_PUMP_STEP lv);
    short getSendWaterLv();
    bool isWaterLevelUpdate();
    
    void setSendSoundLv(short lv);
    short getSendSoundLv();
    bool isSoundVolumeUpdate();
    
    void setSendDryEnabled(short dryEnabled);
    short getSendDryEnabled();
    void setSendDryPower(short dryPower);
    short getSendDryPower();
    void setSendDryHours(short dryHours);
    short getSendDryHours();
    bool isDryMopDataUpdate();
#if 0
    void setSendDryOption(short dryEnabled, short dryHours, E_DRYFAN_LEVEL dryPower);
    short getSendDryOption(short sort);
#endif

    void setSendCleanMode(short mode);
    short getSendCleanMode();
    bool isCleanModeUpdate();

    void setSendLanguage(short language);
    short getSendLanguage();
    bool isLanguageDataUpdate();

    void setSendCountry(short country);
    short getSendCountry();
    bool isCountryDataUpdate();

    void setSendError(E_ERROR_HANDLING_TYPE errType);
    std::string getSendErrorCode();
    std::string getSendErrorDesc();
    bool isErrorDataUpdate();

    void setSendBattery(short _battery);
    short getSendBattery();
    bool isBatteryUpdate();

    /* Robot Info */
    void setSendSSID(std::string _ssid);
    std::string getSendSSID();
    void setSendRSSI(short rssi);
    short getSendRSSI();
    void setSendSerialNum( std::string serialNum);
    std::string getSendSerialNum();
    void setSendApVersion( std::string apVersion);
    std::string getSendApVersion();
    void setSendMcuVersion( std::string mcuVersion);
    std::string getSendMcuVersion();
    void setSendWifiVersion( std::string wifiVersion);
    std::string getSendWifiVersion();
    bool isSettingDataUpdate();


    /* operation Area */
    short getOperationAreaType();

    void setSendAreaAll();
    short getSendAreaAll();

    void setSendAreaSpot();
    std::list<tSpot> getSendAreaSpot();
    short getSendAreaSpotNum();
    
    void setSendAreaRoom();
    std::list<tRoom> getSendAreaRoom();
    short getSendAreaRoomNum();
    
    void setSendAreaCustom();
    std::list<tCustom> getSendAreaCustom();
    short getSendAreaCustomNum();
    bool isOperationAreaUpdate();

    /* forbiddenArea */
    short getSendForbiddenType();
    void setSendForbiddenLine();
    std::list<tForbiddenLine> getSendForbiddenLine();
    short getSendForbiddenLineNum();
    void setSendForbiddenRect();
    std::list<tForbiddenRect> getSendForbiddenRect();
    short getSendForbiddenRectNum();
    bool isForbiddenAreaUpdate();

    /* areaInfo */
    void setSendAreaInfo();
    void setSendDivideArea(tDivideArea *pData);
    tDivideArea getSendDivideArea();
    bool isDivideAreaUpdate();
    void setSendCombineArea(tCombieArea *pData);
    tCombieArea getSendCombineArea();
    bool isCombineAreaUpdate();

    std::list<tAreaInfo> getSendAreaInfo();
    short getSendAreaInfoNum();
    bool isAreaInfoUpdate();

    /* doNotDisturb */
    void setSendDontDisturbStatus();
    short getSendDontDisturbStatus();

    void setSendDontDisturbStartTime();
    char* getSendDontDisturbStartTime();

    void setSendDontDisturbEndTime();
    char* getSendDontDisturbEndTime();
    bool isDistruptDataUpdate();

    /* cleaningSchedule */
    void setSendCleanSchedule();
    std::list<tCleanSchedule> getSendCleaningSchedule();
    short getSendCleanScheduleNum();
    bool isRsvCleanDataUpdate();

    /* OTA */
    void setSendOtaForce();
    short getSendOtaForce();

    void setSendOtaName();
    char* getSendOtaName();

    void setSendOtaVersion();
    char* getSendOtaVersion();

    void setSendOtaScheduled();
    short getSendOtaScheduled();
    
    void setSendOtaScheduleTime();
    char* getSendOtaScheduleTime();
    bool isOtaDataUpdate();

    /* clean history */
    void setSendCleanAreaInfo(std::string areaInfo);
    void setSendCleanTraj(std::list<tPoint> traj);
    void setSendCleanRobotPose(tPose robotPose);
    void setSendCleanCradlePose(tPoint cradlePose);
    void setSendCleanStartTime(std::string cleanStartTime);
    void setSendCleanExitReason(std::string exitReason);
    void setSendCleanedSize(int cleanedSize);
    void setSendCleanTime(int cleanTime);
    
    tAwsCleanHistory getSendCleanHistory();
    bool isCleanHistoryUpdate();
    /* saveMap */
    void setSendSaveMapRobotPose(tPose robotPose);
    void setSendSaveMapTraj(std::list<tPoint> traj);
    void setSendSaveMapUniqueKey(std::string key);
    void setSendSaveMapName(std::string mapName);
    void setSendSaveMapOrder(int order);
    void setSendSaveMapAreaInfo(std::string areaInfo);
    
    tAwsSaveMapInfo getSendSaveMap();
    bool isSaveMapUpdate();
    
};

