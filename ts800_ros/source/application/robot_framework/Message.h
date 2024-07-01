/**
 * @file Message.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <unordered_map>
#include <boost/variant.hpp>
#include "ebtypedef.h"
#include "commonStruct.h"
#include <cstring>
#include <list>
#include "gridmap.h"
#include "coreData/serviceData/storeAwsData.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"

typedef enum
{
  E_MESSAGE_TYPE_VOID,
  E_MESSAGE_TYPE_CONTROL_WHEEL, // 휠 모터 제어 (선속도,각속도)
  E_MESSAGE_TYPE_CONTROL_PUMP,    //펌프 모터 제어
  E_MESSAGE_TYPE_CONTROL_TILT,    //틸팅 모터 제어
  E_MESSAGE_TYPE_CONTROL_DRYFAN,  //걸레 건조 모터 제어

  E_MESSAGE_TYPE_SOUND,
  E_MESSAGE_TYPE_DISPLAY,
  E_MESSAGE_TYPE_LED,
  E_MESSAGE_TYPE_SYSTEM_MODE,
  E_MESSAGE_TYPE_POWER,
  E_MESSAGE_TYPE_MCU_RESET,
  E_MESSAGE_TYPE_ERROR,

  E_MESSAGE_TYPE_INIT_SENSOR,  // 센서데이터 초기화 (IMU)
  E_MESSAGE_TYPE_INIT_MOVING,  // wheel motor data init
  E_MESSAGE_TYPE_SYSTEM_CTR,//시스템 컨트롤.

  E_MESSAGE_TYPE_CONTROL_SLAM,     //cartograpser pause, resume, exit, restart  
  E_MESSAGE_TYPE_ACTIVE_MODE,
  E_MESSAGE_TYPE_CHARGE_MODE,
  
  E_MESSAGE_TYPE_REPORT_ACTION,
  E_MESSAGE_TYPE_REPORT_STATUS,
  E_MESSAGE_TYPE_REPORT_FACTORY_RESET,
  E_MESSAGE_TYPE_REPORT_AWS_DATA,
  E_MESSAGE_TYPE_CONNECT_PHONE,

  E_MESSAGE_TYPE_OTA_COMMAND,

} E_MESSAGE_TYPE;

typedef enum
{
    VELOCITY,
    PWM,
  
}E_DRIVE_WHEEL_TYPE;

typedef enum
{
  E_SYSTEM_CTR_VOID,
  E_SYSTEM_CTR_MOTER_ON,
  E_SYSTEM_CTR_MOTER_OFF,
  E_SYSTEM_CTR_CLEAR_LOCALIZTION,
  E_SYSTEM_CTR_CLEAR_DISTANCE,
  E_SYSTEM_CTR_CLEAR_TOTAL_DISTANCE,
  E_SYSTEM_CTR_CLEAR_ANGLE,
  E_SYSTEM_CTR_SLEEP,
  
} E_SYSTEM_CTR;

typedef enum
{
  E_SYSTEM_POWER_VOID,
  E_SYSTEM_POWER_OFF,

} E_SYSTEM_POWER;

enum class E_SYSTEM_OTA_CMD
{
    VOID,
    OPEN_CLOSE,
    START,
};


struct tMsgCtrDynamicWheel
{
  tMsgCtrDynamicWheel() : seq{-1}, linearVelocity{0}, angularVelocity{0}, radius{0} {}
  int seq; // debug용 시퀀스 번호
  s32 linearVelocity;
  s32 angularVelocity;
  u32 radius;
};

struct tMsgCtrOldWheel
{
  tMsgCtrOldWheel() : seq{-1}, direction{0}, L_Speed{0}, R_Speed{0}, B_Speed{0}, duty{0}, bHeadingPid{false} {}
  int seq; // debug용 시퀀스 번호
  int direction;
  int L_Speed;
  int R_Speed;
  int B_Speed;
  int duty;
  bool bHeadingPid;
};

struct tMsgCtrWheel
{
   E_DRIVE_WHEEL_TYPE type;
   tMsgCtrDynamicWheel dynamic;
   tMsgCtrOldWheel     pwm;
};

typedef struct _tMsgCtrPump
{
  _tMsgCtrPump() : on{false}{} 
  bool on;

}tMsgCtrPump;


typedef struct _tMsgCtrTilt
{
  _tMsgCtrTilt() : control{E_TILTING_CONTROL::STOP}{}
  E_TILTING_CONTROL control;

}tMsgCtrTilt;

typedef struct _tMsgCtrSound
{
  _tMsgCtrSound() : stop{false},data{E_SoundClass::SOUND_BLEEP}{}

  bool stop;
  E_SoundClass data;

}tMsgCtrSound;

typedef struct _tMsgCtrDisplay
{
  _tMsgCtrDisplay() : stop{false},bCustom{false}, img{E_DisplayImageClass::BATTERY_000}, customImg{nullptr} {}
  bool stop;
  bool bCustom;
  E_DisplayImageClass img;
  char* customImg;

}tMsgCtrDisplay;

typedef struct _tMsgCtrLed
{
  u32 dir;
  u8 red;
  u8 green;
  u8 blue;
  u8 white;

}tMsgCtrLed;

typedef struct _tMsgCtrSystem
{
  _tMsgCtrSystem() : ctr{E_SYSTEM_CTR_VOID} {}
  _tMsgCtrSystem(E_SYSTEM_CTR _ctr) : ctr{_ctr} {}
  E_SYSTEM_CTR ctr;
}tMsgCtrSystem;

typedef struct _tMsgCtrDryfan
{
  _tMsgCtrDryfan() : on{false}, speed{0} {}
  _tMsgCtrDryfan(bool _on, u16 _speed) : on{_on}, speed{_speed} {}
  bool on;
  u16 speed;

}tMsgCtrDryfan;

typedef struct _tMsgCtrSlam
{
  _tMsgCtrSlam() : state{0} {}
  u8 state;

} tMsgCtrSlam;

typedef struct _tMsgPowerOff
{
  _tMsgPowerOff() : state{0} {}
  u8 state;

} tMsgPowerOff;

typedef struct _tMsgInterfacePhone
{
  _tMsgInterfacePhone() : type{E_PHONE_INTERFACE_TYPE::INTERFACE_VOID} {}
  _tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE _type) : type(_type) {}

  E_PHONE_INTERFACE_TYPE type;

}tMsgInterfacePhone;

typedef struct _tMsgError
{
  _tMsgError() : type{E_ERROR_TYPE::NO_ERROR} {}
  _tMsgError(E_ERROR_TYPE _type) : type{_type} {}

  E_ERROR_TYPE type;

}tMsgError;

typedef struct _tMsgReportAction
{
  _tMsgReportAction() {}
  _tMsgReportAction(short _actionValue) : actionValue(_actionValue){}

  short actionValue;
    
}tMsgReportAction;

typedef struct _tMsgFactoryReset
{
  _tMsgFactoryReset() {}
  //_tMsgFactoryReset(char _startTime[256], char _descript[256]) : startTime(_startTime),descript(_descript){}

  char startTime[256];
  char descript[256];
    
}tMsgFactoryReset;

typedef struct _tMsgReportStatus
{
  _tMsgReportStatus() {}
  _tMsgReportStatus(short _statusValue) : statusValue(_statusValue) {}
  
  short statusValue;
    
}tMsgReportStatus;
typedef struct _tMsgReportData
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
  tAwsAreaInfo areaInfo;
  tAwsDivideArea divideArea;
  tAwsCombineArea combineArea;
  tAwsCleanHistory cleanHistory;
  tAwsOta otaInfo;
  tAwsRobotPose robotPose;

}tMsgReportData;

typedef struct _tMsgOtaCmd
{
  _tMsgOtaCmd() : type{E_SYSTEM_OTA_CMD::VOID}, bOpen{false}, path{""} {}
  //_tMsgOtaCmd(E_SYSTEM_OTA_CMD _type,bool _bOpen, char* _path) : type{_type},bOpen{_bOpen},path{_path} {}

  E_SYSTEM_OTA_CMD type;
  bool bOpen;
  char path[256];

}tMsgOtaCmd;

//tMsgCtrWheel
class Message {
public:
    typedef boost::variant<tMsgCtrWheel, tMsgCtrPump, tMsgCtrTilt, tMsgCtrDryfan, 
                           tMsgCtrLed, tMsgCtrDisplay, tMsgCtrSound, tMsgCtrSystem, 
                           tMsgCtrSlam, tMsgPowerOff, tMsgError,tMsgInterfacePhone,
                           tMsgReportAction,tMsgReportStatus,tMsgReportData,tMsgFactoryReset,
                           tMsgOtaCmd> 
                           MsgVariant;

    Message(E_MESSAGE_TYPE _what, MsgVariant _arg) 
        : what(_what), arg(_arg) {}
    Message(E_MESSAGE_TYPE _what) 
        : what(_what) {}

    E_MESSAGE_TYPE what;
    MsgVariant arg;
   
    virtual ~Message() {}

    const static unsigned int MESSAGE_CLIENT_SOCKET_THREAD_CLOSED = 100;
};