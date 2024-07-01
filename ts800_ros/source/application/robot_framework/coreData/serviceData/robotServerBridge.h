#pragma once

#include "eblog.h" 
#include "coreData/observer.h"

class CRobotServerBridge : public CObserver
{
private:
    bool awsConnection = false; //AWS 연결 됐는지 확인하는 변수
    bool robotConnection; //핸드폰 연결
    bool actionFlag;   // aws action 데이터가 업데이트 됐는지 확인하는 함수
    short actionCmd;
    
    short soundCmd;
    short dryEnabledCmd;
    short dryPowerCmd;
    short dryHoursCmd;

    short waterLvCmd;
    short cleanModeCmd;
    short cleanDurationCmd;
    short tiltCmd;
    short languageCmd;
    short countryCmd;

    short areaAllCmd;
    std::pair<tSpot*, short> areaSpotCmd;
    std::pair<tRoom*, short> areaRoomCmd;
    std::pair<tCustom*, short> areaCustomCmd;

    std::pair<tForbiddenLine*, short> forbiddenLineCmd;
    std::pair<tForbiddenRect*, short> forbiddenRectCmd;
    
    std::pair<tAreaInfo*, short> areaInfoCmd;
    std::pair<tDivideArea*, short> divideAreaInfoCmd;
    std::pair<tCombieArea*, short> combineAreaInfoCmd;

    short dontDisturbStatusCmd;
    char* dontDisturbStartTimeCmd;
    char* dontDisturbEndTimeCmd;
    
    std::pair<tCleanSchedule*, short> cleanScheduleCmd;

    //OTA
    short otaForceCmd;
    char* otaNameCmd;
    char* otaVersionCmd;
    short otaScheduledCmd;
    char* otaScheduleTimeCmd;

    bool wifiServiceFlag;

    char rsApValue;
    std::pair<char, char> rsApValues;
    char rsStationValue;
    std::pair<char, char> rsStationValues;
    bool rsIsAp;
    bool rsIsStation;
    char rsMode;
    bool rsHomeApConnection;
    E_AP_STAION_CONNECTION_STATE rsApConnection;
    E_AP_STAION_CONNECTION_STATE rsStationConnection;
    bool rsApExitState;
    bool rsStationExitState;

public:
    CRobotServerBridge();
    ~CRobotServerBridge();

    //이거는 externData에서 명령이 온건지만 체크하는 함수
    void update(CExternData* pExternData) override;

    /* AWS Action, state 관련 명령 */
    void setAction(short actionValue, bool flag);
    bool getconnect();
    bool getActionFlag();
    short getAction();

    /* AWS setting */
    short getSound();
    short getDryEnabled();
    short getDryPower();
    short getDryHours();
    short getWaterLv();
    short getCleanMode();
    short getCleanDuration();
    short getTilt();
    short getLanguage();
    short getCountry();

    /* operation Area */
    short getAreaAll();
    std::pair<tSpot*, short> getAreaSpot();
    std::pair<tRoom*, short> getAreaRoom();
    std::pair<tCustom*, short> getAreaCustom();

    /* forbiddenArea */
    std::pair<tForbiddenLine*, short> getForbiddenLine();
    std::pair<tForbiddenRect*, short> getForbiddenRect();
    
    /* areaInfo */
    std::pair<tAreaInfo*, short> getAreaInfo();
    std::pair<tDivideArea*, short> getDivideAreaInfo();
    std::pair<tCombieArea*, short> getCombineAreaInfo();
    
    /* doNotDisturb */
    short getDontDisturbStatus();
    char* getDontDisturbStartTime();
    char* getDontDisturbEndTime();
    
    /* cleaningSchedule */
    std::pair<tCleanSchedule*, short> getCleanSchedule();

    /* OTA */
    short getOtaForce();
    char* getOtaName();
    char* getOtaVersion();
    short getOtaScheduled();
    char* getOtaScheduleTime();

    /* phone 관련 데이터 get */
    void setWifiServiceFlag(bool OnOff);
    char getrsApValue();
    std::pair<char, char> getrsApValues();
    char getrsStationValue();
    std::pair<char, char> getrsStationValues();
    bool getrsIsAp();
    bool getrsIsStation();

    //포인터 생성여부 및 현재 모드 상태
    char getrsMode();
    //연결 여부 상태확인
    bool getrsHomeApConnection();

    //연결 여부
    E_AP_STAION_CONNECTION_STATE getApConnection(); 
    E_AP_STAION_CONNECTION_STATE getStationConnection(); 

    bool getApExit();
    bool getStationExit();
};

