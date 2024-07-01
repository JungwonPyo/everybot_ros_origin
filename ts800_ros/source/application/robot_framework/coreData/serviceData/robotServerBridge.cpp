#include "externData/externData.h"
#include "robotServerBridge.h"
#include "externData/appData.h"

#include <iostream>
#include <cstring> // strcpy 함수를 사용하기 위해 필요
#include <cstdlib> // malloc, free 함수를 사용하기 위해 필요

CRobotServerBridge::CRobotServerBridge() {}
CRobotServerBridge::~CRobotServerBridge() {}


void CRobotServerBridge::update(CExternData* pExternData)
{
    awsConnection = pExternData->appData.getAwsConnection(); 
    //ceblog(LOG_LV_AWS, BOLDCYAN, "robotServer bridge");

    //action은 keystate에서 처리하기 때문에 여기서는 flag만 필요함 
    if(pExternData->appData.getApp() != 0) //나중에 getAppCmd()로 해봐야함 
    {
        // 옵저버에서 에 CRobotServerBridge가 업데이트 후에 keyState가 getappCmd()를 호출하면 다음과 같은 문제가 발생
        // 브릿지에서 안됨 -> 키처리-> 서비스 작동 -> 플래그안되어서 로봇이 직접했다고 판단
        actionFlag = true;
        //eblog(LOG_LV_AWS, "actionFlag : "<<actionFlag);
    } 
#if 0 
    short sound = pExternData->appData.getSoundCmd();
    if(sound != 0)     
    {
        //soundFlag = true;
        soundCmd = sound;
        eblog(LOG_LV_AWS, "soundCmd : "<<soundCmd);
    }   
    short waterLv = pExternData->appData.getWaterLvCmd();
    if( waterLv != 0)
    {
        waterLvCmd = waterLv;
        eblog(LOG_LV_AWS, "waterLvCmd : "<<waterLvCmd);
    }    
    short cleanMode = pExternData->appData.getCleanModeCmd();
    if( cleanMode != 0)
    {
        cleanModeCmd = cleanMode;
        eblog(LOG_LV_AWS, "cleanModeCmd : "<<cleanModeCmd);
    }     
    short duration = pExternData->appData.getCleanDurationCmd();
    if( duration != 0)     
    {
        cleanDurationCmd = duration;
        eblog(LOG_LV_AWS, "cleanDurationCmd"<<cleanDurationCmd);
    }
    short tilt = pExternData->appData.getTiltCmd();
    if( tilt != 0)
    {
        tiltCmd = tilt;
        eblog(LOG_LV_AWS, "tiltCmd : "<<tiltCmd);
    }
    short language = pExternData->appData.getLanguageCmd(); 
    if( language != 0)
    {
        languageCmd = language;
        eblog(LOG_LV_AWS, "languageCmd : "<<languageCmd);
    }
    short country = pExternData->appData.getCountryCmd();
    if( country != 0)
    {
        countryCmd = country;
        eblog(LOG_LV_AWS, "countryCmd : "<<countryCmd);
    }
#else // 한 번 호출 후에 0값 리턴
    soundCmd = pExternData->appData.getSoundCmd();
    waterLvCmd = pExternData->appData.getWaterLvCmd();
    cleanModeCmd = pExternData->appData.getCleanModeCmd();
    
    languageCmd = pExternData->appData.getLanguageCmd();
    countryCmd = pExternData->appData.getCountryCmd();

    dryEnabledCmd = pExternData->appData.getDryEnabledCmd();
    dryPowerCmd = pExternData->appData.getDryPowerCmd();
    dryHoursCmd = pExternData->appData.getDryHoursCmd();

    areaAllCmd = pExternData->appData.getAreaAllCmd();
    areaSpotCmd = pExternData->appData.getAreaSpotCmd();
    areaRoomCmd = pExternData->appData.getAreaRoomCmd();
    areaCustomCmd = pExternData->appData.getAreaCustomCmd();

    forbiddenLineCmd = pExternData->appData.getForbiddenLineCmd();
    forbiddenRectCmd = pExternData->appData.getForbiddenRectCmd();

    //areaInfoCmd = pExternData->appData.getAreaInfoCmd();
    divideAreaInfoCmd = pExternData->appData.getDivideAreaInfoCmd();
    combineAreaInfoCmd = pExternData->appData.getCombineAreaInfoCmd();
    
    dontDisturbStatusCmd = pExternData->appData.getDontDisturbStatusCmd();
    dontDisturbStartTimeCmd = pExternData->appData.getDontDisturbStartTimeCmd();
    dontDisturbEndTimeCmd = pExternData->appData.getDontDisturbEndTimeCmd();

    cleanScheduleCmd = pExternData->appData.getCleanScheduleCmd();

    otaForceCmd = pExternData->appData.getOtaForceCmd();
    otaNameCmd = pExternData->appData.getOtaNameCmd();
    otaVersionCmd = pExternData->appData.getOtaVersionCmd();
    otaScheduledCmd = pExternData->appData.getOtaScheduledCmd();
    otaScheduleTimeCmd = pExternData->appData.getOtaScheduleTimeCmd();
    
    rsStationConnection = pExternData->appData.checkStationConnection();
    rsMode = pExternData->appData.mode();
    
    //phone interface 관련 데이터
    if(wifiServiceFlag == true)
    {
        rsApValue = pExternData->appData.getApValue();
        rsApValues = pExternData->appData.getApValues();
        rsStationValue = pExternData->appData.getStationValue();
        rsStationValues = pExternData->appData.getStationValues();

        rsIsAp = pExternData->appData.isConnectingAP();
        rsIsStation = pExternData->appData.isConnectingStation();

        //포인터 생성 여부
        rsHomeApConnection = pExternData->appData.connection();
        
        //연결 여부
        // rsApConnection = pExternData->appData.checkApConnection();
        //종료 상태
        rsApExitState = pExternData->appData.checkApExitState();
        rsStationExitState = pExternData->appData.checkStationExitState();
    }
#endif
}


/*------------------------- AWS Action, state 관련 명령 --------------------------------*/

void CRobotServerBridge::setAction(short actionValue, bool flag)
{
/*     if ( flag == false)
    {
        pExternData->appData.transAction( actionValue, E_UPDATE_FLAG::REPORT);
    }
    else if (flag == true)
    {
        pExternData->appData.transAction( actionValue, E_UPDATE_FLAG::UPDATE);
    } */
}

bool CRobotServerBridge::getconnect(){return awsConnection;}
// 검증 되면 삭제
short CRobotServerBridge::getAction(){return actionCmd;}
 
bool CRobotServerBridge::getActionFlag()
{
    bool tempFlag = actionFlag;
    actionFlag = false;
    return tempFlag;
}

short CRobotServerBridge::getSound(){return soundCmd;}

short CRobotServerBridge::getDryEnabled(){return dryEnabledCmd;}
short CRobotServerBridge::getDryPower(){return dryPowerCmd;}
short CRobotServerBridge::getDryHours(){return dryHoursCmd;}

short CRobotServerBridge::getWaterLv(){return waterLvCmd;}
short CRobotServerBridge::getCleanMode(){return cleanModeCmd;}
short CRobotServerBridge::getCleanDuration(){return cleanDurationCmd;}
short CRobotServerBridge::getLanguage(){return languageCmd;}
short CRobotServerBridge::getCountry(){return countryCmd;}

short CRobotServerBridge::getAreaAll(){return areaAllCmd;}
std::pair<tSpot*, short> CRobotServerBridge::getAreaSpot(){return areaSpotCmd;}
std::pair<tRoom*, short> CRobotServerBridge::getAreaRoom(){return areaRoomCmd;}
std::pair<tCustom*, short> CRobotServerBridge::getAreaCustom(){return areaCustomCmd;}

std::pair<tForbiddenLine*, short> CRobotServerBridge::getForbiddenLine() {return forbiddenLineCmd;}
std::pair<tForbiddenRect*, short> CRobotServerBridge::getForbiddenRect() {return forbiddenRectCmd;}

std::pair<tAreaInfo*, short> CRobotServerBridge::getAreaInfo(){return areaInfoCmd;}
std::pair<tDivideArea*, short> CRobotServerBridge::getDivideAreaInfo(){return divideAreaInfoCmd;}
std::pair<tCombieArea*, short> CRobotServerBridge::getCombineAreaInfo(){return combineAreaInfoCmd;}

short CRobotServerBridge::getDontDisturbStatus(){return dontDisturbStatusCmd;}
char* CRobotServerBridge::getDontDisturbStartTime(){return dontDisturbStartTimeCmd;}
char* CRobotServerBridge::getDontDisturbEndTime(){return dontDisturbEndTimeCmd;}

std::pair<tCleanSchedule*, short> CRobotServerBridge::getCleanSchedule() {return cleanScheduleCmd;}

//------------------OTA-------------------------------------
short CRobotServerBridge::getOtaForce(){return otaForceCmd;}
char* CRobotServerBridge::getOtaName(){return otaNameCmd;}
char* CRobotServerBridge::getOtaVersion(){return otaVersionCmd;}
short CRobotServerBridge::getOtaScheduled(){return otaScheduledCmd;}
char* CRobotServerBridge::getOtaScheduleTime(){return otaScheduleTimeCmd;}

//------------------------------BTN CONNECTTION---------------------------------
void CRobotServerBridge::setWifiServiceFlag(bool OnOff)
{
    wifiServiceFlag = OnOff;
}

char CRobotServerBridge::getrsApValue(){return rsApValue;}
std::pair<char, char> CRobotServerBridge::getrsApValues(){return rsApValues;}
char CRobotServerBridge::getrsStationValue(){return rsStationValue;}
std::pair<char, char> CRobotServerBridge::getrsStationValues(){return rsStationValues;}
bool CRobotServerBridge::getrsIsAp(){return rsIsAp;}
bool CRobotServerBridge::getrsIsStation(){return rsIsStation;}
/**
 * @brief 
 * 
 * @return char 0 : 생성 된 포인터가 없다
 *         char 1 : ap mode
 *         char 2 : station mode 
 */
char CRobotServerBridge::getrsMode(){return rsMode;}
bool CRobotServerBridge::getrsHomeApConnection(){return rsHomeApConnection;}
E_AP_STAION_CONNECTION_STATE CRobotServerBridge::getApConnection(){return rsApConnection;}
E_AP_STAION_CONNECTION_STATE CRobotServerBridge::getStationConnection(){return rsStationConnection;}
bool CRobotServerBridge::getApExit(){return rsApExitState;}
bool CRobotServerBridge::getStationExit(){return rsStationExitState;}

