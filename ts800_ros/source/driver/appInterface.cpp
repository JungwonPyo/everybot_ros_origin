

#include "appInterface.h"
#include "device_aws_iot.h"
#include "device_connection.h"
#include "eblog.h"
#include "coreData/serviceData.h"
#include "utils.h"
#include <systemTool.h>
#include "fileMng.h"


/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 1 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 10.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/
#define SEND_MAP_TRAJ_POSE_TOGETHER 1 // 1이면 한 번에 다 보냄
#define USE_RDP 1
CRobotToAWS *pAws = nullptr; // aws 통신 인터페이스
CRobotConnection *pAp = nullptr; // phone 통신 인터페이스
CRobotConnection *pStation = nullptr; // phone 통신 인터페이스

CAppInterface::CAppInterface(/* args */)
{
    CStopWatch debugSW;

#if USE_AWS_APP_INTERFACE > 0
    bThreadRunning = true;

    thLoop = std::thread(&CAppInterface::threadLoop, this);
    
#endif

    // TIME_CHECK_END(debugSW.getTime());
}

CAppInterface::~CAppInterface()
{
    CStopWatch debugSW;
    
    if ( thLoop.joinable() )
    {
        bThreadRunning = false;
        thLoop.join();
    }
    disconnectAWS();
    
    // TIME_CHECK_END(debugSW.getTime());
}
/**
 * @brief 처음에 연결시도하고 안되면 안함  --> 문제가 없을까? 2번 3번은 시도 해야할거 같은데....
 * 
 */
void CAppInterface::bootingConnect()
{
    boottingStartTime = SYSTEM_TOOL.getSystemTime();
    //Station (공유기) 연결
    connectStation(E_CONNECT_SORT::BOOTING_CONNECT);
}
/**
 * @brief 아직 만드는중 현재 기능없음
 * 
 */
void CAppInterface::initConnectData()
{
    //wifiConnectData.ssid = '';
    //wifiConnectData.pw = '';
    strncpy(wifiConnectData.ssid, "ebrd", sizeof(wifiConnectData.ssid) - 1);
    wifiConnectData.ssid[sizeof(wifiConnectData.ssid) - 1] = '\0'; // 널 종료 문자 추가

    strncpy(wifiConnectData.pw, "Everyb@t728", sizeof(wifiConnectData.pw) - 1);
    wifiConnectData.pw[sizeof(wifiConnectData.pw) - 1] = '\0'; // 널 종료 문자 추가

}
/**
 * @brief phone 연결 인터페이스 스레드
 * 3개의 파트로 형성 되어있음
 * 
 * 1. AWS 부팅 연결
 * 2. AWS 연결 후 데이터 수신
 * 3. AP, STATION 포인터 삭제 
 * 
 */
void CAppInterface::threadLoop()
{
    unsigned int sec = 0;
    startTime = get_system_time();
    while (bThreadRunning)
    {
        //float debugTime= (SYSTEM_TOOL.getSystemTick() - tick) * 10 ;
        int debugTime = get_system_time(startTime)*1000;
        ceblog(LOG_LV_AWS, BOLDCYAN, "time[ms] : "<< debugTime);
            

        /* 1. AWS 연결 후 데이터 수신 */
        if (isConnectingAWS() )
        {
            //ceblog((LOG_LV_AWS), BOLDCYAN, "AWSConnection - ok ");
            receiveData();

            if ( sec%10 == 0)
            {
                receiveOtaData();
            }
        }
        // else if(debugTime > 3){
        //     ceblog((LOG_LV_AWS), BOLDCYAN, "AWS Lost Connection!!");
        // }

        /* 3. AP, STATION 포인터 삭제 */
        if(apStationExitTrigger)
        {
            //포인터 삭제 함수
            apStationExitTrigger = delApStationPtr();
        }
        
        /* 메모리 관리 */
        checkMemory();

        //ceblog((LOG_LV_AWS | LOG_LV_AWS), BOLDMAGENTA, " AWS threadLoop");
        
        // tick = SYSTEM_TOOL.getSystemTick();
        startTime = get_system_time();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));        
        sec++;
    }
}
bool CAppInterface::delApStationPtr()
{
    auto ret = true;

    if(pAp != nullptr)
    {
        if(pAp->GetApEnd())
        {
            delete pAp;
            pAp = nullptr;
            ceblog(LOG_LV_AWS, BOLDCYAN, "pAp delete");

            ret = false;
        }
    }
    else if (pStation != nullptr)
    {
        if (pStation->GetStationEnd())
        {
            delete pStation;
            pStation = nullptr;
            ceblog(LOG_LV_AWS, BOLDCYAN, "pStation delete");

            ret = false;
        }
    }

    return ret;
}
/**
 * @brief 지도, 경로 등등 메모리 누수 확인하고 삭제해주는 함수
 * 
 */
void CAppInterface::checkMemory()
{
    /* 경로 */
#if SEND_MAP_TRAJ_POSE_TOGETHER
    if(pAws->Inf_State_Get_MapOnceDataPtr() && bDeltransMapSendTraj)
    {
        eblog(LOG_LV_AWS,"del traj data");
        delete[] transMapSendTrajData; 
        transMapSendTrajData = nullptr;
        bDeltransMapSendTraj = false; 
    }
#else
    if(pAws->Inf_State_Get_TrajectoryDataPtr() && bDeltransMapSendTraj)
    {
        eblog(LOG_LV_AWS,"del traj data");
        delete[] transMapSendTrajData; 
        transMapSendTrajData = nullptr;
        bDeltransMapSendTraj = false; 
    }
#endif
#if 0 // 이거 함수 만들어야함
    if(pAws->Inf_State_Get_TrajectoryDataPtr() && bDeltransHistoryTraj)
    {
        eblog(LOG_LV_AWS,"del traj data");
        delete[] transMapSendTrajData; 
        transMapSendTrajData = nullptr;
        bDeltransHistoryTraj = false; 
    }
    if(Inf_State_Get_MapHistorynDataPtr() && bDeltransHistoryTraj)
    {
        eblog(LOG_LV_AWS,"del MapHistory data");
        delete[] transMapHistorySendTrajData; 
        bDeltransHistoryTraj = false; 

    }
    if(Inf_State_Get_MapSaveDataPtr() && bDeltransMapSaveTraj)
    {
        eblog(LOG_LV_AWS,"del MapSave data");
        delete[] transMapSaveSendTrajData; 
        bDeltransMapSaveTraj = false; 
    }
#endif
    /* operation Area, 금지지역등 각종 데이터들 delete 해야함 */ 
}
void CAppInterface::receiveData()
{
    CStopWatch debugSW;
    
    short tempAction;
    if (pAws->Inf_State_Get_action(&tempAction))
    {
        ceblog((LOG_LV_AWS | LOG_LV_AWS), BOLDMAGENTA, " 0. AWS receive action : " <<(int)tempAction<<" (Update !!)");
        action.set(tempAction);

#if 0 // echo로 일단 action 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_action(tempAction, 0);
#endif

    }
    // else
    // {
    //     ceblog((LOG_LV_AWS| LOG_LV_AWS), MAGENTA, " 0. AWS receive action : " <<(int)tempAction);
    // }

    short tempStatus;
    if (pAws->Inf_State_Get_status(&tempStatus))
    {
        status.set(tempStatus);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempSound;
    if (pAws->Inf_State_Get_settings_soundVolume(&tempSound))
    {
        sound.set(tempSound);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempWaterLv;
    if (pAws->Inf_State_Get_settings_waterlevel(&tempWaterLv))
    {
        waterLv.set(tempWaterLv);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempCleanMode;
    if (pAws->Inf_State_Get_settings_mode(&tempCleanMode))
    {
        cleanMode.set(tempCleanMode);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempCleanDuration;
    if (pAws->Inf_State_Get_settings_duration(&tempCleanDuration))
    {
        cleanDuration.set(tempCleanDuration);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempTilt;
    if (pAws->Inf_State_Get_settings_enableTilt(&tempTilt))
    {
        tilt.set(tempTilt);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempLanguage;
    if (pAws->Inf_State_Get_settings_language(&tempLanguage))
    {
        language.set(tempLanguage);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }

    short tempCountry;
    if (pAws->Inf_State_Get_settings_country(&tempCountry))
    {
        country.set(tempCountry);

#if 0 // echo로 일단 status 상태정보 전송 -> 추후 상태정보 전송 위치 고려 필요.
        pAws->Inf_State_Set_status(tempStatus, 1);
#endif
    }
    
    short tempDryEnabled, tempDryPower, tempDryHours;
    if(pAws->Inf_State_Get_settings_dryEnabled(&tempDryEnabled))
    {
        dryEnabled.set(tempDryEnabled); // 자동걸레건조 true : 충전하면 자동으로 걸레건조, false : action에서 키가 들어와야 걸레건조
    }
    if(pAws->Inf_State_Get_settings_dryPower(&tempDryPower))
    {
        dryPower.set(tempDryPower);
    }
    if(pAws->Inf_State_Get_settings_dryHours(&tempDryHours))
    {
        dryHours.set(tempDryHours);
    }

    short tempValue;
    if(pAws->Inf_State_Get_operationArea_all(&tempValue))
    {
        operationAreaAll.set(tempValue);
    }
    
    // tSpot spot[MAXLINE];
    short spotNum; // spot의 개수
    if(pAws->Inf_State_Get_operationArea_spot(&spotNum, (float *)spot))
    {
        operationAreaSpot.set(spot, spotNum);
    }
    // tRoom room[MAXLINE];
    short roomNum; // 방의 개수
    if(pAws->Inf_State_Get_operationArea_room(&roomNum, (float *)room))
    {
        operationAreaRoom.set(room, roomNum);
    }
    // tCustom custom[MAXLINE];
    short customNum;  //커스텀 영역 개수
#if 1
    if(pAws->Inf_State_Get_operationArea_custom(&customNum, (float *)custom))
    {
        operationAreaCustom.set(custom, customNum);
    }
#else
    if(i > 20)
    {
        customNum = 5;
        custom[0].x = 1.12313211;
        custom[0].y = 2.6556565677;
        custom[0].w = 3;
        custom[0].h = 4;
        custom[1].x = 5;
        custom[1].y = 6;
        custom[1].w = 7;
        custom[1].h = 8;
        custom[2].x = 9;
        custom[2].y = 10;
        custom[2].w = 11;
        custom[2].h = 12;
        custom[3].x = 13;
        custom[3].y = 14;
        custom[3].w = 15;
        custom[3].h = 16;
        custom[4].x = 17;
        custom[4].y = 18;
        custom[4].w = debug++;
        custom[4].h = 20;
        operationAreaCustom.set(custom, customNum);
        i = 0;
    }
    i++;
#endif

    short lineNum;
#if 1
    if(pAws->Inf_State_Get_forbiddenArea_line(&lineNum, (float *)line))
    {   
        forbiddenLine.set(line, lineNum);
    }
#else
    if(i>20)
    {   
        lineNum = 2;
        line[0].x1 = debug++;
        line[0].y1 = debug++;
        line[0].x2 = debug++;
        line[0].y2 = debug++;
        line[1].x1 = debug++;
        line[1].y1 = debug++;
        line[1].x2 = debug++;
        line[1].y2 = debug++;
        forbiddenLine.set(line, lineNum);
        i = 0;
    }
    i++;
#endif
    short rectNum;
    if(pAws->Inf_State_Get_forbiddenArea_rect(&rectNum, (float *)rect))
    {
        forbiddenRect.set(rect, rectNum);
    }

    
#if 0
    short areaInfoNum;
    if(pAws->Inf_State_Get_AreaInfo_area(&areaInfoNum, (char *)infoArea))
    {
        areaInfo.set(infoArea, areaInfoNum);
    }
#endif

    short divideAreaNum = proc_state_get_AreaInfo_divide((char *)divideArea);
    short combineAreaNum = proc_state_get_AreaInfo_combine((char *)combineArea);

    if(divideAreaNum > 0){
        divideAreaInfo.set(divideArea,divideAreaNum);
    }
    if(combineAreaNum > 0){
        combineAreaInfo.set(combineArea,combineAreaNum);
    }
#if 1 
    short tempDontDisturbStatus;
    if(pAws->Inf_State_Get_doNotDisturb_status(&tempDontDisturbStatus))
    {
        dontDisturbStatus.set(tempDontDisturbStatus);
    }
    char tempDotDisturbStartTime[16];
    if(pAws->Inf_State_Get_doNotDisturb_startTime(tempDotDisturbStartTime))
    {
        if(tempDotDisturbStartTime != nullptr){
            eblog(LOG_LV_AWS," 2 dontDisturbStartTime = " << tempDotDisturbStartTime);
        }
        dontDisturbStartTime.set(tempDotDisturbStartTime);
    }

    char tempDotDisturbEndTime[16];
    if(pAws->Inf_State_Get_doNotDisturb_endTime(tempDotDisturbEndTime))
    {
        if(tempDotDisturbEndTime != nullptr){
            eblog(LOG_LV_AWS," tempDotDisturbEndTime = " << tempDotDisturbEndTime);
        }
        dontDisturbEndTime.set(tempDotDisturbEndTime);
    }
#else
    short tempDontDisturbStatus;
    if(pAws->Inf_State_Get_doNotDisturb_status(&tempDontDisturbStatus))
    {
        dontDisturbStatus.set(tempDontDisturbStatus);
    }
    char tempDotDisturbStartTime[16];
    if(i >20)
    {
        strcpy(tempDotDisturbStartTime, "13:44");
        dontDisturbStartTime.set(&tempDotDisturbStartTime);
        
        // char sendData[16];
        // strcpy(sendData, *dontDisturbStartTime.get());
        eblog(LOG_LV_AWS," dontDisturbStartTime = " << sendData);
        
        i =0;
    }
    i++;

    char tempDotDisturbEndTime[16];
    if(pAws->Inf_State_Get_doNotDisturb_endTime(tempDotDisturbEndTime))
    {
        dontDisturbEndTime.set(&tempDotDisturbEndTime);
    }
#endif

    short cleanScheduleNum;
    if(pAws->Inf_State_Get_cleanSchedule(&cleanScheduleNum, (char *)scheduleClean))
    {
        cleanSchedule.set(scheduleClean, cleanScheduleNum);
    }
#if 0 // 추후 정리.
    pAws->Inf_State_Get_doNotDisturb(short * status, char * startTime, char * endTime); // 이거는 예비용으로 있는것 같음
    pAws->Inf_State_Get_renewSchedule(short *value);

#endif
    // TIME_CHECK_END(debugSW.getTime());
}

void CAppInterface::receiveOtaData()
{
    CStopWatch debugSW;

#if 1
    if(pAws->Inf_State_Get_ota(&isForce, name, version, &isScheduled, scheduleTime))
    {
        otaForce.set(isForce);
        otaName.set(name);
        otaVersion.set(version);
        otaScheduled.set(isScheduled);
        otaScheduleTime.set(scheduleTime);

        eblog(LOG_LV_AWS," isForce : " << isForce << " name : " << name << " version : " << version
                        <<" isScheduled : " << isScheduled << " scheduleTime : " << scheduleTime);
    }
#else    
    if(i>5)
    {
        isForce = 1;
        strcpy(name, "TEST Name");
        strcpy(version, "TEST Version");
        isScheduled = 1;
        strcpy(scheduleTime, "2024.04.24");

        //방법2
        otaForce.set(isForce);
        otaName.set(&name);
        otaVersion.set(&version);
        otaScheduled.set(isScheduled);
        otaScheduleTime.set(&scheduleTime);

        eblog(LOG_LV_AWS," isForce : " << isForce << " name : " << name << " version : " << version
                        <<" isScheduled : " << isScheduled << " scheduleTime : " << scheduleTime);
        i = 0;
    }
    i++;
#endif
#if 0
        if // 방법1. 구조체로 템플릿을 만들어야하는거 같음
        //장점 : 한 눈에 보기 쉽다(이해하기 쉽다.)
        //단점 : 1.일반적인 상황에 대한 템플릿이 아닌 특수케이스 템플릿이 의미가 있는가?
        // 2. 통일성이 조금 깨짐 
        // 3.  
        ota.set(&isForce, name, version, &isScheduled, scheduleTime);
        else // 방법2. 템플릿 사용가능
        //장점 : 데이터 처리의 통일성
        //단점 : 작업량이 많다.(한 눈에 보기 어렵다.)
        ota.set(&isForce);
        ota.set(name);
        ota.set(version);
        ota.set(&isScheduled);
        ota.set(scheduleTime);
#endif
#if 0 // 추후 정리
    bool Inf_State_Get_ota_ap(short* Flag, char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
    bool Inf_State_Get_ota_ap(char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
    bool Inf_State_Get_ota_mcu(short* Flag, char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
    bool Inf_State_Get_ota_mcu(char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
    bool Inf_State_Get_ota_wifi(short* Flag, char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
    bool Inf_State_Get_ota_wifi(char Url[][MAX_URL_LENGTH],char File[][MAX_FILE_LENGTH]);
#endif
    
    // TIME_CHECK_END(debugSW.getTime());
}
/**
 * @brief 모니터에서 호출하는 함수 (완성률 80%)
 * 남은 작업 : 1. 로그추가 2. 미사용 Enum데이터 삭제         
 * 
 * @param type 
 */
void CAppInterface::transPhoneInterface(E_PHONE_INTERFACE_TYPE type)
{
    /* 현재 주석처리된 것은 사용x 나중에 삭제예정 */
    switch (type)
    {
    case E_PHONE_INTERFACE_TYPE::STOP_BOOTTING_CONNECT :
        boottingAwsTogle = false;
        break;
    case E_PHONE_INTERFACE_TYPE::CONNECT_AP :
        connectAP();
        break;
    case E_PHONE_INTERFACE_TYPE::DISCONNECT_AP :
        disconnectAP();
        break;
    case E_PHONE_INTERFACE_TYPE::CHECK_CONNECTING_STATE_AP :
        //isConnectingAP();
        break;
    case E_PHONE_INTERFACE_TYPE::AP_STATE :
        //apState();
        break;
    case E_PHONE_INTERFACE_TYPE::AP_STATES :
        //apStates();
        break;
    case E_PHONE_INTERFACE_TYPE::CONNECT_STATION :
        connectStation(E_CONNECT_SORT::BTN_CONNECT);
        break;
    case E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION :
        disconnectStation();
        break;
    case E_PHONE_INTERFACE_TYPE::CHECK_CONNECTING_STATE_STATION :
        //isConnectingStation();
        break;
    case E_PHONE_INTERFACE_TYPE::STATION_STATE :
        //stationState();
        break;
    case E_PHONE_INTERFACE_TYPE::STATION_STATES :
        //stationStates();
        break;
    case E_PHONE_INTERFACE_TYPE::CONNECTION :
        //connection();
        break;
    case E_PHONE_INTERFACE_TYPE::MODE :
        //mode();
        break;
    case E_PHONE_INTERFACE_TYPE::CONNECT_AWS :
        connectAWS();
        break;
    case E_PHONE_INTERFACE_TYPE::DISCONNECT_AWS :
        disconnectAWS();
        break;
    case E_PHONE_INTERFACE_TYPE::EXIT_ALL :
        disconnectAP();
        disconnectStation();
        disconnectAWS();
        break;
    case E_PHONE_INTERFACE_TYPE::CONNECTION_INTERRUPT :
        connectStation(E_CONNECT_SORT::INTERRUPT_CONNECT);
        break;
    /* case E_PHONE_INTERFACE_TYPE::CONNECTION_INTERRUPT :
        handleInterruptedConnection();
        break; */
    default:
        break;
    }
}
/**
 * @brief AWS 연결 하기전에 모드를 변경해야하는데 모드 변경함수
 * 
 */
void CAppInterface::connectAP()
{
    CStopWatch debugSW;

    if (pAp == nullptr)
    {
        disconnectAWS();

        pAp = new CRobotConnection();
        pAp->ToAp();
        ceblog(LOG_LV_AWS, BOLDCYAN, "pAp is nullptr \t -> new CRobotConnection and Start");
    }
    
    // TIME_CHECK_END(debugSW.getTime());
}
void CAppInterface::disconnectAP()
{
    CStopWatch debugSW;
    
    if (pAp != nullptr)
    {
        ceblog(LOG_LV_AWS, BOLDCYAN, "pAp exit");
        pAp->ApEnd(); 
        apStationExitTrigger = true;
        // delete pAp;
        // pAp = nullptr;
    }
    
    // TIME_CHECK_END(debugSW.getTime());
}
bool CAppInterface::isConnectingAP()
{
    CStopWatch debugSW;
    
    if ( pAp == nullptr )
    {
        // TIME_CHECK_END(debugSW.getTime());
        return false;
    }
    else
    {
        // TIME_CHECK_END(debugSW.getTime());
        return true;
    }
}

void CAppInterface::apState(void)
{
   apValue = pAp->ApState();
}

void CAppInterface::apStates(void)
{
    apValues = pAp->ApStates();
}
 
/* --------------Station 관련(HomeAp 공유기 - robot)--------------------- */

void CAppInterface::connectStation(E_CONNECT_SORT connect)
{
    /* 이 함수를 사용하여 연결함수 만들어야함 */
    // 시나리오1. 버튼 눌러서 연결
    // AP->Station->AWS
    // 시나리오2. 부팅 연결
    // Station->AWS

    CStopWatch debugSW;

    if (pStation == nullptr)
    {
        pStation = new CRobotConnection();
        ceblog(LOG_LV_AWS, BOLDCYAN, "new CRobotConnection pStation");
    }
#if AWS_MODE == 2
    std::strcpy(robotIp, FIX_IP.c_str());
    ceblog(LOG_LV_AWS, BOLDCYAN, "robotIp : " <<robotIp << "FIX_IP : "<<FIX_IP );
#endif

    if(connect == E_CONNECT_SORT::BTN_CONNECT)
    {
#if AWS_MODE == 2
        pStation->ToStation(robotIp); //고정 ip 접속 
#else
        pStation->ToStation(); //기존에 접속했던 AP에 연결 
#endif
        
        //pStation->ToStation(wifiConnectData.ssid, wifiConnectData.pw);
        //pStation->ToStation(char * SSID,char * PW,char * ip,char * router,char * dns1,char * dns2, char *netmask,char *gateway,char *network,char *broadcast);
        
        ceblog(LOG_LV_AWS, BOLDCYAN, "STATION BTN CONNECT ");
    }
    else if(connect == E_CONNECT_SORT::BOOTING_CONNECT)
    {
         //pStation->ToStation(); //기존에 접속했던 AP에 연결 
         //pStation->ToStation(char *ssid, char *pw);
         //pStation->ToStation(char * SSID,char * PW,char * ip,char * router,char * dns1,char * dns2, char *netmask,char *gateway,char *network,char *broadcast);

         
        if (1)//기존 데이터가 있으면
#if AWS_MODE == 2
        pStation->ToStation(robotIp); //고정 ip 접속
#else
        pStation->ToStation(); //기존에 접속했던 AP에 연결 
#endif
        else if(0) //데이터 없으면
        {
            // pStation->ToStation(wifiConnectData.ssid, wifiConnectData.pw);
            //pStation->ToStation(char * SSID,char * PW,char * ip,char * router,char * dns1,char * dns2, char *netmask,char *gateway,char *network,char *broadcast);
        }

        ceblog(LOG_LV_AWS, BOLDCYAN, "STATION BOOTING CONNECT ");
    }
    else if (connect == E_CONNECT_SORT::INTERRUPT_CONNECT)
    {
#if AWS_MODE == 2
        pStation->ToStation(robotIp); //고정 ip 접속
#else
        pStation->ToStation(); //기존에 접속했던 AP에 연결 
#endif
    }
    
    // TIME_CHECK_END(debugSW.getTime());    
}
void CAppInterface::disconnectStation()
{
    CStopWatch debugSW;
    
    if (pStation != nullptr)
    {
        pStation->StationEnd();
        apStationExitTrigger = true;
        // delete pStation;
        // pStation = nullptr;
        ceblog(LOG_LV_AWS, BOLDCYAN, "Station End");
    }
    
    // TIME_CHECK_END(debugSW.getTime());
}
bool CAppInterface::isConnectingStation()
{
    CStopWatch debugSW;
    
    if ( pStation == nullptr )
    {
        // TIME_CHECK_END(debugSW.getTime());
        return false;
    }
    else
    {
        // TIME_CHECK_END(debugSW.getTime());
        return true;
    }
}

void CAppInterface::stationState(void)
{
    stationValue = pStation->StationState();
}

void CAppInterface::stationStates(void)
{
    stationValues = pStation->StationStates();
}

bool CAppInterface::connection(void)
{
    //실시간으로 업데이트 받아야함
    //아직 수정이 필요할 수도? 
    if (pAp != nullptr)
    {
        return pAp->Connection();
    }
    if (pStation != nullptr)
    {
        return pStation->Connection();
    }
    else
    {
        return false;
    }
}

//AP와 Station 공통 부분
char CAppInterface::mode(void)
{
    //아직 수정이 필요할 수도? 
    if (pAp != nullptr)
    {
        return pAp->Mode();
    }
    else if (pStation != nullptr)
    {
        return pStation->Mode();
    }
    else
    {
        return (char)0;
    }
}

E_AP_STAION_CONNECTION_STATE CAppInterface::checkApConnection()
{   
    if(pAp != nullptr)
    {
        std::pair<char, char> values = getApValues();

        if (values.second == 0xE && values.first == 0xE)
        {
            return E_AP_STAION_CONNECTION_STATE::COMPLETE;
        }
        else if(values.second == 0xF || values.first == 0xF)
        {
            return E_AP_STAION_CONNECTION_STATE::FAIL;
        }
        else
        {
            eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
                            " values.second : "<< std::hex << static_cast<int>(values.second));
            return E_AP_STAION_CONNECTION_STATE::CONNECTING;
        }
    }
    else 
    {
        return E_AP_STAION_CONNECTION_STATE::CONNECTING;
    }
}

E_AP_STAION_CONNECTION_STATE CAppInterface::checkStationConnection()
{    
    if (pStation != nullptr)
    {
        std::pair<char, char> values = getStationValues();
        
        if (values.second == 0xE && values.first == 0xE)
        {
            return E_AP_STAION_CONNECTION_STATE::COMPLETE;
        }
        else if(values.second == 0xF || values.first == 0xF)
        {
            return E_AP_STAION_CONNECTION_STATE::FAIL;
        }
        else
        {
            // eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
            //                 " values.second : "<< std::hex << static_cast<int>(values.second));
            return E_AP_STAION_CONNECTION_STATE::CONNECTING;
        }
    }
    else 
    {
        return E_AP_STAION_CONNECTION_STATE::COMPLETE_OR_VOID;
    }
}

/**
 * @brief AP, STATIO, AWS 연결 중 외부적, 내부적 요인으로 인터럽트 발생시 실행하는 함수
 * 
 */
void CAppInterface::handleInterruptedConnection()
{
    //step1 AP, Staion, Aws 삭제
    disconnectAP();
    disconnectStation();
    disconnectAWS();

    //step2 toStation 연결
    if(pAp == nullptr && pStation == nullptr && pAws == nullptr)
    {
        connectStation(E_CONNECT_SORT::INTERRUPT_CONNECT);
        eblog(LOG_LV_AWS," Interrupt Connect Station ");
    }   

    //step3 Station 연결 완료되면 서비스 종료 getrsMode()가 2이고 완료 된 경우
    //bool checkConnection = checkStationConnection();
    
}

char CAppInterface::getApValue()
{
    if (pAp != nullptr) return pAp->ApState();
    else    return 'F';
}

std::pair<char, char> CAppInterface::getApValues()
{
    if (pAp != nullptr) return pAp->ApStates();
    else    return {'F', 'F'};
}

char CAppInterface::getStationValue()
{
    if (pStation != nullptr) return pStation->StationState();
    else    return 'F';
}

std::pair<char, char> CAppInterface::getStationValues()
{
    if (pStation != nullptr) return pStation->StationStates();
    else    return {'F', 'F'};
    
}

bool CAppInterface::checkApExitState()
{
    if (pAp == nullptr)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool CAppInterface::checkStationExitState()
{
    if(pStation == nullptr)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

void CAppInterface::connectAWS()
{
    CStopWatch debugSW;

    if (pAws == nullptr)
    {
        ceblog((LOG_LV_AWS), BOLDCYAN, "-------------------------");
        ceblog((LOG_LV_AWS), BOLDCYAN, "Wlocome to the new world ");
        ceblog((LOG_LV_AWS), BOLDCYAN, "A new ordeal awaits you  ");
        ceblog((LOG_LV_AWS), BOLDCYAN, "Please wait 2 seconds !!!");
        ceblog((LOG_LV_AWS), BOLDCYAN, "-------------------------");
        pAws = new CRobotToAWS();
        sleep(2);
        pAws->Start();
        ceblog((LOG_LV_AWS), BOLDCYAN, "pAws is nullptr \t -> new CRobotToAWS and Start");
    }
    else
    {
        ceblog(LOG_LV_AWS, BOLDCYAN, "pAws is already extis");
    } 
    
    // TIME_CHECK_END(debugSW.getTime());
}

void CAppInterface::disconnectAWS()
{
    CStopWatch debugSW;
    
    if (pAws != nullptr)
    {
        delete pAws;
        pAws = nullptr;
        ceblog(LOG_LV_AWS, BOLDCYAN, "pAws delete");
    }
    
    // TIME_CHECK_END(debugSW.getTime());
}

bool CAppInterface::isConnectingAWS()
{
    CStopWatch debugSW;
    
    if ( pAws == nullptr )
    {
        // TIME_CHECK_END(debugSW.getTime());
        //ceblog((LOG_LV_AWS), BOLDCYAN, "pAws nullptr");
        return false;
    }
    else
    {
        auto b = pAws->AWSConnection();
        // TIME_CHECK_END(debugSW.getTime());
        //ceblog((LOG_LV_AWS), BOLDCYAN, "AWSConnection : "<< b);
        return b;
        // return pAws->AWSConnection();
    }
}

/**
 * @brief AWS에 Action 상태전송 함수.
 *
 * 앱에 의하여 상태가 변경되는 경우는 flag= REPORT.
 * 로봇에 의하여 상태가 변경되는 경우는 flag= FROM_ROBOT.
 * 
 * @param actionData action 상태정보
 * @param flag 로봇 자체에서 변경되는 경우 FROM_ROBOT.
 */
void CAppInterface::transAction(short actionData)
{
    CStopWatch debugSW;
    
    eblog(LOG_LV_AWS,"Action Report : " << actionData);
    pAws->Inf_State_Set_action(0, 1);
    // TIME_CHECK_END(debugSW.getTime());
}

/**
 * @brief AWS에 Status 상태전송 함수.
 *
 * 앱에 의하여 상태가 변경되는 경우는 flag = REPORT.
 * 로봇에 의하여 상태가 변경되는 경우는 flag = FROM_ROBOT.
 * 
 * @param statusValue status 상태정보
 * @param flag 로봇 자체에서 변경되는 경우 FROM_ROBOT.
 */
void CAppInterface::transStatus(short statusValue)
{
    CStopWatch debugSW;
    
    eblog(LOG_LV_AWS,"Status Report : " << statusValue);

    pAws->Inf_State_Set_status(statusValue, 1);
    TIME_CHECK_END(debugSW.getTime());
}

void CAppInterface::transFactoryReset(char* startTime, char* descript)
{
    CStopWatch debugSW;
    
    eblog(LOG_LV_AWS,"transFactoryReset : " );
    strcpy(factroyResetStartTime,startTime);
    if(factroyResetStartTime != nullptr){
        eblog(LOG_LV_NECESSARY," start Time : "<< factroyResetStartTime);
    }
    strcpy(factoryResetDiscript,descript);
    if(factoryResetDiscript != nullptr){
        eblog(LOG_LV_NECESSARY," descript : "<< factoryResetDiscript);
    }
    pAws->Inf_State_Set_FctoryResetPtr(factroyResetStartTime,factoryResetDiscript);

    TIME_CHECK_END(debugSW.getTime());
}

void CAppInterface::transWaterLevel(short waterLv){
    ceblog(LOG_LV_NECESSARY, CYN," transWaterLevel : " << waterLv);
    pAws->Inf_State_Set_settings_waterlevel(waterLv, 1);
}
void CAppInterface::transSoundLevel( short soundLv){
    ceblog(LOG_LV_NECESSARY, CYN," transSoundLevel : " << soundLv);
    pAws->Inf_State_Set_settings_soundVolume(soundLv, 0);
}
void CAppInterface::transDryMopData(short dryEnabled, short dryHours, short dryPower){
    ceblog(LOG_LV_NECESSARY, CYN," transDryMopData : " << dryEnabled << " , " << dryHours << " ," << dryPower);
    pAws->Inf_State_Set_settings_dry(dryEnabled,dryPower,dryHours,0);
}
void CAppInterface::transCleanMode(short cleanMode){
    ceblog(LOG_LV_NECESSARY, CYN," transCleanMode : " << cleanMode);
    pAws->Inf_State_Set_settings_mode(cleanMode, 0);
}
void CAppInterface::transLanguage(short language){
    ceblog(LOG_LV_NECESSARY, CYN," transLanguage : " << language);
    pAws->Inf_State_Set_settings_language(language, 0);
}
void CAppInterface::transCountry(short country){
    ceblog(LOG_LV_NECESSARY, CYN," transCountry : " << country);
    pAws->Inf_State_Set_settings_country(country, 0);
}
void CAppInterface::transErrorInfo(std::string errorCode, std::string errorDesc){
    ceblog(LOG_LV_NECESSARY, CYN," transErrorInfo : ");
    std::strcpy(code, errorCode.c_str());
    pAws->Inf_State_Set_error_code(code, errorCode.length(), 1);
    std::strcpy(desc, errorDesc.c_str());
    pAws->Inf_State_Set_error_desc(desc, errorDesc.length(), 1); 
}
void CAppInterface::transBatteryPercent(short battery){
    ceblog(LOG_LV_NECESSARY, CYN," transBatteryPercent : " << battery);
    pAws->Inf_State_Set_info_battery(battery, 1);
}
void CAppInterface::transSetting(std::string apVersion, std::string mcuVersion)
{
    ceblog(LOG_LV_NECESSARY, CYN," transSetting : ");
    char string[128];

    std::strcpy(string, apVersion.c_str());
    pAws->Inf_State_Set_info_apVersion(string, apVersion.length(), 1);

    std::strcpy(string, mcuVersion.c_str());
    pAws->Inf_State_Set_info_mcuVersion(string, mcuVersion.length(), 1);

}

#if 0
void CAppInterface::transTotalMapInfo1(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, tPoint cradleCoord)
{
    transMapData(mapData, mapInfo);
    transTrajectory(traj);
    transRobotPose(robotPose);
}
#endif

/**
 * @brief AWS에 맵, 경로, 좌표 전송 함수.
 * 
 * @param mapData simplify 맵 데이터
 * @param mapInfo simplify 맵의 넓이, 높이, 원점 좌표  
 * @param robotPose 로봇의 slam pose 좌표
 * @param traj 로봇의 경로 좌표
 */
void CAppInterface::transTotalMapInfo(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, tPoint cradleCoord, double cleanSize)
{
    ceblog(LOG_LV_NECESSARY, CYN," transTotalMapInfo : ");
    u32 mapSize; 
    u32 mapHeight;
    u32 mapWidth;
    float origin_x;
    float origin_y;
    if (mapData != nullptr)
    {
        mapSize = mapInfo.width * mapInfo.height;
        mapHeight = mapInfo.height;
        mapWidth = mapInfo.width;
        origin_x = mapInfo.origin_x;
        origin_y = mapInfo.origin_y;

        ceblog(LOG_LV_AWS, YELLOW,"width : "<<mapInfo.width
                            <<", height : "<<mapInfo.height);
        ceblog(LOG_LV_AWS, YELLOW,"origin_x : "<<origin_x
                            <<", origin_y : "<<origin_y);
    }
    else
    {
        mapSize = 0;
        mapHeight = 0;
        mapWidth = 0;
        origin_x = 0.0;
        origin_y = 0.0;
        ceblog(LOG_LV_AWS, YELLOW,"mapData is nullptr");
    }

    /* pose */
    if (robotPoseX != robotPose.x || robotPoseY != robotPose.y)
    {
        robotPoseX = robotPose.x;
        robotPoseY = robotPose.y;
    }
    /* trajectory */
#if USE_RDP
    std::list<tPoint> rdpTraj;
    utils::path::ramerDouglasPeucker(traj, 0.05, rdpTraj);
    int trajSize = rdpTraj.size();
    if (!traj.empty())
    {
        int i = 0;

        bDeltransMapSendTraj = false;//포인터 lock
        transMapSendTrajData = new tTrajectorySendBuf[trajSize];         
        eblog(LOG_LV_AWS,"traj data address : "<< &transMapSendTrajData);


        int idx = 0;
        for(const auto& pointData : rdpTraj)
        {
            if (idx < trajSize)
            {
                transMapSendTrajData[idx].x = pointData.x;
                transMapSendTrajData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj transMapSendTrajData X["<< idx <<"]:"<<transMapSendTrajData[idx].x<<
                            "  traj transMapSendTrajData Y ["<< idx <<"]:"<< transMapSendTrajData[idx].y);
            }
            idx++;            
        }
    }
#else
    int trajSize = traj.size();
    if (!traj.empty())
    {
        int trajSize = traj.size();
        int i = 0;

        bDeltransMapSendTraj = false;//포인터 lock
        transMapSendTrajData = new tTrajectorySendBuf[trajSize];         
        eblog(LOG_LV_AWS,"traj data address : "<< &transMapSendTrajData);
        
        int idx = 0;
        for(const auto& pointData : traj)
        {
            if (idx < trajSize)
            {
                transMapSendTrajData[idx].x = pointData.x;
                transMapSendTrajData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj transMapSendTrajData X["<< idx <<"]:"<<transMapSendTrajData[idx].x<<
                            "  traj transMapSendTrajData Y ["<< idx <<"]:"<< transMapSendTrajData[idx].y);
            }
            idx++;            
        }
        /* 디버깅용 코드 */
        pAws->Inf_State_Set_TrajectoryDataPtr((double*)transMapSendTrajData, 2 * trajSize, 100, 100, 100, trajCount++); //이거 size 정의 필요함
        bDeltransMapSendTraj = true; //포인터 unlock
    }
#endif

    /* 기타 */
    int cleanTime = 100;
    double cleanArea = cleanSize;
    int cleanStatus = 100;
    
    double cPoseX;
    double cPoseY; 
    if (cradleCoord.x != 0 || cradleCoord.y != 0)
    {
        cPoseX= cradleCoord.x;
        cPoseY= cradleCoord.y;
    }
    else 
    {
        cPoseX = 0.0;
        cPoseY = 0.0; 
    }

    eblog(LOG_LV_AWS,"Cradle POSE X : "<<cPoseX<<" Cradle POSE Y : "<<cPoseY);
    eblog(LOG_LV_AWS,"cleanArea : "<<cleanArea << "cleanTime" <<cleanTime);
    pAws->Inf_State_Set_MapOnceDataPtr((char*)mapData, mapSize, mapHeight, mapWidth, origin_x, origin_y,
                        (double*)transMapSendTrajData, 2 * trajSize, robotPoseX, robotPoseY, cPoseX, cPoseY, cleanTime, cleanArea, cleanStatus, mapPoseTraj++);
    bDeltransMapSendTraj = true;
}

void CAppInterface::transMapData(u8* mapData, tGridmapInfo mapInfo)
{
    if (mapData != nullptr)
    {
        u32 size = mapInfo.width * mapInfo.height;

        float origin_x = mapInfo.origin_x;
        float origin_y = mapInfo.origin_y;

        
        ceblog(LOG_LV_AWS, YELLOW,"width : "<<mapInfo.width
                            <<", height : "<<mapInfo.height);
        ceblog(LOG_LV_AWS, YELLOW,"origin_x : "<<origin_x
                            <<", origin_y : "<<origin_y);
        if(pAws->Inf_State_Get_MapDataPtr())
        {
            pAws->Inf_State_Set_MapDataPtr((char*)mapData, size, mapInfo.height, mapInfo.width,
                                        (double)origin_x, (double)origin_y, 100, 10.15, 100, mapCount++);
        }
#if 0 //map데이터 디버깅
        for(int i = 0; i < mapInfo.width; i++)
        {
            for(int j = 0; j < mapInfo.height; j++ )
            {
                int out = mapData[i+j*mapInfo.width]==255 ? 0 : 1; //55보다 작으면 0 == 빈공간  1 == 벽
                //std::cout<<(int)pMapArray[i*j+j];
                std::cout<<out;
            }
            std::cout<<std::endl;
        }

        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
#endif
    }
}
void CAppInterface::transRobotPose(tPose robotPose)
{
    if (robotPoseX != robotPose.x || robotPoseY != robotPose.y)
    {
        robotPoseX = robotPose.x;
        robotPoseY = robotPose.y;

        eblog(LOG_LV_AWS,"robotPose X : "<<robotPose.x<<
                   " robotPose Y : "<< robotPose.y);
        
        pAws->Inf_State_Set_PositionDataPtr(robotPoseX, robotPoseY, poseCount++);
    }
}

void CAppInterface::transOtaVersion(char * _version)
{
    strcpy(version,_version);
    ceblog(LOG_LV_NECESSARY, CYN," transOtaVersion : " << version);
    pAws->Inf_State_Set_info_otaVersion(version,strlen(version),false);
}
void CAppInterface::transTrajectory(std::list<tPoint> traj)
{
    if (!traj.empty())
    {
        int trajSize = traj.size();
        int i = 0;
#if 1
        bDeltransMapSendTraj = false;//포인터 lock
        transMapSendTrajData = new tTrajectorySendBuf[trajSize];         
        eblog(LOG_LV_AWS,"traj data address : "<< &transMapSendTrajData);
        
        int idx = 0;
        for(const auto& pointData : traj)
        {
            if (idx < trajSize)
            {
                transMapSendTrajData[idx].x = pointData.x;
                transMapSendTrajData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj transMapSendTrajData X["<< idx <<"]:"<<transMapSendTrajData[idx].x<<
                            "  traj transMapSendTrajData Y ["<< idx <<"]:"<< transMapSendTrajData[idx].y);
            }
            idx++;            
        }

        //pAws->Inf_State_Set_TrajectoryDataPtr(transMapSendTrajData, sizeof(tTrajectorySendBuf) * trajSize, trajCount++); //이거 size 정의 필요함
        pAws->Inf_State_Set_TrajectoryDataPtr((double*)transMapSendTrajData, 2 * trajSize, 100, 100, 100, trajCount++); //이거 size 정의 필요함
        bDeltransMapSendTraj = true; //포인터 unlock
#else 
        for(tPoint& pointData : traj)
        {
            trajPoint[i++] = pointData.x;
            trajPoint[i++] = pointData.y;
            eblog(LOG_LV_AWS,"trajPoint X["<< i-2 <<"]:"<<trajPoint[i-2]<<
                            "trajPoint Y ["<< i-1 <<"]:"<< trajPoint[i-1]);
        }

        pAws->Inf_State_Set_TrajectoryDataPtr(trajPoint, trajSize*2, 100, 100, 100, trajCount++); //이거 size 정의 필요함
#endif 
    }
}
void transCradlePose(tPoint cradleCoord)
{

}
/**
 * @brief 왜 있는지 나도 모름 ai팀에서 만들어 달라고해서 만들어줌 
 * 청소서비스가 완료 될때 잡다한 정보들을 보내준다는 것  
 */
void CAppInterface::transHistoryData
(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj, tPoint cradlePose,
std::string cleanStartTime, std::string exitReason, double cleanedSize, int cleanTime, std::string areaInfo)
{
    /* robot pose */
    ceblog(LOG_LV_NECESSARY, CYN," transHistoryData : ");
    if (robotPoseX != robotPose.x || robotPoseY != robotPose.y)
    {
        robotPoseX = robotPose.x;
        robotPoseY = robotPose.y;
    }


    std::strcpy(cpyCleanStartTime, cleanStartTime.c_str());
    std::strcpy(cpyExitReason, exitReason.c_str());
    std::strcpy(cpyCleanAreaInfo, areaInfo.c_str());

    std::list<tPoint> rdpTraj;
    utils::path::ramerDouglasPeucker(traj, 0.05, rdpTraj);
    if(mapData != nullptr && !traj.empty())
    {
        //취소 된경우
        u32 mapSize = mapInfo.width * mapInfo.height;
        float origin_x = mapInfo.origin_x;
        float origin_y = mapInfo.origin_y;

        /* trajectory */
        int trajSize = rdpTraj.size();
        int i = 0;
        // bDeltransHistoryTraj = false;
        tTrajectorySendBuf* trajSendData = new tTrajectorySendBuf[trajSize]; 
        int idx = 0;
        for(const auto& pointData : rdpTraj)
        {
            if (idx < trajSize)
            {
                trajSendData[idx].x = pointData.x;
                trajSendData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj trajSendData X["<< idx <<"]:"<<trajSendData[idx].x<<
                            "  traj trajSendData Y ["<< idx <<"]:"<< trajSendData[idx].y);

            }
            idx++;            
        }

        eblog(LOG_LV_AWS,"Clean Start Time : "<< cpyCleanStartTime <<" Exit Reason : "<< cpyExitReason << 
                        " Cleaned Size : "<<cleanedSize<<" Clean AreaInfo : "<<cpyCleanAreaInfo<<" Clean Time(sec) : "<<cleanTime<<
                        " Map Size : "<<mapSize<<" map height : "<<mapInfo.height<<
                        " map width : "<<mapInfo.width<<" origin_x : "<<origin_x<<
                        " origin_y : "<<origin_y<<" trajSize : "<<2*trajSize<<
                        " robotPoseX : "<<robotPoseX<<" robotPoseY : "<<robotPoseY<<
                        " cradleX : "<<cradlePose.x <<" cradleY : "<<cradlePose.y);
        pAws->Inf_State_Set_MapHistorynDataPtr(
            cpyCleanStartTime, 
            cpyExitReason, 
            cpyCleanAreaInfo,
            cleanedSize,
            cleanTime,
            (char*)mapData, 
            mapSize, mapInfo.height, mapInfo.width, 
            (double)origin_x, (double)origin_y,
            (double*)trajSendData, 2 * trajSize,
            robotPoseX, robotPoseY,
            cradlePose.x, cradlePose.y);
        /* 포인터 삭제 */     
        // bDeltransHistoryTraj = true;
    }
    else 
    {
        //취소 된경우
        u32 mapSize = mapInfo.width * mapInfo.height;
        float origin_x = mapInfo.origin_x;
        float origin_y = mapInfo.origin_y;

        /* trajectory */
        int trajSize = traj.size();
        int i = 0;
        // bDeltransHistoryTraj = false;
        tTrajectorySendBuf* trajSendData = new tTrajectorySendBuf[trajSize]; 
        int idx = 0;
        for(const auto& pointData : traj)
        {
            if (idx < trajSize)
            {
                trajSendData[idx].x = pointData.x;
                trajSendData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj trajSendData X["<< idx <<"]:"<<trajSendData[idx].x<<
                            "  traj trajSendData Y ["<< idx <<"]:"<< trajSendData[idx].y);

            }
            idx++;            
        }
        eblog(LOG_LV_AWS,"Clean Start Time : "<< cpyCleanStartTime <<" Exit Reason : "<< cpyExitReason << 
                        " Cleaned Size : "<<cleanedSize<<" Clean AreaInfo : "<<cpyCleanAreaInfo<<
                        " Map Size : "<<mapSize<<" map height : "<<mapInfo.height<<
                        " map width : "<<mapInfo.width<<" origin_x : "<<origin_x<<
                        " origin_y : "<<origin_y<<" trajSize : "<<trajSize<<
                        " robotPoseX : "<<robotPoseX<<" robotPoseY : "<<robotPoseY<<
                        " cradleX : "<<cradlePose.x <<" cradleY : "<<cradlePose.y);
        pAws->Inf_State_Set_MapHistorynDataPtr(
            cpyCleanStartTime, 
            cpyExitReason, 
            cpyCleanAreaInfo,
            cleanedSize,
            cleanTime,
            (char*)mapData, 
            mapSize, mapInfo.height, mapInfo.width, 
            (double)origin_x, (double)origin_y,
            (double*)trajSendData, 2 * trajSize,
            robotPoseX, robotPoseY,
            cradlePose.x, cradlePose.y);
    }
}

void CAppInterface::transSavedMapData
(u8* mapData, tGridmapInfo mapInfo, tPose robotPose, std::list<tPoint> traj,
std::string uniqueKey, std::string mapName, int order, std::string areaInfo)
{
    ceblog(LOG_LV_NECESSARY, CYN," transSavedMapData : ");
        /* robot pose */
    if (robotPoseX != robotPose.x || robotPoseY != robotPose.y)
    {
        robotPoseX = robotPose.x;
        robotPoseY = robotPose.y;
    }

    /* trajectory */
    if (!traj.empty())
    {
        int trajSize = traj.size();
        int i = 0;
#if 1
        tTrajectorySendBuf* sendData = new tTrajectorySendBuf[trajSize];         
        eblog(LOG_LV_AWS,"traj data address : "<< &sendData);
            

        int idx = 0;
        for(const auto& pointData : traj)
        {
            if (idx < trajSize)
            {
                sendData[idx].x = pointData.x;
                sendData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj sendData X["<< idx <<"]:"<<sendData[idx].x<<
                            "  traj sendData Y ["<< idx <<"]:"<< sendData[idx].y);
            }
            idx++;            
        }

#else 
        for(tPoint& pointData : traj)
        {
            trajPoint[i++] = pointData.x;
            trajPoint[i++] = pointData.y;
            eblog(LOG_LV_AWS,"trajPoint X["<< i-2 <<"]:"<<trajPoint[i-2]<<
                            "trajPoint Y ["<< i-1 <<"]:"<< trajPoint[i-1]);
        }

        pAws->Inf_State_Set_TrajectoryDataPtr(trajPoint, trajSize*2, 100, 100, 100, trajCount++); //이거 size 정의 필요함
#endif 
    }

    std::strcpy(cpyUniqueKey, uniqueKey.c_str());
    std::strcpy(cpyMapName, mapName.c_str());
    std::strcpy(cpyExplorerAreaInfo, areaInfo.c_str());

    if (mapData == nullptr)
    {
        ceblog(LOG_LV_AWS, BLUE,"mapData is nullptr");
    }
    if (traj.empty())
    {
        ceblog(LOG_LV_AWS, BLUE,"trajectory is empty");
    }

    if(mapData != nullptr)
    {
        u32 mapSize = mapInfo.width * mapInfo.height;
        float origin_x = mapInfo.origin_x;
        float origin_y = mapInfo.origin_y;


#if 0 // 현재 보내주지않음 
        /* trajectory */
        int trajSize = traj.size();
        int i = 0;
        // bDelTransHistoryTraj = false;
        tTrajectorySendBuf* trajSendData = new tTrajectorySendBuf[trajSize]; 
        int idx = 0;
        for(const auto& pointData : traj)
        {
            if (idx < trajSize)
            {
                trajSendData[idx].x = pointData.x;
                trajSendData[idx].y = pointData.y;
                eblog(LOG_LV_AWS,"traj trajSendData X["<< idx <<"]:"<<trajSendData[idx].x<<
                            "  traj trajSendData Y ["<< idx <<"]:"<< trajSendData[idx].y);

            }
            idx++;            
        }
#endif
        ceblog(LOG_LV_AWS, BLUE, "message send ukey : "<<cpyUniqueKey<<" Map Name : "<< cpyMapName<<" Order : "<< order
                            <<" area : "<< cpyExplorerAreaInfo << " Map Size : "<< mapSize << " Map height : "<< mapInfo.height
                            <<" Map width : "<< mapInfo.width << " origin_x : "<<origin_x<<" origin_y : "<<origin_y);
        pAws->Inf_State_Set_MapSaveDataPtr(
            cpyUniqueKey, 
            cpyMapName, 
            order,
            cpyExplorerAreaInfo,
            (char*)mapData,
            mapSize,
            mapInfo.height,
            mapInfo.width,
            origin_x,
            origin_y);
    }
}

void CAppInterface::transCleanAll(short all){
    eblog(LOG_LV_AWS," All Area Clean");
    ceblog(LOG_LV_NECESSARY, CYN," transCleanAll : ");
    pAws->Inf_State_Set_operationArea_all(false);
}
void CAppInterface::transCleanSpot(std::list<tSpot> spot, short spotNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transCleanSpot : ");
    eblog(LOG_LV_AWS,"Spot Clean");
    tSpot* sendSpot = new tSpot[spotNumber];
    int idx = 0;
    for (const auto& tempData : spot) {
        if(idx < spotNumber)
        {
            sendSpot[idx].x = tempData.x;
            sendSpot[idx].y = tempData.y;

            eblog(LOG_LV_AWS," Spot "<<"[" << idx << "] : x = " << sendSpot[idx].x 
                                        <<"[" << idx << "] : y = " << sendSpot[idx].y);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," spotNumber : "<<spotNumber);
    pAws->Inf_State_Set_operationArea_spot(spotNumber, (float *)sendSpot, false);
}
void CAppInterface::transCleanRoom(std::list<tRoom> room, short roomNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transCleanRoom : ");
    eblog(LOG_LV_AWS," Room Clean");
    tRoom* sendRoom = new tRoom[roomNumber];
    int idx = 0;
    for (const auto& tempData : room) {
        if(idx < roomNumber)
        {
            sendRoom[idx].x = tempData.x;
            sendRoom[idx].y = tempData.y;
            sendRoom[idx].w = tempData.w;
            sendRoom[idx].h = tempData.h;
            eblog(LOG_LV_AWS," Room "<<"[" << idx << "] : x = " << sendRoom[idx].x 
                                        <<"[" << idx << "] : y = " << sendRoom[idx].y
                                        <<"[" << idx << "] : w = " << sendRoom[idx].w
                                        <<"[" << idx << "] : h = " << sendRoom[idx].h);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," roomNumber : "<<roomNumber);
    pAws->Inf_State_Set_operationArea_room(roomNumber, (float *)sendRoom, false);
}
void CAppInterface::transCleanCustom(std::list<tCustom> custom, short customNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transCleanCustom : ");
    eblog(LOG_LV_AWS," custom Clean");
    tCustom* sendCustom = new tCustom[customNumber];
    int idx = 0;
    for (const auto& tempData : custom) {
        if(idx < customNumber)
        {
            sendCustom[idx].x = tempData.x;
            sendCustom[idx].y = tempData.y;
            sendCustom[idx].w = tempData.w;
            sendCustom[idx].h = tempData.h;
            eblog(LOG_LV_AWS," Custom "<<"[" << idx << "] : x = " << sendCustom[idx].x 
                                        <<"[" << idx << "] : y = " << sendCustom[idx].y
                                        <<"[" << idx << "] : w = " << sendCustom[idx].w
                                        <<"[" << idx << "] : h = " << sendCustom[idx].h);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," customNumber : "<<customNumber);
    pAws->Inf_State_Set_operationArea_custom(customNumber, (float *)sendCustom, false);
}

void CAppInterface::transOperationArea(short cleanType, short all, std::list<tSpot> spot, short spotNumber, std::list<tRoom> room, short roomNumber, std::list<tCustom> custom, short customNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transOperationArea : ");
    eblog(LOG_LV_AWS," OperationArea");
    if(cleanType == 1)
    {
        pAws->Inf_State_Set_operationArea_all(false);
    }
    else if (cleanType == 2)
    {
        eblog(LOG_LV_AWS," Spot");
        tSpot* sendSpot = new tSpot[spotNumber];
        int idx = 0;
        for (const auto& tempData : spot) {
            if(idx < spotNumber)
            {
                sendSpot[idx].x = tempData.x;
                sendSpot[idx].y = tempData.y;

                eblog(LOG_LV_AWS," Spot "<<"[" << idx << "] : x = " << sendSpot[idx].x 
                                            <<"[" << idx << "] : y = " << sendSpot[idx].y);
            }
            idx++;
        }
        eblog(LOG_LV_AWS," spotNumber : "<<spotNumber);
        pAws->Inf_State_Set_operationArea_spot(spotNumber, (float *)sendSpot, false);
    }
    else if (cleanType == 3)
    {
        eblog(LOG_LV_AWS," Room");
        tRoom* sendRoom = new tRoom[roomNumber];
        int idx = 0;
        for (const auto& tempData : room) {
            if(idx < roomNumber)
            {
                sendRoom[idx].x = tempData.x;
                sendRoom[idx].y = tempData.y;
                sendRoom[idx].w = tempData.w;
                sendRoom[idx].h = tempData.h;
                eblog(LOG_LV_AWS," Room "<<"[" << idx << "] : x = " << sendRoom[idx].x 
                                            <<"[" << idx << "] : y = " << sendRoom[idx].y
                                            <<"[" << idx << "] : w = " << sendRoom[idx].w
                                            <<"[" << idx << "] : h = " << sendRoom[idx].h);
            }
            idx++;
        }
        eblog(LOG_LV_AWS," roomNumber : "<<roomNumber);
        pAws->Inf_State_Set_operationArea_room(roomNumber, (float *)sendRoom, false);
    }
    else if(cleanType == 4)
    {
        eblog(LOG_LV_AWS," custom");
        tCustom* sendCustom = new tCustom[customNumber];
        int idx = 0;
        for (const auto& tempData : custom) {
            if(idx < customNumber)
            {
                sendCustom[idx].x = tempData.x;
                sendCustom[idx].y = tempData.y;
                sendCustom[idx].w = tempData.w;
                sendCustom[idx].h = tempData.h;
                eblog(LOG_LV_AWS," Custom "<<"[" << idx << "] : x = " << sendCustom[idx].x 
                                            <<"[" << idx << "] : y = " << sendCustom[idx].y
                                            <<"[" << idx << "] : w = " << sendCustom[idx].w
                                            <<"[" << idx << "] : h = " << sendCustom[idx].h);
            }
            idx++;
        }
        eblog(LOG_LV_AWS," customNumber : "<<customNumber);
        pAws->Inf_State_Set_operationArea_custom(customNumber, (float *)sendCustom, false);
    }
}

void CAppInterface::transForbiddenLine(std::list<tForbiddenLine> line, short lineNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transForbiddenLine : ");
    eblog(LOG_LV_AWS,"Forbidden line");
    tForbiddenLine* sendLine = new tForbiddenLine[lineNumber];
    int idx = 0;
    for (const auto& tempData : line) {
        if(idx < lineNumber)
        {
            sendLine[idx].x1 = tempData.x1;
            sendLine[idx].y1 = tempData.y1;
            sendLine[idx].x2 = tempData.x2;
            sendLine[idx].y2 = tempData.y2;
            eblog(LOG_LV_AWS," line "<<"[" << idx << "] : x1 = " << sendLine[idx].x1
                                        <<" [" << idx << "] : y1 = " << sendLine[idx].y1
                                        <<" [" << idx << "] : x2 = " << sendLine[idx].x2
                                        <<" [" << idx << "] : y2 = " << sendLine[idx].y2);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," lineNumber : "<<lineNumber);
    pAws->Inf_State_Set_forbiddenArea_line(lineNumber, (float *)sendLine, false);
}
void CAppInterface::transForbiddenRect(std::list<tForbiddenRect> rect, short rectNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transForbiddenRect : ");
    eblog(LOG_LV_AWS,"Forbidden rect");
    tForbiddenRect* sendRect = new tForbiddenRect[rectNumber];
    int idx = 0;
    for (const auto& tempData : rect) {
        if(idx < rectNumber)
        {
            sendRect[idx].x = tempData.x;
            sendRect[idx].y = tempData.y;
            sendRect[idx].w = tempData.w;
            sendRect[idx].h = tempData.h;
            eblog(LOG_LV_AWS," Rect "<<"[" << idx << "] : x = " << sendRect[idx].x 
                                        <<"[" << idx << "] : y = " << sendRect[idx].y
                                        <<"[" << idx << "] : w = " << sendRect[idx].w
                                        <<"[" << idx << "] : h = " << sendRect[idx].h);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," rectNumber : "<<rectNumber);
    pAws->Inf_State_Set_forbiddenArea_rect(rectNumber, (float *)sendRect, false);
}
// void CAppInterface::transForbiddenArea()
void CAppInterface::transForbiddenArea(short forbiddenType, std::list<tForbiddenLine> line, short lineNumber, std::list<tForbiddenRect> rect, short rectNumber)
{
    ceblog(LOG_LV_NECESSARY, CYN," transForbiddenArea : ");
    if (forbiddenType == 1){
        eblog(LOG_LV_AWS," line");
        tForbiddenLine* sendLine = new tForbiddenLine[lineNumber];
        int idx = 0;
        for (const auto& tempData : line) {
            if(idx < lineNumber)
            {
                sendLine[idx].x1 = tempData.x1;
                sendLine[idx].y1 = tempData.y1;
                sendLine[idx].x2 = tempData.x2;
                sendLine[idx].y2 = tempData.y2;
                eblog(LOG_LV_AWS," line "<<"[" << idx << "] : x1 = " << sendLine[idx].x1
                                            <<" [" << idx << "] : y1 = " << sendLine[idx].y1
                                            <<" [" << idx << "] : x2 = " << sendLine[idx].x2
                                            <<" [" << idx << "] : y2 = " << sendLine[idx].y2);
            }
            idx++;
        }
        eblog(LOG_LV_AWS," lineNumber : "<<lineNumber);
        pAws->Inf_State_Set_forbiddenArea_line(lineNumber, (float *)sendLine, false);

    }
    else if (forbiddenType == 2){
        eblog(LOG_LV_AWS," rect");
        tForbiddenRect* sendRect = new tForbiddenRect[rectNumber];
        int idx = 0;
        for (const auto& tempData : rect) {
            if(idx < rectNumber)
            {
                sendRect[idx].x = tempData.x;
                sendRect[idx].y = tempData.y;
                sendRect[idx].w = tempData.w;
                sendRect[idx].h = tempData.h;
                eblog(LOG_LV_AWS," Rect "<<"[" << idx << "] : x = " << sendRect[idx].x 
                                            <<"[" << idx << "] : y = " << sendRect[idx].y
                                            <<"[" << idx << "] : w = " << sendRect[idx].w
                                            <<"[" << idx << "] : h = " << sendRect[idx].h);
            }
            idx++;
        }
        eblog(LOG_LV_AWS," rectNumber : "<<rectNumber);
        pAws->Inf_State_Set_forbiddenArea_rect(rectNumber, (float *)sendRect, false);
    }
}

void CAppInterface::transAreaInfo(std::list<tAreaInfo>info, short areaNum)
{
    ceblog(LOG_LV_NECESSARY, CYN," transAreaInfo 영역 갯수 : " << areaNum);
   
    std::list<tAreaInfo> temp = info;
    for(int i=0; i < areaNum; i++){
        infoArea[i].id = temp.front().id;
        infoArea[i].polyganNum = temp.front().polyganNum;
        memset(infoArea[i].name,0x00,sizeof(infoArea[i].name));
        memset(infoArea[i].color,0x00,sizeof(infoArea[i].color));
        ceblog(LOG_LV_NECESSARY, CYN," id : " << infoArea[i].id << " polygon size : " << (infoArea[i].polyganNum/2));
        for(int j=0; j < infoArea[i].polyganNum; j++)
        {
            infoArea[i].polygon[j] = temp.front().polygon[j];
            if(j%2  == 0){
                ceblog(LOG_LV_NECESSARY, CYN," polygon X : " << infoArea[i].polygon[j]);
            }
            else{
                ceblog(LOG_LV_NECESSARY, CYN," polygon Y : " << infoArea[i].polygon[j]);
            }
        }

        if(!temp.empty()) temp.pop_front();
    }

    pAws->Inf_State_Set_AreaInfo_area(areaNum, (char *)infoArea, true);
}

void CAppInterface::transDivideArea(tDivideArea data)
{
    ceblog(LOG_LV_NECESSARY, CYN," transDivideArea: ");
    
    divideArea[0].id = data.id;
    for(int i=0;i < 4; i++){
        divideArea[0].point[i] = data.point[i];
    }
    if(1){
        pAws->Inf_State_Set_AreaInfo_divide(NULL, false);
    }
    else{
        pAws->Inf_State_Set_AreaInfo_divide((char *)divideArea, false);
    }
}

void CAppInterface::transCombineArea(tCombieArea data)
{
    ceblog(LOG_LV_NECESSARY, CYN," transCombineArea");
    for(i=0; i<2; i++){
        combineArea->point[i] = data.point[i];
    }
    if(1){
        pAws->Inf_State_Set_AreaInfo_combine(NULL, false);
    }
    else{
        pAws->Inf_State_Set_AreaInfo_combine((char *)combineArea, false);
    }
}


void CAppInterface::transDoNotDisturb(short status, char *startTime, char *endTime)
{
    // pAws->Inf_State_Set_doNotDisturb(short status, char * startTime, char * endTime, short update); 예비용?
    ceblog(LOG_LV_NECESSARY, CYN," transDoNotDisturb : ");

    eblog(LOG_LV_AWS,"status : "<< status );
    pAws->Inf_State_Set_doNotDisturb_status(status, false);

    if(sendStartTime != nullptr){
        strcpy(sendStartTime,startTime);
        pAws->Inf_State_Set_doNotDisturb_startTime(sendStartTime, false);
        eblog(LOG_LV_AWS," start Time : "<< sendStartTime);
    }
    
    if(sendEndTime != nullptr){
        strcpy(sendEndTime,endTime);
        pAws->Inf_State_Set_doNotDisturb_endTime(sendEndTime, false);
        eblog(LOG_LV_AWS," End Time : "<< sendEndTime);
    }
    

    //eblog(LOG_LV_AWS,"status : "<< status <<" start Time : "<< startTime <<" End Time : "<< endTime);
}

void CAppInterface::transCleaningSchedule(std::list<tCleanSchedule> schedule, short  scheduleNum)
{
    ceblog(LOG_LV_NECESSARY, CYN," transCleaningSchedule : ");

    tCleanSchedule* sendSchedule = new tCleanSchedule[scheduleNum];
    int idx = 0;
    for (const auto& tempData : schedule) {
        if(idx < scheduleNum)
        {
            strcpy(sendSchedule[idx].areas, tempData.areas);
            strcpy(sendSchedule[idx].weeks, tempData.weeks);
            strcpy(sendSchedule[idx].time, tempData.time);
            sendSchedule[idx].isEnabled = tempData.isEnabled;
            sendSchedule[idx].isValid = tempData.isValid;
            sendSchedule[idx].mode = tempData.mode;
            sendSchedule[idx].waterLevel = tempData.waterLevel;

            eblog(LOG_LV_AWS," CleaningSchedule "<<"[" << idx << "] : areas = " << sendSchedule[idx].areas
                                                <<"[" << idx << "] : weeks = " << sendSchedule[idx].weeks
                                                <<"[" << idx << "] : time = " << sendSchedule[idx].time
                                                <<"[" << idx << "] : isEnabled = " << sendSchedule[idx].isEnabled
                                                <<"[" << idx << "] : isValid = " << sendSchedule[idx].isValid
                                                <<"[" << idx << "] : CleanMode = " << sendSchedule[idx].mode
                                                <<"[" << idx << "] : waterLv = " << sendSchedule[idx].waterLevel);
        }
        idx++;
    }
    eblog(LOG_LV_AWS," scheduleNum : "<< scheduleNum);
    pAws->Inf_State_Set_cleanSchedule(scheduleNum,(char *)sendSchedule, false);
}