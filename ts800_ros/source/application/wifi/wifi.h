/**
 * @file wifi.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-11-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"
#include "coreData/serviceData.h"

#define AP_LIMIT_TIME 180 //sec
#define STATION_LIMIT_TIME 60 //sec
#define AWS_LIMIT_TIME 60 //sec

// 0 : 비활성화 
// 1 : ap 에 무한루프  
// 2 : station에서 무한루프
// 3 : AWS에서 무한루프
#define TIME_EXIT_DEBUG_MODE 0 

enum class E_WIFI_CONNECT_STEP
{
    E_CHECK_EXIT,           // AP, STATION 종료 확인
    E_ENTER_AP_MODE,        // AP mode 진입
    E_AP_MODE,              // AP mode 진입후 데이터 갖고오기
    E_EXIT_AP_MODE,         // AP 종료 명령
    E_DONE_AP_MODE,         // AP mode 완료 또는 종료
    E_ENTER_STATION_MODE,   // StationMode로 전환
    E_STATION_MODE,         // home AP 접속
    E_EXIT_STATION_MODE,   // station 종료 명령
    E_DONE_STATION_MODE,    // 완료 
    E_CONNECT_AWS,           //AWS 접속
    E_CHECK_WIFI,         //모든 절차를 완료하고 AWS 접속을 완료한 상태
    E_FAIL_WIFI             // 실패
};

enum class E_WIFI_CANCEL_STEP
{
    E_CHECK_WIFI_STEP,  //멈춘 wifistep 확인
    E_CANCEL_AP,        //AP 종료
    E_CHECK_AP_EXIT,    //AP 종료 확인
    E_CANCEL_STATION,   //station 종료
    E_CHECK_STATION_EXIT,//station 종료 확인
    E_CANCEL_AWS,       //aws 종료
    E_CHECK_AWS_EXIT,    //aws 종료 확인
    E_ENTER_STATION,    //station 연결
    E_STATION_CONECT,    //station 연결
    E_NO_SEQ            //이 시퀀스를 안해도 됨
};

class CWifi
{
    private:
        //CRobotToAWS *AWS; //이거는 appinterface에 있는 거 하면 되고         
        //CRobotConnection *Conn; // 이건 없네?
        //들고와야함 serviceData <>-- rsBridge <>-- appData --> appinterface

        E_WIFI_CONNECT_STEP wifiStep;
        E_WIFI_CANCEL_STEP wifiCancelStep;

        E_WIFI_CONNECT_STEP checkExit();
        E_WIFI_CONNECT_STEP enteringApMode();
        E_WIFI_CONNECT_STEP apMode();
        E_WIFI_CONNECT_STEP exitApMode();
        E_WIFI_CONNECT_STEP doneApMode();
        E_WIFI_CONNECT_STEP enteringStationMode();
        E_WIFI_CONNECT_STEP stationMode();
        E_WIFI_CONNECT_STEP exitStationMode();
        E_WIFI_CONNECT_STEP doneStationMode();
        E_WIFI_CONNECT_STEP connectAws();

        E_WIFI_CANCEL_STEP checkWifiStep();
        E_WIFI_CANCEL_STEP checkApExit();
        E_WIFI_CANCEL_STEP checkStationExit();
        E_WIFI_CANCEL_STEP checkAwsExit();

        bool checkAws();
        bool failWifi();
        char* extract_and_combine(char* A, char* B);
        void displayQRCodeOnLCDMargin(const char *data, int lcd_width, int lcd_height, int qr_size, 
                    int x_pos, int y_pos, int Color, int border_size, int border_color, int margin);

        double apStartTime;
        double stationStartTime;
        double awsStartTime;
        bool failConnection;
        bool bCheckTrigger;
        
        double QRStartTime;
        bool uiFlag;

        char SerialNumber[256];
        char deviceNeme[256];
    public:
        CWifi();
        ~CWifi();
        void initwifi();
        bool wifiProc();
        bool getFailConnection();
        E_WIFI_CONNECT_STEP getWifiStep();
        bool cancelProc();
        bool failProc();
};