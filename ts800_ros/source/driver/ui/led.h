/**
 * @file led.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"


#define onTime 2.5
#define offTime 2.5
#define totalTime 30
#define LED_LUX 30
class CLed
{
public:
    CLed();
    ~CLed();
private:
    // 
    u32 ledOnTick;
    u32 ledOffTick;
    u32 ledBlinkStartTick;
    
    u32 btnLedOnTick;
    bool ledLR = false;
	
    void playLed(u32 dir, u8 r, u8 g, u8 b, u8 w);
	//void ledFunc(E_LED_DIRECTION dir, E_LED_COLOR color);
	// 깜박임 기능, 껍데기만 있음
	// OnTime: LED 켜져있는 시간, OffTime LED 꺼져있는 시간, TotalTime LED 총 깜박이는 시간 

public:

    void ledFunc(E_LED_DIRECTION dir, E_LED_COLOR color);
    void ledBlinkPlay();
    void setLedTick();

    void ledAllOff();

    void setBtnLedTick();
    u32 getBtnLedTick();
    void btnLedBlue();
    void btnLedWhite();

    void playChargingServiceStart();
    void playChargingServiceEnd();

    void playCleanServiceStart();
    void playCleanServiceEnd();

    void playDockingServiceStart();
    void playDockingServiceEnd();

	void playExplorerServiceStart();
	void playExplorerServiceEnd();


    /* 동작/제어 */
    // {
    //시작/정지
    void powerOn();
    void poweroff();
    void startFunc();
    void pause();
    //청소 영역
    void cleanHome();
    void cleanRoom();
    void cleanRange();
    void cleanFocus();
    //청소 옵션
    void optionBasic();
    void optionDetail();
    void optionQuick();
    void optionRepeat();
    void optionroom();
    //걸레 종류
    void normal();
    void onetime();
    //물공급량
    void waterZero();
    void waterOnetime();
    void waterOne();
    void waterTwo();
    //잔수 제거
    void deleteWater();
    //Wi-fi
    void wifiInit();
    void connectWifi();
    void changeWifi();
    //예약설정
    void setBook();
    //차일드락
    void childLcok();
    //음량조절
    void ctrVol();
    //틸팅
    void tiltingUp();
    void tiltingDown();
    //초기화
    void setInit();
    void setInitF();
    //f/w업데이트
    void updateFw();
    //슬립모드
    void sleepMode();
    // }

    /* 충전 스테이션 */
    // {
    //전원 on/off

    //걸레건조 설정
    void startDry();
    void stopDry();
    void setDry();
    void setDryTime();
    // }

    /* 표시/알림 */
    // {
    //배터리잔량
    void batteryOne();
    void batteryTwo();
    void batterythr();
    //배터리부족
    void batteryZero();
    //청소 상태
    void cleaningmode();
    //위치찾기
    void tmpfunc();
    //알림 메시지
    void sendMsg();
    //오류
    void wheelError();
    void MoveError();
    void liftupRsf();
    void chargingError();
    void chargingMoveError();
    void connectWifiError();
    void sensorError();
    void lidarError();
    void etcError();
    // }

    /* 지도설정 */
    // {
    //지도생성
    void makeMap();
    void delMap();
    //금지영역 설정
    void setBanSite();
    //지도 편집
    void sepaMap();
    void combMap();
    void changeRoomName();
    void rotationMap();
    void changeMapname();
    // }

    /* 기타 */
    // {
    //청소 재개
    void resumeClean();
    //청소 기록
    void cleanRecord();
    //로봇 공유
    void qrCode();
    void account();
    // }
};
