/**
 * @file batteryState.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "ebtypedef.h"
#include "interfaceStruct.h"
#include "coreData/observer.h"
#include "commonStruct.h"

#define BATTERY_CUTOFF_VOLTATE 8.0//8.25 //hjkim230713 - 배터리 방전 전압

//24.04.19 - HW 팀과 상의하여 결정.

//24.06.04 - HW 팀
//1. 배터리 전압 범위 조정 요청 (기존 : 10V~12.35V -> 변경 : 9.6V~12.35V) 
//-> 부하에 따라 전압이 흔들리기 때문에 0.4V정도 Offset 필요
//-> FW에서 배터리 전압 데이터 이동 평균 적용되어야하고, SW 배터리 전압 범위 수정이 필요합니다.
//2. Current limit 저항 변경하여 동작시간 확인 (전압 범위 변경 9.6V ~ 12.35V)
//- 1.5Kohm (1.67A limit) (기존) : 동작시간 약 110분
//- 2Kohm (1.85A limit) : 동작시간 129분
//- 3Kohm (2.08A limit) : 동작시간 145분
//24.06.04 - HW 팀
#define MIN_BATTERY_VOLTAGE 9.6 // 10.0 -> 9.6
#define MAX_BATTERY_VOLTAGE 12.35

#define NEED_CHARGE_PER 5
#define LOW_BATTERY_PER 35
#define MIDDLE_BATTERY_PER 70
#define HIGH_BATTERY_PER 99


class CBatteryState : public CObserver
{
private :
    bool bUpdateState;
    bool stateChanged; 
    double stateChangeStartTime;
    double stateUpdateStartTime;   
    u8 percent;
    double volt, filteredVolt;
    double current;
    E_BATTERY_STATE state;
    E_BATTERY_STATE tempState;

    void initBattery();
    double lowPassFilter(double currentValue, double previousValue, double alpha);
    bool checkStateChange();
    void setBatteryState(E_BATTERY_STATE set);
    void setTempState(E_BATTERY_STATE set);

    E_BATTERY_STATE checkVoidState(u8 percent);
    E_BATTERY_STATE checkNeedChargeState(u8 percent);
    E_BATTERY_STATE checkLowState(u8 percent);
    E_BATTERY_STATE checkMidState(u8 percent);
    E_BATTERY_STATE checkHihgState(u8 percent);
    E_BATTERY_STATE checkFullState(u8 percent);
    bool battStateChecker(u8 percentage);
    void updateBatteryState(tSysBattery batt);
    bool isBatteryStateChanged();

public :
    CBatteryState();
    ~CBatteryState();
    void update(CExternData* pExternData) override;
    double getBatteryCurrent();
    E_BATTERY_STATE getBatteryState();
    u8 getBatteryPercent();
    double getBatteryVolt();
    bool isBatteryNeedCharge();
    bool isUpateState();
    double getStateChangeTime();
};