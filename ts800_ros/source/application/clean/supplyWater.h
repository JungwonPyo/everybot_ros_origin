/**
 * @file supplyWater.h
 * @author 담당자 미정
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "ebtypedef.h"
#include "control/control.h"

typedef enum
{
    WATERSUPPLY_VOID = 0,
    WATERSUPPLY_START   = 1,
    WATERSUPPLY_RUNNING = 2,
    WATERSUPPLY_PAUSE   = 3,
    WATERSUPPLY_END     = 4

}E_WATERSUPPLY_STATE;

typedef struct
{
    E_WATERSUPPLY_STATE state;
	bool water_block; //danielhong:210114 : 추후 필요에 의해 물공급을 중단할 경우
	bool timeout; //danielhong:210222 : NORMAL_WATERSUPPLY_TIME이 지나면 사용자의 리모콘 개입 전까지 물공급 차단, 최대물공급 시간 초과(145분) 시 TRUE
	bool is_draining; //danielhong:210406 : 잔수 제거 기능 추가
    bool isPumpActive;
	bool period_update;	
	u8 period_count; // 물공급 주기 카운트, HD 201014
	u8 supply_count; //hjkim220112 - 물공급 전체 주기에서 물공급을 실행한 횟수
	u8 water_blockcount; //hjkim220112 - 물공급 전체 주기에서 물공급을 차단한 횟수

	u32 pump_activetick;//펌프모터 timing 관리 시작시간
	u32 period_starttick;
	u32 WaterBlockstarttick;//펌프모터 timing 관리 시작시간 

	u32 supply_interval; //물 공급 간격(ontime per cycle)
	double supply_period;
	u32 supplytime;      //서비스 중 총 물 공급 시간 합계

    bool drain_active;
    u32 drain_starttick;

    E_WATER_PUMP_STEP level; // 물공급 레벨
    bool awsChecker;         // app으로 키 입력 받았는지(입력 받으면 false)
    bool drain_exit_flag;    // 잔수제거가 on->off 되면 true (aws에서 필요한 데이터)
	
}tWaterSupplyInfo;


class CSupplyWater
{
private:
    tWaterSupplyInfo watersupply_;
    //start - waterSupply 함수 모음
    double getWaterSupplyPeriod(u32 cleanTime);
    double getLevel1Period(u32 cleanTime);
    bool updateWaterSupplyPeriod(double period);
    E_WATERSUPPLY_STATE startWaterSupply(double startTime);
    E_WATERSUPPLY_STATE pauseWaterSupply(void);
    E_WATERSUPPLY_STATE runWaterSupply(void);
    E_WATERSUPPLY_STATE endWaterSupply(void);
    
    E_WATER_PUMP_STEP ChangeDirectLevel(short waterLv);
    E_WATER_PUMP_STEP ChangeStepLevel();
    //end - waterSupply 함수 모음
    E_WATER_PUMP_STEP convertWaterLevel(short waterLv);

public:
    CSupplyWater();
    ~CSupplyWater();

    void initWaterSupplyData(void);
    void WaterSupplyStateMachine(double startTime);
    void waterDrainONOFF();
    bool isActiveWaterDrain();
    bool checkWaterDrainTimeOut();
    void startWaterSupply();
    void stopWaterSupply();
    void ChangeWaterLevel(short waterLv = 0);
    E_WATER_PUMP_STEP getWaterLevel();
  
    #if WATER_SUPPLY_TEST > 0
    bool waterSupplytest;
    bool isPumpIntervalEnd();//for test
    #endif
};
