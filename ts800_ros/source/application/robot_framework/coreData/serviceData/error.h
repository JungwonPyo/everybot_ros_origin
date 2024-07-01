/**
 * @file error.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "ebtypedef.h"
#include "coreData/observer.h"

typedef union
{
    u32 value;
    struct
    {

        u32 wheel_l_curr : 1; // 좌측 휠 부하
        u32 wheel_r_curr : 1; // 우측 휠 부하
        u32 wheel_b_curr : 1; // 더미 휠 부하

        u32 wheel_l_abnormal : 1; // hjkim220104 - 휠 모터 이물 걸림 및 제어 이상 에러 감지
        u32 wheel_r_abnormal : 1; // hjkim220104 - 휠 모터 이물 걸림 및 제어 이상 에러 감지
        u32 wheel_b_abnormal : 1; // hjkim220104 - 휠 모터 이물 걸림 및 제어 이상 에러 감지

        u32 wheel_Notwork_l_error : 1; // motor ext error

        u32 wheel_Notwork_r_error : 1; // motor ext error
        u32 wheel_Notwork_b_error : 1; //더미 ext error
        u32 Mcu_Interrupt_error : 1;   // 배터리 과방전
        u32 batt_error : 1;            // 기타 배터리 오류

        u32 internal_error : 1; // 시스템 내부 오류 (통신, 기타)
        u32 docking_error : 1;  // 충전기를 찾을 없는 경우,...
        u32 trap_cliff : 1;     // 바닥 감지 구속 상태
        u32 lift_cliff : 1;     // 청소기 들림 상태

        u32 Mcu_critical_error : 1;
        u32 batt_adapter_error : 1; // 시스템 내부 오류 (통신, 기타)
        u32 Adc_critical_error : 1; // 비젼 센서 카메라 불량
        u32 Need_Manual_moving : 1; // water block(물이 있는 경우,)

        u32 Communication_eror : 1; // 비젼 센서 초기화 오류
        u32 gyro_initfail : 1;      // 자이로 처기화 오류
        u32 wheel_wrong_mop : 1;
        u32 trap_obstacle : 1;

        u32 No_Mop_or_Tof_Obs : 1; // 초기 청소 시작 시 걸레 미장착 및 tof 장애물 오류

        u32 rsved : 8; // 자이로 처기화 오류

        // u16     rsved         : 5;
    } b;

} RSF_ERROR;

class CError : public CObserver 
{
private:
    RSF_ERROR error; // 시스템 오류 상태 정보
    int errorWheelCurrentCount;
    int errorTrapObstacleCount;

public:
    CError(/* args */);
    ~CError();
    void update(CExternData* pExternData) override;
    void initError();
    void checkSendErrorMessage();
    // RSF_ERROR *getErrorData();    
};
