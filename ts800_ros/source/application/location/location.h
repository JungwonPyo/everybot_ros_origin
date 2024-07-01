/**
 * @file location.h
 * @author jhnoh
 * @brief 
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/serviceData.h"
#include "control/control.h"
#include "rmcl/rmcl.h"

// location PROC 상태 
enum class E_LOCATION_STATE
{
    NONE,                   // 초기 세팅
    DATA_UPDATE,            // 위치 추정 준비
    RMCL,                   // RMCL을 통한 위치 추정 ( 파티클 필터 생성 및  pose 추정 )
    FAIL_UPDATE,            // 업데이트 실패 ( 맵, 라이다 )
    FAIL_LOCATION,      // 위치 추정 실패
    END,                    // 완료
};
static std::string enumToString(E_LOCATION_STATE value) {
    static const std::unordered_map<E_LOCATION_STATE, std::string> enumToStringMap = {
        { E_LOCATION_STATE::NONE, "NONE," },
        { E_LOCATION_STATE::DATA_UPDATE, "DATA_UPDATE," },
        { E_LOCATION_STATE::RMCL, "RMCL," },
        { E_LOCATION_STATE::FAIL_UPDATE, "FAIL_UPDATE," },
        { E_LOCATION_STATE::FAIL_LOCATION, "FAIL_LOCATION," },
        { E_LOCATION_STATE::END, "END," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_LOCATION_ROTATE_STATE
{
    _1_TURN_LEFT,              // 반시계방향으로 회전
    _2_TURN_RIGHT,             // 시계방향으로 회전
    _3_TURN_LEFT,              // 반시계방향으로 회전
    END,                   // end
};

// LOCATION 정보 구조체
typedef struct
{   
    E_LOCATION_STATE state;                // LOCATION proc 상태
    E_LOCATION_ROTATE_STATE rotateState;       // 회전 상태
    tPose                pose;                // 추정된 로봇 위치
}tLocationInfo;

class CLocation
{
private:
    CRmcl *pRmcl;

public:
    CLocation();
    ~CLocation();

    void initLocation( /* 초기화에 필요한 변수들 */ );
    tLocationInfo locationProc(tPose robotPose);
    
private:
//-----------------------------------------------------------------------//
    tLocationInfo       info;                 // Location 정보
    bool                isSucceedLocation;     // 자기위치추정성공여부
    double targetAng;


    // bool checkMapUpdate;                    // 맵 업데이트 여부 확인
    // bool checkLidarUpdate;                  // 라이다 업데이트 여부 확인

    //procedure 함수 기능 모음
    E_LOCATION_STATE procNone();
    E_LOCATION_STATE procRmcl(tPose robotPose);
    E_LOCATION_STATE procFailLocation();
    E_LOCATION_STATE procFailUpdate(); 
    void locationEnd(); 



    // other
    void checkLocalizeProc(tPose robotPose);
    E_LOCATION_ROTATE_STATE getLocationRotateState();
    void setLocationState(E_LOCATION_STATE set);
    void setLocationRotateState(E_LOCATION_ROTATE_STATE set);
/*-------------------------------디버그용-------------------------------*/
public: /* Debug용 함수 모음 */
    void __debug_state_print();    
    std::string enumToString(E_LOCATION_STATE state);

};