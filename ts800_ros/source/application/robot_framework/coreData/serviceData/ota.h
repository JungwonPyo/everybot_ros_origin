#pragma once

#include "coreData/observer.h"
#include "commonStruct.h"
#include "interfaceStruct.h"

typedef struct tappOta
{
    tappOta() {}

    short force;        // 1 = false // 2 = true (충전 중 일때 바로 업데이트 시작해) 
    char name[256];     // 파일명. 업데이트 경로에 + 파일명을 붙일때 쓴다 
    char version[256];  // 여러가지 버전이 들어있음 string...협의 필요.
    short scheduled;    // 1 = false 예약 업데이트 사용 x // 2 = true 정해진 시간에 업데이트를 해라 
    char scheduleTime[256]; //예약 시간 정보
}tappOta;

class COta : public CObserver
{
private:
    tappOta appData;
    tSysOta sysData;
    bool bRunning;
    char otaPath[256];
    char path[256];

public:
    COta();
    ~COta();
    void update(CExternData* pExternData) override;

    void setAppData();
    tappOta getAppData();
    
    void updateSysOtaData(tSysOta _sysData);
    tSysOta getSysData();
    void setFirmWareUpdateFlag(bool set);
    bool isRunning();

    char* getOtaPath();
};