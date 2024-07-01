#pragma once

#include "commonStruct.h"
#include "rsuContext.h"
#include "ebtypedef.h"
#include "ebtime.h"

class CServiceMacro
{
private:
    std::list<contextDo> doList;
public:
    CServiceMacro();
    ~CServiceMacro();

    std::list<contextDo> getDoList();
    void popDolist();
    void clearDolist();
    // 이전 서비스 취소 -> 새로운 서비스 실행 -> 새로운 서비스 완료 -> 기본 IDLE 서비스로 전환
    void macroServiceConnector(E_SERVICE_ID doId);
    void macroServiceStart(E_SERVICE_ID curId, E_SERVICE_ID doId);
    // 이전 서비스 취소 하고 idle 서비스로 전환
    void macroServiceStop(E_SERVICE_ID curId);
    // 서비스 일시정지
    void macroServicePause(E_SERVICE_ID curId);
    void macroServiceResume(E_SERVICE_ID curId);
    // 부팅해서 idle 서비스 가는 로직.
    void bootIdle();

    void macroServiceCharge(E_SERVICE_ID curId);

    // 언 도킹 -> 입력된 서비스 활성화 -> 도킹 서비스 활성화
    void macroProcess(E_SERVICE_ID curId, E_SERVICE_ID doId);
    // 입력된 서비스 활성화 -> 도킹 서비스 활성화
    void macroRunOnDocked(E_SERVICE_ID curId, E_SERVICE_ID doId);

    // 입력된 서비스 활성화 -> 도킹 서비스 활성화
    void macroRunOnStanby(E_SERVICE_ID curId, E_SERVICE_ID doId);

};