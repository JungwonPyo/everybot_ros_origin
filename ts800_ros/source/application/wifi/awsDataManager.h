#pragma once
#include "MessageHandler.h"
#include "commonStruct.h"
#include "coreData/serviceData.h"
#include <systemTool.h>
#include "define.h"
#include "userInterface.h"
#include  "subTask.h"

#define FIRST_AWS 0 // 부팅연결 시퀀스 옵션 0: station -> aws, 1: aws ->station


class CAwsDataManager
{
private:
    CRsuContext* pContext;

    double AwsMapSendTick;
 
    int tempCount = 0; //AWS 메시지 호출 횟수

public:
    CAwsDataManager(CRsuContext* pContext);
    ~CAwsDataManager();


    bool checkSendMapTime();
    void sendMapDataToAws2();
};