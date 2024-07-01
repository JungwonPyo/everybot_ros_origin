#include "coreData/serviceData/doNotDisturb.h"
#include "coreData/serviceData.h"
#include "eblog.h"

CDoNotDisturb::CDoNotDisturb(){}
CDoNotDisturb::~CDoNotDisturb(){}

void CDoNotDisturb::setDoNotDisturb()
{
    if(ServiceData.rsBridge.getDontDisturbStatus() != 0){
        doNotDisturb.status = ServiceData.rsBridge.getDontDisturbStatus();
        ServiceData.awsData.setSendDontDisturbStatus();
    }
    
    if(ServiceData.rsBridge.getDontDisturbStartTime() != nullptr){
        doNotDisturb.startTime = ServiceData.rsBridge.getDontDisturbStartTime();
        ServiceData.awsData.setSendDontDisturbStartTime();
    }
    if(ServiceData.rsBridge.getDontDisturbStartTime() != nullptr){
        doNotDisturb.endTime = ServiceData.rsBridge.getDontDisturbEndTime();
        ServiceData.awsData.setSendDontDisturbEndTime();
    }
}

void CDoNotDisturb::setDistruptStatus(short set)
{
    if(set == 1){
        ceblog(LOG_LV_NECESSARY, CYN, " 방해금지 모드 Trigger On");
        bDistruptMode = true;
    }else{
        ceblog(LOG_LV_NECESSARY, CYN, " 방해금지 모드 Trigger OFF");
    }
    doNotDisturb.status = set;
}

void CDoNotDisturb::setDistruptMode(bool set)
{
    if(set){
        ceblog(LOG_LV_NECESSARY, CYN, " 방해금지 모드 설정됨");
        bDistruptMode = true;
    }else{
        ceblog(LOG_LV_NECESSARY, CYN, " 방해금지 모드 해제됨");
    }
    bDistruptMode = set;
}
bool CDoNotDisturb::isDistruptMode()
{
    return bDistruptMode;
}

tDoNotDisturb CDoNotDisturb::getDoNotDisturb()
{
    return doNotDisturb;
}