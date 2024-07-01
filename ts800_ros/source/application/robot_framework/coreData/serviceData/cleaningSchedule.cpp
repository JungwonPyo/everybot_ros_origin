#include "coreData/serviceData/cleaningSchedule.h"
#include "coreData/serviceData.h"
#include "eblog.h"

CCleaningSchedule::CCleaningSchedule(){}
CCleaningSchedule::~CCleaningSchedule(){}

void CCleaningSchedule::setCleaningSchedule()
{
    if(ServiceData.rsBridge.getCleanSchedule().second != 0){
        tCleanSchedule* cleaningSchedulePtr = ServiceData.rsBridge.getCleanSchedule().first;
        cleaningSchedule.scheduleNumber = ServiceData.rsBridge.getCleanSchedule().second;
        
        if(!cleaningSchedule.schedule.empty())
        {
            cleaningSchedule.schedule.clear();
        }
        for (int i = 0; i < cleaningSchedule.scheduleNumber; ++i) {
            cleaningSchedule.schedule.push_back(cleaningSchedulePtr[i]);
            eblog(LOG_LV_AWS," areaInfo.info "<<"[" << i << "] : time = " << cleaningSchedule.schedule.back().time 
                                    <<"[" << i << "] : active Day = " << cleaningSchedule.schedule.back().weeks
                                    <<"[" << i << "] : clean mode = " << cleaningSchedule.schedule.back().mode
                                    <<"[" << i << "] : water Lv = " << cleaningSchedule.schedule.back().waterLevel
                                    <<"[" << i << "] : areas = " << cleaningSchedule.schedule.back().areas
                                    <<"[" << i << "] : isEnabled = " << cleaningSchedule.schedule.back().isEnabled
                                    <<"[" << i << "] : isValid = " << cleaningSchedule.schedule.back().isValid);
        }
        
        ServiceData.awsData.setSendCleanSchedule();
    }
}
tCleaningSchedule CCleaningSchedule::getCleaningSchedule()
{
    return cleaningSchedule;
}