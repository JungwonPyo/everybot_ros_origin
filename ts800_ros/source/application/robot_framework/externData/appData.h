/**
 * @file appData.h
 * @author jspark
 * @brief app 으로 주고받은 데이터 클래스
 * 
 * @version 0.1
 * @date 2023-08-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "eblog.h"
#include "appInterface.h"

class CAppData : public CAppInterface
{
private:
    short actionCmd =0 ;
public:
    CAppData()
    {
        eblog(LOG_LV,  "");
    }

    ~CAppData()
    {
        ceblog(LOG_LV, GREEN, "");
    }

    bool getAwsConnection()
    {
        return isConnectingAWS();
    }
    /* keyState에서 호출 */
    short getAppCmd()
    {
        if(action.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get app cmd (Update !!)");
            actionCmd = action.get();
            return actionCmd;
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    /* rsBridge에서 호출함 */
    short getApp()
    {
        return actionCmd;
    }

    short getSoundCmd()
    {
        if(sound.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get sound cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<sound.get());
            return sound.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getDryEnabledCmd()
    {
        if(dryEnabled.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dryEnabled cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<dryEnabled.get());
            return dryEnabled.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getDryPowerCmd()
    {
        if(dryPower.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dryPower cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<dryPower.get());
            return dryPower.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getDryHoursCmd()
    {
        if(dryHours.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dryHours cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<dryHours.get());
            return dryHours.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getWaterLvCmd()
    {
        if(waterLv.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get water lv cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<waterLv.get());
            return waterLv.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getCleanModeCmd()
    {
        if(cleanMode.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get clean mode cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<cleanMode.get());
            return cleanMode.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }
    
    short getCleanDurationCmd()
    {
        if(cleanDuration.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get clean duration cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<cleanDuration.get());
            return cleanDuration.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getTiltCmd()
    {
        if(tilt.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get tilt cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<tilt.get());
            return tilt.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getLanguageCmd()
    {
        if(language.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get lan cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<language.get());
            return language.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    short getCountryCmd()
    {
        if(country.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get country cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<country.get());
            return country.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }
    
    short getAreaAllCmd()
    {
        if(operationAreaAll.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get operationAreaAll cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<operationAreaAll.get());
            return operationAreaAll.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }

    std::pair<tSpot*, short> getAreaSpotCmd()
    {
        if(operationAreaSpot.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get operationAreaSpot cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<operationAreaSpot.get().first <<"number" << operationAreaSpot.get().second);
            return operationAreaSpot.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tRoom*, short> getAreaRoomCmd()
    {
        if(operationAreaRoom.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get operationAreaRoom cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<operationAreaRoom.get().first <<"number" << operationAreaRoom.get().second);
            return operationAreaRoom.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tCustom*, short> getAreaCustomCmd()
    {
        if(operationAreaCustom.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get operationAreaCustom cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<operationAreaCustom.get().first <<"number : " << operationAreaCustom.get().second);
            return operationAreaCustom.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tForbiddenLine*, short> getForbiddenLineCmd()
    {
        if(forbiddenLine.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get forbiddenLine cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<forbiddenLine.get().first <<"number : " << forbiddenLine.get().second);
            return forbiddenLine.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tForbiddenRect*, short> getForbiddenRectCmd()
    {
        if(forbiddenRect.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get forbiddenRect cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<forbiddenRect.get().first <<"number : " << forbiddenRect.get().second);
            return forbiddenRect.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tAreaInfo*, short> getAreaInfoCmd()
    {
        if(areaInfo.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get areaInfo cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<areaInfo.get().first <<"number : " << areaInfo.get().second);
            return areaInfo.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tDivideArea*, short> getDivideAreaInfoCmd()
    {
        if(divideAreaInfo.isUpdate())
        {
#if 1//USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get DivideAreaInfo cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<divideAreaInfo.get().first << "number : " << divideAreaInfo.get().second);
            return divideAreaInfo.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    std::pair<tCombieArea*, short> getCombineAreaInfoCmd()
    {
        if(combineAreaInfo.isUpdate())
        {
#if 1//USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get CombineAreaInfo cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<combineAreaInfo.get().first <<"number : " << combineAreaInfo.get().second);
            return combineAreaInfo.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    short getDontDisturbStatusCmd()
    {
        if(dontDisturbStatus.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dontDisturbStatus cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<< dontDisturbStatus.get());
            return dontDisturbStatus.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }
    char* getDontDisturbStartTimeCmd()
    {
        if(dontDisturbStartTime.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dontDisturbStartTime cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<< dontDisturbStartTime.get());
            return dontDisturbStartTime.get();
#else
            return 0;
#endif
        }
        else
        {
            return nullptr;
        }
    }
    char* getDontDisturbEndTimeCmd()
    {
        if(dontDisturbEndTime.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get dontDisturbEndTime cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<< dontDisturbEndTime.get());
            return dontDisturbEndTime.get();
#else
            return 0;
#endif
        }
        else
        {
            return nullptr;
        }
    }

    std::pair<tCleanSchedule*, short> getCleanScheduleCmd()
    {
        if(cleanSchedule.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get cleanSchedule cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<cleanSchedule.get().first <<"number : " << cleanSchedule.get().second);
            return cleanSchedule.get();
#else
            return std::make_pair(nullptr, 0);
#endif
        }
        else
        {
            return std::make_pair(nullptr, 0);
        }
    }

    /* OTA */
    short getOtaForceCmd()
    {
        if(otaForce.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get otaForce cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<otaForce.get());
            return otaForce.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }
    char* getOtaNameCmd()
    {
        if(otaName.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get otaName cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<otaName.get());
            return otaName.get();
#else
            return 0;
#endif
        }
        else
        {
            return nullptr;
        }
    }
    char* getOtaVersionCmd()
    {
        if(otaVersion.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get otaVersion cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<otaVersion.get());
            return otaVersion.get();
#else
            return 0;
#endif
        }
        else
        {
            return nullptr;
        }
    }
    short getOtaScheduledCmd()
    {
        if(otaScheduled.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get otaScheduled cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<otaScheduled.get());
            return otaScheduled.get();
#else
            return 0;
#endif
        }
        else
        {
            return 0;
        }
    }
    char* getOtaScheduleTimeCmd()
    {
        if(otaScheduleTime.isUpdate())
        {
#if USE_AWS_APP_INTERFACE > 0
            ceblog((LOG_LV_NECESSARY | LOG_LV_AWS), BOLDMAGENTA, " 1. get otaScheduleTime cmd (Update !!)");
            ceblog(LOG_LV_AWS, BOLDMAGENTA, ""<<otaScheduleTime.get());
            return otaScheduleTime.get();
#else
            return 0;
#endif
        }
        else
        {
            return nullptr;
        }
    }
};


