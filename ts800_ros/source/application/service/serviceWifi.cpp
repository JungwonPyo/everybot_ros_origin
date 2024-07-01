#include "serviceWifi.h"
#include "userInterface.h"
#include "commonStruct.h"
#include "control/motionPlanner/motionPlanner.h"
#include "MessageHandler.h"


CServiceWifi::CServiceWifi(CServiceReady* _pServiceReady) : pServiceReady(_pServiceReady)
{
    pWifi = new CWifi();

    //powerMode = MODE_SLEEP;
    eblog(LOG_LV, "service wifi create");
}

CServiceWifi::~CServiceWifi()
{
    eblog(LOG_LV, "");
}    
bool CServiceWifi::checkReadytoWifi()
{
/*     확인해야하는거
    1. 걸레건조 중지??
    2. 폰에서 SSID 받아오기? */
}

void CServiceWifi::serviceInitial()
{
    eblog(LOG_LV_NECESSARY, "serviceInitial - from CServiceWifi Class");
    
    setServiceStatus(E_SERVICE_STATUS::startup); 
} 

E_SERVICE_STATUS_STEP CServiceWifi::startupStepReady() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    eblog(LOG_LV_NECESSARY, "serviceWifi - startup");
    ceblog(LOG_LV_UI, CYN, "UI <- wifi 서비스 시작");
    
    DISPLAY_CTR.stopAutoDisplay();
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::ENTERING_WIFI);
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WIFI_CONNECTION_START);
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
    //ROBOT_CONTROL.tilting.tilting(E_TILTING_CONTROL::STOP);
    
#if USE_AWS_APP_INTERFACE
    //phone interface 관련 데이터 변수 업데이트 on
    ServiceData.rsBridge.setWifiServiceFlag(true);
    pWifi->initwifi(); 
    completedByForce = false;
#else
    completedByForce = false;
#endif

    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::startupStepExecuting() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;
    if(!MOTION.isRunning()){
        if (ServiceData.rsBridge.getrsMode() == (char)0) //pServiceReady->runReayServiceStep(E_SERVICE_ID::WIFI)
        {
            /* wifi 서비스가 시작 될 때 UI 추가 */
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            setServiceStartTime();
        }
    }

    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::startupStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::startupStepTerminaition() 
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    setServiceStatus(E_SERVICE_STATUS::running);

    return ret;
}

E_SERVICE_STATUS_STEP CServiceWifi::runningStepReady() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;


    return ret;
}

E_SERVICE_STATUS_STEP CServiceWifi::runningStepExecuting() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

#if USE_AWS_APP_INTERFACE
    if(pWifi->wifiProc())
    {
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }
#else
    ret = E_SERVICE_STATUS_STEP::TERMINAITION;
#endif
    //ret = E_SERVICE_STATUS_STEP::EXECUTING;

    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::runningStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::runningStepTerminaition() 
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    setServiceStatus(E_SERVICE_STATUS::completed);

    return ret;
}

E_SERVICE_STATUS_STEP CServiceWifi::pauseStepReady()
{

}
E_SERVICE_STATUS_STEP CServiceWifi::pauseStepExecuting()
{

}
E_SERVICE_STATUS_STEP CServiceWifi::pauseStepWaiting() 
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::pauseStepTerminaition()
{

}

E_SERVICE_STATUS_STEP CServiceWifi::completedStepReady() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

#if USE_AWS_APP_INTERFACE
    eblog(LOG_LV_NECESSARY,  "[CServiceWifi Class]");
    
    if(completedByForce || pWifi->getFailConnection())
    {
        /* 외부에서 강제 완료(취소) 시킨 경우 */
        eblog(LOG_LV_AWS,  "completed By Force");
        /* UI */
        if(completedByForce){
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::CANCEL_WIFI_CONNECTION);
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WIFI_CONNECTION_CANCEL);
        }
        else if (pWifi->getFailConnection()){
            DISPLAY_CTR.startDisplay(E_DisplayImageClass::WIFI_CONNECTION_FAILED);
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WIFI_CONNECTION_FAIL);
        }

        /* 강제 종료에 대한 설정 */
        exitStartTime = SYSTEM_TOOL.getSystemTime();
        if (pWifi->getWifiStep() == E_WIFI_CONNECT_STEP::E_EXIT_STATION_MODE || pWifi->getWifiStep() == E_WIFI_CONNECT_STEP::E_DONE_STATION_MODE ||
            pWifi->getWifiStep() == E_WIFI_CONNECT_STEP::E_CONNECT_AWS || pWifi->getWifiStep() == E_WIFI_CONNECT_STEP::E_CHECK_WIFI) 
        {
            sleep(2);// aws가 2초뒤에 생성되어서 시간 맞출려고 2초 슬립줌
        }
        
        // ap,station,aws 종료 시킴
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::EXIT_ALL)));
        
    }
    else
    {
        /* 내부에서 완료 된 경우 */
        eblog(LOG_LV_AWS,  "completed By Nature");
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::WIFI_CONNECTION_COMPLETE);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WIFI_CONNECTION_SUCCESS);
    }
#endif
    
    
    return ret;
}

E_SERVICE_STATUS_STEP CServiceWifi::completedStepExecuting() 
{
    auto ret = E_SERVICE_STATUS_STEP::EXECUTING;

    if(completedByForce || pWifi->getFailConnection()){
        if(completedByForce){
            bool isExitDone = pWifi-> cancelProc();
            if(isExitDone){
                ServiceData.rsBridge.setWifiServiceFlag(false);
                ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            }
        }
        else if (pWifi->getFailConnection()){
            bool isExitDone = pWifi-> cancelProc();
            if(isExitDone){
                ServiceData.rsBridge.setWifiServiceFlag(false);
                ret = E_SERVICE_STATUS_STEP::TERMINAITION;
            }
        }
        
        if(SYSTEM_TOOL.getSystemTime()- exitStartTime > 60){
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION)));    
            ret = E_SERVICE_STATUS_STEP::TERMINAITION;
        }
    }
    else
    {
        eblog(LOG_LV_AWS,  "completed By Nature");
        ret = E_SERVICE_STATUS_STEP::TERMINAITION;
    }

    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::completedStepWaiting()
{
    auto ret = E_SERVICE_STATUS_STEP::WAITING;
     
    return ret;
}
E_SERVICE_STATUS_STEP CServiceWifi::completedStepTerminaition() 
{
    auto ret = E_SERVICE_STATUS_STEP::TERMINAITION;

    return ret;
}