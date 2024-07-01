#include "taskFirmwareUpdate.h"
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "control/control.h"
#include "motionController.h"
#include <iostream>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

#define     OTA_STATE_UNKNOWN       99
#define     OTA_STATE_START         0
#define     OTA_STATE_TAR           1
#define     OTA_STATE_ING           2
#define     OTA_STATE_COMPLETE      3
#define     OTA_STATE_FAIL          4

/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskFirmwareUpdate::CTaskFirmwareUpdate()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskFirmwareUpdate::~CTaskFirmwareUpdate()
{
    CStopWatch __debug_sw;

    setState(OTA_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskFirmwareUpdate::setState(OTA_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskFirmwareUpdate::taskStart()
{
    setState(OTA_STATE::START);
    startTime = SYSTEM_TOOL.getSystemTime();
    debugTime = SYSTEM_TOOL.getSystemTime();
    ROBOT_CONTROL.systemOpenCloseOta(true);
    
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::UPDATE_START);
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_FIRMWARE_UPDATE_START);
}

bool CTaskFirmwareUpdate::taskRun()
{    
    CStopWatch __debug_sw;
    
    int percent = ServiceData.Ota.getSysData().pecent;
    double runTime = SYSTEM_TOOL.getSystemTime()-startTime;
    bool ret = false;

    switch (state)
    {
    case OTA_STATE::NONE :
        /* code */
        break;
    case OTA_STATE::START :
        setState(startUpdate());
        break;
    case OTA_STATE::RUN :
        setState(runUpdate(percent,runTime));
        break;
    case OTA_STATE::RECOVERY :
        /* code */
        break;
    case OTA_STATE::COMPLETE :
        setState(completeUpdate());
        break;
    case OTA_STATE::REBOOT:
        setState(reBoot());
        ret = true;
        break;            
    default:
        break;
    }

    if(SYSTEM_TOOL.getSystemTime()-debugTime > 1){
        debugTime = SYSTEM_TOOL.getSystemTime();
        ceblog(LOG_LV_NECESSARY, CYN, "[OTA - ING.....] STATE : " << enumToString(state) << "OTA_STATE : " << ServiceData.Ota.getSysData().state << " PERCNET : " << percent << " RUNTIME : " << runTime);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

OTA_STATE CTaskFirmwareUpdate::startUpdate()
{
    OTA_STATE ret = OTA_STATE::START;

    if(ServiceData.Ota.getSysData().bOpen){
        ROBOT_CONTROL.systemStartOta();
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::UPDATING);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_FW_UPDATING);
        ret = OTA_STATE::RUN;
    }
    return ret;
}
OTA_STATE CTaskFirmwareUpdate::runUpdate(int percent, double runTime)
{
    OTA_STATE ret = OTA_STATE::RUN;
    int ota_state = ServiceData.Ota.getSysData().state;
    if(runTime >= CONFIG.ota_timeout)
    {
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::UPDATE_FAILED);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_FAIL_UPDATE);
        ROBOT_CONTROL.systemOpenCloseOta(false);
        ret = OTA_STATE::COMPLETE;
        ceblog(LOG_LV_NECESSARY, CYN, "[TIMEOUT - CLOSE OTA!!] STATE : RUNTIME : " << runTime);
    }
    // else if(ota_state == OTA_STATE_START){
    //     ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : START PERCNET : " << percent );
    // }else if(ota_state == OTA_STATE_TAR){
    //     ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : TAR PERCNET : " << percent );
    // }else if(ota_state == OTA_STATE_ING){
    //     ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : ING PERCNET : " << percent );    }
    else if(ota_state == OTA_STATE_COMPLETE){
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::UPDATE_CANCEL);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_UPDATE);
        ROBOT_CONTROL.systemOpenCloseOta(false);
        ret = OTA_STATE::COMPLETE;
        ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : COMPLETE PERCNET : " << percent );
    }else if(ota_state == OTA_STATE_FAIL){
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::UPDATE_FAILED);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_FAIL_UPDATE);
        ROBOT_CONTROL.systemOpenCloseOta(false);
        ret = OTA_STATE::COMPLETE;
        ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : FAIL PERCNET : " << percent );
    }
    // else if(ota_state == OTA_STATE_UNKNOWN){
    //     ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : UNKOWN PERCNET : " << percent );
    // }
    // else if(ota_state != OTA_STATE_START && ota_state != OTA_STATE_TAR && ota_state != OTA_STATE_ING && ota_state != OTA_STATE_UNKNOWN){
    //     ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::RUN] : UNKOWN ota_state : " << ota_state );
    // }
    return ret;
}
OTA_STATE CTaskFirmwareUpdate::completeUpdate()
{
    OTA_STATE ret = OTA_STATE::COMPLETE;
    if(!ServiceData.Ota.getSysData().bOpen)
    {
        processShutDownProcessor(false);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::LOGO_EVERYBOT);//OFF 이미지를 넣을지 선택하자
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_ROBOT_RESTART);
        ceblog(LOG_LV_NECESSARY, CYN, "[OTA_STATE::COMPLETE] OTA CLOSE!! RE-BOOT " );
        ret = OTA_STATE::REBOOT;
    }

    return ret;
}

OTA_STATE CTaskFirmwareUpdate::reBoot()
{
    OTA_STATE ret = OTA_STATE::REBOOT;
    processShutDownProcessor(true);
    system("sudo reboot");
    ret = OTA_STATE::NONE;
    return ret;
}

int CTaskFirmwareUpdate::processShutDownProcessor(bool isKillTS800)
{
    int exit_result = 0;

    eblog(LOG_LV_NECESSARY, "processShutDownProcessor  ");
 
    if (!isKillTS800)
    {
        exit_result = system("ps -ef | sed -n '/static_transform_publisher/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext static_transform_publisher kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext static_transform_publisher kill");
        }
        exit_result = system("ps -ef | sed -n '/imu_filter_node/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext imu_filter_node kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext imu_filter_node kill");
        }
        exit_result = system("ps -ef | sed -n '/cartographer/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext cartographer kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext cartographer kill");
        }
        exit_result = system("ps -ef | sed -n '/roscore/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext roscore kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext roscore kill");
        }
        exit_result = system("ps -ef | sed -n '/rosmaster/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext rosmaster kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext rosmaster kill");
        }
        exit_result = system("ps -ef | sed -n '/rosout/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext rosout kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext rosout kill");
        }
        exit_result = system("ps -ef | sed -n '/roslaunch/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext roslaunch kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext roslaunch kill");
        }
        exit_result = system("ps -ef | sed -n '/everybot_odom_handler/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext everybot_odom_handler kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext everybot_odom_handler kill");
        }
        exit_result = system("ps -ef | sed -n '/everybot_msgs/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext everybot_msgs kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext everybot_msgs kill");
        }
    }
    else
    {
        //mcu power off 가 보내지는 시간을 고려해서 일단 0.5초 이후 ts800을 죽여 버리자.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        exit_result = system("ps -ef | sed -n '/ts800_app/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext ts800_app kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "*ext ts800_app kill");
        }
    }

    return exit_result;
}

    