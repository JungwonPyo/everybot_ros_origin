#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <cstdlib> // for system()
#include <sstream>
#include <cstdio>
#include <cstring>
#include <sys/types.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>

#include "taskPowerOff.h"
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "control/control.h"
#include "motionController.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/
/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskPowerOff::CTaskPowerOff()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskPowerOff::~CTaskPowerOff()
{
    CStopWatch __debug_sw;

    setState(POWER_OFF_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskPowerOff::setState(POWER_OFF_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[POWER_OFF_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskPowerOff::taskStart()
{
    setState(POWER_OFF_STATE::START);
    startTime = SYSTEM_TOOL.getSystemTime();
}

bool CTaskPowerOff::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    switch (state)
    {
    case POWER_OFF_STATE::NONE :
        /* code */
        break;
    case POWER_OFF_STATE::START :
        setState(startPowerOff());
        break;
    case POWER_OFF_STATE::RUN :
         setState(runPowerOff());
        break;
    case POWER_OFF_STATE::COMPLETE :
         setState(completePowerOff());
        break;            
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

POWER_OFF_STATE CTaskPowerOff::startPowerOff()
{
    POWER_OFF_STATE ret = POWER_OFF_STATE::START;
    
#if 1 //임시 코드
    tRobotOption writeData;
    writeData.waterLevel = 10;
    writeData.soundLv = 5;
    writeData.country = 1;

    tCleanSchedule schedule1 = {"08:00", "0110000", 1, 3, "Area1", 1, 1};
    tCleanSchedule schedule2 = {"14:00", "1001111", 2, 2, "Area2", 1, 1};
    writeData.cleanSchedule.push_back(schedule1);
    writeData.cleanSchedule.push_back(schedule2);
#endif

    const std::string filename = "/home/ebot/RobotOption.yaml";
    utils::fileIO::saveRobotOption(filename, writeData);
    DISPLAY_CTR.stopAutoDisplay();
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::OFF);
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_GOOD_DAY);
    LED_CTR.ledAllOff();
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);
    ret = POWER_OFF_STATE::RUN;
    return ret;
}

POWER_OFF_STATE CTaskPowerOff::runPowerOff()
{
    POWER_OFF_STATE ret = POWER_OFF_STATE::RUN;

    if(!MOTION.isRunning() && !DISPLAY_CTR.isLCDPlaying() && !SOUND_CTR.isSoundPlaying()){
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_POWER, tMsgPowerOff()));
        processShutDownProcessor(false); //TS800 은 0.5초 이후 kill
        ret = POWER_OFF_STATE::COMPLETE;
    }
              
    return ret;
}

int CTaskPowerOff::processShutDownProcessor(bool isKillTS800)
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

POWER_OFF_STATE CTaskPowerOff::completePowerOff()
{
    POWER_OFF_STATE ret = POWER_OFF_STATE::COMPLETE;
    processShutDownProcessor(true); //TS800 은 0.5초 이후 kill

    ret = POWER_OFF_STATE::NONE;
    return ret;
}