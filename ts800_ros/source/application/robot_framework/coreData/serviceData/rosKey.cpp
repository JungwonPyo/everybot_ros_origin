#include "coreData/serviceData/rosKey.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CRosKey::CRosKey() {}

CRosKey::~CRosKey() {}

void CRosKey::initRosKey()
{
    rosKey = E_ROS_KEY::NONE;
}

void CRosKey::setRosKey(std::string cmd)
{
    if      (cmd == "movestop")             rosKey = E_ROS_KEY::MOVING_STOP;
    else if (cmd == "movego")               rosKey = E_ROS_KEY::MOVING_UP;
    else if (cmd == "moveback")             rosKey = E_ROS_KEY::MOVING_DOWN;
    else if (cmd == "turnleft")             rosKey = E_ROS_KEY::MOVING_LEFT;
    else if (cmd == "turnright")            rosKey = E_ROS_KEY::MOVING_RIGHT;

    else if (cmd == "STOP")                 rosKey = E_ROS_KEY::STOP;
    else if (cmd == "CLEAN")                rosKey = E_ROS_KEY::CLEAN;
    else if (cmd == "HOME")                 rosKey = E_ROS_KEY::HOME;
    else if (cmd == "EXPLORE")              rosKey = E_ROS_KEY::EXPLORER;
    else if (cmd == "WIFI")                 rosKey = E_ROS_KEY::APP_CONNECT;

    else if (cmd == "TILTING_UP")           rosKey = E_ROS_KEY::TILTING_UP;
    else if (cmd == "TILTING_DONW")         rosKey = E_ROS_KEY::TILTING_DONW;

    else if (cmd == "DRY_MOP")              rosKey = E_ROS_KEY::DRY_MOP;
    else if (cmd == "DRAIN_WATER")          rosKey = E_ROS_KEY::DRAIN_WATER;
    //else if (cmd == "VOLUME")               rosKey = E_ROS_KEY::VOLUME_OPTION;
    else if (cmd == "WATER")                rosKey = E_ROS_KEY::WATER_OPTION;

    else if (cmd == "SLAM_ON")              rosKey = E_ROS_KEY::SLAM_ON;
    else if (cmd == "SLAM_OFF")             rosKey = E_ROS_KEY::SLAM_OFF;
    else if (cmd == "SAVE_MAP")             rosKey = E_ROS_KEY::SAVE_MAP;
    else if (cmd == "DELETE_MAP")           rosKey = E_ROS_KEY::DELETE_MAP;
    
    else if (cmd == "IMU_INIT")             rosKey = E_ROS_KEY::IMU_INIT;
    else if (cmd == "TOF_INIT")             rosKey = E_ROS_KEY::TOF_INIT;

    else if (cmd == "FW_UPDATE")             rosKey = E_ROS_KEY::FW_UPDATE;
    else if (cmd == "FW_RECOVERY")           rosKey = E_ROS_KEY::FW_RECOVERY;
    else if (cmd == "RESERVATION_CLEAN")     rosKey = E_ROS_KEY::RESERVAITON_CLEAN;
    else if (cmd == "RESERVATION_STOP")      rosKey = E_ROS_KEY::RESERVATION_STOP;
    else if (cmd == "INIT_USERSET")          rosKey = E_ROS_KEY::INIT_USER_OPTION;
    else if (cmd == "FACTORY_RESET")         rosKey = E_ROS_KEY::FACTORY_RESET;

    else if (cmd == "POWER_OFF")            rosKey = E_ROS_KEY::POWER_OFF;
    else if (cmd == "AP_RESET")             rosKey = E_ROS_KEY::AP_RESET;
    else if (cmd == "MCU_RESET")            rosKey = E_ROS_KEY::MCU_RESET;
    

    // else if (cmd == "RVIZ_GOAL")            rosKey = E_KEY_FUNCTION::RVIZ_GOAL;
    // else if (cmd == "PATTERN_1")            rosKey = E_KEY_FUNCTION::PATTERN_1;
    // else if (cmd == "PATTERN_2")            rosKey = E_KEY_FUNCTION::PATTERN_2;
    // else if (cmd == "PATTERN_3")            rosKey = E_KEY_FUNCTION::PATTERN_3;
    // else if (cmd == "PATTERN_4")            rosKey = E_KEY_FUNCTION::PATTERN_4;
    // else if (cmd == "PATTERN_5")            rosKey = E_KEY_FUNCTION::PATTERN_5;
    // else if (cmd == "PATTERN_6")            rosKey = E_KEY_FUNCTION::PATTERN_6;
    // else if (cmd == "PATTERN_7")            rosKey = E_KEY_FUNCTION::PATTERN_7;
    // else if (cmd == "PATTERN_8")            rosKey = E_KEY_FUNCTION::PATTERN_8;
    // else if (cmd == "PATTERN_9")            rosKey = E_KEY_FUNCTION::PATTERN_9;
    // else if (cmd == "PATTERN_10")           rosKey = E_KEY_FUNCTION::PATTERN_10;

    else                                    rosKey = E_ROS_KEY::NONE;

    if(rosKey != E_ROS_KEY::NONE)   { ceblog(LOG_LV_NECESSARY, CYN, " rosKey[" << cmd << "]");}
}

E_ROS_KEY CRosKey::getRosKey()
{
    E_ROS_KEY ret = rosKey;
    rosKey = E_ROS_KEY::NONE;
    return ret;
}
