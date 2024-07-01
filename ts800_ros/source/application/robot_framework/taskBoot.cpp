#include "taskBoot.h"
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
CTaskBoot::CTaskBoot()
{
    CStopWatch __debug_sw;
    setState(BOOT_STATE::NONE);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskBoot::~CTaskBoot()
{
    CStopWatch __debug_sw;

    setState(BOOT_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskBoot::setState(BOOT_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[BOOT_STATE] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

BOOT_STATE CTaskBoot::getState()
{
    return state;
}

void CTaskBoot::taskStart()
{
    setState(BOOT_STATE::START);
    startTime = SYSTEM_TOOL.getSystemTime();
    MOTION.startStopOnMap(tProfile(),false);
}

bool CTaskBoot::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    switch (state)
    {
    case BOOT_STATE::NONE :
        break;
    case BOOT_STATE::START :
        setState(bootStart());
        break;
    case BOOT_STATE::RUN :
        setState(bootRun());
        break;
    case BOOT_STATE::COMPLETE :
        setState(bootComplete());
        ret = true;
        break;            
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

BOOT_STATE CTaskBoot::bootStart(){
    BOOT_STATE ret = BOOT_STATE::START;
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_ALWAYS_WITH_EBOT);
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::ON);
    ret = BOOT_STATE::RUN;
    return ret;
}
BOOT_STATE CTaskBoot::bootRun(){
    BOOT_STATE ret = BOOT_STATE::RUN;
    if(!SOUND_CTR.isSoundPlaying() && !DISPLAY_CTR.isLCDPlaying())
    {
        // const std::string filename = "/home/ebot/RobotOption.yaml";
        // tRobotOption readData;
        // utils::fileIO::loadRobotOption(filename, readData);
        ret = BOOT_STATE::COMPLETE;
    }
    
    return ret;
}
BOOT_STATE CTaskBoot::bootComplete(){
    BOOT_STATE ret = BOOT_STATE::COMPLETE;
    ret = BOOT_STATE::NONE;
    return ret;
}