#include "taskDryMop.h"
#include "utils.h"
#include "eblog.h"
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "control/control.h"
#include "subTask.h"
#include "rosPublisher.h"
#include "kinematics.h"
/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CTaskDryMop::CTaskDryMop()
{
    CStopWatch __debug_sw;
    
    option.autorun = false;
    setDryMopState(DRYMOP_STATE::NONE);
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskDryMop::~CTaskDryMop()
{
    CStopWatch __debug_sw;

    
    setDryMopState(DRYMOP_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskDryMop::setDryMopState(DRYMOP_STATE set)
{
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[DryMopState Changed] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

DRYMOP_STATE CTaskDryMop::getDryMopState()
{
    return state;
}

void CTaskDryMop::updateDryOption()
{
    tDryMopOption update = option;

    if(ServiceData.rsBridge.getDryEnabled())
    {
        if (ServiceData.rsBridge.getDryEnabled() == 1)       {update.autorun = false;}       
        else if (ServiceData.rsBridge.getDryEnabled() == 2)  {update.autorun = true;}
        ServiceData.awsData.setSendDryEnabled(ServiceData.rsBridge.getDryEnabled());
    }
    if(ServiceData.rsBridge.getDryHours())
    {
        if (ServiceData.rsBridge.getDryHours() == 1)       update.timeout = 3600;
        else if (ServiceData.rsBridge.getDryHours() == 2)  update.timeout = 3600 * 3;
        else if (ServiceData.rsBridge.getDryHours() == 3)  update.timeout = 3600 * 5;

        ServiceData.awsData.setSendDryHours(ServiceData.rsBridge.getDryHours());
    }

    if(ServiceData.rsBridge.getDryPower())
    {
        if (ServiceData.rsBridge.getDryPower() == 0)        update.level = E_DRYFAN_LEVEL::DRY_LEVEL_0;
        else if (ServiceData.rsBridge.getDryPower() == 1)   update.level = E_DRYFAN_LEVEL::DRY_LEVEL_1;
        else if (ServiceData.rsBridge.getDryPower() == 2)   update.level = E_DRYFAN_LEVEL::DRY_LEVEL_2;
        else if (ServiceData.rsBridge.getDryPower() == 3)   update.level = E_DRYFAN_LEVEL::DRY_LEVEL_3;
        ServiceData.awsData.setSendDryPower(ServiceData.rsBridge.getDryPower());
    }

    eblog(LOG_LV_NECESSARY, "Chagne Dry-Mop option AutoRun : " << (int)update.autorun <<" TimeOut : "<< update.timeout << " Level : " << enumToString(update.level));
    // if(update.autorun != option.autorun || update.timeout != option.timeout || update.level != option.level )
    // {
    //     // if(update.level != option.level) setDryMopState(DRYMOP_STATE::CHANGE_FAN_LEVEL);
    //     // if(update.timeout != option.timeout)
    //     // {
    //     //     if(update.timeout < option.timeout) setDryMopState(DRYMOP_STATE::START);
    //     // }
    //     // if(update.autorun != option.autorun)
    //     // {
    //     //     if(!update.autorun) setDryMopState(DRYMOP_STATE::CANCLE);
    //     // }
    //     setOption(update);
    // }
}

tDryMopOption CTaskDryMop::getDryOption()
{
    tDryMopOption ret;

    short reportDryEnable = ServiceData.awsData.getSendDryEnabled();
    short reportDryHours = ServiceData.awsData.getSendDryHours();
    short reportDryPower = ServiceData.awsData.getSendDryPower();

    if (reportDryEnable == 2)  {ret.autorun = true;}
    else if (reportDryEnable == 1)  {ret.autorun = false;}
    else{
        reportDryEnable = 1;
        ret.autorun = false;
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 자동 시작 설정이 없습니다. 기본 값으로 시작합니다.");
    }       

    if (reportDryHours == 2)        {ret.timeout = 3600 * 3;}         
    else if (reportDryHours == 3)   {ret.timeout = 3600 * 5;}    
    else if (reportDryHours == 1)   {ret.timeout = 3600;}    
    else{
        reportDryHours = 1;
        ret.timeout = 3600;
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 시간 설정이 없습니다. 기본 값으로 시작합니다.");
    }

    if (reportDryPower == 2)         {ret.level = E_DRYFAN_LEVEL::DRY_LEVEL_2;}
    else if (reportDryPower == 3)    {ret.level = E_DRYFAN_LEVEL::DRY_LEVEL_3;}
    else if (reportDryPower == 1)    {ret.level = E_DRYFAN_LEVEL::DRY_LEVEL_1;}
    else{
        reportDryPower = 1;
        ret.level = E_DRYFAN_LEVEL::DRY_LEVEL_1;
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 POWER 설정이 없습니다. 기본 값으로 시작합니다. ");
    }

    ServiceData.awsData.setSendDryEnabled(reportDryEnable);
    ServiceData.awsData.setSendDryHours(reportDryHours);
    ServiceData.awsData.setSendDryPower(reportDryPower);

    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 시작~! 자동실행 : " << (int)ret.autorun << " 시간 :  " << ret.timeout << " RPM LEVEL : " << enumToString(ret.level) );

    return ret;  

}

void CTaskDryMop::dryStart()
{
    setOption(getDryOption());
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 시작~! 자동실행 : " << (int)option.autorun << " 시간 :  " << option.timeout << " RPM LEVEL : " << enumToString(option.level) );
    ROBOT_CONTROL.reportAwsStatus(15);
    setDryMopState(DRYMOP_STATE::START);
}
void CTaskDryMop::dryCancle()
{
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 정지~!");
    setDryMopState(DRYMOP_STATE::CANCLE);
}

void CTaskDryMop::setOption(tDryMopOption set)
{
    if(option.autorun != set.autorun)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "자동 실행 옵션 변경!! 기존 설정 : " << (int)option.autorun << " 변경 설정 :  " << (int )set.autorun);
    }
    // if(option.iteration != set.iteration)
    // {
    //     ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "반복 횟수 옵션 변경!! 기존 설정 : " << option.iteration << " 변경 설정 :  " << (int )set.iteration);
    // }
    if(option.level != set.level)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "팬 RPM 단계 변경!! 기존 설정 : " << enumToString(option.level) << " 변경 설정 :  " << enumToString(set.level));
        setDryMopState(DRYMOP_STATE::CHANGE_FAN_LEVEL);
    }
    if(option.level != set.level)
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 시간 설정 변경!! 기존 설정 : " << option.timeout << " 변경 설정 :  " << set.timeout);
    }

    option = set;
}
/**
 * @brief explorer proc
 * jhnoh, 23.01.16
 * @param robotPose     로봇 좌표 
 * @return tExplorerInfo 
 * 
 * @note  연산시간: 2ms ~ 11.0ms 
 * @date  2023-08-28
 */
bool CTaskDryMop::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;
    
    //updateDryOption();

    switch (state)
    {
    case DRYMOP_STATE::NONE :
        break;
    case DRYMOP_STATE::START :
        setDryMopState(startDryMop());
        break;
     case DRYMOP_STATE::CHECK_FAN_ACK :
        setDryMopState(checkFanAck());
        break;        
    case DRYMOP_STATE::RUN :
        setDryMopState(runDryMop());
        break;
    case DRYMOP_STATE::COMPLETE_FAN_OFF :
        setDryMopState(completeFanOff());
        break;      
    case DRYMOP_STATE::COMPLETE :
        setDryMopState(completeDryMop());
        ret = true;
        break;
    case DRYMOP_STATE::CANCLE :
        setDryMopState(stopDryMop());
        break;
    case DRYMOP_STATE::CHANGE_FAN_LEVEL :
        setDryMopState(changeFanLevel());
    break;                      
    default:
        break;
    }


    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


DRYMOP_STATE CTaskDryMop::startDryMop()
{
   DRYMOP_STATE ret = DRYMOP_STATE::START;
   cmd = FAN_COMMAND::ON;
   ROBOT_CONTROL.dryFanOn(option.level);
   cmdCnt = 1;
   cmdStartTime = SYSTEM_TOOL.getSystemTime();
   startTime = SYSTEM_TOOL.getSystemTime();
   debugStartTime = SYSTEM_TOOL.getSystemTime();
   DISPLAY_CTR.startDisplay(E_DisplayImageClass::START_DRY_RAG);
   SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MOP_DRY_START);
   ceblog((LOG_LV_NECESSARY), BOLDGREEN," 걸레건조 시작");
   ret = DRYMOP_STATE::CHECK_FAN_ACK;
   return ret;
}

DRYMOP_STATE CTaskDryMop::checkFanAck()
{
    DRYMOP_STATE ret = DRYMOP_STATE::CHECK_FAN_ACK;

    double term = SYSTEM_TOOL.getSystemTime()-cmdStartTime;
    if(ServiceData.signal.getFanAck())
    {
        if(!DISPLAY_CTR.isLCDPlaying() && !SOUND_CTR.isSoundPlaying()){
            if(cmd == FAN_COMMAND::ON){
            DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::DRYING);
            ret = DRYMOP_STATE::RUN;
            ceblog((LOG_LV_NECESSARY), BOLDGREEN,"FAN ON~~!! Ack 확인 FAN_COMMAND : " << enumToString(cmd) << " 팬 제어 횟수 : " << cmdCnt);
            }  
            else{
                ret = DRYMOP_STATE::COMPLETE;
                ceblog((LOG_LV_NECESSARY), BOLDGREEN,"FAN OFF~~!! Ack 확인 FAN_COMMAND : " << enumToString(cmd) << " 팬 제어 횟수 : " << cmdCnt);
            }
            cmdCnt = 0;
        }                    
    }
    else if(term > 1)
    {   
        cmdCnt++;

        if(cmd == FAN_COMMAND::ON)  ROBOT_CONTROL.dryFanOn(option.level);
        else                        ROBOT_CONTROL.dryFanOff();

        ceblog((LOG_LV_NECESSARY), BOLDGREEN," 팬 Ack 1초 동안 오지 않아서 재시도 FAN_COMMAND : " << enumToString(cmd) << " 팬 제어 횟수 : " << (int)cmdCnt << "재시도 Term : " << term);
        ceblog((LOG_LV_NECESSARY), BOLDGREEN,"걸레건조 재시도 Term이 1초인 이유는 F/W에서 3회 시도하는 시간이 필요하기 때문에!!");
        cmdStartTime = SYSTEM_TOOL.getSystemTime();
    }
    
    return ret;
}

DRYMOP_STATE CTaskDryMop::runDryMop()
{
    DRYMOP_STATE ret = DRYMOP_STATE::RUN;

    double runTime = SYSTEM_TOOL.getSystemTime()-startTime;
    double debugingTime = SYSTEM_TOOL.getSystemTime()-debugStartTime;

    if(runTime >= (double)option.timeout)
    {
        DISPLAY_CTR.stopAutoDisplay();
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::DRYING_COMPLETED);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MOP_DRY_COMPLETE);
        ceblog((LOG_LV_NECESSARY), BOLDGREEN," 걸레건조 완료...runTime : " << runTime << " TimeOut : " << (double)option.timeout << " autoRun : "  << option.autorun << " FanLevel : " << enumToString(option.level));//<< "iter : " << iteration << " option-iter : " << option.iteration
        ret = DRYMOP_STATE::COMPLETE_FAN_OFF;//if(++iteration >= option.iteration)
    }

    if(debugingTime >= 1)
    {
        ceblog((LOG_LV_NECESSARY), BOLDGREEN," 걸레건조 중...runTime : " << runTime << " TimeOut : " << option.timeout << " autoRun : "  << option.autorun << " FanLevel : " << enumToString(option.level));//<< "iter : " << iteration << " option-iter : " << option.iteration
        debugStartTime = SYSTEM_TOOL.getSystemTime();
    }
    
    return ret;
}
DRYMOP_STATE CTaskDryMop::completeFanOff()
{
    DRYMOP_STATE ret = DRYMOP_STATE::COMPLETE_FAN_OFF;
    cmd = FAN_COMMAND::COMPLETE_OFF;
    ROBOT_CONTROL.dryFanOff();
    cmdCnt = 1;
    cmdStartTime = SYSTEM_TOOL.getSystemTime();
    ceblog((LOG_LV_NECESSARY), BOLDGREEN," 걸레건조가 완료되어 팬 OFF!!");
    ret = DRYMOP_STATE::CHECK_FAN_ACK;
    return ret;
}
DRYMOP_STATE CTaskDryMop::changeFanLevel()
{
    DRYMOP_STATE ret = DRYMOP_STATE::CHANGE_FAN_LEVEL;

    cmd = FAN_COMMAND::ON;
    ROBOT_CONTROL.dryFanOn(option.level);
    cmdCnt = 1;
    cmdStartTime = SYSTEM_TOOL.getSystemTime();
    ceblog((LOG_LV_NECESSARY), BOLDGREEN," 건조 팬 RPM 단계 변경 시작 : " << enumToString(option.level));
    ret = DRYMOP_STATE::CHECK_FAN_ACK;
    
    return ret;
}

DRYMOP_STATE CTaskDryMop::completeDryMop()
{
    DRYMOP_STATE ret = DRYMOP_STATE::COMPLETE;

    bRunning = false;
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MOP_DRY_END);
    DISPLAY_CTR.stopAutoDisplay();
    DISPLAY_CTR.startDisplay(DISPLAY_CTR.getChargingImage()); 
    //여기서 report
    ret = DRYMOP_STATE::NONE;
    return ret;
}

DRYMOP_STATE CTaskDryMop::stopDryMop()
{
    DRYMOP_STATE ret = DRYMOP_STATE::CANCLE;
    cmd = FAN_COMMAND::CANCLE_OFF;
    ROBOT_CONTROL.dryFanOff();
    DISPLAY_CTR.stopAutoDisplay();
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CANCLED);
    cmdCnt = 1;
    cmdStartTime = SYSTEM_TOOL.getSystemTime();
    ceblog((LOG_LV_NECESSARY), BOLDGREEN," 걸레건조 취소!!");
    ret = DRYMOP_STATE::CHECK_FAN_ACK;
    return ret;
}
