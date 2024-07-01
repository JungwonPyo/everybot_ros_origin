#include "taskCharging.h"
#include "utils.h"
#include "eblog.h"
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "rosPublisher.h"
#include "kinematics.h"
#include "control/control.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CTaskCharging::CTaskCharging()
{
    CStopWatch __debug_sw;
    
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskCharging::~CTaskCharging()
{
    CStopWatch __debug_sw;

    setChargingState(CHARGING_STATE::NONE);
    setRunChargingState(RUN_CHARGING_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskCharging::setChargingState(CHARGING_STATE set)
{
    
    if (set != state)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[ChargingState Changed] : "<< enumToString(state)<<" --> "<< enumToString(set) );
    }
    state = set;
}

void CTaskCharging::setRunChargingState(RUN_CHARGING_STATE set)
{
    if (set != runChargeState)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[RunChargingState Changed] : "<< enumToString(runChargeState)<<" --> "<< enumToString(set) );
    }
    runChargeState = set;
}

void CTaskCharging::dryMopStartStop()
{
    if(state == CHARGING_STATE::RUN_DRY_MOP || state == CHARGING_STATE::START_DRY_MOP){
        taskDryMop.dryCancle();
    }
    else{
        setChargingState(CHARGING_STATE::START_DRY_MOP);
        taskDryMop.dryStart();
    }
}

void CTaskCharging::dryMopOptionUpdate()
{
    taskDryMop.updateDryOption();
}

void CTaskCharging::taskStart()
{
    ceblog(LOG_LV_NECESSARY, RED, "CHARGING TASK START!! 김정민 연구원 걸레건조 옵션을 여기에 적용주세요!!!");
    if(/* key == E_ACTION_KEY::DRY_MOP_OPTION && */ ServiceData.rsBridge.getDryEnabled() == 2)
    {
        setChargingState(CHARGING_STATE::START_DRY_MOP);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "걸레 건조 자동 실행 옵션 설정되어 시작합니다~!");
    }
    else
    {
        setRunChargingState(RUN_CHARGING_STATE::MODE_CHECK);
        setChargingState(CHARGING_STATE::RUN_CHARGING);
    }
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
bool CTaskCharging::taskRun()
{    
    CStopWatch __debug_sw;
    
    bool ret = false;

    E_BATTERY_STATE battState = ServiceData.battery.getBatteryState();
    u8 percent = ServiceData.battery.getBatteryPercent();
    bool extPower = ServiceData.power.getExtPower();
    double volt = ServiceData.battery.getBatteryVolt();
    double runTime = SYSTEM_TOOL.getSystemTime()-startTime;

    switch (state)
    {
    case CHARGING_STATE::START_DRY_MOP:
        setChargingState(startDryMop());
        break;    
    case CHARGING_STATE::RUN_DRY_MOP:
        setChargingState(runDryMop(battState));
        break;
    case CHARGING_STATE::RUN_CHARGING:
        setChargingState(runCharging(battState,percent,volt,runTime));
        break;
    case CHARGING_STATE::CHARGING_COMPLETE:
        setChargingState(completeCharging(battState,percent,volt,runTime));
        break;    
    
    default:
        break;
    }

    debugPrint(runTime,battState,percent,volt,extPower);

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

CHARGING_STATE CTaskCharging::startDryMop()
{
    CHARGING_STATE ret = CHARGING_STATE::START_DRY_MOP;
    
    taskDryMop.dryStart();
    return CHARGING_STATE::RUN_DRY_MOP;
}

CHARGING_STATE CTaskCharging::runDryMop(E_BATTERY_STATE battState)
{
    CHARGING_STATE ret = CHARGING_STATE::RUN_DRY_MOP;
    
    if(taskDryMop.taskRun())
    {
        if(battState == E_BATTERY_STATE::BATT_FULL)  ret = CHARGING_STATE::CHARGING_COMPLETE;    
        else                                         {setRunChargingState(RUN_CHARGING_STATE::START);ret = CHARGING_STATE::RUN_CHARGING;}
    }
    
    return ret;
}

CHARGING_STATE CTaskCharging::runCharging(E_BATTERY_STATE battState,u8 percent,double volt,double runTime)
{
    CHARGING_STATE ret = CHARGING_STATE::RUN_CHARGING;
    
    switch (runChargeState)
    {
    case RUN_CHARGING_STATE::NONE :       
        break;  
    case RUN_CHARGING_STATE::MODE_CHECK :
        setRunChargingState(modeCheck());       
        break;  
    case RUN_CHARGING_STATE::START :
        setRunChargingState(chargingStart(battState,percent,volt));
        break;    
    case RUN_CHARGING_STATE::RUN :
        setRunChargingState(chargingRun(runTime,battState,percent,volt));
        break;
    case RUN_CHARGING_STATE::COMPLETE :
        setRunChargingState(RUN_CHARGING_STATE::NONE);
        ROBOT_CONTROL.reportAwsStatus(13);
        ret = CHARGING_STATE::CHARGING_COMPLETE;
        break;            
    default:
        break;
    }

    return ret;
}

CHARGING_STATE CTaskCharging::completeCharging(E_BATTERY_STATE battState, u8 percentage, double volt,double runTime)
{
    CHARGING_STATE ret = CHARGING_STATE::CHARGING_COMPLETE;
    
    return ret;
}

RUN_CHARGING_STATE CTaskCharging::modeCheck()
{
    RUN_CHARGING_STATE ret = RUN_CHARGING_STATE::MODE_CHECK;

    //충전 모드 체크 상태일 경우 슬램을 OFF 시키자
    //슬램 커지기 전에 슬램 지도를 저장해야 될지? 시나리오 컨셉이 아직 정해지지 않음
    //int nResult = ROBOT_CONTROL.slam.saveSlamMapFile(false);
    ROBOT_CONTROL.slam.exitSlam();

   if(ServiceData.power.getPowerState() != E_POWER_STATE::CHARGE)
   {
       modeInitTime = SYSTEM_TOOL.getSystemTime();
       ROBOT_CONTROL.systemModeControl(E_POWER_MODE::MODE_CHARGE);
   }
   ret = RUN_CHARGING_STATE::START; 

   return ret;
}

RUN_CHARGING_STATE CTaskCharging::chargingStart(E_BATTERY_STATE battState, u8 percentage, double volt)
{
   RUN_CHARGING_STATE ret = RUN_CHARGING_STATE::START;
    
    if(ServiceData.power.getPowerState() == E_POWER_STATE::CHARGE)
    {
        //DISPLAY_CTR.startDisplay(E_DisplayImageClass::CHARGING);
        DISPLAY_CTR.stopAutoDisplay();
        DISPLAY_CTR.startDisplay(DISPLAY_CTR.getChargingImage());
        LED_CTR.playChargingServiceStart();
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CHARGE_START);
        ROBOT_CONTROL.reportAwsStatus(12);
        startBattState = battState;
        startPercentage = percentage;
        startVolt = volt;
        tempPercentage = percentage;
        startTime = SYSTEM_TOOL.getSystemTime();
        debugStartTime = SYSTEM_TOOL.getSystemTime();
        ceblog((LOG_LV_NECESSARY), BOLDGREEN," 충전 시작 배터리 상태 : " << enumToString(startBattState)<< " Percentage : " << (int)startPercentage << " Volt : " << startVolt);
        ret = RUN_CHARGING_STATE::RUN;
    }
    else if(SYSTEM_TOOL.getSystemTime()-modeInitTime >= 0.1)
    {
        ret = RUN_CHARGING_STATE::MODE_CHECK;
    }

    return ret;
}

RUN_CHARGING_STATE CTaskCharging::chargingRun(double runTime,E_BATTERY_STATE battState, u8 percentage, double volt)
{
    RUN_CHARGING_STATE ret = RUN_CHARGING_STATE::RUN;

    if(battState == E_BATTERY_STATE::BATT_FULL ) //percentage >= CONFIG.fullChargePercentage
    {
       LED_CTR.playChargingServiceEnd();
       SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_ERROR_CLEAR_EFFECT);
       DISPLAY_CTR.startDisplay(DISPLAY_CTR.getChargingImage());
       //DISPLAY_CTR.startDisplay(E_DisplayImageClass::CHARGING_COMPLETE);
       totalChargingTime = runTime;
       ceblog((LOG_LV_NECESSARY), BOLDGREEN," 충전 완료!! 시작 Percent : " << (int)startPercentage << "% 시작 volt : " << startVolt << " 완료 percent : " << (int)percentage 
        << "% 완료 Volt : " << volt << " 배터리 상태 : " << enumToString(battState) << " 충전완료까지 걸린시간 : " << totalChargingTime);
        ret = RUN_CHARGING_STATE::COMPLETE;
    }
    else if(ServiceData.battery.isUpateState())
    {
        DISPLAY_CTR.startDisplay(DISPLAY_CTR.getChargingImage());
    }
    
    return ret;
}

void CTaskCharging::debugPrint(double runTime,E_BATTERY_STATE battState, u8 percentage, double volt, bool extPower)
{
    double debugingTime = SYSTEM_TOOL.getSystemTime()-debugStartTime;
    
    if(!extPower)
    {
        ceblog((LOG_LV_NECESSARY), RED," 충전 중.....단자 접촉 해제됨!!! : " << (int)extPower);
    }

    if(debugingTime >= 1)
    {
        if(state == CHARGING_STATE::CHARGING_COMPLETE)
        {
            ceblog((LOG_LV_NECESSARY), BOLDGREEN," 충전 완료!! 시작 Percent " << (int)startPercentage << "% 시작 volt : " << startVolt << " 완료 Percent : " << (int)percentage 
            << "% 완료 Volt : " << volt << " 시작 배터리 상태 : " << enumToString(startBattState) << " 배터리 상태 : " << enumToString(battState) << " 충전 완료까지 걸린시간 : " << totalChargingTime << " RunTime : " << runTime);
        }
        else if(state == CHARGING_STATE::RUN_CHARGING)
        {
            ceblog((LOG_LV_NECESSARY), BOLDGREEN," 충전 중...충전 상태 : " << enumToString(runChargeState) << "단자 접촉 : " << (int)extPower << "배터리 상태 : " << enumToString(battState) 
            << " Percentage : " << static_cast<int>(percentage) << "% Volt : " << volt << " RunTime : " << runTime); 
        }

        debugStartTime = SYSTEM_TOOL.getSystemTime();
    }
}

bool CTaskCharging::isDryMopActive()
{
    bool ret = false;
    if(state == CHARGING_STATE::START_DRY_MOP || state == CHARGING_STATE::RUN_DRY_MOP) ret = true;
    return ret;
}