#include "supplyWater.h"
#include "eblog.h"
#include "userInterface.h"
#include "systemTool.h"
#include "MessageHandler.h"
#include "commonStruct.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CSupplyWater::CSupplyWater()
{
    CStopWatch __debug_sw;

    initWaterSupplyData();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CSupplyWater::~CSupplyWater()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

//////////////////////// start - water supply function //////////////////////////////////////

/**
 * @brief 물 공급 단계 별 물공급 주기를 설정하는 함수
 * 물공급 주기란 물공급을 몇 초에 한번씩 모터를 동작 할것인가를 의미함
 * 
 * @param E_WATER_PUMP_STEP step 유저가 설정한 물공급 단계를 의미한다
 * @param E_WATER_PUMP_STEP::WATER_DRY : 물공급 차단
 * @param E_WATER_PUMP_STEP::WATER_LEVEL_1 : 물 공급 적게
 * @param E_WATER_PUMP_STEP::WATER_LEVEL_2 : 물 공급 많이
 * @return 물공급 cycle의 주기(시간)  
 */
double CSupplyWater::getWaterSupplyPeriod(u32 cleanTime)
{
    CStopWatch __debug_sw;

    double ret;
    double level_1_period = getLevel1Period(cleanTime); // 415
    
    switch (watersupply_.level)
    {
        case E_WATER_PUMP_STEP::WATER_DRY :/* 마른걸레 청소 */
        /* code */
        break;
         case E_WATER_PUMP_STEP::WATER_LEVEL_1 :/* 물공급 적게 */
        ret = level_1_period; //WATER_SUPPLY_LEVEL1_PERIOD
        break;
         case E_WATER_PUMP_STEP::WATER_LEVEL_2 :/* 물공급 보통 */
        ret = level_1_period * 0.85; //WATER_SUPPLY_LEVEL2_PERIOD
        break;
         case E_WATER_PUMP_STEP::WATER_DISPOSABLE :/* 물공급 많음 */
        ret = level_1_period * 0.60; //WATER_SUPPLY_DISPOSABLE_PERIOD
        break;
    
        default:
        /* 물공급 설정 없음 or 에러 */
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 예상청소소요시간 (cleanTime) 단위 : 분 [min]
 * 
 * @param cleanTime 
 * @return double 
 */
double CSupplyWater::getLevel1Period(u32 cleanTime)
{
    /*
        예상청소소요시간 : cleanTime [min]
        물통용량 : 260 [mL]
        물빠지는속도 : 3 [mL/sec] 
        분당 205ml
    */
    return cleanTime*60 / (260/3 - 60);
}

/**
 * @brief 설정된 물공급 주기가 업데이트 되었는지 확인하는 함수
 * 물공급 주기란 물공급을 몇 초에 한번씩 모터를 동작 할것인가를 의미함
 * 
 * @param remain_periodtime : 설정 된 주기까지 남은 시간(msec)
 * @param watersupply_.supply_period : 현재 주기를 저장하는 함수 - 인자로 받는 period는 다음 주기에 적용할 period
 * @param watersupply_.period_count : 주기 cycle 횟수
 * @param Ewatersupply_.period_starttick : 현재 주기의 시작시간
 * @param watersupply_.supply_interval : 물공급 펌프 모터를 몇초 동안 동작할 것인지를 의미하는 변수
 * @return 물공급 cycle의 주기(시간)  
 */
bool CSupplyWater::updateWaterSupplyPeriod(double period)
{
    CStopWatch __debug_sw;

    bool ret = false;

    u32 remain_periodtime = (int64_t)(get_system_time()*10000) / 100 -  watersupply_.period_starttick;//SYSTEM_TOOL.getSystemTick() - watersupply_.period_starttick;
	
    //eblog(LOG_LV_NECESSARY, "CSupplyWater period(second): " << period);
    //eblog(LOG_LV_NECESSARY, "CSupplyWater remain_periodtime: " << remain_periodtime);

    //1. 현재 주기가 완료된 시점 새로운 주기를 시작함 - 물공급 단계 변경 시 새로운 주기는 인자로 받는 period 
	if( remain_periodtime >= watersupply_.supply_period ) //현재 주기가 끝나는 시점 확인
	{
		watersupply_.supply_period = period;//현재 주기에 새로운 주기를 적용 ;  230 * 100.00000000 ms
		watersupply_.period_count++; 		//현재 주기의 횟수차를 count
		watersupply_.period_starttick = (int64_t)(get_system_time()*10000) / 100;//SYSTEM_TOOL.getSystemTick(); //새로운 주기의 시작시점 설정
		
        eblog(LOG_LV_NECESSARY, "updateWaterSupplyPeriod : " << watersupply_.supply_period); // 230 000 ms
		if(!watersupply_.timeout)
		{
			ret = true;
		}
	}

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 물 공급 기능 시작 함수 - 청소 시작 5초 후 부터 물공급을 시작한다.
 *  청소 시작 정지를 반복적으로 실행하면 제자리에서 계속 물공급을 할 수 있기 때문에 일정 시간 후 부터 물 공급을 시작 한다.
 * @param cleantime : 청소 시간
 * @param watersupply_.isPumpActive : 펌프모터가 동작 중이면 true / 동작 하지 않으면 false
 * @param watersupply_.pump_activetick : 펌프모터 구동 시작 시간
 * @param E_WATERSUPPLY_STATE : 물공급 기능의 상태 
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_START : 물공급 기능 시작
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING : 물공급 기능 동작 중
 * @return 물공급 기능의 상태  
 */
E_WATERSUPPLY_STATE CSupplyWater::startWaterSupply(double startTime)
{
    CStopWatch __debug_sw;

    E_WATERSUPPLY_STATE ret = E_WATERSUPPLY_STATE::WATERSUPPLY_START;
    u32 cleantime = SYSTEM_TOOL.getSystemTime()-startTime; 

    //청소 시작 5초 후 부터 물공급 시작 - 청소 시작 정지를 연속으로 실행하면 제자리에서 계속 물공급을 할 수 있기 때문에 일정 시간 후 부터 물 공급을 시작 한다.
    #if WATER_SUPPLY_TEST > 0
    if(waterSupplytest)
    #else
    if(cleantime >= 5)
    #endif
    {
        //pump motor on
        watersupply_.isPumpActive = true;
        watersupply_.period_starttick = (int64_t)(get_system_time()*10000) / 100;//SYSTEM_TOOL.getSystemTick();
        watersupply_.pump_activetick = SYSTEM_TOOL.getSystemTick();//(int64_t)(get_system_time()*10000) / 100;//SYSTEM_TOOL.getSystemTick();
        ROBOT_CONTROL.WaterPump(true);
        eblog(LOG_LV_NECESSARY, "startWaterSupply ");
        ret = E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 물 공급 기능 일시정지/재시작 기능을 담당하는 함수
 *  물공급 설정이 되거나 좁은공간 탈출하면 물공급을 다시 시작한다.
 * @param E_WATERSUPPLY_STATE : 물공급 기능의 상태 
 * @return 물공급 기능의 상태  
 */
E_WATERSUPPLY_STATE CSupplyWater::pauseWaterSupply(void)
{
    CStopWatch __debug_sw;

    E_WATERSUPPLY_STATE ret = WATERSUPPLY_PAUSE;

    // CTX_USERSET *user = pContext->GetCtxUserset();

    // if(user->WaterStep != WATER_DRY) //&& !watersupply_.water_block 물공급 설정이 변경되거나 좁은공간 탈출하면 물공급을 다시 시작한다.
    // {
    //     eblog(LOG_LV_NECESSARY, "pauseWaterSupply -> runWaterSupply ");
    //     ret = E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING;
    // }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 물 공급 기능 run 함수 - 주기 업데이트 및 펌프 모터 제어를 관리한다.
 *  청소 시작 정지를 반복적으로 실행하면 제자리에서 계속 물공급을 할 수 있기 때문에 일정 시간 후 부터 물 공급을 시작 한다.
 * @param cleantime : 청소 시간
 * @param watersupply_.isPumpActive : 펌프모터가 동작 중이면 true / 동작 하지 않으면 false
 * @param watersupply_.pump_activetick : 펌프모터 구동 시작 시간
 * @param watersupply_.supplytime : 물공급 펌프가 동작하는 시간. 즉, 물공급을 하고있는 시간 
 * @return 물공급 기능의 상태  
 */
E_WATERSUPPLY_STATE CSupplyWater::runWaterSupply(void)
{
    CStopWatch __debug_sw;

    E_WATERSUPPLY_STATE ret = E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING;

    //청소 시간 신회성 스팩 : 120 hour + @(20분 정도 더)
    //u32 cleanTime = 60; // 예상 청소시간 100분 임시로 박음 (2024.05.28 by hyjoe)
    double preiod_temp = getWaterSupplyPeriod(CONFIG.waterSupplyCleanTime); //유저가 설정한 물공급 단계에 맞는 주기를 설정한다.
    //기존 1단계 1분 WATER_SUPPLY_LEVEL1_PERIOD 60*SEC_1 -> 6000.00000000
    //(120*60)/26 = 276
    //(110*60)/26 = 253
    //(100*60)/26 = 230
    //(90*60)/26 = 207
    //(80*60)/26 = 184
    //(60*60)/26 = 138
    //(50*60)/26 = 115 : 50분 -> 11500.00000000
    //(40*60)/26 = 92
    //(30*60)/26 = 69
    double preiod = preiod_temp*SEC_1;
    //double preiod = WATER_SUPPLY_LEVEL1_PERIOD;//WATER_SUPPLY_LEVEL1_PERIOD; //기존 60초 주기 -> 1시간 30분(습식)
    bool isUpdatePeriod = updateWaterSupplyPeriod(preiod); //현재 주기가 완료되면, 다음 적용될 주기설정을 업데이트한다.
 
    if(isUpdatePeriod)//새로운 주기가 업데이트 되면 물공급 펌프를 동작 시킨다
    {
        //pumpMotor on
        watersupply_.isPumpActive = true;
        ROBOT_CONTROL.WaterPump(true);
        watersupply_.pump_activetick = SYSTEM_TOOL.getSystemTick();
        eblog(LOG_LV_NECESSARY, "runWaterSupply updatePeriod Pump On ");
        //ceblog(LOG_LV_NECESSARY, BOLDRED, "runWaterSupply updatePeriod Pump On");
    }
    else if( watersupply_.isPumpActive)
    {
        watersupply_.supplytime = SYSTEM_TOOL.getSystemTick()-watersupply_.pump_activetick;

        //eblog(LOG_LV_NECESSARY, "runWaterSupplyTime: "<<watersupply_.supplytime << "interval : "<< watersupply_.supply_interval);
        if(watersupply_.supplytime >= watersupply_.supply_interval) //펌프 동작은 정해진 시간만큼만 한다(interval)
        {
            //pumpMotor->off
            watersupply_.isPumpActive = false;
            ROBOT_CONTROL.WaterPump(false);
            //ceblog(LOG_LV_NECESSARY, BOLDRED, "runWaterSupply interval end  pump Off");
            eblog(LOG_LV_NECESSARY, "runWaterSupply interval end  pump Off ");
            //ret = E_WATERSUPPLY_STATE::WATERSUPPLY_END;
        }
    }

    // if(watersupply_.timeout)//물공급 최대 설정시간이 초과하면 물공급 기능을 종료한다.
    // {
    //     eblog(LOG_LV_NECESSARY, "runWaterSupply -> endWaterSupply ");
    //     ret = E_WATERSUPPLY_STATE::WATERSUPPLY_END;
    // }
    // else if( user->WaterStep == WATER_DRY) //|| watersupply_.water_block //물공급 설정을 마른걸레로 변경하거나, 좁은 공간에서 있다고 판단되면 물공급을 일시 정지한다.
    // {
    //      eblog(LOG_LV_NECESSARY, "runWaterSupply -> pauseWaterSupply ");
    //     ret = E_WATERSUPPLY_STATE::WATERSUPPLY_PAUSE;//pauseWaterSupply();//pumpMotor->off
    // }
    // else
    // {
    //     if(isUpdatePeriod)//새로운 주기가 업데이트 되면 물공급 펌프를 동작 시킨다
    //     {
    //         //pumpMotor on
    //         watersupply_.isPumpActive = true;
    //         eblog(LOG_LV_NECESSARY, "runWaterSupply updatePeriod Pump On ");
    //     }
    //     else if( watersupply_.isPumpActive)
    //     {
    //         watersupply_.supplytime = SYSTEM_TOOL.getSystemTick()-watersupply_.pump_activetick;

    //         if(watersupply_.supplytime >= watersupply_.supply_interval) //펌프 동작은 정해진 시간만큼만 한다(interval)
    //         {
    //             //pumpMotor->off
    //             watersupply_.isPumpActive = false;
    //             eblog(LOG_LV_NECESSARY, "runWaterSupply interval end  pump Off ");
    //         }
    //     }
    // }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 물 공급 기능 종료 함수 - 청소 시작 5초 후 부터 물공급을 시작한다.
 * 청소 종료 또는 물공급 설정 시간 끝나는 경우 또는 물이 없는 경우, 단 물이 없는지를 판단하는 알고리즘 및 데이터가 없기때문에 사용할수 없음
 * 물이 없는데 계속해서 펌프를 동작시키는 것은 불필요한 행동이기 때문에, 전력을 효율적으로 관리하려면 물 없음 감지 알고리즘이 필요하다.
 * 펌프를 정지 시키고 물공급 관련 데이터를 초기화 시킨다.
 * @return 물공급 기능의 상태  
 */
E_WATERSUPPLY_STATE CSupplyWater::endWaterSupply(void)
{
    CStopWatch __debug_sw;

    E_WATERSUPPLY_STATE ret = WATERSUPPLY_END;
    if(watersupply_.isPumpActive) ROBOT_CONTROL.WaterPump(false);
    initWaterSupplyData();

    TIME_CHECK_END(__debug_sw.getTime());
    return WATERSUPPLY_VOID;
}

/**
 * @brief 물 공급 기능 모든 시퀀스를 담당하는 함수, 물공급 기능의 상태 변경에 따른 함수를 실행 시켜준다.
 * @param E_WATERSUPPLY_STATE : 물공급 기능의 상태 
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_START : 물공급 기능 시작
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING : 물공급 기능 동작 중
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_PAUSE   : 물공급 기능 일시 정지
 * @param E_WATERSUPPLY_STATE::WATERSUPPLY_END     : 물공급 기능 종료
 * @return 물공급 기능의 상태  
 */
void CSupplyWater::WaterSupplyStateMachine(double startTime)
{
    CStopWatch __debug_sw;

    E_WATERSUPPLY_STATE ret = WATERSUPPLY_VOID;

    switch ( watersupply_.state)
    {
        case E_WATERSUPPLY_STATE::WATERSUPPLY_VOID:
        ret = startWaterSupply(startTime);
        /* code */
        break;
        case E_WATERSUPPLY_STATE::WATERSUPPLY_START:
        ret = startWaterSupply(startTime);
        break;
        case E_WATERSUPPLY_STATE::WATERSUPPLY_RUNNING:
         ret = runWaterSupply();
        break;
        case E_WATERSUPPLY_STATE::WATERSUPPLY_PAUSE:
         ret = pauseWaterSupply();
        break;
        case E_WATERSUPPLY_STATE::WATERSUPPLY_END:
         ret = endWaterSupply();
        break;
        default:
        break;
    }

    if(ret != watersupply_.state)
    {
        eblog(LOG_LV_NECESSARY, "WaterSupplyStateMachine state Change cur : " << (int)watersupply_.state << " -> next : " << (int)ret);
    }
    watersupply_.state = ret;

    TIME_CHECK_END(__debug_sw.getTime());
}

void CSupplyWater::initWaterSupplyData()
{
    CStopWatch __debug_sw;

    watersupply_.level = E_WATER_PUMP_STEP::WATER_LEVEL_1; //level 1로 시작

    watersupply_.state =E_WATERSUPPLY_STATE::WATERSUPPLY_VOID;
	watersupply_.water_block = false; //danielhong:210114 : 추후 필요에 의해 물공급을 중단할 경우
	watersupply_.timeout = false; //danielhong:210222 : NORMAL_WATERSUPPLY_TIME이 지나면 사용자의 리모콘 개입 전까지 물공급 차단, 최대물공급 시간 초과(145분) 시 TRUE
	watersupply_.is_draining = false; //danielhong:210406 : 잔수 제거 기능 추가
    watersupply_.isPumpActive = false;
	watersupply_.period_update = false;	
	watersupply_.period_count = 0; // 물공급 주기 카운트, HD 201014
	watersupply_.supply_count = 0; //hjkim220112 - 물공급 전체 주기에서 물공급을 실행한 횟수
	watersupply_.water_blockcount = 0; //hjkim220112 - 물공급 전체 주기에서 물공급을 차단한 횟수

	watersupply_.pump_activetick = (u32)-1;//펌프모터 timing 관리 시작시간
	watersupply_.period_starttick = (u32)-1;
	watersupply_.WaterBlockstarttick = (u32)-1;//펌프모터 timing 관리 시작시간 
	watersupply_.supply_interval = CONFIG.waterSupplyInterval * SEC_1; //WATER_SUPPLY_PUMP_INTERVAL //1*SEC_1; //물 공급 간격(ontime per cycle)
	watersupply_.supply_period = 1000;
	watersupply_.supplytime = 0;      //서비스 중 총 물 공급 시간 합계

    watersupply_.drain_active = false;
    watersupply_.drain_starttick = (u32)-1;

    #if WATER_SUPPLY_TEST > 0
    waterSupplytest = false;
    #endif
    
    TIME_CHECK_END(__debug_sw.getTime());
}


void CSupplyWater::waterDrainONOFF()
{
    CStopWatch __debug_sw;

    if(watersupply_.drain_active)
    {
        ROBOT_CONTROL.WaterPump(false);
        watersupply_.drain_active = false;
        watersupply_.drain_starttick = (u32)-1;
        eblog(LOG_LV, "CANCLE DRAIN_WATER");
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CANCLED);
        DISPLAY_CTR.startAutoDisplay(true,DISPLAY_CTR.getBlinkEyeImage());
        ROBOT_CONTROL.reportAwsStatus(1);
    }
    else
    {
        ROBOT_CONTROL.WaterPump(true);
        watersupply_.drain_active = true;
        watersupply_.drain_starttick = SYSTEM_TOOL.getSystemTick();
        eblog(LOG_LV, "START DRAIN_WATER");
        
        DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::WATER_REMOVAL);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DRAIN_WATER_START);
        ROBOT_CONTROL.reportAwsStatus(16);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CSupplyWater::checkWaterDrainTimeOut()
{
    CStopWatch __debug_sw;

    if(SYSTEM_TOOL.getSystemTick()-watersupply_.drain_starttick >= WATER_DRAIN_TIMEOUT)
    {
        watersupply_.drain_active = false;
        watersupply_.drain_starttick = (u32)-1;
        ROBOT_CONTROL.WaterPump(false);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::WATER_REMOVAL_COMPLETED);
        DISPLAY_CTR.startAutoDisplay(false,DISPLAY_CTR.getBlinkEyeImage());
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE);
        ROBOT_CONTROL.reportAwsStatus(1);
        eblog(LOG_LV, "FINISH DRAIN_WATER");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CSupplyWater::isActiveWaterDrain()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return watersupply_.drain_active;
}

void CSupplyWater::startWaterSupply()
{
    CStopWatch __debug_sw;

    #if WATER_SUPPLY_TEST > 0
    waterSupplytest = true;
    #endif
    watersupply_.state =E_WATERSUPPLY_STATE::WATERSUPPLY_START;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CSupplyWater::stopWaterSupply()
{
    CStopWatch __debug_sw;

    if(watersupply_.isPumpActive) ROBOT_CONTROL.WaterPump(false);
    initWaterSupplyData();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

E_WATER_PUMP_STEP CSupplyWater::ChangeDirectLevel(short waterLv)
{
    return convertWaterLevel(waterLv);
}
E_WATER_PUMP_STEP CSupplyWater::ChangeStepLevel()
{  
    E_WATER_PUMP_STEP ret;
    switch (watersupply_.level)
    {
    case E_WATER_PUMP_STEP::WATER_DRY:
        ret = E_WATER_PUMP_STEP::WATER_LEVEL_1;
        break;
    case E_WATER_PUMP_STEP::WATER_LEVEL_1:
        ret = E_WATER_PUMP_STEP::WATER_LEVEL_2;
        break;
    case E_WATER_PUMP_STEP::WATER_LEVEL_2:
        ret = E_WATER_PUMP_STEP::WATER_DISPOSABLE;
        break;  
    case E_WATER_PUMP_STEP::WATER_DISPOSABLE:
        ret = E_WATER_PUMP_STEP::WATER_DRY;
        break;      
    default:
        ret = E_WATER_PUMP_STEP::WATER_LEVEL_1;
        break;
    }

    return ret;
}
        
void CSupplyWater::ChangeWaterLevel(short waterLv)
{
    CStopWatch __debug_sw;
    E_WATER_PUMP_STEP before_level = watersupply_.level;
    /* AWS에서 입력 받은 값 반영 */
    if(waterLv != 0)  watersupply_.level = ChangeDirectLevel(waterLv);  
    else              watersupply_.level = ChangeStepLevel();

    switch (watersupply_.level)
    {
    case E_WATER_PUMP_STEP::WATER_DRY:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::DRY_RAG);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WATER_SUPPLY_DRY);
        break;
    case E_WATER_PUMP_STEP::WATER_DISPOSABLE:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::DISPOSABLE);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DISPOSABLE);
        break;
    case E_WATER_PUMP_STEP::WATER_LEVEL_1:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::WATER_SUPPLY1);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WATER_SUPPLY_LEVEL_1);
        break;
    case E_WATER_PUMP_STEP::WATER_LEVEL_2:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::WATER_SUPPLY2);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WATER_SUPPLY_LEVEL_2);
        break;        
    default:
        eblog(LOG_LV_NECESSARY, "Chagne Water Level Error!! WaterLevel : " << (int)watersupply_.level);
        break;
    }

    eblog(LOG_LV_NECESSARY, "Chagne Water Level before :" << before_level <<" new : "<< watersupply_.level);
    ServiceData.awsData.setSendWaterLv(watersupply_.level);
    TIME_CHECK_END(__debug_sw.getTime());
}
/**
 * @brief AWS에서 받은 물공급 설정값을 반영해주는 함수
 * 특이사항 : ChangeWaterLevel(short waterLv)가 현재에서 다음 단계로 넘어가는 것이기 때문에 
 * AWS 설정 값에 한 단계 낮춰서 설정해줘야한다. 
 * 
 * @param waterLv 
 * 1: none
 * 2: 일회용 
 * 3: lv1
 * 4: lv2
 */
E_WATER_PUMP_STEP CSupplyWater::convertWaterLevel(short waterLv)
{
    E_WATER_PUMP_STEP ret;

    switch (waterLv)
    {
    case 1:
        ret = E_WATER_PUMP_STEP::WATER_DRY;
        break;
    case 2:
        ret = E_WATER_PUMP_STEP::WATER_DISPOSABLE;
        break;
    case 3:
        ret = E_WATER_PUMP_STEP::WATER_LEVEL_1;
        break;
    case 4:
        ret = E_WATER_PUMP_STEP::WATER_LEVEL_2;
        break;
    
    default:
        break;
    }

    return ret;
} 
E_WATER_PUMP_STEP CSupplyWater::getWaterLevel()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return watersupply_.level;
}

#if WATER_SUPPLY_TEST > 0
bool CSupplyWater::isPumpIntervalEnd()
{
    bool ret = false;

    if(SYSTEM_TOOL.getSystemTick()-watersupply_.pump_activetick >= WATER_SUPPLY_PUMP_INTERVAL) ret = true;

    return ret;
}
#endif