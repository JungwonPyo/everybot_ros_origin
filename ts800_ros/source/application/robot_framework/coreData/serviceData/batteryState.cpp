#include "coreData/serviceData/batteryState.h"
#include "externData/externData.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/



CBatteryState::CBatteryState() {
    
    initBattery();
}

CBatteryState::~CBatteryState() {}

void CBatteryState::update(CExternData* pExternData)
{
    if ( pExternData->systemData.isUpdateBatteryData() == true )
    {
        updateBatteryState( pExternData->systemData.useBatteryData() );		
    }
}

void CBatteryState::initBattery()
{
    filteredVolt = 0.0;
    stateChanged = false;
    bUpdateState = false;
    setTempState(E_BATTERY_STATE::BATT_VOID);
    setBatteryState(E_BATTERY_STATE::BATT_VOID);
    ceblog(LOG_LV_NECESSARY, CYN, "initBattery  BattState : "<< enumToString(state) << " tempState : " << enumToString(tempState));
}

E_BATTERY_STATE CBatteryState::getBatteryState()
{
    return state;
}

u8 CBatteryState::getBatteryPercent()
{
    return percent;
}

double CBatteryState::getBatteryVolt()
{
    return volt;
}

bool CBatteryState::isBatteryNeedCharge()
{
    bool bRet = false;
    if ( state == E_BATTERY_STATE::BATT_NEED_CHARGE)
    {
        //ceblog(LOG_LV_NECESSARY, CYN, "isBatteryNeedCharge!!  BattState : "<< enumToString(state) << " percent[ " << percent << " ] volt[ " << volt << " ] filteredVolt[ " << filteredVolt << " ]");
        bRet = true;
    }
        
    return bRet;
}

/**
 * @brief 배터리 전류 get
 * 
 * @return double 
 * 
 * @note 연산시간 ms
 * @date 2023-09-12
 * @author hhryu
 */
double CBatteryState::getBatteryCurrent()
{
    return current;
}

double CBatteryState::lowPassFilter(double currentValue, double previousValue, double alpha) {
    return alpha * currentValue + (1 - alpha) * previousValue;
}

void CBatteryState::updateBatteryState(tSysBattery batt)
{
    percent = batt.percent;
    if(percent >= 100) percent = 100;
    volt = (double)batt.battVolt/1000;
    current = batt.current;
    
    filteredVolt = lowPassFilter(volt, filteredVolt, 0.3);
    percent = (filteredVolt - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0;

    if(battStateChecker(percent)) setBatteryState(tempState);
}

void CBatteryState::setBatteryState(E_BATTERY_STATE set){

    if(state != set)
    {
        bUpdateState = true;
        stateUpdateStartTime = SYSTEM_TOOL.getSystemTime();
        ceblog(LOG_LV_NECESSARY, CYN, "[BatteryState Changed] : "<< enumToString(state)<<" --> "<< enumToString(set) );
        ceblog(LOG_LV_NECESSARY, CYN, " percent[ " << percent << " ] volt[ " << volt << " ] filteredVolt[ " << filteredVolt << " ]");
    }
    state = set;
}
void CBatteryState::setTempState(E_BATTERY_STATE set)
{
    if(tempState != set)
    {
        stateChanged = true;
        stateChangeStartTime = SYSTEM_TOOL.getSystemTime();
        ceblog(LOG_LV_NECESSARY, GRAY, "[TempState Changed] : "<< enumToString(tempState)<<" --> "<< enumToString(set) );
        ceblog(LOG_LV_NECESSARY, GRAY, " percent[ " << percent << " ] volt[ " << volt << " ] filteredVolt[ " << filteredVolt << " ]");
    }
    tempState = set;
}

E_BATTERY_STATE CBatteryState::checkVoidState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_VOID;
    
    if(_percent <= NEED_CHARGE_PER)           ret = E_BATTERY_STATE::BATT_NEED_CHARGE;
    else if(_percent <= LOW_BATTERY_PER)      ret = E_BATTERY_STATE::BATT_LOW;
    else if(_percent <= MIDDLE_BATTERY_PER)   ret = E_BATTERY_STATE::BATT_MIDDLE;
    else  if(_percent <= HIGH_BATTERY_PER)    ret = E_BATTERY_STATE::BATT_HIGH;
    else                                      ret = E_BATTERY_STATE::BATT_FULL;

    return ret;
}
E_BATTERY_STATE CBatteryState::checkNeedChargeState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_NEED_CHARGE;
    
    if(_percent > NEED_CHARGE_PER) ret = E_BATTERY_STATE::BATT_LOW;

    return ret;
}
E_BATTERY_STATE CBatteryState::checkLowState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_LOW;
    
    if(_percent <= NEED_CHARGE_PER)          ret = E_BATTERY_STATE::BATT_NEED_CHARGE;//setBatteryState(E_BATTERY_STATE::BATT_LOW);
    else if(_percent > LOW_BATTERY_PER)     ret = E_BATTERY_STATE::BATT_MIDDLE;//setBatteryState(E_BATTERY_STATE::BATT_MIDDLE);

    return ret;
}
E_BATTERY_STATE CBatteryState::checkMidState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_MIDDLE;
    
    if(_percent <= LOW_BATTERY_PER)          ret = E_BATTERY_STATE::BATT_LOW;//setBatteryState(E_BATTERY_STATE::BATT_LOW);
    else  if(_percent > MIDDLE_BATTERY_PER)   ret = E_BATTERY_STATE::BATT_HIGH;//setBatteryState(E_BATTERY_STATE::BATT_HIGH);

    return ret;
}
E_BATTERY_STATE CBatteryState::checkHihgState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_HIGH;
    
    if(_percent <= MIDDLE_BATTERY_PER)        ret = E_BATTERY_STATE::BATT_MIDDLE;//setBatteryState(E_BATTERY_STATE::BATT_MIDDLE);
    else if(_percent > HIGH_BATTERY_PER)     ret = E_BATTERY_STATE::BATT_FULL;//setBatteryState(E_BATTERY_STATE::BATT_FULL);

    return ret;
}
E_BATTERY_STATE CBatteryState::checkFullState(u8 _percent)
{
    E_BATTERY_STATE ret = E_BATTERY_STATE::BATT_FULL;

    if(_percent < HIGH_BATTERY_PER)    ret = E_BATTERY_STATE::BATT_HIGH;

    return ret;
}

bool CBatteryState::checkStateChange()
{
    bool ret = false;
    double runTime = SYSTEM_TOOL.getSystemTime()-stateChangeStartTime;
    if((stateChanged && runTime >= 3)) //(state == E_BATTERY_STATE::BATT_VOID && tempState != E_BATTERY_STATE::BATT_VOID) ||
    {
        stateChanged = false;
        ret = true;
    }
    return ret;
}

bool CBatteryState::battStateChecker(u8 percentage)
{
    bool ret = checkStateChange();
    
    switch (state)
    {
    case E_BATTERY_STATE::BATT_VOID :
        setBatteryState(checkVoidState(percentage));
        break;
    case E_BATTERY_STATE::BATT_NEED_CHARGE :
        setTempState(checkNeedChargeState(percentage));
        break;
    case E_BATTERY_STATE::BATT_LOW :
        setTempState(checkLowState(percentage));
        break;
    case E_BATTERY_STATE::BATT_MIDDLE :
        setTempState(checkMidState(percentage));
        break;
    case E_BATTERY_STATE::BATT_HIGH :
        setTempState(checkHihgState(percentage));
        break;
     case E_BATTERY_STATE::BATT_FULL :
        setTempState(checkFullState(percentage));
        break;    
    
    default:
        break;
    }

    return ret;
}

bool CBatteryState::isBatteryStateChanged()
{
    return stateChanged;
}

double CBatteryState::getStateChangeTime()
{
    return stateUpdateStartTime;
}

bool CBatteryState::isUpateState()
{
    bool ret = false;
    if(bUpdateState){
        ret = bUpdateState;
        bUpdateState = false;
    } 
    return ret;
}