#include "sensorCliff.h"
#include "control/control.h"
#include "control/motionPlanner/motionPlanner.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

bool CCliffSensors::CheckToF_DataValidate(u16 data)
{
    bool ret = true;
    if(data == 4 || data == 1200 ) //|| status == 0x04
    {
        ret = false;
    }

    return ret;
}

void CCliffSensors::accumlate(TOF_DATA *pTofData, u16 data)
{
    bool ret = false;

    if(CheckToF_DataValidate(data))
    {
        if(pTofData->count < 50)
        {
            pTofData->sum += data;
            pTofData->count++;//calib_LeftData.count++;
        }
        else
        {
            pTofData->avg = pTofData->sum/pTofData->count;
            pTofData->cliff_limit = GetCliffLimit(pTofData->avg);
            pTofData->obs_limit = GetObstacleLimit(pTofData->avg);
            pTofData->knoll_limit = GetKnollLimit(pTofData->avg);
            ret =true;
        }
    }

    pTofData->ToFCalib = ret;
}    


CCliffSensors::CCliffSensors(/* args */){        
    eblog(LOG_LV,  "");
}
CCliffSensors::~CCliffSensors(){
    eblog(LOG_LV,  "");
}


void CCliffSensors::initCliffSensor(){
    initItem();
    mask.value = 0;
    pre_mask.value = 0;        
}

RSF_OBSTACLE_MASK CCliffSensors::getCliffMask()
{
    RSF_OBSTACLE_MASK ret;
    ret.value = 0;

    if(pre_mask.value)
    {
        eblog(LOG_LV_OBSTACLE,  "getCliffMask :" << SC<int>(pre_mask.value));
        ret.value = pre_mask.value;
        pre_mask.value = 0;
    }
    
    return ret;
}

/**
 * @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
 * @brief 낙하 ToF를 업데이트 한다.
 * @param cliff 
 * @param tilt 
 * @note 연산시간 ms
 * @date 2023-11-06
 * @author hhryu
 */
void CCliffSensors::updateCliffSensor(tSysCliff cliff, tCliffActionState set)
{
    cliffActionState = set;
    mask = getCliffSensorMaskValue(cliffActionState);
    getCliffSensorMaskValue(cliff);
    if(mask.value) pre_mask.value = mask.value;
}

u16 CCliffSensors::GetCliffLimit(u16 avg)
{
    /* hhryu,230202 : ToF 낙하 감지  
    *  TS800 W/S 1차 ToF 실측 기준 스탠다드 90~110
    *  5cm 낙하 170~200
    *  avg+75로 임시 설정(165~185)
    *  MAX : 180, MIN : 160으로 설정.
    */
    u16 ret = avg + 75;//33;//50; // 5cm 낙하 기준.

    if( ret >= TOF_CLIFF_LIMIT_HIGH ) ret = TOF_CLIFF_LIMIT_HIGH;   //185
    else if( ret <= TOF_CLIFF_LIMIT_LOW) ret = TOF_CLIFF_LIMIT_LOW; //165

    return ret;
}

u16 CCliffSensors::GetObstacleLimit(u16 avg)
{
    u16 ret = (u16)((avg * TOF_OBSTACLE_AVG_PTG)/100);
    
    if( ret >= TOF_OBSTACLE_LIMIT_HIGH ) ret = TOF_OBSTACLE_LIMIT_HIGH;
    else if( ret <= TOF_OBSTACLE_LIMIT_LOW ) ret = TOF_OBSTACLE_LIMIT_LOW;

    return ret;
}

u16 CCliffSensors::GetKnollLimit(u16 avg)
{
    u16 ret = (u16)((avg * TOF_KNOLL_AVG_PTG)/100);

    if( ret >= TOF_KNOLL_LIMIT_HIGH ) ret = TOF_KNOLL_LIMIT_HIGH;
    else if( ret <= TOF_KNOLL_LIMIT_LOW ) ret = TOF_KNOLL_LIMIT_LOW;

    return ret;
}

tCliffActionState CCliffSensors::getActionState()
{
    return cliffActionState;
}



void CCliffSensors::initItem()
{
    debugCnt = 0;
    calib_LeftData = {0};
    calib_RightData = {0};
    isTofAccumulateCplt = false;
}    

bool CCliffSensors::isAccumulateComplete()
{
    return isTofAccumulateCplt;
}


bool CCliffSensors::accumulateCliffSensor(u16 left,u16 right)
{   
    bool ret = false;

    accumlate(&calib_LeftData, left);
    accumlate(&calib_RightData, right);
    if(++debugCnt >= 100){
        debugCnt = 0;
        eblog(LOG_LV_NECESSARY,  "Tof Validate Check left :" << SC<int>(left) << " right : " << SC<int>(right));
    }
    
    if(calib_LeftData.ToFCalib && calib_RightData.ToFCalib) 
        ret = true;

    return ret;
}

/** 
 * @brief 낙하 ToF를 업데이트 한다.
 * @param cliff 
 * @param tilt 
 * @note 연산시간 ms
 * @date 2023-11-06
 * @author hhryu
 */
RSF_OBSTACLE_MASK CCliffSensors::getCliffSensorMaskValue(tCliffActionState cliff)
{
    RSF_OBSTACLE_MASK maskTemp;
    maskTemp.value = 0;

    maskTemp.b.fleft_side = cliff.bLeftDetect;
    maskTemp.b.fright_side = cliff.bRightDetect;

    return maskTemp;
}


RSF_OBSTACLE_MASK CCliffSensors::getCliffSensorMaskValue(tSysCliff cliff)
{
    RSF_OBSTACLE_MASK maskTemp;
    maskTemp.value = 0;

    if (!isAccumulateComplete())
    {
        if (accumulateCliffSensor(cliff.left.rangeAvg, cliff.right.rangeAvg))
        {
            isTofAccumulateCplt = true;
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, "ToFCalibraiton Complete avg[ " << calib_LeftData.avg << " , " << calib_RightData.avg << "] Cliff[ " << calib_LeftData.cliff_limit << " , " << calib_RightData.cliff_limit << " ] ");
        }
    }
    else
    {
        maskTemp = cliffMasking(cliff);        
    }
    return maskTemp;
}

/**
 * @brief cliff count check, cliff mask set.
 * 
 * @param cliff 
 * @param tilt 
 * @return RSF_OBSTACLE_MASK 
 * 
 * @note 연산시간 ms
 * @date 2023-11-07
 * @author hhryu
 */
RSF_OBSTACLE_MASK CCliffSensors::cliffMasking(tSysCliff cliff)
{
    RSF_OBSTACLE_MASK maskTemp;
    maskTemp.value = 0;
    
    return maskTemp;
}

u16 CCliffSensors::getLeftTofCalibAverage()
{
    return calib_LeftData.avg;
}

u16 CCliffSensors::getRightTofCalibAverage()
{
    return calib_RightData.avg;
}
