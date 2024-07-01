/**
 * @file sensorCliff.h
 * @author hjkim
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"


#define TOF_STBY_RECHECK_HIGH_ABNORMAL 120//115//105
//bykim:210725: w/s 2차 기준값이 높다. 추후 재조정.(130->135)
#define TOF_STBY_HIGH_ABNORMAL	135

#define TOF_STBY_LOW_ABNORMAL	85
#define TOF_RANGE_LIMIT		1200
//cliff : 50mm 
#define TOF_STANDBY_AVG_PTG 150//130//70//58 // 58 
////////////////////////////////////////////////////////////////////////////
                    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //    // 업로드 금지      //
////////////////////////////////////////////////////////////////////////////
#define TOF_CLIFF_LIMIT_HIGH 190//160//185//170//135 //650
#define TOF_CLIFF_LIMIT_LOW 130//160//125 //650
#define TOF_CLIFF_TILT_UP_CORRECTION_FACTOR 1.38 //1.35는 2cm회피 // 1.3에서 1cm 둔턱 넘는 도중 회피함.

//hjkim 210225 : T3기준 TOF 센서 노이즈 10% + 마진을 생각하여, 오 동작 가능성을 위한 기준값 개선
//               standby : 100~115
//               0.5 러그 : 85~95
//               최소값 수준의 AVG 100 X 0.9 = 90 (O) /  최대값 수준 AVG 115 X 0.8 = 92(O) / 보통 수준 AVG 110 X 0.85 = 93.5(O)
#define TOF_KNOLL_AVG_PTG	  85///90
//hjkim210318 - Avg high 값이 약 120정도 라 했을 때 둔턱 최대 값은 102까지 나올 수 있다.
//              하지만 calib과정에서 평균값이 조금 하향 되어 잡힐 수 도 있기 때문에 마진을 두고 최대값을 상향 하였음
#define TOF_KNOLL_LIMIT_HIGH  98//100//98
#define TOF_KNOLL_LIMIT_LOW   80

#define TOF_OBSTACLE_AVG_PTG 55
#define TOF_OBSTACLE_LIMIT_HIGH 65
#define TOF_OBSTACLE_LIMIT_LOW 53



typedef struct
{
    u16 avg;
    u16 avg_min;
    u16 avg_max;
    u16 cliff_limit;
    u16 obs_limit;
    u16 knoll_limit;
    u32 sum;
    u8 count;
    u8 maskCount;
    //bool ToFCalib;
    u8 ToFCalib;
    
} TOF_DATA;

/**
 * @brief abstract class
 * 
 */
class CCliffSensors
{
private:
    tCliffActionState cliffActionState;
    TOF_DATA calib_LeftData;
    TOF_DATA calib_RightData;
    bool isTofAccumulateCplt;
    int debugCnt;    

    RSF_OBSTACLE_MASK cliffMasking(tSysCliff cliff);
    bool accumulateCliffSensor(u16 left,u16 right);

protected:
    bool CheckToF_DataValidate(u16 data);
    void accumlate(TOF_DATA *pTofData, u16 data);

public:
    RSF_OBSTACLE_MASK mask;
    RSF_OBSTACLE_MASK pre_mask;    

    CCliffSensors();
    ~CCliffSensors();
    void initItem();

    RSF_OBSTACLE_MASK getCliffSensorMaskValue(tSysCliff cliff);
    RSF_OBSTACLE_MASK getCliffSensorMaskValue(tCliffActionState cliff);
    u16 getLeftTofCalibAverage();
    u16 getRightTofCalibAverage();
    bool isAccumulateComplete();
    

    void initCliffSensor();
    RSF_OBSTACLE_MASK getCliffMask();
    void updateCliffSensor(tSysCliff cliff, tCliffActionState state);
    u16 GetCliffLimit(u16 avg);
    u16 GetObstacleLimit(u16 avg);
    u16 GetKnollLimit(u16 avg);
    tCliffActionState getActionState();

};
