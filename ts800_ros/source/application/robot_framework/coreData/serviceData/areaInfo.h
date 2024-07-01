#pragma once
#include "commonStruct.h"

typedef struct _tAreaInfoData
{
    _tAreaInfoData() {}

    std::list<tAreaInfo> info;
    short areaNumber;
}tAreaInfoData;

typedef struct _tAreaEdit
{
    _tAreaEdit() {}

    int divideNum;
    int combineNum;
    tDivideArea divide;
    tCombieArea combine;

}tAreaEdit;



class CAreaInfo
{
private:
    tAreaInfoData areaInfoData;
    tAreaEdit editArea;

public:
    CAreaInfo();
    ~CAreaInfo();
    
    void setEditAreaData();
    tAreaEdit getEditAreaData();
    tDivideArea getDivideAreaData();
    tCombieArea getCombineAreaData();

    void setAreaInfo();
    tAreaInfoData getAreaInfo();

};