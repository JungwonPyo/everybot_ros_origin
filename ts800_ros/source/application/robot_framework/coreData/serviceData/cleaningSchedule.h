#pragma once
#include "commonStruct.h"


typedef struct _tCleaningSchedule
{
    _tCleaningSchedule() {}

    std::list<tCleanSchedule> schedule;
    short scheduleNumber;
}tCleaningSchedule;

class CCleaningSchedule
{
private:
    tCleaningSchedule cleaningSchedule;

public:
    CCleaningSchedule();
    ~CCleaningSchedule();
    
    void setCleaningSchedule();
    tCleaningSchedule getCleaningSchedule();

};