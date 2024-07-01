#pragma once
#include "commonStruct.h"

typedef struct _tDoNotDisturb
{
    _tDoNotDisturb() : status(0), startTime(nullptr), endTime(nullptr) {}

    short status;
    char* startTime;
    char* endTime;
}tDoNotDisturb;

class CDoNotDisturb
{
private:
    tDoNotDisturb doNotDisturb;
    bool bDistruptMode;

public:
    CDoNotDisturb();
    ~CDoNotDisturb();
    
    void setDoNotDisturb();
    tDoNotDisturb getDoNotDisturb();

    void setDistruptStatus(short set);
    void setDistruptMode(bool set);
    bool isDistruptMode();

};