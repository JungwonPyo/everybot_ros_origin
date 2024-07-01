#pragma once


#include "escapeWall.h"
#include "walltracking.h"
#include "docking/signaltracking.h"
#include "cleanplan.h"

#define SUB_TASK CSubTask::getInstance()

class CSubTask
{
private:
    CSubTask();
    ~CSubTask();
    
public:
    static CSubTask& getInstance();
    void init();    
    
    CWaveFrontier waveFrontier;
    CWalltracking walltracking;
    CSignaltracking signaltracking;
    CCleanPlan cleanPlan;
};