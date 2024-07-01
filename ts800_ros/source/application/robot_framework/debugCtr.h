#pragma once

#include <pthread.h>
#include "rsfMonitor.h"
#include "eblog.h"
#include "utils.h"
#include "define.h"
#include "coreData/serviceData.h"
#include "imgProcessor.h"

#define DEBUG_CTR CDebugCtr::getInstance()

struct tPathplanDebug
{
    std::list<tPoint>       path;                 // 경로 list
    std::list<tPoint>       edgePath;
};

class CDebugCtr 
{

private: /* not use! */
    CDebugCtr();
	~CDebugCtr();
    CDebugCtr(const CDebugCtr& other) = default; // Copy constructor
    CDebugCtr& operator=(const CDebugCtr& other) = default; // Assignment operator

public:
    static CDebugCtr& getInstance();

    cv::Mat dSimplifyMap;
    void d_setSimplifyMap(cv::Mat set);
    cv::Mat d_getSimplifyMap();
    void d_makeArea();

public:
    tUpdateData<bool> isAliveImgProc;
    tUpdateData<bool> isAliveMessage;
    tUpdateData<bool> isAliveRosDataPub;
    tUpdateData<bool> isAliveSystemWhatch;
    tUpdateData<bool> isAliveDstarWallUpdate;
    tUpdateData<bool> isAliveWaveFrontier;
    tUpdateData<bool> isAliveRosCallbackLoop;
    tUpdateData<bool> isAliveSystemInterface;
    tUpdateData<bool> isAliveMotionController;    
    tUpdateData<bool> isAliveMotionPlanner;
    tUpdateData<tPose>  rvizGoalPose; // debug용 rviz의 navigation goal 좌표
};
