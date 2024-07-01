#pragma once

#include <list>
#include "commonStruct.h"
#include "gridmap.h"

class CPathFinderInterface
{
private:
    bool bisNeedMapUpdate;
    bool bisRunMapUpdate;
    pthread_t thPathMapUpdate;    
    static void* threadPathMapUpdateWrap(void* arg)
    {
        CPathFinderInterface* myMotionPlanner = static_cast<CPathFinderInterface*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_DSTAR_WALL_UPDATE);
        myMotionPlanner->threadPathMapUpdate();
    }
    void threadPathMapUpdate();

    bool isRunfindNearestPath;
    virtual void findNearestPath(tPose robotPose, std::list<tPoint> searchingPoint, 
        int searchingCnt) = 0;
    virtual bool isRunFindNearestPath() = 0;
    virtual bool getFindNearSuccess() = 0;
    virtual void updateGridMapWall() = 0;
    virtual void makeWallListFromGridMap(tPose robotPose, tGridmapInfo mapInfo, u8 *pGridmap) = 0;    
    virtual void clearPath() = 0;

public: 
    CPathFinderInterface();    
    virtual ~CPathFinderInterface(){}
    void destroyPathFinderInterface();

    virtual std::list<tPoint> getPath() = 0;
    virtual bool isWall(tPoint point) = 0;
    virtual bool isSameDstarPoint(tPoint point1, tPoint point2) = 0;
    virtual bool copyDstarGridMap(u8 *& dest, tGridmapInfo *pInfo) = 0;
    virtual void updateDstarWallBumper(tPose robotPose, bool left, bool right) = 0;    
    
    bool dofindNearestPath(tPose robotPose, std::list<tPoint> searchingPoints, 
        int searchingCnt);
    bool dofindPath(tPose robotPose, tPoint searchingPoints);
    bool isRunFindPath();
    bool isFindPath();
    bool isNeedMapUpdate();
    void startMapUpdate();
    void stopMapUpdate();
    bool doUpdateDstarMap();
    void updateDstarMap();   

};

