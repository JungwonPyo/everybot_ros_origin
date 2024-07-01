/**
 * @file debugData.h
 * @author jhnoh
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <list>
#include "cleanRoom.h"

struct tExplorerDebug
{
    std::list<tPoint> frontiers;

};



struct tCleanAreaDebug
{
    std::list<CCleanRoom> rooms;
    int currentRoomId;
    std::list<tPoint>     currentAreaPolygons; 
};

struct tPidErrorDebug
{
    double errP;
    double errI;
    double errD;

    double dummy1;
    double dummy2;
};

class CDebugData
{
private:
    tExplorerDebug explorerData;    
    
    tCleanAreaDebug cleanAreaData;
    std::list<tPose> cleanPathData;
    tPidErrorDebug pidErrorData;

    
    bool            bUpdateExplorer;
    bool            bUpdateDocking;
public:
    CDebugData();
    ~CDebugData();

    // explorer
    void explorerDataClear();
    void setExplorerDebug(const tExplorerDebug &set);
    tExplorerDebug getExplorerDebug();
    void setUpdateExplorer(const bool set);
    

    // docking
    bool getUpdateDocking();
    void setUpdateDocking(bool set);
    
    // area
    void cleanAreaDataclear();
    void setCleanAreaDebug(const tCleanAreaDebug &set);
    tCleanAreaDebug getCleanAreaDebug();

    // clean path
    void setCleanPathDebug(const std::list<tPose> &set);
    std::list<tPose> getCleanPathDebug();

    // pid error
    void clearPidErrorData();
    void setPidErrorData(const tPidErrorDebug &set);
    tPidErrorDebug getPidErrorData();
};
