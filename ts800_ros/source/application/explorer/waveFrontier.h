/**
 * @file waveFrontier.h
 * @author 담당자 미정
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "commonStruct.h"
#include "gridmap.h"
#include "pthreadLockGuard.h"

typedef struct _wfdPoint
{
    _wfdPoint(){}
    _wfdPoint(tPoint _pt, bool _bCheck = false) : pt{_pt}, bCheck{_bCheck} {}
    tPoint pt;
    bool bCheck;
}wfdPoint;


class CWaveFrontier
{
private:    
    pthread_mutex_t  mutexWfdPoints;    // wfdPoints 보호용.    
    std::list<wfdPoint> wfdPoints;    
    u8 *wfdMap;
    

public: 
    CWaveFrontier();
    ~CWaveFrontier();    
    
    bool useInvalidNearPoint(tPoint &pt);
    bool isUpdateWdfPoints();    
    void startUpdate();
    void stopUpdate();
    bool isFinish();
    bool isNeedUpdate();
    bool doUpdateWaveFrontier();
    void updateWaveFrontier();

private:
    int MAP_OPEN_LIST, MAP_CLOSE_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST;    
    bool bisNeedUPdate; // true 이면 로봇의 주변을 업데이트.
    bool bisRunMapUpdate;        
    
    pthread_t thWaveFrontier;

    int searchingCnt;

    static void* threadWaveFrontierWrap(void* arg)
    {
        CWaveFrontier* myClass = static_cast<CWaveFrontier*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_WAVE_FRONTIER);
        myClass->threadWaveFrontier();
    }
    void threadWaveFrontier();
   
    void waveFrontier(tGridmapInfo info, int x, int y,tPose robotPose);
    void insertWfd(std::list<tPoint> tunedPoints);
    void sortWfd(tPose robotPose);
    void findValidNearPoint(std::list<tPoint> frontierList);
    void checkInvalidPoint();
    void updateWfdMap(u8 *map, tGridmapInfo info);
    void getNeighbours8(int n_array[], int position, int map_width);
    void getNeighbours4(int n_array[], int position, int map_width);    
    bool isFrontierPoint(u8 * map, int point, int map_size, int map_width);
    std::list<tPoint> meanShiftClustering(const std::list<tPoint>& data) ;
    tPoint meanShift(const tPoint& point, const std::list<tPoint>& data, double bandwidth);
    void convertRobotPoseToWavePose(double rx, double ry, int *wx, int *wy, tGridmapInfo mapInfo);
    
};