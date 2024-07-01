#pragma once
/**
 * @file dstar.h
 * @author jhnoh
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <atomic>
#include <iostream>
#include <cmath>
#include <stack>
#include <queue>
#include <list>
#include <unordered_map>
#include "utils.h"
#include "commonStruct.h"
#include <math.h>
#include "robotmap.h"
#include "systemTool.h" 
#include "pathFinderInterface.h"
#include "pthreadLockGuard.h"
#include "opencv2/opencv.hpp"

#define hash_map unordered_map
using namespace std;
using namespace __gnu_cxx;
class state 
{
public:
    int x;
    int y;
    pair<double,double> k;

    bool operator == (const state &s2) const 
    {
        return ((x == s2.x) && (y == s2.y));
    }

    bool operator != (const state &s2) const 
    {
        return ((x != s2.x) || (y != s2.y));
    }

    bool operator > (const state &s2) const 
    {
        if (k.first-0.00001 > s2.k.first) return true;
        else if (k.first < s2.k.first-0.00001) return false;
        return k.second > s2.k.second;
    }

    bool operator <= (const state &s2) const 
    {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + 0.00001;
    }


    bool operator < (const state &s2) const 
    {
        if (k.first + 0.000001 < s2.k.first) return true;
        else if (k.first - 0.000001 > s2.k.first) return false;
        return k.second < s2.k.second;
    }

};

struct cellInfo 
{
    double g;       // point와 goalpoint까지의 경로 비용
    double rhs;     // point1의 인접한 point의 g를 사용하여 point1의 최적 경로 비용 예측하는 값
    double cost;    // 경로 비용
};

class state_hash 
{
public:
    size_t operator()(const state &s) const 
    {
        return s.x + 34245*s.y;
    }
};

typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
typedef hash_map<state,cellInfo, state_hash, equal_to<state> > ds_ch;
typedef hash_map<state, float, state_hash, equal_to<state> > ds_oh;


typedef struct
{       
    bool isRobotInWall;

    double unknownCellCost;
    double k_m;             // point가 바뀜으로 달라지는 어림값 보정을 위한 값
    int maxSteps;    
    state startPoint, goalPoint, s_last;

    //가까운 경로 및 골 찾기    
    bool isRunFindNearestPath;           // 데이터 업데이트 확인
    bool bFindNearSuccess;          // 데이터 성공여부 확인
    std::list<tPoint> nearestPath;  // 가까운곳 경로
    tPoint            nearestGoal;  // 가까운곳 도착점

}tDstarData;

typedef struct tDstarWallPoint {
    int x, y;
    double costValue;

    // 생성자
    tDstarWallPoint() : x(0), y(0), costValue(0.0) {}
    tDstarWallPoint(int _x, int _y, double _costValue) : x(_x), y(_y), costValue(_costValue) {}

    // == 연산자 오버로딩
    bool operator==(const tDstarWallPoint& other) const {
        return (x == other.x) && (y == other.y) && (costValue == other.costValue);
    }
} tDstarWallPoint;

class CDstar : public CPathFinderInterface
{
private:    
    
    // dstar core
    pthread_mutex_t  mutexDstarCore;    // dstar core 보호용.    
    
    // obstacle wall
    pthread_mutex_t  mutexObstacleWall;    // obstacle wall 보호용.    
    std::list <tDstarWallPoint> obstacleWall;
    bool bObstacleWallUpdate;
    std::list <tDstarWallPoint> useObstacleWall();    
    
    //grid map wall
    pthread_mutex_t  mutexGridMapWall;    // grid map wall 보호용.
    std::list<tDstarWallPoint> gridMapWall;
    bool bUpdatedGridMapWall;
    std::list <tDstarWallPoint> useGridMapWall();

public:    
    CDstar();
    ~CDstar();    
    void findNearestPath(tPose robotPose, std::list<tPoint> searchingPoints, 
        int searchingCnt);

    //wall 관련.
    bool isWall(tPoint point);
    bool isSameDstarPoint(tPoint point1, tPoint point2);
    void updateDstarWallPoint(tPoint wallPoint);
    void updateDstarWallBumper(tPose robotPose, bool left, bool right);
    bool isUpdateObstacleWall();
    bool isUpdateGridMapWall();
    void updateGridMapWall();
    void updateObstacleWall();
    void clearObstacleWall();
    std::list <tDstarWallPoint> combineWall(std::list <tDstarWallPoint> grid, std::list <tDstarWallPoint> obstacle);
    
    void makeWallListFromGridMap(tPose robotPose, tGridmapInfo mapInfo, u8 *pGridmap);
    void setObstacleWall(std::list <tDstarWallPoint> &set);

    void    init(tPose robotPose, tPoint goalPoint);
    bool    isRunFindNearestPath();
    void    setFindNearSuccess(bool ret);
    bool    getFindNearSuccess();



    std::list<tPoint>   makeSmoothPath(std::list<tPoint> rawPath);
    std::list<tPoint>   getNeighboringCoordinates8(tPoint centerXY, double size);
    std::list<tPoint>   getNeighboringCoordinates4(tPoint centerXY, double size);
    std::list<tPoint>   findFarthestPoints(const std::list<tPoint>& firstPoints, const std::list<tPoint>& secondPoints) ;
    std::list<tPoint>   sortCoordinatesByDistance(const tPoint& centerPoint, const tPoint& comparePoint, const std::list<tPoint>& coordinates);
    bool                checkNeighboursWall(state point);    
    void                setNearPath(std::list<tPoint> nearestPath);
    std::list<tPoint>   getPath();
    void clearPath();

    void    updatePoint(std::list <tDstarWallPoint> &set);
    void    updatePoint(int pointX, int pointY, double costValue);
    void    updateGoal(tPoint goalPoint); 
    bool    copyDstarGridMap(u8 *& dest, tGridmapInfo *pInfo);
    bool    isXYclose(double x, double y); 
    bool    replan();
    void    getPath(list<tPoint> &path, tPoint orgTarget);
    void    clearWall();
    list<state> shortestPath;

    Mat where(const Mat &condition, const Mat &x, const Mat &y);



private:
    tDstarData dstarData_;
    ds_pq openList;
    ds_ch cellHash;
    ds_oh openHash;
    tGridmapInfo dstarCellInfo;

    bool    checkValueClose(double value1, double value2);
    void    makeNewCell(state point);
    double  getG(state point);
    double  getRhs(state point);
    void    setG(state point, double g);
    double  setRhs(state point, double rhs);
    double  eightCondist(state point1, state point2);
    int     computeShortestPath();
    void    updateVertex(state point);
    void    insert(state point);
    double  euclideanCost(state point1, state point2);
    double  heuristic(state point1, state point2);
    state   calculateKey(state point);
    void    getNextState(state point, list<state> &nextPoint);
    void    getPerviousState(state point, list<state> &perviousPoint);
    double  cost(state point1, state point2);
    bool    checkOccupied(state point);
    bool    isValid(state point);
    float   keyHashCode(state point);
};
