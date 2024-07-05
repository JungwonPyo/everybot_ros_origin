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
#include <math.h>

#define hash_map unordered_map
using namespace std;
using namespace __gnu_cxx;


/**
 * @brief (x, y) 에 대한 구조체 정의 
 * ex : 각도 정보가 필요 없이 실수의 좌표가 필요한경우
 */
typedef struct _tPoint
{
    _tPoint() : x{0.0}, y{0.0} {}
    _tPoint(double _x, double _y) : x{_x}, y{_y} {}
    double x;   //단위 : m
    double y;   //단위 : m

    /// Operator of points summation by element-wise addition.
    _tPoint operator+(const _tPoint &p) const {return _tPoint(x + p.x, y + p.y);}

    /// Returns additive inverse of the point.
    _tPoint operator-() const {return _tPoint(-x, -y);}

    // Equality operator
    bool operator==(const _tPoint &p) const { return (x == p.x) && (y == p.y); }

     // 유클리디안 거리 계산 함수
    static double distance(const _tPoint& a, const _tPoint& b) {
        return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    }

    // 선분의 기울기 계산 함수
    static double slope(const _tPoint& a, const _tPoint& b) {
        if (b.x == a.x) {
            return 0.0; // 수직 선분
        }
        return static_cast<double>(b.y - a.y) / (b.x - a.x);
    }
}tPoint;


/**
 * @brief x, y, angle 에 대한 위치 구조체
 * ex : 로봇의 위치, 타겟 위치
 */
typedef struct _tPose
{
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    _tPose() : x{0.0}, y{0.0}, angle{0.0}, calSn{0} {}
    _tPose(double _x, double _y, double _angle) : x{_x}, y{_y}, angle{_angle} {}
    _tPose(double _x, double _y, double _angle, unsigned int _calSn) : x{_x}, y{_y}, angle{_angle}, calSn{_calSn} {}
#else
    _tPose() : x{0.0}, y{0.0}, angle{0.0} {}
    _tPose(double _x, double _y, double _angle) : x{_x}, y{_y}, angle{_angle} {}
#endif

    double x;   //단위 : m
    double y;   //단위 : m
    double angle;   //단위 : rad or deg.... -> 통일 필요
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)  //인 경우 calibration no
    unsigned int calSn; //
#endif
    bool operator==(const _tPose& __a)
    {
        return __a.x == this->x && __a.y == this->y && __a.angle == this->angle;
    }
    double distance(const tPoint& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
    tPoint convertPoint() const{
        return tPoint(x, y);
    }
}tPose;


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

/**
 * @brief grid map 이나 이미지 같은곳에서 사용하는 좌표계
 * 
 */
typedef struct
{
    int x;
    int y;
}tCellPoint;


typedef struct _tGridmapInfo
{
    _tGridmapInfo() : resolution{0.0}, width{0}, height{0}, origin_x{0.0}, origin_y{0.0} {}
    _tGridmapInfo(float _resolution, unsigned int _width, unsigned int _height, float _origin_x, float _origin_y) 
        : resolution{_resolution}, width{_width}, height{_height}, origin_x{_origin_x}, origin_y{_origin_y} {}

    float resolution;   // [m/cell]
    unsigned int width;          // [cell]
    unsigned int height;         // [cell]
    float origin_x;     // cell (0,0) 기준 map 중심 위치 [m]
    float origin_y;
}tGridmapInfo;

class CDstar
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
    void findNearestPath(tPoint start, tPoint goal);

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
    
    std::list<tPoint> makeWallListFromGridMap(tPose robotPose, tGridmapInfo mapInfo, unsigned char *pGridmap);
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
    bool    isXYclose(double x, double y); 
    bool    replan();
    void    getPath(list<tPoint> &path, tPoint orgTarget);
    void    clearWall();
    list<state> shortestPath;

    void getNeighbours8(int n_array[], int position, int map_width);
    bool isValid(int x, int y, int rows, int cols);
    bool getNearestValidPosition(int newPosition[], int robotCellX, int robotCellY, tGridmapInfo mapInfo, u8 *pGridmap);

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
