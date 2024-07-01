/**
 * @file escapeWall.h
 * @author jspark
 * @brief Dstar맵에서 로봇이 벽에 갇혔을 때 탈출 알고리즘.
 * 
 * Dstar 맵 cell 번호
 *         x축+
 *     [ 0 |  [ 1 |  [ 2 |
 *     [   |  [   |  [   |
 *   
 * y축+| 7 ]  | 로]  | 3 ]
 *     |   ]  | 봇]  |   ]
 *     
 *     [ 6 |  [ 5 |  [ 4 |
 *     [   |  [   |  [   |
 * @version 0.1
 * @date 2023-12-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "commonStruct.h"
#include "dstar.h"
#include <deque>

class CEscapeWall
{
public:
    enum class E_STATE
    {
        NONE,
        RUNNING,
        COMPLETE,
        FAIL,
    };

    enum class E_CELL_STATE
    {
        UNKNOWN,
        CHECKING,
        COMEBACK,
        EMPTY_PLACE,
    };

    // 범용으로 사용할 control 순서 enum
    enum class E_MOVE_STEP
    {
        _INIT,
        _1,
        _1_WAITING,
        _2,
        _2_WAITING,
        _3,
        _3_WAITING,
    };

    struct tBumperObstacle
    {
        tBumperObstacle(tPose _robotPose, bool _left, bool _right) : robotPose{_robotPose}, left{_left}, right{_right} {}
        tPose robotPose;
        bool left;
        bool right;
    };

    static std::string enumToString(E_STATE value) {
        static const std::unordered_map<E_STATE, std::string> enumToStringMap = {
            { E_STATE::NONE, "None," },
            { E_STATE::RUNNING, "Run," },
            { E_STATE::COMPLETE, "Complete," },
            { E_STATE::FAIL, "Fail," },
        };
        auto it = enumToStringMap.find(value);
        return it!=enumToStringMap.end() ? it->second : "Unknonw";
    }

    static std::string enumToString(E_CELL_STATE value) {
        static const std::unordered_map<E_CELL_STATE, std::string> enumToStringMap = {
            { E_CELL_STATE::UNKNOWN, "Unknown," },
            { E_CELL_STATE::CHECKING, "Checking," },
            { E_CELL_STATE::COMEBACK, "Comeback," },
            { E_CELL_STATE::EMPTY_PLACE, "Empty Place," },
        };
        auto it = enumToStringMap.find(value);
        return it!=enumToStringMap.end() ? it->second : "Unknonw?";
    }

    struct CellInfo
    {
        CellInfo(int _index, tPoint _point) : index{_index}, point{_point}, state{E_CELL_STATE::UNKNOWN} {}
        int index;
        tPoint point;
        E_CELL_STATE state;
    };
    
private:    
    E_STATE state;
    tPoint startPoint;
    /**
     * @brief pari<cell위치, cell상태>
     */
    std::deque<CellInfo> searchCells;
    tPose robotPose;
    E_MOVE_STEP moveStep;
    std::list<tBumperObstacle> bumperList; // bumper 장애물 정보 list
    RSF_OBSTACLE_MASK startBumperMask;

public:
    CEscapeWall();
    ~CEscapeWall();
    E_STATE getState();

    bool isRobotInWall();
    bool isPointInWall(tPoint point);
    void start();
    void start(RSF_OBSTACLE_MASK bumperMask);
    void proc();

private:
    void setState(E_STATE state);
    void initEscapeWall(tPose startPose);
    E_STATE escapeWallAlgorithm();
    E_CELL_STATE escapeWallMove();
    E_CELL_STATE escapeWallChecking();
    E_CELL_STATE escapeWallComeback();

    void getNearCellPoints(double dstarCellResolution, std::vector<tPoint> &points);
    int getStartIndex(double robotAngle);
    bool findSearchOrder(int startIndex, const std::vector<tPoint>& cells);
    bool checkObstacle();
public:
    std::deque<CellInfo> debugCells;
    void debugCellPrint();
};