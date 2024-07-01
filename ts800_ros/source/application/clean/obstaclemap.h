/**
 * @file obstaclemap.h
 * @author 담당자 미정
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "commonStruct.h"
#include <sensor_msgs/LaserScan.h>

#define CELL_OBS_RESOLUTION 0.05    //단위 : m
#define CELL_MOVE_OFFSET 0.06       
#define CELL_OBS_WIDTH 100
#define CELL_OBS_HEIGHT 100
#define CELL_OBS_HALFWIDTH 50
#define CELL_OBS_HALFHEIGHT 50
#define CELL_OBS_SIZE ((CELL_OBS_WIDTH) * (CELL_OBS_HEIGHT))
#define LIDAR_OBS_DISTANCE 150

union cell_obstacle
{
    u8 value;
    struct
    {
        u8 lidar        : 1;    // 0:notuse     1:known - 탐색 지역
        u8 cliff        : 1;    // 0:empty      1:wall
        u8 bumper       : 1;    // 0:unclean    1:clean
        u8 a            : 1;
        u8 b            : 1;
        u8 c            : 1;
        u8 d            : 1;
        u8 e            : 1;
    }b;
};

typedef enum
{
    CELL_OBS_TYPE_LIDAR,
    CELL_OBS_TYPE_CLIFF,
    CELL_OBS_TYPE_BUMPER,
    CELL_OBS_TYPE_A,
    CELL_OBS_TYPE_B,
    CELL_OBS_TYPE_C,
    CELL_OBS_TYPE_D,
    CELL_OBS_TYPE_E,
}E_CELL_OBSTACLE_TYPE;


class CObstacleMapCell{
public:
    CObstacleMapCell();
    ~CObstacleMapCell();

    bool isDetected(){ return bDetected; }
    void setId(int set){ id = set; }
    int getId(){ return id; }
    
    void update();
    void lifeUp();

private:
    bool bDetected;
    int life;
    int term;
    int id;
    void clear();
};

class CObstaclemap
{
private: /* not use! */
    CObstaclemap(const CObstaclemap& other); // Copy constructor
    CObstaclemap& operator=(const CObstaclemap& other); // Assignment operator


    cell_obstacle *cellObstacle;
    CObstacleMapCell cells[CELL_OBS_WIDTH][CELL_OBS_HEIGHT];

public:
    CObstaclemap();
    ~CObstaclemap();
    void setCellFromLidar(double *LidarDist);
    void setCellFromCliff(bool cliffLeft, bool cliffRight);
    void clearCell();
    void lidarUpdate(sensor_msgs::LaserScan set);
    void cellsUpdate();

private:
    bool checkCellIndex(int index);
    void setCell(double x, double y, E_CELL_OBSTACLE_TYPE type);

public:
    cell_obstacle* getcellsObstaclePointer();
};
