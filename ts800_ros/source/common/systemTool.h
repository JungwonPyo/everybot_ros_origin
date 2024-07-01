/**
 * @file systemTool.h
 * @author hhryu
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "ebtime.h"

#define  SYSTEM_TOOL CSystemTool::getInstance() 


class CSystemTool {
private:
    sec_t init_sec;
    CSystemTool();
    CSystemTool(const CSystemTool& ref);
    CSystemTool& operator=(const CSystemTool& ref) {init();}
    ~CSystemTool();

    void init();
public:
    static CSystemTool& getInstance();
    int getSystemTick();
    double getSystemTime();
    bool checkTime(double &startTime, double checkTime);
};
