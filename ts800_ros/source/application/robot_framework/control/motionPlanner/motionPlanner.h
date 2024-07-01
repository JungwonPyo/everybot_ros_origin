/**
 * @file motionPlanner.h
 * @author jspark
 * @brief 세부 작업으로는 'ㄹ'패턴, 방이동, 가상벽 이동, 충전기 이동 등이 있습니다.
 * @version 0.1
 * @date 2023-09-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <pthread.h>
#include <thread>
#include "commonStruct.h"
#include "coordinate.h"
#include "coreData/serviceData.h"
#include "coreData/serviceData/localization.h"
#include "control/motionController.h"
#include "motionPlanner/wayPointManager.h"
#include "pathFinderInterface.h"

#define PATH_PLANNER CPathPlanner::getInstance().pPathFinder


class CPathPlanner
{
private: /* not use! */
    CPathPlanner();
	~CPathPlanner();
    CPathPlanner(const CPathPlanner& other) = default; // Copy constructor
    CPathPlanner& operator=(const CPathPlanner& other) = default; // Assignment operator
public:    
    static CPathPlanner& getInstance();    
    CPathFinderInterface *pPathFinder;
};

