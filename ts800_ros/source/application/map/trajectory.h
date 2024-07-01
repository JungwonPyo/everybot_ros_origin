/**
 * @file trajectory.h
 * @author jhnoh
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <atomic>
#include <list>
#include "coordinate.h"
class CTrajectory 
{
private:	
	std::list<tPoint> trajectory;
    bool bIsUpdate;
	pthread_mutex_t mutex;
public:
	CTrajectory(); 
	~CTrajectory();

    bool isUpdate();
    void setUpdateState(bool set);
	std::list<tPoint> get(void);
	void set(std::list<tPoint> set);
	void clear(void);
};
