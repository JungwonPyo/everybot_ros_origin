/**
 * @file systemControl.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

class CSystemControl
{
private:
public:
	CSystemControl();
	~CSystemControl();

	void initTofSensor();
	void initImuSensor();
};
