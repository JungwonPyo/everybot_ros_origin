

#include "coordinate.h"

class CIoLinearization
{
private:
	double robotTheta;	//로봇 해딩, 단위 rad
	double xd, yd; 		//로봇 목표 위치, 단위 m
	double xd_p, yd_p;	//이전 위치, 단위 m
    

public:
    CIoLinearization();
    ~CIoLinearization();
	void setRobotTheta(double set);
	void proc();
	void clear();
};