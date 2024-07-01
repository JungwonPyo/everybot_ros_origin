#include "StdAfx.h"
#include <math.h>
#include "mm.h"
#include "io_linearization.h"



const double b = 0.2;   //단위 m, 추종할 거리. 로봇크기의 1/2 권장.
const double k1 = 2.1;  //궤적 추종 변수
const double k2 = 2.1;  //궤적 추종 변수
CIoLinearization::CIoLinearization() {
    robotTheta = 0;
    xd=0;
    yd=0;
}

CIoLinearization::~CIoLinearization() {}

void CIoLinearization::clear()
{
    robotTheta = 0;
    xd=0;
    yd=0;
}

void CIoLinearization::ioLinearization(double &v, double &w, double xd, double yd, double xdd, double ydd)
{
	double y1 = x + b*cos(robotTheta);
	double y2 = y + b*sin(robotTheta);

	double y1d = xd;
	double y2d = yd;

	double y1dd = xdd;
	double y2dd = ydd;

	

	double u1 = y1dd + k1*(y1d - y1);
	double u2 = y2dd + k2*(y2d - y2);

	v =  u1*cos(robotTheta)   + u2*sin(robotTheta);
	w = -u1*sin(robotTheta)/b + u2*cos(robotTheta)/b;
}

void CIoLinearization::setRobotTheta(double set)
{
    robotTheta = set;
}


/**
 * @brief 목표 좌표 세팅 
 * 
 * @param x 단위 : m
 * @param y 단위 : m
 */
void CIoLinearization::setDesiredPos(double x, double y)
{
    xd = x;
    yd = y;
}

/**
 * @brief 일정 시간마다 호출 
 * 
 * @param dt : 단위 hz, ex) 50msec -> 0.02hz
 */
void CIoLinearization::proc(double dt)
{

	

	double xdd = (xd - xd_p)/dt;
	double ydd = (yd - yd_p)/dt;

	xd_p = xd; 
	yd_p = yd; 

	t += dt;
	if (t < 3*dt) return;

	double v, w;
	io_linearization (v, w, xd, yd, xdd, ydd);

	dead_reckoning (v, w, dt);

	DrawRobot ();
}

