#pragma once

class CRmclPoint 
{
private:
    double x_, y_;

public:
    CRmclPoint():
        x_(0.0), y_(0.0) {};

    CRmclPoint(double x, double y):
        x_(x), y_(y) {};

    ~CRmclPoint() {};

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setPoint(double x, double y) { x_ = x, y_ = y; }
    inline void setPoint(CRmclPoint p) { x_ = p.x_, y_ = p.y_; }

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline CRmclPoint getPoint(void) { return CRmclPoint(x_, y_); }

}; // class Point