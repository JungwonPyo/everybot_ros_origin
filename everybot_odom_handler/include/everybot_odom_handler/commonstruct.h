#include <iostream>
#include <math.h>

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