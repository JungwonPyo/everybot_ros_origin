/**
 * @file coordinate.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "ebtypedef.h"
#include <iostream>
#include <vector>
#include <list>
#include <unordered_map>
#include "geometry_msgs/Vector3Stamped.h"
#include "define.h"


typedef struct _tPointdouble
{
    _tPointdouble() : x{0.0}, y{0.0} {}
    _tPointdouble(double _x, double _y) : x{_x}, y{_y} {}
    double x;   //단위 : m
    double y;   //단위 : m

    // /// Operator of points summation by element-wise addition.
    // _tPoint operator+(const _tPoint &p) const {return _tPoint(x + p.x, y + p.y);}

    // /// Returns additive inverse of the point.
    // _tPoint operator-() const {return _tPoint(-x, -y);}

    // // Equality operator
    // bool operator==(const _tPoint &p) const { return (x == p.x) && (y == p.y); }

}tPointdouble;

/**
 * @brief (x, y) 에 대한 구조체 정의 
 * ex : 각도 정보가 필요 없이 실수의 좌표가 필요한경우
 */
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


/**
 * @brief x, y, angle 에 대한 위치 구조체
 * ex : 로봇의 위치, 타겟 위치
 */
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


/**
 * @brief 영역의 끝단.
 * 
 */
struct tExtremes {
    tPoint leftmost;
    tPoint rightmost;
    tPoint topmost;
    tPoint bottommost;
};

/**
 * @brief 선속도와 각속도에 대한 구조체
 * @param v 선속도 (단위 m/s)
 * @param w 각속도 (단위 rad/s)
 */
struct tTwist
{
    tTwist(double _v=0.0, double _w=0.0) : v{_v}, w{_w} {}

    // Equality operator
    bool operator==(const tTwist &ref) const { return (v == ref.v) && (w == ref.w); }

    double v;  // 선속도 (단위 m/s)
    double w; // 각속도 (단위 rad/s)
};

struct tPwmSpeed
{
    tPwmSpeed(int _left= 0, int _right = 0, int _back = 0) : left{_left}, right{_right}, back{_back} {}

    // Equality operator
    bool operator==(const tPwmSpeed &ref) const { return (left == ref.left) && (right == ref.right) && (back == ref.back); }

    int left;
    int right;
    int back; 
};

/**
 * @brief grid map 이나 이미지 같은곳에서 사용하는 좌표계
 * 
 */
typedef struct
{
    int x;
    int y;
}tCellPoint;

/**
 * @brief 선속도와 각속도에 대한 구조체
 * @param linear 선속도 (단위 m/s)
 * @param angular 각속도 (단위 rad/s)
 */
struct tDynamicSpeed
{
    tDynamicSpeed() : linear{0.0}, angular{0.0} {}
    tDynamicSpeed(double _linear, double _angular) : linear{_linear}, angular{_angular} {}

    // Equality operator
    bool operator==(const tDynamicSpeed &ref) const { return (linear == ref.linear) && (angular == ref.angular); }

    double linear; // 선속도 (단위 m/s)
    double angular; // 각속도 (단위 rad/s)
};

/**
 * @brief CleanService 에서 area 모서리에 대한 열거형
 * 
 * 2:TL___ 3:TR
 * |       |
 * |       |
 * | _____ |
 * 0:BL    1:BR
 * 
 */
enum class E_AREA_CORNERTYPE
{
    NONE,           // 아직 미정인 상태
    BOTTOM_LEFT,
    BOTTOM_RIGHT,
    TOP_LEFT,
    TOP_RIGHT,
};

static std::string enumToString(E_AREA_CORNERTYPE value) {
    static const std::unordered_map<E_AREA_CORNERTYPE, std::string> enumToStringMap = {
        { E_AREA_CORNERTYPE::NONE, "NONE," },
        { E_AREA_CORNERTYPE::BOTTOM_LEFT, "BOTTOM_LEFT," },
        { E_AREA_CORNERTYPE::BOTTOM_RIGHT, "BOTTOM_RIGHT," },
        { E_AREA_CORNERTYPE::TOP_LEFT, "TOP_LEFT," },
        { E_AREA_CORNERTYPE::TOP_RIGHT, "TOP_RIGHT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_AREA_LINETYPE //섹션 벽면주행 LINE
{
	//hjkim221215 - 가상벽면 라인 설정 시 코너 영역 라인 추가.
    NONE,
    BOTTOM,
    BOTTOM_RIGHT,
    BOTTOM_LEFT,
    TOP,
    TOP_RIGHT,
    TOP_LEFT,
    LEFT,
    RIGHT,
};

enum class E_AREA_LINESTATE //섹션 벽면주행 LINE
{
    NONE,
    OFF, //섹션 안쪽 라인 벗어남 - 벽면 또는 장애물에 의해 섹션안쪽으로 이동
    IN,  //라인안에서 이동 중
    OUT, //섹션 바깥 라인으로 벗어남 - 장애물이 없을 때, 슬립 현상에 의해 라인 바깥으로 벗어나는 경우
    CORNER,
};

typedef union 
{
    u8 value;
    struct
    {
        u8 bottom   : 1;
        u8 top      : 1;
        u8 right    : 1;
        u8 left     : 1;

        u8 reserved : 4;
    }b;

}tSECTION_CORNERS;

tPoint getAreaCornerPoint(tPoint areaCenter, double width, double height, E_AREA_CORNERTYPE type);

std::list<tPoint> generatePointsWithBresenham(tPoint startPoint, tPoint endPoint, double resolution);