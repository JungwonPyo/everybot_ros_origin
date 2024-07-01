/**
 * @file utils.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "commonStruct.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "eblog.h"
#include "clipper.hpp"

#include <opencv2/opencv.hpp>
#include <utility>

#include <iostream>
#include <fstream>
#include <list>
#include <yaml-cpp/yaml.h>

enum class E_DIRECTION
{
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

static std::string enumToString(E_DIRECTION value) {
    static const std::unordered_map<E_DIRECTION, std::string> enumToStringMap = {
        { E_DIRECTION::NONE, "NONE," },
        { E_DIRECTION::UP, "UP," },
        { E_DIRECTION::DOWN, "DOWN," },
        { E_DIRECTION::LEFT, "LEFT," },
        { E_DIRECTION::RIGHT, "RIGHT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

namespace utils
{
    bool getLocalization(
        tf::TransformListener *tf_listener,
        std::string frame_base,
        std::string frame_target,
        geometry_msgs::PoseStamped *pose);
    bool isUpdatePose(tPose temp, tPose currnet);
    template <typename T>
    bool isMaxValue(const T &variable);

namespace fileIO
{
    bool saveRobotOption(const std::string& filename, const tRobotOption& option);
    bool loadRobotOption(const std::string& filename, tRobotOption& option);

    bool writeRobotOption(const std::string& filename, const tRobotOption& option);
    bool readRobotOption(const std::string& filename, tRobotOption& option);
    
    YAML::Emitter& operator<<(YAML::Emitter& out, const tRobotOption& option);
    YAML::Emitter& operator<<(YAML::Emitter& out, const tCleanSchedule& schedule);
    /* 구조체 생길때마다 추가 */
    // YAML::Emitter& operator<<(YAML::Emitter& out, const t추가& 추가);
    
    void operator>>(const YAML::Node& node, tRobotOption& option);    
    void operator>>(const YAML::Node& node, tCleanSchedule& schedule);
    /* 구조체 생길때마다 추가 */
    // void operator>>(const YAML::Node& node, t추가& 추가);
    
    bool compareRobotOption(const tRobotOption& o1, const tRobotOption& o2); 
    bool compareCleanSchedule(const tCleanSchedule& s1, const tCleanSchedule& s2); 
    /* 구조체 생길때마다 추가 */
    // bool compare추가(const t추가& s1, const t추가& s2); 

}
namespace iprocess
{        
    void roiRectangle(char* pSrc, char* pDest, int left, int top, int right, int bottom);
    void getRoiRectangle(char* pSrc, int imgWidth, char* pDest, int left, int top, int right, int bottom);
    void getRoiRectangle(cell* cmpSrc, int roiInitIndex, cell* pDest, int left, int top, int right, int bottom);
    void debug_imgPrint(unsigned char *src, int height, int width, int cropSize);
}


namespace coordination
{
    int convertCoordi2Index(double coordX, double coordY, double resolution, int cellWidth, int cellHeight);
    std::pair<int, int> convertCoordiToCellPoint(double coordX, double coordY, double resolution, int cellWidth, int cellHeight);
    bool convertCoordi2Index(double coordX, double coordY, int *pCellX, int *pCellY, double resolution, int cellWidth, int cellHeight);
    int convertSlamIndex2CellIndex(int slamIndex,
        int slamWidth, int slamHeight, double slamResolution,double slamOriX, double slamOriY,
        int cellWidth, int cellHeight, double cellResolution);
    std::pair<int, int> convertSlamIndex2CellPoint(int slamIndex, int slamWidth, int slamHeight, double slamResolution,double slamOriX, double slamOriY, int cellWidth, int cellHeight, double cellResolution);
    void convertCoord2CellCoord(double coordX, double coordY, double resolution, int cellWidth, int cellHeight, int *cellx, int *celly);
    void convertCoord2CellCoord(double coordX, double coordY, int *cellx, int *celly);
}

namespace math
{
    double distanceTwoPoint(tPoint a, tPoint b);
    double distanceTwoPoint(tPoint a, tPose b);
    double distanceTwoPoint(tPose a, tPoint b);
    double distanceTwoPoint(tPose a, tPose b);
    double distanceTwoPoint(tSysPose a, tSysPose b);
    double getRadianAngle(E_DIRECTION dir);
    double getDegAngle(E_DIRECTION dir);
    double getRadianAngle(E_DIRECTION dir, double upAngle);
    double getRadianAngle(tPoint target, tPoint robot);
    double getRadianAngle(tPoint target, tPose robot);
    double getRadianAngle(double x, double y);
    double rad2rad(double radian);
    double deg2deg(double degree);
    double deg2rad(double degree);
    double rad2deg(double radian);
    bool getTurnDirection(double currentDegree, double targetDegree);
    double getTurnAngle(tPoint target, tPose robot);
    double getTurnAngle(tPoint target, tPose robot, double calibAngle);
    double getTurnAngle(double targetDegAngle, double robotDegAngle);
    double getTurnRadAngle(double targetRadAngle, double robotRadAngle);
    double getTurnRadAngle(tPoint targetPoint, tPose robotPose);
    double getTurnRadAngle(tPose targetPose, tPose robotPose);
    double getTurnAnglePlusDirection(tPose robot, tPoint target);
    double getAverageSlope(std::list<tPoint> points);
    double calculateMinimumRadianAngle(double angle1, double angle2);
    double calculateMinimumDegreeAngle(double angle1, double angle2);

    std::string getCurrentTimeString(int country, int option );
    bool cmpTime(const std::string& start, const std::string& end, int country);

    double interpolateLinearly(tPoint2D range, double x);

    double crossProduct(tPoint a, tPoint b, tPoint c);
    double pointRelativeToLine(tPoint point, tPoint start, tPoint end);
    std::list<double> calculateLineEquation(tPoint start, tPoint end);
    std::vector<double> calculateLineEquationVector(tPoint start, tPoint end);
    double calculateDistanceFromLine(tPoint point, std::list<double> coeff);
    s32 clamp(s32 value, s32 minValue, s32 maxValue);
    double clamp(double value, double minValue, double maxValue);
    std::pair<double, double> findLinearEquation(tPoint p1, tPoint p2);
    bool isWithinRelativeChange(double value1, double value2, double relativeChangeThreshold);

    bool isRobotCrossLine(tPose robotPose, tPoint startPoint, tPoint targetPoint);

    double getCrossProduct(const tPoint& pointA, const tPoint& pointB, const tPoint& pointC);
    bool findClosedLoopDirection(const std::list<tPoint>& curvedPath);
}

namespace area
{
    void resizeArea(std::list<tPoint>& orgArea,std::list<tPoint>& scaleArea,double scaleFactor);
    tPoint findCentroid(std::list<tPoint>& area);
    tPoint findNearPoint(tPose robotPose, std::list<tPoint>& area);
    tPoint findRightPoint(std::list<tPoint>& area);
    tExtremes findExtremes(std::list<tPoint>& points);
    bool isInside(const tPoint& point, std::list<tPoint>& area);
    bool calculateXAxisVerticalIntersection(tPoint point1, tPoint point2, double yValue, tPoint &outputIntersection);
    bool calculateYAxisVerticalIntersection(tPoint point1, tPoint point2, double xValue, tPoint &outputIntersection);
    std::list<tPoint> calculateXAxisLineIntersection(double yValue, const std::list<tPoint> &polygon);
    std::list<tPoint> calculateYAxisLineIntersection(double xValue, const std::list<tPoint> &polygon);
    std::list<tPoint> calculateXAxisLineStartPoints(tPoint minBound, tPoint maxBound, std::list<tPoint> polygon, double wallDistance = 0.3); // areaSegmentation 에 있는 함수. 추후 삭제예정
    std::list<tPoint> calculateYAxisLineStartPoints(tPoint minBound, tPoint maxBound, std::list<tPoint> polygon, double wallDistance = 0.3); // areaSegmentation 에 있는 함수. 추후 
}

namespace cleanmap
{
    tPoint getPointAwayFromWall(tPoint point, int awayCount, cell* cells);
    std::list<tPoint> optimizePoints(const std::list<tPoint>& origin, double resolution);
}

namespace os
{
    bool isProcessRunning(const std::string& processName);

    std::chrono::system_clock::time_point now();
    double msElapsedTime(std::chrono::system_clock::time_point start);
}

namespace path
{
    double perpendicularDistance(const tPoint& pt, const tPoint& lineStart, const tPoint& lineEnd);
    double perpendicularDistance(const cv::Point& pt, const cv::Point& lineStart, const cv::Point& lineEnd);
    void ramerDouglasPeucker(const std::list<tPoint>& pointList, double epsilon, std::list<tPoint>& out);
    void ramerDouglasPeucker(const std::list<cv::Point>& pointList, double epsilon, std::list<cv::Point>& out);
    bool isAboveLine(tPoint robot, tPoint startLine, tPoint endLine);
}
}

class utilThreadWait
{
private:
    std::chrono::time_point<std::chrono::steady_clock>  next;
    std::chrono::milliseconds  period;
    std::chrono::time_point<std::chrono::steady_clock>  now;
public:
    utilThreadWait(unsigned int millisec){
        next = std::chrono::steady_clock::now();
        period = std::chrono::milliseconds(millisec);
        now = std::chrono::steady_clock::now();
    }
    ~utilThreadWait(){}
    void sleepUntil()
    {
        next += period;
        now = std::chrono::steady_clock::now();
        // 현재 시간이 다음 실행 시간 이후인 경우, 즉시 다음 루프로 이동
        if (now < next) {
            // 다음 실행 시간까지 대기
            std::this_thread::sleep_until(next);
        }
        else if(now > next+period)  next = std::chrono::steady_clock::now(); // 주기가 밀린 경우, 다음 주기시간 초기화.
    }

};
