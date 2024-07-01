/**
 * @file cleanArea.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <algorithm>
#include "coordinate.h"
#include "opencv2/opencv.hpp"

enum class E_CLEAN_STATE
{
    UNCLEAN = 0,    // 미 청소인 상태
    RUNNING,        // 청소 중인 상태
    CLEANED,       // 청소 완료 상태
    NOTVALID,       // 휴효하지 않은 상태
};

/**
 * @brief 청소 면적에 대한 클래스
 * cleanPlan --> cleanRomm --> cleanArea
 */
class CCleanArea
{
public:
    CCleanArea();
    CCleanArea(int _id, E_CLEAN_STATE _state);
    ~CCleanArea();
    
    // setting
    void setId(int set);
    void setRobotBeginPose(tPose set);
    void setCleanState(E_CLEAN_STATE set);
    void addPolygon(tPoint add);
    void setPolygon(std::list<tPoint> &set);
    void setContour(std::vector<cv::Point> &set);
 
    // getting 
    int getPolygonSize();
    int getId();
    std::list<tPoint> getPolygon();
    std::vector<cv::Point> getContour();
    tPose getRobotBeginPose();
    E_CLEAN_STATE getCleanState();
    double getLeftYcoord();
    double getrightYcoord();
    tPoint getAreaUpEndPoint();
    tPoint getAreaDownEndPoint();
    tPoint getAreaLeftEndPoint();
    tPoint getAreaRightEndPoint();
    tPoint getAreaCornerPoint(int idx);
   

    // clear
    void clearPolygon();
private:
    int id;
    E_CLEAN_STATE cleanState;   // area의 청소 상태
    bool bCleanEnable;          // 청소를 할 수 있는 area인지
    
    //TODO : getSectionCornerPoint() 과 같은 기능 만드어야 함.
    tPose robotBeginPos;       //area 를 청소하기 위한 로봇 시작위치 

    int offsetDeg;  // 영역 내의 라인청소를 시작하기 전, 각도 틀어짐 보정 값 (단위 deg)
    std::list<tPoint> polygon; // 1개의 area 구성을 위한 월드 좌표 정보.
    std::vector<cv::Point> contour; //1개의 area 구성을 위한 cvimg 좌표 정보
};