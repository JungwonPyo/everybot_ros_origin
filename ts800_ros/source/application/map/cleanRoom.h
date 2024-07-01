/**
 * @file cleanRoom.h
 * @author jhnoh
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "cleanArea.h"
#include "coordinate.h"

/**
 * @brief 청소 방에 대한 클래스
 * cleanPlan --> cleanRoom --> cleanArea
 */
class CCleanRoom
{
public:
    CCleanRoom();
    ~CCleanRoom();

    // getting
    int getAreaSize();
    int getId();
    E_CLEAN_STATE getCleanState();
    std::list<int> getPlanList();
    std::list<CCleanArea> getAreas();
    std::list<tPoint> getPolygons(int targetAreaId);
    int getCleanPriority();

    // setting
    void setAreaCleanState(int areaId, E_CLEAN_STATE set);
    void setId(int set);
    void setPlan(int areaId);
    void setCleanEnable(bool bSet);
    void setCleanState(E_CLEAN_STATE set);
    void setCleanPriority(int set);
    

    // clearing
    void clearAreaPlan();
    void clearAreas();

    // other
    void addArea(CCleanArea area);
    bool addPolygon(int targetAreaId, tPoint polygon);
    bool setPolygon(int targetAreaId, std::list<tPoint> &set);
    bool setContour(int targetAreaId, std::vector<cv::Point> &set);

    
private:
    std::list<CCleanArea> areas;    // area 집합. 절대 public 변환 금지.!!!!! 주소 반환 함수 만들기 금지!!!!
    std::list<int>        areaPlan;
    int id;            // room id  : 0 부터 사용. -1는 오류
    unsigned int cleanPriority;  // 청소 우선순위 점수 낮을수록 우선순위가 높은거다.
    E_CLEAN_STATE cleanState;   // 방의 청소 상태
    bool bCleanEnable;          // 청소를 할 수 있는 방인지    
    double cleanedAreaSize;         //청소 완료 한 area 개수

};
