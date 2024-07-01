/**
 * @file cleanplan.h
 * @author jhnoh
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coordinate.h"
#include "cleanRoom.h"
#include "gridmap.h"
#include <list>

struct tCellRect  // 셀 좌표에서 사각형에 대한 정보 구조체
{ 
    int x; 
    int y; 
    int width; 
    int height; 
}; 
 
struct tPlanInfo // 현재 및 다음 청소에 대한 ID 정보 구조체
{
    tPlanInfo() : currentRoomId(-1), currentAreaId(-1), nextAreaId(-1), nextRoomId(-1) {}

    int currentRoomId;
    int currentAreaId;
    int nextAreaId;
    int nextRoomId;
};
/**
 * @brief 청소 계획(방들에 대한...)에 대한 클래스
 * cleanPlan --> cleanRoom --> cleanArea
 */
class CCleanPlan
{  
public:
    CCleanPlan();
    ~CCleanPlan();

    // other
    void makeCustomArea(tPose robotPose);
    bool makeAreaByMap();
    bool findUncleanArea(tPose robotPose, tPoint& cleanPoint );
    void makeArea(unsigned char *src, tGridmapInfo info);    
    void makePlan(tPose robotPose);

    // checking
    bool checkInArea(tPoint point);
    bool checkAreaCoverage();
   
    // setting
    void setCurrentAreaCleanComplete(bool bSet);
    void setNoGoZoneDocking(tPoint org, double bound);
    void enableNogoZoneDocking();
    void disableNogoZoneDocking();

    // getting 
    E_CLEAN_STATE getAreaCleanState();
    E_CLEAN_STATE getAreaCleanState(int roomId, int areaId);
    std::list<tPoint> getCurrentAreaPolygons();
    std::list<tPoint> getPolygon(int roomId, int areaId);
    std::vector<cv::Point> getCurrentAreaContour();
    std::vector<cv::Point> getContour(int roomId, int areaId);
    std::list<CCleanRoom> getRooms();    
    std::list<CCleanRoom> getAllRooms();
    tPlanInfo getIdInfo();
    int getUnCleanRoomSize();    
    int getCleanedRoomSize();
    double getCleanedSize();

    tPoint getAreaLeftEndPoint();
    tPoint getAreaLeftEndPoint(int roomId, int areaId);
    tPoint getAreaRightEndPoint();
    tPoint getAreaRightEndPoint(int roomId, int areaId);
    tPoint getAreaUpEndPoint();
    tPoint getAreaUpEndPoint(int roomId, int areaId);
    tPoint getAreaDownEndPoint();
    tPoint getAreaDownEndPoint(int roomId, int areaId);
    
    tPoint getNearConerPoint(tPose robotPose, double margin);
    tPoint getNearConerPoint(tPose robotPose, int roomId, int areaId, double margin);
    tPoint getNearConerPoint(tPose robotPose, int roomId, int areaId, double margin, const std::list<int> &checkPoints);
    tPoint getNearConerPoint(tPose robotPose, double margin, const std::list<int> &checkPoints);
    tPoint getAreaCornerPoint(int idx);    
    tPoint getAreaCenterPoint();
    tPoint getAreaCenterPoint(int roomId, int areaId);
    int getCurrentRoomId();
    std::list<tPoint> getNoGoZoneDocking();
    tPoint getNoGoZoneDockingCoord();

    bool isSetupNogoZoneDocking();

    // clearing
    void clearRooms();    

    // other
    void __debug__cleanState__();
    
private:

    // setting
    CCleanRoom& setRoom(int roomId);
    std::list<int>    setAreaCleanPlan(tPose robotPose, int roomId);
 
    // getting 
    std::list<tPoint> getAreaMorphology(double margin);
    std::list<tPoint> getAreaMorphology(int roomId, int areaId, double margin);    
    int    getNumOfWayToGo(int roomId, int areaId);
    int    getNearestAreaId(tPoint point, int roomId);
    
    // other
    CCleanArea findArea(int roomId, int areaId);
    CCleanRoom findRoom(int roomId);
    void updateCurrentId();
    double calcRoomAreaSize(); //방크기 구하기
    void sortPriorityByRobotPose(tPose robotPose);
    
    bool bNoGoZoneSetup;
    std::list<tPoint> noGoZoneDocking;
    tPoint noGoZoneDockingCoord;

    tPlanInfo planInfo;             //계획 정보 모음.
    int cleanPlanId;                // Clean Plan 고유 ID
    std::list<CCleanRoom> rooms;    // 청소할 방들 정보 list, 절대 public 변환 금지 !!!!! 주소 반환 함수 만들기 금지 !!!!    
    double cleanedSize;
};

#if 0 // 일단 사용 안함. 컨셉에 따라 달라질수 있음.

    /**
     * @brief 한 개의 방에 여러개의 다각형 컨셉일 때 사용.
     */
    struct tSectionListInfo
    {
        int mapHeadId;      // TODO: 어떤 맵에 속하는지? (부모를 표시하는 듯한 map)
        int cleanPlanId;    // TODO: 왜 사용하는지 모름.

        int listSize;   // section list 사이즈
        std::list<tPolygonData> sectionList;
    };

    #if 1 // 3i 에서 사용 하는 맵 구조체
    /**
     * @brief 맵 Info 구조체에 맵에 대한 occupy 정보는 없음.
     * 단지 맵 이름, Head ID, Task ID 정보만 있음.
     * ( 의미 불명)
     */
    struct tMapInfo
    {
        int mapHeadId;  // TODO: 맵이 여러개인지? 맵에 ID가 필요한가?   
        // std::string sMapName;   // App에서 사용할 맵 이름.
        // char cMapNameSize;          // App 에서 사용할 맵 이름 사이즈.
        long taskId;    // TODO: 
    };

    /**
     * @brief 맵 픽셀에 대한 정보가 들어있는 구조체
     * 
     */
    struct tMapHeadInfo
    {
        int mapHeadId;  // 맵 픽셀 정보에 대한 고유ID

        char* pMap;     // 맵 픽셀 정보
        int mapType;    // TODO: 왜 사용하는지 모름.
        int mapValid;   // 맵 유효성을 boolean이 아닌 int를 사용함. 좀 더 다양한 상태를 체크할 것으로 보임.

        float maxX;     // 맵 X 값의 최대값        
        float maxY;     // 맵 Y 값의 최대값
        float minX;     // 맵 X 값의 최소값
        float minY;     // 맵 Y 갑의 최소값

        float resolution;   // 맵 해상도
        int sizeX;      // X축 픽셀 수
        int sizeY;      // Y축 픽셀 수
    };
    #endif
#endif