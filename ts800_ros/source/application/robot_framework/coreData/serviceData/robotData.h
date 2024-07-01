#pragma once
#include "commonStruct.h"


typedef struct _tCleanHistory
{
    _tCleanHistory() {}
    _tCleanHistory(std::string _cleanStartTime, std::string _exitReason, int _cleanedSize, int _cleanTime) : cleanStartTime(_cleanStartTime), cleanedSize(_cleanedSize), cleanTime(_cleanTime){}

    tPose robotPose;
    std::list<tPoint> traj;
    tPoint cradlePose;
    std::string cleanStartTime;
    std::string exitReason;
    double cleanedSize;
    int cleanTime;
    std::string areaInfo;
    
}tCleanHistory;
typedef struct _tSaveMap
{
    _tSaveMap() {}

    tPose robotPose;
    std::list<tPoint> traj;
    std::string uniqueKey;
    std::string mapName;
    int order;
    std::string areaInfo;
    
}tSaveMap;
typedef struct _tRobotData
{
  _tRobotData() : country(1) {}

  short cleanMode;
  short language;
  short country;
  
  u8 *map;
  
  std::list<tAreaInfo> areas;
  
  tPose robotPose;
  tCleanHistory cleanHistory;
  tSaveMap saveMap;

}tRobotData;


class CRobotData
{
private:
    tRobotData robotData;
public:
    CRobotData();
    ~CRobotData();

    void setCleanMode();
    void setLanguage();
    void setCountry();

    tRobotData getRobotData();

    /* clean history */
    void setCleanHistoryAreaInfo(std::string areaInfo);
    void setCleanHistoryTraj(std::list<tPoint> traj);
    void setCleanHistoryRobotPose(tPose robotPose);
    void setCleanHistoryCradlePose(tPoint cradlePose);
    void setCleanHistoryStartTime(std::string cleanStartTime);
    void setCleanHistoryExitReason(std::string exitReason);
    void setCleanHistoryCleanedSize(int cleanedSize);
    void setCleanHistoryCleanTime(int cleanTime);
    tCleanHistory getCleanHistory();

    /* save map */
    void setSaveMapRobotPose(tPose robotPose);
    void setSaveMaptraj(std::list<tPoint> traj);
    void setSaveMapUniqueKey(std::string key);
    void setSaveMapName(std::string mapName);
    void setSaveMapOrder(int order);
    void setSaveMapAreaInfo(std::string areaInfo);
    tSaveMap getSaveMap();

};