#include "coreData/serviceData/robotData.h"
#include "coreData/serviceData.h"
#include "eblog.h"
#include "userInterface.h"

CRobotData::CRobotData(){}
CRobotData::~CRobotData(){}

void CRobotData::setCleanMode()
{
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
    robotData.cleanMode = ServiceData.rsBridge.getCleanMode();
    ServiceData.awsData.setSendCleanMode(ServiceData.rsBridge.getCleanMode()); 
}
void CRobotData::setLanguage()
{
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
    robotData.language = ServiceData.rsBridge.getLanguage();
    ServiceData.awsData.setSendCleanMode(ServiceData.rsBridge.getLanguage());
}
void CRobotData::setCountry()
{
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
    robotData.country = ServiceData.rsBridge.getCountry();
    ServiceData.awsData.setSendCleanMode(ServiceData.rsBridge.getCountry());
}

tRobotData CRobotData::getRobotData()
{
    return robotData;
}
void CRobotData::setCleanHistoryAreaInfo(std::string areaInfo)
{
    robotData.cleanHistory.areaInfo = areaInfo;
    ServiceData.awsData.setSendCleanAreaInfo(areaInfo);
}
void CRobotData::setCleanHistoryTraj(std::list<tPoint> traj)
{
    robotData.cleanHistory.traj = traj;
    ServiceData.awsData.setSendCleanTraj(traj);
}
void CRobotData::setCleanHistoryRobotPose(tPose robotPose)
{
    robotData.cleanHistory.robotPose = robotPose;
    ServiceData.awsData.setSendCleanRobotPose(robotPose);
}
void CRobotData::setCleanHistoryCradlePose(tPoint cradlePose)
{
    robotData.cleanHistory.cradlePose = cradlePose;
    ServiceData.awsData.setSendCleanCradlePose(cradlePose);
}
void CRobotData::setCleanHistoryStartTime(std::string cleanStartTime)
{
    robotData.cleanHistory.cleanStartTime = cleanStartTime;
    ServiceData.awsData.setSendCleanStartTime(cleanStartTime);
}
void CRobotData::setCleanHistoryExitReason(std::string exitReason)
{
    robotData.cleanHistory.exitReason = exitReason;
    ServiceData.awsData.setSendCleanExitReason(exitReason);
}
void CRobotData::setCleanHistoryCleanedSize(int cleanedSize)
{
    robotData.cleanHistory.cleanedSize = cleanedSize;
    ServiceData.awsData.setSendCleanedSize(cleanedSize);
}
void CRobotData::setCleanHistoryCleanTime(int cleanTime)
{
    robotData.cleanHistory.cleanTime = cleanTime;
    ServiceData.awsData.setSendCleanTime(cleanTime);
}
tCleanHistory CRobotData::getCleanHistory(){
    return robotData.cleanHistory;
}

/* SAVE MAP */
void CRobotData::setSaveMapRobotPose(tPose robotPose){
    robotData.saveMap.robotPose = robotPose;
    ServiceData.awsData.setSendSaveMapRobotPose(robotPose);
}
void CRobotData::setSaveMaptraj(std::list<tPoint> traj){
    robotData.saveMap.traj = traj;
    ServiceData.awsData.setSendSaveMapTraj(traj);
}
void CRobotData::setSaveMapUniqueKey(std::string key){
    robotData.saveMap.uniqueKey = key;
    ServiceData.awsData.setSendSaveMapUniqueKey(key);
}
void CRobotData::setSaveMapName(std::string mapName){
    robotData.saveMap.mapName = mapName;
    ServiceData.awsData.setSendSaveMapName(mapName);
}
void CRobotData::setSaveMapOrder(int order){
    robotData.saveMap.order = order;
    ServiceData.awsData.setSendSaveMapOrder(order);
}
void CRobotData::setSaveMapAreaInfo(std::string areaInfo){
    robotData.saveMap.areaInfo = areaInfo;
    ServiceData.awsData.setSendSaveMapAreaInfo(areaInfo);
}
tSaveMap CRobotData::getSaveMap(){
    return robotData.saveMap;
}