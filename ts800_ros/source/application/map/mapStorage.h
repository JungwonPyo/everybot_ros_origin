/**
 * @file mapStorage.h
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
#include "gridmap.h"
#include <utility>
#include "coreData/serviceData/signals.h"

#include <yaml-cpp/yaml.h>
#include <fstream>

/**
 * @brief 로봇이 충전기 정보를 모를 때, 충전기 정보를 유추.
 * @brief 이 때, 로봇 좌표를 신호의 종류에 따라 저장... enum 1개당(신호) 좌표 1개 매칭
 */
enum class E_POTENTIAL_CHARGER 
{
    NONE,           // 초기 세팅
    SHORT_CENTER,
    CENTER,
    LEFT_CENTER,
    RIGHT_CENTER,
    LEFT_SIDE_CENTER,
    LEFT_SIDE,
    RIGHT_SIDE_CENTER,
    RIGHT_SIDE,
};

typedef struct
{
    tPose       charger;
    tPose       chargerApproach;
    bool        bKnownCharger;

} tDockingPoseData;

class CMapStorage
{
public:
    CMapStorage();
    ~CMapStorage();

private:

    std::string                 mapDataStorageDir;
    std::pair<tPose, bool> potentialChargerPoses[7];
    tDockingPoseData dockingPoseData;

public:

    //map
    double getUpdateMapSavedTime();
    bool updateGridMapToYAML(u8* dest, tGridmapInfo info, int index=0);
    bool updateTrajectoryToYAML(std::list<tPoint> traj, int index = 0);

    //charger
    bool copySavedStationPoseFromYAML(tPose* pPose);
    double getUpdateChargerSavedTime();
    bool updateChargerToYAML(tPose tmpPose);
    bool isExistSavedChargerPose();

    void setChargerPose(tPose set);
    void resetChargerPose();
    tPose getChargerPose();
    void setChargerApproachPose(tPose set);
    tPose getChargerApproachPose();
    bool getIsKnownCharger();
    void setKnownCharger(bool set);

    //potentialCharger
    bool copyPotentialChargerPoses(int i, tPose* pPose);
    bool updatePotentialCharger(tPose potentialPose, tSignalData signalData);
    bool isExistPotentialChargerPoses();
    void clearPotentialCharger();

private:
    bool checkParticularChargerSignal(E_POTENTIAL_CHARGER sigPose, tPose potentialPose, tSignalData signalData);
};