/**
 * @file explorer.h
 * @author 담당자 미정
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <cstdint>
#include <utility>
#include <future>

// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "coreData/serviceData.h"
#include "commonStruct.h"

#define DEFAULT_MOP_DRYING_TIMEOUT 60

enum class DRYMOP_STATE
{
    NONE,
    START,
    CHECK_FAN_ACK,
    RUN,
    CHANGE_FAN_LEVEL,
    COMPLETE_FAN_OFF,
    COMPLETE,
    CANCLE,
};

static std::string enumToString(DRYMOP_STATE value) {
    static const std::unordered_map<DRYMOP_STATE, std::string> enumToStringMap = {
        { DRYMOP_STATE::NONE, "NONE," },
        { DRYMOP_STATE::START, "START," },
        { DRYMOP_STATE::CHECK_FAN_ACK, "CHECK_FAN_ACK," },
        { DRYMOP_STATE::RUN, "RUN," },
        { DRYMOP_STATE::CHANGE_FAN_LEVEL, "CHANGE_FAN_LEVEL," },
        { DRYMOP_STATE::COMPLETE_FAN_OFF, "COMPLETE_FAN_OFF," },
        { DRYMOP_STATE::COMPLETE, "COMPLETE," },
        { DRYMOP_STATE::CANCLE, "CANCLE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class FAN_COMMAND
{
    ON    = 0,
    CANCLE_OFF   = 1,
    COMPLETE_OFF = 2,
};

static std::string enumToString(FAN_COMMAND value) {
    static const std::unordered_map<FAN_COMMAND, std::string> enumToStringMap = {
        { FAN_COMMAND::ON, "ON," },
        { FAN_COMMAND::CANCLE_OFF, "CANCLE_OFF," },
        { FAN_COMMAND::COMPLETE_OFF, "COMPLETE_OFF," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CTaskDryMop
{
private: /* not use! */
    CTaskDryMop(const CTaskDryMop& other) = default; // Copy constructor
    CTaskDryMop& operator=(const CTaskDryMop& other) = default; // Assignment operator

public:
    CTaskDryMop();
    ~CTaskDryMop();
    
    void dryStart();
    // void dryStart(tDryMopOption option, bool bRobot);
    void dryCancle();
    bool taskRun();
    void updateDryOption();
    DRYMOP_STATE getDryMopState();
    
private:

    DRYMOP_STATE state;
    FAN_COMMAND cmd;
    tDryMopOption option;

    bool bRunning;
    
    u8 cmdCnt;
    
    double cmdStartTime;
    double startTime;
    double debugStartTime;

    void setDryMopState(DRYMOP_STATE set);
    void setOption(tDryMopOption set);
    DRYMOP_STATE startDryMop();
    DRYMOP_STATE checkFanAck();
    DRYMOP_STATE runDryMop();
    DRYMOP_STATE completeFanOff();
    DRYMOP_STATE changeFanLevel();
    DRYMOP_STATE completeDryMop();
    DRYMOP_STATE stopDryMop();
    tDryMopOption getDryOption();
};