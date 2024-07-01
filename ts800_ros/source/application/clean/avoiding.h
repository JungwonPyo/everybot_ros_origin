/**
 * @file avoiding.h
 * @author hjkim
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include "commonStruct.h"
#include "coreData/serviceData.h"
#include "motionPlanner/wayPointManager.h"


enum class E_AVOID_STATUS
{    
    CHECKER,    // 장애물 판단 및 즉시 멈춤
    CHECKKNOLL,   // 장애물 2차 판단
    BACK_MOVING,    // 후진하는 기능
    //MOTION_AVOID,      // 즉시 회피 (회피 동작 첫번째는 보장을 해야 한다.)
    MOTION_ESCAPE,     // 회피 탈출, 탈출 중에 장애물을 다시 확인하는 동작이 들어간다.
    COMPLETE,   // 회피 완료.
};
static std::string enumToString(E_AVOID_STATUS value) {
    static const std::unordered_map<E_AVOID_STATUS, std::string> enumToStringMap = {
        { E_AVOID_STATUS::CHECKER, "E_AVOID_STATUS::CHECKER," },
        { E_AVOID_STATUS::CHECKKNOLL, "E_AVOID_STATUS::CHECKKNOLL," },
        //{ E_AVOID_STATUS::MOTION_AVOID, "E_AVOID_STATUS::MOTION_AVOID," },
        { E_AVOID_STATUS::MOTION_ESCAPE, "E_AVOID_STATUS::MOTION_ESCAPE," },
        { E_AVOID_STATUS::COMPLETE, "E_AVOID_STATUS::COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}
enum class E_AVOID_TYPE
{
    NONE,
    IR,
    BUMPER,
    CLIFF,
    WHEELTRAP,
    KNOLL,
    LIDAR,
};
static std::string enumToString(E_AVOID_TYPE value) {
    static const std::unordered_map<E_AVOID_TYPE, std::string> enumToStringMap = {
        { E_AVOID_TYPE::NONE, "NONE," },
        { E_AVOID_TYPE::IR, "IR," },
        { E_AVOID_TYPE::BUMPER, "BUMPER," },
        { E_AVOID_TYPE::CLIFF, "CLIFF," },
        { E_AVOID_TYPE::WHEELTRAP, "WHEELTRAP," },
        { E_AVOID_TYPE::KNOLL, "KNOLL," },
        { E_AVOID_TYPE::LIDAR, "LIDAR," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_AVOID_KNOLL_STATUS
{
    READY,
    CLIMB,
    ACROSS,
    COMPLETE,
    FAIL,
};

enum class E_AVOID_KNOLL_STEP
{
    NONE,
    START_BALANCE,
    CHECK_BALANCE,
    START_TILTUP,
    CHECK_TILTUP,
    CHECK_TOFCALIB,
};

static std::string enumToString(E_AVOID_KNOLL_STEP value) {
    static const std::unordered_map<E_AVOID_KNOLL_STEP, std::string> enumToStringMap = {
        { E_AVOID_KNOLL_STEP::NONE, "NONE," },
        { E_AVOID_KNOLL_STEP::START_BALANCE, "START_BALANCE," },
        { E_AVOID_KNOLL_STEP::CHECK_BALANCE, "CHECK_BALANCE," },
        { E_AVOID_KNOLL_STEP::START_TILTUP, "START_TILTUP," },
        { E_AVOID_KNOLL_STEP::CHECK_TILTUP, "CHECK_TILTUP," },
        { E_AVOID_KNOLL_STEP::CHECK_TOFCALIB, "CHECK_TOFCALIB," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return std::string("Unknown : ");
    }
}


enum class E_ESCAPE_STEP
{
    NONE,
    SET_WAYPOINT,       //전략 결정, way Point 를 생성 한다.
    RUN_WAYPOINT,       //wayPoint 를 수행
    COMPLETE           //완료.
};
static std::string enumToString(E_ESCAPE_STEP value) {
    static const std::unordered_map<E_ESCAPE_STEP, std::string> enumToStringMap = {
        { E_ESCAPE_STEP::NONE, "NONE," },
        { E_ESCAPE_STEP::SET_WAYPOINT, "SET_WAYPOINT," },
        { E_ESCAPE_STEP::RUN_WAYPOINT, "RUN_WAYPOINT," },
        { E_ESCAPE_STEP::COMPLETE, "COMPLETE," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return std::string("Unknown : ");
    }
}

class CAvoiding
{
private:
    E_AVOID_STATUS avoidstatus;
    E_AVOID_TYPE   avoidtype;
    E_AVOID_TYPE   startAvoidtype;
    E_ESCAPE_STEP   escapeStep;
    
    E_AVOID_KNOLL_STATUS knollStatus;
    E_AVOID_KNOLL_STEP  knollStep;

    tPose          avoidPose;
    std::list<tPose> historyPose;

    tPose          poseTemp;
    double         avoidStartTime;

    RSF_OBSTACLE_MASK avoid_mask;
    bool isKnollAvoiding;    
    u8 knollTrapcount;
    u8 knollclearcount;

    CWayPoint wayPoint;
    CWayPointManager wayPointMng;

    bool tempCheckObstacle;
    tPose startPose;

    bool isTurnningAvoid;
    bool bAvodingEndFlag;
    double KnollHeading;

    E_AVOID_STATUS checker(tPose robotPose,  bool clean);
    E_AVOID_STATUS knollchecker(tPose robotPose);
    E_AVOID_STATUS backMoving(tPose robotPose);   
    E_AVOID_STATUS MotionEscape(tPose robotPose,  bool clean);    
    E_AVOID_STATUS complete(tPose robotPose);
    void escapeStepRunWayPoint(tPose robotPose,  bool clean);


    E_AVOID_KNOLL_STATUS readyAcrossDoorSill(tPose robotPose);
    E_AVOID_KNOLL_STATUS climbDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_AVOID_KNOLL_STATUS acrossDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    E_AVOID_KNOLL_STATUS completeAcrossDoorSill();
    E_AVOID_KNOLL_STATUS failAcrossDoorSill();
    
    void startBalanceDoorSill(tPose robotPose);
    bool checkBalanceDoorSill(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
    

public:

    CAvoiding();
    ~CAvoiding();

    void updateAvoidInfo (tPose robotPose, bool clean, bool isAvoiding);
    
    void setEscapeStep( E_ESCAPE_STEP step );
    E_ESCAPE_STEP getEscapeStep();

    virtual bool checkObstacle(tPose robotPose, bool clean, bool isAvoiding);

    void makeEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeBumperEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeCliffEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    //virtual void makeIrEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeWheelTrapEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeKnollEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeFrontEscapeAction(tPose robotPose, CWayPoint &wayPoint);
    virtual void makeLidarEscapeAction(tPose robotPose, CWayPoint &wayPoint);

    void initAvoiding();
    bool avoidRun(tPose robotPose, bool clean);    
    E_AVOID_STATUS getAvoidStatus ();
    E_AVOID_TYPE getAvoidType();
    E_AVOID_TYPE getStartAvoidType();
    tPose getAvoidPose(void);
    RSF_OBSTACLE_MASK getAvoidMask();
    bool isAvoidingEnd();
    void setAvoidingEnd(bool set);
    void setAvoidPoseHistory(tPose robotPose);
    std::list<tPose> getAvoidPoseHistory();
    tPose getAvoidStartPose();
    bool isTurnAvoiding();
    void setAvoidStatus ( E_AVOID_STATUS status ); 
    void setAvoidType(E_AVOID_TYPE type);
    void setStartAvoidType(E_AVOID_TYPE type);
    void setAvoidMask(RSF_OBSTACLE_MASK mask);
    void setAvoidPose(tPose robotPose);

};

