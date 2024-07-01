/**
 * @file fileMng.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "json/json.h"
#include "fileLog.h"
#include "fileJson.h"
#include "ros/rosParameter.h"

#define FILE_MNG        CFileMng::getInstance()
#define CONFIG          FILE_MNG.getConfigJsonData()

/**************************************************/
#define D_TILTING_UP          0   // 틸팅 업일때
#define D_TILTING_DOWN        1   // 틸팅 다운일때
#define D_TILTING_DEFAULT     2   // 기존 속도
/* 청소 및 경로 이동중 틸팅 상황에 따라 바꿔주세요.! */
#define CLEAN_TILT_CONDITION    D_TILTING_DOWN
#define DSTAR_TILT_CONDITION    D_TILTING_DOWN
/**************************************************/

#if CLEAN_TILT_CONDITION == D_TILTING_UP
    #define CLEAN_SPEED             CONFIG.clean.speed
#elif CLEAN_TILT_CONDITION == D_TILTING_DOWN
    #define CLEAN_SPEED             CONFIG.clean.speed
#else // CLEAN_TILT_CONDITION == D_TILTING_DEFAULT
    //hjkim230111 - system odometry 셋팅 변경으로 인한 속도 적용 중 - 낮은 속도 제어 안됨.
    #define CLEAN_SPEED             CONFIG.clean.speed              //250  //ROS_CONFIG.speed_fwd
#endif
#define CLEAN_APPROACH_SPEED    CONFIG.clean.approachSpeed      //150
#define HEADING_MAX_STEER_SPEED 250 //ROS_CONFIG.max_steering_angle

#define CLEAN_SLOW_SPEED        CONFIG.clean.slowSpeed          //100
#define CLEAN_VERYSLOW_SPEED    CONFIG.clean.verySlowSpeed      //50

#define MANUAL_MOVE_FORWARD     CLEAN_SPEED//CONFIG.manualMoveForwardSpeed   //200
#define MANUAL_MOVE_TURN        CONFIG.manualMoveTurnSpeed     //1

#define CLEAN_TURN_SPEED        CONFIG.clean.turnSpeed                //200
#define CLEAN_SLOW_TURN_SPEED   CONFIG.clean.slowTurnSpeed            //150
#define CLEAN_MIN_SPEED         CONFIG.clean.minSpeed                 //50
#define CLEAN_MIN_TURN_SPEED    CONFIG.clean.minTurnSpeed             //50

#define AVOID_BACK_SPEED            CONFIG.avoidBackSpeed           //150

#define AVOID_TURN_SPEED            CONFIG.avoidTurnSpeed           //50

#define ADJUEST_HEADING_SPEED       CONFIG.adjuestHeadingSpeed      //50

// slam of the motion filter params
#define MAX_TIME_SECONDS       CONFIG.maxTimeSeconds      //1
#define MAX_DISTANCE_METERS       CONFIG.maxDistanceMeters      //0.2
#define MAX_ANGLE_RADIANS       CONFIG.maxAngleRadians      //0.54

#define MAX_DEFAULT_TIME_SECONDS       CONFIG.maxDefaultTimeSeconds      //1
#define MAX_DEFAULT_DISTANCE_METERS       CONFIG.maxDefaultDistanceMeters      //0.2
#define MAX_DEFAULT_ANGLE_RADIANS       CONFIG.maxDefaultAngleRadians      //0.54

#define CLEAN_DUMMY_SPEED   CONFIG.clean.dummyForwardSpeed  // 40
#define CLEAN_DUMMY_TURN_SPEED      CONFIG.clean.dummyTurnSpeed     // 40

#if DSTAR_TILT_CONDITION == D_TILTING_UP
    #define DSTAR_SPEED             CONFIG.dstar.tiltingUpSpeed
#elif DSTAR_TILT_CONDITION == D_TILTING_DOWN
    #define DSTAR_SPEED             CONFIG.clean.speed//CONFIG.dstar.tiltingDownSpeed
#else // DSTAR_TILT_CONDITION == TILTING_DEFAULT
    //hjkim230111 - system odometry 셋팅 변경으로 인한 속도 적용 중 - 낮은 속도 제어 안됨.
    #define DSTAR_SPEED             CONFIG.dstar.speed              //250  //ROS_CONFIG.speed_fwd
#endif

#define DSTAR_SLOW_SPEED                  CONFIG.dstar.slowSpeed
#define DSTAR_TURN_SPEED                  CONFIG.dstar.defaultTurnSpeed
#define DSTAR_SLOW_TURN_SPEED             CONFIG.dstar.slowTurnSpeed

//cradle speed - move, turn
#define CRADLE_MOVE_SPEED             CONFIG.clean.cradleMoveSpeed
#define CRADLE_TURN_SPEED             CONFIG.clean.CradleTrunSpeed

#define DUMMY_SLOW_SPEED          CONFIG.clean.DummySlowSpeed
#define DUMMY_SLOW_TURN_SPEED             CONFIG.clean.DummySlowTurnSpeed

#define ENABLE_ROS_PUBLISH_DEBUG            CONFIG.rosPublishDebug           //2.5 or 3

#define CLEANAREA_UPDOWN_MARGIN_FROM_WALL       CONFIG.clean.updownMarginFromWall // 청소영역과 벽 사이의 위아래 간격
#define CLEANAREA_LEFTRIGHT_MARGIN_FROM_WALL    CONFIG.clean.leftrightMarginFromwall // 청소영역과 벽 사이의 좌우 간격
#define SPACE_OF_CLEANING_LINES                    CONFIG.clean.spaceOfCleaningLines // 청소라인의 간격

#define LATE_OF_ODOM_SEC                          CONFIG.lateOfodomSec                 //odom pub 지연 오차 보상
#define LATE_OF_LIDAR_SEC                         CONFIG.lateOfLidarSec                 //lidar pub 지연 오차 보상
#define LATE_OF_MCU_MS                            CONFIG.lateOfMcuMs                 //RBT_PLUS MCU 지연 오차 보상

#define FIX_IP                      CONFIG.robot_ip

// LIDAR TIME
// ODOM TIME

//#define MOTION_CONTROLLER_DESIRED_V CONFIG.motionCtrDesiredV
//#define MOTION_CONTROLLER_DESIRED_W  (CONFIG.motionCtrDesiredW*0.0174533)

class CFileMng : public CFileLog, public CFileJson
{
private:
    CFileMng(/* args */){}
    CFileMng(const CFileMng& ref){}
    CFileMng& operator=(const CFileMng& ref) {}
    ~CFileMng(){}

public:

    static CFileMng& getInstance() {
        static CFileMng s;
        return s;
    }

    void init()
    {
        initLog("ts800_ros");
        readJsonFile(); // json 파일을 읽어옴.

        if ( isParsingLogJson() )
        {
            setLogSetting(getLogJsonData()); // logSetting.json 값을 넣어줌.
        }

        // __debug_print_log_setting();
        __debug_print_config(false);
    }
};
