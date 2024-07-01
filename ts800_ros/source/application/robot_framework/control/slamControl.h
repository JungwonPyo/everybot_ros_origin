/**
 * @file slamControl.h
 * @author yoon
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <ros/ros.h>

#include "commonStruct.h"

#include <mutex>
#include <condition_variable>


#define LOAD_STATE_FILE   0

/**
 *  0 : 정지,이동 상태에서는 맵 생성됨/정지&회전 상태에서는 맵이 생성되지 않음
    1 : 로봇 좌표만 생성, 맵은 생성되지 않음 ( 사용 처 : 틸업 혹 기타 경우 )
    2 : 정지, 이동, 정지&회전 모두 맵 생성됨 ( 처음 탐색 시작 시점에서 맵 확장이 필요한 시점)
*/
typedef enum
{
    ROBOT_MOVE_DEFAULT = 0,
    ROBOT_MOVE_REMOVE_MAP,
    ROBOT_STOP_LOTATE_INSERT_MAP,
    ROBOT_SKIP_MOTION_FILTER,
    MAP_FILTER_MODE_MAX,
} MAP_FILTER_MODE;

typedef enum
{
    SLAM_STATE_NONE = 0,
    SLAM_STATE_PAUSE,
    SLAM_STATE_RUN,
    SLAM_STATE_LAUNCH,
    SLAM_STATE_EXIT,
    SLAM_STATE_MAX
} E_SLAM_STATE;


enum class E_SAVE_MAP_RESULT
{
    NONE,
    SAVING,
    COMPLETE,
    FAIL,
};

//청소 중일 경우 서브 맵 업데이트 전체 주기를 늦게 처리
//탐색 중이거나 리로컬 같은 경우에는 전체 주기를 기존 처럼 빠르게
typedef enum
{
    SUBMAP_UPDATE_TYPE_EXPLORE  = 0,
    SUBMAP_UPDATE_TYPE_CLEAN,
} SUBMAP_UPDATE_TYPE;

class CSlamControl
{
private :
    MAP_FILTER_MODE map_filter_mode;
    SUBMAP_UPDATE_TYPE submapUpdateType;

    E_SLAM_STATE state;
    int trajectory_id;
    bool isSlamGridUpdated;
    bool isSlamLocalUpdated;
    bool bInitPoseTemp;
    bool setInitPose;
    
    bool isTrackPose;  

    int resetTrjactoryMode;

    ros::NodeHandle nh_ref;

    int ret;    
    std::condition_variable mCondition;
    char systemCmd[5120];
    bool bSlamRun;
    
    bool bEnableLaser;
    //
    bool bSlamMapLoading;
    bool bRelocal;
    bool bMotionRestart;
    bool bEnableGridmap;
    bool bReLocalMotionCompleted;

    std::mutex mResetTrajectoryMutex;
    std::mutex mSlamFilterMutex;
    std::mutex mSubMapUpdateTypeMutex;
    std::mutex mTrackPoseMutex;
    std::mutex mRelocalMutex;

    double tracked_x;     
    double tracked_y;
    double tracked_angle;
    bool bSlamPause;
    bool bIsRunMapSave;

    E_SAVE_MAP_RESULT saveMapResult;

public :
    CSlamControl();
    ~CSlamControl();
    
    E_SLAM_STATE getSlamState();
    void setSlamStatus(E_SLAM_STATE in);
    
    void setSlamFinishTrajectory(void);
    void setSlamStartTrajectory(double x, double y, double angle);

    int IsFileExist(const char* path);
    void setRosNodeRef(ros::NodeHandle _nh);

    void setSlamRunning(bool set);
    bool isSlamRunning(void);
    bool isSlamLocalUpdate(void);
    void setSlamLocalUpdate(bool set);

    bool isSetSlamInitPose(void);
    void setSlamInitPose(bool set);

    //used of the services
    bool isExistedSlamMap(void);
    bool removeSlamMapFile(bool isTemporary);

    //relocalMotion 완료 여부 확인  
    bool getRelocalMotionCompleted(void);
    void setRelocalMotionCompleted(bool set);

    void runSlam(void);
    void runSlamTemporaryMap(void);
    void exitSlam(void);
    void pauseSlam(void);
    void resumeSlam(double x, double y, double angle);
    void resetSlam(double x, double y, double angle);

    //저장 지도 있을 경우 청소 시점에서 회전 160/-160 회전 여부 체크
    void setSlamMapLoading(bool set);
    bool isSlamMapLoading(void);
 
    void setRelocalCompleted(bool set);
    bool istRelocalCompleted(void);
 
    void setSlamGridUpdate(bool set);
    bool isSlamGridUpdate();

    bool watchSlam();
 
    void setTrackX(double in);
    void setTrackY(double in);
    void setTrackAngke(double in);
    double getTrackX();
    double getTrackY();
    double getTrackAngle();
    
    void setTrackPoseUpdate(bool in);
    bool getTrackPoseUpdate();

    void setMapFilterMode(MAP_FILTER_MODE set);
    MAP_FILTER_MODE getMapFilterMode(void);

    SUBMAP_UPDATE_TYPE getSubMapUpdateType(void);
    void setSubMapUpdateType(SUBMAP_UPDATE_TYPE set);

    int getResetTrajectoryNode(void);
    void setResetTrajectoryNode(int set);

    bool isSlamPause(void) { return bSlamPause;}
    bool doSaveSlamMapFile(bool isTemporary);
    void setMapSaveResult(E_SAVE_MAP_RESULT set);
    E_SAVE_MAP_RESULT getMapSaveResult();
    bool isRunMapSave();

private:
    int saveSlamMapFile(bool isTemporary);//cartograper to the pbstream file
    bool createDir();
    void onvertYawToQuaternion(double yaw, double& qx, double& qy, double& qz, double& qw);
    int setExitSlam(void);
    int setSlamLoad(void);
    int setSlamMapLoad(bool isTemporary);
};
