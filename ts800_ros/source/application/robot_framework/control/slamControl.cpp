#include "control/slamControl.h"

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <cmath>
#include <limits>
#include <vector>
#include <cstdlib> // for system()
#include <sstream>
#include <cstdio>
#include <cstring>

#include <sys/types.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>

#include "systemTool.h"
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#if 0
///opt/ros/melodic/include/cartographer
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"

///opt/ros/melodic/include/cartographer_ros_msgs
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
//#include <cartographer_ros_msgs/Pose.h>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "eblog.h"
#include "define.h"
#include "MessageHandler.h"
#include "utils.h"
#include "slamControl.h"

using namespace cv;
using namespace std;

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CSlamControl::CSlamControl() : state(SLAM_STATE_NONE), trajectory_id(0), isSlamGridUpdated(false), isSlamLocalUpdated(false), 
                            bSlamRun(false), setInitPose(false), bInitPoseTemp(false), bSlamMapLoading(false), 
                            bRelocal(false), bMotionRestart(false), bEnableGridmap(true), 
                            map_filter_mode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT),
                            submapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_EXPLORE),
                            isTrackPose(false),
                            resetTrjactoryMode(0),
                            bReLocalMotionCompleted(false)
{
    CStopWatch __debug_sw;
    
    memset(systemCmd, 0, 5120);

    //create of the slam maps of the pbstream dirctory
    if (createDir()) {
        eblog(LOG_LV, "map dir create success.");
    } else {
        eblog(LOG_LV, "map dir create failed.");
    }

    eblog(LOG_LV,  "");
    
    tracked_x = 0.0;     
    tracked_y = 0.0;
    tracked_angle = 0.0;

    saveMapResult = E_SAVE_MAP_RESULT::NONE;

    TIME_CHECK_END(__debug_sw.getTime());   
}

CSlamControl::~CSlamControl()
{
    CStopWatch __debug_sw;
    
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CSlamControl::createDir()
{
    CStopWatch __debug_sw;

    if (!IsFileExist("/home/ebot/map"))
    {
        int status = mkdir("/home/ebot/map", 0777); // or 700 
        if (!status)
        {
            eblog(LOG_LV_NECESSARY, "create of the map folder");
            
            TIME_CHECK_END(__debug_sw.getTime());
            return true;
        }
        else
        {
            eblog(LOG_LV_NECESSARY, "fail create of the map folder");
            
            TIME_CHECK_END(__debug_sw.getTime());
            return false;
        }
    }
    else
    {
        eblog(LOG_LV_NECESSARY, "exist of the map folder");

        TIME_CHECK_END(__debug_sw.getTime());
        return true;
    }
}

int CSlamControl::IsFileExist(const char* path) { return !access(path, F_OK); }

/**
 * @brief 
 * 
 * @return E_SLAM_STATE 
 */
E_SLAM_STATE CSlamControl::getSlamState() {
    return state;
}

/**
 * @brief 
 * 
 * @param in 
 */
void CSlamControl::setSlamStatus(E_SLAM_STATE in) {
    state = in;
}

/**
 * @brief 
 * 
 */
int CSlamControl::setExitSlam(void)
{
    CStopWatch __debug_sw;
    
    int nResult = 0;

    //memory 종료되었는지 확인 -> re kill
    // ps -ef | grep '프로세스명' | awk '{print $2}' | xargs kill
    int ext_result = system("ps -ef | sed -n '/cartographer/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
    if (ext_result != 0)
    {
        eblog(LOG_LV_NECESSARY, "ext kill 실패");
        nResult = -1;
    }

    eblog(LOG_LV_NECESSARY, "setExitSlam");

    //do not insert submap and estamate pos (because of the next slam run)
    setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_EXPLORE);
    setTrackPoseUpdate(false);
    setRelocalMotionCompleted(false);
    TIME_CHECK_END(__debug_sw.getTime());
    return nResult;
}

/**
 * @brief 
 * 
 */
int CSlamControl::setSlamLoad(void)
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, "setSlamLoad --st ");
    int nResult = 0;

    std::string launch_command = "roslaunch ts800_ros cartographer_node.launch &";   
    
    // popen() 함수로 카토그래퍼 launch 파일 실행
    FILE* launch_process = popen(launch_command.c_str(), "r");
    if (launch_process == nullptr) {
        eblog(LOG_LV_NECESSARY, "Failed to launch cartographer.");
        nResult = -1;
    }

    // 실행 완료 대기
    int status = pclose(launch_process);
    if (status == -1) {
        eblog(LOG_LV_NECESSARY, "Error occurred while waiting for cartographer to exit");
        nResult = -1;
    }

    // 실행이 정상적으로 종료되었는지 확인
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        eblog(LOG_LV_NECESSARY, "Cartographer launched successfully.");
    } else {
        eblog(LOG_LV_NECESSARY, "Failed to launch cartographer.");
        nResult = -1;
    }

    eblog(LOG_LV_NECESSARY, "setSlamLoad --ed ");

    TIME_CHECK_END(__debug_sw.getTime());
    return nResult;
}

/**
 * @brief 
 * 
 * @return int 
 */
int CSlamControl::setSlamMapLoad(bool isTemporary)
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_NECESSARY, "setSlamMapLoad --st ");
 
    int nResult = 0;
    std::string launch_command;

    if (!isTemporary)
    {
        eblog(LOG_LV_NECESSARY, "저장된 슬램 맵 로딩!");
        launch_command = "roslaunch ts800_ros cartographer_load_state.launch &";
    }
    else
    {
        eblog(LOG_LV_NECESSARY, "임시로 저장된 맵 로딩!");
        launch_command = "roslaunch ts800_ros cartographer_temp_load_state.launch &";
    }

    // popen() 함수로 카토그래퍼 launch 파일 실행
    FILE* launch_process = popen(launch_command.c_str(), "r");
    if (launch_process == nullptr) {
        eblog(LOG_LV_NECESSARY, "Failed to launch cartographer.");
        nResult = -1;
    }

    // 실행 완료 대기
    int status = pclose(launch_process);
    if (status == -1) {
        eblog(LOG_LV_NECESSARY, "Error occurred while waiting for cartographer to exit");
        nResult = -1;
    }

    // 실행이 정상적으로 종료되었는지 확인
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        eblog(LOG_LV_NECESSARY, "Cartographer launched successfully.");
    } else {
        eblog(LOG_LV_NECESSARY, "Failed to launch cartographer.");
        nResult = -1;
    }

    eblog(LOG_LV_NECESSARY, "setSlamMapLoad --ed ");

    TIME_CHECK_END(__debug_sw.getTime());
    return nResult;
}

/**
 * @brief remove of the pbsream
 * 
 * @return true 
 * @return false 
 */
bool CSlamControl::removeSlamMapFile(bool isTemporary) 
{     
    bool result = false;

    //저장할 맵 삭제
    if (!isTemporary)
    {
        if (IsFileExist("/home/ebot/map/map.pbstream"))
        {
            //remove of the pbstream
            char strPath[] = { "/home/ebot/map/map.pbstream" };
            int nResult = remove( strPath );
            if( nResult == 0 )
            {
                eblog(LOG_LV_NECESSARY, "파일 삭제 성공 ");
                result = true;
            }
            else if( nResult == -1 )
            {
                eblog(LOG_LV_NECESSARY, "파일 삭제 실패");
                result = false;
            }
        } 
        else 
        {
            result = false;
        }
    }
    else
    {
        //청소 중 임시로 저장될 맵
        if (IsFileExist("/home/ebot/map/temp_map.pbstream"))
        {
            //remove of the pbstream
            char strPath[] = { "/home/ebot/map/temp_map.pbstream" };
            int nResult = remove( strPath );
            if( nResult == 0 )
            {
                eblog(LOG_LV_NECESSARY, "파일 삭제 성공 ");
                result = true;
            }
            else if( nResult == -1 )
            {
                eblog(LOG_LV_NECESSARY, "파일 삭제 실패");
                result = false;
            }
        } 
        else 
        {
            result = false;
        }
    }

    return result;
}

/**
 * @brief 
 * 
 */
void CSlamControl::setSlamFinishTrajectory(void)
{
    CStopWatch __debug_sw;

    if ( !bSlamPause )
    {
        eblog(LOG_LV, "finish_trajectory");
        memset(systemCmd, 0, 5120);

        //rosservice call /finish_trajectory 0 # 1
        eblog(LOG_LV, "setSlamFinishTrajectory::finish_trajectory[ " << trajectory_id << " ]");
        sprintf( systemCmd, "rosservice call /finish_trajectory %d", trajectory_id);
        eblog(LOG_LV_NECESSARY, "systemCmd[ " << systemCmd << " ]");
        ret = system(systemCmd);

        bSlamPause = true;
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param angle 
 */
void CSlamControl::setSlamStartTrajectory(double x, double y, double angle)
{
    CStopWatch __debug_sw;
    
    if ( bSlamPause )
    {
        memset(systemCmd, 0, 5120);

        trajectory_id++;
        
        eblog(LOG_LV_NECESSARY,  "setSlamStartTrajectory:: trajectory_id : " << trajectory_id << " x : " << x << "y : " << y << "angle : " << angle);
        sprintf(systemCmd, "rosservice call /start_trajectory \"{ configuration_directory: '/home/ebot/catkin_ws/devel/lib/ts800_ros/config/slam_lua', configuration_basename: 'cartographer_with_odom.lua', relative_to_trajectory_id: %d, use_initial_pose: false, initial_pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }\"", trajectory_id); //version 2.0
        //sprintf( systemCmd, "rosrun cartographer_ros cartographer_start_trajectory --configuration_directory /home/ebot/catkin_ws/devel/lib/ts800_ros/config/slam_lua --configuration_basename cartographer_with_odom.lua --initial_pose '{to_trajectory_id = 0, timestamp=0,relative_pose = { translation = {  %.7lf, %.7lf, 0. }, rotation = {0.0, 0.0, %.5lf} } }'", x, y, angle);  //version 1.0             

        eblog(LOG_LV_NECESSARY, "setSlamStartTrajectory::systemCmd [ " << systemCmd << " ]");
        ret = system(systemCmd);

        bSlamPause = false;
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * Function to convert yaw (in radians) to quaternion (x, y, z, w)
 * @param yaw 
 * @param qx 
 * @param qy 
 * @param qz 
 * @param qw 
 */

void CSlamControl::onvertYawToQuaternion(double yaw, double& qx, double& qy, double& qz, double& qw) {
    double half_yaw = yaw * 0.5;
    qx = 0.0;
    qy = 0.0;
    qz = std::sin(half_yaw);
    qw = std::cos(half_yaw);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CSlamControl::isExistedSlamMap(void)
{
    bool result = false;

    if (IsFileExist("/home/ebot/map/map.pbstream")) {
        result = true;
    } else {
        result = false;
    }

    return result;
}

bool CSlamControl::doSaveSlamMapFile(bool isTemporary)
{
    bool bRet = false;

    if (isRunMapSave() == false)
    {        
        std::thread(&CSlamControl::saveSlamMapFile, this, isTemporary).detach();  
        bRet = true;
    }
    else
    {
        bRet = false;
    }   
    
    return bRet;
}


void CSlamControl::setMapSaveResult(E_SAVE_MAP_RESULT set){
    saveMapResult = set;
}

E_SAVE_MAP_RESULT CSlamControl::getMapSaveResult(){
    return saveMapResult;
}

bool CSlamControl::isRunMapSave(){
    return bIsRunMapSave;
}


int CSlamControl::saveSlamMapFile(bool isTemporary) {
    bIsRunMapSave = true;
    
    std::string writer_command;
    
    setMapSaveResult(E_SAVE_MAP_RESULT::SAVING);

    if (!IsFileExist("/home/ebot/map")) {
        eblog(LOG_LV_NECESSARY, "saveSlamMapFile -- map the dir not exit...");

        setMapSaveResult(E_SAVE_MAP_RESULT::FAIL);
    }
    else{
        //std::string writer_command = "rosservice call /write_state /home/ebot/map/map.pbstream";   //for_rev 1.0
        if (!isTemporary) {
            //저장할 맵
            writer_command = "rosservice call /write_state '{filename: \"/home/ebot/map/map.pbstream\", include_unfinished_submaps: true}'"; //for rev 2.0
        } else {
            //청소 중 임시로 저장될 맵 (단 도킹에서 삭제를 해야 하나? 저장 주기는 어떻게 해야 하나)
            //일단 청소 중 슬램이 켜져 있는 상태에서 5분 단위로 일단 저장 해보자.
            writer_command = "rosservice call /write_state '{filename: \"/home/ebot/map/temp_map.pbstream\", include_unfinished_submaps: true}'"; //for rev 2.0
        }

        // popen() 함수로 카토그래퍼 launch 파일 실행
        FILE* writer_process = popen(writer_command.c_str(), "r");
        if (writer_process == nullptr) {
            eblog(LOG_LV_NECESSARY, "Failed to write_state command.");
            setMapSaveResult(E_SAVE_MAP_RESULT::FAIL);            
        }

        // 실행 완료 대기
        int status = pclose(writer_process);
        if (status == -1) {
            eblog(LOG_LV_NECESSARY, "Error occurred while waiting for cartographer write_state to exit");
            setMapSaveResult(E_SAVE_MAP_RESULT::FAIL);
        }

        // 실행이 정상적으로 종료되었는지 확인
        if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            eblog(LOG_LV_NECESSARY, "Cartographer write_state successfully.");
            setMapSaveResult(E_SAVE_MAP_RESULT::COMPLETE);
        } else {
            eblog(LOG_LV_NECESSARY, "Failed to write_state Cartographer.");
            setMapSaveResult(E_SAVE_MAP_RESULT::FAIL);
        }
    }
    
    bIsRunMapSave = false;    
}

/**
 * @brief 
 * 
 */
void CSlamControl::runSlam(void) 
{
    CStopWatch __debug_sw;
    int result;

    eblog(LOG_LV_NECESSARY, "CSlamControl::runSlam");

    //do not insert submap and estamate pos (because of the next slam run)
    setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    
    // file path home/ebot/catkin_ws/src/ts800/application/ros/ts800_ros/launch
    if (!isExistedSlamMap()) {
        result = setSlamLoad();
    } else {
    //탐색 시점에 저장된 맵을 로딩
        result = setSlamMapLoad(false);
    }

    if (result != 0) {
        //TODO
    }

    //종료 및 시작 시에 아래 resume/pasue 관련된 부분 초기화
    bSlamPause = false;
    trajectory_id = 0;

    setSlamStatus(E_SLAM_STATE::SLAM_STATE_LAUNCH); 
    setTrackPoseUpdate(false);
    setRelocalMotionCompleted(false);
    TIME_CHECK_END(__debug_sw.getTime());
}

 /**
  * 들고 정지 후 다른 위치에 놓을 경우 호출 혹 키드냅 발생시 호출
 */
void CSlamControl::runSlamTemporaryMap(void) 
{
    CStopWatch __debug_sw;
    int result;

    eblog(LOG_LV_NECESSARY, "CSlamControl::runSlamTemporaryMap.");

    //do not insert submap and estamate pos (because of the next slam run)
    setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    
    // file path home/ebot/catkin_ws/src/ts800/application/ros/ts800_ros/launch
    //청소 중 임시로 저장된 맵을 로딩
    result = setSlamMapLoad(true);
    if (result != 0) {
        //TODO
    }

    //종료 및 시작 시에 아래 resume/pasue 관련된 부분 초기화
    bSlamPause = false;
    trajectory_id = 0;

    setSlamStatus(E_SLAM_STATE::SLAM_STATE_LAUNCH); 
    setTrackPoseUpdate(false);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
void CSlamControl::exitSlam(void)
{
    CStopWatch __debug_sw;
    
    ceblog(LOG_LV_NECESSARY, GREEN, "CSlamControl::exitSlam"); 

    int result = setExitSlam();
    if (result != 0) {
        //TODO
    }

    //종료 및 시작 시에 아래 resume/pasue 관련된 부분 초기화
    bSlamPause = false;
    trajectory_id = 0;

    setSlamStatus(E_SLAM_STATE::SLAM_STATE_EXIT);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 */
void CSlamControl::pauseSlam(void)
{
    CStopWatch __debug_sw;
    
    ceblog(LOG_LV_NECESSARY, GREEN, "CSlamControl::pauseSlam"); 
    setSlamFinishTrajectory();
    setSlamStatus(E_SLAM_STATE::SLAM_STATE_PAUSE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 */
void CSlamControl::resumeSlam(double x, double y, double angle)
{
    CStopWatch __debug_sw;
    
    ceblog(LOG_LV_NECESSARY, GREEN, "CSlamControl::resumeSlam"); 
    setSlamStartTrajectory(x,y,angle);
    setSlamStatus(E_SLAM_STATE::SLAM_STATE_RUN);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CSlamControl::resetSlam(double x, double y, double angle)
{
    CStopWatch __debug_sw;
    
    ceblog(LOG_LV_NECESSARY, GREEN, "CSlamControl::ResetSlam");
    setSlamFinishTrajectory();
    //delay 
    setSlamStartTrajectory(x,y,angle);
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CSlamControl::isSlamRunning(void)
{
    return bSlamRun;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setSlamRunning(bool set)
{
    bSlamRun = set;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setSlamMapLoading(bool set)
{
    bSlamMapLoading = set;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CSlamControl::isSlamMapLoading(void)
{
    return bSlamMapLoading;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setRelocalCompleted(bool set)
{
    bRelocal = set;
}

//slam of the tf update
void CSlamControl::setSlamLocalUpdate(bool set) {
    isSlamLocalUpdated = set;
}

bool CSlamControl::isSlamLocalUpdate() {
    return isSlamLocalUpdated;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setSlamGridUpdate(bool set) {
    isSlamGridUpdated = set;
}

/*
*/
bool CSlamControl::isSlamGridUpdate() {
    return isSlamGridUpdated;
}

bool CSlamControl::isSetSlamInitPose(void)
{
    return setInitPose;
}

/**
 * @brief slam 상태 감시
 * 
 */

u32 runSlamTick = (u32)-1;
bool CSlamControl::watchSlam()
{
    bool bRet = false;  // 초기위치 방향설정이 필요할 경우 return true
    bool bRun = false;
#if 1 //카토그래퍼로 부터 맵 매칭 후 좌표 데이터가 172ms 마다 나오는 부분으로 체크: CPU 점유율 1.5 -> 0로 절감됨
    if (getTrackPoseUpdate())
    {
        bRun = true;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, GREEN, "카토그래퍼로 부터 아직 좌표가 안 나옴. 잠시 대기");
        bRun = false;
    }
#else
    if (utils::os::isProcessRunning("cartographer_n"))
    {
        if (getTrackPoseUpdate())
        {
            bRun = true;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, GREEN, "카토그래퍼는 메모리 상주-> 아직 좌표가 안 나옴. 잠시 대기");
            bRun = false;
        }
    }
    else
    {
        bRun = false;
    }
#endif
    setSlamRunning(bRun);

    return bRet;
}

void CSlamControl::setSlamInitPose(bool set)
{
    CStopWatch __debug_sw;

    ceblog(LOG_LV_NECESSARY, GREEN, "setSlamInitPose::set : "<<set); 
    setInitPose = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CSlamControl::setRosNodeRef(ros::NodeHandle _nh)
{
    CStopWatch __debug_sw;
    
    nh_ref = _nh;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @return MAP_FILTER_MODE 
 */
MAP_FILTER_MODE CSlamControl::getMapFilterMode(void)
{
    return map_filter_mode;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setMapFilterMode(MAP_FILTER_MODE set)
{
    std::lock_guard<std::mutex> lock(mSlamFilterMutex);
    map_filter_mode = set;
}

/**
 * @brief 
 * 
 * @return SUBMAP_UPDATE_TYPE 
 */
SUBMAP_UPDATE_TYPE CSlamControl::getSubMapUpdateType(void)
{
    return submapUpdateType;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setSubMapUpdateType(SUBMAP_UPDATE_TYPE set)
{
    std::lock_guard<std::mutex> lock(mSubMapUpdateTypeMutex);
    submapUpdateType = set;
}


/**
 * @brief 
 * 
 * @return resetTrjactoryMode 
 */
int CSlamControl::getResetTrajectoryNode(void)
{
    return resetTrjactoryMode;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setResetTrajectoryNode(int set)
{
    std::lock_guard<std::mutex> lock(mResetTrajectoryMutex);
    resetTrjactoryMode = set;
}

/**
 * @brief 
 * 
 * @param in 

 */
void CSlamControl::setTrackPoseUpdate(bool in) 
{
    std::lock_guard<std::mutex> lock(mTrackPoseMutex);
    isTrackPose = in; 
};

bool CSlamControl::getTrackPoseUpdate() { return isTrackPose;}

bool CSlamControl::getRelocalMotionCompleted(void)
{
    return bReLocalMotionCompleted;
}

/**
 * @brief 
 * 
 * @param set 
 */
void CSlamControl::setRelocalMotionCompleted(bool set)
{
    std::lock_guard<std::mutex> lock(mRelocalMutex);
    bReLocalMotionCompleted = set;
}

//test
void CSlamControl::setTrackX(double in) {tracked_x = in; };
void CSlamControl::setTrackY(double in) {tracked_y = in;};
void CSlamControl::setTrackAngke(double in) {tracked_angle = in;};
double CSlamControl::getTrackX() {return tracked_x;}
double CSlamControl::getTrackY() {return tracked_y;}
double CSlamControl::getTrackAngle() {return tracked_angle;};
 
