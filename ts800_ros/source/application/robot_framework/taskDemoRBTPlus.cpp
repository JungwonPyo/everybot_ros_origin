#include "taskDemoRBTPlus.h"

#include "utils.h"
#include "eblog.h"
#include "waveFrontier.h"  
#include "commonStruct.h"
#include "systemTool.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "rosPublisher.h"
#include "kinematics.h"
#include "coreData/serviceData/keyState.h"

#define _USE_FILE_WRITE         0
#define _USE_WATER_PUMP         0
#define _USE_CHECK_BATTERY      0  

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/
#include <fstream>

std::ofstream ofs_;
std::string filename_;

/**
 * @brief Construct a new CTaskExplorer::CTaskExplorer object
 */
CTaskDemoRBTPlus::CTaskDemoRBTPlus() 
    : currCalSN(0), preCalSN(0), alert_lowbatt(false), lowBatteryLevel(0), bRunPump(false)
{
    CStopWatch __debug_sw;

    movingErrorTime = (u32)-1;
    isSave = false;
    open_ = false;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CTaskExplorer::CTaskExplorer object
 * 
 */
CTaskDemoRBTPlus::~CTaskDemoRBTPlus()
{
    CStopWatch __debug_sw;
#if defined (_USE_FILE_WRITE) && (_USE_FILE_WRITE == 1)
    if(open_)
    {
        open_ = false;
        ofs_.close();
    }
#endif
    isSave = false;

    setDemoRBTPlusState(DEMORBTPLUS_STATE::NONE);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CTaskDemoRBTPlus::setDemoRBTPlusState(DEMORBTPLUS_STATE set)
{
    demoPlusState = set;
}

void CTaskDemoRBTPlus::funcPowerOff()
{
    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, RED, "POWER OFF(0) @@@");

    DISPLAY_CTR.stopAutoDisplay();
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::OFF);
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_GOOD_DAY);
    LED_CTR.ledAllOff();
    if(MOTION.isRunning()) MOTION.startStopOnMap(tProfile(),false);

    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, RED, "POWER OFF(1) @@@");

    //mcu power off 가 보내지는 시간을 고려해서 일단 0.5초 이후 ts800을 죽여 버리자.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, RED, "POWER OFF(2) @@@");
    processShutDownProcessor(false);
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_POWER, tMsgPowerOff()));

    //mcu power off 가 보내지는 시간을 고려해서 일단 0.5초 이후 ts800을 죽여 버리자.
    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, RED, "POWER OFF(3) @@@");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    processShutDownProcessor(true);
}

int CTaskDemoRBTPlus::processShutDownProcessor(bool isKillTS800)
{
    //processShutDown Commad;
    int exit_result = 0;

    eblog(LOG_LV_NECESSARY, "processShutDownProcessor  ");
 
    if (!isKillTS800)
    {
        exit_result = system("ps -ef | sed -n '/static_transform_publisher/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext static_transform_publisher kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext static_transform_publisher kill");
        }
        exit_result = system("ps -ef | sed -n '/imu_filter_node/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext imu_filter_node kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext imu_filter_node kill");
        }
        exit_result = system("ps -ef | sed -n '/cartographer/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext cartographer kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext cartographer kill");
        }
        exit_result = system("ps -ef | sed -n '/roscore/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext roscore kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext roscore kill");
        }
        exit_result = system("ps -ef | sed -n '/rosmaster/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext rosmaster kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext rosmaster kill");
        }
        exit_result = system("ps -ef | sed -n '/rosout/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext rosout kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext rosout kill");
        }
        exit_result = system("ps -ef | sed -n '/roslaunch/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext roslaunch kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "* ext roslaunch kill");
        }
    }
    else
    {
        //mcu power off 가 보내지는 시간을 고려해서 일단 0.5초 이후 ts800을 죽여 버리자.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        exit_result = system("ps -ef | sed -n '/ts800_app/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}");
        if (exit_result != 0)
        {
            eblog(LOG_LV_NECESSARY, "ext ts800_app kill 실패");
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, CYN, "*ext ts800_app kill");
        }
    }

    return exit_result;
}

void CTaskDemoRBTPlus::funcSlamOnOFF(bool on)
{
    if (on)
    {
        eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:SLAM_ON");
        if ( ROBOT_CONTROL.slam.isExistedSlamMap())
        {
            bool nResult = ROBOT_CONTROL.slam.removeSlamMapFile(false);
            if (!nResult)
            {
                eblog(LOG_LV_NECESSARY, " 지도 삭제 실패");
            }             
        }

        if (!ROBOT_CONTROL.slam.isSlamRunning())
        {
            //테스트인 경우 지도 추가 부분을 천천히 처리
            eblog(LOG_LV_NECESSARY, " SLAM ON");
            ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_CLEAN);
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
            ROBOT_CONTROL.slam.runSlam();
        }
        else
        {
            eblog(LOG_LV_NECESSARY, " 슬램이 켜져 있음");
        }
    }
    else
    {
        eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:SLAM_OFF");
 
        ROBOT_CONTROL.slam.exitSlam();
    }
}

void CTaskDemoRBTPlus::funcDriveTestInit()
{
    //m key : 1. IMU 초기화 -> 테스트 경로 생성(오른쪽 방향/왼쪽 방향)
    eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:IMU_INIT");

    //소요 시간 시작 시간 초기화 
    //시작할 때 초기화 하자.
    memset(&driveStartTime, 0, sizeof(driveStartTime));
    memset(&driveEndTime, 0, sizeof(driveEndTime));

    //틸 다운도 시키자
    bRunPump = false;
    ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ROBOT_CONTROL.clearSystemLocalization();
    ROBOT_CONTROL.system.initImuSensor();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " 경로 생성 : 오른쪽/왼쪽 방향(테스트1)/ㄹ 패턴(테스트2) ");
    if ( ROBOT_CONTROL.slam.isExistedSlamMap())
    {
        ServiceData.mapStorage.resetChargerPose();
        bool nResult = ROBOT_CONTROL.slam.removeSlamMapFile(false);
        if (!nResult)
        {
            eblog(LOG_LV_NECESSARY, " failure of the map del..");
        } 
    }

    //오른쪽 방향 경로 이동 로딩
    targetPoint1 = tPoint(0,0); //초기화
    patternList1.clear();
    patternList1 = setDriverTestSquarePattern();

    //주행 성능 : "ㄹ 2회 주행 패턴"
    targetPoint3 = tPoint(0,0); //초기화
    patternList3.clear();
    patternList3 = setDRiverTestPattern();

    move1Target = tPoint(0,0); //초기화
    move2Target = tPoint(0,0); //초기화

    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN, " 패턴 1에 대한 경로를 생성 합니다. 경로 갯수(오른쪽 방향) : " << patternList1.size());
    ceblog((LOG_LV_NECESSARY | LOG_LV_WALL), BOLDGREEN, " 패턴 3에 대한 경로를 생성 합니다. 경로 갯수: " << patternList3.size());
}

void CTaskDemoRBTPlus::funcDriveTestExit()
{
    //s key : ** 테스트 종료 : 정지 및 테스트 경로 삭제하자
    eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY::MOVING_STOP");
    SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);

#if defined (_USE_WATER_PUMP) && (_USE_WATER_PUMP == 1)
    bRunPump = false;
    waterSupply.stopWaterSupply();
#endif
#if defined (_USE_FILE_WRITE) && (_USE_FILE_WRITE == 1)
    if(open_)
    {
        open_ = false;
        ofs_.close();
    }
#endif
    //중지 시키고
    if(MOTION.isRunning())
    {
        MOTION.startStopOnMap(tProfile(),true);
    }

    //패턴 다 날리자.
    targetPoint1 = tPoint(0,0); //초기화
    targetPoint2 = tPoint(0,0); //초기화
    targetPoint3 = tPoint(0,0); //초기화

    move1Target = tPoint(0,0); //초기화
    move2Target = tPoint(0,0); //초기화

    patternList1.clear();
    patternList2.clear();
    patternList3.clear();
    setDemoRBTPlusState(DEMORBTPLUS_STATE::NONE);
}

void CTaskDemoRBTPlus::funcDriveTestRun(bool squareType)
{
#if defined (_USE_FILE_WRITE) && (_USE_FILE_WRITE == 1)  //종료 시점 시간 데이터 파일 생성
    if (open_ == false) 
    {
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        std::string file_base{"/home/ebot/log/time_"};
        filename_ = file_base + std::string(buffer) + ".csv";
        ofs_ << std::fixed << std::setprecision(12);
        ofs_.open(filename_.c_str());
        if (!ofs_) 
        {
            std::cerr << "Could not open " << filename_ << std::endl;
        }
        else 
        {
            std::cerr << "  filename =  " << filename_ << std::endl;
            open_ = true;
        }
    }
#endif
    DISPLAY_CTR.startDisplay(E_DisplayImageClass::START_CLEANING);
    LED_CTR.ledAllOff();

    if (squareType)
    {
        //1 key : 3. 이제 패턴 주행을 위한 대기 후 테스트 시작하자.
        //1
        eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:CLEAN_AUTO");

        //물공급 시작 시키자.
        //동작 시간 테스트 시엔 물 공급 펌프 필요함.
        bRunPump = false;
        setDemoRBTPlusState(DEMORBTPLUS_STATE::START_WATER_SUPPLY); //setDemoRBTPlusState(DEMORBTPLUS_STATE::WAIT);
    }
    else
    {
        //2
        //2 키 : 홈키(도킹)
        eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:HOME");

        //직진성 테스트시에는 펌프 필요 없음
        bRunPump = false;
        setDemoRBTPlusState(DEMORBTPLUS_STATE::WAIT_DRIVE_S);  
    }
}

bool CTaskDemoRBTPlus::testKeyChecker(tPose robotPose)
{
    bool bRet = false;    
    CRobotKinematics k;

    switch(ServiceData.key.getKeyValue()) {
        //전원 OFF
        case E_KEY_TYPE::POWER_OFF:
            funcPowerOff();
            break;
 
        //테스트 시작 시퀀스 
        case E_KEY_TYPE::IMU_INIT: //(i key )
            funcDriveTestInit();
            break;
        case E_KEY_TYPE::SLAM_ON: //f key
            funcSlamOnOFF(true);
            break;
        case E_KEY_TYPE::CLEAN: // 1 key
            funcDriveTestRun(true); //pattern 1(square)
            break;
        case E_KEY_TYPE::HOME:  // 2 key
            funcDriveTestRun(false); //pattern 2(S)
            break;

        //테스트 종료 시퀀스 (s key)
        case E_KEY_TYPE::MOVING_STOP:
            //funcDriveTestExit();
            //if(MOTION.isRunning())
            MOTION.startStopOnMap(tProfile(),true);
            break;

        //전진 테스트
        case E_KEY_TYPE::MOVING_UP :
            {
                MOTION.sendMessageDrvie(15, 0); //30cm 직진
                //moveTarget0 = tPoint(1.0, 0.0); //초기화
                //startPointMOVE = tPoint(robotPose.x, robotPose.y);
                //MOTION.startLinearToPointOnMap(robotPose, moveTarget0,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << moveTarget0.x << " ," << moveTarget0.y);
            }
            break;
        //좌회전 테스트
        case E_KEY_TYPE::MOVING_LEFT:
            {
                tProfile profile = tProfile();
                profile.desAngVel = DEG2RAD(30);
                CRobotKinematics k;
                motionTargetRad0 = k.rotation(robotPose, DEG2RAD(90));

                MOTION.startRotation(robotPose, motionTargetRad0, profile, E_ROTATE_DIR::CCW);
                ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_LEFT_90 ");
            }
            break;
        //우회전 테스트 
        case E_KEY_TYPE::MOVING_RIGHT:
            {
                tProfile profile = tProfile();
                profile.desAngVel = DEG2RAD(30);
                CRobotKinematics k;
                motionTargetRad0 = k.rotation(robotPose, DEG2RAD(-90));

                MOTION.startRotation(robotPose, motionTargetRad0, profile, E_ROTATE_DIR::CW);
                ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_RIGHT_90 ");
            }
            break;
  
        //테스트 종료 시퀀스(g key)
        case E_KEY_TYPE::SLAM_OFF:
            funcDriveTestExit();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            funcSlamOnOFF(false);
            break;

        //지도 생성 조건 한번 테스트
        case E_KEY_TYPE::TILTING_UP:
            //waterSupply.waterDrainONOFF(true);
            eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:TILTING_UP");
            ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_CLEAN);
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
            break;
        case E_KEY_TYPE::TILTING_DONW:
            //waterSupply.waterDrainONOFF(false);
            eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " KEY:TILTING_DONW");
            ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_EXPLORE);
            ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
            break;
        default:
            break;
    }

    return bRet;
}

timespec CTaskDemoRBTPlus::getCurrentTime()
{
    timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return currentTime;
}

void CTaskDemoRBTPlus::debugPrint(u16 time, tPose slamPose, tPose cevaPose, tPose robotPose, double batteryVolt, RSU_OBSTACLE_DATA *pObstacle)
{
    checkTime.check = SYSTEM_TOOL.getSystemTick() - checkTime.start;

	if (checkTime.check >= time)
	{
		checkTime.start = SYSTEM_TOOL.getSystemTick();
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1) 
        currCalSN = SC<int>(ServiceData.localiz.getSysPose().calSn);
        if (preCalSN != currCalSN)
            currTime = getCurrentTime();
        
        preCalSN = currCalSN;

        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "\t" << "|BattVolt:" << PRECISION(8) << batteryVolt << ", " << "CalSN:" << currCalSN << ", " << currTime.tv_sec <<"."<<currTime.tv_nsec);
#else
        ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "\t" << "|BattVolt:" << PRECISION(8) << batteryVolt);
#endif
        if (!ROBOT_CONTROL.slam.isSlamRunning())
        {
            ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDRED, "\t" << "슬램 좌표가 아직 나오지 않았어요.. 잠시만 기다려 주세요");
        }
        else
        {
            //구동 시간
            //driveTime = get_system_time()-driveStartTime;
            //ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "\t" << "|driveStartTime: " << driveTime);

            clock_gettime(CLOCK_MONOTONIC, &driveEndTime);

            // 소요 시간 계산 (초와 나노초를 각각 계산)
            seconds = driveEndTime.tv_sec - driveStartTime.tv_sec;
            nanoseconds = driveEndTime.tv_nsec - driveStartTime.tv_nsec;
            elapsed = seconds + nanoseconds*1e-9;
            ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDRED, "소요 시간(초) : " <<  elapsed);
        }

        //ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "\t" << "|imu:" << SC<int>(pObstacle->imu.Ax)<<","<<SC<int>(pObstacle->imu.Ay)<<","<<SC<int>(pObstacle->imu.Az)<<","<<SC<int>(pObstacle->imu.Gpitch)<<","<<SC<int>(pObstacle->imu.Groll)<<","<<SC<int>(pObstacle->imu.Gyaw));
        ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "\t" << "|SlamPose:" << PRECISION(8) << SC<double>(slamPose.x) << ", " << SC<double>(slamPose.y) << ", " << SC<double>(utils::math::rad2deg(slamPose.angle)));
        ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN , "\t" << "|CevaPose:" << PRECISION(8) << SC<double>(cevaPose.x) << ", " << SC<double>(cevaPose.y) << ", " << SC<double>(utils::math::rad2deg(cevaPose.angle)));
    }
}

void CTaskDemoRBTPlus::checkLowBattery(u16 time, tPose robotPose, double batVolt )
{
    checkBattTime.check = SYSTEM_TOOL.getSystemTick() - checkBattTime.start;
    
    if (checkBattTime.check >= time)
    {
        checkBattTime.start = SYSTEM_TOOL.getSystemTick();
        ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "\t" << "|BattVolt:" << PRECISION(8) << batVolt);
        E_BATTERY_STATE battState = ServiceData.battery.getBatteryState();
        //습식 테스트 일 경우 
        if(battState == E_BATTERY_STATE::BATT_NEED_CHARGE)
        {
            eblog(LOG_LV_NECESSARY|LOG_LV_TEST, " MOVING STOP_LOW BATTERY");

            clock_gettime(CLOCK_MONOTONIC, &driveEndTime);

            // 소요 시간 계산 (초와 나노초를 각각 계산)
            seconds = driveEndTime.tv_sec - driveStartTime.tv_sec;
            nanoseconds = driveEndTime.tv_nsec - driveStartTime.tv_nsec;
            elapsed = seconds + nanoseconds*1e-9;
            ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDRED, "BATT_NEED_CHARGE::소요 시간(초) : " <<  elapsed);

            //사운드 출력하고
            alert_lowbatt = true;
            //DISPLAY_CTR.ChargingDisplay(true,battState);

            //중지.
            if(MOTION.isRunning())
            {
                MOTION.startStopOnMap(tProfile(),true);
                //MOTION.proc();
            }

            //물 공급 중지 시키고
#if defined (_USE_WATER_PUMP) && (_USE_WATER_PUMP == 1)
            bRunPump = false;
            waterSupply.stopWaterSupply();
#endif
            //주행 시간 파일로 저장.
#if defined (_USE_WATER_PUMP) && (_USE_WATER_PUMP == 1)
            if (open_) 
            {
                if (!isSave)
                {
                    ofs_ << elapsed << std::endl;
                    isSave = true;
                }
            }
#endif
            //패턴 다 날리자.
            //if(patternList1.size() > 0) 
            patternList1.clear();

            patternList2.clear();

            //if(patternList3.size() > 0) 
            patternList3.clear();
            
            //상태 변경
            setDemoRBTPlusState(DEMORBTPLUS_STATE::NONE);
        }
    }
}

 void CTaskDemoRBTPlus::runWaterPump(bool enable )
 {
    //물공급
    if ( enable )
    {
        waterSupply.WaterSupplyStateMachine(waterSupplyStartTime);
    }
 }

/**
 * @brief explorer proc
 * jhnoh, 23.01.16
 * @param robotPose     로봇 좌표 
 * @return tExplorerInfo 
 * 
 * @note  연산시간: 2ms ~ 11.0ms 
 * @date  2023-08-28
 */
bool CTaskDemoRBTPlus::taskDemoRBTPlusRun(tPose robotPose, tPose slamPose, tPose cevaPose, double batteryVolt, RSU_OBSTACLE_DATA *pObstacle)
{    
    CStopWatch __debug_sw;
    bool ret = false;
    CRobotKinematics k;

    /* ==============================================================
        * late of the sensors time
        /home/ebot/catkin_ws/devel/lib/ts800_ros/config/config.json
        - "late_Of_odom_sec": 0.25,
        - "late_Of_lidar_sec": 0.145,
        - "late_Of_mcu_ms": 425

        i key(imu init) -> f key(slam on) -> 1 (test start)
        s key(test stop) -> g key(slam off)

        p key(power off)
    =============================================================================*/
    testKeyChecker(robotPose);

    // 0.2 초마다 슬램/좌표 출력
    debugPrint(0.2*SEC_1, slamPose, cevaPose, robotPose, batteryVolt, pObstacle);

    //1초마다 체크 하자
    //low battery 일 경우 로봇 정지 및 경로 삭제 시키자.
    //타스크 시작하자 마자 밧데리 체크가 들어 갈 경우 상태가 로밧데리 상태가 됨..
    //밧데리 레벨은 12 volt임
#if defined (_USE_CHECK_BATTERY) && (_USE_CHECK_BATTERY == 1) 
    checkLowBattery(5*SEC_1, robotPose, batteryVolt);
#endif
    //물공급 상태 머신 돌림.
#if defined (_USE_WATER_PUMP) && (_USE_WATER_PUMP == 1)
    runWaterPump(bRunPump);
#endif
    switch (demoPlusState)
    {
        case DEMORBTPLUS_STATE::NONE:
            break;

        //물공급 시작.
        case DEMORBTPLUS_STATE::START_WATER_SUPPLY:
            {
                if (ROBOT_CONTROL.slam.isSlamRunning())
                {
                    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "START DRIVE TEST");
#if defined (_USE_WATER_PUMP) && (_USE_WATER_PUMP == 1)
                    //물 공급 시작 시간
                    bRunPump = true;
                    waterSupplyStartTime = SYSTEM_TOOL.getSystemTime();
#endif
                    // 시작 시간 측정
                    clock_gettime(CLOCK_MONOTONIC, &driveStartTime);
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_TEST_WAIT);
                }
            }
            break;

        /////////////////////////IMU 특성을 고려한 패턴 제어///////////////////
        case DEMORBTPLUS_STATE::DRIVE_TEST_WAIT:
            {
                movingErrorTime = SYSTEM_TOOL.getSystemTick();
                clock_gettime(CLOCK_MONOTONIC, &driveEndTime);

                // 소요 시간 계산 (초와 나노초를 각각 계산)
                seconds = driveEndTime.tv_sec - driveStartTime.tv_sec;
                nanoseconds = driveEndTime.tv_nsec - driveStartTime.tv_nsec;
                elapsed = seconds + nanoseconds*1e-9;
                //ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDRED, "TEST_WAIT::소요 시간(초) : " <<  elapsed);
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_2_0); // 0,0 -> 2.0 
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_RESTART:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_RESTART ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_2_0);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_2_0: //직진 제어
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(2.0, 0.0); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_2_0_0 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_LEFT1_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_LEFT1_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_LEFT1_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_2_0_5);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_2_0_5: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(2.0, 0.5); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_2_0_5 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_LEFT2_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_LEFT2_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90)); 

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW); //CCW
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_LEFT2_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_0_0_5);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_0_0_5: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(0.0, 0.5); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_2_0_5 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_RIGHT1_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_RIGHT1_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(-90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_RIGHT1_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_0_1_0);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_0_1_0: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(0.0, 1.0); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_0_1_0 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_RIGH2_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_RIGH2_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(-90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_RIGH2_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_2_1);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_2_1: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(2.0, 1.0); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_2_1 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_LEFT3_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_LEFT3_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_LEFT3_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_2_1_5);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_2_1_5: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(2.0, 1.5); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_2_1_5 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_LEFT4_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_LEFT4_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_LEFT3_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_0_1_5);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_0_1_5: //회전 각도 체크 => 직진
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(0.0, 1.5); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_0_1_5 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_RIGHT3_90);
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_RIGHT3_90:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, moveTarget1, 0.15); //0.16
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPointMOVE, moveTarget1))
                {
                    tProfile profile = tProfile();
                    profile.desAngVel = DEG2RAD(30);
                    CRobotKinematics k;
                    motionTargetRad = k.rotation(robotPose, DEG2RAD(90));

                    MOTION.startRotation(robotPose, motionTargetRad, profile, E_ROTATE_DIR::CCW);
                    //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_RIGHT3_90 ");
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_GO_0_0);
                }
            }
            break;

        case DEMORBTPLUS_STATE::DRIVE_GO_0_0:
            if (MOTION.isNearTargetRad(robotPose, motionTargetRad, DEG2RAD(5)))
            {
                moveTarget1 = tPoint(0.0, 0.0); //초기화
                startPointMOVE = tPoint(robotPose.x, robotPose.y);
                MOTION.startLinearToPointOnMap(robotPose, moveTarget1,tProfile());
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "현재 위치 : " << robotPose.x << " ," << robotPose.y); 
                //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "목적지 : " << move1Target.x << " ," << move1Target.y);
                //ceblog(LOG_LV_NECESSARY|LOG_LV_POSE_DEBUG, BOLDYELLOW, "DRIVE_GO_0_0 ");
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_RESTART);
            }
            break;
        /////////////////////////IMU 특성을 고려한 패턴 제어///////////////////
        case DEMORBTPLUS_STATE::WAIT:
            {
                if (ROBOT_CONTROL.slam.isSlamRunning())
                {
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::WAIT_DRIVE_S); //오른 쪽 방향으로 시작

                    clock_gettime(CLOCK_MONOTONIC, &driveEndTime);

                    // 소요 시간 계산 (초와 나노초를 각각 계산)
                    seconds = driveEndTime.tv_sec - driveStartTime.tv_sec;
                    nanoseconds = driveEndTime.tv_nsec - driveStartTime.tv_nsec;
                    elapsed = seconds + nanoseconds*1e-9;

                    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDRED, "WAIT::소요 시간(초) : " <<  elapsed);
                }
            }
            break;

        //신뢰성 "ㄹ" 패턴 2회 주행 성능 테스트 
        case DEMORBTPLUS_STATE::WAIT_DRIVE_S:
            if (ROBOT_CONTROL.slam.isSlamRunning())
                setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_S); //이제 "ㄹ" 패턴 시작하자
            break;

        case DEMORBTPLUS_STATE::DRIVE_S:
            
            if(!MOTION.isRunning())
            {
                if(patternList3.size() > 0)
                {
                    targetPoint3 = patternList3.front();
                    patternList3.pop_front();
                    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDGREEN, "경로가 설정되어 이동을 시작 합니다 목적지 : " << targetPoint3.x << " ," << targetPoint3.y << " 남은 경로 갯수 : " << patternList1.size());
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::START_S_MOVE_TO_POINT);
                }
                else
                {
                    eblog(LOG_LV_NECESSARY|LOG_LV_TEST, "모든 경로 완료 멈추자");
                    if(MOTION.isRunning())
                    {
                        MOTION.startStopOnMap(tProfile(),true);
                    }

                    //패턴 다 날리자.
                    targetPoint1 = tPoint(0,0); //초기화
                    targetPoint3 = tPoint(0,0); //초기화

                    patternList1.clear();
                    patternList3.clear();
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::NONE);
                }
            }
            break;

        case DEMORBTPLUS_STATE::START_S_MOVE_TO_POINT:
            ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, WHITE, "robotPose ("<<BOLDYELLOW<<robotPose.x<<", "<<robotPose.y<<WHITE<<") ");
            ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, WHITE, "타겟 ("<<BOLDYELLOW<<targetPoint3.x<<", "<<targetPoint3.y<<WHITE<<") 좌표로 이동할거에요.");
            startPoint3 = tPoint(robotPose.x, robotPose.y);
            MOTION.startLinearAngularPriorToPointOnMap(robotPose, targetPoint3, tProfile());
            setDemoRBTPlusState(DEMORBTPLUS_STATE::RUN_S_MOVE_TO_POINT);
            break;

        case DEMORBTPLUS_STATE::RUN_S_MOVE_TO_POINT:
            {
                bool bNear = MOTION.isNearTargetPose(robotPose, targetPoint3, 0.1); //0.1
                if(bNear || MOTION.isOverTargetPoint(robotPose,startPoint3,targetPoint3))
                {
                    ceblog(LOG_LV_NECESSARY|LOG_LV_TEST, BOLDYELLOW, "다시 경로 이동");
                    MOTION.startStopOnMap(tProfile(),true);
                    setDemoRBTPlusState(DEMORBTPLUS_STATE::DRIVE_S);
                }
            }
            break;

        default:
            targetPoint1 = tPoint(0,0); //초기화
            targetPoint3 = tPoint(0,0); //초기

            patternList1.clear();
            patternList3.clear();
            setDemoRBTPlusState(DEMORBTPLUS_STATE::NONE);
            break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

//신뢰성 무한 왼/오른쪽 "ㅁ" 패턴
std::list<tPoint> CTaskDemoRBTPlus::setDriverTestSquarePattern()
{
    tPoint pathPoint;
    std::list<tPoint> pathPointLists;

    // 1m 이동으로 하자
    // pixart 오차 1m 이동 시 0.7% 대략 7cm
    // pixart 회전 오파 10cm 정도 발생
    tPoint p1(1.0, 0.0);
    pathPointLists.push_back(p1);
    tPoint p2(1.0, 1.0); //회전
    pathPointLists.push_back(p2);
    tPoint p3(0.0, 1.0);
    pathPointLists.push_back(p3);
    tPoint p4(0.0, 0.0); //회전
    pathPointLists.push_back(p4);

#if 0
    tPoint p5(0.0, 1.0); //직진
    pathPointLists.push_back(p5);
    tPoint p6(1.0, 1.0); //회전
    pathPointLists.push_back(p6);
    tPoint p7(1.0, 0.0); //직진
    pathPointLists.push_back(p7);

    //끝
    tPoint p8(0.0, 0.0); //회전
    pathPointLists.push_back(p8);
#endif

    return pathPointLists;
}

std::list<tPoint> CTaskDemoRBTPlus::setDriverTestSquarePattern2()
{
    tPoint pathPoint;
    std::list<tPoint> pathPointLists;

    tPoint p1(0.0, 1.0); //직진
    pathPointLists.push_back(p1);
    tPoint p2(1.0, 1.0); //회전
    pathPointLists.push_back(p2);
    tPoint p3(1.0, 0.0); //직진
    pathPointLists.push_back(p3);

    tPoint p4(0.0, 0.0); //회전
    pathPointLists.push_back(p4);

    return pathPointLists;
}


//3회 턴 직진 및 속도 성능 주행 경로 테스트 "ㄹ 패턴 2번"
//신뢰성 요구
std::list<tPoint> CTaskDemoRBTPlus::setDRiverTestPattern()
{
    tPoint pathPoint;
    std::list<tPoint> pathPointLists;

    tPoint p1(2.0, 0.0);
    pathPointLists.push_back(p1);

    tPoint p2(2.0, 0.5);
    pathPointLists.push_back(p2);

    tPoint p3(0.0, 0.5);
    pathPointLists.push_back(p3);

    tPoint p4(0.0, 1.0);
    pathPointLists.push_back(p4);

    tPoint p5(2.0, 1.0);
    pathPointLists.push_back(p5);

    tPoint p6(2.0, 1.5);
    pathPointLists.push_back(p6);

    tPoint p7(0.0, 1.5);
    pathPointLists.push_back(p7);

    return pathPointLists;
}