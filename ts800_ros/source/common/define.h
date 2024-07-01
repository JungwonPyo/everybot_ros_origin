/**
 * @file define.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#define Q8_ROBOROCK 1   // 본체 : roborok               ,라이다 YD
#define Q8_3I       2   // 본체 : 3i                    ,라이다 YD
#define Q8_3I_LiDAR 3   // 본체 : 3i                    ,라이다 3i
#define T5_LIDAR    4   // 본체 : ???                   ,라이다 YD
#define TS800_ES    5   // 본체 : ???                   ,라이다 YD
#define TS800_WS    6   // 본체 : W/S 1차 3spin         ,라이다 YD
#define ROBOT_MODEL TS800_WS /* 사용하는 로봇 종류 선택 */
                 
#define TODO_LIST           0   // TODO LIST 용 define. 무조건 0임. 주석 명시용..
#define UI_UPDATE_CYCLE 0.1 //단위 sec, 최소값 : 0.06 (0.06초 이하 인 경우 UI 중복 재생 발생)  

#define GRAY_LV_KNOWN_AREA      255
#define GRAY_LV_UNKNOWN_AREA    128
#define GRAY_LV_KNOWN_WALL      0
#define GRAY_LV_UNKNOWN_WALL    200
// #define GRAY_LV_CLIFF           1
// #define GRAY_LV_DOCK            2

#define UNKNWON_REGION 0
#define WALL_REGION 255
#define EMPTY_REGION 128
#define WALL_THRESHOLD 60

/**
 * @brief 
 *  hit_probability(장애물 존재) = 0.57
 *  miss_probability(장애물 미존재) = 0.492
 */
#define WALL_THRESH             50 //55  //벽일 확률 
#define MOTION_CONTROLLER_DESIRED_V 0.350             // Motion Controller에서 사용할 목표 선속도 ( 단위 m/s )
#define MOTION_CONTROLLER_DESIRED_W (50*0.0174533)//(40*0.0174533)    // Motion Controller에서 사용할 목표 각속도 ( 단위 rad/s )
#define MAP_DOWNSCALE_VALUE 5  // wfd(탐색) 및 d*(경로계획)에서 사용되는 map의 스케일을 결정 ex) 6으로 입력하면 한 셀의 길이가 0.05*6 = 30cm


//******************************************* FOR BLOCK HARDWARE *****************************************************//

#define SKIP_CHECKTILT 0 // 1 // 0 // 1 //hjkim231208 - 틸팅제어 안되는 샘플이 있어 테스트 편의를 위해 생성하였음 : 틸팅 체크 스킵  

//******************************************* END BLOCK HARDWARE *****************************************************//




//******************************************* FOR BLOCK SOFTWARE *****************************************************//

#define USE_KIDNAP          0       // 1 : kidnap check 사용 on, 0 : kidnap check 사용 off
#define USE_SAVEDMAP        0       // 0 : 실시간 그리드 지도 사용, 1 : 저장된 지도 사용
#define USED_MAP_LOAD_SAVE   1      // using of the cartograper scan matcher by re-localizing
#define USE_LPF_POSE false
#define USE_AWS_APP_INTERFACE 0     // 0: AWS 자체 사용 x,  1 : AWS 사용
#define AWS_MODE 0                  // 0 : 부팅연결(station, AWS) 1 : 부팅연결(AWS) 2 : ip 고정 모드 
#define MSG_SENDING_INTERVAL 2      //AWS에 report 보내는 주기(sec)
#define TEST_FORBIDDEN_AREA 0       //AWS 지도관련 작업들 좌표 싱크 맞추기용
//******************************************* END BLOCK SOFTWARE *****************************************************//


#define DRIVE_TEST_TASK          0 //무한 반복 패턴 테스트가 필요할 경우(다른 타스크들은 다 죽임)

#define USED_RBT_PLUS            0 //최종 펌웨어가 RBT+ 일 경우 활성화 하세요.

#define DEBUG_ROS_PUB            0  //rviz로 디버깅이 필요할 때만 1로 모션 쪽에 보니 while 코드가 있음

/* 
    카토그래퍼에서 서브 맵 업데이트 주기 및 제어를 스위칭 할 수 있는 기능(slam_ws 다시 배포할 예정임)
    # 슬램 업데이트 주기/슬램 업데이트 제어(기존 처럼)
    # 사용 방법
    1. 느리게 지도 업데이트 하려면 
        ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_EXPLORE);
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    2. 빠르게 지도 업데잍 하려면 
        ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_CLEAN);
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
*/
#define NEW_SLAM_CONTROL          1

//******************************************* TEST MODE *****************************************************//
#define USE_PIXART_POSE       0 //hjkim240228 - PIXART 좌표를 이용해서 청소하기 테스트
#define USE_LINECLEAN_VELOCITY 0 //hjkim240228 - 속도제어를 이용해서 청소하기 테스트
#define USE_WALLTRACKAVOIDING 0 // 0 // 1
#define TEST_MANUAL_WALLTRACK_AVOID 0 //수동 제어 중 벽타기 회피 테스트 디파인 -> 수동 이동 시작 KEY_UP 실행 -> 장애물 감지 -> 벽타기 시작 -> 벽면주행 원점 도착 -> 이동 이동 시작 지점으로 이동 
#define INFINITE_CLEAN 0 //무한 청소 *현재 미구현
#define WATER_SUPPLY_TEST   0   //기구팀 요청 물공급 테스트 : 물공급 변경 버튼 시 1초 동작 후 멈춤 // 잔수제거 동작 시 60초 동작 후 멈춤
#define TEST_RANDOMPATTERN_FOR_CLEANTIME 0 //청소시간 테스트용 랜덤패턴(배터리 5000mA 기준)

//지도 데이터를 이용한 맵 매칭 및 리로컬 정의(현재는 비활성화 시킴- 향후 활성화 시키세요)
#define USE_SLAM_MAP_LOAD_RELOCAL        0

//******************************************* END TEST MODE *****************************************************//




//******************************************* FOR DEBUG *****************************************************//
#define SW_FOR_WS_TOBE_SEND_TO_CEVA 0   // CEVA 전달용 제품일때 1 - jspark 23.05.25
#define USE_CEVA_LOG_SAVE   0   // CEVA 로그 저장 기능 활성화 - 22.11.11 jspark
#define CLEAN_RVIZ_DEBUG 1      // 청소 영역 시각화 도구 디버거

const double EXPLORE_GOAL_MARGIN = 0.5;

const char* const THREAD_ID_MESSAGE_QUEUE       = "Message Queue";
const char* const THREAD_ID_ROS_DATA_PUB        = "ROS Data pub";
const char* const THREAD_ID_ROS_DEBUG_SUB       = "ROS Debug sub";
const char* const THREAD_ID_RBT_PLUS_RV         = "Rbt Plus RV";
const char* const THREAD_ID_RBT_PLUS_FB         = "Rbt Plus FB";
const char* const THREAD_ID_DEBUG_SIMPLIFY      = "DEBUG SIMPLIFY";
const char* const THREAD_ID_SYSTEM_WATCH        = "System Watch";
const char* const THREAD_ID_WAVE_FRONTIER       = "Wave Frontier";
const char* const THREAD_ID_ROS_CALLBACK        = "ROS Callback";
const char* const THREAD_ID_SYSTEM_INTERFACE    = "SystemInterface";
const char* const THREAD_ID_DSTAR_WALL_UPDATE   = "DstarWallUpdate";
const char* const THREAD_ID_DEBUG_CTR           = "Debug ctr";
const char* const THREAD_ID_IMAGE_PROC          = "Image Proc";
const char* const THREAD_ID_PATH_PLANNER        = "path planner";

const char* const THREAD_ID_TEMP_SLAM_MAP_SAVE  = "Temp Map Save";

//******************************************* END DEBUG *****************************************************//


//******************************************* FOR AWS *****************************************************//
#define MAXLINE             10
#define MAXRECT             10
#define MAXVALUE            10
#define MAX_URL_LENGTH      2048
#define MAX_FILE_LENGTH     512

//******************************************* END AWS *****************************************************//