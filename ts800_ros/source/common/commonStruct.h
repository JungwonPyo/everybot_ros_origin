/**
 * @file commonStruct.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <unordered_map>
#include <memory>
#include <list>
#include "ebtypedef.h"
#include "coordinate.h"
#include "define.h"
#include "interfaceStruct.h"
//#include "userInterface.h"

//TS800 사양
/*===============================================
1. TOF 센서 - Cliff Sensor 4point + Side Sensor 2point = TOF 6point
2. BUMPER - LEFT / RIGHT 2point
3. LIDAR SENSOR - YD_LIDAR_S2_Pro
================================================*/

//TS450+LIDAR
/*===============================================
1. TOF 센서 - Cliff Sensor 2point (LEFT/RIGHT)
2. BUMPER - X
3. LIDAR SENSOR - YD_LIDAR_G4
================================================*/

#if ROBOT_MODEL == TS800_WS
	#define LIDAR_SENSOR_HEAD   -165//-165 // 라이다 각도 보정. ydlidar=90, 3i lidar=-165 //#2-1샘플( -2.97 rad = -170.0 deg ) // #3-1샘플(-2.88 rad == -165 deg)
#elif ROBOT_MODEL == Q8_ROBOROCK
    #define LIDAR_SENSOR_HEAD   180 //Q8 roborock version
#elif ROBOT_MODEL == Q8_3i
    #define LIDAR_SENSOR_HEAD   0 //Q8 - 3i version
#else // ROBOT_MODEL == Q8_3I_LiDAR
    #define LIDAR_SENSOR_HEAD   (-126)
#endif


#define RSF_PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * (RSF_PI / 180.0))
#define RAD2DEG(x) ((x)*180./RSF_PI)
#define LIDAR_DIST_BUFF_SIZE 360

#define GYRO_DIV_UNIT	 100

#define ENC_INIT 0
#define ENC_L_SPEED 1
#define ENC_R_SPEED 2
#define ENC_B_SPEED 3

#define MOTOR_L_CUR 1
#define MOTOR_R_CUR 2
#define MOTOR_B_CUR 3


#define CELL_RESOLUTUION 0.05
#define CELL_X 1000
#define CELL_Y 1000
#define CELL_SIZE CELL_X * CELL_Y
#define CELL_GRID_ORG_X CELL_RESOLUTUION * (CELL_X / 2) * -1
#define CELL_GRID_ORG_Y CELL_RESOLUTUION * (CELL_Y / 2) * +1


#define REVERSE_Y_AXIS true // grid map 을 cleanmap에 업데이트할 때, y축이 반전되어 업데이트 되었음.
#define CELL_INDEX_UP(index, cnt)       ((index)+CELL_X*(cnt))
#define CELL_INDEX_DOWN(index, cnt)     ((index)-CELL_X*(cnt))
#if REVERSE_Y_AXIS == false
    #define CELL_INDEX(x, y) (x)+(y)*CELL_X
    #define CELL_INDEX_LEFT(index, cnt)     ((index)-(cnt))
    #define CELL_INDEX_RIGHT(index, cnt)    ((index)+(cnt))
    #define CHECK_INVALID_CELL_INDEX(index) ((index)<0 || (index)>=CELL_SIZE)
#else
    #define CELL_INDEX(x, y) (x)+(CELL_Y-y)*CELL_X
    #define CELL_INDEX_LEFT(index, cnt)     ((index)+(cnt))
    #define CELL_INDEX_RIGHT(index, cnt)    ((index)-(cnt))
    #define CHECK_INVALID_CELL_INDEX(index) ((index)<0 || (index)>=CELL_SIZE)
#endif


#define offsetRangeValue 3

/* hhryu230111 : docking 관련 define */
// charger signal 수신부 는 정면 기준 좌/우 2개로 구성되어 있음
#define SYS_FRONT_RIGHT_IR       0
#define SYS_FRONT_LEFT_IR        1

//system charger signal 데이터 저장 buffer 관리에 따라 설정 해야함 
#define SYS_Q_LENGTH             20
#define SYS_CHARGER_IR_LENGTH	 2 // hhryu230111 : TS800은 충전 신호 수신부가 두개이므로 2개 배열 필요.

#define SYS_MAX_SIGNAL_COUNT 4096
/* docking define end */


#define WATER_SUPPLY_PUMP_INTERVAL 1*SEC_1
#define WATER_SUPPLY_LEVEL1_PERIOD 60*SEC_1
#define WATER_SUPPLY_LEVEL2_PERIOD 30*SEC_1
#define WATER_SUPPLY_DISPOSABLE_PERIOD 15*SEC_1
#if WATER_SUPPLY_TEST > 0
#define WATER_DRAIN_TIMEOUT 60*SEC_1
#else
#define WATER_DRAIN_TIMEOUT 20*SEC_1
#endif

#define RECEIVER_FRONT_LEFT 0
#define RECEIVER_FRONT_RIGHT 1
#define RECEIVER_SIDE_LEFT 2
#define RECEIVER_SIDE_RIGHT 3

#define RECEIVER_LEFT 0b0001
#define RECEIVER_RIGHT 0b0010
#define RECEIVER_LEFT_SIDE 0b0100
#define RECEIVER_RIGHT_SIDE 0b1000
#define RECEIVER_FRONT_ALL (RECEIVER_LEFT | RECEIVER_RIGHT)
#define RECEIVER_SIDE_ALL (RECEIVER_LEFT_SIDE | RECEIVER_RIGHT_SIDE)
#define RECEIVER_ALL (RECEIVER_LEFT | RECEIVER_RIGHT | RECEIVER_LEFT_SIDE | RECEIVER_RIGHT_SIDE)

//각 서비스 ID (코드 제어용.)
enum class E_SERVICE_ID
{
    NONE,
    IDLE,
    CLEAN,
    CHARGING,
    DOCKING,
    EXPLORER,
    UNDOCKING,
    REDOCKING,
    WIFI,    
};
static std::string enumToString(E_SERVICE_ID value) {
    static const std::unordered_map<E_SERVICE_ID, std::string> enumToStringMap = {
        { E_SERVICE_ID::IDLE, "IDLE" },
        { E_SERVICE_ID::CLEAN, "CLEAN" },
        { E_SERVICE_ID::CHARGING, "CHARGING" },
        { E_SERVICE_ID::DOCKING, "DOCKING" },
        { E_SERVICE_ID::EXPLORER, "EXPLORER" },
        { E_SERVICE_ID::UNDOCKING, "UNDOCKING" },
        { E_SERVICE_ID::REDOCKING, "REDOCKING" },
        { E_SERVICE_ID::WIFI,     "WIFI" }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_SERVICE_READY
{
    CHECK_START,
    CHECK_POWER_MODE,
    TEMP_CHECK_LIDAR,
    CHECK_TILT,
    CHECK_LOCALIZE,
    CHECK_SENSOR,
    CHECK_SLAM,
    COMPLETE,
};
static std::string enumToString(E_SERVICE_READY value) {
    static const std::unordered_map<E_SERVICE_READY, std::string> enumToStringMap = {
        { E_SERVICE_READY::CHECK_START, "CHECK_START," },
        { E_SERVICE_READY::CHECK_TILT, "CHECK_TILT," },
        { E_SERVICE_READY::CHECK_POWER_MODE, "CHECK_POWER_MODE," },
        { E_SERVICE_READY::TEMP_CHECK_LIDAR, "TEMP_CHECK_LIDAR" },
        { E_SERVICE_READY::CHECK_SENSOR, "CHECK_SENSOR," },
        { E_SERVICE_READY::CHECK_LOCALIZE, "CHECK_LOCALIZE," },
        { E_SERVICE_READY::CHECK_SLAM, "CHECK_SLAM," },
        { E_SERVICE_READY::COMPLETE, "COMPLETE," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef enum
{
    MODE_ACTIVE,
    MODE_CHARGE,
    MODE_SLEEP,
}E_POWER_MODE;

typedef enum
{
    CELL_TYPE_KNOWN     = 0X01, // 확인 지역
    CELL_TYPE_WALL      = 0X02, // 벽 
    CELL_TYPE_CLEAN     = 0X04, // 청소 함.
    CELL_TYPE_BUMPER    = 0X08, // 범퍼
    CELL_TYPE_CLIFF     = 0X10, // 낙하
    CELL_TYPE_KNOLL     = 0X20, // 둔턱
    CELL_TYPE_AREA      = 0X40, //  AREA
    CELL_TYPE_PATH      = 0X80, // 예약...
}E_CELL_TYPE;

union cell
{
    u8 value;
    struct
    {
        u8 known        : 1;    // 0:notuse     1:known - 탐색 지역
        u8 wall         : 1;    // 0:empty      1:wall
        u8 clean        : 1;    // 0:unclean    1:clean
        u8 bumper       : 1;
        u8 cliff        : 1;
        u8 knoll        : 1;
        u8 area         : 1;    // 0:empty 1:area
        u8 path         : 1;
    }b;
};

union maskCellBound
{
    u8 value;
    struct
    {
        u8 frontleft  : 1;
        u8 front      : 1;
        u8 frontright : 1;
        u8 left       : 1;
        u8 right      : 1;
        u8 backleft   : 1;
        u8 back       : 1;
        u8 backright  : 1;
    }b;
};

enum E_POWER_STATE
{
    ACTIVE = 2,
    CHARGE = 3,
    SLEEP  = 4,
};

static std::string enumToString(E_POWER_STATE value) {
    static const std::unordered_map<E_POWER_STATE, std::string> enumToStringMap = {
        { E_POWER_STATE::ACTIVE, "ACTIVE," },
        { E_POWER_STATE::CHARGE, "CHARGE," },
        { E_POWER_STATE::SLEEP, "SLEEP," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum E_BATTERY_STATE
{
    BATT_VOID,
    BATT_NEED_CHARGE,
    BATT_LOW,
    BATT_MIDDLE,
    BATT_HIGH,
    BATT_FULL,
    BATT_ERROR
};

static std::string enumToString(E_BATTERY_STATE value) {
    static const std::unordered_map<E_BATTERY_STATE, std::string> enumToStringMap = {
        { E_BATTERY_STATE::BATT_VOID, "BATT_VOID," },
        { E_BATTERY_STATE::BATT_NEED_CHARGE, "BATT_NEED_CHARGE," },
        { E_BATTERY_STATE::BATT_LOW, "BATT_LOW," },
        { E_BATTERY_STATE::BATT_MIDDLE, "BATT_MIDDLE," },
        { E_BATTERY_STATE::BATT_HIGH, "BATT_HIGH," },
        { E_BATTERY_STATE::BATT_FULL, "BATT_FULL," },
        { E_BATTERY_STATE::BATT_ERROR, "BATT_ERROR," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum{
	E_SELFTEST_KEY_INIT,
	E_SELFTEST_KEY_START,
	E_SELFTEST_KEY_STOP
};

typedef enum {
	AXIS_ACCEL_X	= 0x01,
	AXIS_ACCEL_Y	= 0x02,
	AXIS_ACCEL_Z	= 0x04,
	AXIS_GYRO_X		= 0x10,
	AXIS_GYRO_Y		= 0x20,
	AXIS_GYRO_Z		= 0x40,
	AXIS_ACCEL_ALL	= 0x07,
	AXIS_GYRO_ALL	= 0x70,
	AXIS_ALL		= 0x77,
} E_AXIS_TYPE;



enum class E_TASK_ID{
    IDLE,
    EXPLORER,
    CLEAN,
    DOCKING,
    CHARGING,
    UNDOCKING,
	APP_CONNECT,
    SIZE,
};

static std::string enumToString(E_TASK_ID value) {
    static const std::unordered_map<E_TASK_ID, std::string> enumToStringMap = {
        { E_TASK_ID::IDLE, "IDLE,"},
        { E_TASK_ID::EXPLORER, "EXPLORER,"},
        { E_TASK_ID::CLEAN, "CLEAN,"},
        { E_TASK_ID::DOCKING, "DOCKING,"},
        { E_TASK_ID::CHARGING, "CHARGING,"},
        { E_TASK_ID::UNDOCKING, "UNDOCKING,"},
        { E_TASK_ID::APP_CONNECT, "APP_CONNECT,"},
        { E_TASK_ID::SIZE, "SIZE,"}
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_TASK_STATE{
    START,
    RUN,
    PAUSE,
};

static std::string enumToString(E_TASK_STATE value) {
    static const std::unordered_map<E_TASK_STATE, std::string> enumToStringMap = {
        { E_TASK_STATE::START, "START,"},
        { E_TASK_STATE::RUN, "RUN,"},
        { E_TASK_STATE::PAUSE, "PAUSE,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_WALLTRACK_DIR
{
	LEFT,
	RIGHT
};
static std::string enumToString(E_WALLTRACK_DIR value) {
    static const std::unordered_map<E_WALLTRACK_DIR, std::string> enumToStringMap = {
        { E_WALLTRACK_DIR::LEFT, "LEFT," },                
        { E_WALLTRACK_DIR::RIGHT, "RIGHT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}



typedef enum {
	IMU_INIT		= 0x00,
	IMU_BUSY		= 0x01,
	IMU_READY		= 0x02,
} E_IMU_STATUS;
static std::string enumToString(E_IMU_STATUS value) {
    static const std::unordered_map<E_IMU_STATUS, std::string> enumToStringMap = {
        { E_IMU_STATUS::IMU_INIT, "IMU_INIT,"},
        { E_IMU_STATUS::IMU_BUSY, "IMU_BUSY,"},
        { E_IMU_STATUS::IMU_READY, "IMU_READY,"}
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef enum {
	TOF_INIT		= 0,
	TOF_READY		= 4,
	TOF_ERROR		= 99,
} E_TOF_STATUS;

typedef enum
{
    WATER_DRY           = 0,
    WATER_DISPOSABLE    = 1,
    WATER_LEVEL_1       = 2,
	WATER_LEVEL_2       = 3,
	WATER_LEVEL_ABNORMAL	= 0XFE,
	WATER_LEVEL_VOID	= 0XFF,
} E_WATER_PUMP_STEP;

static std::string enumToString(E_WATER_PUMP_STEP value) {
    static const std::unordered_map<E_WATER_PUMP_STEP, std::string> enumToStringMap = {
        { E_WATER_PUMP_STEP::WATER_DRY, "WATER_DRY,"},
        { E_WATER_PUMP_STEP::WATER_DISPOSABLE, "WATER_DISPOSABLE,"},
        { E_WATER_PUMP_STEP::WATER_LEVEL_1, "WATER_LEVEL_1,"},
        { E_WATER_PUMP_STEP::WATER_LEVEL_2, "WATER_LEVEL_2,"},
        { E_WATER_PUMP_STEP::WATER_LEVEL_ABNORMAL, "WATER_LEVEL_ABNORMAL,"},
        { E_WATER_PUMP_STEP::WATER_LEVEL_VOID, "WATER_LEVEL_VOID,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}
typedef enum
{
    DRY_LEVEL_0 = 0,
    DRY_LEVEL_1 = 1,
    DRY_LEVEL_2 = 2,
    DRY_LEVEL_3 = 3,

}E_DRYFAN_LEVEL;

static std::string enumToString(E_DRYFAN_LEVEL value) {
    static const std::unordered_map<E_DRYFAN_LEVEL, std::string> enumToStringMap = {
        { E_DRYFAN_LEVEL::DRY_LEVEL_0, "DRY_LEVEL_0,"},
        { E_DRYFAN_LEVEL::DRY_LEVEL_1, "DRY_LEVEL_1,"},
        { E_DRYFAN_LEVEL::DRY_LEVEL_2, "DRY_LEVEL_2,"},
        { E_DRYFAN_LEVEL::DRY_LEVEL_3, "DRY_LEVEL_3,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_TILTING_CONTROL
{
    STOP,
    UP,
    DOWN
};

static std::string enumToString(E_TILTING_CONTROL value) {
    static const std::unordered_map<E_TILTING_CONTROL, std::string> enumToStringMap = {
        { E_TILTING_CONTROL::STOP, "STOP,"},
        { E_TILTING_CONTROL::UP, "UP,"},
        { E_TILTING_CONTROL::DOWN, "DOWN,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef enum{   // swshin, 20210713 : Volume 조절을 위한 Level
    VOL_STEP_MUTE    = 0, 
    VOL_STEP_1  = 1,
    VOL_STEP_2  = 2,
    VOL_STEP_3  = 3,				// 2021.07.21 dskim volume 단계 3, 4, 5 추가 
    VOL_STEP_4  = 4,
    VOL_STEP_5  = 5,
	VOL_STEP_ABNORMAL = 0xFE,
	VOL_STEP_VOID	= 0xFF
} E_VOLUME_STEP;

typedef enum{   
	
	LANGUAGE_VOID	  = 0x00,
    LANGUAGE_KOREAN   = 0x01, 
    LANGUAGE_ENGLISH  = 0x02, 
	LANGUAGE_ABNORMAL = 0xFF

} E_SOUND_LANGUAGE,E_IoT_SOUND_LANGUAGE;

typedef enum
{
    MOP_DRY,
    WATER_DRAIN,
    CLEAN_BATTERY,
}EAutoDisplayID;

typedef enum
{
    DIR_VOID,
    CENTER_LEFT,
    CENTER_RIGHT,
    LED_ALL_ON,
    LED_END
    
}E_LED_DIRECTION;

typedef enum
{
    LED_VOID,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
    LED_OFF

}E_LED_COLOR;

// 디스플레이 재생 상태
enum class E_DISPLAY_STATE
{
    STOP,       // 정지 상태이거나, 재생이 완료 된 경우
    PLAYING,    // 재생 중인 경우
};

//사운드 재생 상태
enum class E_SOUND_STATE
{
    STOP,
    PLAYING,
};

typedef enum 
{
    INTERFACE_VOID,

    CONNECT_AP,
    DISCONNECT_AP,
    CHECK_CONNECTING_STATE_AP,

    AP_STATE,
    AP_STATES,

    CONNECT_STATION,
    DISCONNECT_STAION,
    CHECK_CONNECTING_STATE_STATION,

    STATION_STATE,
    STATION_STATES,

    CONNECTION,
    CONNECTION_INTERRUPT,
    EXIT_ALL,

    
    DISCONNECT_AWS,
    //사용 될지도 모르는 것
    STOP_BOOTTING_CONNECT,
    MODE,
    CONNECT_AWS,

}E_PHONE_INTERFACE_TYPE;

static std::string enumToString(E_PHONE_INTERFACE_TYPE value) {
    static const std::unordered_map<E_PHONE_INTERFACE_TYPE, std::string> enumToStringMap = {
        { E_PHONE_INTERFACE_TYPE::INTERFACE_VOID, "INTERFACE_VOID,"},
        { E_PHONE_INTERFACE_TYPE::CONNECT_AP, "CONNECT_AP,"},
        { E_PHONE_INTERFACE_TYPE::DISCONNECT_AP, "DISCONNECT_AP,"},
        { E_PHONE_INTERFACE_TYPE::CHECK_CONNECTING_STATE_AP, "CONNECT_AP,"},
        { E_PHONE_INTERFACE_TYPE::AP_STATE, "AP_STATE,"},
        { E_PHONE_INTERFACE_TYPE::AP_STATES, "AP_STATES,"},
        { E_PHONE_INTERFACE_TYPE::CONNECT_STATION, "CONNECT_STATION,"},
        { E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION, "DISCONNECT_STAION,"},
        { E_PHONE_INTERFACE_TYPE::CHECK_CONNECTING_STATE_STATION, "CHECK_CONNECTING_STATE_STATION,"},
        { E_PHONE_INTERFACE_TYPE::STATION_STATE, "STATION_STATE,"},
        { E_PHONE_INTERFACE_TYPE::STATION_STATES, "STATION_STATES,"},
        { E_PHONE_INTERFACE_TYPE::CONNECTION, "CONNECTION,"},
        { E_PHONE_INTERFACE_TYPE::CONNECTION_INTERRUPT, "CONNECTION_INTERRUPT,"},
        { E_PHONE_INTERFACE_TYPE::MODE, "MODE,"},
        { E_PHONE_INTERFACE_TYPE::CONNECT_AWS, "CONNECT_AWS,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}
/**
 * @brief UI에서 wifi 단계를 구분해주는 enum class
 * 
 */
enum class E_WIFI_STEP
{
    E_WIFI_VOID,
    E_ENTERING,
    E_CONNECTING,
    E_COMPLETE_WIFI,
    E_COMPLETE_PHONE,
    E_CANCLE_WIFI,
    E_FAIL_WIFI,

};

/**
 * @brief when no singal docking, robot is moving step 
 * 
 */
enum class E_CHECK_SIG_STEP
{
    IDLE,
    START,
    LEFT_QUARTER_TURN,
    RIGHT_QUARTER_TURN,
    FULL_TURN,
    END,
};

static std::string enumToString(E_CHECK_SIG_STEP value)
{
    static const std::unordered_map<E_CHECK_SIG_STEP, std::string> enumToStringMap = {
        {E_CHECK_SIG_STEP::IDLE, "IDLE"},
        {E_CHECK_SIG_STEP::START, "START"},
        {E_CHECK_SIG_STEP::LEFT_QUARTER_TURN, "LEFT_QUARTER_TURN"},
        {E_CHECK_SIG_STEP::RIGHT_QUARTER_TURN, "RIGHT_QUARTER_TURN"},
        {E_CHECK_SIG_STEP::FULL_TURN, "FULL_TURN"},
        {E_CHECK_SIG_STEP::END, "END"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end())
    {
        return it->second;
    }
    else
    {
        return "Unknown";
    }
}





typedef enum
{
	BATTERY_OFF = 0,
	BATTERY_Low_extreme = 1, 
	BATTERY_Low = 2,		//bykim:211111: 청소가 불가한 배터리 레벨 (battery low부터 )
	BATTERY_Mid = 3,
	BATTERY_Hig = 4
} E_BATTERY_STEP;

typedef enum
{
    wf_void=0,
    wf_left=1,
    wf_right=2
    
} E_WALLFACE_ID;

typedef enum
{
    ti_void=-1,                         // TRAKCING 모드 아님
    ti_charger=0,                       // 충전기 신호 트랙킹 모드 ( Multi-Signal tracking )
    
} E_TRACKING_ID ;  

typedef enum
{
    slam_pause = 1,                         // TRAKCING 모드 아님
    slam_resume = 2,                       // 충전기 신호 트랙킹 모드 ( Multi-Signal tracking )
    slam_exit = 3,                       // 충전기 신호 트랙킹 모드 ( Multi-Signal tracking )
    slam_start = 4,                       // 충전기 신호 트랙킹 모드 ( Multi-Signal tracking )
} E_SLAME_CONTROL ;  

typedef union
{
    u32      keyvalue;
    struct
    {
        u32 autoclean		:1;		  // auto
        u32 spotclean		:1;		  // spot
        u32 wallclean		:1;		  // wall
        u32 StartStop		:1;		  // stop	

        u32 Left 			:1;		   //
        u32 Right			:1;		   //
        u32 Up   			:1;		   //
        u32 Down 			:1;		   //

        u32 VirtualStop   	:1;
        u32 Homekey			:1;
        u32 IsWiFikey		:1;
        u32 ResetUserOption	:1;

        //u32 suctionPower    :1;
        //u32 suctionStep     :1;
        u32 WaterSelect   	:1;		   
        u32 DrainWater		:1;
        u32 menuMode        :1;

        u32 tiltup          :1;
        u32 tiltdown        :1;
        u32 tiltstop        :1;

        u32 pumpon          :1;
        u32 pumpoff         :1;

        u32 menuLeft        :1;
        u32 menuRight       :1;
        u32 menuSelect      :1;   

        u32 ReservSet		:8;
    }b;
}tKey_ts800;

typedef union
{
    u32      keyvalue;
    struct
    {
        u32 autoclean		:1;		  // auto
        u32 spotclean		:1;		  // spot
        u32 wallclean		:1;		  // wall
        u32 StartStop		:1;		  // stop	

        u32 Left 			:1;		   //
        u32 Right			:1;		   //
        u32 Up   			:1;		   //
        u32 Down 			:1;		   //

        u32 VirtualStop   	:1;
        u32 Homekey			:1;
        u32 IsWiFikey		:1;
        u32 ResetUserOption	:1;

        u32 suctionPower    :1;
        u32 suctionStep     :1;
        u32 WaterSelect   	:1;		   
        //u32 DrainWater		:1;   

        u32 ReservSet		:17;
    }b;
}tKey_q8;

#if ROBOT_MODEL == TS800_WS
typedef tKey_ts800 tKey;
#elif ROBOT_MODEL == Q8_3I
typedef tKey_q8 tKey;
#endif

typedef struct
{
    int timeout;
    //int iteration;
    bool autorun;
    E_DRYFAN_LEVEL level;

}tDryMopOption;

typedef enum 
{ 
    FRONT_STATE_OPEN,
    FRONT_STATE_APPROACH,
    FRONT_STATE_OBSTACLE,

}FRONT_OSBATALCE_STATE;

typedef struct tFrontData
{
    FRONT_OSBATALCE_STATE state;
    u16 raw_data;
    u16 lpfData;
    u16 old_lpfData;
    s16 slope;
    s16 maxSlope;

}tFrontData;

typedef struct tFrontIr
{
    tFrontData left;
    tFrontData center;
    tFrontData right;
}tFrontIr;

typedef struct tTofData
{
    tSysTofInfo rcliff;
    tSysTofInfo lcliff;
    tSysTofInfo leftwall;
    tSysTofInfo rightwall;
    tSysTofInfo knoll;

    u16         leftCalibAvg; 	//낙하 Tof센서의 STANBY 상태 accumulate avg 정보 // 1초동안 50회 데이터 평균을 구한다.
    u16         rightCalibAvg;	//낙하 Tof센서의 STANBY 상태 accumulate avg 정보 // 1초동안 50회 데이터 평균을 구한다.

}tTofData;

// typedef struct
// {
// 	tSysTofInfo		knoll;
// 	tSysTofInfo    	wall;
// 	tSysTofInfo		left;
// 	tSysTofInfo		right;

// }RSF_TOF_DATA;

typedef union
{
    u8 value;
    struct
    {
        u8 fright_side : 1;
        u8 fright_Top_side : 1;
        u8 fright_Top_center : 1;
        u8 fright_center : 1;
        u8 fleft_center : 1;
        u8 fleft_Top_center : 1;
        u8 fleft_Top_side : 1;
        u8 fleft_side : 1;

    } b;

} RSF_OBSTACLE_MASK;
static std::string enumToString(RSF_OBSTACLE_MASK value) {
    std::string ret = "flag : ";

    if (value.b.fright_side){ret += "fright_side";}
    if (value.b.fright_Top_side){ret += " ,fright_Top_side";}
    if (value.b.fright_Top_center){ret += " ,fright_Top_center";}
    if (value.b.fright_center){ret += " ,fright_center";}
    if (value.b.fleft_center){ret += " ,fleft_center";}
    if (value.b.fleft_Top_center){ret += " ,fleft_Top_center";}
    if (value.b.fleft_Top_side){ret += " ,fleft_Top_side";}
    if (value.b.fleft_side){ret += " ,fleft_side";}

    return ret;
}

typedef struct
{
    RSF_OBSTACLE_MASK obstacle; // 방향 별 장애물 감지 flag
    RSF_OBSTACLE_MASK approach;

} LIDAR_RSF_OBSTACLE, *PRSF_LIDAR_OBSTACLE; // SENSOR DATA FORMAT

typedef struct
{
    LIDAR_RSF_OBSTACLE  lidar;
    LIDAR_RSF_OBSTACLE  left_wall;
    LIDAR_RSF_OBSTACLE  right_wall;
	LIDAR_RSF_OBSTACLE	front;
    RSF_OBSTACLE_MASK   cliff;
    RSF_OBSTACLE_MASK   bumper;
    RSF_OBSTACLE_MASK   trap;

	tTofData			tof;
	tSysIMU				imu;
	tFrontIr			ir;
    tSysWheelMotor      wheel;

    bool isLift;
    bool approach;

}RSU_OBSTACLE_DATA;

//시스템 전원 상태.
typedef enum {
    SYS_POWER_INIT = 0,
    SYS_POWER_ACTIVE    = 1,
    SYS_POWER_STANDBY   = 2,
    SYS_POWER_CHARGE    = 3,
    SYS_POWER_SLEEP     = 4
}E_SYS_POWER_STATE;


/**
 * @brief 시간을 체크하는 구조체...
 * 초회는 무조건 실행하는 변수를 추가해야할지? 
 * 아니면 초기화를 (u32)-1로 할지...?
 */
typedef struct _tCheckTime
{
    u32 start; // 시작한 시간 저장....
    u32 check; // 체크해야하는 시간
}tCheckTime;

enum class E_DRY_MOP
{
    ON,
    OFF
};

/**
 * @date 2023/08/02 hhryu
 * @brief 양쪽 바퀴 속력을 저장하는 구조체
 */
typedef struct _tPairSpeed
{
    double left;
    double right;
}tPairSpeed;

/**
 * @date 2023/08/02 hhryu
 * @brief 한 점의 좌표 (실제 맵 좌표를 저장하는 tPoint와 구분하였음.)
 * 
 */
typedef struct tPoint1D
{
    double x;
    double y;
}tPoint1D;

/**
 * @date 2023/08/02 hhryu
 * @brief 두 점의 좌표 (실제 맵 좌표를 저장하는 tPoint와 구분하였음.)
 * 
 */
typedef struct tPoint2D
{
    tPoint1D min;
    tPoint1D max;
}tPoint2D;

/**
 * @date 2023/08/02 hhryu
 * @brief 어떠한 범위의 최솟값, 최댓값 저장. 
 */
typedef struct _tDoubleRange
{
    double min;
    double max;
}tDoubleRange;

/**
 * @date 2023/08/02 hhryu
 * @brief 어떠한 범위의 최솟값, 최댓값 저장. 
 */
typedef struct _tS16Range
{
    s16 min;
    s16 max;
}tS16Range;

/**
 * @brief 모션 컨트롤러에 대한 프로파일 값입니다.
 * 
 * @param setHeadingStartPointFlag 시작위치에서 목표점으로 헤딩을 맞출 것인지 여부.
 * @param setHeadingEndPointFlag 목표위치에서 목표각도로 헤딩을 맞출 것인지 여부.
 * @param minLinVel 최소 선속도 (단위 m/s)
 * @param desLinVel 최대 각속도 (단위 m/s)
 * @param minAngVel 최소 각속도 (단위 rad/s)
 * @param desAngVel 최대 각속도 (단위 rad/s)
 * @param distanceToDecel (선속도에서) 감속을 시작할 거리 (단위 m)
 * @param angleToDecel (각속도에서) 감속을 시작할 각도 (단위 rad)
 * @param linearAccel (선속도에서) 가감속값 (단위 m/s^2)
 * @param angularAccel (각속도에서) 가감속값 (단위 rad/s^2)
 */
typedef struct tMotionControllerProfile
{
    tMotionControllerProfile(
        double _minLinVel=0.1, // m/s0.06
        double _minCLinVel = 0.1,
        double _desLinVel=MOTION_CONTROLLER_DESIRED_V,
        double _minAngVel= DEG2RAD(20.0), // rad/s
        double _minCAngVel= DEG2RAD(5.0),
        double _desAngVel=MOTION_CONTROLLER_DESIRED_W,
        double _distanceToDecel=0.10,
        double _angleToDecel=DEG2RAD(20.0),
        double _linearAccel = 0.05,//0.002,
        double _linearDecel = 0.025,
        double _curveAccel = 0.005,//0.002,
        double _curveDecel = 0.01,
        double _linearTargetMargin=0.025,
        double _angularAccel=DEG2RAD(1.0),
        double _angularDecel=DEG2RAD(5.0),
        double _curvAcc = DEG2RAD(1.0),
        bool _isStopAtTarget=true,
        bool _explorer = false) :
        minLinVel{_minLinVel},
        minCLinVel{_minCLinVel},
        desLinVel{_desLinVel},
        minAngVel{_minAngVel},
        minCAngVel{_minCAngVel},
        desAngVel{_desAngVel},
        distanceToDecel{_distanceToDecel},
        angleToDecel{_angleToDecel},
        linearAccel{_linearAccel},
        linearDecel{_linearDecel},
        curveAccel{_curveAccel},
        curveDecel{_curveDecel},
        linearTargetMargin{_linearTargetMargin},
        angularAccel{_angularAccel},
        angularDecel{_angularDecel},
        curvAcc{_curvAcc},
        isStopAtTarget{_isStopAtTarget},
        explorer{_explorer}
        {}

    double minLinVel; // 최소 선속도 (단위 m/s)
    double minCLinVel; // 최소 선속도 (단위 m/s)
    double desLinVel; // 최대 선속도 (단위 m/s)
    double minAngVel; // 최소 각속도 (단위 rad/s)
    double minCAngVel;
    double desAngVel; // 목표 각속도 (단위 rad/s)
    double distanceToDecel; // (선속도에서) 감속을 시작할 거리 (단위 m)
    double angleToDecel; // (각속도에서) 감속을 시작할 각도 (단위 rad)
    double linearAccel; // (선속도에서) 가속값 (단위 m/s^2)
    double linearDecel; // (선속도에서) 감속값 (단위 m/s^2)
    double curveAccel; // (선속도에서) 가속값 (단위 m/s^2)
    double curveDecel; // (선속도에서) 감속값 (단위 m/s^2)
    double linearTargetMargin; // (선속도에서) 목표도달 마진값 (단위 m)
    double angularAccel; // (각속도에서) 가속값 (단위 rad/s^2)
    double angularDecel; // (각속도에서) 감속값 (단위 rad/s^2)
    double curvAcc; // (각속도에서) 가감속값 (단위 rad/s^2)
    bool isStopAtTarget; // 목표에 도착했을 때, 정지을 할지
    bool explorer;
}tProfile;

enum class E_ERROR_TYPE
{
  NO_ERROR,
  
  TRAP_OBSTACLE,              /* 장애물 구속 에러 :센서창을 확인해 주세요*/

  BUMPER_CURRENT,

  BATTERY_ADAPTER,            /* 충전 에러 : 충전이 불가합니다. 서비스 센터에 문의해 주세요. */
  BATTERY_NOSET,              /* 충전 에러 : 충전이 불가합니다. 서비스 센터에 문의해 주세요. */
  BATTERY_ERROR,              /* 충전 에러 : 충전이 불가합니다. 서비스 센터에 문의해 주세요. */

  WIFI,

  ADC_CRITICAL,               /* ADC에러    : 오류가 발생했습니다.서비스센터로 문의해주세요. */
  MCU_CRITICAL,               /* MCU에러    : 오류가 발생했습니다.서비스센터로 문의해주세요. */
  INTERRUPT_CRITICAL,         /* 인터럽트 에러  : 오류가 발생했습니다.서비스센터로 문의해주세요. */
  COMMUNICATION,              /* TOF 통신 에러  : 오류가 발생했습니다.서비스센터로 문의해주세요. */
  
  UNDEFINED,

  LIFT_CLIFF,                 /* 제품 들림        : 평평한 바닥에서 다시 시작해 주세요. */
  GYRO_INIT_FAIL,             /* 자이로초기화에러  : 평평한 바닥에서 다시 시작해 주세요. */
  TRAP_CLIFF,                 /* 낙하 에러      : 평평한 바닥에서 다시 시작해 주세요. */

  WHEEL_CURRENT,              /* 회전판 이물 걸림 : 로봇의 걸레 장착과 바닥면을 확인해주세요. */
  WHEEL_WRONG_MOP,            /* 걸레 오창작  :    로봇의 걸레 장착과 바닥면을 확인해주세요. */
	NO_MOP_OR_TOF_OBS,          /* 청소시작전체크  :    로봇의 걸레 장착과 바닥면을 확인해주세요. */
  
  TILT,                       /* 틸팅 에러 : 틸팅 state가 error (과전류 등)*/

  DOCKING_FAIL,               /* 충전신호 미감지에러: 충전기로 이동할 수 없습니다. 위치를 확인해 주세요. */

  BATTERY_LOW_CANNOT_MOVE,    /* 자동도킹중 배터리부족 : 배터리가 부족하여 충전기로 이동할 수 없습니다. */
  
  MISSING_LIDAR_DATA,         /* ros 데이터 실종 - LiDAR */
  MISSING_GRIDMAP_DATA,       /* ros 데이터 실종 - GridMap */
};
static std::string enumToString(E_ERROR_TYPE value) {
    static const std::unordered_map<E_ERROR_TYPE, std::string> enumToStringMap = {
        { E_ERROR_TYPE::NO_ERROR, "NO_ERROR," },
        { E_ERROR_TYPE::TRAP_OBSTACLE, "TRAP_OBSTACLE," },
        { E_ERROR_TYPE::BUMPER_CURRENT, "BUMPER_CURRENT," },
        { E_ERROR_TYPE::BATTERY_ADAPTER, "BATTERY_ADAPTER," },
        { E_ERROR_TYPE::BATTERY_NOSET, "BATTERY_NOSET," },
        { E_ERROR_TYPE::BATTERY_ERROR, "BATTERY_ERROR," },
        { E_ERROR_TYPE::WIFI, "WIFI," },
        { E_ERROR_TYPE::ADC_CRITICAL, "ADC_CRITICAL," },
        { E_ERROR_TYPE::MCU_CRITICAL, "MCU_CRITICAL," },
        { E_ERROR_TYPE::INTERRUPT_CRITICAL, "INTERRUPT_CRITICAL," },
        { E_ERROR_TYPE::COMMUNICATION, "COMMUNICATION," },
        { E_ERROR_TYPE::UNDEFINED, "UNDEFINED," },
        { E_ERROR_TYPE::LIFT_CLIFF, "LIFT_CLIFF," },
        { E_ERROR_TYPE::GYRO_INIT_FAIL, "GYRO_INIT_FAIL," },
        { E_ERROR_TYPE::TRAP_CLIFF, "TRAP_CLIFF," },
        { E_ERROR_TYPE::WHEEL_CURRENT, "WHEEL_CURRENT," },
        { E_ERROR_TYPE::WHEEL_WRONG_MOP, "WHEEL_WRONG_MOP," },
        { E_ERROR_TYPE::NO_MOP_OR_TOF_OBS, "NO_MOP_OR_TOF_OBS," },
        { E_ERROR_TYPE::TILT, "TILT," },
        { E_ERROR_TYPE::DOCKING_FAIL, "DOCKING_FAIL," },
        { E_ERROR_TYPE::BATTERY_LOW_CANNOT_MOVE, "BATTERY_LOW_CANNOT_MOVE," },
        { E_ERROR_TYPE::MISSING_LIDAR_DATA, "MISSING_LIDAR_DATA," },
        { E_ERROR_TYPE::MISSING_GRIDMAP_DATA, "MISSING_GRIDMAP_DATA," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_AWS_MSG_SORT
{
    WATER_LEVEL,
    SOUND_LEVEL,
    CLEAN_MODE,
    LANGUAGE,
    COUNTRY,
    BATTERY,
    DRY_MOP,
    ERROR,
    SETTTING,
    OPERATION_AREA,
    FORBIDDEN_AREA,
    DISTRUBT_MODE,
    RESERVATION_CLEAN,
    SAVE_MAP_INFO,
    MAP_DATA,
    AREA_INFO,
    DIVIDE_AREA,
    COMBINE_AREA,
    CLEAN_HISTORY,
    ROBOT_POSE,
    OTA_VERSION,
};

static std::string enumToString(E_AWS_MSG_SORT value) {
    static const std::unordered_map<E_AWS_MSG_SORT, std::string> enumToStringMap = {
        { E_AWS_MSG_SORT::WATER_LEVEL, "WATER_LEVEL," },
        { E_AWS_MSG_SORT::SOUND_LEVEL, "SOUND_LEVEL," },
        { E_AWS_MSG_SORT::CLEAN_MODE, "CLEAN_MODE," },
        { E_AWS_MSG_SORT::LANGUAGE, "LANGUAGE," },
        { E_AWS_MSG_SORT::COUNTRY, "COUNTRY," },
        { E_AWS_MSG_SORT::BATTERY, "BATTERY," },
        { E_AWS_MSG_SORT::DRY_MOP, "DRY_MOP," },
        { E_AWS_MSG_SORT::SETTTING, "SETTTING," },
        { E_AWS_MSG_SORT::OPERATION_AREA, "OPERATION_AREA," },
        { E_AWS_MSG_SORT::FORBIDDEN_AREA, "FORBIDDEN_AREA," },
        { E_AWS_MSG_SORT::DISTRUBT_MODE, "DISTRUBT_MODE," },
        { E_AWS_MSG_SORT::RESERVATION_CLEAN, "RESERVATION_CLEAN," },
        { E_AWS_MSG_SORT::SAVE_MAP_INFO, "SAVE_MAP_INFO," },
        { E_AWS_MSG_SORT::MAP_DATA, "MAP_DATA," },
        { E_AWS_MSG_SORT::AREA_INFO, "AREA_INFO," },
        { E_AWS_MSG_SORT::DIVIDE_AREA, "DIVIDE_AREA," },
        { E_AWS_MSG_SORT::COMBINE_AREA, "COMBINE_AREA," },
        { E_AWS_MSG_SORT::CLEAN_HISTORY, "CLEAN_HISTORY," },
        { E_AWS_MSG_SORT::ROBOT_POSE, "ROBOT_POSE," },
        { E_AWS_MSG_SORT::OTA_VERSION, "OTA_VERSION," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


enum E_UPDATE_FLAG
{
    REPORT,   // 어플로 인해 변경된 상태
    UPDATE, // 로봇 자체적으로 변경된 상태 
};
 
enum E_CONNECT_SORT
{
    BTN_CONNECT,   // 버튼으로 인한 HOME AP(공유기) 연결
    BOOTING_CONNECT, // 부팅으로 인한 HOME AP(공유기) 연결
    INTERRUPT_CONNECT, // 비정상 종료 및 연결 실패 HOME AP(공유기) 연결
    CLEAR_CONNECT,
};

enum E_AP_STAION_CONNECTION_STATE
{
    COMPLETE_OR_VOID, //연결 완료후 station 포인터 삭제 상태 또는 아무것도 안한 상태
    FAIL,       //연결 실패
    CONNECTING, //연결 중
    COMPLETE,   //연결 완료
};

enum STATION_MODE_BOOTING_STEP
{
    BOOTTING_VOID,      //부팅 연결 처음 단계
    STATION_CONNECTING, //부팅 연결 Station 연결
    STATION_FAIL,       //부팅 연결 station 연결 실패
    STATION_COMPLETE,   //부팅 연결 station 연결 성공
    BOOTTING_CONNECT_AWS,        //부팅 연결 AWS 연결 
    BOOTING_PROC_FIN,   //부팅 절차 완료
};

// 처리 기준의 에러 종류
enum class E_ERROR_HANDLING_TYPE
{
    NONE,
    GYRO_SENSOR_INIT_FAIL,  // 자이로 센서 초기화 실패 에러
    TOF_SENSOR_INIT_FAIL,   // TOF 센서 초기화 실패 에러
    CONTINUOUS_CLIFF,       // 연속 낙하 감지 에러
    ROBOT_LIFTING,          // 제품 들림 감지 에러
    MOTOR_ROTATION,         //모터 회전부 에러
    BUMPER,                 // 범퍼 에러
    TRAP_OBSTACLE,          // 장애물 구속으로 인한 에러
    CHARGING_ERROR,         //충전 불가
    WIFI_CONNECTION,        //와이 연결 불가
    COMMUNICATION,           // 시스템 통신 에러.
    ETC_ERROR,              //기타(알수 없는 에러)
    DOCKING_FIND,           //충전기 못 찾음, 없음 에러
    DOCKING_FAIL,           //도킹 불가 에러
    SENSOR_BLUR,            //센서창 가림 레어
    TILT,                   // Tilt 에러
    WHEEL_ENCODER,           // 바퀴 장애물 에러
    CRADLE_ERROR,           //크래들 이상 에러
    WATER_SUPPLY,           //물공급 불가 에러
    MISSING_ROS_DATA,       // ROS 데이터 미수신 에러, 라이다 에러
    CMD4,
};


static std::string enumToString(E_ERROR_HANDLING_TYPE value) {
    static const std::unordered_map<E_ERROR_HANDLING_TYPE, std::string> enumToStringMap = {
        { E_ERROR_HANDLING_TYPE::GYRO_SENSOR_INIT_FAIL, "GYRO_SENSOR_INIT_FAIL," },
        { E_ERROR_HANDLING_TYPE::TOF_SENSOR_INIT_FAIL, "TOF_SENSOR_INIT_FAIL," },
        { E_ERROR_HANDLING_TYPE::CONTINUOUS_CLIFF, "CONTINUOUS_CLIFF," },
        { E_ERROR_HANDLING_TYPE::ROBOT_LIFTING, "ROBOT_LIFTING," },
        { E_ERROR_HANDLING_TYPE::MOTOR_ROTATION, "MOTOR_ROTATION," },
        { E_ERROR_HANDLING_TYPE::BUMPER, "BUMPER," },
        { E_ERROR_HANDLING_TYPE::TRAP_OBSTACLE, "TRAP_OBSTACLE," },
        { E_ERROR_HANDLING_TYPE::CHARGING_ERROR, "CHARGING_ERROR," },
        { E_ERROR_HANDLING_TYPE::WIFI_CONNECTION, "WIFI_CONNECTION," },
        { E_ERROR_HANDLING_TYPE::COMMUNICATION, "COMMUNICATION," },
        { E_ERROR_HANDLING_TYPE::ETC_ERROR, "ETC_ERROR," },
        { E_ERROR_HANDLING_TYPE::DOCKING_FIND, "DOCKING_FIND," },
        { E_ERROR_HANDLING_TYPE::DOCKING_FAIL, "DOCKING_FAIL," },
        { E_ERROR_HANDLING_TYPE::SENSOR_BLUR, "SENSOR_BLUR," },
        { E_ERROR_HANDLING_TYPE::TILT, "TILT," },
        { E_ERROR_HANDLING_TYPE::WHEEL_ENCODER, "WHEEL_ENCODER," },
        { E_ERROR_HANDLING_TYPE::CRADLE_ERROR, "CRADLE_ERROR," },
        { E_ERROR_HANDLING_TYPE::WATER_SUPPLY, "WATER_SUPPLY," },
        { E_ERROR_HANDLING_TYPE::CMD4, "CMD4," },

    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}


struct tSpot { 
	float x;
	float y;
};
struct tRoom { 
	float x;
	float y;
	float w;
	float h;	
};
struct tCustom { 
	float x;
	float y;
	float w;
	float h;	
};
struct tForbiddenLine{
	float x1;
	float y1;
	float x2;
	float y2;	
};
struct tForbiddenRect{
	float x;
	float y;
	float w;
	float h;	
};

typedef struct {
	int    id;
	double point[4];
}tDivideArea;

typedef struct {
	int    point[2];
}tCombieArea;

// int id;
// short  polygonNum;
// double polygon[100];
// char   name[64];
// char   color[64];
struct tAreaInfo{
	int id;
    short polyganNum;
    //std::list<tPointdouble> polygon;
    double polygon[100];
    char name[64];
    char color[64];
};
struct tCleanSchedule{
	char time[64];
	char weeks[64];
	short mode;
	short waterLevel;
	char areas[64];
	short isEnabled;
	short isValid;
};


typedef struct _tTestMotionInfo
{    
    _tTestMotionInfo():
    isupdate(false) {}

    bool isupdate;
    tProfile profile;
    tPoint target;
    tTwist curVel;
    tTwist desVel;
}tTestMotionInfo;

typedef struct _tPwmDriveInfo
{    
    _tPwmDriveInfo():
    isupdate(false) {}

    bool isupdate;
    tPwmSpeed desVel;
    tPwmSpeed curVel;

}tPwmDriveInfo;


template <typename T>
struct tUpdateData
{
#if 1 // dev코드
    tUpdateData() : bUpdate{false}, data{T()} {}
    bool isUpdate() { return bUpdate; }
    void set(T input)
    {
        bUpdate = true;
        data = input;
    }
    T get()
    {
        bUpdate = false;
        return data;
    }
#else   //app 코드
    tUpdateData() : bUpdate{false}, data{0} {}
    bool isUpdate()     { return bUpdate; }
    void set(T input)   { bUpdate = true; data = input; }
    T get()             { bUpdate = false; return data; }
#endif

private:
    bool bUpdate;
    T data;
};
template <typename T>
struct tUpdatePtr
{
    tUpdatePtr() : bUpdate{false}, data{nullptr} {}
    bool isUpdate() { return bUpdate; }
    void set(T* input)
    {
        bUpdate = true;
        data = input;
    }
    T* get()
    {
        bUpdate = false;
        return data;
    }

private:
    bool bUpdate;
    T* data;
};

template <typename T, int N>
struct tUpdateBuffer
{
    tUpdateBuffer() : bUpdate{false} {}

    bool isUpdate() const { return bUpdate; }

    void set(const  T (&input)[N])
    {
        bUpdate = true;
        std::memcpy(data, input, N * sizeof(T)); // 배열의 데이터를 복사합니다.
    }

    T* get() 
    {
        bUpdate = false;
        return data;
    }

private:
    bool bUpdate;
    T data[N];
};

template <typename T>
struct tUpdatePairData
{
#if 1 // dev코드
    tUpdatePairData() : bUpdate{false}, data{nullptr, 0} {}
    bool isUpdate() { return bUpdate; }
    void set(T* input, short inputNum)
    {
        bUpdate = true;
        data.first = input;
        data.second = inputNum;
    }
    std::pair<T*, short> get()
    {
        bUpdate = false;
        return data;
    }
#else   //app 코드
    tUpdateData() : bUpdate{false}, data{0} {}
    bool isUpdate()     { return bUpdate; }
    void set(T input)   { bUpdate = true; data = input; }
    T get()             { bUpdate = false; return data; }
#endif

private:
    bool bUpdate;
    std::pair<T*, short> data;
};

enum class LINE_DIR
{
    GO_UP,
    GO_DOWN,
    GO_LEFT,
    GO_RIGHT,
};

static std::string enumToString(LINE_DIR value) {
    static const std::unordered_map<LINE_DIR, std::string> enumToStringMap = {
        { LINE_DIR::GO_UP, "GO_UP," },
        { LINE_DIR::GO_DOWN, "GO_DOWN," },
        { LINE_DIR::GO_LEFT, "GO_LEFT," },
        { LINE_DIR::GO_RIGHT, "GO_RIGHT," },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

struct tRobotOption
{   
    tRobotOption() : waterLevel(1), soundLv(1), country(1) {}

    short waterLevel;
    short soundLv;
    short country;
    std::list<tCleanSchedule> cleanSchedule;

};