#pragma once
#include "commonStruct.h"
#include "define.h"

enum class E_PURPOSE_TYPE
{
    NONE,
    MOVING,
    LINE_CLEAN,
    EXPLORER,
    SIGNAL_TRACKING,
    WALL_FOLLOW
};
static std::string enumToString(E_PURPOSE_TYPE value) {
    static const std::unordered_map<E_PURPOSE_TYPE, std::string> enumToStringMap = {
        { E_PURPOSE_TYPE::NONE, "NONE" },
        { E_PURPOSE_TYPE::MOVING, "MOVING" },
        { E_PURPOSE_TYPE::LINE_CLEAN, "EXPLORER" },
        { E_PURPOSE_TYPE::SIGNAL_TRACKING, "SIGNAL_TRACKING" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_PURPOSE_LINE_CLEAN_INFO
{
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    WALL_FOLLOW
};
static std::string enumToString(E_PURPOSE_LINE_CLEAN_INFO value) {
    static const std::unordered_map<E_PURPOSE_LINE_CLEAN_INFO, std::string> enumToStringMap = {
        { E_PURPOSE_LINE_CLEAN_INFO::NONE, "NONE" },
        { E_PURPOSE_LINE_CLEAN_INFO::UP, "UP" },
        { E_PURPOSE_LINE_CLEAN_INFO::DOWN, "DOWN" },
        { E_PURPOSE_LINE_CLEAN_INFO::LEFT, "LEFT" },
        { E_PURPOSE_LINE_CLEAN_INFO::RIGHT, "RIGHT" },
        { E_PURPOSE_LINE_CLEAN_INFO::WALL_FOLLOW, "WALL_FOLLOW" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_ACTION_TYPE
{
    /**
     * @brief 로봇의 헤딩이 특정 좌표(map)를 바라보도록 제자리 회전
     * @param targetPoint 로봇의 헤딩이 바라볼 좌표
     */
    ROTATE_TO_POINT_ON_MAP,

    /**
     * @brief Map 기준으로 원하는 각도로 제자리 회전
     * @param targetAngle 회전시킬 각도. (회전방향은 가까운 각도로 회전합니다.)
     */
    ROTATE_TO_ANGLE_ON_MAP,

    /**
     * @brief 로봇 기준으로 원하는 각도만큼 제자리 회전
     * @param targetAngle 회전시킬 각도량. (+)는 왼쪽(반시계)로 회전
     */
    ROTATE_TO_ANGLE_ON_ROBOT,

    /**
     * @brief 목표없이 원하는 속도로 회전
     */
    ROTATE_ON_VELOCITY,

    /**
     * @brief Map 기준으로 원하는 좌표로 직진 제어
     * @param targetPoint 직진 제어의 목표 좌표
     */
    LINEAR_TO_POINT_ON_MAP,

    /**
     * @brief Map 기준으로 원하는 직진을 추종하는 제어
     * @param startPose   라인 제어의 시작 좌표
     * @param targetPoint 라인 제어의 목표 좌표
     */
    LINEAR_TO_LINE_ON_MAP,

    /**
     * @brief 로봇 기준으로 원하는 거리만큼 직진 이동
     * @param targetDistance 원하는 거리만큼 이동.
     */
    LINEAR_TO_DISTANCE_ON_ROBOT,

    /**
     * @brief 목표없이 원하는 속도로 직진.
     * 
     */
    LINEAR_ON_VELOCITY,

    /**
     * @brief 로봇 기준으로 원하는 각도만큼 곡선 이동.
     * 
     */
    CURVE_TO_ANGLE_ON_ROBOT,

    /**
     * @brief 목표없이 원하는 속도로 곡선 이동.
     * 
     */
    CURVE_ON_VELOCITY,

    /**
     * @brief 목표없이 원하는 속도로 곡선 이동.
     * 
     */
    STOP,

    /**
     * @brief 로봇 틸팅 up을 함.
     * 
     */
    TILTING_UP,

    /**
     * @brief 로봇 틸팅 down을 함.
     */
    TILTING_DOWN,

    DRIVEWHEEL_ON_PWM,

    NONE,
};
static std::string enumToString(E_ACTION_TYPE value) {
    static const std::unordered_map<E_ACTION_TYPE, std::string> enumToStringMap = {
        { E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_ROBOT, "ROTATE_TO_ANGLE_ON_ROBOT" },
        { E_ACTION_TYPE::ROTATE_TO_ANGLE_ON_MAP, "ROTATE_TO_ANGLE_ON_MAP" },
        { E_ACTION_TYPE::ROTATE_TO_POINT_ON_MAP, "ROTATE_TO_POINT_ON_MAP" },
        { E_ACTION_TYPE::ROTATE_ON_VELOCITY, "ROTATE_ON_VELOCITY" },
        { E_ACTION_TYPE::LINEAR_TO_POINT_ON_MAP, "LINEAR_TO_POINT_ON_MAP" },
        { E_ACTION_TYPE::LINEAR_TO_LINE_ON_MAP, "LINEAR_TO_LINE_ON_MAP" },
        { E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT, "LINEAR_TO_DISTANCE_ON_ROBOT" },
        { E_ACTION_TYPE::LINEAR_ON_VELOCITY, "LINEAR_ON_VELOCITY" },
        { E_ACTION_TYPE::CURVE_TO_ANGLE_ON_ROBOT, "CURVE_TO_ANGLE_ON_ROBOT" },
        { E_ACTION_TYPE::CURVE_ON_VELOCITY, "CURVE_ON_VELOCITY" },
        { E_ACTION_TYPE::STOP, "STOP" },
		{ E_ACTION_TYPE::DRIVEWHEEL_ON_PWM, "DRIVEWHEEL_ON_PWM" },
        { E_ACTION_TYPE::TILTING_UP, "TILTING_UP" },
        { E_ACTION_TYPE::TILTING_DOWN, "TILTING_DOWN" },
        { E_ACTION_TYPE::NONE, "NONE" },
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

struct LINE_CLEAN {
    LINE_CLEAN() : lineInfo{E_PURPOSE_LINE_CLEAN_INFO::NONE} {}
    E_PURPOSE_LINE_CLEAN_INFO lineInfo;
};

struct MOVING {
    // MOVING 구조체에 필요한 추가적인 필드들을 여기에 정의
};

struct EXPLORER {
    // EXPLORER 구조체에 필요한 추가적인 필드들을 여기에 정의
};

struct SIGNAL_TRACKING {
    // SIGNAL_TRACKING 구조체에 필요한 추가적인 필드들을 여기에 정의
};

struct WALL_FOLLOW {
    // WALL_FOLLOW 구조체에 필요한 추가적인 필드들을 여기에 정의
};

struct tActionPurpose {
    tActionPurpose() : type{E_PURPOSE_TYPE::NONE} {}

    E_PURPOSE_TYPE type;
    union {
        LINE_CLEAN lineClean;
        MOVING moving;
        EXPLORER explorer;
        SIGNAL_TRACKING signalTracking;  
        WALL_FOLLOW wallFollow;
    };
};


struct TARGET_ROTATE{
    TARGET_ROTATE() : targetAngle{0.0}, targetPoint{tPoint()}, desiredV{MOTION_CONTROLLER_DESIRED_V}, desiredW{MOTION_CONTROLLER_DESIRED_W} {}
    double targetAngle;     // 단위 rad
    tPoint targetPoint;         // 단위 m, m
    double desiredV, desiredW; // 단위 m/sec, rad/sec
};

struct TARGET_LINEAR{
    TARGET_LINEAR() : startPose(tPose()), targetPoint(tPoint()), targetDistance{0.0}, desiredV{MOTION_CONTROLLER_DESIRED_V}, desiredW{MOTION_CONTROLLER_DESIRED_W} {}

    tPose  startPose;           // 단위 m, m, rad
    tPoint targetPoint;         // 단위 m, m
    double targetDistance;      // 단위 m
    double desiredV, desiredW;  // 단위 m/sec, rad/sec
};

struct TARGET_CURVE{
    TARGET_CURVE() : radius{0.0}, targetAngle{0.0}, desiredV{MOTION_CONTROLLER_DESIRED_V}, desiredW{MOTION_CONTROLLER_DESIRED_W} {}
    double radius;          // 단위 m
    double targetAngle;     // 단위 rad
    double desiredV, desiredW; // 단위 m/sec, rad/sec
};

struct TARGET_DISTANCE{
    TARGET_DISTANCE() : distance{0.0}, desiredV{MOTION_CONTROLLER_DESIRED_V}, desiredW{MOTION_CONTROLLER_DESIRED_W} {}
    double distance;          // 단위 m
    double desiredV, desiredW; // 단위 m/sec, rad/sec
};

struct TARGET_NONE{
    TARGET_NONE() : desiredV{MOTION_CONTROLLER_DESIRED_V}, desiredW{MOTION_CONTROLLER_DESIRED_W} {}
    double desiredV, desiredW; // 단위 m/sec, rad/sec
};

struct TARGET_STOP{
    TARGET_STOP() : waitForMs{300} {}
    int waitForMs; // 단위 msec
};
struct tAction {
    tAction() : type{E_ACTION_TYPE::NONE}, purpose{}, needObstacleCheck{true}, profile{tProfile()} {}

    E_ACTION_TYPE type;
    tActionPurpose purpose; // 움직이는 목적이 필요한 경우 사용.
    bool needObstacleCheck; // 움직이는 중 장애물 감지가 필요한경우 true, 장애물 감지 없이는 false
    bool pwmControl;
    tProfile profile;

    union {
        TARGET_ROTATE rotate;
        TARGET_LINEAR linear;
        TARGET_CURVE curve;
        TARGET_NONE targetNone;
        TARGET_STOP stop;
    };
};

class CWayPoint
{
private:
    std::list<tAction> action;
public:
    CWayPoint(/* args */);
    CWayPoint &operator=(const CWayPoint &);
    CWayPoint(const CWayPoint& other);
    ~CWayPoint();
    
    void pushAction(tAction push);
    tAction popAction();
    tAction getAction();
    void clearAction();
    int getActionCount();

    std::list<tPose> cleanPathList;
    void setCleanPath(std::list<tPose> set);
    std::list<tPose> getCleanPath();
    std::list<tPoint> getCurrentPath();
};