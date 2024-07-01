/**
 * @file signals.h
 * @author hhryu
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/observer.h"
#include "commonStruct.h"
#include "interfaceStruct.h"
#include <limits>

//-----------------------------------------------------------------------------
// Definitions related with Signal Queue
//-----------------------------------------------------------------------------

#define RECEIVER_IDX 4//2//4 // 수신부는 앞 2개, 사이드 2개. // 2 // 수신부는 좌, 우 두개.//7

#define IDX_RECEIVER_VOID           0XFF
#define IDX_RECEIVER_FRONT_LEFT     0
#define IDX_RECEIVER_FRONT_RIGHT    1
#define IDX_RECEIVER_SIDE_LEFT      2
#define IDX_RECEIVER_SIDE_RIGHT     3
#define IDX_RECEIVER_ANYTHING       0xFE

// #define IDX_RECEIVER_VOID           0
// #define IDX_RECEIVER_FRONT_LEFT     (0b01)
// #define IDX_RECEIVER_FRONT_RIGHT    (IDX_RECEIVER_FRONT_LEFT << 1)
// #define IDX_RECEIVER_SIDE_LEFT      (IDX_RECEIVER_FRONT_LEFT << 2)
// #define IDX_RECEIVER_SIDE_RIGHT     (IDX_RECEIVER_FRONT_LEFT << 3)
// #define IDX_RECEIVER_ANYTHING       (IDX_RECEIVER_FRONT_LEFT << 4)

#define SIGNAL_CENTER_SHORT         0x61 // 'a'
#define SIGNAL_RIGHT_CENTER_SHORT   0x62 // b
#define SIGNAL_LEFT_CENTER_SHORT    0X63 // c
#define SIGNAL_RIGHT_SIDE_SHORT     0X64 // d
#define SIGNAL_LEFT_SIDE_SHORT      0X65 // e

#define SIGNAL_CENTER_LONG          0x41 // A
#define SIGNAL_RIGHT_CENTER_LONG    0x42 // B
#define SIGNAL_LEFT_CENTER_LONG     0X43 // C
#define SIGNAL_RIGHT_SIDE_LONG      0X44 // D
#define SIGNAL_LEFT_SIDE_LONG       0X45 // E

#define IS_SIGNAL_CENTER_SHORT         0b1
#define IS_SIGNAL_RIGHT_CENTER_SHORT   0b10
#define IS_SIGNAL_LEFT_CENTER_SHORT    0b100
#define IS_SIGNAL_RIGHT_SIDE_SHORT     0b1000
#define IS_SIGNAL_LEFT_SIDE_SHORT      0b10000

#define IS_SIGNAL_CENTER_LONG          0b100000
#define IS_SIGNAL_RIGHT_CENTER_LONG    0b1000000
#define IS_SIGNAL_LEFT_CENTER_LONG     0b10000000
#define IS_SIGNAL_RIGHT_SIDE_LONG      0b100000000
#define IS_SIGNAL_LEFT_SIDE_LONG       0b1000000000

#define SIGNAL_SHORT_ANYTHING       0x6F
#define SIGNAL_LONG_ANYTHING        0x4F
#define SIGNAL_ANYTHING             0xFE
#define SIGNAL_BLANK                0xFF

#define ACK_FAN                     0x6E


#define SIGNAL_LIFE_TIME 0.48 // 단위 : 초, 인터페이스에서 시그널 받은 시간으로부터 이 만큼 지나면 삭제.
const u8 arraySize = 48;//24;//20;

#define HEADING_OFF_TRACK_ANGLE 40

enum class E_RECEIVER
{
    VOID = 0xFF,
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    SIDE_LEFT = 2,
    SIDE_RIGHT =3,
    ANYTHING = 0xFE,
};

enum class E_SIGNAL
{
    VOID = 0,

    CENTER_LONG = 0x41,         // 65
    RIGHT_CENTER_LONG = 0x42,   // 66
    LEFT_CENTER_LONG = 0X43,    // 67
    RIGHT_SIDE_LONG = 0X44,     // 68
    LEFT_SIDE_LONG = 0X45,      // 69

    CENTER_SHORT = 0x61,        // 141
    RIGHT_CENTER_SHORT = 0x62,  // 142
    LEFT_CENTER_SHORT = 0X63,   // 143
    RIGHT_SIDE_SHORT = 0X64,    // 144
    LEFT_SIDE_SHORT = 0X65,     // 145

    // ANYTHING_LONG = 0xfc,
    // ANYTHING_SHORT = 0xfd,
    ANYTHING = 0xfe,
    BLANK = 0xff
};

enum class E_CHECK_SIGNAL
{
    // VOID,
    IDLE,
    END,
    ING,
    START,
};

typedef struct
{
    u8 message;  // 메시지 ID
    u32 msgtick; // 메시지 생성 시간
    s32 param;   // 현재 사용 안함

} RSF_MESSAGE, *PRSF_MESSAGE; // System Queue에 저장될 Message Format

/**
 * @brief
 * @author hhryu
 * @date 2023/01/11
 * @details 로봇이 충전기 신호를 받은 부분, 작성일 기준 TS800, Q8의 신호 수신부는 좌측 우측 두가지임.
 */
enum class E_SIGNAL_ROBOT_RECEIVER// : u8 // enum type RSF_SIG_RECEIVER에서 이름 변경
{
    VOID          = 0xff,
    CENTER_LEFT   = 0,    // 로봇 좌측 센서
    CENTER_RIGHT  = 1,    // 로봇 우측 센서
    // CENTER  = 2,       // 로봇 전방 센서 - 필요 유무에 따라 생성한다.
};
// typedef enum
// {
//     E_SSR_VOID          = 0xff,
//     E_SSR_CENTER_LEFT   = 0,    // 로봇 좌측 센서
//     E_SSR_CENTER_RIGHT  = 1,    // 로봇 우측 센서
//     // E_SSR_CENTER  = 2,       // 로봇 전방 센서 - 필요 유무에 따라 생성한다.
// } RSF_SIG_RECEIVER;             // 로봇 수신 위치

/**
 * @brief
 * @author hhryu
 * @date 2023/01/11
 * @details 로봇이 충전기의 어느 부분 신호를 받았는지, 작성일 기준 TS800, Q8 크래들 신호 송신은 아래 5포인트임.
 *          크래들 기준 SIDE_LONG 좌, 우에 약 4M의 신호를 줌
 *          CENTER_LONG 좌, 우에 1.5M의 신호를 줌
 *          CENTER_SHORT는 0.3M의 짧은 신호를 줌 (크래들에 매우 근접해야 감지되는 수준)
 */
enum class E_SIGANL_CHARGER_SOURCE // enum type RSF_SIG_SOURCE에서 이름 변경
{
    VOID              = 0xff,
    SIDE_LONG_LEFT    = 1,    // 크래들 좌측 바깥 신호
    CENTER_LONG_LEFT  = 2,    // 크래들 좌측 중앙 신호
    CNETER_SHORT      = 3,    // 크래들 중앙 짧은 신호 (크래들 초근접)
    CENTER_LONG_RIGHT = 4,    // 크래들 우측 중앙 신호
    SIDE_LONG_RIGHT   = 5     // 크래들 우측 바깥 신호
};
// typedef enum
// {
//     E_SSI_VOID              = 0xff,
//     E_SSI_SIDE_LONG_LEFT    = 1,    // 크래들 좌측 바깥 신호
//     E_SSI_CENTER_LONG_LEFT  = 2,    // 크래들 좌측 중앙 신호
//     E_SSI_CNETER_SHORT      = 3,    // 크래들 중앙 짧은 신호 (크래들 초근접)
//     E_SSI_CENTER_LONG_RIGHT = 4,    // 크래들 우측 중앙 신호
//     E_SSI_SIDE_LONG_RIGHT   = 5     // 크래들 우측 바깥 신호
// } RSF_SIG_SOURCE;                   // 충전기 송신 위치

typedef union
{
    u32 signals; // u16 signals; // u32 signals;
    struct
    {
        // u16 sideLeft    : 3;    
        // u16 centerLeft  : 3;    
        // u16 centerShort : 3;    
        // u16 centerRight : 3;    
        // u16 sideRight   : 3;    

        u32 leftSideShort   : 3;    // 크래들 좌측 바깥 짧은 신호
        u32 leftCenterShort : 3;    // 크래들 좌측 중앙 짧은 신호
        u32 centerShort     : 3;    // 크래들 중앙 짧은 신호
        u32 rightSideShort  : 3;    // 크래들 우측 중앙 짧은 신호
        u32 rightCenterShort: 3;    // 크래들 우측 바깥 짧은 신호

        u32 leftSideLong    : 3;    // 크래들 좌측 바깥 긴 신호
        u32 leftCenterLong  : 3;    // 크래들 좌측 중앙 긴 신호
        u32 centerLong      : 3;    // 크래들 중앙 긴 신호
        u32 rightSideLong   : 3;    // 크래들 우측 중앙 긴 신호
        u32 rightCenterLong : 3;    // 크래들 우측 바깥 긴 신호

        u32 reserved : 2;       // 15 + 15 + 2 = 32
    } b;

} tSignalDetails;

typedef union
{
    u32 counts; // u32 count.
    struct
    {
        u32 lc : 6;
        u32 sc : 6;
        // 12
        u32 rc : 6;
        u32 ls : 6;
        // 24
        u32 rs : 6;
        u32 reserved : 2; // 남는 비트
        // 6+6+6+6+6+2 == 32
    } b;

} tSignalCount;


typedef struct{
    bool ackFan;
    s32 checkTime;
    
} tCradleData;

typedef union // Cradle이 Receiver에 준 신호의 위치정보.
{
    u32 detectors; // 디텍터들의 모든 정보를 가지고 있음.
    struct
    {
        // hhryu230310 : 크래들의 왼쪽 신호인데, 크래들 입장에서는 오른쪽 신호임에 주의. 변수명은 로봇입장으로 통일하였음.
        // 최하위 4비트 : leftSide, 최상위 : reserved (확인 완료)
        u32 leftSide    : 4;
        u32 leftCenter  : 4;

        u32 center      : 4;
        u32 rightCenter : 4;

        u32 rightSide   : 4;

        u32 reserved    : 12; // 20+12 = 32
    } b;

} tSignalDetector;

typedef union // Receiver입장에서 받은 cradle 정보
{
    u32 transmitters; // 리시버가 받은 크래들IR(transmitter)의 정보를 담고있다.
    struct
    {
        // hhryu230310 : 크래들의 왼쪽 long신호인데, 크래들 입장에서는 오른쪽 long신호임에 주의. 변수명은 로봇입장으로 통일하였음.
        u32 left : 10;
        u32 right : 10;

        u32 reserved : 12; // 20+12=32
    } b;

}tReceiverSignal;

typedef struct{
    tSignalDetector detector;
    tReceiverSignal receiver;

    tSignalDetector detectorLong;
    tReceiverSignal receiverSide;

    // u16 signal[RECEIVER_IDX][20];

    s32 tryDockAngle;
    bool bSignalCheckStart;
    bool bSignalCheckComplete;

    double signalCheckStartTime;
    double signalCheckEndTime;

}tSignalData;

// typedef enum
// {
//     ssr_void = 0xff,
//     ssr_bleft = 0,   // 로봇 후방 좌측 센서
//     ssr_left = 1,    // 로봇 좌측 센서
//     ssr_lcenter = 2, // 로봇 좌측 대각선 센서
//     ssr_center = 3,  // 로봇 전방 센서
//     ssr_rcenter = 4, // 로봇 우측 대각선 센서
//     ssr_right = 5,   // 로봇 우특 센서
//     ssr_bright = 6   // 로봇 후방 우측 센서

// } RSF_SIG_RECEIVER; // 로봇 수신 위치

typedef struct
{
    u8 mask;
    tSignalDetails receivers[RECEIVER_IDX]; // cheol, RSF_SIG_RECEIVER 배열 순서를 따름.

    bool bCheckSignalData;                    // hhryu230111 : CheckSignals에서 data가 있는지 확인 하기 위해 생성.

} tReceiver, *PRSF_RECEIVER; // SIGNAL DATA FORMA

typedef struct
{
    tReceiver long_;

} RSF_SIGNALDATA;

typedef struct
{
    RSF_SIGNALDATA charger_; // 충전기 원거리 신호 수신 정보

} RSF_SIGNALS, *PRSF_SIGNALS; // Signal Array
//
#define SQ_MAX_DATA_COUNT 32 // 64 // 32
#define SQ_MAX_POOL_COUNT 1 // hhryu230118 : CHARGER 1 + INDICATOR 0 == 1 // 6      // CHARGER + { INDICATOR(1) ~ INDICATOR(5) }

#define SQ_ID_ALL 0xff
#define SQ_ID_CHARGER 0

#define DETECTOR_ANY 0xf0

typedef struct _tSignalQData
{
    _tSignalQData() : detector{0}, source{0}, count{0} {}
    u8 detector; // hhryu230118 : 로봇 수신부, E_SIGNAL_ROBOT_RECEIVER   // cheol, 로봇 수신부, RSF_SIG_RECEIVER
    u16 source;   // hhryu230118 : 충전기 송신부, E_SIGANL_CHARGER_SOURCE // cheol, 충전기 송신부, RSF_SIG_SOURCE
    u16 count;

} tSignalQData; // RSF_SIGNALQ_DATA ;

typedef struct _tSignalPool
{
    _tSignalPool() : head{0}, tail{0}, iter{0} {}
    u8 head;
    u8 tail;
    u8 iter;

    tSignalQData Pool[SQ_MAX_DATA_COUNT];

} tSignalPool; // RSF_SIGNALPOOL ;


typedef struct tSignalCheckPose
{
    bool bCenter;
    bool bLeftSide;
    bool bRightSide;
    tPose centerPose;
    tPose leftSidePose;
    tPose rightSidePose;

} tSignalCheckPose;

typedef struct tSignalArray
{
    u8 receiver[arraySize];
} tSignalArray;

typedef struct tSignalList
{
    std::list<std::pair<u8, double>> dataTimePairs;

} tSignalList;

typedef struct tSignalVector
{
    std::vector<u8> receiver;
} tSignalVector;

typedef struct tTryDockAngle
{
    s32 data;
    u8 detector;
} tTryDockAngle;

typedef struct tSignalTime
{
    double debugFrontLeft;
    double debugFrontRight;
    double debugSideLeft;
    double debugSideRight;

} tSignalTime;

class CSignal : public CObserver
{
private:
    tSignalPool signalQ_; // 충전기의 신호를 저장하는 큐, 여기에 get set을 한다.
    tReceiver   receiveData_;
    tSignalData signalData_;
    tCradleData cradleData_;
    tSignalCheckPose signalCheckPoseData_;
    tSignalCount _sigCount[RECEIVER_IDX];
    tSignalArray _signalArray[RECEIVER_IDX];
    tSignalVector _signalVector[RECEIVER_IDX];
    tSignalTime sigTime;
    bool disableUpdate;

    tSignalList sigList[RECEIVER_IDX];

    E_CHECK_SIGNAL checkSignalState;
    double checkSignalTime;

private:
    void updateSignal(const std::list<tSysSignal>& sysSignal, tPose robotPose);
    void updateSignalList(const std::list<tSysSignal>& sysSignal,tPose robotPose);
    void updateVector(const std::list<tSysSignal>& sysSignal);
    void pushSignalVector(const std::list<tSysSignal>& sysSignal);
    void pushSignalArray(u8 index, u8 number, u8 data);
    double getCheckSignalEndTime();
    void debugSignal(const std::list<tSysSignal>& sysSignal);

public:
    CSignal();
    ~CSignal();
    void setCheckSignalState(E_CHECK_SIGNAL set);
    E_CHECK_SIGNAL getCheckSignalState();
    void update(CExternData *pExternData) override;

    void checkAngleParameterBySignal(double *angleK , double *min, double *max);
    double getAngleBySignal(double angleK, double min, double max);
    double getAngleByLongSignal(double angleK, double sideK, double centerLRK, double centerK, double min, double max);
    double getAngleByShortSignal(double angleK, double sideK, double centerLRK, double centerK, double min, double max);

    double buildTryDockAngle();
    double buildTryDockAngle(tPose robotPose);

    tSignalArray* getSignalArray(u8 receiver);
    bool getFanAck();

    bool getSignalCheckComplete();
    void setSignalCheckComplete(bool set);
    u16 getSignalData(u8 detector);

    u16 getDetector(u16 source);
    u16 getSignals(u8 detector);
    bool buildSignalData(u8 idx,u8 data);

    u8 checkSystemSignalData(u8 detector,tSysSignal sysSignal);
    void setSignalCheckStart(bool set);
    bool getSignalCheckStart();
    tSignalData getDetectedReceiver();

    void updateSignalCheckPose(tPose robotPose);
    tSignalCheckPose getSignalCheckPose();


    void setCheckSignalEndTime(double time);
    
    u16 receiverDetected(u8 receiver);
    u16 getReceivedShortSignalData(u8 receiver);
    u16 getReceivedLongSignalData(u8 receiver);
    u16 processReceiver(const tSignalList& list);
    u16 signalConvertingBitFlag(u8 signal);
    bool isFlagSig(u8 receiver, u16 flagSig);
    bool countBitSignalDetectedCASE1(u8 receiver, u16 bSignal);
    u8 countBitSignalDetectedCASE2(u8 receiver, u16 bSignal);
    u8 countBitSignalDetectedCASE3(u8 receiver, u16 bSignal);
    u16 searchShortSignal(const tSignalList& recevierList);
    u16 searchLongSignal(const tSignalList& recevierList);
    u8 countSignalDetected(u8 receiver, u8 signal);
    u32 countSignalVector(u8 receiver, u8 signal);
    void clearSignalVector();
    void updateTriggerSignalList(u8 receiver, u8 data, bool bSignal);
    void debugSignalPrint();
    void debugSignalPrint(const std::string& str);
    void debugTriggerSignalPrint(const std::string& str);
};