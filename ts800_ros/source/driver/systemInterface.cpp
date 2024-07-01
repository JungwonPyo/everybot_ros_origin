

#include <fstream>

#include "systemInterface.h"
#include "eblog.h"
#include "debugCtr.h"
#include "LibRobotControlInterface.h"
#include "LibRobotControlDrive.h"
#include "LibRobotDataInterface.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"
#include "LibRobotControlLocalization.h"
#include "LibRobotConfigInterface.h"
#include "LibRobotOtaInterface.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/
#define WRITE_SYSTEM_DUTEY_FILE   0

extern CRobotConfigInterface RoConfigInf; // hw config 모듈
extern CRobotControlInterface RoCtrInf; // hw Control Interface 모듈
extern CRobotDataInterface RoDatInf;    // hw Data Interface 모듈
extern CRobotDisplayInterface RoDisplayInf;
extern CRobotSoundInterface RoSoundInf;
extern CRobotOtaInterface RoOtaInf;

#define SEND_RETRY_MAX      80  // (SEND_RETRY_DELAY * SEND_RETRY_MAX)msec
#define SEND_RETRY_DELAY    5   //msec

#if WRITE_SYSTEM_DUTEY_FILE == 1
std::ofstream ofs_;
std::string filename_;
bool open_ = false;
#endif
CSystemInterface::CSystemInterface(/* args */)
{
    bReceiveDataLoopRunning = true;

    bUpdateSysBatteryData = false;
    bUpdateStateButtonData = false;
    bUpdateStateBumperData = false;
    bUpdateStateCheckAliveData = false;
    bUpdateStateCliffData = false;
    bUpdateStateTofData = false;
    bUpdateStateFrontIRData = false;
    bUpdateStateImuData = false;
    bUpdateaStateSysPowerData = false;
    bUpdateStatePumpData = false;
    bUpdateStateRemoteKeyData = false;
    bUpdateStateSignalData = false;
    bUpdateStateSysPoseData = false;
    bUpdateStateTiltingData = false;
    bUpdateStateWheelMotorData = false;
    bUpdateStateErrorData = false;

    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Config Interface\t"<<BOLDWHITE<<"연결중...");
    bool bIsOpen = !RoConfigInf.Open();
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Config Interface\t"<<(bIsOpen?BOLDGREEN:BOLDRED)<<(bIsOpen?"연결성공":"연결실패"));

#if 1 // hhryu240405 : 데이터에 필터 적용 필요시
    imuCombinedState = 0;
    filterdImuState = 0;
#endif

#if WRITE_SYSTEM_DUTEY_FILE == 1
    if (open_ == false) 
    {
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        std::string file_base{"/home/ebot/map/system_"};
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
}

CSystemInterface::~CSystemInterface()
{
    eblog(LOG_LV,  "");
#if WRITE_SYSTEM_DUTEY_FILE == 1
    open_ = false;
    ofs_.close();
#endif
    RoConfigInf.Close();
}

void CSystemInterface::threadLoop()
{
    eblog(LOG_LV_NECESSARY, "system interface thread start");
    utilThreadWait wait(5); // 10ms -> 5ms
    RoDatInf.Cradle.Inf_ClearCradleData();
    RoDatInf.Cradle.Inf_SetCradleDataStoreEnable();
    
    while( bReceiveDataLoopRunning )
    {
#if WRITE_SYSTEM_DUTEY_FILE == 1
        StopWatch _thTime;
#endif
        setRemoteKeyData();        //리모컨 키
        setButtonData();        //button
        setSysPoseData();       //시스템 좌표계
        setImuData();           //imu raw 데이터
        setBumperData();        //Bumper 데이터 
        setTofData();
        setCliffActionState();  //cliff action 상태
        setWheelMotorData();    // 휠 모터 데이터
        setSignalData();        //Cradle IR 데이터        
        setSystemPowerData();   //Power(Battery) 데이터
        setFrontIRData();       //Front IR 데이터
        setTiltingData();       //틸팅 상태 데이터
        setDisplayStateData();  // UI - 디스플레이 상태 업데이트
        setSoundStateData();        
        setError();             //에러 데이터
        setOtaData();

        DEBUG_CTR.isAliveSystemInterface.set(true);

        wait.sleepUntil();
#if WRITE_SYSTEM_DUTEY_FILE == 1
        ofs_ << _thTime.getTime() << std::endl;
#endif
    }
}


/////////////////////////////////////////////////////////////
// Battery data - 데이터 쓰기
void CSystemInterface::setBatteryData()
{
    // RoDatInf.Battery
    bUpdateSysBatteryData = false;
}
// Battery data - 업데이트 확인
bool CSystemInterface::isUpdateBatteryData()
{
    return bUpdateSysBatteryData;
}
// Battery data - 데이터 읽기

tSysBattery CSystemInterface::useBatteryData()
{
    bUpdateSysBatteryData = false;
    return sysBatteryData;
}
tSysBattery CSystemInterface::getBatteryData()
{
    return sysBatteryData;
}

/////////////////////////////////////////////////////////////
// Button data - 데이터 쓰기
void CSystemInterface::setButtonData()
{    
    u8 swPlayStop = 0, swHome = 0, swPeri = 0;
    RoDatInf.Ui.Inf_GetSwitchData(&swPlayStop, &swHome, &swPeri);

    if (swPlayStop != buttonData.buttonStart || swHome != buttonData.buttonHome || swPeri != buttonData.buttonFunc)
    {
        bUpdateStateButtonData = true;
        buttonData.buttonFunc = swPeri;
        buttonData.buttonHome = swHome;
        buttonData.buttonStart = swPlayStop;
        //eblog(LOG_LV_SYSTEMINF, "[System Interface] swPlayStop : "<<(int)swPlayStop<<" ,swHome : "<< (int)swHome<<" ,swPeri : "<< (int)swPeri);
        eblog(LOG_LV_NECESSARY, "[System Interface] swPlayStop : "<<(int)swPlayStop<<" ,swHome : "<< (int)swHome<<" ,swPeri : "<< (int)swPeri);
    }
}
// Button data - 업데이트 확인
bool CSystemInterface::isUpdateButtonData()
{
    return bUpdateStateButtonData;
}
// Button data - 데이터 읽기

tSysButton CSystemInterface::useButtonData()
{
    bUpdateStateButtonData = false;
    return buttonData;
}

tSysButton CSystemInterface::getButtonData()
{
    return buttonData;
}

/////////////////////////////////////////////////////////////
// Bumper data - 데이터 쓰기
void CSystemInterface::setBumperData()
{
    tSysBumper tempBumperData;
    u8 lB = 0, rB = 0;
    RoDatInf.Bumper.Inf_GetBumperData(&lB, &rB);
        
    tempBumperData.left = (bool)lB;
    tempBumperData.right = (bool)rB;

    bumperData = tempBumperData;
    bUpdateStateBumperData = true;
	
    if ( lB != 0 || rB != 0)
    {
        bDebugBumpPush = true;
        // 메시지 스레드 부하로 인해 주석처리 (범핑 체크 필요 없다고 판단 0426) by hjkim, hyjoe
        // if(lB && rB)
        // {
        //     LED_CTR.ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_RED);
        // }
        // else if(lB)
        // {
        //     LED_CTR.ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_GREEN);
        // }
        // else
        // {
        //     LED_CTR.ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_BLUE);
        // }
        //ceblog((LOG_LV_NECESSARY | LOG_LV_OBSTACLE), YELLOW, "[System Interface] Left: "<<(int)lB<<"\tRight: "<< (int)rB); // 240305 : 문제 있어서 열어놨음.
    }
    else if(bDebugBumpPush)
    {
        bDebugBumpPush = false;
        //LED_CTR.ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
    }
}
// Bumper data - 업데이트 확인
bool CSystemInterface::isUpdateBumperData()
{
    return bUpdateStateBumperData;
}
// Bumper data - 데이터 읽기

tSysBumper CSystemInterface ::useBumperData()
{
    bUpdateStateBumperData = false;
    return bumperData;
}

tSysBumper CSystemInterface ::getBumperData()
{
    bUpdateStateBumperData = false;
    return bumperData;
}

/////////////////////////////////////////////////////////////
// Check Aliva data - 데이터 쓰기
void CSystemInterface::setCheckAliveData()
{
    u8 tempCheckAlive = 0;
    // RoDatInf.CheckAlive
    bUpdateStateCheckAliveData = false;
}
// Check Aliva data - 업데이트 확인
bool CSystemInterface::isUpdateCheckAlive()
{
    return bUpdateStateCheckAliveData;
}

u8 CSystemInterface ::useCheckAliveData()
{
    bUpdateStateCheckAliveData = false;
    return checkAliveData;
}
// Check Aliva data - 데이터 읽기
u8 CSystemInterface ::getCheckAliveData()
{
    return checkAliveData;
}

/////////////////////////////////////////////////////////////
// ToF data - Cliff, Side ToF 데이터 쓰기
void CSystemInterface::setTofData()
{
    char    tempCliffLeftDeviceState    = 0, tempCliffLeftRangeStatus   = 0, 
            tempCliffRightDeviceState   = 0, tempCliffRightRangeStatus  = 0,
            tempKnollDeviceState        = 0, tempKnollRangeStatus       = 0;
            
    short   tempCliffLeftRangeAvg  = 0,
            tempCliffRightRangeAvg = 0,
            tempKnollRangeAvg = 0;
            
    char    tempWallRightDeviceState = 0, tempWallRightRangeStatus = 0, tempWallLeftDeviceState = 0, tempWallLeftRangeStatus = 0;
    short   tempWallLeftRangeAvg = 0, tempWallRightRangeAvg = 0;

    RoDatInf.Floor.Inf_GetFloorData(
        &tempCliffLeftDeviceState,  &tempCliffLeftRangeStatus,  &tempCliffLeftRangeAvg,
        &tempCliffRightDeviceState, &tempCliffRightRangeStatus, &tempCliffRightRangeAvg,
        &tempKnollDeviceState, &tempKnollRangeStatus, &tempKnollRangeAvg);
    
    RoDatInf.Wall.Inf_GetWallData(&tempWallRightDeviceState, &tempWallRightRangeStatus, &tempWallRightRangeAvg, &tempWallLeftDeviceState, &tempWallLeftRangeStatus,&tempWallLeftRangeAvg);

    cliffData.left.deviceState      = (u16)tempCliffLeftDeviceState;
    cliffData.left.rangeStatus      = (u16)tempCliffLeftRangeStatus;
    cliffData.left.rangeAvg         = (u16)tempCliffLeftRangeAvg;

    cliffData.right.deviceState     = (u16)tempCliffRightDeviceState;
    cliffData.right.rangeStatus     = (u16)tempCliffRightRangeStatus;
    cliffData.right.rangeAvg        = (u16)tempCliffRightRangeAvg;

    TofData.knollCenter.deviceState = (u16)tempKnollDeviceState;
    TofData.knollCenter.rangeStatus = (u16)tempKnollRangeStatus;
    TofData.knollCenter.rangeAvg    = (u16)tempKnollRangeAvg;

    TofData.rightSide.deviceState   = (u16)tempWallRightDeviceState;
    TofData.rightSide.rangeStatus   = (u16)tempWallRightRangeStatus;
    TofData.rightSide.rangeAvg      = (u16)tempWallRightRangeAvg;
    
    TofData.leftSide.deviceState   = (u16)tempWallLeftDeviceState;
    TofData.leftSide.rangeStatus   = (u16)tempWallLeftRangeStatus;
    TofData.leftSide.rangeAvg       = (u16)tempWallLeftRangeAvg;


    //eblog(LOG_LV_SYSTEMINF , "tof sensor ranAvg SET  : [ "<<cliffData.left.rangeAvg<< " ,  " <<cliffData.right.rangeAvg<< " ], knoll : [ "<<TofData.knollCenter.rangeAvg << " ] Rightwall : [ " << TofData.rightSide.rangeAvg<<" ] LeftWall : [" << TofData.leftSide.rangeAvg << " ]");
    //eblog(LOG_LV_SYSTEMINF , "tof sensor ranSta SET  : [ "<<cliffData.left.rangeStatus<< " ,  " <<cliffData.right.rangeStatus<< " ], knoll : [ "<<TofData.knollCenter.rangeStatus << " ] Rightwall : [ " << TofData.rightSide.rangeStatus<< "] LeftWall : [" << TofData.leftSide.rangeStatus << " ]");
    //eblog(LOG_LV_SYSTEMINF , "tof sensor device SET  : [ "<<cliffData.left.deviceState<< " ,  " <<cliffData.right.deviceState<< " ], knoll : [ "<<TofData.knollCenter.deviceState << " ] Rightwall : [ " << TofData.rightSide.deviceState<< "] LeftWall : [" << TofData.leftSide.deviceState << " ]");
    
    bUpdateStateTofData = true;
    bUpdateStateCliffData = true;
}

void CSystemInterface::setCliffActionState()
{     
    RoDatInf.GetError(&CliffActionState);
    bUpdateStateCliffActionState = true;
}

// ToF data - Cliff 업데이트 확인
bool CSystemInterface::isUpdateCliffData()
{
    return bUpdateStateCliffData;
}
// ToF data - Side ToF 업데이트 확인
bool CSystemInterface::isUpdateTofData()
{
    return bUpdateStateTofData;
}
// ToF data - Cliff 데이터 읽기
tSysCliff CSystemInterface::useCliffData()
{
    bUpdateStateCliffData = false;
    return cliffData;
}

tSysCliff CSystemInterface::getCliffData()
{
    return cliffData;
}
// ToF data - Side ToF 데이터 읽기
tTof CSystemInterface::useTofData()
{
    bUpdateStateTofData = false;
    return TofData;
}

tTof CSystemInterface::getTofData()
{
    bUpdateStateTofData = false;
    return TofData;
}

bool CSystemInterface::isUpdateCliffActionState()
{
    return bUpdateStateCliffActionState;
}

static unsigned int debug_cliff = 0;

tCliffActionState CSystemInterface::useCliffActionState()
{
    tCliffActionState state;
    state.bRobotStop = CliffActionState &   0x10000000;
    state.bLeftDetect = CliffActionState &  0x20000000;
    state.bRightDetect = CliffActionState & 0x40000000;
    state.bTimeOut = CliffActionState &     0x80000000;

#if 0
    if (debug_cliff % 50 ==0){
        eblog(LOG_LV_NECESSARY, "bRobotStop : "<<state.bRobotStop
            <<" ,bLeftDetect : "<<state.bLeftDetect
            <<" ,bRightDetect : "<<state.bRightDetect
            <<" ,bTimeOut : "<<state.bTimeOut
        );
    }

    debug_cliff++;
#endif
    bUpdateStateCliffActionState = false;

    return state;
}

tCliffActionState CSystemInterface::getCliffActionState()
{
    tCliffActionState state;
    state.bRobotStop = CliffActionState &   0x10000000;
    state.bLeftDetect = CliffActionState &  0x20000000;
    state.bRightDetect = CliffActionState & 0x40000000;
    state.bTimeOut = CliffActionState &     0x80000000;

#if 0
    if (debug_cliff % 50 ==0){
        eblog(LOG_LV_NECESSARY, "bRobotStop : "<<state.bRobotStop
            <<" ,bLeftDetect : "<<state.bLeftDetect
            <<" ,bRightDetect : "<<state.bRightDetect
            <<" ,bTimeOut : "<<state.bTimeOut
        );
    }

    debug_cliff++;
#endif

    return state;
}


/////////////////////////////////////////////////////////////
// 전방 IR data - 데이터 쓰기
void CSystemInterface::setFrontIRData()
{
    short tempFront_Center = 0, tempFront_Left = 0, tempFront_Rgiht = 0;

    RoDatInf.Near.Inf_GetNearData(&tempFront_Left,&tempFront_Center,&tempFront_Rgiht);
    FrontIRData.left = (u16)tempFront_Left;
    FrontIRData.center = (u16)tempFront_Center;
    FrontIRData.right = (u16)tempFront_Rgiht;

    bUpdateStateFrontIRData = true;
}
// 전방 IR data - 업데이트 확인
bool CSystemInterface::isUpdateFrontIRData()
{
    return bUpdateStateFrontIRData;
}
// 전방 IR data - 데이터 읽기

tSysFront CSystemInterface::useFrontIRData()
{
    bUpdateStateFrontIRData = false;
    return FrontIRData;
}

tSysFront CSystemInterface::getFrontIRData()
{
    return FrontIRData;
}

/////////////////////////////////////////////////////////////
// Imu data - 데이터 쓰기
void CSystemInterface::setImuData()
{
    struct timespec apTime;
    tSysIMU tempImuData;


#if USE_CEVA_LOG_SAVE == 1
    SHORT gR, gP, gY;
    RoDatInf.Imu.Inf_GetImuDataRaw(&gR, &gP, &gY, &tempImuData.Ax, &tempImuData.Ay, &tempImuData.Az);
    tempImuData.Groll   = gR;
    tempImuData.Gpitch  = gP;
    tempImuData.Gyaw    = gY;
#else
    RoDatInf.Imu.Inf_GetImuState(&tempImuData.state);
        RoDatInf.Imu.Inf_GetImuData(
        &tempImuData.Groll,
        &tempImuData.Gpitch,
        &tempImuData.Gyaw,
        &tempImuData.Ax,
        &tempImuData.Ay,
        &tempImuData.Az,
        &apTime);
#endif    
    // eblog(LOG_LV, "&tempImuData.state : "<<static_cast<int>(tempImuData.state));
    //lidar time  = lidar time t - 145ms => lidar time t 
    //odom time = odom time t + 250ms => odom time t + 250 + 145 - 250

    // 현재 시간을 얻습니다
    //ros::Time now = ros::Time::now();
    //printf("setImuData(before): %ld.%ld \n", apTime.tv_sec, apTime.tv_nsec);
    ros::Time now = convertToRosTime(apTime);
    //after
    //std::stringstream ss;
    //ss <<"setImuData ros::time(after): "<< now.sec << "." << now.nsec;
    //std::cout << ss.str() << std::endl;
    // 250 밀리초 지속 시간 생성
    //ros::Duration delay(LATE_OF_ODOM_SEC);
    // - 200 mns
    //ros::Duration latency_adjustment(0.2);

    // 250 밀리초 전의 시간 계산
    //ros::Time before = now - delay; // - latency_adjustment;
    tempImuData.timeStamp = now; //before;

    imuData = tempImuData;
    imuData.filteredState = imuStateFilter(imuData.state);
    //eblog(LOG_LV_DOCKING , "IMU STATE  : [ "<<(int)imuData.state);
    bUpdateStateImuData = true;
}

/**
 * @brief imu state가 흔들리며 올라오기 때문에 필터링을 통해 안정화하는 함수
 * 자료형만 템플릿으로 하고 조금 수정하면 모듈화해서 여러 데이터를 관리할 때 써도 될 것 같다.
 * 현재는 2비트 데이터만 16번 체크.
 * 
 * @return unsigned char 
 * 
 * @note 연산시간 ms
 * @date 2024-04-03
 * @author hhryu
 */
uint8_t CSystemInterface::imuStateFilter(unsigned char state)
{
    // 이전 데이터를 삭제하고 최신 데이터를 추가하여 imuCombinedState 업데이트
    imuCombinedState    = (imuCombinedState << 2)  | (state & 0x03);
    uint16_t byte1      = (imuCombinedState >> 0)  & 0xFFFF;     // 하위 16비트 추출
    uint16_t byte2      = (imuCombinedState >> 16) & 0xFFFF;     // 다음 16비트 추출
    if (byte1 == byte2)
    {
        filterdImuState = byte1 & 0x03;
        // ceblog(LOG_LV_SERVICESTEP, BLUE, "필터링 된 imu state : " << DEC(filterdImuState));
    }
    else 
    {
        // ceblog(LOG_LV_SERVICESTEP, RED, "필터링 되었던 imu state : " <<  DEC(filterdImuState) << "\tLow pass Filtering, tempCombinedState : " << BINARY(imuCombinedState) << "\t"
        // << "byte1[" << DEC(byte1) <<"] "
        // << "byte2[" << DEC(byte2) <<"] "
        // );
    }
    return filterdImuState;
}

// Imu data - 업데이트 확인
bool CSystemInterface::isUpdateImuData()
{
    return bUpdateStateImuData;
}
// Imu data - 데이터 읽기
tSysIMU CSystemInterface::useImuData()
{
#if 0 // ROS시간 debug
    ceblog(LOG_LV_NECESSARY, BOLDBLACK,
        "RosTime["<<BOLDYELLOW<<imuData.timeStamp<<BOLDBLACK<<"]\t"<<BOLDGREEN<<
        setw(7)<<imuData.Ax<<","<<
        setw(7)<<imuData.Ay<<","<<
        setw(7)<<imuData.Az<<","<<BOLDBLUE<<
        setw(7)<<imuData.Groll<<","<<
        setw(7)<<imuData.Gpitch<<","<<
        setw(7)<<imuData.Gyaw<<","<<BOLDCYAN<<
        setw(7)<<int(imuData.state));
#endif
    bUpdateStateImuData = false;
    return imuData;
}

tSysIMU CSystemInterface::getImuData()
{
    return imuData;
}

/////////////////////////////////////////////////////////////
// Power data - 데이터 쓰기
void CSystemInterface::setSystemPowerData()
{
    unsigned char PowerState= 0, AdapterIn = 0;
    short AdaptVolt = 0, BattVolt = 0, BattTemper = 0, BattPercent = 0, battCurrent = 0;

    RoDatInf.Power.Inf_GetPowerData(&PowerState,&AdapterIn,&AdaptVolt, &BattVolt, &BattTemper, &BattPercent, &battCurrent);

        
    sysBatteryData.battVolt = BattVolt;
    sysBatteryData.temperature = BattTemper;
    sysBatteryData.percent = BattPercent;
    sysBatteryData.current = battCurrent;
    bUpdateSysBatteryData = true;
    //eblog(LOG_LV_NECESSARY, "[System Interface] sys-battery Data battVolt : " << (int)sysBatteryData.battVolt << " temperature : " << (int)sysBatteryData.temperature << "percent : " << (int)sysBatteryData.percent << "current : " <<(int)sysBatteryData.percent);

    sysPowerData.state = PowerState;
    sysPowerData.adaptor_in = AdapterIn;
    sysPowerData.aptVolt = AdaptVolt;
    bUpdateaStateSysPowerData = true;
    //eblog(LOG_LV_NECESSARY, "[System Interface] sys-Power Data state : " << (int)sysPowerData.state << " adaptor : " << (int)sysPowerData.adaptor_in << "aptVolt : " << (int)sysPowerData.aptVolt);
}
// Power data - 업데이트 확인
bool CSystemInterface::isUpdatePowerData()
{
    return bUpdateaStateSysPowerData;
}
// Power data - 데이터 읽기
tSysPower CSystemInterface::usePowerData()
{
    bUpdateaStateSysPowerData = false;
    return sysPowerData;
}

tSysPower CSystemInterface::getPowerData()
{
    return sysPowerData;
}

/////////////////////////////////////////////////////////////
// Pump data - 데이터 쓰기
void CSystemInterface::setPumpData(tSysPump data)
{
    bUpdateStatePumpData = true;
    pumpData = data;
}
// Pump data - 업데이트 확인
bool CSystemInterface::isUpdatePumpData()
{
    return bUpdateStatePumpData;
}
// Pump data - 데이터 읽기
tSysPump CSystemInterface::usePumpData()
{
    bUpdateStatePumpData = false;
    return pumpData;
}
tSysPump CSystemInterface::getPumpData()
{
    return pumpData;
}

/////////////////////////////////////////////////////////////
// key data - 데이터 쓰기
void CSystemInterface::setRemoteKeyData()
{
    u8 tempRemote = RoDatInf.Remote.Inf_GetRemoconData();
    if ( tempRemote != 0 )
    {
        remoteData.remoteKey = tempRemote;
        bUpdateStateRemoteKeyData = true;        
        eblog(LOG_LV_SYSTEMINF, "[System Interface] get remote key : " << tempRemote);
    }
}
// key data - 업데이트 확인
bool CSystemInterface::isUpdateRemoteKeyData()
{
    return bUpdateStateRemoteKeyData;
}
// key data - 데이터 읽기
tSysRemote CSystemInterface::useRemoteKeyData()
{
    bUpdateStateRemoteKeyData = false;
    return remoteData;
}

tSysRemote CSystemInterface::getRemoteKeyData()
{
    return remoteData;
}

/////////////////////////////////////////////////////////////
// Charge Signal data - 데이터 쓰기
double interFrontRightSigTime = 0.0;
void CSystemInterface::setSignalData()
{
    bool bCheckUpdate = false;
    tSysSignal tempSignal;
    unsigned char rxData[4096] = {0};
    unsigned int rxDataCnt = 0;
    double presentTime = SYSTEM_TOOL.getSystemTime();
    double diff = 0;    

    /************* FRONT LEFT *************/
    RoDatInf.Cradle.Inf_GetCradleData(E_CradleRxPosition::FRONT_LEFT,rxData,&rxDataCnt);
    if (rxDataCnt > 0)
    {
        bCheckUpdate = true;    // 한개라도 업데이트 되면 체크 한다.
        //eblog(LOG_LV, "rxDataCnt 0 : "<<rxDataCnt);
    }
    tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].count = rxDataCnt;
    for (u32 i = 0; i < tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].count; i++)
    {
        tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].Data[i] = rxData[i];
        tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].timeStamp[i] = presentTime;
        // ceblog(LOG_LV_DOCKING, BLUE, "LEFT  ["<< tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].Data[i] << "] 데이터의 개수 ["<<tempSignal.ChargerIR[RECEIVER_FRONT_LEFT].count<<"] time[" << PRECISION(5) <<SYSTEM_TOOL.getSystemTime()<< "]");
    } 
    // eblog((LOG_LV << 16), "===LEFT END===");
    rxDataCnt = 0;
    /************* FRONT LEFT *************/

    /************* FRONT RIGHT *************/
    RoDatInf.Cradle.Inf_GetCradleData(E_CradleRxPosition::FRONT_RIGHT,rxData,&rxDataCnt);
    if (rxDataCnt > 0)
    {
        bCheckUpdate = true;    // 한개라도 업데이트 되면 체크 한다.
        // eblog(LOG_LV, "rxDataCnt 1 : "<<rxDataCnt);
    }
    tempSignal.ChargerIR[RECEIVER_FRONT_RIGHT].count = rxDataCnt;    
    for (u32 i = 0; i < tempSignal.ChargerIR[RECEIVER_FRONT_RIGHT].count; i++)
    {
        tempSignal.ChargerIR[RECEIVER_FRONT_RIGHT].Data[i] = rxData[i];
        tempSignal.ChargerIR[RECEIVER_FRONT_RIGHT].timeStamp[i] = presentTime;
        // diff = present - interFrontRightSigTime;
        // interFrontRightSigTime = present;        
        // ceblog (LOG_LV_DOCKING, YELLOW, "detector : 1\tdata : " << tempSignal.ChargerIR[RECEIVER_FRONT_RIGHT].Data[i] << "\tdiffTime : " << diff << "\ttime : "<< PRECISION(5) << SYSTEM_TOOL.getSystemTime());
        // ceblog(LOG_LV_DOCKING, YELLOW, "RIGHT ["<< tempSignal.ChargerIR[1].Data[i]<< "] 데이터의 개수 ["<<tempSignal.ChargerIR[1].count<<"] time[" << PRECISION(5) <<SYSTEM_TOOL.getSystemTime()<< "]");
    }
    rxDataCnt = 0;
    // ceblog((LOG_LV << 16), RED, "===RIGHT END=== checkTime[" << checkTime <<"]");
    /************* FRONT RIGHT *************/

    /************* SIDE LEFT *************/
    RoDatInf.Cradle.Inf_GetCradleData(E_CradleRxPosition::SIDE_LEFT,rxData,&rxDataCnt);
    if (rxDataCnt > 0)
    {
        bCheckUpdate = true;    // 한개라도 업데이트 되면 체크 한다.
        // eblog(LOG_LV, "rxDataCnt 1 : "<<rxDataCnt);
    }
    tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].count = rxDataCnt;    
    for (u32 i = 0; i < tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].count; i++)
    {
        tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].Data[i] = rxData[i];
        tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].timeStamp[i] = presentTime;    
        // ceblog(LOG_LV_DOCKING, GREEN, "Side LEFT ["<< tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].Data[i]<< "] 데이터의 개수 ["<<tempSignal.ChargerIR[RECEIVER_SIDE_LEFT].count<<"] time[" << PRECISION(5) <<SYSTEM_TOOL.getSystemTime()<< "]");
    }
    rxDataCnt = 0;    
    /************* SIDE LEFT *************/

    /************* SIDE RIGHT *************/
    RoDatInf.Cradle.Inf_GetCradleData(E_CradleRxPosition::SIDE_RIGHT,rxData,&rxDataCnt);
    if (rxDataCnt > 0)
    {
        bCheckUpdate = true;    // 한개라도 업데이트 되면 체크 한다.
        // eblog(LOG_LV, "rxDataCnt 1 : "<<rxDataCnt);
    }
    tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].count = rxDataCnt;    
    for (u32 i = 0; i < tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].count; i++)
    {
        tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].Data[i] = rxData[i];
        tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].timeStamp[i] = presentTime;    
        // ceblog(LOG_LV_DOCKING, CYN, "Side RIGHT ["<< tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].Data[i]<< "] 데이터의 개수 ["<<tempSignal.ChargerIR[RECEIVER_SIDE_RIGHT].count<<"] time[" << PRECISION(5) <<SYSTEM_TOOL.getSystemTime()<< "]");
    }
    rxDataCnt = 0;
    /************* SIDE RIGHT *************/
    
    // 업데이트 됐을때만 복사 연산자 실행.
    if (bCheckUpdate)
    {
        signalData = tempSignal; //데이터 업데이트 됐을때만 복사
        bUpdateStateSignalData = true;
        sigList.emplace_back(tempSignal);
    } 
}
// Charge Signal data - 업데이트 확인
bool CSystemInterface::isUpdateSignalData()
{
    return bUpdateStateSignalData;
}
// Charge Signal data - 데이터 읽기
std::list<tSysSignal> CSystemInterface::useSignalData(void)
{
    bUpdateStateSignalData = false;
    std::list<tSysSignal> ret = sigList;
    sigList.clear();
    return ret;
}

std::list<tSysSignal> CSystemInterface::getSignalData(void)
{
    std::list<tSysSignal> ret = sigList;
    return ret;
}

/////////////////////////////////////////////////////////////
// System Pose data - 데이터 쓰기
void CSystemInterface::setSysPoseData()
{
    struct timespec apTime;
    tSysPose tempSysPose;
    s32 tempX = 0, tempY = 0, tempA = 0;
    float tempfA = 0;   
    u32 TempCalSn = 0; 
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    RoDatInf.Localization.Inf_GetLocalizationData(&tempX, &tempY, &tempA, &tempfA, &TempCalSn, &apTime);
#else
    RoDatInf.Localization.Inf_GetLocalizationData(&tempX, &tempY, &tempA, &apTime);
#endif
    ros::Time now = convertToRosTime(apTime);
    ros::Duration delay(LATE_OF_ODOM_SEC);
    ros::Time before = now - delay;
    tempSysPose.timeStamp = before; //before;

    // 결과 출력
    // ceblog(LOG_LV_NECESSARY, MAGENTA,"time stamp: " << tempSysPose.timeStamp.toSec());
    tempSysPose.x = (double)tempX / 1000.0;
    tempSysPose.y = (double)tempY / 1000.0;
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    tempSysPose.angle = utils::math::deg2rad((double)tempfA);
    tempSysPose.calSn = TempCalSn;
#else 
    tempSysPose.angle = utils::math::deg2rad((double)tempA / 10.0);
#endif
    sysPoseData = tempSysPose;
    bUpdateStateSysPoseData = true;

    //ceblog(LOG_LV_NECESSARY, MAGENTA, "tempPose\tx: " << tempX << "\ty: " << tempY << "\tangle: " << tampfA);
    //ceblog(LOG_LV_NECESSARY, MAGENTA, "systemPose\tCalSN: " << tempSysPose.calSn << "systemPose\tx: " << tempSysPose.x << "\ty: " << tempSysPose.y << "\tangle: " << tempSysPose.angle);
}

unsigned int CSystemInterface::getMcuTimeStamp(void)
{
    return mcuTime;
}

unsigned int CSystemInterface::makeMcuOffset(unsigned int mcuTimeStamp)
{
    if (!initMcuTime) {
        initMcuTime = true;
    }
    unsigned int offset = (mcuTimeStamp - preMcuTime);
    preMcuTime = mcuTimeStamp;

    return offset;
}

unsigned int CSystemInterface::makeApOffset(ros::Time rosTime)
{
        if (!initApTime) {
            initApTime = true;
            startApTime = rosTime;
        }
        unsigned int offset = (rosTime.nsec - preApTime);
        preApTime = rosTime.nsec;

        return offset;
}


// System Pose data - 업데이트 확인
bool CSystemInterface::isUpdateSysPoseData()
{
    return bUpdateStateSysPoseData;
}

/**
 * @brief System Pose data - 데이터 읽기
 *
 * @return tPose (m, m, deg)
 */
tSysPose CSystemInterface::useSysPoseData()
{
#if 0 // ROS시간 debug
    ceblog(LOG_LV_NECESSARY, BOLDBLACK,
        "RosTime["<<BOLDYELLOW<<sysPoseData.timeStamp<<BOLDBLACK<<"]\t"<<BOLDGREEN<<
        setw(6)<<sysPoseData.x<<
        setw(6)<<sysPoseData.y<<
        setw(6)<<sysPoseData.angle);
#endif
    bUpdateStateSysPoseData = false;
    return sysPoseData;
}

/**
 * @brief 
 * 
 * @return tPose 
 */
tSysPose CSystemInterface::getSysPoseData()
{
    return sysPoseData;
}

/////////////////////////////////////////////////////////////
// Tilting data - 데이터 쓰기
void CSystemInterface::setTiltingData()
{
    tSysTilting temp;
    switch ( RoDatInf.Tilting.Inf_GetTiltingState() )
    {
    case E_TiltingState::Uknown:
        temp.state = E_SYS_TILT_STATE::UNKNOW;
        // ceblog(LOG_LV_SYSTEMINF, CYN, "Uknown");
        break;
    case E_TiltingState::UP:
        temp.state = E_SYS_TILT_STATE::TILTED_UP;
        // ceblog(LOG_LV_SYSTEMINF, CYN, "UP");
        break;
    case E_TiltingState::DOWN:
        temp.state = E_SYS_TILT_STATE::TILTED_DOWN;
        // ceblog(LOG_LV_SYSTEMINF, CYN, "DOWN");
        break;
    case E_TiltingState::MOVING_UP:
        temp.state = E_SYS_TILT_STATE::TILING_UP;
        // ceblog(LOG_LV_SYSTEMINF, CYN, "MOVING_UP");
        break;
    case E_TiltingState::MOVING_DOWN:
        temp.state = E_SYS_TILT_STATE::TILING_DOWN; 
        // ceblog(LOG_LV_SYSTEMINF, CYN, "MOVING_DOWN");
        break;
    case E_TiltingState::MOVING_STOP:
        temp.state = E_SYS_TILT_STATE::TILING_STOP; 
        // ceblog(LOG_LV_SYSTEMINF, CYN, "MOVING_STOP");
        break;
    case E_TiltingState::ERROR_STOP:
        temp.state = E_SYS_TILT_STATE::TILT_ERROR;  
        // ceblog(LOG_LV_SYSTEMINF, CYN, "ERROR_STOP");
        break;
    case E_TiltingState::CURRENT:
        temp.state = E_SYS_TILT_STATE::TILT_ERROR;  
        // ceblog(LOG_LV_SYSTEMINF, CYN, "CURRENT");   
        break;
    default:
        temp.state = E_SYS_TILT_STATE::TILT_ERROR;  
        // ceblog(LOG_LV_SYSTEMINF, CYN, "???");       
        break;
    }
    tiltingData = temp;
    bUpdateStateTiltingData = true;

}
// Tilting data - 업데이트 확인
bool CSystemInterface::isUpdateTiltingData()
{
    return bUpdateStateTiltingData;
}
tSysTilting CSystemInterface::useTiltingData()
{
    bUpdateStateTiltingData = false;
    return tiltingData;
}

// Tilting data - 데이터 읽기
tSysTilting CSystemInterface::getTiltingData()
{    
    return tiltingData;
}
/////////////////////////////////////////////////////////////
// Error data - 데이터 쓰기
void CSystemInterface::setError()
{
    u32 tempErrorData;
    RoDatInf.GetError(&tempErrorData);
    
    errorData = tempErrorData;

    bUpdateStateErrorData = true;

#if 0 // 디버깅용
    std::bitset<32> bits(tempErrorData);
    eblog(LOG_LV_NECESSARY, "[System Interface] tempError Data : " << tempErrorData);
    eblog(LOG_LV_NECESSARY, "[System Interface] get Error(bit) Data : " << bits);
#endif 
}

bool CSystemInterface::isUpdateErrorData()
{
    return bUpdateStateErrorData;
}
u32 CSystemInterface::useErrorData()
{
    bUpdateStateErrorData = false;
    return errorData;
}    
u32 CSystemInterface::getErrorData()
{
    return errorData;
}
void CSystemInterface::clearError(unsigned int nClear)
{
    RoCtrInf.Error.Inf_ErrorClear(nClear);
}
#if 0
/////////////////////////////////////////////////////////////
// Wheel Current data - 데이터 쓰기
void CSystemInterface::setWheelCurrentData()
{
    tSysWheelMotor tempCurrent;    
    RoDatInf.Drive.Inf_GetDriveCurrentData(&tempCurrent.leftCurrent, &tempCurrent.rightCurrent, &tempCurrent.dummyCurrent);
    wheelCurrentData = tempCurrent;
    bUpdateStateWheelCurrentData = true;
}
// Wheel Current data - 업데이트 확인
bool CSystemInterface::isUpdateWheelCurrentData()
{
    return bUpdateStateWheelCurrentData;
}
// Wheel Current data - 데이터 읽기
tSysWheelMotor CSystemInterface::getWheelCurrentData()
{
    bUpdateStateWheelCurrentData = false;
    return wheelCurrentData;
}

/////////////////////////////////////////////////////////////
// Wheel Encoder data - 데이터 쓰기
void CSystemInterface::setWheelEncoderData(tSysWheelMotor data)
{
    bUpdateStateWheelEncoderData = true;
    wheelEncoderData = data;
}
// Wheel Encoder data - 업데이트 확인
bool CSystemInterface::isUpdateWheelEncoderData()
{
    return bUpdateStateWheelEncoderData;
}
// Wheel Encoder data - 데이터 읽기
tSysWheelMotor CSystemInterface::getWheelEncoderData()
{
    bUpdateStateWheelEncoderData = false;
    return wheelEncoderData;
}
/////////////////////////////////////////////////////////////
#else

/////////////////////////////////////////////////////////////

// Wheel Motor data - 업데이트 확인
bool CSystemInterface::isUpdateWheelMotorData()
{
    return bUpdateStateWheelMotorData;
}
// Wheel Motor data - 데이터 읽기
tSysWheelMotor CSystemInterface::useWheelMotorData()
{
    bUpdateStateWheelMotorData = false;
    return wheelMotorData;
}

tSysWheelMotor CSystemInterface::getWheelMotorData()
{
    return wheelMotorData;
}
/////////////////////////////////////////////////////////////
#endif

void CSystemInterface::setOtaData()
{
    int otaState = RoOtaInf.GetPercent();//RoOtaInf.GetState();
    int otaPercent = RoOtaInf.GetState();//RoOtaInf.GetPercent();
    bool isOpen = RoOtaInf.IsOpen();
    if(otaData.bOpen == false && isOpen == true){
        ceblog(LOG_LV_NECESSARY, CYN, "[setOtaData] OTA - OPEN : ");
    }
    else if(otaData.bOpen == true && isOpen == false){
        ceblog(LOG_LV_NECESSARY, CYN, "[setOtaData] OTA - CLOSE : ");
    }
    otaData.bOpen = isOpen;
    if(otaData.state != otaState){
        ceblog(LOG_LV_NECESSARY, CYN, "[setOtaData] OTA - STATE CHANGED : " << otaData.state );
    }
    otaData.state = otaState;
    otaData.pecent = otaPercent;
    bUdateStateOtaData = true;
    
    
}

bool CSystemInterface::isUpdateOtaData()
{
    return bUdateStateOtaData;
}
// Pump data - 데이터 읽기
tSysOta CSystemInterface::useOtaData()
{
    bUdateStateOtaData = false;
    return otaData;
}
tSysOta CSystemInterface::getOtaData()
{
    return otaData;
}

void CSystemInterface::transCheckAlive(u8 state)
{
    // RoDatInf.TransCheckAlive(state);
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] Data Write CheckAlive ( "<< state << " ) ");
}

void CSystemInterface::transLocalization(void)
{
    //RoDatInf.TransLocalization(x, y, yaw);
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] Data Write Localize INIT");
    RoCtrInf.Localization.Inf_LocalInit();  
}

/* water Pump Contorl interface */
void CSystemInterface::transControlPump(bool on)
{
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] transControlPump [ "<< on << " ]");
    if(on)  RoCtrInf.Pump.Inf_PumpOn();
    else    RoCtrInf.Pump.Inf_PumpOff();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CSystemInterface::transInitSensor()
{
    bool ret = false;
    int retryCnt = 0;
    
    ret = RoCtrInf.Sensor.Inf_SensorInit();
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] Data Write TOF INIT");
    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false && SEND_RETRY_MAX > ++retryCnt)
    {
        eblog(LOG_LV_ERROR, "retry cnt"<< retryCnt);
        ret = RoCtrInf.Sensor.Inf_SensorInit();
        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
    }
    
    return ret;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CSystemInterface::transInitMoving()
{
    bool ret = false;
    int retryCnt = 0;    
    
    ret = RoCtrInf.MovingData.Inf_MovingDataInit();
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] Data Write IMU INIT");
    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false && SEND_RETRY_MAX > ++retryCnt)
    {
        eblog(LOG_LV_ERROR, "retry cnt"<< retryCnt);
        ret = RoCtrInf.MovingData.Inf_MovingDataInit();
        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
    }
    
    return ret;
}

bool CSystemInterface::transActiveMode()
{
    bool ret = false;
    int retryCnt = 0;    
    
    ret = RoCtrInf.Power.Inf_EntryActiveMode();
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] transActiveMode");
    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false && SEND_RETRY_MAX > ++retryCnt)
    {
        eblog(LOG_LV_ERROR, "retry cnt"<< retryCnt);
        ret = RoCtrInf.Power.Inf_EntryActiveMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
    }
    
    return ret;
}
bool CSystemInterface::transChargeMode()
{
    bool ret = false;
    int retryCnt = 0;    
    
    ret = RoCtrInf.Power.Inf_EntryChargeMode();
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] transChargeMode");
    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false && SEND_RETRY_MAX > ++retryCnt)
    {
        eblog(LOG_LV_ERROR, "retry cnt"<< retryCnt);
        ret = RoCtrInf.Power.Inf_EntryChargeMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
    }
    
    return ret;
}

// bool CSystemInterface::transCradleIrOff()
// {
//     bool ret = false;
//     int retryCnt = 0;    
    
//     ret = RoCtrInf.Power.Inf_

//     //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
//     while(ret == false && SEND_RETRY_MAX > ++retryCnt)
//     {
//         eblog(LOG_LV_SYSTEMINF, "retry cnt"<< retryCnt);
//         ret = RoCtrInf.Power.Inf_EntryChargeMode();
//         std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
//     }
    
//     return ret;
// }

void CSystemInterface::transControlPowerOff()
{
    ceblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), RED, "[System Interface] Power Off !!!!");
    RoCtrInf.Power.Inf_EntryPowerOffMode();
}

void CSystemInterface::transControlMcuReset()
{
    ceblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), RED, "[System Interface] MCU RESET !!!!");
    RoCtrInf.Power.Inf_McuReset();
}

void CSystemInterface::debug_print()
{
   int ret = system("clear");

   eblog(LOG_LV, "_____________________________");
   eblog(LOG_LV, "system debug");
   eblog(LOG_LV, "button       : start[ " << buttonData.buttonStart  <<" ] home[ " << buttonData.buttonHome  <<" ] func[ " << buttonData.buttonFunc << " ] ");
   eblog(LOG_LV, "systempose   : x[ " << sysPoseData.x << " ] y[ "<< sysPoseData.y << "] a[ " << sysPoseData.angle << " ]");
   eblog(LOG_LV, "wheelEncoder : STATE[ " << wheelMotorData.actuatorStatus << " ] ");
   eblog(LOG_LV, "   >L Current[ " <<  wheelMotorData.leftCurrent << " ] Enc[ " << wheelMotorData.leftEncoder << " ] sumEnc[ " << wheelMotorData.leftEncAccumCnt << " ] ");
   eblog(LOG_LV, "   >R Current[ " <<  wheelMotorData.rightCurrent << " ] Enc[ " <<wheelMotorData.rightEncoder << " ] sumEnc[ " <<  wheelMotorData.rightEncAccumCnt <<" ] ");
   eblog(LOG_LV, "_____________________________");
   
}

//  - bool Inf_DockingIrOn();
//  - bool Inf_DockingIrOff();

/**
 * @brief docking IR 제어 함수
 * 
 * @param on 
 */
bool CSystemInterface::transControlDockingIR(bool on)
{
    bool bRet = false;
    int retryCnt = 0;

    do
    {
        eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] onoff : " <<SC<int>(on));
        
        if(on) bRet = RoCtrInf.Cradle.Inf_DockingIrOn();
        else   bRet = RoCtrInf.Cradle.Inf_DockingIrOff();

        if (bRet == false)
        {
            if(++retryCnt > SEND_RETRY_MAX)
            {
                eblog(LOG_LV_ERROR, "***ERROR*** retryCnt MAX!!!");
#if TODO_LIST
                // 커뮤니케이션 에러 띄우기;
#endif
                break;
            }
            else
            {
                eblog(LOG_LV_SYSTEMINF, "retry cnt : " << retryCnt);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SEND_RETRY_DELAY));
        }
    } while (!bRet);

    return bRet;
}

/**
 * @brief mcu time convert to ros::time
 * 
 * @param time_spec 
 * @return ros::Time 
 */
ros::Time CSystemInterface::convertToRosTime(const timespec& time_spec) 
{
    // Convert seconds and nanoseconds to a single value in seconds
    if (time_spec.tv_sec < 0 ) // || time_spec.tv_sec > std::numeric_limits<uint32_t>::max())
    {
        eblog(LOG_LV_ERROR, "***ERROR*** negative time value received.");
        ros::Time cur_rosTime = ros::Time::now();
    }
	else
    {
        // timespec 구조체의 초와 나노초를 사용하여 ros::Time 객체 생성
        ros::Time ros_time(time_spec.tv_sec, time_spec.tv_nsec);
        return ros_time;
    }
    //double timeInSeconds = ts.tv_sec + (static_cast<double>(ts.tv_nsec) / 1e9);
    // Create a ros::Time object with the calculated value
    //return ros::Time(timeInSeconds);
}

void CSystemInterface::transOtaOnOff(bool on)
{
    if(on)  RoOtaInf.Open();
    else    RoOtaInf.Close();
    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] transOtaOnOff : " << (int)on);
}
void CSystemInterface::transOtaStart(char* _path){

    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface]transOtaStart path : " << _path);
    RoOtaInf.Start(_path);
}

       
        
       
        
