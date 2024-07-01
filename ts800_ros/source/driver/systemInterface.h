/**
 * @file systemInterface.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <mutex>

#include "define.h"
#include "eblog.h"
#include "ebtime.h"
#include "interfaceStruct.h"


class CSystemInterface
{

public:
    CSystemInterface(/* args */);
    ~CSystemInterface();
    virtual bool connect() = 0;
    virtual bool disConnect() = 0;

protected:
    void threadLoop();
    bool bReceiveDataLoopRunning;
    bool bDebugBumpPush;
private:
    uint32_t imuCombinedState;
    uint8_t filterdImuState;
public:
    /* 배터리 data */
    bool isUpdateBatteryData();
    tSysBattery useBatteryData();    
    tSysBattery getBatteryData();

    /* 버튼 data */
    bool isUpdateButtonData();
    tSysButton useButtonData();    
    tSysButton getButtonData();
    
    /* 범퍼 data */
    bool isUpdateBumperData();
    tSysBumper useBumperData();    
    tSysBumper getBumperData();
    
    /* Check Alive data */
    bool isUpdateCheckAlive();
    u8 useCheckAliveData();    
    u8 getCheckAliveData();
    
    /* 낙하감지 data */
    bool isUpdateCliffData();
    bool isUpdateTofData();
    tSysCliff useCliffData();
    tSysCliff getCliffData();
    tTof useTofData();
    tTof getTofData();

    bool isUpdateCliffActionState();
    tCliffActionState useCliffActionState();
    tCliffActionState getCliffActionState();

    /* 전방 IR data */
    bool isUpdateFrontIRData();
    tSysFront useFrontIRData();    
    tSysFront getFrontIRData();
    
    /* Imu data */
    bool isUpdateImuData();
    tSysIMU useImuData();
    tSysIMU getImuData();

    /* Power data */
    bool isUpdatePowerData();
    tSysPower usePowerData();    
    tSysPower getPowerData();
    
    /* Pump data */
    bool isUpdatePumpData();
    tSysPump usePumpData();    
    tSysPump getPumpData();
    
    /* Remote Key data */
    bool isUpdateRemoteKeyData();
    tSysRemote useRemoteKeyData();    
    tSysRemote getRemoteKeyData();

    /* Signal data */
    bool isUpdateSignalData();    

    std::list<tSysSignal> useSignalData();
    std::list<tSysSignal> getSignalData();
    /* System Pose data*/
    bool isUpdateSysPoseData();    
    tSysPose useSysPoseData();
    tSysPose getSysPoseData();

    unsigned int getMcuTimeStamp(void);

    /* Tilting data */
    bool isUpdateTiltingData();    
    tSysTilting useTiltingData();
    tSysTilting getTiltingData();

    /* Wheel Motor data */
    bool isUpdateWheelMotorData();
    tSysWheelMotor useWheelMotorData();
    tSysWheelMotor getWheelMotorData();

    /* error data */
    bool isUpdateErrorData();
    u32 useErrorData();    
    u32 getErrorData();
    void clearError(unsigned int nClear);

    /* ota data */
    bool isUpdateOtaData();
    tSysOta useOtaData();    
    tSysOta getOtaData();

protected:
    /* 배터리 데이터 */
    bool bUpdateSysBatteryData;
    tSysBattery sysBatteryData;
    void setBatteryData();

    /* 버튼 데이터 */
    bool bUpdateStateButtonData;
    tSysButton buttonData;
    void setButtonData();

    /* 범퍼 data */
    bool bUpdateStateBumperData;
    tSysBumper bumperData;
    void setBumperData();    

    /* Check Alive data */
    bool bUpdateStateCheckAliveData;
    u8 checkAliveData; // 임시 생성
    void setCheckAliveData();

    /* ToF data */
    bool bUpdateStateCliffData;
    bool bUpdateStateTofData;
    tSysCliff cliffData;
    tTof TofData;
    void setTofData();

    bool bUpdateStateCliffActionState;
    unsigned int CliffActionState;
    void setCliffActionState();

    /* 전방 IR data */
    bool bUpdateStateFrontIRData;
    tSysFront FrontIRData;
    void setFrontIRData();

    /* Imu data */
    bool bUpdateStateImuData;
    tSysIMU imuData;
    void setImuData();
    uint8_t imuStateFilter(unsigned char state);
 
    /* Power data */
    bool bUpdateaStateSysPowerData;
    tSysPower sysPowerData;
    void setSystemPowerData();

    /* Pump data */
    bool bUpdateStatePumpData;
    tSysPump pumpData;
    void setPumpData(tSysPump data);

    /* Remote Key data */
    bool bUpdateStateRemoteKeyData;
    tSysRemote remoteData;
    void setRemoteKeyData();

    /* Charge Signal data */
    bool bUpdateStateSignalData;
    tSysSignal signalData;
    std::list<tSysSignal> sigList;
    void setSignalData();

    /* System Pose data*/
    bool bUpdateStateSysPoseData;
    tSysPose sysPoseData;
    void setSysPoseData();

    /* Error data*/
    bool bUpdateStateErrorData;
    u32 errorData;
    void setError();

    bool bUdateStateOtaData;
    tSysOta otaData;
    void setOtaData();

    ros::Time convertToRosTime(const timespec& time_spec);

    /* 디버깅용 변수 및 함수 */
    unsigned int makeMcuOffset(unsigned int mcuTimeStamp);
    unsigned int makeApOffset(ros::Time rosTime);

    bool initMcuTime = false;
    unsigned int preMcuTime;
    ros::Time startApTime;
    bool initApTime = false;
    unsigned int preApTime;
    unsigned int mcuTime;

    /* Tilting data */
    bool bUpdateStateTiltingData;
    // hhryu231106 : cliff에서 tilt state 사용할 수 있도록 public으로 임시, 예외 수정 //tSysTilting tiltingData;
    void setTiltingData();

    /* Wheel Motor data */
    tSysWheelMotor wheelMotorData;
    bool bUpdateStateWheelMotorData;
    virtual void setWheelMotorData() = 0;

    /* Display State data */
    virtual void setDisplayStateData() = 0;

    /* Sound State data */
    virtual void setSoundStateData() = 0;

    void debug_print();

public: /* trans data 함수들 */
    void transLocalization(void); //bool transLocalization(double x, double y, double yaw);
    void transCheckAlive(u8 state);
    void transControlPump(bool on); //// u16 rpm
    bool transInitSensor();
    bool transInitMoving();
    bool transActiveMode();
    bool transChargeMode();
    virtual void transControlDryFan(bool on,u16 speed) = 0;
    void transControlPowerOff();
    void transControlMcuReset();
    // bool transCradleIrOff(bool on);
    bool transControlDockingIR(bool on);
    
    void transOtaOnOff(bool on);
    void transOtaStart(char* _path);

    virtual void transControlDisplayStop() = 0;
    virtual void transControlSoundStop() = 0;

public: 
    // hhryu231106 : cliff에서 tilt state 사용할 수 있도록 public으로 임시, 예외 수정. 
    // getTiltingData을 하면 bUpdateStateTiltingData가 reset되기 때문에 data를 public으로 수정함.... 방식 고민필요.
    tSysTilting tiltingData;
};
