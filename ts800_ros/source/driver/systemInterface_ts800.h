/**
 * @file systemInterface_ts800.h
 * @author jspark
 * @brief TS800에 해당하는 systemInterface만 추가해주세요.
 * @version 0.1
 * @date 2023-04-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <pthread.h>
#include "define.h"
#include "systemInterface.h"
#include "control/control.h"
#include "lcdDebug.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"


class CSystemInterfaceTs800 : public CSystemInterface
{
public:
    CSystemInterfaceTs800();
    ~CSystemInterfaceTs800();
    bool connect() override;
    bool disConnect() override;

private:    
    pthread_t thSystemInterface;

    static void* threadLoopWrap(void* arg)
    {
        CSystemInterfaceTs800* mySystemInterface = static_cast<CSystemInterfaceTs800*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_SYSTEM_INTERFACE);
        mySystemInterface->threadLoop();
    }
    void threadLoop();


protected:
    void setWheelMotorData() override;
    //void setDisplayStateData() override;

public: /* TS800용 trans 함수들 */
    bool transControlWheel(u8 direction, s32 L_Speed, s32 R_Speed, s32 B_Speed, u32 duty, bool bHeadingPid);
    bool transControlWheel(int seq, u8 direction, s32 L_Speed, s32 R_Speed, s32 B_Speed, u32 duty, bool bHeadingPid);
    bool transControlWheel(int seq, s32 linearVelocity, s32 angularVelocity, u32 radius);
    void transControlTilt(E_TILTING_CONTROL control);       // Tilting
    void transControlDryFan(bool on,u16 speed) override;    //Cradle Fan
    void transDockingAlive();                               //Cradle signal

    void transControlCustomDisplay(char* img);
    void transControlDisplay(E_DisplayImageClass img);             //LCD
    void transControlDisplayStop() override;
    void setDisplayStateData() override;
    void transControlLed(u32 dir, u8 r, u8 g, u8 b, u8 w);        //LED
    void transControlSoundplay(E_SoundClass sound);           //Sound
    void transControlSoundStop() override;
    void setSoundStateData() override;
    void transSlamPoseData(int x, int y, unsigned int angle);
    //RBT_PlUS new
    void transSlamPoseData(int x, int y, float fA, int nTimeOffset);    
};