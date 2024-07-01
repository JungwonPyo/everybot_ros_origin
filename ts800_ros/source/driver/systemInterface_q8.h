/**
 * @file systemInterface_q8.h
 * @author jspark
 * @brief Q8에 해당하는 systemInterface만 추가해주세요.
 * @version 0.1
 * @date 2023-04-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "define.h"
#include "systemInterface.h"

class CSystemInterfaceQ8 : public CSystemInterface
{
public:
    CSystemInterfaceQ8();
    ~CSystemInterfaceQ8();
    bool connect() override;
    bool disConnect() override;

private:
    std::thread thSystemInterface;
    void threadLoop();

protected:
    void setWheelMotorData() override;
    void setDisplayStateData() override;
    void setSoundStateData() override;

public:
    bool transControlWheel(u8 direction, s32 L_Speed, s32 R_Speed, u32 duty, bool bHeadingPid);    
    bool transControlSuction(u8 value);
    bool transControlMainBrush(u8 value);
    bool transControlSideBrush(u8 value);
    void transControlDryFan(bool on,u16 speed) override;
    void transControlDisplayStop() override;
    void transControlSoundStop() override;
};