/**
 * @file userInterface.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "eblog.h"
#include "led.h"
#include "sound.h"
#include "display.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"


#define USER_INTERFACE CUserInterface::getInstance() 
#define LED_CTR CUserInterface::getInstance().led
#define SOUND_CTR CUserInterface::getInstance().sound
#define DISPLAY_CTR CUserInterface::getInstance().display

class CUserInterface
{
private:
    CUserInterface();
    CUserInterface(const CUserInterface& ref);
    ~CUserInterface();

public:
    CLed led;
    CSound sound;
    CDisplay display;

    static CUserInterface& getInstance(); 

    void initUi();
    void updateUi();

};