/**
 * @file display.h
 * @author jmk1
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
#include "commonStruct.h"
#include "Message.h"
#include "coreData/serviceData/button.h"
#include "coreData/serviceData/batteryState.h"
#include "robot_framework/errorHandler.h"
#include "LibRobotControlDisplay.h"

enum class DISPLAY_STATUS
{
    VOID,
    STOP,
    START,
    RUN,
};

static std::string enumToString(DISPLAY_STATUS value) {
    static const std::unordered_map<DISPLAY_STATUS, std::string> enumToStringMap = {
        { DISPLAY_STATUS::VOID, "VOID,"},
        { DISPLAY_STATUS::STOP, "STOP,"},
        { DISPLAY_STATUS::START, "START,"},
        { DISPLAY_STATUS::RUN, "RUN,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

class CDisplay
{
public:
    CDisplay(/* args */);
    ~CDisplay();

private:

    DISPLAY_STATUS status;
    E_DISPLAY_STATE displayState;
    E_DisplayImageClass curImage;
    E_DisplayImageClass desImage;
    E_DisplayImageClass autoImage;
    char* curCustomImg;
    char* desCustomImg;
    bool bAutoUpdate;
    bool bCustomDisplay;
    double startTime;
    void setDisplayStatus(DISPLAY_STATUS set);
    void LCDStateMonitor();
    void sendDisplayStopMassage();
    void sendDisplayMassage(E_DisplayImageClass img);
    void sendCustomMassage(char* img);
public:

    void startDisplay(E_DisplayImageClass img);
    void startAutoDisplay(bool nowPlay, E_DisplayImageClass img);
    void stopAutoDisplay();
    void startCustomDisplay(char* img);
    void updateDisplay();
    bool isLCDPlaying();
    
    void setCurImage(E_DisplayImageClass img);
    void setCustomImage(char* img);
    void setDisplayState(E_DISPLAY_STATE state);
    
    void runExploreDisplay();

    E_DisplayImageClass getBlinkEyeImage();
    E_DisplayImageClass getChargingImage();

    //커스텀 LCD
	void printfLCDClear();	
    void DisplayQRCodeOnLCD(const char *data, int lcd_width, int lcd_height, int qr_size, int x_pos, int y_pos, int Color, int border_size, int border_color, int margin);
};