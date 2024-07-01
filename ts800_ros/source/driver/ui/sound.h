/**
 * @file sound.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <iostream>
#include <unordered_map>
#include "commonStruct.h"
#include "Message.h"
#include "robot_framework/errorHandler.h"
#include "externData/systemData.h"
#include "LibRobotSoundInterface.h"

enum class SOUND_PLAY_STATUS
{
    VOID,
    STOP,
    START,
    RUN,
};

static std::string enumToString(SOUND_PLAY_STATUS value) {
    static const std::unordered_map<SOUND_PLAY_STATUS, std::string> enumToStringMap = {
        { SOUND_PLAY_STATUS::VOID, "VOID,"},
        { SOUND_PLAY_STATUS::STOP, "STOP,"},
        { SOUND_PLAY_STATUS::START, "START,"},
        { SOUND_PLAY_STATUS::RUN, "RUN,"},
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef struct _tVolumeSound
{
    _tVolumeSound() : size{E_VOLUME_STEP::VOL_STEP_3}, period{0.0} {}
    E_VOLUME_STEP size;
    double period;
}tVolumeSound;

class CSound
{
public:
    CSound();
    ~CSound();
    
private:
    tVolumeSound volume;

    SOUND_PLAY_STATUS status;
    E_SOUND_STATE soundState;
    E_SoundClass curSound;
    E_SoundClass desSound;
    

public:
    void soundStateMonitor();
    bool isSoundPlaying();
    void soundPlay(bool stop, E_SoundClass sound);

    void setSoundStatus(SOUND_PLAY_STATUS set);
    void setSoundState(E_SOUND_STATE state);

    E_SoundClass getPlayingSound();
    void setCurSound(E_SoundClass sound);

    E_SoundClass getDisireSound();
private:
    void sendSoundStopMessage();
    void sendSoundMessage(E_SoundClass sound);                         
};