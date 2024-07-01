#include "sound.h"
#include "systemTool.h"
#include "MessageHandler.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CSound::CSound(/* args */)
{
    setSoundStatus(SOUND_PLAY_STATUS::VOID);
}

CSound::~CSound()
{

}

void CSound::setSoundStatus(SOUND_PLAY_STATUS set)
{
    if(set != status)
    {
        ceblog(LOG_LV_NECESSARY, BLUE, "status : " << enumToString(status) << " set : " << enumToString(set));
    }
    status = set;
}

void CSound::soundStateMonitor()
{
    switch (status)
    {
    case SOUND_PLAY_STATUS::VOID :
        
        break;
    case SOUND_PLAY_STATUS::STOP :
        if(soundState == E_SOUND_STATE::STOP){
            sendSoundMessage(desSound);
            ceblog(LOG_LV_NECESSARY, BLUE, "SOUND STOP CONFIRM!!");
            setSoundStatus(SOUND_PLAY_STATUS::START);
        }
        break;    
    case SOUND_PLAY_STATUS::START :
        if(soundState == E_SOUND_STATE::PLAYING){
            ceblog(LOG_LV_NECESSARY, BLUE, "SOUND PLAY RUN!!");
            setSoundStatus(SOUND_PLAY_STATUS::RUN);
        }
        break;
    case SOUND_PLAY_STATUS::RUN :
         if(soundState == E_SOUND_STATE::STOP){
            ceblog(LOG_LV_NECESSARY, BLUE, "SOUND PLAY COMPLETE!!");
            setSoundStatus(SOUND_PLAY_STATUS::VOID);
        }
        break;
    default:
        ceblog(LOG_LV_NECESSARY, BLUE, "SOUND PLAY STATUS ERROR!!");
        break;
    }
}

void CSound::soundPlay(bool stop, E_SoundClass sound)
{
    if(stop){
        setSoundStatus(SOUND_PLAY_STATUS::STOP);
        sendSoundStopMessage();
    }else{
        setSoundStatus(SOUND_PLAY_STATUS::START);
        sendSoundMessage(sound);
    }
    
    desSound = sound;
}

bool CSound::isSoundPlaying()
{
    bool ret = false;
    if(status != SOUND_PLAY_STATUS::VOID) ret = true;
    return ret;
}

void CSound::sendSoundStopMessage()
{
    tMsgCtrSound msg = {};
    msg.stop = true;
    ceblog(LOG_LV_NECESSARY, BLUE, "Sound Off ");
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_SOUND, msg));
}
void CSound::sendSoundMessage(E_SoundClass sound)
{
    tMsgCtrSound msg = {};
    msg.stop = false;
    msg.data = sound;
    ceblog(LOG_LV_NECESSARY, BLUE, "Sound Play : " << DEC(msg.data));
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_SOUND, msg));
}

/**
 * @brief 사운드 재생의 상태값을 설정한다. 
 * systeminterface_ts800.cpp에서 사용하는 함수라서 다른 곳에서 사용 안함
 * @param soundState :  디스플레이 재생의 상태 
 */
void CSound::setSoundState(E_SOUND_STATE state)
{
    soundState = state;
}

E_SoundClass CSound::getPlayingSound()
{
    return curSound;
}

void CSound::setCurSound(E_SoundClass sound)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "Play Sound : " << DEC(sound) );
    curSound = sound;
}

E_SoundClass CSound::getDisireSound()
{
    return desSound;
}





