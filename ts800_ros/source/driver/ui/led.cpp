#include "led.h"
#include "MessageHandler.h"
#include "systemTool.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

using namespace std;

CLed::CLed()
{
    
}

CLed::~CLed()
{
    
}

void CLed::playLed(u32 dir, u8 r, u8 g, u8 b, u8 w)
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_LED;
    
    tMsgCtrLed LEDMsg = {};

    LEDMsg.dir = dir;
    LEDMsg.red = r;
    LEDMsg.green = g;
    LEDMsg.blue = b;
    LEDMsg.white = w;

    SEND_MESSAGE(message_t(type, LEDMsg));
}

void CLed::ledFunc(E_LED_DIRECTION dir, E_LED_COLOR color)
{
    eblog(LOG_LV, "tmpLED E_LED_DIRECTION : "<<dir<<" ,E_LED_COLOR : "<<color);
    if (color == E_LED_COLOR::LED_OFF) 
    {
        playLed(0,0,0,0,0);
        playLed(1,0,0,0,0);
    }
    else if (color == E_LED_COLOR::LED_RED)
    {   
        eblog(LOG_LV, "test LED RED");
        switch (dir)
        {
        case CENTER_LEFT:
            playLed(0,LED_LUX,0,0,0);
            playLed(1,0,0,0,0);
            break;
        case CENTER_RIGHT:
            playLed(0,0,0,0,0);
            playLed(1,LED_LUX,0,0,0);
            break;
        case LED_ALL_ON :
            playLed(0,LED_LUX,0,0,0);
            playLed(1,LED_LUX,0,0,0);
            break;    
        default:
            break;
        }
    }
    else if (color == E_LED_COLOR::LED_GREEN)
    {
        // eblog(LOG_LV, "test LED GREEN");
        switch (dir)
        {
        case CENTER_LEFT:
            playLed(0,0,LED_LUX,0,0);
            playLed(1,0,0,0,0);
            break;
        case CENTER_RIGHT:
            playLed(0,0,0,0,0);
            playLed(1,0,LED_LUX,0,0);
            break;
        case LED_ALL_ON :
            playLed(0,0,LED_LUX,0,0);
            playLed(1,0,LED_LUX,0,0);
            break;    
        default:
            break;
        }
    }
    else if (color == E_LED_COLOR::LED_BLUE)
    {
        // eblog(LOG_LV, "test LED BLUE");
        switch (dir)
        {
        case CENTER_LEFT:
            playLed(0,0,0,LED_LUX,0);
            playLed(1,0,0,0,0);
            break;
        case CENTER_RIGHT:
            playLed(0,0,0,0,0);
            playLed(1,0,0,LED_LUX,0);
            break;
        case LED_ALL_ON :
            playLed(0,0,0,LED_LUX,0);
            playLed(1,0,0,LED_LUX,0);
            break;        
        default:
            break;
        }
    }
    else if (color == E_LED_COLOR::LED_WHITE)
    {
        // eblog(LOG_LV, "test LED WHITE");
        switch (dir)
        {
        case CENTER_LEFT:
            playLed(0,0,0,0,LED_LUX);
            playLed(1,0,0,0,0);
            break;
        case CENTER_RIGHT:
            playLed(0,0,0,0,0);
            playLed(1,0,0,0,LED_LUX);
            break;
        case LED_ALL_ON :
            playLed(0,0,0,0,LED_LUX);
            playLed(1,0,0,0,LED_LUX);
            break;     
        default:
            break;
        }
    }
}

/**
 * @brief 깜박이는 LED함수
 * @param ledBlinkStartTick 깜박임 함수 토탈 실행 시간
 * @param ledOnTick ledon on 시간 
 * @param ledOffTick ledoff off 시간
 * 보완해야하는 내용: on/off 한번만 실행(과부하 고려)
 */
void CLed::ledBlinkPlay()
{
    // eblog(LOG_LV, "test LEDBlinkplay");
    //u32 ledOnTick = SYSTEM_TOOL.getSystemTick(); // 한 번만 실행 되어야함

    //totalTime 이하 시간 동안 실행
    if(SYSTEM_TOOL.getSystemTick() - ledBlinkStartTick <= totalTime*SEC_1)
    {
        //onTime 이하 시간 동안 led on
        if(SYSTEM_TOOL.getSystemTick()-ledOnTick <= onTime*SEC_1)
        {
            playLed(0,0,0,0,LED_LUX);
            playLed(1,0,0,0,LED_LUX);
        }
        // onTime 시간 초과하면 off
        else if(SYSTEM_TOOL.getSystemTick()-ledOnTick > onTime*SEC_1)
        {
            
            if(SYSTEM_TOOL.getSystemTick()-ledOffTick <= (offTime + onTime)*SEC_1)
            {    
                playLed(0,0,0,0,0);
                playLed(1,0,0,0,0);
            }
            // offTime 끝나면 초기화
            else if(SYSTEM_TOOL.getSystemTick()-ledOffTick > (offTime + onTime)*SEC_1)
            {    
#if 0 // totalTime시간 동안 깜박임 모드
                ledOnTick = SYSTEM_TOOL.getSystemTick();
                ledOffTick = SYSTEM_TOOL.getSystemTick();
#else // 무한 깜박임 모드
                setLedTick();
#endif
            }
        }
    }
}
/**
 * @brief LED tick 설정 함수
 * 
 */
void CLed::setLedTick()
{
    ledOnTick = SYSTEM_TOOL.getSystemTick();
    ledOffTick = SYSTEM_TOOL.getSystemTick();
    ledBlinkStartTick = SYSTEM_TOOL.getSystemTick(); 
}

void CLed::setBtnLedTick()
{
    btnLedOnTick = SYSTEM_TOOL.getSystemTick();
}

u32 CLed::getBtnLedTick()
{
    return btnLedOnTick;
}

void CLed::ledAllOff()
{
    playLed(0,0,0,0,0);
    playLed(1,0,0,0,0);
}

void CLed::btnLedBlue()
{
    if(ledLR)
    {
        playLed(0,0,0,LED_LUX,0);
        playLed(1,0,0,LED_LUX,0);
        ledLR = false;
    }
    else
    {
        playLed(0,0,0,0,0);
        playLed(1,0,0,0,0);
        ledLR = true;
    }
}

void CLed::btnLedWhite()
{
    if(ledLR)
    {
        playLed(0,0,0,0,LED_LUX);
        playLed(1,0,0,0,LED_LUX);
        ledLR = false;
    }
    else
    {
        playLed(0,0,0,0,0);
        playLed(1,0,0,0,0);
        ledLR = true;
    }
}

/**
 * @brief 충전을 시작할 때 Led 제어
 * 
 */
void CLed::playChargingServiceStart()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 충전이 종료될 때 LED 제어
 * 
 */
void CLed::playChargingServiceEnd()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 청소 서비스가 시작될때 Led 제어
 * 
 */
void CLed::playCleanServiceStart()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 청소 서비스가 종료될때 LED 제어
 * 
 */
void CLed::playCleanServiceEnd()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 자동도킹 서비스가 시작될때 Led 제어
 * 
 */
void CLed::playDockingServiceStart()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 자동도킹 서비스가 종료될때 LED 제어
 * 
 */
void CLed::playDockingServiceEnd()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 탐색 서비스가 시작될때 Led 제어
 * 
 */
void CLed::playExplorerServiceStart()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/**
 * @brief 탐색 서비스가 종료될때 LED 제어
 * 
 */
void CLed::playExplorerServiceEnd()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}

/* 동작/제어 */
// {
//시작/정지
void CLed::powerOn()
{
    ledBlinkPlay(); 
}
void CLed::poweroff()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_RED);
}
void CLed::startFunc()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::pause()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//청소 영역
void CLed::cleanHome()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::cleanRoom()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::cleanRange()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::cleanFocus()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//청소 옵션
void CLed::optionBasic()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::optionDetail()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::optionQuick()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::optionRepeat()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::optionroom()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//걸레 종류
void CLed::normal()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::onetime()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//물공급량
void CLed::waterZero()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::waterOnetime()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::waterOne()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::waterTwo()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//잔수 제거
void CLed::deleteWater()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//Wi-fi
void CLed::wifiInit()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::connectWifi()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::changeWifi()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//예약설정
void CLed::setBook()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//차일드락
void CLed::childLcok()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//음량조절
void CLed::ctrVol()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//틸팅
void CLed::tiltingUp()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::tiltingDown()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//초기화
void CLed::setInit()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::setInitF()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//f/w업데이트
void CLed::updateFw()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//슬립모드
void CLed::sleepMode()
{
    playLed(0,0,0,0,0);
    playLed(1,0,0,0,0);
}
// }

/* 충전 스테이션 */
// {
//전원 on/off

//걸레건조 설정
void CLed::startDry()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::stopDry()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::setDry()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::setDryTime()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
// }

/* 표시/알림 */
// {
//배터리잔량
void CLed::batteryOne()
{
    playLed(0,LED_LUX,0,0,0);
    playLed(1,LED_LUX,0,0,0);
}
void CLed::batteryTwo()
{
    playLed(0,0,0,LED_LUX,0);
    playLed(1,0,0,LED_LUX,0);
}
void CLed::batterythr()
{
    playLed(0,0,LED_LUX,0,0);
    playLed(1,0,LED_LUX,0,0);
}
//배터리부족
void CLed::batteryZero()
{
    playLed(0,0,0,0,0);
    playLed(1,0,0,0,0);
}
//청소 상태
void CLed::cleaningmode()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//위치찾기
void CLed::tmpfunc()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//알림 메시지
void CLed::sendMsg()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//오류
void CLed::wheelError()
{
    playLed(0,0,0,0,0);
    playLed(1,0,0,0,0);
}
void CLed::MoveError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::liftupRsf()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::chargingError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::chargingMoveError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_RED /* hhryu230622 : 에러 색상 변경 LED_WHITE*/);
}
void CLed::connectWifiError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::sensorError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_RED);
}
void CLed::lidarError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::etcError()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
// }

/* 지도설정 */
// {
//지도생성
void CLed::makeMap()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::delMap()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//금지영역 설정
void CLed::setBanSite()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//지도 편집
void CLed::sepaMap()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::combMap()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::changeRoomName()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::rotationMap()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::changeMapname()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
// }

/* 기타 */
// {
//청소 재개
void CLed::resumeClean()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//청소 기록
void CLed::cleanRecord()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
//로봇 공유
void CLed::qrCode()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
void CLed::account()
{
    ledFunc(E_LED_DIRECTION::LED_ALL_ON, E_LED_COLOR::LED_WHITE);
}
// }