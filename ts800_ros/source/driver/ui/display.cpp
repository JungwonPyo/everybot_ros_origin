#include "display.h"
#include "systemTool.h"
#include "MessageHandler.h"
#include "eblog.h"
#include "qrencode.h"
#include "LibRobotDisplayInterface.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CDisplay::CDisplay(/* args */)
{
    bAutoUpdate = false;
    bCustomDisplay = false;
    displayState = E_DISPLAY_STATE::STOP;
    setDisplayStatus(DISPLAY_STATUS::VOID);
}

CDisplay::~CDisplay()
{
}

void CDisplay::setDisplayStatus(DISPLAY_STATUS set)
{
    if(set != status)
    {
        ceblog(LOG_LV_NECESSARY, BLUE, "status : " << enumToString(status) << " set : " << enumToString(set));
    }
    status = set;
}

void CDisplay::LCDStateMonitor()
{
    double runTime;
    switch (status)
    {
    case DISPLAY_STATUS::VOID :
        
        break;
    case DISPLAY_STATUS::STOP :
        if(displayState == E_DISPLAY_STATE::STOP){
            if(bCustomDisplay){
                startCustomDisplay(desCustomImg);
            }else{
                sendDisplayMassage(desImage);
            }
            setDisplayStatus(DISPLAY_STATUS::START);
            startTime = SYSTEM_TOOL.getSystemTime();
        }
        break;    
    case DISPLAY_STATUS::START :
        if(displayState == E_DISPLAY_STATE::PLAYING){
            setDisplayStatus(DISPLAY_STATUS::RUN);
        }
        break;
    case DISPLAY_STATUS::RUN :
         runTime = SYSTEM_TOOL.getSystemTime()-startTime;
         if(displayState == E_DISPLAY_STATE::STOP){
            if(runTime >= 2){
                ceblog(LOG_LV_NECESSARY, BLUE, "LCD DIS-PLAY COMPLETE!!");
                setDisplayStatus(DISPLAY_STATUS::VOID);
            }
            // else{
            //     ceblog(LOG_LV_NECESSARY, BLUE, "SHORT IMAGE DISPLAY At Least 1 SEC WAIT..... " << runTime);
            // }
        }
        break;
    default:
        ceblog(LOG_LV_NECESSARY, BLUE, "LCD DIS-PLAY STATUS ERROR!!");
        break;
    }
}

bool CDisplay::isLCDPlaying()
{
    bool ret = false;
    if(status != DISPLAY_STATUS::VOID) ret = true;
    return ret;
}

void CDisplay::startDisplay(E_DisplayImageClass img)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "startDisplay");
    bCustomDisplay = false;
    sendDisplayStopMassage();
    desImage = img;
    setDisplayStatus(DISPLAY_STATUS::STOP);
}

void CDisplay::startAutoDisplay(bool playNow, E_DisplayImageClass img)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "startAutoDisplay");
    bAutoUpdate = true;
    autoImage = img;
    bCustomDisplay = false;
    if(playNow){
        sendDisplayStopMassage();
        setDisplayStatus(DISPLAY_STATUS::STOP);
        desImage = img;
    }
}

void CDisplay::stopAutoDisplay()
{
    bAutoUpdate = false;
    ceblog(LOG_LV_NECESSARY, BLUE, "stopAutoDisplay");
}

void CDisplay::startCustomDisplay( char* img)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "startCustomDisplay");
    bAutoUpdate = false;
    bCustomDisplay = true;
    sendDisplayStopMassage();
    //sendCustomMassage(img);
    desCustomImg = img;
    setDisplayStatus(DISPLAY_STATUS::STOP);
}

void CDisplay::setCurImage(E_DisplayImageClass img)
{
    curImage = img;
}

void CDisplay::setCustomImage(char* img)
{
    curCustomImg = img;
}

void CDisplay::sendDisplayStopMassage()
{
    tMsgCtrDisplay msg = {};
    msg.stop = true;
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_DISPLAY, msg));
    ceblog(LOG_LV_NECESSARY, BLUE, "STOP LCD DIS-PLAY");
}

void CDisplay::sendDisplayMassage(E_DisplayImageClass img)
{
    tMsgCtrDisplay msg = {};
    msg.stop = false;
    msg.bCustom = false;
    msg.img = img;
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_DISPLAY, msg));
    ceblog(LOG_LV_NECESSARY, BLUE, "START LCD DIS-PLAY : " << DEC(img));
}

void CDisplay::sendCustomMassage(char* img)
{
    tMsgCtrDisplay msg = {};
    msg.bCustom = true;
    msg.customImg = img;
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_DISPLAY, msg));
    ceblog(LOG_LV_NECESSARY, BLUE, "START CUSTOM DIS-PLAY");
}

/**
 * @brief 디스플레이 재생의 상태값을 설정한다. 
 * systeminterface_ts800.cpp에서 사용하는 함수라서 다른 곳에서 사용 안함
 * @param displayState :  디스플레이 재생의 상태 
 */
void CDisplay::setDisplayState(E_DISPLAY_STATE state)
{
    displayState = state;
}

/**
 * @brief 디스플레이를 Queue에 저장하는 함수   
 * @param imageQueue :  디스플레이 데이터를 저장하는 Queue 
 */
void CDisplay::updateDisplay()
{
    LCDStateMonitor();
    
    if(bAutoUpdate){
        if(!isLCDPlaying()){
            setDisplayStatus(DISPLAY_STATUS::START);
            sendDisplayMassage(autoImage);
            ceblog(LOG_LV_NECESSARY, BLUE, "updateDisplay - AutoUpdate");
        } 
    }
}

void CDisplay::runExploreDisplay()
{
    E_DisplayImageClass img;
    if(!isLCDPlaying()){
        ceblog(LOG_LV_NECESSARY, BLUE, "runExploreDisplay");
        if(desImage == E_DisplayImageClass::MAPPING1)   img = E_DisplayImageClass::MAPPING2; 
        else                                            img = E_DisplayImageClass::MAPPING1;

        startDisplay(img);
    }
}

E_DisplayImageClass CDisplay::getBlinkEyeImage()
{
    E_DisplayImageClass img = E_DisplayImageClass::STANDBY_CLEANING10_40;

    switch (ServiceData.battery.getBatteryState())
    {
    case E_BATTERY_STATE::BATT_VOID:
        ceblog(LOG_LV_NECESSARY, BLUE, "Clean Display BattState is Void");
        break;
    case E_BATTERY_STATE::BATT_NEED_CHARGE:
        ceblog(LOG_LV_NECESSARY, BLUE, "Clean Display BattState is NeedCharge");
        break;
    case E_BATTERY_STATE::BATT_LOW :
        ceblog(LOG_LV_NECESSARY, BLUE, "Clean Display BattState is Low");
        break;
    case E_BATTERY_STATE::BATT_MIDDLE :
        img = E_DisplayImageClass::STANDBY_CLEANING40_70;
        break;
    case E_BATTERY_STATE::BATT_HIGH :
    case E_BATTERY_STATE::BATT_FULL :
        img = E_DisplayImageClass::STANDBY_CLEANING70_100;
        break;
    case E_BATTERY_STATE::BATT_ERROR:
        img = E_DisplayImageClass::CHECK_THE_SYSTEM;
        ceblog(LOG_LV_NECESSARY, BLUE, "Clean Display BattState is Error");
        break;                
    default:
        ceblog(LOG_LV_NECESSARY, BLUE, "Clean Display BattState UnKown!!");
        break;
    }

    return img;
}

E_DisplayImageClass CDisplay::getChargingImage()
{
    E_DisplayImageClass img = E_DisplayImageClass::BATTERY_000;
    switch (ServiceData.battery.getBatteryState())
    {
    case E_BATTERY_STATE::BATT_VOID:
        ceblog(LOG_LV_NECESSARY, BLUE, "Charging Display BattState is Void");
        break;
    case E_BATTERY_STATE::BATT_NEED_CHARGE :
        ceblog(LOG_LV_NECESSARY, BLUE, "Charging Display BattState is NeedCharge");
        break;
    case E_BATTERY_STATE::BATT_LOW :
        img = E_DisplayImageClass::BATTERY_001;
        break;
    case E_BATTERY_STATE::BATT_MIDDLE :
        img = E_DisplayImageClass::BATTERY_002;
        break;
    case E_BATTERY_STATE::BATT_HIGH :
        img = E_DisplayImageClass::BATTERY_003;
        break;
    case E_BATTERY_STATE::BATT_FULL :
        img = E_DisplayImageClass::CHARGING_COMPLETE;
        break;
    case E_BATTERY_STATE::BATT_ERROR:
        img = E_DisplayImageClass::CHECK_THE_SYSTEM;
        ceblog(LOG_LV_NECESSARY, BLUE, "Charging Display BattState is Error");
        break;                
    default:
        //eblog(LOG_LV, "ChargingDisplay State Error : " << (int)battState);
        break;
    }

    return img;
}


void CDisplay::DisplayQRCodeOnLCD(const char *data, int lcd_width, int lcd_height, int qr_size, int x_pos, int y_pos, int Color, int border_size, int border_color, int margin)
{
    QRcode *qr = QRcode_encodeString(data, 0, QR_ECLEVEL_L, QR_MODE_8, 1);

    if (qr != NULL) {
        char *lcd_data = (char *)malloc(lcd_width * lcd_height * sizeof(char));
        if (lcd_data == NULL) {
            QRcode_free(qr);
            return;
        }
        memset(lcd_data,0x00, lcd_width * lcd_height);

        int qr_with_border = qr->width + (2 * border_size) + (2 * margin);
        int scale_x = qr_size / qr_with_border;
        int scale_y = qr_size / qr_with_border;
        for (int y = 0; y < qr_with_border; y++) {
            for (int i = 0; i < scale_y; i++) {
                for (int x = 0; x < qr_with_border; x++) {
                    for (int j = 0; j < scale_x; j++) {
                        int qr_x = x - border_size - margin;
                        int qr_y = y - border_size - margin;
                        int lcd_x = x_pos + (x * scale_x) + j;
                        int lcd_y = y_pos + (y * scale_y) + i;
                        if (lcd_x >= 0 && lcd_x < lcd_width && lcd_y >= 0 && lcd_y < lcd_height) {
                            if (qr_x < 0 || qr_x >= qr->width || qr_y < 0 || qr_y >= qr->width) {
                                lcd_data[lcd_y * lcd_width + lcd_x] = border_color;
                            } else {
                                if (qr->data[qr_y * qr->width + qr_x] & 1) {
                                    lcd_data[lcd_y * lcd_width + lcd_x] = 0x00;
                                }else
                                {
                                    lcd_data[lcd_y * lcd_width + lcd_x] = 0xFF;
                                }
                            }
                        }
                    }
                }
            }
        }

        // RoDisplayInf.Inf_Play_Custom((unsigned char *)lcd_data);
        startCustomDisplay(lcd_data);
        
        free(lcd_data);
        QRcode_free(qr);
    } else {

    }
}
