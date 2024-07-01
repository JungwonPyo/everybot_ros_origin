#include "wifi.h"
#include "MessageHandler.h"
#include "qrencode.h"
#include "device_data_manage.h"

#include "userInterface.h"
#include <systemTool.h>


CWifi::CWifi()
{
    failConnection = false;
    bCheckTrigger = false;
}

CWifi::~CWifi()
{
    eblog(LOG_LV,  "");
}


void CWifi::initwifi()
{
    eblog(LOG_LV_AWS,"Init WIFI");
    wifiStep = E_WIFI_CONNECT_STEP::E_CHECK_EXIT;
    wifiCancelStep = E_WIFI_CANCEL_STEP::E_CHECK_WIFI_STEP;
    failConnection = false;
    QRStartTime = 0.0;
    bCheckTrigger = true;
    uiFlag = true;
    // AP삭제
    // if(ServiceData.rsBridge.getrsMode() == (char)1)
    if(!ServiceData.rsBridge.getApExit())
    {
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AP)));
        eblog(LOG_LV_AWS,"disConnect ap 메시지");
    }
    // STATION 삭제
    // if(ServiceData.rsBridge.getrsMode() == (char)2)
    if(!ServiceData.rsBridge.getStationExit())
    {
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION)));
        eblog(LOG_LV_AWS,"disConnect station 메시지");
    }
    //aws는 AP로 갈때 삭제함
}

bool CWifi::wifiProc()
{
    bool ret = false;

    switch (wifiStep)
    {
    case E_WIFI_CONNECT_STEP::E_CHECK_EXIT:
        eblog(LOG_LV_AWS,"E_CHECK_EXIT");
        wifiStep = checkExit();
        break;
    case E_WIFI_CONNECT_STEP::E_ENTER_AP_MODE: // AP mode 진입
        eblog(LOG_LV_AWS,"E_ENTER_AP_MODE");
        wifiStep = enteringApMode();
        break;
    case E_WIFI_CONNECT_STEP::E_AP_MODE:
        eblog(LOG_LV_AWS,"E_AP_MODE");
        wifiStep = apMode();
        break;
    case E_WIFI_CONNECT_STEP::E_EXIT_AP_MODE:
        eblog(LOG_LV_AWS,"E_EXIT_AP_MODE");
        wifiStep = exitApMode();
        break;
    case E_WIFI_CONNECT_STEP::E_DONE_AP_MODE:
        eblog(LOG_LV_AWS,"E_DONE_AP_MODE");
        wifiStep = doneApMode();
        break;
    case E_WIFI_CONNECT_STEP::E_ENTER_STATION_MODE: //station 진입
        eblog(LOG_LV_AWS,"E_ENTER_STATION_MODE");
        wifiStep = enteringStationMode();
        break;
    case E_WIFI_CONNECT_STEP::E_STATION_MODE:
        eblog(LOG_LV_AWS,"E_STATION_MODE");
        wifiStep = stationMode();
        break;
    case E_WIFI_CONNECT_STEP::E_EXIT_STATION_MODE:
        eblog(LOG_LV_AWS,"E_EXIT_STATION_MODE");
        wifiStep = exitStationMode();
        break;
    case E_WIFI_CONNECT_STEP::E_DONE_STATION_MODE:
        eblog(LOG_LV_AWS,"E_DONE_STATION_MODE");
        wifiStep = doneStationMode();
        break;
    case E_WIFI_CONNECT_STEP::E_CONNECT_AWS:    //aws 진입
        eblog(LOG_LV_AWS,"E_CONNECT_AWS");
        wifiStep = connectAws();
        break;
    case E_WIFI_CONNECT_STEP::E_CHECK_WIFI:
        eblog(LOG_LV_AWS,"E_CHECK_AWS");
        ret = checkAws();
        break;
    case E_WIFI_CONNECT_STEP::E_FAIL_WIFI:
        eblog(LOG_LV_AWS,"E_FAIL_WIFI");
        ret = failWifi();
        break;

    
    default:
        /* 스텝이 꼬였다 다시 시작 해줘~ */
        break;
    }
    

    return ret;
}

E_WIFI_CONNECT_STEP CWifi::checkExit()
{
    auto ret = E_WIFI_CONNECT_STEP::E_CHECK_EXIT;

    if(bCheckTrigger)
    {
        bool apCheck = false; 
        bool stationCheck = false;
        if (ServiceData.rsBridge.getApExit()) // 종료 완료 상태
        {
            // wifi service에서 포인터가 사라진 거 체크하고 넘김
            eblog(LOG_LV_AWS,"check AP");
            apCheck = true;
        }
        
        if (ServiceData.rsBridge.getStationExit()) 
        {
            eblog(LOG_LV_AWS,"check 스테이션");
            stationCheck = true;

        }

        if(apCheck && stationCheck)
        {
            eblog(LOG_LV_AWS,"chage ENTER_AP_MODE");
            bCheckTrigger = false;
            ret = E_WIFI_CONNECT_STEP::E_ENTER_AP_MODE; //처음시작 한 경우
        }
    }

    return ret;
}

bool CWifi::getFailConnection()
{
    return failConnection;
}

E_WIFI_CONNECT_STEP CWifi::getWifiStep()
{
    return wifiStep;
}

E_WIFI_CONNECT_STEP CWifi::enteringApMode()
{
    //스마트 폰(앱) - 로봇 연결
    auto ret = E_WIFI_CONNECT_STEP::E_AP_MODE;

    //ApMode(폰-로봇 연결) : 1 , StationMode(homeAP-로봇 연결) : 2
    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_AP)));
    apStartTime = SYSTEM_TOOL.getSystemTime();
    

    return ret;
}
//#include "lcdDebug.h"
E_WIFI_CONNECT_STEP CWifi::apMode()
{
    auto ret =  E_WIFI_CONNECT_STEP::E_AP_MODE;

    if(SYSTEM_TOOL.getSystemTime() - apStartTime >= AP_LIMIT_TIME)
    {
        ret = E_WIFI_CONNECT_STEP::E_FAIL_WIFI;
        return ret;
    }
#if TIME_EXIT_DEBUG_MODE == 1
    return ret;
#else
    eblog(LOG_LV_AWS, "service wifi getrsMode : "<<(int)ServiceData.rsBridge.getrsMode());

    if (ServiceData.rsBridge.getrsMode() == 1) // Ap
    {
        /* ssid 비번? 암튼 필요한 정보 받아옴 */
        std::pair<char, char> values = ServiceData.rsBridge.getrsApValues(); 
        if (values.second == 0xE && values.first == 0x0 && SYSTEM_TOOL.getSystemTime() - QRStartTime > 5 )
        {
            // DISPLAY_CTR.printfLCDClear();
            //led.testDispaly();

            QRStartTime = SYSTEM_TOOL.getSystemTime();

            int lcd_width = 258;
            int lcd_height = 96;
            int qr_size = 75;
            //초기화
            memset(SerialNumber,0x00,sizeof(SerialNumber));
            memset(deviceNeme,0x00,sizeof(deviceNeme));
            Get_BaseData_SerialNumber_Robot(SerialNumber);
            Get_BaseData_DeviceName_Robot(deviceNeme);
                
            //memcpy(SerialNumber,"TESTNUMBER000",strlen("TESTNUMBER000"));
            //memcpy(deviceNeme,"TS800",strlen("TS800"));
                
            char *combined = extract_and_combine(deviceNeme, SerialNumber);

            int x_pos = (lcd_width - qr_size) / 2;
            int y_pos = (lcd_height - qr_size) / 2;

            DISPLAY_CTR.DisplayQRCodeOnLCD(combined, lcd_width, lcd_height, qr_size, x_pos, y_pos , 0xFF ,1, 0xFF, 1);
            
            if(combined != NULL)
                free(combined);
        }

        if (values.second == 0xE && values.first == 0x1 && uiFlag )
        {
            /* UI */
            SOUND_CTR.soundPlay(false,E_SoundClass::SOUND_SCANNING_WIFI);
            uiFlag = false;
        }

        // if(Conn->ApState() == 0xEE)
        if (values.second == 0xE && values.first == 0xE) 
        {
            eblog(LOG_LV_AWS,"complete conect AP Mode");
            //printf("Ap State [%02X] \n", Conn->ApState());
#if false //(APMODE_PASS == 1)                 
            //Set_HomaAp_SSID("EVERYBOT_0000");
            // Set_HomaAp_PW("We!ComeRob@TWorld~");
            //Set_HomaAp_SSID("ebrdai");

            Set_HomaAp_SSID("ebrd");
            //Set_HomaAp_SSID("EVERYBOT");
            Set_HomaAp_PW("Everyb@t728");
            API_HomeApData_Save();
#endif
            eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
                            "values.second : "<< std::hex << static_cast<int>(values.second));
            ret =  E_WIFI_CONNECT_STEP::E_EXIT_AP_MODE;
        }
        else
        {
            //SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::AP_STATES)));
            eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
                            "values.second : "<< std::hex << static_cast<int>(values.second));
        }
    }
    return ret;
#endif
    
}
E_WIFI_CONNECT_STEP CWifi::exitApMode()
{
    auto ret =  E_WIFI_CONNECT_STEP::E_DONE_AP_MODE;

    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AP)));

    return ret;    

}

E_WIFI_CONNECT_STEP CWifi::doneApMode()
{
    auto ret =  E_WIFI_CONNECT_STEP::E_DONE_AP_MODE;

    if(ServiceData.rsBridge.getrsMode() == 0)
    {
        ret =  E_WIFI_CONNECT_STEP::E_ENTER_STATION_MODE;
    }

    return ret;    
}

E_WIFI_CONNECT_STEP CWifi::enteringStationMode()
{
    auto ret = E_WIFI_CONNECT_STEP::E_STATION_MODE;
    DISPLAY_CTR.startAutoDisplay(true,E_DisplayImageClass::CONNECTING_TO_WIFI);

    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_STATION)));
    stationStartTime = SYSTEM_TOOL.getSystemTime();

    return ret;
}

E_WIFI_CONNECT_STEP CWifi::stationMode()
{
    auto ret = E_WIFI_CONNECT_STEP::E_STATION_MODE;

    if(SYSTEM_TOOL.getSystemTime() - stationStartTime >= STATION_LIMIT_TIME)
    {
        ret = E_WIFI_CONNECT_STEP::E_FAIL_WIFI;
        return ret;
    }
#if TIME_EXIT_DEBUG_MODE == 2
    return ret;
#else    
    if (ServiceData.rsBridge.getrsMode() == 2) // Station
    {
        std::pair<char, char> values = ServiceData.rsBridge.getrsStationValues(); 
        if (values.second == 0xE && values.first == 0xE)
        {
            eblog(LOG_LV_AWS,"complete conect Station Mode");
            if (ServiceData.rsBridge.getrsHomeApConnection())
            {
                eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
                                "values.second : "<< std::hex << static_cast<int>(values.second));
                ret = E_WIFI_CONNECT_STEP::E_EXIT_STATION_MODE;
            }
        }
        else
        {
            eblog(LOG_LV_AWS,"values.first : "<< std::hex << static_cast<int>(values.first) << 
                            "values.second : "<< std::hex << static_cast<int>(values.second));
        }
    }
    
    return ret;
#endif
}

E_WIFI_CONNECT_STEP CWifi::exitStationMode()
{
    auto ret = E_WIFI_CONNECT_STEP::E_DONE_STATION_MODE;

    SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION)));    

    return ret;
}

E_WIFI_CONNECT_STEP CWifi::doneStationMode()
{
    auto ret = E_WIFI_CONNECT_STEP::E_DONE_STATION_MODE;

    if (ServiceData.rsBridge.getrsMode() == 0)
    {
        ret = E_WIFI_CONNECT_STEP::E_CONNECT_AWS;
    }

    return ret;
}

E_WIFI_CONNECT_STEP CWifi::connectAws()
{
    auto ret = E_WIFI_CONNECT_STEP::E_CONNECT_AWS;

    if(ServiceData.rsBridge.getconnect() == false)
    {
        ret = E_WIFI_CONNECT_STEP::E_CHECK_WIFI;
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_AWS)));
        awsStartTime = SYSTEM_TOOL.getSystemTime();
    }

    return ret;
}

bool CWifi::checkAws()
{
    if(SYSTEM_TOOL.getSystemTime() - awsStartTime >= AWS_LIMIT_TIME)
    {
        wifiStep = E_WIFI_CONNECT_STEP::E_FAIL_WIFI;
    }
#if TIME_EXIT_DEBUG_MODE == 3
    return false;
#else
    if (ServiceData.rsBridge.getconnect())    return true;
    else        return false; 
#endif
}

bool CWifi::failWifi()
{
    failConnection = true;
    return true;
}

bool CWifi::cancelProc()
{
    bool ret = false;
    std::pair<char, char> values;

    switch (wifiCancelStep)
    {
    case E_WIFI_CANCEL_STEP::E_CHECK_WIFI_STEP :
        eblog(LOG_LV_AWS,"E_CHECK_WIFI_STEP");
        wifiCancelStep = checkWifiStep();
        break;
    case E_WIFI_CANCEL_STEP::E_CANCEL_AP :
        eblog(LOG_LV_AWS,"E_CANCEL_AP");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AP)));
        wifiCancelStep = E_WIFI_CANCEL_STEP::E_CHECK_AP_EXIT;
        break;
    case E_WIFI_CANCEL_STEP::E_CHECK_AP_EXIT :
        eblog(LOG_LV_AWS,"E_CHECK_AP_EXIT");
        wifiCancelStep = checkApExit();
        break;
    case E_WIFI_CANCEL_STEP::E_CANCEL_STATION :
        eblog(LOG_LV_AWS,"E_CANCEL_STATION");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_STAION)));    
        wifiCancelStep = E_WIFI_CANCEL_STEP::E_CHECK_STATION_EXIT;
        break;
    case E_WIFI_CANCEL_STEP::E_CHECK_STATION_EXIT :
        eblog(LOG_LV_AWS,"E_CHECK_STATION_EXIT");
        wifiCancelStep = checkStationExit();
        break;
    case E_WIFI_CANCEL_STEP::E_CANCEL_AWS :
        eblog(LOG_LV_AWS,"E_CANCEL_AWS");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AWS)));    
        wifiCancelStep = E_WIFI_CANCEL_STEP::E_CHECK_AWS_EXIT;
        break;
    case E_WIFI_CANCEL_STEP::E_CHECK_AWS_EXIT :
        eblog(LOG_LV_AWS,"E_CHECK_AWS_EXIT");
        wifiCancelStep = checkAwsExit();
        break;
    case E_WIFI_CANCEL_STEP::E_ENTER_STATION :
        eblog(LOG_LV_AWS,"E_ENTER_STATION");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::CONNECT_STATION)));
        wifiCancelStep = E_WIFI_CANCEL_STEP::E_STATION_CONECT;
        break;
    case E_WIFI_CANCEL_STEP::E_STATION_CONECT :
        eblog(LOG_LV_AWS,"E_STATION_CONECT");
        values = ServiceData.rsBridge.getrsStationValues(); 
        if (values.second == 0xE && values.first == 0xE){
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AWS)));    
            wifiCancelStep = E_WIFI_CANCEL_STEP::E_NO_SEQ;
        }
        else if (values.second == 0xF || values.first == 0xF)
        {
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_CONNECT_PHONE, tMsgInterfacePhone(E_PHONE_INTERFACE_TYPE::DISCONNECT_AWS)));    
            wifiCancelStep = E_WIFI_CANCEL_STEP::E_NO_SEQ;
        }
        break;
    
    default:
        eblog(LOG_LV_AWS,"E_NO_SEQ");
        ret = true; // 취소 시퀀스를 할 필요없음
        break;
    }

    return ret;
}
E_WIFI_CANCEL_STEP CWifi::checkWifiStep()
{
    E_WIFI_CANCEL_STEP ret;

    if(wifiStep == E_WIFI_CONNECT_STEP::E_AP_MODE || wifiStep == E_WIFI_CONNECT_STEP::E_EXIT_AP_MODE){
        ret = E_WIFI_CANCEL_STEP::E_CANCEL_AP;
    }
    else if(wifiStep == E_WIFI_CONNECT_STEP::E_DONE_AP_MODE) {
        ret = E_WIFI_CANCEL_STEP::E_CHECK_AP_EXIT;
    }
    else if(wifiStep == E_WIFI_CONNECT_STEP::E_STATION_MODE || wifiStep == E_WIFI_CONNECT_STEP::E_EXIT_STATION_MODE){
        ret = E_WIFI_CANCEL_STEP::E_CANCEL_STATION;
    }
    else if(wifiStep == E_WIFI_CONNECT_STEP::E_DONE_STATION_MODE){
        ret = E_WIFI_CANCEL_STEP::E_CHECK_STATION_EXIT;
    }
    else if(wifiStep == E_WIFI_CONNECT_STEP::E_CHECK_WIFI){
        ret = E_WIFI_CANCEL_STEP::E_CANCEL_AWS;
    }
    else{
        eblog(LOG_LV_AWS,"취소 절차를 실행 안해도 됨");
        ret = E_WIFI_CANCEL_STEP::E_NO_SEQ;
    }

    return ret;
}
E_WIFI_CANCEL_STEP CWifi::checkApExit()
{
    if (ServiceData.rsBridge.getApExit()){
        eblog(LOG_LV_AWS,"done AP Exit");
        return E_WIFI_CANCEL_STEP::E_ENTER_STATION;
    }
    else {
        return E_WIFI_CANCEL_STEP::E_CHECK_AP_EXIT;
    }
    
}
E_WIFI_CANCEL_STEP CWifi::checkStationExit()
{
    if (ServiceData.rsBridge.getStationExit()){
        eblog(LOG_LV_AWS,"done Staion Exit");
        return E_WIFI_CANCEL_STEP::E_NO_SEQ;
    } 
    else {
        return E_WIFI_CANCEL_STEP::E_CHECK_STATION_EXIT;
    }
}
E_WIFI_CANCEL_STEP CWifi::checkAwsExit()
{
    if (ServiceData.rsBridge.getconnect())    return E_WIFI_CANCEL_STEP::E_CHECK_AWS_EXIT;
    else        return E_WIFI_CANCEL_STEP::E_ENTER_STATION; 
}

bool CWifi::failProc()
{}

char* CWifi::extract_and_combine(char* A, char* B) {

#if 0
    size_t length_B = strlen(B);
    char* last_chars_B = (char*)malloc(6); 
    if (last_chars_B == NULL) {
        //printf("Memory allocation failed.\n");
        return NULL;
    }

    if (length_B >= 5) {
        strncpy(last_chars_B, B + length_B - 5, 5);
        last_chars_B[5] = '\0';
    } else {
        strcpy(last_chars_B, B);
    }

    size_t length_A = strlen(A);

    char* result = (char*)malloc(length_A + 7);
    if (result == NULL) {
        //printf("Memory allocation failed.\n");
        free(last_chars_B); 
        return NULL;
    }

    strcpy(result, A);
    strcat(result, "_");
    strcat(result, last_chars_B);
    free(last_chars_B);
#else
    size_t length_A = strlen(A);
    size_t length_B = strlen(B);
    char* result = (char*)malloc(length_A + length_B + 1);

    strcpy(result, A);
    strcat(result, "/");
    strcat(result, B);    
#endif
    return result;
}

void CWifi::displayQRCodeOnLCDMargin(const char *data, int lcd_width, int lcd_height, int qr_size, int x_pos, int y_pos, int Color, int border_size, int border_color, int margin) {

    QRcode *qr = QRcode_encodeString(data, 0, QR_ECLEVEL_L, QR_MODE_8, 1);

    if (qr != NULL) {
        char *lcd_data = (char *)malloc(lcd_width * lcd_height * sizeof(char));
        if (lcd_data == NULL) {
            fprintf(stderr, "�޸� �Ҵ� ����\n");
            QRcode_free(qr);
            return;
        }
        memset(lcd_data,0x00, lcd_width * lcd_height);

        // QR �ڵ带 LCD �迭�� ����
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
#if 0
        for (int y = 0; y < lcd_height; y++) {
            for (int x = 0; x < lcd_width; x++) {
                printf("%c", lcd_data[y * lcd_width + x]);
            }
            printf("\n");
        }
#endif
        //RoDisplayInf.Inf_Manual((char *)lcd_data);
        //RoDisplayInf.Inf_Manual((char *)lcd);
        // tMsgCtrDisplay displayMsg = {};
        // displayMsg.customImg = (unsigned char *)lcd_data;
        // SEND_MESSAGE(message_t(E_MESSAGE_TYPE_DISPLAY, displayMsg));

        /* TODO : QR추가해야함 */
        // RoDisplayInf.Inf_Play_Custom((unsigned char *)lcd_data); 
        free(lcd_data);
        QRcode_free(qr);
    } else {
        fprintf(stderr, "QR �ڵ� ���� ����: �����Ͱ� �߸��Ǿ��ų� QR �ڵ带 ������ �� �����ϴ�.\n");
    }
}