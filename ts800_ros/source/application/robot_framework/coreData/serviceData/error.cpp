
#include "error.h"
#include "externData/externData.h"
#include "eblog.h"
// #include "Message.h"
#include "MessageHandler.h"

CError::CError()
{
    initError();
}

CError::~CError(){}

void CError::initError()
{
    errorWheelCurrentCount = 0;
    errorTrapObstacleCount = 0;
}

void CError::update(CExternData* pExternData)
{
    if(pExternData->systemData.isUpdateErrorData())
    {
        u32 errorType = pExternData->systemData.useErrorData();
        // eblog(LOG_LV_NECESSARY, "error update : "<< errorType); 디버깅용
        u32 send = 0x00;
        
        for(int i=0; i < 32; i++)
        {
            if( errorType & (1 << i))
            {
                send |= (1 << i);
                if (i == 16)
                {
                    errorTrapObstacleCount++;
#if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0)
                    eblog(LOG_LV_NECESSARY,"errorTrapObstacleCount : "<<errorWheelCurrentCount);
#endif
                }
                else if (i == 17)
                {
                    errorWheelCurrentCount++;
#if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0) 
                    eblog(LOG_LV_NECESSARY,"errorWheelCurrentCount : "<<errorWheelCurrentCount);
#endif
                }
            }
        } 
#if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0)
        checkSendErrorMessage();
#endif
        if(send > 0)
            pExternData->systemData.clearError(send);
    }
}
/**
 * @brief 에러를 판단하여 메시지를 보내는 함수
 * 1. 현재 에러에 대한 동작사양이 없어서 에러 판단을 가볍게 함
 * 2. 굳이 꼭 여기서 에러를 판단해서 메시지 보내지 않아도 됨
 * 
 */
void CError::checkSendErrorMessage()
{
    int errorThreshold = CONFIG.sysWheelErrorCount;

    if(errorTrapObstacleCount > errorThreshold && errorWheelCurrentCount > errorThreshold)
    {
        eblog(LOG_LV_NECESSARY,"errorTrapObstacleCount : "<< errorWheelCurrentCount << " errorWheelCurrentCount : " << errorWheelCurrentCount);
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::WHEEL_CURRENT)));
        initError();
    }
    else if (errorTrapObstacleCount > errorThreshold)
    {
        eblog(LOG_LV_NECESSARY,"errorTrapObstacleCount : "<<errorWheelCurrentCount);
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::TRAP_OBSTACLE)));
        initError();
    }
    else if (errorWheelCurrentCount > errorThreshold)
    {
        eblog(LOG_LV_NECESSARY,"errorWheelCurrentCount : "<<errorWheelCurrentCount);
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::WHEEL_CURRENT)));
        initError();
    }
}

/* 
BATTERY_ERROR_LOW               (1 << 0) 
CHARGE_ERROR_STOP               (1 << 1) 
CHARGE_ERROR_OVERCURRENT        (1 << 2) 
CHARGE_ERROR_ADAPTORVOLTAGE     (1 << 3) 
CHARGE_ERROR_BATTERYVOLTAGE     (1 << 4) 
CHARGE_ERROR_TEMP               (1 << 5) 
CHARGE_ERROR_TIMEOUT            (1 << 6) 
CHARGE_1                        (1 << 7) 
CURRNT_ERROR_PUMP_ABNORMAL      (1 << 8) 
CURRNT_ERROR_TILT_ABNORMAL      (1 << 9) 
CURRNT_ERROR_MOTORL_ABNORMAL    (1 << 10) 
CURRNT_ERROR_MOTORR_ABNORMAL    (1 << 11) 
CURRNT_ERROR_MOTORB_ABNORMAL    (1 << 12) 
CURRNT_1                        (1 << 13) 
CURRNT_2                        (1 << 14) 
CURRNT_3                        (1 << 15) 
DRIVE_WARNING_PWM_PIXART        (1 << 16) 바퀴는 움직이는데 픽사트 값 변화가 없는것 -> 예: 벽에 꼬라박고 있는 상황(범핑 안되고 라이다 감지도 안되는 장애물)
DRIVE_WARNING_PWM_ENCODER       (1 << 17) 바퀴가 정상 속력으로 회전 안함 -> 예: 바퀴에 뭐가 걸려서 회전느려짐 
DRIVE_1                         (1 << 18) 
DRIVE_2                         (1 << 19) 
SENSOR_ERROR_IMU                (1 << 20) 
SENSOR_ERROR_TOF                (1 << 21) 
SENSOR_ERROR_PXART              (1 << 22) 
SENSOR_1                        (1 << 23) 
ETC_1                           (1 << 24) 
ETC_2                           (1 << 25) 
ETC_3                           (1 << 26) 
ETC_4                           (1 << 27) 
CLIFF_WARN_TOF_CONTROL_MCU      (1 << 28) 
CLIFF_WARN_TOF_LEFT             (1 << 29) 
CLIFF_WARN_TOF_RIGHT            (1 << 30) 
CLIFF_WARN_TOF_TIMEOUT          (1 << 31)
*/