
#include "systemInterface_q8.h"
#include "LibRobotControlInterface.h"
#include "LibRobotControlDrive.h"
#include "LibRobotDataInterface.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"
#include "LibRobotControlLocalization.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

extern CRobotControlInterface RoCtrInf; // hw Control Interface 모듈
extern CRobotDataInterface RoDatInf;    // hw Data Interface 모듈
extern CRobotDisplayInterface RoDisplayInf;
extern CRobotSoundInterface RoSoundInf;

CSystemInterfaceQ8::CSystemInterfaceQ8()
{
    if (connect())
    {
        bReceiveDataLoopRunning = true;
        
        thSystemInterface  = std::thread(&CSystemInterfaceQ8::threadLoop, this);
    }
    else
    {
        eblog(LOG_LV, "/n/n/n connect fail /n/n/n");
    }
    eblog(LOG_LV,  "");
}

CSystemInterfaceQ8::~CSystemInterfaceQ8()
{
    bReceiveDataLoopRunning = false;
    thSystemInterface.join();
}

bool CSystemInterfaceQ8::connect()
{
    bool bRet = true;
    bool bIsOpen = false;
    int connectTryCnt = 0;
    const int MaxTryCnt = 10;

    /* 1. Control Interface Open */
    eblog(LOG_LV, "RoCtrInf connect");
    bIsOpen = false;
    connectTryCnt = 0;
    while( bIsOpen == false)
    {
        bIsOpen = RoCtrInf.Open(0);
        usleep(100000);//0.1초
        if ( connectTryCnt > MaxTryCnt )
        {
            eblog(LOG_LV, "RoCtrInf connect fail "<<"retry count : "<< connectTryCnt);            
            bRet = false;
            break;
        }
        connectTryCnt++;
    }
    

    /* 2. Control Interface Open */    
    eblog(LOG_LV, "RoDatInf connect");
    bIsOpen = false;
    connectTryCnt = 0;
    while( bIsOpen == false)
    {
        bIsOpen = RoDatInf.Open();
        usleep(100000);//0.1초
        if ( connectTryCnt > MaxTryCnt )
        {
            eblog(LOG_LV, "RoDatInf connect fail "<<"retry count : "<< connectTryCnt);
            bRet = false;
            break;
        }
        connectTryCnt++;
    }

    /* 3. Sound Interface Open */    
    eblog(LOG_LV, "RoDatInf connect");
    bIsOpen = false;
    connectTryCnt = 0;
    while( bIsOpen == false)
    {
        bIsOpen = RoSoundInf.Open();
        usleep(100000);//0.1초
        if ( connectTryCnt > MaxTryCnt )
        {
            eblog(LOG_LV, "RoDatInf connect fail "<<"retry count : "<< connectTryCnt);
            bRet = false;
            break;
        }
        connectTryCnt++;
    }

    /* 4. LCD Interface Open */    
    eblog(LOG_LV, "RoDatInf connect");
    bIsOpen = false;
    connectTryCnt = 0;
    while( bIsOpen == false)
    {
        bIsOpen = RoDisplayInf.Open(); // Open()시 자동으로 4개 기둥이 있는 화면으로 바뀜.
        usleep(100000);//0.1초
        if ( connectTryCnt > MaxTryCnt )
        {
            ceblog(LOG_LV_NECESSARY, BOLDRED, "RoDatInf 연결 실패했습니다");
            bRet = true; // false; 일단 true로 하여 확인
            break;
        }
        connectTryCnt++;
    }

    return bRet;    
}

bool CSystemInterfaceQ8::disConnect()
{
    eblog(LOG_LV, "[System Interface] disConnect() call!");
    RoCtrInf.Drive.Inf_DriveWheel(0, 0, 0, 0, 0, 0);
    eblog(LOG_LV, "[System Interface] wheel stop");
    
    sleep(1);
    
    bReceiveDataLoopRunning = false; // thread ReceiveData 종료

    RoDatInf.Close();
    eblog(LOG_LV, "[System Interface] Data Interface Close() ");
    RoCtrInf.Close();
    eblog(LOG_LV, "[System Interface] Control Interface Close() ");

    if ( RoDatInf.IsOpen() || RoCtrInf.IsOpen() )
    {
        return false;
    }

    return true;
}

void CSystemInterfaceQ8::threadLoop()
{
    CSystemInterface::threadLoop();
}

void CSystemInterfaceQ8::setWheelMotorData()
{
    tSysWheelMotor tempCurrent;    
    RoDatInf.Drive.Inf_GetDriveCurrentData(&tempCurrent.leftCurrent, &tempCurrent.rightCurrent, 0);
    RoDatInf.Drive.Inf_GetDriveEncoderData(&tempCurrent.leftEncoder, &tempCurrent.rightEncoder, 0);
    RoDatInf.Drive.Inf_GetDriveGainData(&tempCurrent.leftGain, &tempCurrent.rightGain, 0);

    ceblog(LOG_LV_SYSTEMINF, GREEN, "Gain l r d [" << tempCurrent.leftGain << "\t" << tempCurrent.rightGain);
    
    wheelMotorData = tempCurrent;
    bUpdateStateWheelMotorData = true;
}

void CSystemInterfaceQ8::setDisplayStateData()
{
    /* do nothing */
}
void CSystemInterfaceQ8::setSoundStateData()
{
    /* do nothing */
}

/**
 * @brief Q8용 휠 제어 함수
 * 
 * @param direction 
 * @param L_Speed 
 * @param R_Speed 
 * @param duty 
 * @param bHeadingPid 
 * @return true 
 * @return false 
 */
bool CSystemInterfaceQ8::transControlWheel(u8 direction, s32 L_Speed, s32 R_Speed, u32 duty, bool bHeadingPid)
{
    bool ret = false;
    int retryCnt = 0;
    /* 현재는 속도의 변화가 있거나, 방향이 정지 혹은 Dynamic 일때만 휠제어 */    
    //RoCtrInf.Drive.Inf_DriveAnglePid(bHeadingPid);
    ret = RoCtrInf.Drive.Inf_DriveWheel(direction, L_Speed, R_Speed,0, duty, bHeadingPid);

    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false)
    {
        std::cout << "ERROR : trans retry"<<++retryCnt<<std::endl;
        // ret = RoCtrInf.Drive.Inf_DriveWheel(0, 0, 0, 0, 0, false);// error 상황이여 강제 정지 명령.
        ret = RoCtrInf.Drive.Inf_DriveWheel(direction, L_Speed, R_Speed,0, duty, bHeadingPid);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    eblog(LOG_LV_SYSTEMINF, "w: " << L_Speed << "  v: " <<R_Speed << "   R: " << duty );
    return ret;
}

/**
 * @brief Q8용 석션모터 제어 함수
 * 
 * @param value 
 * @return true 
 * @return false 
 */
bool CSystemInterfaceQ8::transControlSuction(u8 value)
{
    return RoCtrInf.Suction.Inf_SuctionSuction(value);// Suction(value);    
}

/**
 * @brief Q8용 메인브러쉬 제어 함수
 * 
 * @param value 
 * @return true 
 * @return false 
 */
bool CSystemInterfaceQ8::transControlMainBrush(u8 value)
{
    return RoCtrInf.Brush.Inf_BrushMain(value); //Brush(value);    
}

/**
 * @brief Q8용 사이드브러쉬 제어 함수
 * 
 * @param value 
 * @return true 
 * @return false 
 */
bool CSystemInterfaceQ8::transControlSideBrush(u8 value)
{
    return RoCtrInf.Brush.Inf_BrushSide(value); //Brush(value);    
}

void CSystemInterfaceQ8::transControlDryFan(bool on,u16 speed)
{
    /* do nothing */
}

//-----------------------------------------------------------------------------
// Q8용 LCD 디스플레이 제어 함수 
//
// Descriptions :
//      - 사용하지 마세요.
//-----------------------------------------------------------------------------
void CSystemInterfaceQ8::transControlDisplayStop()
{
    /* do nothing */
}

//-----------------------------------------------------------------------------
// Q8용 사운드 제어 함수 
//
// Descriptions :
//      - 사용하지 마세요.
//-----------------------------------------------------------------------------
void CSystemInterfaceQ8::transControlSoundStop()
{
    /* do nothing */
}