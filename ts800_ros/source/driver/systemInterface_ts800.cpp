
#include "systemInterface_ts800.h"
#include "LibRobotControlInterface.h"
#include "LibRobotControlDrive.h"
#include "LibRobotDataInterface.h"
#include "LibRobotSoundInterface.h"
#include "LibRobotDisplayInterface.h"
#include "LibRobotControlLocalization.h"
#include "commonStruct.h"
#include "userInterface.h"
#include "control/control.h"
// #include "MessageHandler.h"

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

CSystemInterfaceTs800::CSystemInterfaceTs800()
{
    if (connect())
    {
        eblog(LOG_LV_NECESSARY, "system interface connect ok");	
        bReceiveDataLoopRunning = true;
        pthread_create(&thSystemInterface, nullptr, &CSystemInterfaceTs800::threadLoopWrap, this);
    }
    else
    {
        eblog(LOG_LV, "/n/n/n connect fail /n/n/n");
    }
    eblog(LOG_LV,  "");
}

CSystemInterfaceTs800::~CSystemInterfaceTs800()
{
    bReceiveDataLoopRunning = false;
    pthread_join(thSystemInterface, nullptr);

    ceblog(LOG_LV_NECESSARY, GREEN, "disConnnect st");
    disConnect();
    ceblog(LOG_LV_NECESSARY, GREEN, "disConnnect end");
}

bool CSystemInterfaceTs800::connect()
{
    bool bRet = true;
    bool bIsOpen = false;
    int connectTryCnt = 0;
    const int MaxTryCnt = 10;

    /* 1. Control Interface Open */
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Control Interface\t"<<BOLDWHITE<<"연결중...");
    bIsOpen = true;
    connectTryCnt = 0;
    while( RoCtrInf.Open(0)==false )
    {
        if ( ++connectTryCnt > MaxTryCnt )
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"Control Interface\t"<<BOLDRED<<" RoCtrInf.Open() fail "<<"retry count : "<< connectTryCnt-1);
            bIsOpen = false;
            break;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t연결중...");
        }
        usleep(100000);//0.1초
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Control Interface\t"<<(bIsOpen?BOLDGREEN:BOLDRED)<<(bIsOpen?"연결성공":"연결실패"));    
    

    /* 2. Control Interface Open */    
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Data Interface\t"<<BOLDWHITE<<"연결중...");
    bIsOpen = true;
    connectTryCnt = 0;
    while( RoDatInf.Open()==false )
    {
        if ( ++connectTryCnt > MaxTryCnt )
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"Data Interface\t"<<BOLDRED<<" RoDatInf.Open() fail "<<"retry count : "<< connectTryCnt-1);
            bIsOpen = false;
            break;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t연결중...");
        }
        usleep(100000);//0.1초
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Data Interface\t"<<(bIsOpen?BOLDGREEN:BOLDRED)<<(bIsOpen?"연결성공":"연결실패"));    

    /* 3. Sound Interface Open */
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Sound Interface\t"<<BOLDWHITE<<"연결중...");
    bIsOpen = true;
    connectTryCnt = 0;
    while( RoSoundInf.Open()==false )
    {
        if ( ++connectTryCnt > MaxTryCnt )
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"Sound Interface\t"<<BOLDRED<<" RoSoundInf.Open() fail "<<"retry count : "<< connectTryCnt-1);
            bIsOpen = false;
            break;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t연결중...");
        }
        usleep(100000);//0.1초
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Sound Interface\t"<<(bIsOpen?BOLDGREEN:BOLDRED)<<(bIsOpen?"연결성공":"연결실패"));    

#if 1 // hjkim240206 - launcher program 자동실행 아닐때 -- LCD DISPLAY INIT 실패로 SYSTEM INTERFACE 열리지 않음 그럴때 여기 막으세요
    /* 4. LCD Interface Open */    
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t"<<BOLDWHITE<<"연결중...");
    bIsOpen = true;
    connectTryCnt = 0;
    while( RoDisplayInf.Open()==false ) // Open()시 자동으로 4개 기둥이 있는 화면으로 바뀜.
    {
        if ( ++connectTryCnt > MaxTryCnt )
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,"Display Interface\t"<<BOLDRED<<" RoDisplayInf.Open() fail "<<"retry count : "<< connectTryCnt-1);
            bIsOpen = false;
            break;
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t연결중...");
        }
        usleep(100000);//0.1초
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Display Interface\t"<<(bIsOpen?BOLDGREEN:BOLDRED)<<(bIsOpen?"연결성공":"연결실패")<<"\n\n\n");

#endif

    //firware version : 0.03 이 현재 최종 버전임.
    float fFwVersion = RoDatInf.GetFwVersion();
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Firware version: "<< fFwVersion <<"\n");
    printf("Firware version:  %2.2f\n", fFwVersion);

    return bRet;    
}

bool CSystemInterfaceTs800::disConnect()
{
    eblog(LOG_LV, "[System Interface] disConnect() call!  ");
    RoCtrInf.Drive.Inf_DriveWheel(0, 0, 0, 0, 0, 0);
    eblog(LOG_LV, "[System Interface] wheel stop");
    
    sleep(1);
    
    bReceiveDataLoopRunning = false; // thread ReceiveData 종료
    system("touch /home/ebot/data_close.log");
    RoDatInf.Close();
    eblog(LOG_LV, "[System Interface] Data Interface Close() ");
    system("touch /home/ebot/ctrl_close.log");
    RoCtrInf.Close();
    eblog(LOG_LV, "[System Interface] Control Interface Close() ");

    if ( RoDatInf.IsOpen() || RoCtrInf.IsOpen() )
    {
        return false;
    }

    return true;
}

void CSystemInterfaceTs800::threadLoop()
{
    CSystemInterface::threadLoop();
}

void CSystemInterfaceTs800::setWheelMotorData()
{
    tSysWheelMotor_ts800 tempCurrent;

    RoDatInf.Drive.Inf_GetDriveVelocityData(&tempCurrent.lineVelocity,&tempCurrent.angularVelocity);      
    RoDatInf.Drive.Inf_GetDriveCurrentData(&tempCurrent.leftCurrent, &tempCurrent.rightCurrent, &tempCurrent.dummyCurrent);
    RoDatInf.Drive.Inf_GetDriveEncoderData(&tempCurrent.leftEncoder, &tempCurrent.rightEncoder, &tempCurrent.dummyEncoder);
    RoDatInf.Drive.Inf_GetDriveGainData(&tempCurrent.leftGain, &tempCurrent.rightGain, &tempCurrent.dummyGain);

    //ceblog(LOG_LV_SYSTEMINF, GREEN, "Velocity  v : [" << tempCurrent.lineVelocity << "] w : [" << tempCurrent.angularVelocity << " ]");

    wheelMotorData = tempCurrent;
    bUpdateStateWheelMotorData = true;
}

void CSystemInterfaceTs800::setDisplayStateData()
{
    E_DisplayStatus tempState = RoDisplayInf.Inf_GetStatus();

    if ( tempState == E_DisplayStatus::STOP)    return DISPLAY_CTR.setDisplayState( E_DISPLAY_STATE::STOP );
    if ( tempState == E_DisplayStatus::PLAYING) return DISPLAY_CTR.setDisplayState( E_DISPLAY_STATE::PLAYING );
    else { ceblog((LOG_LV_NECESSARY|LOG_LV_ERROR), RED, " Display State Error @@@@"); }
}

void CSystemInterfaceTs800::setSoundStateData()
{
    E_SoundStatus tempState = RoSoundInf.Inf_GetStatus();

    if ( tempState == E_SoundStatus::STOP)    return SOUND_CTR.setSoundState( E_SOUND_STATE::STOP );
    if ( tempState == E_SoundStatus::PLAYING) return SOUND_CTR.setSoundState( E_SOUND_STATE::PLAYING );
    else { ceblog((LOG_LV_NECESSARY|LOG_LV_ERROR), RED, " Sound State Error @@@@"); }
}

/**
 * @brief TS800용 휠 제어 interface
 * 
 * @param direction 
 * @param L_Speed 
 * @param R_Speed 
 * @param B_Speed 
 * @param duty 
 * @param bHeadingPid 
 * @return true  : 전송 성공
 * @return false : 전송 실패
 */
bool CSystemInterfaceTs800::transControlWheel(u8 direction, s32 L_Speed, s32 R_Speed, s32 B_Speed, u32 duty, bool bHeadingPid)
{
    bool ret = false;
    int retryCnt = 0;
    /* 현재는 속도의 변화가 있거나, 방향이 정지 혹은 Dynamic 일때만 휠제어 */    
    //RoCtrInf.Drive.Inf_DriveAnglePid(bHeadingPid);
    ret = RoCtrInf.Drive.Inf_DriveWheel(direction, L_Speed, R_Speed, B_Speed, duty, bHeadingPid);
    
    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false)
    {
        eblog(LOG_LV_ERROR, "ERROR : trans STOP retry (L: " << L_Speed << "  R: " <<R_Speed << "   B: " << B_Speed  << " dir( " << (int)direction << ") duty(" << duty << ") bHeadingPid( "<<  bHeadingPid << ")tick(" << get_system_time() << ")");
        ret = RoCtrInf.Drive.Inf_DriveWheel(0, 0, 0, 0, 0, false);// error 상황이여 강제 정지 명령.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    //hjkim230630 - 멈춤 현상 확인 될깨까지 켜두세요 - 해결되면  Block
    //eblog(LOG_LV_ERROR, "Control Wheel (L: " << L_Speed << "  R: " <<R_Speed << "   B: " << B_Speed  << " dir( " << (int)direction << ") duty(" << duty << ") bHeadingPid( "<<  bHeadingPid << ")tick(" << get_system_time() << ")");
    return ret;
}

/**
 * @brief TS800용 휠 제어 interface
 * 
 * @param direction 
 * @param L_Speed 
 * @param R_Speed 
 * @param B_Speed 
 * @param duty 
 * @param bHeadingPid 
 * @return true  : 전송 성공
 * @return false : 전송 실패
 */
bool CSystemInterfaceTs800::transControlWheel(int seq, u8 direction, s32 L_Speed, s32 R_Speed, s32 B_Speed, u32 duty, bool bHeadingPid)
{
    bool ret = false;
    int retryCnt = 0;
    /* 현재는 속도의 변화가 있거나, 방향이 정지 혹은 Dynamic 일때만 휠제어 */    
    //RoCtrInf.Drive.Inf_DriveAnglePid(bHeadingPid);
    ret = RoCtrInf.Drive.Inf_DriveWheel(direction, L_Speed, R_Speed, B_Speed, duty, bHeadingPid);
    
    // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[\t"<<seq<<" ] @@@ "<<enumToString((E_WHEEL_CONTROL)direction)
    //     <<" Speed( "<<L_Speed<<",\t"<<R_Speed<<",\t"<<B_Speed<<")\tDuty( "<<duty<<" )\tHeadingPid( "<<bHeadingPid<<" )" << "\ttick( " << get_system_time() << " )");
    
    //ceblog(LOG_LV_NECESSARY,BOLDCYAN, " AP보드 제어 메시지 send 직전 속도: "<<  L_Speed << " , " <<  R_Speed << " , " << B_Speed );

    //return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.
    while(ret == false)
    {
        std::cout << "ERROR : trans STOP retry"<<++retryCnt<<std::endl;
        //ret = RoCtrInf.Drive.Inf_DriveWheel(0, 0, 0, 0, 0, false);// error 상황이여 강제 정지 명령.
        ret = RoCtrInf.Drive.Inf_DriveWheel(direction, L_Speed, R_Speed, B_Speed, duty, bHeadingPid);
        ceblog(LOG_LV_NECESSARY, YELLOW, "[\t"<<seq<<" ] @@@ "
            <<" Speed( "<<0<<",\t"<<0<<",\t"<<0<<")\tDuty( "<<0<<" )\tHeadingPid( "<<false<<" )" << "\ttick( " << get_system_time() << " )");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return ret;
}

s32 debugLV = 0, debugAV = 0;
bool CSystemInterfaceTs800::transControlWheel(int seq, s32 linearVelocity, s32 angularVelocity, u32 radius)
{
#if 0 // 신규 제어기 안정화까지 디버그용으로 print 하겠습니다.
    if(debugLV != linearVelocity || debugAV != angularVelocity)
    {   
        ceblog(LOG_LV_NECESSARY, GRAY, " seq: "<<seq<<"\tlinear: "<<int(linearVelocity)<<" mm/s\tangular: "<<RAD2DEG(double(angularVelocity*0.001))<<" deg/s");
        debugLV = linearVelocity;
        debugAV = angularVelocity;
    }
#endif

    bool ret = false;
    int retryCnt = 0;

    ret = RoCtrInf.Drive.Inf_DriveVelocity(linearVelocity, angularVelocity);

    /* return 값이 false 이면 제어 명령 오류. 될때 까지 STOP 한다.*/
    while (ret == false)
    {
        if(retryCnt > 16)   //3hz 동안 처리 못하면 포기.
        {
            ret = true;
            ceblog(LOG_LV_NECESSARY, BOLDRED, "Error! - trans retry 횟수: " << ++retryCnt);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Warning! - trans retry 횟수: " << ++retryCnt);
        }
        // ret = RoCtrInf.Drive.Inf_DriveVelocity(0, 0);
        ret = RoCtrInf.Drive.Inf_DriveVelocity(linearVelocity, angularVelocity);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " retry val, set : "<<seq<<"\tlinear: "<<int(linearVelocity)<<" mm/s\tangular: "<<RAD2DEG(double(angularVelocity*0.001))<<" deg/s");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return ret;
}
/**
 * @brief TS800용 LCD 디스플레이 제어 함수 - 이미지 플레이
 * 
 * @param type 이미지 종류
 */
void CSystemInterfaceTs800::transControlCustomDisplay(char* img)
{
    RoDisplayInf.Inf_Play_Custom((unsigned char *)img);
    DISPLAY_CTR.setCustomImage(img);
    eblog((LOG_LV_NECESSARY | LOG_LV_SYSTEMINF), "[System Interface] transControlCustomDisplay");
    //eblog(LOG_LV_SYSTEMINF, "[System Interface] transControlDisplay [" << (int)type << " ]");
}
/**
 * @brief TS800용 LCD 디스플레이 제어 함수 - 이미지 플레이
 * 
 * @param type 이미지 종류
 */
void CSystemInterfaceTs800::transControlDisplay(E_DisplayImageClass img)
{
    DISPLAY_CTR.setCurImage(img);
    RoDisplayInf.Inf_Play(img);
    eblog((LOG_LV_NECESSARY | LOG_LV_SYSTEMINF), "[System Interface] transControlDisplay : " << DEC(img));
    //eblog(LOG_LV_SYSTEMINF, "[System Interface] transControlDisplay [" << (int)type << " ]");
}

//-----------------------------------------------------------------------------
// TS800용 LCD 디스플레이 제어 함수 
//
// Descriptions :
//      - LCD 재생을 정지한다. 상태는 STOP으로 변경.
//      - 이 함수 호출 시, 재생 목록은 clear 된다.
//-----------------------------------------------------------------------------
void CSystemInterfaceTs800::transControlDisplayStop()
{
    RoDisplayInf.Inf_Stop();
    eblog((LOG_LV_NECESSARY | LOG_LV_SYSTEMINF), "[System Interface] transControlDisplay Stop !!");
}

//-----------------------------------------------------------------------------
// TS800용 사운드 제어 함수 
//
// Descriptions :
//      - 사운드 재생을 정지한다. 상태는 STOP으로 변경.
//      - 이 함수 호출 시, 재생 목록은 clear 된다.
//-----------------------------------------------------------------------------
void CSystemInterfaceTs800::transControlSoundStop()
{
    RoSoundInf.Inf_Stop();
    eblog((LOG_LV_NECESSARY | LOG_LV_SYSTEMINF), "[System Interface] transControlSound Stop !!");
}

/**
 * @brief TS800용 LED 제어 함수
 * 
 * @param dir 
 * @param R 
 * @param G 
 * @param B 
 */
void CSystemInterfaceTs800::transControlLed(u32 dir, u8 R, u8 G, u8 B, u8 W)
{
    RoCtrInf.Display.Inf_DisplayCustom(dir,R,G,B,W);
    eblog(LOG_LV_SYSTEMINF, "[System Interface] transControlLed [ "<< (int)dir << " ]" );
}

/**
 * @brief TS800용 
 * 
 * @param type 
 */
void CSystemInterfaceTs800::transControlSoundplay(E_SoundClass sound)
{
    SOUND_CTR.setCurSound(sound);
    RoSoundInf.Inf_Play(sound);
    eblog((LOG_LV_NECESSARY | LOG_LV_SYSTEMINF), "[System Interface] transControlSoundplay [" << DEC(sound)<<"]");
    //eblog(LOG_LV_SYSTEMINF, "[System Interface] transControlSoundplay ["<< (int)type << "]" );
}

/**
 * @brief TS800용 Tilting 제어 함수
 * 
 * @param control 
 */
void CSystemInterfaceTs800::transControlTilt(E_TILTING_CONTROL control)
{
    bool bRet = false;
    s16 retryCnt = 0;

    switch (control)
    {
       case E_TILTING_CONTROL::UP : 
       bRet = RoCtrInf.Tilting.Inf_TiltingUp();
       break;
       case E_TILTING_CONTROL::DOWN :
       bRet = RoCtrInf.Tilting.Inf_TiltingDown();
       break;
       default: /*Tilt Stop*/
       bRet = RoCtrInf.Tilting.Inf_TiltingMovingStop();
       break;
    }

    while(bRet == false)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), BOLDGREEN, "TILTING CONTROL - ERROR : CONTROL : " << (int)control << "retry [" << SC<int>(++retryCnt) <<"]" );
        switch (control)
        {
            case E_TILTING_CONTROL::UP : 
            bRet = RoCtrInf.Tilting.Inf_TiltingUp();
            break;
            case E_TILTING_CONTROL::DOWN :
            bRet = RoCtrInf.Tilting.Inf_TiltingDown();
            break;
            default: /*Tilt Stop*/
            bRet = RoCtrInf.Tilting.Inf_TiltingMovingStop();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // retryCnt 높아지면 에러
    }

    eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] transControlTilt [" << (int)control << "] ");
}

/**
 * @brief TS800용 Fan (dry mop) 제어 함수
 * 
 * @param on 
 * @param speed 
 */
void CSystemInterfaceTs800::transControlDryFan(bool on,u16 speed)
{
    bool bRet = false;
    s16 retryCnt = 0;

    if (on)
    {
        bRet = RoCtrInf.Cradle.Inf_FanOn(speed);
        while(bRet == false)
        {
            ceblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), BOLDGREEN, "DRY FAN ON - ERROR : trans STOP retry [" << SC<int>(++retryCnt) <<"]" );
            bRet = RoCtrInf.Cradle.Inf_FanOn(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            // retryCnt 높아지면 에러
        }
        eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] DRY FAN- ON\tspeed: " << speed <<"\t bRet : " << SC<int>(bRet));
    }
    else
    {
        bRet = RoCtrInf.Cradle.Inf_FanOff();
        while(bRet == false)
        {
            ceblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), BOLDGREEN, "DRY FAN - OFF ERROR : trans STOP retry [" << SC<int>(++retryCnt) <<"]" );
            bRet = RoCtrInf.Cradle.Inf_FanOff();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            // retryCnt 높아지면 에러
        }

        eblog((LOG_LV_NECESSARY|LOG_LV_SYSTEMINF), "[System Interface] DRY FAN - OFF \tspeed: " << speed <<"\t bRet : " << SC<int>(bRet));
    }
    // eblog(LOG_LV_SYSTEMINF | LOG_LV_DOCKING, "[System Interface] transControlDryFan RESULT :  " << SC<int>(ret));
}

void CSystemInterfaceTs800::transDockingAlive()
{
    s16 retryCnt = 0;

    while(!RoCtrInf.Cradle.Inf_DockingAlive()) // true면 end.
    {
        if(++retryCnt > 100)
        {
            ceblog((LOG_LV_SYSTEMINF | LOG_LV_DOCKING | LOG_LV_ERROR), BOLDRED, "***ERROR : trans STOP retry [" << SC<int>(retryCnt) <<"]***" );
        }
        else
        {
            ceblog((LOG_LV_SYSTEMINF | LOG_LV_DOCKING), BOLDGREEN, "trans STOP retry [" << SC<int>(retryCnt) <<"]" );
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ceblog((LOG_LV_SYSTEMINF | LOG_LV_DOCKING), YELLOW, "trans docking alive");
}

void CSystemInterfaceTs800::transSlamPoseData(int x, int y, unsigned int angle)
{
    RoCtrInf.MovingData.Inf_SetSlamPoseData(x, y, angle);
}

void CSystemInterfaceTs800::transSlamPoseData(int x, int y, float fA, int nTimeOffset)
{
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    RoCtrInf.MovingData.Inf_SetSlamPoseData(x, y, fA, nTimeOffset);
#endif
}