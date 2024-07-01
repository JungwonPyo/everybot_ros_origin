#include "location.h"

// user defined header
#include "utils.h"
#include "eblog.h"
#include "commonStruct.h"
#include "control/motionPlanner/motionPlanner.h"
#include "kinematics.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CLocation::CLocation()
{
    CStopWatch __debug_sw;
   
    isSucceedLocation = false;
    info.pose = tPose();
    info.rotateState = E_LOCATION_ROTATE_STATE::_1_TURN_LEFT;
    info.state       = E_LOCATION_STATE::NONE;
    pRmcl = new CRmcl();
    printf("new\nCLocation\n");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CLocation::~CLocation()
{
    CStopWatch __debug_sw;
    delete pRmcl;
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief location proc 실행하기전에 location 초기화
 * jhnoh, 23.02.01
 */
void CLocation::initLocation()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "*-*-*-*-*-*-*-* [ LOCATION - START ] *-*-*-*-*-*-*-* ");
    // checkMapUpdate = false;
    // checkLidarUpdate = false;
    isSucceedLocation = false;
    info.pose = tPose();
    info.rotateState = E_LOCATION_ROTATE_STATE::_1_TURN_LEFT;
    info.state       = E_LOCATION_STATE::NONE;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief location proc
 * jhnoh, 23.01.16
 * @param robotPose     로봇 좌표 
 * @return tLocationInfo 
 */
tLocationInfo CLocation::locationProc(tPose robotPose)
{
    CStopWatch __debug_sw;
    
    // location procedure
    switch (info.state)
    {
    case E_LOCATION_STATE::NONE:
        setLocationState(procNone());
        break;
    case E_LOCATION_STATE::RMCL:
        setLocationState(procRmcl(robotPose));
        break;
    case E_LOCATION_STATE::FAIL_UPDATE:
        setLocationState(procFailUpdate());
        break;
    case E_LOCATION_STATE::FAIL_LOCATION:
        setLocationState(procFailLocation());
        break;
    case E_LOCATION_STATE::END:
        break;
    default:
        break;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return info;
}

/**
 * @brief location 초기 세팅
 * jhnoh, 23.02.01
 * @return E_LOCATION_STATE 
 */
E_LOCATION_STATE CLocation::procNone()
{
    CStopWatch __debug_sw;

    E_LOCATION_STATE ret = E_LOCATION_STATE::NONE;

    ret = E_LOCATION_STATE::RMCL;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief location 단계
 * jhnoh, 23.02.01
 * @param robotPose         로봇 좌표 
 * @return E_LOCATION_STATE 
 */
E_LOCATION_STATE CLocation::procRmcl(tPose robotPose)
{
    CStopWatch __debug_sw;
    CRobotKinematics k;
    E_LOCATION_STATE ret = E_LOCATION_STATE::RMCL;



#if USE_SAVEDMAP
    if(ServiceData.robotMap.isExistSavedMap()) //hjkim230906 : 이거 디파인 설정되면 오류 납니다.
    {
        //eblog(LOG_LV, " -- Copied the saved map. --"  );
        mapSet = ServiceData.robotMap.copySavedGridMapFromYAML(pGridmap, &mapInfo);
    }
    else
    {
        // eblog(LOG_LV, " -- There are no saved maps. Real-time map copied. --"  );
        mapSet = ServiceData.robotMap.copySimplifyGridMap(pGridmap, &mapInfo);
    }
#else
    ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
    u8 *pGridmap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
    tGridmapInfo mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();    
#endif
    
    
    // 실시간 RMCL 
    pRmcl->updateMapData(pGridmap, mapInfo);
    ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
    // Lidar 업데이트
    pRmcl->updateLaserData(ServiceData.localiz.getRawLidarDist());
    // odom 업데이트
    pRmcl->updateOdomData(ServiceData.localiz.getSysPose());


    /* 로봇의 움직임만큼 파이클들 pose 업데이트 */
    pRmcl->updateParticlesByMotionModel(); 

    // 1. canUpdateScan_ 를 false 로 하여 scan 값 업데이트를 막음.
    // 2. 현재 scan값이 유효한지 판단.
    // scan 데이터를 확인하여 invalid 거리값들(r < min or r > max)이 많으면
    // scan 데이터를 사용하지 않음. 이때, scanMightInvalid_ 를 true로 함.
    pRmcl->setCanUpdateScan(false);
    
    // 모델 움직임에 따른 가능도 계산
    pRmcl->calculateLikelihoodsByMeasurementModel();
    pRmcl->calculateLikelihoodsByDecisionModel();

    // GLPoseSampler 를 사용할때만 계산
    pRmcl->calculateGLSampledPosesLikelihood();

    // AugmentedMCL 를 사용할때만 계산
    pRmcl->calculateAMCLRandomParticlesRate();

    // sample size 를 계산
    pRmcl->calculateEffectiveSampleSize();

    /* mcl pf 로 추정되는 pose를 mclPose_에 저장 */
    pRmcl->estimatePose();


    pRmcl->resampleParticles();
    
    //pRmcl->plotLikelihoodMap();
    pRmcl->setCanUpdateScan(true);

    info.pose = pRmcl->getExtimatedPose();
    
    static double __maxTime;
    static int __count;
    static double __avgTime;
    static double __prevTime;
    __prevTime = get_system_time();

    /* 짜잔 계산해보장 */
    double calcTime = get_system_time(__prevTime);
    if( calcTime > __maxTime )
    {
        __maxTime = calcTime;
    }
    __avgTime = (__avgTime*__count+calcTime)/(__count+1);
    __count++;
    //////////////////////////////////////

    if((int)(get_system_time()*10) % 10 == 0)
    {
        eblog(LOG_LV, "[Calc Time] curTime : " << int(calcTime*1000) << " (ms) avgTime : "<< int(__avgTime*1000) << " (ms) maxTime : " << int(__maxTime*1000));
        eblog(LOG_LV,  "||______________________________ LOCATION RESULT______________________________||"  );
        eblog(LOG_LV, "Slam: x = " << ServiceData.localiz.getSlamPose().x << " [m], y = " << ServiceData.localiz.getSlamPose().y << " [m], yaw = " << ServiceData.localiz.getSlamPose().angle * (180.0 / M_PI) << " [deg]" );
        pRmcl->printResult();
        eblog(LOG_LV, "||______________________________ *******************______________________________||"  );
    }
    if(isSucceedLocation == false)
    {   
        checkLocalizeProc(robotPose);
        // RMCL 자기 위치 추정 complete 판단 조건
        if(pRmcl->getReliability() == 1 && 0.0 < pRmcl->getFinalMAE() &&  pRmcl->getFinalMAE() < 0.1 && getLocationRotateState() == E_LOCATION_ROTATE_STATE::END)
        {
            ceblog(LOG_LV, YELLOW, " The robot has succeeded in estimating position. ");
            isSucceedLocation = true;
            double pendingAngle = utils::math::rad2deg(robotPose.angle);
            if ( pendingAngle > 180.0 )	pendingAngle = 360.0 - pendingAngle;
            
            // 방향을 결정한다. ( true : 반시계, false : 시계)
            if(utils::math::getTurnDirection(pendingAngle,0)) {
                targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
            }
            else  {
                targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
            }
        }
    }
    else if( isSucceedLocation == true && MOTION.isRunning() == false)
    {
        locationEnd();
        ret = E_LOCATION_STATE::END;
    }
    else
    {
        eblog(LOG_LV_LINECLEAN,  " wait!! The robot is centering.");
    }

    

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 업데이트 실패 시
 * jhnoh, 23.01.30
 * @param robotPose 
 * @return E_LOCATION_STATE 
 */
E_LOCATION_STATE CLocation::procFailUpdate()
{
    CStopWatch __debug_sw;

    E_LOCATION_STATE ret = E_LOCATION_STATE::FAIL_UPDATE;
    eblog(LOG_LV,  "[LOCATION] ::: FAIL_UPDATE - START");

    // if(!checkLidarUpdate)
    // {
    //     // 라이다 데이터 업데이트 실패시 어찌 할까?
    // }

    // if(!checkMapUpdate)
    // {
    //     // 맵 데이터 업데이트 실패시 어찌 할까?
    // }

    eblog(LOG_LV,  "[LOCATION] ::: FAIL_UPDATE - END" );
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief  위치 추정 실패 단계
 * jhnoh, 23.01.30
 * @param robotPose 
 * @return E_LOCATION_STATE 
 */
E_LOCATION_STATE CLocation::procFailLocation()
{
    CStopWatch __debug_sw;

    E_LOCATION_STATE ret = E_LOCATION_STATE::FAIL_LOCATION;

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 탐색 종료 
 * jhnoh, 23.01.30
 */
void CLocation::locationEnd()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "*-*-*-*-*-*-*-* [ LOCATION - END ] *-*-*-*-*-*-*-* " );
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief location 상태 디버그 함수
 *   디버그 규칙 : 1. BOLDCYAN : (대괄호) state 상태 출력
 *                2. green    : (정보) 일반 정보 출력
 *                3. yellow   : (위기) 기회가 있는 실패
 *                4. red      : (실패) 기회가 없는 실패
 * BEAST jhnoh, 23.02.01
 */
void CLocation::__debug_state_print()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "[LOCATION] -  STATE : ( "<< enumToString(info.state) << " )");
    
    TIME_CHECK_END(__debug_sw.getTime());
}



void CLocation::checkLocalizeProc(tPose robotPose)
{
    CRobotKinematics k;
    switch (getLocationRotateState())
    {
    case E_LOCATION_ROTATE_STATE::_1_TURN_LEFT:
        targetAng = k.rotation(robotPose, DEG2RAD(90));
        MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
        setLocationRotateState(E_LOCATION_ROTATE_STATE::_2_TURN_RIGHT);
        break;
    case E_LOCATION_ROTATE_STATE::_2_TURN_RIGHT:
        if (MOTION.isRunning() == false)
        {
            targetAng = k.rotation(robotPose, DEG2RAD(180));
            MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
            setLocationRotateState(E_LOCATION_ROTATE_STATE::_3_TURN_LEFT);
        }
        break;
    case E_LOCATION_ROTATE_STATE::_3_TURN_LEFT:
        if (MOTION.isRunning() == false)
        {
            targetAng = k.rotation(robotPose, DEG2RAD(90));
            MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
            setLocationRotateState(E_LOCATION_ROTATE_STATE::END);
        }
        break;
    case E_LOCATION_ROTATE_STATE::END:
        if (MOTION.isRunning() == false)
        {
            setLocationRotateState(E_LOCATION_ROTATE_STATE::_1_TURN_LEFT);
        }
        break;
    default:
        break;
    }  
}

E_LOCATION_ROTATE_STATE CLocation::getLocationRotateState()         {return info.rotateState;}
void CLocation::setLocationState(E_LOCATION_STATE set)              {info.state = set;}
void CLocation::setLocationRotateState(E_LOCATION_ROTATE_STATE set) {info.rotateState = set;}
