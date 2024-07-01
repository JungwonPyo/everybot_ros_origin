#include "kidnap.h"
#include "eblog.h"
#include "control/motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "kinematics.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CKidnap::CKidnap(CLocation* _pLocation) :
                pLocation(_pLocation),
                bKidnapGeneration(false),
                info(E_KIDNAP_STATE::NONE),
                estimatedPose(tPose()),
                paceData(tKidnapPaceData()),
                prePaceData(tKidnapPaceData()),
                mStep(RELOCAL_STEP::RELOCAL_FIRST_STEP),
                bDebugStatePrint(true),
                bSlamResume(false),
                kidnapInfo(E_KIDNAPROBOT_STATE::ROTATING)
{
    CStopWatch __debug_sw;

// 큐를 이용하기 위해 초기화 불가능 : 큐 동작 확인 후 삭제 예정
#if 0
    preSlamPoses = tPose();  
    preSysPoses  = tPose();   
    preImuDatas  = tSysIMU();   
    preTimeDatas = 0.0;  

    curSlamPoses    = tPose();  
    curSysPoses     = tPose();  
    curImuDatas     = tSysIMU();   
    curTimeDatas    = 0.0;  
#endif

    bAngleSetting = false;

    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CKidnap::~CKidnap()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief kidnap 발생 여부 체크 및 자기 위치 추정 
 * jhnoh, 23.05.08
 * @return true  : kidnap 발생 ~ 자기 위치 추정 중
 * @return false : kinap 발생 안했음
 */
bool CKidnap::autoKidnapAndEstimateRobotPose()
{
    CStopWatch __debug_sw;

    if(bKidnapGeneration == false)
    {
        lastPoseList.emplace_back(ServiceData.localiz.getSlamPose().x,ServiceData.localiz.getSlamPose().y);
        if(lastPoseList.size() > 10)
        {
            lastPoseList.pop_front();
        }

        if(checkKidnap())
        {
            
            
            initKidnap();

            // 카토그래퍼 finish_trajectory
            ROBOT_CONTROL.slam.setRelocalCompleted(true); //for pathplan pose is slam of the yaw
            eblog(LOG_LV_NECESSARY,"\t" << "|SlamPose:" << SC<double>(ServiceData.localiz.getSlamPose().x) << ", " << SC<double>(ServiceData.localiz.getSlamPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
            eblog(LOG_LV_NECESSARY,"\t" << "|CevaPose:" << SC<double>(ServiceData.localiz.getSysPose().x) << ", " << SC<double>(ServiceData.localiz.getSysPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSysPose().angle)));
            ROBOT_CONTROL.slam.setSlamLocalUpdate(false);
            ROBOT_CONTROL.slam.pauseSlam();
            
            bSlamResume = false;
            bKidnapGeneration = true;

            //Clear of the motion filter of the kidnap active flag clear
            //ServiceData.obstacle.clearLidarKidnap();            
        }
    }
    else
    {
        // 키드냅 proc 동작
        kidnapProc(); 

        if(getKidnapState() == E_KIDNAP_STATE::END && MOTION.isRunning() == false)
        {
            bKidnapGeneration = false;
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return bKidnapGeneration;
}

/**
 * @brief kidnap proc!
 * jhnoh, 23.05.08
 */
void CKidnap::kidnapProc()
{
    CStopWatch __debug_sw;

    __debug_kidnap_state_print();

    switch (getKidnapState())
    {
    case E_KIDNAP_STATE::NONE:
        setKidnapState(procNone());
        break;
    case E_KIDNAP_STATE::READY_LOCATION:
        setKidnapState(procReadyLocation());
        break;
    case E_KIDNAP_STATE::LOCATION:
        setKidnapState(procLocation());
        break;
    case E_KIDNAP_STATE::MOVE_LAST_POSITION:
        setKidnapState(procMoveLastPosition());
        break;
    case E_KIDNAP_STATE::ANGLE_SETTING:
        setKidnapState(procAngleSetting(0.0));
        break;
    case E_KIDNAP_STATE::END:
        break;
    default:
        break;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief RMCL을 통해 추정된 pose를 set!
 * jhnoh, 23.05.08
 * @param pose 
 */
void CKidnap::setEstimatedPose(tPose pose)
{
    CStopWatch __debug_sw;

    estimatedPose = pose;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief RMCL을 통해 추정된 pose를 get!
 * jhnoh, 23.05.08
 * @return tPose 
 */
tPose CKidnap::getEstimatedPose()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return estimatedPose;
}

/**
 * @brief 키드냅 발생 검사 함수 
 * jhnoh, 23.05.08
 * @return true : 키드냅 발생 했다!
 * @return false : 키드냅 발생 안했다!
 */
bool bOneTime = false;
int checkCnt = 0;

bool CKidnap::checkKidnap()
{
    CStopWatch __debug_sw;

    bool ret = false;
    tSysIMU tempImu = ServiceData.obstacle.getFilterImuData();
    if(bOneTime == false)
    {
        //ceblog(LOG_LV_RMCL,BOLDGREEN , "Ax Ay Az | roll pitch yaw");    
        bOneTime = true;
    }
    else if( abs(tempImu.Ax) >= 4 || abs(tempImu.Ay) >= 4)
    {
        checkCnt++;
        if( checkCnt>=3)
        {
            std::cout << tempImu.Ax << " , " << tempImu.Ay << " , " << tempImu.Az << " , " << tempImu.Groll<< " , " << tempImu.Gpitch << " , " << tempImu.Gyaw << std::endl;  
#if 1 //after 1 => 0 : imu ax, ay = 3 or 7 of the random (new ceva version) -> pixart..
            ret = false;
#else
            ret =true;
#endif
            checkCnt =0;
        }
    }
    else
    {
        checkCnt =0;
    }
#if 0 // 키드냅 조건 데이터를 위한 확인용 디버그
    // 데이터 비교를 위한 현재 로봇의 pace데이터와 pre데이터를 업데이트
    updateData();

    tKidnapPaceData curData = paceData;
    tKidnapPaceData preData = prePaceData;

    if(fabs(curData.imuPitch-preData.imuPitch)> 3|| fabs(curData.imuRoll-preData.imuRoll)> 3)
    {
        eblog(LOG_LV,  " sys diff [ " <<fabs(curData.sysX-preData.sysX) << " , " << fabs(curData.sysY-preData.sysY) << " , " << fabs(curData.sysAngle-preData.sysAngle) << " ] ");
        eblog(LOG_LV,  " slam diff [ " <<fabs(curData.slamX-preData.slamX) << " , " << fabs(curData.slamY-preData.slamY) << " , "  << fabs(curData.slamAngle-preData.slamAngle) << " ] ");
        eblog(LOG_LV,  " imu diff [ " <<fabs(curData.imuPitch-preData.imuPitch) << " , " <<  fabs(curData.imuRoll-preData.imuRoll) << " ] ");
    }
// 로그로 데이터 체크 후에 확인

    if(fabs(curData.imuPitch-preData.imuPitch) > 3 || fabs(curData.imuRoll-preData.imuRoll) > 3) 
    {
        eblog(LOG_LV,  " imu diff [ " <<fabs(curData.imuPitch-preData.imuPitch) << " , " <<  fabs(curData.imuRoll-preData.imuRoll) << " ] ");
        eblog(LOG_LV,  " [ Kidnap occurrence ] ");
        ret = true;
    }
#else
    /**
     * @brief lidar of the kidnap trigger or IMU of the kidnap trigger (TODO)
     * 
     */
    //ret = ServiceData.obstacle.getLidarKidnap();
#endif
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 이전 데이터를 업데이트 하는 함수!
 * jhnoh, 23.05.08
 * @param updateTime 업데이트 주기 (초) 
 *                   updateTime 초 전에 데이터와 현재 데이터를 비교하기 위함.
 */
void CKidnap::updateData(double checkTime, tPose slamPose, tPose sysPose, tSysIMU imuData)
{
    CStopWatch __debug_sw;

    // 현재 데이터를 이전 데이터 deque의 맨 앞에 추가
    curSlamPoses.push_front(slamPose);
    curSysPoses.push_front(sysPose);
    curImuDatas.push_front(imuData);
    curTimeDatas.push_front(get_system_time());

    // 1초 이상된 데이터는 deque의 맨 뒤에서부터 제거
    while (curTimeDatas.size() > 0 && get_system_time(curTimeDatas.back()) > checkTime)
    {
        curSlamPoses.pop_back();
        curSysPoses.pop_back();
        curImuDatas.pop_back();
        curTimeDatas.pop_back();
    }

    updatePaceData();
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief updatePreData wapper 함수
 * 
 */
void CKidnap::updateData()
{
    CStopWatch __debug_sw;
    
    updateData(1, ServiceData.localiz.getSlamPose(), ServiceData.localiz.getSysPose(),ServiceData.obstacle.getObstacleData()->imu);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CKidnap::updatePaceData()
{
    CStopWatch __debug_sw;

    updatePrePaceData();

    double timePace = get_system_time(curTimeDatas.back());
    //eblog(LOG_LV,  " -- timPace : " << timePace );

    tPose       slamPose    = ServiceData.localiz.getSlamPose();
    tPose       sysPose     = ServiceData.localiz.getSysPose(); 
    tSysIMU     imuData     = ServiceData.obstacle.getObstacleData()->imu; 
    
    paceData.slamX      =   fabs((slamPose.x - curSlamPoses.back().x)/timePace);
    paceData.slamY      =   fabs((slamPose.y - curSlamPoses.back().y)/timePace);
    paceData.slamAngle  =   fabs((slamPose.angle - curSlamPoses.back().angle)/timePace);
    paceData.sysX       =   fabs((sysPose.x - curSysPoses.back().x)/timePace);
    paceData.sysY       =   fabs((sysPose.y - curSysPoses.back().y)/timePace);
    paceData.sysAngle   =   fabs((sysPose.angle - curSysPoses.back().angle)/timePace);
    paceData.imuRoll    =   fabs((imuData.Groll - curImuDatas.back().Groll)/timePace);
    paceData.imuPitch   =   fabs((imuData.Gpitch - curImuDatas.back().Gpitch)/timePace);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CKidnap::updatePrePaceData()
{
    CStopWatch __debug_sw;

    prePaceData.slamX      =   paceData.slamX;
    prePaceData.slamY      =   paceData.slamY;
    prePaceData.slamAngle  =   paceData.slamAngle;
    prePaceData.sysX       =   paceData.sysX;
    prePaceData.sysY       =   paceData.sysY;
    prePaceData.sysAngle   =   paceData.sysAngle;
    prePaceData.imuRoll    =   paceData.imuRoll;
    prePaceData.imuPitch   =   paceData.imuPitch;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief kindnap proc 상태를 set!
 * jhnoh, 23.05.08
 * @param ret 
 */
void CKidnap::setKidnapState(E_KIDNAP_STATE ret)
{
    CStopWatch __debug_sw;

    info = ret;
    
    TIME_CHECK_END(__debug_sw.getTime());
}   


/**
 * @brief kidnap proc 상태를 get!
 * jhnoh, 23.05.08
 * @return E_KIDNAP_STATE 
 */
E_KIDNAP_STATE CKidnap::getKidnapState()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return info;
}

u32 procReadyLocationEndtick = (u32)-1;
u32 procLocationReadyEndtick = (u32)-1;

/**
 * @brief kidnap proc - 최초 실행 1번하는 단계!
 * jhnoh, 23.05.08
 * @return E_KIDNAP_STATE 
 */
E_KIDNAP_STATE CKidnap::procNone()
{
    CStopWatch __debug_sw;

    E_KIDNAP_STATE ret = E_KIDNAP_STATE::READY_LOCATION;

    procLocationReadyEndtick = SYSTEM_TOOL.getSystemTick();
 
    if( ret != E_KIDNAP_STATE::NONE)  bDebugStatePrint = true;    
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief kidnap proc - RMCL(자기위치 추정) 준비 단계!
 * jhnoh, 23.05.08
 * @return E_KIDNAP_STATE 
 */
E_KIDNAP_STATE CKidnap::procReadyLocation()
{
    CStopWatch __debug_sw;

    E_KIDNAP_STATE ret = E_KIDNAP_STATE::READY_LOCATION;
 
    if (!bSlamResume)
    {
        // location 준비
        pLocation->initLocation();

        // 카토그래퍼 start_trajectory
        eblog(LOG_LV_NECESSARY, "RELOCALIZE -> resumeSlam--------------------------------------------------- ");
        eblog(LOG_LV_NECESSARY,"\t" << "|SlamPose:" << SC<double>(ServiceData.localiz.getSlamPose().x) << ", " << SC<double>(ServiceData.localiz.getSlamPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
        eblog(LOG_LV_NECESSARY,"\t" << "|CevaPose:" << SC<double>(ServiceData.localiz.getSysPose().x) << ", " << SC<double>(ServiceData.localiz.getSysPose().y) << ", " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSysPose().angle)));
        eblog(LOG_LV_NECESSARY,  "RELOCALIZE ->resumeSlam-++++++++++++++++++++++++++++++++++++++++++++++++++++++");

        double x = ServiceData.localiz.getSlamPose().x;
        double y = ServiceData.localiz.getSlamPose().y;
        double yaw = ServiceData.localiz.getSlamPose().angle;
        eblog(LOG_LV_NECESSARY,"\t" << "|SlamPose:" << x << ", " << y << ", " << yaw);

        eblog(LOG_LV_NECESSARY,"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
        eblog(LOG_LV_NECESSARY,"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle);

        ROBOT_CONTROL.slam.resumeSlam(ServiceData.localiz.getSlamPose().x, ServiceData.localiz.getSlamPose().y, ServiceData.localiz.getSlamPose().angle);

        bSlamResume = true;
    }

    if( ret != E_KIDNAP_STATE::READY_LOCATION)  bDebugStatePrint = true;
    
    if (SYSTEM_TOOL.getSystemTick()-procLocationReadyEndtick >= 1*SEC_1) // trajecotry = 1 is start of the saft time
    {
        ret = E_KIDNAP_STATE::LOCATION;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief kidnap proc - RMCL(자기위치 추정) 단계!
 * jhnoh, 23.05.08
 * @return E_KIDNAP_STATE 
 */
E_KIDNAP_STATE CKidnap::procLocation()
{
    CStopWatch __debug_sw;
    CRobotKinematics k;

    E_KIDNAP_STATE ret = E_KIDNAP_STATE::LOCATION;
    tPose robotPose = ServiceData.localiz.getPose();
#if 0
    tLocationInfo info;
    info = pLocation->locationProc(ServiceData.localiz.getSysPose());

    if(info.state == E_LOCATION_STATE::END)
    {
        // 자기 위치 추정 성공
        setEstimatedPose(info.pose);
    }
    else if (info.state == E_LOCATION_STATE::FAIL_LOCATION)
    {
        // 자기 위치 추정 실패

    }
#else
    // 여기에 지도 온오프

    //use of the carotogrper scan mather <- rotate 
    switch (mStep)
    {
        case RELOCAL_STEP::RELOCAL_FIRST_STEP:
            {
                
                mStep = RELOCAL_STEP::RELOCAL_SECOND_STEP;
            }
            break;

        case RELOCAL_STEP::RELOCAL_SECOND_STEP:
            if (MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCAL_STEP::RELOCAL_THIRD_STEP;
            }
            break;

        case RELOCAL_STEP::RELOCAL_THIRD_STEP: // 한번 쉬자 -> update of the scan-matcher
            if (MOTION.isRunning() == false)
            {
                mStep = RELOCAL_STEP::RELOCAL_FORTH_STEP;
            }
            break;
        
        case RELOCAL_STEP::RELOCAL_FORTH_STEP:
            if (MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
                mStep = RELOCAL_STEP::RELOCAL_FIFTH_STEP;
            }
            break;

       case RELOCAL_STEP::RELOCAL_FIFTH_STEP:
            if (MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(90));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCAL_STEP::RELOCAL_SIXTH_STEP;
            }
            break;

       case RELOCAL_STEP::RELOCAL_SIXTH_STEP:
            if (MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(90));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
                mStep = RELOCAL_STEP::RELOCAL_SEVENTH_STEP;
            }
            break;

       case RELOCAL_STEP::RELOCAL_SEVENTH_STEP:
            if (MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(90));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                procLocationReadyEndtick = SYSTEM_TOOL.getSystemTick();
                mStep = RELOCAL_STEP::RELOCAL_END_STEP;
            }
            break;

        case RELOCAL_STEP::RELOCAL_END_STEP:
            if (MOTION.isRunning() == false)
            {
                //saft of the scan-matcher 
                if (SYSTEM_TOOL.getSystemTick()-procLocationReadyEndtick >= 2*SEC_1) // trajecotry = 1 is finish_trajectory of the saft time
                {
                    //TODO : 지도 상태 확인
                    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 자기위치 추정을 성공하여 이전 좌표로 복귀합니다.");
                    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 현재 좌표 : " << ServiceData.localiz.getPose().x << " , " <<  ServiceData.localiz.getPose().y << " , " <<  ServiceData.localiz.getPose().angle);
                    ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 이전 좌표 : " << lastPoseList.front().x << " , " <<  lastPoseList.front().y);

                    mStep = RELOCAL_STEP::RELOCAL_FIRST_STEP;
                    ret = E_KIDNAP_STATE::ANGLE_SETTING;
                    

                    // SUB_TASK.drivePath.goGoal(tPoint(lastPoseList.front().x, lastPoseList.front().y));
                    ceblog(LOG_LV_NECESSARY, BOLDRED, "SubTask Navi로 바꿔주세요!!!!!!!!!!!\n\n");
                }
            }
            break;

        default:
            break;
    }
#endif

    if( ret != E_KIDNAP_STATE::LOCATION)  bDebugStatePrint = true;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_KIDNAP_STATE CKidnap::procMoveLastPosition()
{
    CStopWatch __debug_sw;
    CRobotKinematics k;

    E_KIDNAP_STATE ret = E_KIDNAP_STATE::MOVE_LAST_POSITION;

    tPose robotPose = ServiceData.localiz.getPose();
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();    

    ceblog(LOG_LV_NECESSARY, BOLDRED, "SubTask Navi로 바꿔주세요!!!!!!!!!!!\n\n");
    // if (SUB_TASK.drivePath.isArrivalGoal())
    {
        double pendingAngle = utils::math::rad2deg(robotPose.angle);
        if ( pendingAngle > 180.0 )	pendingAngle = 360.0 - pendingAngle;
        
        // 방향을 결정한다. ( true : 반시계, false : 시계)
        if(utils::math::getTurnDirection(pendingAngle,0)) {
            double targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
            MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
        }
        else  {
            double targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
            MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
        }

        eblog(LOG_LV_NECESSARY, "procMoveLastPosition");

        ret = E_KIDNAP_STATE::END;
    }

    if( ret != E_KIDNAP_STATE::MOVE_LAST_POSITION)  bDebugStatePrint = true;

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_KIDNAP_STATE CKidnap::procAngleSetting(double degAngleGoal)
{
    CStopWatch __debug_sw;
    CRobotKinematics k;

    E_KIDNAP_STATE ret = E_KIDNAP_STATE::ANGLE_SETTING;
    tPose robotPose =ServiceData.localiz.getSlamPose();
    double pendingAngle = utils::math::rad2deg(robotPose.angle - degAngleGoal);
    if ( pendingAngle > 180.0 )	pendingAngle = 360.0 - pendingAngle;    
    
    // 방향을 결정한다. ( true : 반시계, false : 시계)
    if(utils::math::getTurnDirection(utils::math::rad2deg(robotPose.angle),degAngleGoal)) {        
        double targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
        MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
    }    
    else{
        double targetAng = k.rotation(robotPose, DEG2RAD(pendingAngle));
        MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
    }

    ceblog(LOG_LV_NECESSARY, BOLDBLUE,"로봇의 헤딩각도를 맞췄습니다.");
    ret = E_KIDNAP_STATE::END;

    if( ret != E_KIDNAP_STATE::ANGLE_SETTING)  bDebugStatePrint = true;
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief kidnap proc를 시작하기 위한 초기화!
 * jhnoh, 23.05.08
 */
void CKidnap::initKidnap()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV_PATHPLAN, "");   

    info = E_KIDNAP_STATE::NONE;
    
    TIME_CHECK_END(__debug_sw.getTime());
}



/**
 * @brief kidnap proc 상태 디버그 함수!
 * jhnoh, 22.12.13
 */
void CKidnap::__debug_kidnap_state_print()
{
    CStopWatch __debug_sw;

    if(bDebugStatePrint)
    {
        eblog(LOG_LV_PATHPLAN, "---- kidnap "<< enumToString(info) << " ---- ");
        bDebugStatePrint = false;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}


E_KIDNAPROBOT_STATE CKidnap::getKidnapRobotState()
{
    return kidnapInfo;
}
void CKidnap::setKidnapRobotState(E_KIDNAPROBOT_STATE set)
{
    kidnapInfo = set;
}


void CKidnap::kidnapRobotProc()
{

    switch (getKidnapRobotState())
    {
    case E_KIDNAPROBOT_STATE::ROTATING:
        setKidnapRobotState(procRotating());
        break;
    case E_KIDNAPROBOT_STATE::MOVE_LAST_POSE:
        setKidnapRobotState(procMoveLastPose());
        break;
    case E_KIDNAPROBOT_STATE::END:
        setKidnapRobotState(procEnd());
        break;
    default:
        break;
    }
}


E_KIDNAPROBOT_STATE CKidnap::procRotating()
{
    E_KIDNAPROBOT_STATE ret = E_KIDNAPROBOT_STATE::ROTATING;
    tPose robotPose = ServiceData.localiz.getPose();
    //use of the carotogrper scan mather <- rotate 
    tProfile pf;
    pf.desAngVel = RAD2DEG(30);
    switch (mStep)
    {
        case RELOCAL_STEP::RELOCAL_FIRST_STEP:
            
            mStep = RELOCAL_STEP::RELOCAL_SECOND_STEP;
            break;
        case RELOCAL_STEP::RELOCAL_SECOND_STEP:
            if (MOTION.isRunning() == false)
            {                
                MOTION.startRotation(robotPose, RAD2DEG(180), pf, E_ROTATE_DIR::CCW);
                mStep = RELOCAL_STEP::RELOCAL_THIRD_STEP;
            }
            break;
        case RELOCAL_STEP::RELOCAL_THIRD_STEP:
            if (MOTION.isRunning() == false)
            {
                MOTION.startRotation(robotPose, RAD2DEG(180), pf, E_ROTATE_DIR::CW);
                mStep = RELOCAL_STEP::RELOCAL_END_STEP;
            }
            break;
        case RELOCAL_STEP::RELOCAL_END_STEP:
            if (MOTION.isRunning() == false)
            {
                mStep = RELOCAL_STEP::RELOCAL_FIRST_STEP;
                ret = E_KIDNAPROBOT_STATE::MOVE_LAST_POSE;
            }
            break;
        default:
            break;
    }

    return ret;

}

E_KIDNAPROBOT_STATE CKidnap::procMoveLastPose()
{
    E_KIDNAPROBOT_STATE ret = E_KIDNAPROBOT_STATE::ROTATING;
    tPose lastPose = ServiceData.kidnapData.getLastRobotPose();
    tPose robotPose = ServiceData.localiz.getPose();

    ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 키드냅 이전 좌표로 이동 : " << lastPose.x << " , " << lastPose.y << " , " << lastPose.angle << " ");
    tPoint target;
    target.x = lastPose.x;
    target.y = lastPose.y;
    MOTION.startLinearToPointOnMap(robotPose, target, tProfile());
    ret = E_KIDNAPROBOT_STATE::END;
    return ret;
}
E_KIDNAPROBOT_STATE CKidnap::procEnd()
{
    E_KIDNAPROBOT_STATE ret = E_KIDNAPROBOT_STATE::END;
    if(MOTION.isRunning()== false)
    {
        ret = E_KIDNAPROBOT_STATE::ROTATING;
        ServiceData.kidnapData.setOccurDisError(false);
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 키드냅 종료 원래 자리 이동 완료 !!!!!");
    }

    return ret;

}

