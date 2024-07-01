#include "rsaDispatcher.h"
#include "userInterface.h"
#include "control/motionPlanner/motionPlanner.h"
#include "define.h"
#include "utils.h"
#include "debugCtr.h"
#include "rosPublisher.h"
#include "subTask.h"
#include "kinematics.h"
#include "userInterface.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/** @brief 사용하지 마시오. */
CRsaDispatcher::CRsaDispatcher(const CRsaDispatcher &ref)
{
    ros::NodeHandle nh;
    pRsfMonitor = new CRsfMonitor(nh);
    pServiceMng = new CServiceManager();
}

/** @brief 사용하지 마시오. */
CRsaDispatcher &CRsaDispatcher::operator=(const CRsaDispatcher &ref)
{
    if (this != &ref)
    {
        pRsfMonitor = ref.pRsfMonitor;
        pServiceMng = ref.pServiceMng;
    }
    return *this;
}

CRsaDispatcher::CRsaDispatcher(ros::NodeHandle _nh)
{
    CStopWatch __debug_sw;
	
    //odom&lidar of the time lantency	
    ROS.init(&_nh);
    DEBUG_PUB.init(&_nh);
    pRsfMonitor = new CRsfMonitor(_nh);
    singleton::CServiceData::getInstance().init( pRsfMonitor->getServiceDataPointer() );
    //MOTION.init();
    SUB_TASK.init();
    pServiceMng = new CServiceManager();
    ROBOT_CONTROL.slam.setRosNodeRef(_nh);

    eblog(LOG_LV, "");
    TIME_CHECK_END(__debug_sw.getTime());
}

CRsaDispatcher::~CRsaDispatcher()
{ 
    ceblog(LOG_LV, GREEN, "");

    delete pRsfMonitor;
    delete pServiceMng;
}

bool CRsaDispatcher::init()
{
    CStopWatch __debug_sw;
    bool bRet = false;

    taskBoot.taskStart();
    bRet = true;
    

    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

bool CRsaDispatcher::loadService()
{
    CStopWatch __debug_sw;
    bool bRet = false;
    if(taskBoot.taskRun()) bRet = true;
    USER_INTERFACE.updateUi();
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

// 이 함수 위치에대한 고민이 필요.
void CRsaDispatcher::callServiceHandler()
{
    CStopWatch __debug_sw;

    double time = 0.0;
    double time_start = get_system_time();//SYSTEM_TOOL.getSystemTime();
#if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 1)
    //종종 imu 데이터가 올라오지 않는 경우 발생
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    tPose robotPose = ServiceData.localiz.getPose(); //로봇 제어 : 슬램
    tPose slamPose = ServiceData.localiz.getSlamPose(); //슬램 좌표
    tPose cevaPose = ServiceData.localiz.getSysPose(); //세바 좌표
    double batVolt = ServiceData.battery.getBatteryVolt(); //밧데리 레벨만 표시
    taskDemoRBTPlus.taskDemoRBTPlusRun(robotPose, slamPose, cevaPose, batVolt, pObstacle);
#else
	pServiceMng->updateServiceManager( pRsfMonitor->getErrorInfo() );
#endif

    time = /*SYSTEM_TOOL.getSystemTime()*/get_system_time() - time_start;        
    // if (time > 0.001) eblog(LOG_LV_ERROR, "time : "<<time);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CRsaDispatcher::updateMonitor()
{
    CStopWatch __debug_sw;
    double time = 0.0;
    double time_start = get_system_time();//SYSTEM_TOOL.getSystemTime();
    
    if ( pRsfMonitor != NULL ) 
    {
        pRsfMonitor->update();
    }
    else 
    {
        eblog(LOG_LV, "is null updateMonitor");
    }
    
    time = get_system_time() - time_start;      
    // if (time > 0.01) eblog(LOG_LV_ERROR,  "time : "<<time);

    TIME_CHECK_END(__debug_sw.getTime());
}

u32 UiTick = (u32)-1;

void CRsaDispatcher::rsaDispatcherExecute()
{
    CStopWatch __debug_sw;
    updateMonitor();
	callServiceHandler();
    controlHandler();
    USER_INTERFACE.updateUi();

#if 1   //dispatcher 주기 체크 디버깅.
    debugDispacher();
#endif

    TIME_CHECK_END(__debug_sw.getTime());
}

void CRsaDispatcher::errorChecker()
{
    if(pRsfMonitor->isErrorDetected()){

    }
}

static double lastDispatchTick = 0.0;
void CRsaDispatcher::debugDispacher()
{
    double currentTick = SYSTEM_TOOL.getSystemTime();
    double diffTick = (currentTick - lastDispatchTick) * 1000;

    if (diffTick > 20.0){
        std::cout << "Execution cycle: " << diffTick << " ms" << std::endl;
    }

    lastDispatchTick = currentTick;
}

void CRsaDispatcher::controlHandler(){
    if(ROBOT_CONTROL.isTiltRunning()){
        ROBOT_CONTROL.procControlTilt();
    }
    if(MOTION.isRunning()){
        MOTION.proc();
    } 
}

