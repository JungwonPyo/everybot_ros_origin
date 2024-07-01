#include "application.h"
#include "fileMng.h"


/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

// DEFINE ROS
#define ROS_DEBUG_PRINT 0                   // ROS용 PRINTF 확인 1, 미확인 0

/**
 * @brief 생성자
 * 
 * @param _nh ros nodehandle
 */
Application::Application(ros::NodeHandle _nh) {
    state = APPLICATION_STATE::START_BOOT; 
    FILE_MNG.init();
    FILE_MNG.__debug_print_config(true);
    pRsaDispatcher = new CRsaDispatcher(_nh); }

/** @brief 사용하지 마시오. */
Application::Application(const Application &ref)
{
    ros::NodeHandle nh;
    pRsaDispatcher = new CRsaDispatcher(nh);
}

/** @brief 사용하지 마시오. */
Application &Application::operator=(const Application &ref) { if (this != &ref) { pRsaDispatcher = ref.pRsaDispatcher; } return *this; }

/** @brief 소멸자 */
Application::~Application()
{
    ceblog(LOG_LV_NECESSARY, GREEN, "");
    
    if ( pRsaDispatcher != nullptr )
    {
        delete pRsaDispatcher;
        pRsaDispatcher = nullptr;
        eblog(LOG_LV , "delete complete!");
    }
}

/**
 * @brief execute() 하기 전에 초기화 함수.
 * 
 */
void Application::init()
{
    //application 초기화.
    bool check = true;
    while (check)
    {
        pRsaDispatcher->updateMonitor();
        if(state == APPLICATION_STATE::START_BOOT){
            if(pRsaDispatcher->init()) state = APPLICATION_STATE::RUN_BOOT;
        }else if(state == APPLICATION_STATE::RUN_BOOT){
            if (pRsaDispatcher->loadService()) state = APPLICATION_STATE::RUN_DISPATCHER;
        }else{
            check = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

/**
 * @brief Application 메인 Loop 함수.
 */
void Application::execute()
{
    const int hz = 100;//50  // 100ms 1000/50 = 20ms -> 1000ms/100 = 10ms

    while (true)
    {
        if ( isRuntimeElapsed(hz) == true)
        {
            /* dispatcher execute */
            try
            {
                pRsaDispatcher->rsaDispatcherExecute();
            }
            catch(const std::exception& e)
            {
                std::cout<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
                std::cerr << e.what() << '\n';
            }
        }   
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

/**
 * @brief 디스패처를 실행시킬지 대기할지 계산하는 함수.
 * 기존 실행->딜레이->실행->딜레이가 누적오차를 발생시키는 구조이므로,
 * 시간 계산을 통해 누적오차를 방지함.
 * @param runtimeHz 디스패처를 실행시킬 주기.
 * @return true 디스패처를 실행시킴.
 * @return false 디스패처를 실행시키지 않음.
 */
bool Application::isRuntimeElapsed(int runtimeHz)
{
    static double startTime = 0.0;

    double targetTime = 1.0/runtimeHz; // 체크할 목표 시간 (단위 s)
    double elapedTime = get_system_time(startTime); // 디스패처 실행후 경과된 시간 (단위 s)
    
    if ( elapedTime >= targetTime)
    {
        // ceblog(LOG_LV, BOLDGREEN, "Dispatcher Cycle(ms) is " << elapedTime*1000.0 << " / " << targetTime*1000.0 << std::endl) ;
        startTime += targetTime; // 경과된 시간이 목표에 도달하면 다음 실행계산을 위해 runtime 시간만큼 빼줌.
        
        // 만약 runTimeHz 을 크게 만족하지 못하는 경우 경고문구 출력
        if ( elapedTime/targetTime > 2) // 주기에 100%
        {
            startTime = get_system_time(); // 경과된 시간이 runtimeHz 를 만족하지 못하는 경우 starMs 를 현재시간으로 초기화 시킴.
            // ceblog(LOG_LV_ERROR, YELLOW, "Warning!! Dispatcher Cycle(ms) is over " << elapedTime*1000.0 << " ms > " << targetTime*1000.0 << " (target x2)");
        }
        return true;
    }

    return false;
}

int Application::executeLidar(void)
{
    std::string command = "roslaunch ts800_ros 3i_lidar.launch &";
    FILE* pipe = popen(command.c_str(), "r");

    if (pipe == nullptr)
    {
        std::cerr << "Failed to launch roslaunch command." << std::endl;
        return -1;
    }

    pclose(pipe);

    return 0;
}