/**
 * @file main.cpp
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#define BOOST_STACKTRACE_USE_BACKTRACE

#include "application.h"
#include "signal.h"
#include <iostream>
#include <boost/stacktrace.hpp>
#include <boost/version.hpp>
#include <execinfo.h>
#include "build_time.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

Application* pApp; // signal_handler에서 사용하기 위해 포인터로 관리. 23.03.17 jspark

std::string sigNum2str(int sig)
{
    switch (sig)
    {
    case SIGINT:        return "SIGINT ("+std::to_string(sig)+")";
    case SIGABRT:       return "SIGABRT ("+std::to_string(sig)+")";
    case SIGSEGV:       return "SIGSEGV ("+std::to_string(sig)+")";
    case SIGTERM:       return "SIGTERM ("+std::to_string(sig)+")";
    case SIGKILL:       return "SIGKILL ("+std::to_string(sig)+")";
    default:            return "SIG??? ("+std::to_string(sig)+")";
    }
}

void callTrace(int sig)
{
    std::cout << "====\t" << "signalHandler() : " << sigNum2str(sig) << "\t=========" << std::endl;
    std::cout << "\033[1m\033[33m"<< boost::stacktrace::stacktrace() << "\e[0m"<< std::endl;
    std::cout << "=================================================" << std::endl;
}

static void signalHandler(int sig)
{
    switch (sig)
    {
    case SIGINT:
    case SIGABRT:
    case SIGSEGV:
    case SIGTERM:
    case SIGKILL:
    default:
        callTrace(sig);
        break;
    }

    if ( pApp != nullptr )
    {
        std::cout << "\033[1m\033[33m" << " pApp is not nullptr !!!!" << std::endl;
        std::cout << "\033[1m\033[33m" << " delete pApp - st" << std::endl;
        delete pApp;
        std::cout << "\033[1m\033[33m" << " delete pApp - ed" << std::endl;
        pApp = nullptr;
    }
    else
    {
        std::cout << "\033[1m\033[33m" << " pApp is nullptr !!!!" << std::endl;
    }
}

int main(int argc, char **argv)
{
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "setCoutToLog call" << std::endl;
    std::ofstream filestr;
    filestr.open("/home/ebot/log/log.txt");
    std::streambuf *psbuf = filestr.rdbuf();  // 파일 스트림 버퍼를 얻는다
    std::cout.rdbuf(psbuf);  // 파일 스트림 버퍼를 cout 에 연관시킨다
    std::cout << "setCoutToLog call end"<< std::endl;
    std::cout << "Build Time: " << BUILD_TIME  << std::endl;

    ros::init(argc, argv, "main", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    signal(SIGINT,  signalHandler);
    signal(SIGKILL, signalHandler);
    signal(SIGTERM, signalHandler);
    // signal(SIGABRT, signalHandler);
    signal(SIGSEGV, signalHandler);

    pApp = new Application(nh);

    //int reslut = pApp->executeLidar();

    pApp->init();
    pApp->execute();

    if ( pApp != nullptr )  delete pApp;
    
    return 0;
}
