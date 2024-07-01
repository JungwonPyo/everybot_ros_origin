#include "control/lidarDriverControl.h"

#include "utils.h"
#include "eblog.h"
#include "commonStruct.h"
#include "systemTool.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CLidarDriverControl::CLidarDriverControl() :  m_fd_driver(-1) {
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());   
}

CLidarDriverControl::~CLidarDriverControl()
{
    CStopWatch __debug_sw;
    
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

int CLidarDriverControl::openLidarDriver(void)
{
    m_fd_driver = open(TS800_LIDAR_DEVICE, O_RDWR);
    if (m_fd_driver < 0)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "Opened fail /dev/ts800_lidar device return: "<<  m_fd_driver);
    }

    return m_fd_driver;
}

void CLidarDriverControl::closeLidarDriver(void)
{
    ceblog(LOG_LV_NECESSARY, BOLDBLUE, "* Close Drive");

    int result = close(m_fd_driver);
    if (result == -1) {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "ERROR: could not close ");
    }
}

int CLidarDriverControl::getLidarDriver(void)
{
    return m_fd_driver;
}

void CLidarDriverControl::setEnableLidarMotor(bool set)
{
    int nResult;

    if (m_fd_driver < 0)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "Opened fail /dev/ts800_lidar device return: " <<  m_fd_driver);
        return;
    }

    //lidar of the motor on&off
    if (set)
    {
        nResult = ioctl(m_fd_driver, SET_LIDAR_GO, NULL);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "SET_LIDAR_GO ");
    }
    else
    {
        nResult = ioctl(m_fd_driver, SET_LIDAR_STOP, NULL);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "SET_LIDAR_STOP ");
    }
}
