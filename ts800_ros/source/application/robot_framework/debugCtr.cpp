#include "debugCtr.h"
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/


CDebugCtr::CDebugCtr() {
}
CDebugCtr::~CDebugCtr() {}

CDebugCtr& CDebugCtr::getInstance()
{
    static CDebugCtr s;
    return s;
}

void CDebugCtr::d_setSimplifyMap(cv::Mat set)
{
    dSimplifyMap = set;
}

cv::Mat CDebugCtr::d_getSimplifyMap()
{
    return dSimplifyMap.clone();
}


void CDebugCtr::d_makeArea()
{
    CStopWatch __debug_sw;
    
    unsigned char *pMap = nullptr;
    int makeId = 0;  
    CImgProcessor imgProc;

    
    if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
    {
        ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
    
        tGridmapInfo info = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
        pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();

        cv::Mat img = imgProc.convertSimplifyGridMap2Mat(pMap, ServiceData.robotMap.robotTrajectory.getCleanedTrajectory(), 
        info.width, info.height, info.resolution, info.origin_x, info.origin_y);

        ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
        d_setSimplifyMap(img);    
    
        //이미지 전체를 검색하여 방들을 구분. 
        std::list<cv::Rect> cvRects = imgProc.fakeBsp(img, 10);

        eblog(LOG_LV_NECESSARY, "cvRects.size : "<< cvRects.size() );

        for ( cv::Rect& rect : cvRects )
        {
            cv::rectangle(img, rect, cv::Scalar(80), 2);            
        }

        d_setSimplifyMap(img);
    }

    
    TIME_CHECK_END(__debug_sw.getTime());
}