
#include "coreData/serviceData.h"
#include "eblog.h"
#include "serviceData.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CServiceData::CServiceData(ros::NodeHandle _nh) // : rosPublishData(_nh)
{
    eblog(LOG_LV, "");

    __testCheckDstar = false; // 디버그용 플레그
    tempLidarSpeed = 5.8;
    tempDutyCycle = 19000;
    motionInfo.curVel.v = 0;
    motionInfo.curVel.w = 0;
}

CServiceData::~CServiceData()
{
    eblog(LOG_LV, "");
}

namespace singleton
{
    CServiceData::CServiceData() {}
    
    CServiceData::CServiceData(const CServiceData &ref) {}
    
    CServiceData &CServiceData::operator=(const CServiceData &ref)
    {
        return *this;
    }
    
    CServiceData::~CServiceData() {}

    CServiceData &CServiceData::getInstance()
    {
        static CServiceData s;
        return s;
    }

    void CServiceData::init(::CServiceData *pServiceData)
    {
        this->pServiceData = pServiceData;
    }
    
    ::CServiceData& CServiceData::getServiceData()
    {
        return *(this->pServiceData);
    }
};
