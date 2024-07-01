#include "coreData/subject.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CSubject::CSubject() {}
CSubject::~CSubject() {}

// 옵저버들을 등록하는 메서드
void CSubject::attach(class CObserver* observer)
{
    observers.push_back(observer);
}

// 옵저버들을 삭제하는 메서드
void CSubject::detach(class CObserver* observer)
{
    auto it = std::find(observers.begin(), observers.end(), observer);
    if (it != observers.end())
    {
        observers.erase(it);
    }
}

/**
 * @brief 옵저버들을 모두 update() 시켜줌.
 * @param externData 
 * @param core 
 */
void CSubject::notify(CExternData* pExternData)
{
    for (auto& observer : observers)
    {
        observer->update(pExternData);
    }
}