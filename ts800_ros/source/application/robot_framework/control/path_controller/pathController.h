#include "coordinate.h"

class CPathController
{
public:
    enum class E_PATH_CTR_STATE
    {
        DOING, // 진행중
        COMPLETE, // 완료
        FAIL, // 실패
    };

    CPathController();
    ~CPathController();
    E_PATH_CTR_STATE getState();

protected:
    void setState(E_PATH_CTR_STATE state);
    tTwist controlVelocity; // 계산된 제어속도

private:
    E_PATH_CTR_STATE state;
};