/**
 * @file findCharger.cpp
 * @author hhryu (hhryu@everybot.net)
 * @brief
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "findCharger.h"
#include "eblog.h"
#include "systemTool.h"
#include "control/motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "kinematics.h"

#define SG ServiceData.signal

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0           // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)                                     \
    {                                                            \
        printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time); \
    }                                                            \
    0
/******************************************************/

CFindCharger::CFindCharger() : CAvoiding()
{
    CStopWatch __debug_sw;
    
    eblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), "");
    init();
    TIME_CHECK_END(__debug_sw.getTime());
}

CFindCharger::~CFindCharger()
{
    CStopWatch __debug_sw;
    eblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), "");
    delete[] findChargerData_.checkedRegionPoint;
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CFindCharger::findChargerEnd()
{
    bool bRet = false;
    // if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_LONG) || SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT))
    if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_ANYTHING))
    {
        SG.debugSignalPrint("Find Charger 중 신호 발견해서 Signal track START");
        bRet = true;
    }
    return bRet;
}

E_FINDCHARGER_STEP CFindCharger::runFindChargerProcedure(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle, tPoint chargerPoint)
{
    /*맵이 있으면? 영역 분할 후 search할 좌표를 찍는다. MOVE POINT로 감*/
    /*맵이 없으면? 탐색 + 이미지 처리 후 search할 좌표를 찍는다. 이것도 MOVE POINT인지..? */

    if (findChargerEnd())
    {
        setFindChargerStep(E_FINDCHARGER_STEP::COMPLELT);
    }

    switch (getFindChargerStep())
    {
    case E_FINDCHARGER_STEP::NONE:
        setFindChargerStep(E_FINDCHARGER_STEP::INIT);
        ceblog(LOG_LV_ERROR, RED, "NONE ERROR");
        break;
    case E_FINDCHARGER_STEP::INIT:
        setFindChargerStep(initProcedure());
        break;
    case E_FINDCHARGER_STEP::WITH_MAP:
        setFindChargerStep(withMapProcedure(chargerPoint, robotPose, pObstacle));
        break;
    case E_FINDCHARGER_STEP::WITHOUT_MAP:
        setFindChargerStep(withoutMapProcedure(robotPose, pObstacle));
        break;
    case E_FINDCHARGER_STEP::COMPLELT:
        ceblog(LOG_LV_DOCKING, BLUE, "COMPLELT");
        break;
    case E_FINDCHARGER_STEP::FAIL:
        ceblog(LOG_LV_DOCKING, BOLDMAGENTA, "FAIL");
        break;
    case E_FINDCHARGER_STEP::MOVE_POINT:
    default:
        break;
    }

    return getFindChargerStep();
}

void CFindCharger::setFindChargerStep(E_FINDCHARGER_STEP set)
{
    findChargerData_.findChargerStep = set;
}

E_FINDCHARGER_STEP CFindCharger::getFindChargerStep(void)
{
    return findChargerData_.findChargerStep;
}

void CFindCharger::startFindCharger(tSignalData singalData)
{
}

void CFindCharger::stopFindCharger(void)
{
}

/**
 * @brief 맵이 있을 때 충전기의 좌표로 이동.
 *
 * @param chargerPoint
 * @param robotPose
 * @return E_FINDCHARGER_STEP
 *
 * @note 연산시간 ms
 * @date 2023-11-30
 * @author hhryu
 */
E_FINDCHARGER_STEP CFindCharger::withMapProcedure(tPoint chargerPoint, tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    CStopWatch __debug_sw;
    E_FINDCHARGER_STEP ret = E_FINDCHARGER_STEP::WITH_MAP;
    tPoint targetPoint;
#if 0
    if (getCheckedRegion(chargerPoint)) //(알고 있는 충전기의 좌표를 간 적이 있나? - 범위는 define으로 바꿀수 있도록 하자.)
    {
        targetPoint = nextCheckPoint(robotPose); // 있다. --> 가야할 좌표를 바꿔주는 함수!!!
        ceblog(LOG_LV_DOCKING, BLUE, "가야할 좌표 변경");
    }
    else
    {
        targetPoint = chargerPoint; // 충전기 위치로 추정되는 좌표를 체크해 본적이 없으니까 타겟으로 설정.
        ceblog(LOG_LV_DOCKING, BLUE, "충전기로 추정되는 위치로 이동합니다.");
    }
#endif
    // if (movePointHandler(robotPose,chargerPoint /*targetPoint*/, pObstacle)) // 다른데 체크하러 감.
    if (checkSignal(robotPose)) // 체크, true면 END.
    {
        if(nextPoint())
        {
            ret = E_FINDCHARGER_STEP::MOVE_POINT;
        }
        else if (false /*좌표찾기 == 중*/)
        {
            ret = E_FINDCHARGER_STEP::WITH_MAP;
        }
        else // 좌표 찾기 없음
        {
            if (runExplorerCharger(robotPose))
            {
                ret = E_FINDCHARGER_STEP::COMPLELT;
            }
            // ret = E_FINDCHARGER_STEP::FAIL;
        }
        // ceblog(LOG_LV_DOCKING, BLUE, "체크 시그널 실패....");
        // ret = E_FINDCHARGER_STEP::FAIL;
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 임시코드. explorer를 돌린다.
 * 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2023-MM-DD
 * @author hhryu
 */
bool CFindCharger::runExplorerCharger(tPose robotPose)
{
    bool bRet = false;

    if (SG.countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_ANYTHING))
    {
        bRet = true;
    }
    else
    {
        
    }

    return bRet;
}

bool CFindCharger::nextPoint()
{
    bool bRet = false;

    // if (getNextPoint().x > 2.9 && getNextPoint().x > 3.1)
    // {
    //     pExplorer->initExplorer();
    //     bRet = false;
    // }
    // else
    // {
    //     setNextPoint(tPoint(3, 2));
    //     bRet = true;
    // }

    bRet = true;
    setNextPoint(tPoint(0, 0));

    return bRet;
}

void CFindCharger::setNextPoint(tPoint set)
{
    findChargerData_.nextPoint = set;
}

tPoint CFindCharger::getNextPoint()
{
    return findChargerData_.nextPoint;
}


E_FINDCHARGER_STEP CFindCharger::withoutMapProcedure(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle)
{
    E_FINDCHARGER_STEP ret = E_FINDCHARGER_STEP::WITHOUT_MAP;
#if 0
    if (!isMovingPoint()) // 움직이지 않는 중.
    {
        if (getCheckedRegion(tPoint(robotPose.x, robotPose.y))) // 현재 위치를 체크한 적이 있나??
        {
            ceblog(LOG_LV_DOCKING, BLUE, "(0,0) 지점으로 이동 중.");
            movePointHandler(robotPose, tPoint(0, 0), pObstacle); // 다른데 체크하러 감.
        }
        else
        {                               // 안움직이면서 현재 지점 체크도 안했음.
            if (checkSignal(robotPose)) // 체크, true면 END.
            {
                ceblog(LOG_LV_DOCKING, BLUE, "체크 시그널.");
                // ret = E_FINDCHARGER_STEP::FAIL;
            }
        }
    }
#endif

    // if (movePointHandler(robotPose, tPoint(0,0), pObstacle)) // 다른데 체크하러 감.
    {
        if (checkSignal(robotPose))
        {
            ret = E_FINDCHARGER_STEP::FAIL;
        }
    }

    return ret;
}

void CFindCharger::init()
{
    setFindChargerStep(E_FINDCHARGER_STEP::INIT);

    findChargerData_.bMovePoint = false;
    findChargerData_.checkSigStep = E_CHECK_SIG_STEP::IDLE;
    findChargerData_.checkedRegionPoint = new tPose[INITIAL_CAPACITY];
    findChargerData_.checkedRegionCount = 0;
    findChargerData_.checkedRegionCapacity = INITIAL_CAPACITY;
    findChargerData_.bRequestPath = false;
    findChargerData_.bChecking = false;
    findChargerData_.nextPoint = tPoint(0,0);

    return;
}

E_FINDCHARGER_STEP CFindCharger::initProcedure()
{
    E_FINDCHARGER_STEP ret = E_FINDCHARGER_STEP::INIT;

    init();

    if (true) // 맵이 있는가?
    {
        ret = E_FINDCHARGER_STEP::WITH_MAP; // 탐색 도중에 원점 복귀를 하면 어떻게 되지????
    }
    else
    {
        ret = E_FINDCHARGER_STEP::WITHOUT_MAP;
    }

    return ret;
}

/**
 * @brief   회전하며 시그널을 체크한다.
 *          감지 시 트래킹.
 *
 * @return true
 * @return false
 *
 * @note 연산시간 ms
 * @date 2023-10-30
 * @author hhryu
 */
bool CFindCharger::checkSignal(tPose robotPose)
{
    CRobotKinematics k;
    bool bRet = false;

    findChargerData_.bChecking = true;

    switch (findChargerData_.checkSigStep)
    {
    case E_CHECK_SIG_STEP::IDLE:
        findChargerData_.checkSigStep = E_CHECK_SIG_STEP::START;
        ceblog((LOG_LV_DOCKING | LOG_LV_NECESSARY), BLUE, "check signal start");
        break;
    case E_CHECK_SIG_STEP::START:
        targetAng = k.rotation(robotPose, DEG2RAD(358));
        MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
        findChargerData_.checkSigStep = E_CHECK_SIG_STEP::FULL_TURN;
        break;
    case E_CHECK_SIG_STEP::FULL_TURN:
        if (!MOTION.isRunning())
        {
            findChargerData_.checkSigStep = E_CHECK_SIG_STEP::END;
        }
        break;
    case E_CHECK_SIG_STEP::END:
        if(MOTION.getControlType() != E_CONTROL_TYPE::STOP)
        {
            
        }
        else
        {
            setCheckedRegion(robotPose);
            findChargerData_.checkSigStep = E_CHECK_SIG_STEP::IDLE;
            bRet = true;
        }
        break;
    }

    return bRet;
}

void CFindCharger::setMovingPoint(bool bSet)
{
    findChargerData_.bMovePoint = bSet;
}

bool CFindCharger::isMovingPoint()
{
    return findChargerData_.bMovePoint;
}

/**
 * @brief check하고 싶은 점을 입력하면, 이미 신호 탐색 완료한 점으로부터 몇 미터 이내인지 아닌지 반환.
 *
 * @param checkPose
 * @return true
 * @return false
 *
 * @note 연산시간 ms
 * @date 2023-11-30
 * @author hhryu
 */
bool CFindCharger::getCheckedRegion(tPoint checkPoint)
{
    CStopWatch __debug_sw;
    bool bRet = false;
    static const double radius = 2.0;
    for (s32 i = 0; i < findChargerData_.checkedRegionCount; ++i)
    {
        // 입력된 위치가 체크 완료 영역(점으로 부터 radius 자취 이내) 내에 있는지를 판단
        double distance = utils::math::distanceTwoPoint(checkPoint, findChargerData_.checkedRegionPoint[i]);
        if (distance <= radius) // radius를 반지름으로 하는 원 범위 내에 속하는가?
        {
            bRet = true;
        }
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

void CFindCharger::setCheckedRegion(tPose robotPose)
{
    // 배열 용량이 부족한 경우, 두 배로 확장
    if (findChargerData_.checkedRegionCount == findChargerData_.checkedRegionCapacity)
    {
        findChargerData_.checkedRegionCapacity *= 2;
        tPose *newCenterArray = new tPose[findChargerData_.checkedRegionCapacity];
        std::copy(findChargerData_.checkedRegionPoint, findChargerData_.checkedRegionPoint + findChargerData_.checkedRegionCount, newCenterArray);
        delete[] findChargerData_.checkedRegionPoint;
        findChargerData_.checkedRegionPoint = newCenterArray;
    }

    // 새로운 중심 좌표 추가
    findChargerData_.checkedRegionPoint[findChargerData_.checkedRegionCount++] = robotPose;
}

s32 testPoint = 0;
tPoint CFindCharger::nextCheckPoint(tPose robotPose)
{
    // 가장 가까운 곳에 있는 getCheckedRegion의 점을 검색하고 그 근처의 2.5m지점을 확인하는...식은 별로겠다. 아이디어 필요

    tPoint point = tPoint(robotPose.x, robotPose.y);
    if (++testPoint % 2 == 1)
    {
        point.x = robotPose.x + 2.5;
    }
    else
    {
        point.y = robotPose.y + 2.5;
    }
    return point;
}