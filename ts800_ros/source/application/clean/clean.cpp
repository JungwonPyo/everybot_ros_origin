#include "clean.h"
#include "eblog.h"
#include "robotmap.h"
#include "obstaclemap.h"
#include "utils.h"
#include <memory>
#include "motionPid.h"
#include "userInterface.h"
#include "motionPlanner/motionPlanner.h"
#include "rosPublisher.h"
#include  "subTask.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0

CClean::CClean()
{
    CStopWatch __debug_sw;
    pObsMap = &(ServiceData.obstaclemap);
    pCleanPatterns = new CCleanPatterns();
    eblog(LOG_LV,  "new CLineClean");
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CClean::~CClean()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "");

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  청소 시작 시 관련 정보 초기화 함수
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
void CClean::initCleanning(void)
{
    CStopWatch __debug_sw;
    
    clearCleanPattern();
    if(cleanData_.mode == E_CLEAN_MODE::FAST)
    {
        addCleanPattern(E_CLEAN_PATTERN::LINE);
    }
    else if(cleanData_.mode == E_CLEAN_MODE::WALL)
    {
        addCleanPattern(E_CLEAN_PATTERN::WALL);
    }
    else //if(cleanData_.mode == E_CLEAN_MODE::AUTO)
    {
        addCleanPattern(E_CLEAN_PATTERN::WALL);
        addCleanPattern(E_CLEAN_PATTERN::LINE);
    }
    
    pCleanPatterns->setPatternState(E_PATTERN_STATE::INIT);
    setCleanStep(E_CLEAN_STEP::INIT);
    // cleanData_.debug_oldTick = SYSTEM_TOOL.getSystemTick();
    // cleanData_.lineCleanStartTime = get_system_time();
    eblog(LOG_LV_NECESSARY,  "initLineclean!!!");
    
    TIME_CHECK_END(__debug_sw.getTime());
}


void CClean::setCleanMode(int set)
{
    E_CLEAN_MODE temp = cleanData_.mode;
    cleanData_.mode = (E_CLEAN_MODE)set;
    
    if(cleanData_.mode == E_CLEAN_MODE::FAST)
    {        
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_50MIN_CLEAN_START);
    }
    else if(cleanData_.mode == E_CLEAN_MODE::WALL)
    {
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_WALL_CLEAN_START);
    }
    else//(cleanData_.mode == E_CLEAN_MODE::AUTO)
    {
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_AUTO_CLEAN_START);
    }
    LCD_TEXT_CTR.debugCleanMode(set); //LCD DEBUG
    LED_CTR.playCleanServiceStart();
    ceblog(LOG_LV_NECESSARY, CYN, "setCleanMode : "<< enumToString(temp)<<" --> "<<enumToString(cleanData_.mode));
}
/**
 * @brief  청소 상태를 set하는 함수
 * E_LINECLEAN_STATE에 따라 관련 처리 함수를 실행한다.
 * @param PLAN_AREA : 저장된 지도를 읽어 청소를 위한 영역계획를 실행한다.
 * @param MOVE_AREA : 설정된 영역으로 이동한다.
 * @param CLEAN_AREA : 영역청소를 실행한다.
 * @param CLEAN_END : 청소 완료 여부를 판단하고, 다음 청소할 영역을 설정한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
void CClean::setCleanStep(E_CLEAN_STEP set)
{
    CStopWatch __debug_sw;

    if (set != cleanData_.cleanStep)
    {
        ceblog(LOG_LV_NECESSARY, CYN, "[state] line clean: "<<enumToString(cleanData_.cleanStep)<<" --> "<<enumToString(set));
    }

    cleanData_.cleanStep = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  청소 상태를 get하는 함수
 * E_LINECLEAN_STATE에 따라 관련 처리 함수를 실행한다.
 * @param PLAN_AREA : 저장된 지도를 읽어 청소를 위한 영역계획를 실행한다.
 * @param MOVE_AREA : 설정된 영역으로 이동한다.
 * @param CLEAN_AREA : 영역청소를 실행한다.
 * @param CLEAN_END : 청소 완료 여부를 판단하고, 다음 청소할 영역을 설정한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
E_CLEAN_STEP CClean::getCleanStep(void)
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return cleanData_.cleanStep;
}


void CClean::addCleanPattern(E_CLEAN_PATTERN set)
{
    cleanData_.patterns.emplace_back(set);
    ceblog(LOG_LV_NECESSARY, BLUE, "SET CLEAN PATTERN :" << enumToString(set) << " < " << cleanData_.patterns.size() << " > 번째 청소 패턴" );
}

void CClean::clearCleanPattern()
{
    cleanData_.patterns.clear();
    ceblog(LOG_LV_NECESSARY, BLUE, "CLEAR CLEAN PATTERN~~~ ");
}

E_CLEAN_PATTERN CClean::getCurrentPattern()
{
    E_CLEAN_PATTERN ret;
    if(cleanData_.patterns.size()!=0)
    {
        ret= cleanData_.patterns.front();
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "  에러 발생.... 남은 패턴이 없어요");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
    }
    return ret;
}

int CClean::getCurrentPatternSize()
{
    return cleanData_.patterns.size();
}

void CClean::popCurrentPattern()
{
    ceblog(LOG_LV_NECESSARY, YELLOW, "현재 패턴 완료 : " << enumToString(cleanData_.patterns.front()));
    cleanData_.patterns.pop_front();
    if(cleanData_.patterns.size()!=0)
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, " 다음 패턴 시작 : " << enumToString(cleanData_.patterns.front()));
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
        ceblog(LOG_LV_NECESSARY, YELLOW, "  에러 발생.... 남은 패턴이 없어요");
        ceblog(LOG_LV_NECESSARY, YELLOW, "********************************");
    }
}


/**
 * @brief  청소 알고리즘 RUN함수.
 * E_LINECLEAN_STATE에 따라 관련 처리 함수를 실행한다.
 * @param PLAN_AREA : 저장된 지도를 읽어 청소를 위한 영역계획를 실행한다.
 * @param MOVE_AREA : 설정된 영역으로 이동한다.
 * @param CLEAN_AREA : 영역청소를 실행한다.
 * @param CLEAN_END : 청소 완료 여부를 판단하고, 다음 청소할 영역을 설정한다.
 * @return void
 * @note 연산시간 
 * @date 2023-08-23
 * @author hjkim  
 */ 
bool CClean::runCleanning(tPose robotPose)
{
    CStopWatch __debug_sw;

    bool ret = false;
    //ServiceData.robotMap.updateCleanMap();
    //ServiceData.robotMap.updateCleaned(robotPose);   //line clean 중 로봇이 움직일때마다 맵 정보를 갱신 한다.

    switch (getCleanStep())
    {
    case E_CLEAN_STEP::INIT:
        setCleanStep(procInit(robotPose));
        break;
    case E_CLEAN_STEP::PLAN_AREA:
        setCleanStep(procPlanArea(robotPose));
        break;
    case E_CLEAN_STEP::CHECK_MOVE:
        setCleanStep(procCheckMove());
        break;    
    case E_CLEAN_STEP::MOVE_AREA:
        setCleanStep(procMoveArea(robotPose));
        break;
    case E_CLEAN_STEP::RE_PLAN_AREA:
        setCleanStep(procRePlanArea(robotPose));
        break;
    case E_CLEAN_STEP::CLEAN_AREA:
        setCleanStep(procRunClean(robotPose));
        break;
    case E_CLEAN_STEP::CHECK_CLEAN_LIST:
        setCleanStep(procCheckCleanList(robotPose));
        break;
    case E_CLEAN_STEP::UPDATE_AREA:
        setCleanStep(procUpdateArea(robotPose));
        eblog(LOG_LV_NECESSARY,  "Line Clean End!!!");
        break;    
    case E_CLEAN_STEP::CLEAN_END:
        ret = procCleanEnd();
        break;    
    default:
        eblog(LOG_LV_NECESSARY,  "line clean state error!!!");
        break;    
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CClean::stopCleanning(void)
{
    CStopWatch __debug_sw;

    setCleanStep(E_CLEAN_STEP::INIT);

    

    eblog(LOG_LV_NECESSARY,  "stopLineclean!!!");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

E_CLEAN_STEP CClean::procInit(tPose robotPose)
{
    E_CLEAN_STEP ret = E_CLEAN_STEP::INIT;

    #if TEST_RANDOMPATTERN_FOR_CLEANTIME > 0
    ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "랜덤 청소 시작~~!!");
    clearCleanPattern();
    addCleanPattern(E_CLEAN_PATTERN::RANDOM);
    ret = E_CLEAN_STEP::CLEAN_AREA;
    #else
    ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "청소 시작~~!! 청소 영역을 확인합니다.");
    if( ROBOT_CONTROL.slam.isExistedSlamMap() )
    {
       ServiceData.robotMap.makeAreaByMap();
       ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "청소영역 준비: 지도를 불러와 청소영역을 생성합니다");
    }
    else
    {
        ServiceData.robotMap.makeCustomArea(robotPose);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소영역 준비: 지도가 없어 임시 청소영역을 생성합니다");
    }

    if(ServiceData.robotMap.getRooms().empty())
    {
        ret = E_CLEAN_STEP::CLEAN_END;
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소영역 준비: 청소할 영역이 없어 청소를 종료합니다.");
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소 계획 시작!!");
        ret = E_CLEAN_STEP::PLAN_AREA;
    }
    #endif

    return ret;
}

E_CLEAN_STEP CClean::procPlanArea(tPose robotPose)
{
	CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::PLAN_AREA;

    ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소를 계획합니다. 청소할 방 개수 : " 
        << ServiceData.robotMap.getRooms().size());
        
    ServiceData.robotMap.makePlan(robotPose);

    ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소계획을 세웠습니다. 청소할 방 개수 : " 
        << ServiceData.robotMap.getRooms().size());

    DEBUG_PUB.publishCleanArea(ServiceData.robotMap.getAllRooms(), 
            ServiceData.robotMap.getCurrentRoomId());
    
    cleanData_.arrangedPolygons.clear();
    std::list<tPoint> polygonPoints = ServiceData.robotMap.getCurrentAreaPolygons();
    
    for(tPoint polygonPoint : polygonPoints)
    {
        ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "청소영역 준비완료. 현재 영역 poligan : " 
            << polygonPoint.x << " , " << polygonPoint.y);
        cleanData_.arrangedPolygons.emplace_back(polygonPoint);        
    }

    if(cleanData_.patterns.front() == E_CLEAN_PATTERN::WALL)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "구석 청소를 시작 합니다.");
        ret = E_CLEAN_STEP::CLEAN_AREA;
    }    
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); //temp code for exception
        if(ServiceData.robotMap.checkPointInArea(tPoint(robotPose.x,robotPose.y)))
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 로봇이 영역 내부에 있어서 청소를 준비합니다.");
            ServiceData.kidnapData.setCheckingKidnap(true);
            ret = E_CLEAN_STEP::CLEAN_AREA;
        }
        else
        {
            PATH_PLANNER->startMapUpdate();
            ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "경로 이동을 위해 D* 에 map update를 시작 합니다.");
            std::list<tPoint> movePoints;
            movePoints.emplace_back( ServiceData.robotMap.getAreaCenterPoint());
            if (PATH_PLANNER->dofindNearestPath(robotPose,movePoints, movePoints.size()))
            {
                ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾습니다.");
                ret = E_CLEAN_STEP::CHECK_MOVE;
            }
            else
            {
                ceblog(LOG_LV_NECESSARY,  RED, "먼저 경로요청이 있었나봐요. 점검해주세요.");
            }
        }
    }                                                        
    

	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_CLEAN_STEP CClean::procCheckMove()
{
    CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::CHECK_MOVE;

    if ( PATH_PLANNER->isRunFindPath() == false )
    {
        if (PATH_PLANNER->isFindPath())
        {            
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾았습니다. 경로이동을 시작합니다. "); 
            SUB_TASK.navi->start( mtpInput(PATH_PLANNER->getPath().back(),0.5) );
            ret = E_CLEAN_STEP::MOVE_AREA;             
        }
        else
        {   
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로계획을 실패하여 재검색합니다. ");
            ret = E_CLEAN_STEP::RE_PLAN_AREA;
        }
    }

    return ret;
}

/**
 * @brief 영역 이동중 어떤 사유에 의하여 다시 선택된 경로를 뽑아야 할때...
 * 
 * @param robotPose 
 * @return E_LINECLEAN_STATE 
 */
E_CLEAN_STEP CClean::procRePlanArea(tPose robotPose)
{
	CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::RE_PLAN_AREA;

    
    std::list<tPoint> movePoints;
    movePoints.emplace_back( ServiceData.robotMap.getAreaCenterPoint());  
    if (PATH_PLANNER->dofindNearestPath(robotPose,movePoints,movePoints.size())){
        ret = E_CLEAN_STEP::CHECK_MOVE;
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "최적 경로 목적지를 찾습니다.");
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "먼저 경로요청이 있었나봐요. 점검해주세요.");
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


/**
 * @brief 현재 영역에서 라인청소를 하기 전에 영역을 따라서 벽면 주행을 함. 
 * 
 * @param robotPose 
 * @return E_LINECLEAN_STATE 
 */
E_CLEAN_STEP CClean::procMoveArea(tPose robotPose)
{
	CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::MOVE_AREA;

    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if(ServiceData.robotMap.checkPointInArea(tPoint(robotPose.x,robotPose.y)))
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 로봇이 영역 내부에 있어서 청소를 준비합니다.");
        PATH_PLANNER->stopMapUpdate();
        SUB_TASK.navi->stop();
        ServiceData.kidnapData.setCheckingKidnap(true);
        ret = E_CLEAN_STEP::CLEAN_AREA;
    }
    else   
    {
        if( SUB_TASK.navi->isRunning() == false )
        // if ( SUB_TASK.drivePath.isArrivalGoal() )
        {
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "***************************************************");
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "영역이동을 완료 하였습니다.");
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "***************************************************");
            ServiceData.kidnapData.setCheckingKidnap(true);
            ret = E_CLEAN_STEP::CLEAN_AREA;
        }
    }
    
    // if ( SUB_TASK.drivePath.requestRePath() )
    if( SUB_TASK.navi->getTaskState() == subTaskState::fail )
    {
        SUB_TASK.navi->stop();
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "청소에서 재 경로 요청을 수락 합니다.");
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 영역 검사를 다시하여 경로를 수정 합니다.");
        ret = E_CLEAN_STEP::PLAN_AREA;

        // SUB_TASK.drivePath.requestRePathClear();
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}




E_CLEAN_STEP CClean::procRunClean(tPose robotPose)
{
	CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::CLEAN_AREA;
    E_CLEAN_PATTERN pattren = getCurrentPattern();
    
    //E_CLEAN_STATE areaState = ServiceData.robotMap.getAreaCleanState();
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    cell_obstacle* pCellObsMap = pObsMap->getcellsObstaclePointer();
    
    if(pCleanPatterns->controlHandler(pattren, robotPose, cleanData_.arrangedPolygons,
        pCellObsMap, pObstacle))
    {
        ret = E_CLEAN_STEP::CHECK_CLEAN_LIST;
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

E_CLEAN_STEP CClean::procCheckCleanList(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::CHECK_CLEAN_LIST;

    if(getCurrentPattern() == E_CLEAN_PATTERN::LINE)
    {
        ServiceData.robotMap.setCurrentAreaCleanComplete(true);
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "Do_LineClean -> Ready_LINECLEAN!!");
        ret = E_CLEAN_STEP::UPDATE_AREA;
    }
    else if(getCurrentPattern() == E_CLEAN_PATTERN::WALL)
    {
        if(getCurrentPatternSize() > 1 )
        {
            popCurrentPattern();
#if 0 // 1 ->무한 벽타기
            cleanData_.patterns.push_front(E_CLEAN_PATTERN::WALL);
#endif
            pCleanPatterns->setPatternState(E_PATTERN_STATE::INIT);
            ret = E_CLEAN_STEP::CLEAN_AREA;
        }
        else
        {
            ret = E_CLEAN_STEP::CLEAN_END;
        }
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


E_CLEAN_STEP CClean::procUpdateArea(tPose robotPose)
{
    CStopWatch __debug_sw;
    E_CLEAN_STEP ret = E_CLEAN_STEP::UPDATE_AREA;
    
    int unCleanRoomSize = (int)ServiceData.robotMap.getUnCleanRoomSize();

    //추가로 생긴 공간이 있다면 AREA를 생성한다.
    //추가로 생긴 공간이 없다면, 모든 영역과 청소한 영역이 같은지 확인한다.
    if(unCleanRoomSize == 0)
    {
        //ret = E_CLEAN_STEP::PLAN_AREA;
        ret = E_CLEAN_STEP::INIT;
    }
    else
    {
        if(getCurrentPatternSize() > 1 )
        {
            popCurrentPattern();
            pCleanPatterns->setPatternState(E_PATTERN_STATE::INIT);
        }
        else
        {
            ret = E_CLEAN_STEP::INIT;
        }
    }

    ceblog(LOG_LV_NECESSARY, BOLDBLUE, "unCleanRoomSize Size : " << unCleanRoomSize);

    return ret;
}
/**
 * @brief 청소가 끝나면 원점으로 복귀
 * 
 * @return E_CLEAN_STEP 
 */
bool CClean::procCleanEnd()
{
    CStopWatch __debug_sw;

    bool ret = true;
    
    ServiceData.robotMap.clearRooms();

    PATH_PLANNER->stopMapUpdate();
    ceblog(LOG_LV_NECESSARY, BOLDBLUE,  "경로 이동 D* 에 map update를 중지 합니다.");

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}




#if 0
std::list<tPose> CClean::getSortByNearest(tPose robotPose, const std::list<tPoint>& targetList)
{
    CStopWatch __debug_sw;

    std::list<tPose> ret;
    std::list<tPoint> tmpList = targetList;
    int size = targetList.size();
    
    if(size == 0)
    {
        return ret;
    }

    tPose nearPose;
    for(int i=0; i < size; i++)
    {
        double dis = utils::math::distanceTwoPoint(tmpList.front(), robotPose);
        nearPose = tPose(tmpList.front().x, tmpList.front().y, 0);
        
        // 폴리건 중 가장 가까운 포인트를 구한다.
        for(tPoint& targetPoint : tmpList)
        {
            double aD = utils::math::distanceTwoPoint(targetPoint, robotPose);
            
            if(dis>aD)
            {
                dis = aD;
                nearPose = tPose(targetPoint.x,targetPoint.y, 0);
            }
        }
        ret.emplace_back(nearPose);
        tmpList.remove(tPoint(nearPose.x,nearPose.y));
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
#endif
