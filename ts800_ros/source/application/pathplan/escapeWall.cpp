#include "escapeWall.h"
#include "coreData/serviceData.h"
#include "motionController.h"
#include "eblog.h"
#include "rosPublisher.h"
#include "motionPlanner/motionPlanner.h"

using namespace std;

CEscapeWall::CEscapeWall()
{
    this->state = E_STATE::NONE;
}

CEscapeWall::~CEscapeWall() {}

CEscapeWall::E_STATE CEscapeWall::getState()
{
    return state;
}

/**
 * @brief 로봇이 벽안에 있는지 체크
 * 
 * @return true 
 * @return false 
 */
bool CEscapeWall::isRobotInWall()
{
    // if(pDstar->isUpdateGridMapWall()==false)
    //     return false;
    tPose robotPose = ServiceData.localiz.getPose();
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Dstar의 isWall 호출합니다.");
    return PATH_PLANNER->isWall(tPoint(robotPose.x, robotPose.y));
}

bool CEscapeWall::isPointInWall(tPoint point)
{
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Dstar의 isWall 호출합니다.");
    return PATH_PLANNER->isWall(point);
}

void CEscapeWall::start()
{
    state = E_STATE::RUNNING;

    tPose robotPose = ServiceData.localiz.getPose();
    robotPose.angle = ServiceData.localiz.getSysPose().angle;
    startPoint = tPoint(robotPose.x, robotPose.y);
    startBumperMask = RSF_OBSTACLE_MASK();
    initEscapeWall(robotPose);
}

void CEscapeWall::start(RSF_OBSTACLE_MASK bumperMask)
{
    state = E_STATE::RUNNING;

    tPose robotPose = ServiceData.localiz.getPose();
    robotPose.angle = ServiceData.localiz.getSysPose().angle;
    startPoint = tPoint(robotPose.x, robotPose.y);
    startBumperMask = bumperMask;
    initEscapeWall(robotPose);
}

void CEscapeWall::proc()
{
    switch (state)
    {
    case E_STATE::NONE:     break;
    case E_STATE::RUNNING:  setState(escapeWallAlgorithm());break;
    case E_STATE::COMPLETE: break;
    case E_STATE::FAIL:     break;
    default:                break;
    }
}

void CEscapeWall::setState(E_STATE state)
{
    if(this->state != state)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "EscapeWall State이 변경 됩니다. ["<<
            WHITE<<enumToString(this->state)<<BOLDBLACK<<"] --> ["<<
            BOLDCYAN<<enumToString(state)<<BOLDBLACK<<"]");
    }
    this->state = state;
}

/**
 * @brief Dstar의 cell 검색 순서 초기화
 * 
 */
void CEscapeWall::initEscapeWall(tPose startPose)
{
    if(!searchCells.empty())
    {
        searchCells.clear();
    }

    std::vector<tPoint> nearCellPoints;
    getNearCellPoints(0.35, nearCellPoints);
    int cellIndex = getStartIndex(startPose.angle);
    bool searchOrder = findSearchOrder(cellIndex, nearCellPoints);
    cellIndex = searchOrder ? (--cellIndex+8)%8 : (++cellIndex)%8;
    while(searchCells.size()!=8)
    {
        if(cellIndex<0 || cellIndex>=8)
        {
            ceblog(LOG_LV_NECESSARY, BOLDRED, "index 값이 이상합니다. index["<<cellIndex<<"]");
        }
        else
        {
            searchCells.push_back(CellInfo(cellIndex, nearCellPoints.at(cellIndex)));
            cellIndex = searchOrder ? (--cellIndex+8)%8 : (++cellIndex)%8;
        }
    }
    moveStep = E_MOVE_STEP::_1; // 제어 순서 초기화

    debugCells.clear();
    bumperList.clear();
#if 1
    debugCellPrint();
#endif
}

int debugCount = 0;
CEscapeWall::E_STATE CEscapeWall::escapeWallAlgorithm()
{
    robotPose = ServiceData.localiz.getPose(); // 로봇위치 업데이트

    // 모든 셀 검색 후에도 탈출을 못하면 실패.
    if(searchCells.empty()==true)   {return E_STATE::FAIL;}

    switch (searchCells.front().state)
    {
    case E_CELL_STATE::UNKNOWN:         searchCells.front().state = escapeWallMove();break;
    case E_CELL_STATE::CHECKING:        searchCells.front().state = escapeWallChecking();break;
    case E_CELL_STATE::COMEBACK:        searchCells.front().state = escapeWallComeback();break;
    case E_CELL_STATE::EMPTY_PLACE:
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"===========================");
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Dstar에 세팅할 장애물이 "<<BOLDGREEN<<bumperList.size()<<BOLDBLACK<<"개 입니다.");
        for(auto obs : bumperList)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK,
                "위치("<<BOLDWHITE<<obs.robotPose.x<<", "<<obs.robotPose.y<<BOLDBLACK<<")\t"<<
                (obs.left?BOLDGREEN:BOLDBLACK)<<"왼쪽"<<BOLDBLACK<<"|"<<
                (obs.right?BOLDGREEN:BOLDBLACK)<<"오른쪽");
            PATH_PLANNER->updateDstarWallBumper(obs.robotPose, obs.left, obs.right);
        }
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDMAGENTA<<"===========================");
        searchCells.clear();
        debugCells.clear();
        DEBUG_PUB.publishEscapeWallCells(searchCells, debugCells);
        // ServiceData.rosPublishData.pubEscapeWallCells(searchCells, debugCells);
        return E_STATE::COMPLETE;
    default: ceblog(LOG_LV_NECESSARY, BOLDRED, "잘못된 E_CELL_STATE 입니다.");break;
    }

#if 1 // cell 상태 디버그 print
    if(debugCount++%20==0)
    {
        // debugCellPrint();
        DEBUG_PUB.publishEscapeWallCells(searchCells, debugCells);
    }
#endif
    return E_STATE::RUNNING;
}

/**
 * @brief 탈출 시퀀스 이동 명령 내리기
 * [E_CELL_STATE::UNKNOWN] 에서 사용.
 * 1. searchOrder.front().first Cell 위치에 맞게 방향 돌림.
 * 2. 벽을 탈출 할 때까지 직진
 * 
 * @return CEscapeWall::E_CELL_STATE 
 */
CEscapeWall::E_CELL_STATE CEscapeWall::escapeWallMove()
{
    bool diagonal; // 탈출할 셀이 대각선인지
    double moveDistance; // 탈출할 셀로 이동할 거리

    // TODO: cell 좌표로 회전 및 일정거리 직진 제어 명령.
    switch (moveStep)
    {
    case E_MOVE_STEP::_INIT:
    case E_MOVE_STEP::_1: // 탈출할 cell로 제자리 회전제어
        // MOTION.startRotateToPointOnMap(robotPose, searchCells.front().point, tProfile());
        moveStep = E_MOVE_STEP::_1_WAITING;
        break;
    case E_MOVE_STEP::_1_WAITING: // 제어중...
        moveStep = MOTION.isRunning() ? E_MOVE_STEP::_1_WAITING : E_MOVE_STEP::_2;
        break;
    case E_MOVE_STEP::_2: // 탈출할 cell로 직진제어
        diagonal = (searchCells.front().index%2==0); // 탈출할 셀이 대각선인지
        // moveDistance = diagonal ? 0.35*1.414 : 0.35;
        moveDistance = utils::math::distanceTwoPoint(robotPose, searchCells.front().point);
        // MOTION.startLinearToDistanceOnRobot(robotPose, moveDistance, tProfile());
        ceblog(LOG_LV_NECESSARY, BOLDRED, "escapeWallMove 타겟 이동 --> moveDistance : " << moveDistance);
        moveStep = E_MOVE_STEP::_INIT; // 제어step 초기화
        return E_CELL_STATE::CHECKING;
    default:
        ceblog(LOG_LV_NECESSARY, BOLDRED, "잘못된 step 입니다.");
        break;
    }
    return E_CELL_STATE::UNKNOWN;
}

/**
 * @brief 제어 종료를 기다리면서, Cell 상태 확인.
 * 1. 현재 위치가 Dstar 인덱스가 변하는지 확인.
 * 2-1. Dstar 인덱스가 변하지 않았는데 장애물(범퍼, 낙하) 감지됨.
 *      cell 상태 obstacle로 변경.
 * 2-2. Dstar 인덱스가 변하였는데 장애물(범퍼, 낙하) 감지된.
 *      cell 상태 obstacle로 변경.
 * 2-3. Dstar 인덱스가 변하였고 직진 제어가 종료됨.
 *      cell 상태 empty place로 변경.
 * 
 * @return CEscapeWall::E_CELL_STATE 
 */
CEscapeWall::E_CELL_STATE CEscapeWall::escapeWallChecking()
{
    if( checkObstacle() )
    {
        
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "장애물이 감지 되었어요. cell 상태 변경  ["<<BOLDWHITE<<"Checking"<<BOLDBLACK<<"] -> ["<<BOLDGREEN<<"Obstacle"<<BOLDBLACK<<"]");
        return E_CELL_STATE::COMEBACK;
    }

    if( !MOTION.isRunning() ) // 직진 제어가 끝났을 때
    {
        tPose robotPose = ServiceData.localiz.getPose();
        if( false == PATH_PLANNER->isSameDstarPoint(tPoint(robotPose.x, robotPose.y), startPoint)) // Dstar 인덱스 변경 확인
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "탈출 성공했어요. cell 상태 변경  ["<<BOLDWHITE<<"Checking"<<BOLDBLACK<<"] -> ["<<BOLDGREEN<<"Empty Place"<<BOLDBLACK<<"]");
            return E_CELL_STATE::EMPTY_PLACE;
        }
        else /* TODO: Dstar 인덱스 변경안되는 이상한 케이스 (?) */
        {
            ceblog(LOG_LV_NECESSARY, BOLDRED, "이상한 케이스 Dstar 인덱스가 변경되지 않았어요. cell 상태 변경  ["<<BOLDWHITE<<"Checking"<<BOLDBLACK<<"] -> ["<<BOLDGREEN<<"Comback"<<BOLDBLACK<<"]");
            return E_CELL_STATE::COMEBACK;
        }
    }
    return E_CELL_STATE::CHECKING;
}

CEscapeWall::E_CELL_STATE CEscapeWall::escapeWallComeback()
{
    double backDistance; // 후진 거리 (시작 위치와 로봇 위치)
    switch (moveStep)
    {
    case E_MOVE_STEP::_INIT:
    case E_MOVE_STEP::_1: // 시작 위치로 복귀 제어
        backDistance = utils::math::distanceTwoPoint(robotPose, startPoint);
        backDistance = backDistance < 0.15 ? 0.15:backDistance;
        // MOTION.startLinearToDistanceOnRobot(robotPose, backDistance*(-1), tProfile());
        moveStep = E_MOVE_STEP::_1_WAITING;
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "시작 위치로 복귀 "<<BOLDGREEN<<"제어"<<BOLDBLACK<<"시작." << "후진거리 : " << backDistance); 
        break;
    case E_MOVE_STEP::_1_WAITING: // 시작 위치로 복귀 제어중
        if( !MOTION.isRunning() )
        {
            debugCells.push_back(searchCells.front());
            searchCells.pop_front();
            moveStep = E_MOVE_STEP::_INIT; // 제어step 초기화
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "시작 위치로 복귀 "<<BOLDGREEN<<"완료"<<BOLDBLACK<<"하였습니다.");
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "탈출할 수 있는 남은 셀은 "<<BOLDWHITE<<int(searchCells.size())<<BOLDBLACK<<"개 입니다.");
            return E_CELL_STATE::UNKNOWN;
        }
        break;
    default:
        ceblog(LOG_LV_NECESSARY, BOLDRED, "잘못된 step 입니다.");
        break;
    }
    return E_CELL_STATE::COMEBACK;
}

/**
 * @brief 시작 위치의 점들을 dstar 해상도에 따라서 8개 생성..
 * @param dstarCellResolution 
 * @return tPoint* 
 */
void CEscapeWall::getNearCellPoints(double dstarCellResolution, std::vector<tPoint> &points)
{    
    const double g = dstarCellResolution;
    points.push_back(tPoint(startPoint.x+g, startPoint.y+g));
    points.push_back(tPoint(startPoint.x+g, startPoint.y  ));
    points.push_back(tPoint(startPoint.x+g, startPoint.y-g));
    points.push_back(tPoint(startPoint.x  , startPoint.y-g));
    points.push_back(tPoint(startPoint.x-g, startPoint.y-g));
    points.push_back(tPoint(startPoint.x-g, startPoint.y  ));
    points.push_back(tPoint(startPoint.x-g, startPoint.y+g));
    points.push_back(tPoint(startPoint.x  , startPoint.y+g));
}

/**
 * @brief 시작 위치의 로봇 헤딩 방향에 따라 시작 위치를 정함.
 * 
 * @param robotAngle (단위 rad)
 * @return int 
 */
int CEscapeWall::getStartIndex(double robotAngle)
{
    double degAngle = utils::math::rad2deg(robotAngle);
    int robotHeadingIndex = 0; // 로봇헤딩이 가리키는 index 값
    if     ( 22.5 <=degAngle && degAngle<67.5  )    robotHeadingIndex = 0;
    else if( 337.5< degAngle || degAngle<22.5  )    robotHeadingIndex = 1;
    else if( 292.5<=degAngle && degAngle<337.5 )    robotHeadingIndex = 2;
    else if( 247.5<=degAngle && degAngle<292.5 )    robotHeadingIndex = 3;
    else if( 202.5<=degAngle && degAngle<247.5 )    robotHeadingIndex = 4;
    else if( 157.5<=degAngle && degAngle<202.5 )    robotHeadingIndex = 5;
    else if( 112.5<=degAngle && degAngle<157.5 )    robotHeadingIndex = 6;
    else /*(  67.5<=degAngle && degAngle<112.5 )*/  robotHeadingIndex = 7;

    return robotHeadingIndex;
}

/**
 * @brief 어느 방향으로 회전하면서 탈출을 시도할지 결정.
 * - 시작 인덱스 기준으로 양방향으로 벽을 체크함.
 * - 벽이 나오지 않은 방향으로 탈출 방향 결정.
 * @param startIndex 
 * @param cells 
 * @return true ccw(반시계 방향)
 * @return false cw(시계 방향)
 */
bool CEscapeWall::findSearchOrder(int startIndex, const std::vector<tPoint>& cells)
{
    /* 범퍼 장애물이 한쪽만 들어올 경우, 방향을 우선 결정 */
    if(startBumperMask.value & 0x0F) // 오른쪽 범퍼 장애물 -> 반시계 방향
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, " 탈출방향 결정\t오른쪽 범퍼 -> ("<<BOLDGREEN<<"  반시계방향"<<BOLDBLACK<<")");
        return true;
    }
    else if(startBumperMask.value & 0xF0) // 왼쪽 범퍼 장애물 -> 시계 방향
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, " 탈출방향 결정\t왼쪽 범퍼 -> ("<<BOLDGREEN<<"  시계방향"<<BOLDBLACK<<")");
        return false;
    }

    int ccwIndex = (--startIndex+8)%8;
    int cwIndex = (++startIndex)%8;

    while(ccwIndex != cwIndex)
    {
        /**
         * @brief 두 방향중 하나만 벽일때만 회전 방향이 결정됨.
         */
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Dstar의 isWall 호출합니다.");
        if(PATH_PLANNER->isWall(cells.at(ccwIndex))==true || PATH_PLANNER->isWall(cells.at(cwIndex))==false)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "탈출방향 결정\tDstar 벽 -> ("<<BOLDGREEN<<"  시계방향"<<BOLDBLACK<<")");
            return false;
        }
        else if(PATH_PLANNER->isWall(cells.at(ccwIndex))==true || PATH_PLANNER->isWall(cells.at(cwIndex))==false)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "탈출방향 결정\tDstar 벽 -> ("<<BOLDBLUE<<"반시계방향"<<BOLDBLACK<<")");
            return true;
        }

        ccwIndex = (--ccwIndex+8)%8;
        cwIndex = (++cwIndex)%8;
    }

    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "탈출방향 결정\tDstar 벽으로 판별이 되지않아 기본값으로 합니다. ("<<BOLDBLUE<<"반시계방향"<<BOLDBLACK<<")");
    return true; // 장애물을 통하여 탈출 방향이 결정되지 않으면, 반시계방향으로 탈출 시도.
}

bool CEscapeWall::checkObstacle()
{
    RSU_OBSTACLE_DATA* pObstacle = ServiceData.obstacle.getObstacleData();
    if( pObstacle->bumper.b.fleft_side || pObstacle->bumper.b.fright_side)
    {
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "장애물 "<<BOLDYELLOW<<"감지");
        bumperList.push_back(tBumperObstacle(ServiceData.localiz.getPose(), pObstacle->bumper.b.fleft_side, pObstacle->bumper.b.fright_side)); // dstar에 세팅할 장애물 정보 저장
        // pDstar->updateDstarWallBumper(robotPose, pObstacle->bumper.b.fleft_side, pObstacle->bumper.b.fright_side); // dstar에 바로 장애물 세팅
        return true;
    }
    return false;
}

void CEscapeWall::debugCellPrint()
{
    bool isFirst = true;
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "--- 현재 cell 상태 ---");
    for(auto cellInfo : searchCells)
    {
        if(isFirst)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "["<<BOLDGREEN<<cellInfo.index<<BOLDBLACK
                <<"]\t|"<<BOLDGREEN<<"("<<setw(5)<<cellInfo.point.x<<", "<<setw(5)<<cellInfo.point.y<<BOLDBLACK
                <<")\t 상태: "<<BOLDCYAN<<enumToString(cellInfo.state));
            isFirst = false;   
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, "["<<GREEN<<cellInfo.index<<BOLDBLACK
                <<"]\t|"<<WHITE<<"("<<setw(5)<<cellInfo.point.x<<", "<<setw(5)<<cellInfo.point.y<<BOLDBLACK
                <<")\t 상태: "<<WHITE<<enumToString(cellInfo.state));
        }
    }
    ceblog(LOG_LV_NECESSARY, BOLDBLACK, "---------------------");
}