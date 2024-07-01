#include "dstar.h"
#include "control/motionController.h"
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 20.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CDstar::CDstar()
{
    CStopWatch __debug_sw;

    pthread_mutex_init(&mutexDstarCore, nullptr);
    pthread_mutex_init(&mutexObstacleWall, nullptr);
    pthread_mutex_init(&mutexGridMapWall, nullptr);

    state tmp;
    tmp.x =0;
    tmp.y =0;
    tmp.k = make_pair(0.0,0.0);    
    
    dstarData_.isRunFindNearestPath = false;
    dstarData_.bFindNearSuccess = false;
    dstarData_.isRobotInWall = false;
    dstarData_.k_m = 0;
    dstarData_.startPoint = tmp;
    dstarData_.goalPoint = tmp;
    dstarData_.s_last = tmp;
    dstarData_.maxSteps = (int)(12000/MAP_DOWNSCALE_VALUE);    // 80000(default) 최대 노드 스텝
    dstarData_.unknownCellCost = 1; // unknown 셀의 cost
    bObstacleWallUpdate = false;
    bUpdatedGridMapWall = false;
    eblog(LOG_LV,  "CDstar");

    init(tPose(0.0,0.0,0.0), tPoint(1.0,1.0));  //임의의 포즈를 생성 해놓는다.
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CDstar::~CDstar()
{
    CStopWatch __debug_sw;
    
    if (!shortestPath.empty()) shortestPath.clear(); 
    while (!openList.empty()) openList.pop();
    if (!cellHash.empty()) cellHash.clear();
    if (!openHash.empty()) openHash.clear();

    pthread_mutex_destroy(&mutexDstarCore);
    pthread_mutex_destroy(&mutexObstacleWall);
    pthread_mutex_destroy(&mutexGridMapWall);
   
    eblog(LOG_LV,  "~CDstar");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 키 해시코드를 반환합니다. (비교하는데 사용)
 *
 * @param point
 * @return float
 */
float CDstar::keyHashCode(state point)
{
    return (float)(point.k.first + 1193 * point.k.second);
}

/**
 * @brief 현재 cell 위치가 올바른지 체크
 *
 * @param point
 * @return true
 * @return false
 */
bool CDstar::isValid(state point)
{
    CStopWatch __debug_sw;
    
    ds_oh::iterator cur = openHash.find(point);

    if (cur == openHash.end())
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return false;
    }
    if (!checkValueClose(keyHashCode(point), cur->second))
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return false;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return true;
}

/**
 * @brief d*로 계산된 최단경로를 반환.
 *
 * @return list<state>
 */
void CDstar::getPath(list<tPoint>& paths, tPoint orgTarget)
{
    CStopWatch __debug_sw;
    
    if(shortestPath.empty())
    {
        eblog(LOG_LV_NECESSARY,  "[ DSTAR ]  origin path empty!");
        paths.emplace_back(tPoint(0, 0));
    }
    else
    {
        eblog(LOG_LV_NECESSARY,  "[ DSTAR]  path origin size : " << shortestPath.size());

        std::list<tPoint> firstTunedPath;
        tPoint pathPoint;

        for (state tmpState : shortestPath)
        {
            if(checkNeighboursWall(tmpState) || (tmpState.x ==shortestPath.back().x && tmpState.y ==shortestPath.back().y))
            {
                pathPoint.x = (double)(tmpState.x*MAP_DOWNSCALE_VALUE+(int)(MAP_DOWNSCALE_VALUE/2.0))*CELL_RESOLUTUION + CELL_RESOLUTUION*(double)((CELL_X / 2) * -1);
                pathPoint.y = (double)(tmpState.y*MAP_DOWNSCALE_VALUE+(int)(MAP_DOWNSCALE_VALUE/2.0))*CELL_RESOLUTUION + CELL_RESOLUTUION*(double)((CELL_Y / 2) * -1);

                firstTunedPath.emplace_back(pathPoint);
            }
            else
            {
                pathPoint.x = (double)(tmpState.x*MAP_DOWNSCALE_VALUE+(int)(MAP_DOWNSCALE_VALUE/2.0))*CELL_RESOLUTUION + CELL_RESOLUTUION*(double)((CELL_X / 2) * -1);
                pathPoint.y = (double)(tmpState.y*MAP_DOWNSCALE_VALUE+(int)(MAP_DOWNSCALE_VALUE/2.0))*CELL_RESOLUTUION + CELL_RESOLUTUION*(double)((CELL_Y / 2) * -1);
                // ceblog(LOG_LV_NECESSARY, BOLDYELLOW ," 주변에 벽이 없어서 제거된 경로 :  " << pathPoint.x << "," << pathPoint.y);    
            }
        }
        // utils::path::ramerDouglasPeucker(firstTunedPath, 0.1, paths);
        if(!firstTunedPath.empty())
        {
            paths = firstTunedPath;
            paths.pop_back();
            paths.emplace_back(orgTarget);  //해상도 문제로 마지막은 목표 좌표를 넣어주기
        }
#if 0   //debug
        for(tPoint pt : firstTunedPath){
            eblog(LOG_LV_NECESSARY,  "org : " << pt.x << "," << pt.y);
        }
        for(tPoint pt : paths){
            eblog(LOG_LV_NECESSARY,  "path : " << pt.x << "," << pt.y);
        }            
#endif
        
    }
    ceblog(LOG_LV_PATHPLAN, BOLDGREEN, "called");
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CDstar::checkNeighboursWall(state point)
{
    bool ret = false;
    std::list<tPoint> neighbors;
    tPoint checkingPoint;
    state neighborState;
    checkingPoint.x = point.x;
    checkingPoint.y = point.y;

    neighbors = getNeighboringCoordinates4(checkingPoint, 1);
    for(tPoint neighbor : neighbors)
    {
        neighborState.x = neighbor.x;
        neighborState.y = neighbor.y;
        if (cellHash.find(neighborState) != cellHash.end())
        {
            if(cellHash[neighborState].cost == -1)    
            {
                ret = true;
                break;
            }
        }
    }
    return ret;
}

/**
 * @brief point의 occupied 확인
 *
 * @param point
 * @return true
 * @return false
 */
bool CDstar::checkOccupied(state point)
{
    CStopWatch __debug_sw;

    ds_ch::iterator cur = cellHash.find(point);

    if (cur == cellHash.end())
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return false;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return (cur->second.cost < 0);
}



/**
 * @brief 로봇의 경로계획을 위한 시작좌표와 목표좌표를 설정한다.
 * 
 * @param robot 로봇의 시작위치
 * @param goal  로봇의 목표위치
 */
void CDstar::init(tPose robotPose,tPoint goalPoint)
{
    CStopWatch __debug_sw;

    cellHash.clear();
    openHash.clear();
    
    tCellPoint goal;    // 목표 셀 좌표
    tCellPoint start;   // 시작 셀 좌표

    utils::coordination::convertCoord2CellCoord(robotPose.x, robotPose.y, &start.x, &start.y);
    utils::coordination::convertCoord2CellCoord(goalPoint.x, goalPoint.y, &goal.x, &goal.y);
        
    ceblog(LOG_LV_PATHPLAN, BOLDBLUE, "openList -pre : " <<openList.size()); 
    ds_pq().swap(openList);
    ceblog(LOG_LV_PATHPLAN, BOLDBLUE, "openList -cls : " <<openList.size()); 

    dstarData_.k_m = 0;

    dstarData_.startPoint.x = start.x;
    dstarData_.startPoint.y = start.y;
    dstarData_.goalPoint.x = goal.x;
    dstarData_.goalPoint.y = goal.y;
    
    cellInfo tmp;
    tmp.g = tmp.rhs = 0;
    tmp.cost = dstarData_.unknownCellCost;

    cellHash[dstarData_.goalPoint] = tmp;

    tmp.g = tmp.rhs = heuristic(dstarData_.startPoint, dstarData_.goalPoint);
    tmp.cost = dstarData_.unknownCellCost;
    cellHash[dstarData_.startPoint] = tmp;
    dstarData_.startPoint = calculateKey(dstarData_.startPoint);

    dstarData_.s_last = dstarData_.startPoint;
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  가장 가까운 goal을 색출하는 thread에서 가장 가까운 목표점에 대한 경로 set
 * jhnoh, 23.08.14
 * @param goalPose 
 */
void CDstar::setNearPath(std::list<tPoint> nearestPath)
{
    dstarData_.nearestPath.clear();
    dstarData_.nearestPath = nearestPath;
    // ceblog(LOG_LV_NECESSARY, BOLDBLUE, "D* 에서 searchedpath 가 갱신 됨");
}

/**
 * @brief  가장 가까운 goal을 색출하는 thread에서 가장 가까운 목표점에 대한 경로 get
 * jhnoh, 23.08.14
 * @param goalPose 
 */
std::list<tPoint> CDstar::getPath()
{
    return dstarData_.nearestPath;
}


void CDstar::clearPath()
{
    setFindNearSuccess(false);
    dstarData_.nearestPath.clear();
}

/**
 * @brief 경로계획 성공여부 set
 * jhnoh, 23.08.14 
 * @param ret 
 */
void CDstar::setFindNearSuccess(bool set)
{
    dstarData_.bFindNearSuccess = set;
}

/**
 * @brief 경로계획 성공여부 get
 * jhnoh, 23.08.14 
 * @return true 
 * @return false 
 */
bool CDstar::getFindNearSuccess()
{
    return dstarData_.bFindNearSuccess; 
}

/**
 * @brief 셀 생성
 * jhnoh, 23.08.14
 * @param point
 */
void CDstar::makeNewCell(state point)
{
    CStopWatch __debug_sw;
    
    if (cellHash.find(point) != cellHash.end())
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return;
    }

    cellInfo tmp;

    tmp.g = tmp.rhs = heuristic(point, dstarData_.goalPoint);
    tmp.cost = dstarData_.unknownCellCost;
    cellHash[point] = tmp;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief   point의 g(point)를 얻음.
 * jhnoh, 23.08.14
 * @param point
 * @return double
 */
double CDstar::getG(state point)
{
    if (cellHash.find(point) == cellHash.end())
    {
        return heuristic(point, dstarData_.goalPoint);
    }

    return cellHash[point].g;
}

/**
 * @brief   point의 rhs(point)를 얻음
 * jhnoh, 23.08.14 
 * @param point 
 * @return double 
 */
double CDstar::getRhs(state point)
{
    CStopWatch __debug_sw;

    if (point == dstarData_.goalPoint)
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return 0;
    }

    if (cellHash.find(point) == cellHash.end())
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return heuristic(point, dstarData_.goalPoint);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return cellHash[point].rhs;
}


/**
 * @brief point의 g(point)를 입력
 * jhnoh, 23.08.14 
 * @param point 
 * @param g 
 */
void CDstar::setG(state point, double g)
{
    CStopWatch __debug_sw;

    makeNewCell(point);
    cellHash[point].g = g;
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief point의 rhs(point)를 입력
 * jhnoh, 23.08.14
 * @param point
 * @param g
 */
double CDstar::setRhs(state point, double rhs)
{
    CStopWatch __debug_sw;

    makeNewCell(point);
    cellHash[point].rhs = rhs;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  로봇이 이동 가능한 8방향으로 두 점 사이의 거리를 계산하고 반환한다.
 * jhnoh, 23.08.14
 * @param point1     첫번째 점
 * @param point2     두번째 점
 * @return double 두 점 사이의 거리 반환
 */
double CDstar::eightCondist(state point1, state point2)
{
    CStopWatch __debug_sw;
    
    double minDis = fabs(point1.x - point2.x);
    double maxDis = fabs(point1.y - point2.y);

    if (minDis > maxDis)
    {
        double temp = minDis;
        minDis = maxDis;
        maxDis = temp;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ((M_SQRT2 - 1.0) * minDis + maxDis);
}


/**
 * @brief 최단경로 계산
 * jhnoh, 23.08.14 
 * @return int 
 */
int CDstar::computeShortestPath()
{
    CStopWatch __debug_sw;

    list<state> pointList;
    list<state>::iterator i;

    if (openList.empty())
    {        
        TIME_CHECK_END(__debug_sw.getTime());
        return 1;
    }

    int k = 0;
    while ((openList.top() < (dstarData_.startPoint = calculateKey(dstarData_.startPoint))) || (getRhs(dstarData_.startPoint) != getG(dstarData_.startPoint)))
    {
        if (k++ > dstarData_.maxSteps)
        {
            eblog(LOG_LV, "computeShortestPath return -1 k[ " << k << " ] maxStep[ " << dstarData_.maxSteps << "] tick[ " << SYSTEM_TOOL.getSystemTick() << " ]");
            return -1;
        }

        state point;

        bool test = (getRhs(dstarData_.startPoint) != getG(dstarData_.startPoint));

        // lazy remove
        while (1)
        {
            if (openList.empty())
            {
                TIME_CHECK_END(__debug_sw.getTime());
                return 1;
            }

            point = openList.top();
            openList.pop();

            if (!isValid(point))
            {
                continue;
            }
            
            if (!(point < dstarData_.startPoint) && (!test))
            {
                TIME_CHECK_END(__debug_sw.getTime());
                return 2;
            }
            break;
        }

        ds_oh::iterator cur = openHash.find(point);
        openHash.erase(cur);

        state k_old = point;

        if (k_old < calculateKey(point))
        { // u is out of date
            insert(point);
        }
        else if (getG(point) > getRhs(point))
        { // needs update (got better)
            setG(point, getRhs(point));
            getPerviousState(point, pointList);
            for (i = pointList.begin(); i != pointList.end(); ++i)
            {
                updateVertex(*i);
            }
        }
        else
        { // g <= rhs, state has got worse
            setG(point, INFINITY);
            getPerviousState(point, pointList);
            for (i = pointList.begin(); i != pointList.end(); ++i)
            {
                updateVertex(*i);
            }
            updateVertex(point);
        }
    }
    TIME_CHECK_END(__debug_sw.getTime());
    return 0;
}

/**
 * @brief  value1과 value2의 차이가 10E-5 미만이면 true 반환
 * jhnoh, 23.08.14
 * @param value1
 * @param value2
 * @return true
 * @return false
 * 
 * @note    연산시간: 1.2ms ~ 10.6ms 
 * @date    2023-08-28
 */
bool CDstar::checkValueClose(double value1, double value2)
{
    CStopWatch __debug_sw;
    
    if (isinf(value1) && isinf(value2))
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return true;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return (fabs(value1 - value2) < 0.00001);
}

/**
 * @brief point 업데이트
 * jhnoh, 23.08.14
 * @param point
 */
void CDstar::updateVertex(state point)
{
    CStopWatch __debug_sw;

    list<state> neighborPoint;
    list<state>::iterator iter;

    if (point != dstarData_.goalPoint)
    {
        getNextState(point, neighborPoint);
        double tmp = INFINITY;

        for (iter = neighborPoint.begin(); iter != neighborPoint.end(); ++iter)
        {
            double  tmp2 = getG(*iter) + cost(point, *iter);
            if (tmp2 < tmp)
            {
                tmp = tmp2;
            }
        }
        if (!checkValueClose(getRhs(point), tmp))
        {
            setRhs(point, tmp);
        }
    }

    if (!checkValueClose(getG(point), getRhs(point)))
    {
        insert(point);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief openHash에 state(point) 추가
 * jhnoh, 23.08.14
 * @param point
 */
void CDstar::insert(state point)
{
    CStopWatch __debug_sw;

    ds_oh::iterator cur;
    float csum;

    point = calculateKey(point);
    cur = openHash.find(point);
    csum = keyHashCode(point);

    openHash[point] = csum;
    openList.push(point);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  point2과 point2 사이의 유클리드 거리 반환
 * jhnoh, 23.08.14 
 * @param point1 
 * @param point2 
 * @return double 
 */
double CDstar::euclideanCost(state point1, state point2)
{
    CStopWatch __debug_sw;

    float x = point1.x - point2.x;
    float y = point1.y - point2.y;
    
    TIME_CHECK_END(__debug_sw.getTime());
    return sqrt(x * x + y * y);
}

/**
 * @brief point1과 point 2까지의 어림값.
 * jhnoh, 23.08.14
 * @param point1 첫번째 점
 * @param point2 두번째 점
 * @return double
 */
double CDstar::heuristic(state point1, state point2)
{
    return eightCondist(point1, point2) * dstarData_.unknownCellCost;
}

/**
 * @brief point의 key(k.first, k.second로 구성)를 계산
 * jhnoh, 23.08.14
 * @param point
 * @return state
 */
state CDstar::calculateKey(state point)
{
    CStopWatch __debug_sw;

    double val = fmin(getRhs(point), getG(point));

    point.k.first = val + heuristic(point, dstarData_.startPoint) + dstarData_.k_m;
    point.k.second = val;

    TIME_CHECK_END(__debug_sw.getTime());
    return point;
}


/**
 * @brief point1에서 point2로 이동하는 cost를 반환 (8-way)
 *  jhnoh, 23.08.14
 * @param point1 첫번째 점
 * @param point2 두번째 점
 * @return double 
 */
double CDstar::cost(state point1, state point2)
{
    CStopWatch __debug_sw;
    
    int distanceX = fabs(point1.x - point2.x);
    int distanceY = fabs(point1.y - point2.y);
    double scale = 1;

    if (distanceX + distanceY > 1)
        scale = M_SQRT2;

    if (cellHash.count(point1) == 0)
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return scale * dstarData_.unknownCellCost;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return scale * cellHash[point1].cost;
}
void CDstar::updatePoint(std::list <tDstarWallPoint> &set)
{
    CPthreadLockGuard lock(mutexDstarCore);
    clearWall();
    // ceblog(LOG_LV_SYSDEBUG, BOLDYELLOW, "locked");
    for(auto it : set){
        updatePoint((int)(it.x/MAP_DOWNSCALE_VALUE), (int)(it.y/MAP_DOWNSCALE_VALUE), it.costValue );        
    }
    // ceblog(LOG_LV_SYSDEBUG, BOLDGREEN, "lock free");
}

/**
 * @brief       point의  cost 업데이트
 * jhnoh, 23.08.14
 * @param pointX     point의 x
 * @param pointY     point의 y
 * @param costValue   cost 값
 */
void CDstar::updatePoint(int pointX, int pointY, double costValue)
{
    CStopWatch __debug_sw;
    
    state point;

    point.x = pointX;
    point.y = pointY;

#if 1 // 로봇 근처의 벽 장애물 제거 코드
    if ((point == dstarData_.startPoint))
    {        
        // if(costValue == -1)
        // {
        //     ceblog(LOG_LV_NECESSARY,BOLDYELLOW, " 로봇이 벽 안에 있어요.");
        // }
        TIME_CHECK_END(__debug_sw.getTime());
        return;
    }
#endif

    if(point == dstarData_.goalPoint)
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return;
    }

    makeNewCell(point);
    if(cellHash[point].cost != -1)
    {
        cellHash[point].cost = costValue;
    }
    updateVertex(point);
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 경로 목적지만 업데이트
 * jhnoh, 23.08.14 
 * @param goalX 
 * @param goalY 
 * 
 * @note  연산시간: 36.0ms ~ 142.0ms 
 * @date  2023-08-28
 */
void CDstar::updateGoal(tPoint goalPoint) 
{
    // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "called");
    CStopWatch __debug_sw;
    list<tDstarWallPoint> toAdd;
    tDstarWallPoint tp;

    ds_ch::iterator i;
    list<tDstarWallPoint>::iterator kk;

    tCellPoint goal;    // 목표 셀 좌표
    utils::coordination::convertCoord2CellCoord(goalPoint.x, goalPoint.y, &goal.x, &goal.y);

    dstarData_.goalPoint.x  = (int)(goal.x/MAP_DOWNSCALE_VALUE);
    dstarData_.goalPoint.y  = (int)(goal.y/MAP_DOWNSCALE_VALUE);
    //ceblog(LOG_LV_NECESSARY, BOLDGREEN, "called cellHash size : "<<cellHash.size());
    for(i=cellHash.begin(); i!=cellHash.end(); i++) 
    {
        if (!isXYclose(i->second.cost, dstarData_.unknownCellCost)) 
        {
            tp.x = i->first.x;
            tp.y = i->first.y;
            tp.costValue = i->second.cost;
            toAdd.push_back(tp);
        }
    }
    //ceblog(LOG_LV_NECESSARY, BOLDGREEN, "called");

    cellHash.clear();
    openHash.clear();

    ds_pq().swap(openList);   
    dstarData_.k_m = 0;

    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    tmp.cost = dstarData_.unknownCellCost;

    cellHash[dstarData_.goalPoint] = tmp;

    tmp.g = tmp.rhs = heuristic(dstarData_.startPoint,dstarData_.goalPoint);
    tmp.cost = dstarData_.unknownCellCost;
    cellHash[dstarData_.startPoint] = tmp;
    dstarData_.startPoint = calculateKey(dstarData_.startPoint);

    dstarData_.s_last = dstarData_.startPoint;
    
    for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
        updatePoint(kk->x, kk->y, kk->costValue);
    }

    

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief  x, y가 가까운지를 판단.
 *  jhnoh, 23.08.14
 * @param x 
 * @param y 
 * @return true 
 * @return false 
 */
bool CDstar::isXYclose(double x, double y) 
{
    CStopWatch __debug_sw;
    
    if (isinf(x) && isinf(y))
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return true;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return (fabs(x-y) < 0.00001);
}

bool CDstar::isWall(tPoint point)
{
    CPthreadLockGuard lock(mutexDstarCore);
    ceblog(LOG_LV_SYSDEBUG, BOLDYELLOW, "locked");

    state statePoint;
    utils::coordination::convertCoord2CellCoord(point.x, point.y, &statePoint.x, &statePoint.y);
    statePoint.x  = (int)(statePoint.x/MAP_DOWNSCALE_VALUE);
    statePoint.y  = (int)(statePoint.y/MAP_DOWNSCALE_VALUE);

#if 1
    bool bWall = checkOccupied(statePoint);
    if(bWall)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "확인할 좌표("<<point.x<<", "<<point.y<<" )\t"<<BOLDRED<<"벽입니다.");
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "확인할 좌표("<<point.x<<", "<<point.y<<" )\t 벽이 아닙니다.");
    }
#endif
    ceblog(LOG_LV_SYSDEBUG, BOLDGREEN, "lock free");
    return checkOccupied(statePoint) ? true : false;
}

bool CDstar::isSameDstarPoint(tPoint point1, tPoint point2)
{
    state statePoint1;
    utils::coordination::convertCoord2CellCoord(point1.x, point1.y, &statePoint1.x, &statePoint1.y);
    statePoint1.x  = (int)(statePoint1.x/MAP_DOWNSCALE_VALUE);
    statePoint1.y  = (int)(statePoint1.y/MAP_DOWNSCALE_VALUE);

    state statePoint2;
    utils::coordination::convertCoord2CellCoord(point2.x, point2.y, &statePoint2.x, &statePoint2.y);
    statePoint2.x  = (int)(statePoint2.x/MAP_DOWNSCALE_VALUE);
    statePoint2.y  = (int)(statePoint2.y/MAP_DOWNSCALE_VALUE);

    return (statePoint1 == statePoint2);
}

void CDstar::updateDstarWallPoint(tPoint wallPoint)
{    
    tDstarWallPoint pt;
    utils::coordination::convertCoord2CellCoord(wallPoint.x, wallPoint.y, &pt.x, &pt.y);
    pt.costValue = -1; 
    std::list <tDstarWallPoint> temp;

    temp.push_back(pt);  

    // 특정 좌표가 벡터에 없을 때만 푸시
    //TODO : dstar setObstacleWall() 함수에서 처리하게 변경
    // if (std::find(obstacleWall.begin(), obstacleWall.end(), pt) == obstacleWall.end()) {
    //     obstacleWall.push_back(pt);
    // }
    setObstacleWall(temp);
}

void CDstar::updateDstarWallBumper(tPose robotPose, bool left, bool right)
{
    const double centerToBumper = 0.1;
    const double centerToSide = 0.13;
    tPoint wallPoint;
    if(left && right)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "범퍼 데이터 dstar 벽 업데이트하겠습니다. ("<<BOLDRED<<"가운데"<<BOLDBLACK<<")");
        wallPoint.x = robotPose.x + centerToBumper*std::cos(robotPose.angle);
        wallPoint.y = robotPose.y + centerToBumper*std::sin(robotPose.angle);
        updateDstarWallPoint(wallPoint);
    }
    else if(left)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "범퍼 데이터 dstar 벽 업데이트하겠습니다. ("<<BOLDYELLOW<<"왼쪽"<<BOLDBLACK<<")");
        wallPoint.x = robotPose.x + centerToBumper*std::cos(robotPose.angle) - centerToSide*std::sin(robotPose.angle);
        wallPoint.y = robotPose.y + centerToBumper*std::sin(robotPose.angle) + centerToSide*std::cos(robotPose.angle);
        updateDstarWallPoint(wallPoint);
    }
    else if(right)
    {
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "범퍼 데이터 dstar 벽 업데이트하겠습니다. ("<<BOLDYELLOW<<"오른쪽"<<BOLDBLACK<<")");
        wallPoint.x = robotPose.x + centerToBumper*std::cos(robotPose.angle) + centerToSide*std::sin(robotPose.angle);
        wallPoint.y = robotPose.y + centerToBumper*std::sin(robotPose.angle) - centerToSide*std::cos(robotPose.angle);
        updateDstarWallPoint(wallPoint);
    }
}

bool CDstar::isUpdateObstacleWall()
{
    return bObstacleWallUpdate;
}

/**
 * @brief 지도에 의해 obstacle 이 삭제 되어 goal 에 도착 했을때 클리어 해야 한다.
 * 
 */
void CDstar::clearObstacleWall()
{
    ceblog(LOG_LV_NECESSARY, RED, "called");
    CPthreadLockGuard lock(mutexObstacleWall);
    obstacleWall.clear();
}

void CDstar::setObstacleWall(std::list <tDstarWallPoint> &set)
{
    ceblog(LOG_LV_NECESSARY, YELLOW, " : " << set.size() <<" 개");
    CPthreadLockGuard lock(mutexObstacleWall);
    for (const auto& element : set) {
        // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "push 전");
        obstacleWall.push_back(element);
        // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "push 후");
    }
    bObstacleWallUpdate = true;
    ceblog(LOG_LV_NECESSARY, YELLOW, "end..");
}

std::list <tDstarWallPoint> CDstar::useObstacleWall()
{
    CPthreadLockGuard lock(mutexObstacleWall);
    std::list <tDstarWallPoint> ret;
    for (const auto& element : obstacleWall) {
        ret.push_back(element);
    }
    bObstacleWallUpdate = false;
    return ret;
}

void CDstar::updateObstacleWall()
{
    std::list <tDstarWallPoint> ret = useObstacleWall();
    updatePoint(ret);
}

std::list <tDstarWallPoint> CDstar::useGridMapWall()
{
    CPthreadLockGuard lock(mutexGridMapWall);
    std::list <tDstarWallPoint> ret;
    for (const auto& element : gridMapWall) {
        ret.push_back(element);
    }
    bUpdatedGridMapWall = false;    
    return ret;
}

/**
 * @brief grid 를 기준으로 obstacle 값에 우선순위를 주어 조합 한다.
 * 
 * @param grid 
 * @param obstacle 
 * @return std::list <tDstarWallPoint> 
 */

std::list <tDstarWallPoint> CDstar::combineWall(std::list <tDstarWallPoint> grid, std::list <tDstarWallPoint> obstacle)
{
    std::list<tDstarWallPoint> combine;

    // grid의 모든 요소를 combine에 추가
    combine.insert(combine.end(), grid.begin(), grid.end());

    // obstacle의 각 요소에 대해
    for (const auto& obstaclePoint : obstacle) {
        // 같은 x, y를 가진 grid의 요소를 찾음
        auto gridPoint = std::find_if(combine.begin(), combine.end(),
            [&obstaclePoint](const tDstarWallPoint& gridElement) {
                return (gridElement.x == obstaclePoint.x) && (gridElement.y == obstaclePoint.y);
            });

        // 같은 x, y를 가진 grid의 요소가 있으면 obstacle의 costValue로 업데이트
        if (gridPoint != combine.end()) {
            gridPoint->costValue = obstaclePoint.costValue;
        } else {
            // 없으면 obstacle의 요소를 combine에 추가
            combine.push_back(obstaclePoint);
        }
    }
    return combine;
}

void CDstar::updateGridMapWall()
{
    if( isUpdateGridMapWall() || isUpdateObstacleWall() )
    {
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, " st");
        std::list <tDstarWallPoint> grid = useGridMapWall();
        // std::list <tDstarWallPoint> obstacle = useObstacleWall();
        // std::list <tDstarWallPoint> combine = combineWall(grid, obstacle);
        
        //combinewall 을 넣는게 맞으나 obstacle을 현재 안쓰고 있어서 grid 만 넣음
        updatePoint(grid);  
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, " fin");
        //ceblog(LOG_LV_NECESSARY, BOLDBLACK, "@@@@ Grid 혹은 Obstacle 이 업데이트되어 "<<BOLDYELLOW<<"updatePoint(combine)"<<BOLDBLACK<<" 실행");
    }
}

bool CDstar::isUpdateGridMapWall()
{
    return bUpdatedGridMapWall;
}

Mat CDstar::where(const Mat &condition, const Mat &x, const Mat &y)
{
    // Ensure the matrices have the same size and type
    if (condition.size() != x.size() || condition.size() != y.size() || condition.type() != CV_8UC1)
    {
        throw std::invalid_argument("Input matrices must have the same size and condition must be of type CV_8UC1.");
    }

    // Create an output matrix with the same size and type as x and y
    Mat result = Mat::zeros(x.size(), x.type());

    // Use the condition mask to copy elements from x or y
    x.copyTo(result, condition);  // Copy elements from x where condition is true
    y.copyTo(result, ~condition); // Copy elements from y where condition is false

    return result;
}

void CDstar::makeWallListFromGridMap(tPose robotPose, tGridmapInfo mapInfo, u8 *pGridmap)
{
    dstarCellInfo = mapInfo;
    auto ox = 0, oy = 0, cellIdx = 0;
    tDstarWallPoint add;
    std::list<tDstarWallPoint> wall;
    std::list<tPoint> debugWallData;
    int robotCellX = 0, robotCellY = 0;
    tPoint robotAroundPoint; // 로봇 주변의 좌표 단위(m, m)
    int index = 0;

    // utils::coordination::convertCoordi2Index(robotPose.x, robotPose.y, 
    //     &robotCellX, &robotCellY, mapInfo.resolution, mapInfo.width, mapInfo.height);
    robotCellX = int((robotPose.x-mapInfo.origin_x)/mapInfo.resolution);
    robotCellY = int((robotPose.y-mapInfo.origin_y)/mapInfo.resolution);
    const int updateRangeSize = 40;    
    for ( int j=robotCellY-updateRangeSize; j<robotCellY+updateRangeSize; j++ )
    {
        for ( int i=robotCellX-updateRangeSize; i<robotCellX+updateRangeSize; i++ )
        {
            index = i+j*mapInfo.width;
            if ( index >= 0 && index < mapInfo.width*mapInfo.height )
            {
                robotAroundPoint.x = (int(index%mapInfo.width)) * mapInfo.resolution + mapInfo.origin_x;
                robotAroundPoint.y = (int(index/mapInfo.width)) * mapInfo.resolution + mapInfo.origin_y;
                utils::coordination::convertCoord2CellCoord(
                    robotAroundPoint.x, robotAroundPoint.y, &ox, &oy);
                add.x = ox;
                add.y = oy;

                if ( pGridmap[index] == GRAY_LV_KNOWN_WALL ||
                    pGridmap[index] == GRAY_LV_UNKNOWN_AREA){
                    debugWallData.emplace_back(robotAroundPoint);
                    add.costValue = -1;                    
                }
                else{
                    add.costValue = 1;                    
                }

                wall.emplace_back(add);
            }
        }
    }

    

    DEBUG_PUB.publishCostMap(robotPose, debugWallData);

    // 정말 업데이트 해야 하나 ?
#if 0 // 검증 중.
    for (auto it = wall.begin(); it != wall.end();) {
        if (std::find(gridMapWall.begin(), gridMapWall.end(), *it) == gridMapWall.end()) {
            it = wall.erase(it);
            
        } else {
            ++it;
        }
    }
#endif
    CPthreadLockGuard lock(mutexGridMapWall);
    gridMapWall.clear();
    for (const auto &element : wall)
    {
        gridMapWall.push_back(element);
        bUpdatedGridMapWall = true;
    }

    tCellPoint start;   // 시작 셀 좌표
    utils::coordination::convertCoord2CellCoord(robotPose.x, robotPose.y, &start.x, &start.y);

    dstarData_.startPoint.x = (int)(start.x/MAP_DOWNSCALE_VALUE);
    dstarData_.startPoint.y = (int)(start.y/MAP_DOWNSCALE_VALUE);

    
    dstarData_.k_m += heuristic(dstarData_.s_last,dstarData_.startPoint);

    dstarData_.startPoint = calculateKey(dstarData_.startPoint);
    dstarData_.s_last  = dstarData_.startPoint;
}

/**
 * @brief 단순화 맵 복사 함수.
 * @param dest 
 * @param pInfo 
 * @return true 
 * @return false 
 */
bool CDstar::copyDstarGridMap(u8 *& dest, tGridmapInfo *pInfo)
{
    bool ret = false;
    if (dstarCellInfo.height == 0 || dstarCellInfo.width == 0 || dstarCellInfo.resolution == 0.0)
    {
        // ceblog(LOG_LV_ERROR, RED, " width : " << simplifyInfo.width << " height  : " << simplifyInfo.height << " resolution : " << simplifyInfo.resolution);/* don't copy! */
        // ceblog(LOG_LV_ERROR, RED, "gridmap copy error!!!! ");/* don't copy! */
    }
    else
    {
        *pInfo = dstarCellInfo;
        int size = pInfo->width * pInfo->height;        
        if (dest != nullptr)
        {
            delete[] dest;
        }

        dest = new u8[size];
        //reset Map : FREE_SPACE
        memset(dest, 0, size * sizeof(u8));

        tPoint wallPoint;
        int wallIndex;
        CPthreadLockGuard lock(mutexDstarCore);
        // ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "locked");
        for (const auto& entry : cellHash) 
        {
            if(entry.second.cost == -1)
            {
                wallPoint.x = (double)
                    (entry.first.x * MAP_DOWNSCALE_VALUE + (int)(MAP_DOWNSCALE_VALUE/2.0)) * CELL_RESOLUTUION + 
                    CELL_RESOLUTUION * (double)((CELL_X / 2) * -1);
                
                wallPoint.y = (double)(entry.first.y*MAP_DOWNSCALE_VALUE+(int)(MAP_DOWNSCALE_VALUE/2.0))*CELL_RESOLUTUION + CELL_RESOLUTUION*(double)((CELL_X / 2) * -1);
                wallIndex = int((wallPoint.x - pInfo->origin_x) / pInfo->resolution) + int((wallPoint.y - pInfo->origin_y) / pInfo->resolution) * pInfo->width;
                if (wallIndex > 0 && wallIndex < size)
                    dest[wallIndex] = 254;
            }
        }
        ret = true;
    }
    // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "lock free");
    return ret;
}


void CDstar::clearWall()
{
    CStopWatch __debug_sw;
    
    
    
    cellHash.clear();
    openHash.clear();
    
    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    tmp.cost = dstarData_.unknownCellCost;

    cellHash[dstarData_.goalPoint] = tmp;

    tmp.g = tmp.rhs = heuristic(dstarData_.startPoint,dstarData_.goalPoint);
    tmp.cost = dstarData_.unknownCellCost;
    cellHash[dstarData_.startPoint] = tmp;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief point의 후행 상태의 모든 이웃셀을 반환한다.
 *
 * @param point
 * @param negihborPoint
 */
void CDstar::getNextState(state point, list<state> &nextPoint)
{
    CStopWatch __debug_sw;
    nextPoint.clear();
    point.k.first = -1;
    point.k.second = -1;

    if (checkOccupied(point))
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return;
    }

    point.x += 1;
    nextPoint.push_front(point);
    point.y += 1;
    nextPoint.push_front(point);
    point.x -= 1;
    nextPoint.push_front(point);
    point.x -= 1;
    nextPoint.push_front(point);
    point.y -= 1;
    nextPoint.push_front(point);
    point.y -= 1;
    nextPoint.push_front(point);
    point.x += 1;
    nextPoint.push_front(point);
    point.x += 1;
    nextPoint.push_front(point);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief point의 선행 상태의 모든 이웃셀을 반환한다.
 *
 * @param point
 * @param negihborPoint
 */
void CDstar::getPerviousState(state point, list<state> &perviousPoint)
{
    CStopWatch __debug_sw;

    perviousPoint.clear();
    point.k.first = -1;
    point.k.second = -1;

    point.x += 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.y += 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.x -= 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.x -= 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.y -= 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.y -= 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.x += 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    point.x += 1;
    if (!checkOccupied(point))
        perviousPoint.push_front(point);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief   모든 셀의 cost를 업데이트 후 최단경로 계산       
 *          - 각 셀의 (cost + g) 비교 
 * @return true  
 * @return false 
 */
bool CDstar::replan()
{
    // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "called");
    CStopWatch __debug_sw;
    shortestPath.clear();

    int pathExistence = computeShortestPath();
    bool goalInfinite = isinf(getG(dstarData_.startPoint));

    // 목적지까지 경로가 존재 여부 판단.
    if (pathExistence < 0 || goalInfinite)
    { 
        TIME_CHECK_END(__debug_sw.getTime());
        return false;
    }

    list<state> n;
    list<state>::iterator i;

    state cur = dstarData_.startPoint;

    while (cur != dstarData_.goalPoint)
    {

        shortestPath.emplace_back(cur);
        getNextState(cur, n);

        if (n.empty())
        {
            eblog(LOG_LV, "replan false n.empty()\n");
            
            TIME_CHECK_END(__debug_sw.getTime());
            return false;
        }

        double cmin = INFINITY;
        double tmin;
        state smin = *n.begin(); // smin 초기화

        for (i = n.begin(); i != n.end(); ++i)
        {
            // if (occupied(*i)) continue;
            double val = cost(cur, *i);
            double val2 = euclideanCost(*i, dstarData_.goalPoint) + 
                euclideanCost(dstarData_.startPoint, *i); // (Euclidean) cost to goal + cost to pred
            val += getG(*i);

            if (checkValueClose(val, cmin))
            {
                if (tmin > val2)
                {
                    tmin = val2;
                    cmin = val;
                    smin = *i;
                }
            }
            else if (val < cmin)
            {
                tmin = val2;
                cmin = val;
                smin = *i;
            }
        }
        n.clear();
        cur = smin;
    }
    shortestPath.emplace_back(dstarData_.goalPoint);

    // ceblog(LOG_LV_NECESSARY, YELLOW, "replan 성공!");
    //ceblog(LOG_LV_PATHPLAN, BOLDGREEN, "lock free");
    TIME_CHECK_END(__debug_sw.getTime());
    return true;
}


bool CDstar::isRunFindNearestPath()
{
    return dstarData_.isRunFindNearestPath; 
}


void CDstar::findNearestPath(tPose robotPose, std::list<tPoint> searchingPoints, 
    int searchingCnt)
{
    dstarData_.isRunFindNearestPath = true;
    ceblog(LOG_LV_NECESSARY, BOLDMAGENTA, "경로계획을 시작합니다.");
    CPthreadLockGuard lock(mutexDstarCore);
    ceblog(LOG_LV_SYSDEBUG, BOLDYELLOW, "locked");    
    int checkCnt = 0;
    double minDis = 999, currentdis = 0;
    std::list<tPoint> rawPath;
    std::list<tPoint> selectedPath;
    std::list<tPoint> smoothShortPath;
    std::list<tPoint> RDPShortPath;
    tPose prePose = robotPose;
    tPoint currentSearchingPoint;
    tPoint nearestGoalPoint;
    // update goal, start
    if(searchingPoints.size() > 0)        
    {
        nearestGoalPoint = searchingPoints.front();
    }

    while (searchingPoints.size() > 0 )
    {
        // ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Searching Points : "<<
        //     BOLDYELLOW<<searchingPoints.size()<<BOLDBLACK<<"개 입니다.");
        currentSearchingPoint = searchingPoints.front();
        updateGoal(currentSearchingPoint);
        searchingPoints.pop_front();


        if(replan() == true)    // replan 성공
        {
            checkCnt++;
            rawPath.clear();
            getPath(rawPath, currentSearchingPoint);

            // 최단거리 목적지 뽑는 기능
            prePose = robotPose;
            currentdis=0;
            if(rawPath.size() >= 2)
            {
                for(tPoint path : rawPath)
                {
                    currentdis += utils::math::distanceTwoPoint(prePose,path);
                    prePose = tPose(path.x, path.y, 0);
                }

                if(currentdis < minDis) //hjkim231107 : 여기서 filter 되면 path가 안나올수 있음..예외 처리 필요
                {
                    selectedPath = rawPath;
                    nearestGoalPoint = currentSearchingPoint;
                    minDis = currentdis;
                }
            }
            else
            {
                selectedPath = rawPath;
                nearestGoalPoint = currentSearchingPoint;
            }

            if(checkCnt >= searchingCnt)
            {
                break;
            }
        }
    }
        
    // 하나라도 경로계획이 성공했는지 체크
    if(checkCnt != 0)
    {
#if 0        
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ################ 튜닝 전 ################# ");
        for (tPoint path : selectedPath)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " path : " << path.x << " , " << path.y );
        }
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ################################################### " );
#endif
        // smoothShortPath = makeSmoothPath(selectedPath);
        utils::path::ramerDouglasPeucker(selectedPath, 0.1, RDPShortPath);
        if(utils::math::distanceTwoPoint(RDPShortPath.front(), robotPose) < (EXPLORE_GOAL_MARGIN/0.8) )    
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 로봇과 경로가 근처라서 제거. 거리마진 ("<<(EXPLORE_GOAL_MARGIN/0.8)<<")");
            if(RDPShortPath.size()>1)
            {
                RDPShortPath.pop_front();
            }
        }
#if 0
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "################ 튜닝 후 ################# ");
        for (tPoint path : RDPShortPath)
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " path : " << path.x << " , " << path.y );
        }        
        ceblog(LOG_LV_NECESSARY, BOLDBLUE, " ################################################### " );
#endif
        // ceblog(LOG_LV_NECESSARY, BOLDBLUE, " D* 는 path 와 goal 을 획득 했습니다.");
        setNearPath(RDPShortPath);
        setFindNearSuccess(true);
    }
    else
    {
        setFindNearSuccess(false);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " 경로 계획 실패 - 경로가 나온 목적지가 없음. " );
    }      
    
    ceblog(LOG_LV_NECESSARY, BOLDMAGENTA, "경로계획을 종료합니다.");
    ceblog(LOG_LV_SYSDEBUG, BOLDGREEN, "lock free");
    dstarData_.isRunFindNearestPath = false;
}

std::list<tPoint> CDstar::makeSmoothPath(std::list<tPoint> rawPath)
{
    ceblog(LOG_LV_NECESSARY, BOLDGREEN, " ############### --- 경로를 부드럽게 만들기 시작합니다. --- ###############  ");
    if(rawPath.size() <= 2)
    {
        ceblog(LOG_LV_NECESSARY, GREEN, "경로가 3개 이하 - 부드럽게 하기 종료");
        return rawPath;
    }
    std::list<tPoint> copyRawPath;
    std::list<tPoint> smoothPath;
    copyRawPath = rawPath;

    double farSize = 0.05*MAP_DOWNSCALE_VALUE;
    tPoint prePoint;
    tPoint prePrePoint;

    smoothPath.emplace_back(copyRawPath.front());
    prePrePoint = copyRawPath.front();
    copyRawPath.pop_front();
    prePoint = copyRawPath.front();
    copyRawPath.pop_front();

    std::list<tPoint> neighbors;
    std::list<tPoint> sortedFirstPoint;
    std::list<tPoint> sortedSecondPoint;
    std::list<tPoint> tunedPoint;

    for(tPoint shortPathPoint : copyRawPath)
    {
        ceblog(LOG_LV_NECESSARY, GREEN, " 경로를 수정할 필요가 있습니다. 검사한 경로 : " << prePoint.x << " , " << prePoint.y);
        neighbors =getNeighboringCoordinates8(prePoint, farSize);

        ceblog(LOG_LV_NECESSARY, GREEN, " **************** 첫번 째 포인트 ****************");
        sortedFirstPoint =sortCoordinatesByDistance(prePoint, prePrePoint, neighbors);
        ceblog(LOG_LV_NECESSARY, GREEN, " **************** 두번 째 포인트 ****************");
        sortedSecondPoint =sortCoordinatesByDistance(prePoint, shortPathPoint, neighbors);

        ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 변경 전 좌표 : " << prePoint.x << " , " << prePoint.y);
        for( tPoint farthestPoint : findFarthestPoints(sortedFirstPoint,sortedSecondPoint))
        {
            ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 변경 후  좌표 : " << farthestPoint.x << " , " << farthestPoint.y);
            if(farthestPoint.x == smoothPath.front().x &&farthestPoint.y == smoothPath.front().y)
            {
                ceblog(LOG_LV_NECESSARY, BOLDGREEN, " 이전 좌표와 동일하여 패스.");
            }
            else
            {
                smoothPath.emplace_back(farthestPoint);
            }
        }
        prePoint = shortPathPoint;
        prePrePoint = smoothPath.back();
    }
    smoothPath.emplace_back(copyRawPath.back());
    return smoothPath;
}


// 4개의 포인트 중 가장 먼 두 개의 포인트를 찾는 함수
std::list<tPoint> CDstar::findFarthestPoints (const std::list<tPoint>& firstPoints, const std::list<tPoint>& secondPoints) 
{
    std::list<tPoint> resultPoints;
    if (firstPoints.size()==1) 
    {
        for (tPoint point : secondPoints)
        {
            if(firstPoints.front().x == point.x || firstPoints.front().y == point.y )
            {
                resultPoints.emplace_back(point);
            }
        }
    }
    else if (secondPoints.size()==1)
    {
        for (tPoint point : firstPoints)
        {
            if(secondPoints.front().x == point.x || secondPoints.front().y == point.y )
            {
                resultPoints.emplace_back(point);
            }
        }
    }
    else
    {
        double maxDistance = 0;
        tPoint firstPoint;
        tPoint secondPoint;

        // 모든 포인트 조합을 비교하여 가장 먼 두 포인트 찾기
        for (tPoint point1 : firstPoints)
        {
            for (tPoint point2 : secondPoints)
            {
                double distance = utils::math::distanceTwoPoint(point1, point2);
                if (distance > maxDistance) 
                {
                    maxDistance = distance;
                    firstPoint = point1;
                    secondPoint = point2;
                }
            }
        }

        resultPoints.emplace_back(firstPoint);
        resultPoints.emplace_back(secondPoint);
    }
    return resultPoints;
}
// 주위 8방의 좌표를 계산하는 함수
std::list<tPoint> CDstar::getNeighboringCoordinates8(tPoint centerXY, double size) 
{
    std::list<tPoint> neighboringCoordinates;

    // 8방향에 대한 상대적인 좌표 변화
    double dx[] = {-size, -size, -size, 0, 0, size, size, size};
    double dy[] = {-size, 0, size, -size, size, -size, 0, size};

    tPoint newXY;
    for (int i = 0; i < 8; ++i) {
        newXY.x = centerXY.x + dx[i];
        newXY.y = centerXY.y + dy[i];
        neighboringCoordinates.emplace_back(newXY);
    }

    return neighboringCoordinates;
}

// 주위 8방의 좌표를 계산하는 함수
std::list<tPoint> CDstar::getNeighboringCoordinates4(tPoint centerXY, double size)
{
    std::list<tPoint> neighboringCoordinates;

    // 8방향에 대한 상대적인 좌표 변화
    double dx[] = {-size,  0, 0, size};
    double dy[] = {0, -size, size, 0};

    tPoint newXY;
    for (int i = 0; i < 4; ++i) {
        newXY.x = centerXY.x + dx[i];
        newXY.y = centerXY.y + dy[i];
        neighboringCoordinates.emplace_back(newXY);
    }

    return neighboringCoordinates;
}


/**
 * @brief // 중심 좌표와 8개의 좌표와의 거리를 계산하고 정렬
 * 
 * @param center 
 * @param coordinates 
 * @return std::list<tPoint> 
 */
std::list<tPoint> CDstar::sortCoordinatesByDistance(const tPoint& centerPoint, const tPoint& comparePoint, const std::list<tPoint>& coordinates) 
{
    // 중심 좌표와 각 좌표와의 거리를 계산하여 벡터에 저장
    double minDistances;
    minDistances = 99;
    tPoint nearestPoint;
    std::list<tPoint> sortedCoordinates;

    for (const auto& coord : coordinates) 
    {
        double distance = utils::math::distanceTwoPoint(comparePoint, coord);
        if(minDistances > distance)
        {
            nearestPoint = coord;
            minDistances = distance;
        }
    }

    if(nearestPoint.x == centerPoint.x || nearestPoint.y == centerPoint.y )
    {
        sortedCoordinates.emplace_back(nearestPoint);
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW,  " 정렬된 좌표 : " << nearestPoint.x << " , " << nearestPoint.y );
        return sortedCoordinates;
    }

    tPoint firstCenter = tPoint();
    tPoint secondCenter = tPoint();
    double sumY =0.0;
    double sumX =0.0;
    for (const auto& coord : coordinates) 
    {
        if(nearestPoint.x == coord.x)
        {
            sumY += coord.y; 
        }

        if(nearestPoint.y == coord.y)
        {
            sumX += coord.x;
        }
    }

    firstCenter.x = nearestPoint.x;
    firstCenter.y = sumY/3;

    secondCenter.x = sumX/3;
    secondCenter.y = nearestPoint.y;

    // 정렬된 좌표를 반환
    sortedCoordinates.emplace_back(firstCenter);
    sortedCoordinates.emplace_back(secondCenter);
    for (const auto& info : sortedCoordinates) 
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW,  " 정렬된 좌표 : " << info.x << " , " << info.y );
    }

    return sortedCoordinates;
}