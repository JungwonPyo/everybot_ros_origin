
#include "obstaclemap.h"

#include "eblog.h"
#include "math.h"
#include "utils.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

const int LIFE_MAX = 1; // 50 -> 1
const int TERM_THRESHOLD = 1; // 20 -> 3
const double DIST_THRESHOLD = 0.50; // 0.5 -> 0.35

/**
 * @brief Construct a new CObstacleMapCell::CObstacleMapCell object
 * 
 */
CObstacleMapCell::CObstacleMapCell()
{
    bDetected   = false;
    life        = 0;
    term        = 0;
    id          = 0;
}
/**
 * @brief Destroy the CObstacleMapCell::CObstacleMapCell object
 * 
 */
CObstacleMapCell::~CObstacleMapCell()
{

}


/**
 * @brief cell 에 생명을 넣는다.
 *  라이다 포인트가 cell 안에 들어온 경우. * 
 */
void CObstacleMapCell::lifeUp()
{
    CStopWatch __debug_sw;

    life = LIFE_MAX;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 이 함수는 특정 주기마다 업데이트 해야한다.
 * 예를 들어 라이더 데이터 업데이트 주기라던지...
 * 
 */
void CObstacleMapCell::update()
{
    CStopWatch __debug_sw;

    //생명을 감소 시킨다.
    life--;

    //생명이 계속 유지되면 term 을 증가 시킨다.
    if (life > 0){
        term++;
    }
    else{   //생명 유지를 못하면 모두 초기화 시킨다.
        clear();
    }

    //term이 특정 값 이상이면 detected 된다.
    if (term > TERM_THRESHOLD){
        //std::cout<<"id : " << id << " , dist : "<< dist<< std::endl;
        bDetected = true;
    }

 #if 0   
    if (id >0 && id <5)
        std::cout<<"id : " << id <<" , life: "<< life 
        << " , term : "<<term <<" , bDetected : "<<bDetected<<" , dist : "<< dist<< std::endl;
#endif

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 데이터를 모두 초기화 한다.
 * 
 */
void CObstacleMapCell::clear()
{
    CStopWatch __debug_sw;

    bDetected = false;
    life = 0;
    term = 0;
    
    TIME_CHECK_END(__debug_sw.getTime());
}


CObstaclemap::CObstaclemap(const CObstaclemap& other) = default; // Copy constructor
CObstaclemap& CObstaclemap::operator=(const CObstaclemap& other) = default; // Assignment operator

CObstaclemap::CObstaclemap()
{
    CStopWatch __debug_sw;

    cellObstacle = new cell_obstacle[CELL_OBS_SIZE];
    
    eblog(LOG_LV,  "new cellObstacle");
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CObstaclemap::~CObstaclemap()
{
    CStopWatch __debug_sw;

    delete cellObstacle;
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}
    
//lidar
void CObstaclemap::setCellFromLidar(double *LidarDist)
{
    CStopWatch __debug_sw;
    
    double theta = 0.0;
    double dist = 0.0;
    double x = 0, y = 0;
    int cellIdx = 0;

    for (int i = 0; i < LIDAR_DIST_BUFF_SIZE; i++)
    {
        dist = LidarDist[i];
        if(dist != 0.0)
        {
            theta = DEG2RAD(i);
            x = cos(theta) * dist;
            y = sin(theta) * dist;
            
            // 라이다 중심 -> 로봇 중심
            if( x != 0.0 || y != 0.0)
            {
                x = x + CELL_MOVE_OFFSET;
            }

            setCell(x, y, CELL_OBS_TYPE_LIDAR);            
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

//cliff
void CObstaclemap::setCellFromCliff(bool cliffLeft, bool cliffRight)
{
    CStopWatch __debug_sw;
    
    double buffer   = 0.0;
    double x        = 0.2;  // cliff set할 기준 x좌표
    double y        = 0.2;  // cliff set할 기준 y좌표
    double gap      = 0.05;
    int cellIdx     = 0;

    for( int i = 0; i < 2; i++)
    {
        if(cliffLeft)
        {
            setCell(x, y + i*gap, CELL_OBS_TYPE_CLIFF);
        }
        
        if(cliffRight)
        {
            setCell(x, -(y + i*gap), CELL_OBS_TYPE_CLIFF);
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

void CObstaclemap::clearCell()
{
    CStopWatch __debug_sw;

    memset(cellObstacle,0,CELL_OBS_SIZE*sizeof(cell_obstacle));
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CObstaclemap::cellsUpdate()
{
    CStopWatch __debug_sw;

    for (int y = 0; y < CELL_OBS_HEIGHT; y++)
    {
        for (int x = 0; x < CELL_OBS_WIDTH; x++)
        {
            cells[x][y].update();
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 라이다 포인트를 cell 에 업데이트 한다.
 * 
 * @param set 
 */
void CObstaclemap::lidarUpdate(sensor_msgs::LaserScan set)
{
    CStopWatch __debug_sw;

    double rad = 0.0, distance = 0.0, x = 0.0, y = 0.0;
    int ix = 0, iy = 0;

    //각 셀의 생명을 업데이트 (주기적 호출)
    cellsUpdate();
        
    for (size_t i = 0; i < set.ranges.size(); ++i) {
        rad = set.angle_min + set.angle_increment * i;
        distance = set.ranges[i];

        //x,y  좌표로 변환
        x = distance * cos(rad);
        y = distance * sin(rad);

        //int 형 index 로 변환
        ix = (int)(x + 0.5);    //반올림
        iy = (int)(y + 0.5);    //반올림

        //cells 크기 안에 들어오면 업데이트
        if (ix < CELL_OBS_WIDTH && iy < CELL_OBS_HEIGHT){
            cells[ix][iy].lifeUp();
        }        
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CObstaclemap::checkCellIndex(int index)
{
    CStopWatch __debug_sw;

    bool ret = false;

    if (index >=0 && index < CELL_OBS_SIZE)
    {
        ret = true;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

void CObstaclemap::setCell(double x, double y, E_CELL_OBSTACLE_TYPE type)
{
    CStopWatch __debug_sw;

    int cellIdx = utils::coordination::convertCoordi2Index(x,y,CELL_OBS_RESOLUTION,CELL_OBS_WIDTH,CELL_OBS_HEIGHT);

    if (checkCellIndex(cellIdx))
    {
        switch (type)
        {
        case CELL_OBS_TYPE_LIDAR :
            cellObstacle[cellIdx].b.lidar = 1;
            break;
        case CELL_OBS_TYPE_CLIFF:
            cellObstacle[cellIdx].b.cliff = 1;
            break;
        case CELL_OBS_TYPE_BUMPER:
            cellObstacle[cellIdx].b.bumper = 1;
            break;
        case CELL_OBS_TYPE_A:
            cellObstacle[cellIdx].b.a = 1;
            break;
        case CELL_OBS_TYPE_B:
            cellObstacle[cellIdx].b.b = 1;
            break;
        case CELL_OBS_TYPE_C:
            cellObstacle[cellIdx].b.c = 1;
            break;
        case CELL_OBS_TYPE_D:
            cellObstacle[cellIdx].b.d = 1;
            break;
        case CELL_OBS_TYPE_E:
            cellObstacle[cellIdx].b.e = 1;
            break;
        
        default:
            break;
        }
    }
    else
    {
        eblog(LOG_LV_ERROR, "index[ " << cellIdx << " ] error ");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

cell_obstacle* CObstaclemap::getcellsObstaclePointer()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
    return cellObstacle;
}