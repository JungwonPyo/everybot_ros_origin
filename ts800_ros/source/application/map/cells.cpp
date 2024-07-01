
#include <cmath>
#include <list>
#include <malloc.h>

#include "cells.h"
#include "utils.h"
#include "systemTool.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

const double gridOrgX = CELL_GRID_ORG_X;
const double gridOrgY = CELL_GRID_ORG_Y;
const int robotBound = 6; 

CCells::CCells()
{
    CStopWatch __debug_sw;

    cellSize = CELL_SIZE;
    cellWidth = CELL_X;
    cellHeight = CELL_Y;
    resolution = CELL_RESOLUTUION;
    bUpdate = false;

    cell push = {0};
    for (int i = 0; i < cellSize; i++)
        vCells[i].value = 0;        
    
    eblog(LOG_LV,  "");        
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CCells::~CCells()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 해당 idx 의 cell 값.
 * 
 * @param idx 
 * @return cell 
 */
cell CCells::getCell(unsigned int idx)
{
    CStopWatch __debug_sw;
    
    cell bRet;    
    if (CELL_SIZE <= idx){        
        eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
    }
    else
    {
        bRet = vCells[idx];
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}


bool CCells::copyCells(cell *& dest)
{
    CStopWatch __debug_sw;

    bool ret = false;
    int size = CELL_SIZE; 
    if (dest == NULL )
    {
        dest = new cell[size];
        memcpy(dest, vCells, sizeof(cell) * size);
        ret = true;
    }
    else
    {
        ret = false;
    }        
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

cell *CCells::getCellsPointer()
{
    return vCells;
}

unsigned int CCells::getCellSize()
{
    return cellSize;
}

unsigned int CCells::getWidth()
{
    return cellWidth;
}

unsigned int CCells::getHeight()
{
    return cellHeight;
}

double CCells::getResolution()
{
    return resolution;
}


/**
 * @brief cells 해당 index cell 을 업데이트 한다.
 * 
 * @param set 
 */
void CCells::set(int idx, cell set)
{
    CStopWatch __debug_sw;
    
    try
    {        
        if (CELL_SIZE <= idx){        
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else
        {
            vCells[idx] = set;
        }        
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitWall(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.wall = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitKnown(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.known = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}



void CCells::setBitClean(std::vector<int> vIdx, bool bSet)
{
    CStopWatch __debug_sw;

    try
    {
        for (int idx : vIdx) // Opening 조건을 만족하는 cell 업데이트
        {
            if (CELL_SIZE <= idx){
                eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
            }
            else{
                vCells[idx].b.clean = bSet;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitClean(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {        
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.clean = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitBumper(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {        
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.bumper = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitArea(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.area = bSet;
        }     
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitCliff(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {        
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.cliff = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitKnoll(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {   
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.knoll = bSet;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 index cell bit 를 업데이트 한다.
 * 
 * @param set 
 */
void CCells::setBitPath(int idx, bool bSet)
{
    CStopWatch __debug_sw;
    
    try
    {        
        if (CELL_SIZE <= idx){
            eblog(LOG_LV_ERROR,   "\n\n index terminate \n\n");
        }
        else{
            vCells[idx].b.path = bSet;
        }        
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cout << idx <<"   ";
        std::cerr << e.what() << '\n';
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cells 해당 좌표(m, m) 를 받아 path 데이터 저장
 * 
 * @param coordX (단위 m)
 * @param coordY (단위 m)
 * @param bSet 0:경로아님  1:경로
 */
void CCells::setBitPath(double coordX, double coordY, bool bSet)
{
    CStopWatch __debug_sw;
    
    int idx = utils::coordination::convertCoordi2Index(coordX, coordY, resolution, cellWidth, cellHeight);
    setBitPath(idx, bSet);
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief cells를 지운다.
 * 
 */
bool CCells::clear(void)
{
    CStopWatch __debug_sw;
    
    bool bRet = true;
    
    for (int i = 0; i < CELL_SIZE; i++)
    {
        vCells[i].value = 0;
    }
    
    bRet = true;
    
    
    TIME_CHECK_END(__debug_sw.getTime());
    return bRet;
}

void CCells::clearBitWall()
{
    CStopWatch __debug_sw;
        
    for (int i = 0; i < CELL_SIZE; i++){
        vCells[i].b.wall = false;
    }    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCells::clearBitClean()
{
    CStopWatch __debug_sw;
    
    for (int i = 0; i < CELL_SIZE; i++){
        vCells[i].b.clean = false;
    }    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCells::clearBitArea()
{
    CStopWatch __debug_sw;    
    
    for (int i = 0; i < CELL_SIZE; i++){
        vCells[i].b.area = false;
    }
    
    
    TIME_CHECK_END(__debug_sw.getTime());
}

bool CCells::isUpdate()
{
    return bUpdate;
}

void CCells::setUpdateState(bool set)
{
    CStopWatch __debug_sw;
    
    bUpdate = set;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

int CCells::getCellIdxfromGridmapIdx(int gridX, int gridY, int gridOrgX, int gridOrgY, int gridHeigh, int gridWidth)
{
    CStopWatch __debug_sw;
    
    int index = 0;

    int cellX = ( CELL_X / 2 )  + gridX - (gridWidth/2);
    int cellY = ( CELL_X / 2 )  + gridY - (gridHeigh/2);

    index = cellX + (cellY * CELL_X);

    if (index >= 0 && index < CELL_SIZE)
    {
        
    }
    else
    {
        eblog(LOG_LV, "index[ " << index << " ] error ");
        index = -1;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return index;
}

int CCells::getCellIdxfromGridmapIdx(int gridIndex, double gridOrgX, double gridOrgY, int gridHeigh, int gridWidth, double resolution)
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

int CCells::getCellIdxfromGridmapIdx(int gridIndex, int cellOrgX, int cellOrgY, u32 gridHeigh, u32 gridWidth)
{
    CStopWatch __debug_sw;
    
    int index = 0;    

    //gridIndex 를 x,y 값으로 변환
    int gridX = gridIndex % gridWidth;
    int gridY = gridIndex / gridWidth;
    
    int cellX = ( CELL_X / 2 )  + (cellOrgX + gridX);
    int cellY = ( CELL_Y / 2 )  - (cellOrgY + gridY);

    index = cellX + (cellY * CELL_X);

    TIME_CHECK_END(__debug_sw.getTime());
    return index;
}

void CCells::updateCells(s8 *pOrgGrid, tGridmapInfo info)
{
    CStopWatch __debug_sw;

    int gridmapSize = info.width * info.height;        
    const int occupy_threshold = 70;    // 90%
    const int emptyThreshold = 10;      // 10% 이하만 빈 영역으로 처리 22.06.22 jspark
    int8_t occupy = -1;
    int cellIndex = 0;

    int convertOrgX = static_cast<int>((info.origin_x / CELL_RESOLUTUION) + 0.5); //반올림
    int convertOrgY = static_cast<int>((info.origin_y / CELL_RESOLUTUION) + 0.5); //반올림

    double __debugTimeCell = get_system_time();
    try
    {
        if (CELL_SIZE > gridmapSize && info.width > 0)
        {
            for(int i=0; i<gridmapSize; i++)
            {                
                occupy = pOrgGrid[i];                
                cellIndex = getCellIdxfromGridmapIdx(i, convertOrgX, convertOrgY, info.height, info.width );

                if (cellIndex >= 0 && cellIndex < CELL_SIZE)
                {
                    if ( occupy <= emptyThreshold  && occupy >= 0) // 10% 이하만 빈 영역으로 처리 22.06.22 jspark
                    {
                        vCells[cellIndex].b.known = true;
                        vCells[cellIndex].b.wall = false;
                    }                         
                    else if( occupy >= occupy_threshold ) //임계점 이상이면 벽으로 간주 , 임계점 기준은 감이다.
                    {               
                        vCells[cellIndex].b.known = true;
                        vCells[cellIndex].b.wall = true;
                    }                    
                    else
                    {
                        //occupy 값이 -1 인것 포함.
                        vCells[cellIndex].b.known = false;// 10% ~ 70% 사이의 값은 unknown 처리!!   
                    }
                }
                else
                {
                    eblog(LOG_LV_ERROR, "index[ "<<cellIndex<< " ] error ");                    
                }
            }

            bUpdate = true;            
        }
        else
        {
            eblog(LOG_LV_ERROR, "clean cell map update fail  - grid size : " << gridmapSize << " ,  cell size : " << CELL_SIZE);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
    if ( int(get_system_time(__debugTimeCell)*1000) > 10 )
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "Warning!! \tGrid\tupdate time\t\t:"<<int(get_system_time(__debugTimeCell)*1000)<<" ms > 10ms");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CCells::updateCellsBySimplifyMap(u8 *pOrgGrid, tGridmapInfo info)
{
    CStopWatch __debug_sw;
    
    

    int gridmapSize = info.width * info.height;        
    const int occupy_threshold = 70;    // 90%
    const int emptyThreshold = 10;      // 10% 이하만 빈 영역으로 처리 22.06.22 jspark
    int8_t occupy = -1;
    int cellIndex = 0;

    int convertOrgX = static_cast<int>((info.origin_x / CELL_RESOLUTUION) + 0.5); //반올림
    int convertOrgY = static_cast<int>((info.origin_y / CELL_RESOLUTUION) + 0.5); //반올림

    double __debugTimeCell = get_system_time();
    try
    {
        if (CELL_SIZE > gridmapSize && info.width > 0)
        {
            for(int i=0; i<gridmapSize; i++)
            {                
                occupy = pOrgGrid[i];                
                cellIndex = getCellIdxfromGridmapIdx(i, convertOrgX, convertOrgY, info.height, info.width );
                if (cellIndex >= 0 && cellIndex < CELL_SIZE)
                {                    
                    if ( occupy == GRAY_LV_KNOWN_WALL ||
                        occupy == GRAY_LV_UNKNOWN_WALL)
                    {
                        vCells[cellIndex].b.wall = true;
                        vCells[cellIndex].b.known = true;
                    }
                    else if( occupy == GRAY_LV_KNOWN_AREA )
                    {               
                        vCells[cellIndex].b.wall = false;
                        vCells[cellIndex].b.known = true;
                    }
                    else if( occupy == GRAY_LV_UNKNOWN_AREA )
                    {
                        vCells[cellIndex].b.wall = false;
                        vCells[cellIndex].b.known = false;
                    }                    
                    else
                    {   
                        vCells[cellIndex].b.wall = false;                     
                        vCells[cellIndex].b.known = false;
                    }
                }
                else
                {
                    eblog(LOG_LV_ERROR, "index[ "<<cellIndex<< " ] error ");                    
                }
            }

            bUpdate = true;            
        }
        else
        {
            eblog(LOG_LV_ERROR, "clean cell map update fail  - grid size : " << gridmapSize << " ,  cell size : " << CELL_SIZE);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
        std::cerr << e.what() << '\n';
    }
#if 0
    if ( int(get_system_time(__debugTimeCell)*1000) > 10 )
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "Warning!! \tGrid\tupdate time\t\t:"<<int(get_system_time(__debugTimeCell)*1000)<<" ms > 10ms");
    }
#endif
    TIME_CHECK_END(__debug_sw.getTime());
}




/**
 * @brief gridmap (0, 0) 기준 실수 좌표계에서
 * cells의 1차 배열 index 변환 함수
 * 
 * @param point 단위 (m, m)
 * @return int Cell index
 */
int CCells::coordinate2index(tPoint point)
{
    CStopWatch __debug_sw;

    int index = 0;
    double resolution = CELL_RESOLUTUION;    
    double width = CELL_X;    

    int calc1 = (int) ( ( point.x - gridOrgX ) / resolution + 0.5);
    int calc2 = (int) ( ( gridOrgY - point.y ) / resolution + 0.5);
    index = calc1 + (calc2 * width); 
    
    if (index >= 0 && index < CELL_SIZE)
    {
        
    }
    else
    {
        eblog(LOG_LV_ERROR, "index[ " << index << " ] error ");
        index = -1;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return index;
}


/**
 * @brief 주변 팔방의 마진만큼의 인덱스 번호를 획득
 * 
 * @param index 
 * @param margin 
 * @return std::list<int> 
 */
std::list<int> CCells::getNeighboringIndices(int index, int margin) {    

    int width = CELL_X;
    int height = CELL_Y;
    int x = index % width;
    int y = index / width;
    

    std::list<int> indices;
    for (int dx = -margin; dx <= margin; ++dx) {
        for (int dy = -margin; dy <= margin; ++dy) {
            if (dx == 0 && dy == 0) continue; // 중심 좌표는 제외
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) { // 배열 범위를 벗어나지 않는 경우에만 추가
                indices.push_back(ny * width + nx);  // 2D 좌표를 1D 인덱스로 변환
            }
        }
    }
    return indices;
}

/**
 * @brief SLAM trajectory 정보를 clean 으로 업데이트
 * 
 * @param trajectory 
 */
void CCells::updateCleaned(tPoint oldPoint, tPoint newPoint, unsigned int cleanBound)
{
    CStopWatch __debug_sw;

    int margin = cleanBound/2;  //기준점으로 앞뒤로 하기때문에 절반만 입력
    
    // 라인을 그리는 함수에서 얻은 각 포인트 주변에 더 많은 포인트를 추가하여 라인을 두껍게 만듭니다.
    for (tPoint linePoint : generatePointsWithBresenham(oldPoint, newPoint, 0.05)) {
        int idx = coordinate2index(linePoint);

        std::list<int> idxList = getNeighboringIndices(idx, margin);

        for (auto i : idxList){
            if (i > 0)
                setBitClean(i, true);
        }
        // ceblog(LOG_LV_NECESSARY, MAGENTA, "called");
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

void CCells::updateCliff(tPoint set)
{
    CStopWatch __debug_sw;
    
    int idx = coordinate2index(set);

    std::list<int> idxList = getNeighboringIndices(idx, 3);

    for (auto i : idxList){
            if (i > 0)
                setBitCliff(i, true);
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 해당 좌표의 cell 값을 리턴
 * 
 * @param point 
 * @return cell 
 */
cell CCells::checkCellValue(tPoint point)
{
    return getCell(coordinate2index(point));
}


bool CCells::checkCleanBound(tPoint point)
{
    bool bRet = false;
    double avg = 0;
    const double minimumCleanRatio = 0.8; //청소 cell 비율 
    int cleanCnt = 0;
    int boundSize = robotBound / 2 ;  // 로봇 크기 의 절반을 입력해야 로봇 크기
    // 입력 받은 점으로 부터 주변의 인덱싱 구하기
    int idx = coordinate2index(point);
    std::list<int> idxList = getNeighboringIndices(idx, boundSize);

    // 인덱싱 값 카운팅
    for (auto i : idxList){
        if (i > 0){
            if (getCell(i).b.clean){
                cleanCnt++;
            }
        }            
    }

    //결과 도출
    int size = idxList.size();
    
    if (size>0){
        avg = static_cast<double>(cleanCnt) / static_cast<double>(idxList.size());
        if (avg > minimumCleanRatio){
            ceblog(LOG_LV_NECESSARY, MAGENTA, "size : "<<size
                <<" , avg : "<<avg <<"% 청소됨.");
            bRet = true;
        }
    }
    else{
        bRet = false;
    }

    return bRet;
}


/**
 * @brief 옆으로 이동할때 가도 되는지 확인.
 * 
 * @param point 
 * @param type 
 * @return true wall 또는 청소 한 지역
 * @return false wall 과 청소한 영역이 아님
 */
bool CCells::checkWallAndCleanBound(tPoint point)
{
    bool bRet = false , bCeanBound = false, bWallBound = false;
    double avg = 0;
    const double minimumCleanRatio = 0.4; //청소 cell 비율 
    const double minimumWallRatio = 0.1; //벽 cell 비율 
    int cleanCnt = 0, wallCnt = 0;
    int boundSize = robotBound / 2;  // 로봇 크기 의 절반을 입력해야 로봇 크기
    // 입력 받은 점으로 부터 주변의 인덱싱 구하기
    int idx = coordinate2index(point);
    std::list<int> idxList = getNeighboringIndices(idx, boundSize);

    // 인덱싱 값 카운팅
    for (auto i : idxList){
        if (i > 0){
            if (getCell(i).b.clean){
                cleanCnt++;
            }
            if (getCell(i).b.wall){
                wallCnt++;
            } 
        }            
    }

    //결과 도출
    int size = idxList.size();
    
    if (size>0){
        avg = static_cast<double>(cleanCnt) / static_cast<double>(idxList.size());
        if (avg > minimumCleanRatio){
        // if (cleanCnt){
            // ceblog(LOG_LV_NECESSARY, MAGENTA, "size : "<<size<<" , avg : "<<avg <<"% 청소됨.");
            bCeanBound = true;
        }


        avg = static_cast<double>(wallCnt) / static_cast<double>(idxList.size());
        // if (avg > minimumWallRatio){
        if (wallCnt){
            // ceblog(LOG_LV_NECESSARY, MAGENTA, "size : "<<size<<" , avg : "<<avg <<"% 벽.");
            bWallBound = true;
        }
    }
    else{
        bRet = false;
    }

    if (bCeanBound || bWallBound)
        bRet = true;

    return bRet;
}
