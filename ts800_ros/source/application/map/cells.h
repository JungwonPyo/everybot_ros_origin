/**
 * @file cells.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

//#include <iostream>
#include <mutex>
#include <vector>

#include "commonStruct.h"
#include "gridmap.h"

class CCells {
private:
	std::atomic_flag atomicFlag = ATOMIC_FLAG_INIT;

    cell vCells[CELL_SIZE];
    //std::vector <cell> vCells;


private:
    unsigned int cellSize;
    unsigned int cellWidth;
    unsigned int cellHeight;
    double resolution;
    bool bUpdate;

public:	

	CCells();
	~CCells();
    bool copyCells(cell *& dest);    
    cell getCell(unsigned int idx);
    cell *getCellsPointer();    
    unsigned int getCellSize();
    unsigned int getWidth();
    unsigned int getHeight();
    double getResolution();
	void set(int idx, cell set);
	void setBitWall(int idx, bool bSet);
	void setBitKnown(int idx, bool bSet);
    void setBitClean(int idx, bool bSet);
    void setBitClean(std::vector<int> vIdx, bool bSet);
	void setBitBumper(int idx, bool bSet);
    void setBitArea(int idx, bool bSet);
	void setBitCliff(int idx, bool bSet);
	void setBitKnoll(int idx, bool bSet);
	void setBitPath(int idx, bool bSet);
    void setBitPath(double coordX, double coordY, bool bSet);
	bool clear(void);
    void clearBitWall();
    void clearBitClean();
    void clearBitArea();
    bool isUpdate();
    void setUpdateState(bool set);
    void updateCleaned(tPoint oldPoint, tPoint newPoint, unsigned int cleanBound);
    bool checkCleanBound(tPoint point);
    bool checkWallAndCleanBound(tPoint point);

    //gridmap index 를 cell index로 변환.
    int getCellIdxfromGridmapIdx(int gridIndex, double gridOrgX, double gridOrgY, int gridHeigh, int gridWidth, double resolution = 0.05);
    int getCellIdxfromGridmapIdx(int gridIndex, int cellOrgX, int cellOrgY, u32 gridHeigh, u32 gridWidth);
    int getCellIdxfromGridmapIdx(int gridX, int gridY, int gridOrgX, int gridOrgY, int gridHeigh, int gridWidth);
    void updateCells(s8 *pGridmapRaw, tGridmapInfo info);
    void updateCellsBySimplifyMap(u8 *pOrgGrid, tGridmapInfo info);
    int coordinate2index(tPoint point);
    std::list<int> getNeighboringIndices(int index, int margin);
    void updateCliff(tPoint set);
    cell checkCellValue(tPoint point);
    
};
