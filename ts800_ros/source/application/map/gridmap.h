/**
 * @file gridmap.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <atomic>
#include "ebtypedef.h"
#include "define.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <pthread.h>
#include <thread>
#include "opencv2/opencv.hpp"
#include <vector>
#include "coordinate.h"

/**
 * @brief slam 에서 나오는 트라젝토리 관리. 
 *        -외부 참조자는 set을 통해서만 업데이트 가능
 * 
 */

typedef struct _tGridmapInfo
{
    _tGridmapInfo() : resolution{0.0}, width{0}, height{0}, origin_x{0.0}, origin_y{0.0} {}
    _tGridmapInfo(float _resolution, u32 _width, u32 _height, float _origin_x, float _origin_y) 
        : resolution{_resolution}, width{_width}, height{_height}, origin_x{_origin_x}, origin_y{_origin_y} {}

    float resolution;   // [m/cell]
    u32 width;          // [cell]
    u32 height;         // [cell]
    float origin_x;     // cell (0,0) 기준 map 중심 위치 [m]
    float origin_y;
}tGridmapInfo;


class CGridMap {
private:

protected:
    bool bIsInit;
    bool bIsUpdate; //map 이 업데이트 됐나?
    tGridmapInfo info;  // Gridmap 원본 메타정보
    
public:
    CGridMap();
	~CGridMap();

	virtual void set(s8* setGrid, tGridmapInfo setInfo) = 0;

    tGridmapInfo getInfo();
    bool isInit();
    bool isUpdate();
    void setUpdateState(bool set);    
};

class CliffPoint
{
public:
    CliffPoint(){};
    ~CliffPoint(){};
    
    void updateCliff(tPoint set){        
        cliff.push_back(set);        
    }

    void clearCliff(){
        cliff.clear();
    }

    std::list<tPoint> getCliff(){
        return cliff;
    }    
private:
    std::list<tPoint> cliff;
};


class CSimplifyGridMap : public CGridMap
{
private:
    u8 *pSimplifyGridMap; //grid map 을 단순화 시킴
    s8* pSetGrid;
    cv::Mat simplifyMatMap;
    bool isSetGrid;
    std::string mapDataStorageDir;
    bool isRunImgProc;
    std::chrono::milliseconds default_priod;
    pthread_mutex_t mutex;
    pthread_t thImgProc;
    static void* threadImgProcWrap(void* arg)
    {
        CSimplifyGridMap* mySimplify = static_cast<CSimplifyGridMap*>(arg);
        int result = pthread_setname_np(pthread_self(), THREAD_ID_IMAGE_PROC);
        mySimplify->threadImgProc();
    }
    void threadImgProc();

    std::vector<s8> awsMap;
    //s8 awsMap[8192];

public:
    CSimplifyGridMap();
	~CSimplifyGridMap();
    CliffPoint cliffPoint;
    void set(s8* setGrid, tGridmapInfo setInfo) override;
    cv::Mat getSimplifyMatMap();
    s8* getGridMap();
    std::vector<s8> getAwsMap();
#if 0
    bool copyGridMap(u8 *& dest, tGridmapInfo *pInfo);
#endif 
    bool copySavedGridMapFromYAML(u8 *& dest, tGridmapInfo *pInfo);
    bool isExistSavedMap();
    void simplifyGridMapLock();
    void simplifyGridMapUnLock();
    u8 *getSimplifyGridMapPtr();
    tGridmapInfo getSimplifyGridMapInfo();
    bool isValidSimplifyGridMap();
};

class CRawGridMap : public CGridMap
{
private:    
    s8 *pGridMapRaw;
    pthread_mutex_t mutex;
    
public:
	CRawGridMap();
	~CRawGridMap();
    void set(s8* setGrid, tGridmapInfo setInfo) override; 
    bool copyGridMap(s8 *& dest, tGridmapInfo *pInfo);
};
