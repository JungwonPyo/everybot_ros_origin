#include "gridmap.h"

#include "eblog.h"
#include<malloc.h>
#include <yaml-cpp/yaml.h>
#include "imgProcessor.h"
#include "systemTool.h"
#include "debugCtr.h"
#include "motionPlanner/motionPlanner.h"
#include "subTask.h"
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 20.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CGridMap::CGridMap()
{
    CStopWatch debugSw;
    
    bIsInit = false;

    info.resolution = CELL_RESOLUTUION;
    info.width = 0;
    info.height = 0;
    info.origin_x = 0.0;
    info.origin_y = 0.0;
    

    bIsUpdate = false;

    eblog(LOG_LV,   "create");
    
    TIME_CHECK_END(debugSw.getTime());
}

CGridMap::~CGridMap()
{
    CStopWatch debugSw;
    
    eblog(LOG_LV,   "destroy");
    
    TIME_CHECK_END(debugSw.getTime());
}

bool CGridMap::isInit(){
    return bIsInit;
}

bool CGridMap::isUpdate(){
    return bIsUpdate;
}

void CGridMap::setUpdateState(bool set)
{
    CStopWatch debugSw;
    
    bIsUpdate = set;
    
    TIME_CHECK_END(debugSw.getTime());
}


tGridmapInfo CGridMap::getInfo()
{
    return info;
}

CSimplifyGridMap::CSimplifyGridMap()
{
    CStopWatch debugSw;
    
    pSimplifyGridMap = NULL;
    pSetGrid = NULL;
    mapDataStorageDir = "/home/ebot/map/";

    default_priod = std::chrono::milliseconds(200);

    isRunImgProc=true;
    pthread_mutex_init(&mutex, nullptr);
    pthread_create(&thImgProc, nullptr, &CSimplifyGridMap::threadImgProcWrap, this);
    
    TIME_CHECK_END(debugSw.getTime());
}

CSimplifyGridMap::~CSimplifyGridMap()
{
    CStopWatch debugSw;

    isRunImgProc = false;
    pthread_join(thImgProc, nullptr);
    pthread_mutex_destroy(&mutex);
    TIME_CHECK_END(debugSw.getTime());
}
/**
 * @brief 심플그리드맵 이미지 처리
 * 이미지 처리 속도개선을 위해 스레드 처리
 * 
 */
void CSimplifyGridMap::threadImgProc()
{    
    while (isRunImgProc)
    {
        CStopWatch debugSw;

        try
        {
            if (isSetGrid)
            {
                auto start_time = std::chrono::high_resolution_clock::now();

                simplifyGridMapLock();
                
                int size = info.width*info.height;                
                s8 * pGridTemp = new s8[size];
                
                // y축 반전 memcpy
                for( int h=0; h<info.height; h++)
                {
                    int srcOffset = h*info.width;
                    int destOffset = (info.height-h-1)*info.width;
                    std::memcpy(pGridTemp + destOffset, pSetGrid + srcOffset, sizeof(s8)*info.width);
                }

                simplifyGridMapUnLock();

                //이미지 연산을 하기위해 생성.
                CImgProcessor imgProc;
                cv::Mat img;

                if (SUB_TASK.cleanPlan.isSetupNogoZoneDocking()){
                    std::list<tPoint> dockingzoneReal = SUB_TASK.cleanPlan.getNoGoZoneDocking();
                
                    std::list<cv::Point> dockingzonePixel;
                    for (const auto& element : dockingzoneReal) {
                        cv::Point pt;                    
                        pt.x = (element.x - info.origin_x) / info.resolution;
                        pt.y = (element.y - info.origin_y) / info.resolution;
                        pt.y = info.height - pt.y;  // ros 와 cv 는 y 축이 다르다.
                        dockingzonePixel.push_back(pt);
                    }

                    //raw grid data 로 벽을 단순화 시킴.                
                    //img = imgProc.wallLineFit(pGridTemp, info.width, info.height, dockingzonePixel);
                    img = imgProc.linemap(pGridTemp, info.width, info.height,dockingzonePixel);
                }
                else{
                    // img = imgProc.wallLineFit(pGridTemp, info.width, info.height);
                    img = imgProc.linemap(pGridTemp, info.width, info.height);
                }
                
                //img = imgProc.linemap(pGridTemp, info.width, info.height,dockingzonePixel);                

                // cliff 데이터가 있을경우 클리프 영역을 칠하여 탐색이 종료 되도록.
                std::list<tPoint> cliffpts = cliffPoint.getCliff();
                if (cliffpts.size() > 0){
                    ceblog(LOG_LV_ERROR, GREEN, "cliff size " << cliffpts.size());
                    std::list<cv::Point> pixelPoint;
                    for (const auto& element : cliffpts) {
                        cv::Point pt;                    
                        pt.x = (element.x - info.origin_x) / info.resolution;
                        pt.y = (element.y - info.origin_y) / info.resolution;
                        pt.y = info.height - pt.y;  // ros 와 cv 는 y 축이 다르다.
                        pixelPoint.push_back(pt);
                    }

                    imgProc.drawCliffPoint(img, pixelPoint);
                }
                
                simplifyMatMap = img.clone();

                info.width = img.cols;
                info.height = img.rows;

                //Mat 이미지를 unsigned char 벡터로 변환.
                std::vector<unsigned char> vGrid = imgProc.convertMat2GridMap(img);
                
                simplifyGridMapLock();
                
                //pSimplifyGridMap 데이터 유무 확인 및 초기화.
                if (pSimplifyGridMap != nullptr){
                    delete[] pSimplifyGridMap;
                    pSimplifyGridMap = nullptr;
                }

                //pSimplifyGridMap 새롭게 사이즈 생성
                pSimplifyGridMap = new u8[size];

                // 벡터를 포인터로 변환.
                memcpy(pSimplifyGridMap, vGrid.data(), size);
                simplifyGridMapUnLock();
                
                delete []pGridTemp;

                setUpdateState(true);
                bIsInit = true;
                isSetGrid = false;

                //D* 업데이트
                PATH_PLANNER->doUpdateDstarMap();
                //wave 업데이트
                SUB_TASK.waveFrontier.doUpdateWaveFrontier();

                // 디버깅.
                DEBUG_PUB.publishSimplifyMapImage(simplifyMatMap, 3);

                auto end_time = std::chrono::high_resolution_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

                std::this_thread::sleep_for(elapsed_time);
            }
        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(debugSw.getTime()); // max = 700ms over
        std::this_thread::sleep_for(std::chrono::milliseconds(default_priod));
        DEBUG_CTR.isAliveImgProc.set(true);
    }
}


/**
 * @brief 저장된 grid map 을 copy 받는다.
 * 
 * @param dest 메모리 할당 안된 포인터
 * @return bool 카피 성공 true, 실패 false
 */
bool CSimplifyGridMap::copySavedGridMapFromYAML(u8 *& dest, tGridmapInfo *pInfo)
{
    CStopWatch debugSw;
    
    bool ret = false;
    if (dest == nullptr)
    {
        int tick_lock = SYSTEM_TOOL.getSystemTick();        
        simplifyGridMapLock();
        tick_lock = SYSTEM_TOOL.getSystemTick() - tick_lock;
        if (tick_lock>0)
            ceblog(LOG_LV_NECESSARY, YELLOW, " copySavedGridMapFromYAML lock tick : "<< tick_lock);

        std::ifstream file(mapDataStorageDir + "savedMapData.yaml");
        if(file.good())
        {
            YAML::Node mapNode = YAML::LoadFile(mapDataStorageDir + "savedMapData.yaml");
            
            s8*  pGridmap;
            tGridmapInfo info;
            if (mapNode["resolution"])   info.resolution  = mapNode["resolution"].as<double>();
            if (mapNode["width"])        info.width       = mapNode["width"].as<double>();
            if (mapNode["height"])       info.height      = mapNode["height"].as<double>();
            if (mapNode["origin_x"])     info.origin_x    = mapNode["origin_x"].as<double>();
            if (mapNode["origin_y"])     info.origin_y    = mapNode["origin_y"].as<double>();
            *pInfo = info;
            
            if (mapNode["value"]) 
            {
                dest = new u8[info.height * info.width];
                std::string charString = mapNode["value"].as<std::string>();
                memcpy(dest, charString.c_str(), sizeof(u8) * info.height * info.width);
                ret = true;
            }
            mapNode.reset();
            file.close();
            //eblog(LOG_LV, " The map data has been imported from the YAML file.");
        }
        else
        {
            //eblog(LOG_LV, " YAML File does not exist.  ");
        }
        simplifyGridMapUnLock();
    }

    TIME_CHECK_END(debugSw.getTime());
    return ret;
}


/**
 * @brief 저장된 맵 존재 확인 함수
 * jhnoh, 23.05.24
 * @return true        저장된 맵이 있어요~
 * @return false       저장된 맵이 없어요~
 */
bool CSimplifyGridMap::isExistSavedMap()
{
    CStopWatch debugSw;
    
    bool ret = false;

    std::ifstream file(mapDataStorageDir + "savedMapData.yaml");
    if(file.good())
    {
        ret = true;
    }

    TIME_CHECK_END(debugSw.getTime());
    return ret;
}


/**
 * @brief 업데이트 한다.
 * 
 * @param set 
 */
void CSimplifyGridMap::set(s8* setGrid, tGridmapInfo setInfo)
{
    CStopWatch debugSw;
  
    simplifyGridMapLock();
    info = setInfo;
    int size = info.width*info.height;

    if (pSetGrid != nullptr)
    {
        delete[] pSetGrid;
        pSetGrid = nullptr;
    }
    
    pSetGrid = new s8[size];

    memcpy(pSetGrid, setGrid, size);    
    isSetGrid = true;

    simplifyGridMapUnLock();

    // ceblog(LOG_LV_NECESSARY, MAGENTA, "simplify set");
    
    TIME_CHECK_END(debugSw.getTime());
}

cv::Mat CSimplifyGridMap::getSimplifyMatMap()
{
    return simplifyMatMap.clone();
}

std::vector<s8> CSimplifyGridMap::getAwsMap()
{
    return awsMap;
}

//aws에 보내는 예비용 gridmap 
s8* CSimplifyGridMap::getGridMap()
{
    return pSetGrid;
}

void CSimplifyGridMap::simplifyGridMapLock()
{
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);
}

void CSimplifyGridMap::simplifyGridMapUnLock()
{
    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");
}

u8 *CSimplifyGridMap::getSimplifyGridMapPtr()
{
    return pSimplifyGridMap;
}
tGridmapInfo CSimplifyGridMap::getSimplifyGridMapInfo()
{
    return info;
}
bool CSimplifyGridMap::isValidSimplifyGridMap()
{
    bool ret = false;
    if (info.height == 0 || info.width == 0 || info.resolution == 0.0 || pSimplifyGridMap == nullptr)
    {
        ret = false;
    }
    else
    {
        ret = true;
    }
    return ret;
}
#if 0
/**
 * @brief 단순화 맵 복사 함수.
 * @param dest 
 * @param pInfo 
 * @return true 
 * @return false 
 */
bool CSimplifyGridMap::copyGridMap(u8 *& dest, tGridmapInfo *pInfo)
{
    CStopWatch debugSw;

    bool ret = false;    
    pthread_mutex_lock(&mutex);
    if (info.height == 0 || info.width == 0 || info.resolution == 0.0 || pSimplifyGridMap == nullptr)
    {
        //ceblog(LOG_LV_ERROR, RED, "width : " << info.width << " ,height  : " << info.height << " ,resolution : " << info.resolution /*<<" ,pSimplifyGridMap : " <<pSimplifyGridMap*/);/* don't copy! */
        //ceblog(LOG_LV_ERROR, RED, "CSimplifyGridMap copy error!!!! ");/* don't copy! */
    }
    else
    {
        *pInfo = info;
        int size = pInfo->width * pInfo->height;        
        if (dest != nullptr){
            delete[] dest;
            dest = nullptr;
        }

        dest = new u8[size];
        //reset Map : FREE_SPACE
        memset(dest, 0, size * sizeof(u8));
        memcpy(dest, pSimplifyGridMap, sizeof(u8) * size);
        ret = true;        
    }    

    pthread_mutex_unlock(&mutex);
    
    TIME_CHECK_END(debugSw.getTime());
    return ret;
}
#endif
CRawGridMap::CRawGridMap()
{
    pGridMapRaw = NULL;
    pthread_mutex_init(&mutex, nullptr);
}

CRawGridMap::~CRawGridMap()
{
    pthread_mutex_destroy(&mutex);
}
/**
 * @brief 업데이트 한다.
 * 
 * @param set 
 */
void CRawGridMap::set(s8* setGrid, tGridmapInfo setInfo)
{
    CStopWatch debugSw;    

    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);
    
    info = setInfo;
    int size = info.width*info.height;        
    if (pGridMapRaw != nullptr)
    {
        delete[] pGridMapRaw;
        pGridMapRaw = nullptr;
    }
    pGridMapRaw = new s8[size];

    //reset Map : FREE_SPACE
    memset(pGridMapRaw, 0, size * sizeof(s8));
    memcpy(pGridMapRaw, setGrid, sizeof(s8) * size);

    bIsUpdate = true;
    bIsInit = true;
    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");

    // ceblog(LOG_LV_NECESSARY, MAGENTA, "gridmap raw set");
    
    TIME_CHECK_END(debugSw.getTime());
}



/**
 * @brief swap 안된 dest 를 입력 받아 grid map 을 copy 받는다.
 * 
 * @param dest 메모리 할당 안된 포인터
 * @return bool 카피 성공 true, 실패 false
 */
bool CRawGridMap::copyGridMap(s8 *& dest, tGridmapInfo *pInfo)
{
    CStopWatch debugSw;    
    bool ret = false;
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock");
    pthread_mutex_lock(&mutex);    

    if (info.height == 0 || info.width == 0 || info.resolution == 0.0 || pGridMapRaw == nullptr)
    {
        //ceblog(LOG_LV_OBSTACLE, RED, " width : " << info.width << " height  : " << info.height << " resolution : " << info.resolution);/* don't copy! */
        //ceblog(LOG_LV_OBSTACLE, RED, "gridmap copy error!!!! ");/* don't copy! */
    }
    else
    {
        *pInfo = info;
        int size = pInfo->width * pInfo->height;        

        if (dest != nullptr)
        {
            delete[] dest;
        }

        dest = new s8[size];
        //reset Map : FREE_SPACE
        memset(dest, 0, size * sizeof(s8));
        memcpy(dest, pGridMapRaw, sizeof(s8) * size);
        ret = true;
    }

    pthread_mutex_unlock(&mutex);
    ceblog(LOG_LV_SYSDEBUG, GRAY, "lock - free");
    
    TIME_CHECK_END(debugSw.getTime());
    return ret;
}

