#include "mapStorage.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

/**
 * @brief Construct a new CMapStorage::CMapStorage object
 * jhnoh, 23.05.24
 */
CMapStorage::CMapStorage() 
{
    CStopWatch __debug_sw;
    
    mapDataStorageDir = "/home/ebot/map/";    

    // 배열 값 초기화
    for (int i = 0; i < 5; i++) 
    {
        potentialChargerPoses[i] = std::make_pair(tPose(), false); 
    }

    dockingPoseData.bKnownCharger = false;

    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief Destroy the CMapStorage::CMapStorage object
 * jhnoh, 23.05.24
 */
CMapStorage::~CMapStorage()
{
    CStopWatch __debug_sw;
    
    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 저장된 충전기 위치를 복사한다. (get)
 * jhnoh, 23.05.24
 * @param pPose     복사 될 충전기 위치
 * @return true     복사 성공! 
 * @return false    복사 실패!
 */
bool CMapStorage::copySavedStationPoseFromYAML(tPose* pPose)
{
    CStopWatch __debug_sw;
    
    bool ret = false;
    
    std::ifstream file(mapDataStorageDir + "savedChargerData.yaml");
    if(file.good())
    {
        YAML::Node mapNode = YAML::LoadFile(mapDataStorageDir + "savedChargerData.yaml");
        tPose stationPose = tPose();
        s8*  pGridmap;
        if (mapNode["stationX"])                   stationPose.x           = mapNode["stationX"].as<double>();
        if (mapNode["stationY"])                   stationPose.y          = mapNode["stationY"].as<double>();
        *pPose = stationPose;
        
        mapNode.reset();
        file.close();
        eblog(LOG_LV, " The map data has been imported from the YAML file.");
        ret =true;
    }
    else
    {
        eblog(LOG_LV, " YAML File does not exist.  ");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;

}

/**
 * @brief 맵을 저장한 time get
 * jhnoh, 23.05.24
 * @return double 
 */
double CMapStorage::getUpdateMapSavedTime()
{
    CStopWatch __debug_sw;
    
    double ret = -1;
    std::ifstream file(mapDataStorageDir + "savedMapData.yaml");
    if(file.good())
    {
        YAML::Node mapNode = YAML::LoadFile(mapDataStorageDir + "savedMapData.yaml");
        if (mapNode["updateTime"])                  ret = mapNode["updateTime"].as<double>();

        eblog(LOG_LV, " Found. update time is  [ " <<   ret << " ]");
    }
    else
    {
        eblog(LOG_LV, " YAML File does not exist.  ");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 충전기 정보를 저장한 time get
 * jhnoh, 23.05.24
 * @return double 
 */
double CMapStorage::getUpdateChargerSavedTime()
{
    CStopWatch __debug_sw;
    
    double ret = -1;
    std::ifstream file(mapDataStorageDir + "savedChargerData.yaml");
    if(file.good())
    {
        YAML::Node mapNode = YAML::LoadFile(mapDataStorageDir + "savedChargerData.yaml");
        if (mapNode["updateTime"])                  ret = mapNode["updateTime"].as<double>();

        eblog(LOG_LV, " Found. update time is  [ " <<   ret << " ]");
    }
    else
    {
        eblog(LOG_LV, " YAML File does not exist.  ");
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;

}

/**
 * @brief 충전기 신호가 들어오는 위치들 정보 확인 함수 
 * jhnoh, 23.05.24
 * @return true         충전기 신호가 들어오는 위치가 저장되어 있어요~
 * @return false        충전기 신호가 들어오는 위치가 저장되어 있지 않아요~
 */
bool CMapStorage::isExistPotentialChargerPoses()
{
    CStopWatch __debug_sw;
    
    bool ret = false;

    if(1)
    {
        ret = true;
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}


/**
 * @brief   지도를 업데이트 한다.
 * jhnoh, 23.05.24
 * @param dest      업데이트 하고 싶은 지도
 * @param info      업데이트 하고 싶은 지도의 정보
 * @return true     업데이트 성공!
 * @return false    업데이트 실패!
 */
bool CMapStorage::updateGridMapToYAML(u8* dest, tGridmapInfo info, int index)
{
    CStopWatch __debug_sw;
    
    //ceblog(LOG_LV_NECESSARY, BOLDCYAN, "index 값 : " << index << " size : " << info.width * info.height);
    bool ret = false;
    int cnt=0;
    std::ofstream file( mapDataStorageDir + "map_info_" + std::to_string(index) +".yaml");

    eblog(LOG_LV, " YAML File  OPEN.  ");
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "resolution";
    emitter << YAML::Value << info.resolution;
    emitter << YAML::Key << "width";
    emitter << YAML::Value << info.width;
    emitter << YAML::Key << "height";
    emitter << YAML::Value << info.height;
    emitter << YAML::Key << "origin_x";
    emitter << YAML::Value << info.origin_x;
    emitter << YAML::Key << "origin_y";
    emitter << YAML::Value << info.origin_y;
    emitter << YAML::Key << "updateTime";
    emitter << YAML::Value << get_system_time();
    // emitter << YAML::Key << "value";
    // std::string destString;
    // destString.reserve(info.width*info.height);
    // for (int i = 0; i < info.width * info.height; ++i)
    // {
    //     destString.push_back(static_cast<char>(dest[i]));
    //     cnt++;
    // }
    // emitter << YAML::Value << destString;
    emitter << YAML::Key << "value_size";
    emitter << YAML::Value << cnt;
    // emitter << YAML::Value << std::string(dest, info.width*info.height);
    emitter << YAML::EndMap;

    file << emitter.c_str();
    file.close();

    // 파일 스트림 생성
    std::ofstream fout(mapDataStorageDir+"map_Data_" + std::to_string(index) +".yaml", std::ios::binary);

    // 데이터를 파일에 저장
    for (int i = 0; i < info.width * info.height; ++i)
    {
        fout.write(reinterpret_cast<const char*>(&(dest[i])), sizeof(u8));
    }
    fout.close();
    ret = true;
    eblog(LOG_LV, " Map data has been written in YAML.");
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
/**
 * @brief   로봇의 경로 좌표를 업데이트 한다.
*/
bool CMapStorage::updateTrajectoryToYAML(std::list<tPoint> traj, int index)
{
    CStopWatch __debug_sw;
    
    bool ret = false;

#if 1
    // 파일 스트림 생성
    std::ofstream fout(mapDataStorageDir+"trajData_" + std::to_string(index) +".yaml", std::ios::binary);

    // 데이터를 파일에 저장
    for (tPoint value : traj) 
    {
        fout.write(reinterpret_cast<const char*>(&(value.x)), sizeof(double));
        fout.write(reinterpret_cast<const char*>(&(value.y)), sizeof(double));
    }

    fout.close();
    std::cout << "Data saved to data.yaml" << std::endl;

#else
    if( index == 0)
    {
        std::ofstream file( mapDataStorageDir + "trajData.yaml");
    }
    else
    {
        std::ofstream file( mapDataStorageDir + "trajData.yaml");
    }

    eblog(LOG_LV, " YAML File  OPEN.  ");
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "trajectory";

    emitter << YAML::Value << YAML::BeginSeq;
    for(const auto& point : traj)
    {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "x";
        emitter << YAML::Value << point.x;
        emitter << YAML::Key << "y";
        emitter << YAML::Value << point.y;
        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;

    emitter << YAML::Key << "updateTime";
    emitter << YAML::Value << get_system_time();
    emitter << YAML::EndMap;

    file << emitter.c_str();
    file.close();
    ret = true;
    
    eblog(LOG_LV, " Trajectory data has been written in YAML.");
    
    TIME_CHECK_END(__debug_sw.getTime());
#endif
    return ret;

}

/**
 * @brief 충전기 정보를 업데이트 한다.
 * jhnoh, 23.05.24
 * @param tmpPose   업데이트 할 충전기 위치
 * @return true     업데이트 성공!
 * @return false    업데이트 실패!
 */
bool CMapStorage::updateChargerToYAML(tPose tmpPose)
{
    CStopWatch __debug_sw;
    
    bool ret = false;
    
    std::ofstream file( mapDataStorageDir + "savedChargerData.yaml");
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "stationX";
    emitter << YAML::Value << tmpPose.x;
    emitter << YAML::Key << "stationY";
    emitter << YAML::Value << tmpPose.y;
    emitter << YAML::Key << "updateTime";
    emitter << YAML::Value << get_system_time();
    emitter << YAML::EndMap;

    file << emitter.c_str();
    file.close();
    ret = true;

    eblog(LOG_LV, " charger data has been written in YAML.");
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 다른 서비스 중 충전기 신호가 들어온 위치 정보를 업데이트 합니다.
 * jhnoh, 23.05.24
 * @return true 
 * @return false 
 */
bool CMapStorage::updatePotentialCharger(tPose potentialPose, tSignalData signalData)
{
    CStopWatch __debug_sw;
    
    bool ret = false;

#if 0 // cppcheck 오류로 인해 임시로 막음. 추후 적용 예정  / jhnoh
    size_t maxSize = sizeof(potentialChargerPoses) / sizeof(potentialChargerPoses[0]);
    E_POTENTIAL_CHARGER sigPose = E_POTENTIAL_CHARGER::SHORT_CENTER;
    for(sigPose; SC<int>(sigPose) < maxSize; SC<E_POTENTIAL_CHARGER>(SC<int>(sigPose) + 1))
    {
        ret = checkParticularChargerSignal(sigPose, potentialPose,signalData);
    }
#endif
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 
 * jhnoh, 23.05.24
 * @param sigPose 
 * @param potentialPose 
 * @param signalData 
 * @return true 
 * @return false 
 */
bool CMapStorage::checkParticularChargerSignal(E_POTENTIAL_CHARGER sigPose, tPose potentialPose, tSignalData signalData)
{
    CStopWatch __debug_sw;
    
    bool ret = false;

    if(potentialChargerPoses[SC<int>(sigPose)].second == true)
    {
        TIME_CHECK_END(__debug_sw.getTime());
        return ret;
    }
    
    switch (sigPose)
    {
    case E_POTENTIAL_CHARGER::SHORT_CENTER:
        if((signalData.detector.b.center & 0b11 ))
        { 
            eblog(LOG_LV, " potential charger update : only short Center ");
            ret = true;
            // 크래들 초근접 좌표 저장(회피조건이기도 함) 
        } 
        break;
    case E_POTENTIAL_CHARGER::LEFT_CENTER:
        if((signalData.detector.b.leftCenter & 0b11) && (signalData.detector.b.rightCenter & 0b10))
        {
            eblog(LOG_LV, " potential charger update : only left Center ");
            ret = true;
            //왼쪽 센터
        } 
        break;
    case E_POTENTIAL_CHARGER::RIGHT_CENTER:
        if((signalData.detector.b.rightCenter & 0b11) && (signalData.detector.b.leftCenter & 0b01))
        {
            eblog(LOG_LV, " potential charger update :  only right Center ");
            ret = true;
            //오른쪽 센터
        } 
        break;
    case E_POTENTIAL_CHARGER::CENTER:
        if((signalData.detector.b.leftCenter & 0b01) && (signalData.detector.b.rightCenter & 0b10))
        { 
            //도킹을 위해 가야할 포인트 (복귀 포인트라고 칭함) 최우선순위. 만약 이전 값이 아래 else if 면 바꿔치기 
            eblog(LOG_LV, " potential charger update : only left Center && right Center ");
            ret = true;
        } 
        break;
    case E_POTENTIAL_CHARGER::LEFT_SIDE:
        if((signalData.detector.b.leftSide & 0b11))
        { 
            eblog(LOG_LV, " potential charger update : only left Long ");
            ret = true;
            //왼쪽사이드 저장 
        } 
        break;
    case E_POTENTIAL_CHARGER::LEFT_SIDE_CENTER:
        if((signalData.detector.b.leftSide & 0b11) && (signalData.detector.b.leftCenter & 0b11))
        { 
            eblog(LOG_LV, " potential charger update : only left Long && left Center ");
            ret = true;
            // 왼쪽 사이드에서 센터사이
        }
    case E_POTENTIAL_CHARGER::RIGHT_SIDE:
        if((signalData.detector.b.rightSide & 0b11))
        { 
            eblog(LOG_LV, " potential charger update : only right Long && right Center ");
            ret = true;
            // 오른쪽 사이드
        }
        break;        
    case E_POTENTIAL_CHARGER::RIGHT_SIDE_CENTER:
        if((signalData.detector.b.rightSide & 0b11) && (signalData.detector.b.rightCenter & 0b11))
        { 
            eblog(LOG_LV, " potential charger update : only right Long && right Center ");
            ret = true;
            // 오른쪽 사이드에서 센터 사이
        }
        break;
    default:
        break;
    }

    if(ret)
    {
        potentialChargerPoses[SC<int>(sigPose)] = std::make_pair(potentialPose, true);
        eblog(LOG_LV, " potential Charger Pose [ " << SC<int>(sigPose) << " ] : update completed");
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief 
 * jhnoh, 23.05.24
 * @param i 
 * @param pPose 
 * @return true 
 * @return false 
 */
bool CMapStorage::copyPotentialChargerPoses(int i, tPose* pPose)
{
    CStopWatch __debug_sw;
    
    bool ret = false;
    
    size_t maxSize = sizeof(potentialChargerPoses) / sizeof(potentialChargerPoses[0]);
    if(0 <= i && i < maxSize)
    {
        ret = potentialChargerPoses[i].second;
        *pPose = potentialChargerPoses[i].first;
        eblog(LOG_LV," i = " << i  <<" check data  : " << ret << "  pose : " << pPose[i].x << " , " << pPose[i].y  );
    }
    else
    {
        
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief potentialChargerPoses 를 clear 하는 함수
 * jhnoh, 23.05.24
 */
void CMapStorage::clearPotentialCharger()
{
    CStopWatch __debug_sw;
    
    // 배열 값 초기화
    for (int i = 0; i < 5; i++) 
    {
        potentialChargerPoses[i] = std::make_pair(tPose(),false); 
    }

    eblog(LOG_LV, "Cleared all potentialChargerPoses.");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @param set 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
void CMapStorage::setChargerPose(tPose set)
{   
    CStopWatch __debug_sw;
    
    setKnownCharger(true);
    dockingPoseData.charger = set;

    TIME_CHECK_END(__debug_sw.getTime());
}
void CMapStorage::resetChargerPose()
{   
    CStopWatch __debug_sw;
    
    setKnownCharger(false);
    dockingPoseData.charger = tPose(0.0,0.0,0.0);

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 
 * 
 * @return tPose 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
tPose CMapStorage::getChargerPose()
{   
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    
    return dockingPoseData.charger;
}
/**
 * @brief 
 * 
 * @param set 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
void CMapStorage::setChargerApproachPose(tPose set)
{   
    CStopWatch __debug_sw;
    
    dockingPoseData.chargerApproach = set;

    TIME_CHECK_END(__debug_sw.getTime());
}
/**
 * @brief 
 * 
 * @return tPose 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
tPose CMapStorage::getChargerApproachPose()
{   
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    
    return dockingPoseData.chargerApproach;
}
/**
 * @brief 
 * 
 * @param set 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
void CMapStorage::setKnownCharger(bool set)
{   
    CStopWatch __debug_sw;
    
    dockingPoseData.bKnownCharger = set;

    TIME_CHECK_END(__debug_sw.getTime());
}
/**
 * @brief 
 * 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2023-09-04
 * @author hhryu
 */
bool CMapStorage::getIsKnownCharger()
{   
    CStopWatch __debug_sw;
    
    TIME_CHECK_END(__debug_sw.getTime());
    
    return dockingPoseData.bKnownCharger;
}