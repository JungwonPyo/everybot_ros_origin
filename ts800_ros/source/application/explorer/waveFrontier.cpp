#include "waveFrontier.h"

#include "utils.h"
#include "eblog.h"
#include "debugCtr.h"
#include <queue>
#include <map>
#include "rosPublisher.h"
#include "motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 1 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 20.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CWaveFrontier::CWaveFrontier()
{
	CStopWatch __debug_sw;
	pthread_mutex_init(&mutexWfdPoints, nullptr);

	wfdMap = nullptr;
	MAP_OPEN_LIST = 1;
	MAP_CLOSE_LIST = 2;
	// FRONTIER_OPEN_LIST = 3;
	// FRONTIER_CLOSE_LIST = 4;
	bisNeedUPdate = false;
	searchingCnt = 0;

	// pthread_create(&thWaveFrontier, nullptr, &CWaveFrontier::threadWaveFrontierWrap, this);

	eblog(LOG_LV,  "");
	
	TIME_CHECK_END(__debug_sw.getTime());
}

CWaveFrontier::~CWaveFrontier()
{
	CStopWatch __debug_sw;
	// pthread_join(thWaveFrontier, nullptr);
  	pthread_mutex_destroy(&mutexWfdPoints);	
    eblog(LOG_LV,  "");
	
	TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief wave Frontier 알고리즘
 * 
 * @param map 
 * @param info 
 * @param pose 
 * @param robotPose 
 * 
 * @note  연산시간: 25ms ~ 100.0ms 
 * @date  2023-08-28
 */
void CWaveFrontier::waveFrontier(tGridmapInfo info, int x, int y, tPose robotPose) 
{
	CStopWatch __debug_sw;

	int N_S = 8;
	int S_N_S = 4;
	int map_width =  (int)(info.width/MAP_DOWNSCALE_VALUE);
	int map_height = (int)(info.height/MAP_DOWNSCALE_VALUE);
	int map_size = map_width*map_height;
	
	std::list<int> frontiers;
	std::map<int, int> cell_states; // All cells are deing initialized as 0 as default
	std::queue<int> q_m;	
	
	// 검색 시작 위치 업데이트 (WFD의 시작포인트)
	int pose = (int)(x/MAP_DOWNSCALE_VALUE) + (int)(y/MAP_DOWNSCALE_VALUE) * map_width;
	q_m.push(pose);
	cell_states[pose] = MAP_OPEN_LIST;
	
	int adj_vector[N_S];
	int check_vector[S_N_S];

	while(!q_m.empty()) 
	{
		int cur_pos = q_m.front();
		q_m.pop();

		// Skip if map_close_list
		if(cell_states[cur_pos] == MAP_CLOSE_LIST)
		{
			continue;
		}

		getNeighbours8(adj_vector, cur_pos, map_width);
		for (int i = 0; i < N_S; i++)
		{
			if (adj_vector[i] < map_size && adj_vector[i] >= 0)
			{
				if (cell_states[adj_vector[i]] != MAP_CLOSE_LIST)
				{
					if (wfdMap[adj_vector[i]] == GRAY_LV_UNKNOWN_WALL || wfdMap[adj_vector[i]] == GRAY_LV_UNKNOWN_AREA)
					{
						frontiers.emplace_back(cur_pos);
						cell_states[adj_vector[i]] = MAP_CLOSE_LIST;
					}
					else if (wfdMap[adj_vector[i]] == GRAY_LV_KNOWN_WALL)
					{
						cell_states[adj_vector[i]] = MAP_CLOSE_LIST;
					}
					else if (cell_states[adj_vector[i]] != MAP_OPEN_LIST && wfdMap[adj_vector[i]] == GRAY_LV_KNOWN_AREA)
					{
						q_m.push(adj_vector[i]);
						cell_states[adj_vector[i]] = MAP_OPEN_LIST;
					}
				}
			}
		}

		// getNeighbours4(check_vector, cur_pos, map_width);
		// for(int i = 0; i < S_N_S; i++) 
		// {
		// 	if(check_vector[i] < map_size && check_vector[i] >= 0) 
		// 	{
		// 		if(cell_states[check_vector[i]] != MAP_CLOSE_LIST) 
		// 		{
		// 			if(wfdMap[check_vector[i]] == GRAY_LV_UNKNOWN_WALL || wfdMap[check_vector[i]] == GRAY_LV_UNKNOWN_AREA) 
		// 			{
		// 				frontiers.emplace_back(cur_pos);
		// 				cell_states[check_vector[i]] = MAP_CLOSE_LIST;
		// 				break;
		// 			}
		// 			else if ( wfdMap[check_vector[i]] == GRAY_LV_KNOWN_WALL )
		// 			{
		// 				cell_states[check_vector[i]] = MAP_CLOSE_LIST;
		// 			}
		// 		}
		// 	}
		// }
		// getNeighbours8(adj_vector, cur_pos, map_width);
		// for (int i = 0; i < N_S; ++i) 
		// {
		// 	if(adj_vector[i] < map_size && adj_vector[i] >= 0) 
		// 	{
		// 		if(cell_states[adj_vector[i]] != MAP_OPEN_LIST && cell_states[adj_vector[i]] != MAP_CLOSE_LIST) 
		// 		{					
		// 			if(wfdMap[adj_vector[i]] == GRAY_LV_KNOWN_AREA) 
		// 			{ 						
		// 				q_m.push(adj_vector[i]);
		// 				cell_states[adj_vector[i]] = MAP_OPEN_LIST;
		// 			}
		// 		}
		// 	}
		// }
		cell_states[cur_pos] = MAP_CLOSE_LIST;
	}

	std::list<tPoint> frontierPoints;
	//std::list<tPoint> tunedPoints;
	tPoint frontierPoint;

    double map_x = info.origin_x / info.resolution;
    double map_y = info.origin_y / info.resolution;
	int searchCnt = frontiers.size();
	if(searchCnt > 1)
	{
		ceblog(LOG_LV_NECESSARY,  BOLDBLUE ," searchCnt 개수 :  "<<searchCnt << " 개");

		for(int frontier  : frontiers) 
		{
			frontierPoint.x = ((frontier % map_width)*MAP_DOWNSCALE_VALUE + map_x) * info.resolution;
			frontierPoint.y = ((frontier / map_width)*MAP_DOWNSCALE_VALUE + map_y) * info.resolution;
			frontierPoints.emplace_back(frontierPoint);
			//ceblog(LOG_LV_NECESSARY,  BOLDBLUE ," origin wavePoint :  "<< frontierPoint.x << " , " << frontierPoint.y);
		}

		// tunedPoints = meanShiftClustering(frontierPoints);

		insertWfd(frontierPoints);
		sortWfd(robotPose);
		
		// ceblog(LOG_LV_NECESSARY,  BOLDBLUE ," 튜닝 후 탐색 개수 :  "<< wfdPoints.size() << " 개");
		// for(tPoint tuned : tunedPoints)
		// {
		// 	ceblog(LOG_LV_NECESSARY,  BOLDBLUE ," tuned wavePoint :  "<< tuned.x << " , " << tuned.y);
		// }
		
		DEBUG_PUB.publishRawFrontiers(frontierPoints);		
		searchingCnt = 0;
	}
	else
	{	
		searchingCnt++;
		ceblog(LOG_LV_NECESSARY,  BOLDYELLOW ,"탐색할 곳을 찾지 못했습니다. searchingCnt : " << searchingCnt);
	}

	TIME_CHECK_END(__debug_sw.getTime());
}

bool CWaveFrontier::isFinish()
{
	bool bRet = false;
	if (searchingCnt > 20)
	{
		searchingCnt = 0;
		bRet = true;
	}

	return bRet;
}

/**
 * @brief 로봇 위치로 부터 특정 반경내에서는 check 하고
 * 로봇위치로 부터 가까운 순대로 정렬
 * 
 */
//void CWaveFrontier::sortWfd(tPose robotPose, std::list<tPoint> path)
void CWaveFrontier::sortWfd(tPose robotPose)
{
	CPthreadLockGuard lock(mutexWfdPoints);
	//std::list<tPoint> path = ServiceData.robotMap.robotTrajectory.getExploredTrajectory();
	//로봇위치로부터 특정 반경은 bCheck true 로 만든다.
	double radius = 1.5;//0.6; // 특정 반경 
	std::vector<std::pair<double, wfdPoint*>> distanceAndPoints;

	// 로봇 위치로부터 각 포인트까지의 거리 계산 및 반경 내 포인트 선정
	for (auto& point : wfdPoints) {
		
		double distance = sqrt(pow(point.pt.x - robotPose.x, 2) + pow(point.pt.y - robotPose.y, 2));
		
		if (distance <= radius) {
			distanceAndPoints.push_back(std::make_pair(distance, &point));
			point.bCheck = true; // 특정 반경 이내로 설정
		}
	}

#if 0// path에 있는 모든 점에 대해, 해당 점 주변의 wfdPoints를 다시 방문하지 않도록 설정
	radius = 1.5; // 특정 반경 
    for (auto& pathPoint : path) {
        for (auto& point : wfdPoints) {
            double distance = sqrt(pow(point.pt.x - pathPoint.x, 2) + pow(point.pt.y - pathPoint.y, 2));

            if (distance <= radius) {
                point.bCheck = true; // 이미 방문한 지점 근처는 방문하지 않도록 설정
            }
        }
    }
#endif

	//로봇 위치 로부터 가까운 순으로 정렬 한다.
	std::sort(distanceAndPoints.begin(), distanceAndPoints.end(),
		[](const std::pair<double, wfdPoint*>& a, const std::pair<double, wfdPoint*>& b) {
			return a.first < b.first;
		});

#if 0	// debug
	std::list<tPoint> dbgPoint;

	for (auto& point : wfdPoints) {		
		if (point.bCheck == false) {
			dbgPoint.push_back(point.pt);
		}
	}

	DEBUG_PUB.publishTunedFrontiers(dbgPoint);
#endif
#if 1	//debug
	int cnt = 0;
	for (auto& point : wfdPoints) {		
		if (point.bCheck == false) {
			cnt++;
		}
	}
	ceblog(LOG_LV, YELLOW, "남은 유효점 개수 : "<<cnt);  
#endif
}

void CWaveFrontier::insertWfd(std::list<tPoint> tunedPoints)
{
	CPthreadLockGuard lock(mutexWfdPoints);
	for(tPoint tmpPoint : tunedPoints)
	{
		wfdPoint tempWfdPoint(tmpPoint);
		wfdPoints.emplace_back(tempWfdPoint);
	}
}

/**
 * @brief 탐색된 포인트가 실제 이동 가능한지 검증.
 * 
 */
void CWaveFrontier::findValidNearPoint(std::list<tPoint> frontierList)
{
#if 0
	// hyjoe . wave frontier 포인트가 2개 이상일 경우 멀리서 부터 탐색하고 가장 가까운 점은 삭제
	if ( frontierList.size() >= 2 )
	{
		while (frontierList.size() > 1)
		{
			std::list<tPoint> reversedFrontierList;
			for ( auto i=frontierList.rbegin(); i != frontierList.rend(); ++i )
			{
				reversedFrontierList.push_back(*i);
			}
			frontierList = reversedFrontierList;
			frontierList.pop_back();
		}
	}
#endif
	
	tPose robotPose = ServiceData.localiz.getPose();
	if (PATH_PLANNER->dofindNearestPath(robotPose, 
		frontierList, frontierList.size())){            
		ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾습니다.");		
		
	}
	else{
		ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "먼저 경로요청이 있었나봐요. 점검해주세요.");
	}
}

void CWaveFrontier::checkInvalidPoint()
{
#if 0
	if ( PATH_PLANNER->isRunFindPath() == false )
    {
        if (PATH_PLANNER->isFindPath())
        {            
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE, "최적 경로 목적지를 찾았습니다.");
			nearPoint = PATH_PLANNER->getPath().back();
        }
        else
        {			
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, " 경로계획을 실패하였습니다.");
        }

		
    }
#endif
}

void CWaveFrontier::updateWfdMap(u8 *map, tGridmapInfo info)
{	
	int map_width =  (int)(info.width / MAP_DOWNSCALE_VALUE);
	int map_height = (int)(info.height / MAP_DOWNSCALE_VALUE);
	int map_size = map_width * map_height;

	if (wfdMap != nullptr){
		delete[] wfdMap;
		wfdMap = nullptr;
    }

	wfdMap = new u8[map_size];
	//reset Map : FREE_SPACE
	memset(wfdMap, GRAY_LV_UNKNOWN_AREA, map_size * sizeof(u8));
	for(int i = 0; i < info.width*info.height; i++)
    {
        int ox = (int)((int)(i % info.width) / MAP_DOWNSCALE_VALUE);
        int oy = (int)((int)(i / info.width) / MAP_DOWNSCALE_VALUE);
		int index = ox+oy*map_width;
		if(index < 0 || index >= map_size)
		{
			continue;
		}
		switch (map[i])
		{
		case GRAY_LV_KNOWN_WALL:
			wfdMap[index] = GRAY_LV_KNOWN_WALL;
			break;
		case GRAY_LV_KNOWN_AREA:
			if (wfdMap[index] != GRAY_LV_KNOWN_WALL)
			{
				wfdMap[index] = GRAY_LV_KNOWN_AREA;
			}
			break;
		case GRAY_LV_UNKNOWN_WALL:
			if(wfdMap[index] != GRAY_LV_KNOWN_WALL && wfdMap[index] != GRAY_LV_KNOWN_AREA)
			{
				wfdMap[index] = GRAY_LV_UNKNOWN_WALL;
			}
			break;		
		case GRAY_LV_UNKNOWN_AREA:
			if(wfdMap[index] != GRAY_LV_KNOWN_WALL && wfdMap[index] != GRAY_LV_UNKNOWN_WALL && wfdMap[index] != GRAY_LV_KNOWN_AREA)
			{
				wfdMap[index] = GRAY_LV_UNKNOWN_AREA;
			}
			break;
		default:
			break;
		}
    }
}

bool CWaveFrontier::useInvalidNearPoint(tPoint &pt)
{
	CPthreadLockGuard lock(mutexWfdPoints);
	bool ret = false;
	for (auto it = wfdPoints.begin(); it != wfdPoints.end(); )
	{
		if (!it->bCheck) {
			it->bCheck = true;
			pt = it->pt;
			it = wfdPoints.erase(it);
			ret = true;
			break;
		} else {
			++it;
		}
	}

	ceblog(LOG_LV, YELLOW, "남은 유효점 개수 : "<< wfdPoints.size());
	#if 0
	for (auto& point : wfdPoints) {		
		if (point.bCheck == false) {
			pt = point.pt;			
			point.bCheck = true;
			ret = true;
			break;
		}
	}
	
	int cnt = 0;
	for (auto& point : wfdPoints) {		
		if (point.bCheck == false) {
			cnt++;
		}
	}
	  
#endif

	//포인트를 가져갔기때문에 이전거는 클리어 하고 새로 받는다.
	//wfdPoints.clear();

    return ret;
}

bool CWaveFrontier::isUpdateWdfPoints()
{
	bool bRet = false;
	CPthreadLockGuard lock(mutexWfdPoints);
	for (auto& point : wfdPoints) {		
		if (point.bCheck == false) {
			bRet = true;
			break;
		}
	}
	return bRet;
}


/**
 * @brief WFD 업데이트 시작
 * 
 */
void CWaveFrontier::startUpdate()
{	  
    ceblog(LOG_LV_NECESSARY, YELLOW, " wave frontier 동작 시킵니다.");  
    bisNeedUPdate = true;
	searchingCnt = 0;
}

/**
 * @brief WFD 업데이트 종료
 * 
 */
void CWaveFrontier::stopUpdate()
{
	ceblog(LOG_LV_NECESSARY, YELLOW,  "wave frontier 를 중지 합니다.");
    bisNeedUPdate = false;
	searchingCnt = 0;
}

/**
 * @brief WFD 업데이트 중인지 판단
 * 
 * @return true   : 업데이트 중
 * @return false  : 업데이트 종료
 */
bool CWaveFrontier::isNeedUpdate()
{
    return bisNeedUPdate;
}

/**
 * @brief 포인트 근처 8개 좌표 구함.
 * 
 * @param n_array 
 * @param position 
 * @param map_width 
 */
void CWaveFrontier::getNeighbours8(int n_array[], int position, int map_width) 
{	
	CStopWatch __debug_sw;
	
	n_array[0] = position - map_width - 1;
	n_array[1] = position - map_width; 
	n_array[2] = position - map_width + 1; 
	n_array[3] = position - 1;
	n_array[4] = position + 1;
	n_array[5] = position + map_width - 1;
	n_array[6] = position + map_width;
	n_array[7] = position + map_width + 1;
	
	TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 포인트 근처 8개 좌표 구함.
 * 
 * @param n_array 
 * @param position 
 * @param map_width 
 */
void CWaveFrontier::getNeighbours4(int n_array[], int position, int map_width) 
{	
	CStopWatch __debug_sw;
	
	n_array[0] = position - map_width; 
	n_array[1] = position - 1;
	n_array[2] = position + 1;
	n_array[3] = position + map_width;
	
	TIME_CHECK_END(__debug_sw.getTime());
}


/**
 * @brief 프론티어 포인트인지 체크하는 함수
 * 
 * @param map 
 * @param point 
 * @param map_size 
 * @param map_width 
 * @return true 
 * @return false 
 */
bool CWaveFrontier::isFrontierPoint(u8 * map, int point, int map_size, int map_width)
{
	// The point under consideration must be known
	CStopWatch __debug_sw;
	bool ret = false;

	if( map[point] == GRAY_LV_UNKNOWN_WALL)
	{
		ret =true;
	}
	else
	{
		ret=false;
	}

	TIME_CHECK_END(__debug_sw.getTime());
	return ret;
}

/**
 * @brief meanShift 군집화 알고리즘
 * jhnoh 23.08.28
 * @param data 
 * @return std::list<tPose> 
 * 
 * @note  연산시간: 25ms ~ 100.0ms 
 * @date  2023-08-28
 */
std::list<tPoint>  CWaveFrontier::meanShiftClustering(const std::list<tPoint>& data)
{
	CStopWatch __debug_sw;
	double bandwidth = 0.5; // 밴드위스 설정
    double tolerance = 2.0; // 수렴 허용 오차
    std::list<tPoint> clusteredData = data;
    bool converged = false;

    while (!converged) {
        converged = true;
        std::list<tPoint> newClusteredData = clusteredData;

        for (tPoint& point : clusteredData) {
            tPoint newPoint = meanShift(point, clusteredData, bandwidth);
            if (utils::math::distanceTwoPoint(point, newPoint) > tolerance) {
                converged = false;
            }
            point = newPoint;
        }

        newClusteredData.clear();
        for (const tPoint& point : clusteredData) {
            bool newCluster = true;
            for (const tPoint& newPoint : newClusteredData) {
                if (utils::math::distanceTwoPoint(point, newPoint) <= tolerance) {
                    newCluster = false;
                    break;
                }
            }
            if (newCluster) {
                newClusteredData.emplace_back(point);
            }
        }

        clusteredData = newClusteredData;
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return clusteredData;
}

/**
 * @brief meanShift 함수 : 주어진 점의 평균 이동을 계산 하는 함수
 * jhnoh, 20.08.28
 * @param point 
 * @param data 
 * @param bandwidth 
 * @return tPose 
 */
tPoint  CWaveFrontier::meanShift(const tPoint& point, const std::list<tPoint>& data, double bandwidth)
{
	CStopWatch __debug_sw;
	tPoint newPoint = {0.0, 0.0};
    double totalWeight = 0.0;

    for (const tPoint& dataPoint : data) {
        double distance = utils::math::distanceTwoPoint(point, dataPoint);
        double weight = exp(-0.5 * (distance * distance) / (bandwidth * bandwidth));
        newPoint.x += weight * dataPoint.x;
        newPoint.y += weight * dataPoint.y;
        totalWeight += weight;
    }

    if (totalWeight > 0.0) {
        newPoint.x /= totalWeight;
        newPoint.y /= totalWeight;
    }

	TIME_CHECK_END(__debug_sw.getTime());
    return newPoint;
}

void CWaveFrontier::convertRobotPoseToWavePose(double rx, double ry, 
	int *wx, int *wy, tGridmapInfo mapInfo)
{	
	double mapX = rx - mapInfo.origin_x;
	double mapY = ry - mapInfo.origin_y;
	*wx = mapX / mapInfo.resolution;
	*wy = mapY / mapInfo.resolution;
}


bool CWaveFrontier::doUpdateWaveFrontier()
{
    bool bRet = false;

    if (isNeedUpdate())
    {
        if (bisRunMapUpdate == false){   
            // ceblog(LOG_LV_NECESSARY, BOLDMAGENTA, "wave 업데이트"); 
            std::thread(&CWaveFrontier::updateWaveFrontier, this).detach();  
            bRet = true;
        }
        else{
            ceblog(LOG_LV_NECESSARY, BOLDRED, "wave 업데이트 중이어서 이번 텀은 포기");
            bRet = false;
        }
    }
    
    return bRet;
}


void CWaveFrontier::updateWaveFrontier()
{
	bisRunMapUpdate = true;
	CStopWatch __debug_sw;   
    tGridmapInfo mapInfo;
    tPose robotPose;
	int cvtX = 0, cvtY = 0;
	try
	{
		robotPose = ServiceData.localiz.getPose();
		
		if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
		{
			ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
			mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
			convertRobotPoseToWavePose(robotPose.x, robotPose.y, &cvtX, &cvtY, mapInfo);
			unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
			updateWfdMap(pMap, mapInfo);
			ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
			waveFrontier( mapInfo, cvtX, cvtY, robotPose);
		}
		
		sortWfd(robotPose);
	}
	catch(const std::exception& e)
	{
		ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
	}
	TIME_CHECK_END(__debug_sw.getTime());
	
	bisRunMapUpdate = false;
}

#if 0 // 심플리 업데이트 되면 바로 업데이트 하게 변경 하여 막음 . 24.04.30, icbaek
/**
 * @brief waveFrontier th
 * 
 */
void CWaveFrontier::threadWaveFrontier()
{
    tGridmapInfo mapInfo;
    tPose robotPose;
	unsigned int tick = 0;
	int cvtX = 0, cvtY = 0;
    while(true)
    {
        CStopWatch __debug_sw;        
        if ( isUpdateRunning() == true )
        {
            try
            {
				robotPose = ServiceData.localiz.getPose();

				if (tick % 5 == 0)
				{
					if (ServiceData.robotMap.simplifyMap.isValidSimplifyGridMap())
					{
						ServiceData.robotMap.simplifyMap.simplifyGridMapLock();
						mapInfo = ServiceData.robotMap.simplifyMap.getSimplifyGridMapInfo();
						convertRobotPoseToWavePose(robotPose.x, robotPose.y, &cvtX, &cvtY, mapInfo);
						unsigned char *pMap = ServiceData.robotMap.simplifyMap.getSimplifyGridMapPtr();
						updateWfdMap(pMap, mapInfo);
						ServiceData.robotMap.simplifyMap.simplifyGridMapUnLock();
						waveFrontier( mapInfo, cvtX, cvtY, robotPose);
					}
				}
				
				sortWfd(robotPose);
            }
            catch(const std::exception& e)
            {
                ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
            }
        }
        TIME_CHECK_END(__debug_sw.getTime());
		tick++;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        DEBUG_CTR.isAliveWaveFrontier.set(true);
    }
}
#endif