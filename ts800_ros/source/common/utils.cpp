#include "utils.h"
#include <iostream>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

#define PI_         3.14159265
#define RAD(deg)    (deg)*(PI_/180.0)
#define DEG(rad)    (rad)*(180.0/PI_)

using namespace cv;

namespace utils
{
    bool getLocalization(
        tf::TransformListener *tf_listener,
        std::string frame_base,
        std::string frame_target,
        geometry_msgs::PoseStamped *pose)
    {
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose.orientation.w = 1.0;
        target_pose.header.frame_id = frame_target;
        try
        {
            tf_listener->waitForTransform(frame_target, frame_base, ros::Time::now(), ros::Duration(0.01));        
            tf_listener->transformPose(frame_base, target_pose, target_pose);
            // ROS_INFO("x: %.2f\t\ty: %.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);  
            target_pose.header.seq += 1;
            target_pose.header.stamp = ros::Time::now();
            *pose = target_pose;
            return true;
        }
        catch(const std::exception& e)
        {
            std::cerr<<"catch : "<<__FILE__ <<" : "<<"["<<__LINE__  <<"]" <<__func__<<"() : ";
            std::cerr << e.what() << '\n';
            return false;
        }
    }

    /**
     * @brief 어떠한 변수가 자료형에 관계 없이, 최댓값인지 여부를 반환한다.
     * EX) s32 variable; 일 때,
     * variable == std::numeric_limits<s32>::max() 와 utils::isMaxValue(variable)의 값은 같다. (typeid(variable).name()는 integer의 i로 출력됨.)
     * @tparam T
     * @param variable
     * @return true
     * @return false
     * @note 연산시간 ms
     * @date 2023-09-18
     * @author hhryu
     */
    template <typename T>
    bool isMaxValue(const T &variable)
    {
        bool bRet = false;
        T maxValue = std::numeric_limits<T>::max(); // 받은 변수의 최댓값.
        bRet = (variable == maxValue); // 변수와 최대값을 비교하여 동일한지 확인
        // ceblog(LOG_LV_NECESSARY, BLUE, "변수의 타입: " << typeid(variable).name() <<"\t결과 : "<< SC<int>(bRet));
        return bRet;
    }
    template bool isMaxValue<s32>(const s32 &variable);
    template bool isMaxValue<s16>(const s16 &variable);
    template bool isMaxValue<s8>(const s8 &variable);
    template bool isMaxValue<u32>(const u32 &variable);
    template bool isMaxValue<u16>(const u16 &variable);   
    template bool isMaxValue<u8>(const u8 &variable); 
    template bool isMaxValue<float>(const float &variable);
    template bool isMaxValue<double>(const double &variable);

    namespace fileIO
    {
        bool saveRobotOption(const std::string& filename, const tRobotOption& option)
        {
            if (writeRobotOption(filename, option)) {
                eblog(LOG_LV_NECESSARY, "데이터를 성공적으로 파일에 저장했습니다.");
            }
            
            tRobotOption cmpData;
            if(readRobotOption(filename, cmpData)){
                eblog(LOG_LV_NECESSARY, "데이터를 성공적으로 파일에서 읽었습니다.(쓰기 검증과정)");
            } else {
                eblog(LOG_LV_NECESSARY, "데이터를 읽는 동안 오류가 발생했습니다.(쓰기 검증과정)");
                return false;
            }

            if (compareRobotOption(option, cmpData)) {
                eblog(LOG_LV_NECESSARY, "저장된 데이터와 읽어온 데이터가 일치합니다.");
                return true;
            } else {
                eblog(LOG_LV_NECESSARY, "저장된 데이터와 읽어온 데이터가 일치하지 않습니다.");
                return false;
            }
        }

        bool loadRobotOption(const std::string& filename, tRobotOption& option){
            if(readRobotOption(filename, option)){
                eblog(LOG_LV_NECESSARY, "데이터를 성공적으로 파일에서 읽었습니다.");
            }

            /* 잘읽어왔는지 어떻게 확인하지? */
        }
        
        bool writeRobotOption(const std::string& filename, const tRobotOption& option) {
            YAML::Emitter out;
            out << option;

            std::ofstream outFile(filename);
            if (!outFile) {
                eblog(LOG_LV_NECESSARY, "파일을 열 수 없습니다: "<<filename);
                return false;
            } else {
#if 1
                eblog(LOG_LV_NECESSARY, "Water Level: " << option.waterLevel);
                eblog(LOG_LV_NECESSARY, "Sound Level: " <<option.soundLv);
                eblog(LOG_LV_NECESSARY, "Country: " << option.country);
                
                int index = 1;
                for (const auto& schedule : option.cleanSchedule) {
                    eblog(LOG_LV_NECESSARY, "Clean Schedule [ " <<index++ <<"] " <<
                        "schedule time: "<<schedule.time << ", schedule weeks" <<schedule.weeks << ", Mode: " <<schedule.mode << ", Water Level: " <<schedule.waterLevel
                        << ", Areas: "<<schedule.areas <<", Enabled: "<< schedule.isEnabled<<", Valid: "<< schedule.isValid);
                }
#endif
            }

            outFile << out.c_str();
            outFile.close();

            return true;
        }

        bool readRobotOption(const std::string& filename, tRobotOption& option) {
            std::ifstream inFile(filename);
            if (!inFile) {
                eblog(LOG_LV_NECESSARY, "파일을 열 수 없습니다: "<<filename);
                return false;
            }

            YAML::Node config = YAML::LoadFile(filename);
            config >> option;

            inFile.close();  // 파일 닫기
#if 1
            eblog(LOG_LV_NECESSARY, "Water Level: " << option.waterLevel);
            eblog(LOG_LV_NECESSARY, "Sound Level: " <<option.soundLv);
            eblog(LOG_LV_NECESSARY, "Country: " << option.country);
            
            int index = 1;
            for (const auto& schedule : option.cleanSchedule) {
                eblog(LOG_LV_NECESSARY, "Clean Schedule [ " <<index++ <<"] " <<
                    "schedule time: "<<schedule.time << ", schedule weeks" <<schedule.weeks << ", Mode: " <<schedule.mode << ", Water Level: " <<schedule.waterLevel
                    << ", Areas: "<<schedule.areas <<", Enabled: "<< schedule.isEnabled<<", Valid: "<< schedule.isValid);
            }
#endif

            return true;
        }

        void operator>>(const YAML::Node& node, tRobotOption& option) {
            option.waterLevel = node["waterLevel"].as<short>();
            option.soundLv = node["soundLv"].as<short>();
            option.country = node["country"].as<short>();
            const YAML::Node& schedules = node["cleanSchedule"];
            /* 예약청소 */
            for (size_t i = 0; i < schedules.size(); ++i) {
                tCleanSchedule schedule;
                schedules[i] >> schedule;
                option.cleanSchedule.push_back(schedule);
            }

            /* 추가 데이터 */
        }

        void operator>>(const YAML::Node& node, tCleanSchedule& schedule) {
            strcpy(schedule.time, node["time"].as<std::string>().c_str());
            strcpy(schedule.weeks, node["weeks"].as<std::string>().c_str());
            schedule.mode = node["mode"].as<short>();
            schedule.waterLevel = node["waterLevel"].as<short>();
            strcpy(schedule.areas, node["areas"].as<std::string>().c_str());
            schedule.isEnabled = node["isEnabled"].as<short>();
            schedule.isValid = node["isValid"].as<short>();
        }

        YAML::Emitter& operator<<(YAML::Emitter& out, const tRobotOption& option) {
            out << YAML::BeginMap;
            out << YAML::Key << "waterLevel" << YAML::Value << option.waterLevel;
            out << YAML::Key << "soundLv" << YAML::Value << option.soundLv;
            out << YAML::Key << "country" << YAML::Value << option.country;
            out << YAML::Key << "cleanSchedule" << YAML::Value << YAML::BeginSeq;
            /* 예약청소 */
            for (const auto& schedule : option.cleanSchedule) {
                out << schedule;
            }

            /* 추가 데이터 */

            out << YAML::EndSeq;
            out << YAML::EndMap;
            return out;
        }

        YAML::Emitter& operator<<(YAML::Emitter& out, const tCleanSchedule& schedule) {
            out << YAML::BeginMap;
            out << YAML::Key << "time" << YAML::Value << schedule.time;
            out << YAML::Key << "weeks" << YAML::Value << schedule.weeks;
            out << YAML::Key << "mode" << YAML::Value << schedule.mode;
            out << YAML::Key << "waterLevel" << YAML::Value << schedule.waterLevel;
            out << YAML::Key << "areas" << YAML::Value << schedule.areas;
            out << YAML::Key << "isEnabled" << YAML::Value << schedule.isEnabled;
            out << YAML::Key << "isValid" << YAML::Value << schedule.isValid;
            out << YAML::EndMap;
            return out;
        }

        bool compareRobotOption(const tRobotOption& o1, const tRobotOption& o2) {
            if (o1.waterLevel != o2.waterLevel ||
                o1.soundLv != o2.soundLv ||
                o1.country != o2.country ||
                o1.cleanSchedule.size() != o2.cleanSchedule.size()) {
                return false;
            }

            auto it1 = o1.cleanSchedule.begin();
            auto it2 = o2.cleanSchedule.begin();
            while (it1 != o1.cleanSchedule.end() && it2 != o2.cleanSchedule.end()) {
                if (!compareCleanSchedule(*it1, *it2)) {
                    return false;
                }
                ++it1;
                ++it2;
            }

            return true;
        }

        bool compareCleanSchedule(const tCleanSchedule& s1, const tCleanSchedule& s2) {
            return std::strcmp(s1.time, s2.time) == 0 &&
                std::strcmp(s1.weeks, s2.weeks) == 0 &&
                s1.mode == s2.mode &&
                s1.waterLevel == s2.waterLevel &&
                std::strcmp(s1.areas, s2.areas) == 0 &&
                s1.isEnabled == s2.isEnabled &&
                s1.isValid == s2.isValid;
        }

    }

    namespace iprocess
    {
        int getIndexFromPose(int x, int y, int imgWidth)
        {
            int Index;

            Index = (y*imgWidth) + x;

            return Index;
        }


        /**
         * @brief 사각형 ROI 생성 하여 pdest 에 담는다.
         * 
         * @param pSrc 원본이미지
         * @param imgWidth 이미지 크기
         * @param pDest 관심영역
         * @param left 
         * @param top 
         * @param right 
         * @param bottom 
         */
        void getRoiRectangle(char* pSrc, int imgWidth, char* pDest, int left, int top, int right, int bottom)
        {            
            int areaCnt = 0;
            
            int index = 0;
            for (int r = top; r < bottom; r++)
            {
                for (int c = left; c < right; c++)
                {
                    index = iprocess::getIndexFromPose(r, c, imgWidth);
                    pDest[areaCnt++] = pSrc[index];
                }
            }

        }
        /**
         * @brief 사각형 ROI 생성 하여 pdest 에 담는다.
         * 
         * @param pSrc 원본 이미지
         * @param roiInitIndex 관심영역 시작 인덱스
         * @param pDest 관심영역
         * @param left 
         * @param top 
         * @param right 
         * @param bottom 
         */
        void getRoiRectangle(cell* pSrc, int roiInitIndex, cell* pDest, int left, int top, int right, int bottom)
        {
            int areaCnt = 0;

            for (int r = 0; r < (bottom - top); r++)
            {
                for (int c = 0; c < (right - left); c++)
                {             
                    pDest[areaCnt].b.wall = pSrc[roiInitIndex].b.wall;

                    areaCnt += 1; 
                    roiInitIndex += 1; 
                }
                roiInitIndex = roiInitIndex + CELL_X - (right-left); 
            }

        }

        /**
         *  @brief 이미지를 프린트해서 찍어 보는 디버깅용
        */
        void debug_imgPrint(unsigned char *src, int height, int width, int cropSize)
        {
            std::cout <<std::endl;            
            std::cout <<"-------------------------------------------"<<std::endl;
            std::cout << " debug::     src-crop-print" <<std::endl;
            std::cout <<"-------------------------------------------"<<std::endl;
            std::cout <<std::endl;

            int h = height;
            int w = width;
            int idx = 0;
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    if ( x > cropSize && x < (w - cropSize) ){
                        if ( y > cropSize && y < (h - cropSize) ){
                            eblog(LOG_LV, src[idx]);
                        }
                    }
                    idx++;
                }

                if ( y > cropSize && y < (h - cropSize) ){
                    eblog(LOG_LV, "");
                }    
            }
        }
    }

namespace coordination
{
    /** 
     * @brief m단위의 (x, y) 좌표계를 cell map의 index 값으로 변환함. 
     *  
     * @param coordX m단위 좌표계 x값  
     * @param coordY m단위 좌표계 y값 
     * @param resolution 단위 m/cell 
     * @param cellWidth cell단위 좌표계 width 
     * @param cellHeight cell단위 좌표계 height 
     * @return cell map의 index, -1 이면 오류
     */ 
    int convertCoordi2Index(double coordX, double coordY, double resolution, int cellWidth, int cellHeight)
    {   
        int index = 0;
        int cellSize = cellWidth * cellHeight;
        double originCellX = resolution * (cellWidth / 2) * -1;    
        double originCellY = resolution * (cellHeight / 2) * -1;

        int cellX = (int) ( ( coordX - originCellX ) / resolution + 0.5); //반올림
        int cellY = (int) ( ( coordY - originCellY ) / resolution + 0.5); //반올림
        index = cellX + (cellY * cellWidth);
        
        if( cellX >= cellWidth || cellY >= cellHeight)
        {
            index = -1;
        }

        if (index < 0 || index >= cellSize)
        {
            index = -1;
        }

        return index;
    }

    std::pair<int, int> convertCoordiToCellPoint(double coordX, double coordY, double resolution, int cellWidth, int cellHeight)
    {
        std::pair<int, int> cellPoint;

        int cellSize = cellWidth * cellHeight;
        double originCellX = resolution * (cellWidth / 2) * -1;    
        double originCellY = resolution * (cellHeight / 2) * -1;

        cellPoint.first = (int) ( ( coordX - originCellX ) / resolution + 0.5); //반올림
        cellPoint.second = (int) ( ( coordY - originCellY ) / resolution + 0.5); //반올림
        
        if( cellPoint.first >= cellWidth || cellPoint.second >= cellHeight)
        {
            cellPoint.first = -1;
            cellPoint.second = -1;
            return cellPoint;
        }

        return cellPoint;
    }

    /** 
     * @brief m단위의 (x, y) 좌표계를 cell map의 x, y 값으로 변환함. 
     *  
     * @param coordX m단위 좌표계 x값  
     * @param coordY m단위 좌표계 y값 
     * @param pCellX cell x값  
     * @param pCellY cell y값 
     * @param resolution 단위 m/cell 
     * @param cellWidth cell단위 좌표계 width 
     * @param cellHeight cell단위 좌표계 height 
     * @return cell map의 index, -1 이면 오류
     */ 
    bool convertCoordi2Index(double coordX, double coordY, int *pCellX, int *pCellY, double resolution, int cellWidth, int cellHeight)
    {   
        bool bRet = true;
        int cellSize = cellWidth * cellHeight;
        double originCellX = resolution * (cellWidth / 2) * -1;    
        double originCellY = resolution * (cellHeight / 2) * -1;

        int cellX = (int) ( ( coordX - originCellX ) / resolution + 0.5); //반올림
        int cellY = (int) ( ( coordY - originCellY ) / resolution + 0.5); //반올림
        
        if( cellX >= cellWidth || cellY >= cellHeight || cellX < 0 || cellY < 0){
            bRet = false;
        }

        *pCellX = cellX;
        *pCellY = cellY;

        return bRet;
    }

    /**
     * @brief SLAM map 의 index값을 Clean map 의 index 값으로 변환함.
     * 
     * @param slamIndex SLAM map의 index 값
     * @param slamWidth SLAM map의 width 값 (단위, pixel)
     * @param slamHeight SLAM map의 height 값 (단위, pixel)
     * @param slamResolution SLAM map의 resolution
     * @param slamOriX SLAM map의 origin x 값 (단위, m)
     * @param slamOriY SLAM map의 origin y 값 (단위, m)
     * @param cellWidth Clean map의 width 값 (단위, pixel)
     * @param cellHeight Clean map의 height 값 (단위, pixel)
     * @param slamResolution Clean map의 resolution
     * @return Clean map의 index 값, -1이면 오류
     */
    int convertSlamIndex2CellIndex(int slamIndex,
        int slamWidth, int slamHeight, double slamResolution, double slamOriX, double slamOriY,
        int cellWidth, int cellHeight, double cellResolution)
    {
        int cellIndex;

        int slamCntX = slamIndex % slamWidth;
        int slamCntY = slamIndex / slamWidth;

        double slamCoordiX = slamOriX + slamCntX * slamResolution;
        double slamCoordiY = slamOriY + slamCntY * slamResolution;

        cellIndex = convertCoordi2Index(slamCoordiX, slamCoordiY, cellResolution, cellWidth, cellHeight);
        if ( cellIndex < 0 )
        {
            eblog(LOG_LV, "SLAM MAP[ " << slamIndex << " ] is out of CLEAN MAP[ " << cellIndex << "] index ");
        }

        return cellIndex;
    }

    std::pair<int, int> convertSlamIndex2CellPoint(int slamIndex, int slamWidth, int slamHeight, double slamResolution, double slamOriX, double slamOriY, int cellWidth, int cellHeight, double cellResolution)
    {
        std::pair<int, int> cellPoint;

        int slamCntX = slamIndex%slamWidth;
        int slamCntY = slamIndex/slamWidth;

        double slamCoordiX = slamOriX + slamCntX*slamResolution;
        double slamCoordiY = slamOriY + slamCntY*slamResolution;

        double originCellX = (-0.5)*cellResolution*cellWidth;
        double originCellY = (-0.5)*cellResolution*cellHeight;

        cellPoint.first     = static_cast<int>((slamCoordiX-originCellX)/cellResolution + 0.5); //반올림
        cellPoint.second    = static_cast<int>((slamCoordiY-originCellY)/cellResolution + 0.5); //반올림
        
        if( cellPoint.first >= cellWidth || cellPoint.second >= cellHeight)
        {
            cellPoint.first = -1;
            cellPoint.second = -1;
            eblog(LOG_LV, "SLAM MAP[ " << slamIndex << " ] is out of CLEAN MAP[ "<<"] index ");
        }

        return cellPoint;
    }

    /** 
     * @brief m단위의 (x, y) 좌표계를 cell map의 좌표 값으로 변환함. 
     *  
     * @param coordX m단위 좌표계 x값  
     * @param coordY m단위 좌표계 y값 
     * @param resolution 단위 m/cell 
     * @param cellWidth cell단위 좌표계 width 
     * @param cellHeight cell단위 좌표계 height 
     * @param *cellx cell단위 좌표계 x 
     * @param *cellx cell단위 좌표계 y
     */ 
    void convertCoord2CellCoord(double coordX, double coordY, double resolution, int cellWidth, int cellHeight, int *cellx, int *celly)
    {   
        int index = 0;
        int cellSize = cellWidth * cellHeight;
        double originCellX = (-0.5)*resolution*cellWidth;
        double originCellY = (-0.5)*resolution*cellHeight;

        *cellx = (int) ( ( coordX - originCellX ) / resolution + 0.5); //반올림
        *celly = (int) ( ( coordY - originCellY ) / resolution + 0.5); //반올림        
    }

    const int cellSize = CELL_X * CELL_Y;
    const int originCellX = CELL_RESOLUTUION * (CELL_X / 2) * -1;
    const int originCellY = CELL_RESOLUTUION * (CELL_Y / 2) * -1;
    void convertCoord2CellCoord(double coordX, double coordY, int *cellx, int *celly)
    {   
        int index = 0;        
        
        *cellx = (int) ( ( coordX - originCellX ) / CELL_RESOLUTUION + 0.5); //반올림
        *celly = (int) ( ( coordY - originCellY ) / CELL_RESOLUTUION + 0.5); //반올림        
    }
}

namespace math
{
    /**
     * @brief 두 점 사이의 거리 리턴
     * @return double 
     */
    double distanceTwoPoint(tPoint a, tPoint b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }
    double distanceTwoPoint(tPoint a, tPose b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }
    double distanceTwoPoint(tPose a, tPoint b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }
    double distanceTwoPoint(tPose a, tPose b)//icbaek, 22.10.20 추가 오버라이드
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }
    double distanceTwoPoint(tSysPose a, tSysPose b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }

    /**
     * @brief 아래쪽, 오른쪽, 위쪽, 왼쪽 방향에 대한 radian 값을 가져옴.
     * 
     * @param dir down, right, up, left
     * @return -3.14 <= radian < 3.14 (up방향이 0)
     */
    double getRadianAngle(E_DIRECTION dir)
    {
        switch (dir)
        {
        case E_DIRECTION::DOWN:
            return -PI_;
            break;
        case E_DIRECTION::RIGHT:
            return -PI_/2;
            break;
        case E_DIRECTION::UP:
            return 0;
            break;
        case E_DIRECTION::LEFT:
            return PI_/2;
            break;
        default:
            eblog(LOG_LV, "direction type error!!");
            return 0;
            break;
        }
    }

    double getDegAngle(E_DIRECTION dir)
    {
        switch (dir)
        {
        case E_DIRECTION::DOWN:
            return 180.0;
            break;
        case E_DIRECTION::RIGHT:
            return 270.0;
            break;
        case E_DIRECTION::UP:
            return 0;
            break;
        case E_DIRECTION::LEFT:
            return 90.0;
            break;
        default:
            eblog(LOG_LV, "direction type error!!");
            return 0;
            break;
        }
    }

    /**
     * @brief up angle radian 값이 0이 아닐 경우
     * 
     * @param dir 
     * @param upAngle 
     * @return double 
     */
    double getRadianAngle(E_DIRECTION dir, double upAngle)
    {
        return rad2rad(getRadianAngle(dir) + upAngle);
    }

    /**
     * @brief 두 점 사이의 각도를 구함.
     * 
     * @param target 목표 점
     * @param robot 기준 점
     * @return double 로봇 정면이 0인 각도 (단위:radian) (범위: -3.14 ~ 3.14)
     */
    double getRadianAngle(tPoint target, tPoint robot)
    {
        double radAngle = atan2(target.x - robot.x, target.y - robot.y);
        return rad2rad(radAngle);
    }

    double getRadianAngle(double x, double y)
    {
        double radAngle = atan2(x,y);
        return rad2rad(radAngle);
    }

    /**
     * @brief 두 점 사이의 각도를 구함.
     * 
     * @param target 목표 점
     * @param robot 기준 점
     * @return 로봇 정면이 0인 각도 (단위:radian) (범위: -3.14 ~ 3.14)
     */
    double getRadianAngle(tPoint target, tPose robot)
    {
        double radAngle = atan2(target.y - robot.y, target.x - robot.x);
        return rad2rad(radAngle);
    }

    /**
     * @brief -3.14 <= radian < 3.14 사이의 값으로 변환
     * 
     * @param radian 
     * @return double 
     */
    double rad2rad(double radian)
    {
        radian = std::fmod(radian, 2*M_PI);
        return (radian>=M_PI) ? (radian-2*M_PI) : radian;
    }

    /**
     * @brief degree 의 범위 (0~360)에 맞도록 리턴
     * 
     * @param degree 
     * @return int 
     */
    double deg2deg(double degree)
    {
        degree = std::fmod(degree, 360.0);
        return (degree<0) ? (degree+360.0) : degree;
    }

    double deg2rad(double degree)
    {
        return rad2rad(RAD(degree));
    }

    /**
     * @brief radian을 degree로 변환
     * radian 로봇 정면이 0, 범위는 (-3.14 ~ 3.14)
     * degree 로봇 정면이 0, 범위는 (0 ~ 360)
     * @param radian 
     * @return 
     */
    double rad2deg(double radian)
    {
        return deg2deg(DEG(radian));
    }

    /**
     * @brief 왼쪽으로 돌지, 오른쪽으로 돌지 리턴
     * 
     * @param currentDegree 
     * @param targetDegree 
     * @return true 왼쪽으로 돈다. 반시계 방향
     * @return false 오른쪽으로 돈다. 시계 방향
     */
    bool getTurnDirection(double currentDegree, double targetDegree)
    {
        // TODO: 추후 완성.
        bool ret = true;
        double flagDegree = 0;
        double robotDegree = 0;

        double gap = utils::math::deg2deg( targetDegree - currentDegree );

        if (gap >=180 && gap < 360)
        {
            return false;
        }
        else if (gap >= 0 && gap < 180)
        {
            return true;
        }
        else if (gap >= -180 && gap < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief Target 좌표를 바라보기 위해 로봇이 돌아야 하는 각도를 리턴.
     * 
     * @param target 목표 좌표
     * @param robot 현재 로봇 좌표
     * @return int 돌아야 하는 각도 (단위:degree, 회전방향 +:ccw -:cw)
     */
    double getTurnAngle(tPoint target, tPose robot)
    {
        double targetRadAngle = utils::math::getRadianAngle(target, robot);
        double targetDegAngle = utils::math::rad2deg(targetRadAngle); // degree
        double robotDegAngle = utils::math::rad2deg(robot.angle);
        double errDegAngle = 0;
        
        errDegAngle = targetDegAngle - robotDegAngle;    // degree;

        if ( errDegAngle > 180 )    return errDegAngle-360;
        if ( errDegAngle < -180 )   return errDegAngle+360;
        return errDegAngle;
    }

    /**
     * @brief Target 좌표를 바라보기 위해 로봇이 돌아야 하는 각도를 리턴.
     * 
     * @param target 목표 좌표
     * @param robot 현재 로봇 좌표
     * @param calibAngle SLAM 으로 보정된 System 각도 (단위 degree)
     * @return int 돌아야 하는 각도 (단위:degree, 회전방향 +:ccw -:cw)
     */
    double getTurnAngle(tPoint target, tPose robot, double calibAngle)
    {
        double targetRadAngle = utils::math::getRadianAngle(target, robot);
        double targetDegAngle = utils::math::rad2deg(targetRadAngle); // degree
        double errDegAngle = 0;
        
        errDegAngle = targetDegAngle - calibAngle;    // degree;

        if ( errDegAngle > 180 )    return errDegAngle-360;
        if ( errDegAngle < -180 )   return errDegAngle+360;
        return errDegAngle;
    }

    /**
     * @brief Get the Turn Angle Plus Direction object
     * 
     * @param robot 
     * @param target 
     * @return double 단위 deg
     */
    double getTurnAnglePlusDirection(tPose robot,tPoint target)
    {
        double ret = 0;
        double targetRadAngle = atan2(target.y - robot.y, target.x - robot.x);//utils::math::getRadianAngle(edgePath.front(), robot); // rad 계산 (목표)
        double targetDegAngle = utils::math::rad2deg(targetRadAngle);//utils::math::rad2deg(targetRadAngle); //  rad -> degree (목표)
        double robotDegAngle = utils::math::rad2deg(robot.angle); //  rad -> degree (로봇)
        double degAngleDiff = fabs(targetDegAngle - robotDegAngle); // 로봇과 목표의 각도 deg차이 (절댓값)
        bool bTurnLeft = utils::math::getTurnDirection(robotDegAngle,targetDegAngle);

        if(degAngleDiff > 180) degAngleDiff = 360 - degAngleDiff;

        if(bTurnLeft) ret = degAngleDiff;
        else          ret = -degAngleDiff;

        return ret;  
    }

    /**
     * @brief Target 각도를 바라보기 위해 로봇이 돌아야 하는 각도를 리턴.
     * 
     * @param targetDegAngle 목표 절대 각도 (단위:degree)
     * @param robot 현재 로봇 좌표
     * @return int 돌아야 하는 각도 (단위:degree, 회전방향 +:ccw -:cw)
     */
    double getTurnAngle(int targetDegAngle, tPose robot)
    {
        double robotDegAngle = utils::math::rad2deg(robot.angle);
        double errDegAngle = 0;
        
        errDegAngle = targetDegAngle - robotDegAngle;    // degree;

        if ( errDegAngle > 180 )    return errDegAngle-360;
        if ( errDegAngle < -180 )   return errDegAngle+360;
        return errDegAngle;
    }

    double getTurnAngle(double targetDegAngle, double robotDegAngle)
    {
        robotDegAngle = utils::math::deg2deg(robotDegAngle);
        targetDegAngle = utils::math::deg2deg(targetDegAngle);
        double errDegAngle = 0;
        
        errDegAngle = targetDegAngle - robotDegAngle;    // degree;

        if ( errDegAngle > 180 )    
        {
            errDegAngle -= 360;
        }
        else if ( errDegAngle < -180 )
        {
            errDegAngle += 360;
        }
        return errDegAngle;
    }

    /**
     * @brief 로봇이 목표각도에 도달하기 위해 회전해야 하는 각도를 구함.
     * 
     * @param targetRadAngle 목표각도 (rad)
     * @param robotRadAngle 로봇각도 (rad)
     * @return double 회전각도 (rad, +가 반시계방향)
     */
    double getTurnRadAngle(double targetRadAngle, double robotRadAngle)
    {
        // 각도 범위를 -pi에서 pi로 정규화
        robotRadAngle = fmod(robotRadAngle + M_PI, 2 * M_PI) - M_PI;
        targetRadAngle = fmod(targetRadAngle + M_PI, 2 * M_PI) - M_PI;

        // 반시계방향 각도 변환
        double rotation = targetRadAngle - robotRadAngle;

        // 최단 회전 각 계산
        if (rotation > M_PI)
            rotation -= 2 * M_PI;
        else if (rotation <= -M_PI)
            rotation += 2 * M_PI;

        return rotation;
    }

    /**
     * @brief 로봇이 목표점을 바라보기 위해 회전해야 하는 각도를 구함.
     * 
     * @param targetPoint 목표점 (m, m)
     * @param robotPose 로봇위치 (m, m, rad)
     * @return double 회전각도 (rad, +가 반시계방향)
     */
    double getTurnRadAngle(tPoint targetPoint, tPose robotPose)
    {
        double targetRadAngle = atan2(targetPoint.y - robotPose.y, targetPoint.x - robotPose.x);
        return getTurnRadAngle(targetRadAngle, robotPose.angle);
    }

       /**
     * @brief 로봇이 목표점을 바라보기 위해 회전해야 하는 각도를 구함.
     * 
     * @param targetPoint 목표점 (m, m)
     * @param robotPose 로봇위치 (m, m, rad)
     * @return double 회전각도 (rad, +가 반시계방향)
     */
    double getTurnRadAngle(tPose targetPose, tPose robotPose)
    {
        double targetRadAngle = atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x);
        return getTurnRadAngle(targetRadAngle, robotPose.angle);
    }

    /**
     * @brief Get the Average Slope object
     * 
     * @param points 
     * @return - pi/2 < slope <= pi/2
     */
    double getAverageSlope(std::list<tPoint> points)
    {
        int size = points.size();

        double avgX = 0.0;
        double avgY = 0.0;
        for ( tPoint p : points)
        {
            avgX += p.x;
            avgY += p.y;
        }
        avgX /= size;
        avgY /= size;

        double sum1 = 0.0;
        double sum2 = 0.0;
        for ( tPoint p : points)
        {
            sum1 += (p.x-avgX)*(p.x-avgX);
            sum2 += (p.x-avgX)*(p.y-avgY);
        }

        if ( sum1 == 0 )
        {
            return PI_/2;
        }
        else
        {
            return sum2/sum1;
        }
    }

    /**
     * @brief 두 각사이의 예각을 구함.
     * 
     * @param angle1 -pi ~ pi
     * @param angle2  -pi ~ pi
     * @return double 
     */
    double calculateMinimumRadianAngle(double angle1, double angle2)
    {
        double diff = angle2 - angle1;
        diff = std::fmod(diff + M_PI, 2 * M_PI);  // Wrap the difference between -π and π
        if (diff < 0)
            diff += 2 * M_PI;
        return diff - M_PI;
    }

    /**
     * @brief 두 각사이의 예각을 구함.
     * 
     * @param angle1 0 ~ 360
     * @param angle2  0 ~ 360
     * @return double 0 ~ 360
     */
    double calculateMinimumDegreeAngle(double angle1, double angle2)
    {
        double diff = angle2 - angle1;
        diff = fmod(diff + 360, 360);  // Wrap the difference between 0 and 360 degrees

        if (diff > 180)
            diff = 360 - diff;

        return diff;
    }

    std::string getCurrentTimeString(int country, int option) {
        const char* timeZone = nullptr;

        if (country == 1)
            timeZone = "Asia/Seoul";
        else if (country == 2)
            timeZone = "Asia/Shanghai";
        else {
            std::cerr << "잘못된 국가 코드입니다." << std::endl;
            return "";
        }

        // 시간대 설정
        setenv("TZ", timeZone, 1); // 'TZ' 환경 변수를 설정합니다.
        tzset(); // 'TZ' 환경 변수에 따라 시간대 정보를 초기화합니다.

        // 현재 시간 얻기
        std::time_t currentTime = std::time(nullptr);

        // 현지 시간으로 변환
        std::tm* localTime = std::localtime(&currentTime);

        // 시간대 초기화 (시스템 기본 시간대로 복구)
        unsetenv("TZ"); // 'TZ' 환경 변수를 제거합니다.
        tzset(); // 시스템 기본 시간대로 초기화합니다.

        // 시간 정보를 문자열로 변환
        char buffer[256]; // 충분한 크기의 버퍼를 할당합니다.
        std::size_t result = 0;
        if (option == 1)
            result = std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localTime); //ex: 2024-06-05 16:30:26 
        else if (option == 2)
            result = std::strftime(buffer, sizeof(buffer), "%Y-%m-%d", localTime);
        else if (option == 3)
            result = std::strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
        else if (option == 4)
            result = std::strftime(buffer, sizeof(buffer), "(%A)", localTime); //ex:(Wednesday)
            
        if(result == 0) {
            std::cerr << "버퍼가 충분하지 않습니다." << std::endl;
            return "";
        }
        return std::string(buffer); // 문자열로 변환된 시간을 반환합니다.
    }

    bool cmpTime(const std::string& start, const std::string& end, int country) {
        std::string currentTimeStr = getCurrentTimeString(country, 3);
        
        // 현재 시간을 시:분:초 형식에서 시:분 형식으로 변환
        std::string currentTime = currentTimeStr.substr(0, 5);

        if (currentTime >= start || currentTime < end) {
            std::cout << "청소 안해" << std::endl;
            return false;
        } else {
            std::cout << "청소 해" << std::endl;
            return true;
        }
    }

    // 외적을 이용하여 두 벡터의 방향 판단
    double crossProduct(tPoint a, tPoint b, tPoint c) 
    {
        double x1 = b.x - a.x;
        double y1 = b.y - a.y;
        double x2 = c.x - a.x;
        double y2 = c.y - a.y;
        return x1 * y2 - x2 * y1;
    }

    // 특정 점이 직선의 어느 쪽에 있는지를 판단하는 함수
    // 왼쪽에 있으면 음수, 오른쪽에 있으면 양수, 직선 상에 있으면 0을 반환
    double pointRelativeToLine(tPoint point, tPoint start, tPoint end) 
    {
        return crossProduct(start, end, point);
    }

    // 두 점을 이용하여 Ax+By+C=0 직선을 구하는 함수
    std::list<double> calculateLineEquation(tPoint start, tPoint end) 
    {
        std::list<double> ret;
        // Ax+By+C=0
        double A = end.y - start.y;
        ret.push_back(A);
        double B = start.x - end.x;
        ret.push_back(B);
        double C = start.y*end.x - end.y*start.x;
        ret.push_back(C);

        //ceblog(LOG_LV_NECESSARY, BOLDYELLOW,"ret 사이즈 : " << ret.size());
        return ret;
    }

    // 두 점을 이용하여 Ax+By+C=0 직선을 구하는 함수 (벡터 반환)
    std::vector<double> calculateLineEquationVector(tPoint start, tPoint end) 
    {
        std::vector<double> ret;
        // Ax+By+C=0
        double A = end.y - start.y;
        ret.push_back(A);
        double B = start.x - end.x;
        ret.push_back(B);
        double C = start.y*end.x - end.y*start.x;
        ret.push_back(C);

        //ceblog(LOG_LV_NECESSARY, BOLDYELLOW,"ret 사이즈 : " << ret.size());
        return ret;
    }

    // 특정 점이 주어진 직선과의 거리를 계산하는 함수
    double calculateDistanceFromLine(tPoint point, std::list<double> coeff) 
    {
        if (coeff.size() == 3)
        {
            double A = coeff.front();
            coeff.pop_front();
            double B = coeff.front();
            coeff.pop_front();
            double C = coeff.front();
            coeff.pop_front();
            
            return fabs(A * point.x + B * point.y + C) / sqrt(A*A + B*B);
        }
        else
        {
            ceblog(LOG_LV_NECESSARY, BOLDMAGENTA, "직선의 방정식 생성 오류");

            return 0;
        }
    }

    /**
     * @date 2023/08/02 hhryu
     * @brief value의 범위를 minValue ~ maxValue로 제한함.
     * 
     * @param value 
     * @param minValue 
     * @param maxValue 
     * @return s32 
     */
    s32 clamp(s32 value, s32 minValue, s32 maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }

    /**
     * @date 2023/08/02 hhryu
     * @brief value의 범위를 minValue ~ maxValue로 제한함.
     * 
     * @param value 
     * @param minValue 
     * @param maxValue 
     * @return s32 
     */
    double clamp(double value, double minValue, double maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }

    /**
     * @date 2023/08/02 hhryu
     * @brief 원하는 두 점(좌표가 아니여도)을 지나는 1차 방정식의 기울기와 y절편을 구하는 함수 
     * @brief 굳이 min과 max를 클램핑은 따로 하지 않는다.... 필요하면 필요한 곳에서 clamp함수를 사용하면 됨.
     * @param min(p1)
     * @param max(p2) 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> findLinearEquation(tPoint1D min, tPoint1D max)
    {
        std::pair<double, double> ret;

        if (std::abs(max.x - min.x ) < 0.01) // 기울기가 무한대이다!! 만약 점의 차이가 0이면 시스템 에러를 일으킬 수 있으므로 0을 리턴.
        {
            ceblog(LOG_LV_NECESSARY, BLUE, "Dividing by zero is not allowed. \tp2.x : "<<max.x<<"\tp1.x : "<<min.x);
            ret = std::make_pair(0,0);
        }
        else if (std::abs(max.y - min.y) < 0.01)
        {
            // 이는 기울기가 0인 x축에 평행한 직선이다. --> 어느 x에서든 y를 그대로 리턴함.
            ret = std::make_pair(0, max.y);
        }
        else
        {
            double slope = (max.y - min.y) / (max.x - min.x); // 기울기 계산
            double intercept = min.y - slope * min.x;       // y 절편 계산
            ret = std::make_pair(slope, intercept);
        }

        return ret;
    }

    /**
     * @date 2023/08/02 hhryu
     * @brief range.min ~ range.max 두 점을 지나는 1차 방정식의 기울기와 y 절편을 계산하여 인자로 받은 x 값에 대한 y 값을 선형 보간.
     * @param range, x 
     * @return double y
     */
    double interpolateLinearly(tPoint2D range, double x)
    {
        double y = 0;
        /*  utils::math::findLinearEquation --> 두 점을 지나는 1차 방정식의 기울기 및 y절편을 반환.
            equation.first  // 기울기
            equation.second // y절편
        */
        std::pair<double, double> equation = utils::math::findLinearEquation(range.min, range.max);
        y = equation.first * x + equation.second; // y = mx + a

        return y;
    }

    /**
     * @brief 두 값이 일정 비율 내에서 변화했는지 확인하는 함수
     * 
     * @param value1 
     * @param value2 
     * @param relativeChangeThreshold 
     * @return true 
     * @return false 
     * 
     * @note 연산시간 ms
     * @date 2023-MM-DD
     * @author hhryu
     */
    bool isWithinRelativeChange(double value1, double value2, double relativeChangeThreshold)
    {
        bool bRet = false;
        double absDiff = std::fabs(value1 - value2); // 두 값의 차이를 절대값으로 계산

        // ceblog(LOG_LV, YELLOW, "value1:"<<SC<int>(value1) <<"\tvalue2:"<<SC<int>(value2));

        if (absDiff <= std::numeric_limits<double>::epsilon())
        {
            bRet = true; // 두 값이 모두 0.0인 경우 변화 없음으로 판단
            // ceblog(LOG_LV, BLUE, "TRUE.");
        }
        else
        {
            // 두 값 중에서 큰 값의 비율을 계산
            double maxValue = std::max(std::fabs(value1), std::fabs(value2));
            double relativeChange = absDiff / maxValue;

            bRet = (relativeChange <= relativeChangeThreshold); // 비율이 지정한 임계값 이하인 경우 변화가 미미하다고 판단
            // ceblog(LOG_LV, GREEN, "maxValue:"<<SC<int>(maxValue) <<"\telativeChange:"<<SC<int>(relativeChange)<<"\trelativeChangeThreshold:"<<relativeChangeThreshold<<"\tbRet:"<<SC<int>(bRet));
        }
        
        return bRet; 
    }

    /**
     * @brief 로봇이 목표점을 포함하면서 이동경로에 수직인 직선을 벗어났는지 확인하는 함수 (true: 벗어남 | false: 벗어나지 않음)
     * 
     * @param robotPose 
     * @param startPoint 
     * @param targetPoint 
     * @return true 
     * @return false 
     */
    bool isRobotCrossLine(tPose robotPose, tPoint startPoint, tPoint targetPoint)
    {
        bool ret;

        double startSign    = (startPoint.x - targetPoint.x) * startPoint.x
                            + (targetPoint.y - startPoint.y) * startPoint.y
                            + (targetPoint.x - startPoint.x) * targetPoint.x
                            + (startPoint.y - targetPoint.y) * targetPoint.y;

        double robotSign    = (startPoint.x - targetPoint.x) * robotPose.x
                            + (targetPoint.y - startPoint.y) * robotPose.y
                            + (targetPoint.x - startPoint.x) * targetPoint.x
                            + (startPoint.y - targetPoint.y) * targetPoint.y;

        if (startSign > 0)
        {
            if (robotSign > 0)  {ret = false;}
            else                {ret = true;}
        }
        else
        {
            if (robotSign < 0)  {ret = false;}
            else                {ret = true;}
        }

        // if (ret)
        //     ceblog(LOG_LV_NECESSARY, BOLDRED, " cross over stop. | 시작점 : ( " << startPoint.x << " , " << startPoint.y << " ) | 목표점 : ( " << targetPoint.x << " , " << targetPoint.y << " )");

        return ret;
    }

    double getCrossProduct(const tPoint& pointA, const tPoint& pointB, const tPoint& pointC)
    {
        tPoint vectorAB(pointB.x - pointA.x, pointB.y - pointA.y);
        tPoint vectorBC(pointC.x - pointB.x, pointC.y - pointB.y);
        double magnitudeAB = sqrt(vectorAB.x * vectorAB.x + vectorAB.y * vectorAB.y);
        double magnitudeBC = sqrt(vectorBC.x * vectorBC.x + vectorBC.y * vectorBC.y);

        tPoint unitAB(vectorAB.x/magnitudeAB, vectorAB.y/magnitudeAB);
        tPoint unitBC(vectorBC.x/magnitudeBC, vectorBC.y/magnitudeBC);

        double crossProduct = unitAB.x * unitBC.y - unitAB.y * unitBC.x;

        return crossProduct;      
    }
    
    /**
     * @brief 폐곡선의 회전 방향 결정 (true: 반시계 | false: 시계 방향)
     * 
     * @return true 
     * @return false 
     */
    bool findClosedLoopDirection(const std::list<tPoint>& curvedPath)
    {
        bool ret = true;
        int sum = 0;

        if (curvedPath.size() < 3)
        {
            ceblog(LOG_LV_NECESSARY, BOLDRED, " 폐곡선이 아니네요. Path의 포인트가 2개 이하로 나와요. ");
        }
        else
        {            
            for (auto it1 = curvedPath.begin(); it1 != std::prev(curvedPath.end()); ++it1)
            {
                auto it2 = std::next(it1);
                auto it3 = std::next(it2);
                
                double crossProduct = getCrossProduct(*it1, *it2, *it3);
                if (crossProduct > 0)
                    sum += 1;   // 반시계방향
                else if (crossProduct < 0)
                    sum -= 1;   // 시계방향
                else
                    ceblog(LOG_LV_NECESSARY, BOLDRED, "폐곡선이 일직선이네요.");
            }
        }

        if (sum > 0)            {ret = true;}
        else if (sum < 0)       {ret = false;}
        else                    {ceblog(LOG_LV_NECESSARY, BOLDRED, " 폐곡선의 회전 방향성을 알 수 없어요. path를 다시 뽑아야 해요 ");}

        return ret;
    }

    // 선분의 기울기 계산 함수
    double slope(const tPoint& a, const tPoint& b) {
        if (b.x == a.x) {
            return std::numeric_limits<double>::infinity(); // 수직 선분
        }
        return (b.y - a.y) / (b.x - a.x);
    }
}

namespace area
{
    // 영역의 중심점을 계산하는 함수
    tPoint findCentroid(std::list<tPoint>& area) {
        double sumX = 0, sumY = 0;
        int pointsCount = area.size();
        for (const auto& point : area) {
            sumX += point.x;
            sumY += point.y;
        }
        return {sumX / pointsCount, sumY / pointsCount};
    }

    // 영역 크기 조절 함수 (clipper 라이브러리 사용), 원래는 vector로 받아오는게 좋다. 지금은 일단 함수 내에서 list를 vector로 변환해서 사용한다...
    //ex scaleFactor : 20 => 20% 확장(양수) / -20 => 20% 축소(음수)
    void resizeArea(std::list<tPoint>& orgArea, std::list<tPoint>& scaleArea, double scaleFactor)
    {
        std::vector<tPoint> orgVector(orgArea.begin(), orgArea.end());

        ClipperLib::Path subj; // Clipper 라이브러리에서 사용하는 형식으로 변환 (정수형으로 변환)
        for (const auto& vertex : orgVector) {
            subj << ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(vertex.x * 100), static_cast<ClipperLib::cInt>(vertex.y * 100));
        }

        ClipperLib::ClipperOffset co;
        ClipperLib::Paths solution;
        co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon); // jtMiter(뾰족한 모서리), jtSquare(테이퍼드 모서리), jtRound(라운드 모서리)
        co.Execute(solution, scaleFactor);

        scaleArea.clear();
        for (const auto& path : solution)
        {
            for (auto it = path.begin(); it != path.end(); ++it)
            {
                const ClipperLib::IntPoint& point = *it;
                scaleArea.emplace_back(tPoint(point.X / 100.0, point.Y / 100.0));
            }
        }
    }

    tPoint findRightPoint(std::list<tPoint>& area)
    {
        auto it = area.begin();
        tPoint rightmostPoint = *it; // 시작점을 첫 번째 요소로 설정
        ++it;

        // 리스트를 순회하면서 가장 오른쪽 좌표 찾기
        for (; it != area.end(); ++it) {
            if (it->y < rightmostPoint.y) {
                rightmostPoint = *it;
            }
        }

        return rightmostPoint;
    }

    
    tExtremes findExtremes(std::list<tPoint>& points) {
        auto it = points.begin();
        tExtremes result;
        result.leftmost = *it;
        result.rightmost = *it;
        result.topmost = *it;
        result.bottommost = *it;
        ++it;

        // 리스트를 순회하면서 각 방향의 극점 찾기
        for (; it != points.end(); ++it) {
            if (it->x < result.leftmost.x) {
                result.leftmost = *it;
            }
            if (it->x > result.rightmost.x) {
                result.rightmost = *it;
            }
            if (it->y > result.topmost.y) {
                result.topmost = *it;
            }
            if (it->y < result.bottommost.y) {
                result.bottommost = *it;
            }
        }

        return result;
    }

    tPoint findNearPoint(tPose robotPose, std::list<tPoint>& area)
    {
        double minDistance = std::numeric_limits<double>::max();
        tPoint near;
        
        for (const auto& point : area) {
            double dist = robotPose.distance(point);
            if (dist < minDistance) {
                minDistance = dist;
                near = point;
            }
        }
        return near;
    }


    // 레이-캐스팅 알고리즘을 이용한 내부 판단
    //내부 TRUE, 외부 FALSE
    bool isInside(const tPoint& point, std::list<tPoint>& area) {
        int count = 0;
        bool lastBelow;
        auto it = area.begin();
        auto end = area.end();
        auto first = it;
        tPoint lastPoint = *it;
        lastBelow = (lastPoint.y < point.y);

        if (area.empty())
            return false; // 다각형이 비어있으면 항상 외부

        // 다각형의 각 선분에 대해 검사
        for (it = std::next(it); it != end; ++it) {
            bool currentBelow = (it->y < point.y);
            if (currentBelow != lastBelow) {
                double atX = (it->x - lastPoint.x) * (point.y - it->y) - (it->y - lastPoint.y) * (point.x - it->x);

                if (atX == 0) {
                    return true; // 점이 선분 위에 있음
                }
                if (currentBelow == (atX > 0)) {
                    count++;
                }
            }
            lastPoint = *it;
            lastBelow = currentBelow;
        }

        // 마지막 점과 처음 점을 연결하는 선분 처리
        bool currentBelow = (first->y < point.y);
        if (currentBelow != lastBelow) {
            double atX = (first->x - lastPoint.x) * (point.y - first->y) - (first->y - lastPoint.y) * (point.x - first->x);

            if (atX == 0) {
                return true; // 점이 선분 위에 있음
            }
            if (currentBelow == (atX > 0)) {
                count++;
            }
        }

        return count % 2 == 1; // 교차 횟수가 홀수면 내부
    }

    /**
     * @brief 두 점으로 만든 선분과 x축에 수직한 직선의 교점을 계산하는 함수.
     *
     * @param point1 선분의 한점.
     * @param point2 선분의 또 다른 한점.
     * @param yValue x = yValue 직선
     * @param outputIntersection 교점의 좌표
     * @return true
     * @return false 교점이 없음.
     */
    bool calculateXAxisVerticalIntersection(tPoint point1, tPoint point2, double yValue, tPoint &outputIntersection)
    {
        if (point1.y >= yValue && point2.y >= yValue)
        {
            return false;
        }
        if (point1.y <= yValue && point2.y <= yValue)
        {
            return false;
        }
        if (std::fabs(point1.y - point2.y) < 0.001)
        {
            return false;
        }

        outputIntersection.x = (point2.x * yValue - point1.x * yValue + point2.y * point1.x - point1.y * point2.x) / (point2.y - point1.y);
        outputIntersection.y = yValue;
        return true;
    }

    /**
     * @brief 두 점으로 만든 선분과 y축에 수직한 직선의 교점을 계산하는 함수
     *
     * @param point1 선분의 한점.
     * @param point2 선분의 또 다른 한점.
     * @param xValue y = xValue 직선.
     * @param outputIntersection 교점의 좌표
     * @return true
     * @return false 교점이 없음.
     */
    bool calculateYAxisVerticalIntersection(tPoint point1, tPoint point2, double xValue, tPoint &outputIntersection)
    {
        if (point1.x >= xValue && point2.x >= xValue)
        {
            return false;
        }
        if (point1.x <= xValue && point2.x <= xValue)
        {
            return false;
        }
        if (std::fabs(point1.x - point2.x) < 0.001)
        {
            return false;
        }

        outputIntersection.x = xValue;
        outputIntersection.y = (point2.y * xValue - point1.y * xValue + point2.x * point1.y - point1.x * point2.y) / (point2.x - point1.x);
        return true;
    }

    /**
     * @brief 다각형에서 x축방향 청소의 시작 위치들을 계산.
     *
     * @param minBound 탐색을 시작할 최소 값
     * @param maxBound 탐색을 시작할 최대 값
     * @param polygon 다각형 점들
     * @param wallDistance 시작 위치와 벽 사이의 거리.
     * @return std::list<tPoint>
     */
    std::list<tPoint> calculateXAxisLineStartPoints(tPoint minBound, tPoint maxBound, std::list<tPoint> polygon, double wallDistance)
    {
        const double searchDistance = 1; // 탐색기준 커서 이동거리
        std::list<tPoint> startPoints;
        std::list<tPoint> minIntersections;
        tPoint prePoint;
        double curser, searchLimit;

        /* 1. min y 부터 search */
        curser = minBound.y + wallDistance;
        searchLimit = (minBound.y + maxBound.y) * 0.5;
        while (curser < searchLimit)
        {
            printf("[2-1] [curser y값 %.2lf] 교점 ", curser);
            minIntersections = calculateXAxisLineIntersection(curser, polygon);

            if (minIntersections.size() == 2)
            {
                printf("[2-2] [교점이 2개]\n");
                minIntersections.sort([](const tPoint &a, const tPoint &b)
                                    {  return a.x < b.x; });
                if (minIntersections.back().x - minIntersections.front().x > wallDistance)
                {
                    printf("[2-3] [벽과 거리 체크]\n");
                    double distance;
                    bool isFarFromWall = true;
                    tPoint minStartPoint1 = tPoint(minIntersections.front().x + wallDistance, minIntersections.front().y);
                    tPoint minStartPoint2 = tPoint(minIntersections.back().x - wallDistance, minIntersections.back().y);

                    for (tPoint p : calculateYAxisLineIntersection(minStartPoint1.x, polygon))
                    {
                        distance = std::fabs(p.y - minStartPoint1.y);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", minStartPoint1.x, minStartPoint1.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", minStartPoint1.x, minStartPoint1.y, p.x, p.y, distance);
                        }
                    }

                    for (tPoint p : calculateYAxisLineIntersection(minStartPoint1.x, polygon))
                    {
                        distance = std::fabs(p.y - minStartPoint2.y);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", minStartPoint2.x, minStartPoint2.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", minStartPoint2.x, minStartPoint2.y, p.x, p.y, distance);
                        }
                    }

                    if (isFarFromWall)
                    {
                        printf("[2-4] 시작 위치 선정 완료 (%.2lf %.2lf) (%.2lf %.2lf)\n\n", minStartPoint1.x, minStartPoint1.y, minStartPoint2.x, minStartPoint2.y);
                        startPoints.push_back(minStartPoint1);
                        startPoints.push_back(minStartPoint2);
                        break;
                    }
                }
                else
                {
                    printf("\t[ x ] 두 교점사이가 %.2lf 보다 작아서 버림.\n", wallDistance);
                    minIntersections.clear();
                }
            }
            else
            {
                minIntersections.clear();
            }
            curser += searchDistance;
        }

        /* 2. max y 부터 search */
        std::list<tPoint> maxIntersections;
        prePoint = polygon.back();
        curser = maxBound.y - wallDistance;
        searchLimit = (minBound.y + maxBound.y) * 0.5;
        while (curser > searchLimit)
        {
            printf("[2-1][curser y값 %.2lf] ", curser);
            maxIntersections = calculateXAxisLineIntersection(curser, polygon);
            if (maxIntersections.size() == 2)
            {
                printf("[2-2][교점이 2개]\n");
                maxIntersections.sort([](const tPoint &a, const tPoint &b)
                                    {  return a.x < b.x; });
                if (maxIntersections.back().x - maxIntersections.front().x > wallDistance)
                {
                    printf("[2-3][벽과 거리 체크]\n");
                    double distance;
                    bool isFarFromWall = true;
                    tPoint maxStartPoint1 = tPoint(maxIntersections.front().x + wallDistance, maxIntersections.front().y);
                    tPoint maxStartPoint2 = tPoint(maxIntersections.back().x - wallDistance, maxIntersections.back().y);

                    for (tPoint p : calculateYAxisLineIntersection(maxStartPoint1.x, polygon))
                    {
                        distance = std::fabs(p.y - maxStartPoint1.y);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", maxStartPoint1.x, maxStartPoint1.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", maxStartPoint1.x, maxStartPoint1.y, p.x, p.y, distance);
                        }
                    }

                    for (tPoint p : calculateYAxisLineIntersection(maxStartPoint2.x, polygon))
                    {
                        distance = std::fabs(p.y - maxStartPoint2.y);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", maxStartPoint2.x, maxStartPoint2.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", maxStartPoint2.x, maxStartPoint2.y, p.x, p.y, distance);
                        }
                    }

                    if (isFarFromWall)
                    {
                        printf("[2-4] 시작 위치 선정 완료 (%.2lf %.2lf) (%.2lf %.2lf)\n\n", maxStartPoint1.x, maxStartPoint1.y, maxStartPoint2.x, maxStartPoint2.y);
                        startPoints.push_back(maxStartPoint1);
                        startPoints.push_back(maxStartPoint2);
                        break;
                    }
                }
                else
                {
                    printf("\t[ x ] 두 교점사이가 %.2lf 보다 작아서 버림.\n", wallDistance);
                    maxIntersections.clear();
                }
            }
            else
            {
                maxIntersections.clear();
            }
            curser -= searchDistance;
        }
        return startPoints;
    }

    /**
     * @brief 다각형에서 y축방향 청소의 시작 위치들을 계산.
     *
     * @param minBound 탐색을 시작할 최소 값
     * @param maxBound 탐색을 시작할 최대 값
     * @param polygon 다각형 점들
     * @param wallDistance 시작 위치와 벽 사이의 거리.
     * @return std::list<tPoint>
     */
    std::list<tPoint> calculateYAxisLineStartPoints(tPoint minBound, tPoint maxBound, std::list<tPoint> polygon, double wallDistance)
    {
        const double searchDistance = 1; // 탐색기준 커서 이동거리
        std::list<tPoint> startPoints;
        std::list<tPoint> minIntersections;
        tPoint prePoint;
        double curser, searchLimit;

        /* 1. min x 부터 search */
        curser = minBound.x + wallDistance;
        searchLimit = (minBound.x + maxBound.x) * 0.5;
        while (curser < searchLimit)
        {
            printf("[2-1] [curser x값 %.2lf] 교점 ", curser);
            minIntersections = calculateYAxisLineIntersection(curser, polygon);

            if (minIntersections.size() == 2)
            {
                printf("[2-2] [교점이 2개]\n");
                minIntersections.sort([](const tPoint &a, const tPoint &b)
                                    {  return a.y < b.y; });
                if (minIntersections.back().y - minIntersections.front().y > wallDistance)
                {
                    printf("[2-3] [벽과 거리 체크]\n");
                    double distance;
                    bool isFarFromWall = true;
                    tPoint minStartPoint1 = tPoint(minIntersections.front().x, minIntersections.front().y + wallDistance);
                    tPoint minStartPoint2 = tPoint(minIntersections.back().x, minIntersections.back().y - wallDistance);

                    for (tPoint p : calculateXAxisLineIntersection(minStartPoint1.y, polygon))
                    {
                        distance = std::fabs(p.x - minStartPoint1.x);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", minStartPoint1.x, minStartPoint1.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", minStartPoint1.x, minStartPoint1.y, p.x, p.y, distance);
                        }
                    }

                    for (tPoint p : calculateXAxisLineIntersection(minStartPoint2.y, polygon))
                    {
                        distance = std::fabs(p.x - minStartPoint2.x);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", minStartPoint2.x, minStartPoint2.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", minStartPoint2.x, minStartPoint2.y, p.x, p.y, distance);
                        }
                    }

                    if (isFarFromWall)
                    {
                        printf("[2-4] 시작 위치 선정 완료 (%.2lf %.2lf) (%.2lf %.2lf)\n\n", minStartPoint1.x, minStartPoint1.y, minStartPoint2.x, minStartPoint2.y);
                        startPoints.push_back(minStartPoint1);
                        startPoints.push_back(minStartPoint2);
                        break;
                    }
                }
                else
                {
                    printf("\t[ x ] 두 교점사이가 %.2lf 보다 작아서 버림.\n", wallDistance);
                    minIntersections.clear();
                }
            }
            else
            {
                minIntersections.clear();
            }
            curser += searchDistance;
        }

        /* 2. max x 부터 search */
        std::list<tPoint> maxIntersections;
        prePoint = polygon.back();
        curser = maxBound.x - wallDistance;
        searchLimit = (minBound.x + maxBound.x) * 0.5;
        while (curser > searchLimit)
        {
            printf("[2-1] [curser y값 %.2lf] 교점 ", curser);
            maxIntersections = calculateYAxisLineIntersection(curser, polygon);
            if (maxIntersections.size() == 2)
            {
                printf("[2-2] [교점이 2개]\n");
                maxIntersections.sort([](const tPoint &a, const tPoint &b)
                                    {  return a.y < b.y; });
                if (maxIntersections.back().y - maxIntersections.front().y > wallDistance)
                {
                    printf("[2-3] [벽과 거리 체크]\n");
                    double distance;
                    bool isFarFromWall = true;
                    tPoint maxStartPoint1 = tPoint(maxIntersections.front().x, maxIntersections.front().y + wallDistance);
                    tPoint maxStartPoint2 = tPoint(maxIntersections.back().x, maxIntersections.back().y - wallDistance);

                    for (tPoint p : calculateXAxisLineIntersection(maxStartPoint1.y, polygon))
                    {
                        distance = std::fabs(p.x - maxStartPoint1.x);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", maxStartPoint1.x, maxStartPoint1.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", maxStartPoint1.x, maxStartPoint1.y, p.x, p.y, distance);
                        }
                    }

                    for (tPoint p : calculateXAxisLineIntersection(maxStartPoint2.y, polygon))
                    {
                        distance = std::fabs(p.x - maxStartPoint2.x);
                        if (distance < wallDistance)
                        {
                            isFarFromWall = false;
                            printf("\t[ x ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 가까워요 (거리 %.2lf  < %.2lf)\n", maxStartPoint2.x, maxStartPoint2.y, p.x, p.y, distance, wallDistance);
                        }
                        else
                        {
                            printf("\t[ o ] (%.2lf %.2lf)와 벽(%.2lf %.2lf)\t 벽과 멀어요 (거리 %.2lf)\n", maxStartPoint2.x, maxStartPoint2.y, p.x, p.y, distance);
                        }
                    }

                    if (isFarFromWall)
                    {
                        printf("[2-4] 시작 위치 선정 완료 (%.2lf %.2lf) (%.2lf %.2lf)\n\n", maxStartPoint1.x, maxStartPoint1.y, maxStartPoint2.x, maxStartPoint2.y);
                        startPoints.push_back(maxStartPoint1);
                        startPoints.push_back(maxStartPoint2);
                        break;
                    }
                }
                else
                {
                    printf("\t[ x ] 두 교점사이가 %.2lf 보다 작아서 버림.\n", wallDistance);
                    maxIntersections.clear();
                }
            }
            else
            {
                maxIntersections.clear();
            }
            curser -= searchDistance;
        }
        return startPoints;
    }

    /**
     * @brief x축방향 라인청소을 위한 다각형과 y=yValue 직선이 만나는 교점 계산.
     *
     * @param yValue y=yValue
     * @param polygon 다각형의 꼭지점들
     * @return std::list<tPoint>
     */
    std::list<tPoint> calculateXAxisLineIntersection(double yValue, const std::list<tPoint> &polygon)
    {
        std::list<tPoint> intersections;

        tPoint prePoint = polygon.back();
        for (tPoint point : polygon)
        {
            tPoint intersection;
            if (calculateXAxisVerticalIntersection(prePoint, point, yValue, intersection))
            {
                intersections.push_back(intersection);
                // printf("(%.2lf %.2lf)\t", intersection.x, intersection.y);
            }
            prePoint = point;
        }
        printf("\n");
        return intersections;
    }

    /**
     * @brief y축방향 라인청소를 위한 다각형과 x=xValue 직선이 만나는 교점 계산.
     *
     * @param xValue x=xValue
     * @param polygon 다각형의 꼭지점들
     * @return std::list<tPoint>
     */
    std::list<tPoint> calculateYAxisLineIntersection(double xValue, const std::list<tPoint> &polygon)
    {
        std::list<tPoint> intersections;

        tPoint prePoint = polygon.back();
        for (tPoint point : polygon)
        {
            tPoint intersection;
            if (calculateYAxisVerticalIntersection(prePoint, point, xValue, intersection))
            {
                intersections.push_back(intersection);
                // printf("(%.2lf %.2lf)\t", intersection.x, intersection.y);
            }
            prePoint = point;
        }
        printf("\n");
        return intersections;
    }
}

namespace cleanmap
{
    /**
     * @brief Cleanmap 에서 어떠한 좌표를 벽으로 부터 띄어진 빈 곳의 좌표로 바꿀때 사용.
     * (경로 이동)
     * @param point 벽으로 부터 띄우고 싶은 좌표
     * @param awayCount 벽으로 부터 몇 칸 이상 띄울지.
     * @param cells cleanmap 
     * @return tPoint 
     */
    tPoint getPointAwayFromWall(tPoint point, int awayCount, cell* cells)
    {
        if ( awayCount < 1 ) // away count 가 너무 낮을 때, 계산하지 않음.
        {
            //__color_yellow_;printf("[untils - getPointAwayFromWall] Error, away Count is %d < 1 \n", awayCount);__color_nc_;    
            return point;
        }
        // __color_cyan_;printf("[untils - getPointAwayFromWall] start - away Count is %d\n", awayCount);__color_nc_;

        
        tPoint inputP = point;  // input 포인트 좌표
        tCellPoint inputCp;     // input 셀 포인트 좌표
        tCellPoint resultCp;    // result 셀 포인트 좌표
        tPoint resultP;         // result 포인트 좌표

        /* 1. input P -> input Cp */
        utils::coordination::convertCoord2CellCoord(inputP.x, inputP.y, CELL_RESOLUTUION, CELL_X, CELL_Y, &inputCp.x, &inputCp.y);

        /* 2. inpout Cp -> result Cp */
        int searchIndex; // 벽인지 확인할 인덱스 변수
        bool bFindWall; // break용 변수
        const int searchWidth = 10/2; // 벽 탐색할때 체크할 너비 수

        /* 2-1-1. left 방향 벽 탐색 */
        int leftCount   = 0;
        bFindWall          = false;
        searchIndex     = CELL_INDEX(inputCp.x, inputCp.y);
        for( int i=1; i<=awayCount; i++)
        {
            leftCount       = i;
            searchIndex     = CELL_INDEX_LEFT(searchIndex, 1);

            #if 1    
                if ( cells[searchIndex].b.wall )                     break;
                if ( cells[CELL_INDEX_UP(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_UP(searchIndex, 2)].b.wall ) break;
                if ( cells[CELL_INDEX_DOWN(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_DOWN(searchIndex, 2)].b.wall ) break;
            #else
                /* search index 기준으로 넓게 벽 체크 */
                for ( int n=0; n<=searchWidth; n++)
                {
                    if ( cells.at(CELL_INDEX_UP(searchIndex, searchWidth)).b.wall || cells.at(CELL_INDEX_DOWN(searchIndex, searchWidth)).b.wall )
                    {
                        bFindWall = true;
                        break;
                    }
                }

                if (bFindWall) break;
            #endif
        }

        /* 2-1-2. right 방향 벽 탐색 */
        int rightCount  = 0;
        bFindWall          = false;
        searchIndex     = CELL_INDEX(inputCp.x, inputCp.y);
        for( int i=1; i<=awayCount; i++)
        {
            rightCount      = i;
            searchIndex     = CELL_INDEX_RIGHT(searchIndex, 1);
            
            #if 1    
                if ( cells[searchIndex].b.wall )                     break;
                if ( cells[CELL_INDEX_UP(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_UP(searchIndex, 2)].b.wall ) break;
                if ( cells[CELL_INDEX_DOWN(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_DOWN(searchIndex, 2)].b.wall ) break;
            #else
                /* search index 기준으로 넓게 벽 체크 */
                for ( int n=0; n<=searchWidth; n++)
                {
                    if ( cells.at(CELL_INDEX_UP(searchIndex, searchWidth)).b.wall || cells.at(CELL_INDEX_DOWN(searchIndex, searchWidth)).b.wall )
                    {
                        bFindWall = true;
                        break;
                    }
                }

                if (bFindWall) break;
            #endif
        }

        /* 2-1-3. up 방향 벽 탐색 */
        int upCount     = 0;
        bFindWall          = false;
        searchIndex     = CELL_INDEX(inputCp.x, inputCp.y);
        for( int i=1; i<=awayCount; i++)
        {
            upCount         = i;
            searchIndex     = CELL_INDEX_UP(searchIndex, 1);

            #if 1    
                if ( cells[searchIndex].b.wall )                     break;
                if ( cells[CELL_INDEX_LEFT(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_LEFT(searchIndex, 2)].b.wall ) break;
                if ( cells[CELL_INDEX_RIGHT(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_RIGHT(searchIndex, 2)].b.wall ) break;
            #else
                /* search index 기준으로 넓게 벽 체크 */
                for ( int n=0; n<=searchWidth; n++)
                {
                    if ( cells.at(CELL_INDEX_LEFT(searchIndex, searchWidth)).b.wall || cells.at(CELL_INDEX_RIGHT(searchIndex, searchWidth)).b.wall )
                    {
                        bFindWall = true;
                        break;
                    }
                }

                if (bFindWall) break;
            #endif
        }

        /* 2-1-4. down 방향 벽 탐색 */
        int downCount   = 0;
        bFindWall          = false;
        searchIndex     = CELL_INDEX(inputCp.x, inputCp.y);
        for( int i=1; i<=awayCount; i++)
        {
            downCount       = i;
            searchIndex     = CELL_INDEX_DOWN(searchIndex, 1);

            #if 1    
                if ( cells[searchIndex].b.wall )                     break;
                if ( cells[CELL_INDEX_LEFT(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_LEFT(searchIndex, 2)].b.wall ) break;
                if ( cells[CELL_INDEX_RIGHT(searchIndex, 1)].b.wall ) break;
                if ( cells[CELL_INDEX_RIGHT(searchIndex, 2)].b.wall ) break;
            #else
                /* search index 기준으로 넓게 벽 체크 */
                for ( int n=0; n<=searchWidth; n++)
                {
                    if ( cells.at(CELL_INDEX_LEFT(searchIndex, searchWidth)).b.wall || cells.at(CELL_INDEX_RIGHT(searchIndex, searchWidth)).b.wall )
                    {
                        bFindWall = true;
                        break;
                    }
                }

                if (bFindWall) break;
            #endif
        }
#if 0
        __color_cyan_;
        eblog(LOG_LV, "[untils - getPointAwayFromWall] wall search result ");
        eblog(LOG_LV, "[untils - getPointAwayFromWall]  u: " << upCount);
        eblog(LOG_LV, "[untils - getPointAwayFromWall] l: " << leftCount << "  r: " << rightCount);
        eblog(LOG_LV, "[untils - getPointAwayFromWall]  d: " << downCount);
        __color_nc_;
#endif
        /* 2-2. 탐색에 따른 포인트 이동 */        
        resultCp.x = inputCp.x - (rightCount-leftCount);
        resultCp.y = inputCp.y - (upCount-downCount);
#if 0
        __color_bold_magent_;
        eblog(LOG_LV, "[untils - getPointAwayFromWall] x axis -> (right-left) = " << rightCount-leftCount);
        eblog(LOG_LV, "[untils - getPointAwayFromWall] y axis -> (up-down) = " << upCount-downCount);
        __color_nc_;
#endif
        // __color_cyan_;printf("[untils - getPointAwayFromWall] input cp(%d %d) -> result cp(%d %d)\n", inputCp.x, inputCp.y, resultCp.x, resultCp.y);__color_nc_;

        /* 3. resultCp -> resultP */
        resultP.x = (resultCp.x-CELL_X/2.0)*CELL_RESOLUTUION;
        resultP.y = (resultCp.y-CELL_Y/2.0)*CELL_RESOLUTUION;
        // __color_cyan_;printf("[untils - getPointAwayFromWall] result cp(%d %d) -> result p(%.2lf %.2lf)\n", resultCp.x, resultCp.y, resultP.x, resultP.y);__color_nc_;
#if 0
        __color_bold_cyan_;printf("[untils - getPointAwayFromWall] end! p(%.2lf %.2lf) -> p(%.2lf %.2lf)\n", inputP.x, inputP.y, resultP.x, resultP.y);__color_nc_;
#endif
        return resultP;
    }

    /**
     * @brief 로봇 궤적 좌표 최적화.
     * 1. 좌표의 해상도를 줄임.
     * 2. 연속되는 같은 좌표은 제거
     * 
     * @param origin
     * @param resolution
     * @return std::list<tPoint> 
     */
    std::list<tPoint> optimizePoints(const std::list<tPoint>& origin, double resolution)
    {
        // ceblog(LOG_LV_NECESSARY, GREEN, "start");
        double __startTime = get_system_time();

        std::list<tPoint> optimizedPoints;
        tPoint lastPoint(-999, -999);

        for (const auto& point : origin) {
            // 원칙 1: 좌표 해상도 축소
            tPoint optimizedPoint;
            optimizedPoint.x = std::round(point.x / resolution) * resolution;
            optimizedPoint.y = std::round(point.y / resolution) * resolution;

            // 원칙 2: 연속되는 같은 위치의 좌표 제거
            if ( optimizedPoint == lastPoint) {continue;}

            optimizedPoints.push_back(optimizedPoint);
            lastPoint = optimizedPoint;
        }
        // ceblog(LOG_LV_NECESSARY, GREEN, "end! calc time[ "<<get_system_time(__startTime)*1000<<" ms]\t\tsize[ "<<origin.size()<<" -> "<<optimizedPoints.size()<<" ]");
        return optimizedPoints;
    }
}

namespace os
{
    /**
     * @brief 프로세스 실행 상태 확인.
     * @ 주의사항
     *  - 대기시간 약 100ms!
     *  - 프로세스 이름길이 14자를 넘지 마시오!
     * @param processName 
     * @return true 
     * @return false 
     */
    bool isProcessRunning(const std::string& processName)
    {
        std::string cmd = "pgrep " + processName + " > /dev/null";
        int result = system(cmd.c_str());

        if (result == 0)
            return true;
        else
            return false;
    }

    /**
     * @brief
     * 
     * @return chrono::system_clock::time_point 
     */
    std::chrono::system_clock::time_point now() {
        return std::chrono::system_clock::now();
    }

    /**
     * @brief
     * 
     * @param start 
     * @return double 
     */
    double msElapsedTime(std::chrono::system_clock::time_point start) {
        auto end = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }
}

namespace path
{   

    /**
     * @brief 선분과 점 사이의 수직 거리 구함.
     * 
     * @param pt 
     * @param lineStart 
     * @param lineEnd 
     * @return double 
     */
    double perpendicularDistance(const tPoint& pt, const tPoint& lineStart, const tPoint& lineEnd)
    {
        double dx = lineEnd.x - lineStart.x;
        double dy = lineEnd.y - lineStart.y;

        // Normalise
        double mag = std::sqrt(dx * dx + dy * dy);
        if (mag > 0.0)
        {
            dx /= mag;
            dy /= mag;
        }

        double pvx = pt.x - lineStart.x;
        double pvy = pt.y - lineStart.y;

        // Get dot product (project pv onto normalized direction)
        double pvdot = dx * pvx + dy * pvy;

        // Scale line direction vector
        double dsx = pvdot * dx;
        double dsy = pvdot * dy;

        // Subtract this from pv
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        return std::sqrt(ax * ax + ay * ay);
    }

    double perpendicularDistance(const Point& pt, const Point& lineStart, const Point& lineEnd)
    {
        double dx = lineEnd.x - lineStart.x;
        double dy = lineEnd.y - lineStart.y;

        // Normalise
        double mag = std::sqrt(dx * dx + dy * dy);
        if (mag > 0.0)
        {
            dx /= mag;
            dy /= mag;
        }

        double pvx = pt.x - lineStart.x;
        double pvy = pt.y - lineStart.y;

        // Get dot product (project pv onto normalized direction)
        double pvdot = dx * pvx + dy * pvy;

        // Scale line direction vector
        double dsx = pvdot * dx;
        double dsy = pvdot * dy;

        // Subtract this from pv
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        return std::sqrt(ax * ax + ay * ay);
    }

    /**
     * @brief RDP 알고리즘
     * 선분으로 구성된 곡선을 점 수가 적은 유사한 곡선으로 줄이는 알고리즘.
     * @param pointList 원본 좌표들
     * @param epsilon 입실론이 클수록 단순화가 커짐..
     * @param out 결과 좌표들
     */
    void ramerDouglasPeucker(const std::list<tPoint>& pointList, double epsilon, std::list<tPoint>& out)
    {
        std::list<tPoint> temp;
        if (pointList.size() < 2)
        {
            out = pointList;
            ceblog(LOG_LV_NECESSARY,  BOLDBLUE," throw std::invalid_argument(Not enough points to simplify");
            return;
        }

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = pointList.size() - 1;
        size_t i = 1;

        auto it = pointList.begin();
        std::advance(it, 1);

        for (; i < end; ++i, ++it)
        {
            double d = perpendicularDistance(*it, pointList.front(), pointList.back());
            if (d > dmax)
            {
                index = i;
                dmax = d;
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        if (dmax > epsilon)
        {
            // Recursive call
            std::list<tPoint> recResults1;
            std::list<tPoint> recResults2;

            auto firstLineBegin = pointList.begin();
            auto firstLineEnd = std::next(firstLineBegin, index + 1);
            std::list<tPoint> firstLine(firstLineBegin, firstLineEnd);

            auto lastLineBegin = std::next(firstLineBegin, index);
            std::list<tPoint> lastLine(lastLineBegin, pointList.end());

            ramerDouglasPeucker(firstLine, epsilon, recResults1);
            ramerDouglasPeucker(lastLine, epsilon, recResults2);

            // Build the result list
            temp = std::move(recResults1);
            temp.splice(temp.end(), recResults2);

            if (temp.size() < 2)
            {
                throw std::runtime_error("Problem assembling output");
            }
        }
        else
        {
            // Just return start and end points
            temp.clear();
            temp.push_back(pointList.front());
            temp.push_back(pointList.back());
        }

        out.push_back(temp.front());
        for(tPoint point: temp)
        {
            if(utils::math::distanceTwoPoint(point, out.back())<0.01)
            {
                continue;
            }
            out.push_back(point);
        }
    }

    void ramerDouglasPeucker(const std::list<Point>& pointList, double epsilon, std::list<Point>& out)
    {
        if (pointList.size() < 2)
        {
            out = pointList;
            return;
        }

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = pointList.size() - 1;
        size_t i = 1;

        auto it = pointList.begin();
        std::advance(it, 1);

        for (; i < end; ++i, ++it)
        {
            double d = perpendicularDistance(*it, pointList.front(), pointList.back());
            if (d > dmax)
            {
                index = i;
                dmax = d;
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        if (dmax > epsilon)
        {
            // Recursive call
            std::list<Point> recResults1;
            std::list<Point> recResults2;

            auto firstLineBegin = pointList.begin();
            auto firstLineEnd = std::next(firstLineBegin, index + 1);
            std::list<Point> firstLine(firstLineBegin, firstLineEnd);

            auto lastLineBegin = std::next(firstLineBegin, index);
            std::list<Point> lastLine(lastLineBegin, pointList.end());

            ramerDouglasPeucker(firstLine, epsilon, recResults1);
            ramerDouglasPeucker(lastLine, epsilon, recResults2);

            // Build the result list
            out = std::move(recResults1);
            out.splice(out.end(), recResults2);

            if (out.size() < 2)
            {
                throw std::runtime_error("Problem assembling output");
            }
        }
        else
        {
            // Just return start and end points
            out.clear();
            out.push_back(pointList.front());
            out.push_back(pointList.back());
        }
    }

    /**
     * @brief 로봇이 직선 위에 있는지 아래 있는지 계산.
     * 로봇이 직선과 겹칠 경우는 위로 판단.
     * 직선이 y축 평행일 경우는 왼쪽이 위로 판단.
     * @param robot 
     * @param startLine 
     * @param endLine 
     * @return true 로봇이 직선 위에 있는 경우.
     * @return false 로봇이 직선 아래에 있는 경우.
     */
    bool isAboveLine(tPoint robot, tPoint startLine, tPoint endLine)
    {
        // 기울기가 0인 경우, 겹치거나 왼쪽이 위.
        if ( startLine.x == endLine.x )
        {
            if ( robot.x <= startLine.x )   { return true; }
            else                            { return false; }
        }

        double slope = (endLine.y-startLine.y)/(endLine.x-startLine.x);
        double lineY = slope*(robot.x-startLine.x)+startLine.y;

        if ( robot.y >= lineY ) { return true; }
        else                    { return false; }
    }
}

 /** hjkim - 230906 업데이트 
     * @brief 로봇의 이전 좌표 정보와 갱신된 좌표 정보를 비교해서 로봇의 좌표 정보가 업데이트 되었는지 확인하는 함수.
     * @param temp : 로봇의 이전 좌표 정보 
     * @param current : 로봇의 현재 갱신된 좌표 정보
     * @return true 좌표 업데이트 됨
     * @return false 좌표 업데이트 안됨
     */
bool isUpdatePose(tPose temp, tPose currnet)
{
    bool ret = false;
    if((temp.x != currnet.x) || (temp.y != currnet.y))//if (utils::math::distanceTwoPoint(temp, currnet) >= 0.03)//if((temp.x != currnet.x) || (temp.y != currnet.y)) //(temp.angle != currnet.angle) ||
    {   
        ret = true;
    }

    return ret;
}
}// namespace utils
