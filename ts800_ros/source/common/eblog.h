/**
 * @file eblog.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <bitset>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <cstdarg>
#include <boost/filesystem.hpp>
#include "fileMng.h"
#include "ebtime.h"
#include "commonStruct.h"
#include "define.h"
#include "ebcolor.h"

#define SC static_cast
#define PRECISION(x) std::fixed << std::setprecision(x) // 소수 x자리까지 출력
// hhryu231228 : 원하는 진법(base)로 출력할 수 있도록 하는 define.
#define LOG_NUMERIC_BASE(data, base) std::setbase(base) << data << std::setbase(10)
#define BINARY(data) std::bitset<sizeof(data) * 8>(data) << std::setbase(10)
#define OCTAL(data) std::oct << data << std::dec
#define DEC(data) SC<s32>(data)
#define HEX(data) std::hex << data << std::dec
#define ELSE_ERROR else{ceblog(LOG_LV_ERROR, RED, "!!!Error in non-existent case!!!");} // hhryu240417 : else를 타면 안될 때 디버깅용으로 확인할 수 있는 define

// hhryu230630 : define 1은 비워둠. (eblog사용시 실수로 비트연산자가 아닌 조건식으로 걸 때 문제 생길 수 있음.)
#define LOG_LV              (0x0002)            // 안쓰는 로그... 지우는 중입니다
#define LOG_LV_NECESSARY    (LOG_LV << 1)       // 필요한 로그 (중요 포인트에 걸어둔다. 절대 여러번 찍어서는 안됨.)
#define LOG_LV_ERROR        (LOG_LV << 2)       // 에러 로그

#define LOG_LV_PATHPLAN     (LOG_LV << 3)       // 
#define LOG_LV_EXPLORER     (LOG_LV << 4)       // 
#define LOG_LV_DOCKING      (LOG_LV << 5)       // 도킹 로그
#define LOG_LV_SYSTEMINF    (LOG_LV << 6)       // 시스템 인터페이스
#define LOG_LV_UI           (LOG_LV << 7)       // UI 로그
#define LOG_LV_TILTING      (LOG_LV << 8)       // 틸 팅 로그
#define LOG_LV_LINECLEAN    (LOG_LV << 9)       // linclean 전용
#define LOG_LV_MOTION       (LOG_LV << 10)      // 
#define LOG_LV_LIDAR        (LOG_LV << 11)      // 라이다 센서 버퍼 검색용
#define LOG_LV_CEVA_ONLY    (LOG_LV << 12)      // ceva에 전달할 시료에 적용할 로그
#define LOG_LV_OBSTACLE     (LOG_LV << 13)      // obs 로그
#define LOG_LV_CONTROL      (LOG_LV << 14)      // 제어 로그
#define LOG_LV_SERVICESTEP  (LOG_LV << 15)      // 서비스 스텝 변화 로그
#define LOG_LV_AWS          (LOG_LV << 16)      // AWS 디버깅 로그
#define LOG_LV_WALL         (LOG_LV << 17)      // 벽타기 로그
#define LOG_LV_SYSDEBUG     (LOG_LV << 18)      // 시스템 크리티컬 디버깅
#define LOG_LV_POSE_DEBUG     (LOG_LV << 19)      //슬램, 시스템 좌표 디버깅

#define LOG_LV_TEST         (LOG_LV << 30)      // 최대 30까지만 가능.

#define CLOSE_LOG           0 // LOG_LV_ERROR --> default?
#define DEFULAT_LOG         (LOG_LV_NECESSARY | LOG_LV_ERROR) // 열고 테스트 하면 디버깅에 용이.
#define ALL_LOG             (~LOG_LV_CEVA_ONLY)
// OPEN_LOG에 확인하고싶은 로그 디파인을 OR로 추가합니다. ex) ERROR와 LINECLEAN : OPEN_LOG (LOG_ERROR | LOG_LINECLEAN)
#define OPEN_LOG            (0)

/**********************************************************************/
/**********************************************************************/
// dev_merge할 때는 DEFULAT_LOG 로 바꿔주세요.  //
// release(배포)할 때는 CLOSE_LOG로 바꿔주세요. //
// debug할 때는 OPEN_LOG 로 바꿔주세요.         //
// @date:230315
// hhryu, jspark
#define ACTIVE_LV   (DEFULAT_LOG | OPEN_LOG)   // 수정 포인트
/**********************************************************************/
/**********************************************************************/

#define eblog(lv, str_cout) {\
    if (ACTIVE_LV & lv) {\
        std::cout<<std::right<<std::setw(25)<<__func__<<"():"<<std::setw(4)<<__LINE__<<": "<<str_cout<<std::endl;\
    }\
}0

#define ceblog(lv, color, str_cout) {\
    if (ACTIVE_LV & lv) {\
        std::cout<<color<<std::right<<std::setw(25)<<__func__<<"():"<<std::setw(4)<<__LINE__<<": "<<str_cout<<NC<<std::endl;\
    }\
}0

// ceva 전달용 log 출력 형식
#define cevalog(lv, str_cout) {\
    if (ACTIVE_LV & lv) {\
        double sec = get_system_time();\
        std::cout << std::fixed << std::setprecision(4) << sec << ", " << str_cout << std::endl;\
    }\
}0

// 측정된 시간 출력용 매크로
//
// time 에는 단위가 ms인 시간만 넣어주세요.
#define printTime(active, time_limit, time) {\
    if (active) {\
        if ( time >= time_limit && time < time_limit*2) {\
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " [시간 측정]\t " <<time<< " ms 입니다. >= "<<time_limit<<" ms \t(시간이 상당이 걸려요...)");\
        }\
        else if (time >= time_limit*2) {\
            ceblog(LOG_LV_NECESSARY, BOLDRED, " [시간 측정]\t " <<time<< " ms 입니다. >= "<<time_limit*2<<" ms \t(시간이 너무 많이 걸려요!!)");\
        }\
    }\
}0

/**
 * @brief logWirte()에 전달할 값들 생성 후, logWrite()에 전달 매크로 (std::cout 출력포맷)
 * @param strTime 부팅시간
 * @param strFile 파일이름
 * @param strFunc 함수이름
 * @param nLine 출력라인수
 */
#define __buffer_to_logwrite_by_cout__( format ) {\
    std::stringstream ss;\
    ss << format;\
    \
    std::string strTime = " ";\
    strTime = std::to_string(get_system_time());\
    if ( strTime.size() > 2 ) {\
        strTime.pop_back();strTime.pop_back();\
    }\
    \
    std::string strFile = " ";\
    strFile = __FILE__;\
    \
    std::string strFunc = " ";\
    strFunc = __FUNCTION__;\
    \
    int nLine           = 0;\
    nLine = __LINE__;\
    \
    CFileMng::getInstance().logProc(strTime, strFile, strFunc, nLine, ss.str());\
}\

#if USE_CEVA_LOG_SAVE == 1 // 22.10.13 jspark

enum class E_CEVA_TYPE
{
    IMU,
    WHEEL_ENCODER,
    ROBOT_POSE,
    SLAM_POSE
};

#define CEVA_LOG_PATH "/home/ebot/catkin_ws/src/ts800/software/ros/ts800_ros/log"
#define CEVA_LOG CEbLog::GetInst().__CEVA_LOG_WRITE


class CEbLog
{
private:
    CEbLog()
    { 
        std::cout << "new LogSingleton! " << std::endl;
        bReadyToLogWrite = false;
        bLogFileExist = false;
        startSec = get_system_time();
        
        /* Sequence 번호 초기화 */
        SlamPoseSeq     = 0;
        SystemPoseSeq   = 0;
        ImuSeq          = 0;
        EncSeq          = 0;
    }
    CEbLog(const CEbLog &ref);
    CEbLog &operator=(const CEbLog &ref);
    ~CEbLog()
    {
        std::cout << "delete LogSingleton!" << std::endl;
        ofs.close();
    }

private:
    bool bReadyToLogWrite;
    sec_t startSec;
    std::string packagePath;
    bool bLogFileExist;
    std::string fileName;
    std::ofstream ofs;
    
    size_t SlamPoseSeq;     // Slam Pose Data sequence 번호
    size_t SystemPoseSeq;   // System Pose Data sequence 번호
    size_t ImuSeq;          // Imu Data sequence 번호
    size_t EncSeq;          // Encoder Data sequence 번호

public:
    static CEbLog& GetInst()
    {
        static CEbLog instance_;
        return instance_;
    };

    void __CEVA_LOG_WRITE(E_CEVA_TYPE type, double x, double y, double angle)
    {
        if ( bReadyToLogWrite == false )    { initLogWrite(); }

        ofs << getLogString(type, x, y, angle);
    };

    void __CEVA_LOG_WRITE(E_CEVA_TYPE type, int imuData1, int imuData2, int imuData3, int imuData4, int imuData5, int imuData6)
    {
        if ( bReadyToLogWrite == false )    { initLogWrite(); }

        ofs << getLogString(imuData1, imuData2, imuData3, imuData4, imuData5, imuData6);
    };

    void __CEVA_LOG_WRITE(E_CEVA_TYPE type, int leftCount, double rightCount)
    {
        if ( bReadyToLogWrite == false )    { initLogWrite(); }

        ofs << getLogString(leftCount, rightCount);
    };

private:
    /**
     * @brief Log File 쓰기를 위한 초기화 작업
     * bReadyToLogWrite 가 false 면 한번 수행하여야 함.
     */
    void initLogWrite()
    {
        /* 1. log 디렉토리 존재하는지 확인 */
        if ( isLogDirectory() == false )
        {
            /* log 디렉토리 생성 */
            boost::filesystem::create_directory(CEVA_LOG_PATH);
            std::cout << " Create New Log Directory " << std::endl;
        }

        /* 2. 생성한 ceva log 파일이 존재하는지 확인 */
        if ( bLogFileExist == false )
        {
            fileName = "";
            fileName += CEVA_LOG_PATH;
            fileName += "/everybot_data_";
            fileName += getDay();
            fileName += ".csv";
            bLogFileExist = true;
            ofs.open(fileName);
            std::cout << " Create New Log File Open (" << fileName <<")"<< std::endl;
        }

        bReadyToLogWrite = true;
    }

    /**
     * @brief Log 디렉토리 존재 확인함수
     * 
     * @return true 
     * @return false 
     */
    bool isLogDirectory()
    {
        if ( access(CEVA_LOG_PATH, F_OK) == -1 )
        {
            std::cout << " Log Directory is not exist! " << std::endl;
            return false;
        }
        return true;
    }

    std::string getDay(void)
    {
        time_t ttime = time(0);
        tm* local_time = localtime(&ttime);
        std::string curDay = ""+std::to_string(local_time->tm_year + 1900) + std::to_string(local_time->tm_mon + 1)\
                                + std::to_string(local_time->tm_mday)+ "_" + std::to_string(local_time->tm_hour)+"h"\
                                + std::to_string(local_time->tm_min)+"m"+ std::to_string(local_time->tm_sec)+ "s";
        return curDay;
    }

    std::string getLogString(E_CEVA_TYPE type, double x, double y, double angle)
    {
        std::string logString = "";
        if ( type == E_CEVA_TYPE::ROBOT_POSE )
        {
            logString += std::to_string(SystemPoseSeq++);
            logString += ",";
            logString += std::to_string(get_system_time(startSec));    
            logString += ",system_pose,";
        }
        else // type == E_CEVA_TYPE::SLAM_POSE
        {
            logString += std::to_string(SlamPoseSeq++);
            logString += ",";
            logString += std::to_string(get_system_time(startSec));    
            logString += ",slam_pose,";
        }
        logString += std::to_string(x);
        logString += ",";
        logString += std::to_string(y);
        logString += ",";
        logString += std::to_string(angle);
        logString += "\n";
        std::cout << logString;
        return logString;
    }

    std::string getLogString(int imuData1, int imuData2, int imuData3, int imuData4, int imuData5, int imuData6)
    {
        std::string logString = "";
        logString += std::to_string(ImuSeq++);
        logString += ",";
        logString += std::to_string(get_system_time(startSec));
        logString += ",imu,";
        logString += std::to_string(imuData1);
        logString += ",";
        logString += std::to_string(imuData2);
        logString += ",";
        logString += std::to_string(imuData3);
        logString += ",";
        logString += std::to_string(imuData4);
        logString += ",";
        logString += std::to_string(imuData5);
        logString += ",";
        logString += std::to_string(imuData6);
        logString += "\n";

        return logString;
    }

    std::string getLogString(int leftCount, int rightCount)
    {
        std::string logString = "";
        logString += std::to_string(EncSeq++);
        logString += ",";
        logString += std::to_string(get_system_time(startSec));
        logString += ",wheel_encoder,";
        logString += std::to_string(leftCount);
        logString += ",";
        logString += std::to_string(rightCount);
        logString += "\n";

        return logString;
    }
    
};
#endif