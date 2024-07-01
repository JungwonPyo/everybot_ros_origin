/**
 * @file fileLog.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <boost/filesystem.hpp>
#include "jsonStruct.h"

/**
 * @brief 파일 write을 위한 정보가 담기는 구조체.
 * 
 */
struct tLogFileInfo
{
    std::string extension;  // 확장자
    std::string name;       // 파일 이름
    std::string path;       // 파일 경로

    bool isExist;           // 파일이 생성되었는지 확인!!

    std::ofstream ofs;      // 파일 write 용 ofstream
    std::mutex ofsMutex;    // 파일 write 뮤텍스
};

class CFileLog
{
public:
    CFileLog();
    ~CFileLog();
    void initLog(std::string project);
    void setLogSetting(tLogSettingJsonData newSetting);
    void logWrite(std::string log);
    void logProc(std::string strTime, std::string strFile, std::string strFunction, int nLine, std::string log);

private:
    tLogSettingJsonData logSetting;   // json에서 파싱된 log setting 정보
    tLogFileInfo file;          // 파일 write 위한 정보

    bool isExistDirectory(std::string dirPath);
    void createLogFile(std::string pathName, std::string fileName);
    void fileWrite(std::string str);
    std::string getDayStr(void);
    std::string getFileNameStr(std::string fileExtension);
    std::string getFilePathStr(std::string project);
};