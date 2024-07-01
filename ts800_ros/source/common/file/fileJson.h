/**
 * @file fileJson.h
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
#include "json/json.h"
#include "jsonStruct.h"
#include "fileLog.h"

class CFileJson
{
public:
    CFileJson();
    ~CFileJson();
    void readJsonFile();
    tSwConfigJsonData getConfigJsonData();
    bool isParsingLogJson();
    tLogSettingJsonData getLogJsonData();

private:
    std::string robotConfigJsonFileName;
    std::string logSettingJsonFileName;

    tSwConfigJsonData configData;             // config.json 파싱한 원본
    tLogSettingJsonData logSettingData;     // log.json 파싱한 원본

    bool readJsonFile(const std::string fileName, Json::Value& value);
    void parsingRobotConfigJson(Json::Value value);
    
    bool bParsingLogJson;
    void parsingLogJson(Json::Value value);

    void setDataByType(Json::ValueIterator it, bool &data);
    void setDataByType(Json::ValueIterator it, int &data);
    void setDataByType(Json::ValueIterator it, double &data);
    void setDataByType(Json::ValueIterator it, std::string &data);

public: // debug
    void __debug_print_log_setting();   // debug용 log setting 값 출력
    void __debug_print_config(bool isUseEblog);        // debug용 config 값 출력

};