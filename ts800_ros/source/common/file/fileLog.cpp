#include "fileLog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CFileLog::CFileLog()
{
    /* nothing */
    initLog("ts800_ros");
}

CFileLog::~CFileLog()
{
    if ( file.ofs.is_open() == true)
    {
        file.ofs.close();
    }
}

/**
 * @brief File Log 를 생성하기 위한 초기화 함수.
 * 
 * @param project 우선 ts800-ros or q8-ros 로 사용. TODO: 추후 ros 패키지 이름 변경시. ts800 or q8 사용.
 */
void CFileLog::initLog(std::string project)
{
    file.isExist = false;
    file.extension  = ".txt";
    file.name       = getFileNameStr(file.extension);
    file.path       = getFilePathStr(project);

    /* Debug Log 설정 초기화 */
    logSetting.debug.bUse      = false;
    logSetting.debug.bTime     = true;
    logSetting.debug.bFileName = false;
    logSetting.debug.bFuncName = true;
    logSetting.debug.bLineNum  = false;

    /* Debug Log 설정 초기화 */
    logSetting.file.bUse      = true;
    logSetting.file.bTime     = true;
    logSetting.file.bFileName = false;
    logSetting.file.bFuncName = true;
    logSetting.file.bLineNum  = true;
}

void CFileLog::setLogSetting(tLogSettingJsonData newSetting)
{
    logSetting = newSetting;
}

void CFileLog::logWrite(std::string log)
{
    if ( file.isExist == true )
    {
        fileWrite(log);
        return;
    }
    else // 파일이 없으면 log 파일 생성후 write.
    {
        createLogFile(file.path, file.name);
        if ( file.isExist == true )
        {
            fileWrite(log);
        }
    }
}

void CFileLog::logProc(std::string strTime, std::string strFile, std::string strFunc, int nLine, std::string log)
{
    /* 0. Log printf 할 것인지 확인 */
    if ( logSetting.debug.bUse )
    {
        if ( logSetting.debug.bTime )
        {
            std::cout << "["<< strTime << "] ";
        }
        if ( logSetting.debug.bFileName )
        {
            std::cout << strFile << ":";
        }
        if ( logSetting.debug.bFuncName )
        {
            std::cout << strFunc << "():";
        }
        if ( logSetting.debug.bLineNum )
        {
            std::cout << nLine << ":";
        }
        std::cout <<" "<< log;
    }

    if (logSetting.file.bUse )
    {
        /* 1. Log Setting 에 따라서 저장할 로그 문자열 생성 */
        std::string strLog = "";
        if ( 1 )    { strLog += ("["+strTime+"] ");}
        if ( 0 )    { strLog += (strFile+":");}
        if ( 1 )    { strLog += (strFunc+"():");}
        if ( 1 )    { strLog += (std::to_string(nLine)+":");}
        strLog += " ";
        strLog += log;

        /* 2. 로그 파일 쓰기. 파일이 없으면 생성후 쓰기 */
        if ( file.isExist == true )
        {
            fileWrite(strLog);
            return;
        }
        else // 파일이 없으면 log 파일 생성후 write.
        {
            createLogFile(file.path, file.name);
            if ( file.isExist == true )
            {
                fileWrite(strLog);
            }
        }
    }
}

/**
 * @brief input 경로에 디렉토리 존재 확인 함수
 * 
 * @return true 
 * @return false 
 */
bool CFileLog::isExistDirectory(std::string dirPath)
{
    if ( access(dirPath.c_str(), F_OK) == -1 )
    {
        std::cout << " Log Directory is not exist! " << std::endl;
        return false;
    }
    return true;
}

void CFileLog::createLogFile(std::string pathName, std::string fileName)
{
    std::lock_guard<std::mutex> guard(file.ofsMutex);   // mutex 잠금

    if ( isExistDirectory(file.path) == false ) // log 디렉토리가 없으면 디렉토리 생성
    {
        /* log 디렉토리 생성 */
        boost::filesystem::create_directory(pathName);
        std::cout << " Create New Log Directory " << std::endl;
    }

    file.ofs.open(pathName + fileName);
    if ( file.ofs.is_open() == true )
    {
        std::cout << " Create New Log File Open (" << pathName + fileName <<")"<< std::endl;
        file.isExist = true;
    }
    else
    {
        std::cout << " Create New Log File Failed !!! (" << pathName + fileName <<")"<< std::endl;
        file.isExist = false;
    }
}

/**
 * @brief 문자열 str 를 로그파일에 입력 함수.
 * 
 * @param str 파일 쓰기를 할 문자열
 * @warning file.ofs 사용시 std::endl 가 있어야 쓰기됨.
 */
void CFileLog::fileWrite(std::string str)
{
    std::lock_guard<std::mutex> guard(file.ofsMutex);   // mutex 잠금
    
    if ( file.ofs.fail() || file.ofs.bad() )    { return; }

    if ( str.back() == 10 )
    {
        str.pop_back();
    }
    file.ofs << str << std::endl;
}

/**
 * @brief 현재 날짜와 시간 string 을 가져오는 함수
 * 
 * @return 현재 날짜 string
 */
std::string CFileLog::getDayStr(void)
{
    time_t ttime = time(0);
    tm* local_time = localtime(&ttime);
    std::string curDay = ""+std::to_string(local_time->tm_year + 1900) + std::to_string(local_time->tm_mon + 1)\
                            + std::to_string(local_time->tm_mday)+ "_" + std::to_string(local_time->tm_hour)+"h"\
                            + std::to_string(local_time->tm_min)+"m"+ std::to_string(local_time->tm_sec)+ "s";
    return curDay;
}

/**
 * @brief 파일 이름 string 을 가져오는 함수.
 * 현재 날짜와 시간을 기준으로 파일이름을 생성함.
 * 
 * @param fileExtension 저장할 파일 확장자.
 * @return 저장할 파일 이름
 */
std::string CFileLog::getFileNameStr(std::string fileExtension)
{
    #if 0 // 파일이름에 날짜사용
        std::string name = "";
        name += "everybot_log_";
        name += getDayStr();
        name += fileExtension;
    #else
        std::string name = "eblog.txt";
    #endif
    return name;
}

/**
 * @brief 파일 경로 string 을 가져오는 함수.
 * 프로젝트에 따라 경로가 다름.
 * 
 * @param project ts800 or q8
 * @return 저장할 파일 경로
 */
std::string CFileLog::getFilePathStr(std::string project)
{
    #if 0 // devel/lib/log 안에 저장하는 경로
        std::string path = "";
        path += "/home/ebot/catkin_ws/devel/lib/";
        path += project;
        path += "/log/";
    #else
        std::string path = "";
        path += "/home/ebot/log/";
    #endif
    return path;
}