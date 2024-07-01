#include "fileJson.h"
#include "ebcolor.h"
#include <iomanip>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CFileJson::CFileJson()
{
    configData = tSwConfigJsonData();
    logSettingData = tLogSettingJsonData();

    bParsingLogJson = false;
}

CFileJson::~CFileJson()
{
    /* nothing */
}

void CFileJson::readJsonFile()
{
    robotConfigJsonFileName = "config.json";
    logSettingJsonFileName  = "log_setting.json";

    Json::Value configValue;
    if ( true == readJsonFile( robotConfigJsonFileName, configValue) )
    {
        parsingRobotConfigJson(configValue);
    }

    Json::Value logSettingValue;
    if ( true == readJsonFile( logSettingJsonFileName, logSettingValue) )
    {
        parsingLogJson(logSettingValue);
        bParsingLogJson = true;
    }
}

/**
 * @brief 파싱된 config.json 정보를 가져옴.
 * 
 * @return tSwConfigJsonData 
 */
tSwConfigJsonData CFileJson::getConfigJsonData()
{
    return configData;
}

bool CFileJson::isParsingLogJson()
{
    return bParsingLogJson;
}

/**
 * @brief 파싱된 log.json 정보를 가져옴.
 * 
 * @return tSwConfigJsonData 
 */
tLogSettingJsonData CFileJson::getLogJsonData()
{
    return logSettingData;
}

/**
 * @brief json file 을 읽어서 value 에 저장함.
 * 
 * @param fileName json file 이름
 * @param value json 값이 저장됨.
 * @return true json 읽기 성공.
 * @return false json 읽기 실패.
 */
bool CFileJson::readJsonFile(const std::string fileName, Json::Value& value)
{
    /* 1. 파일 경로 string */
    std::string filePath = "/home/ebot/catkin_ws/devel/lib/ts800_ros/config/";
    filePath += fileName;

    /* 2. Json 파일 읽기 */
    Json::Reader reader;	// Json 의 스크립트 Parsor. Root table을 채워주고 성공실패여부 리턴
    std::ifstream ifs;
    ifs.open( filePath.c_str() );
    if ( ifs.is_open() == false ) // json 파일 존재하는지 확인.
    {
        std::cout << " [CFileJson] " + fileName + " is not Exist! \n";
        return false;
    }
    if ( reader.parse(ifs, value) == false ) // json 파일 깨지지 않았는지 확인.
    {
        std::cout << " [CFileJson] " + fileName + " Parsing Failed! \n";
        return false;
    }
    return true;
}

void CFileJson::parsingRobotConfigJson(Json::Value value)
{
    for ( auto it=value.begin(); it!=value.end(); it++ )
    {
        std::string key = it.key().asCString();
        void* pValue    = (void*)&(*it);

        /* Pid 계수 */
        if      (key == "PID_P")                        { setDataByType(it, configData.pid.kP); }
        else if (key == "PID_I")                        { setDataByType(it, configData.pid.kI); }
        else if (key == "PID_D")                        { setDataByType(it, configData.pid.kD); }

        /* Dstar Settig */
        else if (key == "Dstar_Morphology")             { setDataByType(it, configData.dstar.morphology); }
        else if (key == "Dstar_Default_Speed")          { setDataByType(it, configData.dstar.defaultSpeed); }
        else if (key == "Dstar_Default_Turn_Speed")     { setDataByType(it, configData.dstar.defaultTurnSpeed); }
        else if (key == "Dstar_Slow_Speed")             { setDataByType(it, configData.dstar.slowSpeed); }
        else if (key == "Dstar_Slow_Turn_Speed")        { setDataByType(it, configData.dstar.slowTurnSpeed); }
        else if (key == "Dstar_Tilting_Up_Speed")       { setDataByType(it, configData.dstar.tiltingUpSpeed); }
        else if (key == "Dstar_Tilting_Down_Speed")     { setDataByType(it, configData.dstar.tiltingDownSpeed); }
        else if (key == "Dstar_Speed_Gain")             { setDataByType(it, configData.dstar.speedGain); }
        else if (key == "Dstar_Debug_State")            { setDataByType(it, configData.dstar.bDebugState); }
        else if (key == "Dstar_Debug_Wall")             { setDataByType(it, configData.dstar.bDebugWall); }
        else if (key == "Dstar_Debug_Path")             { setDataByType(it, configData.dstar.bDebugPath); }
        else if (key == "Dstar_Debug_Wheel")            { setDataByType(it, configData.dstar.bDebugWheel); }
        
        /* Clean Settig */
        else if (key == "Clean_Speed")                  { setDataByType(it, configData.clean.speed); }
        else if (key == "Clean_Forward_Dummy_Speed")    { setDataByType(it, configData.clean.dummyForwardSpeed); }
        else if (key == "Clean_Turn_Dummy_Speed")       { setDataByType(it, configData.clean.dummyTurnSpeed); }
        else if (key == "Clean_Approach_Speed")         { setDataByType(it, configData.clean.approachSpeed); }
        else if (key == "Clean_Slow_Speed")             { setDataByType(it, configData.clean.slowSpeed); }
        else if (key == "Clean_Very_Slow_Speed")        { setDataByType(it, configData.clean.verySlowSpeed); }
        else if (key == "Clean_Turn_Speed")             { setDataByType(it, configData.clean.turnSpeed); }
        else if (key == "Clean_Slow_Turn_Speed")        { setDataByType(it, configData.clean.slowTurnSpeed); }
        else if (key == "Clean_Min_Speed")              { setDataByType(it, configData.clean.minSpeed); }
        else if (key == "Clean_Min_Turn_Speed")         { setDataByType(it, configData.clean.minTurnSpeed); }
        else if (key == "Clean_Tilting_Up_Speed")       { setDataByType(it, configData.clean.tiltingUpSpeed); }
        else if (key == "Clean_Tilting_Down_Speed")     { setDataByType(it, configData.clean.tiltingDownSpeed); }
        /* 기타 setting */
        else if (key == "Manual_Move_Forward_Speed")    { setDataByType(it, configData.manualMoveForwardSpeed); }
        else if (key == "Manual_Move_Turn_Speed")       { setDataByType(it, configData.manualMoveTurnSpeed); }
        else if (key == "Avoid_Back_Speed")             { setDataByType(it, configData.avoidBackSpeed); }
        else if (key == "Avoid_Turn_Speed")             { setDataByType(it, configData.avoidTurnSpeed); }
        else if (key == "Adjuest_Heading_Speed")             { setDataByType(it, configData.adjuestHeadingSpeed); }
        else if (key == "motion_ctr_desired_v")             { setDataByType(it, configData.motionCtrDesiredV); }
        else if (key == "motion_ctr_desired_w")             { setDataByType(it, configData.motionCtrDesiredW); }
        /* slam of the motion filter params*/
        else if (key == "max_time_seconds")             { setDataByType(it, configData.maxTimeSeconds); }
        else if (key == "max_distance_meters")             { setDataByType(it, configData.maxDistanceMeters); }
        else if (key == "max_angle_radians")             { setDataByType(it, configData.maxAngleRadians); }
        else if (key == "max_default_time_seconds")             { setDataByType(it, configData.maxDefaultTimeSeconds); }
        else if (key == "max_default_distance_meters")             { setDataByType(it, configData.maxDefaultDistanceMeters); }
        else if (key == "max_default_angle_radians")             { setDataByType(it, configData.maxDefaultAngleRadians); }
        /*is ros publish for debug */
        else if (key == "Ros_Publish_Debug_State")    { setDataByType(it, configData.rosPublishDebug); }

        /* cradle move and turn speed */
        else if (key == "Cradle_Move_Speed")       { setDataByType(it, configData.clean.cradleMoveSpeed); }
        else if (key == "Cradle_Trun_Speed")     { setDataByType(it, configData.clean.CradleTrunSpeed); }
        else if (key == "Dummy_slow_speed")       { setDataByType(it, configData.clean.DummySlowSpeed); }
        else if (key == "Dummy_slow_turn_Speed")       { setDataByType(it, configData.clean.DummySlowTurnSpeed); }

        /* Demo - Cleanarea margin and space of Cleanline */
        else if (key == "Updown_Margin_From_Wall")       { setDataByType(it, configData.clean.updownMarginFromWall); }
        else if (key == "Leftright_Margin_From_Wall")     { setDataByType(it,configData.clean.leftrightMarginFromwall); }
        else if (key == "Space_Of_Cleaning_Lines")       { setDataByType(it, configData.clean.spaceOfCleaningLines); }

        else if (key == "wallTrack_v2_decel_V")       { setDataByType(it, configData.wallTrack_v2_decel_V); }
        else if (key == "wallTrack_v2_accel_V")       { setDataByType(it, configData.wallTrack_v2_accel_V); }
        else if (key == "wallTrack_v2_decel_Dis")       { setDataByType(it, configData.wallTrack_v2_decel_Dis); }
        else if (key == "wallTrack_v2_accel_decel_W")       { setDataByType(it, configData.wallTrack_v2_accel_decel_W); }
        else if (key == "wallTrack_v2_left_accel_decel_W")       { setDataByType(it, configData.wallTrack_v2_left_accel_decel_W); }
        else if (key == "wallTrack_v2_W_Weight")       { setDataByType(it, configData.wallTrack_v2_W_Weight); }
        else if (key == "wallTrack_v2_left_W_Weight")       { setDataByType(it, configData.wallTrack_v2_left_W_Weight); }
        else if (key == "wallTrack_v2_left_V_weight")       { setDataByType(it, configData.wallTrack_v2_left_V_weight); }
        
        else if (key == "wallTrack_v2_slow_W")       { setDataByType(it, configData.wallTrack_v2_slow_W); }
        else if (key == "wallTrack_v2_left_slow_W")       { setDataByType(it, configData.wallTrack_v2_left_slow_W); }
        
        else if (key == "signalTrack_decel_V")       { setDataByType(it, configData.signalTrack_decel_V); }
        else if (key == "signalTrack_accel_V")       { setDataByType(it, configData.signalTrack_accel_V); }
        else if (key == "signalTrack_accel_W")       { setDataByType(it, configData.signalTrack_accel_W); }
        else if (key == "signalTrack_decel_W")       { setDataByType(it, configData.signalTrack_decel_W); }
        else if (key == "signalTrack_rotate_W")     { setDataByType(it,configData.signalTrack_rotate_W); }
        else if (key == "signalTrack_max_V")       { setDataByType(it, configData.signalTrack_max_V); }
        else if (key == "signalTrack_max_W")       { setDataByType(it, configData.signalTrack_max_W); }
        
        else if (key == "tryDock_accel_V")          { setDataByType(it, configData.tryDock_accel_V); }
        else if (key == "tryDock_accel_W")          { setDataByType(it, configData.tryDock_accel_W); }
        else if (key == "tryDock_max_V")            { setDataByType(it, configData.tryDock_max_V);   }
        else if (key == "noSignalTimeOut")            { setDataByType(it, configData.noSignalTimeOut);   }

        else if (key == "wallTrack_v2_rotate_accel_W")       { setDataByType(it, configData.wallTrack_v2_rotate_accel_W); }
        else if (key == "wallTrack_v2_rotate_accel_V")       { setDataByType(it, configData.wallTrack_v2_rotate_accel_V); }
        else if (key == "wallTrack_v2_left_rotate_accel_W")       { setDataByType(it, configData.wallTrack_v2_left_rotate_accel_W); }
        else if (key == "wallTrack_v2_left_rotate_accel_V")       { setDataByType(it, configData.wallTrack_v2_left_rotate_accel_V); }
        else if (key == "wallTrack_v2_left_rotate_W_max")       { setDataByType(it, configData.wallTrack_v2_left_rotate_W_max); }
        else if (key == "wallTrack_v2_left_rotate_V_max")       { setDataByType(it, configData.wallTrack_v2_left_rotate_V_max); }
        else if (key == "wallTrack_v2_oppback_time")       { setDataByType(it, configData.wallTrack_v2_oppback_time); }
        else if (key == "wallTrack_v2_back_time")       { setDataByType(it, configData.wallTrack_v2_back_time); }
        else if (key == "wallTrack_v2_rotate_time")       { setDataByType(it, configData.wallTrack_v2_rotate_time); }
        else if (key == "wallTrack_v2_oppback_accel_V")       { setDataByType(it, configData.wallTrack_v2_oppback_accel_V); }
        else if (key == "wallTrack_v2_oppback_accel_W")       { setDataByType(it, configData.wallTrack_v2_oppback_accel_W); }
        else if (key == "wallTrack_v2_back_V")       { setDataByType(it, configData.wallTrack_v2_back_V); }
        else if (key == "wallTrack_v2_back_W")       { setDataByType(it, configData.wallTrack_v2_back_W); }
        
        else if (key == "pointControl_angle_P_gain")       { setDataByType(it, configData.pointControl_angle_P_gain); }
        else if (key == "pointControl_angle_I_gain")       { setDataByType(it, configData.pointControl_angle_I_gain); }
        else if (key == "pointControl_angle_D_gain")       { setDataByType(it, configData.pointControl_angle_D_gain); }
        else if (key == "pointControl_Line_P_gain")       { setDataByType(it, configData.pointControl_Line_P_gain); }
        else if (key == "pointControl_accel_V")       { setDataByType(it,    configData.pointControl_accel_V); }
        else if (key == "pointControl_decel_dis")       { setDataByType(it,    configData.pointControl_decel_dis); }
        else if (key == "pointControl_max_V")       { setDataByType(it,    configData.pointControl_max_V); }
        else if (key == "pointControl_max_W")       { setDataByType(it,    configData.pointControl_max_W); }
        else if (key == "pointControl_decel_V")       { setDataByType(it,    configData.pointControl_decel_V); }
        else if (key == "pointControl_accel_W")       { setDataByType(it,    configData.pointControl_accel_W); }
        else if (key == "pointControl_decel_W")       { setDataByType(it,    configData.pointControl_decel_W); }
        else if (key == "pointControlReverse_angle_P_gain")       { setDataByType(it, configData.pointControlReverse_angle_P_gain); }
        else if (key == "pointControlReverse_Line_P_gain")       { setDataByType(it, configData.pointControlReverse_Line_P_gain); }
        else if (key == "pointControlReverse_accel_V")       { setDataByType(it,    configData.pointControlReverse_accel_V); }
        else if (key == "pointControlReverse_decel_V")       { setDataByType(it,    configData.pointControlReverse_decel_V); }
        else if (key == "pointControlRotate_deadzone_w")       { setDataByType(it,    configData.pointControlRotate_deadzone_w); }
        
        else if (key == "stopControl_decel_V")       { setDataByType(it,    configData.stopControl_decel_V); }
        else if (key == "stopControl_decel_W")       { setDataByType(it,    configData.stopControl_decel_W); }
        /* Option setting */
        else if (key == "bActivePredictAngle")       { setDataByType(it,    configData.bActivePredictAngle); }
        else if(key == "UnDockingEscapeDistance")         { setDataByType(it,    configData.UnDockingEscapeDistance); }
        else if(key == "avoidWalltrackEndCheckTime")         { setDataByType(it,    configData.avoidWalltrackEndCheckTime); }
        else if(key == "fullChargePercentage")         { setDataByType(it,    configData.fullChargePercentage); }

        else if(key == "lineInterval")         { setDataByType(it,    configData.lineInterval); }
        else if(key == "lineMaxDist")         { setDataByType(it,    configData.lineMaxDist); }
        else if(key == "lineXdelta")         { setDataByType(it,    configData.lineXdelta); }
        else if(key == "mainlineGoalMargin")         { setDataByType(it,    configData.mainlineGoalMargin); }
        else if(key == "sidelineGoalMargin")         { setDataByType(it,    configData.sidelineGoalMargin); }
        else if(key == "cleanBoundPixel")         { setDataByType(it,    configData.cleanBoundPixel); }
        else if(key == "lineSideIntervalK")         { setDataByType(it,    configData.lineSideIntervalK); }
		else if (key == "robot_ip")       { setDataByType(it, configData.robot_ip); }

        //odom & lidar & muc time late
        else if (key == "late_Of_odom_sec")             { setDataByType(it, configData.lateOfodomSec); }    
        else if (key == "late_Of_lidar_sec")          { setDataByType(it, configData.lateOfLidarSec); }
        else if (key == "late_Of_mcu_ms")             { setDataByType(it, configData.lateOfMcuMs); }
        
        else if (key == "limitAccelX")             { setDataByType(it, configData.limitAccelX); }
        else if (key == "limitAccelY")             { setDataByType(it, configData.limitAccelY); }
        else if (key == "limitAccelZ")             { setDataByType(it, configData.limitAccelZ); }
        else if (key == "limitRoll")             { setDataByType(it, configData.limitRoll); }
        else if (key == "limitPitch")             { setDataByType(it, configData.limitPitch); }
        else if (key == "limitYaw")             { setDataByType(it, configData.limitYaw); }

        else if (key == "standAccelX")             { setDataByType(it, configData.standAccelX); }
        else if (key == "standAccelY")             { setDataByType(it, configData.standAccelY); }
        else if (key == "tilUpstandAccelY")             { setDataByType(it, configData.tilUpstandAccelY); }
        else if (key == "standAccelZ")             { setDataByType(it, configData.standAccelZ); }
        else if (key == "standRoll")             { setDataByType(it, configData.standRoll); }
        else if (key == "standPitch")             { setDataByType(it, configData.standPitch); }

        else if (key == "limitCliff")             { setDataByType(it, configData.limitCliff); }
        else if (key == "limitWall")             { setDataByType(it, configData.limitWall); }

        else if (key == "pwmAccel")             { setDataByType(it, configData.pwmAccel); }
        else if (key == "pwmDeccel")             { setDataByType(it, configData.pwmDeccel); }

        else if (key == "shrink_area_scale_factor")             { setDataByType(it, configData.shrink_area_scale_factor); }
        //예상 청소 시간 및 물 공급 간격
        else if (key == "motionController_waypoint_dist")             { setDataByType(it, configData.motionController_waypoint_dist); }
        else if (key == "water_supply_clean_time")             { setDataByType(it, configData.waterSupplyCleanTime); }
        else if (key == "water_supply_interval")             { setDataByType(it, configData.waterSupplyInterval); }
        else if (key == "ota_timeout")             { setDataByType(it, configData.ota_timeout); }
        else if (key == "standWheelTrap")             { setDataByType(it, configData.standWheelTrap); }
        else if (key == "lidar_max_infi")             { setDataByType(it, configData.lidar_max_infi); }
        else if (key == "movePathTimeLimit")             { setDataByType(it, configData.movePathTimeLimit); }
        else if (key == "sysWheelErrorCount")             { setDataByType(it, configData.sysWheelErrorCount); }
        else
        {
            printf("[parsingRobotConfigJson] invalid key !\n");
        }
    }
}

void CFileJson::parsingLogJson(Json::Value value)
{
    for ( auto it=value.begin(); it !=value.end(); it++ )
    {
        std::string key = it.key().asCString();
        void* pValue    = (void *)&(*it);

        // TODO: parse
        if      (key == "LOG_DEBUG_USE")                { setDataByType(it, logSettingData.debug.bUse); }
        else if (key == "LOG_DEBUG_ADD_TIME")           { setDataByType(it, logSettingData.debug.bTime); }
        else if (key == "LOG_DEBUG_ADD_FILE_NAME")      { setDataByType(it, logSettingData.debug.bFileName); }
        else if (key == "LOG_DEBUG_ADD_FUNC_NAME")      { setDataByType(it, logSettingData.debug.bFuncName); }
        else if (key == "LOG_DEBUG_ADD_LINE")           { setDataByType(it, logSettingData.debug.bLineNum); }
        else if (key == "LOG_SAVE_USE")                 { setDataByType(it, logSettingData.file.bUse); }
        else if (key == "LOG_SAVE_ADD_TIME")            { setDataByType(it, logSettingData.file.bTime); }
        else if (key == "LOG_SAVE_ADD_FILE_NAME")       { setDataByType(it, logSettingData.file.bFileName); }
        else if (key == "LOG_SAVE_ADD_FUNC_NAME")       { setDataByType(it, logSettingData.file.bFuncName); }
        else if (key == "LOG_SAVE_ADD_LINE")            { setDataByType(it, logSettingData.file.bLineNum); }
        else
        {
            printf("[parsingLogJson] invalid key !\n");
        }
    }
}

/**
 * @brief key에 해당하는 값이 숫자면 타입에 맞도록 저장함.
 * (static_cast 을 할 경우, 타입이 다르면 잘못된 수치가 저장됨. 주의)
 * @param it Json::Value 의 it
 * @param data 저장될 위치
 */
void CFileJson::setDataByType(Json::ValueIterator it, bool &data)
{
    if ( (*it).isBool() )        { data = (*it).asBool(); }
    else { std::cout << "parsing data type is wrong!  - not boolean\n"; }
}

/**
 * @brief key에 해당하는 값이 숫자면 타입에 맞도록 저장함.
 * (static_cast 을 할 경우, 타입이 다르면 잘못된 수치가 저장됨. 주의)
 * @param it Json::Value 의 it
 * @param data 저장될 위치
 */
void CFileJson::setDataByType(Json::ValueIterator it, int &data)
{
    if ( (*it).isDouble() )        { data = (*it).asDouble(); }
    else if ( (*it).isInt() )      { data = (*it).asInt(); }
    else { std::cout << "parsing data type is wrong!  - not number\n"; }
}

/**
 * @brief key에 해당하는 값이 숫자면 타입에 맞도록 저장함.
 * (static_cast 을 할 경우, 타입이 다르면 잘못된 수치가 저장됨. 주의)
 * @param it Json::Value 의 it
 * @param data 저장될 위치
 */
void CFileJson::setDataByType(Json::ValueIterator it, double &data)
{
    if ( (*it).isDouble() )        { data = (*it).asDouble(); }
    else if ( (*it).isInt() )      { data = (*it).asInt(); }
    else { std::cout << "parsing data type is wrong!  - not number\n"; }
}

/**
 * @brief key에 해당하는 값이 문자열 타입에 맞도록 저장함.
 * (static_cast 을 할 경우, 타입이 다르면 잘못된 수치가 저장됨. 주의)
 * @param it Json::Value 의 it
 * @param data 저장될 위치
 */
void CFileJson::setDataByType(Json::ValueIterator it, std::string &data)
{
    if ( (*it).isString() )        { data = (*it).asString(); }
    else { std::cout << "parsing data type is wrong!  - not string\n"; }
}

/**
 * @brief log setting 값 출력함수. (debug용)
 * 
 */
void CFileJson::__debug_print_log_setting()
{
    printf("\n=================================================\n");
    printf("log_setting.json - debug\n");
    printf("use \t\t%d\n",      logSettingData.debug.bUse);
    printf("time \t\t%d\n",     logSettingData.debug.bTime);
    printf("file name \t%d\n",logSettingData.debug.bFileName);
    printf("func name \t%d\n",logSettingData.debug.bFuncName);
    printf("line \t\t%d\n",     logSettingData.debug.bLineNum);
    
    printf("log_setting.json - file\n");
    printf("use \t\t%d\n",       logSettingData.file.bUse);
    printf("time \t\t%d\n",      logSettingData.file.bTime);
    printf("file name \t%d\n", logSettingData.file.bFileName);
    printf("func name \t%d\n", logSettingData.file.bFuncName);
    printf("line \t\t%d\n",      logSettingData.file.bLineNum);
    printf("=================================================\n");
}

/**
 * @brief config 값 출력함수. (debug용)
 * 
 */
void CFileJson::__debug_print_config(bool isUseEblog)
{
    if(isUseEblog)
    {
        std::cout<<BOLDMAGENTA<<"\n==============================================================="<<std::endl;
        std::cout<<BOLDCYAN<<"     _                    ____             __ _       "<<std::endl;
        std::cout<<BOLDCYAN<<"    | |___  ___  _ __    / ___|___  _ __  / _(_) __ _ "<<std::endl;
        std::cout<<BOLDCYAN<<" _  | / __|/ _ \\| '_ \\  | |   / _ \\| '_ \\| |_| |/ _` |"<<std::endl;
        std::cout<<BOLDCYAN<<"| |_| \\__ \\ (_) | | | | | |__| (_) | | | |  _| | (_| |"<<std::endl;
        std::cout<<BOLDCYAN<<" \\___/|___/\\___/|_| |_|  \\____\\___/|_| |_|_| |_|\\__, |"<<std::endl;
        std::cout<<BOLDCYAN<<"                                            	|___/ "<<std::endl;  
        std::cout<<BOLDMAGENTA<<"==============================================================="<<std::endl;
        std::cout<<BOLDCYAN<<std::right<<std::setw(30)<<"PID Gain"<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"P Gain : "<<configData.pid.kP<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"I Gain : "<<configData.pid.kI<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"D Gain : "<<configData.pid.kD<<std::endl;
        std::cout<<BOLDMAGENTA<<"==============================================================="<<std::endl;
        std::cout<<BOLDCYAN<<std::right<<std::setw(30)<<"Dstar Setting"<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"morphology : "<<configData.dstar.morphology<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Default Speed : "<<configData.dstar.defaultSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Default Turn Speed : "<<configData.dstar.defaultTurnSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Slow Speed : "<<configData.dstar.slowSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Slow Turn Speed : "<<configData.dstar.slowTurnSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Speed Gain : "<<configData.dstar.speedGain<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Tilting Up Speed : "<<configData.dstar.tiltingUpSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Tilting Down Speed : "<<configData.dstar.tiltingDownSpeed<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Debug State : "<<configData.dstar.bDebugState<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Debug Wall : "<<configData.dstar.bDebugWall<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Debug Path : "<<configData.dstar.bDebugPath<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Debug Wheel : "<<configData.dstar.bDebugWheel<<std::endl;
        std::cout<<BOLDMAGENTA<<"==============================================================="<<std::endl;
        std::cout<<BOLDCYAN<<std::right<<std::setw(30)<<"Clean Setting"<<std::endl;
        std::cout<<BOLDMAGENTA<<std::right<<std::setw(30)<<"이하 생략 ..."<<std::endl;
        printf("config.json - Clean\n");
        printf("Speed \t\t\t%d\n",              configData.clean.speed);
        printf("Dummy Forward Speed \t\t\t%d\n",              configData.clean.dummyForwardSpeed);
        printf("Dummy Turn Speed \t\t\t%d\n",              configData.clean.dummyTurnSpeed);
        printf("Approach Speed \t\t%d\n",     configData.clean.approachSpeed);
        printf("Slow Speed \t\t%d\n",         configData.clean.slowSpeed);
        printf("Very Slow Speed \t%d\n",    configData.clean.verySlowSpeed);
        printf("Turn Speed \t\t%d\n",         configData.clean.turnSpeed);
        printf("Slow Turn Speed \t%d\n",    configData.clean.slowTurnSpeed);
        printf("Min Speed \t\t%d\n",          configData.clean.minSpeed);
        printf("Min Turn Speed \t\t%d\n",     configData.clean.minTurnSpeed);
        printf("Tilting Up Speed\t\t\t%.2lf\n",   configData.clean.tiltingUpSpeed);
        printf("Tilting Down Speed\t\t\t%.2lf\n",   configData.clean.tiltingDownSpeed);

        printf("AAdjuest Heading Speed \t\t%d\n",          configData.adjuestHeadingSpeed);
        printf("Avoid turn speed \t\t%d\n",          configData.avoidTurnSpeed);
        printf("Ros Publish Debug \t\t%d\n",          configData.rosPublishDebug);

        printf("max_time_seconds \t\t%.2lf\n",          configData.maxTimeSeconds);
        printf("max_distance_meters \t\t%.2lf\n",          configData.maxDistanceMeters);
        printf("max_angle_radians \t\t%.2lf\n",          configData.maxAngleRadians);

        printf("max_default_time_seconds \t\t%.2lf\n",          configData.maxDefaultTimeSeconds);
        printf("max_default_distance_meters \t\t%.2lf\n",          configData.maxDefaultDistanceMeters);
        printf("max_default_angle_radians \t\t%.2lf\n",          configData.maxDefaultAngleRadians);

        printf("cradle Move Speed\t\t\t%.2lf\n",   configData.clean.cradleMoveSpeed);
        printf("Cradle Trun Speed\t\t\t%.2lf\n",   configData.clean.CradleTrunSpeed);

        printf("Dummy Slow Speed\t\t\t%.2lf\n",   configData.clean.DummySlowSpeed);
        printf("Dummy Slow TurnSpeed\t\t\t%.2lf\n",   configData.clean.DummySlowTurnSpeed);

        std::cout<<GREEN<<std::right<<std::setw(30)<<"Updown_Margin_From_Wall : "<<configData.clean.updownMarginFromWall<<" m"<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Leftright_Margin_From_Wall : "<<configData.clean.leftrightMarginFromwall<<" m"<<std::endl;
        std::cout<<GREEN<<std::right<<std::setw(30)<<"Space_Of_Cleaning_Lines : "<<configData.clean.spaceOfCleaningLines<<" m"<<std::endl;

        printf("late Of odom Sec\t\t\t%.2lf\n",          configData.lateOfodomSec);
        printf("late Of Lidar Sec\t\t\t%.2lf\n",          configData.lateOfLidarSec);
        printf("late Of MCU ms\t\t\t%d\n",          configData.lateOfMcuMs);
        printf("water supply clean time(min)\t\t\t%d\n",          configData.waterSupplyCleanTime);
        printf("water supply interval(s)\t\t\t%d\n",          configData.waterSupplyInterval);
        printf("lidar_max_infi\t\t\t%d\n",          configData.lidar_max_infi);
        std::cout<<BOLDMAGENTA<<"==============================================================="<<std::endl;
    }
    else
    {
        printf("\n=================================================\n");
        printf("config.json - PID\n");
        printf("P \t%.2lf\n",      configData.pid.kP);
        printf("I \t%.2lf\n",      configData.pid.kI);
        printf("D \t%.2lf\n",      configData.pid.kD);
        
        printf("config.json - Dstar\n");
        printf("morphology \t\t%d\n",             configData.dstar.morphology);
        printf("Default Speed \t\t%.2lf\n",       configData.dstar.defaultSpeed);
        printf("Default Turn Speed \t%.2lf\n",  configData.dstar.defaultTurnSpeed);
        printf("Slow Speed \t\t%.2lf\n",          configData.dstar.slowSpeed);
        printf("Slow Turn Speed \t%.2lf\n",     configData.dstar.slowTurnSpeed);
        printf("Speed Gain \t\t\t%.2lf\n",               configData.dstar.speedGain);
        printf("Tilting Up Speed\t\t\t%.2lf\n",   configData.dstar.tiltingUpSpeed);
        printf("Tilting Down Speed\t\t\t%.2lf\n",   configData.dstar.tiltingDownSpeed);
        printf("Debug State \t\t%d\n",            configData.dstar.bDebugState);
        printf("Debug Wall \t\t%d\n",             configData.dstar.bDebugWall);
        printf("Debug Path \t\t%d\n",             configData.dstar.bDebugPath);
        printf("Debug Wheel \t\t%d\n",            configData.dstar.bDebugWheel);
        
        printf("config.json - Clean\n");
        printf("Speed \t\t\t%d\n",              configData.clean.speed);
        printf("Dummy Forward Speed \t\t\t%d\n",              configData.clean.dummyForwardSpeed);
        printf("Dummy Turn Speed \t\t\t%d\n",              configData.clean.dummyTurnSpeed);
        printf("Approach Speed \t\t%d\n",     configData.clean.approachSpeed);
        printf("Slow Speed \t\t%d\n",         configData.clean.slowSpeed);
        printf("Very Slow Speed \t%d\n",    configData.clean.verySlowSpeed);
        printf("Turn Speed \t\t%d\n",         configData.clean.turnSpeed);
        printf("Slow Turn Speed \t%d\n",    configData.clean.slowTurnSpeed);
        printf("Min Speed \t\t%d\n",          configData.clean.minSpeed);
        printf("Min Turn Speed \t\t%d\n",     configData.clean.minTurnSpeed);
        printf("Tilting Up Speed\t\t\t%.2lf\n",   configData.clean.tiltingUpSpeed);
        printf("Tilting Down Speed\t\t\t%.2lf\n",   configData.clean.tiltingDownSpeed);

        printf("AAdjuest Heading Speed \t\t%d\n",          configData.adjuestHeadingSpeed);
        printf("Avoid turn speed \t\t%d\n",          configData.avoidTurnSpeed);
        printf("Ros Publish Debug \t\t%d\n",          configData.rosPublishDebug);

        printf("max_time_seconds \t\t%.2lf\n",          configData.maxTimeSeconds);
        printf("max_distance_meters \t\t%.2lf\n",          configData.maxDistanceMeters);
        printf("max_angle_radians \t\t%.2lf\n",          configData.maxAngleRadians);

        printf("max_default_time_seconds \t\t%.2lf\n",          configData.maxDefaultTimeSeconds);
        printf("max_default_distance_meters \t\t%.2lf\n",          configData.maxDefaultDistanceMeters);
        printf("max_default_angle_radians \t\t%.2lf\n",          configData.maxDefaultAngleRadians);

        printf("cradle Move Speed\t\t\t%.2lf\n",   configData.clean.cradleMoveSpeed);
        printf("Cradle Trun Speed\t\t\t%.2lf\n",   configData.clean.CradleTrunSpeed);

        printf("Dummy Slow Speed\t\t\t%.2lf\n",   configData.clean.DummySlowSpeed);
        printf("Dummy Slow TurnSpeed\t\t\t%.2lf\n",   configData.clean.DummySlowTurnSpeed);

        printf("Updown_Margin_From_Wall\t\t\t%.2lf\n",   configData.clean.updownMarginFromWall);
        printf("Leftright_Margin_From_Wall\t\t\t%.2lf\n",   configData.clean.leftrightMarginFromwall);
        printf("Space_Of_Cleaning_Lines\t\t\t%.2lf\n",   configData.clean.spaceOfCleaningLines);

        printf("late Of odom Sec\t\t\t%.2lf\n",          configData.lateOfodomSec);
        printf("late Of Lidar Sec\t\t\t%.2lf\n",          configData.lateOfLidarSec);
        printf("late Of MCU ms\t\t\t%d\n",          configData.lateOfMcuMs);
        printf("water supply clean time(min)\t\t\t%d\n",          configData.waterSupplyCleanTime);
        printf("water supply interval(s)\t\t\t%d\n",          configData.waterSupplyInterval);
        printf("lidar_max_infi\t\t\t%d\n",          configData.lidar_max_infi);
        printf("=================================================\n");
    }
}
