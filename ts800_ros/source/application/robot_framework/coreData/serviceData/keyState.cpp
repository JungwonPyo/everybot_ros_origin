#include "externData/externData.h"
#include "coreData/serviceData/keyState.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CKeyState::CKeyState()
{
    initKeyState();
}

CKeyState::~CKeyState() {}

void CKeyState::update(CExternData* pExternData)
{
    /* buttonKey 가공 */
    button.setButtonKey( pExternData->systemData.useButtonData() );
    E_FUNCTION_BUTTON buttonKey = button.getButtonKey();

    /* actionKey 가공(action만) */
    appCmd.setActionKey(pExternData->appData.getAppCmd());
    E_ACTION_KEY actionKey = appCmd.getActionKey();
    
    rosCmd.setRosKey( pExternData->rosData.getRosCmd() );
    E_ROS_KEY rosKey = rosCmd.getRosKey();
  
    setAppOption(pExternData);

    // Key 우선순위
    // 1. Button
    // 2. app command
    // 3. ros command

    if ( buttonKey != E_FUNCTION_BUTTON::NONE){
        ceblog(LOG_LV_NECESSARY, CYN, "button key \t[" << enumToString(buttonKey) << "]");
        convertButtonKey(buttonKey);
        return;
    }

    if ( actionKey != E_ACTION_KEY::NONE){
        ceblog(LOG_LV_NECESSARY, CYN, "aws action key \t[" << enumToString(actionKey) << "]");
        convertAwsActionKey(actionKey);
        return;
    }

    if(rosKey != E_ROS_KEY::NONE){
        ceblog(LOG_LV_NECESSARY, CYN, "ros key \t[" << enumToString(rosKey) << "]");
        convertRosKey(rosKey);
        return;
    }

    setKeyValue(E_KEY_TYPE::VOID);
}

void CKeyState::initKeyState()
{
    button.initButton();
    rosCmd.initRosKey();
    appOption.bSoundVolume = false;
    appOption.bCountry = false;
    appOption.bLanguage = false;
    appOption.bCleanMode = false;
    appOption.bWaterLevel = false;
    appOption.bAreaInfo = false;
    appOption.bOperationAll = false;
    appOption.bOperationRoom = false;
    appOption.bOperationSpot = false;
    appOption.bOperationCustom = false;
    appOption.bDryMop= false;
    appOption.bOta = false;
    appOption.bScheduleClean = false;
    appOption.bForbiddenLine = false;
    appOption.bForbiddenRect = false;
    appOption.bDistruptMode = false;
}

void CKeyState::setKeyValue(E_KEY_TYPE newKey)
{
    key = newKey;
    if(key != E_KEY_TYPE::VOID){
        ceblog(LOG_LV_NECESSARY, CYN, " E_KEY_TYPE : "  << enumToString(key));
    }
}
E_KEY_TYPE CKeyState::getKeyValue(void)
{
    return key;
}

void CKeyState::convertButtonKey(E_FUNCTION_BUTTON button)
{
    E_KEY_TYPE keyVel = E_KEY_TYPE::VOID;

    switch (button)
    {
    case E_FUNCTION_BUTTON::START_LONG :
        keyVel = E_KEY_TYPE::POWER_OFF;
        break;
    case E_FUNCTION_BUTTON::START_SHORT :
        keyVel = E_KEY_TYPE::CLEAN;
        break;
    case E_FUNCTION_BUTTON::HOME_LONG :
        keyVel = E_KEY_TYPE::DRY_MOP;
        break;
    case E_FUNCTION_BUTTON::HOME_SHORT :
        keyVel = E_KEY_TYPE::HOME;
        break;
    case E_FUNCTION_BUTTON::FUNC_LONG :
        keyVel = E_KEY_TYPE::DRAIN_WATER;
        break;
    case E_FUNCTION_BUTTON::FUNC_SHORT :
        keyVel = E_KEY_TYPE::WATER_OPTION;
        break;
    case E_FUNCTION_BUTTON::HOME_FUNC_LONG :
        keyVel = E_KEY_TYPE::MCU_RESET;
        break;
    case E_FUNCTION_BUTTON::START_FUNC_LONG :
        keyVel = E_KEY_TYPE::APP_CONNECT;
        break;
    case E_FUNCTION_BUTTON::START_HOME_LONG :
        keyVel = E_KEY_TYPE::INIT_USER_OPTION;
        break;                                  
    default:
        ceblog(LOG_LV_NECESSARY, CYN, " UNKWON BUTTON KEY : "  << enumToString(button));
        break;
    }

    setKeyValue(keyVel);
}

void CKeyState::convertAwsActionKey(E_ACTION_KEY action)
{
    E_KEY_TYPE keyVel = E_KEY_TYPE::VOID;

    switch (action)
    {
    case E_ACTION_KEY::CLEAN :
        keyVel = E_KEY_TYPE::CLEAN;
        break;
    case E_ACTION_KEY::HOMING :
        keyVel = E_KEY_TYPE::HOME;
        break;
    case E_ACTION_KEY::EXPLORER :
        keyVel = E_KEY_TYPE::EXPLORER;
        break;
    case E_ACTION_KEY::STOP :
        keyVel = E_KEY_TYPE::STOP;
        break;
    case E_ACTION_KEY::START_STOP_DRY_MOP :
        keyVel = E_KEY_TYPE::DRY_MOP;
        break;
    case E_ACTION_KEY::START_STOP_DRAIN_WATER :
        keyVel = E_KEY_TYPE::DRAIN_WATER;
        break;
    case E_ACTION_KEY::START_INIT_USERSET :
        keyVel = E_KEY_TYPE::INIT_USER_OPTION;
        break;
    case E_ACTION_KEY::START_FW_UPDATE :
        keyVel = E_KEY_TYPE::FW_UPDATE;
        break;
    case E_ACTION_KEY::START_FW_RECOVERY :
        keyVel = E_KEY_TYPE::FW_RECOVERY;
        break;
    case E_ACTION_KEY::START_FACTORY_RESET :
        keyVel = E_KEY_TYPE::FACTORY_RESET;
        break;
    case E_ACTION_KEY::DELETE_MAP :
        keyVel = E_KEY_TYPE::DELETE_MAP;
        break;                                                  
    default:
        ceblog(LOG_LV_NECESSARY, CYN, " UNKWON ACTION KEY : "  << enumToString(action));
        break;
    }

    setKeyValue(keyVel);
}

void CKeyState::setAppOption(CExternData* pExternData)
{
    if(pExternData->appData.sound.isUpdate())                   appOption.bSoundVolume = true;
    if(pExternData->appData.country.isUpdate())                 appOption.bCountry = true;
    if(pExternData->appData.language.isUpdate())                appOption.bLanguage = true;
    if(pExternData->appData.cleanMode.isUpdate())               appOption.bCleanMode = true;
    if(pExternData->appData.waterLv.isUpdate())                 appOption.bWaterLevel = true;
    
    //if(pExternData->appData.areaInfo.isUpdate())                appOption.bAreaInfo = true;
    if(pExternData->appData.divideAreaInfo.isUpdate())          appOption.bDivideArea = true;
    if(pExternData->appData.combineAreaInfo.isUpdate())         appOption.bCombineArea = true;

    if(pExternData->appData.operationAreaAll.isUpdate())        appOption.bOperationAll = true;
    if(pExternData->appData.operationAreaRoom.isUpdate())       appOption.bOperationRoom = true;
    if(pExternData->appData.operationAreaSpot.isUpdate())       appOption.bOperationSpot = true;
    if(pExternData->appData.operationAreaCustom.isUpdate())     appOption.bOperationCustom = true;

    if(pExternData->appData.dryEnabled.isUpdate() ||
        pExternData->appData.dryHours.isUpdate() ||
        pExternData->appData.dryPower.isUpdate())               appOption.bDryMop= true;

    
    if(pExternData->appData.otaForce.isUpdate() || 
        pExternData->appData.otaName.isUpdate() ||
        pExternData->appData.otaVersion.isUpdate() ||
        pExternData->appData.otaScheduled.isUpdate() ||
        pExternData->appData.otaScheduleTime.isUpdate())        appOption.bOta = true;

    
    if(pExternData->appData.cleanSchedule.isUpdate())           appOption.bScheduleClean = true;
    
    if(pExternData->appData.forbiddenLine.isUpdate())           appOption.bForbiddenLine = true;
    if(pExternData->appData.forbiddenRect.isUpdate())           appOption.bForbiddenRect = true;
    
    if(pExternData->appData.dontDisturbStatus.isUpdate() ||
        pExternData->appData.dontDisturbStartTime.isUpdate() ||
        pExternData->appData.dontDisturbEndTime.isUpdate())     appOption.bDistruptMode = true;

}

void CKeyState::convertRosKey(E_ROS_KEY rosKey)
{
    E_KEY_TYPE keyVel = E_KEY_TYPE::VOID;

    switch (rosKey)
    {
     case E_ROS_KEY::POWER_OFF :
        keyVel = E_KEY_TYPE::POWER_OFF;
        break;
    case E_ROS_KEY::CLEAN :
        keyVel = E_KEY_TYPE::CLEAN;
        break;
    case E_ROS_KEY::EXPLORER :
        keyVel = E_KEY_TYPE::EXPLORER;
        break;    
    case E_ROS_KEY::HOME :
        keyVel = E_KEY_TYPE::HOME;
        break;
    case E_ROS_KEY::APP_CONNECT :
        keyVel = E_KEY_TYPE::APP_CONNECT;
        break;    
    case E_ROS_KEY::STOP :
        keyVel = E_KEY_TYPE::STOP;
        break;
    case E_ROS_KEY::DRY_MOP :
        keyVel = E_KEY_TYPE::DRY_MOP;
        break;
    case E_ROS_KEY::DRAIN_WATER :
        keyVel = E_KEY_TYPE::DRAIN_WATER;
        break; 
    case E_ROS_KEY::INIT_USER_OPTION :
        keyVel = E_KEY_TYPE::INIT_USER_OPTION;
        break;
    case E_ROS_KEY::FW_UPDATE :
        keyVel = E_KEY_TYPE::FW_UPDATE;
        break;
    case E_ROS_KEY::FW_RECOVERY :
        keyVel = E_KEY_TYPE::FW_RECOVERY;
        break;
    case E_ROS_KEY::WATER_OPTION :
        keyVel = E_KEY_TYPE::WATER_OPTION;
        break;     

    case E_ROS_KEY::MOVING_UP :
        keyVel = E_KEY_TYPE::MOVING_UP;
        break;
    case E_ROS_KEY::MOVING_DOWN :
        keyVel = E_KEY_TYPE::MOVING_DOWN;
        break;
    case E_ROS_KEY::MOVING_LEFT :
        keyVel = E_KEY_TYPE::MOVING_LEFT;
        break;
    case E_ROS_KEY::MOVING_RIGHT :
        keyVel = E_KEY_TYPE::MOVING_RIGHT;
        break;
    case E_ROS_KEY::MOVING_STOP :
        keyVel = E_KEY_TYPE::MOVING_STOP;
        break;
    case E_ROS_KEY::TILTING_UP :
        keyVel = E_KEY_TYPE::TILTING_UP;
        break;
    case E_ROS_KEY::TILTING_DONW :
        keyVel = E_KEY_TYPE::TILTING_DONW;
        break;
    case E_ROS_KEY::SLAM_ON :
        keyVel = E_KEY_TYPE::SLAM_ON;
        break;
    case E_ROS_KEY::SLAM_OFF :
        keyVel = E_KEY_TYPE::SLAM_OFF;
        break;
    case E_ROS_KEY::IMU_INIT :
        keyVel = E_KEY_TYPE::IMU_INIT;
        break;
    case E_ROS_KEY::TOF_INIT :
        keyVel = E_KEY_TYPE::TOF_INIT;
        break;    
    case E_ROS_KEY::SAVE_MAP :
        keyVel = E_KEY_TYPE::SAVE_MAP;
        break;
    case E_ROS_KEY::DELETE_MAP :
        keyVel = E_KEY_TYPE::DELETE_MAP;
        break;
    case E_ROS_KEY::RESERVAITON_CLEAN :
        keyVel = E_KEY_TYPE::RESERVAITON_CLEAN;
        break;
    case E_ROS_KEY::RESERVATION_STOP :
        keyVel = E_KEY_TYPE::RESERVATION_STOP;
        break;
    case E_ROS_KEY::FACTORY_RESET :
        keyVel = E_KEY_TYPE::FACTORY_RESET;
        break;
    case E_ROS_KEY::AP_RESET :
        keyVel = E_KEY_TYPE::AP_RESET;
        break;
    case E_ROS_KEY::MCU_RESET :
        keyVel = E_KEY_TYPE::MCU_RESET;
        break;
    default:
        ceblog(LOG_LV_NECESSARY, CYN, " UNKWON OPTION KEY : "  << enumToString(rosKey));
        break;
    }

    setKeyValue(keyVel);
}

bool CKeyState::isUpdatForbiddenLine(){
    bool ret = false;
    if(appOption.bForbiddenLine){
        ret = true;
        appOption.bForbiddenLine = false;
    }
    return ret;
}
bool CKeyState::isUpdateForbiddenRect(){
    bool ret = false;
    if(appOption.bForbiddenRect){
        ret = true;
        appOption.bForbiddenRect = false;
    }
    return ret;
}
bool CKeyState::isUpdatOperationAll(){
    bool ret = false;
    if(appOption.bOperationAll){
        ret = true;
        appOption.bOperationAll = false;
    }
    return ret;
}
bool CKeyState::isUpdatOperationSpot(){
    bool ret = false;
    if(appOption.bOperationSpot){
        ret = true;
        appOption.bOperationSpot = false;
    }
    return ret;
}
bool CKeyState::isUpdatOperationRoom(){
    bool ret = false;
    if(appOption.bOperationRoom){
        ret = true;
        appOption.bOperationRoom = false;
    }
    return ret;
}
bool CKeyState::isUpdatOperationCustom(){
    bool ret = false;
    if(appOption.bOperationCustom){
        ret = true;
        appOption.bOperationCustom = false;
    }
    return ret;
}
bool CKeyState::isUpdatAreaInfo(){
    bool ret = false;
    if(appOption.bAreaInfo){
        ret = true;
        appOption.bAreaInfo = false;
    }
    return ret;
}
bool CKeyState::isUpdatDivideArea()
{
    bool ret = false;
    if(appOption.bDivideArea){
        ret = true;
        appOption.bDivideArea = false;
    }
    return ret;
}
bool CKeyState::isUpdatCombineArea()
{
    bool ret = false;
    if(appOption.bCombineArea){
        ret = true;
        appOption.bCombineArea = false;
    }
    return ret;
}
bool CKeyState::isUpdatWaterLevel(){
    bool ret = false;
    if(appOption.bWaterLevel){
        ret = true;
        appOption.bWaterLevel = false;
    }
    return ret;
}
bool CKeyState::isUpdatSoundVolume(){
    bool ret = false;
    if(appOption.bSoundVolume){
        ret = true;
        appOption.bSoundVolume = false;
    }
    return ret;
}
bool CKeyState::isUpdatDryMop(){
    bool ret = false;
    if(appOption.bDryMop){
        ret = true;
        appOption.bDryMop = false;
    }
    return ret;
}

bool CKeyState::isUpdatScheduleClean(){
    bool ret = false;
    if(appOption.bScheduleClean){
        ret = true;
        appOption.bScheduleClean = false;
    }
    return ret;
}
bool CKeyState::isUpdatDistruptMode(){
    bool ret = false;
    if(appOption.bDistruptMode){
        ret = true;
        appOption.bDistruptMode = false;
    }
    return ret;
}

bool CKeyState::isUpdatCleanMode(){
    bool ret = false;
    if(appOption.bCleanMode){
        ret = true;
        appOption.bCleanMode = false;
    }
    return ret;
}
bool CKeyState::isUpdatLanguage(){
    bool ret = false;
    if(appOption.bLanguage){
        ret = true;
        appOption.bLanguage = false;
    }
    return ret;
}
bool CKeyState::isUpdatCountry(){
    bool ret = false;
    if(appOption.bCountry){
        ret = true;
        appOption.bCountry = false;
    }
    return ret;
}
bool CKeyState::isUpdatOta(){
    bool ret = false;
    if(appOption.bOta){
        ret = true;
        appOption.bOta = false;
    }
    return ret;
}