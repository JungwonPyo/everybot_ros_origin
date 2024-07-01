#include "keyHandler.h"
#include "userInterface.h"
#include "control/control.h"
#include "eblog.h"
#include "utils.h"

CKeyHanler::CKeyHanler(CServiceMacro* _pServiceMacro, CSupplyWater* _pSupplyWater, CTaskCharging* _pTaskCharging) : 
pServiceMacro(_pServiceMacro),pSupplyWater(_pSupplyWater),pTaskCharging(_pTaskCharging)
{
    bServiceUdate = false;
}

CKeyHanler::~CKeyHanler()
{

}

bool CKeyHanler::isUpdateKeyService()
{
    bool ret = bServiceUdate;
    bServiceUdate = false;
    return ret;
}

E_KEY_VALUE CKeyHanler::convertKeyValue(E_KEY_TYPE type, E_SERVICE_ID curId, E_SERVICE_STATUS curState)
{
    E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    E_SERVICE_ID doId;

    switch (type)
    {
    case E_KEY_TYPE::VOID:
        break;    
    case E_KEY_TYPE::POWER_OFF :
        ret = E_KEY_VALUE::POWER_OFF;
        break;
    case E_KEY_TYPE::EXPLORER :
    case E_KEY_TYPE::CLEAN :
    case E_KEY_TYPE::HOME :
    case E_KEY_TYPE::APP_CONNECT :
    case E_KEY_TYPE::STOP :
        if(pSupplyWater->isActiveWaterDrain())      ret = E_KEY_VALUE::START_STOP_DRAIN_WATER;
        else if(pTaskCharging->isDryMopActive())    ret = E_KEY_VALUE::START_STOP_DRY_MOP;
        else{
            doId = getDoingServiceId(type,curId);
            ret = getServiceKeyValue(doId,getDoingServiceState(doId,curId,curState));
        }
        break;
    case E_KEY_TYPE::DRY_MOP :
        ret = E_KEY_VALUE::START_STOP_DRY_MOP;
        break;
    case E_KEY_TYPE::DRAIN_WATER :
        ret = E_KEY_VALUE::START_STOP_DRAIN_WATER;
        break;
    case E_KEY_TYPE::DELETE_MAP :
        ret = E_KEY_VALUE::DELETE_MAP;
        break;
    case E_KEY_TYPE::INIT_USER_OPTION :
        if(pSupplyWater->isActiveWaterDrain())      ret = E_KEY_VALUE::START_STOP_DRAIN_WATER;
        else if(pTaskCharging->isDryMopActive())    ret = E_KEY_VALUE::START_STOP_DRY_MOP;
        else                                        ret = E_KEY_VALUE::START_INIT_USERSET;
        break;
    case E_KEY_TYPE::RESERVAITON_CLEAN :
        ret = E_KEY_VALUE::START_RESERVAITON_CLEAN;
        break;
    case E_KEY_TYPE::RESERVATION_STOP :
        ret = E_KEY_VALUE::START_RESERVATION_STOP;
        break;
    case E_KEY_TYPE::FW_UPDATE :
       ret = E_KEY_VALUE::START_FW_UPDATE;
        break;
    case E_KEY_TYPE::FW_RECOVERY :
        ret = E_KEY_VALUE::START_FW_RECOVERY;
        break;
    case E_KEY_TYPE::FACTORY_RESET :
        ret = E_KEY_VALUE::START_FACTORY_RESET;
        break;
    case E_KEY_TYPE::WATER_OPTION :
        ret = E_KEY_VALUE::WATER_OPTION;
        break;
    //FOR TEST       
    case E_KEY_TYPE::MOVING_UP:
        ret = E_KEY_VALUE::MOVING_UP;
        break;
    case E_KEY_TYPE::MOVING_DOWN:
        ret = E_KEY_VALUE::MOVING_DOWN;
        break;
    case E_KEY_TYPE::MOVING_LEFT:
        ret = E_KEY_VALUE::MOVING_LEFT;
        break;
    case E_KEY_TYPE::MOVING_RIGHT:
        ret = E_KEY_VALUE::MOVING_RIGHT;
        break;
    case E_KEY_TYPE::MOVING_STOP:
        ret = E_KEY_VALUE::MOVING_STOP;
        break;
    case E_KEY_TYPE::TILTING_UP:
        ret = E_KEY_VALUE::TILTING_UP;
        break;
    case E_KEY_TYPE::TILTING_DONW:
        ret = E_KEY_VALUE::TILTING_DONW;
        break;
    case E_KEY_TYPE::SLAM_ON:
        ret = E_KEY_VALUE::SLAM_ON;
        break;
    case E_KEY_TYPE::SAVE_MAP:
        ret = E_KEY_VALUE::SAVE_MAP;
        break;
    case E_KEY_TYPE::SLAM_OFF:
        ret = E_KEY_VALUE::SLAM_OFF;
        break;
    case E_KEY_TYPE::IMU_INIT:
        ret = E_KEY_VALUE::IMU_INIT;
        break;
    case E_KEY_TYPE::TOF_INIT:
        ret = E_KEY_VALUE::TOF_INIT;
        break;
    case E_KEY_TYPE::AP_RESET:
        ret = E_KEY_VALUE::AP_RESET;
        break;
    case E_KEY_TYPE::MCU_RESET:
        ret = E_KEY_VALUE::MCU_RESET;
        break;                                                                                            
    default:
        eblog(LOG_LV_NECESSARY, "UNKOWN KEY-TYPE : " << enumToString(type));
        break;
    }

    if(ret != E_KEY_VALUE::NONE)
    {
        eblog(LOG_LV_NECESSARY, "KEY INPUT KEY-VALUE : " << enumToString(ret));
    }

    return ret;
}

E_SERVICE_ID CKeyHanler::getDoingServiceId(E_KEY_TYPE type,E_SERVICE_ID curId)
{
    E_SERVICE_ID ret;

    if(curId == E_SERVICE_ID::WIFI || type == E_KEY_TYPE::STOP){
        ret = E_SERVICE_ID::IDLE;
    }else if(type == E_KEY_TYPE::CLEAN){
        if(curId == E_SERVICE_ID::UNDOCKING)        ret = E_SERVICE_ID::IDLE;
        else if(curId == E_SERVICE_ID::EXPLORER)    ret = E_SERVICE_ID::EXPLORER;
        else                                        ret = E_SERVICE_ID::CLEAN;
    }
    else if(type == E_KEY_TYPE::EXPLORER)           ret = E_SERVICE_ID::EXPLORER;    
    else if(type == E_KEY_TYPE::HOME)               ret = E_SERVICE_ID::DOCKING;
    else if(type == E_KEY_TYPE::APP_CONNECT)        ret = E_SERVICE_ID::WIFI;
    else{
        ceblog(LOG_LV_NECESSARY, RED, "convert error!! Key : " << enumToString(type));
    }
    
    ceblog(LOG_LV_NECESSARY, BLUE, " Key : " << enumToString(type) << " do ID : " << enumToString(ret));
    return ret;
}

E_SERVICE_STATUS CKeyHanler::getDoingServiceState(E_SERVICE_ID doId, E_SERVICE_ID curId, E_SERVICE_STATUS curState)
{
    E_SERVICE_STATUS ret;
    if(doId == E_SERVICE_ID::IDLE || doId == E_SERVICE_ID::WIFI || doId != curId)   ret = E_SERVICE_STATUS::startup;
    else if(curState == E_SERVICE_STATUS::paused)                                   ret = E_SERVICE_STATUS::running;
    else                                                                            ret = E_SERVICE_STATUS::paused;

    ceblog(LOG_LV_NECESSARY, BLUE, " do ID : " << enumToString(doId) << " cur ID : " << enumToString(curId) << " cur State : " << enumToString(curState) << " do State : " << enumToString(ret));
    
    return ret;
}

E_KEY_VALUE CKeyHanler::getServiceKeyValue(E_SERVICE_ID doId,E_SERVICE_STATUS doState)
{
    E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    
    switch (doState)
    {
    case E_SERVICE_STATUS::status_void :
    case E_SERVICE_STATUS::initializing :
    case E_SERVICE_STATUS::startup :
        ret = getServiceStartKey(doId);
        break;
     case E_SERVICE_STATUS::running :
        ret =getServiceResumeKey(doId);
        break;
     case E_SERVICE_STATUS::paused :
        ret = getServicePauseKey(doId);
        break;
    case E_SERVICE_STATUS::completed :
        ret = E_KEY_VALUE::STOP;
        break;        
    default:
        ceblog(LOG_LV_NECESSARY, RED, "state Error!! ServiceStatus : " << enumToString(doState));
        break;
    }

    ceblog(LOG_LV_NECESSARY, BLUE, " do ID : " << enumToString(doId) <<  " do State : " << enumToString(doState) << "Key Val : " << enumToString(ret));

    return ret;
}

E_KEY_VALUE CKeyHanler::getServicePauseKey(E_SERVICE_ID doId)
{
    E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    if(doId == E_SERVICE_ID::EXPLORER || doId == E_SERVICE_ID::CLEAN || doId == E_SERVICE_ID::DOCKING ){
        ceblog(LOG_LV_NECESSARY, RED, "일시정지 & 재시작 오류로 해결시 까지 대기상태로 보냄!! Id : " << enumToString(doId));
        ret = E_KEY_VALUE::STOP;
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "PAUSE SERVICE Error!! Id : " << enumToString(doId));
    }
    #if 0
    if(doId == E_SERVICE_ID::EXPLORER){
        ret = E_KEY_VALUE::EXPLORER_PAUSE;
    }
    else if(doId == E_SERVICE_ID::CLEAN){
        ret = E_KEY_VALUE::CLEAN_PAUSE;
    }
    else if(doId == E_SERVICE_ID::DOCKING){
        ret = E_KEY_VALUE::HOME_PAUSE;
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "PAUSE SERVICE Error!! Id : " << enumToString(doId));
    }
    #endif
    return ret;
}

E_KEY_VALUE CKeyHanler::getServiceResumeKey(E_SERVICE_ID doId)
{
    E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    if(doId == E_SERVICE_ID::EXPLORER){
        ret = E_KEY_VALUE::EXPLORER_RESUME;
    }
    else if(doId == E_SERVICE_ID::CLEAN){
        ret = E_KEY_VALUE::CLEAN_RESUME;
    }
    else if(doId == E_SERVICE_ID::DOCKING){
        ret = E_KEY_VALUE::HOME_RESUME;
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "RESUME SERVICE Error!! Id : " << enumToString(doId));
    }
    return ret;
}

E_KEY_VALUE CKeyHanler::getServiceStartKey(E_SERVICE_ID doId)
{
     E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    if(doId == E_SERVICE_ID::EXPLORER){
        ret = E_KEY_VALUE::EXPLORER_START;
    }
    else if(doId == E_SERVICE_ID::CLEAN){
        ret = E_KEY_VALUE::CLEAN_START;
    }
    else if(doId == E_SERVICE_ID::DOCKING){
        ret = E_KEY_VALUE::HOME_START;
    }
    else if(doId == E_SERVICE_ID::IDLE){
        ret = E_KEY_VALUE::STOP;
    }
    else if(doId == E_SERVICE_ID::WIFI){
        ret = E_KEY_VALUE::START_APP_CONNECT;
    }
    else{
        ceblog(LOG_LV_NECESSARY, RED, "START SERVICE ERROR!! Id : " << enumToString(doId));
    }
    return ret;
}


void CKeyHanler::keyChecker(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus, bool bNeedCharge , bool *powerOff, bool *FirmwareUpdate)
{
   monitorAppOption();
   E_KEY_VALUE trigger = checkTriggerOption(curId,curStatus);
   E_KEY_VALUE keyVel = convertKeyValue(ServiceData.key.getKeyValue(),curId,curStatus);
   
   if(keyVel != E_KEY_VALUE::NONE)
   {
        eblog(LOG_LV_NECESSARY, "Key : " << enumToString(keyVel) << "입력!! 실행중인 Service ID , State : " << enumToString(curId) << enumToString(curStatus) << " 배터리 부족 : " << (int)bNeedCharge );
        if(isKeyValidate(keyVel,curId,curStatus,bNeedCharge)){
            keyProc(keyVel,curId,powerOff,FirmwareUpdate);
        }
        else{
            eblog(LOG_LV_NECESSARY, "Key : " << enumToString(keyVel) << "는 실행할 수 없습니다. 실행중인 Service ID , State : " << enumToString(curId) << enumToString(curStatus) << " 배터리 부족 : " << (int)bNeedCharge );
        }
   }

   if(trigger != E_KEY_VALUE::NONE)
   {
        eblog(LOG_LV_NECESSARY, "Trigger : " << enumToString(trigger) << "입력!! 실행중인 Service ID , State : " << enumToString(curId) << enumToString(curStatus) << " 배터리 부족 : " << (int)bNeedCharge );
        if(isKeyValidate(trigger,curId,curStatus,bNeedCharge)){
            keyProc(trigger,curId,powerOff,FirmwareUpdate);
        }
        else{
            eblog(LOG_LV_NECESSARY, "Trigger : " << enumToString(trigger) << "는 실행할 수 없습니다. 실행중인 Service ID , State : " << enumToString(curId) << enumToString(curStatus) << " 배터리 부족 : " << (int)bNeedCharge );
        }
   }

   return;
}

void CKeyHanler::keyProc(E_KEY_VALUE keyVel, E_SERVICE_ID curId, bool *powerOff,bool *FirmwareUpdate)
{
    switch (keyVel)
    {
    case E_KEY_VALUE::POWER_OFF :
        *powerOff = true;
        break;     
    case E_KEY_VALUE::STOP :
        bServiceUdate = true;
        DISPLAY_CTR.startAutoDisplay(true,DISPLAY_CTR.getBlinkEyeImage());
        pServiceMacro->macroServiceStop(curId);
        break;
    case E_KEY_VALUE::EXPLORER_START :
        bServiceUdate = true;
        pServiceMacro->macroProcess(curId,E_SERVICE_ID::EXPLORER);
        break;    
    case E_KEY_VALUE::EXPLORER_PAUSE :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
        pServiceMacro->macroServicePause(E_SERVICE_ID::EXPLORER);
        break;
    case E_KEY_VALUE::EXPLORER_RESUME :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_NAVIGATION_RESUME);
        pServiceMacro->macroServiceResume(E_SERVICE_ID::EXPLORER);
        break;
    case E_KEY_VALUE::CLEAN_START :
        bServiceUdate = true;
        pServiceMacro->macroProcess(curId,E_SERVICE_ID::CLEAN);
        break;        
    case E_KEY_VALUE::CLEAN_PAUSE :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
        pServiceMacro->macroServicePause(E_SERVICE_ID::CLEAN);
        break;
    case E_KEY_VALUE::CLEAN_RESUME :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CLEAN_RESUME);
        pServiceMacro->macroServiceResume(E_SERVICE_ID::CLEAN);
        break;
    case E_KEY_VALUE::HOME_START :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_MOVE_TO_CHARGER);
        // DISPLAY_CTR.startDisplay(E_DisplayImageClass::GO_TO_CHARGER);
        pServiceMacro->macroProcess(curId,E_SERVICE_ID::DOCKING);
        break;            
    case E_KEY_VALUE::HOME_PAUSE :
        bServiceUdate = true;
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_PAUSED);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::STOP);
        pServiceMacro->macroServicePause(E_SERVICE_ID::DOCKING);
        break;
    case E_KEY_VALUE::HOME_RESUME :
        bServiceUdate = true;
        pServiceMacro->macroServiceResume(E_SERVICE_ID::DOCKING);
        break;
    case E_KEY_VALUE::START_APP_CONNECT :
        bServiceUdate = true;
        pServiceMacro->macroProcess(curId,E_SERVICE_ID::WIFI);
        break;
    case E_KEY_VALUE::START_STOP_DRAIN_WATER :
        pSupplyWater->waterDrainONOFF();
        break;
    case E_KEY_VALUE::START_STOP_DRY_MOP :
        pTaskCharging->dryMopStartStop();
        break;
    case E_KEY_VALUE::START_INIT_USERSET :
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_INIT_SETTING_INFO);
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::INITIALIZATION);
        ROBOT_CONTROL.slam.removeSlamMapFile(false);
        break;
    case E_KEY_VALUE::WATER_OPTION :
        pSupplyWater->ChangeWaterLevel();
        break;
    case E_KEY_VALUE::DELETE_MAP :
        ROBOT_CONTROL.slam.removeSlamMapFile(false);
        break;
    case E_KEY_VALUE::TILTING_UP:
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
        break;
    case E_KEY_VALUE::TILTING_DONW:
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
        break;
    case E_KEY_VALUE::SLAM_ON:
        ROBOT_CONTROL.slam.runSlam();
        break;
    case E_KEY_VALUE::SAVE_MAP:
        if(ROBOT_CONTROL.slam.isExistedSlamMap()){
            ServiceData.mapStorage.resetChargerPose();
            //removeSlamMapFile-> arg is true : temp map remove & false : saved map remove
            bool nResult = ROBOT_CONTROL.slam.removeSlamMapFile(false);
            if (!nResult){
                eblog(LOG_LV_NECESSARY, " 지도 삭제 실패");
            }else{
                eblog(LOG_LV_NECESSARY, " 지도 삭제 성공");
            }
        }else{
            eblog(LOG_LV_NECESSARY, " 지도가 존재하지 않습니다");
        }
        break;
    case E_KEY_VALUE::SLAM_OFF:
        ROBOT_CONTROL.slam.exitSlam();
        break;
    case E_KEY_VALUE::IMU_INIT:
        ROBOT_CONTROL.clearSystemLocalization();
        ROBOT_CONTROL.system.initImuSensor();
        break;
    case E_KEY_VALUE::TOF_INIT:
        ROBOT_CONTROL.system.initTofSensor();
        break;
    case E_KEY_VALUE::AP_RESET:
        system("sudo reboot");
        break;
    case E_KEY_VALUE::MCU_RESET:
        ROBOT_CONTROL.robotSystemReset();
        break;
    case E_KEY_VALUE::START_FACTORY_RESET:
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        ROBOT_CONTROL.reportFactoryReset();
        break;
    case E_KEY_VALUE::START_FW_UPDATE:
        *FirmwareUpdate = true;
        break;
    case E_KEY_VALUE::START_RESERVAITON_CLEAN :
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_RESERVATION_CLEAN_START);
        break;
    case E_KEY_VALUE::START_RESERVATION_STOP :
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DISTURB_TIME_MOVE_TO_CHARGER);
        break;    
    case E_KEY_VALUE::MOVING_UP :
    case E_KEY_VALUE::MOVING_DOWN:
    case E_KEY_VALUE::MOVING_LEFT:
    case E_KEY_VALUE::MOVING_RIGHT:
    case E_KEY_VALUE::MOVING_STOP:  
    case E_KEY_VALUE::START_FW_RECOVERY :
        eblog(LOG_LV_NECESSARY, " 기능 없음!! 만들어주세요 " << enumToString(keyVel));
        break;
    
    default:
        eblog(LOG_LV_NECESSARY, "UNKOWN KEY : " << enumToString(keyVel));
        break;
    }

    return;        
}

bool CKeyHanler::isKeyValidate(E_KEY_VALUE keyVel, E_SERVICE_ID svcId, E_SERVICE_STATUS svcStatus, bool battLow)
{
    bool ret = false;

    if(pSupplyWater->isActiveWaterDrain())
    {
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DRAIN_WATER_WAIT); 
        return ret;
    }

    switch (keyVel)
    {
    case E_KEY_VALUE::POWER_OFF :
        if(ServiceData.power.getExtPower()){
            SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_CANT_POWEROFF_CHARGING); 
        }
        else{
            ret = true;
        }                                
        break; 
    case E_KEY_VALUE::STOP :
    case E_KEY_VALUE::AP_RESET :
    case E_KEY_VALUE::MCU_RESET:
    case E_KEY_VALUE::START_FACTORY_RESET:
        ret = true;
        break;
    case E_KEY_VALUE::EXPLORER_PAUSE :
        if(svcId == E_SERVICE_ID::EXPLORER && svcStatus != E_SERVICE_STATUS::paused) ret = true;
        break;
    case E_KEY_VALUE::CLEAN_PAUSE :
        if(svcId == E_SERVICE_ID::CLEAN && svcStatus != E_SERVICE_STATUS::paused) ret = true;
        break;    
    case E_KEY_VALUE::HOME_PAUSE :
        if(svcId == E_SERVICE_ID::DOCKING && svcStatus != E_SERVICE_STATUS::paused) ret = true;
        break;
    case E_KEY_VALUE::EXPLORER_RESUME :
        if(svcId == E_SERVICE_ID::EXPLORER && svcStatus == E_SERVICE_STATUS::paused) ret = true;
        break;
    case E_KEY_VALUE::CLEAN_RESUME :
        if(svcId == E_SERVICE_ID::CLEAN && svcStatus == E_SERVICE_STATUS::paused) ret = true;
        break;    
    case E_KEY_VALUE::HOME_RESUME :
        if(svcId == E_SERVICE_ID::DOCKING && svcStatus == E_SERVICE_STATUS::paused) ret = true;
        break;
    case E_KEY_VALUE::EXPLORER_START :
    case E_KEY_VALUE::CLEAN_START :
    case E_KEY_VALUE::START_RESERVAITON_CLEAN :
        if(!battLow) ret = true;
        else{SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_LOW_BATT_CANT_EXECUTE);}
        break;    
    case E_KEY_VALUE::HOME_START :
    case E_KEY_VALUE::START_RESERVATION_STOP :
        if(svcId != E_SERVICE_ID::CHARGING) ret = true;
        break;
    case E_KEY_VALUE::START_STOP_DRAIN_WATER :
        if(svcId == E_SERVICE_ID::IDLE || ((svcId == E_SERVICE_ID::CLEAN || svcId == E_SERVICE_ID::EXPLORER || svcId == E_SERVICE_ID::DOCKING ) && svcStatus == E_SERVICE_STATUS::paused)) ret = true;
        break;
    case E_KEY_VALUE::START_FW_UPDATE :
    case E_KEY_VALUE::START_FW_RECOVERY :    
    case E_KEY_VALUE::START_STOP_DRY_MOP :
        if(svcId == E_SERVICE_ID::CHARGING) ret = true;
        break;
    case E_KEY_VALUE::START_INIT_USERSET :
    case E_KEY_VALUE::START_APP_CONNECT :
        if(svcId == E_SERVICE_ID::IDLE || svcId == E_SERVICE_ID::CHARGING || svcStatus == E_SERVICE_STATUS::paused) ret = true;
        break;
    case E_KEY_VALUE::WATER_OPTION :
    case E_KEY_VALUE::DELETE_MAP:
        if(svcId != E_SERVICE_ID::WIFI) ret = true;
        break;
    case E_KEY_VALUE::MOVING_UP :
    case E_KEY_VALUE::MOVING_DOWN:
    case E_KEY_VALUE::MOVING_LEFT:
    case E_KEY_VALUE::MOVING_RIGHT:
    case E_KEY_VALUE::MOVING_STOP:          
    case E_KEY_VALUE::TILTING_UP:
    case E_KEY_VALUE::TILTING_DONW:
    case E_KEY_VALUE::SLAM_ON:
    case E_KEY_VALUE::SAVE_MAP:
    case E_KEY_VALUE::SLAM_OFF:
    case E_KEY_VALUE::IMU_INIT:
    case E_KEY_VALUE::TOF_INIT:
        if(svcId == E_SERVICE_ID::IDLE || svcStatus == E_SERVICE_STATUS::paused) ret = true;
        break;
    default:
        eblog(LOG_LV_NECESSARY, "UNKOWN KEY : " << enumToString(keyVel));
        break;
    }

    return ret;
}

void CKeyHanler::monitorAppOption()
{
    if(ServiceData.key.isUpdatForbiddenLine()) ServiceData.forbiddenArea.setForbiddenLine();
    if(ServiceData.key.isUpdateForbiddenRect()) ServiceData.forbiddenArea.setForbiddenRect();
    if(ServiceData.key.isUpdatOperationAll()) ServiceData.operationArea.setAreaAll();
    if(ServiceData.key.isUpdatOperationSpot()) ServiceData.operationArea.setAreaSpot();
    if(ServiceData.key.isUpdatOperationRoom()) ServiceData.operationArea.setAreaRoom();
    if(ServiceData.key.isUpdatOperationCustom()) ServiceData.operationArea.setAreaCustom();

    //if(ServiceData.key.isUpdatAreaInfo())       ServiceData.areaInfo.setAreaInfo();
    if(ServiceData.key.isUpdatDivideArea())     ServiceData.areaInfo.setEditAreaData();
    if(ServiceData.key.isUpdatCombineArea())    ServiceData.areaInfo.setEditAreaData();

    if(ServiceData.key.isUpdatWaterLevel()) pSupplyWater->ChangeWaterLevel(ServiceData.rsBridge.getWaterLv());
    if(ServiceData.key.isUpdatSoundVolume()) keySoundVolume(ServiceData.rsBridge.getSound());
    if(ServiceData.key.isUpdatDryMop()) pTaskCharging->dryMopOptionUpdate();
    if(ServiceData.key.isUpdatScheduleClean()) ServiceData.cleaningSchedule.setCleaningSchedule();
    if(ServiceData.key.isUpdatDistruptMode()) ServiceData.doNotDisturb.setDoNotDisturb();
    if(ServiceData.key.isUpdatCleanMode()) ServiceData.robotData.setCleanMode();
    if(ServiceData.key.isUpdatLanguage()) ServiceData.robotData.setLanguage();
    if(ServiceData.key.isUpdatCountry()) ServiceData.robotData.setCountry();
    if(ServiceData.key.isUpdatOta()) ServiceData.Ota.setAppData();
}

/**
 * @brief 사운드 크기 조절 함수
 *        현재는 둘 공간이 없어서 매니저에 넣음 추후에 잡다한 기능들을 관리하는 클래스 만들어서 옮길 예정
 * 
 * @param _soundLv 핸드폰 app에서 들어온 제어명령
 * 입력
 * 0: 제어 명령 안 들어옴
 * 1: 음소거
 * 2: 소리 1단계
 * 3,4,5 : ~
 * 6: 소리 5단계
 */
void CKeyHanler::keySoundVolume(short _soundLv)
{
    //ServiceData.soundVolume.set(_soundLv);
    ServiceData.awsData.setSendSoundLv(_soundLv);
    switch (_soundLv)
    {
    case 0:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_003);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 80%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control default");
        /* UI */
        break;
    case 1: // 음소거
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::MUTE);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 0%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control mute");
        break;
    case 2:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_001);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 60%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control 1");
        break;
    case 3:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_002);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 70%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control 2");
        break;
    case 4:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_003);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 80%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control 3");
        break;
    case 5:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_004);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 90%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control 4");
        break;
    case 6:
        DISPLAY_CTR.startDisplay(E_DisplayImageClass::VOLUME_005);
        SOUND_CTR.soundPlay(true,E_SoundClass::SOUND_DONE_EFFECT);
        system("sudo amixer set Master 100%");
        ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "sound control 5");
        break;
    
    default:
        ceblog(LOG_LV_NECESSARY, RED, "sound Cmd is problem.");
        break;
    }
    return;
}

E_KEY_VALUE CKeyHanler::checkTriggerOption(E_SERVICE_ID curId, E_SERVICE_STATUS curStatus)
{
    E_KEY_VALUE ret = E_KEY_VALUE::NONE;
    std::string curTime;
    std::string today;
    std::string startTime, endTime;
    
    if(ServiceData.doNotDisturb.getDoNotDisturb().status == 1){
        curTime = utils::math::getCurrentTimeString(1,3);
        startTime = ServiceData.doNotDisturb.getDoNotDisturb().startTime;
        endTime = ServiceData.doNotDisturb.getDoNotDisturb().endTime;

        if(utils::math::cmpTime(startTime,endTime,1)){
            ServiceData.doNotDisturb.setDistruptMode(true);
            ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "방해금지 시간입니다. 충전기로 이동합니다.");
            ret = E_KEY_VALUE::START_RESERVATION_STOP;
        }else{
            ServiceData.doNotDisturb.setDistruptMode(false);
        }
    }

    //ota 시간 trigger 
    if(!ServiceData.Ota.isRunning()){
        if(ServiceData.Ota.getAppData().force == 2){
            ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "펌웨어 업데이트 바로 시작합니다!!");
            ServiceData.Ota.setFirmWareUpdateFlag(true);
            ret = E_KEY_VALUE::START_FW_UPDATE;
        }
        else if(ServiceData.Ota.getAppData().scheduled == 2){
            curTime = utils::math::getCurrentTimeString(1,3);
            startTime = ServiceData.Ota.getAppData().scheduleTime;
            endTime = nullptr;//startTime+1;
            if(utils::math::cmpTime(startTime,endTime,1)){
                ceblog((LOG_LV_NECESSARY|LOG_LV_AWS), BLUE, "예약 펌웨어 업데이트를 시작합니다.");
                ServiceData.Ota.setFirmWareUpdateFlag(true);
                ret = E_KEY_VALUE::START_FW_UPDATE;
            }
        }
    }
    
    // if(ServiceData.cleaningSchedule.getCleaningSchedule().scheduleNumber != 0)
    // {
    //     std::list<tCleaningSchedule> schedule = ServiceData.cleaningSchedule.getCleaningSchedule().schedule;
    //     for(tCleanSchedule temp :  schedule)
    //     {
    //         today = utils::math::getCurrentTimeString(1,4);
    //         if(temp.isEnabled)
    //         {
    //             if(today == "monday" && temp.weeks == "1000000") 요일비교
    //             {
    //                 if(curTime == temp.time)
    //                 {
    //                     ret = E_KEY_VALUE::START_RESERVAITON_CLEAN;    
    //                 }
    //             }
    //         }
    //     }
    // }


    
    return ret;

}