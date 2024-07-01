#include "rsfMonitor.h"
#include "eblog.h"
#include "utils.h"
#include "define.h"
#include "motionPlanner/motionPlanner.h"
#include "debugCtr.h"
#include "lidarInterface.h"
#include "rosPublisher.h"
#include "subTask.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 170.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/


CRsfMonitor::CRsfMonitor(ros::NodeHandle _nh)
    : externData(_nh), serviceData(_nh)
{
    isRunningMessage = true;
    isRunningRosDebug = true;    
    isRunningRosDebugRbtPlusRotationVector = true;
    isRunningRosDebugRbtPlusFeedBack = true;
    isRunningRosDataPub = true;
    isRunningSystemWatch = true;
    isRunningDstarWallUpdate = true;
    isRunningWaveFrontier = true;        
    isRunningLidar = true;

    thMessage = std::thread(&CRsfMonitor::threadMessage, this);    

    pthread_create(&thRosDataPub, nullptr, &CRsfMonitor::threadRosDataPubWrap, this); //for_odom (필요한 쓰레드)
    pthread_create(&thRosImuDataPub, nullptr, &CRsfMonitor::threadRosDataImuPubWrap, this); //for_imu(필요한 쓰레드)
    pthread_create(&thSystemWatch, nullptr, &CRsfMonitor::threadSystemWatchWrap, this); //for_slam 동작 확인(필요한 쓰레드)
    pthread_create(&thTemporarySaveSlamMap, nullptr, &CRsfMonitor::threadTemporarySaveSlamMapWrap, this); //for_temp slam map saver
    pthread_create(&thLidar, nullptr, &CRsfMonitor::threadLidarWrap, this); //for_lidar
    pthread_create(&thRosDebug, nullptr, &CRsfMonitor::threadRosDebugSubWrap, this); //for_debug
#if defined (NEW_SLAM_CONTROL) && (NEW_SLAM_CONTROL == 0)
    pthread_create(&thSubMapFilter, nullptr, &CRsfMonitor::threadRosInsertSubMapFilterWrap, this); //for_submap_carto
#else 
    /* 카토그래퍼에서 서브 맵 업데이트 주기 및 제어를 스위칭 할 수 있는 기능(slam_ws 다시 배포할 예정임)
        # 슬램 업데이트 주기/슬램 업데이트 제어(기존 처럼)
        # 사용 방법
          1. 느리게 지도 업데이트 하려면 
             ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_EXPLORE);
             ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
          2. 빠르게 지도 업데잍 하려면 
             ROBOT_CONTROL.slam.setSubMapUpdateType(SUBMAP_UPDATE_TYPE::SUBMAP_UPDATE_TYPE_CLEAN);
             ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);
    */
    pthread_create(&thSlamControl, nullptr, &CRsfMonitor::threadRosSlamControlWrap, this);
#endif
    pthread_create(&thCleanMapProc, nullptr, &CRsfMonitor::threadCleanMapProcWrap, this);
#if 0 //사용하지 않음 ->  lidar topic publish는 wall timer로 이동
    pthread_create(&thLidarTf, nullptr, &CRsfMonitor::threadLidarTfPubWrap, this);
#endif
#if 0 //for_requset of the ceva ROS custom message
    pthread_create(&thRosDebugPlusRotationVector, nullptr, &CRsfMonitor::threadRosDebugRbtPlusRotationVectorWrap, this);
    pthread_create(&thRosDebugRbtPlusFeedBack, nullptr, &CRsfMonitor::threadRosDebugRbtPlusFeedBackWrap, this);
#endif

    enroll();
    eblog(LOG_LV_NECESSARY, "");
}

CRsfMonitor::~CRsfMonitor(){

    isRunningMessage = false;
    isRunningRosDebug = false;    
    isRunningRosDataPub = false;
    //for_requset of the ceva ROS custom message
    isRunningRosDebugRbtPlusRotationVector = false;
    isRunningRosDebugRbtPlusFeedBack = false;

    isRunningSystemWatch = false;
    isRunningDstarWallUpdate = false;
    isRunningWaveFrontier = false;    
    isRunningLidar = false;
    
    thMessage.join();
    pthread_join(thRosDebug, nullptr); //for_debug 목적 : 양산시 비 활성화 -> thread

#if 0  //for_requset of the ceva ROS custom message
    pthread_join(thRosDebugPlusRotationVector, nullptr);
    pthread_join(thRosDebugRbtPlusFeedBack, nullptr);
#endif
    pthread_join(thRosDebugSimplifyGridmap, nullptr);    
    pthread_join(thSystemWatch, nullptr);
    pthread_join(thTemporarySaveSlamMap, nullptr); //for 청소 중 슬램 지도 데이터 저장 목적
    pthread_join(thRosDataPub, nullptr); //for_odom
    pthread_join(thRosImuDataPub, nullptr); //for_imu
    pthread_join(thLidar, nullptr);
#if 0 //사용하지 않음 ->
    pthread_join(thLidarTf, nullptr);
#endif
#if defined (NEW_SLAM_CONTROL) && (NEW_SLAM_CONTROL == 0)
    pthread_join(thSubMapFilter, nullptr);
#else
    pthread_join(thSlamControl, nullptr);
#endif
    pthread_join(thCleanMapProc, nullptr);
    eblog(LOG_LV_NECESSARY,  "");
}

/**
 * @brief 업데이트할 Observer 들을 등록.
 * attach 하는 순서가 update 하는 순서입니다.
 */
void CRsfMonitor::enroll()
{
    attach(&serviceData.localiz);
    attach(&serviceData.obstacle);
    attach(&serviceData.signal);
    attach(&serviceData.robotMap);
    attach(&serviceData.tilting);
    attach(&serviceData.battery);
    attach(&serviceData.key);
    attach(&serviceData.power);
    attach(&serviceData.error);
    attach(&serviceData.Ota);
    attach(&serviceData.rsBridge); //항상 key데이터보다 늦게 attach되어야함 아닐시 app제어 동작 꼬임 (ex: app에 제어 했지만 robot제어로 판단)
}

CServiceData* CRsfMonitor::getServiceDataPointer()
{
    return &serviceData;
}

CExternData* CRsfMonitor::getExternDataPointer()
{
    return &externData;
}

void CRsfMonitor::update()
{   
    notify(&externData); // service data 업데이트
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    sendServiceDataToSystemInterface(); // service data -> system interface
#endif
}

void CRsfMonitor::messageTypeControlWheel(message_t* pMsg)
{  
    if (tMsgCtrWheel* arg = boost::get<tMsgCtrWheel>(&pMsg->arg)){
        if(arg->type == E_DRIVE_WHEEL_TYPE::VELOCITY){
            externData.systemData.transControlWheel(arg->dynamic.seq, arg->dynamic.linearVelocity, arg->dynamic.angularVelocity, arg->dynamic.radius);
        }
        else{
            externData.systemData.transControlWheel(arg->pwm.seq, arg->pwm.direction, arg->pwm.L_Speed,arg->pwm.R_Speed,arg->pwm.B_Speed, arg->pwm.duty, arg->pwm.bHeadingPid);
        }
    }
}

/*water Pump motor contorl msg add*/
void CRsfMonitor::messageTypeControlPump(message_t* pMsg){
    if (tMsgCtrPump* arg = boost::get<tMsgCtrPump>(&pMsg->arg)){
        externData.systemData.transControlPump(arg->on);
    }
}

/*tilting motor contorl msg add*/
void CRsfMonitor::messageTypeControlTilt(message_t* pMsg){
    if (tMsgCtrTilt* arg = boost::get<tMsgCtrTilt>(&pMsg->arg)){
        externData.systemData.transControlTilt(arg->control);
    }
}

/*Speaker contorl msg add*/
void CRsfMonitor::messageTypePlaySound(message_t* pMsg){
    if (tMsgCtrSound* arg = boost::get<tMsgCtrSound>(&pMsg->arg)){
        if(arg->stop){
            externData.systemData.transControlSoundStop();
        }else{
            externData.systemData.transControlSoundplay(arg->data);
        }
    }
}

/*LCD contorl msg add*/
void CRsfMonitor::messageTypePlayDisplay(message_t* pMsg){
    if (tMsgCtrDisplay* arg = boost::get<tMsgCtrDisplay>(&pMsg->arg)){
        if(arg->stop){
            externData.systemData.transControlDisplayStop();
        }else{
            if(!arg->bCustom)
                externData.systemData.transControlDisplay(arg->img);
            else// if(arg->bCustom)
                externData.systemData.transControlCustomDisplay(arg->customImg);
        }
    }
}

/*LED contorl msg add*/
void CRsfMonitor::messageTypePlayLed(message_t* pMsg){
    if (tMsgCtrLed* arg = boost::get<tMsgCtrLed>(&pMsg->arg)){
        externData.systemData.transControlLed(arg->dir, arg->red, arg->green, arg->blue, arg->white);
    }
}

/* dry mop fanMotor contorl msg add*/
void CRsfMonitor::messageTypeControlDryFan(message_t* pMsg){
    if (tMsgCtrDryfan* arg = boost::get<tMsgCtrDryfan>(&pMsg->arg)){
        externData.systemData.transControlDryFan(arg->on,arg->speed);
    }
}

void CRsfMonitor::messageTypeClaerLocalization(void)
{
    externData.systemData.transLocalization();
}

void CRsfMonitor::messageTypeInitSensor()
{
    externData.systemData.transInitSensor();
    eblog(LOG_LV, "");
}

void CRsfMonitor::messageTypeInitMoving()
{
    externData.systemData.transInitMoving();
    eblog(LOG_LV, "");
}

void CRsfMonitor::messageTypeActiveMode()
{
    externData.systemData.transActiveMode();
    eblog(LOG_LV, "");
}

void CRsfMonitor::messageTypeChargeMode()
{
    externData.systemData.transChargeMode();
    eblog(LOG_LV, "");
}

// void CRsfMonitor::messageTypeControlDockingIR()
// {
//     externData.systemData.transControlDockingIR();
//     eblog(LOG_LV, "");
// }

void CRsfMonitor::messageTypeSystemCtr(message_t* pMsg){
    
    if (tMsgCtrSystem* arg = boost::get<tMsgCtrSystem>(&pMsg->arg)){
        switch (arg->ctr)
        {
        case E_SYSTEM_CTR_MOTER_ON:
            eblog(LOG_LV,  "TODO : trans MOTOR ON");
            break;
        case E_SYSTEM_CTR_MOTER_OFF:
            eblog(LOG_LV,  "TODO : trans MOTOR OFF");
            break;
            case E_SYSTEM_CTR_CLEAR_LOCALIZTION: 
            eblog(LOG_LV,  "TODO : trans CLEAR LOCALIZATION");
            messageTypeClaerLocalization();
            break;    
        case E_SYSTEM_CTR_CLEAR_DISTANCE:
            eblog(LOG_LV,  "TODO : trans clera distance");
            break;
        case E_SYSTEM_CTR_CLEAR_ANGLE:
            eblog(LOG_LV,  "TODO : trans clera angle");
            break;
        default:
            eblog(LOG_LV,  "invalid ctr type : " << arg->ctr);
            break;
        }        
    }
}

void CRsfMonitor::messageTypePowerOff()
{
    ceblog(LOG_LV_NECESSARY, RED, " Power Off");
    externData.systemData.transControlPowerOff();
}

void CRsfMonitor::messageTypeMcuReset()
{
    ceblog(LOG_LV_NECESSARY, RED, " MCU RESET");
    externData.systemData.transControlMcuReset();
}

void CRsfMonitor::messageTypeReportAction(message_t* pMsg)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "message server bridge, Report Action ");
    if (tMsgReportAction* arg = boost::get<tMsgReportAction>(&pMsg->arg)){
        externData.appData.transAction( arg->actionValue);
    }
} 

void CRsfMonitor::messageTypeReportStatus(message_t* pMsg)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "message server bridge, Report Status");
    if (tMsgReportStatus* arg = boost::get<tMsgReportStatus>(&pMsg->arg)){
        externData.appData.transStatus( arg->statusValue);
    }
}

void CRsfMonitor::messageTypeReportAwsData(message_t* pMsg)
{
    tMsgReportData* arg = boost::get<tMsgReportData>(&pMsg->arg);

    ceblog(LOG_LV_NECESSARY, BLUE, "message server bridge, Report AwsData sort :" << enumToString(arg->sort));
    switch (arg->sort)
    {
    case E_AWS_MSG_SORT::WATER_LEVEL :
        externData.appData.transWaterLevel(arg->waterLv.value);
        break;
    case E_AWS_MSG_SORT::SOUND_LEVEL :
        externData.appData.transSoundLevel(arg->soundLv.value);
        break;
    case E_AWS_MSG_SORT::CLEAN_MODE :
        externData.appData.transCleanMode(arg->cleanMode.value);
        break;
    case E_AWS_MSG_SORT::LANGUAGE :
        externData.appData.transLanguage(arg->language.value);
        break;
    case E_AWS_MSG_SORT::COUNTRY :
        externData.appData.transCountry(arg->country.value);
        break;
    case E_AWS_MSG_SORT::BATTERY :
        externData.appData.transBatteryPercent(arg->battery.value);
        break;
    case E_AWS_MSG_SORT::DRY_MOP :
        externData.appData.transDryMopData(arg->dryData.dryEnabled,arg->dryData.dryHours,arg->dryData.dryPower);
        break;
    case E_AWS_MSG_SORT::ERROR :
        externData.appData.transErrorInfo(arg->error.errorCode,arg->error.errorDesc);
        break;
    case E_AWS_MSG_SORT::SETTTING :
        externData.appData.transSetting(arg->setting.apVersion,arg->setting.mcuVersion);
        break;
    case E_AWS_MSG_SORT::OPERATION_AREA :
        if(arg->operationArea.type == 1)        externData.appData.transCleanAll(arg->operationArea.all);
        else if(arg->operationArea.type == 2)   externData.appData.transCleanSpot(arg->operationArea.spot,arg->operationArea.spotNumber);
        else if(arg->operationArea.type == 3)   externData.appData.transCleanRoom(arg->operationArea.room,arg->operationArea.roomNumber);
        else if(arg->operationArea.type == 4)   externData.appData.transCleanCustom(arg->operationArea.custom,arg->operationArea.customNumber);
        break;
    case E_AWS_MSG_SORT::FORBIDDEN_AREA :
        if(arg->forbiddenArea.type == 1)        externData.appData.transForbiddenLine(arg->forbiddenArea.line,arg->forbiddenArea.lineNumber);
        else if(arg->forbiddenArea.type == 2)   externData.appData.transForbiddenRect(arg->forbiddenArea.rect,arg->forbiddenArea.rectNumber);
        break;
    case E_AWS_MSG_SORT::DISTRUBT_MODE :
        externData.appData.transDoNotDisturb(arg->distruptData.status,arg->distruptData.startTime,arg->distruptData.endTime);
        break;
    case E_AWS_MSG_SORT::RESERVATION_CLEAN :
        externData.appData.transCleaningSchedule(arg->rsvCleanData.schedule,arg->rsvCleanData.scheduleNumber);
        break;
    case E_AWS_MSG_SORT::SAVE_MAP_INFO :
        externData.appData.transSavedMapData(arg->saveMapInfo.mapData,arg->saveMapInfo.mapInfo,arg->saveMapInfo.robotPose,arg->saveMapInfo.traj,arg->saveMapInfo.uniqueKey,arg->saveMapInfo.mapName,arg->saveMapInfo.order,arg->saveMapInfo.areaInfo);
        break;
    case E_AWS_MSG_SORT::MAP_DATA :
        externData.appData.transTotalMapInfo( arg->mapData.mapData,arg->mapData.mapInfo, arg->mapData.robotPose, arg->mapData.traj, arg->mapData.cradleCoord, arg->mapData.cleanSize); 
        break;    
    case E_AWS_MSG_SORT::AREA_INFO :
        externData.appData.transAreaInfo(arg->areaInfo.info,arg->areaInfo.areaNumber);
        break;
     case E_AWS_MSG_SORT::DIVIDE_AREA :
        externData.appData.transDivideArea(arg->divideArea.data);
        break;
     case E_AWS_MSG_SORT::COMBINE_AREA :
        externData.appData.transCombineArea(arg->combineArea.data);
        break;        
    case E_AWS_MSG_SORT::CLEAN_HISTORY :
        externData.appData.transHistoryData(arg->cleanHistory.mapData,arg->cleanHistory.mapInfo,arg->cleanHistory.robotPose,arg->cleanHistory.traj,arg->cleanHistory.cradlePose,
                                            arg->cleanHistory.cleanStartTime,arg->cleanHistory.exitReason,arg->cleanHistory.cleanedSize,arg->cleanHistory.cleanTime,arg->cleanHistory.areaInfo);
        break;                        
    case E_AWS_MSG_SORT::ROBOT_POSE :
        externData.appData.transRobotPose(arg->robotPose.robotPose);
        break;
    case E_AWS_MSG_SORT::OTA_VERSION :
        externData.appData.transOtaVersion(arg->otaInfo.version);
        break;    
    default:
        ceblog(LOG_LV_NECESSARY | LOG_LV_AWS, BLUE, "message Aws Report Tpye Error!! ");
        break;
    }
}

void CRsfMonitor::messageTypeReportFactoryReset(message_t* pMsg)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "message server bridge, ReportFactoryReset");
    if (tMsgFactoryReset* arg = boost::get<tMsgFactoryReset>(&pMsg->arg)){
        externData.appData.transFactoryReset( arg->startTime,arg->descript);
    }
}

void CRsfMonitor::messageTypeOtaCommand( message_t* pMsg)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "message OtaCommand");
    if (tMsgOtaCmd* arg = boost::get<tMsgOtaCmd>(&pMsg->arg)){
        if(arg->type == E_SYSTEM_OTA_CMD::OPEN_CLOSE){
            externData.systemData.transOtaOnOff(arg->bOpen);
        }else if(arg->type == E_SYSTEM_OTA_CMD::START){
            externData.systemData.transOtaStart(arg->path);
        }else{
            ceblog(LOG_LV_NECESSARY, BLUE, "wrong commad!!");
        }
    }
}

void CRsfMonitor::messageTypePhoneInterface(message_t* pMsg)
{
    ceblog(LOG_LV_NECESSARY, BLUE, "message phone interface");
    if (tMsgInterfacePhone* arg = boost::get<tMsgInterfacePhone>(&pMsg->arg)){
        externData.appData.transPhoneInterface( arg->type);
    }
}

void CRsfMonitor::threadMessage()
{
    while(isRunningMessage) 
    {
        DEBUG_CTR.isAliveMessage.set(true);
        message_t message = MSGHANDLE.dequeue();
#if USE_SHARED_PTR == 1
        if (message == nullptr){
            eblog(LOG_LV, "is null thread_loop");
            return;
        }
#endif

        //ceblog(LOG_LV_NECESSARY, GREEN, "MSG type : " << (int)GET_MESSAGE_VAR(message, what));
        
        switch( GET_MESSAGE_VAR(message, what) )
        {
        case E_MESSAGE_TYPE_CONTROL_WHEEL:
            messageTypeControlWheel(&message);
            break;
        case E_MESSAGE_TYPE_CONTROL_PUMP:
            messageTypeControlPump( &message );
            break;            
        case E_MESSAGE_TYPE_CONTROL_TILT:
            messageTypeControlTilt( &message );
            break;            
        case E_MESSAGE_TYPE_CONTROL_DRYFAN:
            messageTypeControlDryFan( &message );
            break;
        case E_MESSAGE_TYPE_SOUND:
            messageTypePlaySound( &message );
            break;
        case E_MESSAGE_TYPE_DISPLAY:
            messageTypePlayDisplay( &message );                
            break;
        case E_MESSAGE_TYPE_LED:
            messageTypePlayLed( &message );           
            break;
        case E_MESSAGE_TYPE_SYSTEM_CTR:
            messageTypeSystemCtr( &message );                
            break;
        case E_MESSAGE_TYPE_INIT_SENSOR:
                messageTypeInitSensor();
            break;
        case E_MESSAGE_TYPE_INIT_MOVING:
                messageTypeInitMoving();
            break;
        case E_MESSAGE_TYPE_ACTIVE_MODE:
                messageTypeActiveMode();
            break;
        case E_MESSAGE_TYPE_CHARGE_MODE:
                messageTypeChargeMode();
            break;
        case E_MESSAGE_TYPE_POWER:
            messageTypePowerOff();
            break;
        case E_MESSAGE_TYPE_MCU_RESET:
            messageTypeMcuReset();
            break;
        case E_MESSAGE_TYPE_REPORT_ACTION: // AWS에 로봇 제어 응답 명령
            messageTypeReportAction(&message);
            break;
        case E_MESSAGE_TYPE_REPORT_STATUS: // AWS에 로봇 상태 SET 명령
            messageTypeReportStatus(&message);
            break;
        case E_MESSAGE_TYPE_REPORT_AWS_DATA:
            messageTypeReportAwsData(&message);
            break;
        case E_MESSAGE_TYPE_REPORT_FACTORY_RESET:
            messageTypeReportFactoryReset(&message);
            break;    
        case E_MESSAGE_TYPE_CONNECT_PHONE:
            messageTypePhoneInterface(&message);
            break;
        case E_MESSAGE_TYPE_ERROR:
            setErrorInfo( &message );
            break;
        case E_MESSAGE_TYPE_OTA_COMMAND:
            messageTypeOtaCommand(&message);
            break;
        default :
            eblog(LOG_LV_NECESSARY, "ERROR : default");
        break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if(isRunningMessage == false) 
    {
        eblog(LOG_LV_NECESSARY, "Message thread_Loop() break mIsRunning: " << isRunningMessage );
    }
}

/**
 * @brief ROS 수신 디버그
 * 
 */
void CRsfMonitor::threadRosDebugSub()
{
    unsigned int milliSec = 0;
    
    //for_RBT+ test or slam postion of the distance test
    bool bTestDistance;
    tPose slamPoseStart;
    tPose slamPoseEnd;
    unsigned int taskNumOld;

    ceblog(LOG_LV_NECESSARY, BOLDGREEN, "ros receive debug");
    
    while (isRunningRosDebug)
    {
        /* Debug용 다이나믹 제어 */
        // if ( externData.rosData.velocityControl.isUpdate() )
        // {
        //     tTwist ds = externData.rosData.velocityControl.get();

        //     ServiceData.key.velocityControl.set(ds);
            

        //     eblog(LOG_LV_NECESSARY, "ds.v : "<< ds.v<<" , ds.w : "<<ds.w);
        // }

        if ( externData.rosData.velocityControl.isUpdate() )
        {
            tTwist ds = externData.rosData.velocityControl.get();
            s32 linearVelocity = (s32)(ds.v*1000);
            s32 angularVelocity = (s32)(ds.w*1000);
            
            E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_WHEEL;
            tMsgCtrWheel msg = {};
            msg.type = E_DRIVE_WHEEL_TYPE::VELOCITY;
            msg.dynamic.seq = 0;
            msg.dynamic.linearVelocity = linearVelocity;
            msg.dynamic.angularVelocity = angularVelocity;
            msg.dynamic.radius = 150;
            SEND_MESSAGE(message_t(type, msg));
        }

        if ( externData.rosData.targetPose.isUpdate() )
        {
            ServiceData.key.targetPose.set(externData.rosData.targetPose.get());
        }

        /* Debug용 타겟 제어 */
        #if 0//TEST_BLOCK_SERVICE == 0
        if ( externData.rosData.targetPose.isUpdate() )
        {
           
            tPose targetPose = externData.rosData.targetPose.get();
            // serviceData.rosPublishData.pubTargetPose(targetPose);
            
            if ( targetPose.x == 0 && targetPose.y == 0 && targetPose.angle == 0) // 비상용 0,0 타겟 제어가 들어오면 네비게이션을 멈춤.
            {
                SUB_TASK.allStop();
                
                ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "타겟 제어 네비게이션을 멈춥니다. @@@@@@@@@@@@@@@@");
            }
            else if ( targetPose.x == 0 && targetPose.y == 0 )
            {
                CWayPoint wayPoint;
                wayPoint.clearAction();
                tAction action = tAction();
                action.profile.isStopAtTarget = false;
                // PATH_PLANNER.startTrackingWayPoints(wayPoint);
                // ceblog(LOG_LV_NECESSARY, BOLDGREEN, "way point 시작 ");
                
                ceblog(LOG_LV_NECESSARY, BOLDGREEN, "타겟 제어 네비게이션을 시작합니다. 목표 각도 ("<<RAD2DEG(targetPose.angle)<<" m)");
                MOTION.startRotateToAngleOnMap(robotPose, targetPose.angle, tProfile());
            }
            else
            {
                ceblog(LOG_LV_NECESSARY, BOLDGREEN, "타겟 제어 네비게이션을 시작합니다. 목표 좌표 ("<<targetPose.x<<", "<<targetPose.y<<")");
                // MOTION.startLinearToPointOnMap(robotPose, tPoint(targetPose.x, targetPose.y), tProfile());                
            }
        }
        #endif

        tSysPose sysPoseData = externData.systemData.getSysPoseData();
        tPose sysPose = tPose(sysPoseData.x, sysPoseData.y, sysPoseData.angle);        
        


            //for_RBT+ test or slam postion of the distance test
            if ( bTestDistance ) {
                bTestDistance = false;
                slamPoseEnd = serviceData.localiz.getSlamPose();
                ceblog(LOG_LV_NECESSARY, BOLDRED, "END::SLAM 좌표 ("<<slamPoseEnd.x<<", "<<slamPoseEnd.y<<")");
                double distance = utils::math::distanceTwoPoint(slamPoseStart, slamPoseEnd);
                ceblog(LOG_LV_NECESSARY, BOLDRED, "SLAM DISTANCE : " << distance );
            }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        milliSec += 10;
    }
}


/**
 * @brief 
 * 
 */
void CRsfMonitor::threadRosDebugRbtPlusRotationVector()
{
    while (isRunningRosDebugRbtPlusRotationVector)
    {
        CStopWatch __debug_sw;
        try
        {
            tSysPose sysPose = externData.systemData.getSysPoseData();
            unsigned int mcuTimeStamp = externData.systemData.getMcuTimeStamp();
            ROS.pubRbtPlusRotationVector(sysPose, mcuTimeStamp); //double x, double y, double angle, unsigned int mcuTime
        }
        catch(const std::exception& e)
        {
           // ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(__debug_sw.getTime());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/**
 * @brief 
 * 
 */
void CRsfMonitor::threadRosDebugRbtPlusFeedBack()
{
    while (isRunningRosDebugRbtPlusFeedBack)
    {
        CStopWatch __debug_sw;
        try
        {
            tSysPose sysPose = externData.systemData.getSysPoseData();
            unsigned int mcuTimeStamp = externData.systemData.getMcuTimeStamp();
            double heading = sysPose.angle; //radian ? degree

            ROS.pubRbtPlusFeedBack(heading, mcuTimeStamp);
        }
        catch(const std::exception& e)
        {
            //ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(__debug_sw.getTime());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/**
 * @brief ROS DATA publish 관리.
 * 
 */
void CRsfMonitor::threadRosImuDataPub()
{
    tSysIMU oldImu;
    while (isRunningRosDataPub)
    {
        CStopWatch __debug_sw;
        try
        {
            tSysIMU newImu = externData.systemData.getImuData();
            ROS.updateImuData(newImu);
            oldImu = newImu;

        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(__debug_sw.getTime());
#if 1 // 10ms_speed_test
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif
        //DEBUG_CTR.isAliveRosDataPub.set(true);
    }
}

void CRsfMonitor::threadRosDataPub()
{
    tSysPose oldSysPose;
    while (isRunningRosDataPub)
    {
        CStopWatch __debug_sw;
        try
        {
            tSysPose newSysPose = externData.systemData.getSysPoseData();
            ROS.updateOdometryData(newSysPose);
            oldSysPose = newSysPose;

        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(__debug_sw.getTime());
#if 1 //10ms_speed_test
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif
	DEBUG_CTR.isAliveRosDataPub.set(true);
    }
}


void CRsfMonitor::threadLidar()
{
    CLidarInterface lidarInterface;

    bool nRet = lidarInterface.init();
    if (!nRet) {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[lidar] Open serail port fail!");
    }
 
    while(isRunningLidar)
    {
        /**
         * @brief proc()에서 2ms 정도의 연산시간이 걸리고
         * proc()를 16번 실행을 하였을 때, ScanMsg가 업데이트 됨.
         * 
         */
        lidarInterface.proc();

        //lidar pwn 커널에서 실시간 처리함.
        //double current_lidar_speed = lidarInterface.getLidarCurrentSpeed();
        //ServiceData.tempLidarSpeed = current_lidar_speed; // 임시로 ServiceData에 lidar speed 값 저장.
        //double lidar_offset_angle = static_cast<double>(lidarInterface.getLidarOffsetAngle());
        //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "lidar_offset_angle: "<< lidar_offset_angle);
        //ROS.updateLidarOffset(lidar_offset_angle);
        if(lidarInterface.isUpdateScanMsg())
        {
            ROS.updateLidarData(lidarInterface.getScanMsg()); //lidar & odom time lantency
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1)); // 50us -> 5us -> 1us : LiDAR Time stamp & odom Time stamp 주기 보장 => 기능 구현 목적 
    }
}

/**
 * @brief 
 * 
 */
void CRsfMonitor::threadLidarTfPub()
{
    unsigned int milliSecCnt = 0;
#if 0 // lidar topic publish는 wall timer로 이동
    while (isRunningLidar)
    {
        CStopWatch __debug_sw;
        try
        {
            // serviceData.rosPublishData.pubLidarTf();
        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        TIME_CHECK_END(__debug_sw.getTime());
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        DEBUG_CTR.isAliveRosDataPub.set(true);
    }
#endif
}

/**
 * @brief 
 * 
 */
void CRsfMonitor::threadRosInsertSubMapFilter()
{
    while(isRunningSystemWatch)
    {
        //0 : 정지,이동 상태에서는 맵 생성됨/정지&회전 상태에서는 맵이 생성되지 않음
        //1 : 로봇 좌표만 생성, 맵은 생성되지 않음 ( 사용 처 : 틸업 혹 기타 경우 )
        //2 : 정지, 이동, 정지&회전 모두 맵 생성됨 ( 처음 탐색 시작 시점에서 맵 확장이 필요한 시점, 혹은 지도 로딩 시)
        MAP_FILTER_MODE mapFilterMode = ROBOT_CONTROL.slam.getMapFilterMode();
        int mapMode = static_cast<int>(mapFilterMode);
        //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "mapMode: "<< mapMode);
        ROS.updateSubmapFilter(mapMode); 
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 10ms -> 5ms
    }
}

/**
 * @brief 
 * 
 */
void CRsfMonitor::threadRosSlamControl()
{
    while(isRunningSystemWatch)
    {
        // 0 : normal ( 6 초 - 청소 시작 전 )
        // 1 : fast (1초 - 탐색이나 리로컬 등등)
        SUBMAP_UPDATE_TYPE update_type = ROBOT_CONTROL.slam.getSubMapUpdateType();
        int maxTime = static_cast<int>(update_type);

        //0 : 정지,이동 상태에서는 맵 생성됨/정지&회전 상태에서는 맵이 생성되지 않음
        //1 : 로봇 좌표만 생성, 맵은 생성되지 않음 ( 사용 처 : 틸업 혹 기타 경우 )
        //2 : 정지, 이동, 정지&회전 모두 맵 생성됨 ( 처음 탐색 시작 시점에서 맵 확장이 필요한 시점, 혹은 지도 로딩 시)
        MAP_FILTER_MODE mapFilterMode = ROBOT_CONTROL.slam.getMapFilterMode();
        int mapMode = static_cast<int>(mapFilterMode);

        ROS.pubSlamControlFunc(maxTime, mapMode );
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 10ms -> 5ms
    }
}

void CRsfMonitor::threadSystemWatch()
{
    unsigned int milliSecCnt = 0;
    while (isRunningSystemWatch)
    {
        try
        {
            if (milliSecCnt%5 == 0){
                serviceData.kidnapData.watchKidnap(serviceData.localiz.getSysPose(), serviceData.localiz.getSlamPose());
            }

            if (milliSecCnt%10 == 0){            
                if ( ROBOT_CONTROL.slam.watchSlam() )
                {
                    // serviceData.rosPublishData.pubInitPoseSet(tPose());
                }
            }
#if 0
            if (milliSecCnt%30 == 0){
                sendFakeBsp();
            }
#endif
        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }

        milliSecCnt++;        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        DEBUG_CTR.isAliveSystemWhatch.set(true);
    }
}

void CRsfMonitor::threadTemporarySaveSlamMap()
{
    unsigned int milliSecCnt = 0;
    while (isRunningSystemWatch)
    {
        try
        {
            //주기를 어떻게 하지..음.. 일단 1분 단위
            //청소 중 로봇 회전 아닌 상태에서 저장하는 것이 좋다.. 
            //가장 좋은 상태는 로봇 정지 상태..(청소 중이니 항상 움직이겠지만...)
            //회전 아닌 상태를 알 수 있나?
            //임시로 서비스 -> 청소 상태 체크
            //NONE =0 , IDLE  = 1, CLEAN = 2, CHARGING, DOCKING = 4, EXPLORER = 5, UNDOCKING, REDOCKING, WIFI,
            //빈 스레드만 일단 돌려 놓고.. 정해지면 카토그래퍼 센서 데이터 셋 저장하자..
            
            if (ROBOT_CONTROL.slam.isSlamRunning()) 
            {
                //eblog((LOG_LV_NECESSARY),"threadTemporarySaveSlamMap(서비스 ID) : " << ServiceData.tempRunServceID);
                if (ServiceData.tempRunServceID == 2)
                {
#if 0 //일단 저장되는 것만 확인됨.. 사용 여부는 나중에
                    //temp_map.pbstream 로 저장됨
                    //연속적으로 저장할 경우 덮어 씌우기가 됨
                    //들고 다시 내려 놓을 경우 슬램을 키기 위해서는 아래 함수를 사용하세요
                    //주의 :ROBOT_CONTROL.slam.runSlamTemporaryMap()
                    eblog((LOG_LV_NECESSARY),"청소 중 지도를 임시로 저장하자 ~~~");
                    int nResult = ROBOT_CONTROL.slam.saveSlamMapFile(true);
                    if (nResult == -1)
                    {
                        eblog(LOG_LV_NECESSARY, " write of the pbstream failure!!  " );
                    }
#endif
                }
            }
        }
        catch(const std::exception& e)
        {
            ceblog(LOG_LV_ERROR, YELLOW, "[Try-Exception] " << e.what());
        }
     
        std::this_thread::sleep_for(std::chrono::seconds(60)); //일단 60초 주기 쓰레드 처리 -> 분 단위로 저장.
    }
}

/// @brief BSP 테스트 데이타 확인 함수.
void CRsfMonitor::sendFakeBsp()
{
    CStopWatch __debug_sw;
    CImgProcessor imgProc;
    
    if (serviceData.robotMap.simplifyMap.isValidSimplifyGridMap())
    {
        ceblog(LOG_LV_NECESSARY, YELLOW, "9");
        serviceData.robotMap.simplifyMap.simplifyGridMapLock();
    
        tGridmapInfo info = serviceData.robotMap.simplifyMap.getSimplifyGridMapInfo();

        cv::Mat img = imgProc.convertSimplifyGridMap2Mat(
            serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(), 
            //serviceData.robotMap.robotTrajectory.getCleanedTrajectory(), 
            info.width, info.height
            //,info.resolution, info.origin_x, info.origin_y
            );

        serviceData.robotMap.simplifyMap.simplifyGridMapUnLock();
        
        std::vector<std::vector<cv::Point>> contours = imgProc.fakeBspV2(img);
        //DEBUG_PUB.publishContour(imgProc.contourToPolygon(contour));

        std::list<std::list<tPoint>> doors = imgProc.findDoor(img);
        for (auto& door : doors) { 
            for (auto& point : door) {                
                point.x = point.x * info.resolution + info.origin_x;
                point.y = point.y * info.resolution + info.origin_y;
            }
        }
        DEBUG_PUB.publishDoor(doors);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CRsfMonitor::threadCleanMapProc()
{
    while (true)
    {        
        if(serviceData.robotMap.simplifyMap.isValidSimplifyGridMap())
        {
            //심플리 정보를 클린맵에 업데이트
            serviceData.robotMap.simplifyMap.simplifyGridMapLock();            
            serviceData.robotMap.cleanMap.updateCellsBySimplifyMap(
                serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(), 
                serviceData.robotMap.simplifyMap.getInfo());
            serviceData.robotMap.simplifyMap.simplifyGridMapUnLock();

            DEBUG_PUB.publishCleanMap(serviceData.robotMap.cleanMap.getCellsPointer(),
                1000, 1000, 300);            

        }        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
}

void CRsfMonitor::sendServiceDataToSystemInterface()
{
    if(externData.rosData.getUpdateSlamPose())
    {
        sendSlamPoseData();
        externData.rosData.setUpdateSlamPose(false);
    }
}

timespec CRsfMonitor::getCurrentTime() {
    timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return currentTime;
}

long long CRsfMonitor::getTimeDifference(const timespec& start, const timespec& end) {
    long long diff;
    long long last = (end.tv_sec * 1000000000L) + end.tv_nsec;
    long long pre = (start.tv_sec * 1000000000L) + start.tv_nsec;

    diff = last - pre;
    return diff;
}

void CRsfMonitor::sendSlamPoseData()
{
    tPose slamPose;
    int cevaX = 0;
    int cevaY = 0;
    unsigned int cevaAngle = 0;
    float cavaFloatAngle = 0.0;
    //시작 시간 측정
    currTime = getCurrentTime();

    slamPose = serviceData.localiz.getSlamPose();
    cevaX = (int)(slamPose.x * 1000);
    cevaY = (int)(slamPose.y * 1000);

#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    cavaFloatAngle = static_cast<float>(utils::math::rad2deg(slamPose.angle));
    int lateOfMcuTime = LATE_OF_MCU_MS;
    //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "\t" << "|MCU 시간 조정:" << lateOfMcuTime);
#else
    cevaAngle = utils::math::rad2deg(slamPose.angle) * 10;
#endif
    //이전 이후 시간 차
    long long diff = getTimeDifference(preTime, currTime);
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 1)
    //ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "\t" << "|슬램 좌표 보내기:" << PRECISION(8) << cevaX << ", " << cevaY <<", "<< cavaFloatAngle << ", " << diff);
    externData.systemData.transSlamPoseData(cevaX, cevaY, cavaFloatAngle, lateOfMcuTime);
#else
    externData.systemData.transSlamPoseData(cevaX,cevaY,cevaAngle);
#endif
    //이전 시간 갱신
    preTime = currTime;
}