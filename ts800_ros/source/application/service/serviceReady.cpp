#include "serviceReady.h"
#include "motionPlanner/motionPlanner.h"
#include "kinematics.h"
#include <systemTool.h>

CServiceReady::CServiceReady()
{
    initServiceReady();
}

CServiceReady::~CServiceReady()
{

}

void CServiceReady::initServiceReady()
{
    setServiceReadyState(E_SERVICE_READY::CHECK_START);
    checkCount = 0;
    readyStartTime = 0;
    fanOn = false;
    powerCtrl = E_POWER_MODE::MODE_SLEEP;
}

void CServiceReady::clearServiceReadyStep()
{
    setCheckSensorStep(E_CEHCK_SENSOR_STEP::INIT_IMU);
    setTiltChangeStep(E_TILTING_CHANGE_STEP::INIT_ACTIVEMODE);
    setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::INIT_POWER_MODE_CHANGE);
    setCheckSlamStep(E_CHECK_SLAM_STEP::START_CHECK_SLAM);
}

E_SERVICE_READY CServiceReady::checkReadyStart(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::CHECK_START;
    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "CHECK_START -> CHECK_TILT : ");
    #if SKIP_CHECKTILT > 0
    ret = E_SERVICE_READY::CHECK_POWER_MODE;
    #else
    ret = E_SERVICE_READY::CHECK_TILT;
    #endif
    return ret;
}

void CServiceReady::setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP _step)
{
    if (step_power != _step)
    {
        readyStepTime = SYSTEM_TOOL.getSystemTime();
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN,  "current : "<< enumToString(step_power) << " next : " << enumToString(_step));
    }
    step_power = _step;
}

E_POWERMODE_CHANGE_STEP CServiceReady::getPowerModeChangeStep()
{
    return step_power;
}

void CServiceReady::setServiceReadyState(E_SERVICE_READY set)
{
    if (readystate != set)
    {
        if(set == E_SERVICE_READY::CHECK_TILT)              setTiltChangeStep(E_TILTING_CHANGE_STEP::INIT_ACTIVEMODE);
        else if(set == E_SERVICE_READY::CHECK_POWER_MODE)   setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::INIT_POWER_MODE_CHANGE);
        else if(set == E_SERVICE_READY::CHECK_SENSOR)       setCheckSensorStep(E_CEHCK_SENSOR_STEP::INIT_IMU);
        else if(set == E_SERVICE_READY::CHECK_SLAM)         setCheckSlamStep(E_CHECK_SLAM_STEP::START_CHECK_SLAM);
        else                                                clearServiceReadyStep();
        //readyStepTime = SYSTEM_TOOL.getSystemTime();
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "current : "<< enumToString(readystate) << " next : " <<enumToString(set));
    }
    readystate = set;
}

E_SERVICE_READY CServiceReady::getServiceReadyState()
{
    return readystate;
}


E_SERVICE_READY CServiceReady::stepServiceReadyPowerMode(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::CHECK_POWER_MODE;
    E_POWER_STATE powerState = ServiceData.power.getPowerState(); 

    switch (getPowerModeChangeStep())
    {
       case E_POWERMODE_CHANGE_STEP::INIT_POWER_MODE_CHANGE:
        if(service_id == E_SERVICE_ID::CHARGING)    powerCtrl = E_POWER_MODE::MODE_CHARGE;
        else                                        powerCtrl = E_POWER_MODE::MODE_ACTIVE;
        
        ROBOT_CONTROL.systemModeControl(powerCtrl);
        setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::CHECK_POWER_MODE);
       break;

        case E_POWERMODE_CHANGE_STEP::CHECK_POWER_MODE:
        if((powerCtrl == E_POWER_MODE::MODE_CHARGE && powerState == E_POWER_STATE::CHARGE) || (powerCtrl == E_POWER_MODE::MODE_ACTIVE && powerState == E_POWER_STATE::ACTIVE))
        {
            setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::INIT_FAN_CONTROL);
        }
        else if(SYSTEM_TOOL.getSystemTime()-readyStepTime >= 0.1)
        {
            setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::INIT_POWER_MODE_CHANGE);
        }
        break;

        case E_POWERMODE_CHANGE_STEP::INIT_FAN_CONTROL:        
        setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP::COMPLETE);
        break;

        case E_POWERMODE_CHANGE_STEP::COMPLETE:
        if(service_id == E_SERVICE_ID::CHARGING)
        {
            ret = E_SERVICE_READY::CHECK_SLAM;//E_SERVICE_READY::COMPLETE;
        }
        else
        {
            setCheckSensorStep(E_CEHCK_SENSOR_STEP::INIT_IMU);
            ret = E_SERVICE_READY::CHECK_SENSOR;    
        }
        break;
    default:
        break;
    }
    
    return ret;
}

E_SERVICE_READY CServiceReady::stepServiceReadyLidar(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::TEMP_CHECK_LIDAR;
#if 0 //나중에 컨셉 정하면 라이더 모터 제어 걸자
    //IDLE일 경우 -> lidar motor off
    //타 서비스 일 경우 -> lidar motor on
    //단 충전 상태에서는 lidar가 자동으로 전원이 off됨에 따라서 제어 할 필요가 없다
    switch(service_id)
    {
        case E_SERVICE_ID::IDLE:
            ROBOT_CONTROL.lidar.setEnableLidarMotor(false);
            break;

        default:
            ROBOT_CONTROL.lidar.setEnableLidarMotor(true);
            break;
    }
#endif
     ret = E_SERVICE_READY::CHECK_SLAM;

    return ret;
}

void CServiceReady::setTiltChangeStep(E_TILTING_CHANGE_STEP _step)
{
    if (step_tilt != _step)
    {
        readyStepTime = SYSTEM_TOOL.getSystemTime();
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN,  "current : "<< enumToString(step_tilt)<< " next : " << enumToString(_step));
    }

    step_tilt = _step;
}

void CServiceReady::setRelocalFirstChangeStep(RELOCALIZE_STEP _step)
{
    if (mStep != _step)
    {
        readyStepTime = SYSTEM_TOOL.getSystemTime();
    }

    mStep = _step;
}

E_TILTING_CHANGE_STEP CServiceReady::getTiltChangeStep()
{
    return step_tilt;
}

E_SERVICE_READY CServiceReady::stepServiceReadyTilt(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::CHECK_TILT;
    E_POWER_STATE powerState = ServiceData.power.getPowerState();
    E_SYS_TILT_STATE tiltState = ServiceData.tilting.getStateValue();
    tAction action;

    switch (getTiltChangeStep())
    {
    case E_TILTING_CHANGE_STEP::INIT_ACTIVEMODE :
        if(powerState != E_POWER_STATE::ACTIVE)    ROBOT_CONTROL.systemModeControl(E_POWER_MODE::MODE_ACTIVE);
        setTiltChangeStep(E_TILTING_CHANGE_STEP::CHECK_ACTIVEMODE);
        break;
    case E_TILTING_CHANGE_STEP::CHECK_ACTIVEMODE :
        if(powerState == E_POWER_STATE::ACTIVE)
        {
            if(service_id == E_SERVICE_ID::CHARGING || service_id == E_SERVICE_ID::UNDOCKING)
            {
                ceblog(LOG_LV_NECESSARY, GREEN, " > 틸팅 UP");
                ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
            }
            else
            {
                ceblog(LOG_LV_NECESSARY, GREEN, " > 틸팅 DOWN");
                ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
            }
            
            ROBOT_CONTROL.WaterPump(false);
           
            setTiltChangeStep(E_TILTING_CHANGE_STEP::CHECK_TILT_STATE);
        }
        else if(SYSTEM_TOOL.getSystemTime()-readyStepTime >= 1.5)
        {
            setTiltChangeStep(E_TILTING_CHANGE_STEP::INIT_ACTIVEMODE);
        }
        break;
    case E_TILTING_CHANGE_STEP::CHECK_TILT_STATE :
        // service_id가 OTA, WiFi 접속 중이면....????
        
        if((service_id == E_SERVICE_ID::CHARGING || service_id == E_SERVICE_ID::UNDOCKING)
        && (tiltState == E_SYS_TILT_STATE::TILTED_UP))
        {
            ceblog((LOG_LV_DOCKING|LOG_LV_NECESSARY), RED, "틸 업 체크 완료!!! power State[" << DEC(powerState) << "] tilt state[" << enumToString(tiltState) << "] time[" << SYSTEM_TOOL.getSystemTime() << "]");
            setTiltChangeStep(E_TILTING_CHANGE_STEP::COMPLETE);
        }
        else if ((service_id != E_SERVICE_ID::CHARGING && service_id != E_SERVICE_ID::UNDOCKING)
        && (tiltState == E_SYS_TILT_STATE::TILTED_DOWN))
        {
            ceblog((LOG_LV_DOCKING|LOG_LV_NECESSARY), RED, "틸 다운 체크 완료!!! power State[" << DEC(powerState) << "] tilt state[" << enumToString(tiltState) << "] time[" << SYSTEM_TOOL.getSystemTime() << "]");
            setTiltChangeStep(E_TILTING_CHANGE_STEP::COMPLETE);
        }
        break;
    case E_TILTING_CHANGE_STEP::COMPLETE :
        if(service_id == E_SERVICE_ID::CHARGING) ret = E_SERVICE_READY::CHECK_POWER_MODE;
        else                                     ret = E_SERVICE_READY::CHECK_SENSOR;
        break;                
    default:
        break;
    }

    return ret;
}


void CServiceReady::setCheckSensorStep(E_CEHCK_SENSOR_STEP _step)
{
    if (step_sensor != _step)
    {
        readyStepTime = SYSTEM_TOOL.getSystemTime();
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "current : "<< enumToString(step_sensor) <<" set : " << enumToString(_step));
    }
    step_sensor = _step;
}

E_CEHCK_SENSOR_STEP CServiceReady::getCheckSensorStep()
{
    return step_sensor;
}

E_SERVICE_READY CServiceReady::stepServiceReadyCheckSensor(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::CHECK_SENSOR;
    RSU_OBSTACLE_DATA* pObstacle = ServiceData.obstacle.getObstacleData();
    
    //eblog(LOG_LV_NECESSARY,  "stepServiceReadyCheckSensor");

    switch (getCheckSensorStep())
    {
    case E_CEHCK_SENSOR_STEP::INIT_IMU:
        setCheckSensorStep(stepInitImu(service_id,(E_IMU_STATUS)pObstacle->imu.filteredState));
    break;
    case E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT:
        setCheckSensorStep(stepCheckImuInit((E_IMU_STATUS)pObstacle->imu.filteredState));
    break;
    case E_CEHCK_SENSOR_STEP::CHECK_IMU_READY:
        setCheckSensorStep(stepCheckImuReay(service_id,(E_IMU_STATUS)pObstacle->imu.filteredState));
    break;
    case E_CEHCK_SENSOR_STEP::INIT_TOF:
        setCheckSensorStep(stepInitTof(pObstacle->tof));
    break;
    case E_CEHCK_SENSOR_STEP::CHECK_TOF_READY:
        setCheckSensorStep(stepCheckTofReady(pObstacle->tof));
    break;
    case E_CEHCK_SENSOR_STEP::COMPLETE:
        ret = E_SERVICE_READY::TEMP_CHECK_LIDAR;// E_SERVICE_READY::CHECK_SLAM; ret = E_SERVICE_READY::COMPLETE;
    default:
        break;
    }

    return ret;
}

E_CEHCK_SENSOR_STEP CServiceReady::stepInitImu(E_SERVICE_ID service_id,E_IMU_STATUS status){
    E_CEHCK_SENSOR_STEP ret = E_CEHCK_SENSOR_STEP::INIT_IMU;

    if(service_id == E_SERVICE_ID::UNDOCKING)
    {
        ROBOT_CONTROL.clearSystemLocalization();
        ROBOT_CONTROL.system.initImuSensor();
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "INIT_IMU -> CHECK_IMU_INIT ");
        ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT;
    }
    else if(service_id == E_SERVICE_ID::CLEAN || service_id == E_SERVICE_ID::DOCKING)
    {
        if(!ROBOT_CONTROL.slam.isSlamRunning()) //hjkim231201 - 탐색종료후 청소시작할때 imu초기화 하지 않도록 임시 수정(지도 돌아감) - 서비스 및 Slam 시퀀스 맞춰야함
        {
            ROBOT_CONTROL.system.initImuSensor();
            ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "INIT_IMU -> CHECK_IMU_INIT ");
            ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT;
        }
        else
        {
            ret = E_CEHCK_SENSOR_STEP::INIT_TOF;
        }
    }
    else
    {
        ROBOT_CONTROL.system.initImuSensor();
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "INIT_IMU -> CHECK_IMU_INIT ");
        ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT;
    }
    
    return ret;
}

E_CEHCK_SENSOR_STEP CServiceReady::stepCheckImuInit(E_IMU_STATUS status){
    E_CEHCK_SENSOR_STEP ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT;
    if (status != E_IMU_STATUS::IMU_READY)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "INIT_CHECK_IMU -> CHECK_IMU_READY ");
        ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_READY;
    }
    return ret;
}

E_CEHCK_SENSOR_STEP CServiceReady::stepCheckImuReay(E_SERVICE_ID service_id , E_IMU_STATUS status){
    E_CEHCK_SENSOR_STEP ret = E_CEHCK_SENSOR_STEP::CHECK_IMU_READY;
    // hhryu240405 : 3초 기다린 후 체크 --> status LPF 적용으로 인해 삭제.
    if (status == E_IMU_STATUS::IMU_READY)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), GREEN, "IMU Sensor OK !");
        if (service_id == E_SERVICE_ID::UNDOCKING)
        {
            ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "충전 중에 ToF 초기화 불가능, CHECK_IMU_READY -> COMPLETE ");
            ret = E_CEHCK_SENSOR_STEP::COMPLETE;
        }
        else
        {
            ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "CHECK_IMU_READY -> INIT_TOF ");
            ret = E_CEHCK_SENSOR_STEP::INIT_TOF;
        }
    }
    else if(SYSTEM_TOOL.getSystemTime()-readyStepTime > 3.5)
    {
        ROBOT_CONTROL.robotSystemReset();
        ret = E_CEHCK_SENSOR_STEP::INIT_IMU;
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "IMU 센서 초기화에 실패하였습니다. 확인해주세요. (IMU INIT 3.5 SEC OVER)");
    }
    else
    {
        ceblog((LOG_LV_SERVICESTEP), WHITE, "레디를 기다려요 status[" << enumToString(status) <<"]");
    }

    return ret;
}

E_CEHCK_SENSOR_STEP CServiceReady::stepInitTof(tTofData tof){
    E_CEHCK_SENSOR_STEP ret = E_CEHCK_SENSOR_STEP::INIT_TOF;

    ROBOT_CONTROL.system.initTofSensor();
    ServiceData.obstacle.initCliffAccumulate();
    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "INIT_TOF -> CHECK_TOF_READY ");
    ret = E_CEHCK_SENSOR_STEP::CHECK_TOF_READY;
    return ret;
}

E_CEHCK_SENSOR_STEP CServiceReady::stepCheckTofReady(tTofData tof){
    E_CEHCK_SENSOR_STEP ret = E_CEHCK_SENSOR_STEP::CHECK_TOF_READY;

    if(ServiceData.obstacle.isCliffAccumulate() && 
        tof.knoll.deviceState == E_TOF_STATUS::TOF_READY &&
        tof.leftwall.deviceState == E_TOF_STATUS::TOF_READY && 
        tof.rightwall.deviceState == E_TOF_STATUS::TOF_READY && 
        tof.lcliff.deviceState == E_TOF_STATUS::TOF_READY && 
        tof.rcliff.deviceState == E_TOF_STATUS::TOF_READY)
    {        
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "CHECK_TOF_READY -> CHECK_SLAM_INIT ");	
        ret = E_CEHCK_SENSOR_STEP::COMPLETE;		
    }
    
    if(SYSTEM_TOOL.getSystemTime()-readyStepTime >= 3) // tof 초기화 시점으로 부터 3초 이내 초기화 안되면 재시도 - 반복 시 에러
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "ToF 센서 초기화에 실패했습니다.(TOF SENSOR INIT 3 SEC OVER)");

        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "Init 0, ready 4, error 99");
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "tof.knoll.deviceState : " << tof.knoll.deviceState << " data : " << tof.knoll.rangeAvg);
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "tof.leftwall.deviceState : " << tof.leftwall.deviceState << " data : " << tof.leftwall.rangeAvg);
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "tof.rightwall.deviceState : " << tof.rightwall.deviceState << " data : " << tof.rightwall.rangeAvg);
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "tof.lcliff.deviceState : " << tof.lcliff.deviceState << " data : " << tof.lcliff.rangeAvg);
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "tof.rcliff.deviceState : " << tof.rcliff.deviceState << " data : " << tof.rcliff.rangeAvg);
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), BOLDRED, "isCliffAccumulate : " << ServiceData.obstacle.isCliffAccumulate());
        ROBOT_CONTROL.robotSystemReset();
        ret = E_CEHCK_SENSOR_STEP::INIT_TOF;
    }
    
    return ret;
}

void CServiceReady::setCheckSlamStep(E_CHECK_SLAM_STEP _step)
{
    if (step_slam != _step)
    {
        readyStepTime = SYSTEM_TOOL.getSystemTime();
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "current : "<< enumToString(step_slam) << " set : " << enumToString(_step));
    }

    step_slam = _step;
}

E_CHECK_SLAM_STEP CServiceReady::getCheckSlamStep()
{
    return step_slam;
}

E_SERVICE_READY CServiceReady::stepServiceReadyCheckSlam(E_SERVICE_ID service_id)
{
    E_SERVICE_READY ret = E_SERVICE_READY::CHECK_SLAM;
    
    switch (getCheckSlamStep())
    {    
    case E_CHECK_SLAM_STEP::START_CHECK_SLAM:
        /* code */
        setCheckSlamStep(stepCheckSalmStart(service_id));
        break;
    case E_CHECK_SLAM_STEP::INIT_SLAM:
        /* code */
        setCheckSlamStep(stepInitSlam());
        break;
    case E_CHECK_SLAM_STEP::INIT_LOCALIZE:
        setCheckSlamStep(stepInitLocalize());
    break;
    case E_CHECK_SLAM_STEP::RUN_IMU_CALIBRATION:
         {
            //max of the cava yaw calibaration time margine max = 60 or 30 sec
            if (SYSTEM_TOOL.getSystemTick()-imuCalibrationTime >= 60*SEC_1) {
                setCheckSlamStep(stepIMUCalibration());
            } else {
                ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "RUN_IMU_CALIBRATION -> wating");
            }
         }
    break;
    case E_CHECK_SLAM_STEP::CHECK_LOCALIZE_INIT:
        setCheckSlamStep(stepCheckLocalizeInit());
    break;
    case E_CHECK_SLAM_STEP::CHECK_LOCALIZE_READY:
        setCheckSlamStep(stepCheckLocalizeReay());
    break;
    case E_CHECK_SLAM_STEP::CHECK_SLAM_READY:
        /* code */
        setCheckSlamStep(stepCheckSlamReady());
        break;
    // case E_CHECK_SLAM_STEP::CHECK_REROCALIZE:
    //     setCheckSlamStep(stepReLocalize());
    //     break;
    case E_CHECK_SLAM_STEP::CHECK_MAP:
        setCheckSlamStep(stepCheckMap());
        break;
    case E_CHECK_SLAM_STEP::EXIT_SLAM:
        setCheckSlamStep(stepExitSlam());
    case E_CHECK_SLAM_STEP::COMPLETE:
        ret = E_SERVICE_READY::COMPLETE;
        break;
    default:
        break;
    }

    return ret;
}


E_CHECK_SLAM_STEP CServiceReady::stepInitLocalize()
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::INIT_LOCALIZE;

    ROBOT_CONTROL.clearSystemLocalization();
    ret = E_CHECK_SLAM_STEP::CHECK_LOCALIZE_INIT;
    ceblog(LOG_LV, CYN, "INIT_LOCALIZE -> CHECK_LOCALIZE_INIT ");
    return ret;
}

E_CHECK_SLAM_STEP CServiceReady::stepIMUCalibration()
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::RUN_IMU_CALIBRATION;

    //if ( status != E_IMU_STATUS::IMU_READY)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "RUN_IMU_CALIBRATION -> CHECK_LOCALIZE_INIT ");
        ret = E_CHECK_SLAM_STEP::CHECK_LOCALIZE_INIT;
    }
}
E_CHECK_SLAM_STEP CServiceReady::stepCheckLocalizeInit()
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_LOCALIZE_INIT;

    //if ( status != E_IMU_STATUS::IMU_READY)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), CYN, "CHECK_LOCALIZE_INIT -> CHECK_LOCALIZE_READY ");
        ret = E_CHECK_SLAM_STEP::CHECK_LOCALIZE_READY;
    }
}

E_CHECK_SLAM_STEP CServiceReady::stepCheckLocalizeReay(){
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_LOCALIZE_READY;
    //if (status == E_IMU_STATUS::IMU_READY && SYSTEM_TOOL.getSystemTick()-readyStepTick >= 3*SEC_1)

    ret = E_CHECK_SLAM_STEP::CHECK_SLAM_READY;
    return ret;
}

E_CHECK_SLAM_STEP CServiceReady::stepCheckSalmStart(E_SERVICE_ID service_id)
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::START_CHECK_SLAM;
    
    switch (service_id)
    {
        case E_SERVICE_ID::IDLE:
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "IDLE::CHECK_START -> EXIT_SLAM ");
        //ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
        ret = E_CHECK_SLAM_STEP::COMPLETE;
        break;

        case E_SERVICE_ID::CLEAN:
        if ( ROBOT_CONTROL.slam.isSlamRunning())
        {
            ret = E_CHECK_SLAM_STEP::CHECK_SLAM_READY;
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "CHECK_START -> CHECK_SLAM_INIT ");
        }
        else
        {
            ret = E_CHECK_SLAM_STEP::INIT_SLAM;
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "CHECK_START -> INIT_SLAM ");
        }
        break;
        case E_SERVICE_ID::EXPLORER:
        if ( ROBOT_CONTROL.slam.isExistedSlamMap())
        {
            ServiceData.mapStorage.resetChargerPose();
            //removeSlamMapFile-> arg is true : temp map remove & false : saved map remove
            bool nResult = ROBOT_CONTROL.slam.removeSlamMapFile(false);
            if (!nResult) 
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), " 지도 삭제 실패");
            } 
            else 
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), " 지도 삭제 성공");
            }
        } 
        else 
        {
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), " 지도가 존재하지 않습니다");
        }
        ret = E_CHECK_SLAM_STEP::INIT_SLAM;
        break;
        case E_SERVICE_ID::CHARGING:
        //ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
        ret = E_CHECK_SLAM_STEP::COMPLETE;
        break;
        case E_SERVICE_ID::DOCKING:
        if (ROBOT_CONTROL.slam.isSlamRunning())
        {
            if (ROBOT_CONTROL.slam.isExistedSlamMap())
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP|LOG_LV_DOCKING), "slam on, map exist : relocalize ");
            }
            ret = E_CHECK_SLAM_STEP::CHECK_SLAM_READY;
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "CHECK_START -> CHECK_SLAM_READY ");
        }
        else
        {
            ret = E_CHECK_SLAM_STEP::INIT_SLAM;
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP|LOG_LV_DOCKING), "slam off or map X : CHECK_START -> INIT_SLAM");
        }
        break;
        case E_SERVICE_ID::UNDOCKING:
            ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
            //ret = E_CHECK_SLAM_STEP::COMPLETE;
        break;
        case E_SERVICE_ID::REDOCKING:
            ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
        break;
        case E_SERVICE_ID::WIFI:
             ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
        break;
        default:
            break;
    }

    return ret;
}   

E_CHECK_SLAM_STEP CServiceReady::stepInitSlam()
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::INIT_SLAM;
    ROBOT_CONTROL.slam.setSlamLocalUpdate(false);
    if(!ROBOT_CONTROL.slam.isSlamRunning())
    {
        ret = E_CHECK_SLAM_STEP::INIT_LOCALIZE;
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "INIT_SLAM -> CHECK_SLAM_READY ");
        ROBOT_CONTROL.slam.runSlam();
    }     
    else
    {
        ROBOT_CONTROL.slam.exitSlam();
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "SLAM RUNNING -> EXIT SLAM & INIT");
    }                                        
    
    return ret;
} 
E_CHECK_SLAM_STEP CServiceReady::stepCheckSlamReady(){
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_SLAM_READY;
    if ( ROBOT_CONTROL.slam.isSlamRunning()) // && ROBOT_CONTROL.slam.isSlamLocalUpdate()) 
    {
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
        ret = E_CHECK_SLAM_STEP::CHECK_MAP;
    }
    
    return ret;
}
E_CHECK_SLAM_STEP CServiceReady::stepCheckMap(){
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_MAP;

    //if(checkReadyMap()) //map ready 의미가 무엇인지 정의할 필요가 있음. map ready 후 area 생성 오류로 인해 청소시작 안되는 경우가 있는데 area생성이 안되는데 map ready는 무슨 의미가 있는지?
    {
        ret = E_CHECK_SLAM_STEP::COMPLETE;
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "CHECK_MAP -> COMPLETE ");
    }
    
    return ret;
}
E_CHECK_SLAM_STEP CServiceReady::stepExitSlam(){
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::EXIT_SLAM;
    ROBOT_CONTROL.slam.setSlamLocalUpdate(false);
    ROBOT_CONTROL.slam.exitSlam();
    ret = E_CHECK_SLAM_STEP::COMPLETE;
    return ret;
}
/**
 * @brief with of the cartograper scan-matcher relocalize  
 * 
 * ROTATE_360 -> ROTATE_360 -> 90_ROTATE -> 180_ROTATE-> 90_ROTATE
 * SET SENARIO
 * IS MAP -> NEXT CLEN
 * IS NOT MAP -> NEXT EXPLORER
 */
E_CHECK_SLAM_STEP CServiceReady::stepReLocalize()
{
    E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_REROCALIZE;
    CRobotKinematics k;
    tPose robotPose = ServiceData.localiz.getPose();
    //CStopWatch __debug_sw;
	//hjkim231201 - 너무 빨리돌리면 지도가 돌아가 버려요..저속회전 필요 && 심플리파이 맵이 업데이트 될 수 있도록 회전을 더 많이하도록 수정
    tProfile profile = tProfile();
    profile.desAngVel = DEG2RAD(30);
    switch (mStep)
    {
        case RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP:
            {
                ROBOT_CONTROL.slam.setRelocalCompleted(false);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 1");
                
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle); 

                mStep = RELOCALIZE_STEP::RELOCALIZE_SECOND_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_SECOND_STEP:
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 2");
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle);
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCALIZE_STEP::RELOCALIZE_THIRD_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_THIRD_STEP:
            if(!MOTION.isRunning()) //(MOTION.isRunning() == false)
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 3");
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle); 
                
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
                relocalTime = SYSTEM_TOOL.getSystemTick();
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP;//RELOCALIZE_STEP::RELOCALIZE_FORTH_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_FORTH_STEP:
            if(!MOTION.isRunning())
            {
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 4");
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle); 
                
                double targetAng = k.rotation(robotPose, DEG2RAD(90));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP;//RELOCALIZE_STEP::RELOCALIZE_SIXTH_STEP;
            }
            
            break;

        case RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP:
            if(!MOTION.isRunning())//if (MOTION.isRunning() == false)
            {
                if (SYSTEM_TOOL.getSystemTick()-relocalTime >= 5*SEC_1)
                {
                    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 5");
                    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                    eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle); 

                    mStep = RELOCALIZE_STEP::RELOCALIZE_END_STEP;
                }
            }
            break;

        // case RELOCALIZE_STEP::RELOCALIZE_SIXTH_STEP:
        //     if(!MOTION.isRunning())//if (MOTION.isRunning() == false)
        //     {
        //         eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP), "자기위치 추정 스탭 6");
        //         eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
        //         eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
        //         eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
        //         eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle); 
                
        //         MOTION.startRotateToAngleOnRobot(robotPose, DEG2RAD(90) * -1, profile);
                
        //         mStep = RELOCALIZE_STEP::RELOCALIZE_END_STEP;
        //         relocalTime = SYSTEM_TOOL.getSystemTick();
        //     }
        //     break;

        case RELOCALIZE_STEP::RELOCALIZE_END_STEP:
            if(!MOTION.isRunning())//if (SYSTEM_TOOL.getSystemTick()-cleanReLocalizeStarttick >= 2*SEC_1)
            {
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP;

                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP)," STEP_END::자기위치 추정 성공 ");
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose angle(degree): " << SC<double>(utils::math::rad2deg(ServiceData.localiz.getSlamPose().angle)));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose angle(degree ): "<< RAD2DEG(ServiceData.localiz.getSysPose().angle));
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|SlamPose: angle (radian): " << ServiceData.localiz.getSlamPose().angle);
                eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),"\t" << "|CevaPose: angle (radian): " << ServiceData.localiz.getSysPose().angle);           
                ROBOT_CONTROL.slam.setRelocalCompleted(true);
                ret = E_CHECK_SLAM_STEP::CHECK_MAP;
            }
            break;

        default:
            break;
    }

    return ret;

    //TIME_CHECK_END(__debug_sw.getTime());
}

bool CServiceReady::runReayServiceStep(E_SERVICE_ID service_id)
{
    bool ret = false;
    
    switch (getServiceReadyState())
    {
    case E_SERVICE_READY::CHECK_START:
        setServiceReadyState(checkReadyStart(service_id));
        break;
    case E_SERVICE_READY::CHECK_TILT:
        setServiceReadyState(stepServiceReadyTilt(service_id));
    break;    
    case E_SERVICE_READY::CHECK_POWER_MODE:
        setServiceReadyState(stepServiceReadyPowerMode(service_id));
    break;
    case E_SERVICE_READY::TEMP_CHECK_LIDAR:
        setServiceReadyState(stepServiceReadyLidar(service_id));
    break;
    case E_SERVICE_READY::CHECK_SENSOR:
        setServiceReadyState(stepServiceReadyCheckSensor(service_id));
    break;
    case E_SERVICE_READY::CHECK_SLAM:
        setServiceReadyState(stepServiceReadyCheckSlam(service_id));
    break;
    case E_SERVICE_READY::COMPLETE:
        #if SKIP_CHECKTILT > 0
        if(service_id == E_SERVICE_ID::UNDOCKING)
        {
            ceblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),RED, "TILTING is Block CanNot Start UnDocking Please Robot Move to the Ground by Manual!!!!!!!!");
        }
        else
        {
            ret = true;
            eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "READY COMPLETE TO SERVICE :  " << enumToString(service_id) <<  " TotalReadyTime :"<< (int)(SYSTEM_TOOL.getSystemTime()-readyStartTime));
        }
        #else
        ret = true;
        eblog((LOG_LV_NECESSARY|LOG_LV_SERVICESTEP),  "COMPLETE -> CLEAN READY TotalReadyTime : " << (int)(SYSTEM_TOOL.getSystemTime()-readyStartTime));
        #endif
    break;
    default:
        break;
    }

    return ret;
}

bool CServiceReady::runRelocalizeServiceStep()
{
    bool ret = false;

    // E_CHECK_SLAM_STEP ret = E_CHECK_SLAM_STEP::CHECK_REROCALIZE;
    tPose robotPose = ServiceData.localiz.getPose();
    tProfile profile = tProfile();
    CRobotKinematics k;
    profile.desAngVel = DEG2RAD(30);
    switch (mStep)
    {
        case RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP:
            {
                ROBOT_CONTROL.slam.setRelocalCompleted(false);
                
                mStep = RELOCALIZE_STEP::RELOCALIZE_SECOND_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_SECOND_STEP:
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCALIZE_STEP::RELOCALIZE_THIRD_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_THIRD_STEP:
            if(!MOTION.isRunning()) //(MOTION.isRunning() == false)
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(358));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CW);
                relocalTime = SYSTEM_TOOL.getSystemTick();
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP;//RELOCALIZE_STEP::RELOCALIZE_FORTH_STEP;
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_FORTH_STEP:
            if(!MOTION.isRunning())
            {
                double targetAng = k.rotation(robotPose, DEG2RAD(90));
                MOTION.startRotation(robotPose,targetAng,tProfile(),E_ROTATE_DIR::CCW);
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP;//RELOCALIZE_STEP::RELOCALIZE_SIXTH_STEP;
            }
            
            break;

        case RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP:
            if(!MOTION.isRunning())//if (MOTION.isRunning() == false)
            {
                if (SYSTEM_TOOL.getSystemTick()-relocalTime >= 5*SEC_1)
                {
                    mStep = RELOCALIZE_STEP::RELOCALIZE_END_STEP;
                }
            }
            break;

        case RELOCALIZE_STEP::RELOCALIZE_END_STEP:
            if(!MOTION.isRunning())//if (SYSTEM_TOOL.getSystemTick()-cleanReLocalizeStarttick >= 2*SEC_1)
            {
                mStep = RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP;
                ROBOT_CONTROL.slam.setRelocalCompleted(true);
                ret = true;
            }
            break;

        default:
            break;
    }

    return ret;
}
