#pragma once

#include <iostream>
#include <unordered_map>
#include "ebtypedef.h"
#include "commonStruct.h"
#include "control/control.h"
#include "service.h"
#include "coreData/serviceData.h"
#include "commonStruct.h"

enum class E_TILTING_CHANGE_STEP
{
    INIT_ACTIVEMODE,
    CHECK_ACTIVEMODE,
    INIT_TILT_CHANGE,
    CHECK_TILT_STATE,
    COMPLETE,
};

static std::string enumToString(E_TILTING_CHANGE_STEP value) {
    static const std::unordered_map<E_TILTING_CHANGE_STEP, std::string> enumToStringMap = {
        { E_TILTING_CHANGE_STEP::INIT_ACTIVEMODE, "INIT_ACTIVEMODE," },
        { E_TILTING_CHANGE_STEP::CHECK_ACTIVEMODE, "CHECK_ACTIVEMODE," },
        { E_TILTING_CHANGE_STEP::INIT_TILT_CHANGE, "INIT_TILT_CHANGE," },
        { E_TILTING_CHANGE_STEP::CHECK_TILT_STATE, "CHECK_TILT_STATE," },
        { E_TILTING_CHANGE_STEP::COMPLETE, "COMPLETE," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_POWERMODE_CHANGE_STEP
{
    INIT_POWER_MODE_CHANGE,
    CHECK_POWER_MODE,
    INIT_FAN_CONTROL,
    COMPLETE,
};

static std::string enumToString(E_POWERMODE_CHANGE_STEP value) {
    static const std::unordered_map<E_POWERMODE_CHANGE_STEP, std::string> enumToStringMap = {
        { E_POWERMODE_CHANGE_STEP::INIT_POWER_MODE_CHANGE, "INIT_POWER_MODE_CHANGE," },
        { E_POWERMODE_CHANGE_STEP::CHECK_POWER_MODE, "CHECK_POWER_MODE," },
        { E_POWERMODE_CHANGE_STEP::INIT_FAN_CONTROL, "INIT_FAN_CONTROL," },
        { E_POWERMODE_CHANGE_STEP::COMPLETE, "COMPLETE," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_CEHCK_SENSOR_STEP
{
    INIT_IMU,
    CHECK_IMU_INIT,
    CHECK_IMU_READY,
    INIT_TOF,
    CHECK_TOF_READY,
    COMPLETE,
};

static std::string enumToString(E_CEHCK_SENSOR_STEP value) {
    static const std::unordered_map<E_CEHCK_SENSOR_STEP, std::string> enumToStringMap = {
        { E_CEHCK_SENSOR_STEP::INIT_IMU, "INIT_IMU," },
        { E_CEHCK_SENSOR_STEP::CHECK_IMU_INIT, "CHECK_IMU_INIT," },
        { E_CEHCK_SENSOR_STEP::CHECK_IMU_READY, "CHECK_IMU_READY," },
        { E_CEHCK_SENSOR_STEP::INIT_TOF, "INIT_TOF," },
        { E_CEHCK_SENSOR_STEP::CHECK_TOF_READY, "CHECK_TOF_READY," },
        { E_CEHCK_SENSOR_STEP::COMPLETE, "COMPLETE," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class E_CHECK_SLAM_STEP
{
    START_CHECK_SLAM,
    INIT_SLAM,
    INIT_LOCALIZE,
    CHECK_LOCALIZE_INIT,
    CHECK_LOCALIZE_READY,
    CHECK_SLAM_READY,
    CHECK_REROCALIZE,
    RUN_IMU_CALIBRATION, //RBT+
    CHECK_MAP,
    EXIT_SLAM,
    COMPLETE,
};

static std::string enumToString(E_CHECK_SLAM_STEP value) {
    static const std::unordered_map<E_CHECK_SLAM_STEP, std::string> enumToStringMap = {
        { E_CHECK_SLAM_STEP::START_CHECK_SLAM, "START_CHECK_SLAM," },
        { E_CHECK_SLAM_STEP::INIT_SLAM, "INIT_SLAM," },
        { E_CHECK_SLAM_STEP::INIT_LOCALIZE, "INIT_LOCALIZE," },
        { E_CHECK_SLAM_STEP::CHECK_LOCALIZE_INIT, "CHECK_LOCALIZE_INIT," },
        { E_CHECK_SLAM_STEP::CHECK_LOCALIZE_READY, "CHECK_LOCALIZE_READY," },
        { E_CHECK_SLAM_STEP::CHECK_SLAM_READY, "CHECK_SLAM_READY," },
        { E_CHECK_SLAM_STEP::CHECK_REROCALIZE, "CHECK_REROCALIZE," },
        { E_CHECK_SLAM_STEP::RUN_IMU_CALIBRATION, "RUN_IMU_CALIBRATION," },
        { E_CHECK_SLAM_STEP::CHECK_MAP, "CHECK_MAP," },
        { E_CHECK_SLAM_STEP::EXIT_SLAM, "EXIT_SLAM," },
        { E_CHECK_SLAM_STEP::COMPLETE, "COMPLETE," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

enum class RELOCALIZE_STEP
{
    RELOCALIZE_FIRST_STEP = 0,
    RELOCALIZE_SECOND_STEP,
    RELOCALIZE_THIRD_STEP,
    RELOCALIZE_FORTH_STEP,
    RELOCALIZE_FIFTH_STEP,
    RELOCALIZE_SIXTH_STEP,
    RELOCALIZE_SEVENTH_STEP,
    RELOCALIZE_END_STEP,
};

static std::string enumToString(RELOCALIZE_STEP value) {
    static const std::unordered_map<RELOCALIZE_STEP, std::string> enumToStringMap = {
        { RELOCALIZE_STEP::RELOCALIZE_FIRST_STEP, "RELOCALIZE_FIRST_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_SECOND_STEP, "RELOCALIZE_SECOND_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_THIRD_STEP, "RELOCALIZE_THIRD_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_FORTH_STEP, "RELOCALIZE_FORTH_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_FIFTH_STEP, "RELOCALIZE_FIFTH_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_SEVENTH_STEP, "RELOCALIZE_SEVENTH_STEP," },
        { RELOCALIZE_STEP::RELOCALIZE_END_STEP, "RELOCALIZE_END_STEP," }
    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}



class CServiceReady
{
private:    
    int checkCount;

    double readyStartTime;
    double readyStepTime;
    double relocalTime;
    double imuCalibrationTime;

    bool fanOn;

    E_POWER_MODE powerCtrl;

    E_SERVICE_READY readystate;
    E_POWERMODE_CHANGE_STEP step_power;
    E_CEHCK_SENSOR_STEP step_sensor;
    E_TILTING_CHANGE_STEP step_tilt;
    E_CHECK_SLAM_STEP   step_slam; 
    RELOCALIZE_STEP mStep;

    void initServiceReady();
    void clearServiceReadyStep();
    E_SERVICE_READY checkReadyStart(E_SERVICE_ID service_id);

    E_POWERMODE_CHANGE_STEP getPowerModeChangeStep();
    E_SERVICE_READY stepServiceReadyPowerMode(E_SERVICE_ID service_id);

    E_SERVICE_READY stepServiceReadyLidar(E_SERVICE_ID service_id);

    void setCheckSensorStep(E_CEHCK_SENSOR_STEP _step);
    E_CEHCK_SENSOR_STEP getCheckSensorStep();
    E_SERVICE_READY stepServiceReadyCheckSensor(E_SERVICE_ID service_id);

    void setTiltChangeStep(E_TILTING_CHANGE_STEP _step);
    E_TILTING_CHANGE_STEP getTiltChangeStep();
    E_SERVICE_READY stepServiceReadyTilt(E_SERVICE_ID service_id);

    E_CEHCK_SENSOR_STEP stepInitImu(E_SERVICE_ID service_id,E_IMU_STATUS status);
    E_CEHCK_SENSOR_STEP stepCheckImuInit(E_IMU_STATUS status);
    E_CEHCK_SENSOR_STEP stepCheckImuReay(E_SERVICE_ID service_id, E_IMU_STATUS status);
    E_CEHCK_SENSOR_STEP stepInitTof(tTofData tof);
    E_CEHCK_SENSOR_STEP stepCheckTofReady(tTofData tof);

    void setCheckSlamStep(E_CHECK_SLAM_STEP _step);
    E_CHECK_SLAM_STEP getCheckSlamStep();
    E_SERVICE_READY stepServiceReadyCheckSlam(E_SERVICE_ID service_id);

    E_CHECK_SLAM_STEP stepInitLocalize();
    E_CHECK_SLAM_STEP stepCheckLocalizeInit();
    E_CHECK_SLAM_STEP stepCheckLocalizeReay();
    E_CHECK_SLAM_STEP stepCheckSalmStart(E_SERVICE_ID service_id);
    E_CHECK_SLAM_STEP stepInitSlam();
    E_CHECK_SLAM_STEP stepCheckSlamReady();
    E_CHECK_SLAM_STEP stepCheckMap();
    E_CHECK_SLAM_STEP stepExitSlam();
    E_CHECK_SLAM_STEP stepReLocalize();
    E_CHECK_SLAM_STEP stepIMUCalibration();
public:
    void setPowerModeChangeStep(E_POWERMODE_CHANGE_STEP _step);
    void setServiceReadyState(E_SERVICE_READY ready);
    void setRelocalFirstChangeStep(RELOCALIZE_STEP _step);
    E_SERVICE_READY getServiceReadyState();
    bool runReayServiceStep(E_SERVICE_ID service_id);
    bool runRelocalizeServiceStep();
    CServiceReady();
    ~CServiceReady();
};


