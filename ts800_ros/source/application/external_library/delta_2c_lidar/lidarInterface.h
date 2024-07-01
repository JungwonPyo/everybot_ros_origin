#pragma once

#include <ctime>

#include "CDefine.h"

#include "CSerialConnection.h"
#include "C3iroboticsLidar.h"
#include "sensor_msgs/LaserScan.h"

#include "control/control.h"

class CLidarInterface
{
public:
    typedef struct _rslidar_data
    {
        _rslidar_data()
        {
            signal = 0;
            angle = 0.0;
            distance = 0.0;
        }
        uint8_t signal;
        float   angle;
        float   distance;
    }RslidarDataComplete;

private:
    everest::hwdrivers::CSerialConnection* pSerialConnect;
    everest::hwdrivers::C3iroboticsLidar* pRoboticsLidar;
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    ros::Time diff_scan_time;
    double scan_duration;
    double scan_duration_sec;
    bool bUpdateScanMsg;
    sensor_msgs::LaserScan scan_msg;
    uint32_t seq;

public:
    CLidarInterface();
    ~CLidarInterface();
    bool init();
    void proc();

    bool initSerialConnect();
    
    bool isUpdateScanMsg();
    sensor_msgs::LaserScan getScanMsg();
    /* get current lidar*/
    double getLidarCurrentSpeed();

    /* get lidar offset angle */
    float getLidarOffsetAngle();

private:
    ros::Time convertToRosTime(const timespec& time_spec);
    void makeScanMsg(_rslidar_data *nodes, size_t node_count, ros::Time start, double scan_time, float angle_min, float angle_max);
};