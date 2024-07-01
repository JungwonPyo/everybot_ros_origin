
#include <iostream>

#include "ros/ros.h"
#include "lidarInterface.h"
#include "rosPublisher.h"
#include "eblog.h"

using namespace std;
using namespace everest::hwdrivers;

CLidarInterface::CLidarInterface()
{
    pSerialConnect = new CSerialConnection();
    pRoboticsLidar = new C3iroboticsLidar();
}

CLidarInterface::~CLidarInterface()
{
    //Close lidar linux driver
    ROBOT_CONTROL.lidar.closeLidarDriver();
 
    delete pRoboticsLidar;
    delete pSerialConnect;
}

bool CLidarInterface::init()
{
    bool nResult;
    nResult = initSerialConnect();
    if (nResult)
    {
        //Open linux driver
        int lidar_fd = ROBOT_CONTROL.lidar.openLidarDriver();
        if (lidar_fd < 0)
        {
            nResult = false;
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Opened fail /dev/ts800_lidar device ");
        }
        else
        {
            //Motor ON
            //나중에 라이더 동작 관련해서 컨셉이 정해지면 IDLE 서비스에서는 OFF
            //타 서비스에서는 ON으로 하자
            //단. 충전 모드일 경우에는 자동으로 전원이 커짐.
            ROBOT_CONTROL.lidar.setEnableLidarMotor(true);
            nResult = true;
        }
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "Opened fail serial port ");
        nResult = false;
    }
    
    return nResult;
}

///dev/ttyS4 of the open
bool CLidarInterface::initSerialConnect()
{
    bool nResult;
    int    opt_com_baudrate = 115200;//230400;
	string opt_com_path = "/dev/ttyS4";
    
    pSerialConnect->setBaud(opt_com_baudrate);
    pSerialConnect->setPort(opt_com_path.c_str());
    if(pSerialConnect->openSimple())
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "[AuxCtrl] Open serail port sucessful!");
        pRoboticsLidar->initilize(pSerialConnect);

        start_scan_time = ros::Time::now();
        seq = 0;
        nResult = true;
    }
    else
    {
        ceblog(LOG_LV_NECESSARY, BOLDRED, "[AuxCtrl] Open serail port "<<opt_com_path.c_str()<<" failed!");
        nResult = false;
    }

    return nResult;
}

void CLidarInterface::proc()
{
    // LIDAR_GRAB_ING(0), LIDAR_GRAB_SUCESS, LIDAR_GRAB_ERRO, LIDAR_GRAB_ELSE
    // timespec
    struct timespec lidar_time;
    TLidarGrabResult result = pRoboticsLidar->getScanData();
    switch(result)
    {
        case LIDAR_GRAB_ING:
        {
            break;
        }
        case LIDAR_GRAB_SUCESS:
        {
            TLidarScan lidar_scan = pRoboticsLidar->getLidarScan();
            size_t lidar_scan_size = lidar_scan.getSize();

            lidar_time = lidar_scan.getLidarTime();
            //printf("CLidarInterface(proc) time: %ld.%ld \n", lidar_time.tv_sec, lidar_time.tv_nsec);  

            std::vector<RslidarDataComplete> send_lidar_scan_data;
            send_lidar_scan_data.resize(lidar_scan_size);
            RslidarDataComplete one_lidar_data;
            //printf("[Main] CLidarInterface::LIDAR_GRAB_ING::lidar_scan_size = %d\n", lidar_scan_size);
            for(size_t i = 0; i < lidar_scan_size; i++)
            {
                one_lidar_data.signal = lidar_scan.signal[i];
                one_lidar_data.angle = lidar_scan.angle[i];
                one_lidar_data.distance = lidar_scan.distance[i];
                send_lidar_scan_data[i] = one_lidar_data;
                //CHECK_LIDAR
                //printf("%d %5.2f %5.2f ", send_lidar_scan_data[i].signal, send_lidar_scan_data[i].distance, send_lidar_scan_data[i].angle);
            }
            //printf("\n");

            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            //end_scan_time = ros::Time::now()
            end_scan_time = convertToRosTime(lidar_time); //current
            scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;
            //scan_duration_sec = (end_scan_time - start_scan_time).toSec();
            //proc(): 130: LIDAR_GRAB_SUCESS : end_scan_time 1713233756.521878976 , start_scan_time 1713233756.349860743 , scan_duration 0.00 , diffSec 0.17
            //if successful, publish lidar scan
            //proc(): 131: end_scan_time 1713237681.994773519 , start_scan_time 1713237681.822782533 , scan_duration 0.00017199 , diffSec 0.17199099
            //proc(): 131: end_scan_time 1713237680.791921629 , start_scan_time 1713237681.994773519 , scan_duration -0.00120285 , diffSec -1.20285189 ????
            //proc(): 131: end_scan_time 1713237682.338780284 , start_scan_time 1713237680.791921629 , scan_duration 0.00154686 , diffSec 1.54685866
            //proc(): 131: end_scan_time 1713237682.510785854 , start_scan_time 1713237682.338780284 , scan_duration 0.00017201 , diffSec 0.17200557
            //proc(): 131: end_scan_time 1713237682.681828038 , start_scan_time 1713237682.510785854 , scan_duration 0.00017104 , diffSec 0.17104218
            //proc(): 131: end_scan_time 1713237682.853832733 , start_scan_time 1713237682.681828038 , scan_duration 0.00017200 , diffSec 0.17200470
            if ( scan_duration > 0)
            {
                int start_node = 0, end_node = 359;
                makeScanMsg(&send_lidar_scan_data[0], lidar_scan_size, end_scan_time, scan_duration, angle_min, angle_max);
                bUpdateScanMsg = true;    
                start_scan_time = end_scan_time; //before
            }
            #if 0
            else
            {
                ceblog(LOG_LV_NECESSARY, RED, "end_scan_time(new) " << PRECISION(8) << end_scan_time 
                        << " , start_scan_time(old) " << start_scan_time 
                        << " , scan_duration " << scan_duration);
            }
            #endif
            break;
        }
        case LIDAR_GRAB_ERRO:
        {
            break;
        }
        case LIDAR_GRAB_ELSE:
        {
            printf("[Main] LIDAR_GRAB_ELSE!\n");
            break;
        }
    }
}

/**
 * @brief get current lidar speed
 * 
 * @return double 
 */
double CLidarInterface::getLidarCurrentSpeed() 
{   
    double current_lidar_speed = 0.0;

    current_lidar_speed = pRoboticsLidar->getLidarCurrentSpeed();

    return current_lidar_speed;
}

float CLidarInterface::getLidarOffsetAngle()
{
    float lidar_offset_angle = 0.0;

    lidar_offset_angle = pRoboticsLidar->getLidarOffsetAngle();

    return lidar_offset_angle;
}

bool CLidarInterface::isUpdateScanMsg()
{
    return bUpdateScanMsg;
}

sensor_msgs::LaserScan CLidarInterface::getScanMsg()
{
    bUpdateScanMsg = false;
    return scan_msg;
}

ros::Time CLidarInterface::convertToRosTime(const timespec& time_spec) 
{
    // Convert seconds and nanoseconds to a single value in seconds
    if (time_spec.tv_sec < 0 ) // || time_spec.tv_sec > std::numeric_limits<uint32_t>::max())
    {
        eblog(LOG_LV_ERROR, "***ERROR(CLidarInterface::convertToRosTime)*** negative time value received.");
        ros::Time cur_rosTime = ros::Time::now();
    }
	else
    {
        ros::Time ros_time(time_spec.tv_sec, time_spec.tv_nsec);
        return ros_time;
    }
}

void CLidarInterface::makeScanMsg(_rslidar_data *nodes, size_t node_count, ros::Time start, double scan_time, float angle_min, float angle_max)
{
    sensor_msgs::LaserScan newScan;
    newScan.header.seq = seq++;
    
    //scan_time_shift & lider & odom time lantency
    ros::Time now = start;

    ros::Duration delay(LATE_OF_LIDAR_SEC); // 0.14~0.18 sec of the past time : from ceva is 145ms -- 160ms
    ros::Time before = now - delay;

    newScan.header.stamp = before;
    newScan.header.frame_id = "laser";
    newScan.angle_min = angle_min;
    newScan.angle_max = angle_max;
    newScan.angle_increment = (newScan.angle_max - newScan.angle_min) / (360.0f - 1.0f);

    newScan.scan_time = scan_time;
    //The problem is: time_increment: 0.0. 
    //This needs to be set to the correct value, otherwise Cartographer's scan unskewing doesn't work correctl
    newScan.time_increment = scan_time / (double)(node_count-1);
    
    //lidar speck : 0.16m～10m(反射率 80%)
    newScan.range_min = 0.16;
    newScan.range_max = 6.0;//8.0

    newScan.ranges.resize(360, std::numeric_limits<float>::infinity());
    newScan.intensities.resize(360, 0.0);

    //Unpack data
    for (size_t i = 0; i < node_count; i++)
    {
        size_t current_angle = floor(nodes[i].angle);
        if(current_angle > 360.0)
        {
            printf("Lidar angle is out of range %d\n", (int)current_angle);
            continue;
        }
        float read_value = (float) nodes[i].distance;
        if (read_value < newScan.range_min || read_value > newScan.range_max)
            newScan.ranges[360- 1- current_angle] = std::numeric_limits<float>::infinity();
        else
            newScan.ranges[360 -1- current_angle] = read_value;

		float intensities = (float) nodes[i].signal;
		newScan.intensities[360 -1- current_angle] = intensities;
	}
    //ROS.updateLidarData(newScan); //scan_time_shift & lider & odom time lantency
    scan_msg = newScan;
}
