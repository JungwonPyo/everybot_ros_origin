/*********************************************************************************
File name:	  C3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#include "eblog.h"
/********************************* File includes **********************************/
#include "C3iroboticsLidar.h"

/******************************* Current libs includes ****************************/
#include <iostream>

/********************************** Name space ************************************/
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     C3iroboticsLidar
Description:  The constructor of C3iroboticsLidar
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
C3iroboticsLidar::C3iroboticsLidar()
{
    m_device_connect = NULL;
    m_data_with_signal = true;
    m_receive_lidar_speed = false;
    m_current_lidar_speed = -1.0;
    m_lidar_offset_angle = 0.0;

    resetScanGrab();
}

/***********************************************************************************
Function:     C3iroboticsLidar
Description:  The destructor of C3iroboticsLidar
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
C3iroboticsLidar::~C3iroboticsLidar()
{

}

/***********************************************************************************
Function:     initilize
Description:  Set device connect
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::initilize(CDeviceConnection *device_connect)
{
    if(device_connect == NULL || device_connect->getStatus() != CDeviceConnection::STATUS_OPEN)
    {
        printf("[C3iroboticsLidar] Init failed Can not open device connect!\n");
        return false;
    }
    else
    {
        printf("[C3iroboticsLidar] Init device conenct sucessful!\n");
        m_receiver.setDeviceConnection(device_connect);
        return true;
    }
}

/***********************************************************************************
Function:     getScanData
Description:   Get scan data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::getScanData()
{
    TLidarGrabResult grab_result;
    while(1)
    {
        if(m_remainder_flag)
        {
            printf("[C3iroboticsLidar] Handle remainer scan!\n");
            resetScanGrab();
            combineScan(m_remainder_tooth_scan);
        }
        else if(m_receiver.receivePacket(&m_packet))
        {
            grab_result = analysisPacket(m_packet);
            break;
        }
    }
    return grab_result;
}

/***********************************************************************************
Function:     analysisPacket
Description:  Analysis packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisPacket(CLidarPacket &lidar_packet)
{
    TLidarCommandID command_id = TLidarCommandID(lidar_packet.getCommandID());
    //lidar_packet.printHex();
    switch(command_id)
    {
        case I3LIDAR_DISTANCE:  //0xA9 <- buf[5]
            {
                return analysisToothScan(lidar_packet);
            }
        case I3LIDAR_HEALTH:  //0xAB <- buf[5]
            {
                return analysisHealthInfo(lidar_packet);
            }
        case I3LIDAR_LIDAR_SPEED: ////0xAE <- buf[5]
            {
                return analysisLidarSpeed(lidar_packet);
            }
        case I3LIDAR_NEW_DISTANCE : //0xAD <- buf[5]
            {
                //printf("[C3iroboticsLidar] NEW_DISTANCE command id %d!\n", command_id);
                //printf("analysisPacket time: %ld.%ld \n", time.tv_sec, time.tv_nsec);    
                return analysisNewToothScan(lidar_packet);
            }        
        default: 
            {
                //printf("[C3iroboticsLidar] Special command id %d!\n", command_id);
                return LIDAR_GRAB_ELSE;
            }
    }
}

/***********************************************************************************
Function:     analysisToothScan
Description:  Analysis tooth scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisToothScan(CLidarPacket &lidar_packet)
{
    TToothScan tooth_scan;
    if(m_data_with_signal)
    {
        tooth_scan = CLidarUnpacket::unpacketLidarScan2(lidar_packet);
    }
    else
    {
        tooth_scan = CLidarUnpacket::unpacketLidarScan(lidar_packet);
    }
    return combineScan(tooth_scan);
}

/***********************************************************************************
Function:     analysisToothScan
Description:  Analysis tooth scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisNewToothScan(CLidarPacket &lidar_packet)
{
    TToothScan tooth_scan;
    if(m_data_with_signal)
    {
        tooth_scan = CLidarUnpacket::unpacketNewLidarScanHasSingal(lidar_packet);
    }
    else
    {
        tooth_scan = CLidarUnpacket::unpacketNewLidarScanNoSingal(lidar_packet);
    }

    m_current_lidar_speed = tooth_scan.lidar_speed;
    m_receive_lidar_speed = true;

    return combineScan(tooth_scan);
}

/***********************************************************************************
Function:     combineScan
Description:  Combine scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::combineScan(TToothScan &tooth_scan)
{
    //GRAB_SCAN_FIRST = 0,
    //GRAB_SCAN_ELSE_DATA
    //printf("[C3iroboticsLidar::combineScan] m_grab_scan_state %d, m_grab_scan_count %d!\n", m_grab_scan_state, m_grab_scan_count);
    //struct timespec lidar_time;
    //m_lidar_scan.setLidarTime(time);
    switch(m_grab_scan_state)
    {
        case GRAB_SCAN_FIRST:
        {
            m_data_count_down.setTime(m_params.scan_time_out_ms);

            /* First scan come */
            if(isFirstScan(tooth_scan))
            {
                resetScanGrab();
                m_lidar_scan.setLidarTime(tooth_scan.lidar_time);
                grabFirstScan(tooth_scan);
            }
            else
            {
                //ceblog(LOG_LV_NECESSARY, GREEN, "GRAB_SCAN_FIRST tooth scan angle " << tooth_scan.angle); 
                printf("[C3iroboticsLidar] GRAB_SCAN_FIRST tooth scan angle %5.2f!\n",
                          tooth_scan.angle);
            }
            return LIDAR_GRAB_ING;
        }
        case GRAB_SCAN_ELSE_DATA:
        {
            //m_lidar_scan.setLidarTime(time);
            //printf("combineScan time(1-1): %ld.%ld \n", time.tv_sec, time.tv_nsec);
            if(m_data_count_down.isEnd())
            {
                printf("[C3iroboticsLidar] grab scan is time out %d ms! Reset grab scan state, current is %5.2f, last is %5.2f! \n",
                          (int)m_params.scan_time_out_ms, tooth_scan.angle, m_last_scan_angle);
                m_lidar_scan.setLidarTime(tooth_scan.lidar_time);
                m_grab_scan_state = GRAB_SCAN_FIRST;
                return LIDAR_GRAB_ING;
            }
            m_data_count_down.setTime(m_params.scan_time_out_ms);
            //printf("[C3iroboticsLidar::combineScan] tooth_scan.angle %5.2f, m_last_scan_angle %5.2f!\n",
            //          tooth_scan.angle, m_last_scan_angle);
            /* Handle angle suddenly reduces */
            if(tooth_scan.angle < m_last_scan_angle)
            {
                printf("[C3iroboticsLidar::combineScan] may receive next scan, current %5.2f, last %5.2f!\n",
                          tooth_scan.angle, m_last_scan_angle);
                if(isFirstScan(tooth_scan))
                {
                    m_lidar_scan.setLidarTime(tooth_scan.lidar_time);
                    m_remainder_flag = true;
                    m_remainder_tooth_scan = tooth_scan;
                }
                return LIDAR_GRAB_SUCESS;
            }

            /* Insert tooth scan in scan */
            TLidarScan part_scan_data;
            toothScan2LidarScan(tooth_scan, part_scan_data);
            m_lidar_scan.insert(part_scan_data);
            m_lidar_scan.setLidarTime(tooth_scan.lidar_time);
            m_grab_scan_count++;
            m_last_scan_angle = tooth_scan.angle;

            /* Judge whether finish grab one compelte scan */
            if(m_grab_scan_count == m_params.tooth_number)
            {
                m_grab_scan_state = GRAB_SCAN_FIRST;
                return LIDAR_GRAB_SUCESS;
            }
            else
            {
                return LIDAR_GRAB_ING;
            }
        }
        default:
            printf("[C3iroboticsLidar::combineScan] Uknow grab scan data state %d!\n",
                      m_grab_scan_state);
        break;
    }
    printf("[C3iroboticsLidar] combineScan should not come to here!\n");
    return LIDAR_GRAB_ERRO;
}

/***********************************************************************************
Function:     grabFirstScan
Description:  Grab first scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::grabFirstScan(TToothScan &tooth_scan)
{
//    printf("[C3iroboticsLidar] Enter first grab scan data 1 !\n");

    // Change grab state
    m_grab_scan_state = GRAB_SCAN_ELSE_DATA;
    m_grab_scan_count = 1;
    m_last_scan_angle = tooth_scan.angle;


    // Insert scan data
    TLidarScan part_scan_data;

    toothScan2LidarScan(tooth_scan, part_scan_data);

    m_lidar_scan.insert(part_scan_data);

    return LIDAR_GRAB_ING;
}

/***********************************************************************************
Function:     isFirstScan
Description:  Return true if tooth scan is first
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::isFirstScan(TToothScan &tooth_scan)
{
    if(tooth_scan.angle >= 0 && tooth_scan.angleEnd <= 22.5)
	return true;
    else
	return false;
    //return tooth_scan.angle < 0.0001? true: false;
}

/***********************************************************************************
Function:     toothScan2LidarScan
Description:  Change tooth scan to lidar scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::toothScan2LidarScan(TToothScan &tooth_scan, TLidarScan &lidar_scan)
{
    size_t size = tooth_scan.getSize();
    float  first_angle = tooth_scan.getAngle();
    float  last_angle = tooth_scan.getAngleEnd();
    bool has_signal = !tooth_scan.signal.empty();

//    printf()
    /*
                           last_angle - first_angle
        angle_step =  ---------------------------------
                              tooth_scan_size
    */
    float  angle_step = float(last_angle - first_angle) / (float(size));
    lidar_scan.angle.resize(size);
    lidar_scan.distance.resize(size);

    for(size_t i = 0; i < size; i++)
    {
        lidar_scan.angle[i] = first_angle + i * angle_step;
        //std::cout << " angle = " << lidar_scan.angle[i]  << " angle step = " << angle_step << std::endl;
        //std::cout << " offset_angle = " << tooth_scan.offset_angle << std::endl;
        m_lidar_offset_angle = tooth_scan.offset_angle;
        lidar_scan.distance[i] = tooth_scan.distance[i];
    }
    if(has_signal)
    {
       lidar_scan.signal.resize(size);
       for(size_t i = 0; i < size; i++)
        lidar_scan.signal[i] = tooth_scan.signal[i];
    }
    else
    {
        lidar_scan.signal.clear();
    }
}

/***********************************************************************************
Function:     analysisHealthInfo
Description:  Analysis health info
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisHealthInfo(CLidarPacket &lidar_packet)
{
    TLidarError lidar_error = CLidarUnpacket::unpacketHealthInfo(lidar_packet);
    printf("[C3iroboticsLidar] Lidar error is %d!\n", (int)lidar_error );
    return LIDAR_GRAB_ERRO;
}

/***********************************************************************************
Function:     analysisLidarSpeed
Description:  Analysis health info
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisLidarSpeed(CLidarPacket &lidar_packet)
{
    double lidar_erro_speed = CLidarUnpacket::unpacketLidarSpeed(lidar_packet) * 0.05f;
    printf("[C3iroboticsLidar] Lidar error speed is %5.5f!\n", lidar_erro_speed);

    m_current_lidar_speed = lidar_erro_speed;
    m_receive_lidar_speed = true;

    return LIDAR_GRAB_ERRO;
}

/***********************************************************************************
Function:     resetScanGrab
Description:  Reset scan grab
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::resetScanGrab()
{
    m_lidar_scan.clear();
    m_grab_scan_state = GRAB_SCAN_FIRST;
    m_grab_scan_count = 0;
    m_last_scan_angle = 0.0;
    m_remainder_flag = false;
}

