/*********************************************************************************
File name:	  CLidarUnpacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-06
Description:  3irobotics Lidar unpacket
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************* File includes **********************************/
#include "CLidarUnpacket.h"

/******************************* System libs includes *****************************/
#include <string.h>
#include <stdio.h>
#include <iostream>
/********************************** Name space ************************************/
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     CLidarUnpacket
Description:  The constructor of CLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarUnpacket::CLidarUnpacket()
{

}

/***********************************************************************************
Function:     CLidarUnpacket
Description:  The destructor of CLidarUnpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarUnpacket::~CLidarUnpacket()
{

}

/***********************************************************************************
Function:     unpacketLidarScan
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketLidarScan(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    uint16_t length = packet.getParamLength();
    uint8_t *buffer = packet.getParamPtr();
    uint16_t tooth_angle = 0;
    uint16_t distance_number = 0;
    uint16_t distance = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = CLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 2;
    tooth_scan.distance.resize(distance_number);
    //printf("[CLidarUnpacket::unpacketLidarScan] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);

    tooth_scan.lidar_time = packet.getLidarCurrentTime();
    //printf("unpacketLidarScan::LiDAR time: %ld.%ld\n", tooth_scan.lidar_time.tv_sec, tooth_scan.lidar_time.tv_nsec);   

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = CLidarPacket::bufToUByte2(buffer + 2 * i + 2);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketLidarScan2(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    uint16_t length = packet.getParamLength();
    uint8_t *buffer = packet.getParamPtr();
    uint16_t tooth_angle = 0;
    uint16_t distance_number = 0;
    uint16_t distance = 0;
    uint8_t signal = 0;

    // Get tooth angle, unit is 0.01 degree
    tooth_angle = CLidarPacket::bufToUByte2(buffer);
    tooth_scan.angle = float(tooth_angle) / 100.0;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - 2(tooth_angle_bytes)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - 2) / 3;
    tooth_scan.distance.resize(distance_number);
    tooth_scan.signal.resize(distance_number);
    //printf("[CLidarUnpacket::unpacketLidarScan2] distance_number %d! packe length %d\n", distance_number, length);

    // Get distance, unit is 0.25mm
    //packet.printHex();
    tooth_scan.lidar_time = packet.getLidarCurrentTime();
    //printf("unpacketLidarScan2::LiDAR time: %ld.%ld\n", tooth_scan.lidar_time.tv_sec, tooth_scan.lidar_time.tv_nsec);    

    for(size_t i = 0 ; i < distance_number; i++)
    {
        signal = CLidarPacket::bufToUByte((buffer+2) + 3*i);
        distance = CLidarPacket::bufToUByte2((buffer+2) + 3*i + 1);
        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
        tooth_scan.signal[i] = int(signal);
        //printf("[CLidarUnpacket::unpacketLidarScan2]sigal %d! distance %5.2f\n", tooth_scan.signal[i], tooth_scan.distance[i]);
    }
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketNewLidarScanNoSingal
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketNewLidarScanNoSingal(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 0;
    uint16_t length = packet.getParamLength();
    uint8_t *buffer = packet.getParamPtr();
    uint16_t tooth_angle = 0;
    uint8_t  lidar_speed = 0;
    int16_t lidar_offset_angle = 0;
    uint16_t distance_number = 0;
    uint16_t distance = 0;

    // Get tooth angle, unit is 0.01 degree
    lidar_speed = CLidarPacket::bufToUByte(buffer + head_ptr_offset);                   head_ptr_offset += 1;
    lidar_offset_angle = CLidarPacket::bufToByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle = CLidarPacket::bufToUByte2(buffer + head_ptr_offset);                  head_ptr_offset += 2;

    tooth_scan.angle = float(tooth_angle) * 0.01f;
    tooth_scan.offset_valid = false;
    tooth_scan.offset_angle = lidar_offset_angle * 0.01f;
    tooth_scan.lidar_speed = lidar_speed * 0.05f;

    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - head_data_bytes(5)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - head_ptr_offset) / 2;
    tooth_scan.distance.resize(distance_number);
    //printf("[CLidarUnpacket::unpacketNewLidarScanNoSingal] angle %5.2f, distance_number %d !\n",tooth_scan.angle, distance_number);
    // Get distance, unit is 0.25mm

    tooth_scan.lidar_time = packet.getLidarCurrentTime();
    //printf("unpacketNewLidarScanNoSingal::LiDAR time: %ld.%ld\n", tooth_scan.lidar_time.tv_sec, tooth_scan.lidar_time.tv_nsec);   

    for(size_t i = 0 ; i < distance_number; i++)
    {
        distance = CLidarPacket::bufToUByte2(buffer + 2 * i + head_ptr_offset);

        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
    }

    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketLidarScan2
Description:  Lidar unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TToothScan CLidarUnpacket::unpacketNewLidarScanHasSingal(CLidarPacket &packet)
{
    TToothScan tooth_scan;
    int head_ptr_offset = 0;
    uint16_t length = packet.getParamLength();
    uint8_t *buffer = packet.getParamPtr();
    uint16_t tooth_angle = 0;
    uint8_t  lidar_speed = 0;
    int16_t lidar_offset_angle = 0;
    uint16_t distance_number = 0;
    uint16_t distance = 0;
    uint8_t signal = 0;
    uint16_t tooth_angle_start = 0, tooth_angle_end = 0;

    // Get tooth angle, unit is 0.01 degree
    lidar_speed = CLidarPacket::bufToUByte(buffer + head_ptr_offset);                   head_ptr_offset += 1;
    lidar_offset_angle = CLidarPacket::bufToByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle_start = CLidarPacket::bufToUByte2(buffer + head_ptr_offset);            head_ptr_offset += 2;
    tooth_angle_end = CLidarPacket::bufToUByte2(buffer + head_ptr_offset);            	head_ptr_offset += 2;

    tooth_scan.angle = float(tooth_angle_start) * 0.01f;
    tooth_scan.angleEnd = float(tooth_angle_end) * 0.01f;
    tooth_scan.offset_valid = false;
    tooth_scan.offset_angle = lidar_offset_angle * 0.01f;
    tooth_scan.lidar_speed = lidar_speed * 0.05f;


    // Get distance number
    // tooth_angle + distance1 + distance2 + ...
    //    2bytes       2bytes      2bytes  + ...
    //                      length - head_data_bytes(5)
    //  distance_number =   -----------------------------
    //                            2(distance_bytes)
    distance_number = (length - head_ptr_offset) / 3;
    tooth_scan.distance.resize(distance_number);
    tooth_scan.signal.resize(distance_number);
    //CHECK_LIDA
    //printf("[CLidarUnpacket::unpacketNewLidarScanHasSingal] tooth_scan.angle %5.2f tooth_scan.offset_angle %5.2f, tooth_scan.lidar_speed %5.2f, distance_number %d! packe length %d\n",
    //          tooth_scan.angle, tooth_scan.offset_angle, tooth_scan.lidar_speed, distance_number, length);
    // Get distance, unit is 0.25mm
    //packet.printHex();
    
    tooth_scan.lidar_time = packet.getLidarCurrentTime();
    //printf("unpacketNewLidarScanHasSingal(print)::LiDAR time: %ld.%ld\n", tooth_scan.lidar_time.tv_sec, tooth_scan.lidar_time.tv_nsec);    

    // Get distance, unit is 0.25mm
    for(size_t i = 0 ; i < distance_number; i++)
    {
        signal = CLidarPacket::bufToUByte((buffer+ head_ptr_offset) + 3*i);
        distance = CLidarPacket::bufToUByte2((buffer+ head_ptr_offset) + 3*i + 1);
        tooth_scan.distance[i] = float(distance) / 4.0f / 1000.0f;
        tooth_scan.signal[i] = int(signal);
        //printf("[CLidarUnpacket]sigal %d! distance %5.2f\n", tooth_scan.signal[i], tooth_scan.distance[i]);
    }
    return tooth_scan;
}

/***********************************************************************************
Function:     unpacketHealthInfo
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarError CLidarUnpacket::unpacketHealthInfo(CLidarPacket &packet)
{
    uint8_t health_info = *(packet.getParamPtr());
    return TLidarError(health_info);
}

/***********************************************************************************
Function:     unpacketLidarSpeed
Description:  Health info unpacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int CLidarUnpacket::unpacketLidarSpeed(CLidarPacket &packet)
{
    int lidar_speed = *(packet.getParamPtr());
    return lidar_speed;
}
