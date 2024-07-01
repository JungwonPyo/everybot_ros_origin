/*********************************************************************************
File name:	  CLidarPacketReceiver.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  lidar packet receiver
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#include <iostream>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <termio.h>
#include <arpa/inet.h>
#include <thread>
#include "eblog.h"
/********************************* File includes **********************************/
#include "CLidarPacketReceiver.h"

/******************************* Current libs includes ****************************/
#include "CDeviceConnection.h"
#include "CCountDown.h"

//for linux lidar driver of the getting packet
#define GET_PARSER_DATA             _IOR('k', 1, int)

/********************************** Name space ************************************/
using namespace everest;
using namespace everest::hwdrivers;

#define LIDAR_PACEKT_HEADER1 0xAA
#define LIDAR_PACEKT_HEADER2 0x00

/***********************************************************************************
Function:     CLidarPacketReceiver
Description:  The constructor of CLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::CLidarPacketReceiver() : pTempDeviceData(nullptr)
{
    m_device_conn = NULL;
    m_log_when_receive_time_over = false;

    //lidar_packet of the 256 byte
    try {
        pTempDeviceData = new unsigned char[256];
        std::memset(pTempDeviceData, 0, 256);
        std::memset(&lidar_time, 0, sizeof(lidar_time));
    } catch (const std::bad_alloc& e) {
        // Handle memory allocation failure
        std::cerr << "Memory allocation failed: " << e.what() << std::endl;
        // Consider exiting or other error handling
    }

    reset();
    m_counter = 0;
}

/***********************************************************************************
Function:     CLidarPacketReceiver
Description:  The destructor of CLidarPacketReceiver
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::~CLidarPacketReceiver()
{
    if(m_save_fp)
    {
        m_save_fp.close();
    }

    if (pTempDeviceData) 
    {
        delete[] pTempDeviceData;
        pTempDeviceData = nullptr;
    }
}

/***********************************************************************************
Function:     receivePacket

Description:  Receive lidar packet, if return true, it means receive a valid packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacketReceiver::receivePacket(CLidarPacket *packet)
{
	/* Judge whether serial is connecting */
	if ( packet == NULL 
        || m_device_conn == NULL 
        || m_device_conn->getStatus() != CDeviceConnection::STATUS_OPEN
        || ROBOT_CONTROL.lidar.getLidarDriver() < 0 )
	{
        ceblog(LOG_LV_NECESSARY, BOLDRED, "[CLidarPacketReceiver] receivePacket: connection not open! ");
		//printf("[CLidarPacketReceiver] receivePacket: connection not open!\n", ROBOT_CONTROL.lidar.getLidarDriver());
		return false;
	}

    /* Read packet */
    m_count_down.setTime((double)m_params.packet_max_time_ms);
    m_parser_completed = false;

	while(1)
	{
        //for_linux_uart_driver
        TPacketResult packet_result = PACKET_ING;

        /* Read packet */
        int ret = ioctl(ROBOT_CONTROL.lidar.getLidarDriver(), GET_PARSER_DATA, pTempDeviceData); //&pTempDeviceData[0]);
        if ( ret < 0)
        {
            //wait data from kernel driver
            //커널에서 라이더 데이터 쓰레드가 주기가 몇 us 라고 함.=> 너무 길게 주면 데이터를 놓치는 현상이
            //발생한다...
            //sleep을 주지 않으면 커널에서 올라오는 data가 이전 데이터로 점프를 한다.ㅠㅠ
            //sleep을 주면 커널에서 올라오는 data가 360도 모두 올라오지 않는 경우가 있다..ㅠㅠ
            packet_result = PACKET_ING;
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            //continue;
        }
        else
        {
            unsigned short nLength = ( (pTempDeviceData[1] << 8) | pTempDeviceData[2]) + 2;
            unsigned int duty = ( (pTempDeviceData[240] << 24) | (pTempDeviceData[241] << 16) | (pTempDeviceData[242] << 8) | pTempDeviceData[243]);
            unsigned int apDiff = ( (pTempDeviceData[244] << 24) | (pTempDeviceData[245] << 16) | (pTempDeviceData[246] << 8) | pTempDeviceData[247]);
            lidar_time.tv_sec = ( (pTempDeviceData[248] << 24) | (pTempDeviceData[249] << 16) | (pTempDeviceData[250] << 8) | pTempDeviceData[251]);
            lidar_time.tv_nsec = ( (pTempDeviceData[252] << 24) | (pTempDeviceData[253] << 16) | (pTempDeviceData[254] << 8) | pTempDeviceData[255]);
            //printf("LiDAR duty: %dHz, ap_diff: %d, time: %ld.%ld Length: %d, data: ", duty, apDiff, lidar_time.tv_sec, lidar_time.tv_nsec, nLength);    
            //printf("LiDAR time: %ld.%ld\n", lidar_time.tv_sec, lidar_time.tv_nsec);    

            //for_packet of the all buffer clear
            packet->reset();
            m_count_down.setTime(m_params.packet_wait_time_ms);
            packet->setVerifyCheckSum(true);
            
            //insert of the lidar packet with time stamp
            packet->setLidarCurrentTime(lidar_time);
            
            for(int i=0;i<nLength;i++)
            {
                packet->pushBack(pTempDeviceData[i]); // unsigned char
            }

            m_parser_completed = true;
            packet_result = PACKET_SUCCESS;

            //after packet is pushed , reset 
            reset();
            return true;
        }
    }

	printf("[CLidarPacketReceiver] It should not come to here!\n");
	return false;
}

/***********************************************************************************
Function:     readPacket
Description:  Read packet, if it return ture, it means read complete packet or enter
              erro state
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::readPacket(CLidarPacket *packet, uint8_t ch)
{
    TPacketResult packet_result = PACKET_ING;
    switch(m_state)
    {
        case STATE_HEADER1: packet_result = processStateHeader1(packet, ch); break;
        case STATE_HEADER2: packet_result = processStateHeader2(packet, ch); break;
        case STATE_LENGHT: packet_result =  processStateLength(packet, ch); break;
        case STATE_ACQUIRE_DATA: packet_result = processStateAcquireData(packet, ch); break;
        default:
            printf("[CLidarPacketReceiver] Enter erro state %d!\n", m_state);
        break;
    }
    return packet_result;
}

/***********************************************************************************
Function:     processStateHeader1
Description:  Proces state header1
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateHeader1(CLidarPacket *packet, uint8_t ch)
{
    if(ch == LIDAR_PACEKT_HEADER1)
    {
        packet->reset();
        packet->pushBack(ch);
        m_state = STATE_HEADER2;
        m_count_down.setTime(m_params.packet_wait_time_ms);
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateHeader2
Description:  Proces state header2
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateHeader2(CLidarPacket *packet, uint8_t ch)
{
    if(ch == LIDAR_PACEKT_HEADER2)
    {
        packet->pushBack(ch);
        m_state = STATE_LENGHT;
    }
    else
    {
        reset();
        printf("[CLidarPacketReceiver] Find erro header2 0x%x!\n", ch);
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateLength
Description:  Process state length
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateLength(CLidarPacket *packet, uint8_t ch)
{
#if 1
    /* Limit packet length */
    if(ch < 6 || ch > 250)
    {
        reset();
        printf("[CLidarPacketReceiver] Find erro length is 0x%x!\n", (ch));
        return PACKET_ING;
    }
#endif
    packet->pushBack(ch);

    // Add 2bytes for receive CRC16, sub 3 bytes for header(1bytes) and length(2bytes)
    m_packet_length = (int)ch + 2 - 3;

    m_state = STATE_ACQUIRE_DATA;

    return PACKET_ING;
}

/***********************************************************************************
Function:     processStateAcquireData
Description:  Process state acquire data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacketReceiver::TPacketResult CLidarPacketReceiver::processStateAcquireData(CLidarPacket *packet, uint8_t ch)
{
    m_actual_count++;
    packet->pushBack(ch);
    if(m_actual_count == m_packet_length)
    {
        reset();
		
        if(packet->verifyCheckSum(packet->getPrototypeCode()))
        {
            return PACKET_SUCCESS;
        }
        else
        {
            printf("[CLidarPacketReceiver] CRC verify wrong!\n");
//            packet->printHex();
            return PACKET_FAILED;
        }
    }
    return PACKET_ING;
}

/***********************************************************************************
Function:     reset
Description:  Reset
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacketReceiver::reset()
{
    m_state = STATE_HEADER1;
    m_actual_count = 0;
    m_packet_length = 0;
}
