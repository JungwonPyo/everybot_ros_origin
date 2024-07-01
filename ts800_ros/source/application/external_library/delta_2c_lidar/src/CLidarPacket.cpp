/*********************************************************************************
File name:	  CLidarPacket.cpp
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-04
Description:  lidar packet
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

/********************************** File includes *********************************/
#include "CLidarPacket.h"

/********************************** Current libs includes *************************/
#include "C3iroboticsLidarProtocol.h"

/********************************** System includes *******************************/
#include <string.h>
#include <stdio.h>
#include <iostream>

/********************************** Name space ************************************/
using namespace std;
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     CLidarPacket
Description:  The constructor of CLidarPacket
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CLidarPacket::CLidarPacket()
{
	reset();
}

/***********************************************************************************
Function:     reset
Description:  Reset packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::reset()
{
    m_buf.resize(m_params.buf_size);
    m_read_length = 0;
    m_length = 0;
    m_valid = false;
	
	//lidar of the time reset
	memset(&lidar_time, 0, sizeof(lidar_time));
}

/***********************************************************************************
Function:     hasWriteCapacity
Description:  Reset packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacket::hasWriteCapacity(int bytes)
{
	if (bytes < 0)
	{
	    printf("[CLidarPacket] You should input bytes %d less than 0!\n", bytes);
		return false;
	}

	// Make sure there's enough room in the packet
	#if 0 ////for_lidar_packet_check
	if ((m_length + bytes) <= m_params.buf_size)
	{
        return true;
	}
	else
	{
		return false;
	}
	#endif
	return true;
}

/***********************************************************************************
Function:     pushBack
Description:  pushBack
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::pushBack(uint8_t ch)
{
#if 0 ////for_lidar_packet_check
	if(!hasWriteCapacity(1))
	{
		return ;
	}
#endif
	m_buf[m_length] = ch;
	m_length++;
}


void CLidarPacket::setLidarCurrentTime(struct timespec &time)
{ 
	lidar_time.tv_sec = time.tv_sec;
	lidar_time.tv_nsec = time.tv_nsec;
}

/***********************************************************************************
Function:     calcCheckSum
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
uint16_t CLidarPacket::calcCheckSum(uint8_t *start_bytes, uint16_t num_bytes)
{
	uint8_t  uchCRCHi = 0xFF;                        // CRC高字节的初始化
	uint8_t  uchCRCLo = 0xFF;                        // CRC低字节的初始化
	uint16_t uIndex = 0;                             // CRC查找表的指针
	while (num_bytes--)
	{
		uIndex = uchCRCLo ^ *start_bytes++;     // 计算CRC
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}
/***********************************************************************************
Function:     calcCheckSum_Xor
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
uint16_t CLidarPacket::calcCheckSum_Xor(uint8_t *start_bytes, uint16_t num_bytes)
{
	uint16_t checksum = 0;                             // Checksum
	while (num_bytes--)
	{
		checksum += *start_bytes++;     // 计算Checksum
	}
	return checksum;
}

/***********************************************************************************
Function:     verifyCheckSum
Description:  Copies the given packets buffer into the buffer of this packet, also
			  sets this length and readlength to what the given packet has
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CLidarPacket::verifyCheckSum(uint8_t ProtoType)
{
    if (m_length < m_params.least_packet_len)
    {
        m_valid = false;
        return false;
    }

    uint8_t c2 = m_buf[m_length - 2];
    uint8_t c1 = m_buf[m_length - 1];
    uint16_t chksum = (c2 << 8) | c1 ;
    uint16_t caculate;

	if(ProtoType < 1)
		caculate = calcCheckSum(&m_buf[0], m_length - 2);
	else
		caculate = calcCheckSum_Xor(&m_buf[0], m_length - 2);

#if 0
    printf("[CLidarPacket] m_length %d CRC 0x%X 0x%X, receive CRC %d, caculate CRC %d!\n",
              m_length, c2, c1, chksum, caculate);
#endif
    if (chksum == caculate)
	{
	    m_valid = true;
        return true;
    }
    else
	{
	    m_valid = false;
		return false;
	}
}

/***********************************************************************************
Function:     getParamLength
Description:  Get params data length
Output:       None
Return:       None
Others:       None
***********************************************************************************/
uint16_t CLidarPacket::getParamLength()
{
    if(isValid())
    {
        return bufToUByte2(&m_buf[6]);
    }
    else
    {
        return 0;
    }
}

/***********************************************************************************
Function:     bufToUByte2
Description:  buf to uint16_t
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
uint16_t CLidarPacket::bufToUByte2(uint8_t *src_ptr)
{
    uint16_t data = (src_ptr[0] << 8) | ((uint8_t)src_ptr[1]);
    return data;
}

/***********************************************************************************
Function:     bufToUByte2
Description:  buf to uint16_t
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int16_t CLidarPacket::bufToByte2(uint8_t *src_ptr)
{
    int16_t data = (src_ptr[0] << 8) | ((int8_t)src_ptr[1]);
    return data;
}

/***********************************************************************************
Function:     bufToUByte
Description:  buf to uint8_t
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
uint8_t CLidarPacket::bufToUByte(uint8_t *src_ptr)
{
    uint8_t data = src_ptr[0];
    return data;
}

/***********************************************************************************
Function:     printHex
Description:  Buffer to data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::bufferToData(void *dest_ptr, void *src_ptr, size_t length)
{
    printf("[CLidarPacket] It has not realize now!\n");
}

/***********************************************************************************
Function:     printHex
Description:  printf packet buf as hex format
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CLidarPacket::printHex()
{
    printf("\t[CRobotPacket] length %d, read_length %d!\n",
              m_length, m_read_length);

    for(size_t i = 0; i < m_length; i++)
    {
		//%02X  0x%x!
        //printf("buf[%d] = 0x%x!\n", (int)i, (m_buf[i]));
		printf("%02X ", (m_buf[i]));
    }
}
