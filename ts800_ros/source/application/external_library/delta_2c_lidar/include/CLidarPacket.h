/*********************************************************************************
File name:	  CLidarPacket.h
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

#ifndef EVEREST_LIDAR_CLIDARPACKET_H
#define EVEREST_LIDAR_CLIDARPACKET_H

/******************************* Current libs includes ****************************/
#include "typedef.h"

/******************************* System libs includes *****************************/
#include <vector>
#include <stddef.h>
#include <fstream>
#include <ctime>

namespace everest
{
	namespace hwdrivers
	{
	    class CLidarPacket
	    {
	        public:
                /* Constructor */
                CLidarPacket();

                /* Destructor */
                ~CLidarPacket() { }

                /* Return true if buffer is empty */
                bool isEmpty() { return m_length == 0? true: false; }

                /* Return buffer size */
                uint16_t getSize() const { return m_length; }

                /* Return true when crc verify good */
                bool isValid() const { return m_valid; }

                /* Push ch in buffer end */
                void pushBack(uint8_t ch);

                void setLidarCurrentTime(struct timespec &time);

                struct timespec getLidarCurrentTime() { return lidar_time;}

                //for_lidar_packet_check
                void setVerifyCheckSum(bool value) { m_valid = value; }

                /* Reset packet */
                void reset();

                /* return true if has write capacity */
				bool hasWriteCapacity(int bytes);

				/* CRC sum calculate */
				uint16_t calcCheckSum(uint8_t *start_bytes, uint16_t num_bytes);
				uint16_t calcCheckSum_Xor(uint8_t *start_bytes, uint16_t num_bytes);
				bool verifyCheckSum(uint8_t ProtoType);

				/* Print hex */
                void printHex();

                /* Get address code */
                uint8_t getPrototypeCode() const { return m_buf[3]; }

                /* Get frame type */
                uint8_t getFrameType() const { return isValid()? m_buf[4]: 0xFF; }

                /* Get command ID */
                uint8_t getCommandID() const { return isValid()? m_buf[5]: 0xFF; }

                /* Get params data ptr */
                uint8_t* getParamPtr() {return isValid()? (&m_buf[8]): NULL; }

                /* Get params data length */
                uint16_t getParamLength();

                /* buf to uint16_t */
                static uint16_t bufToUByte2(uint8_t *src_ptr);

                /* buf to int16_t */
                static int16_t bufToByte2(uint8_t *src_ptr);

                /* buf to uint8_t */
                static uint8_t bufToUByte(uint8_t *src_ptr);

                /* Buffer to data */
                static void bufferToData(void *dest_ptr, void *src_ptr, size_t length);

                struct TParams
                {
                    TParams()
                    {
                        buf_size = 512;
                        // 帧头 帧长度 地址码 帧类型 命令字 参数长度 参数 CRC16
                        //  1    2     1     1     1     2           2
                        //  当帧类型为命令帧时，参数长度以及参数可为0，因此最小帧长度为
                        //  1 +  2  +  1  +  1  +  1  +  0  +  0  +  2 = 8
                        least_packet_len = 8;
                    }

                    uint16_t buf_size;
                    uint16_t least_packet_len;
                };
            private:
                uint8_t* getBufPtr() { return &m_buf[0]; }

            public:
                std::vector<uint8_t>    m_buf;
                struct timespec         lidar_time;
                uint16_t                m_read_length;
                uint16_t                m_length;
                TParams            m_params;
                bool               m_valid;
	    };
	}
}

#endif


