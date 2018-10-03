/*
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@ncsu.edu>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "E-TC32.h"

static int nBits(uint32_t num)
{
  int count = 0;
  int i;
  
  // counts the number of bits in a number
  for (i = 0; i < 32; i++) {
    if ((num & 0x1) == 0x1) count++;
    num = (num >> 0x1);
  }
  return count;
}

/*********************************************
 *        Digital I/O  Commands              *
 *********************************************/
bool DIn_E_TC32(DeviceInfo_TC32 *device_info, uint8_t value[2])
{
  /* This command reads the current state of the DIO pins.  A 0 in a
     bit position indicates the correspoing pin is reading a low
     state, and a 1 indicates a high state.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DIN;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  value[0] = replyBuffer[MSG_INDEX_DATA];
  	  value[1] = replyBuffer[MSG_INDEX_DATA+1];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DIn_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutR_E_TC32(DeviceInfo_TC32 *device_info, uint32_t value[2])
{
  /* This command reads the DIO output latch value.  The factory power
     on default is all 1 (pins are floating).  A 0 in a bit position indicates the
     corresponding pin driver is low, a 1 indicates it is floating.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(&value[0], &replyBuffer[MSG_INDEX_DATA], 4);
	  memcpy(&value[1], &replyBuffer[MSG_INDEX_DATA+4], 4);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DOutR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, uint32_t value)
{
  /* This command writes the DIO latch value.  The factory power on
     default is all 1 (pins are floating.)  Writing a 0 to a bit will
     set the corresponding pin driver low, writing a 1 allows it to
     float.  If pin(s) are configured as alarm outputs this command
     does not affect their value.

     index: bit 0   1 = Base Unit
            bit 1:  1 = EXP
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 5;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_W;
  buffer[MSG_INDEX_DATA]           = index;
  memcpy(&buffer[MSG_INDEX_DATA+1], &value, 4);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DOutW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *      Temperature Input  Commands          *
 *********************************************/

bool Tin_E_TC32(DeviceInfo_TC32 *device_info, uint8_t channel, uint8_t units, uint8_t wait, float *value)
{
  /* This command reads the value of a single thermocouple channel.  There are some
     special return values:

     -777.0: Input voltage outside valid common-mode voltage range
     -888.0: Open thermocouple detected
     -999.0: Channel disabled (also returned if EXP channels are specified but 
             no EXP is connected) 

    channel: the channel to read (0-63)
    units:   0 - Celsius, linearized by TC type
             1 - Voltage
             2 - ADC code (uncalibrated)
    wait:    0 - return current value, 1 - wait for new reading before returning
  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_TIN;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_DATA+1]         = units;
  buffer[MSG_INDEX_DATA+2]         = wait;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 4;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(value, &replyBuffer[MSG_INDEX_DATA], 4);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Tin_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CJC_E_TC32(DeviceInfo_TC32 *device_info, uint8_t channel, float *value)
{
  /* This command reads the most recent value of a single CJC sensor in Celsius.  The
     value -999.0 will be returned if an EXP sensor is specified bu no EXP is connected.
     special return values:

    channel: the channel to read (0-63)

  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CJC;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 4;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(value, &replyBuffer[MSG_INDEX_DATA], 4);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CJC_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinMultiple_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* Ths command reads the value of multiple thermocouple channels.
     The channels to be read are passed as a bitmap when calling the
     command.  The data will be returned in the order low channel
     number to high channel number.  The number of floating point
     values returned will be equal to the number of channels specified
     (max 64).  The special return values listed in the TIn command
     also apply to this command.

     wait:             0 - return current value
                       1 - wait for new reading before returning
     units:            0 - Celsius
                       1 - Voltage
	               2 - ADC code (uncalibraded)
    channel_mask_base: the channel bitmask for the base unit (channel 0-31)
    channel_mask_exp:  the channel bitmask for the EXP unit (channel 32-63)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[272];
  unsigned char replyBuffer[272];
  bool result = false;
  int length;
  int dataCount = 10;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_TIN_MULTIPLE;
  buffer[MSG_INDEX_DATA]           = device_info->wait;
  buffer[MSG_INDEX_DATA+1]         = device_info->units;
  memcpy(&buffer[MSG_INDEX_DATA+2], &device_info->channel_mask[0], 4);
  memcpy(&buffer[MSG_INDEX_DATA+6], &device_info->channel_mask[1], 4);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(float)*(nBits(device_info->channel_mask[0]) + nBits(device_info->channel_mask[1]));
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->Tin_values, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinMultiple_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CJCMultiple_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the value of multiple CJC sensors.  The
     sensors to be read are passed as a bitmap when calling the
     command.  The data will be returned in the order low channel
     number to high channel.  The number of floating point values
     returned will be equal to the number of channels specified (max
     64).  The CJC values only update once per second so there is no
     need to call this faster.  The value -9999.0 will be returned if
     an EXO sensor is specified but no EXP is connected.

     cjc_mask_base: the channel bitmask for the base unit (channel 0-31)
     cjc_mask_exp:  the channel bitmask for the EXP unit (channel 32-63)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[272];
  unsigned char replyBuffer[272];
  bool result = false;
  int length;
  int dataCount = 8;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CJC_MULTIPLE;
  memcpy(&buffer[MSG_INDEX_DATA],   &device_info->cjc_mask[0], 4);
  memcpy(&buffer[MSG_INDEX_DATA+4], &device_info->cjc_mask[1], 4);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(float)*(nBits(device_info->cjc_mask[0]) + nBits(device_info->cjc_mask[1]));
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->CJC_values, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CJCMultiple_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinConfigR_E_TC32(DeviceInfo_TC32 *device_info)
{
    /* This command reads the thermocouple channel configurations.  Each
       configuration is a uint8_t with the following possible values:

     0 - channel disabled
     1 - TC type J
     2 - TC type K
     3 - TC type T
     4 - TC type E
     5 - TC type R
     6 - TC type S
     7 - TC type B
     8 - TC type N
  */

  int sock = device_info->device.sock;
  unsigned char buffer[128];
  unsigned char replyBuffer[128];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_TIN_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 64;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->config_values, &replyBuffer[MSG_INDEX_DATA], 64);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinConfigR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinConfigW_E_TC32(DeviceInfo_TC32 *device_info)
{
    /* This command writes the thermocouple channel
       configurations. The micro stores these values in EEPROM and
       loads them from EEPROM at power on.  Each configuration is a
       uint8_t with the following possible values:

     0 - channel disabled
     1 - TC type J
     2 - TC type K
     3 - TC type T
     4 - TC type E
     5 - TC type R
     6 - TC type S
     7 - TC type B
     8 - TC type N
  */

  int sock = device_info->device.sock;
  unsigned char buffer[128];
  unsigned char replyBuffer[128];
  bool result = false;
  int length;
  int dataCount = 64;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_TIN_CONFIG_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], device_info->config_values, 64);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinConfigW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinStatus_E_TC32(DeviceInfo_TC32 *device_info)
{
    /* This command reads the status of the temperature readings.  If
       a bit is set the corresponding channel has a new reading that
       has not been read with either the Tin or TinMultiple command.
    */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_TIN_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->Tin_status, &replyBuffer[MSG_INDEX_DATA], 8);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinStatus_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool OTDStatus_E_TC32(DeviceInfo_TC32 *device_info)
{
    /* This command reads the status of the open thermocouple
       detection.  If a bit is set an open thermocouple is currently
       detected on the corresponding channel.  The LED on the front
       of the device is on if any bits are set in this value.
    */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_OTD_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->OTD_status, &replyBuffer[MSG_INDEX_DATA], 8);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in OTDStatus_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureConfigR_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the measurement configuration. 

     bit 0: 0 - OTD enable,    1 - OTD disabled
     bit 1: 0 - notch @ 60 Hz, 1 - notch @ 50 Hz
     bit 2: 0 - factory coef.  1 - field coef.
*/

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_MEASURE_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->config_measure, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureConfigR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureConfigW_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the measurement configuration. 

     bit 0: 0 - OTD enable,    1 - OTD disabled
     bit 1: 0 - notch @ 60 Hz, 1 - notch @ 50 Hz
     bit 2: 0 - factory coef.  1 - field coef.
*/

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_MEASURE_CONFIG_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = device_info->config_measure[0];
  buffer[MSG_INDEX_DATA+1]         = device_info->config_measure[1];
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureConfigW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureModeR_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the measurement mode.  The power on default is mode 0 (normal mode) 
       mode_base: the measurement mode for the base unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 0 and 16 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 16.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 0 and 16.
       mode_exp: the measurement mode for the EXP unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 32 and 48 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 32 and 48.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 32 and 48.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_MEASURE_MODE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->mode_measure, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureModeR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureModeW_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command writes the measurement mode.  The power on default is mode 0 (normal mode) 
       mode_base: the measurement mode for the base unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 0 and 16 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 16.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 0 and 16.
       mode_exp: the measurement mode for the EXP unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 32 and 48 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 32 and 48.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 32 and 48.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_MEASURE_MODE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = device_info->mode_measure[0];
  buffer[MSG_INDEX_DATA+1]         = device_info->mode_measure[1];
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureModeW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *         Alarm  Commands               *
 *****************************************/

bool AlarmConfigR_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the temperature alarm configurations. There
     are configuration values and two threshold values for each of the
     32/64 thermocouple channels.

    alarm_config: the alarm configuration
      bit 0: Alarm enable
             0 - alarm disabled, associated bit is controlled by DOut
             1 - alarm enabled,  associated bit is controlled by status
      bit 1: Alarm invert
             0 - normal polarity   (output is low when in alarm condition)
             1 - inverted polarity (output is high when in alarm condition)
      bits 2-3: Alarm type
             0 - High level: alarm when reading >= threshold 1, reset when reading < threshold 2
             1 - Low level: alarm when reading <= threshold 1, reset when reading > threshold 2
             2 - Outside window: alarm when reading < threshold 1 or > threshold 2
      bit 4: Alarm latch
             0 - no latch, alarm output status indicates current state of alarm
             1 - latch, alarm output is active if an alarm condition is detected 
                 and remains active until cleared with AlarmStatus command
      bits 5-6: Error alarms
            00 - Alarm can only be set by valid temperature reading
            01 - An open thermocouple or common-mode voltage error will also set the alarm
            10 - Only an open thermocouple or common-mode voltage will set the alarm,
                 termperature is ignored.
            11 - invalid.
      bit 7: reserved.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[592];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_ALARM_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 576;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->alarm_config,          &replyBuffer[MSG_INDEX_DATA],      32);
	  memcpy(device_info->alarm_threshold1,      &replyBuffer[MSG_INDEX_DATA+32],  128);
	  memcpy(device_info->alarm_threshold2,      &replyBuffer[MSG_INDEX_DATA+160], 128);
	  memcpy(&device_info->alarm_config[32],     &replyBuffer[MSG_INDEX_DATA+288],  32);
	  memcpy(&device_info->alarm_threshold1[32], &replyBuffer[MSG_INDEX_DATA+320], 128);
	  memcpy(&device_info->alarm_threshold2[32], &replyBuffer[MSG_INDEX_DATA+448], 128);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmConfigR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmConfigW_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command writes the temperature alarm configurations. There
     are configuration values and two threshold values for each of the
     32/64 thermocouple channels.  The configuration is stored in
     EEPROM and restored at power on.

    alarm_config: the alarm configuration
      bit 0: Alarm enable
             0 - alarm disabled, associated bit is controlled by DOut
             1 - alarm enabled,  associated bit is controlled by status
      bit 1: Alarm invert
             0 - normal polarity   (output is low when in alarm condition)
             1 - inverted polarity (output is high when in alarm condition)
      bits 2-3: Alarm type
             0 - High level: alarm when reading >= threshold 1, reset when reading < threshold 2
             1 - Low level: alarm when reading <= threshold 1, reset when reading > threshold 2
             2 - Outside window: alarm when reading < threshold 1 or > threshold 2
      bit 4: Alarm latch
             0 - no latch, alarm output status indicates current state of alarm
             1 - latch, alarm output is active if an alarm condition is detected 
                 and remains active until cleared with AlarmStatus command
      bits 5-6: Error alarms
            00 - Alarm can only be set by valid temperature reading
            01 - An open thermocouple or common-mode voltage error will also set the alarm
            10 - Only an open thermocouple or common-mode voltage will set the alarm,
                 termperature is ignored.
            11 - invalid.
      bit 7: reserved.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[592];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 576;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_ALARM_CONFIG_W;
  memcpy(&buffer[MSG_INDEX_DATA],      device_info->alarm_config,         32);
  memcpy(&buffer[MSG_INDEX_DATA+32],   device_info->alarm_threshold1,    128);
  memcpy(&buffer[MSG_INDEX_DATA+160],  device_info->alarm_threshold2,    128);
  memcpy(&buffer[MSG_INDEX_DATA+288], &device_info->alarm_config[32],     32);
  memcpy(&buffer[MSG_INDEX_DATA+320], &device_info->alarm_threshold1[32], 32);
  memcpy(&buffer[MSG_INDEX_DATA+448], &device_info->alarm_threshold2[32], 32);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmConfigW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmStatusR_E_TC32(DeviceInfo_TC32 *device_info)
{

  /* This command reads or clears the status of the temperature
     alarms. If a bit is set an alarm condition exists or is latched
     on the corresponding channel. If the alarm is configured for
     latching then the status will not clear when the alarm condition
     is no longer present. It must be cleared by writing a 1 to the
     corresponding bit. The LED on the front of the device is on if
     any bits are set in this value.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_ALARM_STATUS_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(device_info->alarm_status, &replyBuffer[MSG_INDEX_DATA], 8);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmStatusR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmStatusW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, uint32_t clear_masks)
{
  /* This command clears the alarm status.  Writing a 1 to a bit will
     clear the status for the corresponding channel. 
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 5;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_ALARM_STATUS_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = index;
  memcpy(&buffer[MSG_INDEX_DATA+1], &clear_masks, 4);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmStatusW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *         Memory  Commands              *
 *****************************************/

bool UserMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile user memory. The
     user memory is spread among 3 EEPROM parts

      Address                   Value
    --------------------------------------------
    0x0000 - 0x0EFF     Comms micro memory
    0x1000 - 0x1DFF     Measurement micro memory
    0x2000 - 0x2DFF     EXP micro memory
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 4;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 1024) {
    printf("UserMemoryR_E_TC32: max value of count is 1024\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_USER_MEMORY_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], &count, 2);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], count);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in UserMemoryR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolitile user memory. The amount of data to be written is
     inferred from the frame count - 2. The maximum that can be written in one transfer is 
     (1024 - 2) bytes.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = count;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 1022) {
    printf("UserMemoryW_E_TC32: max value of count is 1022\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_USER_MEMORY_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], &data, count);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in UserMemoryW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile network memory. The
     settings memory is 32 bytes (address 0 - 0x1F).
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 4;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (address > 0x1F) {
    printf("SettingsMemoryR_E_TC32: max value of address is 0x1F.\n");
    return false;
  }
  if (count > 32) {
    printf("SettingsMemoryR_E_TC32: max value of count is 32\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_SETTINGS_MEMORY_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], &count, 2);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], count);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in SettingsMemoryR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolitile settings memory. The settings
     memory is 32 bytes (address 0 - 0x1F).  The amount of data to be
     written is inferred from the frame count - 2.  The maximum that
     can be written is 32 bytes.  The settings will be implemented
     after a device reset.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = count+2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (address > 0x1F) {
    printf("SettingsMemoryW_E_TC32: max value of address is 0x1F.\n");
    return false;
  }
    
  if (count > 32) {
    printf("SettingsMemoryW_E_TC32: max value of count is 0x32.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_SETTINGS_MEMORY_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in SettingsMemoryW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool ConfigMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile device configuration memory. The
     configuration memory is 16 bytes (address 0 - 0x0F).
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 4;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (address > 0x0F) {
    printf("ConfigMemoryR_E_TC32: max value of address is 0x0F.\n");
    return false;
  }
  if (count > 16) {
    printf("ConfigMemoryR_E_TC32: max value of count is 16\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CONFIG_MEMORY_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], &count, 2);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], count);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in ConfigMemoryR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ConfigMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolitile device configuration
     memory. The configuration memory is 16 bytes (address 0 - 0x0F).
     The configuration will be implemented after a device reset.  The
     configuration should only be written during factory setup.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = count;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (address > 0x0F) {
    printf("ConfigMemoryW_E_TC32: max value of address is 0x0F.\n");
    return false;
  }
    
  if (count > 16) {
    printf("ConfignMemoryW_E_TC32: max value of count is 0x16.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CONFIG_MEMORY_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in ConfigMemoryW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCoefficientsR_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the factory calibration coefficients.  Each
     coefficient is a float.  The firmware applies the coefficients
     when calculating the voltage and temperature values for each
     channel.  The coefficients are applied immediately and stored in
     the EEPROM.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+sizeof(calCoeff_TC32)];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FACTORY_COEF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(calCoeff_TC32);
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(&device_info->calCoeffFactory, &replyBuffer[MSG_INDEX_DATA], sizeof(calCoeff_TC32));
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FactoryCoefficientsR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCoefficientsW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index)
{
  /* This command writes the factory calibration coefficients.  The
     microcontroller stores the values in EEPROM and restores them at
     power on. 
  */

  int sock = device_info->device.sock;
  unsigned char buffer[49];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 33;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FACTORY_COEF_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &index, 1);
  if (index == 0) {
    memcpy(&buffer[MSG_INDEX_DATA+1], device_info->calCoeffFactory.slope_60_base, 32);
  } else {
    memcpy(&buffer[MSG_INDEX_DATA+1], device_info->calCoeffFactory.slope_60_exp, 32);
  }
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FactoryCoefficientsW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCoefficientsR_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the field calibration coefficients.  Each
     coefficient is a float.  The firmware applies the coefficients
     when calculating the voltage and temperature values for each
     channel.  The coefficients are applied immediately and stored in
     the EEPROM.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+sizeof(calCoeff_TC32)];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FIELD_COEF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(calCoeff_TC32);
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(&device_info->calCoeffField, &replyBuffer[MSG_INDEX_DATA], sizeof(calCoeff_TC32));
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FieldCoefficientsR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCoefficientsW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index)
{
  /* This command writes the field calibration coefficients.  The
     microcontroller stores the values in EEPROM and restores them at
     power on. 
  */

  int sock = device_info->device.sock;
  unsigned char buffer[49];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 33;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FIELD_COEF_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &index, 1);
  if (index == 0) {
    memcpy(&buffer[MSG_INDEX_DATA+1], device_info->calCoeffField.slope_60_base, 32);
  } else {
    memcpy(&buffer[MSG_INDEX_DATA+1], device_info->calCoeffField.slope_60_exp, 32);
  }
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FieldCoefficientsW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CalDateR_E_TC32(DeviceInfo_TC32 *device_info, struct tm *date_base, struct tm *date_exp)
{
  /* This command reads the calibration dates. */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;
  time_t time;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CAL_DATE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 12;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  date_base->tm_year = replyBuffer[MSG_INDEX_DATA] + 100;
	  date_base->tm_mon = replyBuffer[MSG_INDEX_DATA+1] - 1;
	  date_base->tm_mday = replyBuffer[MSG_INDEX_DATA+2];
	  date_base->tm_hour = replyBuffer[MSG_INDEX_DATA+3];
	  date_base->tm_min = replyBuffer[MSG_INDEX_DATA+4];
	  date_base->tm_sec = replyBuffer[MSG_INDEX_DATA+5];
	  time = mktime(date_base);
	  date_base = localtime(&time);
	  
	  date_exp->tm_year = replyBuffer[MSG_INDEX_DATA+6] + 100;
	  date_exp->tm_mon = replyBuffer[MSG_INDEX_DATA+7] - 1;
	  date_exp->tm_mday = replyBuffer[MSG_INDEX_DATA+8];
	  date_exp->tm_hour = replyBuffer[MSG_INDEX_DATA+9];
	  date_exp->tm_min = replyBuffer[MSG_INDEX_DATA+10];
	  date_exp->tm_sec = replyBuffer[MSG_INDEX_DATA+11];
	  time = mktime(date_exp);
	  date_exp = localtime(&time);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CalDateR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CalDateW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, struct tm *date)
{
  /* This command writes the calibration dates  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 7;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CAL_DATE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA] = index;
  buffer[MSG_INDEX_DATA+1] = date->tm_year - 100;
  buffer[MSG_INDEX_DATA+2] = date->tm_mon + 1;
  buffer[MSG_INDEX_DATA+3] = date->tm_mday;
  buffer[MSG_INDEX_DATA+4] = date->tm_hour;
  buffer[MSG_INDEX_DATA+5] = date->tm_min;
  buffer[MSG_INDEX_DATA+6] = date->tm_sec; 
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CalDateW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool GainVoltageR_E_TC32(DeviceInfo_TC32 *device_info, gainVoltages *gain)
{
  /* This command reads the gain calibration voltage reference values. */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+sizeof(gainVoltages)];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_GAIN_VOLTAGE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(gainVoltages);
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(gain, &replyBuffer[MSG_INDEX_DATA], sizeof(gainVoltages));
	}
      }
    }
  }

  if (result == false) {
    printf("Error in GainVoltageR_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool GainVoltageW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, gainVoltages *gain)
{
  /* This command writes the gain calibration voltage reference values.  These values
     should only be written during manufacturing setup.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[49];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 17;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_GAIN_VOLTAGE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &index, 1);
  if (index == 0) {
    memcpy(&buffer[MSG_INDEX_DATA+1], &gain->voltage_0_base, 16);
  } else {
    memcpy(&buffer[MSG_INDEX_DATA+1], &gain->voltage_0_exp, 16);
  }
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in GainVoltageW_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *       Miscellaneous  Commands         *
 *****************************************/

bool BlinkLED_E_TC32(DeviceInfo_TC32 *device_info, uint8_t count)
{
  /* This command will blink the device power LED */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_BLINK_LED;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = count;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in BlinkLED_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Reset_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command resets the device */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_RESET;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in RESET_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Status_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command reads the device status.

    status: bit 0 - no EXP detected, 1 - EXP detected
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(&device_info->status, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Status_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Version_E_TC32(DeviceInfo_TC32 *device_info, struct version_t *version)
{
  /* This command reads the device firmware versions.  Each version
     will be in hex BCD (i.e. 0x0103 is version 1.03)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_VERSION;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 12;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(version, &replyBuffer[MSG_INDEX_DATA], 12);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Version_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool NetworkConfig_E_TC32(DeviceInfo_TC32 *device_info, struct networkDeviceInfo_t *network)
{
  /* This command reads the current device network configuration */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_NETWORK_CONFIG;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 12;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(network, &replyBuffer[MSG_INDEX_DATA], 12);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in NetworkConfig_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ADCal_E_TC32(DeviceInfo_TC32 *device_info)
{
  /* This command causes the measurement loop to pause and an A/D
     system offset calibration to run.  The calibration requires
     approximately 50ms to complete then the measurement loop
     automatically returns to the current mode.  The calibration will
     run on all A/Ds on both the main unit and EXP simultaneously.
     The command reply will not be sent until the calibration
     completes.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AD_CAL;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT_LOW] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_COUNT_HIGH] == (unsigned char) (replyCount >> 8))     &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in ADCal_E_TC32. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
