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
#include "E-TC.h"

static int nBits8(uint8_t num)
{
  int count = 0;
  int i;
  
  // counts the number of bits in a number
  for (i = 0; i < 8; i++) {
    if ((num & 0x1) == 0x1) count++;
    num = (num >> 0x1);
  }
  return count;
}

/*********************************************
 *        Digital I/O  Commands              *
 *********************************************/

bool DIn_E_TC(DeviceInfo_TC *device_info, uint8_t *value)
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

  buffer[MSG_INDEX_COMMAND]        = DIN;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  *value = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DIn_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutR_E_TC(DeviceInfo_TC *device_info, uint8_t *value)
{
  /* This command reads the DIO output latch value.  The factory power
     on default is all 0. A 0 in a bit position indicates the
     corresponding pin latch register is low, a 1 indicates it is high.
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

  buffer[MSG_INDEX_COMMAND]        = DOUT_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  *value =  replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DOutR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutW_E_TC(DeviceInfo_TC *device_info, uint8_t value)
{
  /* This command writes the DIO latch value.  Writing a 0 to a bit
     will set the corresponding pin driver low, writing a 1 sets it
     high.  If pin(s) are configured as alarm outputs this command
     does not affect their value.
  */

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

  buffer[MSG_INDEX_COMMAND]        = DOUT_W;
  buffer[MSG_INDEX_DATA]           = value;
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
    printf("Error in DOutW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigR_E_TC(DeviceInfo_TC *device_info, uint8_t *value)
{
  /* This command reads the DIO configuration value.  A 1 in a bit
     position indicates the corresponding pin is set to an input, a 0
     indicates it is set to an output.  The power on default is all 1
     (input).
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

  buffer[MSG_INDEX_COMMAND]        = DCONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  *value =  replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DConfigR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigW_E_TC(DeviceInfo_TC *device_info, uint8_t value)
{
  /* This command writes the DIO configuration value.  A 1 in a bit
     position sets the corresponding pin to an input, a 0 sets it to
     an output .  The power on default is all 1 (input).  If one or
     more alarms are configured, they will forcde the corresponding
     DIO bit to an output and may or may be overriddrn with this
     command.
  */

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

  buffer[MSG_INDEX_COMMAND]        = DCONFIG_W;
  buffer[MSG_INDEX_DATA]           = value;
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
    printf("Error in DOutW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *      Temperature Input  Commands          *
 *********************************************/

bool Tin_E_TC(DeviceInfo_TC *device_info, uint8_t channel_mask, uint8_t units, uint8_t wait, float *value)
{
  /* This command reads the value of one or more thermocouple
     channels. The channels to be read are passed as a bitmask when
     calling the command. The data will be returned in the order low
     channel number to high channel. The number of floating point
     values returned will be equal to the number of channels specified
     (max 8). Each temperature is calculated by performing cold
     junction compensation as follows:

     T = reverse_nist_function(Vin + Vcjc), where Vin is input
     voltage, Vcjc is calculated cold junction voltage based on the
     terminal temperature, and reverse_nist_function() is the NIST
     thermocouple polynomial for voltage to temperature.

     Vcjc = nist_function(Tcjc), where Tcjc is the temperature of the
     screw terminal and nist_function() is the NIST thermocouple
     polynomial for temperature to voltage.

     Tcjc = Tsensor + Toffset + Tuser, where Tsensor is the
     appropriate CJC sensor temperature, Toffset is a per-channel
     offset characterized over a number of boards and fixed in
     firmware, and Tuser is an optional user-specified offset (see
     CJCOffset_r and CJCOffset_w).  

     There are some special return values: 
         -6666.0: Over range on input (temperature < -300C, only reported with units = 0) 
         -8888.0: Open thermocouple detected (only reported with units = 0) 
         -9999.0: Channel disabled

     channel_mask:  bitmask, the channels to be read
     units:         the units for the returned values
                      0 - Celsius, linerarized by TC type
                      1 - Voltage
                      2 - ADC code (uncalibrated)
     wait:            0 - return current value, 1 - wait for new readings before returning

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

  buffer[MSG_INDEX_COMMAND]        = TIN;
  buffer[MSG_INDEX_DATA]           = channel_mask;
  buffer[MSG_INDEX_DATA+1]         = units;
  buffer[MSG_INDEX_DATA+2]         = wait;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(float)*nBits8(channel_mask);
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
	  memcpy(value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Tin_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CJC_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the most recent value of the CJC sensors in Celsius.  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_CJC;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2*sizeof(float);
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
	  memcpy(device_info->CJC_value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CJC_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinConfigR_E_TC(DeviceInfo_TC *device_info)
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
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = TIN_CONFIG_R;
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
	  memcpy(device_info->config_values, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinConfigR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinConfigW_E_TC(DeviceInfo_TC *device_info)
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
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 8;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = TIN_CONFIG_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], device_info->config_values, dataCount);
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
    printf("Error in TinConfigW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool TinStatus_E_TC(DeviceInfo_TC *device_info)
{
    /* This command reads the status of the temperature readings.  If
       a bit is set the corresponding channel has a new reading that
       has not been read with the Tin command.
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

  buffer[MSG_INDEX_COMMAND]        = TIN_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  device_info->Tin_status = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in TinStatus_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool OTDStatus_E_TC(DeviceInfo_TC *device_info)
{
    /* This command reads the status of the open thermocouple
       detection.  If a bit is set an open thermocouple is currently
       detected on the corresponding channel.  
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

  buffer[MSG_INDEX_COMMAND]        = OTD_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  device_info->OTD_status = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in OTDStatus_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureConfigR_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the measurement configuration. 

     bit 0: OTD disable         0 - OTD enable,          1 - OTD disabled
     bit 1: Coefficient select  0 - factory coefficients 1 - field coefficients
     bit 2-7:  Reserved 
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

  buffer[MSG_INDEX_COMMAND]        = MEASURE_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  device_info->config_measure =  replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureConfigR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureConfigW_E_TC(DeviceInfo_TC *device_info)
{
  /* This command writes the measurement configuration. 

     bit 0:    0 - OTD enable,    1 - OTD disabled
     bit 1:    0 - factory coef.  1 - field coef.
     bits 2-7: Reserved
*/

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

  buffer[MSG_INDEX_COMMAND]        = MEASURE_CONFIG_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = device_info->config_measure;
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
    printf("Error in MeasureConfigW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureModeR_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the measurement mode.  The power on default is mode 0 (normal mode) 
       mode_base: the measurement mode for the base unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 0 and 4 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 4.
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

  buffer[MSG_INDEX_COMMAND]        = MEASURE_MODE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  device_info->mode_measure = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in MeasureModeR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool MeasureModeW_E_TC(DeviceInfo_TC *device_info)
{
  /* This command writes the measurement mode. 
       mode: the measurement mode:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence.
             1 = Test mode: the muxes are fixed on channel 0 and 4 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 4.
  */

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

  buffer[MSG_INDEX_COMMAND]        = MEASURE_MODE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = device_info->mode_measure;
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
    printf("Error in MeasureModeW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCoefficientsR_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the factory calibration coefficients.  Each
     coefficient is a float.  The firmware applies the coefficients
     when calculating the voltage and temperature values for each
     channel.  The coefficients are applied immediately and stored in
     the EEPROM.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FACTORY_COEF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(calCoeff);
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
	  memcpy(&device_info->calCoeffFactory, &replyBuffer[MSG_INDEX_DATA], sizeof(calCoeff));
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FactoryCoefficientsR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCoefficientsW_E_TC(DeviceInfo_TC *device_info)
{
  /* This command writes the factory calibration coefficients.  The
     microcontroller stores the values in EEPROM and restores them at
     power on. 
  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 4*sizeof(float);
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FACTORY_COEF_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], device_info->calCoeffFactory.slope, dataCount);
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
    printf("Error in FactoryCoefficientsW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCoefficientsR_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the field calibration coefficients.  Each
     coefficient is a float.  The firmware applies the coefficients
     when calculating the voltage and temperature values for each
     channel.  The coefficients are applied immediately and stored in
     the EEPROM.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[49];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FIELD_COEF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = sizeof(calCoeff);
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
	  memcpy(&device_info->calCoeffField, &replyBuffer[MSG_INDEX_DATA], sizeof(calCoeff));
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FieldCoefficientsR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCoefficientsW_E_TC(DeviceInfo_TC *device_info)
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
  int dataCount = 4*sizeof(float);
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FIELD_COEF_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA+1], device_info->calCoeffField.slope, dataCount);
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
    printf("Error in FieldCoefficientsW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCalDateR_E_TC(DeviceInfo_TC *device_info, struct tm *date)
{
  /* This command reads the factory calibration date. */

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

  buffer[MSG_INDEX_COMMAND]        = FACTORY_CAL_DATE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 6;
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
	  date->tm_year = replyBuffer[MSG_INDEX_DATA] + 100;
	  date->tm_mon = replyBuffer[MSG_INDEX_DATA+1] - 1;
	  date->tm_mday = replyBuffer[MSG_INDEX_DATA+2];
	  date->tm_hour = replyBuffer[MSG_INDEX_DATA+3];
	  date->tm_min = replyBuffer[MSG_INDEX_DATA+4];
	  date->tm_sec = replyBuffer[MSG_INDEX_DATA+5];
	  time = mktime(date);
	  date = localtime(&time);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FactoryCalDateR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FactoryCalDateW_E_TC(DeviceInfo_TC *device_info, struct tm *date)
{
  /* This command writes the factory calibration date  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 6;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FACTORY_CAL_DATE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]   = date->tm_year - 100;
  buffer[MSG_INDEX_DATA+1] = date->tm_mon + 1;
  buffer[MSG_INDEX_DATA+2] = date->tm_mday;
  buffer[MSG_INDEX_DATA+3] = date->tm_hour;
  buffer[MSG_INDEX_DATA+4] = date->tm_min;
  buffer[MSG_INDEX_DATA+5] = date->tm_sec; 
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
    printf("Error in FactoryCalDateW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCalDateR_E_TC(DeviceInfo_TC *device_info, struct tm *date)
{
  /* This command reads the field calibration date. */

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

  buffer[MSG_INDEX_COMMAND]        = FIELD_CAL_DATE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 6;
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
	  date->tm_year = replyBuffer[MSG_INDEX_DATA] + 100;
	  date->tm_mon = replyBuffer[MSG_INDEX_DATA+1] - 1;
	  date->tm_mday = replyBuffer[MSG_INDEX_DATA+2];
	  date->tm_hour = replyBuffer[MSG_INDEX_DATA+3];
	  date->tm_min = replyBuffer[MSG_INDEX_DATA+4];
	  date->tm_sec = replyBuffer[MSG_INDEX_DATA+5];
	  time = mktime(date);
	  date = localtime(&time);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FieldCalDateR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FieldCalDateW_E_TC(DeviceInfo_TC *device_info, struct tm *date)
{
  /* This command writes the field calibration date */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 6;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FIELD_CAL_DATE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]   = date->tm_year - 100;
  buffer[MSG_INDEX_DATA+1] = date->tm_mon + 1;
  buffer[MSG_INDEX_DATA+2] = date->tm_mday;
  buffer[MSG_INDEX_DATA+3] = date->tm_hour;
  buffer[MSG_INDEX_DATA+4] = date->tm_min;
  buffer[MSG_INDEX_DATA+5] = date->tm_sec; 
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
    printf("Error in FieldCalDateW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ADCal_E_TC(DeviceInfo_TC *device_info)
{
  /* This command causes the measurement loop to pause and an A/D
     system offset calibration to run.  The calibration requires
     approximately 800ms to complete then the measurement loop
     automatically returns to the current mode.  The calibration will
     run on both A/Ds simultaneously.  The command reply will not be
     sent until the calibration completes.
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

  buffer[MSG_INDEX_COMMAND]        = AD_CAL;
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
    printf("Error in ADCal_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CJCOffsetR_E_TC(DeviceInfo_TC *device_info)
{
    /* This command reads the CJC user offset values.  Each value is
       added to the appropriate CJC sensor and factory offset reading
       prior to performing te NIST calculation for the temperature for
       that channel.
    */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[128];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CJC_OFFSET_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8*sizeof(float);
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
	  memcpy(device_info->CJC_offsets, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CJCOffsetR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CJCOffsetW_E_TC(DeviceInfo_TC *device_info)
{
    /* This command writes the CJC user offset values.  Each value is
       added to the appropriate CJC sensor reading and factory offset
       prior to performing the NIST calculation for the temperature
       for that channel.
    */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 8*sizeof(float);
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CJC_OFFSET_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], device_info->CJC_offsets, dataCount);
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
    printf("Error in CJCOffsetsW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *         Alarm  Commands               *
 *****************************************/

bool AlarmConfigR_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the temperature alarm configurations. There
     are configuration values and two threshold values for each of the
     thermocouple channels.

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

    threshold_1  float[8] the current alarm threshold 1 values in Celsius
    threshold_2  float[8] the current alarm threshold 2 values in Celsius
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

  buffer[MSG_INDEX_COMMAND]        = ALARM_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 16*sizeof(float)+8;
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
	  memcpy(device_info->alarm_config,          &replyBuffer[MSG_INDEX_DATA],     8);
	  memcpy(device_info->alarm_threshold1,      &replyBuffer[MSG_INDEX_DATA+8],  32);
	  memcpy(device_info->alarm_threshold2,      &replyBuffer[MSG_INDEX_DATA+40], 32);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmConfigR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmConfigW_E_TC(DeviceInfo_TC *device_info)
{
  /* This command writes the temperature alarm configurations. There
     are configuration values and two threshold values for each of the
     thermocouple channels.  The configuration is stored in EEPROM and
     restored at power on.

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

    threshold_1  float[8] the current alarm threshold 1 values in Celsius
    threshold_2  float[8] the current alarm threshold 2 values in Celsius
  */

  int sock = device_info->device.sock;
  unsigned char buffer[592];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = ALARM_CONFIG_W;
  memcpy(&buffer[MSG_INDEX_DATA],    device_info->alarm_config,         8);
  memcpy(&buffer[MSG_INDEX_DATA+8],  device_info->alarm_threshold1,    32);
  memcpy(&buffer[MSG_INDEX_DATA+40], device_info->alarm_threshold2,    32);
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
    printf("Error in AlarmConfigW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmStatusR_E_TC(DeviceInfo_TC *device_info)
{

  /* This command reads the status of the temperature
     alarms. If a bit is set an alarm condition exists or is latched
     on the corresponding channel. If the alarm is configured for
     latching then the status will not clear when the alarm condition
     is no longer present. It must be cleared by writing a 1 to the
     corresponding bit using the AlarmStatusW command.
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

  buffer[MSG_INDEX_COMMAND]        = ALARM_STATUS_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (sendMessage(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
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
	  device_info->alarm_status = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AlarmStatusR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AlarmStatusW_E_TC(DeviceInfo_TC *device_info, uint8_t clear_mask)
{
  /* This command clears the alarm status.  Writing a 1 to a bit will
     clear the status for the corresponding channel. 
  */

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

  buffer[MSG_INDEX_COMMAND]        = ALARM_STATUS_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_DATA]           = clear_mask;
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
    printf("Error in AlarmStatusW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *         Counter  Commands              *
 *****************************************/
bool CounterR_E_TC(DeviceInfo_TC *device_info)
{

  /* The command read the event counter. */

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

  buffer[MSG_INDEX_COMMAND]        = COUNTER_R;
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
	  memcpy(&device_info->counter, &replyBuffer[MSG_INDEX_DATA], 4);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CounterR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ResetCounter_E_TC(DeviceInfo_TC *device_info)
{
  /* This command resets the event counter.  On a write, the counter
     will be reset to 0.
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

  buffer[MSG_INDEX_COMMAND]        = COUNTER_W;
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
    printf("Error in ResetCounter_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *         Memory  Commands              *
 *****************************************/

bool ConfigMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
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
    printf("ConfigMemoryR_E_TC: max value of address is 0x0F.\n");
    return false;
  }
  if (count > 16) {
    printf("ConfigMemoryR_E_TC: max value of count is 16\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CONFIG_MEMORY_R;
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
    printf("Error in ConfigMemoryR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ConfigMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
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
    printf("ConfigMemoryW_E_TC: max value of address is 0x0F.\n");
    return false;
  }
    
  if (count > 16) {
    printf("ConfigMemoryW_E_TC: max value of count is 0x16.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CONFIG_MEMORY_W;
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
    printf("Error in ConfigMemoryW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile user memory. 

      Address                   Value
    --------------------------------------------
    0x0000 - 0x0DFF     Available for UL use

    address: the start address for reading (max value 3583)
    count:   the number of bytes to read (max 1024 due to protocol)
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

  if (count > 1024 || address > 0xdff) {
    printf("UserMemoryR_E_TC: max value of count is 1024.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = USER_MEMORY_R;
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
    printf("Error in UserMemoryR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolitile user memory. The amount of data to be written is
     inferred from the frame count - 2. The maximum that can be written in one transfer is 
     (1024 - 2) bytes.
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

  if (count > 0xdff) {
    printf("UserMemoryW_E_TC: max value of count is 0xdff\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = USER_MEMORY_W;
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
    printf("Error in UserMemoryW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
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
    printf("SettingsMemoryR_E_TC: max value of address is 0x1F.\n");
    return false;
  }
  if (count > 32) {
    printf("SettingsMemoryR_E_TC: max value of count is 32.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = SETTINGS_MEMORY_R;
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
    printf("Error in SettingsMemoryR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
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
    printf("SettingsMemoryW_E_TC: max value of address is 0x1F.\n");
    return false;
  }
    
  if (count > 32) {
    printf("SettingsMemoryW_E_TC: max value of count is 0x32.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = SETTINGS_MEMORY_W;
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
    printf("Error in SettingsMemoryW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool BootloaderMemoryR_E_TC(DeviceInfo_TC *device_info, uint32_t address, uint16_t count, uint8_t *data)
{
  /* The command reads the bootloader stored in nonvolatile FLASH
     memory.  The bootloader is located in program FLASH memory in two
     physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
     code and 0x1FC00000 - 0x1FC01FFF for C startup code and
     interrupts.  Reads may be performed at any time.

     address:  the address for reading
     count:    the number of byes gto read (max 1024)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[1048];
  bool result = false;
  int length;
  int dataCount = 6;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (!((address >= 0x1D000000 && address <= 0x1D007FFF) || (address >= 0x1FC00000 && address <= 0x1FC01FFF))) {
    printf("BootloaderMemoryR_E_TC: Memory out of range.\n");
    return false;
  }
  if (count > 1024) {
    printf("BootloaderMemoryR_E_TC: max value of count is 1024\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = BOOTLOADER_MEMORY_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 4);
  memcpy(&buffer[MSG_INDEX_DATA+4], &count, 2);
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
    printf("Error in BootloaderMemoryR_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool BootloaderMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* 
    This command writes the bootloader stored in nonvolatile FLASH
    memory. The bootloader is located in program FLASH memory in two
    physical address ranges: 0x1D000000 – 0x1D007FFF for bootloader
    code and 0x1FC00000 – 0x1FC01FFF for C startup code and
    interrupts. Writes outside these ranges are ignored. The
    bootloader memory is write protected and must be unlocked in order
    to write the memory. The unlock procedure is to write the unlock
    code 0xAA55 to address 0xFFFFFFFE. Writes to the entire memory
    range are then possible. Write any other value to address
    0xFFFFFFFE to lock the memory after writing.

    The FLASH memory must be erased prior to programming. A bulk erase
    is performed by writing 0xAA55 to address 0x80000000 after
    unlocking the memory for write. The bulk erase will require
    approximately 150 ms to complete. Once the erase is complete the
    memory may be written; however, the device will not be able to
    boot unless it has a valid bootloader so the device should not be
    reset until the bootloader is completely written and verified
    using BootloaderMemory_r.  

    The writes are performed on 4-byte boundaries internally and it is
    recommended that the output data be sent in the same manner. The
    amount of data to be written is inferred from the frame count –
    2. The maximum count value is 1024.

    Output Arguments 
      address: the address for writing (see above)
      data:    the data to be written
  */

  int sock = device_info->device.sock;
  unsigned char buffer[1048];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = count+2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (!((address >= 0x1D000000 && address <= 0x1D007FFF) || (address >= 0x1FC00000 && address <= 0x1FC01FFF))) {
    printf("BootloaderMemoryW_E_TC: bootload address out of range.\n");
    return false;
  }
    
  if (count > 1024) {
    printf("BooloaderMemoryW_E_TC: max value of count is 1024.\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = BOOTLOADER_MEMORY_W;
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
    printf("Error in BootloaderMemoryW_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*****************************************
 *       Miscellaneous  Commands         *
 *****************************************/

bool BlinkLED_E_TC(DeviceInfo_TC *device_info, uint8_t count)
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

  buffer[MSG_INDEX_COMMAND]        = BLINK_LED;
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
    printf("Error in BlinkLED_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Reset_E_TC(DeviceInfo_TC *device_info)
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

  buffer[MSG_INDEX_COMMAND]        = RESET;
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
    printf("Error in RESET_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Status_E_TC(DeviceInfo_TC *device_info)
{
  /* This command reads the device status.

    status: bit 0-15 Reserved
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

  buffer[MSG_INDEX_COMMAND]        = STATUS;
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
    printf("Error in Status_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool NetworkConfig_E_TC(DeviceInfo_TC *device_info, struct networkDeviceInfo_t *network)
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

  buffer[MSG_INDEX_COMMAND]        = NETWORK_CONFIG;
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
    printf("Error in NetworkConfig_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FirmwareUpgrade_E_TC(DeviceInfo_TC *device_info)
{
  /* This command causes the device to reset and enter the bootloader for firmware upgrade */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 2;
  int replyCount;
  uint16_t key = 0xADAD;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = FIRMWARE_UPGRADE;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], &key, dataCount);
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
    printf("Error in FirmwareUpgrade_E_TC. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
