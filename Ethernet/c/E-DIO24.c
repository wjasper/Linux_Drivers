/*
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <string.h>
#include <stdbool.h>
#include "E-DIO24.h"

/*********************************************
 *        Digital I/O  Commands              *
 *********************************************/

bool DIn_DIO24(EthernetDeviceInfo *device_info, uint32_t *value)
{
  /* This command reads the current state of the DIO pins.  A 0 in a
     bit position indicates the correspoing pin is reading a low
     state, and a 1 indicates a high state.
  */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DIN_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 3;
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
    printf("Error in DIn_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutR_DIO24(EthernetDeviceInfo *device_info, uint32_t *value)
{
  /* This command reads the DIO output latch value.  The factory power
     on default is all 0.  A 0 in a bit position indicates the
     corresponding pin driver is low, a 1 indicates it is high.
  */

  int sock = device_info->sock;
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
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 3;
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
    printf("Error in DOutR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOut_DIO24(EthernetDeviceInfo *device_info, uint32_t mask, uint32_t value)
{
  /* This command writes the DIO latch value.  The factory power on
     default is all 1 (pins are floating.)  Writing a 0 to a bit will set
     the corresponding pin driver low, writing a 1 sets it high.
     Individual bits may be written using the port bitmask.
  */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 6;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_W;
  memcpy(&buffer[MSG_INDEX_DATA],   &mask, 3);
  memcpy(&buffer[MSG_INDEX_DATA+3], &value, 3);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in DOut_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigR_DIO24(EthernetDeviceInfo *device_info, uint32_t *value)
{
  /* This command reads the DIO configuration value.  A 1 in a bit
      position indicates the corresponding pin is set to an input, a 0
      indicates it is set to an output.  The power on default is all 1
      (input).
  */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 3;
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
          *value = 0x0;
          memcpy(value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }
  if (result == false) {
    printf("Error in DConfigR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigW_DIO24(EthernetDeviceInfo *device_info, uint32_t mask, uint32_t value)
{
  /* This command writes the DIO configuration value.  A 1 in a bit
     position sets the corresponding pin to an input, a 0 sets it to an
     output.  The power on default is all 1 (input).
     Individual configurations may be written using the port bitmask.
  */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 6;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_W;
  memcpy(&buffer[MSG_INDEX_DATA],   &mask, 3);
  memcpy(&buffer[MSG_INDEX_DATA+3], &value, 3);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in DConfigW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *         Counter  Commands                 *
 *********************************************/
bool CounterR_DIO24(EthernetDeviceInfo *device_info, uint32_t *count)
{
  /* This command reads the event counter on pin P2D7. Configure it as
     input.  The pin accepts frequency input up to 10 MHz.  The
     internal counter increments when the TTL levels transition from
     low to high.
  */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          *count = 0x0;
          memcpy(count, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }
  if (result == false) {
    printf("Error in CounterR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool CounterW_DIO24(EthernetDeviceInfo *device_info)
{
  /* This command resets the event counter.  On a write the counter will be reset to 0. */

  int sock = device_info->sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;  // no data for this command
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in CounterW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}


/*********************************************
 *        Miscellaneous Commands             *
 *********************************************/

bool BlinkLED_DIO24(EthernetDeviceInfo *device_info, unsigned char count)
{
  // This comman will blink the device power LED "count" times.

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_BLINKLED;
  buffer[MSG_INDEX_DATA]           = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in BlinkLED_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Reset_DIO24(EthernetDeviceInfo *device_info)
{
  // The command resets the device.

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no data for this command
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_RESET;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in Reset_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Status_DIO24(EthernetDeviceInfo *device_info, uint16_t *status)
{
  // The command reads the device status

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no input data for this command
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_STATUS;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy(status, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Status_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool NetworkConfig_DIO24(EthernetDeviceInfo *device_info, struct in_addr network[3])
{
  /* The command reads the current network configuration
     network[0] = ip_address
     network[1] = subnet mask
     network[2] = gateway_address
  */

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no input data for this command
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_NETWORK_CONF;;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy((unsigned char*) network, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in NetworkConfig_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FirmwareUpgrade_DIO24(EthernetDeviceInfo *device_info)
{
  /* This command causes the device to reset and enter the bootloader
     for a firmware upgrade.  It erases a portion of the program memory so
     the device must have firmware downloaded through the bootloader before
     it can be used again.
  */

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FIRMWARE;
  buffer[MSG_INDEX_DATA]           = 0xad;     // key
  buffer[MSG_INDEX_DATA+1]         = 0xad;     // key
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
    printf("Error in FirmwareUpgrade_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *           Memory  Commands                *
 *********************************************/

bool ConfigMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  // This command reads the nonvolatile configuration memory.  The cal memory is 16 bytes (address 0 - 0xf)

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;

  if (sock < 0 || count > 16) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_CONF_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in ConfigMemoryR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool ConfigMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile configuration memory.  The
     config memory is 16 bytes (address 0 - 0xf) The config memory
     should only be written during factory setup and has an additional
     lock mechanism to prevent inadvertent writes.  To enable writes
     to the config memory, first write the unlock code 0xAA55 to
     address 0x10.  Writes to the entire meemory range are then
     possible.  Write any other value to address 0x10 to lock the
     memory after writing.  The amount of data to be writeen is
     inferred from the frame count - 2.

     address: the start address for writing (0-0xf)
     data     the data to be written (frame count -2)
  */

  int sock = device_info->sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;

  if (sock < 0) {
    return false;
  }
  if (count > 512) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_CONF_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
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
    printf("Error in ConfigMemoryW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile user memory.  The user memory is 3827 bytes (address 0 - 0xeef)
     address: the start address for reading (0-0xeef)
     count:   the number of bytes to read (max 1024 due to protocol)
  */

  int sock = device_info->sock;
  unsigned char buffer[1050];
  unsigned char replyBuffer[1050];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 1024 || address > 0xeef) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in UserMemoryR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool UserMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile user memory.  The user memory
     is 3824 bytes (address 0 - 0xeef). The amount of data to be
     written is inferred from the frame count - 2.  The maximum that
     can be writtenin one transfer is 1024 bytes.
  */

  int sock = device_info->sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;

  if (sock < 0 || address > 0xeef || count > 1024) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
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
    printf("Error in UserMemoryW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile settings memory.  The settings memory is 256 bytes (0x00-0xFF)

     address: the start address for reading (0-0xff)
     count:   the number of bytes to read (max 256 due to protocol)
  */

  int sock = device_info->sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;

  if (sock < 0 || count > 256 || address > 0xff) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in SettingMemoryR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool SettingsMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile settings memory.  The
     settings memory is 256 bytes (address 0 - 0xff). The amount of
     data to be written is inferred from the frame count - 2.  The
     settings will be implemented after a device reset.
  */

  int sock = device_info->sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;

  if (sock < 0 || count > 512 || address > 0xff) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
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
    printf("Error in SettingMemoryW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool BootloaderMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the bootloader stored in nonvolatile FLASH
     memory.  The bootloader is located in program FLASH memory in two
     physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
     code and 0x1FC00000 - 0x1FC01FFF for C startup code and
     interrupts.  Reads may be performed at any time.

     address: the start address for reading (see above)
     count:   the number of bytes to read (max 1024)
  */

  int sock = device_info->sock;
  unsigned char buffer[1050];
  unsigned char replyBuffer[1050];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;

  if (sock < 0 || count > 1024) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
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
          memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in BootloaderMemoryR_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool BootloaderMemory_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the bootloader stored in nonvolatile FLASH
     memory.  The bootloader is located in program FLASH memory in two
     physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
     code and 0x1FC00000 - 0x1FC01FFF for C startup code and
     interrupts.  Writes outside these ranges are ignored.  The
     bootloader memory is write protected and must be unlocked in
     order to write the memory.  The unlock proceedure is to write the
     unlock code 0xAA55 to address 0xFFFFFFFE.  Writes to the entire
     memory range are then possible.  Write any other value to address
     0xFFFFFFFE to lock the memory after writing.

     The FLASH memory must be erased prior to programming.  A bulk
     erase is perfomred by writing 0xAA55 to address 0x80000000 after
     unlocking the memory for write.  The bulk erase will require
     approximately 150ms to complete.  Once the erase is complete, the
     memory may be written; however, the device will not be able to
     boot unless it has a valid bootloader so the device shold not be
     reset until the bootloader is completely written and verified
     using readBootloaderMemory_DIO24().

     The writes are perfomred on 4-byte boundaries internally and it
     is recommended that the output data be sent in the same manner.
     The amount of data to be written is inferred frolm the frame
     count - 2.  The maximum count value is 1024.
  */

  int sock = device_info->sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;

  if (sock < 0 || count > 1024) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
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
    printf("Error in BootloaderMemoryW_DIO24. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
