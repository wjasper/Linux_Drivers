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
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "E-1608.h"

void buildGainTableAIn_E1608(DeviceInfo_E1608 *device_info)
{
  /* Builds a lookup table of calibration coefficients to translate values into voltages:
     The calibration coefficients are stored in the onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code*slope + intercept

  */

  uint16_t address;
  int i;

  // Analog Input Calibration, differential 0x000 - 0x01C
  address = 0x0;
  for (i = 0; i < NGAINS; i++) {
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AInDF[i].slope);
    address += 4;
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AInDF[i].intercept);
    address += 4;
  }

  // Analog Input Calibration, single ended 0x020 - 0x03C
  address = 0x020;
  for (i = 0; i < NGAINS; i++) {
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AInSE[i].slope);
    address += 4;
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AInSE[i].intercept);
    address += 4;
  }
}

void buildGainTableAOut_E1608(DeviceInfo_E1608 *device_info)
{
  uint16_t address;
  int i;
  
  address = 0x40;
  for (i = 0; i < NCHAN_AOUT; i++) {
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AOut[i].slope);
    address += 4;
    CalMemoryR_E1608(device_info, address, 4, (uint8_t *) &device_info->table_AOut[i].intercept);
    address += 4;
  }
}

/*********************************************
 *        Digital I/O  Commands              *
 *********************************************/

bool DIn_E1608(DeviceInfo_E1608 *device_info, uint8_t *value)
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
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DIN_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in DIn_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutR_E1608(DeviceInfo_E1608 *device_info, uint8_t *value)
{
  /* This command reads the DIO output latch value.  The factory power
     on default is all 0.  A 0 in a bit position indicates the
     corresponding pin driver is low, a 1 indicates it is high.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;
  int timeout = device_info->timeout;

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

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in DOutR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOut_E1608(DeviceInfo_E1608 *device_info, uint8_t value)
{
  /* This command writes the DIO latch value.  The factory power on
     default is all 1 (pins are floating.)  Writing a 0 to a bit will set
     the corresponding pin driver low, writing a 1 sets it high.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;
  int timeout = device_info->timeout;
  
  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_W;
  buffer[MSG_INDEX_DATA]           = value;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in DOut_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigR_E1608(DeviceInfo_E1608 *device_info, uint8_t *value)
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
  int timeout = device_info->timeout;
  
  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in DConfigR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DConfigW_E1608(DeviceInfo_E1608 *device_info, uint8_t value)
{
  /* This command writes the DIO configuration value.  A 1 in a bit
     position sets the corresponding pin to an input, a 0 sets it to an
     output.  The power on default is all 1 (input).
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_W;
  buffer[MSG_INDEX_DATA]           = value;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in DConfigW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *        Analog Input                       *
 *********************************************/

bool AIn_E1608(DeviceInfo_E1608 *device_info, uint8_t channel, uint8_t range, uint16_t *value)
{
  /* This command reads the value of an analog input channel.  This commands will
     not return valid data if AIn scan is currently running.

     channel 0-7  single ended
     channel 8-11 differential
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int data;
  int length;
  int dataCount = 2;
  int replyCount;
  int timeout = device_info->timeout;
  
  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AIN;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_DATA+1]         = range;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
	  memcpy((unsigned char *) value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AIn_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
    *value = 0xffff;
    return result;
  }

  if (channel < DF) { // single ended
    data = rint((*value)*device_info->table_AInSE[range].slope + device_info->table_AInSE[range].intercept);
  } else {  // differential
    data = rint((*value)*device_info->table_AInDF[range].slope + device_info->table_AInDF[range].intercept);    
  }

  if (data >= 65536) {
    *value = 65535;
  } else if (data < 0) {
    *value = 0;
  } else {
    *value = data;
  }
  
  return result;
}

bool AInScanStart_E1608(DeviceInfo_E1608 *device_info, uint32_t count, double frequency, uint8_t options)
{
  /* This command starts an analog input scan.  The channel ordering
     and number of channels per scan is set by the channel gain queue
     which must first be set with the AInQueue_w command.  This
     command will respond with the busy error if an AIn scan is
     currently running.

     The pacer rate is set by an internal 32-bit timer running at a
     base rate of 80 MHz.  The timer is controlled by pacer_period.
     This value is the period of the scan and the A/D is clocked at
     this rate.  A pulse will be ouput at the AICKO pin at every
     pacer_period interval.  The equation for calculating
     tracer_period is:

     pacer_period = [80 MHz / (sample frequency)] -1

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  It uses the AICKI pin as an input and the user must
     provice the pacer source.  The A/D acquires data on every rising
     edge of the pacer clock: the maximum allowable input frequency is
     250 kHz.

     The data is read and sent to the host using the AInScan data TCP
     port.  The device checks for a connection on this port when
     AInScanStart is called and will return an error code if it is not
     connected.  The scan will not start until the command reply ACK
     is received; see the Ethernet Communication Mechanism section for
     more details.

     Scan data will be acquired until an overrun occurs, the specified
     count is reached, or an AInScanStop command is sent.  the scan
     data will be in the format:

     First channel sample 0: second channel sample 0: .. : last channel sample 0
     First channel sample 1: second channel sample 1: .. : last channel sample 1
     ...
     First channel sample n: second channel sample n: .. : last channel sample n

     If the host does not receive the data in a timely manner (due to
     a communications error, overrun, etc.) it can check the status of
     the scan with the Status command.  Any data in the scan data TCP
     buffer will be sent every 40ms or when the MTU size is reached.

     The external trigger may be used to start the scan.  If enabled,
     the device will wait until the appropriate trigger condition is
     detected then betgin sampling data at the specified rate.  No
     data will be available until the trigger is detected.

     count:      The total number or scan to scquire, 0 for continuou8s scan
     frequency:  the sampling frequency.  Use 0 for external clock.
     options:    Bit field that controls scan options
                 bits 0-1:   Reserved
                 bits 2-4:   Trigger setting:
                             0: no trigger
			     1: Edge/rising
			     2: Edge / falling
			     3: Level / high
			     4: Level / low
		 bit 5:      Reserved
		 bit 6:      Reserved
		 bit 7:      Reserved
  */
  struct sockaddr_in sendaddr;  // second TCP sockt for scan data.
  int scan_sock;
  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  uint32_t pacer_period;
  int length;
  int dataCount = 9;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  if (frequency > 250000.) frequency = 250000; // 250kHz max
  if (frequency < 0) return false;

  if (frequency == 0) {
    pacer_period = 0;
  } else {
    pacer_period = rint((80.E6 / frequency) - 1.0);
    device_info->timeout = 1000*(1.0 + 1/frequency);
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AIN_SCAN_START;
  memcpy(&buffer[MSG_INDEX_DATA],   &count, 4);
  memcpy(&buffer[MSG_INDEX_DATA+4], &pacer_period, 4);
  memcpy(&buffer[MSG_INDEX_DATA+8], &options, 1);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  // create a  scan socket
  if ((scan_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    perror("AInScanStart_E16088: error creating TCP socket.");
    return false;
  }

  // set up the send address to the scan port
  memcpy(&sendaddr, &device_info->device.Address, sizeof(struct sockaddr_in));
  sendaddr.sin_port = htons(SCAN_PORT);

  // create a tcp connection
  if ((connect(scan_sock, (const struct sockaddr*) &sendaddr, sizeof(sendaddr))) < 0) {
    perror("AInScanStart_E1608: can not connect to device.");
    close(scan_sock);
    return false;
  }

  device_info->device.scan_sock = scan_sock;

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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

  // send ACK to start scan
  if (result != true) {
    printf("AInScanStart_E1608: Error sending start packet.  Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
    return false;
  }
  send(sock, 0x0, 1, 0);  // send a single byte;

  return result;
}

int AInScanRead_E1608(DeviceInfo_E1608 *device_info, uint32_t count, uint8_t nChan, uint16_t *data)
{
  int sock = device_info->device.scan_sock;
  int length = 0;
  int replyCount;
  int index = 0;
  int bytesReceived = 0;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return -1;
  }
 
  replyCount = count*nChan*2;

  do {
    bytesReceived = receiveMessage(sock, &data[index], replyCount - length, timeout);
    if (bytesReceived <= 0) {  // error
      printf("Error in AInScanRead: length = %d     replyCount = %d \n", length, replyCount);
      return -1;
    } else {
      length += bytesReceived;
      index += bytesReceived/2;
    }
  } while (length < replyCount);
  
  return length;
}

bool AInQueueR_E1608(DeviceInfo_E1608 *device_info)
{
  /* This command reads the analog input scan channel gain queue
      count       the number of queue entries, max 8
      channel_0   the channel number of the first queue element [0-11]
      range_0     the range number of the first queue element   [0-3]
      ...
      channel_n   the channel number of the last queue element [0-11]
      range_n     the range number of the last queue element   [0-3]
  */

  int sock = device_info->device.sock;
  unsigned char buffer[24];
  unsigned char replyBuffer[24];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AIN_QUEUE_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2*device_info->queue[0]+1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
	  memcpy(device_info->queue, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInQueueR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInQueueW_E1608(DeviceInfo_E1608 *device_info)
{
  /* This command writes the analog input scan channel gain queue for
     AInScan.  This command will result in an error response if an AIn
     scan is currently running.

      count       the number of queue entries, max 8
      channel_0   the channel number of the first queue element [0-11]
      range_0     the range number of the first queue element   [0-3]
      ...
      channel_n   the channel number of the last queue element [0-11]
      range_n     the range number of the last queue element   [0-3]
  */

  int sock = device_info->device.sock;
  unsigned char buffer[24];
  unsigned char replyBuffer[24];
  bool result = false;
  int length;
  int dataCount = 2*device_info->queue[0] + 1;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  if (dataCount > 17) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AIN_QUEUE_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  memcpy(&buffer[MSG_INDEX_DATA], device_info->queue, dataCount);
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in AInQueueW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanStop_E1608(DeviceInfo_E1608 *device_info, uint8_t close_socket)
{
  /* The command stops the analog input scan (if running).  It will clear the scan data FIFO.

     close_socket:   1 = close and reopen the data socket
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;
  int timeout = device_info->timeout;
  
  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AIN_SCAN_STOP;
  buffer[MSG_INDEX_DATA]           = close_socket;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in AInScanStop_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *        Analog Output                      *
 *********************************************/

bool AOutR_E1608(DeviceInfo_E1608 *device_info, uint16_t value[2])
{
  /* This command reads the value of the analog output channels

     value[0]   the current value for analog output channel 0
     value[1]   the current value for analog output channel 1
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_AOUT_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 4;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
	  memcpy((unsigned char *) value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AOutR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }

  value[0] = rint(value[0]*device_info->table_AOut[0].slope + device_info->table_AOut[0].intercept);
  value[1] = rint(value[1]*device_info->table_AOut[1].slope + device_info->table_AOut[1].intercept);
  return result;
}

bool AOut_E1608(DeviceInfo_E1608 *device_info, uint8_t channel, uint16_t value)
{
  /* The command writes the value of ananalog output channel.

     channel:  the channel to write (0-1)
     value     the value to write
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
  value = rint(value*device_info->table_AOut[channel].slope + device_info->table_AOut[channel].intercept);

  buffer[MSG_INDEX_COMMAND]        = CMD_AOUT_W;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) value;
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) (value >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in AOut_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}


/*********************************************
 *        Miscellaneous Commands             *
 *********************************************/

bool BlinkLED_E1608(DeviceInfo_E1608 *device_info, unsigned char count)
{
  // This comman will blink the device power LED "count" times.

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_BLINKLED;
  buffer[MSG_INDEX_DATA]           = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in BlinkLED_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Reset_E1608(DeviceInfo_E1608 *device_info)
{
  // The command resets the device.

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no data for this command
  int replyCount;
  int timeout = device_info->timeout;

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

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in Reset_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Status_E1608(DeviceInfo_E1608 *device_info, uint16_t *status)
{
  // The command reads the device status

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no input data for this command
  int replyCount;
  int timeout = device_info->timeout;

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

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 2;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in Status_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool NetworkConfig_E1608(DeviceInfo_E1608 *device_info, struct in_addr network[3])
{
  /* The command reads the current network configuration
     network[0] = ip_address
     network[1] = subnet mask
     network[2] = gateway_address
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  // no input data for this command
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_NETWORK_CONF;;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 12;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in NetworkConfig_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FirmwareUpgrade_E1608(DeviceInfo_E1608 *device_info)
{
  /* This command causes the device to reset and enter the bootloader
     for a firmware upgrade.  It erases a portion of the program memory so
     the device must have firmware downloaded through the bootloader before
     it can be used again.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 2;
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CMD_FIRMWARE;
  buffer[MSG_INDEX_DATA]           = 0xad;     // key
  buffer[MSG_INDEX_DATA+1]         = 0xad;     // key
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) (dataCount);
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in FirmwareUpgrade_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *          Counter  Commands                *
 *********************************************/

bool CounterR_E1608(DeviceInfo_E1608 *device_info, uint32_t *counter)
{
  // This command reads the event counter

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 4;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
          memcpy(counter, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in readCounter_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ResetCounter_E1608(DeviceInfo_E1608 *device_info)
{
  /* This command resets the event counter.  On a write, the
     counter will be reset to 0.
   */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 0;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  dataCount = 0;
  buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in ResetCounter_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *           Memory  Commands                *
 *********************************************/

bool CalMemoryR_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  // This command reads the nonvolatile calibration memory.  The cal memory is 512 bytes (address 0 - 0xff)

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0 || count > 512) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_CAL_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in CalMemoryR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool CalMemoryW_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile calibration memory.  The cal
     memory is 512 bytes (address 0 - 0xff) The cal memory should
     only be written during factory calibration and setup and has an
     additional lock mechanism to prevent inadvertent writes.  To
     enable srites to the cal memory, first write the unlock code
     0xAA55 to address 0x200.  Writes to the entire memory range are
     then possible.  Write any other value to address 0x200 to lock the
     memory after writing.  The amount of data to be written is
     inferred from the frame count - 2.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
  if (count > 512) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_CAL_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in CalMemoryW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryR_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile user memory.  The user memory is 1024 bytes (address 0 - 0x3ff)
     address: the start address for reading (0-0x3ff)
     count:   the number of bytes to read (max 512 due to protocol)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  if (count > 512) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in UserMemoryR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool UserMemoryW_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile user memory.  The user memory
     is 1024 bytes (address 0 - 0x3ff). The amount of data to be
     written is inferred from the frame count - 2.  The maximum that
     can be writtenin one transfer is 512 bytes.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
  if (count > 512) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in UserMemoryW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryR_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the nonvolatile settings memory.  The settings memory is 512 bytes (0x00-0x1FF)

     address: the start address for reading (0-0x1ff)
     count:   the number of bytes to read (max 512 due to protocol)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  if (count > 512) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in SettingMemoryR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool SettingsMemoryW_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command writes the nonvolatile settings memory.  The settings memory
     is 512 bytes (address 0 - 0x1ff). The amount of data to be
     written is inferred from the frame count - 2.  The maximum that
     can be writtenin one transfer is 512 bytes.The settings will be implemented
     after a device reset.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
  if (count > 512) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in SettingMemoryW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool BootloaderMemoryR_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads the bootloader stored in nonvolatile FLASH
     memory.  The bootloader is located in program FLASH memory in two
     physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
     code and 0x1FC00000 - 0x1FC01FFF for C startup code and
     interrupts.  Reads may be performed at any time.

     address: the start address for reading (see above)
     count:   the number of bytes to read (max 512)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount = 4;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }

  if (count > 512) {
    return false;
  }
 
  buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_R;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) count;
  buffer[MSG_INDEX_DATA+3]         = (unsigned char) (count >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = count;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in BootloaderMemoryR_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
  
bool BootloaderMemoryW_E1608(DeviceInfo_E1608 *device_info, uint16_t address, uint16_t count, uint8_t *data)
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
     using readBootloaderMemory_E1608().

     The writes are perfomred on 4-byte boundaries internally and it
     is recommended that the output data be sent in the same manner.
     The amount of data to be written is inferred frolm the frame
     count - 2.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[520];
  unsigned char replyBuffer[520];
  bool result = false;
  int length;
  int dataCount;  
  int replyCount;
  int timeout = device_info->timeout;

  if (sock < 0) {
    return false;
  }
  if (count > 512) {
    return false;
  }

  dataCount = count + 2;           // total size of the data frame
  buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_W;
  buffer[MSG_INDEX_DATA]           = (unsigned char) address;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) (address >> 8);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT_LOW]      = (unsigned char) dataCount;
  buffer[MSG_INDEX_COUNT_HIGH]     = (unsigned char) (dataCount >> 8);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0; // no input arguments
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, timeout)) > 0) {
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
    printf("Error in BootloaderMemoryW_E1608. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

void getMFGCAL_E1608(DeviceInfo_E1608 *device_info, struct tm *date)
{
  // get the manufacturers calibration data (timestamp) from the Calibration memory

  time_t time;
  uint16_t address;
  uint8_t data;

  // get the year (since 2000)
  address = 0x50;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_year = data + 100;

  // get the month
  address = 0x51;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_mon = data - 1;

  // get the day
  address = 0x52;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_mday = data;

  // get the hour
  address = 0x53;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_hour = data;

  // get the minute
  address = 0x54;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_min = data;

  // get the second
  address = 0x55;
  CalMemoryR_E1608(device_info, address, 1, &data);
  date->tm_sec = data;

  time = mktime(date);
  date = localtime(&time);
}

double volts_E1608(uint16_t value, uint8_t range)
{
  // Converts a raw value to volts for a given range
  double volt = 0.0;

  switch(range) {
    case BP_10V:   volt = (value - 0x8000)*10.0/32768.; break;
    case BP_5V:    volt = (value - 0x8000)*5.0/32768.; break;
    case BP_2V:    volt = (value - 0x8000)*2.0/32768.; break;
    case BP_1V:    volt = (value - 0x8000)*1.0/32768.; break;
    default: printf("Unknown range.\n"); break;
  }
  return volt;
}

uint16_t valueAOut_E1608(double volts)
{
  // convertrs volts to a 16 bit raw value for +/-10V output
  if (volts >= 10.0) {
    return 0xffff;
  } else  if (volts <= -10.00) {
    return 0x0;
  } else {
    return (uint16_t) (volts*32768/10. + 0x8000);
  }
}
