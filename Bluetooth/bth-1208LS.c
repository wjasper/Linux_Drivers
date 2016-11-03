/*
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include "bth-1208LS.h"

void BuildGainTable_DE_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /* Builds a lookup table of differential mode calibration
     coefficents to translate values into voltages: The calibration
     coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j, k;
  uint16_t address = 0x0;

  for (i = 0; i < NGAINS; i++ ) {
    for (j = 0; j < NCHAN_DE; j++) {
      for (k = 0; k < 2; k++) {
	CalMemoryR_BTH1208LS(device_info, address, 4, (uint8_t *) &device_info->table_DE_AIN[i][j][k]);
	address += 4;
      }
    }
  }
}

void BuildGainTable_SE_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /* Builds a lookup table of differential mode calibration
     coefficents to translate values into voltages: The calibration
     coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j;
  uint16_t address = 0x100;

  for (i = 0; i < NCHAN_SE; i++) {
    for (j = 0; j < 2; j++) {
      CalMemoryR_BTH1208LS(device_info, address, 4, (uint8_t *) &device_info->table_SE_AIN[i][j]);
      address += 4;
    }
  }
}

void CalDate_BTH1208LS(DeviceInfo_BTH1208LS *device_info, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x200;  // beginning of MFG Calibration date
  time_t time;

  CalMemoryR_BTH1208LS(device_info, address, sizeof(calDate), (uint8_t *) &calDate);
  date->tm_year = calDate.year + 100;
  date->tm_mon = calDate.month - 1;
  date->tm_mday = calDate.day;
  date->tm_hour = calDate.hour;
  date->tm_min = calDate.minute;
  date->tm_sec = calDate.second;
  time = mktime(date);
  date = localtime(&time);
}

double volts_BTH1208LS(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_20V:   volt = (value - 0x800)*20.0/2048.; break;
    case BP_10V:   volt = (value - 0x800)*10.0/2048.; break;
    case BP_5V:    volt = (value - 0x800)*5.0/2048.;  break;
    case BP_4V:    volt = (value - 0x800)*4.0/2048.;  break;
    case BP_2_5V:  volt = (value - 0x800)*2.5/2048.;  break;
    case BP_2V:    volt = (value - 0x800)*2.0/2048.;  break;
    case BP_1_25V: volt = (value - 0x800)*1.25/2048.; break;
    case BP_1V:    volt = (value - 0x800)*1.0/2048.;  break;
    case UP_2_5V:  volt = value*2.5/4096.0;           break; // analog output
    default:       printf("Unknown range.\n");        break;
  }
  return volt;
}

/*********************************************
 *        Digital I/O  Commands              *
 *********************************************/

bool DIn_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t *value)
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

  buffer[MSG_INDEX_COMMAND]        = DIN_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  *value = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DIn_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOutR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t *value)
{
  /* This command reads the DIO output latch value. The factory power
     on default is all 1 (pins are floating.) Writing a 0 to a bit
     drives it low, writing a 1 allows it to float.
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
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 1;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  *value = replyBuffer[MSG_INDEX_DATA];
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DOutR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool DOut_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t value)
{
  /* This command reads the DIO output latch value. The factory power
     on default is all 1 (pins are floating.) Writing a 0 to a bit
     drives it low, writing a 1 allows it to float.
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
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)             &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in DOut_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
bool AIn_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t channel, uint8_t mode, uint8_t range, uint16_t *value)
{
    /* This command reads the value of an analog input channel.  This
     command will result in a bus stall if an AInScan is currently
     running.  The range parameter is ignored if the mode is specified
     as single ended.

     channel: the channel to read (0-7)
     mode:    the input mode 0 - single ended, 1 - differential
     range:   the input range for the channel (0-7)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = AIN;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_DATA+1]         = mode;
  buffer[MSG_INDEX_DATA+2]         = range;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy((unsigned char *) value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AIn_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanStart_BTH1208LS(DeviceInfo_BTH1208LS *device_info,uint32_t count, uint32_t retrig_count,
		  double frequency, uint8_t channels, uint8_t options)
{
  /* This command starts an analog input scan. This command will
     respond with 0 if an AIn scan is currently running. The device
     will not generate an internal pacer faster than 50 kHz.

     The pacer rate is set by an internal 32-bit timer running at a
     base rate of 40 MHz. The timer is controlled by
     pacer_period. This value is the period of the scan and the A/Ds
     are clocked at this rate. A pulse will be output at the SYNC pin
     at every pacer_period interval if SYNC is configured as an
     output. The equation for calculating pacer_period is:
   
           pacer_period = [40 MHz / (sample frequency)] - 1 

     If pacer_period is set to 0 the device does not generate an A/D
     clock. It uses the SYNC pin as an input and the user must provide
     the pacer source. The A/Ds acquire data on every rising edge of
     SYNC; the maximum allowable input frequency is 50 kHz.  

    The scan will not begin until this command is sent and any trigger
    conditions are met. Data will be acquired until an overrun occurs,
    the specified count is reached, or an AInScanStop command is sent.

    The data is read using the AInScanSendData command. The data will
    be in the format:

     lowchannel sample 0 : lowchannel + 1 sample 0 : … : hichannel sample 0 
     lowchannel sample 1 : lowchannel + 1 sample 1 : … : hichannel sample 1
      … 
     lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n 

     If the host does not receive the data in a timely manner (due to
     a communications error, etc.) it can issue the AInScanResendData
     command. The device will resend the last packet that was
     transmitted. The device does not remove the sent data from its
     FIFO until a new AInScanSendData command is received. This keeps
     the data available for a resend. The host must send an
     AInScanStop command at the end of a finite scan to let the device
     know that the final packet was received successfully and allow
     the scan to end.

     The external trigger may be used to start the scan. If enabled,
     the device will wait until the appropriate trigger condition is
     detected then begin sampling data at the specified rate. No data
     will be available until the trigger is detected. In retrigger
     mode the trigger will be automatically rearmed and the scan will
     restart after retrig_count samples have been acquired. The count
     parameter specifies the total number of samples to acquire and
     should be >= retrig_count. Specifying 0 for count causes
     continuous retrigger scans.

     Overruns are indicated in the status field of the AInScanDataRead
     command response. The host may also read the status to verify.


     count:         the total number of samples to acquire, 0 for continuous scan
     retrig_count:  the number of samples to acquire for each trigger in retrigger mode
     pacer_period:  the pacer timer period (0 for external clock)
     channels:      bit field that selects the channels in the scan, upper 4 bits ignored in differential mode. 
        ---------------------------------------------------------
        | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
        ---------------------------------------------------------
        | Ch7  |  Ch6 | Ch5  | Ch4  | Ch3  | Ch2  | Ch1  | Ch 0 |
        ---------------------------------------------------------

	options:  bit field that controls scan options
	    bit 0: Reserved
	    bit 1: 1 = differential mode, 0 = single ended mode
	    bits 2-4: Trigger setting:
	      0: no trigger
  	      1: Edge / rising
	      2: Edge / falling
	      3: Level / high
	      4: Level / low
            bit 5: 1 = retrigger mode, 0 = normal trigger mode
	    bit 6: Reserved
	    bit 7: Reserved
  */

  int sock = device_info->device.sock;
  unsigned char buffer[32];
  unsigned char replyBuffer[32];
  bool result = false;
  int length;
  int dataCount = 14;
  int replyCount;
  uint32_t pacer_period;

  if (sock < 0) {
    return false;
  }

  if (frequency > 50000.) frequency = 50000.;
  if (frequency > 0.) {
    pacer_period = rint((40.E6 / frequency) - 1);
  } else {
    pacer_period = 0;
  }

  buffer[MSG_INDEX_COMMAND]        = AIN_SCAN_START;
  memcpy(&buffer[MSG_INDEX_DATA], &count, 4);
  memcpy(&buffer[MSG_INDEX_DATA+4], &retrig_count, 4);
  memcpy(&buffer[MSG_INDEX_DATA+8], &pacer_period, 4);
  buffer[MSG_INDEX_DATA+12]        = channels;
  buffer[MSG_INDEX_DATA+13]        = options;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInScanStart_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanSendData_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t count, uint16_t *data)
{
  /* This command reads data from the scan FIFO. The device will
     return all data currently in the FIFO up to the maximum amount
     allowed in a frame (255 bytes / 127 samples). Incrementing frame
     IDs are recommended in order to make use of the AInScanResendData
     command on an error.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[272];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  replyCount = count*2;
  if (replyCount > 255) replyCount = 255;  // 255 the maximum number of bytes transmitted in a frame
  
  buffer[MSG_INDEX_COMMAND]        = AIN_SCAN_SEND_DATA;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 5000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInScanSendData_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanResendData_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t count, uint8_t nChan, uint16_t *data)
{
  /* This command resends the previous scan data that matches the
     specified frame ID. This is used when the host doesn’t receive a
     response from the device after sending AInScanSendData. The
     device only buffers one most recently sent response and frame ID;
     if the frame ID matches the device assumes that its response was
     not received by the host and resends the previous data. If the
     specified frame ID doesn’t match the most recent frame ID the
     device assumes it did not receive the last AInScanSendData
     command and sends new data from the FIFO.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[272];
  bool result = false;
  int length;
  int dataCount = 0;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  replyCount = count*nChan*2;
  if (replyCount > 255) replyCount = 255;    // 255 the maximum number of samples transmitted in a frame
  device_info->device.frameID--;             // decrement to the previous frame that needs resending.
  
  buffer[MSG_INDEX_COMMAND]        = AIN_SCAN_RESEND_DATA;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInScanResendData_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanStop_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /* This command stops the analog input scan (if running)  */

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

  buffer[MSG_INDEX_COMMAND]        = AIN_SCAN_STOP;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	replyCount = 0;
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInScanStop_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInConfigR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t ranges[4])
{
  /* This command reads the analog input range configuration for
     AInScan in differential mode.
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

  buffer[MSG_INDEX_COMMAND]        = AIN_CONFIG_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(ranges, &replyBuffer[MSG_INDEX_DATA], 4);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInConfigR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInConfigW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t ranges[4])
{
  /* This command writes the analog input range configuration for
     AInScan in differential mode.  This command will result in an
     error respone if an AIn scan is currently running.
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

  buffer[MSG_INDEX_COMMAND]        = AIN_CONFIG_W;
  memcpy(&buffer[MSG_INDEX_DATA], ranges, 4);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	replyCount = 0;
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInConfigW_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AInScanClearFIFO_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /* This command clears the scan data FIFO */

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

  buffer[MSG_INDEX_COMMAND]        = AIN_SCAN_CLEAR_FIFO;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 0;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	replyCount = 0;
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AInScanClearFIFO_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *        Analog Output                      *
 *********************************************/

bool AOutR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t value[2])
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

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = AOUT_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy((unsigned char *) value, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AOutR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool AOut_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t channel, uint16_t value)
{
  /* The command writes the value of ananalog output channel.

     channel:  the channel to write (0-1)
     value     the value to write   (0-4095)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = AOUT_W;
  buffer[MSG_INDEX_DATA]           = channel;
  buffer[MSG_INDEX_DATA+1]         = (unsigned char) value;
  buffer[MSG_INDEX_DATA+2]         = (unsigned char) (value >> 8);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in AOut_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/***********************************************
 *            Counter/Timer                    *
 ***********************************************/

bool Counter_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t *counter)
{
  /* The command reads the event counter. */

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

  buffer[MSG_INDEX_COMMAND]        = COUNTER_R;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
          memcpy(counter, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Counter_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool ResetCounter_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
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

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = COUNTER_W;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in ResetCounter_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
bool CalMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data)
{
  /* This command reads the nonvolatile calibration memory.  The cal
     memory is 768 bytes (address 0 - 0x2FF)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[512];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 255) {
    printf("CalMemoryR_BTH1208LS: count must be less than 256\n");
    return false;
  }

  if (address > 0x2ff) {
    printf("CalMemoryR_BTH1208LS: address must be between 0x0-0x2FF\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = CAL_MEMORY_R;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  buffer[MSG_INDEX_DATA+2]         = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in CalMemoryR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data)
{
  /* This command reads the nonvolatile user memory.  The user
     memory is 256 bytes (address 0 - 0xFF)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[512];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 255) {
    printf("UserMemoryR_BTH1208LS: count must be less than 256\n");
    return false;
  }

  if (address > 0xff) {
    printf("UserMemoryR_BTH1208LS: address must be between 0x0-0xFF\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = USER_MEMORY_R;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  buffer[MSG_INDEX_DATA+2]         = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in UserMemoryR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool UserMemoryW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data)
{
  /* This command writes the nonvolatile user memory.  The user memory
     is 256 bytes (address 0 - 0xFF).  The amount of data to be
     written is inferred from the frame count - 2;
  */

  int sock = device_info->device.sock;
  unsigned char buffer[512];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = count + 2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 255) {
    printf("UserMemoryW_BTH1208LS: count must be less than 256\n");
    return false;
  }

  if (address > 0xff) {
    printf("UserMemoryW_BTH1208LS: address must be between 0x0-0xFF\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = USER_MEMORY_W;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in UserMemoryW_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data)
{
  /* This command reads the nonvolatile settings memory.  The settings
     memory is 1024 bytes (address 0 - 0x3FF)
  */

  int sock = device_info->device.sock;
  unsigned char buffer[16];
  unsigned char replyBuffer[1048];
  bool result = false;
  int length;
  int dataCount = 3;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 255) {
    printf("SettingsMemoryR_BTH1208LS: count must be less than 256\n");
    return false;
  }

  if (address > 0x3ff) {
    printf("SettingsMemoryR_BTH1208LS: address must be between 0x0-0x3FF\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = SETTINGS_MEMORY_R;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  buffer[MSG_INDEX_DATA+2]         = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(data, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in SettingsMemoryR_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool SettingsMemoryW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data)
{
  /* This command writes the nonvolatile settings memory.  The
     settings memory is 1024 bytes (address 0 - 0x3FF).  The amount of
     data to be written is inferred from the frame count - 2. The
     settings will be implemented immediately.
  */

  int sock = device_info->device.sock;
  unsigned char buffer[1048];
  unsigned char replyBuffer[16];
  bool result = false;
  int length;
  int dataCount = count + 2;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  if (count > 255) {
    printf("SettingsMemoryW_BTH1208LS: count must be less than 256\n");
    return false;
  }

  if (address > 0x3ff) {
    printf("SettingsMemoryW_BTH1208LS: address must be between 0x0-0x3FF\n");
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = SETTINGS_MEMORY_W;
  memcpy(&buffer[MSG_INDEX_DATA], &address, 2);
  memcpy(&buffer[MSG_INDEX_DATA+2], data, count);
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in SettingsMemoryW_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

/*********************************************
 *        Miscellaneous Commands             *
 *********************************************/

bool BlinkLED_BTH1208LS(DeviceInfo_BTH1208LS *device_info, unsigned char count)
{
  /* This comman will blink the device power LED "count" times. */

  int sock = device_info->device.sock;
  unsigned char buffer[64];
  unsigned char replyBuffer[64];
  bool result = false;
  int length;
  int dataCount = 1;
  int replyCount;

  if (sock < 0) {
    return false;
  }

  buffer[MSG_INDEX_COMMAND]        = BLINK_LED;
  buffer[MSG_INDEX_DATA]           = count;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in BlinkLED_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool GetSerialNumber_BTH1208LS(DeviceInfo_BTH1208LS *device_info, char serial[9])
{
  /*
    This commands reads the device serial number.  The serial number
    consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001").
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

  buffer[MSG_INDEX_COMMAND]        = SERIAL;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
  buffer[MSG_INDEX_DATA+dataCount] = (unsigned char) 0xff - calcChecksum(buffer, MSG_INDEX_DATA+dataCount);

  if (send(sock, buffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount, 0) > 0) {
    replyCount = 8;
    if ((length = receiveMessage(sock, replyBuffer, MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount, 1000)) > 0) {
      // check response
      if (length == MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) {
	if ((replyBuffer[MSG_INDEX_START] == buffer[0])                                  &&
            (replyBuffer[MSG_INDEX_COMMAND] == (buffer[MSG_INDEX_COMMAND] | MSG_REPLY))  &&
	    (replyBuffer[MSG_INDEX_FRAME] == buffer[2])                                  &&
	    (replyBuffer[MSG_INDEX_STATUS] == MSG_SUCCESS)                               &&
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  strncpy(serial, (char *) &replyBuffer[MSG_INDEX_DATA], replyCount);
	  serial[8] = '\0';
	}
      }
    }
  }

  if (result == false) {
    printf("Error in GetSerialNumber_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Reset_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /*
    This command resets the device
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

  buffer[MSG_INDEX_COMMAND]        = RESET;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Reset_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Status_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *status)
{
  /*
    This command reads the device status.

      status: bit 0:     Reserved
              bit 1:     1 = AIn scan running
              bit 2:     1 = AIn scan overrun
              bits 3-7:  Reserved
              bits 8-10: Charger status
                           0 = no battery
                           1 = fast charge
                           2 = maintenance charge
                           3 = fault (not charging)
	                   4 = disabled, operating on battery power
              bits 11-15:   Reserved
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
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
          memcpy(status, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Status_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool Ping_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *status)
{
  /*
    This command checks communications with the device. Note that
    repetetive use of this command is not recommended for maximum
    battery life.
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

  buffer[MSG_INDEX_COMMAND]        = PING;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	}
      }
    }
  }

  if (result == false) {
    printf("Error in Ping_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool FirmwareVersion_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *version)
{
  /*
    This command reads the firmware version.  The version consistes of
    16 bits in hexadecimal BCD notation, i.e. version 2.15 would be
    0x0215.
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

  buffer[MSG_INDEX_COMMAND]        = FIRMWARE_VERSION;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
          memcpy(version, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in FirmwareVersion_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool RadioFirmwareVersion_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *version)
{
  /*
    This command reads the radio firmware version.  The version consistes of
    16 bits in hexadecimal BCD notation, i.e. version 6.15 would be
    0x0615.
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

  buffer[MSG_INDEX_COMMAND]        = RADIO_FIRMWARE_VERSION;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
          memcpy(version, &replyBuffer[MSG_INDEX_DATA], 2);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in RadioFirmwareVersion_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}

bool BatteryVoltage_BTH1208LS(DeviceInfo_BTH1208LS *device_info)
{
  /*
    This command reads the battery voltage. The value is returned in millivolts.
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

  buffer[MSG_INDEX_COMMAND]        = BATTERY_VOLTAGE;
  buffer[MSG_INDEX_START]          = MSG_START;
  buffer[MSG_INDEX_FRAME]          = device_info->device.frameID++;  // increment frame ID with every send
  buffer[MSG_INDEX_STATUS]         = 0;
  buffer[MSG_INDEX_COUNT]          = (unsigned char) (dataCount);
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
	    (replyBuffer[MSG_INDEX_COUNT] == (unsigned char) replyCount)                 &&
	    (replyBuffer[MSG_INDEX_DATA+replyCount] + calcChecksum(replyBuffer, MSG_HEADER_SIZE+replyCount) == 0xff)) {
	  result = true;
	  memcpy(&device_info->voltage, &replyBuffer[MSG_INDEX_DATA], replyCount);
	}
      }
    }
  }

  if (result == false) {
    printf("Error in GetSerialNumber_BTH1208LS. Status = %d\n", replyBuffer[MSG_INDEX_STATUS]);
  }
  return result;
}
