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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-5200.h"

/* Commands and Report ID for USB 5201 and USB 5203 */
// Digital I/O Commands
#define DCONFIG            (0x01) // Configure digital port
#define DCONFIG_BIT        (0x02) // Configure individual digital port bits
#define DIN                (0x03) // Read digital port
#define DOUT               (0x04) // Write digital port
#define DBIT_IN            (0x05) // Read digital port bit
#define DBIT_OUT           (0x06) // Write digital port bit

#define TIN                (0x18) // Read input channel
#define TIN_SCAN           (0x19) // Read multiple input channels

// Memory Commands
#define MEM_READ           (0x30) // Read Memory
#define MEM_WRITE          (0x31) // Write Memory

// Miscellaneous Commands
#define BLINK_LED          (0x40) // Causes LED to blink
#define RESET              (0x41) // Reset USB interface
#define GET_STATUS         (0x44) // Get device status
#define SET_ITEM           (0x49) // Set a configuration item
#define GET_ITEM           (0x4A) // Get a configuration item
#define CALIBRATE          (0x4B) // Perform a channel calibration
#define GET_BURNOUT_STATUS (0x4C) // Get thermocouple burnout detection status

// Code Update Commands
#define PREPARE_DOWNLOAD   (0x50) // Prepare for program memory download
#define WRITE_CODE         (0x51) // Write program memory
#define WRITE_SERIAL       (0x53) // Write a new serial number to device
#define READ_CODE          (0x55) // Read program memory

// Data Logging Commands (only valid for USB-5201 and USB-5203)
#define FORMAT_CARD        (0x60) // Format memory card for logging use
#define READ_CLOCK         (0x61) // Read time from device
#define SET_CLOCK          (0x62) // Set device time
#define GET_FIRST_FILE     (0x63) // Get info on first file on volume
#define GET_NEXT_FILE      (0x64) // Get info on next file
#define GET_FILE_INFO      (0x65) // Get info on specified file
#define GET_DISK_INFO      (0x66) // Get information on memory card
#define READ_FILE          (0x67) // Read file from volume
#define DELETE_FILE        (0x68) // Delete File from volume
#define CONFIGURE_LOGGING  (0x69) // Configure data logging feature
#define GET_LOGGING_CONFIG (0x6a) // Read data logging configuration
#define GET_FILE_HEADER    (0x6d) // Read log file header
#define READ_FILE_ACK      (0x6e) // Acknowledge ReadFile data
#define READ_FILE_ABORT    (0x6f) // Abort ReadFile

// Alarm Commands
#define CONFIGURE_ALARM    (0x6b) // Configure temperature alarm
#define GET_ALARM_CONFIG   (0x6c) // Read current temperature alarm configuration

#define FS_DELAY 10000

/* configures digital port */
void usbDConfigPort_USB5200(hid_device *hid, uint8_t direction)
{
  struct config_port_t {
    uint8_t reportID;
    uint8_t direction;
  } config_port;

  config_port.reportID = DCONFIG;
  config_port.direction = direction;

  PMD_SendOutputReport(hid, (uint8_t*) &config_port, sizeof(config_port));
}

/* configures digital bit */
void usbDConfigBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t direction)
{
    struct config_bit_t {
    uint8_t reportID;
    uint8_t bit_num;      
    uint8_t direction;
  } config_bit;

  config_bit.reportID = DCONFIG_BIT;
  config_bit.bit_num = bit_num;
  config_bit.direction = direction;

  PMD_SendOutputReport(hid, (uint8_t*) &config_bit, sizeof(config_bit));
}

/* reads digital port  */
void usbDIn_USB5200(hid_device *hid, uint8_t* value)
{
  uint8_t reportID = DIN;
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  PMD_SendOutputReport(hid, &reportID, 1);
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);
  *value = read_port.value;
  return;
}

/* reads digital bit  */
void usbDInBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t* value)
{
  struct read_bit_t {
    uint8_t reportID;
    uint8_t value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit_num;

  PMD_SendOutputReport(hid, (uint8_t*) &read_bit, 2);
  PMD_GetInputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit), FS_DELAY);
  *value = read_bit.value;
  return;
}

/* writes digital port */
void usbDOut_USB5200(hid_device *hid, uint8_t value)
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_port, sizeof(write_port));
}

/* writes digital bit  */
void usbDOutBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t value)
{
  struct write_bit_t {
    uint8_t reportID;
    uint8_t bit_num;
    uint8_t value;
  } write_bit;

  write_bit.reportID = DBIT_OUT;
  write_bit.bit_num = bit_num;
  write_bit.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_bit, sizeof(write_bit));
  return;
}

void usbTin_USB5200(hid_device *hid, uint8_t channel, uint8_t units, float *value)
{

  /*
    This command reads the value from the specified input channel.  The return
    value is a 32-bit floating point value in the units configured from the
    channel.  CJC readings will always be in Celsius.
  */
  
  struct tin_t {
    uint8_t reportID;
    uint8_t channel;  // 0 - 7
    uint8_t units;    // 0 - temperature, 1 - raw measurement
  } tin;

  struct tin_val_t {
    uint8_t reportID;
    uint8_t value[4];
  } tin_val;

  tin.reportID = TIN;
  tin.channel = channel;
  tin.units = units;

  PMD_SendOutputReport(hid, (uint8_t*) &tin, sizeof(tin));
  PMD_GetInputReport(hid, (uint8_t*) &tin_val, sizeof(tin_val), FS_DELAY);
  memcpy(value, tin_val.value, 4);
}

void usbTinScan_USB5200(hid_device *hid, uint8_t start_chan, uint8_t end_chan, uint8_t units, float value[])
{
  int nchan;
  struct tinScan_t {
    uint8_t reportID;
    uint8_t start_chan;  // the first channel to return 0-7 
    uint8_t end_chan;    // the last channel to return 0-7
    uint8_t units;       // 0 - temperature, 1 - raw measurement
  } tinScan;

  struct tinScan_val_t {
    uint8_t reportID;
    uint8_t value[32];  // maximum number of measurements 
  } tinScan_val;

  tinScan.reportID = TIN_SCAN;
  tinScan.start_chan = start_chan;
  tinScan.end_chan = end_chan;
  tinScan.units = units;
  nchan = (end_chan - start_chan + 1);

  PMD_SendOutputReport(hid, (uint8_t*) &tinScan, sizeof(tinScan));
  PMD_GetInputReport(hid, (uint8_t*) &tinScan_val, nchan*sizeof(float)+1, FS_DELAY);
  memcpy(value, tinScan_val.value, nchan*sizeof(float));
}

/* blinks the LED of USB device */
void usbBlink_USB5200(hid_device *hid)
{
  uint8_t reportID = BLINK_LED;

  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

int usbReset_USB5200(hid_device *hid)
{
  uint8_t reportID = RESET;

  return PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

uint8_t usbGetStatus_USB5200(hid_device *hid)
{
  struct statusReport_t {
  uint8_t reportID;
  uint8_t status;
  } statusReport;

  statusReport.reportID = GET_STATUS;
  PMD_SendOutputReport(hid, &statusReport.reportID, 1);
  PMD_GetInputReport(hid, (uint8_t*) &statusReport, sizeof(statusReport), FS_DELAY);
  return statusReport.status;
}

void usbReadMemory_USB5200(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t *memory)
{
  struct readMemory_t {
    uint8_t reportID;
    uint16_t address;
    uint8_t type;     // 0 = main microcontroller  1 = isolated microcontroller
    uint8_t count;
  } readMemory;

  struct readMemoryI_t {
    uint8_t reportID;
    uint8_t memory[62];
  } readMemoryI;

  if ( count > 62 && type == 0) count = 62;  // 62 bytes max for main microcontroller
  if ( count > 60 && type == 1) count = 60;  // 60 bytes max for isolated microcontroller

  readMemory.reportID = MEM_READ;
  readMemory.type = type;
  readMemory.address = address;
  readMemory.count = count;

  PMD_SendOutputReport(hid, (uint8_t *) &readMemory, sizeof(readMemory));
  PMD_GetInputReport(hid, (uint8_t *) &readMemoryI, count+1, FS_DELAY);
  memcpy(memory, readMemoryI.memory, count);
}

int usbWriteMemory_USB5200(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t* data)
{
  // Locations 0x00-0xFF are available on the main microcontroller
  int i;

  struct writeMemory_t {
    uint8_t  reportID;
    uint16_t address;   // start address for the write (0x00-0xFF)
    uint8_t  type;      // 0 = main microcontroller  1 = isolated microcontroller
    uint8_t  count;     // number of bytes to write (59 max)
    uint8_t  data[count];
  } writeMemory;

  if (address > 0xff) return -1;
  if (count > 59) count = 59;

  writeMemory.reportID = MEM_WRITE;
  writeMemory.address = address;
  writeMemory.count = count;
  writeMemory.type = type;

  for ( i = 0; i < count; i++ ) {
    writeMemory.data[i] = data[i];
  }
  PMD_SendOutputReport(hid, (uint8_t *) &writeMemory, sizeof(writeMemory));
  return 0;
}

void usbSetItem_USB5201(hid_device *hid, uint8_t item, uint8_t subitem, uint32_t value)
{
  /*
    This command sets the values of the configuration items.  Because of byte alignment
    issues and the fact that some items take unsigned char and others take floats, two
    structures are used.
  */
    
  struct setItem_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
    uint8_t value;
  } setItem;

  struct setItemFloat_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
    uint8_t value[4];
  } setItemFloat;

  switch (subitem) {
    case FILTER_RATE:
    case CH_0_TC:
    case CH_1_TC:
    case CH_0_GAIN:
    case CH_1_GAIN:
      setItem.reportID = SET_ITEM;
      setItem.item = item;
      setItem.subitem = subitem;
      setItem.value = (uint8_t) value;
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItem));
      break;
    case VREF:
      setItemFloat.reportID = SET_ITEM;
      setItemFloat.item = item;
      setItemFloat.subitem = subitem;
      memcpy(setItemFloat.value, &value, 4);
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItemFloat));
      break;
    default:
      return;
  }
}

void usbSetItem_USB5203(hid_device *hid, uint8_t item, uint8_t subitem, float fValue)
{
  /*
    This command sets the values of the configuration items.  Because of byte alignment
    issues and the fact that some items take unsigned char and others take floats, two
    structures are used.
  */

  struct setItem_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
    uint8_t value;
  } setItem;

  struct setItemFloat_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
    uint8_t value[4];
  } setItemFloat;

  if (item > 3) {
    printf("Error: usbSetItem_USBTEMP  Item = %d too large.\n", item);
  }

  switch (subitem) {
    case SENSOR_TYPE:
    case CONNECTION_TYPE:
    case FILTER_RATE:
    case EXCITATION:
    case CH_0_TC:
    case CH_1_TC:
    case CH_0_GAIN:
    case CH_1_GAIN:
      setItem.reportID = SET_ITEM;
      setItem.item = item;
      setItem.subitem = subitem;
      setItem.value = (uint8_t) fValue;
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItem));
      break;
    case VREF:
    case I_value_0:
    case I_value_1:
    case I_value_2:
    case V_value_0:
    case V_value_1:
    case V_value_2:
    case CH_0_COEF_0:
    case CH_1_COEF_0:
    case CH_0_COEF_1:
    case CH_1_COEF_1:
    case CH_0_COEF_2:
    case CH_1_COEF_2:
    case CH_0_COEF_3:
    case CH_1_COEF_3:
      setItemFloat.reportID = SET_ITEM;
      setItemFloat.item = item;
      setItemFloat.subitem = subitem;
      memcpy(setItemFloat.value, &fValue, 4);
      PMD_SendOutputReport(hid, (uint8_t *) &setItemFloat, sizeof(setItemFloat));
      break;
  default:
    printf("Error usbSetItem_USB5203: subitem = %#x unknown\n", subitem);
    return;
    break;
  }
}

int usbGetItem_USB5200(hid_device *hid, uint8_t item, uint8_t subitem, void* value)
{
  uint8_t cmd[5];  // The returning data could be one byte or a 4 byte float.
  
  struct getItem_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
  } getItem;

  if (item > 3) {
    printf("Error: usbGetItem_USBTC  Item = %d too large.\n", item);
  }

  getItem.reportID = GET_ITEM;
  getItem.item = item;
  getItem.subitem = subitem;

  PMD_SendOutputReport(hid, (uint8_t *) &getItem, sizeof(getItem));

  switch (subitem) {
    case SENSOR_TYPE:
    case CONNECTION_TYPE:
    case FILTER_RATE:
    case EXCITATION:
    case CH_0_TC:
    case CH_1_TC:
    case CH_0_GAIN:
    case CH_1_GAIN:
      PMD_GetInputReport(hid, cmd, 2, FS_DELAY);
      memcpy(value, &cmd[1], 1);  // one byte value
      return 1;
      break;
    case VREF:
    case I_value_0:
    case I_value_1:
    case I_value_2:
    case V_value_0:
    case V_value_1:
    case V_value_2:
    case CH_0_COEF_0:
    case CH_1_COEF_0:
    case CH_0_COEF_1:
    case CH_1_COEF_1:
    case CH_0_COEF_2:
    case CH_1_COEF_2:
    case CH_0_COEF_3:
    case CH_1_COEF_3:
      PMD_GetInputReport(hid,  cmd, 5, FS_DELAY);
      memcpy(value, &cmd[1], 4);
      return 4;
      break;
    default:
      printf("Error usbGetItem_USBTEMP: subitem = %#x unknown\n", subitem);
      return -1;
  }
  return 0;
}

void usbCalibrate_USB5200(hid_device *hid)
{
  uint8_t reportID = CALIBRATE;

  printf("Calibrating.  Please wait ");
  PMD_SendOutputReport(hid, &reportID, 1);
  do {
    sleep(1);
    printf(".");
  } while ((usbGetStatus_USB5200(hid) & 0x1) == 1);
  printf("\n");
}

uint8_t  usbGetBurnoutStatus_USB5200(hid_device *hid, uint8_t mask)
{
  /*
     This command returns the status of burnout detection for thermocouple channels.  The
     return value is a bitmap indicating the burnout detection status for all 8 channels.
     Individual bits will be set if an open circuit has been detected on that channel.  The
     bits will be cleared after the call using the mask that is passed as a parameter. If
     a bit is set, the corresponding bit in the status will be left at its current value.
  */
 
  struct burnoutStatus_t {
    uint8_t reportID;
    uint8_t status;
  } burnoutStatus;

  burnoutStatus.reportID = GET_BURNOUT_STATUS;
  burnoutStatus.status = mask;

  PMD_SendOutputReport(hid, (uint8_t *) &burnoutStatus, sizeof(burnoutStatus));
  PMD_GetInputReport  (hid, (uint8_t *) &burnoutStatus, sizeof(burnoutStatus), FS_DELAY);

  return (burnoutStatus.status);
}

void usbPrepareDownload_USB5200(hid_device *hid, uint8_t micro)
{
  /*
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with usbWriteCode.  If not in
    code update mode, any usbWriteCode will be ignored.  A usbReset command must be issued at
    the end of the code download in order to return the device to operation with the new code.
  */

  struct download_t {
    uint8_t reportID;
    uint8_t unlock_code;
    uint8_t micro;
  } download;

  download.reportID = PREPARE_DOWNLOAD;
  download.unlock_code = 0xad;
  download.micro = micro; // 0 = main, 1 = isolated
  
  PMD_SendOutputReport(hid, (uint8_t *)  &download, sizeof(download));
}

void usbWriteCode_USB5200(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  /*
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading
    a nex hex file, so it supports memory ranges that may be found in the hex file.  The
    microcontroller that is being written to is selected with the "Prepare Download" command.

    The address ranges are:

    0x000000 - 0x0075FF:  Microcontroller FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)
    0xF00000 - 0xF03FFF:  EEPROM memory

    FLASH program memory: The device must receive data in 64-byte segments that begin
    on a 64-byte boundary.  The data is sent in messages containing 32 bytes.  count
    must always equal 32.

    Other memory: Any number of bytes up to the maximum (32) may be sent.
    
  */

  struct writecode_t {
    uint8_t reportID;
    uint8_t address[3];
    uint8_t count;
    uint8_t data[32];
  } writecode;

  writecode.reportID = WRITE_CODE;
  memcpy(writecode.address, &address, 3);   // 24 bit address
  writecode.count = count;
  memcpy(writecode.data, data, count);      
  PMD_SendOutputReport(hid, (uint8_t *) &writecode, count+5);
}

int usbReadCode_USB5200(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  struct readCode_t {
    uint8_t reportID;
    uint8_t address[3];
    uint8_t count;
  } readCode;

  struct readCodeI_t {
    uint8_t reportID;
    uint8_t data[62];
  } readCodeI;

  int bRead;  // bytes read

  if (count > 62) count = 62;  

  readCode.reportID = READ_CODE;
  memcpy(readCode.address, &address, 3);   // 24 bit address
  readCode.count = count;
  PMD_SendOutputReport(hid, (uint8_t *) &readCode, sizeof(readCode));

  bRead = PMD_GetInputReport(hid, (uint8_t *) &readCodeI, count+1, FS_DELAY);
  memcpy(data, readCodeI.data, count);
  return bRead;
}

void usbWriteSerial_USB5200(hid_device *hid, uint8_t serial[8])
{
  // Note: The new serial number will be programmed but not used until hardware reset.
  struct writeSerialNumber_t {
    uint8_t reportID;
    uint8_t serial[8];
  } writeSerialNumber;

  writeSerialNumber.reportID = WRITE_SERIAL;
  memcpy(writeSerialNumber.serial, serial, 8);
  
  PMD_SendOutputReport(hid, (uint8_t*) &writeSerialNumber, sizeof(writeSerialNumber));
}

void usbConfigureLogging_USB5200(hid_device *hid, uint8_t options, uint8_t channels, uint8_t units, uint32_t seconds,
				 uint16_t filenumber, deviceTime starttime)
{
  struct log_t {
    uint8_t reportID;
    uint8_t options;
    uint8_t channels;
    uint8_t units;
    uint8_t seconds[4];
    uint8_t filenumber[2];
    uint8_t starttime[6];
  } log;

  log.reportID = CONFIGURE_LOGGING;
  log.options = options;
  log.channels = channels;
  log.units = units;
  memcpy(&log.seconds, &seconds, 4);
  memcpy(&log.filenumber, &filenumber, 2);
  memcpy(&log.starttime, &starttime, 6);
  
  if (usbGetStatus_USB5200(hid) & DAUGHTERBOARD_PRESENT) {
    PMD_SendOutputReport(hid, (uint8_t *) &log, sizeof(log));
  } else {
    printf("usbConfigureLogging_USB5200: No daughterboard card present.\n");
  }
}

void usbGetLoggingConfig_USB5200(hid_device *hid, uint8_t *options, uint8_t *channels, uint8_t *units, uint32_t *seconds,
				 uint16_t *filenumber, deviceTime *starttime)
{
  struct log_t {
    uint8_t reportID;
    uint8_t options;
    uint8_t channels;
    uint8_t units;
    uint8_t seconds[4];
    uint8_t filenumber[2];
    uint8_t starttime[6];
  } log;

  log.reportID = GET_LOGGING_CONFIG;
  
  if (usbGetStatus_USB5200(hid) & DAUGHTERBOARD_PRESENT) {
    PMD_SendOutputReport(hid, &log.reportID, 1);
    PMD_GetInputReport(hid, (uint8_t *) &log, sizeof(log), FS_DELAY);
    
    *channels = log.channels;
    *units = log.units;
    memcpy(seconds, &log.seconds, 4);
    memcpy(filenumber, &log.filenumber, 2);
    memcpy(starttime, &log.starttime, 6);
  } else {
    printf("usbGetLoggingConfig_USB5200: No daughterboard card present.\n");
  }
}

void usbGetDeviceTime_USB5200(hid_device *hid, deviceTime *date)
{
  /*
    seconds   seconds in BCD range 0-59 (eg. 0x25 is 25 seconds)
    minutes   minutes in BCD range 0-59
    hours     hours in BCD   range 0-23 (eg. 0x22 is 2200 or 10 p.m.)
    day       day in BCD     range 1-31
    month     month in BCD   range 1-12
    year      year - 2000 in BCD range 0-99  (represents 2000-2099).
    time_zone time zone correction factor to be added to hours for local time.
  */

  struct deviceTime_t {
    uint8_t reportID;
    uint8_t date[7];
  } dtime;

  dtime.reportID = READ_CLOCK;

  if (usbGetStatus_USB5200(hid) & DAUGHTERBOARD_PRESENT) {
    PMD_SendOutputReport(hid, &dtime.reportID, 1);
    PMD_GetInputReport(hid, (uint8_t *)&dtime, sizeof(dtime), FS_DELAY);
    memcpy(date, &dtime.date, sizeof(deviceTime));
  } else {
    printf("usbGetDeviceTime_USB5200: No daughterboard card present.\n");
  }
}

void usbSetDeviceTime_USB5200(hid_device *hid, deviceTime *date)
{
  /*
    seconds   seconds in BCD range 0-59; (eg. 0x25 is 25 seconds)
    minutes   minutes in BCD range 0-59
    hours     hours in BCD   range 0-23 (eg. 0x22 is 2200 or 10 p.m.)
    day       day in BCD     range 1-31
    month     month in BCD   range 1-12
    year      year - 2000 in BCD range 0-99  (represents 2000-2099).
    time_zone time zone correction factor to be added to hours for local time.
  */

  struct deviceTime_t {
    uint8_t reportID;
    uint8_t date[7];
  } dtime;

  dtime.reportID = SET_CLOCK;
  memcpy(dtime.date, date, sizeof(deviceTime));

  if (usbGetStatus_USB5200(hid) & DAUGHTERBOARD_PRESENT) {
    PMD_SendOutputReport(hid, (uint8_t *) date, sizeof(dtime));
  } else {
    printf("usbSetDeviceTime_USB5200: No daughterboard card present.\n");
  }
}

void usbFormatCard_USB5200(hid_device *hid)
{
  uint8_t reportID = FORMAT_CARD;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &reportID, 1);
  } else {
    printf("usbFormatCard_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbGetFirstFile_USB5200(hid_device *hid, dir_entry *entry)
{
  struct dirEntry_t {
    uint8_t reportID;
    dir_entry entry;
  } dirEntry;

  dirEntry.reportID = GET_FIRST_FILE;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &dirEntry.reportID, 1);
    PMD_GetInputReport(hid, (uint8_t *)&dirEntry, sizeof(dirEntry), FS_DELAY);
    memcpy(entry, &dirEntry.entry, sizeof(dir_entry));
  } else {
    printf("usbGetFirstFile_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbGetNextFile_USB5200(hid_device *hid, dir_entry *entry)
{
  struct dirEntry_t {
    uint8_t reportID;
    dir_entry entry;
  } dirEntry;

  dirEntry.reportID = GET_NEXT_FILE;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &dirEntry.reportID, 1);
    PMD_GetInputReport(hid, (uint8_t *)&dirEntry, sizeof(dir_entry), FS_DELAY);
    memcpy(entry, &dirEntry.entry, sizeof(dir_entry));
  } else {
    printf("usbGetNextFile_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbGetFileInfo_USB5200(hid_device *hid, char *filename, dir_entry *dirEntry)
{
  struct getFileInfoIn_t {
    uint8_t reportID;
    char filename[11];
  } getFileInfoIn;

  struct getFileInfoOut_t {
    uint8_t reportID;
    dir_entry entry;
  } getFileInfoOut;

  getFileInfoIn.reportID = GET_FILE_INFO;
  getFileInfoOut.reportID = GET_FILE_INFO;
  strncpy(getFileInfoIn.filename, filename, 11);

  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, (uint8_t *) &getFileInfoIn, strlen(getFileInfoIn.filename)+1);
    PMD_GetInputReport(hid, (uint8_t *) &getFileInfoOut, sizeof(getFileInfoOut), FS_DELAY);
  } else {
    printf("usbGetFileInfo_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbGetDiskInfo_USB5200(hid_device *hid, disk_info *diskInfo)
{
  struct getDiskInfo_t {
    uint8_t reportID;
    disk_info diskInfo;
  } getDiskInfo;

  getDiskInfo.reportID = GET_DISK_INFO;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &getDiskInfo.reportID, 1);
    PMD_GetInputReport(hid, (uint8_t *) &getDiskInfo, sizeof(diskInfo), FS_DELAY);
  } else {
    printf("usbGetDiskInfo_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbDeleteFile_USB5200(hid_device *hid, char *filename)
{
  struct deleteFile_t {
    uint8_t reportID;
    char filename[11];  // null-terminated string representing the DOS 8.3 filename
  } deleteFile;

  deleteFile.reportID = DELETE_FILE;
  strncpy(deleteFile.filename, filename, 11);
    
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, (uint8_t *) &deleteFile,  strlen(deleteFile.filename)+1);
  } else {
    printf("usbDeleteFile_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbGetFileHeader_USB5200(hid_device *hid, char *filename, file_header *header)
{
  struct fileHeaderIn_t {
    uint8_t reportID;
    char filename[11];
  } fileHeaderIn;

  struct fileHeaderOut_t {
    uint8_t reportID;
    file_header header;
  } fileHeaderOut;

  fileHeaderIn.reportID = GET_FILE_HEADER;
  fileHeaderOut.reportID = GET_FILE_HEADER;
  strncpy(fileHeaderIn.filename, filename, 11);
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, (uint8_t *) &fileHeaderIn, strlen(filename)+1);
    PMD_GetInputReport(hid, (uint8_t *) &fileHeaderOut, sizeof(fileHeaderOut), FS_DELAY);
    memcpy(header, &fileHeaderOut.header, sizeof(file_header));
  } else {
    printf("usbGetFileHeader_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbReadFileAck_USB5200(hid_device *hid)
{
  uint8_t reportID = READ_FILE_ACK;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &reportID, 1);
  } else {
    printf("usbReadFileAck_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbReadFileAbort_USB5200(hid_device *hid)
{
  uint8_t reportID = READ_FILE_ABORT;
  
  if (usbGetStatus_USB5200(hid) & (DAUGHTERBOARD_PRESENT | MEMORYCARD_PRESENT)) {
    PMD_SendOutputReport(hid, &reportID, 1);
  } else {
    printf("usbReadFileAbort_USB5200: No daughterboard or memory card present.\n");
  }
}

void usbConfigureAlarm_USB5200(hid_device *hid, uint8_t number, uint8_t in_options, uint8_t out_options, float value_1, float value_2)
{
  struct alarm_t {
    uint8_t reportID;
    uint8_t number;      // alarm number to configure (0-7)
    uint8_t in_options;  /* bit field that controls various options for the input
			  bits 0-2:  input channel (0-7)
			  bit 3:     units (0=temperature, 1 = raw reading)
			  bits 4-6:   threshold type
			    000 - alarm when reading > value_1
  			    001 - alarm when reading > value_1, reset when reading < value_2
			    010 - alarm when reading < value_1
			    011 - alarm when reading < value_1, reset when reading > value_2
			    100 - alarm when reading < value_1 or reading > value_2
			    101 - not used
			    110 - not used
			    111 - not used
			    bit 7: not used, must be 0
		          */
    uint8_t out_options; /* bit field that controls various options for the output
			    bit 0:    1 - enable alarm, 0 - disable alarm
			    bit 1:    alarm level, 0 - active low alarm, 1 - active high alarm
			   bits 2-7: not used
		        */
    uint8_t value_1[4];  // threshold value 1 for the alarm (float in Celsius)
    uint8_t value_2[4];  // threshold value 2 for the alarm (float in Celsius)
  } alarm;

  alarm.reportID = CONFIGURE_ALARM;
  alarm.number = number;
  alarm.in_options = in_options & 0x7f;;
  alarm.out_options = out_options & 0x3;
  memcpy(&alarm.value_1, &value_1, 4);
  memcpy(&alarm.value_2, &value_2, 4);
  PMD_SendOutputReport(hid, (uint8_t *)&alarm, sizeof(alarm));
}

void usbGetAlarmConfig_USB5200(hid_device *hid, uint8_t number, uint8_t *in_options, uint8_t *out_options, float *value_1, float *value_2)
{

  struct alarmIn_t {
    uint8_t reportID;
    uint8_t number;
  } alarmIn;
  
  struct alarmOut_t {
    uint8_t reportID;
    uint8_t in_options;  /* bit field that controls various options for the input
			    bits 0-2:  input channel (0-7)
			    bit 3:     units (0=temperature, 1 = raw reading)
			    bits 4-6:   threshold type
			      000 - alarm when reading > value_1
			      001 - alarm when reading > value_1, reset when reading < value_2
			      010 - alarm when reading < value_1
			      011 - alarm when reading < value_1, reset when reading > value_2
			      100 - alarm when reading < value_1 or reading > value_2
			      101 - not used
			      110 - not used
			      111 - not used
			    bit 7: not used, must be 0
		          */
    uint8_t out_options; /* bit field that controls various options for the output
			    bit 0:    1 - enable alarm, 0 - disable alarm
			    bit 1:    alarm level, 0 - active low alarm, 1 - active high alarm
			    bits 2-7: not used
		          */
    uint8_t value_1[4];  // threshold value 1 for the alarm (float in Celsius)
    uint8_t value_2[4];  // threshold value 2 for the alarm (float in Celsius)
  } alarm;

  alarmIn.reportID = GET_ALARM_CONFIG;
  alarmIn.number = number & 0x7;
  alarm.reportID = GET_ALARM_CONFIG;

  PMD_SendOutputReport(hid, (uint8_t *)&alarmIn, 2);
  PMD_GetInputReport(hid, (uint8_t *)&alarm, sizeof(alarm), FS_DELAY);

  *in_options = alarm.in_options;
  *out_options = alarm.out_options;
  memcpy(value_1, &alarm.value_1, 4);
  memcpy(value_2, &alarm.value_2, 4);
}
