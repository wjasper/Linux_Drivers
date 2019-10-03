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

#include "pmd.h"
#include "usb-tc-ai.h"

/* Commands and HID Report ID for the USB TC-AI */
// Digital I/O Commands
#define DCONFIG            (0x01) // Configure digital port
#define DCONFIG_BIT        (0x02) // Configure individual digital port bits
#define DIN                (0x03) // Read digital port
#define DOUT               (0x04) // Write digital port
#define DBIT_IN            (0x05) // Read digital port bit
#define DBIT_OUT           (0x06) // Write digital port bit

// Analog Input Commands
#define AIN                (0x18) // Read input channel
#define AIN_SCAN           (0x19) // Read multiple input channels

// Counter Commands
#define CINIT              (0x20) // Initialize event counter
#define CIN                (0x21) // Read event counter

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
#define CAL_CONFIG         (0x4D) // Configure Calibration MUX
#define CAL_STEPS          (0x4E) // Retreive number of steps in calibration sequence

// Code Update Commands
#define PREPARE_DOWNLOAD   (0x50) // Prepare for program memory download
#define WRITE_CODE         (0x51) // Write program memory
#define WRITE_SERIAL       (0x53) // Write a new serial number to device
#define READ_CODE          (0x55) // Read program memory

#define FS_DELAY 10000

/* configures digital port */
void usbDConfigPort_USBTC_AI(hid_device *hid, uint8_t direction)
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
void usbDConfigBit_USBTC_AI(hid_device *hid, uint8_t bit_num, uint8_t direction)
{
  struct config_bit_t {
    uint8_t reportID;
    uint8_t bit_num;      
    uint8_t direction;
  } config_bit;

  config_bit.reportID = DCONFIG_BIT;
  config_bit.bit_num = bit_num;
  config_bit.direction = direction;

  PMD_SendOutputReport(hid, (uint8_t *) &config_bit, sizeof(config_bit));
}

/* reads digital port  */
void usbDIn_USBTC_AI(hid_device *hid, uint8_t *value)
{
  uint8_t reportID = DIN;
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  PMD_SendOutputReport(hid, (uint8_t *) &reportID, 1);
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);
  *value = read_port.value;
  return;
}

/* reads digital bit  */
void usbDInBit_USBTC_AI(hid_device *hid, uint8_t bit_num, uint8_t* value)
{
  struct read_bit_t {
    uint8_t reportID;
    uint8_t value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit_num;

  PMD_SendOutputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit));
  PMD_GetInputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit), FS_DELAY);
  *value = read_bit.value;
  return;
}

/* writes digital port */
void usbDOut_USBTC_AI(hid_device *hid, uint8_t value)
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.value = value;

  PMD_SendOutputReport(hid, (uint8_t *) &write_port, sizeof(write_port));
}

/* writes digital bit  */
void usbDOutBit_USBTC_AI(hid_device *hid, uint8_t bit_num, uint8_t value)
{
  struct write_bit_t {
    uint8_t reportID;
    uint8_t bit_num;
    uint8_t value;
  } write_bit;

  write_bit.reportID = DBIT_OUT;
  write_bit.bit_num = bit_num;
  write_bit.value = value;

  PMD_SendOutputReport(hid, (uint8_t *) &write_bit, sizeof(write_bit));
  return;
}

/* Initialize event counter */
void usbInitCounter_USBTC_AI(hid_device *hid)
{
  uint8_t reportID = CINIT;
  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

/* Read the event counter */
uint32_t usbReadCounter_USBTC_AI(hid_device *hid)
{
  struct readCounter_t {
  uint8_t reportID;
  uint8_t value[4];
  } readCounter;

  uint32_t value;

  readCounter.reportID = CIN;
    
  PMD_SendOutputReport(hid, &readCounter.reportID, 1);
  PMD_GetInputReport(hid, (uint8_t*) &readCounter, sizeof(readCounter), FS_DELAY);

  memcpy(&value, &readCounter.value, sizeof(value));
  return value;
}

void usbAin_USBTC_AI(hid_device *hid, uint8_t channel, uint8_t units, float *value)
{
  /*
    This command reads the value from the specified input channel.  The return
    value is a 32-bit floating point value in the units configured for the
    channel.  CJC readings will always be in Celsius. Channels 0-3 are temperature
    input channels, channels 4-7 are voltage channels.

    On return:
      -8888.0 indicates open thermocouple on that channel
      -9000.0 indicates an initialization error
      -9999.0 indicates a floating point error
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

  tin.reportID = AIN;
  tin.channel = channel;
  tin.units = units;

  PMD_SendOutputReport(hid, (uint8_t*) &tin, sizeof(tin));
  PMD_GetInputReport(hid, (uint8_t*) &tin_val, sizeof(tin_val), FS_DELAY);
  memcpy(value, tin_val.value, 4);
}

void usbAinScan_USBTC_AI(hid_device *hid, uint8_t start_chan, uint8_t end_chan, uint8_t units, float value[])
{
  /* This command reads multiple input channels and sends the */
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

  tinScan.reportID = AIN_SCAN;
  tinScan.start_chan = start_chan;
  tinScan.end_chan = end_chan;
  tinScan.units = units;
  nchan = (end_chan - start_chan + 1);

  PMD_SendOutputReport(hid, (uint8_t*) &tinScan, sizeof(tinScan));
  PMD_GetInputReport(hid, (uint8_t*) &tinScan_val, nchan*sizeof(float)+1, FS_DELAY);
  memcpy(value, tinScan_val.value, nchan*sizeof(float));
}

/* blinks the LED of USB device */
void usbBlink_USBTC_AI(hid_device *hid)
{
  uint8_t reportID = BLINK_LED;

  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

int usbReset_USBTC_AI(hid_device *hid)
{
  uint8_t reportID = RESET;

  return PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

uint8_t usbGetStatus_USBTC_AI(hid_device *hid)
{
  /* 
    Status byte:
      bits [0:5] calibration progress:  0 = no calibration in progress or calibration complete
                                        other values indicate the progress of calibration, increasing sequentially.
      bit 6      calibratation type in progress
                                        0 = temperature calibration; 1 = voltage calibration
      bit 7      calibration error status
                                        0 = no error; 1 = error
  */

  struct statusReport_t {
  uint8_t reportID;
  uint8_t status;
  } statusReport;

  statusReport.reportID = GET_STATUS;
  PMD_SendOutputReport(hid, &statusReport.reportID, 1);
  PMD_GetInputReport(hid, (uint8_t*) &statusReport, sizeof(statusReport), FS_DELAY);
  return statusReport.status;
}

void usbReadMemory_USBTC_AI(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t *memory)
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

  if (count > 62 && type == 0) count = 62;  // 62 bytes max for main microcontroller
  if (count > 60 && type == 1) count = 60;  // 60 bytes max for isolated microcontroller

  readMemory.reportID = MEM_READ;
  readMemory.type = type;
  readMemory.address = address;
  readMemory.count = count;

  PMD_SendOutputReport(hid, (uint8_t *) &readMemory, sizeof(readMemory));
  PMD_GetInputReport(hid, (uint8_t *) &readMemoryI, count+1, FS_DELAY);
  memcpy(memory, readMemoryI.memory, count);
}

int usbWriteMemory_USBTC_AI(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t* data)
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

void usbSetItem_USBTC_AI(hid_device *hid, uint8_t item, uint8_t subitem, float fValue)
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
    printf("Error: usbSetItem_USBTC_AI  Item = %d too large.\n", item);
  }

  switch (subitem) {
    case FILTER_RATE:
    case CH_0_TC:
    case CH_1_TC:
    case CH_0_GAIN:
    case CH_1_GAIN:
    case CH_0_VOLT_CONN:
    case CH_1_VOLT_CONN:
      setItem.reportID = SET_ITEM;
      setItem.item = item;
      setItem.subitem = subitem;
      setItem.value = (uint8_t) fValue;
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItem));
      break;
    case VREF:
      setItemFloat.reportID = SET_ITEM;
      setItemFloat.item = item;
      setItemFloat.subitem = subitem;
      memcpy(setItemFloat.value, &fValue, 4);
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItemFloat));
      break;
    default:
      return;
  }
}

void usbSetItem_USBTEMP_AI(hid_device *hid, uint8_t item, uint8_t subitem, float fValue)
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
    printf("Error: usbSetItem_USBTC_AI  Item = %d too large.\n", item);
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
    case CH_0_VOLT_CONN:
    case CH_1_VOLT_CONN:
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
      PMD_SendOutputReport(hid, (uint8_t *) &setItem, sizeof(setItemFloat));
      break;
    default:
      return;
  }
}

int usbGetItem_USBTC_AI(hid_device *hid, uint8_t item, uint8_t subitem, void *value)
{
  uint8_t cmd[5];  // The returning data could be one byte or a 4 byte float.
  
  struct getItem_t {
    uint8_t reportID;
    uint8_t item;
    uint8_t subitem;
  } getItem;

  if (item > 3) {
    printf("Error: usbGetItem_USBTC_AI  Item = %d too large.\n", item);
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
      PMD_GetInputReport(hid, cmd, 5, FS_DELAY);
      memcpy(value, &cmd[1], 4);
      return 4;
      break;
    default:
      printf("Error usbGetItem_USBTEMP: subitem = %#x unknown\n", subitem);
      return -1;
  }
  return 0;
}


void usbCalibrate_USBTC_AI(hid_device *hid, uint8_t type)
{
  /*
    The command instructs the device to perform a calibration on all channels.  Used after
    reconfiguring the channel(s).  This may take up to several seconds, and the completion
    may be determined by polling the status with usbGetStatus.  Temperature readings will
    not be updated while the calibration is ongoing, but DIO operations may be performed.
    The device will not accept usbSetItem or usbMemWrite commands while calibration is
    being performed. Additionally, any Calibrate commands with type argument other than 255
    (calibrate abort) will be ignored.  After a calibration is aborted, usbGetStatus will
    indicate a calibration error until a new calibration is started.  Once voltage calibration
    has been completed successfully, the calibration path location in the isolated
    microcontroller's EEPROM will be  updatated to indicate which path was used for the most
    recent calibration.
  */
  uint8_t steps[2];
  int status;
  struct calibrate_t {
    uint8_t reportID;
    uint8_t type;  /* 0 = temperature calibration
		   1 = voltage calibration
		   255 = abort calibration */
    uint8_t path;  /* 0 = Channel Hi path (Channel Lo is referenced.)
		   1 = Channel Lo path (Channel Hi is referenced.) */
  } calibrate;

  calibrate.reportID = CALIBRATE;
  calibrate.type = type;
  calibrate.path = CHAN_HI_PATH;

  usbGetCalSteps_USBTC_AI(hid, steps);
  usbGetCalSteps_USBTC_AI(hid, steps);
  fflush(stdout);
  PMD_SendOutputReport(hid, (uint8_t *) &calibrate, sizeof(calibrate));
  do {
    sleep(1);
    status = usbGetStatus_USBTC_AI(hid);
    if ((status & 0x3f) == 0x0) break;
    if ((status & 0x40)) {
      printf("Calibrating voltage:  %d%% Complete\r", (status & 0x3f)*100/steps[1]);
    } else {
      printf("Calibrating temperature:  %d%% Complete\r", (status & 0x3f)*100/steps[0]);
    }
    fflush(stdout);
  } while ((status & 0x3f) != 0x0);
  printf("\n");
}

uint8_t  usbGetBurnoutStatus_USBTC_AI(hid_device *hid, uint8_t mask)
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

void usbPrepareDownload_USBTC_AI(hid_device *hid, uint8_t micro)
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
  
  PMD_SendOutputReport(hid, (uint8_t *) &download, sizeof(download));
}

void usbWriteCode_USBTC_AI(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
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

void usbWriteSerial_USBTC_AI(hid_device *hid, uint8_t serial[8])
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

int usbReadCode_USBTC_AI(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
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

void usbCalConfig_USBTC_AI(hid_device *hid, uint8_t gain, uint8_t polarity, uint8_t path)
{
  /* 
     This command configures the calibration mux for voltage channels
     4-7.  To use the calibration input, the channel must be
     configured for calibration using usbSetItem().
  */

  struct calConfig_t {
    uint8_t reportID;
    uint8_t gain;
    uint8_t polarity;   // 0 = positive, 1 = negative
    uint8_t path;       // 0 = chan hi path, 1 = chan lo path
  } calConfig;

  calConfig.reportID = CAL_CONFIG;
  calConfig.gain = gain;
  calConfig.polarity = polarity;
  calConfig.path = path;

  PMD_SendOutputReport(hid, (uint8_t *) &calConfig, sizeof(calConfig));
}

void usbGetCalSteps_USBTC_AI(hid_device *hid, uint8_t steps[2])
{
  /*
    This command returns the number of steps in the calibration sequence for both temperature
    and voltage calibration.  Both calibration routines start with a sequence number 1 and this
    command returns the last number in the seqence.
  */

  struct calSteps_t {
  uint8_t reportID;
  uint8_t steps[2];
  } calSteps;

  calSteps.reportID = CAL_STEPS;
  PMD_SendOutputReport(hid, &calSteps.reportID, 1);
  PMD_GetInputReport(hid, (uint8_t*) &calSteps, sizeof(calSteps), FS_DELAY);
  steps[0] = calSteps.steps[0];
  steps[1] = calSteps.steps[1];
}

void usbConfigAlarm_USBTC_AI(hid_device *hid, uint8_t number, uint8_t in_options, uint8_t out_options, float value_1, float value_2)
{
  /*
    This command configures the temperature alarm.  There are 8 temperature alarms available,
    corresponding to the 8 available DIO bits.  If an alarm is enabled, its associated DIO line
    will be configured as an output on power on and driven to their non-alarmed state.  The alarms
    are evaluated every measurement cycle.

    input options:
        bit         description
       ============================
        [0-2]   input channel (0-7)
           3    units 0 = temperature, 1 = raw
        [4-6]   threshold type:
                000 - alarm when reading > value_1
                001 - alarm when reading > value_1, reset when reading < value_2
                010 - alarm when reading < value_1
                011 - alarm when reading < value_1, reset when reading > value_2
                100 - alarm when reading < value_1 or > value_2
                101 - not used
                110 - not used
                111 - not used
           7    not used, must be 0

    output options:
        bit         description
       ============================
         0      1 = enable alarm, 0 = disable alarm
         1      0 = active low alarm, 1 = active high alarm
      [2-7]     not used

  */
  
  struct configAlarm_t {
    uint8_t reportID;
    uint8_t number;       // alarm number to configure
    uint8_t in_options;   // input options
    uint8_t out_options;  // output options
    uint8_t value_1[4];   // threashold value 1 for the alarm
    uint8_t value_2[4];   // threashold value 2 for the alarm
  } configAlarm;

  configAlarm.reportID = CONFIG_ALARM;
  configAlarm.number = number;
  configAlarm.in_options = in_options & 0x7f;
  configAlarm.out_options = out_options & 0x1;
  memcpy(&configAlarm.value_1, &value_1, sizeof(value_1));
  memcpy(&configAlarm.value_2, &value_2, sizeof(value_2));

  PMD_SendOutputReport(hid, (uint8_t *) &configAlarm, sizeof(configAlarm));
}

void usbGetAlarmConfig_USBTC_AI(hid_device *hid, uint8_t number, uint8_t *in_options, uint8_t *out_options, float *value_1, float *value_2)
{
  struct arg_t {
    uint8_t reportID;
    uint8_t number;
  } arg;

  struct alarmConfig_t {
    uint8_t reportID;
    uint8_t in_options;   // input options
    uint8_t out_options;  // output options
    uint8_t value_1[4];   // threashold value 1 for the alarm
    uint8_t value_2[4];   // threashold value 2 for the alarm
  } alarmConfig;

  arg.reportID = GET_ALARM_CONFIG;
  arg.number = number;
  alarmConfig.reportID = GET_ALARM_CONFIG;

  PMD_SendOutputReport(hid, (uint8_t*) &arg, sizeof(arg));
  PMD_GetInputReport(hid, (uint8_t*) &alarmConfig, sizeof(alarmConfig), FS_DELAY);

  *in_options = alarmConfig.in_options;
  *out_options = alarmConfig.out_options;
  memcpy((char *) value_1, &alarmConfig.value_1, 4);  // 32 bit float
  memcpy((char *) value_2, &alarmConfig.value_2, 4);  // 32 bit float
}
