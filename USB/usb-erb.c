/*
 *
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include "usb-erb.h"

#define FS_DELAY (500)
/* reads digital port  */
uint8_t usbDIn_USBERB(hid_device *hid, uint8_t port)
{
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  read_port.reportID = DIN;
  read_port.value = port;

  PMD_SendOutputReport(hid,  (uint8_t*) &read_port, sizeof(read_port));
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);

  return read_port.value;
}

/* writes digital port */
void usbDOut_USBERB(hid_device *hid, uint8_t port, uint8_t value) 
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.port = port;
  write_port.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_port, sizeof(write_port));
}

/* reads digital port bit */
uint8_t usbDBitIn_USBERB(hid_device *hid, uint8_t port, uint8_t bit) 
{
  struct read_bit_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.port = port;
  read_bit.value = bit;

  PMD_SendOutputReport(hid, (uint8_t*) &read_bit, sizeof(read_bit));
  PMD_GetInputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit), FS_DELAY);
  
  return read_bit.value;
}

/* writes digital port bit */
void usbDBitOut_USBERB(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value)
{
  struct write_bit_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t bit_num;
    uint8_t value;
  } write_bit;

  write_bit.reportID = DBIT_OUT;
  write_bit.port = port;
  write_bit.bit_num = bit;
  write_bit.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_bit, sizeof(write_bit));
  return;
}

void usbReadMemory_USBERB(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory)
{
  /*
    The command reads data from the configuration memory (EEPROM).
    All of the memory may be read.  The USB hub chip EEPROM may be
    read, and its address range is 0x0400 - 0x04FF.
  */

    struct arg_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t type;
    uint8_t count;
  } arg;

  if ( count > 62 ) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.count = count;

  PMD_SendOutputReport(hid, (uint8_t *) &arg, sizeof(arg));
  PMD_GetInputReport(hid, (uint8_t *) &memory, count, FS_DELAY);

}
int usbWriteMemory_USBERB(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data)
{
  /*
    This command writes to non-volatile EEPROM memory on the device.  The non-volatile
    memory is used to store calibration coefficients, system information, and user data.  Locations
    0x000-0x07F are reserved for firmware use and my not be written.  This device has external
    EEPROM for the USB hub chip configuration, and the values for that EEPROM may be written
    through this command.  The address range for the hub EEPROM is 0x0400-0x04FF.
  */
  int i;
  struct mem_write_report_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t count;
    uint8_t data[count];
  } arg;

  if ( address <= 0x7f ) return -1;
  if ( count > 59 ) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte

  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }
  PMD_SendOutputReport(hid, (uint8_t *) &arg, sizeof(arg));
  return 0;
}

/* blinks the LED of USB device */
void usbBlink_USBERB(hid_device *hid)
{
    uint8_t reportID = BLINK_LED;

    PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

/* resets the USB device */
int usbReset_USBERB(hid_device *hid)
{
  uint8_t reportID = RESET;

  return PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

uint16_t usbGetStatus_USBERB(hid_device *hid)
{
  /*
    Bit 0: Port A polarity setting       (0 = inverted,  1 = normal)  (N/A on ERB08)
    Bit 1: Port B polarity setting       (0 = inverted,  1 = normal)  (N/A on ERB08)
    Bit 2: Port C Low polarity setting   (0 = inverted,  1 = normal)
    Bit 3: Port C High polarity setting  (0 = inverted,  1 = normal)
    Bit 4: Port A pull-up setting        (0 = pull down, 1 = pull up) (N/A on ERB08)
    Bit 5: Port B pull-up setting        (0 = pull down, 1 = pull up) (N/A on ERB08)
    Bit 6: Port C Low pull-up setting    (0 = pull down, 1 = pull up)
    Bit 7: Port C High pull-up setting   (0 = pull down, 1 = pull up)
  */

  int nread;
  uint16_t status;
  int try = 10;
  struct statusReport_t {
    uint8_t reportID;
    uint8_t status[2];
  } statusReport;

  statusReport.reportID = GET_STATUS;
    
  PMD_SendOutputReport(hid, &statusReport.reportID, 1);
  do {
    statusReport.reportID = 0x0;
    nread = PMD_GetInputReport(hid, (uint8_t *) &statusReport, sizeof(statusReport), FS_DELAY);

    if (try-- == 0) {
      printf("Error is getting status from USB-ERB\n.");
      break;
    }
  } while (statusReport.reportID != GET_STATUS && (nread != sizeof(statusReport)));
  memcpy(&status, statusReport.status, 2);
  return status;
}

float usbGetTemp_USBERB(hid_device *hid)
{
  int16_t temp;
  uint8_t reportID = GET_TEMP;
  struct get_temp_t { 
    uint8_t reportID;
    uint8_t temperature[2];
  } get_temp;

  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
  PMD_GetInputReport(hid, (uint8_t *) &get_temp, sizeof(get_temp), FS_DELAY);
  memcpy(&temp, get_temp.temperature, 2);
  return (temp*0.1);
}

void usbPrepareDownload_USBERB(hid_device *hid)
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
  } download;

  download.reportID = PREPARE_DOWNLOAD;
  download.unlock_code = 0xad;
  
  PMD_SendOutputReport(hid, (uint8_t *) &download, sizeof(download));
}

void usbWriteCode_USBERB(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  /*
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading
    a new hex file, so it supports memory ranges that may be found in the hex file.  

    The address ranges are:

    0x000000 - 0x007AFF:  Microcontroller FLASH program memory
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

  if (count > 32) count = 32;               // 32 byte max 
  writecode.reportID = WRITE_CODE;
  memcpy(writecode.address, &address, 3);   // 24 bit address
  writecode.count = count;                  // the number of byes of data (max 32)
  memcpy(writecode.data, data, count);      // the program data
  PMD_SendOutputReport(hid, (uint8_t *) &writecode, count+5);
}

int usbReadCode_USBERB(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  struct readCode_t {
    uint8_t reportID;
    uint8_t address[3];    // 24 bit start address for the read
    uint8_t count;         // the number of bytes to read (62 byte max)
  } readCode;

  struct readCodeI_t {
    uint8_t reportID;
    uint8_t data[62];
  } readCodeI;

  int bRead;  // bytes read

  if ( count > 62 ) count = 62;  

  readCode.reportID = READ_CODE;
  memcpy(readCode.address, &address, 3);   // 24 bit address
  readCode.count = count;
  PMD_SendOutputReport(hid, (uint8_t *) &readCode, sizeof(readCode));
  do {
    readCode.reportID = 0x0;
    bRead = PMD_GetInputReport(hid, (uint8_t *) &readCodeI, count+1, FS_DELAY);
  } while (readCodeI.reportID != READ_CODE && (bRead != count+1));
  memcpy(data, readCodeI.data, count);
  return bRead;
}

void usbWriteSerial_USBERB(hid_device *hid, uint8_t serial[8])
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
