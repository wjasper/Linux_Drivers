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
#include "usb-ssr.h"

#define FS_DELAY (500)

/* reads digital port  */
uint8_t usbDIn_USBSSR(hid_device *hid, uint8_t port)
{
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  read_port.reportID = DIN;
  read_port.value = port;

  PMD_SendOutputReport(hid, (uint8_t *) &read_port, sizeof(read_port));
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);

  return read_port.value;
}

/* writes digital port */
void usbDOut_USBSSR(hid_device *hid, uint8_t port, uint8_t value) 
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.port = port;
  write_port.value = value;

  PMD_SendOutputReport(hid, (uint8_t *) &write_port, sizeof(write_port));
}

/* reads digital port bit */
uint8_t usbDBitIn_USBSSR(hid_device *hid, uint8_t port, uint8_t bit)
{
  struct read_bit_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.port = port;
  read_bit.value = bit;

  PMD_SendOutputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit));
  PMD_GetInputReport(hid, (uint8_t *) &read_bit, sizeof(read_bit), FS_DELAY);
  
  return read_bit.value;
}

/* writes digital port bit */
void usbDBitOut_USBSSR(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value)
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

  PMD_SendOutputReport(hid, (uint8_t *) &write_bit, sizeof(write_bit));
  return;
}

void usbReadMemory_USBSSR(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory)
{
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

int usbWriteMemory_USBSSR(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data)
{
  // Locations 0x00-0x7F are reserved for firmware and my not be written.
  int i;
  struct mem_write_report_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t count;
    uint8_t data[count];
  } arg;

  if (address <= 0x7f) return -1;
  if (count > 59) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte

  arg.count = count;
  for (i = 0; i < count; i++) {
    arg.data[i] = data[i];
  }
  PMD_SendOutputReport(hid, (uint8_t *) &arg, sizeof(arg));
  return 0;
}

/* blinks the LED of USB device */
void usbBlink_USBSSR(hid_device *hid)
{
    uint8_t reportID = BLINK_LED;

    PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

/* resets the USB device */
int usbReset_USBSSR(hid_device *hid)
{
  uint8_t reportID = RESET;

  return PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

uint16_t usbGetStatus_USBSSR(hid_device *hid)
{
  int nread;
  uint16_t status;
  int try = 10;

  /*
    Bit 0: Port A direction setting      (0 = output,    1 = input)
    Bit 1: Port B direction setting      (0 = output,    1 = input)
    Bit 2: Port C Low direction setting  (0 = output,    1 = input)
    Bit 3: Port C High direction setting (0 = output,    1 = input)
    Bit 4: Port A polarity setting       (0 = inverted,  1 = normal)
    Bit 5: Port B polarity setting       (0 = inverted,  1 = normal)
    Bit 6: Port C Low polarity setting   (0 = inverted,  1 = normal)
    Bit 7: Port C High polarity setting  (0 = inverted,  1 = normal)
    Bit 8: Port A pull-up setting        (0 = pull down, 1 = pull up)
    Bit 9: Port B pull-up setting        (0 = pull down, 1 = pull up)
    Bit 10: Port C Low  pull-up setting  (0 = pull down, 1 = pull up)
    Bit 11: Port C High pull-up setting  (0 = pull down, 1 = pull up)
  */
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
      printf("Error is getting status from USB-SSR\n.");
      break;
    }
  } while (statusReport.reportID != GET_STATUS && (nread != sizeof(statusReport)));
  memcpy(&status, statusReport.status, 2);
  return status;
}

void usbGetAll_USBSSR(hid_device *hid, uint8_t data[])
{
  /* Reads value from all digital I/O's
     uint8_t Port_A
     uint8_t Port_B
     uint8_t Port_C_Low
     uint8_t Port_C_High
  */

  uint8_t reportID = GET_ALL;
  struct get_all_t {
    uint8_t reportID;
    uint8_t values[4];
  } get_all;

  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
  PMD_GetInputReport(hid, (uint8_t *) &get_all, sizeof(get_all), FS_DELAY);
		     
  memcpy(data, get_all.values, 4);
  return;
}

void usbPrepareDownload_USBSSR(hid_device *hid)
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
  
  PMD_SendOutputReport(hid,  (uint8_t *)  &download, sizeof(download));
}

void usbWriteCode_USBSSR(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
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
  writecode.count = count;
  memcpy(writecode.data, data, count);      
  PMD_SendOutputReport(hid, (uint8_t *) &writecode, count+5);
}

int usbReadCode_USBSSR(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
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
  do {
    readCode.reportID = 0x0;
    bRead = PMD_GetInputReport(hid, (uint8_t *) &readCodeI, count+1, FS_DELAY);
  } while (readCodeI.reportID != READ_CODE && (bRead != count+1));
  memcpy(data, readCodeI.data, count);
  return bRead;
}

void usbWriteSerial_USBSSR(hid_device *hid, char* serial)
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
