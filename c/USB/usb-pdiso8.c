/*
 *
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
#include "usb-pdiso8.h"

/* Commands and Codes for USB PDISO8 HID reports */
#define DIN              (0x03) // Read digital port
#define DOUT             (0x04) // Write digital port
#define DBIT_IN          (0x05) // Read Digital port bit
#define DBIT_OUT         (0x06) // Write Digital port bit

#define MEM_READ         (0x30) // Read Memory
#define MEM_WRITE        (0x31) // Write Memory
#define BLINK_LED        (0x40) // Causes LED to blink
#define WRITE_SERIAL     (0x53) // Write new serial number to device

#define FS_DELAY (500)

/* reads digital port  */
uint8_t usbDIn_USBPDISO8(hid_device *hid, uint8_t port)
{
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  read_port.reportID = DIN;
  read_port.value = port;

  PMD_SendOutputReport(hid, (uint8_t*) &read_port, sizeof(read_port));
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);

  return read_port.value;
}

/* writes digital port */
void usbDOut_USBPDISO8(hid_device *hid, uint8_t port, uint8_t value) 
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
uint8_t usbDBitIn_USBPDISO8(hid_device *hid, uint8_t port, uint8_t bit)
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
void usbDBitOut_USBPDISO8(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value)
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

void usbReadMemory_USBPDISO8(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory)
{
  /*
    This command reads data from the configuration memory (EEPROM). All
    memory may be read.  Max number of bytes read is 4.
  */

    struct arg_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t type;
    uint8_t count;
  } arg;

  if (count > 4) count = 4;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.count = count;


  PMD_SendOutputReport(hid, (uint8_t *) &arg, sizeof(arg));
  PMD_GetInputReport(hid, (uint8_t *) &memory, count, FS_DELAY);
}

int usbWriteMemory_USBPDISO8(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data)
{
  // Locations 0x00-0xF are reserved for firmware and my not be written.
  int i;
  struct mem_write_report_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t count;
    uint8_t data[count];
  } arg;

  if (address <= 0xf) return -1;
  if (count > 4) count = 4;

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
void usbBlink_USBPDISO8(hid_device *hid)
{
    uint8_t reportID = BLINK_LED;

    PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

void usbWriteSerial_USBPDISO8(hid_device *hid, uint8_t serial[8])
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
