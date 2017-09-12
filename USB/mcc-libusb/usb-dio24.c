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
#include "usb-dio24.h"

/* Commands and Report codes for the USB-DIO24 */
#define DCONFIG     (0x0D)  // Configure digital port
#define DIN         (0x00)  // Read digital port
#define DOUT        (0x01)  // Write digital port
#define DBIT_IN     (0x02)  // Read Digital port bit
#define DBIT_OUT    (0x03)  // Write Digital port bit

#define CINIT       (0x05)  // Initialize counter
#define CIN         (0x04)  // Read Counter

#define MEM_READ    (0x09)  // Read Memory
#define MEM_WRITE   (0x0A)  // Write Memory

#define BLINK_LED   (0x0B)  // Causes LED to blink
#define RESET       (0x11)  // Reset USB interface
#define SET_ID      (0x0C)  // Set the user ID
#define GET_ID      (0x0F)  // Get the user ID

#define LS_DELAY (3000)

static uint8_t PortC = 0;

/* configures digital port */
void usbDConfigPort_USBDIO24(hid_device *hid, uint8_t port, uint8_t direction)
{
  struct report_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t direction;
    uint8_t pad[5];
  } report;

  report.reportID = DCONFIG;
  report.port = port;
  report.direction = direction;

  PMD_SendOutputReport(hid,  (uint8_t*) &report, sizeof(report));
}

/* reads digital port  */
void usbDIn_USBDIO24(hid_device *hid, uint8_t port, uint8_t *din_value)
{
  uint8_t cmd[8];
  
  cmd[0] = 0;     // since DIN = 0, pad and extra zero in front of it.  issue in HIDAPI
  cmd[1] = DIN;
  cmd[2] = port;
  
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, (uint8_t *) cmd, sizeof(cmd), LS_DELAY);

  *din_value = cmd[0];

  if (port == DIO_PORTC_HI)  *din_value >>= 4;
  if (port == DIO_PORTC_LOW) *din_value &= 0xf;
}

/* writes digital port */
void usbDOut_USBDIO24(hid_device *hid, uint8_t port, uint8_t value) 
{
  uint8_t cmd[8];
  
  cmd[0] = DOUT;
  cmd[1] = port;
  cmd[2] = value;

  if (port == DIO_PORTC_LOW) {
    PortC &= (0xf0);
    PortC |= (value & 0xf);
    cmd[2] = PortC;
  }

  if (port == DIO_PORTC_HI) {
    PortC &= (0x0f);
    PortC |= (value << 0x4);
    cmd[2] = PortC;
  }

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

/* reads digital port bit */
uint8_t usbDBitIn_USBDIO24(hid_device *hid, uint8_t port, uint8_t bit) 
{
  uint8_t cmd[8];

  cmd[0] = DBIT_IN;
  cmd[1] = port;
  cmd[2] = bit;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, (uint8_t *) cmd, sizeof(cmd), LS_DELAY);

  return cmd[0];
}

/* writes digital port bit */
void usbDBitOut_USBDIO24(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value)
{
  uint8_t cmd[8];
  
  cmd[0] = DBIT_OUT;
  cmd[1] = port;
  cmd[2] = bit;
  cmd[3] = value;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

/* Initialize the counter */
void usbInitCounter_USBDIO24(hid_device *hid)
{
  uint8_t cmd[8];
  
  cmd[0] = CINIT;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint32_t usbReadCounter_USBDIO24(hid_device *hid)
{
  uint8_t cmd[8];
  uint32_t value;

  cmd[0] = CIN;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, (uint8_t  *) &value, sizeof(value), LS_DELAY);
  return value;
}

void usbReadMemory_USBDIO24(hid_device *hid, uint16_t address, uint8_t *data, uint8_t count)
{
  uint8_t cmd[8];

  if (count > 8) {
    printf("usbReadMemory_USBDIO24: count must be less than 8\n");
    return;
  }
 
  cmd[0] = MEM_READ;
  cmd[1] = (uint8_t) (address & 0xff);  // low byte
  cmd[2] = (uint8_t) (address >> 0x8);  // high byte
  cmd[3] = count;
  
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);

  memcpy(data, cmd, count);
}

/* blinks the LED of USB device */
void usbBlink_USBDIO24(hid_device *hid)
{
  struct report_t {
    uint8_t cmd;
    uint8_t pad[7];
  } report;

  report.cmd = BLINK_LED;
  PMD_SendOutputReport(hid, (uint8_t *) &report, sizeof(report));
}

/* resets the USB device */
int usbReset_USBDIO24(hid_device *hid)
{
  uint8_t cmd[8];

  cmd[0] = RESET;

  return PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint8_t usbGetID_USBDIO24(hid_device *hid)
{
  uint8_t cmd[8];

  cmd[0] = GET_ID;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);

  return cmd[0];
}

void usbSetID_USBDIO24(hid_device *hid, uint8_t id)
{
  uint8_t cmd[8];

  cmd[0] = SET_ID;
  cmd[1] = id;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

