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
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-1024LS.h"

/* Commands and Codes for USB 1024-LS HID reports */
#define DCONFIG     (0x0D)     // Configure digital port
#define DIN         (0x00)     // Read digital port
#define DOUT        (0x01)     // Write digital port
#define DBIT_IN     (0x02)     // Read Digital port bit
#define DBIT_OUT    (0x03)     // Write Digital port bit

#define CINIT       (0x05)     // Initialize counter
#define CIN         (0x04)     // Read Counter

#define MEM_READ    (0x09)     // Read Memory
#define MEM_WRITE   (0x0A)     // Write Memory

#define BLINK_LED   (0x0B)     // Causes LED to blink
#define RESET       (0x11)     // Reset USB interface
#define SET_ID      (0x0C)     // Set the user ID
#define GET_ID      (0x0F)     // Get the user ID

#define LS_DELAY (1500)

static uint8_t PortC = 0;

/* configures digital port */
void usbDConfigPort_USB1024LS(hid_device *hid, uint8_t port, uint8_t direction)
{
  struct report_t {
    uint8_t report_id;
    uint8_t cmd;
    uint8_t port;
    uint8_t direction;
    uint8_t pad[5];
  } report;

  report.report_id = 0x0;  // always zero
  report.cmd = DCONFIG;
  report.port = port;
  report.direction = direction;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
}

/* reads digital port  */
void usbDIn_USB1024LS(hid_device *hid, uint8_t port, uint8_t* din_value)
{
  uint8_t cmd[9];
  
  cmd[0] = 0x0;  //report_id;
  cmd[1] = DIN;
  cmd[2] = port;
  
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);
  *din_value = cmd[0];
  
  if (port == DIO_PORTC_HI)  *din_value >>= 4;
  if (port == DIO_PORTC_LOW) *din_value &= 0xf;
}

/* writes digital port */
void usbDOut_USB1024LS(hid_device *hid, uint8_t port, uint8_t value) 
{
  uint8_t cmd[8];
  
  cmd[0] = 0;     // report_id always zero
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
uint8_t usbDBitIn_USB1024LS(hid_device *hid, uint8_t port, uint8_t bit) 
{
  uint8_t cmd[8];

  cmd[0] = 0;
  cmd[1] = DBIT_IN;
  cmd[2] = port;
  cmd[3] = bit;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);

  return cmd[0];
}

/* writes digital port bit */
void usbDBitOut_USB1024LS(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value)
{
  uint8_t cmd[8];
  
  cmd[0] = 0;
  cmd[1] = DBIT_OUT;
  cmd[2] = port;
  cmd[3] = bit;
  cmd[4] = value;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

/* Initialize the counter */
void usbInitCounter_USB1024LS(hid_device *hid)
{
  uint8_t cmd[8];

  cmd[0] = 0;  
  cmd[1] = CINIT;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint32_t usbReadCounter_USB1024LS(hid_device *hid)
{
  struct report_t {
    uint8_t report_id;
    uint8_t cmd;
    uint8_t pad[7];
  } report;

  struct in_t {
    uint32_t count;
    uint8_t pad[4];
  } in;

  report.report_id = 0;   // always
  report.cmd = CIN;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t*) &in, sizeof(in), LS_DELAY);

  return in.count;
}

void usbReadMemory_USB1024LS(hid_device *hid, uint16_t address, uint8_t *data, uint8_t count)
{
  uint8_t cmd[8];

  if (count > 8) count = 8;             // max count is 8.
 
  cmd[0] = 0;                           // report number always 0
  cmd[1] = MEM_READ;
  cmd[2] = (uint8_t) (address & 0xff);  // low byte
  cmd[3] = (uint8_t) (address >> 0x8);  // high byte
  cmd[4] = count;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);

  memcpy(data, cmd, count);
}

/* blinks the LED of USB device */
void usbBlink_USB1024LS(hid_device *hid)
{
  struct report_t {
    uint8_t report_id;
    uint8_t cmd;
    uint8_t pad[6];
  } report;

  report.report_id = 0;
  report.cmd = BLINK_LED;
  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
}

/* resets the USB device */
int usbReset_USB1024LS(hid_device *hid)
{
  uint8_t cmd[8];

  cmd[0] = 0;
  cmd[1] = RESET;

  return PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint8_t usbGetID_USB1024LS(hid_device *hid)
{
  uint8_t cmd[8];

  cmd[0] = 0;
  cmd[1] = GET_ID;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
  PMD_GetInputReport(hid, cmd, sizeof(cmd), LS_DELAY);

  return cmd[0];
}

void usbSetID_USB1024LS(hid_device *hid, uint8_t id)
{
  uint8_t cmd[8];

  cmd[0] = 0;
  cmd[1] = SET_ID;
  cmd[2] = id;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

