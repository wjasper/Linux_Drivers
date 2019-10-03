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

#ifndef USB_PDISO8_H
#define USB_PDISO8_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USBPDISO8_PID            (0x008c)
#define USBSWITCH_AND_SENSE_PID  (0x0084)

#define RELAY_PORT      (0x0)
#define ISO_PORT        (0x1)
#define FILTER_PORT     (0x2)
#define DEBUG_PORT      (0x3)
#define DAC_PORT        (0x4)

/* function prototypes for the USB-PDIS08 */
uint8_t usbDIn_USBPDISO8(hid_device *hid, uint8_t port);
void usbDOut_USBPDISO8(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USBPDISO8(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USBPDISO8(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
void usbReadMemory_USBPDISO8(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory);
int usbWriteMemory_USBPDISO8(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data);
void usbBlink_USBPDISO8(hid_device *hid);
void usbWriteSerial_USBPDISO8(hid_device *hid, uint8_t serial[8]);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_PDISO8_H


