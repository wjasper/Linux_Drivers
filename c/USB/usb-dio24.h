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

#ifndef USB_DIO24_H
#define USB_DIO24_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USBDIO24_PID  (0x0093)
#define USBDIO24H_PID (0x0094)

#define DIO_PORTA (0x01)
#define DIO_PORTB (0x04)
#define DIO_PORTC_LOW (0x08)
#define DIO_PORTC_HI  (0x02)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

/* function prototypes for the USB-DIO24 */
void usbDConfigPort_USBDIO24(hid_device *hid, uint8_t port, uint8_t direction);
void usbDIn_USBDIO24(hid_device *hid, uint8_t port, uint8_t* din_value);
void usbDOut_USBDIO24(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USBDIO24(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USBDIO24(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
void usbInitCounter_USBDIO24(hid_device *hid);
uint32_t usbReadCounter_USBDIO24(hid_device *hid);
void usbReadMemory_USBDIO24(hid_device *hid, uint16_t address, uint8_t *data, uint8_t count);
void usbBlink_USBDIO24(hid_device *hid);
int usbReset_USBDIO24(hid_device *hid);
uint8_t usbGetID_USBDIO24(hid_device *hid);
void usbSetID_USBDIO24(hid_device *hid, uint8_t id);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_DIO24_H
