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

#ifndef USB_DIO96H_H
#define USB_DIO96H_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB1096HFS_PID   (0x0083)
#define USBDIO96H_PID    (0x0092)
#define USBDIO96H_50_PID (0x0095)

#define DIO_PORT0A     (0x0)
#define DIO_PORT0B     (0x1)
#define DIO_PORT0C_LOW (0x2)
#define DIO_PORT0C_HI  (0x3)
#define DIO_PORT1A     (0x4)
#define DIO_PORT1B     (0x5)
#define DIO_PORT1C_LOW (0x6)
#define DIO_PORT1C_HI  (0x7)
#define DIO_PORT2A     (0x8)
#define DIO_PORT2B     (0x9)
#define DIO_PORT2C_LOW (0xa)
#define DIO_PORT2C_HI  (0xb)
#define DIO_PORT3A     (0xc)
#define DIO_PORT3B     (0xd)
#define DIO_PORT3C_LOW (0xe)
#define DIO_PORT3C_HI  (0xf)

#define DIO_DIR_OUT (0x00)
#define DIO_DIR_IN (0x01)

/* function prototypes for the USB-DIO96H */
void usbDConfigPort_USBDIO96H(hid_device *hid, uint8_t port, uint8_t direction);
uint8_t usbDIn_USBDIO96H(hid_device *hid, uint8_t port);
void usbDOut_USBDIO96H(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USBDIO96H(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USBDIO96H(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
void usbInitCounter_USBDIO96H(hid_device *hid);
uint32_t usbReadCounter_USBDIO96H(hid_device *hid);
void usbReadMemory_USBDIO96H(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory);
int usbWriteMemory_USBDIO96H(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data);
void usbBlink_USBDIO96H(hid_device *hid);
int usbReset_USBDIO96H(hid_device *hid);
uint16_t usbGetStatus_USBDIO96H(hid_device *hid);
void usbGetAll_USBDIO96H(hid_device *hid, uint8_t data[]);

void usbPrepareDownload_USBDIO96H(hid_device *hid);
void usbWriteSerial_USBDIO96H(hid_device *hid, uint8_t serial[8]);
void usbWriteCode_USBDIO96H(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
int usbReadCode_USBDIO96H(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_DIO96H_H
