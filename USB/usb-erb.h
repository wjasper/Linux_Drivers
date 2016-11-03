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

#ifndef USB_ERB_H
#define USB_ERB_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USBERB08_PID  (0x008b)
#define USBERB24_PID  (0x008a)

#define DIO_PORTA     (0x0)
#define DIO_PORTB     (0x1)
#define DIO_PORTC_LOW (0x2)
#define DIO_PORTC_HI  (0x3)

/* Commands and Codes for USB ERB08/24 HID reports */
#define DIN              (0x03) // Read digital port
#define DOUT             (0x04) // Write digital port
#define DBIT_IN          (0x05) // Read Digital port bit
#define DBIT_OUT         (0x06) // Write Digital port bit

#define MEM_READ         (0x30) // Read Memory
#define MEM_WRITE        (0x31) // Write Memory

#define BLINK_LED        (0x40) // Causes LED to blink
#define RESET            (0x41) // Reset USB interface
#define GET_STATUS       (0x44) // Retrieve device status
#define GET_TEMP         (0x47) // Retrieve board temperature

#define PREPARE_DOWNLOAD (0x50) // Prepare for program memory download
#define WRITE_CODE       (0x51) // Write program memory
#define WRITE_SERIAL     (0x53) // Write new serial number to device
#define READ_CODE        (0x55) // Read program memory

/* function prototypes for the USB-ERB */
uint8_t usbDIn_USBERB(hid_device *hid, uint8_t port);
void usbDOut_USBERB(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USBERB(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USBERB(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
void usbReadMemory_USBERB(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory);
int usbWriteMemory_USBERB(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data);
void usbBlink_USBERB(hid_device *hid);
int usbReset_USBERB(hid_device *hid);
uint16_t usbGetStatus_USBERB(hid_device *hid);
float usbGetTemp_USBERB(hid_device *hid);

void usbPrepareDownload_USBERB(hid_device *hid);
void usbWriteSerial_USBERB(hid_device *hid, uint8_t serial[8]);
void usbWriteCode_USBERB(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
int usbReadCode_USBERB(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_ERB_H



