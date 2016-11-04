/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@tx.ncsu.edu>
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

#ifndef USB_1024LS_H
#define USB_1024LS_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USB1024LS_PID  (0x0076)
#define USB1024HLS_PID (0x007F)

#define LS_DELAY (1500)

#define DIO_PORTA (0x01)
#define DIO_PORTB (0x04)
#define DIO_PORTC_LOW (0x08)
#define DIO_PORTC_HI  (0x02)

#define DIO_DIR_IN (0x01)
#define DIO_DIR_OUT (0x00)


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

/* function prototypes for the USB-1024LS */
void usbDConfigPort_USB1024LS(hid_device *hid, uint8_t port, uint8_t direction);
void usbDIn_USB1024LS(hid_device *hid, uint8_t port, uint8_t* din_value);
void usbDOut_USB1024LS(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USB1024LS(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USB1024LS(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
void usbInitCounter_USB1024LS(hid_device *hid);
uint32_t usbReadCounter_USB1024LS(hid_device *hid);
void usbReadMemory_USB1024LS(hid_device *hid, uint16_t address, uint8_t *data, uint8_t count);
void usbBlink_USB1024LS(hid_device *hid);
int usbReset_USB1024LS(hid_device *hid);
uint8_t usbGetID_USB1024LS(hid_device *hid);
void usbSetID_USB1024LS(hid_device *hid, uint8_t id);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_1024LS_H
