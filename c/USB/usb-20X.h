/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
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


#ifndef USB_20X_H

#define USB_20X_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB201_PID   (0x0113)
#define USB202_PID   (0x012b)
#define USB204_PID   (0x0114)
#define USB205_PID   (0x012c)

/* Analog Input Scan Options */
#define IMMEDIATE_TRANSFER_MODE (0x1)
#define BLOCK_TRANSFER_MODE     (0x0)
#define STALL_ON_OVERRUN        (0x0)
#define CONTINUOUS              (0x1 << 6)
#define INHIBIT_STALL           (0x1 << 7)
#define NO_TRIGGER              (0x0)
#define TRIGGER                 (0x1)
#define EDGE_RISING             (0x0)
#define EDGE_FALLING            (0x1)
#define LEVEL_HIGH              (0x2)
#define LEVEL_LOW               (0x3)
#define CHAN0                   (0x1)
#define CHAN1                   (0x2)
#define CHAN2                   (0x4)
#define CHAN3                   (0x8)
#define CHAN4                   (0x10)
#define CHAN5                   (0x20)
#define CHAN6                   (0x40)
#define CHAN7                   (0x80)
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)

/* Calibration Coefficients */
#define NCHAN_USB20X      8  // max number of A/D channels in the device
#define MAX_PACKET_SIZE  64  // max packet size for FS device

/* function prototypes for the USB-20X */
uint8_t usbDTristateR_USB20X(libusb_device_handle *udev);
void usbDTristateW_USB20X(libusb_device_handle *udev, uint8_t value);
uint8_t usbDPort_USB20X(libusb_device_handle *udev);
uint8_t usbDLatchR_USB20X(libusb_device_handle *udev);
void usbDLatchW_USB20X(libusb_device_handle *udev, uint8_t value);

uint16_t usbAIn_USB20X(libusb_device_handle *udev, uint8_t channel);
void usbAOut_USB20X(libusb_device_handle *udev, uint8_t channel, uint16_t value);
void usbAInScanStart_USB20X(libusb_device_handle *udev, uint32_t count, double frequency, uint8_t channels, uint8_t options, uint8_t trigger_source, uint8_t trigger_mode);
int usbAInScanRead_USB20X(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, uint8_t options, unsigned int timeout);
void usbAInScanStop_USB20X(libusb_device_handle *udev);
void usbAInScanClearFIFO_USB20X(libusb_device_handle *udev);
void usbAInBulkFlush_USB20X(libusb_device_handle *udev, uint8_t count);

uint32_t usbCounter_USB20X(libusb_device_handle *udev);
void usbCounterInit_USB20X(libusb_device_handle *udev);
void usbReadCalMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteCalMemory_USB20X(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbReadUserMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteUserMemory_USB20X(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbReadMBDMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteMBDMemory_USB20X(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbBlink_USB20X(libusb_device_handle *udev, uint8_t count);
void usbReset_USB20X(libusb_device_handle *udev);
uint16_t usbStatus_USB20X(libusb_device_handle *udev);
void usbGetSerialNumber_USB20X(libusb_device_handle *udev, char serial[9]);
void usbDFU_USB20X(libusb_device_handle *udev);
void usbMBDCommand_USB20X(libusb_device_handle *udev, uint8_t str[]);
void usbMBDRaw_USB20X(libusb_device_handle *udev, uint8_t cmd[]);
void cleanup_USB20X(libusb_device_handle *udev);
void usbBuildGainTable_USB20X(libusb_device_handle *udev, float table[NCHAN_USB20X][2]);
void usbCalDate_USB20X(libusb_device_handle *udev, struct tm *date);
double volts_USB20X(uint16_t value);
  

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // USB_1208HS_H
