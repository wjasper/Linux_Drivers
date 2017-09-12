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

#ifndef USB_1608FS_PLUS_H

#define USB_1608FS_PLUS_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB1608FS_PLUS_PID (0x00ea)

/* Analog Input Scan Options */
#define IMMEDIATE_TRANSFER_MODE (0x1)
#define BLOCK_TRANSFER_MODE     (0x0)
#define INTERNAL_PACER_ON       (0x2) // output internal pacer on SYNC
#define INTERNAL_PACER_OFF      (0x0) 
#define NO_TRIGGER              (0x0)
#define TRIG_EDGE_RISING        (0x1 << 2)
#define TRIG_EDGE_FALLING       (0x2 << 2)
#define TRIG_LEVEL_HIGH         (0x3 << 2)
#define TRIG_LEVEL_LOW          (0x4 << 2)
#define DEBUG_MODE              (0x20)
#define STALL_ON_OVERRUN        (0x0)
#define CONTINUOUS              (0x1 << 6)
#define INHIBIT_STALL           (0x1 << 7)

/* Aanalog Input */
#define SINGLE_ENDED   0
#define DIFFERENTIAL   1
#define CALIBRATION    3
#define LAST_CHANNEL   (0x80)
#define PACKET_SIZE    64       // max bulk transfer size in bytes
  
/* Ranges */
#define BP_10V   0x0      // +/- 10 V
#define BP_5V    0x1      // +/- 5V
#define BP_2V    0x3      // +/- 2V  
#define BP_1V    0x5      // +/- 1V
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)

#define NCHAN_USB1608FS_PLUS     8  // max number of A/D channels in the device
#define NGAINS_USB1608FS_PLUS    8  // max number of gain levels
#define MAX_PACKET_SIZE         64  // max packet size for FS device

/* function prototypes for the USB-1608FS-Plus */
uint8_t usbDTristateR_USB1608FS_Plus(libusb_device_handle *udev);
void usbDTristateW_USB1608FS_Plus(libusb_device_handle *udev, uint8_t value);
uint8_t usbDPort_USB1608FS_Plus(libusb_device_handle *udev);
uint8_t usbDLatchR_USB1608FS_Plus(libusb_device_handle *udev);
 void usbDLatchW_USB1608FS_Plus(libusb_device_handle *udev, uint8_t value);
uint16_t usbAIn_USB1608FS_Plus(libusb_device_handle *udev, uint8_t channel, uint8_t range);
void usbAInScanStart_USB1608FS_Plus(libusb_device_handle *udev, uint32_t count, double frequency, uint8_t channels, uint8_t options);
void usbAInScanConfig_USB1608FS_Plus(libusb_device_handle *udev, uint8_t ranges[8]);
void usbAInScanConfigR_USB1608FS_Plus(libusb_device_handle *udev, uint8_t *ranges);
int usbAInScanRead_USB1608FS_Plus(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, uint8_t options);
void usbAInScanStop_USB1608FS_Plus(libusb_device_handle *udev);
void usbAInScanClearFIFO_USB1608FS_Plus(libusb_device_handle *udev);
uint32_t usbCounter_USB1608FS_Plus(libusb_device_handle *udev);
void usbCounterInit_USB1608FS_Plus(libusb_device_handle *udev);
void usbReadCalMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteCalMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbReadUserMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteUserMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbReadMBDMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t memory[]);
void usbWriteMBDMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address,  uint16_t count, uint8_t data[]);
void usbBlink_USB1608FS_Plus(libusb_device_handle *udev, uint8_t count);
void usbReset_USB1608FS_Plus(libusb_device_handle *udev);
uint16_t usbStatus_USB1608FS_Plus(libusb_device_handle *udev);
void usbGetSerialNumber_USB1608FS_Plus(libusb_device_handle *udev, char serial[9]);
void usbDFU_USB1608FS_Plus(libusb_device_handle *udev);
void usbMBDCommand_USB1608FS_Plus(libusb_device_handle *udev, uint8_t str[]);
void usbMBDRaw_USB1608FS_Plus(libusb_device_handle *udev, uint8_t cmd[], uint16_t size);
void cleanup_USB1608FS_Plus(libusb_device_handle *udev);
void usbBuildGainTable_USB1608FS_Plus(libusb_device_handle *udev, float table[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2]);
double volts_USB1608FS_Plus(uint16_t value, uint8_t range);
void usbCalDate_USB1608FS_Plus(libusb_device_handle *udev, struct tm *date);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_1608FS_PLUS_H
