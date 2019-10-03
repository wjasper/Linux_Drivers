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

#ifndef USB_1208FS_H
#define USB_1208FS_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB1208FS_PID (0x0082)

#define DIO_PORTA (0x00)
#define DIO_PORTB (0x01)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

// #define OFFSET_ADJUSTMENT  (0x1F00)   // Offset Adjustment for the A/D        0x1F00 - 0x1F4F
// #define SE_GAIN_ADJUSTMENT (0x1F50)   // Single Ended Gain Adjustment for A/D 0x1F50 - 0x1F5F
// #define DE_GAIN_ADJUSTMENT (0x1F60)   // Differential Gain Adjustment for A/D 0x1F60 - 0x1F67
// #define CAL_PIN_VOLTAGE    (0x1FA0)   // Calibration pin voltage              0x1FA0 - 0x1FA3

/* Status Bits (for 16 bit status word) */
#define SYNC           0x1  // 0 = Sync slave, 1 = Sync master
#define EXT_TRIG_EDGE  0x2  // 0 = trigger falling edge, 1 = trigger rising edge
#define UPDATE_MODE 0x8000  // 1 = program memory update mode

// Gain Ranges
#define SE_10_00V  (0x9)           // Single Ended 0-10.0 V
#define BP_20_00V  (0x0)           // Differential +/- 20.0 V
#define BP_10_00V  (0x1)           // Differential +/- 10.0 V
#define BP_5_00V   (0x2)           // Differential +/- 5.00 V
#define BP_4_00V   (0x3)           // Differential +/- 4.00 V
#define BP_2_50V   (0x4)           // Differential +/- 2.50 V
#define BP_2_00V   (0x5)           // Differential +/- 2.00 V
#define BP_1_25V   (0x6)           // Differential +/- 1.25 V
#define BP_1_00V   (0x7)           // Differential +/- 1.00 V

// Option values for AInScan
#define AIN_EXECUTION     0x1  // 1 = single execution, 0 = continuous execution
#define AIN_TRANSFER_MODE 0x2  // 1 = Immediate Transfer mode  0 = block transfer mode
#define AIN_TRIGGER       0x4  // 1 = Use External Trigger
#define AIN_DEBUG         0x8  // 1 = debug mode.
#define AIN_GAIN_QUEUE    0x10 // 1 = Use Channel Gain Queue, 0 = Use channnel parameters

/* function prototypes for the USB-1208FS */
int usbDConfigPort_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t direction);
int usbDIn_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t* din_value);
void usbDOut_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t value);

signed short usbAIn_USB1208FS(libusb_device_handle *udev, uint8_t channel, uint8_t range);
void usbAOut_USB1208FS(libusb_device_handle *udev, uint8_t channel, uint16_t value);
int usbAOutScan_USB1208FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel,
			  uint32_t count, float *frequency, uint16_t data[], uint8_t options);
void usbAOutStop_USB1208FS(libusb_device_handle *udev);
void usbAInStop_USB1208FS(libusb_device_handle *udev);
int usbAInScan_USB1208FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			 float *frequency, uint8_t options, int16_t data[]);
int usbAInScan_USB1208FS_SE(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			 float *frequency, uint8_t options, int16_t data[]);
void usbALoadQueue_USB1208FS(libusb_device_handle *udev, uint8_t num, uint8_t chan[], uint8_t gains[]);

void usbInitCounter_USB1208FS(libusb_device_handle *udev);
uint32_t usbReadCounter_USB1208FS(libusb_device_handle *udev);

void usbReadMemory_USB1208FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t memory[]);
int usbWriteMemory_USB1208FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t data[]);
void usbBlink_USB1208FS(libusb_device_handle *udev);
int usbReset_USB1208FS(libusb_device_handle *udev);
uint16_t usbGetStatus_USB1208FS(libusb_device_handle *udev);
void usbSetTrigger_USB1208FS(libusb_device_handle *udev, uint8_t type);
void usbSetSync_USB1208FS(libusb_device_handle *udev, uint8_t type);
void usbGetAll_USB1208FS(libusb_device_handle *udev, uint8_t data[]);
float volts_FS(const int gain, const signed short num);
float volts_SE(const signed short num);
int init_USB1208FS(libusb_device_handle *udev);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_1208FS_H
