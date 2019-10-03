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


#ifndef USB_2001_TC_H

#define USB_2001_TC_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB_2001_TC_PID (0x00f9)


/* Commands and USB Report ID for USB-2001-TC */

/* MBD Control Transfers */
//#define STRING_MESSAGE (0x80)  // Send string messages to the device
//#define RAW_DATA       (0x81)  // Return RAW data from the device

/* UL Control Transfers */
#define AIN            (0x10) // Read analog input channel
#define GET_ALL        (0x46) // Read Status, CJC and analog input channel
#define MEMORY         (0x30) // Read/Write memory
#define MEMADDRESS     (0x31) // Read/Write memory address
#define BLINK_LED      (0x40) // Cause LED to blink
#define RESET          (0x41) // Force a device reset
#define SERIAL         (0x48) // Read/Write serial number
#define UPDATE_MODE    (0xb0) // Put device into code update mode

#define NGAINS_USB2001_TC 8

typedef struct TC_data_t {
  uint8_t  status;     // 0x00 - Ready,  0x01 - Busy,  0xff - Error
  int16_t  CJC;        // CJC Value in Counts
  uint32_t ADC_Value;  // ADC Value in Counts
} TC_data;

#define BP_1_17V      0  // +/- 1.17V
#define BP_585_mV     1  // +/- 585mV
#define BP_292_5_mV   2  // +/- 292.5mV
#define BP_146_25mV   3  // +/- 146.25mV
#define BP_73_125mV   4  // +/- 73.125mV
#define BP_36_5625    5  // +/- 36.5625mV
#define BP_18_28125mV 6  // +/- 18.28125mV
#define BP_9_140625mV 7  // +/- 9.140625mV

// Status values
#define READY   0
#define ERROR   1
#define BUSY    2
#define UNKNOWN 3

/* function prototypes for the USB-2001-TC */
void usbBuildGainTable_USB2001TC(libusb_device_handle *udev, float table[NGAINS_USB2001_TC][2]);
void usbStringMessage_USB2001TC(libusb_device_handle *udev, char string[64]);
void usbRawData_USB2001TC(libusb_device_handle *udev, uint8_t *data);
void usbAIn_USB2001TC(libusb_device_handle *udev, uint32_t *data);
void usbGetAll_USB2001TC(libusb_device_handle *udev, TC_data data);
void usbReadMemory_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t *data);
void usbWriteMemory_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t *data);
void usbReadMemAddress_USB2001TC(libusb_device_handle *udev, uint16_t address);
void usbWriteMemAddress_USB2001TC(libusb_device_handle *udev, uint16_t address);
void usbBlink_USB2001TC(libusb_device_handle *udev, uint8_t count);
void usbReset_USB2001TC(libusb_device_handle *udev, uint8_t type);
void usbGetSerialNumber_USB2001TC(libusb_device_handle *udev, char serial[9]);
void usbSetSerialNumber_USB2001TC(libusb_device_handle *udev, char serial[9]);
void cleanup_USB2001TC(libusb_device_handle *udev);
void usbMemoryR_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t data[]);
void usbMemoryW_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t data[]);
void usbMemoryUnlock_USB2001TC(libusb_device_handle *udev);
void usbMemoryLock_USB2001TC(libusb_device_handle *udev);
void usbSetMemoryAddress_USB2001TC(libusb_device_handle *udev, uint16_t address) ;
void usbGetMemoryAddress_USB2001TC(libusb_device_handle *udev, uint16_t *address) ;
int getStatus_USB2001TC(libusb_device_handle *udev);
void setVoltageRange_USB2001TC(libusb_device_handle *udev, int range);
void getVoltageRange_USB2001TC(libusb_device_handle *udev, int *range);
void sendSensorType_USB2001TC(libusb_device_handle *udev, char type);
void getSensorType_USB2001TC(libusb_device_handle *udev, char *type);
void getCJC_USB2001TC(libusb_device_handle *udev, double *CJC_temperature);
void getCJCDegC_USB2001TC(libusb_device_handle *udev, double *CJC_temperature);
void getCJCDegF_USB2001TC(libusb_device_handle *udev, double *CJC_temperature);
void getCJCDegKelvin_USB2001TC(libusb_device_handle *udev, double *CJC_temperature);
void getSlope_USB2001TC(libusb_device_handle *udev, double *slope);
void getOffset_USB2001TC(libusb_device_handle *udev, double *offset);
void getFirmwareVersion_USB2001TC(libusb_device_handle *udev, char *version);
void getMFGCAL_USB2001TC(libusb_device_handle *udev, struct tm *date);
int getMFGCALYear_USB2001TC(libusb_device_handle *udev);
int getMFGCALMonth_USB2001TC(libusb_device_handle *udev);
int getMFGCALDay_USB2001TC(libusb_device_handle *udev);
int getMFGCALHour_USB2001TC(libusb_device_handle *udev);
int getMFGCALMinute_USB2001TC(libusb_device_handle *udev);
int getMFGCALSecond_USB2001TC(libusb_device_handle *udev);
void resetDefault_USB2001TC(libusb_device_handle *udev);
void reset_USB2001TC(libusb_device_handle *udev);
int getValue_USB2001TC(libusb_device_handle *udev, uint32_t *value);

double NISTCalcVoltage_USB2001TC(unsigned char tc_type, double temp);
double NISTCalcTemp_USB2001TC(unsigned char tc_type, double voltage);
int tc_temperature_USB2001TC(libusb_device_handle *udev, char tc_type, double *temp);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB-2001-TC
