
/*
 *
 *  Copyright (c) 2014-2015 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef PMD_H
#define PMD_H

#ifdef __cplusplus
extern "C" { 
#endif 
#include <libusb-1.0/libusb.h>
#include "hidapi/hidapi.h"
#include <time.h>

/* These definitions are used to build the request type in usb_control_msg */
#define MCC_VID         (0x09db)  // Vendor ID for Measurement Computing
#define CTRL_IN         (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT        (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define INTR_LENGTH     64

#define  INPUT_REPORT   (1 << 8)
#define  OUTPUT_REPORT  (2 << 8)

/* Description of the requestType byte */
// Data transfer direction D7
#define HOST_TO_DEVICE (0x0 << 7)
#define DEVICE_TO_HOST (0x1 << 7)
// Type D5-D6
#define STANDARD_TYPE (0x0 << 5)
#define CLASS_TYPE    (0x1 << 5)
#define VENDOR_TYPE   (0x2 << 5)
#define RESERVED_TYPE (0x3 << 5)
// Recipient D0 - D4
#define DEVICE_RECIPIENT    (0x0)
#define INTERFACE_RECIPIENT (0x1)
#define ENDPOINT_RECIPIENT  (0x2)
#define OTHER_RECIPIENT     (0x3)
#define RESERVED_RECIPIENT  (0x4) 

/* For USB devices */
libusb_device_handle* usb_device_find_USB_MCC(int productId, char *serialID);
int usb_get_max_packet_size(libusb_device_handle* udev, int endpointNum);

/* MDB Control Transfers */
#define MAX_MESSAGE_LENGTH 64      // max length of MBD Packet in bytes
#define STRING_MESSAGE     (0x80)  // Send string messages to the device
#define RAW_DATA           (0x81)  // Return RAW data from the device

typedef struct calibrationTimeStamp_t {
  uint8_t year;   // Calibration date year
  uint8_t month;  // Calibration date month
  uint8_t day;    // Calibration date day
  uint8_t hour;   // Calibration date hour
  uint8_t minute; // Calibration date minute
  uint8_t second; // Calibration date second
} calibrationTimeStamp;
  
// Hid device wrapper functions
int PMD_SendOutputReport(hid_device* hid, uint8_t* values, size_t length);
int PMD_GetInputReport(hid_device* hid, uint8_t *values, size_t length, int delay);
int PMD_GetFeatureReport(hid_device* hid, uint8_t *data, int length);
int getUsbSerialNumber(libusb_device_handle *udev, unsigned char serial[]);

int sendStringRequest(libusb_device_handle *udev, char *message);
int getStringReturn(libusb_device_handle *udev, char *message);
void getRawData(libusb_device_handle *udev, void* data);


/* Structures for Temperature */
//*******************************************************************
// NIST Thermocouple coefficients
//
// The following types are supported:
//
//	J, K, R, S, T, N, E, B

/* Define the types of Thermocouples supported */
#define TYPE_J	0
#define TYPE_K	1
#define TYPE_T	2
#define TYPE_E	3
#define TYPE_R	4
#define TYPE_S	5
#define TYPE_B	6
#define TYPE_N	7

typedef struct NIST_Table_t {
  unsigned char nCoefficients;
  double VThreshold;
  const double* Coefficients;
} NIST_Table;

typedef struct NIST_Reverse_t {
  unsigned char nCoefficients;
  const double* Coefficients;
} NIST_Reverse;
	
typedef struct Thermocouple_Data_t {
  unsigned char nTables;
  const NIST_Reverse* ReverseTable;
  const NIST_Table* Tables;
} Thermocouple_Data;

double NISTCalcVoltage(unsigned char tc_type, double temp);
double NISTCalcTemp(unsigned char tc_type, double voltage);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif 

#endif  //PMD_H
