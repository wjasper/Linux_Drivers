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


#ifndef USB_500_H
#define USB_500_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include <time.h>


//#define USB_DEBUG

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


#define USB500_MAX_VALUES (0x4000)   // maximum number of samples 16384

#define USB500_WAIT_READ  500
#define USB500_WAIT_WRITE 100

#define USB500_VID 0x10c4  // Cygnal Integrated Products (Silicon Laboratories)
#define USB500_PID 0x0002  // EL-USB-[1,2]  MCC USB-50[1,2]

#define HIGH_ALARM_STATE (0x1 << 0)     // 1 = High Alarm Enabled, 0 = High Alarm Disabled
#define LOW_ALARM_STATE  (0x1 << 1)     // 1 = Low Alarm Enabled,  0 = Low Alarm Disabled
#define HIGH_ALARM_LATCH (0x1 << 2)     // 1 = High Alarm Indication Hold Enabled
                                        // 0 = High Alarm Indication Hold Disabled
#define LOW_ALARM_LATCH  (0x1 << 3)     // 1 = Low Alarm Indication Hold Enabled                 EL-USB-2 Only
                                        // 0 = Low Alarm Indication Hold Disabled
#define CH2_HIGH_ALARM_STATE (0x1 << 4) // 1 = High Alarm Enabled, 0 = High Alarm Disabled       EL-USB-2 Only
#define CH2_LOW_ALARM_STATE  (0x1 << 5) // 1 = High Alarm Enabled, 0 = High Alarm Disabled       
#define CH2_HIGH_ALARM_LATCH (0x1 << 6) // 1 = High Alarm Indication Hold Enabled                EL-USB-2 Only
                                        // 0 = High Alarm Indication Hold Disabled               
#define CH2_LOW_ALARM_LATCH (0x1 << 7)  // 1 = Low Alarm Indication Hold Enabled                 EL-USB-2 Only
                                        // 0 = Low Alarm Indication Hold Disabled
#define LOGGING_STATE (0x1 << 8)        // 1 = Delayed Start or Logging,  0 = Off
#define UNREAD        (0x1 << 9)        // 1 = Data has NOT been downloaded.
                                        // 0 = Data has been downloaded.
#define BATTERY_LOW   (0x1 << 10)       // 1 = During the last acquistion battery
                                        //     level dropped to a low level 
                                        //     and logging stopped
#define BATTERY_FAIL  (0x1 << 11)       // 1 = During the last acquistion battery
                                        //     level dropped to a critical level 
                                        //     and logging stopped
#define SENSOR_FAIL   (0x1 << 12)       // 1 = During last acquisition there were sensor errors. EL-USB-2 Only

/* 
  Note:
    EL-USB clears after current data has been downloaded.
*/

typedef struct configurationBlock_t {
  unsigned char type;              // 1-7
  unsigned char command;           //
  char name[16];                   // Null terminated
  unsigned char startTimeHours;    // 0-23
  unsigned char startTimeMinutes;  // 0-59
  unsigned char startTimeSeconds;  // 0-59
  unsigned char startTimeDay;      // 1-31
  unsigned char startTimeMonth;    // 1-12
  unsigned char startTimeYear;     // 00-99
  uint32_t startTimeOffset;        // 0-4294967295
  uint16_t sampleRate;             // 0-65535
  uint16_t sampleCount;            // 0-16382
  uint16_t flagBits;               //
  union {
    uint16_t highAlarm;            // 0-65535   USB-503 & USB-504
    struct {
      unsigned char highAlarm;     // 0-255 USB-501 & USB-502
      unsigned char lowAlarm;      // 0-255
    } channel1;
  } alarm1;
  float calibrationMValue;         // 3.4E +/- 38 (7dig)
  float calibrationCValue;         // 3.4E +/- 38 (7dig)
  uint16_t rawInputReading;        // 0 - 65535 (little endian)
  unsigned char inputType;         // 0-1
  unsigned char flagBits2;         // USB-503 & USB-504 only.
  char version[4];                 // Not Null terminated
  uint32_t serialNumber;           // 0-4294967295
  union {
    uint16_t lowAlarm;             // 0-65535   USB-503 & USB-504
    struct {
      unsigned char highAlarm;     // 0-255 USB-501 & USB-502
      unsigned char lowAlarm;      // 0-255
    } channel2;
  } alarm2;
  unsigned char notUsed[6];
  char displayUnitText[12];
  char calibrationInput1Text[8];
  char calibrationOutput1Text[8];
  char calibrationInput2Text[8];
  char calibrationOutput2Text[8];
  float capScalingFactor;
  char highAlarmLevelText[8];
  char lowAlarmLevelText[8];
  char defaultRangeDescriptionText[14];
  char defaultInputUnitText[12];
  char defaultDisplayUnit[12];
  char defaultCalibrationInput1Text[8];
  char defaultCalibrationOutput1Text[8];
  char defaultCalibrationInput2Text[8];
  char defaultCalibrationOutput2Text[8];
  char defaultHighAlarmLevelText[8];
  char defaultLowAlarmLevelText[8];
  uint16_t lowBatteryAlarm;
  char notUsed2[40];
} configurationBlock;

typedef struct usb501_data_t {
  time_t time;         // time sample was taken
  float  temperature;  // temperature
} usb501_data;

typedef struct usb502_data_t {
  time_t time;         // time sample was taken
  float  temperature;  // temperature
  float  humidity;     // humidity (USB-502 only)
} usb502_data;

typedef struct usb503_data_t {
  time_t time;         // time sample was taken
  float  voltage;      // 0-30V range
} usb503_data;

typedef struct usb504_data_t {
  time_t time;         // time sample was taken
  float  current;      // 4-20 mA
} usb504_data;

float celsius2fahr(float celsius);
float fahr2celsius(float fahr);
void cleanup_USB500(libusb_device_handle *udev);
libusb_device_handle* usb_device_find_USB500(int vendorId, int productId);
void unlock_device_USB500(libusb_device_handle *udev);
void lock_device_USB500(libusb_device_handle *udev);
int read_configuration_block_USB500(libusb_device_handle *udev, configurationBlock *cblock);
void write_configuration_block_USB500(libusb_device_handle *udev, configurationBlock *cblock);
void stop_logging_USB500(libusb_device_handle *udev, configurationBlock *cblock);
void start_logging_USB500(libusb_device_handle *udev, configurationBlock *cblock);
void set_alarm(libusb_device_handle *udev, configurationBlock *cblock);

int read_recorded_data_USB501(libusb_device_handle *udev, configurationBlock *cblock, usb501_data *data);
int read_recorded_data_USB502(libusb_device_handle *udev, configurationBlock *cblock, usb502_data *data);
int read_recorded_data_USB503(libusb_device_handle *udev, configurationBlock *cblock, usb503_data *data);
int read_recorded_data_USB504(libusb_device_handle *udev, configurationBlock *cblock, usb504_data *data);
 
void write_recorded_data_USB501(configurationBlock *cblock, usb501_data *data_501);
void write_recorded_data_USB502(configurationBlock *cblock, usb502_data *data_502);
void write_recorded_data_USB503(configurationBlock *cblock, usb503_data *data_503);
void write_recorded_data_USB504(configurationBlock *cblock, usb504_data *data_504);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_500_H
