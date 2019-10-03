/*
 *
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef BTH_1208LS_H
#define BTH_1208LS_H

#include <stdint.h>
#include "mccBluetooth.h"

#define BTH1208LS_PID  6883

#ifdef __cplusplus
extern "C" {
#endif

/* Commands and Report ID for BTH-1208LS  */
#define DIN_R                  (0x00) // Read the current state of the DIO pins
#define DOUT_R                 (0x02) // Read DIO latch value
#define DOUT_W                 (0x03) // Write DIO latch value
// Analog Input Commands
#define AIN                    (0x10) // Read analog input channel
#define AIN_SCAN_START         (0x11) // Start analog input scan
#define AIN_SCAN_SEND_DATA     (0x12) // Read data from the analog input scan FIFO
#define AIN_SCAN_STOP          (0x13) // Stop analog input scan
#define AIN_CONFIG_R           (0x14) // Read analog input configuration
#define AIN_CONFIG_W           (0x15) // Write analog input configuration
#define AIN_SCAN_CLEAR_FIFO    (0x16) // Clear the analog input scan FIFO
#define AIN_SCAN_RESEND_DATA   (0x18) // Resend data from the analog input scan
// Analog Output Commands
#define AOUT_R                 (0x20) // Read analog output channel
#define AOUT_W                 (0x21) //  analog output channel
// Counter Commands
#define COUNTER_R              (0x30) // Read event counter
#define COUNTER_W              (0x31) // Reset event counter
// Memory Commands
#define CAL_MEMORY_R           (0x40) // Read calibration memory
#define USER_MEMORY_R          (0x42) // Read user memory
#define USER_MEMORY_W          (0x43) // Write user memory
#define SETTINGS_MEMORY_R      (0x44) // Read settings memory
#define SETTINGS_MEMORY_W      (0x45) // Write settings memory
// Miscellaneous Commands
#define BLINK_LED              (0x50) // Blink the LED
#define RESET                  (0x51) // Reset the device
#define STATUS                 (0x52) // Read device status
#define SERIAL                 (0x54) // Read serial number
#define PING                   (0x55) // Check device communications
#define FIRMWARE_VERSION       (0x56) // Read the firmware version
#define RADIO_FIRMWARE_VERSION (0x5A) // Read the radio firmware version
#define BATTERY_VOLTAGE        (0x58) // Read battery voltage

#define NGAINS                   8  // max number of input gain levels (differential mode only)
#define NCHAN_DE                 4  // max number of A/D differential channels
#define NCHAN_SE                 8  // max number of A/D single-ended channels
#define NCHAN_AOUT               2  // max number of D/A 12 bit 0-2.5V output channels

/* Analog Input */
#define SINGLE_ENDED   0
#define DIFFERENTIAL   1

/* Analog Input Scan Options */
#define IMMEDIATE_TRANSFER_MODE (0x1)
#define BLOCK_TRANSFER_MODE     (0x0)
#define DIFFERENTIAL_MODE       (0x2)
#define SINGLE_ENDED_MODE       (0x0)
#define NO_TRIGGER              (0x0)
#define TRIG_EDGE_RISING        (0x1 << 2)
#define TRIG_EDGE_FALLING       (0x2 << 2)
#define TRIG_LEVEL_HIGH         (0x3 << 2)
#define TRIG_LEVEL_LOW          (0x4 << 2)
#define RETRIGGER_MODE          (0x1 << 5)
#define STALL_ON_OVERRUN        (0x0)
#define INHIBIT_STALL           (0x1 << 7)

/* Ranges */
#define BP_20V   0x0      // +/- 20 V
#define BP_10V   0x1      // +/- 10V
#define BP_5V    0x2      // +/- 5V
#define BP_4V    0x3      // +/- 4V
#define BP_2_5V  0x4      // +/- 2.5V
#define BP_2V    0x5      // +/- 2V
#define BP_1_25V 0x6      // +/- 1.25V
#define BP_1V    0x7      // +/- 1V
#define UP_2_5V  0x8      // 0-2.5V

/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define NO_BATTERY         (0x0)
#define FAST_CHARGE        (0x1 << 8)
#define MAINTENANCE_CHARGE (0x2 << 8)
#define FAULT_CHARGING     (0x3 << 8)
#define DISABLE_CHARGING   (0x4 << 8)

typedef struct calibrationTimeStamp_t {
  uint8_t year;   // Calibration date year - 2000
  uint8_t month;  // Calibration date month (1-12)
  uint8_t day;    // Calibration date day
  uint8_t hour;   // Calibration date hour
  uint8_t minute; // Calibration date minute
  uint8_t second; // Calibration date second
} calibrationTimeStamp;

typedef struct Calibration_t {
  float slope;
  float intercept;
} Calibration;

typedef struct DeviceInfo_BTH1208LS_t {
  BluetoothDeviceInfo device;
  Calibration table_AInDE[NCHAN_DE][NGAINS];  // calibration slope and offset differential mode
  Calibration table_AInSE[NCHAN_SE];          // calibration slope and offset single ended mode
  float frequency;
  unsigned long nDelay;
  int nChan;
  uint16_t voltage;
  uint16_t status;
  uint8_t  options;

} DeviceInfo_BTH1208LS;

/* function prototypes for the BTH-1208LS */
bool DIn_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t *value);
bool DOutR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t *value);
bool DOut_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t value);
bool AIn_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t channel, uint8_t mode, uint8_t range, uint16_t *value);
bool AInScanStart_BTH1208LS(DeviceInfo_BTH1208LS *device_info,uint32_t count, uint32_t retrig_count, double frequency,
		  uint8_t channels, uint8_t options);
int  AInScanSendData_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t count, uint16_t *data, unsigned long timeout);
int  AInScanRead_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t nScan, uint16_t *data);  
int  AInScanResendData_BTH_1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t count, uint16_t *data);
bool AInScanStop_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
bool AInConfigR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t ranges[4]);
bool AInConfigW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t ranges[4]);
bool AInScanClearFIFO_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
bool AOutR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t value[2]);
bool AOut_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint8_t channel, uint16_t value);
bool Counter_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint32_t *counter);
bool ResetCounter_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
bool CalMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data);
bool UserMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data);
bool UserMemoryW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data);
bool SettingsMemoryR_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data);
bool SettingsMemoryW_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t address, uint8_t count, uint8_t *data);
bool BlinkLED_BTH1208LS(DeviceInfo_BTH1208LS *device_info, unsigned char count);
bool Reset_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
bool GetSerialNumber_BTH1208LS(DeviceInfo_BTH1208LS *device_info, char serial[9]);
bool Status_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *status);
bool Ping_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *status);
bool FirmwareVersion_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *version);
bool RadioFirmwareVersion_BTH1208LS(DeviceInfo_BTH1208LS *device_info, uint16_t *version);
bool BatteryVoltage_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
void BuildGainTable_DE_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
void BuildGainTable_SE_BTH1208LS(DeviceInfo_BTH1208LS *device_info);
void CalDate_BTH1208LS(DeviceInfo_BTH1208LS *device_info, struct tm *date);
double volts_BTH1208LS(uint16_t value, uint8_t range);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // BTH_1208LS_H

/* 
    Settings memory map
|===========================================================================================|
|    Address    |        Value                                                              |
|===========================================================================================|
| 0x000         | DOut Bluetooth connection mode.  This determines the DOut value           |
|               | when the Bluetooth connection status changes (not applicable in USB mode) |
|               |    0 = no change                                                          |
|               |    1 = apply specified values                                             |
|-------------------------------------------------------------------------------------------|
| 0x001         | DOut value when Bluetooth connected                                       |
|-------------------------------------------------------------------------------------------|
| 0x002         | DOut value when Bluetooth disconnected                                    |
|-------------------------------------------------------------------------------------------|
| 0x003         | AOut channel 0 Bluetooth connection mode.  This determines the AOut value |
|               | when the Bluetooth connection status changes (not applicable in USB mode) |
|               |    0 = no change                                                          |
|               |    1 = apply specified values                                             |
|-------------------------------------------------------------------------------------------|
| 0x004 - 0x005 | AOut channel 0 value when Bluetooth connected                             |
|-------------------------------------------------------------------------------------------|
| 0x006 - 0x007 | AOut channel 0 value when Bluetooth disconnected                          |
|-------------------------------------------------------------------------------------------|
| 0x008         | AOut channel 1 Bluetooth connection mode.  This determines the AOut value |
|               | when the Bluetooth connection status changes (not applicable in USB mode) |
|               |    0 = no change                                                          |
|               |    1 = apply specified values                                             |
|-------------------------------------------------------------------------------------------|
| 0x009 - 0x00A | AOut channel 1 value when Bluetooth connected                             |
|-------------------------------------------------------------------------------------------|
| 0x00B - 0x00C | AOut channel 1 value when Bluetooth disconnected                          |
|-------------------------------------------------------------------------------------------|
| 0x00C         | Auto shutdown timer value in minutes (0 = no auto shutdown).  When this   |
|               | is used the device will automatically power down when powered by          |
|               | batteries and no Bluetooth connection is present for this amount of time. |
|-------------------------------------------------------------------------------------------|
| 0x00E         | Allow charging when Bluetooth connected                                   |
|               |   0 = do not allow                                                        |
|               |   1 = allow                                                               |
|-------------------------------------------------------------------------------------------|
| 0x00F - 0x3FF | Unused                                                                    |
|===========================================================================================|

*/
