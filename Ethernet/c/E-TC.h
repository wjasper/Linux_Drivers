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

#ifndef E_TC_H

#define E_TC_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <time.h>
#include "mccEthernet.h"

#define ETC_PID    0x0138

// Digital I/O Commands
#define DIN                 (0x00)  // Read DIO pins
#define DOUT_R              (0x02)  // Read DIO latch value
#define DOUT_W              (0x03)  // Write DIO latch value
#define DCONFIG_R           (0x04)  // Read DIO configuration value
#define DCONFIG_W           (0x05)  // Write DIO configuration value

// Temperature Input Commands
#define TIN                 (0x10) // Read single thermocouple channel
#define CMD_CJC             (0x11) // Read single CJC sensor
#define TIN_CONFIG_R        (0x12) // Read temperature channel configuration
#define TIN_CONFIG_W        (0x13) // Write temperature channel configuration
#define TIN_STATUS          (0x14) // Read temperature channel data status
#define OTD_STATUS          (0x15) // Read open thermocouple detect data status
#define MEASURE_CONFIG_R    (0x16) // Read measurement configuration
#define MEASURE_CONFIG_W    (0x17) // Write measurement configuration
#define MEASURE_MODE_R      (0x18) // Read measurement mode
#define MEASURE_MODE_W      (0x19) // Write measurement mode
#define FACTORY_COEF_R      (0x1A) // Write factory calibration coefficients
#define FACTORY_COEF_W      (0x1B) // Write factory calibration coefficients
#define FIELD_COEF_R        (0x1C) // Read field calibration coefficients
#define FIELD_COEF_W        (0x1D) // Write field calibration coefficients
#define FACTORY_CAL_DATE_R  (0x1E) // Read factory calibration date
#define FACTORY_CAL_DATE_W  (0x1F) // Write factory calibration date
#define FIELD_CAL_DATE_R    (0x20) // Read field calibration date
#define FIELD_CAL_DATE_W    (0x21) // Write field calibration date
#define AD_CAL              (0x22) // Run the A/D offset calibration
#define CJC_OFFSET_R        (0x24) // Read user CJC offset values
#define CJC_OFFSET_W        (0x25) // Write user CJC offset values
  
// Alarm Commands
#define ALARM_CONFIG_R      (0x28) // Read temperature alarm configuration
#define ALARM_CONFIG_W      (0x29) // Write temperature alarm configuration
#define ALARM_STATUS_R      (0x2A) // Read temperature alarm status
#define ALARM_STATUS_W      (0x2B) // Clear temperature alarm status

// Counter Commands
#define COUNTER_R           (0x30) // Read event counter
#define COUNTER_W           (0x31) // Write event counter

// Memory Commands
#define CONFIG_MEMORY_R     (0x40) // Read device configuration memory
#define CONFIG_MEMORY_W     (0x41) // Write device configuration memory
#define USER_MEMORY_R       (0x42) // Read user memory
#define USER_MEMORY_W       (0x43) // Write user memory
#define SETTINGS_MEMORY_R   (0x44) // Read network settings memory
#define SETTINGS_MEMORY_W   (0x45) // Write network settings memory
#define BOOTLOADER_MEMORY_R (0x46) // Read bootloader memory
#define BOOTLOADER_MEMORY_W (0x47) // Write bootloader memory

// Miscellaneous Commands
#define BLINK_LED           (0x50) // Blink the LED
#define RESET               (0x51) // Reset the device
#define STATUS              (0x52) // Read device status
#define NETWORK_CONFIG      (0x54) // Read the current network configuration
#define FIRMWARE_UPGRADE    (0x60) // Enter firmware upgrade mode

#define UNITS_CELSIUS       (0x0) // read in Celsius
#define VOLTAGE             (0x1) // read in Voltage
#define ADC_CODE            (0x2) // uncalibraded


typedef struct calCoeff_t {
  float slope[2];     // slope values for each ADC
  float intercept[2]; // intercept values for each ADC
} calCoeff;

typedef struct DeviceInfo_TC_t {
  EthernetDeviceInfo device;
  uint8_t channel_mask;       // the channel bitmask (channels 0-7)
  uint32_t Tin_status;        // the reading status of the Tin channels
  uint32_t OTD_status;        // the status of the open thermocouple detect.
  uint32_t counter;           // the value of the counter
  uint8_t config_measure;     // the measurement configuration
  uint8_t mode_measure;       // the measurement mode
  uint8_t units;              // the units for the returned values: 0 - Celsius, 1 - Voltage, 2 - ADC code (uncalibraded)
  uint8_t wait;               // 0 - return current value, 1 - wait for new value before returning.
  float Tin_values[8];        // the values read from the configured channels
  float CJC_offsets[8];       // the per channel CJC user offsets
  float CJC_value[2];         // the CJC values in Celsius.
  uint8_t config_values[8];   // the configuration value of each channel (type of thermocouple);
  uint8_t alarm_status;       // the alarm status of each channel
  uint8_t alarm_config[8];    // the alarm configuration
  float alarm_threshold1[8];  // the alarm threshold 1 values in Celsius
  float alarm_threshold2[8];  // the alarm threshold 2 values in Celsius
  calCoeff calCoeffFactory;   // the factory calibration coefficients (slope and offset).
  calCoeff calCoeffField;     // the factory calibration coefficients (slope and offset).
  uint16_t status;            // 1 - EXP detected, 0 - no EXP detected
} DeviceInfo_TC;

/* function prototypes for the E-TC */  
bool DIn_E_TC(DeviceInfo_TC *device_info, uint8_t *value);
bool DOutR_E_TC(DeviceInfo_TC *device_info, uint8_t *value);
bool DOutW_E_TC(DeviceInfo_TC *device_info, uint8_t value);
bool DConfigR_E_TC(DeviceInfo_TC *device_info, uint8_t *value);
bool DConfigW_E_TC(DeviceInfo_TC *device_info, uint8_t value);
bool Tin_E_TC(DeviceInfo_TC *device_info, uint8_t channel_mask, uint8_t units, uint8_t wait, float *value);
bool CJC_E_TC(DeviceInfo_TC *device_info);
bool TinConfigR_E_TC(DeviceInfo_TC *device_info);
bool TinConfigW_E_TC(DeviceInfo_TC *device_info);
bool TinStatus_E_TC(DeviceInfo_TC *device_info);
bool OTDStatus_E_TC(DeviceInfo_TC *device_info);
bool MeasureConfigR_E_TC(DeviceInfo_TC *device_info);
bool MeasureConfigW_E_TC(DeviceInfo_TC *device_info);
bool MeasureModeR_E_TC(DeviceInfo_TC *device_info);
bool MeasureModeW_E_TC(DeviceInfo_TC *device_info);
bool FactoryCoefficientsR_E_TC(DeviceInfo_TC *device_info);
bool FactoryCoefficientsW_E_TC(DeviceInfo_TC *device_info);
bool FieldCoefficientsR_E_TC(DeviceInfo_TC *device_info);
bool FieldCoefficientsW_E_TC(DeviceInfo_TC *device_info);
bool FactoryCalDateR_E_TC(DeviceInfo_TC *device_info, struct tm *date);
bool FactoryCalDateW_E_TC(DeviceInfo_TC *device_info, struct tm *date);
bool FieldCalDateR_E_TC(DeviceInfo_TC *device_info, struct tm *date);
bool FieldCalDateW_E_TC(DeviceInfo_TC *device_info, struct tm *date);
bool ADCal_E_TC(DeviceInfo_TC *device_info);
bool CJCOffsetR_E_TC(DeviceInfo_TC *device_info);
bool CJCOffsetW_E_TC(DeviceInfo_TC *device_info);
bool AlarmConfigR_E_TC(DeviceInfo_TC *device_info);
bool AlarmConfigW_E_TC(DeviceInfo_TC *device_info);
bool AlarmStatusR_E_TC(DeviceInfo_TC *device_info);
bool AlarmStatusW_E_TC(DeviceInfo_TC *device_info, uint8_t clear_mask);
bool CounterR_E_TC(DeviceInfo_TC *device_info);
bool ResetCounter_E_TC(DeviceInfo_TC *device_info);
bool ConfigMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool ConfigMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool UserMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool UserMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryR_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryR_E_TC(DeviceInfo_TC *device_info, uint32_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryW_E_TC(DeviceInfo_TC *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BlinkLED_E_TC(DeviceInfo_TC *device_info, uint8_t count);
bool Reset_E_TC(DeviceInfo_TC *device_info);
bool Status_E_TC(DeviceInfo_TC *device_info);
bool NetworkConfig_E_TC(DeviceInfo_TC *device_info, struct networkDeviceInfo_t *network);
bool FirmwareUpgrade_E_TC(DeviceInfo_TC *device_info);  
  
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // E_TC_H

/* 
    Configuration memory map
|=================================================================|
|    Address   |        Value                                     |
|=================================================================|
| 0x00 - 0x07  | Serial number (not used by firmware)             |
|-----------------------------------------------------------------|
| 0x08 - 0x09  | Reserved                                         |
|-----------------------------------------------------------------|
| 0x0A - 0x0F  | MAC address                                      |
|              |  If all 6 bytes are 0xFF then the firmware will  |
|              |  use the Microchip unique MAC address that       |
|              |  is programmed into the micro.                   |
|=================================================================|


    Settings memory map
|=====================================================================================|
|    Address    |        Value                                        | Default value |
|=====================================================================================|
| 0x000 - 0x001 | Network options:                                    | 0x0000        |
|               |   Bit 0: 0 = DHCP enabled     1 = DHCP disabled     |               |
|               |   Bit 1: 0 = Auto IP enabled  1 = Auto IP disabled  |               |
|               |   Bits 2-15 reserved                                |               |
|-------------------------------------------------------------------------------------|
| 0x002 - 0x005 | Default IP address                                  | 192.168.0.101 |
|-------------------------------------------------------------------------------------|
| 0x006 - 0x009 | Default subnet mask                                 | 255.255.255.0 |
|-------------------------------------------------------------------------------------|
| 0x00A - 0x00D | Default gateway address                             | 192.168.0.1   |
|-------------------------------------------------------------------------------------|
| 0x00E - 0x00F | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x010 - 0x011 | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x012 - 0x015 | Connection code, 4 bytes                            | 0x00000000    |
|-------------------------------------------------------------------------------------|
| 0x016 - 0x01F | Reserved                                            |               |
|=====================================================================================|

Note: The settings do not take effect until after device is reset or power cycled.

    User memory map
|=================================================================|
|    Address     |        Value                                   |
|=================================================================|
| 0x000 - 0xDFF  | Available for UL use                           |
|=================================================================|

*/
