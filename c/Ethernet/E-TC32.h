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

#ifndef E_TC32_H

#define E_TC32_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <time.h>
#include "mccEthernet.h"
  

#define ETC32_PID       0x0132

// Digital I/O Commands
#define CMD_DIN            (0x00)  // Read DIO pins
#define CMD_DOUT_R         (0x02)  // Read DIO latch value
#define CMD_DOUT_W         (0x03)  // Write DIO latch value

// Temperature Input Commands
#define CMD_TIN               (0x10)  // Read single thermocouple channel
#define CMD_CJC               (0x11)  // Read single CJC sensor
#define CMD_TIN_MULTIPLE      (0x12)  // Read multiple thermocouple channels
#define CMD_CJC_MULTIPLE      (0x13)  // Read multiple CJC sensors
#define CMD_TIN_CONFIG_R      (0x14)  // Read temperature channel configuration
#define CMD_TIN_CONFIG_W      (0x15)  // Write temperature channel configuration
#define CMD_TIN_STATUS        (0x16)  // Read temperature channel data status
#define CMD_OTD_STATUS        (0x17)  // Read open thermocouple detect data status
#define CMD_MEASURE_CONFIG_R  (0x18)  // Read measurement configuration
#define CMD_MEASURE_CONFIG_W  (0x19)  // Write measurement configuration
#define CMD_MEASURE_MODE_R    (0x1a)  // Read measurement mode
#define CMD_MEASURE_MODE_W    (0x1b)  // Write measurement mode

// Alarm Commands
#define CMD_ALARM_CONFIG_R    (0x20)  // Read alarm configuration
#define CMD_ALARM_CONFIG_W    (0x21)  // Write alarm configuration
#define CMD_ALARM_STATUS_R    (0x22)  // Read temperature alarm status
#define CMD_ALARM_STATUS_W    (0x23)  // Clear temperature alarm status

// Memory Commands
#define CMD_USER_MEMORY_R     (0x30) // Read user memory
#define CMD_USER_MEMORY_W     (0x31) // Write user memory
#define CMD_SETTINGS_MEMORY_R (0x32) // Read network settings memory
#define CMD_SETTINGS_MEMORY_W (0x33) // Write network settings memory
#define CMD_CONFIG_MEMORY_R   (0x34) // Read device configuration memory
#define CMD_CONFIG_MEMORY_W   (0x35) // Write device configuration memory
#define CMD_FACTORY_COEF_R    (0x36) // Read factory calibration coefficients
#define CMD_FACTORY_COEF_W    (0x37) // Write factory calibration coefficients
#define CMD_FIELD_COEF_R      (0x38) // Read field calibration coefficients
#define CMD_FIELD_COEF_W      (0x39) // Write field calibration coefficients
#define CMD_CAL_DATE_R        (0x3a) // Read factory calibration coefficients
#define CMD_CAL_DATE_W        (0x3b) // Write factory calibration coefficients
#define CMD_GAIN_VOLTAGE_R    (0x3c) // Read gain reference voltages
#define CMD_GAIN_VOLTAGE_W    (0x3d) // Write gain reference voltages

// Miscellaneous Commands
#define CMD_BLINK_LED         (0x50) // Blink the LED
#define CMD_RESET             (0x51) // Reset the device
#define CMD_STATUS            (0x52) // Read device status
#define CMD_VERSION           (0x53) // Read firmware versions
#define CMD_NETWORK_CONFIG    (0x54) // Read the current network configuration
#define CMD_AD_CAL            (0x55) // Run the A/D offset calibration

#define BASE                (0x1) // Base unit
#define EXP                 (0x2) // expansion unit

#define UNITS_CELSIUS       (0x0) // read in Celsius
#define VOLTAGE             (0x1) // read in Voltage
#define ADC_CODE            (0x2) // uncalibraded
  
struct version_t {
  uint16_t version_comms;       // the communications micro firmware version
  uint16_t boot_version_comms;  // the communications micro bootloader firmware version
  uint16_t version_base;        // the base measurement micro firmware version
  uint16_t boot_version_base;   // the base measurement micro bootloader firmware version
  uint16_t version_exp;         // the EXP measurement micro firmware version
  uint16_t boot_version_exp;    // the EXP measurement micro firmware version
};

typedef struct gainVoltages_t {
  float voltage_0_base;       // the measured gain calibration voltage for ADC 0, 60 Hz for the base unit
  float voltage_1_base;       // the measured gain calibration voltage for ADC 0, 50 Hz for the base unit
  float voltage_2_base;       // the measured gain calibration voltage for ADC 1, 60 Hz for the base unit
  float voltage_3_base;       // the measured gain calibration voltage for ADC 1, 50 Hz for the base unit
  float voltage_0_exp;        // the measured gain calibration voltage for ADC 0, 60 Hz for the EXP unit
  float voltage_1_exp;        // the measured gain calibration voltage for ADC 0, 50 Hz for the EXP unit
  float voltage_2_exp;        // the measured gain calibration voltage for ADC 1, 60 Hz for the EXP unit
  float voltage_3_exp;        // the measured gain calibration voltage for ADC 1, 50 Hz for the EXP unit
} gainVoltages;

typedef struct calCoeff_TC32_t {
  float slope_60_base[2];     // 60Hz slope values for each ADC (base unit)
  float slope_50_base[2];     // 50Hz slope values for each ADC (base unit)
  float intercept_60_base[2]; // 60Hz intercept values for each ADC (base unit)
  float intercept_50_base[2]; // 50Hz intercept values for each ADC (base unit)
  float slope_60_exp[2];      // 60Hz slope values for each ADC (EXP unit)
  float slope_50_exp[2];      // 50Hz slope values for each ADC (EXP unit)
  float intercept_60_exp[2];  // 60Hz intercept values for each ADC (EXP unit)
  float intercept_50_exp[2];  // 50Hz intercept values for each ADC (EXP unit)
} calCoeff_TC32;
  
typedef struct DeviceInfo_TC32_t {
  EthernetDeviceInfo device;
  uint32_t channel_mask[2];   // the channel bitmask for the base unit (channels 0-31)
                              // the channel bitmask for the EXP  unit (channels 32-63)
  uint32_t cjc_mask[2];       // the CJC bitmask for the base unit (channels 0-31)
                              // the CJC bitmask for the EXP  unit (channels 32-63)
  uint32_t Tin_status[2];     // the reading status of the Tin channels
  uint32_t OTD_status[2];     // the status of the open thermocouple detect.
  uint8_t config_measure[2];  // the measurement configuration
  uint8_t mode_measure[2];    // the measurement mode
  uint8_t units;              // the units for the returned values: 0 - Celsius, 1 - Voltage, 2 - ADC code (uncalibraded)
  uint8_t wait;               // 0 - return current value, 1 - wait for new value before returning.
  float Tin_values[64];       // the values read from the configured channels
  float CJC_values[64];       // the values read from the configured channels
  uint8_t config_values[64];  // the configuration value of each channel (type of thermocouple);
  uint32_t alarm_status[2];   // the alarm status of each channel
  uint8_t alarm_config[64];   // the alarm configuration
  float alarm_threshold1[64]; // the alarm threshold 1 values in Celsius
  float alarm_threshold2[64]; // the alarm threshold 2 values in Celsius
  calCoeff_TC32 calCoeffFactory; // the factory calibration coefficients (slope and offset).
  calCoeff_TC32 calCoeffField;   // the factory calibration coefficients (slope and offset).
  uint16_t status;            // 1 - EXP detected, 0 - no EXP detected
} DeviceInfo_TC32;

/* function prototypes for the E-TC32 */
bool DIn_E_TC32(DeviceInfo_TC32 *device_info, uint8_t value[2]);
bool DOutR_E_TC32(DeviceInfo_TC32 *device_info, uint32_t value[2]);
bool DOutW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, uint32_t value);
bool Tin_E_TC32(DeviceInfo_TC32 *device_info, uint8_t channel, uint8_t units, uint8_t wait, float *value);
bool CJC_E_TC32(DeviceInfo_TC32 *device_info, uint8_t channel, float *value);
bool TinMultiple_E_TC32(DeviceInfo_TC32 *device_info);
bool CJCMultiple_E_TC32(DeviceInfo_TC32 *device_info);
bool TinConfigR_E_TC32(DeviceInfo_TC32 *device_info);
bool TinConfigW_E_TC32(DeviceInfo_TC32 *device_info);
bool TinStatus_E_TC32(DeviceInfo_TC32 *device_info);
bool OTDStatus_E_TC32(DeviceInfo_TC32 *device_info);
bool MeasureConfigR_E_TC32(DeviceInfo_TC32 *device_info);
bool MeasureConfigW_E_TC32(DeviceInfo_TC32 *device_info);
bool MeasureModeR_E_TC32(DeviceInfo_TC32 *device_info);
bool MeasureModeW_E_TC32(DeviceInfo_TC32 *device_info);
bool AlarmConfigR_E_TC32(DeviceInfo_TC32 *device_info);
bool AlarmConfigW_E_TC32(DeviceInfo_TC32 *device_info);
bool AlarmStatusR_E_TC32(DeviceInfo_TC32 *device_info);
bool AlarmStatusW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, uint32_t clear_masks);
bool UserMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool UserMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool ConfigMemoryR_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool ConfigMemoryW_E_TC32(DeviceInfo_TC32 *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool FactoryCoefficientsR_E_TC32(DeviceInfo_TC32 *device_info);
bool FactoryCoefficientsW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index);
bool FieldCoefficientsR_E_TC32(DeviceInfo_TC32 *device_info);
bool FieldCoefficientsW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index);
bool CalDateR_E_TC32(DeviceInfo_TC32 *device_info, struct tm *date_base, struct tm *date_exp);
bool CalDateW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, struct tm *date);
bool GainVoltageR_E_TC32(DeviceInfo_TC32 *device_info, gainVoltages *gain);  
bool GainVoltageW_E_TC32(DeviceInfo_TC32 *device_info, uint8_t index, gainVoltages *gain);
bool BlinkLED_E_TC32(DeviceInfo_TC32 *device_info, uint8_t count);
bool Reset_E_TC32(DeviceInfo_TC32 *device_info);
bool Status_E_TC32(DeviceInfo_TC32 *device_info);
bool Version_E_TC32(DeviceInfo_TC32 *device_info, struct version_t *version);
bool NetworkConfig_E_TC32(DeviceInfo_TC32 *device_info, struct networkDeviceInfo_t *network);
bool ADCal_E_TC32(DeviceInfo_TC32 *device_info);
  
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // E_TC32_H

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
| 0x002 - 0x005 | Default IPv4 address                                | 192.168.0.101 |
|-------------------------------------------------------------------------------------|
| 0x006 - 0x009 | Default IPv4 subnet mask                            | 255.255.255.0 |
|-------------------------------------------------------------------------------------|
| 0x00A - 0x00D | Default IPv4 gateway address                        | 192.168.0.1   |
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
|===================================================================|
|    Address       |        Value                                   |
|===================================================================|
| 0x0000 - 0x0EFF  | Comms micro memory, available for UL use       |
|-------------------------------------------------------------------|
| 0x1000 - 0x1DFF  | Measurement micro memory, available for UL use |
|-------------------------------------------------------------------|
| 0x2000 - 0x2DFF  | EXP micro memory, available for UL use         |
|===================================================================|

*/
