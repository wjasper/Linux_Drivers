/*
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@tx.ncsu.edu>
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

#ifndef USB_TC32_H
#define USB_TC32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define USB_TC32_PID (0x0131)

/* Commands and Report ID for USB TC-32  */
// Digital I/O Commands

#define DIN             (0x00)  // Read digital input
#define DOUT            (0x02)  // Read/Write digital output

// Temperature Input Commands
#define TIN             (0x10)  // Read single thermocouple channel
#define CJC             (0x11)  // Read single CJC sensor
#define TIN_MULTIPLE    (0x12)  // Read multiple thermocouple channels
#define CJC_MULTIPLE    (0x13)  // Read multiple CJC sensors
#define TIN_CONFIG      (0x14)  // Read/write temperature channel configuration
#define TIN_STATUS      (0x16)  // Read temperature channel data status
#define OTD_STATUS      (0x17)  // Read open thermocouple detect data status
#define MEASURE_CONFIG  (0x18)  // Read/write measurement configuration
#define MEASURE_MODE    (0x1a)  // Read/write measurement mode

// Alarm Commands
#define ALARM_CONFIG    (0x20)  // Read/write alarm configuration
#define ALARM_STATUS    (0x22)  // Read/clear temperature alarm status

// Memory Commands
#define USER_MEMORY     (0x30) // Read/write user memory
#define SETTINGS_MEMORY (0x32) // Read/write network settings memory
#define CONFIG_MEMORY   (0x34) // Read/write device configuration memory
#define FACTORY_COEF    (0x36) // Read/write factory calibration coefficients
#define FIELD_COEF      (0x38) // Read/write field calibration coefficients
#define CAL_DATE        (0x3a) // Read/write factory calibration coefficients
#define GAIN_VOLTAGE    (0x3c) // Read/write gain reference voltages

// Miscellaneous Commands
#define BLINK_LED       (0x40) // Blink the LED
#define RESET           (0x41) // Reset the device
#define STATUS          (0x42) // Read device status
#define VERSION         (0x43) // Read firmware versions
#define AD_CAL          (0x44) // Run the A/D offset calibration
#define NETWORK_CONFIG  (0x45) // Read the current network configuration
#define LAST_RESULT     (0x46) // Read the result code for the last command

// Firmware Upgrade Commands
#define FIRMWARE_UPGRADE (0x50) // Enter firmware upgrade mode
#define BLMEMORY_COMMS   (0x51) // Enter firmware upgrade mode
#define BLMEMORY_BASE    (0x52) // Read/write comms micro bootloader memory
#define BLMEMORY_EXP     (0x53) // Read/write EXP measurement micro bootloader memory

// Channel type
#define CHAN_DISABLE 0
#define TC_TYPE_J    1  // Type J thermocouple
#define TC_TYPE_K    2  // Type K thermocouple
#define TC_TYPE_T    3  // Type T thermocouple
#define TC_TYPE_E    4  // Type E thermocouple
#define TC_TYPE_R    5  // Type R thermocouple
#define TC_TYPE_S    6  // Type S thermocouple
#define TC_TYPE_B    7  // Type B thermocouple
#define TC_TYPE_N    8  // Type N thermocouplep

// Measurement Configuration
#define OTD_DISABLE         (0x1)  // Open Thermocouple Detect disabled
#define OTD_ENABLE          (0x0)  // Open Thermocopple Detect enabled
#define NOTCH_60HZ          (0x2)  // notch at 60 Hz
#define NOTCH_50HZ          (0x0)  // notch at 40 Hz
#define SELECT_FACTORY_COEF (0x0)  // select factory coefficients
#define SELECT_FIELD_COEF   (0x4)  // select field coefficients

#define BASE                (0x1) // Base unit
#define EXP                 (0x2) // expansion unit

#define CELSIUS             (0x0) // read in Celsius
#define VOLTAGE             (0x1) // read in Voltage
#define ADC_CODE            (0x2) // uncalibraded
  
struct networkDeviceInfo_t {
  uint32_t ip_address;      // current device IP address
  uint32_t subnet_mask;     // current device subnet mask
  uint32_t gateway_address; // current gatewayaddress
};

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
  
typedef struct calCoeff_t {
  float slope_60_base[2];     // 60Hz slope values for each ADC (base unit)
  float slope_50_base[2];     // 50Hz slope values for each ADC (base unit)
  float intercept_60_base[2]; // 60Hz intercept values for each ADC (base unit)
  float intercept_50_base[2]; // 50Hz intercept values for each ADC (base unit)
  float slope_60_exp[2];      // 60Hz slope values for each ADC (EXP unit)
  float slope_50_exp[2];      // 50Hz slope values for each ADC (EXP unit)
  float intercept_60_exp[2];  // 60Hz intercept values for each ADC (EXP unit)
  float intercept_50_exp[2];  // 50Hz intercept values for each ADC (EXP unit)
} calCoeff;

typedef struct tc32_channel_t {
  uint32_t channel_mask[2];   // the channel bitmask for the base unit (channels 0-31)
                              // the channel bitmask for the EXP  unit (channels 32-63)
  uint32_t cjc_mask[2];       // the CJC bitmask for the base unit (channels 0-31)
                              // the CJC bitmask for the EXP  unit (channels 32-63)
  uint32_t Tin_status[2];     // the reading status of the Tin channels
  uint32_t OTD_status[2];     // the status of the open thermocouple detect.
  uint8_t config_measure[2];  // the measurement configuration
  uint8_t mode_measure[2];    // the measurement mode
  uint8_t all_channels;       // 0 - use channel mask, 1 - return all 64 sensor values
  uint8_t units;              // the units for the returned values: 0 - Celsius, 1 - Voltage, 2 - ADC code (uncalibraded)
  uint8_t wait;               // 0 - return current value, 1 - wait for new value before returning.
  float Tin_values[64];       // the values read from the configured channels
  float CJC_values[64];       // the values read from the configured channels
  uint8_t config_values[64];  // the configuration value of each channel (type of thermocouple);
  uint32_t alarm_status[2];   // the alarm status of each channel
  uint8_t alarm_config[64];   // the alarm configuration
  float alarm_threshold1[64]; // the alarm threshold 1 values in Celsius
  float alarm_threshold2[64]; // the alarm threshold 2 values in Celsius
  calCoeff calCoeffFactory;   // the factory calibration coefficients (slope and offset).
  calCoeff calCoeffField;     // the factory calibration coefficients (slope and offset).
  uint16_t status;             // 1 - EXP detected, 0 - no EXP detected
} tc32_channel;

/* function prototypes for the USB TC-32 */
void usbDIn_TC32(libusb_device_handle *udev, uint8_t value[2]);
void usbDOut_TC32(libusb_device_handle *udev, uint8_t index, uint32_t values);
void usbDOutR_TC32(libusb_device_handle *udev, uint32_t value[2]);
float usbTin_TC32(libusb_device_handle *udev, uint8_t channel, uint8_t units, uint8_t wait);
float usbCJC_TC32(libusb_device_handle *udev, uint8_t channel);
void usbTinMultiple_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbCJCMultiple_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbTinConfig_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbTinConfigR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbTinConfigR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbTinStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbOTDStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbMeasureConfigR_T32(libusb_device_handle *udev, tc32_channel *cdev);
void usbMeasureConfigW_T32(libusb_device_handle *udev, tc32_channel *cdev);
void usbMeasureModeR_T32(libusb_device_handle *udev, tc32_channel *cdev);
void usbMeasureModeW_T32(libusb_device_handle *udev, tc32_channel *cdev);
void usbAlarmConfigR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbAlarmStatusR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbAlarmStatusW_TC32(libusb_device_handle *udev, uint8_t index, uint32_t clear_mask);
void usbUserMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbUserMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbSettingsMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbSettingsMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbConfigMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbConfigMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data);
void usbFactoryCoefficientsR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbFieldCoefficientsR_TC32(libusb_device_handle *udev, tc32_channel *cdev);
void usbFieldCoefficientsW_TC32(libusb_device_handle *udev, uint8_t index, calCoeff *coeff);
void usbCalDate_TC32(libusb_device_handle *udev, struct tm *date_base, struct tm *date_exp);
void usbGainVoltages_TC32(libusb_device_handle *udev, gainVoltages *gain);
void usbBlink_TC32(libusb_device_handle *udev, uint8_t count);
void usbReset_TC32(libusb_device_handle *udev);  
void usbStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev);  
void usbVersion_TC32(libusb_device_handle *udev, struct version_t *version);
void usbADCal_TC32(libusb_device_handle *udev);
void usbNetworkConfig_TC32(libusb_device_handle *udev, struct networkDeviceInfo_t *device);
void usbLastResult(libusb_device_handle *udev, uint8_t *result);
void usbFirmwareUpgradeMode(libusb_device_handle *udev);  
void usbBLMemoryCommsR(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data);
void usbBLMemoryCommsW(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data);
void usbBLMemoryEXPR(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data);
void usbBLMemoryEXPW(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data);
  
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_TC32_H
  
