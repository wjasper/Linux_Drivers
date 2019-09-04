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

#ifdef __cplusplus
extern "C" {
#endif

#define BTH1208LS_PID (0x011a)

/* Commands and Report ID for BTH-1208LS  */
#define DIN                    (0x01) // Read the current state of the DIO pins
#define DOUT                   (0x02) // Read/write DIO latch value
// Analog Input Commands
#define AIN                    (0x10) // Read analog input channel
#define AIN_SCAN_START         (0x11) // Start analog input scan
#define AIN_SCAN_STOP          (0x12) // Stop analog input scan
#define AIN_CONFIG             (0x14) // Read/write analog input configuration
#define AIN_SCAN_CLEAR_FIFO    (0x15) // Clear the analog input scan FIFO
// Analog Output Commands
#define AOUT                   (0x18) // Read/write analog output channel
// Counter Commands
#define COUNTER                (0x20) // Read/reset event counter
// Memory Commands
#define CAL_MEMORY             (0x30) // Read/write calibration memory
#define USER_MEMORY            (0x31) // Read/write user memory
#define SETTINGS_MEMORY        (0x32) // Read/write settings memory
// Miscellaneous Commands
#define BLINK_LED              (0x41) // Blink the LED
#define RESET                  (0x42) // Reset the device
#define STATUS                 (0x44) // Read device status
#define INIT_RADIO             (0x45) // Reset radio to default settings
#define BLUETOOTH_PIN          (0x46) // Read/write Bluetooth PIN
#define BATTERY_VOLTAGE        (0x47) // Read battery voltage
#define SERIAL                 (0x48) // Read/srite serial number
#define RADIO_FIRMWARE_VERSION (0x49) // Read radio firmware version
#define DFU                    (0x50) // Enter device firmware upgrade mode

#define MAX_PACKET_SIZE         64  // max packet size for FS device
#define NGAINS                   8  // max number of input gain levels (differential mode only)
#define NCHAN_DE                 4  // max number of A/D differential channels
#define NCHAN_SE                 8  // max number of A/D single-ended channels
#define NCHAN_AOUT               2  // max number of D/A 12 bit 0-2.5V output channels
  
/* Aanalog Input */
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

  // Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define NO_BATTERY         (0x0)
#define FAST_CHARGE        (0x1 << 8)
#define MAINTENANCE_CHARGE (0x2 << 8)
#define FAULT_CHARGING     (0x3 << 8)
#define DISABLE_CHARGING   (0x4 << 8)

/* function prototypes for the BTH-1208LS */
void usbBuildGainTable_DE_BTH1208LS(libusb_device_handle *udev, float table_DE[NGAINS][NCHAN_DE][2]);
void usbBuildGainTable_SE_BTH1208LS(libusb_device_handle *udev, float table_SE[NCHAN_SE][2]);
void usbCalDate_BTH1208LS(libusb_device_handle *udev, struct tm *date);
void usbDIn_BTH1208LS(libusb_device_handle *udev, uint8_t* value);
void usbDOut_BTH1208LS(libusb_device_handle *udev, uint8_t value);
void usbDOutR_BTH1208LS(libusb_device_handle *udev, uint8_t *value);
void usbAIn_BTH1208LS(libusb_device_handle *udev, uint8_t channel, uint8_t mode, uint8_t range, uint16_t *value);
void usbAInScanStart_BTH1208LS(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count,
			       double frequency, uint8_t channel, uint8_t options);
int usbAInScanRead_BTH1208LS(libusb_device_handle *udev, uint32_t count, uint16_t *data, uint8_t options);
void usbAInScanStop_BTH1208LS(libusb_device_handle *udev);
void usbAInConfigW_BTH1208LS(libusb_device_handle *udev, uint8_t ranges[4]);
void usbAInConfigR_BTH1208LS(libusb_device_handle *udev, uint8_t ranges[4]);
void usbAInScanClearFIFO_BTH1208LS(libusb_device_handle *udev);
void usbBlinkLED_BTH1208LS(libusb_device_handle *udev, uint8_t count);
void usbReset_BTH1208LS(libusb_device_handle *udev);
uint16_t usbStatus_BTH1208LS(libusb_device_handle *udev);
void usbInitRadio_BTH1208LS(libusb_device_handle *udev);
void usbBluetoothPinR_BTH1208LS(libusb_device_handle *udev, char *pin);
void usbBluetoothPinR_BTH1208LS(libusb_device_handle *udev, char *pin);
void usbBluetoothPinW_BTH1208LS(libusb_device_handle *udev, char *pin);
void usbBatteryVoltage_BTH1208LS(libusb_device_handle *udev, uint16_t *voltage);
void usbGetSerialNumber_BTH1208LS(libusb_device_handle *udev, char serial[9]);
void usbRadioFirmwareVersion_BTH1208LS(libusb_device_handle *udev, uint16_t *version);
void usbDFU_BTH1208LS(libusb_device_handle *udev);
void usbCalMemoryR_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[]);
void usbCalMemoryW_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[]);
void usbAOut_BTH1208LS(libusb_device_handle *udev, uint8_t channel, uint16_t value);
uint16_t usbAOutR_BTH1208LS(libusb_device_handle *udev, uint8_t channel);
void usbCounter_BTH1208LS(libusb_device_handle *udev, uint32_t *counter);
void usbResetCounter_BTH1208LS(libusb_device_handle *udev);
void cleanup_BTH1208LS(libusb_device_handle *udev);
double volts_BTH1208LS(uint16_t value, uint8_t range);
  
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif // BTH_1208LS_H

