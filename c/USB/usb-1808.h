/*
 *
 *  Copyright (c) 2017 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef USB_1808_H

#define USB_1808_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB1808_PID        (0x013d)
#define USB1808X_PID       (0x013e)

/* Counter Timer */
#define COUNTER0           0x0   // Counter 0
#define COUNTER1           0x1   // Counter 1
#define ENCODER0           0x2   // Counter 2
#define ENCODER1           0x3   // Counter 3
#define TIMER0             0x0   // Timer 0
#define TIMER1             0x1   // Timer 0

/* Timer Control */
#define TIMER_ENABLE       0x1   // Enable timer
#define TIMER_RUNNING      0x2   // Timer running
#define TIMER_INVERTED     0x4   // Timer inverted output
#define TIMER_OTRIG_BEGIN 0x10   // Timer will begin output when the OTRIG pin has triggered
#define TIMER_OTRIG       0x40   // Timer will continue to output on every OTRIG it receives

/* Counter Modes */
#define COUNTER_TOTALIZE   0x0    // Total Count (counts total number of pulses)
#define COUNTER_PERIOD     0x1    // Counter returns Period [100us]
#define COUNTER_PULSEWIDTH 0x2    // Counter returns pulse width [100us]
#define COUNTER_TIMING     0x3
#define PERIOD_MODE_1X     0x0    // Period Mode x1
#define PERIOD_MODE_10X    0x4    // Period Mode x10
#define PERIOD_MODE_100X   0x8    // Period Mode x100
#define PERIOD_MODE_1000X  0xc    // Period Mode x1000
#define TICK_SIZE_20NS     0x0    // Tick size 20ns (fundamental unit of time in nano seconds)
#define TICK_SIZE_200NS    0x10   // 200 ns
#define TICK_SIZE_2000NS   0x20   // 2000 ns
#define TICK_SIZE_20000NS  0x30   // 20000 ns

/* Counter Options */
#define CLEAR_ON_READ     0x1     // Clear on Read
#define NO_RECYCLE        0x2     // No recycle mode
#define COUNT_DOWN        0x4     // Count Down
#define RANGE_LIMIT       0x8     // Range Limit (use max and min limits)
#define FALLING_EDGE      0x10    // Count on the falling edge

/* Aanalog Input */
#define DIFFERENTIAL      0
#define SINGLE_ENDED      1
#define GROUNDED          3
#define PACKET_SIZE       512    // max bulk transfer size in bytes
#define CONTINUOUS        1      // continuous input mode
#define EXTERNAL_TRIGGER  0x1    // 1 = use external trigger
#define PATTERN_DETECTION 0x2    // 1 = use Pattern Detection trigger
#define RETRIGGER_MODE    0x4    // 1 = retrigger mode, 0 = normal trigger
#define COUNTER_VALUE     0x8    // 1 = Maintain counter value on scan start
                                 // 0 = Clear counter value on scan start
#define SINGLE_IO         0x10   // 1 = use SINGLE_IO data transfer,  0 = use BLOCK_IO transfer
  
/* Ananlog Output Scan Options */
#define AO_CHAN0       0x1   // Include Channel 0 in output scan
#define AO_CHAN1       0x2   // Include Channel 1 in output scan
#define AO_TRIG        0x10  // Use Trigger
#define AO_RETRIG_MODE 0x20  // Retrigger Mode
  
/* Ranges */
#define BP_10V 0x0      // +/- 10V
#define BP_5V  0x1      // +/- 5V
#define UP_10V 0x2      // 0 - 10V
#define UP_5V  0x3      // 0 - 5V
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)  // input pacer running
#define AIN_SCAN_OVERRUN   (0x1 << 2)  // input scan overrun
#define AOUT_SCAN_RUNNING  (0x1 << 3)  // output scan running
#define AOUT_SCAN_UNDERRUN (0x1 << 4)  // output scan overrun
#define AIN_SCAN_DONE      (0x1 << 5)  // input scan done
#define AOUT_SCAN_DONE     (0x1 << 6)  // output scan done
#define FPGA_CONFIGURED    (0x1 << 8)  // 1 = FPGA configured
#define FPGA_CONFIG_MODE   (0x1 << 9)  // 1 = FPGA config mode

#define NCHAN_1808           8  // max number of A/D channels in the device
#define NGAINS_1808          4  // max number of gain levels
#define NCHAN_AO_1808        2  // number of analog output channels
#define MAX_PACKET_SIZE_HS  512 // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64 // max packet size for FS device

typedef struct Calibration_AIN_t {
  float slope;
  float offset;
} Calibration_AIN;

typedef struct Calibration_AOUT_t {
  float slope;
  float offset;
} Calibration_AOUT;

typedef struct timerParams_t {
  uint32_t period;
  uint32_t pulseWidth;
  uint32_t count;
  uint32_t delay;
} timerParams;

typedef struct ScanList_t {
  uint8_t range;       // BP10V, BP5V, UP10V, UP5V
  uint8_t mode;        // Differential or Single Ended
} ScanList;

typedef struct usbDevice1808_t {
  libusb_device_handle *udev;                           // libusb 1.0 handle
  Calibration_AIN  table_AIn[NCHAN_1808][NGAINS_1808];  // calibration coefficients ADC
  Calibration_AOUT table_AOut[NCHAN_AO_1808];           // calibration coefficients DAC
  ScanList list[NCHAN_1808];
  uint8_t scan_list[NCHAN_1808];                        // scan list
  uint8_t options;
  int nChannels;
} usbDevice1808;

/* function prototypes for the USB-1808 */
void usbCalDate_USB1808(libusb_device_handle *udev, struct tm *date);
void usbDTristateW_USB1808(libusb_device_handle *udev, uint8_t value);
uint8_t usbDTristateR_USB1808(libusb_device_handle *udev);
uint8_t usbDPort_USB1808(libusb_device_handle *udev);
void usbDLatchW_USB1808(libusb_device_handle *udev, uint8_t value);
uint8_t usbDLatchR_USB1808(libusb_device_handle *udev);
void usbBlink_USB1808(libusb_device_handle *udev, uint8_t count);
void cleanup_USB1808( libusb_device_handle *udev);
void usbGetSerialNumber_USB1808(libusb_device_handle *udev, char serial[9]);
void usbReset_USB1808(libusb_device_handle *udev);
void usbFPGAConfig_USB1808(libusb_device_handle *udev);
void usbFPGAData_USB1808(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB1808(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB1808(libusb_device_handle *udev);
int usbInit_1808(libusb_device_handle *udev);
void usbMemoryR_USB1808(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB1808(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB1808(libusb_device_handle *udev, uint16_t *address);
void usbMemAddressW_USB1808(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB1808(libusb_device_handle *udev);
void usbReset_USB1808(libusb_device_handle *udev);
int usbTriggerConfigW_USB1808(libusb_device_handle *udev, uint8_t options);
int usbTriggerConfigR_USB1808(libusb_device_handle *udev, uint8_t *options);
int usbPatternDetectConfigW_USB1808(libusb_device_handle *udev, uint8_t value, uint8_t mask, uint8_t options);
int usbPatternDetectConfigR_USB1808(libusb_device_handle *udev, uint8_t *value, uint8_t *mask, uint8_t *options);
int usbAIn_USB1808(libusb_device_handle *udev, uint32_t value[8], Calibration_AIN table[NCHAN_1808][NGAINS_1808], ScanList list[NCHAN_1808]);
int usbADCSetupR_USB1808(libusb_device_handle *udev, ScanList list[8]);
int usbADCSetupW_USB1808(libusb_device_handle *udev, ScanList list[8]);
int usbAInScanStart_USB1808(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency,uint8_t options);
void usbAInScanStop_USB1808(libusb_device_handle *udev);
int usbAInScanRead_USB1808(libusb_device_handle *udev, int nScan, int nChan, uint32_t *data, unsigned int timeout, int options);
int usbAInScanConfigW_USB1808(libusb_device_handle *udev, uint8_t scanQueue[13], uint8_t lastChan);
int usbAInScanConfigR_USB1808(libusb_device_handle *udev, uint8_t *scanQueue, uint8_t lastChan);
void usbAInScanClearFIFO_USB1808(libusb_device_handle *udev);
void usbAInBulkFlush_USB1808(libusb_device_handle *udev, uint8_t count);
void usbBuildGainTableAI_USB1808(libusb_device_handle *udev, Calibration_AIN table[NCHAN_1808][NGAINS_1808]);
void usbBuildGainTableAO_USB1808(libusb_device_handle *udev, Calibration_AOUT table_AO[NCHAN_AO_1808]);
void usbAOut_USB1808(libusb_device_handle *udev, uint8_t channel, double voltage, Calibration_AOUT table_AO[NCHAN_AO_1808]);
int usbAOutScanConfigW_USB1808(libusb_device_handle *udev, uint8_t scanQueue[3], uint8_t lastChan);
int usbAOutScanConfigR_USB1808(libusb_device_handle *udev, uint8_t *scanQueue, uint8_t lastChan);
int usbAOutScanStart_USB1808(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
void usbAOutScanStop_USB1808(libusb_device_handle *udev);
void usbAOutScanClearFIFO_USB1808(libusb_device_handle *udev);
int usbCounterW_USB1808(libusb_device_handle *udev, uint8_t counter, uint32_t count);
int usbCounterR_USB1808(libusb_device_handle *udev, uint8_t counter, uint32_t *count);
int usbCounterOptionsW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t options);
int usbCounterOptionsR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *options);
int usbCounterLimitsW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint32_t value);
int usbCounterLimitsR_UBS1808(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint32_t *value);
int usbCounterModeW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t mode);
int usbCounterModeR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *mode);
int usbCounterParametersW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t mode, uint8_t options);
int usbCounterParametersR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *mode, uint8_t *options);
int usbTimerControlW_USB1808(libusb_device_handle *udev, uint8_t timer, uint8_t control);
int usbTimerControlR_USB1808(libusb_device_handle *udev, uint8_t timer, uint8_t *control);
int usbTimerParametersW_USB1808(libusb_device_handle *udev, uint8_t timer, double frequency, double dutyCycle, uint32_t count, uint32_t delay);
int usbTimerParametersR_USB1808(libusb_device_handle *udev, uint8_t timer, uint32_t *period, uint32_t *pulseWidth, uint32_t *count, uint32_t *delay);
uint16_t voltsTou16_USB1808(double volts, int channel, float table_AO[NCHAN_AO_1808][2]);
double volts_USB1808(const uint8_t gain, uint32_t value);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_1808_H
