/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef USB_CTR_H

#define USB_CTR_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define USB_CTR08_PID      (0x0127)
#define USB_CTR04_PID      (0x012E)

#define USB_CTR_NTIMER    4     // Number of PWM Timers
#define USB_CTR_NCOUNTER  8     // Number of Counters
#define BASE_CLOCK        96.E6 // Base clock frequency
#define MAX_PACKET_SIZE_HS  512 // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64 // max packet size for FS device

// Status bit values 
#define USB_CTR_PACER_RUNNING      (0x1 << 1)
#define USB_CTR_SCAN_OVERRUN       (0x1 << 2)
#define USB_CTR_SCAN_DONE          (0x1 << 5)
#define USB_CTR_FPGA_CONFIGURED    (0x1 << 8)
#define USB_CTR_FPGA_CONFIG_MODE   (0x1 << 9)

// Mode Register
#define USB_CTR_TOTALIZE            (0x0)
#define USB_CTR_PERIOD              (0x1)
#define USB_CTR_PULSEWIDTH          (0x2)
#define USB_CTR_TIMING              (0x3)
#define USB_CTR_PERIOD_MODE_1X      (0x0)
#define USB_CTR_PERIOD_MODE_10X     (0x1 << 2)
#define USB_CTR_PERIOD_MODE_100X    (0x2 << 2)
#define USB_CTR_PERIOD_MODE_1000X   (0x3 << 2)
#define USB_CTR_TICK_SIZE_20_83ns   (0x0)
#define USB_CTR_TICK_SIZE_208_3ns   (0x1 << 4)
#define USB_CTR_TICK_SIZE_2083_3ns  (0x2 << 4)
#define USB_CTR_TICK_SIZE_20833_3ns (0x3 << 4)  

// Options Register
#define USB_CTR_CLEAR_ON_READ (0x1 << 0)
#define USB_CTR_NO_RECYCLE    (0x1 << 1)
#define USB_CTR_COUNT_DOWN    (0x1 << 2)
#define USB_CTR_RANGE_LIMIT   (0x1 << 3)
#define USB_CTR_FALLING_EDGE  (0x1 << 4)

typedef struct counterParams_t {
  uint8_t counter;
  uint8_t modeOptions;
  uint8_t counterOptions;
  uint8_t gateOptions;
  uint8_t outputOptions;
  uint8_t debounce;
} CounterParams;  

typedef struct timerParams_t {
  uint8_t timer;
  uint32_t period;
  uint32_t pulseWidth;
  uint32_t count;
  uint32_t delay;
} TimerParams;

#define USB_CTR_CONTINUOUS_READOUT (0x1)
#define USB_CTR_SINGLEIO           (0x2)
#define USB_CTR_FORCE_PACKET_SIZE  (0x4)

#define USB_CTR_SCAN_DIO           (0x20)
#define USB_CTR_PADZERO            (0x40)

typedef struct scanData_t {
  uint8_t scanList[33];   // the channel configuration
  uint8_t lastElement;    // the last element of the scan list [0-32]
  uint32_t count;         // the total number of scans to perform (0 for continuous scan)
  uint32_t retrig_count;  // the number of scans to perform for each trigger in retrigger mode
  double frequency;       // frequency of the scan  (0 for external clock)
  uint8_t options;        /*   bit 0:   1 = Maintain counter value on scan start, 0 = Clear counter value on scan start
                               bit 1:   Reserved
                               bit 2:   Reserved
                               bit 3:   1 = use trigger
                               bit 4:   Reserved
                               bit 5:   Reserved
                               bit 6:   1 = retrigger mode,  0 = normal trigger
                               bit 7:   Reserved */
  uint8_t mode;            /* mode bits:
                               bit 0:   0 = counting mode,  1 = CONTINUOUS_READOUT
                               bit 1:   1 = SINGLEIO
                               bit 2:   1 = use packet size passed scanData->packet_size
			   */
  uint16_t packet_size;    // number of samples to return from FIFO
  uint16_t status;         // status word of the device
  int bytesToRead;         // number of bytes left to read in the scan
} ScanData;


/* function prototypes for the USB-CTR */
void usbInit_CTR(libusb_device_handle *udev);
void usbDTristateW_USB_CTR(libusb_device_handle *udev, uint16_t value);
uint16_t usbDTristateR_USB_CTR(libusb_device_handle *udev);
uint16_t usbDPort_USB_CTR(libusb_device_handle *udev);
void usbDLatchW_USB_CTR(libusb_device_handle *udev, uint16_t value);
uint16_t usbDLatchR_USB_CTR(libusb_device_handle *udev);
void usbBlink_USB_CTR(libusb_device_handle *udev, uint8_t count);
void cleanup_USB_CTR( libusb_device_handle *udev);
void usbGetSerialNumber_USB_CTR(libusb_device_handle *udev, char serial[9]);
void usbReset_USB_CTR(libusb_device_handle *udev);
void usbFPGAConfig_USB_CTR(libusb_device_handle *udev);
void usbFPGAData_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB_CTR(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB_CTR(libusb_device_handle *udev);
void usbMemoryR_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB_CTR(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USB_CTR(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB_CTR(libusb_device_handle *udev);
void usbTriggerConfig_USB_CTR(libusb_device_handle *udev, uint8_t options);
void usbTriggerConfigR_USB_CTR(libusb_device_handle *udev, uint8_t *options);
void usbTimerControlR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint8_t *control);
void usbTimerControlW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint8_t control);
void usbTimerPeriodR_USB_CTR(libusb_device_handle *udev, uint8_t timer, float *period);
void usbTimerPeriodW_USB_CTR(libusb_device_handle *udev, uint8_t timer, float period);
void usbTimerPulseWidthR_USB_CTR(libusb_device_handle *udev, uint8_t timer, float *pulseWidth);
void usbTimerPulseWidthW_USB_CTR(libusb_device_handle *udev, uint8_t timer, float pulseWidth);
void usbTimerCountR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t *count);
void usbTimerCountW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t count);
void usbTimerDelayR_USB_CTR(libusb_device_handle *udev, uint8_t timer, float *delay);
void usbTimerDelayW_USB_CTR(libusb_device_handle *udev, uint8_t timer, float delay);
void usbTimerParamsR_USB_CTR(libusb_device_handle *udev, uint8_t timer, TimerParams *params);
void usbTimerParamsW_USB_CTR(libusb_device_handle *udev, uint8_t timer, TimerParams params);
void usbCounterSet_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint64_t count);
uint64_t usbCounter_USB_CTR(libusb_device_handle *udev, uint8_t counter);  
void usbCounterModeR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *mode);
void usbCounterModeW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t mode);
void usbCounterOptionsR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options);
void usbCounterOptionsW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options);
void usbCounterDebounceR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *debounce);
void usbCounterDebounceW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t debounce);
void usbCounterGateConfigR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options);
void usbCounterGateConfigW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options);
void usbCounterOutConfigR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options);
void usbCounterOutConfigW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options);
void usbCounterOutValuesR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t *value);
void usbCounterOutValuesW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t value);
void usbCounterLimitValuesR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t *value);
void usbCounterLimitValuesW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t value);
void usbCounterParamsR_USB_CTR(libusb_device_handle *udev, uint8_t counter, CounterParams *params);
void usbCounterParamsW_USB_CTR(libusb_device_handle *udev, uint8_t counter, CounterParams params);

void usbScanConfigR_USB_CTR(libusb_device_handle *udev, ScanData *scanData);
void usbScanConfigW_USB_CTR(libusb_device_handle *udev, ScanData scanData);
void usbScanStart_USB_CTR(libusb_device_handle *udev, ScanData *scanData);
int usbScanRead_USB_CTR(libusb_device_handle *udev, ScanData *scanData, uint16_t *data);
void usbScanStop_USB_CTR(libusb_device_handle *udev);
void usbScanClearFIFO_USB_CTR(libusb_device_handle *udev);
void usbScanBulkFlush_USB_CTR(libusb_device_handle *udev, uint8_t count);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_CTR

