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

#ifndef USB_2600_H

#define USB_2600_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB2623_PID (0x0120)
#define USB2627_PID (0x0121)
#define USB2633_PID (0x0118)
#define USB2637_PID (0x0119)

/* Counter Timer */
#define COUNTER0         0x0     // Counter 0
#define COUNTER1         0x1     // Counter 1
#define COUNTER2         0x2     // Counter 2
#define COUNTER3         0x3     // Counter 3
#define TIMER0           0x0     // Timer 0 
#define TIMER1           0x1     // Timer 1 
#define TIMER2           0x2     // Timer 2 
#define TIMER3           0x3     // Timer 3 

/* Aanalog Input */
#define SINGLE_ENDED   0
#define CALIBRATION    1
#define LAST_CHANNEL   (0x80)
#define PACKET_SIZE    512       // max bulk transfer size in bytes
#define CONTINUOUS     1         // continuous input mode
  
/* Ranges */
#define BP_10V 0x0    // +/- 10 V
#define BP_5V  0x1    // +/- 5V
#define BP_2V  0x2    // +/- 2V
#define BP_1V  0x3    // +/- 1V

/* Ananlog Output Scan Options */
#define AO_CHAN0       0x1   // Include Channel 0 in output scan
#define AO_CHAN1       0x2   // Include Channel 1 in output scan
#define AO_CHAN2       0x4   // Include Channel 2 in output scan
#define AO_CHAN3       0x8   // Include Channel 3 in output scan
#define AO_TRIG        0x10  // Use Trigger
#define AO_RETRIG_MODE 0x20  // Retrigger Mode
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define AOUT_SCAN_RUNNING  (0x1 << 3)
#define AOUT_SCAN_UNDERRUN (0x1 << 4)
#define AIN_SCAN_DONE      (0x1 << 5)
#define AOUT_SCAN_DONE     (0x1 << 6)
#define FPGA_CONFIGURED    (0x1 << 8)
#define FPGA_CONFIG_MODE   (0x1 << 9)

#define NCHAN_2600           64 // max number of A/D channels in the device
#define NGAINS_2600           4 // max number of gain levels
#define NCHAN_AO_26X7         4 // number of analog output channels 
#define MAX_PACKET_SIZE_HS  512 // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64 // max packet size for FS device

typedef struct timerParams_t {
  uint32_t period;
  uint32_t pulseWidth;
  uint32_t count;
  uint32_t delay;
} timerParams;

typedef struct ScanList_t {
  uint8_t mode;
  uint8_t range;
  uint8_t channel;
} ScanList;

/* function prototypes for the USB-2600 */
void usbDTristateW_USB2600(libusb_device_handle *udev, uint8_t port, uint16_t value);
uint16_t usbDTristateR_USB2600(libusb_device_handle *udev, uint8_t port);
uint16_t usbDPort_USB2600(libusb_device_handle *udev, uint8_t port);
void usbDLatchW_USB2600(libusb_device_handle *udev,uint8_t port,  uint16_t value);
uint16_t usbDLatchR_USB2600(libusb_device_handle *udev, uint8_t port);
void usbBlink_USB2600(libusb_device_handle *udev, uint8_t count);
void cleanup_USB2600( libusb_device_handle *udev);
void usbTemperature_USB2600(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB2600(libusb_device_handle *udev, char serial[9]);
void usbReset_USB2600(libusb_device_handle *udev);
void usbFPGAConfig_USB2600(libusb_device_handle *udev);
void usbFPGAData_USB2600(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB2600(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB2600(libusb_device_handle *udev);
void usbInit_2600(libusb_device_handle *udev);
void usbCounterInit_USB2600(libusb_device_handle *udev, uint8_t counter);
uint32_t usbCounter_USB2600(libusb_device_handle *udev, uint8_t counter);
void usbTimerControlR_USB2600(libusb_device_handle *udev, uint8_t timer, uint8_t *control);
void usbTimerControlW_USB2600(libusb_device_handle *udev, uint8_t timer, uint8_t control);
void usbTimerPeriodR_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t *period);
void usbTimerPeriodW_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t period);
void usbTimerPulseWidthR_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t *pulseWidth);
void usbTimerPulseWidthW_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t pulseWidth);
void usbTimerCountR_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t *count);
void usbTimerCountW_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t count);
void usbTimerDelayR_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t *delay);
void usbTimerDelayW_USB2600(libusb_device_handle *udev, uint8_t timer, uint32_t delay);
void usbTimerParamsR_USB2600(libusb_device_handle *udev, uint8_t timer, timerParams *params);
void usbTimerParamsW_USB2600(libusb_device_handle *udev, uint8_t timer, timerParams *params);
void usbMemoryR_USB2600(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB2600(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB2600(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USB2600(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB2600(libusb_device_handle *udev);
void usbTriggerConfig_USB2600(libusb_device_handle *udev, uint8_t options);
void usbTriggerConfigR_USB2600(libusb_device_handle *udev, uint8_t *options);
void usbTemperature_USB2600(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB2600(libusb_device_handle *udev, char serial[9]);
uint16_t usbAIn_USB2600(libusb_device_handle *udev, uint16_t channel);
void usbAInScanStart_USB2600(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency,
			      uint8_t packet_size, uint8_t options);
void usbAInScanStop_USB2600(libusb_device_handle *udev);
int usbAInScanRead_USB2600(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, unsigned int timeout, int options);
void usbAInConfig_USB2600(libusb_device_handle *udev, ScanList scanList[NCHAN_2600]);
void usbAInConfigR_USB2600(libusb_device_handle *udev, ScanList *scanList);
void usbAInScanClearFIFO_USB2600(libusb_device_handle *udev);
void usbAOut_USB26X7(libusb_device_handle *udev, uint8_t channel, double voltage, float table_AO[NCHAN_AO_26X7][2]);
void usbAOutR_USB26X7(libusb_device_handle *udev, uint8_t channel, double *voltage, float table_AO[NCHAN_AO_26X7][2]);
void usbAOutScanStop_USB26X7(libusb_device_handle *udev);
void usbAOutScanClearFIFO_USB26X7(libusb_device_handle *udev);
void usbAOutScanStart_USB2600(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
void usbBuildGainTable_USB2600(libusb_device_handle *udev, float table[NGAINS_2600][2]);
void usbBuildGainTable_USB26X7(libusb_device_handle *udev, float table_AO[NCHAN_AO_26X7][2]);
void usbAOut_USB26X7(libusb_device_handle *udev, uint8_t channel, double voltage, float table_AO[NCHAN_AO_26X7][2]);
void usbAOutScanStop_USB26X7(libusb_device_handle *udev);
void usbAOutScanClearFIFO_USB26X7(libusb_device_handle *udev);
void usbAOutScanStart_USB26X7(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
double volts_USB2600(const uint8_t gain, uint16_t value);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_2600_H
