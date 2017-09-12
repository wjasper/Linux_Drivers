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

#ifndef USB_1608G_H

#define USB_1608G_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB1608G_PID         (0x0110)
#define USB1608GX_PID        (0x0111)
#define USB1608GX_2AO_PID    (0x0112)

#define USB1608G_V2_PID      (0x0134)
#define USB1608GX_V2_PID     (0x0135)
#define USB1608GX_2AO_V2_PID (0x0136)

/* Counter Timer */
#define COUNTER0         0x0     //  Counter 0
#define COUNTER1         0x1     //  Counter 1

/* Aanalog Input */
#define SINGLE_ENDED   0
#define DIFFERENTIAL   1
#define CALIBRATION    3
#define LAST_CHANNEL   (0x80)
#define PACKET_SIZE    512       // max bulk transfer size in bytes
#define CONTINUOUS     1         // continuous input mode
  
/* Ananlog Output Scan Options */
#define AO_CHAN0       0x1   // Include Channel 0 in output scan
#define AO_CHAN1       0x2   // Include Channel 1 in output scan
#define AO_TRIG        0x10  // Use Trigger
#define AO_RETRIG_MODE 0x20  // Retrigger Mode
  
/* Ranges */
#define BP_10V 0x0      // +/- 10 V
#define BP_5V  0x1      // +/- 5V
#define BP_2V  0x2      // +/- 2V
#define BP_1V  0x3      // +/- 1V
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define AOUT_SCAN_RUNNING  (0x1 << 3)
#define AOUT_SCAN_UNDERRUN (0x1 << 4)
#define AIN_SCAN_DONE      (0x1 << 5)
#define AOUT_SCAN_DONE     (0x1 << 6)
#define FPGA_CONFIGURED    (0x1 << 8)
#define FPGA_CONFIG_MODE   (0x1 << 9)

#define NCHAN_1608G          16  // max number of A/D channels in the device
#define NGAINS_1608G          4  // max number of gain levels
#define NCHAN_AO_1608GX       2  // number of analog output channels
#define MAX_PACKET_SIZE_HS  512  // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64  // max packet size for FS device

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

typedef struct usbDevice1608G_t {
  libusb_device_handle *udev;         // libusb 1.0 handle
  float table_AIn[NGAINS_1608G][2];   // calibration coefficients
  float table_AOut[NCHAN_AO_1608GX][2];
  ScanList list[NCHAN_1608G];
  uint8_t scan_list[NCHAN_1608G];     // scan list
  uint8_t options;
  int nChannels;
} usbDevice1608G;

/* function prototypes for the USB-1608G */
void usbCalDate_USB1608G(libusb_device_handle *udev, struct tm *date);
void usbDTristateW_USB1608G(libusb_device_handle *udev, uint16_t value);
uint16_t usbDTristateR_USB1608G(libusb_device_handle *udev);
uint16_t usbDPort_USB1608G(libusb_device_handle *udev);
void usbDLatchW_USB1608G(libusb_device_handle *udev, uint16_t value);
uint16_t usbDLatchR_USB1608G(libusb_device_handle *udev);
void usbBlink_USB1608G(libusb_device_handle *udev, uint8_t count);
void cleanup_USB1608G( libusb_device_handle *udev);
void usbTemperature_USB1608G(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB1608G(libusb_device_handle *udev, char serial[9]);
void usbReset_USB1608G(libusb_device_handle *udev);
void usbFPGAConfig_USB1608G(libusb_device_handle *udev);
void usbFPGAData_USB1608G(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB1608G(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB1608G(libusb_device_handle *udev);
void usbInit_1608G(libusb_device_handle *udev, int version);
void usbCounterInit_USB1608G(libusb_device_handle *udev, uint8_t counter);
uint32_t usbCounter_USB1608G(libusb_device_handle *udev, uint8_t counter);
void usbTimerControlR_USB1608G(libusb_device_handle *udev, uint8_t *control);
void usbTimerControlW_USB1608G(libusb_device_handle *udev, uint8_t control);
void usbTimerPeriodR_USB1608G(libusb_device_handle *udev, uint32_t *period);
void usbTimerPeriodW_USB1608G(libusb_device_handle *udev, uint32_t period);
void usbTimerPulseWidthR_USB1608G(libusb_device_handle *udev, uint32_t *pulseWidth);
void usbTimerPulseWidthW_USB1608G(libusb_device_handle *udev, uint32_t pulseWidth);
void usbTimerCountR_USB1608G(libusb_device_handle *udev, uint32_t *count);
void usbTimerCountW_USB1608G(libusb_device_handle *udev, uint32_t count);
void usbTimerDelayR_USB1608G(libusb_device_handle *udev, uint32_t *delay);
void usbTimerDelayW_USB1608G(libusb_device_handle *udev, uint32_t delay);
void usbTimerParamsR_USB1608G(libusb_device_handle *udev, timerParams *params);
void usbTimerParamsW_USB1608G(libusb_device_handle *udev, timerParams *params);
void usbMemoryR_USB1608G(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB1608G(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB1608G(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USB1608G(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB1608G(libusb_device_handle *udev);
void usbReset_USB1608G(libusb_device_handle *udev);
void usbTriggerConfig_USB1608G(libusb_device_handle *udev, uint8_t options);
void usbTriggerConfigR_USB1608G(libusb_device_handle *udev, uint8_t *options);
void usbTemperature_USB1608G(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB1608G(libusb_device_handle *udev, char serial[9]);
uint16_t usbAIn_USB1608G(libusb_device_handle *udev, uint16_t channel);
void usbAInScanStart_USB1608G(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
void usbAInScanStop_USB1608G(libusb_device_handle *udev);
int usbAInScanRead_USB1608G(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, unsigned int timeout, int options);
void usbAInConfig_USB1608G(libusb_device_handle *udev, ScanList scanList[NCHAN_1608G]);
int usbAInConfigR_USB1608G(libusb_device_handle *udev, uint8_t *scanList);
void usbAInScanClearFIFO_USB1608G(libusb_device_handle *udev);
void usbBuildGainTable_USB1608G(libusb_device_handle *udev, float table[NGAINS_1608G][2]);
double volts_USB1608G(const uint8_t gain, uint16_t value);
void usbBuildGainTable_USB1608GX_2AO(libusb_device_handle *udev, float table_AO[NCHAN_AO_1608GX][2]);
uint16_t voltsTou16_USB1608GX_AO(double volts, int channel, float table_AO[NCHAN_AO_1608GX][2]);
void usbAOut_USB1608GX_2AO(libusb_device_handle *udev, uint8_t channel, double voltage, float table_AO[NCHAN_AO_1608GX][2]);
void usbAOutR_USB1608GX_2AO(libusb_device_handle *udev, uint8_t channel, double *voltage, float table_AO[NCHAN_AO_1608GX][2]);
void usbAOutScanStop_USB1608GX_2AO(libusb_device_handle *udev);
void usbAOutScanClearFIFO_USB1608GX_2AO(libusb_device_handle *udev);
void usbAOutScanStart_USB1608GX_2AO(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_1608G_H
